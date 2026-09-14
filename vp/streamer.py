#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
streamer.py — 板端 JPEG 帧 TCP 推流 (YOLO 画框画面 -> 上位机)

协议: 每帧一个报文 = struct.pack('<I', 帧长) + JPEG bytes
      (上位机按 4 字节长度前缀切帧, cv2.imdecode 显示)

设计:
  - JpegStreamer 是 daemon 线程: start() 后监听 bind_host:port
  - 每个已连接客户端独占发送: 以 fps 节流发"最新一帧" (队列满丢旧, 低延迟)
  - 断连自动回到 accept; stop() 后 ~1s 内退出
"""
import queue
import socket
import struct
import threading
import time

import cv2


def put_latest(q, item):
    """最新帧覆盖式入队: 队列满时丢弃旧帧再入队, 保证始终发最新画面."""
    while True:
        try:
            q.put_nowait(item)
            return
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                pass


class JpegStreamer(threading.Thread):
    """单监听端口 JPEG 推流线程 (front/bottom 各一个实例)."""

    def __init__(self, name, port, bind_host='0.0.0.0',
                 fps=30, quality=80, queue_size=1):
        """初始化推流线程：设置监听地址/端口、发送节拍(interval)、JPEG 质量与帧队列。daemon 线程，start() 后开始监听。"""
        super().__init__(daemon=True)
        self.name = name
        self.port = int(port)
        self.bind_host = bind_host
        self.interval = 1.0 / max(1, int(fps))
        self.quality = int(quality)
        self._q = queue.Queue(maxsize=max(1, int(queue_size)))
        self._stop = threading.Event()
        self._clients = 0
        self._ready = threading.Event()   # 监听已建立 (供测试/日志)

    # ---- worker 线程调用 ----
    def put_frame(self, vis):
        """把画框 BGR 帧送入发送队列 (非阻塞, 满则丢旧保新)."""
        if not self._stop.is_set():
            put_latest(self._q, vis)

    # ---- 状态 ----
    @property
    def connected(self):
        """是否有客户端正在接收 (无客户端时不画框不编码, 零开销)."""
        return self._clients > 0

    def stop(self):
        """请求停止：置停止事件，线程在 ~1s 内退出。"""
        self._stop.set()

    def wait_ready(self, timeout=3.0):
        """等待监听建立完成（测试/日志用）；返回是否就绪。"""
        return self._ready.wait(timeout)

    # ---- 线程主体 ----
    def run(self):
        """线程主体：监听端口 -> accept 单个客户端 -> _serve -> 断线回到 accept。

        监听失败（端口被占等）打印警告后直接禁用推流，不影响检测链路。
        """
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.settimeout(0.5)
            srv.bind((self.bind_host, self.port))
            srv.listen(1)
            self._ready.set()
            print(f'[*] streamer[{self.name}]: 监听 {self.bind_host}:{self.port}', flush=True)
        except OSError as exc:
            print(f'[警告] streamer[{self.name}]: 监听失败 {exc!r}, 推流禁用', flush=True)
            return
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with conn:
                    self._clients += 1
                    print(f'[*] streamer[{self.name}]: 客户端接入 {addr[0]}:{addr[1]}', flush=True)
                    conn.settimeout(1.0)
                    try:
                        self._serve(conn)
                    except Exception as exc:
                        print(f'[警告] streamer[{self.name}]: 发送异常 {exc!r}', flush=True)
                    finally:
                        self._clients -= 1
                        print(f'[*] streamer[{self.name}]: 客户端断开 {addr[0]}:{addr[1]}', flush=True)
        finally:
            try:
                srv.close()
            except OSError:
                pass
            print(f'[*] streamer[{self.name}]: 已停止', flush=True)

    def _serve(self, conn):
        """为单个已连接的客户端发送 JPEG 帧：按 fps 节流，每周期发最新一帧。

        编码参数在进入循环前只构造一次；发送失败（客户端断开）时返回，
        由 run() 收回 accept 等待下一个客户端。
        """
        enc_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        next_send = 0.0
        while not self._stop.is_set():
            now = time.perf_counter()
            wait = next_send - now
            if wait > 0:
                if self._stop.wait(min(wait, 0.05)):   # 短等待, 保证 stop 及时退出
                    return
                now = time.perf_counter()
                if now < next_send:
                    continue
            try:
                vis = self._q.get(timeout=0.5)
            except queue.Empty:
                next_send = time.perf_counter() + self.interval
                continue
            ok, buf = cv2.imencode('.jpg', vis, enc_params)
            if not ok:
                continue
            try:
                conn.sendall(struct.pack('<I', len(buf)) + buf.tobytes())
            except OSError:
                return          # 客户端断开/异常 -> 回到 accept 等下一个客户端
            next_send = time.perf_counter() + self.interval
