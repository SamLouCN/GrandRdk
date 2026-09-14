#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mjpeg_bridge.py — vp5.0 STREAM(TCP JPEG) -> HTTP MJPEG 桥

目的: 让 ROV 控制站 pc_main2.py (cv2.VideoCapture http://IP:5000/cam1|cam2)
      直接显示 vp5.0 的 YOLO 画框画面, 无需改上位机取流代码.

架构:
  [front.py streamer :9000] <--TCP--> [本桥 /cam1] <--HTTP MJPEG--> PC
  [bottom.py streamer:9001] <--TCP--> [本桥 /cam2] <--HTTP MJPEG--> PC

说明:
  - 每路只维持 1 条上游 TCP(streamer 单客户端), 断线自动重连
  - HTTP 端支持多客户端, 各自读最新帧广播 (cv2.VideoCapture 容错丢帧)
"""
import argparse
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = b"frame"


def _recv_exact(s, n):
    """从 TCP 套接字精确读取 n 字节（length-prefix 协议切帧用）；连接中断返回 None。"""
    buf = b''
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            return None
        buf += c
    return buf


class Upstream:
    """连接 vp5.0 JpegStreamer 端口, 持续拉取最新 JPEG 帧."""

    def __init__(self, host, port):
        """启动上游拉流线程（daemon）：host/port 指向 vp5.0 的 JpegStreamer 端口，线程在后台持续拉取最新 JPEG 帧，断线自动重连（见 _run）。"""
        self.host = host
        self.port = int(port)
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0
        self._stop = threading.Event()
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def _run(self):
        # 上游拉流线程: 断线自动每 1s 重连 streamer; 每收到一帧就覆盖 self._jpeg (只留最新)
        """上游拉流线程主体：连接 streamer -> 按 4 字节长度前缀切帧 -> 覆盖式保存最新 JPEG；连接失败时每 1s 重试。收到非法长度（<=0 或超 4MB）视为协议错位，主动重连。"""
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=5)
                s.settimeout(2.0)
            except OSError:
                time.sleep(1)          # 上游(streamer.py)未启动/已断开 → 稍后重试
                continue
            try:
                while not self._stop.is_set():
                    hdr = _recv_exact(s, 4)
                    if hdr is None:
                        break
                    n = struct.unpack('<I', hdr)[0]   # 4 字节小端长度前缀 + JPEG 体
                    if n <= 0 or n > 4_000_000:
                        break                          # 非法长度 → 视为协议错位, 重连
                    buf = _recv_exact(s, n)
                    if buf is None:
                        break
                    with self._lock:
                        self._jpeg = buf
                        self._seq += 1    # 每收到新帧 seq+1; HTTP 端据此跳过重复帧
            except OSError:
                pass
            finally:
                try:
                    s.close()
                except OSError:
                    pass
            time.sleep(1)

    def latest(self):
        """返回 (最新 JPEG 帧, 序号)；无帧时 jpeg 为 None。线程安全。"""
        with self._lock:
            return self._jpeg, self._seq

    def stop(self):
        """请求停止上游拉流线程。"""
        self._stop.set()


class Handler(BaseHTTPRequestHandler):
    """HTTP MJPEG 请求处理器：把路径(/cam1|/cam2)映射到对应的 Upstream 实例，向客户端按 multipart/x-mixed-replace 流动画广播最新 JPEG 帧。"""
    upstreams = {}

    def do_GET(self):
        """处理 GET 请求：校验路径 -> 发 MJPEG 流响应头 -> 循环广播最新帧（seq 变化才发，无新帧睡 10ms 避免忙等）。"""
        path = self.path.split('?')[0].rstrip('/')
        up = self.upstreams.get(path)
        if up is None:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=' + BOUNDARY.decode())
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'close')
        self.end_headers()
        last = 0
        try:
            while True:
                # 广播循环: 每个 HTTP 客户端独立轮询最新帧; seq 未变(无新帧)则睡 10ms
                jpeg, seq = up.latest()
                if seq == last:
                    time.sleep(0.01)
                    continue
                last = seq
                if jpeg is None:
                    continue
                try:
                    self.wfile.write(b'--' + BOUNDARY +
                                     b'\r\nContent-Type: image/jpeg\r\n\r\n' +
                                     jpeg + b'\r\n')
                    self.wfile.flush()
                except OSError:
                    return
        except Exception:
            pass

    def log_message(self, *args):
        """屏蔽默认访问日志（每请求一行会刷屏）。"""
        pass


def main():
    """CLI 入口：创建 cam1/cam2 两路 Upstream，注册 /cam1 /cam2 路由，启动 HTTP 服务；Ctrl-C 时停上游线程并退出。"""
    ap = argparse.ArgumentParser(description='vp5.0 STREAM -> HTTP MJPEG 桥')
    ap.add_argument('--host', default='127.0.0.1', help='上游 streamer 地址')
    ap.add_argument('--front-port', type=int, default=9000)
    ap.add_argument('--bottom-port', type=int, default=9001)
    ap.add_argument('--http-port', type=int, default=5000)
    ap.add_argument('--bind', default='0.0.0.0')
    args = ap.parse_args()

    cam1 = Upstream(args.host, args.front_port)
    cam2 = Upstream(args.host, args.bottom_port)
    Handler.upstreams = {'/cam1': cam1, '/cam2': cam2}

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    print(f'[*] mjpeg_bridge: HTTP :{args.http_port} /cam1<-{args.front_port} '
          f'/cam2<-{args.bottom_port}', flush=True)
    print('[*] mjpeg_bridge: http://<board-ip>:{}/cam1|/cam2'.format(args.http_port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam1.stop()
        cam2.stop()


if __name__ == '__main__':
    main()
