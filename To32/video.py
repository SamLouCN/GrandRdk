#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""video.py — 中位机图像回传：USB 相机 -> HTTP MJPEG（上位机取流方式与 vp5.1 一致）

三机命名见 README_中位机.md：
    上位机 cv2.VideoCapture("http://<板端IP>:5000/cam1")
      <- 中位机 本模块（采集 + JPEG 编码 + MJPEG 服务）
      <- USB 相机 /dev/video0

与 vp5.1 的分工：
    vp5.1 = front/bottom(YOLO) -> streamer(:9000/:9001) -> mjpeg_bridge(:5000)，是「带检测框」的重链路；
    本模块 = main 内置的轻量图像回传（不跑模型），与 main.py 同进程同生命周期。
    两者都要独占 /dev/video*，不要同时启动。

$VID 语义（上位机 -> 中位机 UDP 8080，板端拦截、不下发下位机）：
    $VID,1# -> 开启图像回传（懒加载打开相机，开始采集/推流）
    $VID,0# -> 关闭图像回传（释放相机；已连接客户端改收占位帧，不断流不黑屏）

设计要点：
    每路 Source = 独立 daemon 线程：read -> imencode(JPEG) -> 覆盖式保存最新帧（带 seq）
    HTTP 多客户端各自按 VIDEO_FPS 节流取「最新帧」（丢旧保新，低延迟）
    某路打不开（如 /dev/video1 是元数据节点）-> 标记 offline，只发占位帧，不影响其它路
    HTTP 服务常驻（即使视频关着），保证上位机随时能连上 /cam1 /cam2
"""
from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

BOUNDARY = b"frame"


def _make_placeholder(text, w=640, h=360):
    """生成一张带文字的占位 JPEG（视频关/相机离线时用，避免客户端黑屏或断流）。"""
    img = np.full((h, w, 3), 32, dtype=np.uint8)
    cv2.putText(img, text, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (190, 190, 190), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes() if ok else b""


class Source(object):
    """单路相机源：可反复 start_capture / stop_capture，覆盖式保存最新 JPEG 帧。"""

    def __init__(self, name, device, cfg, log):
        self.name = name
        self.device = device
        self.cfg = cfg
        self.log = log
        self.online = False
        self.last_error = ""
        self.frames = 0
        self._jpeg = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = None
        self._thread = None
        self._placeholder = None

    # ---- 对外 ----
    def latest(self):
        """返回 (最新 JPEG, seq, online)。线程安全。"""
        with self._lock:
            return self._jpeg, self._seq, self.online

    def placeholder(self):
        """离线/关闭时的占位帧（惰性生成并缓存）。"""
        if self._placeholder is None:
            self._placeholder = _make_placeholder("%s offline" % self.name)
        return self._placeholder

    def start_capture(self):
        """启动采集线程（幂等；已在跑则直接返回）。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="Video-" + self.name, daemon=True)
        self._thread.start()

    def stop_capture(self):
        """停止采集并释放相机（阻塞至多 2s）。"""
        if self._stop is not None:
            self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        self._thread = None
        self.online = False
        with self._lock:
            self._jpeg = None
            self._seq += 1

    # ---- 采集线程 ----
    def _loop(self):
        w = int(getattr(self.cfg, "VIDEO_WIDTH", 1280))
        h = int(getattr(self.cfg, "VIDEO_HEIGHT", 720))
        fps = max(1, int(getattr(self.cfg, "VIDEO_FPS", 15)))
        quality = int(getattr(self.cfg, "VIDEO_QUALITY", 70))
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.online = False
            self.last_error = "open failed"
            self.log("[VIDEO] %s 打开失败: %s -> 只用占位帧" % (self.name, self.device))
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        params = [cv2.IMWRITE_JPEG_QUALITY, quality]
        self.online = True
        self.last_error = ""
        self.log("[VIDEO] %s 已打开 %s (%dx%d@%dfps q%d)"
                 % (self.name, self.device, w, h, fps, quality))
        fail = 0
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    fail += 1
                    if fail >= 30:
                        self.online = False
                        self.last_error = "read failed x%d" % fail
                        self.log("[VIDEO] %s 连续读失败 -> 转占位帧" % self.name)
                        break
                    time.sleep(0.02)
                    continue
                fail = 0
                okj, buf = cv2.imencode(".jpg", frame, params)
                if okj:
                    data = buf.tobytes()
                    with self._lock:
                        self._jpeg = data
                        self._seq += 1
                        self.frames += 1
        finally:
            try:
                cap.release()
            except Exception:
                pass
            self.online = False
            self.log("[VIDEO] %s 采集线程退出（累计 %d 帧）" % (self.name, self.frames))


class _Handler(BaseHTTPRequestHandler):
    """MJPEG 处理器：每个客户端循环取「最新帧」按节流发送；关/离线时发占位帧。"""

    routes = {}
    svc = None

    def do_GET(self):
        path = self.path.split("?")[0].rstrip("/") or "/"
        src = _Handler.routes.get(path)
        if src is None:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode())
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        fps = max(1, int(getattr(_Handler.svc.cfg, "VIDEO_FPS", 15)))
        interval = 1.0 / fps
        last = -1
        try:
            while True:
                jpeg, seq, _online = src.latest()
                if not _Handler.svc.enabled() or jpeg is None:
                    frame, delay = src.placeholder(), 0.5      # 关/离线：低频占位
                else:
                    if seq == last:
                        time.sleep(0.01)
                        continue
                    frame, delay, last = jpeg, interval, seq
                try:
                    self.wfile.write(b"--" + BOUNDARY +
                                     b"\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
                except OSError:
                    return
                time.sleep(delay)
        except Exception:
            pass

    def log_message(self, *args):
        """屏蔽默认访问日志（每请求一行会刷屏）。"""
        pass


class VideoService(object):
    """图像回传总服务：HTTP MJPEG 常驻 + 各路采集线程按 $VID 开关。"""

    def __init__(self, cfg, log):
        self.cfg = cfg
        self.log = log
        self.port = int(getattr(cfg, "VIDEO_HTTP_PORT", 5000))
        self.bind = str(getattr(cfg, "VIDEO_BIND_IP", "0.0.0.0") or "0.0.0.0")
        paths = dict(getattr(cfg, "VIDEO_PATHS", {"cam1": "/dev/video0"}))
        self.sources = {}
        for name in sorted(paths):
            self.sources["/" + name] = Source(name, paths[name], cfg, log)
        self._enabled = False
        self._lock = threading.Lock()
        self._http = None

    # ---- 状态 ----
    def enabled(self):
        return self._enabled

    def summary(self):
        """一行状态摘要（用于 mode_dispatcher 的周期汇报）。"""
        parts = []
        for path in sorted(self.sources):
            s = self.sources[path]
            parts.append("%s=%s(%d帧)" % (path.lstrip("/"),
                                          "online" if s.online else "offline", s.frames))
        return "%s | %s" % ("开" if self._enabled else "关", " ".join(parts))

    # ---- 生命周期 ----
    def start(self):
        """启动 HTTP 服务（常驻），并按配置决定是否立即开始采集。"""
        _Handler.routes = self.sources
        _Handler.svc = self
        try:
            self._http = ThreadingHTTPServer((self.bind, self.port), _Handler)
            self._http.daemon_threads = True
        except OSError as e:
            self.log("[VIDEO] HTTP :%d 绑定失败: %s -> 图像回传不可用" % (self.port, e))
            self._http = None
            return
        threading.Thread(target=self._http.serve_forever, name="VideoHTTP", daemon=True).start()
        self.log("[VIDEO] HTTP MJPEG 已监听 :%d %s" % (self.port, " ".join(sorted(self.sources))))
        if bool(getattr(self.cfg, "VIDEO_ENABLED_AT_START", True)):
            self.set_enabled(True)
        else:
            self.log("[VIDEO] 初始为关（等上位机 $VID,1#）")

    def set_enabled(self, on):
        """$VID 开关：开=启动采集；关=停采集并释放相机（客户端改收占位帧）。"""
        on = bool(on)
        with self._lock:
            if on == self._enabled:
                return
            self._enabled = on
            for src in self.sources.values():
                if on:
                    src.start_capture()
                else:
                    src.stop_capture()
        self.log("[VIDEO] 图像回传 %s" % ("开启（$VID,1）" if on else "关闭（$VID,0，已释放相机）"))

    def stop(self):
        self.set_enabled(False)
        if self._http is not None:
            try:
                self._http.shutdown()
                self._http.server_close()
            except Exception:
                pass
            self._http = None
        self.log("[VIDEO] 已停止")


if __name__ == "__main__":
    # 独立调试：python3 video.py --cam1 /dev/video0 [--off]
    import argparse
    import types as _types

    ap = argparse.ArgumentParser(description="图像回传独立调试（HTTP MJPEG）")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--cam1", default="/dev/video0")
    ap.add_argument("--cam2", default="/dev/video1")
    ap.add_argument("--off", action="store_true", help="启动时先不采集")
    a = ap.parse_args()

    c = _types.SimpleNamespace(
        VIDEO_HTTP_PORT=a.port, VIDEO_BIND_IP="0.0.0.0",
        VIDEO_WIDTH=1280, VIDEO_HEIGHT=720, VIDEO_FPS=15, VIDEO_QUALITY=70,
        VIDEO_PATHS={"cam1": a.cam1, "cam2": a.cam2},
        VIDEO_ENABLED_AT_START=(not a.off),
    )

    def _p(msg):
        print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)

    svc = VideoService(c, _p)
    svc.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        svc.stop()