#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# speed_flow_demo.py - 光流 + BPU 深度测速(m/s)演示
# ============================================================
# 设计:
#   1. 本文件用于"速度测量": 固定相机 + 画面中物体运动
#   2. 光流: DIS 稠密光流(CPU, 逐像素 px/帧)
#   3. 深度: Depth Anything V2(官方 BPU 模型, 后台线程刷新, 不阻塞主循环)
#      -> 相对深度经仿射标定(Z = depth_a*rel + depth_b)转米
#   4. 融合: v(m/s) = |flow|(px/帧) * Z(米) / focal_px / dt(秒)
#      dt 用每帧实测时间间隔, 不用写死的 real_fps, 丢帧/变帧率自动正确
#   5. 可视化: Web MJPEG(http://<板卡IP>:web_port/) + 可选桌面窗口
# 运行: python3 speed_flow_demo.py (参数全部来自 config.ini)
# 抗噪措施([motion] 段): blur_ksize + motion_threshold + min_motion_ratio
# 处理尺度: speed.process_scale<1 时光流在半分辨率算, 快 ~4 倍,
#   速度换算回原图像素后参与公式, 结果与全分辨率等价
# 注意: 未标定前 depth_a=1 depth_b=0, 输出为"相对单位"速度
# ============================================================

import configparser        # 标准库: 解析 config.ini
import os                  # 标准库: sys.path 拼接
import socket              # 标准库: 探测本机 IP
import sys                 # 标准库: 路径与退出
import threading           # 标准库: 后台 Web 服务
import time                # 标准库: 帧率计时 + 实测 dt
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # MJPEG

import cv2                 # OpenCV: 摄像头 / DIS 光流
import numpy as np         # NumPy: 光流场运算

CONFIG_PATH = "config.ini"


def load_config(path):
    cfg = configparser.ConfigParser()
    cfg.read(path, encoding="utf-8")
    return cfg


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def open_camera(cfg):
    """打开摄像头(失败重试一次后退出)。"""
    dev = cfg.getint("camera", "device_index")
    w = cfg.getint("camera", "width")
    h = cfg.getint("camera", "height")
    fps = cfg.getint("camera", "fps")
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[INFO] 摄像头打开失败, 2 秒后重试...")
        time.sleep(2)
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERROR] 无法打开摄像头 /dev/video{dev}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)                # 请求高帧率, 实际能到多快看日志
    # 把缓冲区缩到 1, 减少"卡帧"导致的 dt 虚高
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class MjpegStreamer:
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()
        self._httpd = None
        self._thread = None

    def set_frame(self, jpg):
        with self._lock:
            self._frame = jpg

    def get_frame(self):
        with self._lock:
            return self._frame

    def start(self, port):
        httpd = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(self))
        self._httpd = httpd
        self._thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()


def _make_handler(streamer):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._serve_page()
            elif self.path == "/video.mjpg":
                self._serve_stream()
            else:
                self.send_response(404)
                self.end_headers()

        def _serve_page(self):
            page = ('<!DOCTYPE html><html><head><meta charset="utf-8">'
                    '<title>RDK S100 光流测速</title></head>'
                    '<body style="background:#111;text-align:center">'
                    '<h3 style="color:#eee">光流 + 深度测速(m/s)</h3>'
                    '<img src="/video.mjpg" style="max-width:95%"></body></html>')
            data = page.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _serve_stream(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = streamer.get_frame()
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: " +
                                         str(len(jpg)).encode() + b"\r\n\r\n")
                        self.wfile.write(jpg + b"\r")
                    time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    cfg = load_config(CONFIG_PATH)

    # --- 深度测速器 ---
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "depth"))
    from speed_fusion import DepthSpeedMeter
    meter = DepthSpeedMeter(cfg)

    # --- DIS 光流器 ---
    dis = cv2.DISOpticalFlow_create(cfg.getint("dis", "preset"))
    dis.setGradientDescentIterations(cfg.getint("dis", "gradient_descent_iterations"))

    # --- 抗噪 + 处理尺度 ---
    blur_ksize = cfg.getint("motion", "blur_ksize")
    min_motion_ratio = cfg.getfloat("motion", "min_motion_ratio")
    texture_thr = cfg.getfloat("motion", "texture_threshold")
    process_scale = cfg.getfloat("speed", "process_scale")   # 光流计算尺度(0~1)

    cap = open_camera(cfg)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cam_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] 摄像头: /dev/video{cfg.getint('camera', 'device_index')} "
          f"{fw}x{fh} 请求 fps={cfg.getint('camera', 'fps')} 协商 fps={cam_fps:.1f}")
    print(f"[INFO] 深度测速: {'启用' if meter.enable else '未启用'}, "
          f"process_scale={process_scale} focal={meter.focal_px:.1f}")

    enable_window = cfg.getboolean("visualization", "enable_window")
    enable_web = cfg.getboolean("visualization", "enable_web")
    web_port = cfg.getint("visualization", "web_port")
    mjpg_quality = cfg.getint("visualization", "mjpg_quality")
    save_path = cfg.get("output", "save_image").strip()
    log_interval = cfg.getint("output", "log_interval")
    max_frames = cfg.getint("camera", "max_frames")

    streamer = None
    if enable_web:
        streamer = MjpegStreamer()
        try:
            streamer.start(web_port)
        except OSError:
            print(f"[ERROR] Web 端口 {web_port} 被占用, 改 config.ini 的 web_port")
            return 4
        print(f"[INFO] Web 可视化地址: http://{get_local_ip()}:{web_port}/")

    window_ok = False
    if enable_window and os.environ.get("DISPLAY"):
        window_ok = True

    t0 = time.time()
    frame_count = 0
    prev_t = None                # 上一帧时间戳
    prev_gray_p = None           # 上一帧(处理尺度)灰度
    last_viz = None
    median_mps = None
    moving_ratio = 0.0
    dt_smooth = 0.0              # 滑动平均 dt, 用于打印与兜底

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[ERROR] 读取摄像头帧失败")
                return 2

            now = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if blur_ksize > 1:
                gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)

            # 处理尺度缩放(光流输入)
            if process_scale < 0.999:
                gray_p = cv2.resize(gray, None,
                                    fx=process_scale, fy=process_scale,
                                    interpolation=cv2.INTER_AREA)
            else:
                gray_p = gray

            if prev_gray_p is None or prev_t is None:
                prev_gray_p = gray_p
                prev_t = now
                continue

            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 1e-3
            if dt_smooth == 0:
                dt_smooth = dt
            else:
                dt_smooth = 0.9 * dt_smooth + 0.1 * dt    # 滑动平均

            # 1) DIS 稠密光流(在处理尺度上)
            flow_p = dis.calc(prev_gray_p, gray_p, None)
            # 原图尺度位移(用于 mean_motion 日志与屏幕叠加)
            mag_orig = np.sqrt(flow_p[..., 0] ** 2 + flow_p[..., 1] ** 2) / process_scale
            mean_motion = float(mag_orig.mean())

            # 纹理掩码: 低梯度平板区域(DIS 孔径问题)不计入运动
            gx = cv2.Sobel(gray_p, cv2.CV_32F, 1, 0)
            gy = cv2.Sobel(gray_p, cv2.CV_32F, 0, 1)
            tex_mask = (np.sqrt(gx * gx + gy * gy) > texture_thr)

            # 2) 深度(后台线程刷新, 不阻塞主循环)
            meter.update(frame)

            # 3) 速度: 用实测 dt, 考虑处理尺度
            median_mps, mean_mps, moving_ratio = meter.measure(
                flow_p, dt=dt_smooth, scale=process_scale, tex_mask=tex_mask)
            if moving_ratio is not None and moving_ratio < min_motion_ratio:
                median_mps = 0.0        # 静止: 抑制底噪

            # 4) 可视化
            viz = frame.copy()
            viz = DepthSpeedMeter.annotate(viz, median_mps, moving_ratio)
            last_viz = viz

            frame_count += 1
            actual_fps = 1.0 / dt_smooth
            if frame_count % log_interval == 0:
                spd = "--" if median_mps is None else f"{median_mps:5.2f} m/s"
                mr_pct = 0.0 if moving_ratio is None else moving_ratio * 100
                print(f"[speed] f={frame_count:5d} fps={actual_fps:5.1f} "
                      f"dt={dt*1000:5.1f}ms mean={mean_motion:5.2f}px "
                      f"moving={mr_pct:4.1f}% median_speed={spd}")

            if streamer is not None:
                ok_enc, buf = cv2.imencode(".jpg", viz,
                                           [cv2.IMWRITE_JPEG_QUALITY, mjpg_quality])
                if ok_enc:
                    streamer.set_frame(buf.tobytes())
            if window_ok:
                cv2.imshow("RDK S100 光流测速", viz)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            prev_gray_p = gray_p
            if max_frames > 0 and frame_count >= max_frames:
                break
    except KeyboardInterrupt:
        print("[INFO] 收到中断信号, 结束采集")

    elapsed = time.time() - t0
    avg_fps = frame_count / elapsed if elapsed > 0 else 0.0
    spd = "--" if median_mps is None else f"{median_mps:.2f}"
    mr = 0.0 if moving_ratio is None else moving_ratio
    print("=" * 60)
    print(f"summary: frames={frame_count} avg_fps={avg_fps:.1f} "
          f"final_median_speed={spd} m/s moving_ratio={mr:.3f}")
    if save_path and last_viz is not None:
        cv2.imwrite(save_path, last_viz)
        print(f"[save] {save_path}")
    if window_ok:
        cv2.destroyAllWindows()
    if streamer is not None:
        streamer.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())