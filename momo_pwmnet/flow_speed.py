#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_speed.py — 光流测速独立进程 (读 vp5.1 共享帧, 不占相机)
# ============================================================
# 与 vp5.1 配合: vp5.1 的 bottom producer 把解出的 NV12 最新帧写入
#   /dev/shm/momo_flow_bottom.bin (下视; 光流测速只读 bottom, front 不再写共享帧)
# 本进程从共享内存取帧, 在 Mali-G78AE GPU(OpenCL/UMat) 上做
#   稀疏金字塔 LK 光流 -> 图像平面速度统计 (px/帧), 不占 BPU/不加载深度。
#   完全不打开摄像头, 不影响 vp5.1 的 YOLO 帧率。
#
# vp5.1 GPU 测速改造 (2026-09-07):
#   - 光流整链路迁 GPU: goodFeaturesToTrack + calcOpticalFlowPyrLK 走 UMat(OCL),
#     实测(640x480): CPU 链路 248% CPU -> GPU 链路 27% CPU, 计算不再抢 front/bottom 核
#   - 移除深度链路: DepthSpeedMeter / DAV2 / depth_any.hbm 不再加载, BPU 全让给 YOLO
#   - 速度输出为图像平面速度 px/帧 + moving%(无深度无真实尺度);
#     [speed]*px_to_m 常数仅用于显示近似, 文档明确"非标定 m/s"
#   - 测速循环全速(40-50fps), Web MJPEG 独立 10-15fps 节流, JPEG 编码不拖累测速
#
# vp5.1 物理标定接入 (2026-09-09):
#   - config.ini 新增 [calib] 段: focal_px(焦距) + range_m(相机到场景平面距离)
#     按针孔模型横向平移近似换算: v(m/s) = 像素速度(px/帧) × (range_m/focal_px) / dt
#   - 前提: 相机平移 ⊥ 光轴、场景近似为距离 range_m 的平面(见 config.ini 注释);
#     沿光轴前进/旋转场景不适用, 精度要求高需 IMU 补偿或测距模型
#   - enable=false 时行为与 vp5.1 原版完全一致(仅 px/帧 + moving%)
#   - 默认开启"常见值占位标定"(focal≈554/range=1.0m), 输出持续标注, 真实标定后替换
#
# 用法:
#   python3 flow_speed.py [--fps 50] [--port 8080]
#     --fps  光流目标处理帧率(默认 50, 受 CPU/GPU 实际能力约束)
#     --port Web 推流端口, 默认 8080
# 参数(算法/测速)复用 /userdata/momo_pwmnet/config.ini
# ============================================================

import argparse
import configparser
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, "/userdata/vp/vp5.1/utils")     # flow_share 读端
from flow_share import FlowShareReader, FMT_NV12, FMT_BGR

CONFIG_PATH = os.path.join(_HERE, "config.ini")
MAX_PTS = 200                # 角点上限(与 sparse.max_corners 一致, 控 GPU 预算)
STREAM_FPS = 12.0            # MJPEG 推流独立节流(与测速解耦, 避免 JPEG 抢 CPU)


# ---------------- Web MJPEG (复用与原实现一致) ----------------
class MjpegStreamer:
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()

    def set_frame(self, jpg):
        with self._lock:
            self._frame = jpg

    def get_frame(self):
        with self._lock:
            return self._frame


def _make_handler(streamer, title="光流测速(下视共享帧 · GPU)"):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = ('<!DOCTYPE html><html><head><meta charset="utf-8">'
                        '<title>光流测速(共享帧)</title></head>'
                        '<body style="background:#111;text-align:center">'
                        f'<h3 style="color:#eee">{title}</h3>'
                        '<img src="/video.mjpg" style="max-width:95%"></body></html>')
                data = page.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path == "/video.mjpg":
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
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, fmt, *args):
            pass

    return Handler


def annotate(img, mean_motion, moving_ratio, px_to_m=None,
             motion_thr=1.2, tracked=-1, speed_mps=None):
    """画面左上叠加速度/运动文本(返回新图, 不改原图)。"""
    out = img.copy()
    if moving_ratio is None:
        cv2.putText(out, "flow: -- (waiting)", (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        return out
    moving_ratio = float(moving_ratio)
    txt = f"motion {mean_motion:5.2f} px/f  ({moving_ratio*100:3.0f}%)"
    if speed_mps is not None and speed_mps > 0:
        txt += f"  ~{speed_mps:5.2f} m/s(cal)"
    elif px_to_m:
        txt += f"  ~{mean_motion*px_to_m:5.2f} m/f(non-cal)"
    if tracked >= 0:
        txt += f"  [trk {tracked}]"
    cv2.putText(out, txt, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return out


def main():
    ap = argparse.ArgumentParser(description="光流测速独立进程 (vp5.1 共享帧 · GPU)")
    ap.add_argument("--fps", type=int, default=50, help="光流目标帧率(默认50)")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    cfg = configparser.ConfigParser()
    with open(CONFIG_PATH, encoding="utf-8") as f:
        cfg.read_file(f)

    if not cv2.ocl.haveOpenCL():
        print("[ERROR] 板上 cv2 无 OpenCL 后端, GPU 光流不可用", flush=True)
        return 4
    cv2.ocl.setUseOpenCL(True)

    # ---- 算法参数(全部来自 config.ini) ----
    max_corners = cfg.getint("sparse", "max_corners", fallback=MAX_PTS)
    quality_level = cfg.getfloat("sparse", "quality_level", fallback=0.01)
    min_distance = cfg.getint("sparse", "min_distance", fallback=8)
    win = cfg.getint("sparse", "win_size", fallback=21)
    max_level = cfg.getint("sparse", "max_level", fallback=2)
    blur_ksize = cfg.getint("motion", "blur_ksize", fallback=5)
    motion_thr = cfg.getfloat("motion", "motion_threshold", fallback=1.2)
    min_motion_ratio = cfg.getfloat("motion", "min_motion_ratio", fallback=0.05)
    process_scale = cfg.getfloat("speed", "process_scale", fallback=0.5)
    px_to_m = cfg.getfloat("speed", "px_to_m", fallback=0.0)  # 仅显示近似, 非标定
    mjpg_quality = cfg.getint("visualization", "mjpg_quality", fallback=80)
    # ---- 物理标定 [calib] (2026-09-09 新增): 光流 px/帧 -> 相机运动 m/s ----
    cal_enable = cfg.getboolean("calib", "enable", fallback=False)
    cal_focal_px = cfg.getfloat("calib", "focal_px", fallback=0.0)
    cal_range_m = cfg.getfloat("calib", "range_m", fallback=0.0)
    cal_source = cfg.get("calib", "source", fallback="")
    cal_nominal_fps = cfg.getfloat("calib", "nominal_fps", fallback=50.0)
    cal_use_nominal = cfg.getboolean("calib", "use_nominal_fps", fallback=False)
    # 尺度: 每个像素对应的真实距离 (m/px)。需 焦距>0 且 距离>0 才有效。
    cal_scale_mpp = (cal_range_m / cal_focal_px) if (
        cal_enable and cal_focal_px > 0 and cal_range_m > 0) else 0.0

    streamer = MjpegStreamer()
    try:
        httpd = ThreadingHTTPServer(("0.0.0.0", args.port), _make_handler(streamer))
    except OSError:
        print(f"[ERROR] 端口 {args.port} 被占用", flush=True)
        return 3
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"[INFO] Web: http://<板卡IP>:{args.port}/", flush=True)

    print("[INFO] 连接共享帧流 'bottom'(下视) ...", flush=True)
    rd = FlowShareReader("bottom")
    print(f"[INFO] 已连接: {rd.path} ({rd._w}x{rd._h} fmt={'NV12' if rd._fmt == FMT_NV12 else 'BGR'})", flush=True)
    if cal_scale_mpp > 0:
        print(f"[INFO] 物理标定: 焦距={cal_focal_px:.1f}px 距离={cal_range_m:.2f}m "
              f"尺度={cal_scale_mpp*1000:.3f}mm/px 来源={cal_source or 'n/a'} "
              f"({'固定' if cal_use_nominal else '实测'}帧率换算) [占位近似, 真实标定后改 config.ini [calib]]",
              flush=True)
    else:
        print("[INFO] 物理标定: 未启用 (config.ini [calib] enable=false 或参数缺失), 只输出 px/帧", flush=True)

    last_seq = -1
    prev_gray_p = None        # UMat, 处理尺度下
    prev_gray_np = None       # CPU numpy 备份(关键: 防 UMat buffer 池复用导致 p1==p0)
    prev_gray_full = None     # UMat, 原尺度(供特征检测参考, 可选)
    prev_t = None
    dt_smooth = 0.0
    t0 = time.time()
    n = 0
    mean_motion = 0.0
    moving_ratio = 0.0
    tracked = -1
    speed_mps = 0.0
    last_viz_t = 0.0
    target_dt = 1.0 / max(args.fps, 1)

    try:
        while True:
            loop_t = time.time()
            seq, img = rd.read(last_seq)
            if img is None:
                time.sleep(0.003)          # 无新帧: 微睡等 vp5.1
                continue
            last_seq = seq
            # 关键: read() 返回的是 mmap 共享内存视图; 不拷贝则下一帧读取时
            # writer 已覆盖该内存, prev/cur 会指向同一内容(光流永远 0 位移)
            img = img.copy()

            now = time.time()
            # ---- 取灰度: NV12 直接 Y 平面(零转换), BGR 转灰度 ----
            if rd._fmt == FMT_NV12:
                gray = np.ascontiguousarray(img[:rd._h, :])   # Y 平面
            else:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if blur_ksize > 1:
                gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)

            # ---- 下采样到处理尺度, 全部转 UMat(上传 GPU) ----
            gray_u = cv2.UMat(gray)
            if process_scale < 0.999:
                gray_p = cv2.resize(gray_u, None,
                                    fx=process_scale, fy=process_scale,
                                    interpolation=cv2.INTER_AREA)
            else:
                gray_p = gray_u

            if prev_gray_p is None or prev_t is None:
                prev_gray_np = gray_p.get().copy()   # CPU 备份(首帧)
                prev_gray_p = cv2.UMat(prev_gray_np) # UMat 由 CPU numpy 独立重建
                prev_t = now
                continue

            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 1e-3
            dt_smooth = dt if dt_smooth == 0 else 0.9 * dt_smooth + 0.1 * dt

            # ---- GPU: 特征检测 + 稀疏金字塔 LK ----
            try:
                pts = cv2.goodFeaturesToTrack(prev_gray_p, maxCorners=max_corners,
                                              qualityLevel=quality_level,
                                              minDistance=min_distance, blockSize=7)
                pts = cv2.UMat.get(pts) if isinstance(pts, cv2.UMat) else pts
                if pts is None or len(pts) == 0:
                    mean_motion, moving_ratio, tracked = 0.0, 0.0, 0
                else:
                    # 整链路 GPU: pts 也转 UMat 传入, 避免 prev/cur UMat + pts ndarray
                    # 混合时 OCL LK 不实际执行 (p1==p0) 的兼容问题
                    pts_lk = cv2.UMat(np.ascontiguousarray(pts, dtype=np.float32))
                    nxt, status, err = cv2.calcOpticalFlowPyrLK(
                        prev_gray_p, gray_p, pts_lk, None,
                        winSize=(win, win), maxLevel=max_level)
                    nxt_m = cv2.UMat.get(nxt) if isinstance(nxt, cv2.UMat) else nxt
                    st_m = cv2.UMat.get(status) if isinstance(status, cv2.UMat) else status
                    if st_m is None or nxt_m is None:
                        mean_motion, moving_ratio, tracked = 0.0, 0.0, 0
                        prev_gray_np = gray_p.get().copy()
                        prev_gray_p = cv2.UMat(prev_gray_np)
                        continue
                    ok = (st_m.ravel() == 1)
                    if not ok.any():
                        mean_motion, moving_ratio, tracked = 0.0, 0.0, 0
                    else:
                        p0 = pts.reshape(-1, 2)[ok]
                        p1 = nxt_m.reshape(-1, 2)[ok]
                        d = np.sqrt(((p1 - p0) ** 2).sum(axis=1)) / max(process_scale, 1e-6)
                        mean_motion = float(d.mean())
                        moving_ratio = float((d > motion_thr).mean())
                        tracked = int(ok.sum())
            except cv2.error as e:
                # 偶发 OCL 内部 error: 记一笔并继续, 不崩进程
                mean_motion, moving_ratio, tracked = 0.0, 0.0, 0
                if n % 50 == 0:
                    print(f"[WARN] GPU 光流异常: {e}", flush=True)

            # ---- 输出语义: px/帧(相对量) + moving% ----
            if moving_ratio < min_motion_ratio:
                out_motion = 0.0
            else:
                out_motion = mean_motion
            # ---- 物理速度: 像素位移 × 尺度(m/px) / 帧间隔 (m/s) ----
            if cal_scale_mpp > 0 and out_motion > 0:
                dt_cal = (1.0 / cal_nominal_fps) if cal_use_nominal else dt_smooth
                speed_mps = out_motion * cal_scale_mpp / max(dt_cal, 1e-3)
            else:
                speed_mps = 0.0

            n += 1
            if n % 10 == 0:
                txt = (f"[flow] f={n:6d} seq={seq} fps={1/dt_smooth:5.1f} "
                       f"mean={mean_motion:5.2f}px moving={moving_ratio*100:4.1f}% "
                       f"trk={tracked:3d}")
                if cal_scale_mpp > 0:
                    txt += f" v={speed_mps:5.2f}m/s(cal)"
                print(txt, flush=True)

            # ---- 可视化: 独立 10-15fps 节流, 不拖累测速 ----
            if now - last_viz_t >= 1.0 / STREAM_FPS:
                last_viz_t = now
                viz = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                viz = annotate(viz, out_motion, moving_ratio,
                               px_to_m=px_to_m if px_to_m > 0 else None,
                               motion_thr=motion_thr, tracked=tracked,
                               speed_mps=speed_mps)
                ok_enc, buf = cv2.imencode(".jpg", viz,
                                           [cv2.IMWRITE_JPEG_QUALITY, mjpg_quality])
                if ok_enc:
                    streamer.set_frame(buf.tobytes())

            # prev: 用 CPU numpy 备份重建 UMat, 完全绕开 OpenCV UMat buffer 池复用坑
            prev_gray_np = gray_p.get().copy()   # 备份当前帧供下次 LK(先备份, 再重建)
            prev_gray_p = cv2.UMat(prev_gray_np)
            # 帧率节流: 抽帧到 --fps (默认 50; 实际受 GPU 能力限制)
            spent = time.time() - loop_t
            if spent < target_dt:
                time.sleep(target_dt - spent)
    except KeyboardInterrupt:
        print("[INFO] 停止", flush=True)
    finally:
        httpd.shutdown()
        rd.close()
        elapsed = time.time() - t0
        if n > 0:
            print(f"summary: frames={n} avg_fps={n/elapsed:.1f}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())