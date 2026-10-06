#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_speed.py — 光流测速独立进程 (读本目录共享帧, 不占相机)
# ============================================================
# ⚠ [2026-10-04 光流停用] 本进程已不再由 run.sh 拉起, 且写端 bottom.py 也不再写
#    /dev/shm/momo_flow_bottom.bin —— 即"计算 + 共享"两条路都已停用。
#    本文件保留未删, 便于将来恢复: 恢复时把 run.sh 的 FLOW_DISABLED 置 0、
#    config/main_config.py 的 ENABLE_FLOW_SHARE_BOTTOM 置 True 即可。
#    ⚠ 图像回传(/cam1 /cam2)不经过本文件, 停用本进程对回传无影响。
# ============================================================
# 位置: /userdata/GrandRDK/src/flow_speed.py (2026-09-17 由 /userdata/momo_pwmnet 迁入)
# 数据源: bottom.py 写入的共享帧 /dev/shm/momo_flow_bottom.bin (NV12)
#         —— 由 config/main_config.ENABLE_FLOW_SHARE_BOTTOM 控制; front 默认不写
#         —— 读写接口: 本目录 utils/flow_share.py (FlowShareReader / FlowShareWriter)
# 参数: 全部来自 config/main_config.py 的 FLOW_* 键 (2026-09-17 由 config.ini 融入)
# 计算: Mali-G78AE GPU (OpenCL/UMat) 稀疏金字塔 LK 光流 -> 图像平面速度 px/帧 + moving%
#       不占 BPU、不打开摄像头, 不影响 front/bottom 的 YOLO 帧率
#
# 物理标定 (main_config.FLOW_CALIB_*, 针孔模型: 相机平移 ⊥ 光轴、场景近似 range_m 平面):
#   v(m/s) = 像素位移(px/帧) × (range_m / focal_px) / dt
#   FLOW_CALIB_ENABLE=false 时只输出 px/帧 + moving% (无物理尺度)
#
# 用法:
#   python3 src/flow_speed.py [--fps 50] [--port 8000]
#     --fps  : 光流目标处理帧率 (省略则用 main_config.FLOW_FPS)
#     --port : Web MJPEG 调试推流端口 (省略则用 main_config.FLOW_WEB_PORT)
#   浏览器: http://<板卡IP>:<port>/  (测速画面叠加, 独立 ~12fps 节流)
#
# 前提: bottom 在写共享帧 (main_config.ENABLE_FLOW_SHARE_BOTTOM=True)
# 启动顺序(重要, 2026-09-17 实测):
#   必须先起 bottom(它创建/截断共享帧), 再起本进程;
#   bottom 重启后必须一并重启本进程 —— 写入端重建会把 seq 归零, 而 reader 判定
#   seq <= last_seq 即丢弃, 于是旧实例会永久收不到新帧(静默不出数、也不报错)。
# ============================================================

import argparse  # 解析 --fps/--port 命令行参数
import os  # 拼路径、注入 sys.path
import sys  # 模块搜索路径与退出码
import threading  # Web 推流帧的互斥锁
import time  # 计时: dt、帧率节流、可视化节流
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # 调试用 MJPEG 服务

import cv2  # OpenCV: UMat/OpenCL 光流 + JPEG 编码
import numpy as np  # 数组运算: 位移距离、统计

# ---- 路径注入: config/ + src/ + src/utils/ (与 web_server.py 一致) ----
_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录 src/
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 工程根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE, os.path.join(_HERE, 'utils')):  # 依次优先: config、src、src/utils
    if _p not in sys.path:
        sys.path.insert(0, _p)  # 插到最前, 保证解析到工程内的同名模块

import main_config as MC  # 全部 FLOW_* 参数来源
from flow_share import FlowShareReader, FMT_NV12, FMT_BGR  # 共享内存帧读取器与格式常量

MAX_PTS = 200                # 角点上限兜底(与 FLOW_MAX_CORNERS 同义, 控 GPU 预算)


# ---------------- Web MJPEG (复用与原实现一致) ----------------
class MjpegStreamer:
    def __init__(self):
        self._frame = None  # 最新一帧 JPEG, None 表示还没出图
        self._lock = threading.Lock()  # 保护 _frame 的读写互斥

    def set_frame(self, jpg):
        with self._lock:
            self._frame = jpg  # 覆盖最新帧, 旧帧直接丢弃

    def get_frame(self):
        with self._lock:
            return self._frame  # 取当前帧(可能仍为 None)


def _make_handler(streamer, title="光流测速(下视共享帧 · GPU)"):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = ('<!DOCTYPE html><html><head><meta charset="utf-8">'  # 页面头部
                        '<title>光流测速(共享帧)</title></head>'  # 标题
                        '<body style="background:#111;text-align:center">'  # 黑底居中布局
                        f'<h3 style="color:#eee">{title}</h3>'  # 显示外部传入标题
                        '<img src="/video.mjpg" style="max-width:95%"></body></html>')  # 图片标签指向 MJPEG 流
                data = page.encode("utf-8")  # 中文标题按 utf-8 编码
                self.send_response(200)  # 返回页面
                self.send_header("Content-Type", "text/html; charset=utf-8")  # 声明 HTML 编码
                self.send_header("Content-Length", str(len(data)))  # 字节长度
                self.end_headers()  # 头部结束
                self.wfile.write(data)  # 写正文
            elif self.path == "/video.mjpg":
                self.send_response(200)  # 开始 MJPEG 流
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")  # multipart 替换式流
                self.end_headers()  # 头部结束(长连接持续推帧)
                try:
                    while True:
                        jpg = streamer.get_frame()  # 取最新预览帧
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\n")  # 分块起始边界
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")  # 帧类型
                            self.wfile.write(b"Content-Length: " +
                                             str(len(jpg)).encode() + b"\r\n\r\n")  # 帧长度 + 头部结束空行
                            self.wfile.write(jpg + b"\r")  # 帧数据(注意此处只补 \r, 未按惯例补 \r\n)
                        time.sleep(0.03)  # 约 30fps 的轮询节奏
                except (BrokenPipeError, ConnectionResetError):
                    pass  # 浏览器关闭/刷新导致的断连, 正常忽略
            else:
                self.send_response(404)  # 其它路径一律 404
                self.end_headers()  # 404 也要结束头部

        def log_message(self, fmt, *args):
            pass  # 屏蔽默认访问日志, 避免刷屏

    return Handler  # 返回绑定了 streamer 的处理器类


def annotate(img, mean_motion, moving_ratio, px_to_m=None,
             motion_thr=1.2, tracked=-1, speed_mps=None):
    """画面左上叠加速度/运动文本(返回新图, 不改原图)。"""
    out = img.copy()  # 复制后再画, 避免污染原帧
    if moving_ratio is None:
        cv2.putText(out, "flow: -- (waiting)", (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)  # 还没算出结果时先提示等待
        return out
    moving_ratio = float(moving_ratio)  # 统一转 float, 防 None/整型除法的坑
    txt = f"motion {mean_motion:5.2f} px/f  ({moving_ratio*100:3.0f}%)"  # 主信息: 平均位移 + 运动点占比
    if speed_mps is not None and speed_mps > 0:
        txt += f"  ~{speed_mps:5.2f} m/s(cal)"  # 有标定过的物理速度就显示 m/s
    elif px_to_m:
        txt += f"  ~{mean_motion*px_to_m:5.2f} m/f(non-cal)"  # 只有换算系数时标记为非标定
    if tracked >= 0:
        txt += f"  [trk {tracked}]"  # 追加成功跟踪的点数
    cv2.putText(out, txt, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)  # 绿色文字画在左上角
    return out


def main():
    ap = argparse.ArgumentParser(description="光流测速独立进程 (GrandRDK 共享帧 · GPU)")
    ap.add_argument("--fps", type=int, default=int(getattr(MC, "FLOW_FPS", 50)),
                    help="光流目标帧率(默认取 main_config.FLOW_FPS)")  # 目标处理帧率
    ap.add_argument("--port", type=int, default=int(getattr(MC, "FLOW_WEB_PORT", 8000)),
                    help="Web MJPEG 端口(默认取 main_config.FLOW_WEB_PORT)")  # 调试推流端口
    args = ap.parse_args()  # 解析命令行

    # ---- 算法参数(全部来自 config/main_config.py 的 FLOW_* 键) ----
    share_name = str(getattr(MC, "FLOW_SHARE_NAME", "bottom") or "bottom")  # 共享帧名字, 对应 bottom 写端
    max_corners = int(getattr(MC, "FLOW_MAX_CORNERS", MAX_PTS))  # 最多跟踪角点数
    quality_level = float(getattr(MC, "FLOW_QUALITY_LEVEL", 0.01))  # 角点质量阈值, 越小点越多
    min_distance = int(getattr(MC, "FLOW_MIN_DISTANCE", 8))  # 角点间最小像素间距
    win = int(getattr(MC, "FLOW_WIN_SIZE", 21))  # LK 光流搜索窗口边长(奇数)
    max_level = int(getattr(MC, "FLOW_MAX_LEVEL", 2))  # 金字塔层数, 越大可测位移越大
    blur_ksize = int(getattr(MC, "FLOW_BLUR_KSIZE", 5))  # 预处理高斯模糊核大小
    motion_thr = float(getattr(MC, "FLOW_MOTION_THRESHOLD", 1.2))  # 判定为"在动"的像素位移阈值
    min_motion_ratio = float(getattr(MC, "FLOW_MIN_MOTION_RATIO", 0.05))  # 运动点占比下限, 低于则视为静止
    process_scale = float(getattr(MC, "FLOW_PROCESS_SCALE", 0.5))  # 处理分辨率缩放: 0.5=半尺寸省算力
    px_to_m = float(getattr(MC, "FLOW_PX_TO_M", 0.0))       # 仅显示近似, 非标定
    mjpg_quality = int(getattr(MC, "FLOW_MJPEG_QUALITY", 80))  # 调试流 JPEG 质量
    stream_fps = float(getattr(MC, "FLOW_STREAM_FPS", 12.0) or 12.0)  # 可视化推流帧率上限
    # ---- 物理标定 FLOW_CALIB_*: 光流 px/帧 -> 相机运动 m/s ----
    cal_enable = bool(getattr(MC, "FLOW_CALIB_ENABLE", False))  # 是否启用物理标定
    cal_focal_px = float(getattr(MC, "FLOW_CALIB_FOCAL_PX", 0.0) or 0.0)  # 等效焦距(像素)
    cal_range_m = float(getattr(MC, "FLOW_CALIB_RANGE_M", 0.0) or 0.0)  # 对地/对景距离(米)
    cal_source = str(getattr(MC, "FLOW_CALIB_SOURCE", "") or "")  # 标定来源说明, 仅用于日志
    cal_nominal_fps = float(getattr(MC, "FLOW_CALIB_NOMINAL_FPS", 50.0) or 50.0)  # 固定帧率换算用的名义帧率
    cal_use_nominal = bool(getattr(MC, "FLOW_CALIB_USE_NOMINAL_FPS", False))  # True=用名义帧率, False=用实测 dt
    # 尺度: 每个像素对应的真实距离 (m/px)。需 焦距>0 且 距离>0 才有效。
    cal_scale_mpp = (cal_range_m / cal_focal_px) if (
        cal_enable and cal_focal_px > 0 and cal_range_m > 0) else 0.0  # m/px; 参数不全时为 0 表示无物理尺度

    streamer = MjpegStreamer()  # 调试画面中转对象
    try:
        httpd = ThreadingHTTPServer(("0.0.0.0", args.port), _make_handler(streamer))  # 监听指定端口
    except OSError:
        print(f"[ERROR] 端口 {args.port} 被占用", flush=True)  # 端口冲突: 直接失败退出
        return 3  # 退出码 3 = 端口占用
    threading.Thread(target=httpd.serve_forever, daemon=True).start()  # 后台守护线程跑 HTTP
    print(f"[INFO] Web: http://<板卡IP>:{args.port}/", flush=True)

    print(f"[INFO] 连接共享帧流 '{share_name}' ...", flush=True)
    rd = FlowShareReader(share_name)  # 连接共享内存读端
    print(f"[INFO] 已连接: {rd.path} ({rd._w}x{rd._h} fmt={'NV12' if rd._fmt == FMT_NV12 else 'BGR'})", flush=True)  # 打印分辨率与格式
    if cal_scale_mpp > 0:
        print(f"[INFO] 物理标定: 焦距={cal_focal_px:.1f}px 距离={cal_range_m:.2f}m "
              f"尺度={cal_scale_mpp*1000:.3f}mm/px 来源={cal_source or 'n/a'} "
              f"({'固定' if cal_use_nominal else '实测'}帧率换算) [占位近似, 真实标定后改 main_config.FLOW_CALIB_*]",
              flush=True)
    else:
        print("[INFO] 物理标定: 未启用 (FLOW_CALIB_ENABLE=false 或参数缺失), 只输出 px/帧", flush=True)

    last_seq = -1  # 已处理的最新帧序号, -1 表示还没收到过有效帧
    prev_gray_p = None        # UMat, 处理尺度下
    prev_gray_np = None       # CPU numpy 备份(关键: 防 UMat buffer 池复用导致 p1==p0)
    prev_gray_full = None     # UMat, 原尺度(供特征检测参考, 可选)
    prev_t = None  # 上一帧时间戳, 用于算 dt
    dt_smooth = 0.0  # 平滑后的帧间隔(秒)
    t0 = time.time()  # 进程起始时刻, 用于算平均帧率
    n = 0  # 已处理帧数
    mean_motion = 0.0  # 本帧平均像素位移
    moving_ratio = 0.0  # 本帧运动点占比
    tracked = -1  # 有效跟踪点数, -1 表示未知
    speed_mps = 0.0  # 换算出的对地速度(m/s)
    last_viz_t = 0.0  # 上次出可视化画面的时刻
    target_dt = 1.0 / max(args.fps, 1)  # 每帧目标耗时, max 防除零

    try:
        while True:
            loop_t = time.time()  # 本轮开始时刻, 用于末尾节流
            seq, img = rd.read(last_seq)  # 读比 last_seq 新的帧, 无新帧返回 None
            if img is None:
                time.sleep(0.003)          # 无新帧: 微睡等 producer
                continue
            last_seq = seq  # 更新序号, 下次只读更新的帧
            # 关键: read() 返回的是 mmap 共享内存视图; 不拷贝则下一帧读取时
            # writer 已覆盖该内存, prev/cur 会指向同一内容(光流永远 0 位移)
            img = img.copy()  # 立刻拷出来, 脱离共享内存

            now = time.time()
            # ---- 取灰度: NV12 直接 Y 平面(零转换), BGR 转灰度 ----
            if rd._fmt == FMT_NV12:
                gray = np.ascontiguousarray(img[:rd._h, :])   # Y 平面
            else:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # BGR 需转灰度
            if blur_ksize > 1:
                gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)  # 降噪, 抑制水面反光带来的误光流

            # ---- 下采样到处理尺度, 全部转 UMat(上传 GPU) ----
            gray_u = cv2.UMat(gray)  # 上传到 GPU 显存
            if process_scale < 0.999:
                gray_p = cv2.resize(gray_u, None,
                                    fx=process_scale, fy=process_scale,
                                    interpolation=cv2.INTER_AREA)  # 面积插值下采样, 抗锯齿且更快
            else:
                gray_p = gray_u  # 缩放为 1 时直接复用, 省一次拷贝

            if prev_gray_p is None or prev_t is None:
                prev_gray_np = gray_p.get().copy()   # CPU 备份(首帧)
                prev_gray_p = cv2.UMat(prev_gray_np) # UMat 由 CPU numpy 独立重建
                prev_t = now  # 只记时间, 首帧没有可比对象
                continue  # 首帧只做基准帧, 不计算光流

            dt = now - prev_t  # 本帧与上一帧的真实时间间隔
            prev_t = now  # 滚动时间戳
            if dt <= 0:
                dt = 1e-3  # 时钟回拨/相同时间戳的兜底, 防除零
            dt_smooth = dt if dt_smooth == 0 else 0.9 * dt_smooth + 0.1 * dt  # 一阶低通, 抑制 dt 抖动

            # ---- GPU: 特征检测 + 稀疏金字塔 LK ----
            try:
                pts = cv2.goodFeaturesToTrack(prev_gray_p, maxCorners=max_corners,
                                              qualityLevel=quality_level,
                                              minDistance=min_distance, blockSize=7)  # 在上一帧上找角点(blockSize=7 为默认)
                pts = cv2.UMat.get(pts) if isinstance(pts, cv2.UMat) else pts  # 结果可能是 UMat, 统一取回 numpy
                if pts is None or len(pts) == 0:
                    mean_motion, moving_ratio, tracked = 0.0, 0.0, 0  # 没角点(如纯水面)则本次记 0
                else:
                    # 整链路 GPU: pts 也转 UMat 传入, 避免 prev/cur UMat + pts ndarray
                    # 混合时 OCL LK 不实际执行 (p1==p0) 的兼容问题
                    pts_lk = cv2.UMat(np.ascontiguousarray(pts, dtype=np.float32))  # 角点也上传 GPU, 保证类型一致
                    nxt, status, err = cv2.calcOpticalFlowPyrLK(
                        prev_gray_p, gray_p, pts_lk, None,
                        winSize=(win, win), maxLevel=max_level)  # 稀疏金字塔 LK 跟踪
                    nxt_m = cv2.UMat.get(nxt) if isinstance(nxt, cv2.UMat) else nxt  # 取回跟踪结果点
                    st_m = cv2.UMat.get(status) if isinstance(status, cv2.UMat) else status  # 取回逐点状态位
                    if st_m is None or nxt_m is None:
                        mean_motion, moving_ratio, tracked = 0.0, 0.0, 0  # 光流异常: 本帧记 0
                        prev_gray_np = gray_p.get().copy()  # 仍要滚动基准帧, 否则下一帧 dt 跨太远
                        prev_gray_p = cv2.UMat(prev_gray_np)
                        continue
                    ok = (st_m.ravel() == 1)  # status==1 表示跟踪成功
                    if not ok.any():
                        mean_motion, moving_ratio, tracked = 0.0, 0.0, 0  # 全部跟丢
                    else:
                        p0 = pts.reshape(-1, 2)[ok]  # 成功点的上一帧坐标
                        p1 = nxt_m.reshape(-1, 2)[ok]  # 成功点的当前帧坐标
                        d = np.sqrt(((p1 - p0) ** 2).sum(axis=1)) / max(process_scale, 1e-6)  # 每点位移, 折算回原尺度
                        mean_motion = float(d.mean())  # 平均像素位移 px/帧
                        moving_ratio = float((d > motion_thr).mean())  # 位移超阈值的点占比
                        tracked = int(ok.sum())  # 有效跟踪点数
            except cv2.error as e:
                # 偶发 OCL 内部 error: 记一笔并继续, 不崩进程
                mean_motion, moving_ratio, tracked = 0.0, 0.0, 0  # 异常帧不参与统计
                if n % 50 == 0:
                    print(f"[WARN] GPU 光流异常: {e}", flush=True)  # 每 50 帧报一次, 避免刷屏

            # ---- 输出语义: px/帧(相对量) + moving% ----
            if moving_ratio < min_motion_ratio:
                out_motion = 0.0  # 运动点太少判为悬停/抖噪, 输出归零
            else:
                out_motion = mean_motion  # 达到占比才认可这一帧位移
            # ---- 物理速度: 像素位移 × 尺度(m/px) / 帧间隔 (m/s) ----
            if cal_scale_mpp > 0 and out_motion > 0:
                dt_cal = (1.0 / cal_nominal_fps) if cal_use_nominal else dt_smooth  # 名义帧率 vs 实测帧率
                speed_mps = out_motion * cal_scale_mpp / max(dt_cal, 1e-3)  # px/帧 → m/s
            else:
                speed_mps = 0.0  # 未标定或静止时速度为 0

            n += 1  # 计数 +1
            if n % 10 == 0:
                txt = (f"[flow] f={n:6d} seq={seq} fps={1/dt_smooth:5.1f} "
                       f"mean={mean_motion:5.2f}px moving={moving_ratio*100:4.1f}% "
                       f"trk={tracked:3d}")  # 每 10 帧打一行: 帧数/序号/帧率/位移/占比/点数
                if cal_scale_mpp > 0:
                    txt += f" v={speed_mps:5.2f}m/s(cal)"  # 有标定则追加物理速度
                print(txt, flush=True)

            # ---- 可视化: 独立 10-15fps 节流, 不拖累测速 ----
            if now - last_viz_t >= 1.0 / stream_fps:
                last_viz_t = now  # 更新上次出图时刻
                viz = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # 灰度转 BGR 才能在上面画彩字
                viz = annotate(viz, out_motion, moving_ratio,
                               px_to_m=px_to_m if px_to_m > 0 else None,
                               motion_thr=motion_thr, tracked=tracked,
                               speed_mps=speed_mps)  # 叠加文本得到可视化帧
                ok_enc, buf = cv2.imencode(".jpg", viz,
                                           [cv2.IMWRITE_JPEG_QUALITY, mjpg_quality])  # 编码 JPEG
                if ok_enc:
                    streamer.set_frame(buf.tobytes())  # 交给 Web 线程推流

            # prev: 用 CPU numpy 备份重建 UMat, 完全绕开 OpenCV UMat buffer 池复用坑
            prev_gray_np = gray_p.get().copy()   # 备份当前帧供下次 LK(先备份, 再重建)
            prev_gray_p = cv2.UMat(prev_gray_np)
            # 帧率节流: 抽帧到 --fps (实际受 GPU 能力限制)
            spent = time.time() - loop_t  # 本轮实际耗时
            if spent < target_dt:
                time.sleep(target_dt - spent)  # 补足剩余时间, 维持目标 fps
    except KeyboardInterrupt:
        print("[INFO] 停止", flush=True)  # Ctrl-C 正常退出
    finally:
        httpd.shutdown()  # 关闭 Web 服务
        rd.close()  # 释放共享内存映射
        elapsed = time.time() - t0  # 总运行时长
        if n > 0:
            print(f"summary: frames={n} avg_fps={n/elapsed:.1f}", flush=True)  # 收尾打印平均帧率
    return 0  # 0 = 正常退出


if __name__ == "__main__":
    sys.exit(main())  # 以 main 返回值作为进程退出码
