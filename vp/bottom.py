#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bottom.py — 下视任务入口（真实/虚拟相机 + YOLO 检测输出）

全面联动 main_config.DEFAULT_CONFIG：
  ENABLE_BOTTOM_CAM / ENABLE_BOTTOM_YOLO
  BOTTOM_CAMERA (device/index/width/height/fps/format/backend/mark_point)
  BOTTOM_YOLO   (backend/model_path/score_thres/nms_thres/strides/priority/
                 bpu_cores/class_names/target_class_names)
  CAMERA_QUEUE_SIZE / LOOP_SLEEP / SHOW / ENABLE_TIMING / TIMING_INTERVAL

多线程：1 采集 + N worker + 1 显示；每 worker 独立 YoloDetector 实例（榨 BPU 多核）

输出（受 ENABLE_TIMING 开关控制）：
  - 默认 False：只输出 YOLO 检测结果（最原始检测输出逻辑，零日志零计时开销）
  - True：额外打印 相机运行时间/帧率 + 采集/检测/显示各阶段平均耗时

数据流（"下视"一路, 与 front.py 相同, 仅相机/端口/模型/目标类别不同）:
  camera / 虚拟帧 --producer采集线程--> 有界队列 q --worker x N检测线程--> YOLO 检测结果
                                                    |
                                                    |---> 终端/result/log (检测结果)
                                                    |---> 画框帧 -> streamer.py (TCP :9001)
                                                          -> mjpeg_bridge.py (HTTP :5000 /cam2)
                                                          -> 上位机 pc_main2 显示
  遥测虚拟参数($TEL)由 telem_sender.py 独立进程负责 (UDP 8081 -> 上位机),
  与图像同由 run.sh 统一拉起/结束 (见 run.sh 注释)。

注意: main_config.py 里的 BOTTOM_HSV (LineDetector 巡线) 是本仓库保留的巡线能力,
      当前 bottom.py 未接线 (主用 BOTTOM_YOLO)。需要巡线时参考 function.py 内
      LineDetector 的用法自行接入, 或退回 vp4.x 版本。
"""
import argparse
import json
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np

from main_config import DEFAULT_CONFIG as CFG
from function import YoloDetector, format_detection, draw_detections
from streamer import JpegStreamer


TASK = 'bottom'
CAM = CFG['BOTTOM_CAMERA']
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, 'utils'))   # flow_share
# 下视写共享帧开关(光流测速读 bottom 下视帧): 仅由 ENABLE_FLOW_SHARE_BOTTOM 独立控制
FLOW_SHARE_ON = bool(CFG.get('ENABLE_FLOW_SHARE_BOTTOM', False))
YOLO_CFG = CFG['BOTTOM_YOLO']

# ---------- 联动 config 字段 ----------
W = int(CAM.get('width', 640))
H = int(CAM.get('height', 480))
FPS = max(1, min(int(CAM.get('fps', 60)), 60))   # 虚拟相机节拍取 fps（封顶 60）
MARK_POINT = CAM.get('mark_point')
Q_SIZE = int(CFG.get('CAMERA_QUEUE_SIZE', 8))
LOOP_SLEEP = float(CFG.get('LOOP_SLEEP', 0.0))
SHOW = bool(CFG.get('SHOW', True))
N_WORKERS = int(CFG.get('N_WORKERS', 3))   # worker 线程数：由 config 配置（--workers 可覆盖）
TIMING = bool(CFG.get('ENABLE_TIMING', False))
TIMING_INTERVAL = int(CFG.get('TIMING_INTERVAL', 30))
SIMPLE_TIMING = bool(CFG.get('SIMPLE_TIMING', False))  # 简化时间统计: 只打印/统计帧率, 不做各阶段计时
LOG_ENABLED = bool(CFG.get('ENABLE_LOG', False))
LOG_DIR = str(CFG.get('LOG_DIR', 'result/log'))

# 模块开关
CAM_ENABLED = bool(CFG.get('ENABLE_BOTTOM_CAM', True))
YOLO_ENABLED = bool(CFG.get('ENABLE_BOTTOM_YOLO', True))

# YOLO 预处理 (NV12 直通) —— 与 function.YoloDetector 同读一套配置
PRE_MODE = str(YOLO_CFG.get('preprocess_mode', 'auto')).lower()


# ============================================================
# 日志写入器：将每一帧的检测结果保存到 ./result/log/ 下
# 每帧一行 JSON：{"frame": 帧号, "ts": 时间戳, "dets": [...]}
# 多 worker 并发写同一文件，用锁串行化，保证每行完整不交错
# ============================================================
class FrameLog:
    """帧级检测结果日志写入器：把每帧检测结果以追加模式写到 ./result/log/<task>.log，多线程安全（内部锁串行化，保证每行完整不交错）。每帧一行的格式见 write()。"""
    def __init__(self, path, t_start_perf):
        """打开日志文件（追加模式，UTF-8）并初始化写入锁。t_start_perf 为进程启动时刻（perf_counter），用于在日志里显示相对运行秒数。"""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fp = open(path, 'a', encoding='utf-8')
        self.lock = threading.Lock()
        self.t_start_perf = t_start_perf     # 进程启动时刻（perf_counter），用于算运行时间

    def write(self, frame_id, dets, status='done'):
        # 写一行：[HH:MM:SS][运行秒数][frame:N] [status]: 检测结果
        # status: done = 正常处理（检测结果可能为 none）；dropped = 队列满丢弃
        #        no_yolo = YOLO 未启用时跳过检测
        """写一行检测记录。status 决定结果怎么写：done=正常（dets 可为空）、dropped=队列满丢帧、no_yolo=YOLO 未启用、no_data=相机读帧失败。"""
        ts_str = time.strftime('%H:%M:%S')
        elapsed = time.perf_counter() - self.t_start_perf
        if status == 'dropped':
            result = 'dropped(queue_full)'
        elif status == 'no_yolo':
            result = 'no_yolo'
        elif dets:
            det_strs = [
                f"{d['label']} conf={d['score']:.2f} bbox={tuple(d['bbox'])} center={tuple(d['center'])}"
                for d in dets
            ]
            result = '; '.join(det_strs)
        else:
            result = 'none'
        line = f'[{ts_str}][{elapsed:.3f}s][frame:{frame_id}] [{status}]: {result}\n'
        with self.lock:
            self.fp.write(line)
            self.fp.flush()

    def close(self):
        """关闭日志文件（加锁防与 write 并发，幂等，异常被吞掉不影响退出）。"""
        with self.lock:
            try:
                self.fp.close()
            except Exception:
                pass


# 时间统计器：统计 相机运行时间 / 帧率 / 各任务阶段耗时
# 所有计时点均带注释说明"这段代码在测什么任务"
# ============================================================
class StageStats:
    """线程安全的阶段耗时累计器。

    字段含义（均按 秒 累计）：
      capture_s  : 采集阶段总耗时（cap.read / 虚拟帧合成）——测"相机取一帧要多久"
      detect_s   : YOLO 检测阶段总耗时（preprocess+infer+postprocess）——测"BPU推理链路"
      display_s  : 显示阶段总耗时（画框 + imshow）——测"可视化开销"
      frames     : 已处理帧数——用于计算帧率与单帧平均耗时
    """

    def __init__(self, simple=False):
        """初始化全部阶段耗时累计器与窗口统计基准（初始为 0）。

        simple=True（简化时间统计）: 只计帧数供 FPS, 不累加各阶段耗时。
        """
        self.lock = threading.Lock()
        self.simple = bool(simple)
        self.capture_s = 0.0
        self.detect_s = 0.0
        self.display_s = 0.0
        self.frames = 0
        self.capture_frames = 0
        self.detect_frames = 0
        self.display_frames = 0
        self._base_capture = 0.0
        self._base_detect = 0.0
        self._base_display = 0.0
        self._base_capture_f = 0
        self._base_detect_f = 0
        self._base_display_f = 0

    def add(self, stage, dt):
        """按阶段名累加耗时与帧数。stage 取值：capture / detect / display。

        简化模式(simple=True)下 dt 不累计（各阶段耗时保持 0），只保留帧数计数供 FPS。
        """
        with self.lock:
            if stage == 'capture':
                if not self.simple:
                    self.capture_s += dt
                self.capture_frames += 1
            elif stage == 'detect':
                if not self.simple:
                    self.detect_s += dt
                self.detect_frames += 1
            elif stage == 'display':
                if not self.simple:
                    self.display_s += dt
                self.display_frames += 1
            self.frames += 1

    # ---- 窗口统计：以两次快照之间的增量计算真实阶段耗时 ----
    def mark_window(self):
        """记录窗口基准（当前累计值），下次 delta_window 与之比较。"""
        with self.lock:
            self._base_capture = self.capture_s
            self._base_detect = self.detect_s
            self._base_display = self.display_s
            self._base_capture_f = self.capture_frames
            self._base_detect_f = self.detect_frames
            self._base_display_f = self.display_frames

    def delta_window(self):
        """返回自 mark_window 以来的增量 (秒) 与帧数。"""
        with self.lock:
            d_capture = self.capture_s - self._base_capture
            d_detect = self.detect_s - self._base_detect
            d_display = self.display_s - self._base_display
            n_capture = self.capture_frames - self._base_capture_f
            n_detect = self.detect_frames - self._base_detect_f
            n_display = self.display_frames - self._base_display_f
        return (d_capture, d_detect, d_display,
                n_capture, n_detect, n_display)


def format_timing(elapsed, interval_frames, d_capture, d_detect, d_display, title):
    """格式化时间统计行（含当前帧率与各阶段平均单帧耗时）。

    计算口径（窗口增量，非累计值，避免随时间虚增）：
      平均帧率   = 本窗口帧数 / 本窗口墙钟时间   -> 相机整体吞吐
      采集/检测/显示 = 本窗口内该阶段耗时增量 / 本窗口帧数 -> 单帧真实耗时
    参数 d_capture/d_detect/d_display 由调用方传入窗口增量（秒）。
    """
    if interval_frames <= 0 or elapsed <= 0:
        return ''
    fps = interval_frames / elapsed
    ms = lambda s: s / interval_frames * 1000.0
    return (f'[{TASK}] {title}: 帧数={interval_frames} 运行={elapsed:.2f}s '
            f'平均帧率={fps:.1f}fps | '
            f'采集={ms(d_capture):.2f}ms/帧 '
            f'检测={ms(d_detect):.2f}ms/帧 '
            f'显示={ms(d_display):.2f}ms/帧')


def make_frame(frame_id):
    """合成测试帧：动态渐变背景 + 移动黄色圆（对应 yellow-ball 目标）。"""
    t = frame_id
    img = np.zeros((H, W, 3), dtype=np.uint8)
    col = np.arange(W, dtype=np.float32) / W
    img[:, :, 0] = (90 + 60 * np.sin(t * 0.03)).astype(np.uint8)
    img[:, :, 1] = (col * 80 + 50).astype(np.uint8)
    img[:, :, 2] = (col * 40 + 40).astype(np.uint8)
    x = int(W - 40 - ((t * 3) % (W - 80)))
    y = int(H // 3 + 50 * np.sin(t * 0.04 + 1.0))
    cv2.circle(img, (x, y), 36, (0, 255, 255), -1)
    cv2.putText(img, f'{TASK} #{t}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


def open_camera(device, index):
    """打开真实相机（V4L2/MJPG）。优先 JPU 硬件解码，失败自动回退 cv2 软解。

    hardware_decode=True 时走 vp4.5 已验证的 JPU 硬解封装 (hw_camera.HwMjpgCamera,
    V4L2 抓原始 MJPG + JPU 硬解到 NV12 -> BGR)，接口与 cv2.VideoCapture 对齐：
      .read() -> (ok, bgr_frame) / .isOpened() / .get(prop) / .release()
    硬解不可用或异常时打印警告并回退 cv2 软解，保证相机链路不中断。
    """
    if CAM.get('hardware_decode', False):
        dev = device if device is not None else '/dev/video%d' % (index if index is not None else 0)
        try:
            from hw_camera import HwMjpgCamera
            cam = HwMjpgCamera(dev, W, H, int(CAM.get('fps', 200)))
            if cam.isOpened():
                print(f'[*] [{TASK}] 相机 {dev}: 启用 JPU 硬件解码', flush=True)
                return cam
            print(f'[警告] [{TASK}] 相机 {dev}: JPU 硬件解码不可用, 回退 cv2 软解', flush=True)
        except Exception as exc:
            print(f'[警告] [{TASK}] 相机硬件解码初始化异常 {exc!r}, 回退 cv2 软解', flush=True)
    src = device if device is not None else index
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        return None
    fmt = CAM.get('format')
    if fmt:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    fps_req = CAM.get('fps')
    if fps_req:
        cap.set(cv2.CAP_PROP_FPS, int(fps_req))
    try:
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)   # 读帧超时 1s：快速判定相机无数据
    except cv2.error:
        pass
    return cap


def producer(cap, virtual, q, max_frames, stop, n_workers, stats, enable_timing, log_writer,
             prefer_nv12=False, want_bgr=True, flow_share_w=None):
    """采集线程：真实相机读帧 或 虚拟相机合成帧，按 CAMERA_QUEUE_SIZE 有界入队。

    采集阶段计时：测量 cap.read()/make_frame() 从"申请到拿到一帧"的耗时，
    反映相机链路（USB 解码 / 合成）的吞吐瓶颈。
    """
    fid = 0
    t_win = time.perf_counter()   # 窗口起点：用于计算本窗口运行时间/帧率
    win_frames = 0
    fail_cnt = 0                  # 连续读帧失败计数（select 超时/掉线判定）
    READ_FAIL_LIMIT = 5           # 连续 5 次读不到帧 -> 判定相机掉线，停止本进程
    try:
        while not stop.is_set():
            if max_frames and fid >= max_frames:
                break
            t0 = time.perf_counter()          # 计时起点：准备取/造一帧
            nv12 = None
            if virtual:
                frame = make_frame(fid)
            elif prefer_nv12:
                # JPU 硬解: 直接保留原始 NV12 直通模型预处理 (省 NV12->BGR + BGR->YUV 两次转换)
                if want_bgr:
                    ok, frame, nv12 = cap.read_both()
                else:
                    ok, nv12 = cap.grab_raw_nv12()
                    frame = None
                if not ok or nv12 is None:
                    fail_cnt += 1
                    if log_writer is not None:
                        log_writer.write(fid, [], status='no_data')  # 如实记录读失败帧
                    if fail_cnt >= READ_FAIL_LIMIT:
                        print(f'[{TASK}] 相机掉线/无数据(select超时)，本进程退出', flush=True)
                        stop.set()            # 通知所有 worker 停止，进程退出
                        break
                    time.sleep(0.1)
                    fid += 1                  # 失败帧也占一个 fid，保证日志帧号连续
                    continue
                fail_cnt = 0
            else:
                ok, frame = cap.read()
                if not ok or frame is None:
                    fail_cnt += 1
                    if log_writer is not None:
                        log_writer.write(fid, [], status='no_data')  # 如实记录读失败帧
                    if fail_cnt >= READ_FAIL_LIMIT:
                        print(f'[{TASK}] 相机掉线/无数据(select超时)，本进程退出', flush=True)
                        stop.set()            # 通知所有 worker 停止，进程退出
                        break
                    time.sleep(0.1)
                    fid += 1                  # 失败帧也占一个 fid，保证日志帧号连续
                    continue
                fail_cnt = 0
            t1 = time.perf_counter()          # 计时终点：帧已就绪
            if flow_share_w is not None:      # 帧共享桥(光流测速): 写最新帧, ~0.1ms
                try:
                    flow_share_w.write(nv12 if nv12 is not None else frame)
                except Exception:
                    pass
            if enable_timing:
                stats.add('capture', t1 - t0)      # 采集阶段耗时入账
            try:
                q.put((fid, frame, nv12), timeout=1.0)
            except queue.Full:
                # 队列满：worker 处理不过来，本帧丢弃。如实写入日志，便于回看真实丢帧情况。
                if log_writer is not None:
                    log_writer.write(fid, [], status='dropped')
            fid += 1
            win_frames += 1
            # 周期性滚动统计：每 TIMING_INTERVAL 帧打印一次本窗口帧率/阶段均值
            # 阶段耗时取窗口增量（delta_window），避免用累计值导致数值随时间虚增
            if enable_timing and win_frames >= TIMING_INTERVAL:
                now = time.perf_counter()
                if SIMPLE_TIMING:
                    # 简化时间统计: 只打印帧率, 不做各阶段耗时统计
                    fps = win_frames / max(now - t_win, 1e-9)
                    print(f'[{TASK}] 简化时间统计@{fid}帧: 帧数={win_frames} '
                          f'运行={now - t_win:.2f}s 平均帧率={fps:.1f}fps', flush=True)
                else:
                    d_c, d_d, d_dp, n_c, n_d, n_dp = stats.delta_window()
                    n = n_c if n_c else win_frames
                    print(format_timing(now - t_win, n, d_c, d_d, d_dp,
                                        f'时间统计@{fid}帧'), flush=True)
                    stats.mark_window()
                t_win = now
                win_frames = 0
            if virtual:
                time.sleep(1.0 / FPS)
            elif LOOP_SLEEP > 0:
                time.sleep(LOOP_SLEEP)
    finally:
        for _ in range(n_workers):
            q.put(None)             # 每 worker 一个哨兵


def worker(wid, q, disp_q, show, stop, stats, enable_timing, log_writer,
           streamer=None):
    """检测 worker：每线程独立 detector 避免 BPU 冲突。

    检测阶段计时：测量 detector.detect() 完整链路耗时（预处理+BPU推理+后处理），
    反映该 worker 处理一帧的推理开销。
    显示阶段计时：测量画框 + 入显示队列耗时，反映可视化对主链路的挤占。
    """
    detector = YoloDetector(YOLO_CFG) if YOLO_ENABLED else None
    idx = 0
    while not stop.is_set():
        try:
            item = q.get(timeout=0.5)
        except queue.Empty:
            continue
        if item is None:
            break
        fid, frame, nv12 = item
        # 决定状态：YOLO 关闭时本帧未走推理，标 no_yolo；正常处理标 done
        status = 'no_yolo' if detector is None else 'done'
        if enable_timing:
            t0 = time.perf_counter()          # 计时起点：开始 YOLO 检测
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []
            t1 = time.perf_counter()          # 计时终点：检测完成
            stats.add('detect', t1 - t0)      # 检测阶段耗时入账
        else:
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []
        if log_writer is not None:
            log_writer.write(fid, dets, status=status)   # worker 处理完如实写入（含 no_yolo）
        # 画框帧去向: 本机弹窗(show) 与/或 上位机推流(streamer 有客户端时)
        need_vis = bool(show or (streamer is not None and streamer.connected))
        if need_vis:
            if frame is None and nv12 is not None:
                frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)  # NV12 直通帧需显示/推流时再转 BGR
            t2 = time.perf_counter()          # 计时起点：开始画框+入显示/推流队列
            if frame is not None:
                vis = draw_detections(frame, dets) if dets else frame
                if show:
                    try:
                        disp_q.put_nowait(vis)
                    except queue.Full:
                        pass
                if streamer is not None and streamer.connected:
                    streamer.put_frame(vis)
            t3 = time.perf_counter()          # 计时终点：显示/推流准备完成
            if enable_timing:
                stats.add('display', t3 - t2)     # 显示阶段耗时入账
        for d in dets:
            print(f'[{TASK}] ' + format_detection(d, MARK_POINT, idx), flush=True)
            idx += 1
    q.task_done()


def display_loop(disp_q, stop):
    """显示线程：imshow + waitKey。无 DISPLAY 时静默退出。"""
    win = TASK
    try:
        while not stop.is_set():
            try:
                vis = disp_q.get(timeout=0.1)
            except queue.Empty:
                continue
            cv2.imshow(win, vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop.set()
                break
    except cv2.error:
        pass
    finally:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass


def main():
    """bottom 进程入口：解析 CLI -> 配置联动 -> 启相机/队列/worker/推流 -> 循环直至退出.

    与 front.py 对称，仅相机端口/模型/目标类别不同；CLI 参数见下方 argparse。
    """
    ap = argparse.ArgumentParser(description='bottom 任务（真实/虚拟相机 + YOLO）')
    ap.add_argument('--device', type=str, default=None, help='相机设备路径，覆盖 config')
    ap.add_argument('--index', type=int, default=None, help='相机 index，覆盖 config')
    ap.add_argument('--virtual', action='store_true', help='使用虚拟相机（合成帧）')
    ap.add_argument('--no-show', action='store_true', help='关闭 imshow 显示')
    ap.add_argument('--no-stream', action='store_true', help='关闭 STREAM 推流（纯检测模式）')
    ap.add_argument('--workers', type=int, default=N_WORKERS,
                    help=f'worker 线程数（默认 {N_WORKERS}，每线程独立 BPU 实例）')
    ap.add_argument('--frames', type=int, default=0, help='运行帧数，0=无限（默认）')
    ap.add_argument('--timing', action='store_true', help='强制开启时间统计/帧率计时')
    ap.add_argument('--no-timing', action='store_true', help='强制关闭时间统计')
    ap.add_argument('--log', action='store_true', help='强制开启帧日志保存')
    ap.add_argument('--no-log', action='store_true', help='强制关闭帧日志保存')
    args = ap.parse_args()

    enable_timing = TIMING
    if args.timing:
        enable_timing = True
    if args.no_timing:
        enable_timing = False
    enable_log = LOG_ENABLED
    if args.log:
        enable_log = True
    if args.no_log:
        enable_log = False

    if not CAM_ENABLED and not args.virtual:
        print(f'[{TASK}] ENABLE_BOTTOM_CAM=False，跳过真实相机；可用 --virtual 联调')
        return 0

    show = SHOW and not args.no_show

    # ---- STREAM 推流配置 (main_config.STREAM; --no-stream 可临时关闭) ----
    STREAM_CFG = CFG.get('STREAM', {}) or {}
    stream_on = bool(STREAM_CFG.get('enable', False)) and not args.no_stream
    stream_bind = str(STREAM_CFG.get('bind_host', '0.0.0.0'))
    stream_port = int(STREAM_CFG.get(f'{TASK}_port', 9000))
    stream_fps = int(STREAM_CFG.get('fps', 30))
    stream_quality = int(STREAM_CFG.get('jpeg_quality', 80))
    if stream_on:
        print(f'[*] [{TASK}] STREAM: 监听 {stream_bind}:{stream_port} '
              f'({stream_fps}fps, JPEG q={stream_quality})', flush=True)

    stats = StageStats(simple=SIMPLE_TIMING)
    stats.mark_window()      # 记录统计基准：此后每个窗口按增量统计
    t_start = time.perf_counter()   # 总计时起点：测量该相机进程的总运行时间

    # 日志开关（config 或 CLI 覆盖）：开启则每帧结果写入 ./result/log/<task>.log
    log_writer = None
    if enable_log:
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                LOG_DIR, f'{TASK}.log')
        log_writer = FrameLog(log_path, t_start)   # t_start 为 main 启动时刻

    # 帧共享桥: 本进程把 NV12(JPU硬解) 最新帧写给光流测速进程
    flow_share_w = None
    if FLOW_SHARE_ON and not args.virtual:
        try:
            from flow_share import FlowShareWriter
            flow_share_w = FlowShareWriter('bottom', W, H, fmt=0)
            print(f'[*] [{TASK}] 帧共享已开启: /dev/shm/momo_flow_bottom.bin (NV12)', flush=True)
        except Exception as exc:
            print(f'[警告] [{TASK}] 帧共享初始化失败: {exc!r}, 继续原有流程', flush=True)

    cap = None
    if not args.virtual:
        device = args.device if args.device is not None else CAM.get('device')
        index = args.index if args.index is not None else CAM.get('index')
        cap = open_camera(device, index)
        if cap is None:
            print(f'[{TASK}] 真实相机打开失败: device={device} index={index}，进程退出', flush=True)
            return 1

    # YOLO 预处理直通: JPU 硬解相机能提供原始 NV12 且 config 允许时, 队列里走 NV12
    src_is_hw = cap is not None and hasattr(cap, 'grab_raw_nv12') and hasattr(cap, 'read_both')
    prefer_nv12 = bool(src_is_hw and PRE_MODE in ('auto', 'nv12'))
    want_bgr = bool(show or stream_on)        # 弹窗/推流需要 BGR 画框帧时, producer 同时备一份

    q = queue.Queue(maxsize=Q_SIZE)
    disp_q = queue.Queue(maxsize=2)
    stop = threading.Event()

    # STREAM 推流线程: 画框帧 -> JPEG -> TCP (独立于采集/worker/显示, 不挤占检测)
    streamer = None
    if stream_on:
        streamer = JpegStreamer(TASK, stream_port, bind_host=stream_bind,
                                fps=stream_fps, quality=stream_quality)

    threads = [threading.Thread(target=producer,
                                args=(cap, args.virtual, q, args.frames, stop,
                                      args.workers, stats, enable_timing, log_writer,
                                      prefer_nv12, want_bgr, flow_share_w),
                                daemon=True)]
    for i in range(args.workers):
        threads.append(threading.Thread(target=worker,
                                        args=(i, q, disp_q, show, stop,
                                              stats, enable_timing, log_writer, streamer),
                                        daemon=True))
    if show:
        threads.append(threading.Thread(target=display_loop,
                                        args=(disp_q, stop), daemon=True))

    for th in threads:
        th.start()
    if streamer is not None:
        streamer.start()          # 推流线程独立启动, 结束阶段显式 stop
    try:
        for th in threads:
            th.join()
    except KeyboardInterrupt:
        stop.set()
    finally:
        if streamer is not None:
            streamer.stop()       # 有限帧跑完 / Ctrl-C 均触发推流停止
        if cap is not None:
            cap.release()
        if log_writer is not None:
            log_writer.close()
        if enable_timing:
            elapsed = time.perf_counter() - t_start
            total = max(stats.capture_frames, 1)
            print(f'[{TASK}] 结束统计: 总运行={elapsed:.2f}s '
                  f'总帧数={stats.capture_frames} 总检测帧={stats.detect_frames} '
                  f'平均帧率={total / elapsed:.1f}fps', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())