#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py — vp6.0 双相机空闲超时单向切换 + YOLO 检测主入口（真·多进程版）

需求（来自本文件最初的注释，已实现）：
  1) 开机（开始运行）时先调用 cam1（默认 /dev/video0）进行 YOLO 识别；
  2) 空闲计时器从开机（会话启动）起置零、开始累计；
  3) 检测到目标 -> 计时器归零，重新开始累计（"只要还在看得到目标就一直
     留在 cam1"）；
  4) 连续无目标时长 >= CAM1_IDLE_TIMEOUT（main_config 配置）-> 关闭 cam1
     与计时器，启用 cam2 进行 YOLO 识别；
  5) 单向切换：切到 cam2 后固定使用 cam2，不再切回 cam1
     （CAM2_IDLE_TIMEOUT 仅保留作字段，不触发切回）；
  6) 吃满 6 核：检测 worker 是真·多进程（N_WORKERS 个独立进程，每进程一个
     Python 解释器 + 独立 YoloDetector/BPU 实例），规避单进程 GIL 锁死，
     预处理/后处理/多实例推理并行；配合 run.sh 的 taskset -c 0-5 绑核运行；
  7) 相机健壮性：打开失败/运行中掉线自动重试（CAM_OPEN_RETRIES ×
     CAM_OPEN_RETRY_DELAY），不再静默退出；显示窗口常驻（无数据时显示
     提示帧，切到 cam2 也能看到窗口与状态）；
  8) 相对 vp5.0：不做推流（去掉 STREAM）；其余功能全部由
     main_config.DEFAULT_CONFIG 开关控制。

架构（多进程流水线）：
  主进程（本进程）:
    - 状态机：cam1 会话 -> (空闲超时) -> cam2 会话（单向）
    - producer 线程 : 相机读帧(可自动重连) -> JPEG 编码 -> in_q 队列
    - display 线程   : 常驻 imshow 窗口（叠加最近检测框；无数据显示状态帧）
    - listener 线程  : 收 worker 检测结果 -> 打印/写日志/空闲计时归零/统计
    - idle watchdog  : 空闲计时 >= 超时 -> 置停止与切换原因（仅 cam1）
  N_WORKERS 个 worker 进程（multiprocessing.Process，daemon）:
    - 各持一个独立 YoloDetector（BPU 多实例并行）
    - 从 in_q 取 JPEG 帧 -> 解码 -> 可选预处理 -> YOLO -> 结果回 out_q

帧传输用 JPEG（FRAME_JPEG_QUALITY）而不是 pickle numpy：多进程队列
传大数组带宽/序列化开销高，JPEG 中间格式在 640x480 下代价可忽略。
worker 进程先于相机打开而 fork，保证子进程不继承 V4L2 fd；相机句柄
（含重连后的新句柄）统一由 cap_holder 持有、会话结束时释放。

会话退出 reason（决定主流程是否切 cam2）：
  idle_timeout : 空闲超时（需求第 4 条 -> 切 cam2）
  no_data      : 相机反复打开失败/掉线无法恢复（failover -> 尝试 cam2）
  init_fail    : YOLO 模型初始化失败（cam2 同模型也会失败 -> 不再切）
  frames       : --frames 限帧跑完（验收用，不再切）
  manual       : Ctrl-C / 手动停止（不再切）
  disabled     : ENABLE_CAM1=False 直接跳过

数据流/日志：
  - 终端: 命中行 [cam1] #i label 中心=.. 相对标记点=.. conf=..
          （命中即空闲计时归零）；ENABLE_TIMING=True 时每 TIMING_INTERVAL
          帧打印滚动统计（窗口帧率 + 检测均值 + 当前空闲秒数）
  - result/log/cam1.log|cam2.log（ENABLE_LOG=True）：每帧一行检测记录，
    含 [idle:..s] 便于回看"计时归零/超时"过程

用法（必须 cd 到 vp6.0 目录，依赖 utils/py_utils 与 function.py）:
  bash run.sh                        # taskset 绑 0-5（推荐，6 核全用）
  python3 main.py                    # 直接前台运行（不绑核）
  python3 main.py --virtual          # 无实体相机联调（合成帧，默认带移动目标）
  python3 main.py --virtual --no-target --timeout 3
                                     # 验证单向切换：无目标 3s -> cam1 超时 ->
                                     # 切 cam2 后不再切回（cam2 也跑虚拟帧）
  python3 main.py --frames 100       # 每会话限帧（验收）
"""
import argparse
import os
import queue
import threading
import time
import multiprocessing as mp

import cv2
import numpy as np

from main_config import DEFAULT_CONFIG as CFG
from function import YoloDetector, format_detection, draw_detections, preprocess

# ============================================================
# 配置常量（全部来自 main_config.DEFAULT_CONFIG）
# ============================================================
_T0 = time.perf_counter()          # 进程启动时刻（滚动统计用）
LOOP_SLEEP = float(CFG.get('LOOP_SLEEP', 0.001))
N_WORKERS = int(CFG.get('N_WORKERS', 6))          # 检测 worker 进程数（对齐 6 核）
Q_SIZE = int(CFG.get('CAMERA_QUEUE_SIZE', 12))
JPEG_Q = int(CFG.get('FRAME_JPEG_QUALITY', 85))
TIMING = bool(CFG.get('ENABLE_TIMING', True))
TIMING_INTERVAL = int(CFG.get('TIMING_INTERVAL', 100))
LOG_ENABLED = bool(CFG.get('ENABLE_LOG', True))
LOG_DIR = str(CFG.get('LOG_DIR', 'result/log'))
SHOW = bool(CFG.get('SHOW', False))
IDLE_CHECK = float(CFG.get('IDLE_CHECK_INTERVAL', 0.2))
OPEN_RETRIES = int(CFG.get('CAM_OPEN_RETRIES', 5))
OPEN_RETRY_DELAY = float(CFG.get('CAM_OPEN_RETRY_DELAY', 2.0))
READ_FAIL_LIMIT = int(CFG.get('READ_FAIL_LIMIT', 5))

_ctx = mp.get_context('fork')      # Linux 默认 fork：worker 继承已加载模块


# ============================================================
# 空闲计时器（线程安全）：last_hit 时间戳 = 计时器当前值
# ============================================================
class IdleTimer:
    """空闲计时器：记录"最近一次检测到目标"的时刻。"""
    def __init__(self):
        self._lock = threading.Lock()
        self._last_hit = time.perf_counter()   # 会话启动即开始累计

    def reset(self):
        with self._lock:
            self._last_hit = time.perf_counter()

    def idle(self):
        with self._lock:
            return time.perf_counter() - self._last_hit


# ============================================================
# 帧级日志写入器（主进程 listener 单线程调用；锁兜底）
# ============================================================
class FrameLog:
    """把每帧检测结果以追加模式写到 ./result/log/<cam>.log。"""
    def __init__(self, path, t_start_perf):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fp = open(path, 'a', encoding='utf-8')
        self.lock = threading.Lock()
        self.t_start_perf = t_start_perf

    def write(self, task, frame_id, dets, idle_s, status='done'):
        ts_str = time.strftime('%H:%M:%S')
        elapsed = time.perf_counter() - self.t_start_perf
        if status == 'dropped':
            result = 'dropped(queue_full)'
        elif status == 'no_yolo':
            result = 'no_yolo'
        elif status == 'no_data':
            result = 'no_data(read_fail)'
        elif dets:
            det_strs = [f"{d['label']} conf={d['score']:.2f} "
                        f"bbox={tuple(d['bbox'])} center={tuple(d['center'])}"
                        for d in dets]
            result = '; '.join(det_strs)
        else:
            result = 'none'
        line = (f'[{ts_str}][{elapsed:.3f}s][{task}][frame:{frame_id}]'
                f'[idle:{idle_s:.2f}s][{status}]: {result}\n')
        with self.lock:
            self.fp.write(line)
            self.fp.flush()

    def close(self):
        with self.lock:
            try:
                self.fp.close()
            except Exception:
                pass


# ============================================================
# 相机打开（含失败自动重试）
# ============================================================
def open_camera(cam_cfg):
    """打开真实相机一次。hw 硬解不可用时自动回退 cv2 软解。

    Returns:
        cap 或 None（打开失败）
    """
    dev = cam_cfg.get('device')
    index = cam_cfg.get('index')
    W = int(cam_cfg.get('width', 640))
    H = int(cam_cfg.get('height', 480))
    fps = int(cam_cfg.get('fps', 250))
    fmt = cam_cfg.get('format')

    if cam_cfg.get('hardware_decode', False):
        try:
            from hw_camera import HwMjpgCamera
            cam = HwMjpgCamera(dev, W, H, fps)
            if cam.isOpened():
                return cam
            print(f'[警告] 硬件解码打开失败，回退 cv2 软解', flush=True)
        except Exception as exc:
            print(f'[警告] 硬件解码不可用({exc!r})，回退 cv2 软解', flush=True)

    src = dev if dev is not None else index
    if src is None:
        return None
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        return None
    if fmt:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)   # 读帧超时 1s
    except cv2.error:
        pass
    return cap


def open_camera_with_retry(task, cam_cfg, status, stop):
    """按 CAM_OPEN_RETRIES/DELAY 重试打开相机；期间更新显示状态文字。

    Args:
        status: dict {'text': str}，display 线程读取显示
        stop:   threading.Event，会话停止信号（重试期间响应退出）
    Returns:
        cap 或 None（多次重试仍失败）
    """
    attempts = max(1, OPEN_RETRIES + 1)   # 首次 + 重试次数
    for i in range(attempts):
        if stop.is_set():
            return None
        status['text'] = f'[相机] 打开 {cam_cfg.get("device")} 尝试 {i + 1}/{attempts}'
        cap = open_camera(cam_cfg)
        if cap is not None:
            status['text'] = f'[相机] {cam_cfg.get("device")} 就绪'
            return cap
        if i < attempts - 1:
            print(f'[{task}] 相机打开失败，{OPEN_RETRY_DELAY:.0f}s 后重试 '
                  f'({i + 1}/{attempts - 1})', flush=True)
            for _ in range(int(OPEN_RETRY_DELAY * 10)):
                if stop.is_set():
                    return None
                time.sleep(0.1)
    return None


# ============================================================
# worker 进程：JPEG 帧 -> 解码 -> YOLO -> 结果回传
# ============================================================
def worker_main(wid, yolo_cfg, pre_cfg, enable_pre, in_q, out_q, stop):
    """检测 worker 进程（每进程独立 YoloDetector/BPU 实例）。

    从 in_q 取 (fid, jpeg_bytes)；结果放回 out_q:
      {'type':'det', 'fid':.., 'dets':[...], 'dt':推理耗时秒}
      初始化失败: {'type':'init_fail', 'err':..}
    """
    try:
        detector = YoloDetector(yolo_cfg)
    except Exception as exc:
        out_q.put({'type': 'init_fail', 'wid': wid, 'err': repr(exc)})
        return
    while not stop.is_set():
        try:
            item = in_q.get(timeout=0.3)
        except queue.Empty:
            continue
        if item is None:
            break
        fid, jpg = item
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        if enable_pre and pre_cfg is not None:
            frame = preprocess(frame, pre_cfg)
        t0 = time.perf_counter()
        dets = detector.detect(frame)
        dt = time.perf_counter() - t0
        # dets 为统一格式 list[dict]，可 pickle；空列表也回传（供统计帧率）
        out_q.put({'type': 'det', 'wid': wid, 'fid': fid,
                   'dets': dets, 'dt': dt})


# ============================================================
# 主进程线程：producer / display / listener / idle watchdog
# ============================================================
def producer(task, cam_cfg, cap_holder, in_q, n_workers, virtual,
             with_target, max_frames, stop, reason, status, latest, stats,
             enable_timing, log_writer, idle_timer, dropped_cnt):
    """采集线程：读帧 -> (掉线自动重连) -> JPEG 编码入队。

    cap_holder: {'cap': ...} 相机句柄容器；重连成功后把新句柄写回，
                会话结束时由 run_session 统一释放，避免旧句柄泄漏。
    latest:     {'frame': BGR 或 None}，供 display 线程显示。
    """
    cap = cap_holder.get('cap')
    fid = 0
    fail_cnt = 0
    virt_fps = max(1, min(int(cam_cfg.get('fps', 60)), 60))
    while not stop.is_set():
        if max_frames and fid >= max_frames:
            reason[0] = 'frames'
            stop.set()
            break
        t0 = time.perf_counter()
        ok, frame = True, None
        if virtual:
            frame = make_frame(task, fid, with_target)
        elif cap is not None:
            try:
                ok, frame = cap.read()
            except Exception:
                ok, frame = False, None
        else:
            ok = False
        if not ok or frame is None:
            fail_cnt += 1
            if fail_cnt >= READ_FAIL_LIMIT:
                # 运行中掉线 -> 释放旧句柄并重连（重试间隔给 USB 枚举窗口）
                print(f'[{task}] 相机掉线({fail_cnt}次读失败)，尝试重连...',
                      flush=True)
                status['text'] = '[相机] 掉线，尝试重连...'
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap_holder['cap'] = None
                cap = open_camera_with_retry(task, cam_cfg, status, stop)
                if cap is None:
                    reason[0] = 'no_data'
                    status['text'] = '[相机] 无法恢复，会话结束'
                    stop.set()
                    break
                cap_holder['cap'] = cap    # 新句柄交还容器，收尾时统一释放
                fail_cnt = 0
                continue
            if log_writer is not None:
                log_writer.write(task, fid, [], idle_timer.idle(),
                                 status='no_data')
            time.sleep(0.1)
            fid += 1
            continue
        fail_cnt = 0
        t1 = time.perf_counter()
        if enable_timing:
            stats.add('capture', t1 - t0)
        # 最新帧给显示线程（cv2.read 每次返回新数组，旧引用安全）
        latest['frame'] = frame
        # JPEG 编码后入进程队列（满则丢新帧并计数）
        okj, buf = cv2.imencode('.jpg', frame,
                                [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        if okj:
            try:
                in_q.put_nowait((fid, buf.tobytes()))
            except queue.Full:
                dropped_cnt[0] += 1
        fid += 1
        if virtual:
            time.sleep(1.0 / virt_fps)
        elif LOOP_SLEEP > 0:
            time.sleep(LOOP_SLEEP)
    # 哨兵：通知每个 worker 进程退出（队列满则等 worker 消费后重试）
    for _ in range(n_workers):
        for _r in range(40):          # 有限重试 ~2s；仍满则由 proc_stop 兜底
            try:
                in_q.put_nowait(None)
                break
            except queue.Full:
                time.sleep(0.05)
            except Exception:
                break


def listener(task, out_q, stop, reason, idle_timer, stats, enable_timing,
             log_writer, mark_point, latest):
    """结果监听线程：收 worker 检测结果 -> 打印/日志/空闲归零/滚动统计。"""
    idx = 0
    win_frames = 0
    t_win = time.perf_counter()
    while not stop.is_set():
        try:
            msg = out_q.get(timeout=0.2)
        except queue.Empty:
            continue
        if msg is None:
            continue
        mtype = msg.get('type')
        if mtype == 'init_fail':
            print(f'[{task}] worker{msg.get("wid")} 模型初始化失败: '
                  f'{msg.get("err")}, 本会话结束', flush=True)
            reason[0] = 'init_fail'
            stop.set()
            return
        if mtype != 'det':
            continue
        fid = msg.get('fid', 0)
        dets = msg.get('dets') or []
        dt = msg.get('dt', 0.0)
        if enable_timing:
            stats.add('detect', dt)
        if dets:
            idle_timer.reset()          # 检测到目标 -> 空闲计时器归零
            latest['dets'] = dets       # 供 display 叠加画框（最新一次结果）
        if log_writer is not None:
            log_writer.write(task, fid, dets, idle_timer.idle())
        for d in dets:
            print(f'[{task}] ' + format_detection(d, mark_point, idx),
                  flush=True)
            idx += 1
        # 滚动统计（每 TIMING_INTERVAL 个检测结果打印一次）
        win_frames += 1
        if enable_timing and win_frames >= TIMING_INTERVAL:
            now = time.perf_counter()
            elapsed = now - t_win
            if elapsed > 0:
                fps = win_frames / elapsed
                avg_ms = (stats.detect_s /
                          max(stats.detect_frames, 1) * 1000)
                print(f'[{task}] 时间统计: 运行={now - _T0:.2f}s '
                      f'检测帧率={fps:.1f}fps 检测={avg_ms:.2f}ms/帧 '
                      f'空闲={idle_timer.idle():.2f}s', flush=True)
            t_win = now
            win_frames = 0


def display_loop(task, stop, status, latest):
    """显示线程（常驻）：显示最新帧并叠加最近检测框；无数据显示状态帧。

    窗口标题 = task（cam1/cam2），会话结束才销毁；相机无数据期间窗口
    保持并显示 status['text']，避免"切到 cam2 没窗口/没画面"。
    """
    try:
        while not stop.is_set():
            frame = latest.get('frame')
            if frame is None:
                # 无帧：画提示帧（黑底 + 状态文字）
                vis = np.zeros((240, 320, 3), dtype=np.uint8)
                text = status.get('text') or f'[{task}] waiting camera...'
                cv2.putText(vis, f'[{task}]', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(vis, text[:30], (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                cv2.imshow(task, vis)
            else:
                dets = latest.get('dets') or []
                vis = draw_detections(frame, dets) if dets else frame
                cv2.imshow(task, vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop.set()
                break
            time.sleep(0.03)            # ~30fps 显示节流
    except cv2.error:
        pass
    finally:
        try:
            cv2.destroyWindow(task)
        except cv2.error:
            pass


def idle_watchdog(task, idle_timer, timeout, stop, reason, switch_log):
    """空闲计时监督线程：连续无目标 >= timeout 秒 -> 置停止与切换原因。

    仅 cam1 会话启用（单向切换的触发源）；cam2 会话不启动该线程。
    """
    while not stop.is_set():
        if idle_timer.idle() >= timeout:
            reason[0] = 'idle_timeout'
            if switch_log:
                print(f'[{task}] 空闲超时: 连续 {timeout:.1f}s 无目标, '
                      f'关闭本相机与计时器', flush=True)
            stop.set()
            return
        time.sleep(IDLE_CHECK)


# ============================================================
# 统计与虚拟帧
# ============================================================
class StageStats:
    """累计 capture/detect 两阶段耗时与帧数（滚动统计用累计均值）。"""
    def __init__(self):
        self.lock = threading.Lock()
        self.capture_s = 0.0
        self.detect_s = 0.0
        self.capture_frames = 0
        self.detect_frames = 0

    def add(self, stage, dt):
        with self.lock:
            if stage == 'capture':
                self.capture_s += dt
                self.capture_frames += 1
            elif stage == 'detect':
                self.detect_s += dt
                self.detect_frames += 1


def make_frame(task, frame_id, with_target=True):
    """合成测试帧：动态渐变背景 +（可选）移动红色圆，模拟 red-ball 目标。"""
    t = frame_id
    W, H = 640, 480
    img = np.zeros((H, W, 3), dtype=np.uint8)
    col = np.arange(W, dtype=np.float32) / W
    img[:, :, 0] = (120 + 60 * np.sin(t * 0.02)).astype(np.uint8)
    img[:, :, 1] = (col * 60 + 40).astype(np.uint8)
    img[:, :, 2] = (col * 30 + 30).astype(np.uint8)
    if with_target:
        x = int((t * 4) % (W - 80)) + 40
        y = int(H // 2 + 60 * np.sin(t * 0.05))
        cv2.circle(img, (x, y), 36, (0, 0, 255), -1)
    cv2.putText(img, f'{task} #{t}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


# ============================================================
# 会话运行器：主进程 + N worker 进程跑一路相机，返回 reason
# ============================================================
def run_session(task, cam_cfg, yolo_cfg, pre_cfg, enable_pre, enable_yolo,
                virtual, with_target, max_frames, idle_timeout=None,
                workers=None):
    """运行一路相机会话（真·多进程：主进程采集/显示 + N worker 进程推理）。

    Args:
        task:          'cam1' / 'cam2'（日志与终端前缀）
        cam_cfg:       CAMx_CAMERA 配置段
        yolo_cfg:      CAMx_YOLO 配置段
        enable_yolo:   ENABLE_CAMx_YOLO 开关
        idle_timeout:  空闲超时秒数；仅 cam1 传入（None = 不监督，cam2 用）
    Returns:
        reason: 'idle_timeout' / 'no_data' / 'init_fail' / 'frames' / 'manual'
    """
    workers = workers if workers is not None else N_WORKERS
    mark_point = cam_cfg.get('mark_point')
    show = SHOW
    enable_timing = TIMING

    stats = StageStats()
    t_start = time.perf_counter()
    status = {'text': f'[{task}] 启动中...'}     # display 状态文字
    latest = {'frame': None, 'dets': []}         # display 最新帧+最近检测

    # 日志（主进程单点写）
    log_writer = None
    if LOG_ENABLED:
        base = os.path.dirname(os.path.abspath(__file__))
        log_path = os.path.join(base, LOG_DIR, f'{task}.log')
        log_writer = FrameLog(log_path, t_start)

    stop = threading.Event()
    reason = ['manual']
    idle_timer = IdleTimer()

    print(f'[*] [{task}] 启动: worker进程数={workers} 空闲超时='
          f'{idle_timeout if idle_timeout else "不限"} '
          f'YOLO={enable_yolo} 虚拟={virtual} 显示={show}', flush=True)

    # ---- worker 进程（先于相机 fork，保证子进程不继承 V4L2 fd）----
    in_q = _ctx.Queue(maxsize=Q_SIZE)
    out_q = _ctx.Queue(maxsize=64)
    proc_stop = _ctx.Event()
    procs = []
    if enable_yolo:
        for i in range(workers):
            p = _ctx.Process(target=worker_main,
                             args=(i, yolo_cfg, pre_cfg, enable_pre,
                                   in_q, out_q, proc_stop),
                             daemon=True)
            p.start()
            procs.append(p)

    # ---- 相机（进程 ready 后再开，避免 fork 后 fd 混乱；失败自动重试）----
    cap_holder = {'cap': None}
    if not virtual:
        cap_holder['cap'] = open_camera_with_retry(task, cam_cfg, status, stop)
        if cap_holder['cap'] is None:
            print(f'[{task}] 相机打开失败(重试 {OPEN_RETRIES} 次后仍失败)，'
                  f'会话结束', flush=True)
            if log_writer is not None:
                log_writer.write(task, 0, [], 0.0, status='no_data')
                log_writer.close()
            proc_stop.set()
            for p in procs:
                p.join(timeout=3)
            try:
                in_q.close()
                out_q.close()
            except Exception:
                pass
            return 'no_data'

    # ---- 线程组 ----
    dropped_cnt = [0]
    threads = [
        threading.Thread(target=producer,
                         args=(task, cam_cfg, cap_holder, in_q, workers,
                               virtual, with_target, max_frames, stop,
                               reason, status, latest, stats, enable_timing,
                               log_writer, idle_timer, dropped_cnt),
                         daemon=True),
        threading.Thread(target=listener,
                         args=(task, out_q, stop, reason, idle_timer, stats,
                               enable_timing, log_writer, mark_point, latest),
                         daemon=True),
    ]
    if show:
        threads.append(threading.Thread(target=display_loop,
                                        args=(task, stop, status, latest),
                                        daemon=True))
    watchdog = None
    if idle_timeout is not None and idle_timeout > 0:
        watchdog = threading.Thread(target=idle_watchdog,
                                    args=(task, idle_timer, idle_timeout,
                                          stop, reason,
                                          bool(CFG.get('SWITCH_LOG', True))),
                                    daemon=True)

    for th in threads:
        th.start()
    if watchdog is not None:
        watchdog.start()

    # ---- 主线程等待结束信号 ----
    try:
        while not stop.is_set():
            time.sleep(IDLE_CHECK)
    except KeyboardInterrupt:
        stop.set()
        reason[0] = 'manual'

    # ---- 收尾：停采集线程 -> 停 worker 进程 -> 关相机 ----
    for th in threads:
        th.join(timeout=5)
    proc_stop.set()
    for p in procs:
        p.join(timeout=5)
        if p.is_alive():
            p.terminate()
    try:
        in_q.close()
        out_q.close()
    except Exception:
        pass

    # 结束统计
    if enable_timing:
        elapsed = time.perf_counter() - t_start
        total = max(stats.capture_frames, 1)
        print(f'[{task}] 结束统计: 会话原因={reason[0]} 总运行={elapsed:.2f}s '
              f'总帧数={stats.capture_frames} 总检测帧={stats.detect_frames} '
              f'采集帧率={total / elapsed:.1f}fps '
              f'队列丢帧={dropped_cnt[0]}', flush=True)

    cap = cap_holder.get('cap')
    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass
    if log_writer is not None:
        log_writer.close()
    return reason[0]


# ============================================================
# main
# ============================================================
def main():
    """CLI 入口：cam1 会话（空闲超时监督）-> 切 cam2 会话（单向，固定跑）。"""
    ap = argparse.ArgumentParser(description='vp6.0 双相机单向切换 + YOLO 检测')
    ap.add_argument('--virtual', action='store_true',
                    help='使用虚拟相机（合成帧，无实体相机联调）')
    ap.add_argument('--no-target', action='store_true',
                    help='虚拟帧不带目标（用于验证空闲超时切换路径）')
    ap.add_argument('--frames', type=int, default=0,
                    help='每会话限帧数，0=无限（默认）')
    ap.add_argument('--workers', type=int, default=N_WORKERS,
                    help=f'worker 进程数（默认 {N_WORKERS}）')
    ap.add_argument('--timeout', type=float, default=None,
                    help='覆盖 cam1 空闲超时秒数（默认读 config '
                         'CAM1_IDLE_TIMEOUT）')
    ap.add_argument('--no-show', action='store_true', help='关闭 imshow 显示')
    ap.add_argument('--no-timing', action='store_true',
                    help='关闭时间统计/帧率计时')
    ap.add_argument('--no-log', action='store_true', help='关闭帧日志保存')
    args = ap.parse_args()

    global SHOW, TIMING, LOG_ENABLED
    if args.no_show:
        SHOW = False
    if args.no_timing:
        TIMING = False
    if args.no_log:
        LOG_ENABLED = False

    cam1_on = bool(CFG.get('ENABLE_CAM1', True))
    cam2_on = bool(CFG.get('ENABLE_CAM2', True))
    timeout = args.timeout if args.timeout is not None \
        else float(CFG.get('CAM1_IDLE_TIMEOUT', 30.0))

    # ---- 阶段一: cam1（空闲超时 -> 切 cam2）----
    cam1_reason = 'disabled'
    if cam1_on:
        cam1_reason = run_session(
            'cam1',
            CFG['CAM1_CAMERA'],
            CFG['CAM1_YOLO'],
            CFG.get('CAM1_PREPROCESS'),
            enable_pre=bool(CFG.get('ENABLE_PREPROCESS_CAM1', False)),
            enable_yolo=bool(CFG.get('ENABLE_CAM1_YOLO', True)),
            virtual=args.virtual,
            with_target=not args.no_target,
            max_frames=args.frames,
            idle_timeout=timeout,
            workers=args.workers)

    # ---- 阶段二: cam2（单向切换，固定运行；不再设 idle 监督）----
    # cam1 未启用/相机无法恢复/空闲超时 -> 切 cam2（failover + 单向切换）；
    # init_fail / frames / manual -> 不再切
    if cam2_on and cam1_reason in ('idle_timeout', 'no_data', 'disabled'):
        if cam1_reason != 'disabled':
            print(f'[*] cam1 会话结束({cam1_reason})，切换到 cam2，之后不再'
                  f'切回', flush=True)
        else:
            print('[*] ENABLE_CAM1=False，直接启用 cam2', flush=True)
        run_session('cam2',
                    CFG['CAM2_CAMERA'],
                    CFG['CAM2_YOLO'],
                    CFG.get('CAM2_PREPROCESS'),
                    enable_pre=bool(CFG.get('ENABLE_PREPROCESS_CAM2', False)),
                    enable_yolo=bool(CFG.get('ENABLE_CAM2_YOLO', True)),
                    virtual=args.virtual,
                    with_target=not args.no_target,
                    max_frames=args.frames,
                    idle_timeout=None,      # 单向：cam2 不再切回
                    workers=args.workers)
    else:
        print(f'[*] 程序结束：cam1 会话退出原因={cam1_reason} '
              f'(cam2_on={cam2_on})', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
