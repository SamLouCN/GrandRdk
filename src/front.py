#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
front.py — 前视任务入口（真实相机 + YOLO 检测输出）

数据流:
  camera --producer--> q --worker x N--> YOLO 结果
                                     |
                                     |--> 终端 / logs/front.log
                                     |--> 帧: /dev/shm/momo_frame_front.bin (JPEG)
                                     |--> 检测: /dev/shm/momo_det_front.json
                                     |--> 统计: /dev/shm/momo_stats_front.json
                                     |--> imshow（SHOW=True 时）

历史变更：
  - 2026-09-21: 删除 --virtual 与 make_frame() 合成帧逻辑。
    所有数据依靠 To32 中位机回传的真实链路, 板端只做采集/检测/转发, 不再生成伪造帧。
"""
import argparse  
import os   
import queue  
import sys  
import threading  
import time  

# ---- 路径注入: config/ + src/ + src/utils/ ----
_HERE = os.path.dirname(os.path.abspath(__file__))  # 当前文件所在目录 src/ 的绝对路径，避免受运行目录影响
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 工程根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'),  # 依次把三个目录加入模块搜索路径
           _HERE,  # 第二个路径：src/ 目录本身
           os.path.join(_HERE, 'utils')):  # 第三个路径：src/utils/ 补充工具模块
    if _p not in sys.path:  # 已存在就跳过，避免重复插入同名路径
        sys.path.insert(0, _p)  # 插到最前面，保证优先命中本工程而不是系统同名模块

import cv2  
import numpy as np  

import main_config as MC                                              # 主配置模块，提供共享内存名等全局常量
from main_config import DEFAULT_CONFIG as CFG                         # 默认配置字典，本脚本所有参数都从这里读
from function import YoloDetector, format_detection, draw_detections  # YOLO 检测器、检测结果格式化、画检测框
from shm_writer import ShmFrameWriter, ShmJsonWriter                  # 共享内存写端：帧二进制写与 JSON 写


TASK = 'front'                                                        # 任务名，用于日志前缀和共享内存命名
CAM = CFG['FRONT_CAMERA']                                             # 前视相机参数：分辨率、帧率、设备号、像素格式
YOLO_CFG = CFG['FRONT_YOLO']                                          # 前视 YOLO 参数：模型路径、置信度阈值、预处理方式

W = int(CAM.get('width', 640))  # 采集宽度，缺省 640
H = int(CAM.get('height', 480))  # 采集高度，缺省 480
FPS = max(1, min(int(CAM.get('fps', 60)), 60))  # 期望帧率，钳在 1~60 之间，防止非法值
MARK_POINT = CAM.get('mark_point')  # 标定点坐标，用于把像素位置换算成实际距离
Q_SIZE = int(CFG.get('CAMERA_QUEUE_SIZE', 8))  # 采集队列容量，满了就丢帧不让内存涨
LOOP_SLEEP = float(CFG.get('LOOP_SLEEP', 0.0))  # 采集循环每帧额外休眠，给其他线程让 CPU
CV_FRAME_MIN_INTERVAL_S = 0.10  # 干净帧最小写入间隔(节流): 过门 CV 只要 ~10Hz, 不按相机全速二次编码
CV_FRAME_JPEG_QUALITY = 85      # 干净帧 JPEG 质量: 红杆边缘足够, 比 q100 明显省 CPU
_cv_last_t = 0.0                # 最近一次干净帧写入时刻（模块级, 多 worker 共享节流）
SHOW = bool(CFG.get('SHOW', True))  # 是否弹窗显示画面，板端无屏时应关掉
N_WORKERS = int(CFG.get('N_WORKERS', 3))  # 检测 worker 线程数，决定推理并行度
TIMING = bool(CFG.get('ENABLE_TIMING', False))  # 是否统计各阶段耗时
TIMING_INTERVAL = int(CFG.get('TIMING_INTERVAL', 30))  # 每累计多少帧输出一次统计
SIMPLE_TIMING = bool(CFG.get('SIMPLE_TIMING', False))  # 简化统计模式：只记帧数，不细分阶段耗时
LOG_ENABLED = bool(CFG.get('ENABLE_LOG', False))  # 是否写逐帧日志文件
LOG_DIR = str(CFG.get('LOG_DIR', 'logs'))  # 日志目录

CAM_ENABLED = bool(CFG.get('ENABLE_FRONT_CAM', True))  # 前视相机总开关，关闭时 main 直接返回不采集
YOLO_ENABLED = bool(CFG.get('ENABLE_FRONT_YOLO', True))  # YOLO 总开关，关闭时只转发帧不做检测

PRE_MODE = str(YOLO_CFG.get('preprocess_mode', 'auto')).lower()  # 预处理模式，auto 时允许走 NV12 直通


# ============================================================
# 日志
# ============================================================
class FrameLog:  # 逐帧日志记录器，写到 logs/front.log
    def __init__(self, path, t_start_perf):  # path 为日志文件路径，t_start_perf 为进程起始计时点
        os.makedirs(os.path.dirname(path), exist_ok=True)  # 日志目录不存在就自动创建
        self.fp = open(path, 'a', encoding='utf-8')  # 以追加方式打开，进程重启不清空历史日志
        self.lock = threading.Lock()  # 写日志的互斥锁，防止多线程写串行
        self.t_start_perf = t_start_perf  # 记录起始时刻，日志里输出相对秒数

    def write(self, frame_id, dets, status='done'):  # 写一行帧日志，dets 为检测结果，status 标记帧状态
        ts_str = time.strftime('%H:%M:%S')  # 墙钟时间，方便人工对照其他设备日志
        elapsed = time.perf_counter() - self.t_start_perf  # 相对进程启动的秒数，比墙钟精度更高
        if status == 'dropped':  # 队列满被丢弃的帧
            result = 'dropped(queue_full)'  # 标记为队列满丢弃
        elif status == 'no_yolo':  # 未启用 YOLO 的帧
            result = 'no_yolo'  # 标记本帧没做检测
        elif dets:  # 有检测结果时逐项展开
            result = '; '.join(  # 多个检测目标拼成一行，用分号分隔
                f"{d['label']} conf={d['score']:.2f} "  # 标签与置信度，置信度保留两位小数
                f"bbox={tuple(d['bbox'])} center={tuple(d['center'])}"  # 框坐标与中心点，便于事后回溯误检位置
                for d in dets)  # 遍历本帧所有检测目标
        else:  # 既没有检测也没有结果
            result = 'none'  # 标记本帧无目标
        line = f'[{ts_str}][{elapsed:.3f}s][frame:{frame_id}] [{status}]: {result}\n'  # 拼成固定格式的一行日志文本
        with self.lock:  # 加锁，避免多线程写串行
            self.fp.write(line)  # 写入文件
            self.fp.flush()  # 立即刷盘，掉电也能看到最近的日志

    def close(self):  # 关闭日志文件
        with self.lock:  # 加锁，避免与正在写的线程冲突
            try:  # 关闭可能失败，比如句柄已失效
                self.fp.close()  # 释放文件句柄
            except Exception:  # 吞掉所有异常，保证收尾流程不崩
                pass  # 关闭失败就忽略


# ============================================================
# 阶段耗时统计
# ============================================================
class StageStats:  # 分阶段耗时统计：采集/检测/显示
    def __init__(self, simple=False):  # simple 为真时只累计帧数不累计耗时
        self.lock = threading.Lock()  # 多线程累加计数用的互斥锁
        self.simple = bool(simple)  # 记住是否为简化统计模式
        self.capture_s = 0.0  # 采集阶段累计耗时，单位秒
        self.detect_s = 0.0  # 检测阶段累计耗时
        self.display_s = 0.0  # 显示阶段累计耗时
        self.frames = 0  # add 调用总次数
        self.capture_frames = 0  # 已采集帧数
        self.detect_frames = 0  # 已检测帧数
        self.display_frames = 0  # 已投显帧数
        self._base_capture = 0.0  # 窗口基线：采集耗时快照
        self._base_detect = 0.0  # 窗口基线：检测耗时快照
        self._base_display = 0.0  # 窗口基线：显示耗时快照
        self._base_capture_f = 0  # 窗口基线：采集帧数快照
        self._base_detect_f = 0  # 窗口基线：检测帧数快照
        self._base_display_f = 0  # 窗口基线：显示帧数快照

    def add(self, stage, dt):  # 记录一次采样到对应阶段
        with self.lock:  # 加锁，保证多线程累加准确
            if stage == 'capture':  # 采集阶段
                if not self.simple:  # 简化模式下不累计耗时
                    self.capture_s += dt  # 累加采集耗时
                self.capture_frames += 1  # 采集帧计数，简化模式也要统计
            elif stage == 'detect':  # 检测阶段
                if not self.simple:  # 同上，简化模式跳过耗时
                    self.detect_s += dt  # 累加检测耗时
                self.detect_frames += 1  # 检测帧计数
            elif stage == 'display':  # 显示阶段
                if not self.simple:  # 同上，简化模式跳过耗时
                    self.display_s += dt  # 累加显示耗时
                self.display_frames += 1  # 显示帧计数
            self.frames += 1  # 每次调用都计入总次数

    def mark_window(self):  # 把当前累计值记为下一个统计窗口的基线
        with self.lock:  # 加锁，保证读取的三个值属于同一时刻
            self._base_capture = self.capture_s  # 快照采集耗时
            self._base_detect = self.detect_s  # 快照检测耗时
            self._base_display = self.display_s  # 快照显示耗时
            self._base_capture_f = self.capture_frames  # 快照采集帧数
            self._base_detect_f = self.detect_frames  # 快照检测帧数
            self._base_display_f = self.display_frames  # 快照显示帧数

    def delta_window(self):  # 返回自上次 mark_window 以来的各项增量
        with self.lock:  # 加锁取增量，避免边读边改导致错位
            return (  # 返回六元组：三段耗时 + 三个帧数
                self.capture_s - self._base_capture,  # 窗口内采集耗时增量
                self.detect_s - self._base_detect,  # 窗口内检测耗时增量
                self.display_s - self._base_display,  # 窗口内显示耗时增量
                self.capture_frames - self._base_capture_f,  # 窗口内采集帧数增量
                self.detect_frames - self._base_detect_f,  # 窗口内检测帧数增量
                self.display_frames - self._base_display_f,  # 窗口内显示帧数增量
            )


def format_timing(elapsed, interval_frames, d_capture, d_detect, d_display, title):  # 把窗口统计拼成一行可读文本
    if interval_frames <= 0 or elapsed <= 0:  # 帧数或时间为非正就放弃输出
        return ''  # 返回空串，调用方据此跳过打印
    fps = interval_frames / elapsed  # 窗口平均帧率
    ms = lambda s: s / interval_frames * 1000.0  # 把秒折算成每帧毫秒的换算函数
    return (f'[{TASK}] {title}: 帧数={interval_frames} 运行={elapsed:.2f}s '  # 开头部分：任务名、标题、帧数与运行时长
            f'平均帧率={fps:.1f}fps | '  # 平均帧率，保留一位小数
            f'采集={ms(d_capture):.2f}ms/帧 '  # 采集阶段单帧平均耗时
            f'检测={ms(d_detect):.2f}ms/帧 '  # 检测阶段单帧平均耗时
            f'显示={ms(d_display):.2f}ms/帧')  # 显示阶段单帧平均耗时


def open_camera(device, index):  # 打开相机：优先 JPU 硬解，不可用则回退 cv2 软解
    if CAM.get('hardware_decode', False):  # 配置要求尝试硬件解码
        dev = device if device is not None else '/dev/video%d' % (index if index is not None else 0)  # 未指定设备路径时按 index 拼出 /dev/videoN
        try:  # 导入 hw_camera 可能失败，必须容错
            from hw_camera import HwMjpgCamera  # 延迟导入，避免没有 JPU 环境时启动即崩
            cam = HwMjpgCamera(dev, W, H, int(CAM.get('fps', 200)))  # 用 MJPEG 源 + JPU 硬解打开，帧率上限给得更宽
            if cam.isOpened():  # 硬解设备可用
                print(f'[*] [{TASK}] 相机 {dev}: 启用 JPU 硬件解码', flush=True)  # 提示已启用硬解，flush 保证日志及时可见
                return cam  # 返回硬解相机对象
            print(f'[警告] [{TASK}] 相机 {dev}: JPU 硬件解码不可用, 回退 cv2 软解', flush=True)  # 打不开就警告，继续走软解分支
        except Exception as exc:  # 硬解初始化任何异常都不算致命
            print(f'[警告] [{TASK}] 相机硬件解码初始化异常 {exc!r}, 回退 cv2 软解', flush=True)  # 打印异常原因后回退软解
    src = device if device is not None else index  # 软解直接用设备路径或索引号
    cap = cv2.VideoCapture(src)  # 用 OpenCV 打开采集源
    if not cap.isOpened():  # 打开失败
        return None  # 返回 None，由上层决定是否退出进程
    fmt = CAM.get('format')  # 像素格式四字符码，例如 MJPG
    if fmt:  # 只有显式配了才设置，避免覆盖驱动默认值
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))  # 指定编码格式，MJPEG 流才能跑高帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)  # 设置采集宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)  # 设置采集高度
    fps_req = CAM.get('fps')  # 取配置的请求帧率
    if fps_req:  # 配了才下发帧率
        cap.set(cv2.CAP_PROP_FPS, int(fps_req))  # 向驱动请求目标帧率
    try:  # 部分 OpenCV 构建不支持超时属性
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)  # 读帧超时 1 秒，防止相机掉线时永久卡死
    except cv2.error:  # 不支持该属性就忽略
        pass  # 什么都不做
    return cap  # 返回可用的相机对象


def producer(cap, q, max_frames, stop, n_workers, stats, enable_timing,  # 采集线程：读帧入队，并周期性写共享内存统计
             log_writer, stats_w, prefer_nv12=False, want_bgr=True):  # prefer_nv12 走硬解原始帧，want_bgr 表示需要 BGR 输出
    """采集线程：读帧 -> 入队；周期性写统计到共享内存。

    只读真实相机; 不再支持虚拟帧(2026-09-21 移除 --virtual/make_frame)。
    """
    fid = 0  # 帧序号，从 0 起自增
    t_win = time.perf_counter()  # 当前统计窗口的起点时刻
    win_frames = 0  # 本窗口内已采集帧数
    fail_cnt = 0  # 连续读帧失败计数
    READ_FAIL_LIMIT = 5  # 连续失败多少帧判定相机掉线
    try:  # 无论正常结束还是异常都要给 worker 发结束信号
        while not stop.is_set():  # 停止事件未置位就一直采集
            if max_frames and fid >= max_frames:  # 达到指定帧数上限
                break  # 退出采集循环
            t0 = time.perf_counter()  # 本次读帧的起始时刻
            nv12 = None  # 原始 NV12 数据，软解时为 None
            if prefer_nv12:  # 硬解通道，可直接拿 NV12
                if want_bgr:  # 既要 NV12 也要 BGR
                    ok, frame, nv12 = cap.read_both()  # 一次同时取 BGR 与 NV12，避免重复解码
                else:  # 只要 NV12 的场合
                    ok, nv12 = cap.grab_raw_nv12()  # 只抓原始 NV12，省一次色彩转换
                    frame = None  # 此路径不带 BGR，后面按需转换
                if not ok or nv12 is None:  # 读失败或没拿到数据
                    fail_cnt += 1  # 连续失败计数加一
                    if log_writer is not None:  # 开了日志才写记录
                        log_writer.write(fid, [], status='no_data')  # 记为无数据帧，便于事后定位掉线时刻
                    if fail_cnt >= READ_FAIL_LIMIT:  # 连续失败达到阈值
                        print(f'[{TASK}] 相机掉线/无数据，本进程退出', flush=True)  # 提示相机已不可用
                        stop.set()  # 置停止事件，通知其他线程收尾
                        break  # 跳出采集循环
                    time.sleep(0.1)  # 短暂失败先等 100ms，避免瞬时抖动误判
                    fid += 1  # 失败也推进帧号，保持日志帧序连续
                    continue  # 本轮不投递，下一轮重试
                fail_cnt = 0  # 读成功就把失败计数清零
            else:  # 普通 cv2 软解通道
                ok, frame = cap.read()  # 读一帧 BGR 图像
                if not ok or frame is None:  # 读帧失败或拿到空帧
                    fail_cnt += 1  # 连续失败计数加一
                    if log_writer is not None:  # 开了日志才写记录
                        log_writer.write(fid, [], status='no_data')  # 记录无数据帧
                    if fail_cnt >= READ_FAIL_LIMIT:  # 连续失败达到上限
                        print(f'[{TASK}] 相机掉线/无数据，本进程退出', flush=True)  # 提示相机已不可用
                        stop.set()  # 通知所有线程收尾
                        break  # 退出采集循环
                    time.sleep(0.1)  # 等 100ms 后重试
                    fid += 1  # 帧号继续推进
                    continue  # 本轮不投递
                fail_cnt = 0  # 软解读成功，清零失败计数
            t1 = time.perf_counter()  # 读帧结束时刻
            if enable_timing:  # 只在开启统计时才计时
                stats.add('capture', t1 - t0)  # 记录本次采集耗时
            try:  # 入队可能超时
                q.put((fid, frame, nv12), timeout=1.0)  # 最多等 1 秒，避免 worker 全挂时无限阻塞
            except queue.Full:  # 队列已满，本帧丢弃
                if log_writer is not None:  # 开了日志才记
                    log_writer.write(fid, [], status='dropped')  # 记录丢帧，用于分析背压情况
            fid += 1  # 帧号递增
            win_frames += 1  # 窗口内帧数递增

            # ---- 周期性写 stats 到共享内存 ----
            if win_frames >= TIMING_INTERVAL:  # 攒满一个统计窗口就输出并写共享内存
                now = time.perf_counter()  # 取当前时刻
                elapsed = now - t_win  # 本窗口实际耗时
                fps_val = win_frames / max(elapsed, 1e-9)  # 窗口平均帧率，分母加保护防除零
                d_c, d_d, d_dp, n_c, n_d, n_dp = stats.delta_window()  # 取窗口内三段耗时与三个帧数的增量
                cap_ms = (d_c / n_c * 1000.0) if n_c else 0.0  # 采集单帧平均毫秒，无帧时记 0
                det_ms = (d_d / n_d * 1000.0) if n_d else 0.0  # 检测单帧平均毫秒，无帧时记 0
                if stats_w is not None:  # 共享内存统计写端存在才写
                    try:  # 写共享内存失败不能影响采集主流程
                        stats_w.write({  # 写入统计字典，供 Web 端读取
                            'fps': round(fps_val, 2),  # 实时帧率，保留两位小数
                            'capture_ms': round(cap_ms, 2),  # 采集耗时毫秒
                            'detect_ms': round(det_ms, 2),  # 检测耗时毫秒
                            'frames': fid,  # 累计帧号
                            'ts': time.time(),  # Unix 时间戳，供前端判断数据新鲜度
                        })
                    except Exception:  # 写共享内存异常一概忽略
                        pass  # 失败就跳过，下一窗口再试
                if enable_timing:  # 开了耗时统计才打印
                    if SIMPLE_TIMING:  # 区分简化与完整两种输出
                        print(f'[{TASK}] 简化时间统计@{fid}帧: 帧数={win_frames} '  # 简化输出：只报帧数、时长、帧率
                              f'运行={elapsed:.2f}s 平均帧率={fps_val:.1f}fps', flush=True)  # 续接上一行的运行时间与帧率
                    else:  # 完整输出模式
                        print(format_timing(elapsed, win_frames, d_c, d_d, d_dp,  # 调用 format_timing 生成详细一行
                                            f'时间统计@{fid}帧'), flush=True)  # 以当前帧号作为统计标题
                stats.mark_window()  # 结算后更新窗口基线
                t_win = now  # 新窗口从当前时刻起算
                win_frames = 0  # 窗口帧数归零

            if LOOP_SLEEP > 0:  # 配置了额外休眠才执行
                time.sleep(LOOP_SLEEP)  # 主动让出 CPU，控制采集节奏
    finally:  # 无论正常结束还是异常都执行
        for _ in range(n_workers):  # 给每个 worker 各放一个结束哨兵
            q.put(None)  # None 是 worker 的退出信号


def worker(wid, q, disp_q, show, stop, stats, enable_timing, log_writer,  # 检测 worker：推理并把帧与结果写进共享内存
           frame_w, det_w, cv_w=None):  # 共享内存写端：帧二进制、检测结果 JSON 与【干净帧】(CV 专用，可缺省)
    """检测 worker：检测 -> 写帧/检测结果到共享内存。"""
    detector = YoloDetector(YOLO_CFG) if YOLO_ENABLED else None  # 每个线程独立建检测器，避免跨线程争用模型资源
    idx = 0  # 本 worker 的检测序号，用于终端输出计数
    while not stop.is_set():  # 停止事件未置位就持续取帧
        try:  # 取队列可能超时
            item = q.get(timeout=0.5)  # 半秒超时，保证能及时响应停止事件
        except queue.Empty:  # 队列空
            continue  # 继续下一轮等待，不退出线程
        if item is None:  # 收到生产者放的结束哨兵
            break  # 退出 worker 循环
        fid, frame, nv12 = item  # 拆包得到帧号、BGR 帧、NV12 原始数据
        status = 'no_yolo' if detector is None else 'done'  # 未启用 YOLO 时日志状态标记为 no_yolo
        if enable_timing:  # 只在开启统计时计时
            t0 = time.perf_counter()  # 检测开始时刻
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []  # 有 NV12 就直接喂给模型，省一次色彩转换
            t1 = time.perf_counter()  # 检测结束时刻
            stats.add('detect', t1 - t0)  # 记录本次检测耗时
        else:  # 不开统计时走同一逻辑但不计时
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []  # 与上面等价，只是省掉计时开销

        if log_writer is not None:  # 开了日志才写
            log_writer.write(fid, dets, status=status)  # 每帧落一条日志

        # ---- 准备可视帧（BGR）----
        if frame is None and nv12 is not None:  # 只有 NV12 时需要自己转成 BGR
            frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)  # NV12 转 BGR，供显示和 JPEG 编码使用
        if frame is not None:  # 有可视帧才做后续写共享内存与显示
            vis = draw_detections(frame, dets) if dets else frame  # 有检测目标就画框，否则直接用原图

            # ---- 写帧到共享内存 ----
            if frame_w is not None:  # 帧共享内存写端存在才写
                ok, buf = cv2.imencode(  # 编码为 JPEG，比原始 BGR 小得多
                    '.jpg', vis,
                    [cv2.IMWRITE_JPEG_QUALITY, MC.WEB_MJPEG_QUALITY])  # 压缩质量取全局配置，平衡带宽与画面清晰度
                if ok:  # 编码成功才有数据可写
                    try:
                        frame_w.write(buf.tobytes())  # 把 JPEG 字节写入共享内存 /dev/shm
                    except Exception:  # 写失败不影响主流程
                        pass  # 忽略异常，继续处理下一帧

            # ---- 写【干净帧】到 CV 专用共享内存（2026-10-08 过门找红杆用）----
            # OpenCV 找红杆要吃原始像素：黄框线宽 2 正好压在 ~8px 宽的红杆上会把线切断。
            # frame 本体自始未被画过（draw_detections 内部先 copy），这里直接编码原始帧。
            # ★节流: CV 只需 ~10Hz, 相机 200fps 全速二次编码会显著抬高 CPU（回传被饿死的元凶之一）
            if cv_w is not None:  # 干净帧写端存在才写
                global _cv_last_t
                now_cv = time.time()
                if now_cv - _cv_last_t >= CV_FRAME_MIN_INTERVAL_S:
                    try:
                        ok_cv, buf_cv = cv2.imencode(  # 编码原始帧（无任何叠加）
                            '.jpg', frame,
                            [cv2.IMWRITE_JPEG_QUALITY, CV_FRAME_JPEG_QUALITY])
                        if ok_cv:  # 编码成功才写
                            cv_w.write(buf_cv.tobytes())  # 写入 momo_frame_front_cv.bin
                            _cv_last_t = now_cv
                    except Exception:  # 编码/写失败一概不影响主流程
                        pass  # CV 侧按丢帧处理

            # ---- 写检测结果 JSON ----
            if det_w is not None:  # 检测 JSON 写端存在才写
                try:  # 写失败不中断
                    det_w.write({  # 写入本帧检测结果
                        'frame': fid,  # 帧号，供消费端对齐画面
                        'ts': time.time(),  # 时间戳，供消费端判断数据是否过期
                        'dets': dets,  # 检测目标列表
                    })
                except Exception:  # 写 JSON 异常一概忽略
                    pass  # 跳过本帧，下一帧再试

            # ---- imshow ----
            if show:  # 开了显示才往显示队列投帧
                t2 = time.perf_counter()  # 投递开始时刻
                try:  # 投显示队列可能失败
                    disp_q.put_nowait(vis)  # 非阻塞投递，显示跟不上就丢旧帧保实时性
                except queue.Full:  # 显示队列已满
                    pass  # 丢弃即可，显示不属于关键链路
                t3 = time.perf_counter()  # 投递结束时刻
                if enable_timing:  # 只在开启统计时记录
                    stats.add('display', t3 - t2)  # 计入显示阶段耗时

        for d in dets:  # 逐个目标打印到终端
            print(f'[{TASK}] ' + format_detection(d, MARK_POINT, idx), flush=True)  # 带标定点换算后输出目标的距离与方位
            idx += 1  # 检测序号递增
    q.task_done()  # 通知队列本任务处理完成


def display_loop(disp_q, stop):  # 显示线程：独立跑 imshow，避免阻塞推理
    win = TASK  # 窗口名用任务名 front
    try:  # 捕获无 GUI 环境下的 OpenCV 报错
        while not stop.is_set():  # 未收到停止信号就一直显示
            try:  # 取显示帧可能超时
                vis = disp_q.get(timeout=0.1)  # 100ms 超时，保证能及时检查退出条件
            except queue.Empty:  # 没有新帧
                continue  # 继续等待下一帧
            cv2.imshow(win, vis)  # 显示带检测框的画面
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 q 键手动退出
                stop.set()  # 通知所有工作线程收尾
                break  # 结束显示循环
    except cv2.error:  # headless 环境没有窗口可用
        pass  # 直接忽略，不影响其他线程
    finally:  # 无论如何都尝试清理窗口
        try:  # 窗口可能已被销毁
            cv2.destroyWindow(win)  # 关闭显示窗口释放资源
        except cv2.error:  # 关闭失败
            pass  # 忽略即可


def main():  # 程序入口：组装参数、起线程、收尾
    ap = argparse.ArgumentParser(description='front 任务（真实相机 + YOLO）')  # 命令行解析器
    ap.add_argument('--device', type=str, default=None)  # 指定相机设备路径，缺省取配置文件
    ap.add_argument('--index', type=int, default=None)  # 指定相机索引号，与 --device 二选一
    ap.add_argument('--no-show', action='store_true')  # 关闭画面显示，无屏环境必带
    ap.add_argument('--workers', type=int, default=N_WORKERS)  # 覆盖配置里的 worker 数量
    ap.add_argument('--frames', type=int, default=0)  # 限制采集帧数，0 表示不限制
    ap.add_argument('--timing', action='store_true')  # 强制开启耗时统计
    ap.add_argument('--no-timing', action='store_true')  # 强制关闭耗时统计
    ap.add_argument('--log', action='store_true')  # 强制开启逐帧日志
    ap.add_argument('--no-log', action='store_true')  # 强制关闭逐帧日志
    args = ap.parse_args()  # 解析命令行参数

    enable_timing = TIMING  # 默认取配置文件里的统计开关
    if args.timing:  # 命令行显式要求开启
        enable_timing = True  # --timing 优先级最高
    if args.no_timing:  # 命令行显式要求关闭
        enable_timing = False  # --no-timing 覆盖配置文件
    enable_log = LOG_ENABLED  # 日志开关默认取配置文件
    if args.log:  # 命令行要求开日志
        enable_log = True  # 命令行 --log 优先生效
    if args.no_log:  # 命令行要求关日志
        enable_log = False  # --no-log 覆盖配置文件

    if not CAM_ENABLED:  # 前视相机总开关被关闭
        print(f'[{TASK}] ENABLE_FRONT_CAM=False，跳过真实相机任务')  # 提示跳过任务（此行未加 flush，可能被缓冲）
        return 0  # 正常退出，不算错误

    show = SHOW and not args.no_show  # 配置为开且命令行没关时才显示画面

    stats = StageStats(simple=SIMPLE_TIMING)  # 创建阶段耗时统计对象
    stats.mark_window()  # 先记基线，第一个窗口的统计才准确
    t_start = time.perf_counter()  # 进程起始时刻，日志与总耗时都以它为基准

    log_writer = None  # 默认不写日志
    if enable_log:  # 日志开关打开才创建写对象
        log_writer = FrameLog(os.path.join(LOG_DIR, f'{TASK}.log'), t_start)  # 日志文件为 logs/front.log

    # ---- 共享内存写端 ----
    print(f'[*] [{TASK}] 初始化共享内存...', flush=True)  # 提示开始初始化共享内存
    if os.environ.get('GRDK_STUB_MODEL') == '1':  # 测试接缝：置 1 跳过 BPU 模型、改用桩检测器。仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
        class _StubDet:  # 假检测器：吐居中且缓慢放大的门框
            def __init__(self, *_a, **_k):
                self._n = 0
            def detect(self, frame, nv12=None):
                self._n += 1
                cx, cy = W // 2, H // 2  # 板端 cv2 4.11 的 rectangle 只认 int 点，必须取整
                half = min(60 + (self._n % 200) * 6 // 10, W // 2 - 10)
                hh = max(1, int(half * 0.75))
                bbox = (int(cx - half), int(cy - hh), int(cx + half), int(cy + hh))
                return [{'bbox': bbox, 'center': (int(cx), int(cy)),
                         'label': 'door', 'score': 0.92}]
        globals()["YoloDetector"] = _StubDet  # 必须写全局：worker() 读的是模块级名字
    frame_w = ShmFrameWriter(MC.SHM_FRAME_FRONT, W, H)  # 帧共享内存写端，路径见 MC.SHM_FRAME_FRONT
    cv_w    = ShmFrameWriter(MC.SHM_FRAME_FRONT_CV, W, H)  # 【干净帧】写端，专供过门 CV 找红杆（无叠加框）
    det_w   = ShmJsonWriter(MC.SHM_DET_FRONT)  # 检测结果 JSON 写端
    stats_w = ShmJsonWriter(MC.SHM_STATS_FRONT)  # 统计 JSON 写端
    print(f'[*] [{TASK}] 帧共享 : {MC.SHM_FRAME_FRONT}', flush=True)  # 打印帧共享内存路径，便于部署时核对
    print(f'[*] [{TASK}] CV帧共享: {MC.SHM_FRAME_FRONT_CV}', flush=True)  # 打印干净帧路径，便于部署时核对
    print(f'[*] [{TASK}] 检测共享: {MC.SHM_DET_FRONT}', flush=True)  # 打印检测共享内存路径
    print(f'[*] [{TASK}] 统计共享: {MC.SHM_STATS_FRONT}', flush=True)  # 打印统计共享内存路径

    device = args.device if args.device is not None else CAM.get('device')  # 命令行 --device 优先，否则用配置的设备路径
    index = args.index if args.index is not None else CAM.get('index')  # 命令行 --index 优先，否则用配置的索引号
    cap = open_camera(device, index)  # 打开相机，内部优先尝试硬解
    if cap is None:  # 相机打开失败
        print(f'[{TASK}] 真实相机打开失败: device={device} index={index}，进程退出', flush=True)  # 打印失败详情，便于现场排查设备号
        frame_w.close(); det_w.close(); stats_w.close(); cv_w.close()  # 退出前先释放已创建的共享内存写端
        return 1  # 返回非零退出码，供看门狗判定启动失败

    src_is_hw = cap is not None and hasattr(cap, 'grab_raw_nv12') and hasattr(cap, 'read_both')  # 用鸭子类型判断是不是硬解相机对象
    prefer_nv12 = bool(src_is_hw and PRE_MODE in ('auto', 'nv12'))  # 只有硬解且预处理模式允许时才走 NV12 直通
    want_bgr = True   # 需要写共享内存，必须能拿到 BGR

    q = queue.Queue(maxsize=Q_SIZE)  # 采集到检测的队列，容量受限形成背压
    disp_q = queue.Queue(maxsize=2)  # 显示队列很小，最差情况丢旧帧保实时性
    stop = threading.Event()  # 全局停止事件，所有线程靠它统一收尾

    threads = [threading.Thread(  # 线程列表，第一个是采集线程
        target=producer,  # 线程入口为 producer
        args=(cap, q, args.frames, stop, args.workers,  # 传入相机、队列、帧上限、停止事件与统计对象
              stats, enable_timing, log_writer, stats_w,  # 继续传统计、日志与共享内存写端
              prefer_nv12, want_bgr),  # 是否走 NV12 直通、是否需要 BGR
        daemon=True)]  # 设为守护线程，主进程退出即刻回收
    for i in range(args.workers):  # 按配置数量启动多个检测 worker
        threads.append(threading.Thread(  # 追加一个 worker 线程
            target=worker,  # 线程入口为 worker
            args=(i, q, disp_q, show, stop, stats, enable_timing, log_writer,  # 第 i 个 worker 的全部入参
                  frame_w, det_w, cv_w),  # 共享内存写端：帧、检测结果与干净帧(CV)
            daemon=True))  # 守护线程属性
    if show:  # 需要显示才起显示线程
        threads.append(threading.Thread(  # 追加显示线程
            target=display_loop, args=(disp_q, stop), daemon=True))  # 显示线程只需要显示队列和停止事件

    for th in threads:  # 遍历所有线程
        th.start()  # 统一启动
    try:  # join 期间可被 Ctrl+C 打断
        for th in threads:  # 逐个等待结束
            th.join()  # 阻塞等待线程退出
    except KeyboardInterrupt:  # 捕获 Ctrl+C
        stop.set()  # 置停止事件，让各线程自行退出循环
    finally:  # 不论正常还是异常都清理资源
        if cap is not None:  # 相机对象存在才释放
            cap.release()  # 释放相机设备，否则下次可能打不开
        if log_writer is not None:  # 日志对象存在才关闭
            log_writer.close()  # 关闭逐帧日志文件
        frame_w.close()  # 关闭帧共享内存写端
        cv_w.close()  # 关闭干净帧共享内存写端
        det_w.close()  # 关闭检测 JSON 写端
        stats_w.close()  # 关闭统计 JSON 写端
        if enable_timing:  # 开了统计才打印汇总
            elapsed = time.perf_counter() - t_start  # 计算总运行时长
            total = max(stats.capture_frames, 1)  # 至少取 1，防止除零
            print(f'[{TASK}] 结束统计: 总运行={elapsed:.2f}s '  # 打印汇总信息开头：总运行时长
                  f'总帧数={stats.capture_frames} 总检测帧={stats.detect_frames} '  # 总帧数与总检测帧数
                  f'平均帧率={total / elapsed:.1f}fps', flush=True)  # 平均帧率按总帧数除以总时长估算
    return 0  # 正常退出


if __name__ == '__main__':  # 仅直接运行时才执行，被 import 时不启动
    raise SystemExit(main())  # 把 main 的返回值作为进程退出码
