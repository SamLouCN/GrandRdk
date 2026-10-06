#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bottom.py — 下视任务入口（真实/虚拟相机 + YOLO 检测输出）

数据流:
  camera/虚拟帧 --producer--> q --worker x N--> YOLO 结果
                                     |
                                     |--> 终端 / logs/bottom.log
                                     |--> 帧: /dev/shm/momo_frame_bottom.bin (JPEG)
                                     |--> 检测: /dev/shm/momo_det_bottom.json
                                     |--> 统计: /dev/shm/momo_stats_bottom.json
                                     |--> 光流: /dev/shm/momo_flow_bottom.bin (NV12) [2026-10-04 已停用, 不再写]
                                     |--> imshow（SHOW=True 时）
"""
import argparse  # 命令行参数解析，用于覆盖配置项
import os  # 路径拼接与目录创建
import queue  # 线程安全队列，连接采集与检测线程
import sys  # 模块搜索路径注入
import threading  # 多线程与停止事件
import time  # 计时、时间戳与帧率统计

_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录，即 src/
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 项目根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'),  # 待注入的第一个路径：config/
           _HERE,  # 第二个路径：src/ 自身
           os.path.join(_HERE, 'utils')):  # 第三个路径：src/utils/
    if _p not in sys.path:  # 已存在则不重复插入
        sys.path.insert(0, _p)  # 插到最前面，优先命中本地模块

import cv2  # OpenCV：读帧、颜色转换、绘制、imshow
import numpy as np  # 数值数组，供图像处理与帧共享使用

import main_config as MC  # 全局配置模块，提供共享内存名与画质参数
from main_config import DEFAULT_CONFIG as CFG  # 默认配置字典
from function import YoloDetector, format_detection, draw_detections  # 检测器、结果格式化与绘制
from shm_writer import ShmFrameWriter, ShmJsonWriter  # 共享内存帧写端与 JSON 写端


TASK = 'bottom'  # 任务标识，用于日志前缀与共享内存命名
CAM = CFG['BOTTOM_CAMERA']  # 下视相机配置段
YOLO_CFG = CFG['BOTTOM_YOLO']  # 下视 YOLO 配置段

FLOW_SHARE_ON = bool(CFG.get('ENABLE_FLOW_SHARE_BOTTOM', False))  # 是否向外共享光流帧

W = int(CAM.get('width', 640))  # 采集图像宽度，缺省 640
H = int(CAM.get('height', 480))  # 采集图像高度，缺省 480
FPS = max(1, min(int(CAM.get('fps', 60)), 60))  # 请求帧率，夹在 1~60 之间防非法值
MARK_POINT = CAM.get('mark_point')  # 画面标记点，用于算目标相对偏移
ROTATE_180 = bool(CAM.get('rotate_180', False))  # 下视相机物理装反时置 True，软件补偿 180° 旋转
Q_SIZE = int(CFG.get('CAMERA_QUEUE_SIZE', 8))  # 采集队列容量，满则丢帧
LOOP_SLEEP = float(CFG.get('LOOP_SLEEP', 0.0))  # 每帧采集后的额外休眠，用于限流
SHOW = bool(CFG.get('SHOW', True))  # 是否弹窗显示画面
N_WORKERS = int(CFG.get('N_WORKERS', 3))  # 默认检测线程数
TIMING = bool(CFG.get('ENABLE_TIMING', False))  # 是否统计各阶段耗时
TIMING_INTERVAL = int(CFG.get('TIMING_INTERVAL', 30))  # 每多少帧输出一次统计
SIMPLE_TIMING = bool(CFG.get('SIMPLE_TIMING', False))  # 简化统计输出开关
LOG_ENABLED = bool(CFG.get('ENABLE_LOG', False))  # 是否写逐帧日志文件
LOG_DIR = str(CFG.get('LOG_DIR', 'logs'))  # 日志文件存放目录

CAM_ENABLED = bool(CFG.get('ENABLE_BOTTOM_CAM', True))  # 下视相机总开关
YOLO_ENABLED = bool(CFG.get('ENABLE_BOTTOM_YOLO', True))  # 下视推理总开关

PRE_MODE = str(YOLO_CFG.get('preprocess_mode', 'auto')).lower()  # 预处理模式：auto 优先走 NV12


# ============================================================
# FrameLog / StageStats / format_timing  — 同 front.py
# ============================================================
class FrameLog:  # 逐帧日志写入器，多线程共用一个文件句柄
    def __init__(self, path, t_start_perf):  # 打开日志文件并记录启动时间基准
        os.makedirs(os.path.dirname(path), exist_ok=True)  # 确保日志目录存在，已存在不报错
        self.fp = open(path, 'a', encoding='utf-8')  # 追加方式打开，避免覆盖历史日志
        self.lock = threading.Lock()  # 写锁，防止多线程写串
        self.t_start_perf = t_start_perf  # 进程启动的 perf 时间，用于算相对时刻

    def write(self, frame_id, dets, status='done'):  # 记录一帧的处理结果或状态
        ts_str = time.strftime('%H:%M:%S')  # 当前挂钟时间，便于人眼定位
        elapsed = time.perf_counter() - self.t_start_perf  # 相对启动时刻的秒数
        if status == 'dropped':  # 帧因队列满被丢弃
            result = 'dropped(queue_full)'  # 写明丢弃原因，便于排查性能问题
        elif status == 'no_yolo':  # 推理被关闭
            result = 'no_yolo'  # 标记本帧未做检测
        elif dets:  # 有检测结果
            result = '; '.join(  # 多目标拼接成一行
                f"{d['label']} conf={d['score']:.2f} "  # 类别名与置信度
                f"bbox={tuple(d['bbox'])} center={tuple(d['center'])}"  # 外接框与中心点
                for d in dets)  # 遍历本帧全部检测
        else:  # 推理正常但没检出目标
            result = 'none'  # 标记空结果
        line = f'[{ts_str}][{elapsed:.3f}s][frame:{frame_id}] [{status}]: {result}\n'  # 组装整行日志
        with self.lock:  # 加锁保护文件写入
            self.fp.write(line)  # 写入日志行
            self.fp.flush()  # 立即落盘，进程被杀时也不丢日志

    def close(self):  # 关闭日志文件释放句柄
        with self.lock:  # 与写操作互斥
            try:  # 关闭可能因文件已失效而抛错
                self.fp.close()  # 关闭句柄
            except Exception:  # 关闭失败不影响退出流程
                pass  # 静默忽略


class StageStats:  # 采集/检测/显示三阶段的累计耗时与帧数统计
    def __init__(self, simple=False):  # simple=True 时只记帧数不记耗时，降低开销
        self.lock = threading.Lock()  # 保护所有统计字段
        self.simple = bool(simple)  # 简化模式标志
        self.capture_s = 0.0  # 采集阶段累计秒数
        self.detect_s = 0.0  # 检测阶段累计秒数
        self.display_s = 0.0  # 显示阶段累计秒数
        self.frames = 0  # add 调用总次数
        self.capture_frames = 0  # 采集帧数
        self.detect_frames = 0  # 检测帧数
        self.display_frames = 0  # 显示帧数
        self._base_capture = 0.0  # 统计窗口起点：采集秒数
        self._base_detect = 0.0  # 统计窗口起点：检测秒数
        self._base_display = 0.0  # 统计窗口起点：显示秒数
        self._base_capture_f = 0  # 统计窗口起点：采集帧数
        self._base_detect_f = 0  # 统计窗口起点：检测帧数
        self._base_display_f = 0  # 统计窗口起点：显示帧数

    def add(self, stage, dt):  # 累加某个阶段的一次耗时
        with self.lock:  # 加锁保证多线程累加正确
            if stage == 'capture':  # 采集阶段
                if not self.simple:  # 简化模式跳过时间累计
                    self.capture_s += dt  # 累加采集耗时
                self.capture_frames += 1  # 采集帧数加一
            elif stage == 'detect':  # 检测阶段
                if not self.simple:  # 同上，简化模式不计时
                    self.detect_s += dt  # 累加检测耗时
                self.detect_frames += 1  # 检测帧数加一
            elif stage == 'display':  # 显示阶段
                if not self.simple:  # 同上，简化模式不计时
                    self.display_s += dt  # 累加显示耗时
                self.display_frames += 1  # 显示帧数加一
            self.frames += 1  # 总调用次数加一

    def mark_window(self):  # 把当前累计值存为下一个统计窗口的基线
        with self.lock:  # 与 add 互斥，避免读到半更新状态
            self._base_capture = self.capture_s  # 记录采集时间基线
            self._base_detect = self.detect_s  # 记录检测时间基线
            self._base_display = self.display_s  # 记录显示时间基线
            self._base_capture_f = self.capture_frames  # 记录采集帧数基线
            self._base_detect_f = self.detect_frames  # 记录检测帧数基线
            self._base_display_f = self.display_frames  # 记录显示帧数基线

    def delta_window(self):  # 返回自上次 mark_window 以来的各项增量
        with self.lock:  # 加锁取一致快照
            return (  # 依次为三个阶段的秒数和帧数增量
                self.capture_s - self._base_capture,  # 采集耗时增量
                self.detect_s - self._base_detect,  # 检测耗时增量
                self.display_s - self._base_display,  # 显示耗时增量
                self.capture_frames - self._base_capture_f,  # 采集帧数增量
                self.detect_frames - self._base_detect_f,  # 检测帧数增量
                self.display_frames - self._base_display_f,  # 显示帧数增量
            )


def format_timing(elapsed, interval_frames, d_capture, d_detect, d_display, title):  # 格式化一行时间统计
    if interval_frames <= 0 or elapsed <= 0:  # 无有效窗口数据则不输出
        return ''  # 返回空串，调用方据此跳过打印
    fps = interval_frames / elapsed  # 窗口内的平均帧率
    ms = lambda s: s / interval_frames * 1000.0  # 把窗口总秒数换算成单帧毫秒
    return (f'[{TASK}] {title}: 帧数={interval_frames} 运行={elapsed:.2f}s '  # 输出表头与窗口概况
            f'平均帧率={fps:.1f}fps | '  # 输出平均帧率
            f'采集={ms(d_capture):.2f}ms/帧 '  # 输出采集单帧耗时
            f'检测={ms(d_detect):.2f}ms/帧 '  # 输出检测单帧耗时
            f'显示={ms(d_display):.2f}ms/帧')  # 输出显示单帧耗时


def open_camera(device, index):  # 打开下视相机，优先 JPU 硬解，失败回退 cv2 软解
    if CAM.get('hardware_decode', False):  # 配置要求走硬件解码
        dev = device if device is not None else '/dev/video%d' % (index if index is not None else 0)  # 确定设备节点路径
        try:  # 硬解依赖可能缺失，需捕获
            from hw_camera import HwMjpgCamera  # 板端 JPU 硬解相机封装
            cam = HwMjpgCamera(dev, W, H, int(CAM.get('fps', 200)))  # 按目标宽高与帧率打开相机
            if cam.isOpened():  # 打开成功才使用硬解路径
                print(f'[*] [{TASK}] 相机 {dev}: 启用 JPU 硬件解码', flush=True)  # 提示已走硬解
                return cam  # 返回硬解相机对象
            print(f'[警告] [{TASK}] 相机 {dev}: JPU 硬件解码不可用, 回退 cv2 软解', flush=True)  # 提示降级
        except Exception as exc:  # 导入或初始化异常一律降级
            print(f'[警告] [{TASK}] 相机硬件解码初始化异常 {exc!r}, 回退 cv2 软解', flush=True)  # 打印降级原因
    src = device if device is not None else index  # cv2 接受设备路径或索引
    cap = cv2.VideoCapture(src)  # 用 OpenCV 打开相机
    if not cap.isOpened():  # 打开失败
        return None  # 交由上层决定是否退出
    fmt = CAM.get('format')  # 配置的像素格式 fourcc，如 MJPG
    if fmt:  # 指定格式才设置，避免覆盖驱动默认
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))  # 强制使用指定编码
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)  # 设置采集宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)  # 设置采集高度
    fps_req = CAM.get('fps')  # 请求帧率
    if fps_req:  # 配置了才下发
        cap.set(cv2.CAP_PROP_FPS, int(fps_req))  # 设置采集帧率
    try:  # 该属性在部分 OpenCV 版本不存在
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)  # 读帧超时 1 秒，防止掉线后卡死
    except cv2.error:  # 旧版本不支持该属性
        pass  # 忽略，继续用默认行为
    return cap  # 返回可用的相机对象


def producer(cap, q, max_frames, stop, n_workers, stats, enable_timing,  # 采集线程：读帧入队并周期写统计
             log_writer, stats_w, prefer_nv12=False, want_bgr=True,  # 日志/统计写端与数据格式偏好
             flow_share_w=None):  # 光流共享写端，未开启时为 None
    """采集线程：读帧 -> 入队；周期性写 stats；若 flow_share_w 存在则同时写光流共享帧。

    只读真实相机; 不再支持虚拟帧(2026-09-21 移除 --virtual/make_frame)。
    """
    fid = 0  # 帧序号，随每帧递增
    t_win = time.perf_counter()  # 当前统计窗口的起始时刻
    win_frames = 0  # 当前窗口内已采集的帧数
    fail_cnt = 0  # 连续读帧失败计数
    READ_FAIL_LIMIT = 5  # 连续失败上限，超过判定相机掉线
    try:  # 用 finally 保证退出时唤醒 worker
        while not stop.is_set():  # 未收到停止信号就持续采集
            if max_frames and fid >= max_frames:  # 达到指定帧数即结束，0 表示不限
                break  # 跳出采集循环
            t0 = time.perf_counter()  # 记录本次采集开始时刻
            nv12 = None  # 默认未取得 NV12 原始帧
            if prefer_nv12:  # 硬解相机走 NV12 通道，省一次转换
                if want_bgr:  # 同时需要 BGR 与 NV12
                    ok, frame, nv12 = cap.read_both()  # 一次读出 BGR 与 NV12
                else:  # 只要 NV12
                    ok, nv12 = cap.grab_raw_nv12()  # 只取 NV12 原始帧
                    frame = None  # 无 BGR 帧
                if not ok or nv12 is None:  # 读帧失败或未取到数据
                    fail_cnt += 1  # 连续失败计数加一
                    if log_writer is not None:  # 开了日志才记录
                        log_writer.write(fid, [], status='no_data')  # 记一条无数据日志
                    if fail_cnt >= READ_FAIL_LIMIT:  # 连续失败达到上限
                        print(f'[{TASK}] 相机掉线/无数据，本进程退出', flush=True)  # 提示相机异常
                        stop.set()  # 通知所有线程一起退出
                        break  # 结束采集
                    time.sleep(0.1)  # 短暂等待后重试，避免空转烧 CPU
                    fid += 1  # 帧号继续推进
                    continue  # 进入下一轮采集
                fail_cnt = 0  # 成功则清零失败计数
            else:  # 普通 cv2 相机，直接读 BGR
                ok, frame = cap.read()  # 读一帧
                if not ok or frame is None:  # 读帧失败
                    fail_cnt += 1  # 连续失败计数加一
                    if log_writer is not None:  # 开了日志才记录
                        log_writer.write(fid, [], status='no_data')  # 记一条无数据日志
                    if fail_cnt >= READ_FAIL_LIMIT:  # 连续失败达到上限
                        print(f'[{TASK}] 相机掉线/无数据，本进程退出', flush=True)  # 提示相机异常
                        stop.set()  # 通知所有线程退出
                        break  # 结束采集
                    time.sleep(0.1)  # 等待后重试
                    fid += 1  # 帧号继续推进
                    continue  # 进入下一轮
                fail_cnt = 0  # 成功则清零

            # ---- 下视相机物理装反：在采集源头把原始帧转 180°，保证下游一致 ----
            # 必须在写光流 / 入队 / 检测之前完成，否则显示、检测坐标、光流三者会出现朝向不一致。
            # frame(BGR) 与 nv12 必须同时旋转，且旋转后二者仍保持像素对齐，检测 nv12 预处理才正确。
            if ROTATE_180:  # 仅当配置开启时执行，正常相机零开销
                if frame is not None:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)  # BGR 帧旋转 180°
                if nv12 is not None:
                    nv12 = cv2.rotate(nv12, cv2.ROTATE_180)  # NV12 原始帧同步旋转，与 BGR 保持一致

            t1 = time.perf_counter()  # 记录采集结束时刻

            # ---- 写光流共享帧（NV12 优先）----
            if flow_share_w is not None:  # 仅开启光流共享时才写
                try:  # 共享写失败不能影响主流程
                    flow_share_w.write(nv12 if nv12 is not None else frame)  # 优先写原始 NV12，否则退用 BGR
                except Exception:  # 写共享内存异常
                    pass  # 忽略，继续采集

            if enable_timing:  # 开启计时才统计采集耗时
                stats.add('capture', t1 - t0)  # 记录本次采集耗时
            try:  # 入队可能因队列满超时
                q.put((fid, frame, nv12), timeout=1.0)  # 帧入队给检测线程，超时 1 秒防死锁
            except queue.Full:  # 队列满说明下游跟不上，直接丢帧
                if log_writer is not None:  # 开了日志才记录
                    log_writer.write(fid, [], status='dropped')  # 记一条丢帧日志
            fid += 1  # 帧号加一
            win_frames += 1  # 窗口帧数加一

            # ---- 周期性写 stats ----
            if win_frames >= TIMING_INTERVAL:  # 达到统计间隔就输出一次
                now = time.perf_counter()  # 当前时刻
                elapsed = now - t_win  # 本窗口实际运行时长
                fps_val = win_frames / max(elapsed, 1e-9)  # 窗口平均帧率，防除零
                d_c, d_d, d_dp, n_c, n_d, n_dp = stats.delta_window()  # 取三个阶段的耗时与帧数增量
                cap_ms = (d_c / n_c * 1000.0) if n_c else 0.0  # 采集单帧毫秒，无数据记 0
                det_ms = (d_d / n_d * 1000.0) if n_d else 0.0  # 检测单帧毫秒，无数据记 0
                if stats_w is not None:  # 统计共享写端存在才写
                    try:  # 写共享内存可能失败
                        stats_w.write({  # 写入统计 JSON 共享内存
                            'fps': round(fps_val, 2),  # 当前帧率
                            'capture_ms': round(cap_ms, 2),  # 采集单帧耗时
                            'detect_ms': round(det_ms, 2),  # 检测单帧耗时
                            'frames': fid,  # 已采集总帧数
                            'ts': time.time(),  # 写入时的时间戳
                        })
                    except Exception:  # 写失败不影响运行
                        pass  # 忽略
                if enable_timing:  # 开启计时才打印
                    if SIMPLE_TIMING:  # 简化模式只报帧率
                        print(f'[{TASK}] 简化时间统计@{fid}帧: 帧数={win_frames} '  # 打印帧数
                              f'运行={elapsed:.2f}s 平均帧率={fps_val:.1f}fps', flush=True)  # 打印耗时与帧率
                    else:  # 完整模式分阶段输出
                        print(format_timing(elapsed, win_frames, d_c, d_d, d_dp,  # 生成完整统计行
                                            f'时间统计@{fid}帧'), flush=True)  # 带上当前帧号标题
                stats.mark_window()  # 重置窗口基线
                t_win = now  # 新窗口从当前时刻起算
                win_frames = 0  # 窗口帧数清零

            if LOOP_SLEEP > 0:  # 配置了额外休眠才限流
                time.sleep(LOOP_SLEEP)  # 降低采集速率，给下游留出时间
    finally:  # 无论正常结束还是异常退出都要执行
        for _ in range(n_workers):  # 每个 worker 发一个毒丸
            q.put(None)  # None 作为结束信号


def worker(wid, q, disp_q, show, stop, stats, enable_timing, log_writer,  # 检测线程：推理并写共享内存
           frame_w, det_w):  # 帧共享写端与检测结果共享写端
    """检测 worker：检测 -> 写帧/检测结果到共享内存。"""
    detector = YoloDetector(YOLO_CFG) if YOLO_ENABLED else None  # 每线程独立实例化检测器，关闭时为 None
    idx = 0  # 打印用的全局检测序号
    while not stop.is_set():  # 未收到停止信号就持续取帧
        try:  # 带超时取帧
            item = q.get(timeout=0.5)  # 超时 0.5 秒，便于及时响应停止信号
        except queue.Empty:  # 队列暂时为空
            continue  # 重新检查停止信号后继续等
        if item is None:  # 收到毒丸
            break  # 结束本 worker
        fid, frame, nv12 = item  # 拆出帧号、BGR 帧与 NV12 帧
        status = 'no_yolo' if detector is None else 'done'  # 未启用推理时日志标记为 no_yolo
        if enable_timing:  # 需要计时则包一层计时
            t0 = time.perf_counter()  # 推理开始时刻
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []  # 执行检测，未启用则返回空
            t1 = time.perf_counter()  # 推理结束时刻
            stats.add('detect', t1 - t0)  # 记录本次检测耗时
        else:  # 不计时，直接推理
            dets = detector.detect(frame, nv12=nv12) if detector is not None else []  # 执行检测

        if log_writer is not None:  # 开了日志才写
            log_writer.write(fid, dets, status=status)  # 记录本帧检测结果

        # ---- 准备可视帧（BGR）----
        if frame is None and nv12 is not None:  # 只有 NV12 数据时需转成 BGR
            frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)  # NV12 转 BGR
        if frame is not None:  # 有可视帧才做后续写共享与显示
            vis = draw_detections(frame, dets) if dets else frame  # 有检测结果才叠加绘制，否则用原帧

            # ---- 写帧到共享内存 ----
            if frame_w is not None:  # 帧共享写端存在才写
                ok, buf = cv2.imencode(  # 编码成 JPEG，减小共享内存占用
                    '.jpg', vis,  # 目标格式与待编码图像
                    [cv2.IMWRITE_JPEG_QUALITY, MC.WEB_MJPEG_QUALITY])  # 使用网页推流配置的画质
                if ok:  # 编码成功才写
                    try:  # 写共享内存可能异常
                        frame_w.write(buf.tobytes())  # 写入帧共享内存
                    except Exception:  # 写失败忽略
                        pass  # 继续处理下一帧

            # ---- 写检测结果 JSON ----
            if det_w is not None:  # 检测共享写端存在才写
                try:  # 写共享内存可能异常
                    det_w.write({  # 写入检测结果 JSON
                        'frame': fid,  # 对应帧号
                        'ts': time.time(),  # 写入时间戳
                        'dets': dets,  # 本帧检测列表
                    })
                except Exception:  # 写失败忽略
                    pass  # 继续处理

            # ---- imshow ----
            if show:  # 需要显示才投递
                t2 = time.perf_counter()  # 投递开始时刻
                try:  # 显示队列容量很小
                    disp_q.put_nowait(vis)  # 非阻塞投递，避免拖慢检测
                except queue.Full:  # 队列满则丢弃这一帧画面
                    pass  # 显示丢帧可接受
                t3 = time.perf_counter()  # 投递结束时刻
                if enable_timing:  # 开启计时才统计
                    stats.add('display', t3 - t2)  # 记录显示投递耗时

        for d in dets:  # 逐条输出检测结果
            print(f'[{TASK}] ' + format_detection(d, MARK_POINT, idx), flush=True)  # 打印编号、类别与相对偏移
            idx += 1  # 检测序号递增
    q.task_done()  # 标记本线程任务完成


def display_loop(disp_q, stop):  # 显示线程：取帧并弹窗显示
    win = TASK  # 窗口名直接用任务名
    try:  # 无图形环境时 imshow 会抛错
        while not stop.is_set():  # 未停止就持续显示
            try:  # 短超时取帧，保持响应
                vis = disp_q.get(timeout=0.1)  # 取一帧待显示图像
            except queue.Empty:  # 暂无新帧
                continue  # 继续等待
            cv2.imshow(win, vis)  # 显示图像
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按下 q 键退出
                stop.set()  # 通知其它线程停止
                break  # 结束显示循环
    except cv2.error:  # 无显示环境或窗口操作失败
        pass  # 忽略，安静退出
    finally:  # 收尾释放窗口
        try:  # 窗口可能已被销毁
            cv2.destroyWindow(win)  # 关闭窗口释放资源
        except cv2.error:  # 销毁失败
            pass  # 忽略


def main():  # 下视任务主入口
    ap = argparse.ArgumentParser(description='bottom 任务（真实/虚拟相机 + YOLO）')  # 构造参数解析器
    ap.add_argument('--device', type=str, default=None)  # 相机设备路径，优先于配置
    ap.add_argument('--index', type=int, default=None)  # 相机索引，设备路径为空时生效
    ap.add_argument('--no-show', action='store_true')  # 关闭窗口显示
    ap.add_argument('--workers', type=int, default=N_WORKERS)  # 检测线程数
    ap.add_argument('--frames', type=int, default=0)  # 最多处理帧数，0 表示不限
    ap.add_argument('--timing', action='store_true')  # 强制开启耗时统计
    ap.add_argument('--no-timing', action='store_true')  # 强制关闭耗时统计
    ap.add_argument('--log', action='store_true')  # 强制开启日志
    ap.add_argument('--no-log', action='store_true')  # 强制关闭日志
    args = ap.parse_args()  # 解析命令行参数

    enable_timing = TIMING  # 默认取配置中的统计开关
    if args.timing:  # 命令行显式开启
        enable_timing = True  # 覆盖配置
    if args.no_timing:  # 命令行显式关闭，优先级最高
        enable_timing = False  # 覆盖配置
    enable_log = LOG_ENABLED  # 默认取配置中的日志开关
    if args.log:  # 命令行显式开启
        enable_log = True  # 覆盖配置
    if args.no_log:  # 命令行显式关闭
        enable_log = False  # 覆盖配置

    if not CAM_ENABLED:  # 相机被总开关关闭
        print(f'[{TASK}] ENABLE_BOTTOM_CAM=False，跳过真实相机任务')  # 提示跳过
        return 0  # 正常退出，不视为错误

    show = SHOW and not args.no_show  # 配置开启且命令行未禁用才显示

    stats = StageStats(simple=SIMPLE_TIMING)  # 创建统计对象
    stats.mark_window()  # 建立首个统计窗口基线
    t_start = time.perf_counter()  # 记录进程启动时刻

    log_writer = None  # 默认不写日志
    if enable_log:  # 开启日志才创建
        log_writer = FrameLog(os.path.join(LOG_DIR, f'{TASK}.log'), t_start)  # 日志文件为 logs/bottom.log

    # ---- 共享内存写端 ----
    print(f'[*] [{TASK}] 初始化共享内存...', flush=True)  # 提示开始初始化
    if os.environ.get('GRDK_STUB_MODEL') == '1':  # 测试接缝：置 1 跳过 BPU 模型、改用桩检测器。仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
        class _StubDet:  # 假检测器：吐居中的红球框
            def __init__(self, *_a, **_k):
                self._n = 0
            def detect(self, frame, nv12=None):
                self._n += 1
                cx, cy = W // 2, H // 2  # 板端 cv2 4.11 的 rectangle 只认 int 点，必须取整
                r = min(40 + (self._n % 150) * 5 // 10, W // 2 - 10)
                r = max(1, int(r))
                bbox = (int(cx - r), int(cy - r), int(cx + r), int(cy + r))
                return [{'bbox': bbox, 'center': (int(cx), int(cy)),
                         'label': 'red-ball', 'score': 0.88}]
        globals()["YoloDetector"] = _StubDet  # 必须写全局：worker() 读的是模块级名字
    frame_w = ShmFrameWriter(MC.SHM_FRAME_BOTTOM, W, H)  # 帧共享内存，按采集宽高创建
    det_w   = ShmJsonWriter(MC.SHM_DET_BOTTOM)  # 检测结果 JSON 共享内存
    stats_w = ShmJsonWriter(MC.SHM_STATS_BOTTOM)  # 统计 JSON 共享内存
    print(f'[*] [{TASK}] 帧共享 : {MC.SHM_FRAME_BOTTOM}', flush=True)  # 打印帧共享路径
    print(f'[*] [{TASK}] 检测共享: {MC.SHM_DET_BOTTOM}', flush=True)  # 打印检测共享路径
    print(f'[*] [{TASK}] 统计共享: {MC.SHM_STATS_BOTTOM}', flush=True)  # 打印统计共享路径

    # ---- 光流共享帧 ----
    # [2026-10-04 光流停用] 双保险：即使配置里 ENABLE_FLOW_SHARE_BOTTOM 被改回 True，
    #   这里也强制不建立光流共享写端（用户要求先停掉光流共享）。
    #   恢复办法：把 ENABLE_FLOW_SHARE_BOTTOM 改回 True，并把下方 FLOW_SHARE_DISABLED 置 False。
    #   ⚠ 与图像回传无关：/cam2 走的是 SHM_FRAME_BOTTOM(JPEG) 写端，仍照常工作。
    FLOW_SHARE_DISABLED = True  # 硬开关：True = 彻底不共享光流帧
    flow_share_w = None  # 默认不共享光流帧
    if FLOW_SHARE_DISABLED:  # 光流共享已停用
        print(f'[*] [{TASK}] 光流共享已停用 (不写 {MC.SHM_FLOW_BOTTOM})', flush=True)  # 明确提示已停用
    elif FLOW_SHARE_ON:  # 配置开启才初始化
        try:  # 光流模块可能不存在或创建失败
            from flow_share import FlowShareWriter  # 光流共享写端
            flow_share_w = FlowShareWriter('bottom', W, H, fmt=0)  # fmt=0 表示共享 NV12 格式
            print(f'[*] [{TASK}] 光流共享: {MC.SHM_FLOW_BOTTOM} (NV12)', flush=True)  # 打印光流共享路径
        except Exception as exc:  # 初始化失败不阻断主任务
            print(f'[警告] [{TASK}] 光流共享初始化失败: {exc!r}', flush=True)  # 打印失败原因

    device = args.device if args.device is not None else CAM.get('device')  # 命令行优先，否则用配置设备
    index = args.index if args.index is not None else CAM.get('index')  # 命令行优先，否则用配置索引
    cap = open_camera(device, index)  # 打开相机
    if cap is None:  # 打开失败
        print(f'[{TASK}] 真实相机打开失败: device={device} index={index}，进程退出', flush=True)  # 打印失败细节
        frame_w.close(); det_w.close(); stats_w.close()  # 释放已创建的共享内存后退出
        return 1  # 非正常退出

    src_is_hw = cap is not None and hasattr(cap, 'grab_raw_nv12') and hasattr(cap, 'read_both')  # 判断是否硬解相机
    prefer_nv12 = bool(src_is_hw and PRE_MODE in ('auto', 'nv12'))  # 硬解且模式允许才走 NV12 通道
    want_bgr = True   # 需要写共享内存

    q = queue.Queue(maxsize=Q_SIZE)  # 采集到检测的帧队列
    disp_q = queue.Queue(maxsize=2)  # 显示队列，只留最新两帧
    stop = threading.Event()  # 全局停止信号

    threads = [threading.Thread(  # 第一个线程固定为采集线程
        target=producer,  # 线程入口函数
        args=(cap, q, args.frames, stop, args.workers,  # 相机、队列、限帧、停止信号与 worker 数
              stats, enable_timing, log_writer, stats_w,  # 统计对象、开关与写端
              prefer_nv12, want_bgr, flow_share_w),  # 数据格式偏好与光流写端
        daemon=True)]  # 设为守护线程，主进程退出即结束
    for i in range(args.workers):  # 按数量启动检测线程
        threads.append(threading.Thread(  # 创建 worker 线程
            target=worker,  # 线程入口函数
            args=(i, q, disp_q, show, stop, stats, enable_timing, log_writer,  # worker 所需全部参数
                  frame_w, det_w),  # 帧与检测的共享写端
            daemon=True))  # 守护线程
    if show:  # 需要显示才起显示线程
        threads.append(threading.Thread(  # 创建显示线程
            target=display_loop, args=(disp_q, stop), daemon=True))  # 显示队列与停止信号

    for th in threads:  # 逐个启动
        th.start()  # 启动线程
    try:  # 等待线程结束，可能被 Ctrl+C 打断
        for th in threads:  # 依次等待
            th.join()  # 阻塞直到线程结束
    except KeyboardInterrupt:  # 用户按下 Ctrl+C
        stop.set()  # 置停止信号，让各线程自行退出
    finally:  # 收尾释放资源
        if cap is not None:  # 相机存在则释放
            cap.release()  # 关闭相机
        if log_writer is not None:  # 日志存在则关闭
            log_writer.close()  # 关闭日志文件
        frame_w.close()  # 关闭帧共享内存
        det_w.close()  # 关闭检测共享内存
        stats_w.close()  # 关闭统计共享内存
        if enable_timing:  # 开启统计才打印汇总
            elapsed = time.perf_counter() - t_start  # 总运行时长
            total = max(stats.capture_frames, 1)  # 取帧数并防除零
            print(f'[{TASK}] 结束统计: 总运行={elapsed:.2f}s '  # 打印总时长
                  f'总帧数={stats.capture_frames} 总检测帧={stats.detect_frames} '  # 打印采集与检测帧数
                  f'平均帧率={total / elapsed:.1f}fps', flush=True)  # 打印整体平均帧率
    return 0  # 正常退出


if __name__ == '__main__':  # 仅在直接运行本文件时执行
    raise SystemExit(main())  # 用 main 的返回值作为进程退出码
