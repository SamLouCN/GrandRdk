#!/usr/bin/env python3                                   # Shebang：指定用环境中的 python3 解释器执行本脚本
# -*- coding: utf-8 -*-                                  # 编码声明：源码按 UTF-8 解析，保证中文注释与字符串不乱码
"""中位机入口（RDK S100 中间层控制程序）—— 上位机(PC UI) <-> 中位机 <-> 下位机(STM32)。

用法:
  python3 main.py --list                          # 列出候选串口(CH348 A~H 口)
  python3 main.py --stm32 sim --sim-telemetry      # 无硬件联调：合成遥测 + 验证模式切换
  python3 main.py                                  # 正式跑：config.SERIAL_PORT(CH348 F 口)
  python3 main.py --stm32 off                      # 只跑上位机侧，不碰下位机
  python3 main.py --mode auv                       # 强制以 AUV 启动（忽略模式记忆）
  python3 main.py --no-mode-persist                # 本次不记忆模式（总用 START_MODE）

模式：AUV / ROV，由上位机 $CMD 的 mode 字段切换（0=ROV 遥控，1=AUV 自主），
编排与安全见 mode_dispatcher.py；两模式各自的运行逻辑见 mode_rov.py / mode_auv.py。

启动模式：config.START_MODE（默认 ROV）；若 config.MODE_PERSIST=True，上位机最后一次
切换的模式会写入 config.MODE_STATE_PATH，下次启动自动沿用（例：本次以 AUV 退出，
下次启动即 AUV）。命令行 --mode rov|auv 强制覆盖，--no-mode-persist 临时关闭记忆。
"""
from __future__ import annotations                        # 延迟求值注解（PEP 563）：类型标注可写前向引用而不必加引号

import argparse                                           # 命令行参数解析（--stm32 / --mode / --list 等）
import glob                                               # 按通配符匹配设备文件（/dev/ttyUSB* 等）
import math                                               # 数学库：sim 模式用 sin() 生成周期性变化的深度
import os                                                 # 系统接口：路径拼接、目录/文件名处理
import queue                                              # 线程安全队列：上位机收包线程 -> 编排层
import signal                                             # 信号处理：注册 SIGTERM，被 pkill 时也能优雅退出
import socket                                             # 套接字：端口占用探测与 UDP/TCP 通信
import struct                                             # 二进制打包/解包：下位机帧与遥测字段
import sys                                                # 系统接口：sys.exit 返回进程退出码
import threading                                          # 线程与锁：日志互斥锁、后台线程
import time                                               # 时间：时间戳、节拍 sleep、计时
import types                                              # types.SimpleNamespace：构造 config 的只读视图对象

import config as cfg                                      # 项目配置模块（端口、波特率、模式、视频等参数）
import link_stm32 as S                                    # 下位机(STM32)链路：组帧/解析/串口收发
import link_pc as PC                                      # 上位机(PC)链路：UDP 收发指令与遥测
import video as VID                                       # 图像回传服务：相机采集 + HTTP 推流
from mode_dispatcher import Dispatcher                    # 编排核心：模式切换、急停安全、上下行调度

CH_LETTERS = "ABCDEFGH"                                   # CH348 八路串口的字母编号（A~H），用于打印口位映射

# config.py 里可能尚未定义的键（链路/编排需要）：缺失时用这里的默认值，并提示补进 config.py
CONFIG_DEFAULTS = {                                       # 兜底配置字典：config.py 里缺失的键用这里的值
    "PC_BIND_IP": "0.0.0.0",                              # 上位机 UDP 绑定地址：0.0.0.0 = 监听所有网卡
    "STM32_BAUD": 115200,                                 # 下位机串口波特率
    "STM32_POLL_HZ": 10.0,                                # 下位机遥测轮询频率（Hz）
    "TICK_HZ": 20.0,                                      # 编排层主循环节拍频率（Hz）
    "TEL_HZ": 10.0,                                       # 向上位机发送遥测的频率（Hz）
    "REPORT_S": 5.0,                                      # 周期状态汇报间隔（秒）
    "ESTOP_TIMEOUT_S": 0,                               # 下行静默多少秒后自动急停（秒，0=关闭）
    "ESTOP_LATCH": False,                                  # 急停是否锁存（锁存后需显式解除）
    "DEFAULT_MODE": 0,                                    # 默认模式编号：0=ROV 遥控
    "START_MODE": 0,                                      # 启动模式：0=ROV，1=AUV
    "MODE_PERSIST": True,                                 # 是否记忆上次退出时的模式
    "MODE_STATE_PATH": os.path.join(os.path.dirname(os.path.abspath(__file__)), "mode_state.json"),  # 模式记忆文件：脚本同目录下的 mode_state.json
    "MODE_FORCE_START": False,                            # 是否强制用 START_MODE（忽略模式记忆）
}                                                         # 字典定义结束


def make_logger(log_path="", quiet=False):                # 构造日志函数：返回 log(msg)，可同时输出终端与文件
    """终端 + 可选文件日志（多线程共用一个锁）。"""        # 文档字符串：本函数用途
    fh = open(log_path, "a", buffering=1) if log_path else None   # 给了日志路径则追加打开并行缓冲(1)，否则不写文件
    lock = threading.Lock()                               # 互斥锁：保证多线程写日志不交错

    def log(msg):                                         # 内部闭包：真正执行写日志的函数
        line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)   # 拼接 [时:分:秒] 时间前缀与消息内容
        with lock:                                        # 进入临界区（加锁）
            if not quiet:                                 # 非安静模式才输出到终端
                print(line, flush=True)                   # 立即刷新打印，避免缓冲造成延迟
            if fh is not None:                            # 若日志文件已打开
                fh.write(line + "\n")                     # 追加写入一行到日志文件

    return log                                            # 返回日志函数供其他模块调用


def make_sim_log_view(log, enabled):                      # 包装日志：sim 空跑时对 0x09 的 20Hz 重复行做折叠
    """sim 空跑时 ROV 的 0x09 会 20Hz 打印：对同一行做 1s 折叠，其余原样。"""   # 文档字符串：折叠策略说明
    if not enabled:                                       # 未启用折叠
        return log                                        # 直接返回原日志函数
    state = {}                                            # 折叠状态表：key -> (上次输出时间, 已折叠次数)
    lock = threading.Lock()                               # 互斥锁：保护 state 的并发访问

    def _log(msg):                                        # 内部闭包：带折叠逻辑的日志函数
        text = str(msg)                                   # 统一转成字符串处理
        key = text.split(")")[0] if text.startswith("[STM32·sim] TX(") else None   # 仅对 sim 的 TX 行取 "前缀)" 作折叠键，其他行为 None
        if key is None:                                   # 非待折叠的行
            log(text)                                     # 原样输出
            return                                        # 结束本次调用
        with lock:                                        # 加锁：读写折叠状态
            now = time.time()                             # 当前时间戳
            last_t, dup = state.get(key, (0.0, 0))        # 取出该键的上次输出时间与已折叠计数
            if now - last_t < 1.0:                        # 距上次输出不足 1 秒
                state[key] = (last_t, dup + 1)            # 只累加折叠计数，不输出
                return                                    # 本次不打印
            state[key] = (now, 0)                         # 超过 1 秒：刷新时间并清零计数
        if dup:                                           # 若上一条曾被折叠
            log("      ... 上一行重复 %d 次" % dup)      # 补打一行“重复 N 次”的提示
        log(text)                                         # 输出当前这一条

    return _log                                           # 返回折叠版日志函数


def parse_mode_arg(text):                                 # 解析命令行 --mode 的取值
    """命令行 --mode: rov/ROV/遥控/0 -> 0；auv/AUV/自主/1 -> 1；其他 -> None"""   # 文档字符串：取值映射规则
    s = str(text).strip().lower()                          # 去首尾空白并转小写，便于比较
    if s in ("rov", "0", "遥控", "remote"):                # ROV / 遥控的多种写法
        return 0                                           # 返回 0（ROV 遥控模式）
    if s in ("auv", "1", "自主", "auto"):                  # AUV / 自主的多种写法
        return 1                                           # 返回 1（AUV 自主模式）
    if s.isdigit():                                        # 其他纯数字写法
        return int(s)                                      # 按数字返回模式编号
    return None                                            # 非法取值返回 None


def build_cfg(overrides=None, warn=None):                 # 构造配置视图：config.py + 缺省值 + 命令行覆盖
    """config.py 的只读视图 + 缺失键默认值 + 命令行覆盖（不修改 config.py 本体）。"""   # 文档字符串
    view = types.SimpleNamespace()                        # 用命名空间对象承载配置项（点号属性访问）
    for name in dir(cfg):                                 # 遍历 config 模块的所有属性名
        if name.isupper():                                # 只取全大写常量（约定为配置项）
            setattr(view, name, getattr(cfg, name))       # 复制到视图对象上
    missing = [k for k in CONFIG_DEFAULTS if not hasattr(view, k)]   # 找出视图里缺失的兜底键
    for k in missing:                                     # 逐个补齐
        setattr(view, k, CONFIG_DEFAULTS[k])              # 用兜底默认值填充
    if missing and warn:                                  # 有缺失且提供了告警函数
        warn("[CFG] config.py 缺少 %s -> 本次用默认值（建议补进 config.py）" % ",".join(missing))   # 提示补充配置
    for k, v in (overrides or {}).items():                # 应用命令行覆盖项
        if v is not None:                                 # 仅覆盖非 None 的值（None 表示未指定）
            setattr(view, k, v)                           # 写入视图
    return view                                           # 返回最终生效的配置视图


def list_stm32_ports():                                   # 列出候选下位机串口
    """列出候选下位机串口，并标注 CH348 A~H 口对应关系。"""   # 文档字符串
    out = []                                              # 结果列表：元素为 (设备路径, 口位标注)
    paths = (sorted(glob.glob("/dev/ttyCH9344USB*"))      # 优先匹配 CH348 八路 USB 串口
             + sorted(glob.glob("/dev/ttyUSB*"))          # 再匹配通用 USB 转串口
             + sorted(glob.glob("/dev/ttyACM*")))         # 最后匹配 ACM/CDC 类串口
    for path in paths:                                    # 逐个设备处理
        letter = ""                                       # 默认无口位标注
        tail = path.rsplit("USB", 1)[-1]                  # 取 "USB" 之后的编号（用于判断 CH348 口位）
        if path.startswith("/dev/ttyCH9344USB") and tail.isdigit() and int(tail) < 8:   # 是 CH348 且编号在 0~7
            letter = " (CH348 %s 口)" % CH_LETTERS[int(tail)]   # 映射成 A~H 口标注
        out.append((path, letter))                        # 收集 (路径, 标注)
    return out                                            # 返回候选列表


class SimTelemetry(object):                               # 合成遥测发生器（--stm32 sim 时使用）
    """--stm32 sim 时的合成遥测：按真实下位机帧布局生成 0x0C 回帧，走同一条解析路径喂给编排层。"""   # 文档字符串

    def __init__(self, cfgv, dispatcher, log):            # 构造函数
        self.cfg = cfgv                                   # 保存配置视图
        self.dispatcher = dispatcher                      # 保存编排层：把遥测喂给它
        self.log = log                                    # 保存日志函数
        self._stop = threading.Event()                    # 停止事件：通知后台线程退出
        self._t0 = time.time()                            # 起始时间：用于生成随时间变化的仿真数据
        self._thread = None                               # 后台线程句柄，start() 时创建

    def start(self):                                      # 启动合成遥测线程
        hz = max(0.5, float(getattr(self.cfg, "STM32_POLL_HZ", 10.0)))   # 发送频率：至少 0.5Hz，缺省 10Hz
        self.log("[SIM] 合成遥测已启动 (%.1f Hz, 深度 40~60cm 正弦, 目标 50cm)" % hz)   # 打印启动信息
        self._thread = threading.Thread(target=self._loop, args=(1.0 / hz,), daemon=True)   # 创建守护线程，周期 1/hz 秒
        self._thread.start()                              # 启动线程

    def stop(self):                                       # 停止合成遥测
        self._stop.set()                                  # 置位停止事件，后台循环随即退出

    def frame(self):                                      # 生成一帧仿真遥测数据（二进制帧）
        t = time.time() - self._t0                        # 相对起始时刻的秒数
        depth_cm = 50.0 + 10.0 * math.sin(t / 5.0)        # 深度按正弦在 40~60cm 波动，目标 50cm
        data = struct.pack("<12h",                        # 小端打包 12 个 int16（角度/角速度单位 0.01）
                           0, 0, 0,                                              # 目标 Pitch/Yaw/Roll
                           int(1.5 * 100), int(0.8 * 100), int(-0.5 * 100),       # 实际姿态
                           int(0.3 * 100), int(0.1 * 100), int(-0.2 * 100),       # 角速度
                           int(0.05 * 100), int(0.02 * 100), int(9.80 * 100))     # 加速度
        data += struct.pack("<8h", *[int(100 * (i % 4)) for i in range(8)])   # 追加 8 个 int16：八路推进器/PWM 反馈
        data += struct.pack("<HH", int(50.0 * 100), int(depth_cm * 100))      # 追加 2 个 uint16：目标深度、实际深度（cm×100）
        return S.build_frame(S.FUNC_TELEMETRY, data)      # 按协议组帧返回（功能码 0x0C 遥测）

    def _loop(self, period):                              # 后台循环：周期性合成并投递遥测
        parser = S.FrameParser()                          # 复用与真实链路相同的帧解析器
        while not self._stop.is_set():                    # 未收到停止信号则持续循环
            for func, data in parser.feed(self.frame()):  # 把合成帧喂给解析器，取出 (功能码, 数据)
                if func != S.FUNC_TELEMETRY:              # 只关心遥测帧
                    continue                              # 其他功能码跳过
                tel = S.parse_telemetry(data)             # 解析成遥测结构
                if tel:                                   # 解析成功
                    self.dispatcher.on_stm32_telemetry(tel)   # 投递给编排层（与真实链路同一入口）
            time.sleep(period)                            # 按周期休眠，控制发送频率


def preflight_ports(cfgv, log):                           # 启动前端口占用探测（只探测，不抢端口）
    """启动前探测 8080/8081 是否已被占用（cmd_watch.py / relay.py 等）。只探测，不抢端口。"""   # 文档字符串
    busy = []                                             # 被占用的端口列表
    bind_ip = str(getattr(cfgv, "PC_BIND_IP", "0.0.0.0") or "0.0.0.0")   # 绑定地址，默认 0.0.0.0
    for name in ("CMD_PORT", "TELEM_PORT"):               # 依次检查指令端口与遥测端口
        port = int(getattr(cfgv, name, 0) or 0)           # 取端口号，非法值视为 0
        if port <= 0:                                     # 端口未配置
            continue                                      # 跳过该端口
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # 建 UDP 套接字（这两个端口都是 UDP）
        try:                                              # 尝试绑定
            s.bind((bind_ip, port))                       # 绑定成功说明端口空闲
        except OSError:                                   # 绑定失败说明已被占用
            busy.append("%s(%d)" % (name, port))          # 记录占用项
        finally:                                          # 无论成功失败
            s.close()                                     # 关闭探测套接字，释放端口
    # 图像回传的 HTTP TCP 端口（被占则上位机取流会失败；同样只探测不抢）
    vport = int(getattr(cfgv, "VIDEO_HTTP_PORT", 0) or 0) # 图像回传端口号
    if vport > 0:                                         # 配置了才检查
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # 建 TCP 套接字
        try:                                              # 尝试绑定
            s.bind((bind_ip, vport))                      # 绑定成功 = 端口空闲
        except OSError:                                   # 绑定失败 = 已被占用
            busy.append("VIDEO_HTTP_PORT(%d)" % vport)    # 记录占用
        finally:                                          # 收尾
            s.close()                                     # 关闭探测套接字
    if busy:                                              # 若存在被占用的端口
        log("[LINK] 警告: %s 已被占用 -> 上位机指令/遥测可能收不到；先停掉占用进程(如 cmd_watch.py / relay.py)"   # 告警文本（%s 为占用端口的占位符）
            % "、".join(busy))                            # 打印告警与处理建议
    return busy                                           # 返回占用列表


def build_runtime(cfgv, log, port_spec="sim", video=None):   # 装配三件套：上位机链路 + 下位机链路 + 编排核心
    """装配 上位机链路 + 下位机链路 + 编排核心（main 与自测共用）。"""   # 文档字符串
    rx_queue = queue.Queue()                              # 上位机收包队列：收包线程 -> 编排层
    pc = PC.PcLink(cfgv, log, rx_queue)                   # 创建上位机 UDP 链路对象
    stm32 = S.Stm32Link(cfgv, make_sim_log_view(log, port_spec == "sim"),   # 创建下位机链路；sim 时用折叠日志减少刷屏
                        port_spec=port_spec, on_telemetry=None)             # 指定串口来源与遥测回调（稍后回填）
    disp = Dispatcher(cfgv, pc, stm32, log, rx_queue=rx_queue, video=video) # 创建编排核心，注入两条链路与视频服务
    stm32.on_telemetry = disp.on_stm32_telemetry          # 回填遥测回调：下位机帧 -> 编排层
    return pc, stm32, disp                                # 返回三个已互相连好的对象


def main():                                               # 程序主入口：解析参数、装配、启动、阻塞等待退出
    ap = argparse.ArgumentParser(description="中位机 (RDK S100 中间层控制): 上位机 <-> 下位机")   # 创建命令行解析器并设置描述
    ap.add_argument("--stm32", default=getattr(cfg, "SERIAL_PORT", "/dev/ttyCH9344USB5"),   # 指定下位机串口，默认取 config.SERIAL_PORT
                    help="下位机串口: 设备路径 / 'sim'(空跑) / 'off'(禁用)")                 # 参数帮助文本
    ap.add_argument("--baud", type=int, default=getattr(cfg, "BAUD", 115200))               # 串口波特率，默认 config.BAUD
    ap.add_argument("--pc-bind", default=None, help="上位机 UDP 绑定地址（默认 config.PC_BIND_IP）")   # 上位机 UDP 绑定地址
    ap.add_argument("--pc-port", type=int, default=None, help="上位机指令端口（默认 8080）")            # 上位机指令接收端口
    ap.add_argument("--tel-port", type=int, default=None, help="遥测/PING 端口（默认 8081）")           # 遥测 / PING 端口
    ap.add_argument("--estop-timeout", type=float, default=None, help="下行静默自动急停秒数, 0=关闭")   # 下行静默自动急停时长
    ap.add_argument("--estop-no-latch", action="store_true", help="急停不锁存：收到新 $CMD 即解除")      # 急停不锁存开关
    ap.add_argument("--mode", default=None,                                                # 启动模式覆盖项
                    help="启动模式: rov/auv 或 0/1（覆盖模式记忆与 config.START_MODE）")    # 帮助：模式取值与优先级
    ap.add_argument("--no-mode-persist", action="store_true",                              # 关闭模式记忆开关
                    help="关闭模式记忆：每次启动都用 config.START_MODE")                    # 帮助：不记忆模式
    ap.add_argument("--mode-state", default=None,                                          # 模式记忆文件路径覆盖
                    help="模式记忆文件路径（默认 config.MODE_STATE_PATH）")                 # 帮助：记忆文件路径
    ap.add_argument("--sim-telemetry", action="store_true", help="--stm32 sim 时合成下位机遥测")   # 是否合成遥测
    ap.add_argument("--log", default="", help="追加写日志文件")                             # 日志文件路径
    ap.add_argument("--quiet", action="store_true", help="不打印到终端（仅写日志）")          # 安静模式
    ap.add_argument("--list", action="store_true", help="列出候选串口后退出")                 # 只列出串口后退出
    ap.add_argument("--no-video", action="store_true", help="不做图像回传（不占 :5000 / 不开相机）")   # 关闭图像回传
    ap.add_argument("--video-port", type=int, default=None, help="图像回传 HTTP 端口（默认 5000）")    # 图像回传端口
    ap.add_argument("--cam1", default=None, help="cam1 相机设备（默认 /dev/video0）")        # cam1 设备路径
    ap.add_argument("--cam2", default=None, help="cam2 相机设备（默认 /dev/video1，通常占位）")   # cam2 设备路径
    args = ap.parse_args()                                # 解析命令行参数到 args

    if args.list:                                         # --list：只列出候选串口
        print("候选串口（下位机 STM32）:")                 # 打印标题
        cands = list_stm32_ports()                        # 获取候选串口列表
        for path, letter in cands:                        # 逐个打印候选串口
            print("  %s%s" % (path, letter))              # 打印设备路径与 CH348 口位标注
        if not cands:                                     # 一个都没找到
            print("  (未发现; 无硬件时可用 --stm32 sim)")  # 提示可用 sim 空跑
        return 0                                          # 正常退出

    mode_override = None                                  # 命令行指定的启动模式（None = 未指定）
    if args.mode is not None:                             # 用户传了 --mode
        mode_override = parse_mode_arg(args.mode)         # 解析成 0 / 1
        if mode_override is None:                         # 解析失败（非法取值）
            print("--mode 取值非法: %r（可用 rov / auv / 0 / 1）" % (args.mode,))   # 打印错误提示
            return 2                                      # 返回参数错误码

    log = make_logger(args.log, args.quiet)               # 创建日志函数（终端 + 可选文件）
    overrides = {                                         # 命令行覆盖项字典（值为 None 表示不覆盖）
        "PC_BIND_IP": args.pc_bind,                       # 上位机 UDP 绑定地址
        "CMD_PORT": args.pc_port,                         # 上位机指令端口
        "TELEM_PORT": args.tel_port,                      # 遥测端口
        "STM32_BAUD": args.baud,                          # 下位机串口波特率
        "ESTOP_TIMEOUT_S": args.estop_timeout,            # 自动急停超时（0=关闭）
        "ESTOP_LATCH": (False if args.estop_no_latch else None),   # 急停锁存：传了 --estop-no-latch 则关闭
        "VIDEO_HTTP_PORT": args.video_port,               # 图像回传 HTTP 端口
        "START_MODE": mode_override,                      # 启动模式（命令行优先）
        "MODE_FORCE_START": (True if mode_override is not None else None),   # 指定了 --mode 则强制启动模式
        "MODE_PERSIST": (False if args.no_mode_persist else None),   # 关闭模式记忆
        "MODE_STATE_PATH": args.mode_state,               # 模式记忆文件路径
    }                                                     # 覆盖项字典结束
    cfgv = build_cfg(overrides, warn=log)                 # 合成最终生效的配置视图
    if args.cam1 or args.cam2:                            # 若命令行指定了相机设备
        paths = dict(getattr(cfgv, "VIDEO_PATHS", {}))    # 复制现有相机路径表（不改动原对象）
        if args.cam1:                                     # 指定了 cam1
            paths["cam1"] = args.cam1                     # 覆盖 cam1 设备路径
        if args.cam2:                                     # 指定了 cam2
            paths["cam2"] = args.cam2                     # 覆盖 cam2 设备路径
        cfgv.VIDEO_PATHS = paths                          # 写回配置视图
    vid = None if args.no_video else VID.VideoService(cfgv, log)   # 按需创建图像回传服务（--no-video 则不建）
    pc, stm32, disp = build_runtime(cfgv, log, port_spec=args.stm32, video=vid)   # 装配链路与编排核心

    sim_tel = None                                        # 合成遥测对象（默认不启用）
    if args.sim_telemetry:                                # 传了 --sim-telemetry
        if stm32.sim:                                     # 且下位机处于 sim 模式
            sim_tel = SimTelemetry(cfgv, disp, log)       # 创建合成遥测发生器
        else:                                             # 非 sim 模式
            log("[SIM] --sim-telemetry 仅在 --stm32 sim 时生效，已忽略")   # 提示该参数被忽略

    preflight_ports(cfgv, log)                            # 启动前探测端口占用（仅告警，不抢端口）
    disp.start()                                          # 启动编排核心（含各工作线程）
    if vid is not None:                                   # 若启用了图像回传
        vid.start()                                       # 启动图像回传服务
    log("[中位机] 已启动 | 上位机 UDP %d/%d | 下位机 %s | 初始模式 %s | 图像回传 %s" % (   # 打印启动横幅
        cfgv.CMD_PORT, cfgv.TELEM_PORT, stm32.snapshot()["port"], disp.mode().name,      # 端口 / 串口 / 模式名
        ("http :%d" % cfgv.VIDEO_HTTP_PORT) if vid is not None else "关"))               # 图像回传状态
    _src = {"forced": "命令行--mode", "saved": "模式记忆", "start": "config.START_MODE"}.get(   # 把模式来源代码翻译成中文
        getattr(disp, "mode_source", "start"), getattr(disp, "mode_source", "?"))        # 取来源，缺省显示 "?"
    log("[中位机] 启动模式来源 %s | 模式记忆 %s | 记忆文件 %s" % (   # 打印模式来源与记忆配置
        _src, "开" if disp.mode_persist else "关", disp.mode_state_path or "-"))   # 来源 / 开关 / 记忆文件路径
    if sim_tel:                                           # 若创建了合成遥测
        sim_tel.start()                                   # 启动合成遥测线程

    def _on_term(_signum, _frame):                        # SIGTERM 信号处理器（pkill / kill 触发）
        raise KeyboardInterrupt          # pkill(SIGTERM) 也走干净退出   # 抛 KeyboardInterrupt 复用统一退出流程

    try:                                                  # 注册信号处理（非主线程环境可能不支持）
        signal.signal(signal.SIGTERM, _on_term)           # 把 SIGTERM 绑定到上面的处理器
    except (ValueError, OSError):                         # 注册失败的两种异常
        pass                                              # 忽略异常，继续运行
    try:                                                  # 主循环
        while True:                                       # 常驻：真正的收发活在各自线程里
            time.sleep(0.5)                               # 轻量休眠，交出 CPU
    except KeyboardInterrupt:                             # 收到 Ctrl+C 或 SIGTERM
        log("[中位机] 退出中...")                          # 提示正在退出
    finally:                                              # 无论以何种方式退出都要清理
        if sim_tel:                                       # 若启用了合成遥测
            sim_tel.stop()                                # 停止合成遥测线程
        if vid is not None:                               # 若启用了图像回传
            vid.stop()                                    # 停止图像回传服务
        disp.stop()                                       # 停止编排核心与两条链路
    return 0                                              # 返回正常退出码


if __name__ == "__main__":                                # 仅当作为脚本直接运行时执行（被 import 时不执行）
    sys.exit(main())                                      # 执行 main() 并把返回值作为进程退出码
