# -*- coding: utf-8 -*-
"""中位机配置 —— S100 中间层控制程序

命名约定（本项目统一）：
    上位机 = PC UI（ROV控制站 v3.2）
    中位机 = 本程序（RDK S100）
    下位机 = STM32 运动控制层（XLB-V1.0-Servo）

链路：
    上位机 --UDP 8080 $CMD/$PID/$VID--> 中位机 --UART V2 二进制(CH348 F口)--> 下位机
    上位机 <--UDP 8081 $TEL/PONG------ 中位机 <--0x0C 48B 遥测--------------- 下位机
"""

import os

# ==================== 网络（与上位机） ====================
CMD_PORT = 8080          # 上行指令
TELEM_PORT = 8081        # 遥测上行 + PING/PONG
PC_IP = ""               # 留空 = 从下行帧自动学习；也可 --pc 写死

# ==================== 串口（与下位机） ====================
SERIAL_PORT = "/dev/ttyCH9344USB5"   # CH348 F 口（A~H 对应 USB0~7）
BAUD = 115200

# ==================== 节奏 ====================
POLL_HZ = 10.0           # 0x0C 遥测请求频率（固件 1:1 回帧）
TEL_HZ = 10.0            # $TEL 上行频率
REPORT_S = 5.0           # 状态汇报周期
MAIN_LOOP_S = 0.02       # 主循环步长
ESTOP_TIMEOUT = 1.0      # 下行失联多少秒自动急停（deadman），0=关闭

# ==================== 模式 ====================
# 上位机 $CMD 第 7 字段(mode) 的取值 → 中位机模式
MODE_ROV = 0             # 遥控模式
MODE_AUV = 1             # AUV 自主模式
MODE_NAMES = {MODE_ROV: "ROV(遥控)", MODE_AUV: "AUV(自主)"}

# ==================== 启动模式 / 模式记忆 ====================
# 程序启动时进入哪个模式：0=ROV 遥控, 1=AUV 自主
# 生效优先级：命令行 --mode（MODE_FORCE_START=True）> 模式记忆文件(若 MODE_PERSIST) > START_MODE
START_MODE = MODE_ROV

MODE_PERSIST = True       # True: 记住上位机最后一次切换的模式，下次启动自动沿用
MODE_STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "mode_state.json")   # 模式记忆文件（原子写：tmp + os.replace）
MODE_FORCE_START = False  # True: 忽略记忆文件，强制用 START_MODE（命令行 --mode 走这条）

# ==================== V2 协议常量（下位机） ====================
V2_HEAD = 0xCD
V2_TAIL = 0xDC
V2_MAX_FRAME = 64

FUNC_PID = 0x01
FUNC_TEST_THROTTLE = 0x02
FUNC_ANGLE = 0x03
FUNC_MODE = 0x04
FUNC_SAVE = 0x05
FUNC_READ_ANGLE = 0x08
FUNC_MOTION = 0x09        # 摇杆综合运动（11B DATA）
FUNC_DEPTH = 0x0A
FUNC_READ_DEPTH = 0x0B
FUNC_TELEM = 0x0C         # 遥测请求/返回

MODE_CMD_START = 0x00
MODE_CMD_ESTOP = 0x01
MODE_CMD_PREPROGRAM = 0x02
MODE_CMD_ROV = 0x03
MODE_CMD_TEST = 0x04

# ==================== 映射参数（ROV 模式） ====================
SURGE_FULL_SCALE = 127.0   # [-1,1] → int8 满量程（固件再 ×2）
YAW_RATE_DPS = 60.0        # yaw 摇杆满偏 → 航向角速率 (度/秒)
DEPTH_RATE_CMS = 20.0      # heave 摇杆满偏 → 深度速率 (cm/秒)
HEAVE_SIGN = 1             # +1: heave>0 = 上浮(深度减小)
YAW_MIRROR = True          # 抵消固件对 Yaw 的取负归一化
DEPTH_MAX_CM = 200.0       # 固件钳位上限（2.00 m）

# ==================== 量纲（需实测标定） ====================
MOTOR_SCALE = 1000.0       # V2 电机原值 → [-1,1]
DEPTH_SCALE = 10000.0      # V2 深度 raw(=cm×100) → 米（如需换其他标度只改这里）

# ==================== 编排层 / 链路（main.py + mode_dispatcher.py + link_*.py） ====================
# 编排层与链路读这些键；已有参数用别名指向，保持"单一来源"，改上面一处即可。
PC_BIND_IP = "0.0.0.0"           # 上位机 UDP 绑定地址（0.0.0.0 = 全部网卡）
STM32_BAUD = BAUD                # 下位机波特率（link_stm32 读此键）
STM32_POLL_HZ = POLL_HZ          # 0x0C 遥测轮询频率（link_stm32 读此键）
TICK_HZ = 20.0                   # 模式 tick 频率（与上位机 $CMD 的 20Hz 对齐）
ESTOP_TIMEOUT_S = ESTOP_TIMEOUT  # 下行静默自动急停秒数（mode_dispatcher 读此键）
ESTOP_LATCH = True               # 急停后锁存（抑制 0x09）；解除需 $ESTOP,0# 或 --estop-no-latch
DEFAULT_MODE = MODE_ROV          # 启动时的初始模式

# ==================== 图像回传（main 内置 MJPEG，对接上位机 cv2.VideoCapture） ====================
# 上位机取流: http://<板端IP>:5000/cam1 | /cam2   （与 vp5.1 的 mjpeg_bridge 同一约定）
# 与 vp5.1 互斥: 两者都独占 /dev/video*，不要同时启动（vp5.1 是带 YOLO 检测框的重链路）
VIDEO_ENABLED_AT_START = True    # 启动即开图像回传；False = 等上位机 $VID,1#
VIDEO_HTTP_PORT = 5000           # MJPEG HTTP 端口（上位机 VideoCapture 目标）
VIDEO_BIND_IP = "0.0.0.0"        # 监听地址（0.0.0.0 = 全部网卡）
VIDEO_WIDTH = 1280               # 采集宽（/dev/video0 支持 MJPG 1280x720@30）
VIDEO_HEIGHT = 720               # 采集高
VIDEO_FPS = 15                   # 采集与推流帧率（越高越吃带宽；720p@15 约 0.7 MB/s）
VIDEO_QUALITY = 70               # JPEG 质量（0~100）
# 摄像头路径映射: 本板 /dev/video0 = USB UVC（可独立采集）；/dev/video1 为元数据节点，
# 打不开 -> 该路只发占位帧（不黑屏、不断流，便于上位机固定用 /cam1 /cam2 两路）。
VIDEO_PATHS = {"cam1": "/dev/video0", "cam2": "/dev/video1"}