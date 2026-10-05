# -*- coding: utf-8 -*-
"""中位机配置 —— S100 中间层控制程序                                          # 模块文档字符串: 文件用途说明

命名约定（本项目统一）:                                                  # 注释: 三层命名约定
    上位机 = PC UI（ROV控制站 v3.2）                                     # 注释: 上位机定义
    中位机 = 本程序（RDK S100）                                          # 注释: 中位机定义
    下位机 = STM32 运动控制层（XLB-V1.0-Servo）                          # 注释: 下位机定义

链路:                                                                       # 注释: 通信链路说明
    上位机 --UDP 8080 $CMD/$PID/$VID--> 中位机 --UART V2 二进制(CH348 F口)--> 下位机  # 注释: 上行+下行链路
    上位机 <--UDP 8081 $TEL/PONG------ 中位机 <--0x0C 48B 遥测--------------- 下位机  # 注释: 遥测+PING链路
"""                                                                          # 注释: 文档字符串结束

import os                                                                    # 标准库: 模式记忆文件路径拼接(os.path.join)

# ==================== 网络（与上位机） ====================                  # 注释: 分组标题
CMD_PORT = 8080          # 上行指令                                       # 注释: UDP 端口说明
TELEM_PORT = 8081        # 遥测上行 + PING/PONG                           # 注释: UDP 端口说明

# ==================== 串口（与下位机） ====================                  # 注释: 分组标题
SERIAL_PORT = "/dev/ttyCH9344USB5"   # CH348 F 口（A~H 对应 USB0~7）      # 注释: 串口节点
BAUD = 921600                                                              # 注释: 串口波特率

# ==================== 节奏 ====================                              # 注释: 分组标题
POLL_HZ = 10.0           # 0x0C 遥测请求频率（固件 1:1 回帧）              # 注释: 下行轮询频率
TEL_HZ = 10.0            # $TEL 上行频率                                   # 注释: 上行遥测频率
REPORT_S = 5.0           # 状态汇报周期                                     # 注释: 周期汇报间隔
ESTOP_TIMEOUT = 1.0      # 下行失联多少秒自动急停（deadman），0=关闭       # 注释: 自动急停延时

# ==================== 模式 ====================                              # 注释: 分组标题
# 上位机 $CMD 第 7 字段(mode) 的取值 → 中位机模式                          # 注释: 模式字段含义
# ⚠ MODE_IDLE(-1) **不是** $CMD 可下发的取值：它是"待命态"的内部 id，
#   上电后先停在 IDLE，直到上位机发来带 mode 的 $CMD（0 或 1）才进入对应模式。
MODE_IDLE = -1           # 待命模式（上电默认；等上位机 $CMD.mode 才进 ROV/AUV）  # 注释: 待命模式编号
MODE_ROV = 0             # 遥控模式                                        # 注释: ROV 模式编号
MODE_AUV = 1             # AUV 自主模式                                    # 注释: AUV 模式编号

# ==================== 启动模式 / 模式记忆 ====================              # 注释: 分组标题
# [2026-10-04 变更] 启动不再自动进入 ROV/AUV：默认停在 MODE_IDLE 待命态，
#   等上位机按下「有线遥控 / 自主航行」按钮、下发带 mode 的 $CMD 后再进入。
#   （旧行为是 START_MODE = MODE_ROV，上电即进有线 ROV。）
# 程序启动时进入哪个模式：-1=IDLE 待命, 0=ROV 遥控, 1=AUV 自主             # 注释: 默认模式
# 生效优先级：命令行 --mode（MODE_FORCE_START=True）> 模式记忆文件(若 MODE_PERSIST) > START_MODE  # 注释: 优先级
START_MODE = MODE_IDLE                                                    # 注释: 启动默认模式（待命，等上位机指令）

MODE_PERSIST = False       # True: 记住上位机最后一次切换的模式，下次启动自动沿用  # 注释: 是否持久化模式
# 模式记忆文件路径：兼容两种部署布局（2026-10-04 适配扁平部署）
#   A) 规范布局 <根>/config/to32_config.py  -> <根>/logs/to32_mode_state.json（原设计）
#   B) 扁平布局 <根>/to32_config.py         -> <根>/logs/to32_mode_state.json（就地）
# 判据：若本文件所在目录名不是 config，则不向上多跳一级。
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))          # 本文件所在目录
_ROOT = (os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "config"  # 规范布局：上一级即根
         else _THIS_DIR)                                        # 扁平布局：自身目录即根
MODE_STATE_PATH = os.path.join(_ROOT, "logs", "to32_mode_state.json")  # 模式记忆文件（原子写：tmp + os.replace）  # 注释: 模式记忆文件路径
MODE_FORCE_START = False  # True: 忽略记忆文件，强制用 START_MODE（命令行 --mode 走这条）  # 注释: 是否强制模式

# ==================== V2 协议常量 ====================                      # 注释: 分组标题
# 协议码（HEAD/TAIL/MAX_FRAME/FUNC_*/MODE_*）的**唯一来源是 link_stm32.py**。  # 注释: 协议常量唯一来源
# 2026-09-15 整理：此处原有与 link_stm32.py 重复的一份定义（V2_HEAD/V2_TAIL/V2_MAX_FRAME/  # 注释: 历史变更
# FUNC_*/MODE_CMD_*），经全项目引用扫描确认 0 引用 —— 已删除，避免"改一处不生效"。  # 注释: 删除原因
# 要改协议码请改 link_stm32.py（HEAD/TAIL/MAX_FRAME/FUNC_SET_PID…FUNC_TELEMETRY/MODE_*）。  # 注释: 维护指引

# ==================== 映射参数（ROV 模式） ====================              # 注释: 分组标题
# 2026-09-15：以下键**已成为唯一生效来源**（mode_rov.py 直接读）。          # 注释: 来源说明
# 此前它们是死参数（真实值硬编码在 mode_rov.py:32-35，改这里不生效）—— README_中位机.md U7。  # 注释: 历史问题
SURGE_FULL_SCALE = 127.0   # [-1,1] → int8 满量程（固件再 ×2）；frame_motion 读此键  # 注释: 满量程映射
YAW_RATE_DPS = 60.0        # yaw 摇杆满偏 → 航向角速率 (度/秒)            # 注释: yaw 角速率
DEPTH_RATE_CMS = 20.0      # heave 摇杆满偏 → 深度速率 (cm/秒)            # 注释: heave 速率
HEAVE_SIGN = 1             # +1: heave>0 = 上浮(深度减小)                  # 注释: heave 方向约定
YAW_MIRROR = True          # 抵消固件对 Yaw 的取负归一化                   # 注释: yaw 符号补偿
DEPTH_MAX_CM = 200.0       # 固件钳位上限（2.00 m）；frame_motion 读此键   # 注释: 深度钳位上限

# ==================== $PID（上位机 PID 调试面板） ====================      # 注释: 分组标题
# 2026-09-15 决策 D：**暂不打通**。上位机 UI 是 12 通道（文案"每台推进器一路 PID 速度环"），  # 注释: 暂不接通 PID
# 而固件 0x01 只认 ch 0~3 = Pitch/Yaw/Roll/Depth → 语义不对齐，先保持"丢弃 + 计数"。  # 注释: 语义不对齐
# 将来打通：PID_PASSTHRU 置 True，并实现 mode_rov.on_pid（frame_set_pid 已就绪）。  # 注释: 将来打通方法
PID_PASSTHRU = False                                                     # 注释: 是否下发 PID 帧
PID_MAX_CH = 3             # 固件 0x01 接受的通道上限（超出整帧丢弃）     # 注释: 通道上限
PID_VALUE_LIMIT = 327.67   # int16×100 的表示上限；超范围由 frame_set_pid 钳位  # 注释: 数值钳位

# ==================== 空闲静默策略（2026-09-15 需求） ====================  # 注释: 分组标题
# 需求：摇杆在中间（未动）时，中位机不要给下位机发送任何信号。            # 注释: 需求说明
# 实现：0x09 只在**载荷字节发生变化**时下发（逐字节比对上一帧）；           # 注释: 原始实现
#       回中瞬间载荷变化（surge/sway 归零）仍会发一帧把推力归零，之后保持静默。  # 注释: 原始细节
# 固件依据（源码确认）：Motion_Flag / Motion_Time 只在 JXZK_XLB_Protocol.c:760/761/786  # 注释: 固件依据
#       写入、全项目无读取 → 不存在"靠周期 0x09 维持"的看门狗 → 静默不会触发欠帧保护。  # 注释: 静默安全
# 2026-09-21 修订: 原"载荷字节比对"语义在"用户持续前推(surge/sway 数值恒定)"时会把 0x09  # 注释: 修订原因
#   全部静默, 导致固件收不到持续推力指令, S100→32 下行消失。改为"四轴回中"才静默:     # 注释: 修订策略
#   surge/sway/heave/yaw 输入侧全 0(真正的"摇杆回中")才不发, 其它情况(含恒定推力)每帧发。  # 注释: 修订策略
MOTION_SILENT_WHEN_IDLE = True            # True: 启用空闲静默(由下方 _AXES_ONLY 进一步限定); False: 每帧发  # 注释: 总开关
MOTION_SILENT_WHEN_IDLE_AXES_ONLY = True  # True: 仅 surge=sway=heave=yaw=0 才判为回中(推荐, 修复"持续推杆被吞")  # 注释: 判定方式
                                          # False: 旧字节比对(已知有 bug, 不推荐, 仅留作回退)  # 注释: 回退说明
POLL_IDLE_SILENT = False         # True: 空闲静默期间连 0x0C 遥测请求也不发（注意 $TEL 会断流）  # 注释: 是否同步停轮询
IDLE_SILENT_AFTER_S = 1.0        # 「空闲」判定：距最近一条上位机下行指令超过该秒数（仅 POLL_IDLE_SILENT 用）  # 注释: 空闲超时

# ==================== 量纲（需实测标定） ====================              # 注释: 分组标题
MOTOR_SCALE = 1000.0       # V2 电机原值 → [-1,1]                          # 注释: 电机量纲

# ==================== 编排层 / 链路（main.py + mode_dispatcher.py + link_*.py） ====================  # 注释: 分组标题
# 编排层与链路读这些键；已有参数用别名指向，保持"单一来源"，改上面一处即可。  # 注释: 设计原则
PC_BIND_IP = "0.0.0.0"           # 上位机 UDP 绑定地址（0.0.0.0 = 全部网卡）  # 注释: 绑定地址
STM32_BAUD = BAUD                # 下位机波特率（link_stm32 读此键）         # 注释: 串口波特率别名
STM32_POLL_HZ = POLL_HZ          # 0x0C 遥测轮询频率（link_stm32 读此键）   # 注释: 轮询频率别名
TICK_HZ = 20.0                   # 模式 tick 频率（与上位机 $CMD 的 20Hz 对齐）  # 注释: 主循环节拍
ESTOP_TIMEOUT_S = ESTOP_TIMEOUT  # 下行静默自动急停秒数（mode_dispatcher 读此键）  # 注释: 急停秒数别名
ESTOP_LATCH = True               # 急停后锁存（抑制 0x09）；解除需 $ESTOP,0# 或 --estop-no-latch  # 注释: 急停锁存
DEFAULT_MODE = MODE_IDLE         # 启动时的初始模式（与 START_MODE 一致：待命）  # 注释: 初始模式别名

# ==================== 图像回传（main 内置 MJPEG，对接上位机 cv2.VideoCapture） ====================  # 注释: 分组标题
# 上位机取流: http://<板端IP>:5000/cam1 | /cam2   （与 vp5.1 的 mjpeg_bridge 同一约定）  # 注释: 取流约定
# 与 vp5.1 互斥: 两者都独占摄像头设备节点，不要同时启动（vp5.1 是带 YOLO 检测框的重链路）  # 注释: 互斥说明
# VIDEO_ENABLED_AT_START = True    # 原值：启动即开图像回传                  # 注释: 历史原值
# 2026-09-21: To32 不再做图像回传（cam1/cam2 已停用），避开与 GrandRDK 视觉链路抢设备。  # 注释: 停用原因
# 注意: video.py 里是 getattr(cfg,"VIDEO_ENABLED_AT_START", True)，键缺失会回落 True，  # 注释: 回落陷阱
#       所以这里必须显式写 False，不能直接把这一行删掉/注释掉。            # 注释: 显式说明
VIDEO_ENABLED_AT_START = False   # 不主动开图像回传（即使跑 main.py 不带 --no-video 也不碰相机）  # 注释: 是否启动时开回传
VIDEO_HTTP_PORT = 5000           # MJPEG HTTP 端口（上位机 VideoCapture 目标）  # 注释: HTTP 端口
VIDEO_BIND_IP = "0.0.0.0"        # 监听地址（0.0.0.0 = 全部网卡）          # 注释: 绑定地址
VIDEO_WIDTH = 1280               # 采集宽（cam1 节点支持 MJPG 1280x720@30）  # 注释: 采集宽度
VIDEO_HEIGHT = 720               # 采集高                                    # 注释: 采集高度
VIDEO_FPS = 15                   # 采集与推流帧率（越高越吃带宽；720p@15 约 0.7 MB/s）  # 注释: 帧率
VIDEO_QUALITY = 70               # JPEG 质量（0~100）                        # 注释: 画质
# 摄像头路径映射（已停用）：To32 不再独占任何摄像头设备节点；VIDEO_PATHS 留作调试兜底  # 注释: 路径说明
# 旧值（2026-09-21 前）含两个 cam 的设备节点映射（cam1/cam2），节点号已废弃，仅作历史记录留痕  # 注释: 历史说明
# 2026-09-21: 停用 cam1/cam2 —— 本板图像服务由 web_server.py(:5000 /cam1 /cam2) 提供，  # 注释: 移交说明
# To32 不再独占任何摄像头设备节点（否则会抢 front.py 的 cam1 与 bottom.py 的 cam2）。  # 注释: 互斥风险
# 保留空字典而不是删 Key：命令行 --cam1/--cam2 仍可临时单独启用某路。     # 注释: 保留理由
VIDEO_PATHS = {}                  # 不挂任何相机 -> VideoService.sources 为空  # 注释: 相机路径映射

# ==================== AUV 自主任务（[AUV-MISSION 2026-09-26 新增]） ====================
# AUV 任务的**所有**参数统一放在同目录的 auv_config.py —— 那是唯一调参入口。
# 这里整体并入，业务代码继续写 `import to32_config as C` 再取 C.AUV_XXX，调用方式不用改。
# 要调 AUV 参数请改 auv_config.py，**不要在本文件里加 AUV_* 键**（避免改一处不生效）。
from auv_config import *  # noqa: F401,F403
