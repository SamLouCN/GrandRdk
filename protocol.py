"""
============================================================
  通讯协议定义  (PC 和 X5 共享此文件)
============================================================
文件: protocol.py v3.7
说明: PC端和X5端都 import 这个文件, 保证格式、端口、字段完全一致
============================================================

【整体架构 v3.2 (双链路)】
  有线模式(网线): PC (Windows) <--- UDP/HTTP ---> RDK X5/S100 <--- 串口 ---> STM32
                  所有数据(指令/遥测/视频/PID) 全部走网线
  无线模式(LoRa): PC (USB串口) --> 地面LoRa ~~~空中~~~ 机载LoRa --> 串口 --> STM32
                  仅遥控指令下行($CMD帧透传), 无遥测/无视频
                  运动指令沿用 $CMD；PID 单独采用参考二进制协议

【数据流向 (有线模式)】
  手柄  PC GUI  [UDP CMD_PORT]  X5  [UART]  STM32 (指令下行)
  STM32  [UART]  X5  [UDP TELEM_PORT]  PC GUI           (遥测上行)
  X5 摄像头  [HTTP VIDEO_PORT /cam1 /cam2]  PC GUI         (视频流)
  PC PID 调试 [UDP二进制透传 / 直连串口] STM32          (参考0x01帧)
  PC 视频开关 [UDP CMD_PORT]  X5 (板端拦截, 不透传STM32)    ($VID帧)

【数据流向 (无线模式)】
  手柄  PC GUI  [USB串口 9600]  地面LoRa ~~ 机载LoRa  [串口]  STM32 (指令下行, 5Hz)

【v3.2 变更说明】
  1) 新增 LoRa 无线链路配置段 (LORA_*)
  2) 新增视频传输开关帧: $VID,1#(开) / $VID,0#(关), 由X5板端解析拦截,
     STM32固件不需要认识该帧
  3) X5_IP 默认 192.168.127.10 (vp5.0 板端 eth1 网线直连, 沿用zxq联调值)

【v3.3 变更说明（2026-09-15，急停语义；板端决策 B）】
  上位机原来的"急停"只有一招：发一帧全零 $CMD —— 它与"摇杆回中"在报文上完全同形，
  板端无法区分，因此只能做到零推力、进不了 STANDBY。
  新增两帧显式急停（S100 板端拦截，不透传 STM32）：
    $ESTOP#\r\n    → 板端立刻下发 V2 0x04 0x01 进 STANDBY 并锁存（锁存期间抑制 0x09）
    $ESTOP,0#\r\n  → 解除锁存（现场自行恢复的唯一手段）
  触发：UI「急停」按钮 / 手柄断开；解除：UI「解除急停」按钮。
  注意：这两帧只走有线 UDP 8080；无线 LoRa 直连 STM32，STM32 不认识该帧，故无线下不下发。

【v3.7 变更说明】
  PID 改为 CD 0A 01 ... DC，P/I/D ×100 的 int16 小端。
  GUI 提供七项业务名称，默认编号0～6可改。参考原固件4～7为补偿回路，
  业务名称与编号需由现役固件匹配；UDP中位机必须原样透传二进制。

【v3.1 历史变更说明（旧PID文本现已移除）】
  1) 推进器数量 10 -> 12  (THRUSTER_COUNT = 12), 新增 thr10 / thr11
  2) 遥测帧尾新增 加速度 ax/ay/az 与 高度 alt, 追加在帧尾以保持向后兼容
     (旧帧/旧10字段帧仍可解析)
  3) 新增 PID 调试专用帧:  $PID,ch,p,i,d#   (ch = 通道号 0~11, p/i/d 为增益)
"""

import math
import struct
from decimal import Decimal, ROUND_HALF_UP

# ==================== 网络端口 ====================
CMD_PORT    = 8080
TELEM_PORT  = 8081
VIDEO_PORT  = 5000

# ==================== 高度计 CH348 (v3.2.1) ====================
# 板端 /userdata/USART/read_altimeter.py 以 UDP 单播到 PC, 帧格式:
#   $ALT,<通道A-H>,<距离mm>,<状态>#\r\n    状态: OK / EMPTY(未接) / ERR / DOWN
ALT_PORT    = 8082

# ==================== AUV 状态提示 (v3.5, 2026-10-05) ====================
# 板端 /userdata/GrandRDK/src/to32/move_test/auv_task/notify.py 以 UDP 单播到 PC, 帧格式:
#   $MSG,<ts>,<level>,<code>,<stage>,<text>#\r\n
#     level: INFO / WARN / ERROR      (上位机据此在终端染色: 白 / 黄 / 红)
#     code : STAGE / ABORT / TIMEOUT / SKIP / DEGRADE / BUDGET / TEST /
#            KILLED / PC_TAKEBACK / MODE / SERVO / VISION / HB
#     stage: 当前 AUV 阶段名 (DIVE / SEEK_BALL_F / PASS_GATE_1 / SURFACE ...)
#     text : 中文一句话提示 (内部逗号已转 ';', '#' 已剔除, 绝不破坏帧结构)
# ⚠ 与 $AUV(16 字段数值快照) 共用端口, 靠**帧头**区分: $AUV 帧这里解析返回 None, 互不影响。
# ⚠ 无缆验收时板端不会发(上位机地址是"收到帧才学到"的), 收不到属正常, 不是故障。
MSG_PORT    = 8085

# ==================== 默认IP ====================
X5_IP       = "192.168.127.10"  # vp5.0 联调: 板端 eth1 IP (网线直连, RDK Studio SSH 同链路)
PC_BIND     = "0.0.0.0"

# ==================== 串口(X5端) ====================
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

# ==================== 摄像头(X5端) ====================
CAM1_ID = 0
CAM2_ID = 2
CAM3_ID = -1     # v3.3: 第三路画面(预留); -1=未接入, 接好后填实际 /dev/videoX 序号
# 第三路画面(CAM3)取流地址 (v3.3): 默认指向板端 show_cam.py 自带的 MJPEG 推流服务
#   http://<板卡IP>:8084/stream     (别名 /cam3, 快照 /snapshot)
# 若改用板端 mjpeg_bridge(:5000) 的 /cam3, 改成 CAM3_PORT = 5000, CAM3_PATH = "/cam3"
CAM3_PORT = 8084
CAM3_PATH = "/stream"
CAM_W, CAM_H, CAM_FPS = 640, 480, 15
JPEG_QUALITY = 60

# ==================== LoRa 无线链路(PC端 v3.2) ====================
# 地面LoRa模块通过USB转串口接PC, 机载LoRa串口线直连STM32
# 透传模式: PC串口写什么, STM32就收到什么($CMD帧)
LORA_BAUD    = 9600     # LoRa透传模块默认波特率
LORA_SEND_HZ = 5        # 无线模式指令降频(20Hz*45字节≈7200bps会超出空速, 5Hz≈1800bps安全)

# ==================== 手柄(PC端) ====================
SEND_HZ  = 20
DEADZONE = 0.08
LED_STEP = 5
MAX_LED  = 100

# ==================== 推进器数量 ====================
# v3.1: 由 10 增加到 12
THRUSTER_COUNT = 12

# ==================== 参考协议 PID（v3.7） ====================
PID_CHANNELS = 7   # 七项控制参数；不再是推进器速度环
PID_REFERENCE_MAX_INDEX = 7  # 参考串口协议允许编号 0～7

# ==================== 协议格式定义 ====================
#
# 【上行指令 v3】 PC  X5  STM32   (文本帧, UTF-8)
# 格式: $CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#\r\n
# 示例: $CMD,0.50,-0.30,0.00,0.20,60,80,0,0,0#\r\n
# 字段:
#   surge  [-1.0, 1.0]  前进/后退
#   sway   [-1.0, 1.0]  左移/右移
#   heave  [-1.0, 1.0]  上浮/下潜
#   yaw    [-1.0, 1.0]  左转/右转
#   led1   [0, 100]     LED1亮度百分比
#   led2   [0, 100]     LED2亮度百分比
#   mode   0=ROV手动, 1=AUV自主
#   grab   0=无, 1=捡球, 2=抛球
#   store  0=无, 1=开始录制/存储, 2=停止录制/存储
#
# 【下行遥测 v3.1】 STM32  X5  PC   (文本帧, UTF-8)
# 格式: $TEL,roll,pitch,yaw,gx,gy,gz,depth,vx,vy,vz,
#        t_roll,t_pitch,t_yaw,t_gx,t_gy,t_gz,t_depth,t_vx,t_vy,t_vz,
#        batt_v,batt_a,batt_soc,batt_temp,cabin_temp,
#        thr0,thr1,...,thr9,thr10,thr11,ax,ay,az,alt#\r\n
# 字段索引:
#   0-9   实际: roll,pitch,yaw,gx,gy,gz,depth,vx,vy,vz   (角度/角速度/深度/线速度)
#   10-19 目标: t_roll~t_vz
#   20-24 电池/温度: batt_v,batt_a,batt_soc,batt_temp,cabin_temp
#   25-36 推进器油门: thr0~thr11  [-1.0, 1.0]
#   37-40 v3.1新增: ax,ay,az(加速度), alt(高度)
#
# 【PID 参数帧 v3.7】参考二进制格式，共11字节
# CD 0A 01 PID_INDEX P_L P_H I_L I_H D_L D_H DC
# P/I/D ×100，int16小端；编号0～7。


def build_cmd(surge, sway, heave, yaw, led1, led2, mode=0, grab=0, store=0):
    """构建上行指令字符串 v3"""
    return f"$CMD,{surge:.2f},{sway:.2f},{heave:.2f},{yaw:.2f},{led1},{led2},{mode},{grab},{store}#\r\n"


def build_emergency_stop():
    """构建紧急停机指令"""
    return "$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#\r\n"


def build_pid(ch, p, i, d):
    """参考串口协议 0x01：返回11字节 bytes，不再构造 $PID 文本。"""
    if not isinstance(ch, int) or isinstance(ch, bool) or not 0 <= ch <= PID_REFERENCE_MAX_INDEX:
        raise ValueError("PID 编号必须为0～7的整数")
    encoded = []
    for gain in (p, i, d):
        try:
            value = Decimal(str(gain))
            if not value.is_finite() or not Decimal("-327.68") <= value <= Decimal("327.67"):
                raise ValueError("PID 参数范围为-327.68～327.67")
            encoded.append(int((value * 100).quantize(Decimal("1"), rounding=ROUND_HALF_UP)))
        except (ArithmeticError, TypeError):
            raise ValueError("PID 参数必须是有效数值") from None
    return b"\xcd\x0a\x01" + struct.pack("<Bhhh", ch, *encoded) + b"\xdc"


def build_vid(on):
    """构建视频传输开关帧 v3.2 (仅X5板端解析, 不透传STM32)
    on: True=开始推流 / False=停止推流"""
    return f"$VID,{1 if on else 0}#\r\n"


def build_estop():
    """显式急停帧 v3.3（板端决策 B）: S100 收到即下发 V2 0x04 0x01 进 STANDBY 并锁存。

    与 build_emergency_stop() 的区别：后者是"全零 $CMD"，只保证零推力、进不了 STANDBY，
    且与摇杆回中报文同形。两者配合：先零杆位停推，再 $ESTOP# 真正进急停。
    """
    return "$ESTOP#\r\n"


def build_estop_release():
    """解除急停帧 v3.3（板端决策 B）: S100 收到即解除锁存并恢复正常控制。"""
    return "$ESTOP,0#\r\n"


def parse_vid(frame):
    """解析视频开关帧 v3.2, 返回 True/False/None(非VID帧)"""
    frame = frame.strip()
    if not (frame.startswith("$VID,") and frame.endswith("#")):
        return None
    try:
        return int(frame[5:-1]) == 1
    except ValueError:
        return None


def parse_cmd(frame):
    """解析上行指令, 返回dict或None"""
    frame = frame.strip()
    if not (frame.startswith("$CMD,") and frame.endswith("#")):
        return None
    try:
        parts = frame[5:-1].split(",")
        if len(parts) < 6:
            return None
        d = {
            "surge": float(parts[0]),
            "sway":  float(parts[1]),
            "heave": float(parts[2]),
            "yaw":   float(parts[3]),
            "led1":  int(parts[4]),
            "led2":  int(parts[5]),
        }
        if len(parts) > 6: d["mode"]  = int(parts[6])
        if len(parts) > 7: d["grab"]  = int(parts[7])
        if len(parts) > 8: d["store"] = int(parts[8])
        return d
    except (ValueError, IndexError):
        return None


def parse_pid(frame):
    """解析参考 0x01 PID 二进制帧；拒绝旧 $PID 文本。"""
    if not isinstance(frame, (bytes, bytearray)) or len(frame) != 11:
        return None
    if frame[:3] != b"\xcd\x0a\x01" or frame[-1] != 0xdc:
        return None
    ch, p, i, d = struct.unpack("<Bhhh", frame[3:-1])
    if ch > PID_REFERENCE_MAX_INDEX:
        return None
    return {"ch": ch, "p": p / 100, "i": i / 100, "d": d / 100}


class ReferenceTelemetryBuffer:
    """从串口字节流提取参考协议48B遥测应答，容忍拆包、粘包和调试文本。"""
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        self._buffer.extend(data)
        if len(self._buffer) > 4096:
            del self._buffer[:-4096]
        frames = []
        while self._buffer:
            start = self._buffer.find(b"\xcd")
            if start < 0:
                self._buffer.clear()
                break
            del self._buffer[:start]
            if len(self._buffer) < 2:
                break
            if self._buffer[1] != 0x2f:
                del self._buffer[0]
                continue
            if len(self._buffer) < 48:
                break
            if self._buffer[2] != 0x0c or self._buffer[47] != 0xdc:
                del self._buffer[0]
                continue
            frames.append(bytes(self._buffer[:48]))
            del self._buffer[:48]
        return frames


def parse_telemetry(frame):
    """解析下行遥测 v3.1, 返回dict或None
    兼容: 旧版10字段 / v3 10+10+5+10 / v3.1 10+10+5+10+6
    返回的键:
      实际: roll,pitch,yaw,gx,gy,gz,depth,vx,vy,vz
      目标: t_roll,t_pitch,t_yaw,t_gx,t_gy,t_gz,t_depth,t_vx,t_vy,t_vz
      电池: batt_v,batt_a,batt_soc,batt_temp,cabin_temp
      推进器: thrusters (list, 长度=THRUSTER_COUNT, [-1,1])
      v3.1新增: ax,ay,az(加速度), alt(高度); thr10,thr11(若存在)
      v3.6新增: fused_depth(板端融合深度,m), clearance(离底净空,m) —— 帧尾追加, 短帧无此键
    """
    frame = frame.strip()
    if not (frame.startswith("$TEL,") and frame.endswith("#")):
        return None
    try:
        p = frame[5:-1].split(",")
        if len(p) < 7:
            return None

        def f(i, default=0.0):
            try:
                return float(p[i])
            except (IndexError, ValueError):
                return default

        d = {
            "roll":  float(p[0]),
            "pitch": float(p[1]),
            "yaw":   float(p[2]),
            "gx":    float(p[3]),
            "gy":    float(p[4]),
            "gz":    float(p[5]),
            "depth": float(p[6]),
            "vx":    f(7),
            "vy":    f(8),
            "vz":    f(9),
        }
        if not all(math.isfinite(d[key]) for key in ("roll", "pitch", "yaw", "gx", "gy", "gz", "depth")):
            return None
        # 旧七字段帧没有真实速度，不能把兼容填充的 0 当作定位观测。
        try:
            d["position_velocity_valid"] = len(p) > 8 and all(
                math.isfinite(float(p[index])) for index in (7, 8))
        except ValueError:
            d["position_velocity_valid"] = False
        # v3 扩展字段: 目标值 (index 10-19)
        target_keys = ("t_roll", "t_pitch", "t_yaw", "t_gx", "t_gy", "t_gz",
                       "t_depth", "t_vx", "t_vy", "t_vz")
        for index, key in enumerate(target_keys, 10):
            if index >= len(p):
                break
            try:
                value = float(p[index])
                if math.isfinite(value):
                    d[key] = value
            except ValueError:
                pass  # 缺失/非法目标保留为空，不绘制假的零目标线。
        # v3 扩展字段: 电池/温度 (index 20-24)
        if len(p) > 20:
            d["batt_v"]     = f(20)
            d["batt_a"]     = f(21)
            d["batt_soc"]   = f(22)
            d["batt_temp"]  = f(23)
            d["cabin_temp"] = f(24)
        # v3 扩展字段: 推进器油门 (index 25 起), 12路连排 25~36, 补齐到 THRUSTER_COUNT 个
        thr_raw = []
        if len(p) > 25:
            n = min(THRUSTER_COUNT, len(p) - 25)
            thr_raw = [f(25 + i) for i in range(n)]
        d["thrusters"] = (thr_raw + [0.0] * THRUSTER_COUNT)[:THRUSTER_COUNT]
        # v3.1 新增: 加速度/高度 (追加在全部推进器之后, index 37-40)
        if len(p) > 37:
            d["ax"]  = f(37)
            d["ay"]  = f(38)
            d["az"]  = f(39)
            d["alt"] = f(40)
        # v3.6 新增: 板端融合深度/离底净空 (板端 $TEL 帧尾追加, index 41/42, 单位 m;
        # 板端 depth_kalman 未运行/数据超期时帧只有 41 字段, 这里自然缺省)
        if len(p) > 41:
            d["fused_depth"] = f(41)
        if len(p) > 42:
            d["clearance"] = f(42)
        return d
    except (ValueError, IndexError):
        return None


def parse_auv(frame):
    """解析 AUV 状态快照帧 $AUV,<16 个数值字段>#  (占位: 目前上位机不显示, 仅计数)

    保留它是为了让接收线程能把 $AUV 与 $MSG 区分开 —— 两者共用 MSG_PORT(8085),
    靠帧头分发; 这里返回 None 表示"不是我要处理的帧", 线程里据此忽略。
    """
    frame = frame.strip()
    if not (frame.startswith("$AUV,") and frame.endswith("#")):
        return None
    try:
        parts = frame[5:-1].split(",")
        return {"fields": parts}
    except (ValueError, IndexError):
        return None


def build_alt(ch, mm=None, status="OK"):
    """构建高度计帧 v3.2.1: $ALT,<通道>,<mm>,<状态>#  (板端推送 / PC 显示)"""
    v = "" if mm is None else str(int(mm))
    return f"$ALT,{ch},{v},{status}#\r\n"


def parse_alt(frame):
    """解析高度计帧, 返回 {'ch','mm'(int或None),'status'} 或 None"""
    frame = frame.strip()
    if not (frame.startswith("$ALT,") and frame.endswith("#")):
        return None
    try:
        parts = frame[5:-1].split(",")
        if len(parts) < 3:
            return None
        raw = parts[1].strip()
        return {
            "ch": parts[0].strip().upper(),
            "mm": int(raw) if raw else None,
            "status": parts[2].strip().upper(),
        }
    except (ValueError, IndexError):
        return None


def parse_msg(frame):
    """解析 AUV 状态提示帧 (v3.5): $MSG,<ts>,<level>,<code>,<stage>,<text>#

    返回 {'ts','level','code','stage','text'} 或 None(不是 $MSG 帧 / 字段数不足)。
    ⚠ 字段数**恒为 6**(含帧头 $MSG); text 为空时也是空串占位, 不会少字段。
    """
    frame = frame.strip()
    if not (frame.startswith("$MSG,") and frame.endswith("#")):
        return None
    try:
        # ⚠ 去掉 "$MSG,"(5 字符) 与帧尾 "#" 之后, 剩 ts/level/code/stage/text 共 5 段
        #   (说"6 字段"是把帧头 $MSG 算进去了, 别在这里按 6 取, 否则永远返回 None)
        parts = frame[5:-1].split(",")
        if len(parts) < 5:
            return None
        return {
            "ts": parts[0].strip(),
            "level": parts[1].strip().upper(),
            "code": parts[2].strip().upper(),
            "stage": parts[3].strip().upper(),
            "text": ",".join(parts[4:]).strip(),   # 文本里理论上没有逗号, 这里兜底合并
        }
    except (ValueError, IndexError):
        return None
