# -*- coding: utf-8 -*-
"""下位机（STM32）链路 —— V2 二进制协议【骨架，预留】

本文件只搭"传输 + 组帧/收帧 + 遥测轮询"骨架；
「各模式要把什么下发下去」的映射逻辑写在各模式文件里（mode_rov.py / mode_auv.py）。

协议依据（以固件源码为准，勿凭记忆改）:
  《上位机对接说明_V2》 + 《V2_固件字段表_T0.1》
  - 帧:  CD | LEN | FUNC | DATA(n) | DC      LEN = n+3, 整帧 = LEN+1, ≤64B, 多字节小端
  - 只有 0x0C 遥测请求会回帧（1:1）；命令帧 0x01~0x0B 只执行、不回包
  - 0x09 摇杆综合运动 = 15B:
        [0..1] 目标Pitch int16×100   [2..3] 目标Yaw int16×100   [4..5] 目标Roll int16×100
        [6..7] 目标深度 uint16×100(cm)  [8] surge int8×2  [9] sway int8×2  [10] FLAG(bit0)
  - 0x04 模式: 0x00 启动/恢复  0x01 急停  0x02 预编程  0x03 ROV  0x04 测试
  - 0x01 PID 通道只有 0~3 = Pitch/Yaw/Roll/Depth（与上位机 0~11 语义不同）
  - **0x09 没有 STANDBY 守卫**：急停后再发 0x09 会静默退出急停 → 急停锁存必须由中位机负责
"""
import threading
import time

HEAD = 0xCD
TAIL = 0xDC
MAX_FRAME = 64

# ---- 功能码 ----
FUNC_SET_PID        = 0x01
FUNC_TEST_THROTTLE  = 0x02
FUNC_TARGET_ANGLE   = 0x03
FUNC_MODE           = 0x04
FUNC_SAVE           = 0x05
FUNC_READ_ANGLE     = 0x08
FUNC_MOTION         = 0x09
FUNC_DEPTH          = 0x0A
FUNC_READ_DEPTH     = 0x0B
FUNC_TELEMETRY      = 0x0C

# ---- 模式命令 ----
MODE_START     = 0x00
MODE_ESTOP     = 0x01
MODE_PREPROGRAM = 0x02
MODE_ROV       = 0x03
MODE_TEST      = 0x04

TELEM_FRAME_LEN = 48    # 0x0C 遥测回帧整帧 48B（LEN=0x2F）
# ==================== 组帧 ====================
def build_frame(func, data=b""):
    """通用 V2 组帧: CD LEN FUNC DATA DC"""
    data = bytes(data)
    return bytes([HEAD, len(data) + 3, func & 0xFF]) + data + bytes([TAIL])


def frame_telemetry_request():
    """0x0C 遥测请求（唯一会触发回帧的帧）"""
    return build_frame(FUNC_TELEMETRY)


def frame_mode(mode):
    """0x04 模式命令"""
    return build_frame(FUNC_MODE, bytes([mode & 0xFF]))


def frame_motion(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0, depth_cm=0.0,
                 surge=0.0, sway=0.0, stick_stop=False):
    """0x09 摇杆综合运动（预留工具函数，供 ROV 模式调用）

    pitch/yaw/roll: 目标姿态角(度)；depth_cm: 目标深度(cm, 钳位 0~200)
    surge/sway: [-1,1] → int8(×127)，固件内再 ×2 → 推力域 ±254
    stick_stop: FLAG bit0 = 停 surge/sway（保留姿态与深度）
    """
    import struct
    p = int(round(max(-90.0, min(90.0, float(pitch_deg))) * 100))
    y = int(round(float(yaw_deg) * 100))
    r = int(round(max(-90.0, min(90.0, float(roll_deg))) * 100))
    d = int(round(max(0.0, min(200.0, float(depth_cm))) * 100))
    sg = int(round(max(-1.0, min(1.0, float(surge))) * 127))
    sw = int(round(max(-1.0, min(1.0, float(sway))) * 127))
    flag = 0x01 if stick_stop else 0x00
    data = struct.pack("<hhhHbbB", p, y, r, d, sg, sw, flag)
    return build_frame(FUNC_MOTION, data)


def frame_set_pid(ch, kp, ki, kd):
    """0x01 设置 PID（固件仅认 ch 0~3）"""
    import struct
    return build_frame(FUNC_SET_PID, struct.pack(
        "<Bhhh", int(ch) & 0xFF,
        int(round(float(kp) * 100)), int(round(float(ki) * 100)), int(round(float(kd) * 100))))


def frame_target_depth(meters):
    """0x0A 目标深度（米 → uint16LE ×100 cm，钳位 0~2.00m）"""
    import struct
    cm = max(0.0, min(2.00, float(meters))) * 100.0
    return build_frame(FUNC_DEPTH, struct.pack("<H", int(round(cm * 100))))
def hex_str(data):
    return " ".join("%02X" % b for b in bytes(data))


# ==================== 收帧 ====================
class FrameParser:
    """CD/LEN/DC 定界状态机 + 失步重同步；返回 [(func, data), ...]"""

    def __init__(self, max_frame=MAX_FRAME):
        self.max_frame = max_frame
        self._buf = bytearray()
        self.bad_tail = 0
        self.bad_len = 0
        self.resync_bytes = 0

    def feed(self, chunk):
        out = []
        if chunk:
            self._buf += chunk
        while self._buf:
            if self._buf[0] != HEAD:
                idx = self._buf.find(HEAD)
                if idx < 0:
                    self.resync_bytes += len(self._buf)
                    self._buf.clear()
                    break
                self.resync_bytes += idx
                del self._buf[:idx]
            if len(self._buf) < 2:
                break
            ln = self._buf[1]
            total = ln + 1
            if ln < 3 or total > self.max_frame:
                self.bad_len += 1
                del self._buf[0]
                continue
            if len(self._buf) < total:
                break
            if self._buf[total - 1] != TAIL:
                self.bad_tail += 1
                del self._buf[0]
                continue
            func = self._buf[2]
            data = bytes(self._buf[3:total - 1])
            del self._buf[:total]
            out.append((func, data))
        return out


def parse_telemetry(data):
    """0x0C 遥测 DATA(44B) → dict；字段偏移见《V2_固件字段表_T0.1》§2"""
    import struct
    if len(data) < 44:
        return None
    (tp, ty, tr, ap, ay, ar,
     gp, gy, gr, ax, ay2, az) = struct.unpack_from("<12h", data, 0)
    motors = list(struct.unpack_from("<8h", data, 24))
    t_depth, a_depth = struct.unpack_from("<HH", data, 40)
    return {
        "target_pitch": tp / 100.0, "target_yaw": ty / 100.0, "target_roll": tr / 100.0,
        "actual_pitch": ap / 100.0, "actual_yaw": ay / 100.0, "actual_roll": ar / 100.0,
        "gyro_pitch": gp / 100.0, "gyro_yaw": gy / 100.0, "gyro_roll": gr / 100.0,
        "acc_x": ax / 100.0, "acc_y": ay2 / 100.0, "acc_z": az / 100.0,
        "motors": motors,
        "target_depth_cm": t_depth / 100.0, "actual_depth_cm": a_depth / 100.0,
    }


# ==================== 链路 ====================
class Stm32Link:
    """下位机链路。

    port_spec: "off" = 完全不启用；"sim" = 空跑（只打印将发送的帧，不开串口）；
               其它 = 串口设备路径（如 /dev/ttyCH9344USB5）
    """

    def __init__(self, cfg, log, port_spec="sim", on_telemetry=None):
        self.cfg = cfg
        self.log = log
        self.port_spec = port_spec
        self.on_telemetry = on_telemetry
        self.parser = FrameParser()
        self.ser = None
        self.enabled = (port_spec != "off")
        self.sim = (port_spec == "sim")
        self.stats = {"tx_frames": 0, "rx_frames": 0, "bad_tail": 0, "bad_len": 0}
        self._stop = threading.Event()
        self._thread = None
        self.last_req_ts = 0.0      # 最近一次发出 0x0C 请求的时间（模式层/兜底轮询共享）

    def open(self):
        if not self.enabled:
            self.log("[STM32] 已禁用（--stm32 off）")
            return
        if self.sim:
            self.log("[STM32] 空跑模式(sim)：不打开串口，只打印将下发的 V2 帧")
            return
        try:
            import serial
            self.ser = serial.Serial(self.port_spec, self.cfg.STM32_BAUD, timeout=0.05)
            self.log("[STM32] 已打开 %s @ %d" % (self.port_spec, self.cfg.STM32_BAUD))
        except Exception as e:
            self.log("[STM32] 打开失败: %s（转为空跑）" % e)
            self.sim = True

    def start(self):
        if not self.enabled or self.sim:
            return
        self._thread = threading.Thread(target=self._poll_loop, name="Stm32Poll", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def send(self, frame, note=""):
        """下发一帧（预留：由各模式决定帧内容）"""
        if not self.enabled or not frame:
            return
        self.stats["tx_frames"] += 1
        if self.sim or self.ser is None:
            self.log("[STM32·sim] TX(%s) %s" % (note, hex_str(frame)))
            return
        try:
            self.ser.write(bytes(frame))
        except Exception as e:
            self.log("[STM32] 写失败: %s" % e)

    def send_telemetry_request(self, note="0x0C 遥测请求"):
        """发一帧 0x0C 遥测请求（按协议，下位机收到即回一帧）。同时记时间戳供兜底轮询去重。"""
        self.last_req_ts = time.time()
        self.send(frame_telemetry_request(), note)

    def _poll_loop(self):
        """链路层兜底轮询：只在"最近没有模式层按节拍请求过"时才补发 0x0C。

        ROV 模式活跃时由 mode_rov.tick() 按同一节拍请求（mode_base.pump_telemetry），
        这里检测到刚请求过就跳过，避免重复（协议 1:1，重复会翻倍遥测率）；
        AUV 模式、以及急停锁存期间(tick 暂停) 则由本循环保底，保证 $TEL 不断。
        """
        period = 1.0 / max(0.5, self.cfg.STM32_POLL_HZ)
        while not self._stop.is_set():
            t0 = time.time()
            try:
                if (time.time() - self.last_req_ts) >= period:
                    self.ser.write(frame_telemetry_request())
                    self.stats["tx_frames"] += 1
                    self.last_req_ts = time.time()
                chunk = self.ser.read(512)
                if chunk:
                    for func, data in self.parser.feed(chunk):
                        self.stats["rx_frames"] += 1
                        if func == FUNC_TELEMETRY and self.on_telemetry:
                            tel = parse_telemetry(data)
                            if tel:
                                self.on_telemetry(tel)
                        else:
                            self.log("[STM32] RX func=0x%02X data=%s" % (func, hex_str(data)))
            except Exception as e:
                self.log("[STM32] 轮询异常: %s" % e)
                self._stop.wait(0.5)
            # 量测差值再睡：read(timeout=0.05) 会吃掉节拍，固定 sleep(period)
            # 会把兜底轮询拖到 ~6.7Hz；按剩余时间等待可稳定 ~POLL_HZ(10Hz)
            remain = period - (time.time() - t0)
            if remain > 0:
                self._stop.wait(remain)

    def snapshot(self):
        s = dict(self.stats)
        s["parser_bad_tail"] = self.parser.bad_tail
        s["parser_bad_len"] = self.parser.bad_len
        s["port"] = ("sim" if self.sim else self.port_spec)
        return s