# -*- coding: utf-8 -*-
"""ROV 遥控模式 —— 中位机模式之一

运行逻辑（首版，映射语义参照 /userdata/To32_1/README.md §4 已验证结论）:
  - surge / sway → V2 `0x09` 推力槽 [8]/[9]（[-1,1] ×127，固件内再 ×2 → ±254）
  - heave        → 积分成目标深度（默认 20 cm/s；heave>0=上浮=深度减小）
  - yaw          → 积分成目标航向（默认 60 °/s；固件对 Yaw 取负归一化 → 此处取反抵消）
  - pitch/roll   → 目标恒 0（上位机 $CMD 不下发姿态）
  - led/grab/store → V2 无对应字段，丢弃并计数
  - **急停锁存期间抑制一切 0x09**（0x09 无 STANDBY 守卫，可绕过急停，安全红线）

预留空间:
  - 单推进器分配 / 速度环（tick 钩子）
  - 目标姿态/深度闭环（on_downlink 钩子已有, 可接 PID）
  - $PID 透传 ch0~3（默认丢弃, 语义不对齐）
"""
import time

import config as C
import link_stm32 as S
from mode_base import ModeBase


class RovMode(ModeBase):
    id = C.MODE_ROV
    name = "ROV"
    desc = "遥控模式：手柄杆位 → V2 0x09"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.wait_dt_max = 0.25          # 积分 dt 上限（帧间隔异常时防跳变）
        self.yaw_rate = 60.0             # 摇杆满偏时航向角速率 °/s（可调）
        self.depth_rate = 20.0           # 摇杆满偏时深度速率 cm/s（可调）
        self.heave_sign = +1             # heave>0=上浮 → 深度减小（反了改 -1）
        self.yaw_mirror = True           # 抵消固件 Yaw 取负
        self.target_yaw_deg = 0.0
        self.target_depth_cm = 0.0
        self.last_tel = None
        self.dropped = {"led": 0, "grab": 0, "store": 0}
        self._last_t = None

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        super().on_enter(prev_id)
        self._last_t = time.time()
        # 进入遥控模式先让固件处于 ROV 模式（不发 0x09 也能先就位）
        if not self.ctx.estop_latch:
            self.send_downlink(S.frame_mode(S.MODE_ROV), "0x04 ROV")
        # 上位机把中位机切到 ROV 后：按协议立刻请求一次回传（0x0C，1 请求回 1 帧），
        # 之后由本模式 tick() 按 STM32_POLL_HZ 持续请求（见下方 tick）
        self._last_poll_ts = time.time()
        self.link_stm32.send_telemetry_request("0x0C 遥测请求(ROV 进入)")

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):
        """每帧 $CMD → 0x09 摇杆综合运动"""
        if self.ctx.estop_latch:
            self.log("[ROV] 急停锁存中, 抑制 0x09（恢复必须解除锁存）")
            return
        now = time.time()
        if self._last_t is None:
            self._last_t = now
        dt = min(self.wait_dt_max, max(0.001, now - self._last_t))
        self._last_t = now

        # yaw 积分 → 目标航向（固件镜像取反）
        self.target_yaw_deg = self.target_yaw_deg + cmd["yaw"] * self.yaw_rate * dt
        # heave 积分 → 目标深度（钳位 0~200cm）
        self.target_depth_cm = max(0.0, min(200.0,
                                self.target_depth_cm - cmd["heave"] * self.heave_sign * self.depth_rate * dt))
        yaw_out = (-self.target_yaw_deg) if self.yaw_mirror else self.target_yaw_deg

        frame = S.frame_motion(pitch_deg=0.0, yaw_deg=yaw_out, roll_deg=0.0,
                               depth_cm=self.target_depth_cm,
                               surge=cmd["surge"], sway=cmd["sway"])
        self.send_downlink(frame, "0x09 ROV")

        # V2 装不下的字段：丢弃并计数（预留：后续可扩展 0x0D/0x0E/0x0F）
        if cmd["led1"] or cmd["led2"]:
            self.dropped["led"] += 1
        if cmd["grab"]:
            self.dropped["grab"] += 1
        if cmd["store"]:
            self.dropped["store"] += 1

    def on_pid(self, pid):
        """$PID: 上位机 0~11 通道 ↔ V2 0x01 仅 0~3(Pitch/Yaw/Roll/Depth), 默认丢弃"""
        self.log("[ROV] $PID ch=%d 默认丢弃（V2 0x01 仅 ch0~3; 预留 --pid-passthru）" % pid["ch"])

    # ---------------- 周期 ----------------
    def tick(self, now, dt):
        """ROV 期间按 STM32_POLL_HZ 持续请求下位机回传（0x0C）；控制律暂为预留"""
        self.pump_telemetry(now)

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):
        self.last_tel = tel

    def build_telemetry(self):
        """$TEL 41 字段组装（首版: 有遥测才发, 映射 TODO 见 To32_1/README §6）"""
        if not self.last_tel:
            return None
        t = self.last_tel
        vals = [
            t.get("actual_roll", 0), t.get("actual_pitch", 0), t.get("actual_yaw", 0),
            t.get("gyro_roll", 0), t.get("gyro_pitch", 0), t.get("gyro_yaw", 0),
            t.get("actual_depth_cm", 0) / 100.0,
            0, 0, 0,                                   # vx vy vz（光流预留）
            t.get("target_roll", 0), t.get("target_pitch", 0), t.get("target_yaw", 0),
            0, 0, 0, t.get("target_depth_cm", 0) / 100.0, 0, 0, 0,   # t_gx..t_vz（16=目标深度 m）
            0, 0, 0, 0, 0,                            # 电池/温度（无 → 0）
        ]
        vals += [m / 1000.0 for m in t.get("motors", [0] * 8)]          # thr0..7
        vals += [0.0] * (12 - 8)                                        # thr8..11（预留）
        vals += [t.get("acc_x", 0), t.get("acc_y", 0), t.get("acc_z", 0), 0.0]  # ax ay az alt
        import protocol as P
        return P.build_tel(vals[:P.TEL_FIELD_COUNT])