# -*- coding: utf-8 -*-
"""AUV 自主模式 —— GrandRDKv2.5 全新重写的模式壳（替代 v2.2 的 mode_auv.py）

与主链路的契约（不许破坏）：
  - 继承 mode_base.ModeBase，id=MODE_AUV；由 mode_dispatcher 注册
  - on_enter 下发 0x04 模式帧（值 0x05 = AUV 语义，与固件约定不变）
  - tick 每拍: pump_telemetry → 急停红线检查 → mission.step() → 组 0x09 下发
  - 安全红线：0x09 帧没有 STANDBY 守卫，急停闩锁期间一个字节都不能发
    （否则会静默退出急停）—— estop_latch 检查必须排在一切下发之前

v2.5 重写版与 v2.2 的差异：
  - 状态机换成 mission.Mission（阶段注册制，阶段表见 task_config.STAGE_TABLE）
  - 观测接口合并为 obs.VisionIF / obs.DepthIF（数据格式不变）
  - 移除 kalman_launcher 托管与 auv_report 状态回传（需要时按旧版思路重加）
  - 本文件自带 sys.path 注入：v2.2 的 run.sh PYTHONPATH 不含 move_test 段也能接回
"""
import os
import sys

# 把本目录放进 sys.path —— move_test 内部文件用平级 import（run.sh 无需改动）
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import to32_config as C   # 模式常量 MODE_AUV 等仍取自 to32_config
import link_stm32 as S    # 0x04/0x09 组帧
from mode_base import ModeBase

import task_config as TC
import obs
from mission import Mission, apply_yaw_mirror


class AuvMode(ModeBase):
    """AUV 自主模式：on_enter 建状态机，tick 每拍驱动并把控制量组 0x09 下发"""
    id = C.MODE_AUV
    name = "AUV"
    desc = "自主任务模式（v2.5 重写骨架）"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.state = "idle"            # idle / run（供状态展示）
        self.last_tel = None           # 缓存最近一帧下位机遥测，传给状态机
        self._ignore_cnt = 0           # 被忽略的手动杆位帧计数（只提示前 3 帧）
        self.mission = None            # 状态机实例：on_enter 重建，on_exit 丢弃
        self.yaw_mirror = bool(getattr(TC, 'YAW_MIRROR', True))
        self.log_every = float(getattr(TC, 'AUV_LOG_EVERY_S', 1.0))
        self._last_log_ts = 0.0
        self._last_note_stage = ''

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        super().on_enter(prev_id)
        self._ignore_cnt = 0
        # 模式帧不被急停闩锁抑制：与固件配合，AUV 就位优先（协议 §12.3）
        self.send_downlink(S.frame_mode(S.MODE_AUV), "0x04 AUV")
        self.state = "idle"
        # 每次进入都重建状态机：保证每次切进 AUV 都从头跑
        self.mission = Mission(TC, vision=obs.VisionIF(log=self.log),
                               depth=obs.DepthIF(log=self.log), log=self.log)
        n = len(getattr(TC, 'STAGE_TABLE', []) or [])
        if n == 0:
            self.log("[AUV] 阶段表为空（v2.5 骨架）—— 状态机开机即 DONE，不会下发运动指令")
        else:
            self.log("[AUV] 任务状态机已就绪（%d 个阶段）" % n)

    def on_exit(self, next_id):
        self.state = "idle"
        self.mission = None            # 丢弃状态机：阶段/计时器不残留到下次
        super().on_exit(next_id)

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):
        """自主模式忽略手动杆位（只记录）"""
        self._ignore_cnt += 1
        if self._ignore_cnt <= 3:
            self.log("[AUV] 忽略手动杆位 surge=%.2f sway=%.2f heave=%.2f yaw=%.2f"
                     % (cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))

    # ---------------- 周期 ----------------
    def tick(self, now, dt):
        self.pump_telemetry(now)          # AUV 期间由本模式掌握遥测回传节拍
        if self.ctx.estop_latch:          # 安全红线：急停闩锁期间一个字节都不发
            return
        if self.mission is None:
            return
        cmd = self.mission.step(now, dt, self.last_tel)
        if not cmd:                       # None = 任务已结束，不再下发
            return
        self.state = "run" if cmd["stage"] not in ("DONE", "ABORT") else "idle"
        yaw = apply_yaw_mirror(cmd["yaw"], self.yaw_mirror)
        self.send_downlink(
            S.frame_motion(
                pitch_deg=0.0,            # 任务脚本不控俯仰
                yaw_deg=yaw,              # 目标航向绝对角
                roll_deg=0.0,
                depth_cm=cmd["depth"],    # 目标深度(cm，固件内闭环)
                surge=cmd["surge"],
                sway=cmd["sway"],
                stick_stop=cmd["stop"]),  # 仅任务结束时置 1 停推
            self._note(now, cmd))

    def _note(self, now, cmd):
        """0x09 日志标注：阶段切换立即打，同阶段内按 AUV_LOG_EVERY_S 节流"""
        if cmd["stage"] != self._last_note_stage:
            self._last_note_stage = cmd["stage"]
            self._last_log_ts = now
            return "0x09 AUV:%s %s" % (cmd["stage"], cmd["note"])
        if (now - self._last_log_ts) >= self.log_every:
            self._last_log_ts = now
            return "0x09 AUV:%s %s" % (cmd["stage"], cmd["note"])
        return ""

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):
        """缓存最近一帧遥测；解析统一交给 tel_builder，这里只缓存"""
        self.last_tel = tel

    def build_telemetry(self):
        """返回 None = 本模式不发自有 $TEL，由 mode_dispatcher 回落 tel_builder 统一映射"""
        return None
