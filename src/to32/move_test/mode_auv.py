# -*- coding: utf-8 -*-
"""AUV 自主模式 —— GrandRDKv2.5 全新重写的模式壳（替代 v2.2 的 mode_auv.py）

与主链路的契约（不许破坏）：
  - 继承 mode_base.ModeBase，id=MODE_AUV；由 mode_dispatcher 注册
  - on_enter 下发 0x04 模式帧（值 0x05 = AUV 语义，与固件约定不变）
  - tick 每拍: pump_telemetry → 急停红线检查 → mission.step() → 组 0x09 下发
  - on_exit 先发停推帧（stick_stop=1，急停闩锁时让路）再丢弃状态机，防推力悬挂
  - 安全红线：0x09 帧没有 STANDBY 守卫，急停闩锁期间一个字节都不能发
    （否则会静默退出急停）—— estop_latch 检查必须排在一切下发之前

v2.5 重写版与 v2.2 的差异：
  - 状态机换成 mission.Mission（阶段注册制，阶段表见 task_config.STAGE_TABLE）
  - 观测接口：obs.VisionIF（视觉）。
  - ★ [2026-10-11 用户要求] **深度卡尔曼已从板端移除**：不再注入 obs.DepthIF、
    不再经 kalman_launcher 拉起 depth_kalman；**所有深度判定一律改用固件深度计遥测
    actual_depth_cm**（与下发 depth_cm 是同一固件帧，可直接相减）。
  - auv_report 状态回传未接（需要时按旧版思路重加）
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
# [2026-10-11] 深度卡尔曼托管已移除（用户要求）：不再 import DepthKalmanLauncher


class AuvMode(ModeBase):
    """AUV 自主模式：on_enter 建状态机，tick 每拍驱动并把控制量组 0x09 下发"""
    id = C.MODE_AUV
    name = "AUV"
    desc = "自主任务模式（v2.5 重写骨架）"
    RESELECT_RESTARTS = True       # [2026-10-10] 重复选择本模式 = 重新开始（整盘复位，见 dispatcher.switch_mode）

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
        self._paused_since = None      # [2026-10-10] 连续 paused（本拍不发 0x09）起始时刻
        self._paused_log_ts = 0.0      # paused 日志节流时间戳

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        super().on_enter(prev_id)
        self._ignore_cnt = 0
        # 模式帧不被急停闩锁抑制：与固件配合，AUV 就位优先（协议 §12.3）
        self.send_downlink(S.frame_mode(S.MODE_AUV), "0x04 AUV")
        self.state = "idle"
        # 每次进入都重建状态机：保证每次切进 AUV 都从头跑
        # [2026-10-11] depth=None —— 深度卡尔曼已移除，深度判定走固件深度计遥测
        #   （t_function.dive_step / exit_step 直接读 ctx.tel['actual_depth_cm']）
        self.mission = Mission(TC, vision=obs.VisionIF(log=self.log),
                               depth=None, log=self.log, task_pids=self.ctx.task_pids)
        n = len(getattr(TC, 'STAGE_TABLE', []) or [])
        self._paused_since = None
        self._paused_log_ts = 0.0
        if n == 0:
            self.log("[AUV] 阶段表为空（v2.5 骨架）—— 状态机开机即 DONE，不会下发运动指令")
        else:
            self.log("[AUV] 任务状态机已就绪（%d 个阶段）：%s"
                     % (n, ' → '.join(getattr(c, 'NAME', getattr(c, '__name__', str(c)))
                                      for c in (getattr(TC, 'STAGE_TABLE', []) or []))))

    def on_exit(self, next_id):
        # 切走前先停推：退出时上一拍可能仍挂着任务最后一段推力（对齐 TestMode.on_exit 做法）
        if not self.ctx.estop_latch:   # 安全红线：急停闩锁期间一个字节都不能发（见文件头契约）
            try:
                self.send_downlink(
                    S.frame_motion(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0,
                                   depth_cm=0.0, surge=0.0, sway=0.0, stick_stop=1),
                    "0x09 AUV 退出停推")
            except Exception as e:
                self.log("[AUV] 退出停推帧下发异常: %r" % e)
        self.state = "idle"
        if self.mission is not None:
            self.mission.close()
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
            self._paused_since = None
            return
        if cmd.get('paused'):
            self._note_paused(now, cmd)   # 本拍不发 0x09：留下可读原因（原来完全静默）
            return
        self._paused_since = None
        self.state = "run" if cmd["stage"] not in ("DONE", "ABORT") else "idle"
        yaw = apply_yaw_mirror(cmd["yaw"], self.yaw_mirror)
        # 推力符号在**输出侧**统一施加（2026-10-08）：任务系 → 固件系的换算只在这一处，
        # 覆盖 forward_step/sway_step/撞球 ram·back/过门 surge（它们各自的 cmd 都到这里）。
        # AUV_SURGE_SIGN / AUV_SWAY_SIGN 都是「上车方向标定键」，默认 1.0 = 直通。
        surge = max(-1.0, min(1.0, float(cmd["surge"]) * float(getattr(TC, 'AUV_SURGE_SIGN', 1.0))))
        sway = max(-1.0, min(1.0, float(cmd["sway"]) * float(getattr(TC, 'AUV_SWAY_SIGN', 1.0))))
        self.send_downlink(
            S.frame_motion(
                pitch_deg=0.0,            # 任务脚本不控俯仰
                yaw_deg=yaw,              # 目标航向绝对角
                roll_deg=0.0,
                depth_cm=cmd["depth"],    # 目标深度(cm，固件内闭环)
                surge=surge,
                sway=sway,
                stick_stop=cmd["stop"]),  # 仅任务结束时置 1 停推
            self._note(now, cmd))

    def _note_paused(self, now, cmd):
        """paused = 本拍不下发 0x09（如等遥测/等判据）。首拍立即提示，之后按 log_every 重复。

        [2026-10-10 增] 与 test_runner 同款修复：原来 paused 是裸 return —— 任务静默不动、
        日志里没有任何线索。现在显式打出"本拍没发 0x09 + 原因 + 已持续多久"。
        """
        note = str(cmd.get('note') or '')
        if self._paused_since is None:
            self._paused_since = now
            self._paused_log_ts = now
            self.log("[AUV] 本拍不下发 0x09（paused）：%s —— 判据/遥测未就绪时会持续如此，"
                     "属宁停勿猜，不是掉线" % (note or '无说明'))
            return
        if (now - self._paused_log_ts) >= max(1.0, self.log_every):
            self._paused_log_ts = now
            self.log("[AUV] 仍不下发 0x09（已持续 %.1fs）：%s"
                     % (now - self._paused_since, note or '无说明'))

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
