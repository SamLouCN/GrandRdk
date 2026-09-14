# -*- coding: utf-8 -*-
"""AUV 自主模式 —— 中位机模式之一

当前为"骨架 + 占位"：进入时把下位机切到 0x02 预编程模式（AUV 语义），
忽略上位机手动杆位（只记录不动作），自主逻辑全部经 tick() 扩展。

预留空间（后续各子系统的落点）:
  - tick():     AUV 任务状态机（int → run → hold → abort）
  - on_downlink: 用下位机遥测做深度/航向闭环
  - 规划/视觉:   从 vp5.x 视觉任务取目标（任务间用 ctx 共享）
  - on_cmd:      可选择性响应目标深度/姿态等扩展字段（协议暂未定义）
"""
import time

import config as C
import link_stm32 as S
from mode_base import ModeBase


class AuvMode(ModeBase):
    id = C.MODE_AUV
    name = "AUV"
    desc = "自主模式：忽略手动杆位，运行自主逻辑(占位)"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.state = "idle"          # 预留: idle/init/run/hold/abort
        self.last_tel = None
        self._ignore_cnt = 0

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        super().on_enter(prev_id)
        self._ignore_cnt = 0
        if not self.ctx.estop_latch:
            # AUV = 预编程模式（0x02）；帧语义见 V2 字段表
            self.send_downlink(S.frame_mode(S.MODE_PREPROGRAM), "0x04 AUV(预编程)")
        self.state = "idle"

    def on_exit(self, next_id):
        self.state = "idle"
        super().on_exit(next_id)

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):
        """自主模式忽略手动杆位; mode 字段由主循环负责切换, 这里只记录"""
        self._ignore_cnt += 1
        if self._ignore_cnt <= 3:
            self.log("[AUV] 忽略手动杆位 surge=%.2f sway=%.2f heave=%.2f yaw=%.2f"
                     % (cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))

    def on_pid(self, pid):
        """预留：AUV 内部 PID 参数（或直接下发 0x01 ch0~3）"""
        self.log("[AUV] $PID ch=%d 预留（AUV 自主控制参数待规划）" % pid["ch"])

    # ---------------- 周期：自主逻辑入口（预留） ----------------
    def tick(self, now, dt):
        """AUV 主状态机占位——后续在这里写任务/路径/闭环节点"""
        # if self.state == "idle" and self.ready():   self._start_mission()
        # if self.state == "run":                     self._update_control(now, dt)
        pass

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):
        self.last_tel = tel

    def build_telemetry(self):
        # 与 ROV 相同结构；首版占位（后续可带任务状态附加信息）
        if not self.last_tel:
            return None
        return None  # TODO: 复用 / 组装 $TEL；暂不发