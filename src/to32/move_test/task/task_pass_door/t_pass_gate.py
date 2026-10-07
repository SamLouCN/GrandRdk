# -*- coding: utf-8 -*-
"""穿门阶段：复用GateMission和S100可调GatePid，等待有效遥测后计算。"""
import math
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                           # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mission import Stage
from task_pid_controller import TaskPidController
from gate_filter import EFilter
import gate_config as GC
from gate_mission import ST_DONE


class _VisionAdapter:
    def __init__(self, ctx):
        self.ctx = ctx

    def poll(self, now):
        obs = self.ctx.vision.poll('front', 'gate', now)
        if not obs:
            return None
        width = float(getattr(self.ctx.vision, 'w', GC.GATE_IMG_W))
        error = float(obs['ex']) * GC.GATE_E_SIGN
        if width <= 0 or not math.isfinite(error):
            return None
        return dict(e_raw=error, w_ratio=float(obs['w']) / width,
                    score=obs.get('score', 0), frame=obs.get('frame'))


class PassGateAll(Stage):
    """把已有GateMission接到阶段框架，共用S100正在被调参的GatePid。"""
    NAME = 'PassGate'

    def enter(self, now):
        self.controller = self.ctx.task_pids or TaskPidController()
        self.gate = None
        self.command = None
        self.hold_depth = None

    def _tel(self):
        tel = self.ctx.tel or {}
        return {'yaw': tel.get('actual_yaw'), 'omega': tel.get('gyro_yaw', 0.0)}

    def _send(self, yaw, surge, sway):
        self.command = dict(stage=self.NAME, note=self.gate.state, yaw=yaw,
                            depth=self.hold_depth, surge=surge, sway=sway, stop=0)

    def step(self, now, dt):
        tel = self.ctx.tel or {}
        try:
            yaw = float(tel['actual_yaw'])
            depth = float(tel['actual_depth_cm'])
            if not all(math.isfinite(v) for v in (yaw, depth)):
                raise ValueError
        except (KeyError, TypeError, ValueError):
            return dict(stage=self.NAME, note='等待有效遥测', paused=True)
        if self.gate is None:
            self.hold_depth = max(0.0, min(200.0, depth))
            filt = EFilter(GC.GATE_EF_TYPE, GC.GATE_VISKF_PATH, GC.GATE_STALE_S, GC.GATE_EF_ALPHA)
            self.gate = self.controller.create_gate_mission(_VisionAdapter(self.ctx), filt,
                                                           self._tel, self._send, self.ctx.log)
        self.gate.tick(now)
        if self.gate.state == ST_DONE:
            return None
        return self.command


# 仅提供可选任务表，不修改正式/测试任务的默认排序。
PASS_GATE_TABLE = [PassGateAll]
