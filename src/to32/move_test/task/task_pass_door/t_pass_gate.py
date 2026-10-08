# -*- coding: utf-8 -*-
"""穿门阶段：复用GateMission和S100可调GatePid，等待有效遥测后计算。

2026-10-08 quad_cv 落地：_VisionAdapter 挂 GateCvVision——door bbox + 干净帧
→ psi/交点中心增强观测（全兜底, CV 挂了自动退纯 bbox 行为）。
"""
import math
import os
import sys

# 把 task_pass_door/ 与 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task/task_pass_door
_TASK = os.path.dirname(_HERE)                            # move_test/task
_PARENT = os.path.dirname(_TASK)                          # move_test
for _p in (_HERE, _TASK, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mission import Stage
from task_pid_controller import TaskPidController
from gate_filter import EFilter
import gate_config as GC
from gate_mission import ST_DONE


class _VisionAdapter:
    """ctx.vision 的 door 观测 + GateCvVision 增强 → GateMission 的 obs。"""

    def __init__(self, ctx):
        self.ctx = ctx
        self.cv = None
        if GC.GATE_CV_ENABLE:
            try:
                from gate_vision_quad import GateCvVision
                self.cv = GateCvVision(
                    img_w=float(GC.GATE_IMG_W), img_h=float(GC.GATE_IMG_H),
                    e_sign=float(GC.GATE_E_SIGN), fx=float(GC.GATE_CV_FX_PX),
                    frame_path=str(GC.GATE_CV_FRAME),
                    det_path=str(GC.GATE_DET_PATH),
                    period_s=float(GC.GATE_CV_PERIOD_S),
                    stale_s=float(GC.GATE_CV_STALE_S),
                    cv_opts=GC.GATE_CV_OPTS, log=ctx.log)
            except Exception as e:
                if ctx.log:
                    ctx.log('[gate] CV 适配层不可用, 退纯 bbox 行为: %s' % e)
                self.cv = None

    def poll(self, now):
        obs = self.ctx.vision.poll('front', 'gate', now)
        if not obs:
            return None
        width = float(getattr(self.ctx.vision, 'w', GC.GATE_IMG_W))
        # det 口径基础键（与老版本一致）
        det_obs = dict(
            x1=float(obs['x1']), y1=float(obs['y1']),
            x2=float(obs['x2']), y2=float(obs['y2']),
            w=float(obs['w']), score=obs.get('score', 0), frame=obs.get('frame'))
        det_obs['w_ratio'] = float(obs['w']) / width if width > 0 else 0.0

        if self.cv is None:
            # 纯 bbox 口径（v4 行为）
            error = float(obs['ex']) * GC.GATE_E_SIGN
            if width <= 0 or not math.isfinite(error):
                return None
            det_obs['e_raw'] = error
            return det_obs

        try:
            return self.cv.observe(det_obs, now)
        except Exception:
            # 适配层自身异常 → 退纯 bbox, 绝不挡任务
            error = float(obs['ex']) * GC.GATE_E_SIGN
            det_obs['e_raw'] = error if math.isfinite(error) else None
            det_obs['cv_lvl'] = 0
            return det_obs


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
        # note 带实时读数（mode_auv 1Hz 日志直接打出来, 手动整定 PID 靠它看数）
        snap = self.gate.snapshot()
        note = '%s lvl=%d e=%s psi=%s w=%.2f' % (
            snap['state'], snap['cv_lvl'],
            ('%+.3f' % snap['e']) if snap['e'] is not None else ' -- ',
            ('%+.3f' % snap['psi_f']) if snap['psi_f'] is not None else ' -- ',
            float(snap['w_ratio'] or 0.0))
        self.command = dict(stage=self.NAME, note=note, yaw=yaw,
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
