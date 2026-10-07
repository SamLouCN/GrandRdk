#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_mission.py — 过门对准三态流程（方案 v4 §二）

状态:
    WAIT   等门: 原地保持( surge=0, sway=0, 航向保持当前值 )
    ALIGN  对准: yaw+sway 双环闭环, surge=0; 达标 → 锁航向 → 关视觉 → BLIND
    BLIND  盲跑: 视觉整块停用, ψ_target 冻结, surge 前进, 航向交固件航向环(陀螺仪)
           出口: T_BLIND 到时 → 停 surge（过门判定后接, 留桩）
    DONE   本期终点: 保持航向, surge=0

通用丢帧规则（WAIT/ALIGN 生效）: 丢帧瞬间冻结上一帧位置——ψ_target/sway 保持
最后一拍值, surge=0; LOST_WAIT 内恢复无缝继续, 超时回 WAIT。

外部接口（全部注入, 本包不碰串口/遥测文件）:
    tel()  -> {'yaw': 实际航向° or None, 'omega': 陀螺仪z轴角速度°/s or None}
    send(psi_target, surge, sway)   # 每拍运动输出
    log(msg)                        # 状态迁移/告警, 可为 None
"""
import time

from gate_pid import GatePid, wrap180

ST_WAIT = 'WAIT'
ST_ALIGN = 'ALIGN'
ST_BLIND = 'BLIND'
ST_DONE = 'DONE'


class GateMission(object):

    def __init__(self, cfg, vision, efilter, pid, tel, send, log=None,
                 allow_blind=True):
        self.cfg = cfg
        self.vision = vision          # GateVision
        self.filt = efilter           # EFilter
        self.pid = pid                # GatePid
        self.tel = tel                # callable -> dict
        self.send = send              # callable(psi_target, surge, sway)
        self.log = log
        self.allow_blind = bool(allow_blind)   # False=只跑对准环(开环回放用)

        self.state = ST_WAIT
        self.psi_target = None        # 当前下发目标航向
        self.psi_lock = None          # 锚定航向（对准基准）
        self._gate_frames = 0         # 连续见门计数
        self._ok_since = None         # |e|<死区 连续起始时刻
        self._last_seen = None        # 最后一拍有效观测时刻（丢帧计时基准）
        self._blind_t0 = None         # 盲跑起始时刻
        self._wait_warned = False     # 等门超时只告警一次
        self._t_prev = None           # 上一拍时刻(算 dt)
        self._last_sway = 0.0          # 初次进ALIGN后即丢帧，也能保持零横移。

    # ------------------------------------------------------------ 内部
    def _log(self, msg):
        if self.log:
            self.log('[gate] ' + msg)

    def _tel_yaw(self):
        try:
            t = self.tel() or {}
        except Exception:
            t = {}
        try:
            return float(t['yaw']) if t.get('yaw') is not None else None
        except (TypeError, ValueError):
            return None

    def _tel_omega(self):
        try:
            t = self.tel() or {}
        except Exception:
            t = {}
        try:
            return float(t['omega']) if t.get('omega') is not None else 0.0
        except (TypeError, ValueError):
            return 0.0

    def _set_state(self, st):
        if st != self.state:
            self._log('state %s -> %s' % (self.state, st))
            self.state = st

    # ------------------------------------------------------------ 主流程
    def tick(self, now=None, obs=None):
        """每拍调用一次（20Hz）。obs 可外部注入（回放/仿真），缺省轮询视觉。"""
        if now is None:
            now = time.time()
        dt = 0.0 if self._t_prev is None else max(0.0, now - self._t_prev)
        self._t_prev = now
        cfg = self.cfg

        if self.state == ST_DONE:
            self.send(self.psi_target, 0.0, 0.0)
            return

        if self.state == ST_BLIND:
            # 视觉整块停用: 不读 det/viskf, ψ_target 冻结
            self.send(self.psi_target, cfg['GATE_SURGE'], 0.0)
            if now - self._blind_t0 >= cfg['GATE_T_BLIND']:
                self.send(self.psi_target, 0.0, 0.0)   # 出口留桩: 过门判定后接这里
                self._log('blind done (%.1fs), surge=0' % cfg['GATE_T_BLIND'])
                self._set_state(ST_DONE)
            return

        # ---- WAIT / ALIGN: 需要视觉 ----
        if obs is None:
            obs = self.vision.poll(now)
        e_f = None
        if obs is not None:
            e_f, ok = self.filt.update(obs['e_raw'], now)
            if not ok:
                e_f = None                              # viskf 不可用视同丢帧

        if self.state == ST_WAIT:
            yaw = self._tel_yaw()
            hold = self.psi_target if self.psi_target is not None else \
                (yaw if yaw is not None else 0.0)
            self.psi_target = wrap180(hold)
            self.send(self.psi_target, 0.0, 0.0)

            if e_f is None:
                self._gate_frames = 0
                if (self._wait_warned is False and
                        self._last_seen is not None and
                        now - self._last_seen > cfg['GATE_WAIT_TIMEOUT']):
                    self._log('no gate for %.0fs, keep holding' %
                              cfg['GATE_WAIT_TIMEOUT'])
                    self._wait_warned = True
                return
            self._last_seen = now
            self._wait_warned = False
            self._gate_frames += 1
            if self._gate_frames >= cfg['GATE_LOCK_FRAMES']:
                self.pid.reset()
                self._ok_since = None
                yaw = self._tel_yaw()
                if yaw is not None:
                    self.psi_lock = wrap180(yaw)        # 锚定当前实际航向
                self._set_state(ST_ALIGN)
            return

        # ---- ALIGN ----
        if e_f is None:
            # 丢帧冻结: ψ_target/sway 保持最后一拍, surge=0; 超时回等门
            self.send(self.psi_target, 0.0, self._last_sway)
            if (self._last_seen is not None and
                    now - self._last_seen > cfg['GATE_LOST_WAIT']):
                self._log('lost > %.1fs, back to WAIT' % cfg['GATE_LOST_WAIT'])
                self._set_state(ST_WAIT)
                self._gate_frames = 0
            return

        self._last_seen = now
        self._last_e = e_f
        self._last_w_ratio = obs.get('w_ratio', 0.0)

        # 锚定: 死区内用当前实际航向持续刷新, 保证 e→0 时 ψtar 收敛到真实对中航向
        yaw = self._tel_yaw()
        if abs(e_f) < cfg['GATE_E_DEAD'] and yaw is not None:
            self.psi_lock = wrap180(yaw)

        # 位置式 PD → 绝对目标航向
        omega = self._tel_omega() * cfg['GATE_GYRO_SIGN']
        dpsi, sway = self.pid.step(e_f, omega, dt)
        self.psi_target = wrap180((self.psi_lock or 0.0) + dpsi)
        self._last_sway = sway
        self.send(self.psi_target, 0.0, sway)

        # 对准达标: 死区连续保持 T_ALIGN → 锁航向关视觉进盲跑
        if abs(e_f) < cfg['GATE_E_DEAD']:
            if self._ok_since is None:
                self._ok_since = now
            elif now - self._ok_since >= cfg['GATE_T_ALIGN']:
                if not self.allow_blind:
                    self._ok_since = now        # 回放模式: 保持在 ALIGN 持续观察
                else:
                    self._blind_t0 = now
                    self._log('aligned (e=%.3f), vision OFF, blind run' % e_f)
                    self._set_state(ST_BLIND)
        else:
            self._ok_since = None

    # ------------------------------------------------------------ 供日志/仿真
    def snapshot(self):
        """当前内部状态, 供外部按拍落盘/仿真评价。"""
        return {'state': self.state, 'psi_target': self.psi_target,
                'psi_lock': self.psi_lock, 'e': getattr(self, '_last_e', None),
                'w_ratio': getattr(self, '_last_w_ratio', None),
                'sway': getattr(self, '_last_sway', 0.0)}

    _last_sway = 0.0
    _last_e = None
    _last_w_ratio = None
