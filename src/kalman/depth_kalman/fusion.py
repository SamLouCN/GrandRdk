# -*- coding: utf-8 -*-
"""一拍编排：装配 → 预测 → 逐路观测更新 → 门限/夹紧/发散重置 → 组包输出。

纯计算 + 调 sinks 落盘由 main.py 负责；本模块不含任何 I/O 与 sleep，
因此可以直接喂合成数据做自检。
"""
import math
import time

import numpy as np

from kf import EKF
from model import Model


class DepthFusion(object):
    def __init__(self, cfg, log=None):
        self.cfg = cfg
        self.log = log
        self.model = Model(cfg)
        self.kf = EKF(self.model.initial_state(), self.model.initial_covar())

        # 锚路：H 的状态值是该路的「有效池底深度」（已折进该路 dz）
        self.anchor = None
        self._pick_anchor()

        self.t_last = None
        self.t_start = None
        self.inited = False
        self.last_depth_obs = None

        self.tok_telem = None
        self.tok_alt = {}

        self.cnt = {'pred': 0, 'depth': 0, 'alt': 0,
                    'rej_depth': 0, 'rej_alt': 0, 'sat': 0, 'reset': 0, 'inflate': 0,
                    'accel_used': 0, 'alt_by_ch': {}, 'alt_rej_by_ch': {},
                    'rej_range': 0, 'rej_tilt': 0}
        self.rej_streak = {}
        self.gate_var = {}          # 每路新息的 EWMA(ν²)，自适应门限的尺度来源
        self.last_tel = None
        self.last_alt = {}
        self._loop_t = []

    # ------------------------------------------------------------------ 锚路
    def _pick_anchor(self):
        explicit = str(getattr(self.cfg, 'ALT_ANCHOR', '') or '').strip().upper()
        channels = [c.upper() for c in getattr(self.cfg, 'ALT_CHANNELS', []) or []]
        if explicit and explicit in channels:
            self.anchor = explicit
        elif channels:
            self.anchor = channels[0]
        else:
            self.anchor = None
        self.model.set_anchor(self.anchor)

    # ------------------------------------------------------------------ 主步
    def step(self, ts, tel=None, alt=None):
        cfg = self.cfg
        alt = alt or {}
        if tel is not None:
            self.last_tel = tel
        else:
            tel = self.last_tel
        for ch, s in alt.items():
            if s is not None and not s.stale:
                self.last_alt[ch] = s
        if self.t_start is None:
            self.t_start = ts
        self._loop_t.append(ts)
        if len(self._loop_t) > 200:
            del self._loop_t[:-200]

        dt = 0.0
        if self.t_last is not None:
            dt = ts - self.t_last
            if dt < 0.0:
                dt = 0.0
            elif dt > float(cfg.MAX_DT):
                dt = float(cfg.MAX_DT)
        self.t_last = ts

        # ---------------------------------------------------------- 1. 预测
        accel_used = False
        level_assumed = False
        if dt > 0.0:
            F, Q = self.model.predict_matrices(dt)
            self.kf.predict(F, Q)
            a_dn, used, level_assumed = self.model.accel_down(tel)
            if used:
                iD, iV = self.model['D'], self.model['v_z']
                self.kf.x[iD] += 0.5 * dt * dt * a_dn
                self.kf.x[iV] += dt * a_dn
                accel_used = True
                self.cnt['accel_used'] += 1
                # 注：加速度不精确带来的不确定度已经在 Q 里（Model.predict_matrices
                # 的 G·σ_a²·Gᵀ 项），这里不要再手工往 P 里加东西。
            self.cnt['pred'] += 1

        # ---------------------------------------------------------- 2. 深度计
        depth_sat = False
        if tel is not None and not tel.stale and tel.token is not None \
                and tel.token != self.tok_telem:
            self.tok_telem = tel.token
            if tel.depth_m is None and tel.depth_raw is not None:
                depth_sat = True
                self.cnt['sat'] += 1          # 固件把负值钳成 0 → 近水面饱和，丢弃
            elif tel.depth_m is not None:
                self._update_depth(tel.depth_m)

        # ---------------------------------------------------------- 3. 高度计
        alt_ok = []
        for ch in self._channels():
            s = alt.get(ch) or self.last_alt.get(ch)
            if s is None or s.stale or not s.ok or s.clearance_m is None:
                continue
            if self.tok_alt.get(ch) == s.token:
                continue                      # 同一帧不重复吃
            self.tok_alt[ch] = s.token
            if self._update_alt(ch, s, tel):
                alt_ok.append(ch)
        self.cnt['alt_by_ch'] = dict((k, v) for k, v in self.cnt['alt_by_ch'].items())

        # ---------------------------------------------------------- 4. 保护
        self.kf.clamp(*self.model.clamp_bounds())
        self._check_divergence(tel, alt_ok)

        # ---------------------------------------------------------- 5. 组包
        return self.snapshot(ts, tel, alt, alt_ok, accel_used, level_assumed, depth_sat)

    # ------------------------------------------------------------------ 观测
    def _gate_ok(self, key, nu, S):
        """自适应新息门限。返回 True = 放行。

        尺度 = ``sqrt(EWMA(ν²) + S + GATING_FLOOR²)``；``EWMA(ν²)`` 只被**已接受**的样本更新。
        这样"持续偏差"会把尺度一起抬高（正常放行），"突发野值"则远超尺度被砍掉。
        """
        cfg = self.cfg
        ss = float(getattr(cfg, 'GATING_FLOOR', 0.002)) ** 2
        var = self.gate_var.get(key, 0.0)
        scale = math.sqrt(max(0.0, var) + max(0.0, S) + ss)
        return abs(nu) <= float(cfg.GATING_N_SIGMA) * scale

    def _gate_update(self, key, nu, accepted):
        if not accepted:
            return
        a = float(getattr(self.cfg, 'GATING_ADAPT_ALPHA', 0.1))
        cur = self.gate_var.get(key, 0.0)
        self.gate_var[key] = (1.0 - a) * cur + a * nu * nu

    def _update_depth(self, depth_m):
        cfg = self.cfg
        iD = self.model['D']
        if not self.inited and bool(getattr(cfg, 'INIT_FROM_FIRST_OBS', True)):
            self.kf.x[iD] = float(depth_m)
            self.inited = True
        h, off = self.model.obs_depth()
        z = float(depth_m) - off
        r = float(cfg.R_DEPTH) ** 2
        nu, S = self.kf.innovate(h, z, r)
        accepted = False
        if self._gate_ok('depth', nu, S):
            accepted, _, _ = self.kf.update_scalar(h, z, r, gate=None)
        self._gate_update('depth', nu, accepted)
        if accepted:
            self.cnt['depth'] += 1
            self.inited = True
            self.last_depth_obs = float(depth_m)
            self.rej_streak['depth'] = 0
        else:
            self.cnt['rej_depth'] += 1
            self.rej_streak['depth'] = self.rej_streak.get('depth', 0) + 1
            self._inflate_if_stuck('depth', self.rej_streak['depth'])

    def _update_alt(self, ch, sample, tel):
        cfg = self.cfg
        iD = self.model['D']
        theta = (tel.pitch_rad or 0.0) if (tel is not None and tel.has_att) else 0.0
        phi = (tel.roll_rad or 0.0) if (tel is not None and tel.has_att) else 0.0

        # ---------- 失效硬拒（2026-10-06）：在自适应门限**之前**，且**不喂** P 放大看门狗。
        # 传感器失效（读数变 -3 / 超程大数 / 波束入射角过大）不是「P 过度乐观」——
        # 若让它累积 rej_streak 触发 _inflate_if_stuck，等于把垃圾观测放大 P 后放进来。
        # 深度计的偶发跳变不在这里防：它由上面的自适应新息门限（EWMA 尺度）兜住。
        c_m = float(sample.clearance_m)
        if not (float(getattr(cfg, 'ALT_MIN_VALID_M', 0.02)) <= c_m
                <= float(getattr(cfg, 'ALT_MAX_VALID_M', 2.5))):
            self.cnt['rej_alt'] += 1
            self.cnt['rej_range'] = self.cnt.get('rej_range', 0) + 1
            return False
        tilt_max = float(getattr(cfg, 'ALT_MAX_TILT_DEG', 30.0))
        if tilt_max > 0.0 and tel is not None and tel.has_att \
                and max(abs(theta), abs(phi)) > math.radians(tilt_max):
            self.cnt['rej_alt'] += 1
            self.cnt['rej_tilt'] = self.cnt.get('rej_tilt', 0) + 1
            return False

        h, off, sigma = self.model.obs_alt(ch, theta, phi, sample.clearance_m)
        z = float(sample.clearance_m) - off
        r = sigma ** 2
        if not self.inited and bool(getattr(cfg, 'INIT_FROM_FIRST_OBS', True)):
            # 用高度计反推深度初值：D = H_eff − clearance
            self.kf.x[iD] = float(self.kf.x[self.model['H']]) - float(sample.clearance_m)
            self.inited = True
        nu, S = self.kf.innovate(h, z, r)
        accepted = False
        if self._gate_ok('alt_' + ch, nu, S):
            accepted, _, _ = self.kf.update_scalar(h, z, r, gate=None)
        self._gate_update('alt_' + ch, nu, accepted)
        if accepted:
            self.cnt['alt'] += 1
            self.cnt['alt_by_ch'][ch] = self.cnt['alt_by_ch'].get(ch, 0) + 1
            self.rej_streak['alt_' + ch] = 0
        else:
            self.cnt['rej_alt'] += 1
            self.cnt['alt_rej_by_ch'][ch] = self.cnt['alt_rej_by_ch'].get(ch, 0) + 1
            k = 'alt_' + ch
            self.rej_streak[k] = self.rej_streak.get(k, 0) + 1
            self._inflate_if_stuck(k, self.rej_streak[k])
        return accepted

    # ------------------------------------------------------------------ 门限看门狗
    def _inflate_if_stuck(self, key, streak):
        """同一路**连续**被拒很多次 → 不是野值，是滤波器自己的 P 过度乐观。

        做法：把 P_D（必要时连 P_H）放大 ``GATING_INFLATE_P`` 倍（σ 扩 3 倍），
        把门限撑开让正确观测重新进来。比"死等 P 自己长回来"可靠得多。
        """
        cfg = self.cfg
        limit = int(getattr(cfg, 'GATING_MAX_CONSEC_REJ', 25))
        if limit <= 0 or streak < limit:
            return
        f = float(getattr(cfg, 'GATING_INFLATE_P', 9.0))
        iD = self.model['D']
        self.kf.P[iD, iD] *= f
        for i in range(self.kf.n):                      # 同步放大 D 与其它状态的交叉项
            if i != iD:
                self.kf.P[iD, i] *= math.sqrt(f)
                self.kf.P[i, iD] *= math.sqrt(f)
        self.kf.P = 0.5 * (self.kf.P + self.kf.P.T)
        self.cnt['inflate'] = self.cnt.get('inflate', 0) + 1
        self.rej_streak[key] = 0
        if self.log is not None:
            self.log.warn('「%s」连续被拒 %d 次 → 判定 P 过度乐观，P_D 放大 %.0f 倍'
                          '（第 %d 次）' % (key, streak, f, self.cnt['inflate']))

    # ------------------------------------------------------------------ 保护
    def _check_divergence(self, tel, alt_ok):
        cfg = self.cfg
        iD = self.model['D']
        bad = (self.kf.sigma(iD) > float(cfg.RESET_SIGMA_D)
               or self.kf.trace() > float(cfg.RESET_TRACE_P))
        if not bad:
            return
        d_ref = self.last_depth_obs
        if d_ref is None and alt_ok:
            ch = alt_ok[0]
            s = self.last_alt.get(ch)
            if s is not None and s.clearance_m is not None:
                d_ref = float(self.kf.x[self.model['H']]) - float(s.clearance_m)
        if d_ref is None:
            if not self.inited:
                # 还没有任何观测 → 协方差只是白白长大（长时间无数据会涨到天上）。
                # 收敛回先验即可，等第一条观测进来再说。
                self.kf.reset(self.kf.x, self.model.initial_covar())
            return
        self.cnt['reset'] += 1
        x = self.kf.x.copy()
        x[iD] = d_ref
        x[self.model['v_z']] = 0.0
        P = self.model.initial_covar()
        self.kf.reset(x, P)
        if self.log is not None:
            self.log.warn('协方差发散，已重置：D ← %.3f m（累计 %d 次）'
                          % (d_ref, self.cnt['reset']))

    # ------------------------------------------------------------------ 组包
    def _channels(self):
        return [c.upper() for c in getattr(self.cfg, 'ALT_CHANNELS', []) or []]

    def snapshot(self, ts, tel, alt, alt_ok, accel_used, level_assumed, depth_sat):
        cfg = self.cfg
        idx = self.model.idx
        x = self.kf.x
        theta = (tel.pitch_rad or 0.0) if (tel is not None and tel.has_att) else 0.0
        phi = (tel.roll_rad or 0.0) if (tel is not None and tel.has_att) else 0.0

        sigma = dict((n, round(self.kf.sigma(i), 5)) for n, i in idx.items())
        clearance = {}
        for ch in self._channels():
            clearance[ch] = round(self.model.clearance_of(x, ch, theta, phi), 4)

        alt_age = {}
        for ch in self._channels():
            s = alt.get(ch) or self.last_alt.get(ch)
            alt_age[ch] = (round(s.age, 3) if s is not None and s.age != float('inf') else None)

        fps = 0.0
        if len(self._loop_t) >= 2:
            span = self._loop_t[-1] - self._loop_t[0]
            if span > 0:
                fps = round((len(self._loop_t) - 1) / span, 1)

        val = dict((n, round(float(x[i]), 5)) for n, i in idx.items())
        return {
            'ts': round(float(ts), 4),
            'valid': bool(self.inited),
            'D': val['D'],
            'v_z': val['v_z'],
            'b_d': val.get('b_d'),
            'b_a': val.get('b_a'),
            'H': val['H'],
            'sigma': sigma,
            'clearance': clearance,
            'anchor': self.anchor,
            'counters': {
                'depth': self.cnt['depth'], 'alt': self.cnt['alt'],
                'rej_depth': self.cnt['rej_depth'], 'rej_alt': self.cnt['rej_alt'],
                'rej_range': self.cnt.get('rej_range', 0),
                'rej_tilt': self.cnt.get('rej_tilt', 0),
                'sat': self.cnt['sat'], 'reset': self.cnt['reset'],
                'inflate': self.cnt.get('inflate', 0),
                'alt_by_ch': dict(self.cnt['alt_by_ch']),
            },
            'sources': {
                'telem_age': (round(tel.age, 3) if tel is not None and tel.age != float('inf') else None),
                'telem_stale': bool(tel is None or tel.stale),
                'alt_age': alt_age,
                'alt_ok': list(alt_ok),
            },
            'flags': {
                'degraded': len(alt_ok) == 0,
                'accel_used': bool(accel_used),
                'level_assumed': bool(level_assumed),
                'depth_sat': bool(depth_sat),
                'h_mode': str(getattr(cfg, 'H_MODE', 'known')),
                'ba_enabled': bool(self.model.enable_ba),
                'bd_enabled': bool(self.model.enable_bd),
                'accel_enabled': bool(getattr(cfg, 'ACCEL_ENABLED', False)),
            },
            'stats': {'loop_hz': fps, 'steps': self.cnt['pred']},
        }
