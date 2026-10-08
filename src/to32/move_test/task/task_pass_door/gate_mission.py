#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_mission.py — 过门任务状态机（5 态, quad_cv过门落地方案_2026-10-08.md §一）

状态:
    SCAN     找门: 悬停 + yaw ±GATE_SCAN_AMP 来回扫（端点驻留等 YOLO 确认）。
             ★ 用户口径 2026-10-08: 过门启动时门不在视野里, 第一步是主动找门不是等门。
    APPROACH 远·仅 YOLO: surge 前进逼近 + GatePid 追框心; 出口=屏占比≥W_FAR 且 CV 就绪。
    FINE     近·CV 主导: surge=0, Δψ = GatePid(e_f) + PSI_SIGN·KP_PSI·psi_f 并联;
             e 源已由适配层升级为四角交点（lvl>=2）; 回正判定 → 锁航向关视觉 → BLIND。
    BLIND    盲冲: 视觉整块停用, ψ_target 冻结, surge 前进, 航向交固件航向环。
    DONE     本期终点: 保持航向, surge=0。

丢帧/丢门规则:
  * 丢帧（LOST_WAIT 内）: 冻结上一拍位置——ψ_target/sway 保持, surge=0, 无缝续;
  * 丢门超 LOST_WAIT: 回 SCAN 重扫（扫心=当前航向）——门丢了就该找;
  * GATE_WAIT_TIMEOUT 一直没门: 只告警继续扫, 不自行处置（v4 精神）。

psi 通道（仅 FINE 参与控制）:
  * 测量: lvl=4 才有 psi（quad_cv_det 主路 psi_src='edge', 绝不出方向可疑的 ψ）;
  * 平滑: vservo.KF1D（3σ 新息门限防野值）; vservo 不可用时退一阶 EMA;
  * 可信: 距上次 lvl=4 ≤ HOLD_S 且近 1s 内 lvl=4 拍数 ≥ CV4_MIN 且 KF trust;
  * 降级: FINE 中连续 CV_LOST_BACK_S 无 lvl=4 → 退回 v4 纯 bbox 判据（行为不劣于现状）。

外部接口（全部注入, 本包不碰串口/遥测文件）:
    tel()  -> {'yaw': 实际航向° or None, 'omega': 陀螺仪z轴角速度°/s or None}
    send(psi_target, surge, sway)   # 每拍运动输出
    log(msg)                        # 状态迁移/告警, 可为 None
    tick(now, obs=None)             # obs 可注入（回放/仿真）, 缺省轮询 vision
"""
import time

from gate_pid import GatePid, wrap180

try:
    from vservo import KF1D
except Exception:                                    # task/ 不在 sys.path 时（回放等）
    KF1D = None

ST_SCAN = 'SCAN'
ST_APPROACH = 'APPROACH'
ST_FINE = 'FINE'
ST_BLIND = 'BLIND'
ST_DONE = 'DONE'
# 兼容旧引用（replay_gate_a 等打印状态名用）
ST_WAIT = ST_SCAN
ST_ALIGN = ST_FINE


class GateMission(object):

    def __init__(self, cfg, vision, efilter, pid, tel, send, log=None,
                 allow_blind=True):
        self.cfg = cfg
        self.vision = vision          # poll(now) -> obs dict | None
        self.filt = efilter           # EFilter.update(e_raw, now) -> (e_f, ok)
        self.pid = pid                # GatePid
        self.tel = tel
        self.send = send
        self.log = log
        self.allow_blind = bool(allow_blind)   # False=只跑对准环（开环回放用）

        self.state = ST_SCAN
        self.psi_target = None        # 当前下发目标航向
        self.psi_lock = None          # 锚定航向（对准基准）
        self._gate_frames = 0         # 连续见门计数
        self._ok_since = None         # 回正条件连续保持起始时刻
        self._last_seen = None        # 最后一拍有效观测时刻
        self._blind_t0 = None
        self._scan_warned = False
        self._t_prev = None
        self._last_sway = 0.0

        # ---- SCAN 扫描（渐进增幅）----
        self._scan_center = None      # 扫心 = 进态时实际航向（锁存）
        self._scan_side = 1.0
        self._scan_amp = None         # 当前扫描半幅角（由 START 起, 每折返 +STEP, 封顶 AMP）
        self._scan_tgt = None
        self._dwell_until = None
        self._scan_t0 = None

        # ---- psi 平滑 / 可信 / 降级 ----
        self._psi_f = None            # 滤波后 psi
        self._psi_t = None            # 最近一次 lvl=4 时刻
        self._cv4_times = []          # 近 1s 内 lvl=4 拍时刻（可信度窗口）
        self._cv4_gap_t0 = None       # FINE 中 lvl=4 断流起点（降级计时）
        if KF1D is not None:
            self._psi_kf = KF1D(float(cfg.get('GATE_PSI_KF_R', 0.04)),
                                float(cfg.get('GATE_PSI_KF_Q', 0.5)),
                                gate_nsigma=3.0, reset_n=5, trust_age_s=0.2)
        else:
            self._psi_kf = None
        self._cv_ready = 0            # APPROACH 中连续 lvl>=3 计数

        # ---- 供日志/快照 ----
        self._last_e = None
        self._last_w_ratio = None
        self._last_lvl = 0
        self._last_psi_used = None

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

    def _reset_cv_state(self):
        """回 SCAN 时清 psi/CV 状态——旧目标的 psi 不许带到新目标。"""
        self._psi_f = None
        self._psi_t = None
        self._cv4_times = []
        self._cv4_gap_t0 = None
        self._cv_ready = 0
        if self._psi_kf is not None:
            self._psi_kf.reset()
        self._last_psi_used = None

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
            # 视觉整块停用: 不读 det/CV, ψ_target 冻结
            self.send(self.psi_target, cfg['GATE_SURGE'], 0.0)
            if now - self._blind_t0 >= cfg['GATE_T_BLIND']:
                self.send(self.psi_target, 0.0, 0.0)   # 出口留桩: 过门判定后接这里
                self._log('blind done (%.1fs), surge=0' % cfg['GATE_T_BLIND'])
                self._set_state(ST_DONE)
            return

        # ---- SCAN / APPROACH / FINE: 需要视觉 ----
        if obs is None:
            obs = self.vision.poll(now)
        e_f = None
        e_raw = None
        if obs is not None:
            try:
                e_raw = float(obs['e_raw'])
            except (TypeError, ValueError, KeyError):
                e_raw = None
        if e_raw is not None:
            e_f, ok = self.filt.update(e_raw, now)
            if not ok:
                e_f = None                              # viskf 不可用视同丢帧
        lvl = int((obs or {}).get('cv_lvl') or 0)
        psi_raw = (obs or {}).get('psi')
        par_h = (obs or {}).get('par_h')
        w_ratio = (obs or {}).get('w_ratio')
        self._last_lvl = lvl

        # ---- psi 平滑与可信（SCAN/APPROACH 也更新, 进 FINE 即有暖数据）----
        if psi_raw is not None:
            acc = False
            if self._psi_kf is not None:
                acc = bool(self._psi_kf.update(float(psi_raw), now))
                if acc:
                    self._psi_f = float(self._psi_kf.x[0])
            else:
                a = 0.4
                self._psi_f = (float(psi_raw) if self._psi_f is None
                               else self._psi_f + a * (float(psi_raw) - self._psi_f))
                acc = True
            if acc:
                self._psi_t = now
                self._cv4_times.append(now)
        cutoff = now - 1.0
        self._cv4_times = [t for t in self._cv4_times if t >= cutoff]
        psi_trust = (self._psi_f is not None
                     and self._psi_t is not None
                     and (now - self._psi_t) <= float(cfg.get('GATE_PSI_HOLD_S', 1.0))
                     and len(self._cv4_times) >= int(cfg.get('GATE_CV4_MIN', 5)))
        if psi_trust and self._psi_kf is not None:
            psi_trust = bool(self._psi_kf.trust_ok(now, float(cfg.get('GATE_PSI_SIGMA_MAX', 0.25))))

        # =================================================================
        if self.state == ST_SCAN:
            yaw = self._tel_yaw()
            if self._scan_center is None:               # 进态: 锁存扫心
                self._scan_center = (yaw if yaw is not None
                                     else (self.psi_target if self.psi_target is not None else 0.0))
                self._scan_t0 = now
                self._scan_side = 1.0
                self._scan_amp = min(float(cfg['GATE_SCAN_AMP']),
                                     float(cfg.get('GATE_SCAN_AMP_START', 10.0)))
                self._scan_tgt = wrap180(self._scan_center + self._scan_side * self._scan_amp)
                self._dwell_until = None
                self._log('scan: center=%.1f amp %.0f->%.0f step %.0f'
                          % (self._scan_center, self._scan_amp, cfg['GATE_SCAN_AMP'],
                             cfg.get('GATE_SCAN_AMP_STEP', 5.0)))
            if self._dwell_until is None:               # 转向中: 等到位
                if yaw is not None and abs(wrap180(yaw - self._scan_tgt)) <= float(cfg['GATE_SCAN_TOL']):
                    self._dwell_until = now + float(cfg['GATE_SCAN_DWELL'])
            else:                                       # 驻留: 计时到折返, 幅角缓慢加大
                if now >= self._dwell_until:
                    self._scan_side = -self._scan_side
                    self._scan_amp = min(float(cfg['GATE_SCAN_AMP']),
                                         self._scan_amp + float(cfg.get('GATE_SCAN_AMP_STEP', 5.0)))
                    self._scan_tgt = wrap180(self._scan_center + self._scan_side * self._scan_amp)
                    self._dwell_until = None
            self.psi_target = self._scan_tgt
            self.send(self.psi_target, 0.0, 0.0)

            if (not self._scan_warned and self._scan_t0 is not None
                    and now - self._scan_t0 > float(cfg['GATE_WAIT_TIMEOUT'])):
                self._log('no gate for %.0fs, keep scanning' % cfg['GATE_WAIT_TIMEOUT'])
                self._scan_warned = True

            if e_f is not None:
                self._last_seen = now
                self._scan_warned = False
                self._gate_frames += 1
                if self._gate_frames >= int(cfg['GATE_LOCK_FRAMES']):
                    self.pid.reset()
                    self._ok_since = None
                    self._cv_ready = 0
                    yaw = self._tel_yaw()
                    self.psi_lock = wrap180(yaw if yaw is not None else self.psi_target)
                    self._set_state(ST_APPROACH)
            else:
                self._gate_frames = 0
            return

        # =================================================================
        if self.state == ST_APPROACH:
            if e_f is None:
                # 丢帧冻结: surge=0; 丢门超时回 SCAN 重扫
                self.send(self.psi_target, 0.0, self._last_sway)
                if (self._last_seen is not None and
                        now - self._last_seen > float(cfg['GATE_LOST_WAIT'])):
                    self._log('lost > %.1fs, back to SCAN' % cfg['GATE_LOST_WAIT'])
                    self._gate_frames = 0
                    self._reset_cv_state()
                    self._scan_center = None
                    self._set_state(ST_SCAN)
                return

            self._last_seen = now
            self._last_e = e_f
            self._last_w_ratio = w_ratio
            omega = self._tel_omega() * cfg['GATE_GYRO_SIGN']
            dpsi, sway = self.pid.step(e_f, omega, dt)
            self.psi_target = wrap180((self.psi_lock or 0.0) + dpsi)
            self._last_sway = sway
            self.send(self.psi_target, cfg['GATE_SURGE_FAR'], sway)

            # CV 就绪计数（连续 lvl>=3）
            self._cv_ready = self._cv_ready + 1 if lvl >= 3 else 0
            near = (w_ratio is not None and w_ratio >= float(cfg['GATE_W_BLIND']))
            if (w_ratio is not None and w_ratio >= float(cfg['GATE_W_FAR'])
                    and self._cv_ready >= int(cfg['GATE_CV_LOCK_FRAMES'])):
                self.pid.reset()
                self._ok_since = None
                self._cv4_gap_t0 = None
                self._set_state(ST_FINE)
            elif near:
                # 兜底: CV 一直不就绪但已很近 → 纯 bbox 进 S2（S2 内走 v4 降级判据）
                self._log('cv not ready, near enough (w=%.2f), bbox-only FINE'
                          % w_ratio)
                self.pid.reset()
                self._ok_since = None
                self._cv4_gap_t0 = None
                self._set_state(ST_FINE)
            return

        # =================================================================
        # ST_FINE — 近·CV 主导
        if e_f is None:
            self.send(self.psi_target, 0.0, self._last_sway)
            if (self._last_seen is not None and
                    now - self._last_seen > float(cfg['GATE_LOST_WAIT'])):
                self._log('lost > %.1fs, back to SCAN' % cfg['GATE_LOST_WAIT'])
                self._gate_frames = 0
                self._reset_cv_state()
                self._scan_center = None
                self._set_state(ST_SCAN)
            return

        self._last_seen = now
        self._last_e = e_f
        self._last_w_ratio = w_ratio

        # CV 断流降级计时: FINE 中连续无 lvl=4 超时 → 退 v4 纯 bbox 判据
        if psi_raw is not None:
            self._cv4_gap_t0 = None
        elif self._cv4_gap_t0 is None:
            self._cv4_gap_t0 = now
        degraded = (self._cv4_gap_t0 is not None and
                    now - self._cv4_gap_t0 > float(cfg.get('GATE_CV_LOST_BACK_S', 6.0)))

        # 锚定: 死区内用当前实际航向持续刷新（v4 口径不变）
        yaw = self._tel_yaw()
        if abs(e_f) < cfg['GATE_E_DEAD'] and yaw is not None:
            self.psi_lock = wrap180(yaw)

        # 位置式 PD（e 环）+ psi 正对项并联
        omega = self._tel_omega() * cfg['GATE_GYRO_SIGN']
        dpsi, sway = self.pid.step(e_f, omega, dt)
        psi_term = 0.0
        if psi_trust and not degraded:
            psi_term = float(cfg['GATE_PSI_SIGN']) * float(cfg['GATE_PSI_KP']) * self._psi_f
        self._last_psi_used = self._psi_f if (psi_trust and not degraded) else None
        self.psi_target = wrap180((self.psi_lock or 0.0) + dpsi + psi_term)
        self._last_sway = sway
        self.send(self.psi_target, 0.0, sway)

        # 回正判定（v4 口径 + psi 正对项; ★ surge=0 不接近, 不设 w_ratio 门槛——
        #   盲冲距离由 GATE_T_BLIND 覆盖, 真过门出口二期接 out_frac）
        ok_e = abs(e_f) < float(cfg['GATE_E_DEAD'])
        if degraded or not psi_trust:
            aligned = ok_e                              # v4 降级口径: 只看居中
        else:
            ok_psi = abs(self._psi_f) <= float(cfg['GATE_PSI_DEAD'])
            ok_fresh = (self._psi_t is not None and
                        now - self._psi_t <= float(cfg['GATE_PSI_HOLD_S']))
            ok_rect = (par_h is not None and
                       abs(float(par_h)) <= float(cfg.get('GATE_RECT_PAR_MAX', 8.0)))
            aligned = ok_e and ok_psi and ok_fresh and ok_rect

        if aligned:
            if self._ok_since is None:
                self._ok_since = now
            elif now - self._ok_since >= float(cfg['GATE_T_ALIGN']):
                if not self.allow_blind:
                    self._ok_since = now        # 回放模式: 保持在 FINE 持续观察
                else:
                    self._blind_t0 = now
                    self._log('aligned (e=%.3f psi=%s), vision OFF, blind run'
                              % (e_f, ('%.3f' % self._psi_f) if self._psi_f is not None else 'n/a'))
                    self._set_state(ST_BLIND)
        else:
            self._ok_since = None

    # ------------------------------------------------------------ 供日志/仿真
    def snapshot(self):
        """当前内部状态, 供外部按拍落盘/仿真评价。"""
        return {'state': self.state, 'psi_target': self.psi_target,
                'psi_lock': self.psi_lock, 'e': self._last_e,
                'w_ratio': self._last_w_ratio, 'sway': self._last_sway,
                'cv_lvl': self._last_lvl, 'psi_f': self._psi_f,
                'psi_used': self._last_psi_used}
