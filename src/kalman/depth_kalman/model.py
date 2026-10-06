# -*- coding: utf-8 -*-
"""状态与观测模型：预测矩阵、观测方程、姿态扣重力、量纲换算。

状态
----
``x = [D, v_z, (b_a), H]``

===========  ==================================  ================
分量          含义                                单位
===========  ==================================  ================
``D``        深度（水面向下为正）                  m
``v_z``      垂向速度（向下为正）                  m/s
``b_a``      垂向加速度零偏（可选）                m/s²
``H``        **锚路**的池底深度（已折进该路 dz）   m
===========  ==================================  ================

``b_a`` 在 ``ENABLE_BA=False`` 或 ``ACCEL_ENABLED=False`` 时**不进状态**（维度自动退化）。

观测方程
--------
* O1 深度计：``z = D``
* O2 高度计 i：``z = (H - dz_i) - D - PITCH_SIGN*y_i*θ + ROLL_SIGN*x_i*φ``
  （2026-10-06 修正耦合配对：前向偏移 y 耦合**俯仰** θ，右向偏移 x 耦合**横滚** φ。
  旧式 x↔θ、y↔φ 配对反了；x=y=0 时两者等价，填入真实安装偏移后必须用本式。）
* O3 加速度：只进**预测**，不做观测
"""
import math

import numpy as np

G = 9.80665

STATUS_OK = ('ok',)


class Model(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.enable_ba = bool(getattr(cfg, 'ENABLE_BA', False)) and bool(getattr(cfg, 'ACCEL_ENABLED', False))
        # b_d（深度计零偏）**只在有独立绝对深度基准时才可观**：
        #   高度计在场 + H_MODE='known'。否则 D 与 b_d 完全不可分辨（两个都加减同一个常数
        #   对任何观测量都没影响），b_d 会把真实机动吃掉。
        #   注意：方案文档里"不要给深度计设零偏"的前提是 **H 当随机游走**；H 已知时前提不成立。
        self.enable_bd = (bool(getattr(cfg, 'ENABLE_BD', False))
                          and bool(getattr(cfg, 'ALT_CHANNELS', None))
                          and str(getattr(cfg, 'H_MODE', 'known')) == 'known')
        names = ['D', 'v_z']
        if self.enable_bd:
            names.append('b_d')
        if self.enable_ba:
            names.append('b_a')
        names.append('H')
        self.names = names
        self.idx = dict((n, i) for i, n in enumerate(names))
        self.n = len(names)
        # 锚路：H 的状态值 = 该路的「有效池底深度」 H_eff_anchor = H_true − dz_anchor，
        # 其它路按 H_eff_i = H_eff_anchor + (dz_anchor − dz_i) 换算。
        self.anchor = None
        self.anchor_dz = 0.0

    def set_anchor(self, ch):
        """设置锚路（决定 H 状态的参考零点）。``None`` 表示无锚路。"""
        self.anchor = ch
        self.anchor_dz = self._mount(ch)[2] if ch else 0.0
        return self.anchor

    # -------------------------------------------------------------- 索引
    def __getitem__(self, name):
        return self.idx[name]

    # -------------------------------------------------------------- 初值
    def initial_state(self, d0=0.0):
        x = np.zeros(self.n)
        x[self.idx['D']] = float(d0)
        x[self.idx['v_z']] = 0.0
        if self.enable_bd:
            x[self.idx['b_d']] = 0.0
        if self.enable_ba:
            x[self.idx['b_a']] = 0.0
        x[self.idx['H']] = float(getattr(self.cfg, 'H_M', 1.0))
        return x

    def initial_covar(self):
        cfg = self.cfg
        P = np.zeros((self.n, self.n))
        P[self.idx['D'], self.idx['D']] = float(cfg.P0_D)
        P[self.idx['v_z'], self.idx['v_z']] = float(cfg.P0_V)
        if self.enable_bd:
            P[self.idx['b_d'], self.idx['b_d']] = float(cfg.P0_BD)
        if self.enable_ba:
            P[self.idx['b_a'], self.idx['b_a']] = float(cfg.P0_BA)
        if getattr(cfg, 'H_MODE', 'known') == 'known':
            P[self.idx['H'], self.idx['H']] = float(cfg.H_SIGMA_KNOWN) ** 2
        else:
            P[self.idx['H'], self.idx['H']] = 0.25
        return P

    # -------------------------------------------------------------- 预测
    def input_sigma(self):
        """未建模垂向加速度的 σ（m/s²）—— 决定 Q 里"未知输入"那一块。

        为什么必须有它：加速度不是精确已知的输入。姿态角有 σ_θ 的误差就会漏进
        ``g·sinσ_θ`` 的假加速度（1° → 0.17 m/s²）；再叠加计噪声与零偏不确定性。
        如果**不**把这块算进 Q，P 会过度乐观 → 新息门限把正确观测全拒掉 →
        滤波器被自己的门限锁死。这是自检实测踩出来的，不是理论洁癖。
        """
        cfg = self.cfg
        if not bool(getattr(cfg, 'ACCEL_ENABLED', False)):
            return float(getattr(cfg, 'SIGMA_A_NO_ACCEL', 0.5))
        s_theta = math.radians(float(getattr(cfg, 'ALT_SIGMA_THETA_DEG', 1.0)))
        leak = G * math.sin(abs(s_theta))
        s2 = float(getattr(cfg, 'ACCEL_SIGMA', 0.05)) ** 2 + leak ** 2
        if not self.enable_ba:
            # b_a 关掉时，零偏完全没人吸收 → 也算进未建模部分
            s2 += float(getattr(cfg, 'ACCEL_BIAS_UNC', 0.10)) ** 2
        return math.sqrt(s2)

    def predict_matrices(self, dt):
        """返回 (F, Q)。

        Q = 对角过程噪声率×dt + ``G·σ_a²·Gᵀ``（G 是加速度输入对状态的增益）——
        后半项把"加速度不精确"如实反映到协方差上。
        """
        cfg = self.cfg
        iD, iV = self.idx['D'], self.idx['v_z']
        F = np.eye(self.n)
        F[iD, iV] = dt
        rates = [cfg.Q_D, cfg.Q_V]
        if self.enable_bd:
            # b_d 用 **OU 过程**（有限带宽的漂移），而不是纯随机游走：
            #   纯随机游走没有时间尺度，滤波器会顺手把深度计的**随机噪声**也吸进 b_d，
            #   降噪效果直接归零（自检实测：C1 的 1.78x 掉到 1.05x）。
            #   OU 的带宽 = 1/(2π·TAU_BD)，1 分钟量级 → 跟得上漂移，跟不上秒级噪声。
            iBD = self.idx['b_d']
            tau = float(getattr(cfg, 'TAU_BD', 60.0))
            if tau > 0.0:
                rho = math.exp(-dt / tau)
                F[iBD, iBD] = rho
                q_bd = float(cfg.SIGMA_BD) ** 2 * (1.0 - rho * rho)
            else:
                q_bd = float(cfg.Q_BD) * dt
            rates.append(q_bd / max(dt, 1e-9))   # 下面统一乘 dt，这里除回去
        if self.enable_ba:
            iB = self.idx['b_a']
            F[iD, iB] = -0.5 * dt * dt     # D ← D + v_z·dt + 0.5·(a_dn - b_a)·dt²
            F[iV, iB] = -dt                # v_z ← v_z + (a_dn - b_a)·dt
            rates.append(cfg.Q_BA)
        rates.append(0.0 if getattr(cfg, 'H_MODE', 'known') == 'known' else cfg.Q_H_RW)
        assert len(rates) == self.n, '过程噪声率顺序与状态顺序不一致'
        Q = np.diag([max(0.0, r) * dt for r in rates])
        g_vec = np.zeros(self.n)
        g_vec[iD] = 0.5 * dt * dt
        g_vec[iV] = dt
        Q = Q + np.outer(g_vec, g_vec) * (self.input_sigma() ** 2)
        return F, Q

    def clamp_bounds(self):
        """返回 (lo, hi)，与状态顺序对齐；None 表示不夹。"""
        cfg = self.cfg
        lo = [cfg.D_MIN, -cfg.V_MAX]
        hi = [cfg.D_MAX, cfg.V_MAX]
        if self.enable_bd:
            lo.append(-cfg.BD_MAX)
            hi.append(cfg.BD_MAX)
        if self.enable_ba:
            lo.append(-cfg.BA_MAX)
            hi.append(cfg.BA_MAX)
        lo.append(None)
        hi.append(None)
        return lo, hi

    # -------------------------------------------------------------- 加速度
    def accel_down(self, tel):
        """世界系垂向**向下为正**的加速度，由比力扣重力得到。

        推导（R = Rz(ψ)·Ry(θ)·Rx(φ)，其第三行 `[-sinθ, cosθ·sinφ, cosθ·cosφ]` 与 ψ 无关）::

            (R·f)_z = -sinθ·f_x + cosθ·sinφ·f_y + cosθ·cosφ·f_z
            a_down  = g + (R·f)_z                       ← 惯性系 a = f + g，g 朝下为正
                    = g - sinθ·f_x + cosθ·sinφ·f_y + cosθ·cosφ·f_z

        自查：水平静止时 f=(0,0,-g) → a_down = g - g = 0 ✓

        ⚠ 方案文档 §3.2 里写的 `a_dn = g + sinθ·f_x − cosθ·sinφ·f_y − cosθ·cosφ·f_z`
        是**错的**（把 a_up 与 (R·f)_z 混了，整体差一个负号）；以本函数为准。

        返回 ``(a_dn, used, level_assumed)``。未启用/数据不可用时返回 ``(0.0, False, ...)``。
        """
        cfg = self.cfg
        if not bool(getattr(cfg, 'ACCEL_ENABLED', False)) or tel is None or tel.acc is None:
            return 0.0, False, False
        fx, fy, fz = tel.acc
        if tel.has_att:
            theta = tel.pitch_rad or 0.0
            phi = tel.roll_rad or 0.0
            level_assumed = False
        else:
            # 没有姿态角时只能假设水平 —— 只对 acc_z 的简化式，倾斜时重力会漏进 x/y。
            theta = phi = 0.0
            level_assumed = True
        a_dn = (G
                - math.sin(theta) * fx
                + math.cos(theta) * math.sin(phi) * fy
                + math.cos(theta) * math.cos(phi) * fz)
        return a_dn, True, level_assumed

    # -------------------------------------------------------------- 观测
    def obs_depth(self):
        """O1：``z = D + b_d``（b_d 关闭时退化为 ``z = D``）。

        ``b_d`` 是深度计的**零偏/慢漂移**。有高度计且 H 已知时它与 D 可分辨
        （高度计给出独立的绝对 D，差值就是 b_d）。
        """
        h = np.zeros(self.n)
        h[self.idx['D']] = 1.0
        if self.enable_bd:
            h[self.idx['b_d']] = 1.0
        return h, 0.0

    def obs_alt(self, ch, theta, phi, clearance_m):
        """O2：高度计。

        被观测量（预测的净空）::

            = H_eff_i - D - PITCH_SIGN*y_i*θ + ROLL_SIGN*x_i*φ
            = (H + anchor_dz - dz_i) - D - PITCH_SIGN*y_i*θ + ROLL_SIGN*x_i*φ

        （``H`` 是**锚路**的有效池底深度，见 ``set_anchor``）

        耦合配对（2026-10-06 修正，推导见 config/depth_config.py 观测方程段）：
        抬头 θ>0 时前向偏移 y 大的探头升高、净空变大 ⇒ y 耦合 θ；
        横滚时右向偏移 x 改变探头高度 ⇒ x 耦合 φ。符号由 PITCH_SIGN/ROLL_SIGN 现场校核。

        返回 ``(h, offset, sigma)``；``sigma`` 已含传感器精度、姿态耦合残差与基础项。
        """
        cfg = self.cfg
        x, y, dz = self._mount(ch)
        c_theta = float(cfg.PITCH_SIGN) * y   # ★ y（前向）耦合俯仰 θ —— 旧版错写成 x
        c_phi = float(cfg.ROLL_SIGN) * x      # ★ x（右向）耦合横滚 φ —— 旧版错写成 y
        h = np.zeros(self.n)
        h[self.idx['D']] = -1.0
        h[self.idx['H']] = 1.0
        offset = (self.anchor_dz - dz) - c_theta * theta + c_phi * phi

        s_sensor = float(cfg.ALT_SIGMA_BASE) + float(cfg.ALT_SIGMA_REL) * abs(clearance_m)
        r_arm = math.hypot(x, y)
        s_theta = math.radians(float(cfg.ALT_SIGMA_THETA_DEG))
        sigma = math.sqrt(float(cfg.R_ALT_BASE) ** 2
                          + s_sensor ** 2
                          + (r_arm * s_theta) ** 2)
        return h, offset, sigma

    def _mount(self, ch):
        m = getattr(self.cfg, 'ALT_MOUNT', {}) or {}
        v = m.get(ch, (0.0, 0.0, 0.0))
        x, y, dz = (list(v) + [0.0, 0.0, 0.0])[:3]
        return float(x or 0.0), float(y or 0.0), float(dz or 0.0)

    # -------------------------------------------------------------- 观测换算
    def depth_raw_to_m(self, raw):
        """深度计 raw → m，并扣零点标定。返回 ``None`` 表示该样本无效（饱和/非法）。"""
        cfg = self.cfg
        if raw is None:
            return None
        try:
            r = float(raw)
        except (TypeError, ValueError):
            return None
        if not (r == r) or r == float('inf') or r == float('-inf'):
            return None
        if r < cfg.DEPTH_RAW_MIN_VALID or r > cfg.DEPTH_RAW_MAX_VALID:
            return None          # 固件把负值钳成 0 → 近水面饱和，必须丢弃
        return r * cfg.DEPTH_SCALE - float(cfg.DEPTH_ZERO_OFFSET)

    def accel_raw_to_ms2(self, ax, ay, az):
        """三轴 raw → m/s²（含符号与单位换算）。``ACCEL_UNIT='unknown'`` 时由上层拒绝启动。"""
        cfg = self.cfg
        if ax is None or ay is None or az is None:
            return None
        try:
            vals = [float(ax), float(ay), float(az)]
        except (TypeError, ValueError):
            return None
        scale = float(cfg.ACCEL_RAW_SCALE)
        unit = str(getattr(cfg, 'ACCEL_UNIT', 'unknown')).strip().lower()
        if unit in ('g', 'grav', 'gravity'):
            scale *= G
        elif unit in ('m/s^2', 'm/s2', 'ms2', 'si'):
            pass
        else:
            return None
        sx, sy, sz = getattr(cfg, 'ACCEL_SIGN', (1.0, 1.0, 1.0))
        return (vals[0] * scale * sx, vals[1] * scale * sy, vals[2] * scale * sz)

    # -------------------------------------------------------------- 输出换算
    def clearance_of(self, x, ch, theta=0.0, phi=0.0):
        """由状态反算某路的离底净空（m）。耦合配对与 obs_alt 保持一致（2026-10-06 修正）。"""
        xr, yr, dz = self._mount(ch)
        c_theta = float(self.cfg.PITCH_SIGN) * yr   # ★ y（前向）耦合俯仰 θ
        c_phi = float(self.cfg.ROLL_SIGN) * xr      # ★ x（右向）耦合横滚 φ
        return ((x[self.idx['H']] + self.anchor_dz - dz)
                - x[self.idx['D']] - c_theta * theta + c_phi * phi)
