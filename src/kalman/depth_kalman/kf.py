# -*- coding: utf-8 -*-
"""EKF 内核：预测 + 标量观测的 Joseph 形式更新 + 新息门限 + 夹紧/发散重置。

只有 numpy 依赖，无任何 I/O，便于自检与复用。
"""
import math

import numpy as np


class EKF(object):
    """最简 EKF。观测按**标量逐个**更新（本工程所有观测都是 1 维），
    这样门限判断最直观，也不需要对矩阵求逆。"""

    def __init__(self, x0, P0):
        self.x = np.asarray(x0, dtype=float).copy()
        self.P = np.asarray(P0, dtype=float).copy()
        self.n = self.x.size

    # ------------------------------------------------------------ 预测
    def predict(self, F, Q):
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        return self.x

    # ------------------------------------------------------------ 更新
    def innovate(self, h, z, r):
        """只算新息与它的协方差（不做更新）。供上层先做门限判断。

        返回 ``(nu, S)``，``S == 0`` 表示该观测不可用。
        """
        H = np.asarray(h, dtype=float).reshape(1, -1)
        nu = float(z - (H @ self.x)[0])
        S = float((H @ self.P @ H.T)[0, 0]) + float(r)
        return nu, S

    def update_scalar(self, h, z, r, gate=None):
        """标量观测更新。

        参数
        ----
        h : array(n)  观测行向量 H
        z : float     观测值（已扣除已知偏移项）
        r : float     观测噪声方差 R（标量）
        gate : float | None
            新息门限（σ 倍数）。``None`` 表示不做门限。

        返回
        ----
        (accepted, innovation, sqrt_S)
        """
        H = np.asarray(h, dtype=float).reshape(1, -1)
        nu = float(z - (H @ self.x)[0])
        S = float((H @ self.P @ H.T)[0, 0]) + float(r)
        if S <= 0.0:
            return False, nu, 0.0
        sqrt_S = math.sqrt(S)
        if gate is not None and abs(nu) > gate * sqrt_S:
            return False, nu, sqrt_S

        K = (self.P @ H.T / S).ravel()          # (n,)
        self.x = self.x + K * nu
        # Joseph 形式，保证对称正定，长期跑不跑偏
        I_KH = np.eye(self.n) - np.outer(K, H)
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * float(r)
        self.P = 0.5 * (self.P + self.P.T)
        return True, nu, sqrt_S

    # ------------------------------------------------------------ 诊断
    def sigma(self, i):
        return float(math.sqrt(max(self.P[i, i], 0.0)))

    def trace(self):
        return float(np.trace(self.P))

    # ------------------------------------------------------------ 保护
    def clamp(self, lo, hi):
        """按分量夹紧状态：``lo``/``hi`` 为等长序列，``None`` 表示该分量不夹。"""
        for i in range(self.n):
            a, b = lo[i], hi[i]
            if a is not None and self.x[i] < a:
                self.x[i] = a
            if b is not None and self.x[i] > b:
                self.x[i] = b
        return self.x

    def reset(self, x, P):
        self.x = np.asarray(x, dtype=float).copy()
        self.P = np.asarray(P, dtype=float).copy()
        self.P = 0.5 * (self.P + self.P.T)
        return self.x
