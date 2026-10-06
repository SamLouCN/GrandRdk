#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_pid.py — 过门对准位置式 PD(+I 备用)

控制律（方案 v4）:
    dpsi  = KP_YAW·e + KI·∫e·dt − KD_YAW·ω     # 航向修正量(°)
    ψtar  = wrap180(ψ_lock + dpsi)             # ψ_lock 锚定在 gate_mission 维护
    sway  = KP_SWAY·e                          # 横移与转向同动, 小死区

约定:
  - e 已带符号（门中心在画面右侧为正）, ω 已带符号（右转为正, °/s）;
  - D 项用陀螺仪角速度（对测量微分）, 不对图像做差分;
  - 纯计算模块, 不碰文件与串口, 仿真与实机共用同一份代码。
"""
import math


def wrap180(a):
    """角度归一到 (-180, 180]"""
    return (a + 180.0) % 360.0 - 180.0


class GatePid(object):
    """位置式 PD(+I 备用)。一次 step 输出 (dpsi, sway)。"""

    def __init__(self, kp_yaw, kd_yaw, kp_sway, sway_dead=0.0,
                 ki=0.0, i_max=1.0, psi_max=0.0):
        self.kp_yaw = float(kp_yaw)
        self.kd_yaw = float(kd_yaw)
        self.kp_sway = float(kp_sway)
        self.sway_dead = float(sway_dead)
        self.ki = float(ki)
        self.i_max = float(i_max)
        self.psi_max = float(psi_max)   # 0 = 不限幅
        self.reset()

    def reset(self):
        """清积分。换阶段/重进对准时调用。"""
        self._integ = 0.0

    def step(self, e, omega, dt):
        """e: 滤波后横向误差; omega: 陀螺仪 z 轴角速度(°/s); dt: 拍间隔 s。

        返回 (dpsi, sway)。dt<=0 时本拍不推进积分。
        """
        dt = max(0.0, float(dt))

        # P
        dpsi = self.kp_yaw * e

        # I（默认 ki=0 不参与; 积分限幅兜底）
        if self.ki != 0.0 and dt > 0.0:
            self._integ += e * dt
            if self._integ > self.i_max:
                self._integ = self.i_max
            elif self._integ < -self.i_max:
                self._integ = -self.i_max
            dpsi += self.ki * self._integ

        # D: 陀螺仪阻尼（对测量微分, 机体正在转向时自动刹车）
        dpsi -= self.kd_yaw * omega

        # 输出限幅（默认关）
        if self.psi_max > 0.0:
            if dpsi > self.psi_max:
                dpsi = self.psi_max
            elif dpsi < -self.psi_max:
                dpsi = -self.psi_max

        # sway: 与转向同动, 死区内归零
        sway = self.kp_sway * e
        if abs(e) < self.sway_dead:
            sway = 0.0

        return dpsi, sway
