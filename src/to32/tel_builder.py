# -*- coding: utf-8 -*-
"""V2 遥测 -> 上位机 $TEL（41 字段）的统一映射。

为什么单独一个文件：
  $TEL 的 41 个索引是上位机**硬编码**的（见《上位机通讯协议.md》），谁都不该各写一份；
  这里给出唯一一份「V2 遥测字段 -> 上位机索引」映射，模式里没有特殊需求时直接用本模块。

输入 = link_stm32.parse_telemetry() 的 dict；输出 = 可直接 sendto 的 $TEL 文本帧。
V2 没有的字段（线速度 vx/vy/vz、目标角速度、电池/舱温、高度 alt、9~12 路电机）一律补 0 占位，
**不能省略**，否则上位机整帧错位。
"""
from __future__ import annotations

import protocol as P

try:
    import to32_config as C
    MOTOR_SCALE = float(getattr(C, "MOTOR_SCALE", 1000.0)) or 1000.0
except Exception:
    MOTOR_SCALE = 1000.0

TEL_FIELD_COUNT = int(getattr(P, "TEL_FIELD_COUNT", 41))


def tel_values(tel):
    """V2 遥测 dict -> 41 个数值（顺序即上位机索引）；无遥测返回 None。"""
    if not tel:
        return None
    f = [0.0] * TEL_FIELD_COUNT

    def put(idx, key, scale=1.0):
        try:
            f[idx] = float(tel.get(key, 0.0)) / scale
        except (TypeError, ValueError):
            f[idx] = 0.0

    # 0-5  实际姿态 + 角速度（注意上位机顺序是 roll, pitch, yaw）
    put(0, "actual_roll")
    put(1, "actual_pitch")
    put(2, "actual_yaw")
    put(3, "gyro_roll")
    put(4, "gyro_pitch")
    put(5, "gyro_yaw")
    # 6    实际深度(m)
    put(6, "actual_depth_cm", 100.0)
    # 7-9  vx/vy/vz：V2 无（后续接光流/多普勒时在此填）
    # 10-12 目标姿态
    put(10, "target_roll")
    put(11, "target_pitch")
    put(12, "target_yaw")
    # 13-15 目标角速度：V2 无
    # 16   目标深度(m)
    put(16, "target_depth_cm", 100.0)
    # 17-19 目标线速度：V2 无
    # 20-24 电池电压/电流/SOC/温度/舱温：V2 无
    # 25-32 推进器 thr0..thr7（V2 电机原值 -> [-1,1]）
    motors = tel.get("motors") or []
    for i in range(min(8, len(motors))):
        try:
            f[25 + i] = float(motors[i]) / MOTOR_SCALE
        except (TypeError, ValueError):
            f[25 + i] = 0.0
    # 33-36 thr8..thr11：V2 只有 8 路
    # 37-39 加速度
    put(37, "acc_x")
    put(38, "acc_y")
    put(39, "acc_z")
    # 40    alt：由高度计（USART / UDP 8082）单独提供，V2 无
    return f


def build_tel(tel):
    """-> '$TEL,...,#\\r\\n' 文本帧；无遥测返回 None。"""
    vals = tel_values(tel)
    if vals is None:
        return None
    return P.build_tel(vals)