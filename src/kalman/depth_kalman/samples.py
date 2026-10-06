# -*- coding: utf-8 -*-
"""数据样本的轻量容器（纯数据结构，无 I/O）。"""

import math


class Telemetry(object):
    """下位机遥测（来自 ``momo_telemetry.json``）。

    字段都是**已换算好的物理量**；缺失的字段为 ``None``。
    """

    __slots__ = ('token', 'ts', 'age', 'depth_m', 'depth_raw',
                 'acc', 'pitch_rad', 'roll_rad', 'yaw_rad', 'has_att',
                 'acc_fresh', 'stale')

    def __init__(self, token=None, ts=None, age=float('inf'),
                 depth_m=None, depth_raw=None, acc=None,
                 pitch_rad=None, roll_rad=None, yaw_rad=None,
                 has_att=False, acc_fresh=False, stale=True):
        self.token = token
        self.ts = ts
        self.age = age
        self.depth_m = depth_m
        self.depth_raw = depth_raw
        self.acc = acc                      # (ax, ay, az) m/s^2，已扣符号/单位
        self.pitch_rad = pitch_rad
        self.roll_rad = roll_rad
        self.yaw_rad = yaw_rad
        self.has_att = has_att
        self.acc_fresh = acc_fresh
        self.stale = stale

    @property
    def has_depth(self):
        return self.depth_m is not None

    def __repr__(self):
        return ('Telemetry(token=%r, age=%.3f, depth_m=%s, acc=%s, att=%s)'
                % (self.token, self.age if self.age != float('inf') else -1.0,
                   self.depth_m, self.acc,
                   (self.pitch_rad, self.roll_rad) if self.has_att else None))


class AltSample(object):
    """单路高度计样本（来自 ``momo_alt.json``）。"""

    __slots__ = ('ch', 'token', 'ts', 'age', 'clearance_m', 'status', 'ok', 'stale')

    def __init__(self, ch, token=None, ts=None, age=float('inf'),
                 clearance_m=None, status='', ok=False, stale=True):
        self.ch = ch
        self.token = token
        self.ts = ts
        self.age = age
        self.clearance_m = clearance_m    # 探头面 → 池底，m
        self.status = status
        self.ok = ok
        self.stale = stale

    def __repr__(self):
        return ('AltSample(%s, age=%.3f, clearance_m=%s, status=%r, ok=%s)'
                % (self.ch, self.age if self.age != float('inf') else -1.0,
                   self.clearance_m, self.status, self.ok))


def deg_to_rad(v):
    if v is None:
        return None
    return v * math.pi / 180.0
