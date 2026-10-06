#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_filter.py — e 滤波适配层（方案 v4 §三）

三种模式:
  none          直通（本地仿真/回放用）
  viskf         消费 /dev/shm/momo_viskf.json 的 e_x（实机默认, 门宽归一化口径）
  complementary 一阶低通兜底

统一接口: update(e_raw, now) -> (e_f or None, ok)
  ok=False 表示当前无可用滤波值（viskf 没跑/trust=False/超期）,
  上层按丢帧冻结处理。模式为 none 时 e_raw 永远可用。
"""
import json
import os
import time


class EFilter(object):

    def __init__(self, mode='none', viskf_path='/dev/shm/momo_viskf.json',
                 stale_s=0.5, alpha=0.3):
        self.mode = str(mode)
        self.viskf_path = str(viskf_path)
        self.stale_s = float(stale_s)
        self.alpha = float(alpha)
        self._lp = None            # complementary 状态

    # ------------------------------------------------ 内部
    def _read_viskf(self, now):
        """读 viskf 输出, 返回 (e_x or None, trust)。"""
        try:
            mt = os.stat(self.viskf_path).st_mtime
        except OSError:
            return None, False
        if mt <= 0.0 or (now - mt) > self.stale_s:
            return None, False
        try:
            with open(self.viskf_path, 'rb') as f:
                d = json.loads(f.read().decode('utf-8', 'replace'))
        except Exception:
            return None, False
        e_x = d.get('e_x')
        if e_x is None:
            return None, False
        try:
            e_x = float(e_x)
        except (TypeError, ValueError):
            return None, False
        trust = bool(d.get('trust', False)) and bool(d.get('gate_visible', False))
        return e_x, trust

    # ------------------------------------------------ 接口
    def update(self, e_raw, now=None):
        """返回 (e_f or None, ok)。"""
        if now is None:
            now = time.time()
        if self.mode == 'viskf':
            e_x, trust = self._read_viskf(now)
            if e_x is None or not trust:
                return None, False
            return e_x, True
        if self.mode == 'complementary':
            a = min(max(self.alpha, 0.0), 1.0)
            self._lp = e_raw if self._lp is None else a * self._lp + (1.0 - a) * e_raw
            return self._lp, True
        # none: 直通
        return e_raw, True
