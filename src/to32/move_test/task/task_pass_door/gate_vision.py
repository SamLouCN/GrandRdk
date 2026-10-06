#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_vision.py — 视觉源: 读 momo_det_front.json → 门观测

输出观测 dict:
    {'e_raw', 'w_px', 'cx', 'cy', 'score', 'w_ratio', 'frame'}
    e_raw ∈ [-1,1] 半宽归一化, 已乘 E_SIGN
无有效门返回 None（含: 文件超期/读坏/无 door）。

实现要点（照抄已验证做法）:
  1. 帧号去重: 同一帧不重复出观测
  2. mtime 新鲜度: 写端挂了不能拿旧数据
  3. 同标签取 score 最高
  4. 读 JSON 一律 try/except 兜空（覆盖写时可能读到半个 JSON）

observe() 与 poll() 共用同一套选择逻辑, 便于回放/仿真直接喂 dets 列表。
"""
import json
import os
import time


class GateVision(object):

    def __init__(self, det_path, img_w=640, img_h=480, stale_s=0.5,
                 label='door', e_sign=+1):
        self.path = str(det_path)
        self.w = float(img_w)
        self.h = float(img_h)
        self.stale_s = float(stale_s)
        self.label = str(label)
        self.e_sign = float(e_sign)
        self._last_frame = None

    # ------------------------------------------------ 选择逻辑（回放/实机共用）
    def observe(self, dets, img_w=None, img_h=None, frame=None):
        """从一帧 dets 列表选门并算误差。无有效门返回 None。"""
        w = float(img_w) if img_w else self.w
        best = None
        for d in (dets or []):
            if str(d.get('label', '')).strip().lower() != self.label:
                continue
            try:
                score = float(d.get('score', 0.0))
            except (TypeError, ValueError):
                continue
            if best is None or score > best.get('score', 0.0):
                best = d
        if best is None:
            return None
        bbox = best.get('bbox') or [0, 0, 0, 0]
        try:
            x1, y1, x2, y2 = (float(v) for v in bbox[:4])
        except (TypeError, ValueError):
            return None
        cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        w_px = max(1.0, x2 - x1)
        e_raw = (cx - 0.5 * w) / (0.5 * w)          # 半宽归一化 ∈ [-1,1]
        e_raw *= self.e_sign
        if frame is not None:
            self._last_frame = frame
        return {'e_raw': e_raw, 'w_px': w_px, 'cx': cx, 'cy': cy,
                'score': float(best.get('score', 0.0)),
                'w_ratio': w_px / w, 'frame': frame}

    # ------------------------------------------------ 实机入口
    def poll(self, now=None):
        """读检测文件返回观测 dict 或 None。now 缺省取当前时间。"""
        if now is None:
            now = time.time()
        try:
            mt = os.stat(self.path).st_mtime
        except OSError:
            return None
        if mt <= 0.0 or (now - mt) > self.stale_s:
            return None
        try:
            with open(self.path, 'rb') as f:
                obj = json.loads(f.read().decode('utf-8', 'replace'))
        except Exception:
            return None

        frame = obj.get('frame')
        if frame is not None and frame == self._last_frame:
            return None                              # 同一帧不重复出
        obs = self.observe(obj.get('dets') or [],
                           obj.get('img_w'), obj.get('img_h'), frame)
        return obs
