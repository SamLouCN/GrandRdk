#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# speed_fusion.py - 深度(米) + 光流(px) -> 3D 速度(m/s) 融合器
# ============================================================
# 原理(固定相机近似):
#   同一深度平面上的像素位移满足: dX_m = dPx * Z_m / focal_px
#   每像素速度: v(m/s) = dPx(px/帧) / dt(秒/帧) * Z_m(米) / focal_px(px)
#            = dPx * Z_m / focal_px / dt      (dt 为实测帧间隔, 不写死 fps)
# 深度来源: Depth Anything V2(官方 BPU 模型, 后台线程刷新, 不阻塞主循环)
# 单目深度是"相对深度"(尺度不定), 转米用仿射标定:
#   Z_m = depth_a * rel_depth + depth_b
# 处理尺度: 光流可在半分辨率计算(process_scale<1), px 位移换算回原图后
#   再参与速度公式, 保证结果与尺度无关
# ============================================================

import os
import sys
import threading

import cv2
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))


class DepthSpeedMeter:
    """把 BPU 深度与逐像素光流融合, 输出 m/s 速度统计。"""

    def __init__(self, cfg):
        """从 config 读取 [depth]/[speed]/[motion] 全部参数。"""
        self.enable = cfg.getboolean("depth", "enable")        # 深度测速总开关
        self.model_path = cfg.get("depth", "model_path")       # depth_any.hbm 路径
        self.interval = cfg.getint("depth", "interval_frames")  # 每多少帧刷新一次深度
        self.depth_a = cfg.getfloat("depth", "depth_a")        # 仿射标定: Z = a*rel + b
        self.depth_b = cfg.getfloat("depth", "depth_b")
        self.focal_px = cfg.getfloat("speed", "focal_px")      # 焦距(像素), 需标定
        self.motion_thr = cfg.getfloat("motion", "motion_threshold")  # 运动阈值(px/帧, 原图尺度)

        self._model = None          # DAV2 推理器(懒加载)
        self._depth_m = None        # 最近一次深度图, 已换算为米 (H,W) float32
        self._n = 0                 # 帧计数
        self._loaded_fail = False   # 模型加载失败标记
        self._worker = None         # 后台刷新线程
        self._lock = threading.Lock()

    def _load_model(self):
        """懒加载 DAV2 模型; 失败时置 disable 并打印原因。"""
        sys.path.insert(0, _THIS_DIR)             # 同目录 depth_anything.py
        try:
            from depth_anything import DepthAnythingV2, DepthAnythingV2Config
            self._model = DepthAnythingV2(DepthAnythingV2Config(self.model_path))
            print(f"[depth] DAV2 模型加载完成: {self.model_path}")
        except Exception as e:                    # hbm 缺失/损坏等
            print(f"[depth] 模型加载失败, 已停用深度测速: {e}")
            self.enable = False
            self._loaded_fail = True

    def update(self, frame):
        """主循环每帧调用: 到达刷新间隔时把推理丢给后台线程。

        返回 True 表示本帧触发了深度刷新(异步, 不阻塞主循环)。
        """
        if not self.enable:
            return False
        self._n += 1
        if self._model is None:
            if self._loaded_fail:
                return False
            self._load_model()
            if self._model is None:
                return False
        if self._n % self.interval != 0:          # 未到刷新间隔
            return False
        # 上一次刷新还没算完则跳过本轮(深度短暂滞后, 主循环不受阻)
        if self._worker is not None and self._worker.is_alive():
            return False
        self._worker = threading.Thread(
            target=self._refresh_worker, args=(frame.copy(),), daemon=True)
        self._worker.start()
        return True

    def _refresh_worker(self, frame):
        """后台线程: BPU 推理并更新深度缓存。"""
        try:
            rel = self._model.predict(frame)       # BPU 推理(~150ms)
            z = self.depth_a * rel.astype(np.float32) + self.depth_b
            with self._lock:
                self._depth_m = z
        except Exception as e:
            print(f"[depth] 后台刷新失败: {e}")

    def depth_map_m(self):
        """返回最近一次深度图(米)或 None。"""
        with self._lock:
            return self._depth_m

    def measure(self, flow, dt=1.0 / 30.0, scale=1.0, tex_mask=None):
        """把光流场换算为 m/s 并统计。

        参数:
            flow:  (H,W,2) float32, 单位 px/帧(按计算尺度, 见 scale)
            dt:    实测帧间隔(秒): v(m/s) = |flow|/scale * Z / focal / dt
            scale: 光流计算尺度(0~1): 将 flow 位移换算回原图像素
        返回:
            (median_mps, mean_mps, moving_ratio): 无深度/无运动时部分为 None
        """
        if not self.enable:
            return None, None, None
        with self._lock:
            z = self._depth_m
            if z is None:
                return None, None, None
            z = z.copy()
        if flow is None:
            return None, None, None

        # 深度图与光流图对齐到同一尺度(按较小的 flow 尺寸取深度)
        fh, fw = flow.shape[:2]
        if z.shape[:2] != (fh, fw):
            z = cv2.resize(z, (fw, fh), interpolation=cv2.INTER_LINEAR)

        # px(计算尺度)/帧 -> px(原图)/帧
        mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2) / max(scale, 1e-6)
        moving = mag > self.motion_thr                # 运动掩码(原图 px 阈值)
        if tex_mask is not None:                      # 纹理护栏: 平板区不算运动
            hh, ww = moving.shape[:2]
            if tex_mask.shape[:2] != (hh, ww):
                tex_mask = cv2.resize(tex_mask.astype(np.uint8), (ww, hh),
                                      interpolation=cv2.INTER_NEAREST) > 0
            moving = moving & tex_mask
        valid = moving & (z > 0.001)                  # 有运动且有深度
        moving_ratio = float(moving.mean())
        if not valid.any():
            return 0.0, 0.0, moving_ratio

        v = mag * z / self.focal_px / dt              # px -> m/s
        vs = v[valid]
        return float(np.median(vs)), float(vs.mean()), moving_ratio

    @staticmethod
    def annotate(img, median_mps, moving_ratio):
        """在画面左上叠加速度文本(返回新图, 不改原图)。"""
        out = img.copy()
        if median_mps is None:
            cv2.putText(out, "speed: -- (depth warming)", (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            return out
        if moving_ratio is None:
            moving_ratio = 0.0
        txt = f"speed: {median_mps:5.2f} m/s  (motion {moving_ratio*100:.0f}%)"
        cv2.putText(out, txt, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return out