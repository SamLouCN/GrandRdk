#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# depth_anything.py - Depth Anything V2 BPU 推理封装(去 torch 版)
# ============================================================
# 说明:
#   1. 官方样本 depth_anything_v2.py 的后处理依赖 torch(F.interpolate),
#      板端未装 torch, 本模块改为 numpy/cv2 等价实现(双线性 resize)
#   2. 输入: 任意尺寸 BGR 图像 -> 拉伸到模型输入 518x686
#   3. 输出: 返回 float32 相对深度图(原图尺寸), 不做 0~255 归一化,
#      保留连续深度值供"深度->米"标定与测速使用(官方转 uint8 会损失精度)
#   4. 单目深度是相对深度(尺度不定), 转米需外部 scale_m 标定
# 用法:
#   from depth_anything import DepthAnythingV2Config, DepthAnythingV2
#   m = DepthAnythingV2(DepthAnythingV2Config('/path/depth_any.hbm'))
#   depth_f32 = m.predict(img_bgr)   # (H, W) float32, 数值越大=越近(DAV2惯例)
# ============================================================

import os
import cv2
import numpy as np
import hbm_runtime
from dataclasses import dataclass
from typing import Optional

# ImageNet 归一化参数(与官方样本一致)
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


@dataclass
class DepthAnythingV2Config:
    """Depth Anything V2 推理配置。

    Attributes:
        model_path: 编译好的 *.hbm 模型路径
    """
    model_path: str


class DepthAnythingV2:
    """基于 hbm_runtime 的 Depth Anything V2 单目深度推理封装。"""

    def __init__(self, config: DepthAnythingV2Config):
        """加载 .hbm 模型并读取输入/输出规格。"""
        self.model = hbm_runtime.HB_HBMRuntime(config.model_path)
        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]
        self.input_h = self.input_shapes[self.input_names[0]][2]   # 518
        self.input_w = self.input_shapes[self.input_names[0]][3]   # 686
        # 输出规格(形状可能在后面未知维度, 取已知 2 维)
        out_shape = self.model.output_shapes[self.model_name][self.output_names[0]]
        self.out_h = out_shape[-2]
        self.out_w = out_shape[-1]

    def pre_process(self, img: np.ndarray) -> dict:
        """BGR 图 -> NCHW float32(拉伸 resize + RGB + ImageNet zscore)。"""
        resized = cv2.resize(img, (self.input_w, self.input_h),
                             interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        normed = (rgb - _IMAGENET_MEAN) / _IMAGENET_STD
        tensor = np.transpose(normed, (2, 0, 1))[np.newaxis].astype(np.float32)
        return {self.model_name: {self.input_names[0]: tensor}}

    def forward(self, input_tensor: dict) -> dict:
        """BPU 推理, 返回原始输出张量字典。"""
        return self.model.run(input_tensor)

    def post_process(self, outputs: dict, ori_h: int, ori_w: int) -> np.ndarray:
        """把原始深度输出 resize 回原图尺寸, 返回 float32 相对深度图。

        说明: DAV2 输出值遵循"越近越大"的相对深度惯例(disparity 风格),
        值域约为 [0, ~1+]; 本函数不做 0~255 归一化。
        """
        depth = np.asarray(outputs[self.model_name][self.output_names[0]])
        depth = np.squeeze(depth).astype(np.float32)   # (518, 686) 或含1维
        if depth.ndim != 2:
            raise ValueError(f"深度输出维度异常: {depth.shape}")
        if depth.shape != (ori_h, ori_w):              # 分辨率不同则插值回原尺寸
            depth = cv2.resize(depth, (ori_w, ori_h),
                               interpolation=cv2.INTER_LINEAR)
        return depth

    def predict(self, img: np.ndarray) -> np.ndarray:
        """完整推理管线: 预处理 -> BPU -> 后处理, 返回 (H,W) float32 相对深度。"""
        ori_h, ori_w = img.shape[:2]
        return self.post_process(self.forward(self.pre_process(img)), ori_h, ori_w)

    def __call__(self, img: np.ndarray) -> np.ndarray:
        return self.predict(img)

    @staticmethod
    def colorize(depth_f32: np.ndarray) -> np.ndarray:
        """把 float32 相对深度归一化并上色, 仅用于可视化显示。"""
        d = depth_f32
        vmin, vmax = float(np.percentile(d, 2)), float(np.percentile(d, 98))
        d8 = np.clip((d - vmin) / max(vmax - vmin, 1e-6) * 255.0, 0, 255).astype(np.uint8)
        return cv2.applyColorMap(d8, cv2.COLORMAP_INFERNO)
