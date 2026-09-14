#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# algo_common.py - 光流算法共享组件(HSV 渲染 + 运动统计)
# ============================================================
# 说明:
#   1. 被 algo_dense.py / algo_dis.py / algo_sparse.py 共同 import
#   2. 只放与具体算法无关的公共函数, 避免各算法文件重复代码
# 用法:
#   from algo_common import hsv_flow_viz, motion_stats
# ============================================================

import cv2           # OpenCV: 图像颜色空间转换 / 极坐标换算
import numpy as np   # NumPy: 数组运算


def hsv_flow_viz(flow):
    """把稠密光流场编码为 HSV 彩色渲染图(BGR 输出)。

    色相 = 运动方向, 亮度 = 运动速度; 供稠密类算法(DIS/Farneback)可视化。
    """
    h, w = flow.shape[:2]                                    # 取图像高宽
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])   # x/y 分量 -> 幅值+角度
    mag_n = np.clip(mag, 0, 30) / 30.0                       # 幅值截断到 0~30px 并归一化
    hsv = np.zeros((h, w, 3), dtype=np.uint8)                # 新建 HSV 三通道图像
    hsv[..., 0] = ang * 180.0 / np.pi / 2.0                  # H: 角度 -> 色相(方向)
    hsv[..., 1] = 255                                        # S: 饱和度固定最大
    hsv[..., 2] = (mag_n * 255).astype(np.uint8)             # V: 速度 -> 亮度
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)              # 转 BGR 供保存/显示


def motion_stats(flow, motion_thr):
    """根据稠密光流场统计平均运动量与运动像素占比。

    参数:
        flow: (H,W,2) 光流场
        motion_thr: 判定"有运动"的速度阈值(像素/帧)
    返回:
        (mean_motion, moving_ratio)
    """
    mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])     # 求每像素速度幅值
    mean_motion = float(mag.mean())                          # 全图平均速度
    moving_ratio = float((mag > motion_thr).mean())          # 超过阈值的像素占比
    return mean_motion, moving_ratio                         # 返回统计结果
