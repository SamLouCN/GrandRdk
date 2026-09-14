# -*- coding: utf-8 -*-
# ============================================================
# flow_viz.py - 光流公共可视化/统计工具
# ============================================================
# 说明: 本文件被 algo_farneback.py / algo_dis.py 等算法文件共用,
#       放公共函数, 避免每个算法文件重复实现。
# ============================================================

import cv2           # OpenCV: 颜色空间转换等
import numpy as np   # NumPy: 数组运算


def hsv_flow_viz(flow):
    """把稠密光流场编码为 HSV 彩色渲染图(BGR 输出)。

    约定: 色相(颜色) = 运动方向, 亮度 = 运动速度, 越亮动得越快。
    """
    h, w = flow.shape[:2]                                       # 取图像高宽
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])      # x/y 分量 -> 幅值+角度
    mag_n = np.clip(mag, 0, 30) / 30.0                          # 幅值截断到 0~30px 并归一化
    hsv = np.zeros((h, w, 3), dtype=np.uint8)                   # 新建 HSV 三通道图像
    hsv[..., 0] = ang * 180.0 / np.pi / 2.0                     # H: 角度 -> 色相
    hsv[..., 1] = 255                                           # S: 饱和度固定最大
    hsv[..., 2] = (mag_n * 255).astype(np.uint8)                # V: 速度 -> 亮度
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)                 # 转 BGR 供保存/显示


def flow_stats(flow, threshold):
    """统计稠密光流, 返回(全图平均速度, 超过阈值的运动像素占比)。"""
    mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])        # 求每像素速度幅值
    mean_motion = float(mag.mean())                             # 全图平均速度
    moving_ratio = float((mag > threshold).mean())              # 超过阈值的像素占比
    return mean_motion, moving_ratio                            # 返回两个统计量
