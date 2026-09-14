#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_dis.py - 稠密光流算法: DIS (Dense Inverse Search)
# ============================================================
# 说明:
#   本文件由主程序 optical_flow_demo.py 按 config.ini 的
#   [algorithm] mode = dis 时自动加载调用(推荐算法)。
#   参数来源: config.ini 的 [dis] 段
# 接口(与 flow_farneback.py / flow_pyrlk.py 保持一致):
#   class DisFlow(cfg)
#   .process(gray_prev, gray, frame, motion_thr)
#       -> (viz, mean_motion, moving_ratio)
# ============================================================

import cv2            # OpenCV: DIS 光流计算
import numpy as np    # NumPy: 幅值/角度等数组运算


def hsv_flow_viz(flow):
    """把稠密光流场编码为 HSV 彩色渲染图(BGR 输出)。"""
    h, w = flow.shape[:2]                                       # 取图像高宽
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])      # x/y 分量 -> 幅值+角度
    mag_n = np.clip(mag, 0, 30) / 30.0                          # 幅值截断到 0~30px 并归一化
    hsv = np.zeros((h, w, 3), dtype=np.uint8)                   # 新建 HSV 三通道图像
    hsv[..., 0] = ang * 180.0 / np.pi / 2.0                     # H: 角度 -> 色相(方向)
    hsv[..., 1] = 255                                           # S: 饱和度固定最大
    hsv[..., 2] = (mag_n * 255).astype(np.uint8)                # V: 速度 -> 亮度
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)                 # 转 BGR 供保存/显示


class DisFlow:
    """DIS(Dense Inverse Search)稠密光流实现(当前默认推荐算法)。"""

    def __init__(self, cfg):
        """从 config.ini 的 [dis] 段读取参数, 并预建光流器(只建一次)。"""
        preset = cfg.getint("dis", "preset")                    # 取预设档位(0/1/2)
        iters = cfg.getint("dis", "gradient_descent_iterations")  # 取迭代次数
        self.dis = cv2.DISOpticalFlow_create(preset)            # 创建 DIS 光流器
        self.dis.setGradientDescentIterations(iters)            # 设置梯度下降迭代次数

    def process(self, gray_prev, gray, frame, motion_thr):
        """计算上一帧到当前帧的稠密光流, 返回(渲染图, 平均速度, 运动占比)。"""
        flow = self.dis.calc(gray_prev, gray, None)             # 用 DIS 计算稠密光流场
        mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])    # 求每像素速度幅值
        mean_motion = float(mag.mean())                         # 全图平均速度
        moving_ratio = float((mag > motion_thr).mean())         # 超过阈值的像素占比
        viz = hsv_flow_viz(flow)                                # 生成 HSV 彩色渲染图
        return viz, mean_motion, moving_ratio                   # 返回统一三元组
