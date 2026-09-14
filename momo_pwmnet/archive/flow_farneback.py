#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_farneback.py - 稠密光流算法: Farneback(原实现, 独立成模块)
# ============================================================
# 说明:
#   本文件由主程序 optical_flow_demo.py 按 config.ini 的
#   [algorithm] mode = dense 时自动加载调用。
#   参数来源: config.ini 的 [dense] 段
# 接口(与 flow_dis.py / flow_pyrlk.py 保持一致):
#   class FarnebackFlow(cfg)            # 传入全局配置对象
#   .process(gray_prev, gray, frame, motion_thr)
#       -> (viz, mean_motion, moving_ratio)
# ============================================================

import cv2            # OpenCV: Farneback 光流计算
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


class FarnebackFlow:
    """Farneback 稠密光流实现(原 optical_flow_demo 的 dense 模式)。"""

    def __init__(self, cfg):
        """从 config.ini 的 [dense] 段读取全部 Farneback 参数。"""
        self.pyr_scale = cfg.getfloat("dense", "pyr_scale")     # 金字塔缩放系数
        self.levels = cfg.getint("dense", "levels")             # 金字塔层数
        self.winsize = cfg.getint("dense", "winsize")           # 邻域窗口大小
        self.iterations = cfg.getint("dense", "iterations")     # 迭代次数
        self.poly_n = cfg.getint("dense", "poly_n")             # 多项式展开邻域
        self.poly_sigma = cfg.getfloat("dense", "poly_sigma")   # 高斯标准差

    def process(self, gray_prev, gray, frame, motion_thr):
        """计算上一帧到当前帧的稠密光流, 返回(渲染图, 平均速度, 运动占比)。"""
        flow = cv2.calcOpticalFlowFarneback(                    # 计算稠密光流场(整幅图)
            gray_prev, gray, None,                              # 输入: 上一帧/当前帧
            pyr_scale=self.pyr_scale,                           # 金字塔缩放系数
            levels=self.levels,                                 # 金字塔层数
            winsize=self.winsize,                               # 邻域窗口大小
            iterations=self.iterations,                         # 迭代次数
            poly_n=self.poly_n,                                 # 多项式展开邻域
            poly_sigma=self.poly_sigma,                         # 高斯标准差
            flags=0)                                            # 无附加标志
        mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])    # 求每像素速度幅值
        mean_motion = float(mag.mean())                         # 全图平均速度
        moving_ratio = float((mag > motion_thr).mean())         # 超过阈值的像素占比
        viz = hsv_flow_viz(flow)                                # 生成 HSV 彩色渲染图
        return viz, mean_motion, moving_ratio                   # 返回统一三元组
