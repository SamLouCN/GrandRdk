# -*- coding: utf-8 -*-
# ============================================================
# algo_farneback.py - 历史经典算法①: Farneback 稠密光流
# ============================================================
# 说明:
#   - 这是"原来的 dense 模式"完整实现, 保留未删, 封装成独立文件
#   - 由主程序 optical_flow_demo.py 按 config 的 mode=dense 动态加载
#   - 参数段: config.ini 的 [dense]
# 特点: 全图稠密运动场, 速度快; 对大位移/弱纹理区域鲁棒性一般
# ============================================================

import cv2           # OpenCV: Farneback 光流实现
from flow_viz import hsv_flow_viz, flow_stats   # 复用公共渲染与统计工具


class FarnebackDense:
    """Farneback 稠密光流算法对象(原 dense 分支实现, 逐行注释)。"""

    def __init__(self, cfg):
        """从 config 读取本算法全部参数, 并保存运动判定阈值。"""
        self.thr = cfg.getfloat("motion", "motion_threshold")   # 取"运动"判定阈值
        self.pyr_scale = cfg.getfloat("dense", "pyr_scale")     # 取金字塔缩放系数
        self.levels = cfg.getint("dense", "levels")             # 取金字塔层数
        self.winsize = cfg.getint("dense", "winsize")           # 取邻域窗口大小
        self.iterations = cfg.getint("dense", "iterations")     # 取迭代次数
        self.poly_n = cfg.getint("dense", "poly_n")             # 取多项式展开邻域
        self.poly_sigma = cfg.getfloat("dense", "poly_sigma")   # 取高斯标准差

    def run(self, prev_gray, gray, frame=None):
        """在(上一帧->当前帧)灰度上计算 Farneback 稠密光流。

        参数:
            prev_gray: 上一帧灰度图(单通道 uint8)
            gray:      当前帧灰度图(单通道 uint8)
            frame:     当前帧彩色图, 稠密算法不需要, 仅为接口统一
        返回:
            (viz, mean_motion, moving_ratio):
            viz           HSV 彩色渲染图(BGR)
            mean_motion   全图平均速度(像素/帧)
            moving_ratio  超过阈值的运动像素占比
        """
        flow = cv2.calcOpticalFlowFarneback(                    # 计算稠密光流场(整幅图)
            prev_gray, gray, None,                              # 输入: 上一帧/当前帧
            pyr_scale=self.pyr_scale,                           # 金字塔缩放系数
            levels=self.levels,                                 # 金字塔层数
            winsize=self.winsize,                               # 邻域窗口大小
            iterations=self.iterations,                         # 迭代次数
            poly_n=self.poly_n,                                 # 多项式展开邻域
            poly_sigma=self.poly_sigma,                         # 高斯标准差
            flags=0)                                            # 无附加标志
        mean_motion, moving_ratio = flow_stats(flow, self.thr)  # 计算运动统计量
        return hsv_flow_viz(flow), mean_motion, moving_ratio    # 返回渲染图与统计量
