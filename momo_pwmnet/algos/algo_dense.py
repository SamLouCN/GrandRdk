#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# algo_dense.py - Farneback 稠密光流(纯函数式, mode = dense)
# ============================================================
# 说明:
#   1. 由 optical_flow_demo.py 按 config.ini 的 mode = dense 动态加载
#   2. 本文件只暴露一个纯函数 process(), 无类、无隐藏跨帧状态
#   3. 参数从 config.ini 的 [dense] 段与 [motion] 段实时读取
# 统一接口(与 algo_dis.py / algo_sparse.py 完全一致):
#   process(prev_gray, gray, frame, cfg, state=None)
#       -> (viz, mean_motion, moving_ratio, state)
#   稠密算法无跨帧状态, state 参数原样返回(恒为 None)
# 想改算法? 只改本函数内部即可; 想新增算法? 复制本文件改名并改 mode
# ============================================================

import cv2                                     # OpenCV: Farneback 光流
from .algo_common import hsv_flow_viz, motion_stats  # 共享渲染与统计(纯函数)


def process(prev_gray, gray, frame, cfg, state=None):
    """计算(上一帧 -> 当前帧)的 Farneback 稠密光流并渲染。

    参数:
        prev_gray: 上一帧灰度图(H, W), uint8
        gray:      当前帧灰度图(H, W), uint8
        frame:     当前帧彩色图(BGR); 稠密算法不使用, 仅为统一接口保留
        cfg:       configparser 对象, 读取 [dense] 与 [motion] 段参数
        state:     稠密算法无跨帧状态, 恒为 None(仅为接口统一)
    返回:
        (viz, mean_motion, moving_ratio, state):
        viz           HSV 光流渲染图(BGR), 色相=方向 亮度=速度
        mean_motion   全图平均运动速度(像素/帧)
        moving_ratio  超过阈值的运动像素占比(0~1)
        state         恒为 None(稠密算法不维护状态)
    """
    # --- 从 config 读取本算法全部参数(每次调用读取, 便于热改配置) ---
    pyr_scale = cfg.getfloat("dense", "pyr_scale")            # 金字塔缩放系数
    levels = cfg.getint("dense", "levels")                    # 金字塔层数
    winsize = cfg.getint("dense", "winsize")                  # 邻域窗口大小
    iterations = cfg.getint("dense", "iterations")            # 每层迭代次数
    poly_n = cfg.getint("dense", "poly_n")                    # 多项式展开邻域(5/7)
    poly_sigma = cfg.getfloat("dense", "poly_sigma")          # 多项式高斯标准差
    motion_thr = cfg.getfloat("motion", "motion_threshold")   # 运动判定阈值

    # --- 核心算法: 计算整幅稠密光流场 ---
    flow = cv2.calcOpticalFlowFarneback(                      # 计算稠密光流场
        prev_gray, gray, None,                                # 输入: 上一帧/当前帧
        pyr_scale=pyr_scale,                                  # 金字塔缩放系数
        levels=levels,                                        # 金字塔层数
        winsize=winsize,                                      # 邻域窗口大小
        iterations=iterations,                                # 迭代次数
        poly_n=poly_n,                                        # 多项式展开邻域
        poly_sigma=poly_sigma,                                # 高斯标准差
        flags=0)                                              # 无附加标志

    # --- 统计与渲染(复用 algo_common 的纯函数) ---
    mean_motion, moving_ratio = motion_stats(flow, motion_thr)  # 统计运动量
    return hsv_flow_viz(flow), mean_motion, moving_ratio, state  # 返回四元组
