#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# algo_tvl1.py - TV-L1 稠密光流(纯函数式, mode = tvl1)
# ============================================================
# 说明:
#   1. 由 optical_flow_demo.py 按 config.ini 的 mode = tvl1 动态加载
#   2. 本文件只暴露一个纯函数 process(), 无类、无隐藏跨帧状态
#   3. 算法: DualTV-L1 (Zach et al., ICCV 2007), OpenCV 实现 cv2.DualTVL1OpticalFlow
#   4. 参数从 config.ini 的 [tvl1] 段与 [motion] 段实时读取
# 统一接口(与 algo_dense.py / algo_dis.py / algo_sparse.py 完全一致):
#   process(prev_gray, gray, frame, cfg, state=None)
#       -> (viz, mean_motion, moving_ratio, state)
#   稠密算法无跨帧状态, state 参数原样返回(恒为 None)
# 想调参? 改 config.ini 的 [tvl1] 段即可, 无需碰代码
# ============================================================

import cv2                                     # OpenCV: DualTV-L1 光流实现
from .algo_common import hsv_flow_viz, motion_stats  # 共享渲染与统计(纯函数)


def process(prev_gray, gray, frame, cfg, state=None):
    """计算(上一帧 -> 当前帧)的 TV-L1 稠密光流并渲染。

    参数:
        prev_gray: 上一帧灰度图(H, W), uint8
        gray:      当前帧灰度图(H, W), uint8
        frame:     当前帧彩色图(BGR); 稠密算法不使用, 仅为统一接口保留
        cfg:       configparser 对象, 读取 [tvl1] 与 [motion] 段参数
        state:     稠密算法无跨帧状态, 恒为 None(仅为接口统一)
    返回:
        (viz, mean_motion, moving_ratio, state):
        viz           HSV 光流渲染图(BGR), 色相=方向 亮度=速度
        mean_motion   全图平均运动速度(像素/帧)
        moving_ratio  超过阈值的运动像素占比(0~1)
        state         恒为 None(稠密算法不维护状态)
    """
    # --- 从 config 读取本算法全部参数 ---
    tau = cfg.getfloat("tvl1", "tau")                      # 时间步长(0.1~0.3)
    lambda_l = cfg.getfloat("tvl1", "lambda")              # 数据/平滑权重(越大越平滑)
    nscales = cfg.getint("tvl1", "nscales")                # 金字塔层数(1~5)
    warps = cfg.getint("tvl1", "warps")                    # 每层 warp 次数(1~10)
    epsilon = cfg.getfloat("tvl1", "epsilon")               # 终止阈值(默认 0.01)
    median = cfg.getint("tvl1", "median_filtering")        # 中值滤波窗口(0 关闭)
    motion_thr = cfg.getfloat("motion", "motion_threshold")  # 运动判定阈值

    # --- 核心算法: TV-L1 稠密光流(每次调用新建实例, 保持函数无副作用) ---
    tvl1 = cv2.DualTVL1OpticalFlow_create()                # 创建 TV-L1 光流器
    tvl1.setTau(tau)                                       # 设置时间步长
    tvl1.setLambda(lambda_l)                               # 设置数据/平滑权重
    tvl1.setNscales(nscales)                               # 设置金字塔层数
    tvl1.setWarpings(warps)                                # 设置 warp 次数
    tvl1.setEpsilon(epsilon)                               # 设置终止阈值
    tvl1.setMedianFiltering(median)                       # 设置中值滤波窗口
    flow = tvl1.calc(prev_gray, gray, None)                # 计算稠密光流场

    # --- 统计与渲染(复用 algo_common 的纯函数) ---
    mean_motion, moving_ratio = motion_stats(flow, motion_thr)  # 统计运动量
    return hsv_flow_viz(flow), mean_motion, moving_ratio, state  # 返回四元组
