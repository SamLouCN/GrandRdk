#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# algo_dis.py - DIS 稠密光流(纯函数式, mode = dis)
# ============================================================
# 说明:
#   1. 由 optical_flow_demo.py 按 config.ini 的 mode = dis 动态加载
#   2. 本文件只暴露一个纯函数 process(), 无类、无隐藏跨帧状态
#   3. DIS = Dense Inverse Search(ECCV 2016 开源算法, OpenCV 官方收录)
#   4. 参数从 config.ini 的 [dis] 段与 [motion] 段实时读取
# 统一接口(与 algo_dense.py / algo_sparse.py 完全一致):
#   process(prev_gray, gray, frame, cfg, state=None)
#       -> (viz, mean_motion, moving_ratio, state)
#   稠密算法无跨帧状态, state 参数原样返回(恒为 None)
# 想改算法? 只改本函数内部即可; 想新增算法? 复制本文件改名并改 mode
# ============================================================

import cv2                                     # OpenCV: DIS 光流实现
from .algo_common import hsv_flow_viz, motion_stats  # 共享渲染与统计(纯函数)


def process(prev_gray, gray, frame, cfg, state=None):
    """计算(上一帧 -> 当前帧)的 DIS 稠密光流并渲染。

    参数:
        prev_gray: 上一帧灰度图(H, W), uint8
        gray:      当前帧灰度图(H, W), uint8
        frame:     当前帧彩色图(BGR); 稠密算法不使用, 仅为统一接口保留
        cfg:       configparser 对象, 读取 [dis] 与 [motion] 段参数
        state:     稠密算法无跨帧状态, 恒为 None(仅为接口统一)
    返回:
        (viz, mean_motion, moving_ratio, state):
        viz           HSV 光流渲染图(BGR), 色相=方向 亮度=速度
        mean_motion   全图平均运动速度(像素/帧)
        moving_ratio  超过阈值的运动像素占比(0~1)
        state         恒为 None(稠密算法不维护状态)
    """
    # --- 从 config 读取本算法全部参数 ---
    preset = cfg.getint("dis", "preset")                      # 预设档位 0/1/2
    iterations = cfg.getint("dis", "gradient_descent_iterations")  # 迭代次数
    motion_thr = cfg.getfloat("motion", "motion_threshold")   # 运动判定阈值

    # --- 核心算法: DIS 稠密光流(每次调用新建实例, 保持函数无副作用) ---
    dis = cv2.DISOpticalFlow_create(preset)                   # 按档位创建 DIS 光流器
    dis.setGradientDescentIterations(iterations)              # 设置迭代次数(质量/速度)
    flow = dis.calc(prev_gray, gray, None)                    # 计算稠密光流场

    # --- 统计与渲染(复用 algo_common 的纯函数) ---
    mean_motion, moving_ratio = motion_stats(flow, motion_thr)  # 统计运动量
    return hsv_flow_viz(flow), mean_motion, moving_ratio, state  # 返回四元组
