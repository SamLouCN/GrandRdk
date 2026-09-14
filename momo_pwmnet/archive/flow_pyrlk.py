#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_pyrlk.py - 稀疏光流算法: PyrLK 特征点跟踪(原实现, 独立成模块)
# ============================================================
# 说明:
#   本文件由主程序 optical_flow_demo.py 按 config.ini 的
#   [algorithm] mode = sparse 时自动加载调用。
#   参数来源: config.ini 的 [sparse] 段
# 接口(与 flow_farneback.py / flow_dis.py 保持一致):
#   class PyrLKFlow(cfg)
#   .process(gray_prev, gray, frame, motion_thr)
#       -> (viz, mean_motion, moving_ratio)   # moving_ratio 恒为 0
# ============================================================

import cv2            # OpenCV: 角点检测 / PyrLK 跟踪 / 绘制
import numpy as np    # NumPy: 坐标与位移运算


class PyrLKFlow:
    """PyrLK 稀疏光流实现(原 optical_flow_demo 的 sparse 模式)。"""

    def __init__(self, cfg):
        """从 config.ini 的 [sparse] 段读取参数, 并初始化跟踪状态。"""
        self.max_corners = cfg.getint("sparse", "max_corners")    # 最多角点数
        self.quality = cfg.getfloat("sparse", "quality_level")    # 角点质量阈值
        self.min_dist = cfg.getint("sparse", "min_distance")      # 角点最小间距
        self.win_size = cfg.getint("sparse", "win_size")          # 搜索窗口边长
        self.max_level = cfg.getint("sparse", "max_level")        # 金字塔层数
        self._pts = None                                          # 上一帧角点(跨帧状态)

    def _detect(self, gray):
        """在灰度图上检测 Shi-Tomasi 角点(点数不足时重新选点)。"""
        return cv2.goodFeaturesToTrack(                           # 返回角点坐标 (N,1,2)
            gray,                                                 # 输入灰度图
            maxCorners=self.max_corners,                          # 最多点数
            qualityLevel=self.quality,                            # 质量阈值(0~1)
            minDistance=self.min_dist,                            # 点间最小间距
        )

    def _track(self, gray_prev, gray, pts_prev):
        """用 PyrLK 把上一帧角点跟踪到当前帧, 返回(新点集, 平均位移)。"""
        if pts_prev is None or len(pts_prev) == 0:                # 没有可跟踪的点?
            return None, 0.0                                      # 返回空结果
        ws = self.win_size                                        # 取搜索窗口边长
        pts_nxt, status, _ = cv2.calcOpticalFlowPyrLK(            # LK 金字塔光流跟踪
            gray_prev, gray, pts_prev, None,                      # 上一帧/当前帧/旧点
            winSize=(ws, ws),                                     # 搜索窗口大小
            maxLevel=self.max_level,                              # 金字塔层数
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),  # 迭代终止条件
        )
        if pts_nxt is None:                                       # 整体跟踪失败?
            return None, 0.0                                      # 返回空结果
        good = status.ravel() == 1                                # 标记跟踪成功的点
        pts_ok = pts_nxt[good].reshape(-1, 1, 2)                  # 只保留成功的点
        if len(pts_ok) == 0:                                      # 成功点数为零?
            return None, 0.0                                      # 返回空结果
        dists = np.linalg.norm(pts_ok - pts_prev[good], axis=2).ravel()  # 每个点的移动距离
        return pts_ok, float(dists.mean())                        # 返回新点与平均位移

    @staticmethod
    def _draw(viz, pts_prev, pts_nxt):
        """在当前帧画运动箭头: 红色=明显移动, 绿色=基本静止。"""
        for p0, p1 in zip(pts_prev.reshape(-1, 2), pts_nxt.reshape(-1, 2)):  # 逐对(起点, 终点)
            a = tuple(p0.astype(int))                             # 起点像素坐标
            b = tuple(p1.astype(int))                             # 终点像素坐标
            moved = np.linalg.norm(p1 - p0) > 2.0                 # 位移超过 2 像素视为运动
            color = (0, 0, 255) if moved else (0, 255, 0)         # 红=动 绿=静
            cv2.arrowedLine(viz, a, b, color, 1, tipLength=0.3)   # 画箭头线段

    def process(self, gray_prev, gray, frame, motion_thr):
        """跟踪上一帧角点到当前帧, 返回(带箭头的当前帧, 平均位移, 0)。"""
        if self._pts is None or len(self._pts) < 10:              # 角点缺失或过少?
            self._pts = self._detect(gray_prev)                   # 在上一帧重新检测角点
        pts_nxt, mean_motion = self._track(gray_prev, gray, self._pts)  # 跟踪到当前帧
        viz = frame.copy()                                        # 以当前帧画面为底图
        if self._pts is not None and pts_nxt is not None:         # 跟踪有结果?
            self._draw(viz, self._pts, pts_nxt)                   # 画运动箭头
            self._pts = pts_nxt                                   # 用新点接续下一帧
        return viz, mean_motion, 0.0                              # 稀疏模式运动占比恒为 0
