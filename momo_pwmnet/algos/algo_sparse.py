#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# algo_sparse.py - PyrLK 稀疏跟踪(纯函数式, mode = sparse)
# ============================================================
# 说明:
#   1. 由 optical_flow_demo.py 按 config.ini 的 mode = sparse 动态加载
#   2. 本文件只暴露一个纯函数 process(); 唯一的跨帧状态(上一帧角点)
#      通过 state 参数显式传入、在返回值中传出, 由调用方持有
#   3. 参数从 config.ini 的 [sparse] 段实时读取
# 统一接口(与 algo_dense.py / algo_dis.py 完全一致):
#   process(prev_gray, gray, frame, cfg, state=None)
#       -> (viz, mean_motion, moving_ratio, state)
#   state: dict {"pts": (N,1,2) float32 上一帧角点} 或 None
# 想改算法? 只改本文件内部即可; 想新增算法? 复制本文件改名并改 mode
# ============================================================

import cv2                                     # OpenCV: 角点检测与 LK 光流
import numpy as np                             # NumPy: 坐标数组运算


def _detect(gray, p):
    """在灰度图上检测 Shi-Tomasi 角点, 返回 (N,1,2) float32; 无点时返回空数组。"""
    found = cv2.goodFeaturesToTrack(                        # 检测角点
        gray,                                               # 输入灰度图
        maxCorners=p["max_corners"],                        # 最多点数
        qualityLevel=p["quality"],                          # 质量阈值(0~1)
        minDistance=p["min_dist"],                          # 点间最小间距
    )
    if found is None:                                       # 场景太单调检测不到?
        return np.empty((0, 1, 2), dtype=np.float32)        # 返回空点集
    return found                                            # 返回角点数组


def _track(prev_gray, gray, pts, p):
    """用 PyrLK 把 pts 从 prev_gray 跟踪到 gray。

    返回:
        (pts_ok, mean_disp): 跟踪成功的点集 (M,1,2) 与平均位移(像素)
        全部跟丢时返回 (None, 0.0)
    """
    ws = p["win_size"]                                      # 取搜索窗口边长
    pts_nxt, status, _ = cv2.calcOpticalFlowPyrLK(          # LK 金字塔光流跟踪
        prev_gray, gray, pts, None,                         # 上一帧/当前帧/旧点
        winSize=(ws, ws),                                   # 搜索窗口大小
        maxLevel=p["max_level"],                            # 金字塔层数
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),  # 终止条件
    )
    if pts_nxt is None:                                     # 整体跟踪失败?
        return None, 0.0                                    # 返回空结果
    good = status.ravel() == 1                              # 标记跟踪成功的点
    pts_ok = pts_nxt[good].reshape(-1, 1, 2)                # 只保留成功的点
    if len(pts_ok) == 0:                                    # 成功点数为零?
        return None, 0.0                                    # 返回空结果
    dists = np.linalg.norm(pts_ok - pts[good], axis=2).ravel()  # 每个点的移动距离
    return pts_ok, float(dists.mean())                      # 返回新点与平均位移


def _draw(viz, pts_prev, pts_nxt):
    """在当前帧上画运动箭头: 红色=位移>2像素, 绿色=基本静止。"""
    for p0, p1 in zip(pts_prev.reshape(-1, 2), pts_nxt.reshape(-1, 2)):  # 逐对(起, 终)
        a = tuple(p0.astype(int))                           # 起点像素坐标
        b = tuple(p1.astype(int))                           # 终点像素坐标
        moved = np.linalg.norm(p1 - p0) > 2.0               # 位移超过 2 像素视为运动
        color = (0, 0, 255) if moved else (0, 255, 0)       # 红=动 绿=静
        cv2.arrowedLine(viz, a, b, color, 1, tipLength=0.3) # 画箭头线段


def process(prev_gray, gray, frame, cfg, state=None):
    """把上一帧角点跟踪到当前帧并画出运动箭头(纯函数, 状态显式传递)。

    参数:
        prev_gray: 上一帧灰度图(H, W), uint8
        gray:      当前帧灰度图(H, W), uint8
        frame:     当前帧彩色图(BGR), 作为箭头底图
        cfg:       configparser 对象, 读取 [sparse] 段参数
        state:     上一帧的状态 dict {"pts": (N,1,2) float32} 或 None
    返回:
        (viz, mean_motion, moving_ratio, state):
        viz           带运动箭头的彩色帧(BGR)
        mean_motion   跟踪点的平均位移(像素/帧), 无跟踪点时 0.0
        moving_ratio  稀疏模式不统计, 恒为 0.0
        state         新状态 dict {"pts": ...}, 调用方须传给下一帧
    """
    # --- 从 config 读取本算法全部参数 ---
    p = {
        "max_corners": cfg.getint("sparse", "max_corners"),       # 最多角点数
        "quality": cfg.getfloat("sparse", "quality_level"),       # 角点质量阈值
        "min_dist": cfg.getint("sparse", "min_distance"),         # 角点最小间距
        "win_size": cfg.getint("sparse", "win_size"),             # LK 窗口边长
        "max_level": cfg.getint("sparse", "max_level"),           # 金字塔层数
    }
    prev_pts = state.get("pts") if state else None          # 取上一帧角点(显式状态)

    # --- 角点不足时重新检测(在上一帧上找特征点) ---
    if prev_pts is None or len(prev_pts) < 10:              # 无点或剩余太少?
        prev_pts = _detect(prev_gray, p)                    # 重新检测角点
    if len(prev_pts) == 0:                                  # 依然没有角点?
        return frame.copy(), 0.0, 0.0, None                 # 返回空结果, 下帧再试

    # --- 跟踪上一帧角点到当前帧 ---
    pts_nxt, mean_motion = _track(prev_gray, gray, prev_pts, p)  # 跟踪旧点
    viz = frame.copy()                                      # 以当前帧画面为底图
    if pts_nxt is None:                                     # 点全部跟丢?
        return viz, 0.0, 0.0, {"pts": prev_pts}             # 保留旧点, 下帧再试
    _draw(viz, prev_pts, pts_nxt)                           # 画运动箭头
    return viz, mean_motion, 0.0, {"pts": pts_nxt}          # 返回渲染图与新状态
