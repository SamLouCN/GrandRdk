# -*- coding: utf-8 -*-
# ============================================================
# algo_pyrlk.py - 历史经典算法②: PyrLK 稀疏光流跟踪
# ============================================================
# 说明:
#   - 这是"原来的 sparse 模式"完整实现, 保留未删, 封装成独立文件
#   - 由主程序 optical_flow_demo.py 按 config 的 mode=sparse 动态加载
#   - 参数段: config.ini 的 [sparse]
# 特点: 只跟踪图像角点, 速度最快; 输出为箭头而非全图运动场
# ============================================================

import cv2           # OpenCV: 角点检测与 LK 光流跟踪
import numpy as np   # NumPy: 坐标数组运算


class PyrLKSparse:
    """PyrLK 稀疏光流跟踪算法对象(原 sparse 分支实现, 逐行注释)。"""

    def __init__(self, cfg):
        """从 config 读取本算法全部参数; 角点状态在对象内部跨帧保持。"""
        self.max_corners = cfg.getint("sparse", "max_corners")  # 取最多角点数
        self.quality = cfg.getfloat("sparse", "quality_level")  # 取角点质量阈值
        self.min_dist = cfg.getint("sparse", "min_distance")    # 取角点最小间距
        self.win_size = cfg.getint("sparse", "win_size")        # 取搜索窗口边长
        self.max_level = cfg.getint("sparse", "max_level")      # 取金字塔层数
        self.pts = None                                         # 上一帧角点(跨帧状态)

    def _detect(self, gray):
        """在灰度图上检测 Shi-Tomasi 角点, 返回 (N,1,2) 坐标数组。"""
        found = cv2.goodFeaturesToTrack(                        # 检测角点
            gray,                                               # 输入灰度图
            maxCorners=self.max_corners,                        # 最多点数
            qualityLevel=self.quality,                          # 质量阈值(0~1)
            minDistance=self.min_dist,                          # 点间最小间距
        )
        if found is None:                                       # 场景太单调检测不到?
            found = np.empty((0, 1, 2), dtype=np.float32)       # 返回空点集
        return found                                            # 返回角点数组

    def run(self, prev_gray, gray, frame=None):
        """把上一帧角点跟踪到当前帧, 画出运动箭头, 输出统计。

        参数:
            prev_gray: 上一帧灰度图
            gray:      当前帧灰度图
            frame:     当前帧彩色图(作为箭头底图; 为 None 时自动转灰度)
        返回:
            (viz, mean_motion, moving_ratio):
            viz          带运动箭头的彩色帧(BGR)
            mean_motion  跟踪点的平均位移(像素), 无点时返回 0
            moving_ratio 稀疏模式不统计, 恒为 0.0
        """
        if self.pts is None or len(self.pts) < 10:              # 无角点或剩余太少?
            self.pts = self._detect(prev_gray)                  # 在上一帧重新检测
        if len(self.pts) == 0:                                  # 依然没有角点?
            viz = self._base_image(frame, gray)                 # 底图(无箭头)
            return viz, 0.0, 0.0                                # 直接返回空结果

        pts_nxt, status, _ = cv2.calcOpticalFlowPyrLK(          # LK 金字塔光流跟踪
            prev_gray, gray, self.pts, None,                    # 上一帧/当前帧/旧点
            winSize=(self.win_size, self.win_size),             # 搜索窗口大小
            maxLevel=self.max_level,                            # 金字塔层数
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),  # 终止条件
        )
        if pts_nxt is None:                                     # 整体跟踪失败?
            viz = self._base_image(frame, gray)                 # 底图(无箭头)
            return viz, 0.0, 0.0                                # 返回空结果
        good = status.ravel() == 1                              # 标记跟踪成功的点
        pts_ok = pts_nxt[good].reshape(-1, 1, 2)                # 只保留成功的点
        if len(pts_ok) == 0:                                    # 一个点都没跟上?
            self.pts = self._detect(prev_gray)                  # 重置并重新检测
            viz = self._base_image(frame, gray)                 # 底图
            return viz, 0.0, 0.0                                # 返回空结果

        prev_ok = self.pts[good].reshape(-1, 1, 2)              # 对应的上一帧点
        dists = np.linalg.norm(pts_ok - prev_ok, axis=2).ravel()  # 每个点的移动距离
        mean_motion = float(dists.mean())                       # 平均位移
        viz = self._base_image(frame, gray)                     # 画底图
        for p0, p1 in zip(prev_ok.reshape(-1, 2), pts_ok.reshape(-1, 2)):  # 逐对画箭头
            a = tuple(p0.astype(int))                           # 起点像素坐标
            b = tuple(p1.astype(int))                           # 终点像素坐标
            moved = np.linalg.norm(p1 - p0) > 2.0               # 位移超过 2 像素视为运动
            color = (0, 0, 255) if moved else (0, 255, 0)       # 红=动 绿=静
            cv2.arrowedLine(viz, a, b, color, 1, tipLength=0.3) # 画箭头线段
        self.pts = pts_ok                                       # 用新点接续下一帧
        return viz, mean_motion, 0.0                            # 返回渲染图与平均位移

    def _base_image(self, frame, gray):
        """取箭头底图: 优先用传入彩色帧, 否则把灰度转成 BGR。"""
        if frame is not None:                                   # 主程序给了彩色帧?
            return frame.copy()                                 # 直接拷贝一份
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)           # 灰度转 BGR 兜底
