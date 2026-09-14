"""
function.py — vp5.0 板端核心库（三大功能合一份）

本文件把 3 个原本独立的子模块合并到同一个文件，便于 front.py / bottom.py 一次性 import。
从顶到底的阅读顺序就是 3 个分节，建议按需跳读：

    ① HSV 巡线检测器 (LineDetector)
         - 适用于 bottom 相机走“引导线巡线”模式时的入口
         - 本仓库保留实现，但当前 front.py / bottom.py 未实际接线（仅走 YOLO）
         - 需要巡线时: detector = LineDetector(); detector.detect(frame)

    ② YOLO 检测器 (YoloDetector + YoloDetect + 通用辅助)
         - 当前 front.py / bottom.py 实际使用的检测入口
         - 适配 hbm (RDK 板端 BPU) / ultralytics (开发机) 两种后端
         - 上层调用: detector = YoloDetector(CFG['FRONT_YOLO']); detector.detect(frame, nv12=...)

    ③ 水下图像预处理 (preprocess / _underwater_color_correct / _apply_clahe)
         - 颜色校正 + 高斯去噪 + CLAHE 三个独立开关
         - 被 front.py / bottom.py / LineDetector 三处复用

外部依赖:
    cv2 / numpy / copy
    main_config.DEFAULT_CONFIG        (来自 main_config.py)
    utils.py_utils.preprocess / postprocess  (YOLO 前后处理, 与本文件同级目录)

阅读建议: 第一次接触先看 ②(YoloDetector.detect 是主入口)，再按需要跳读 ① 和 ③。
"""

import copy

import cv2
import numpy as np

from main_config import DEFAULT_CONFIG


# =====================================================================
# ① HSV 巡线检测器 (LineDetector)
# =====================================================================
class LineDetector:
    """
    巡线检测器 (HSV 引导线识别)

    用法:
        detector = LineDetector(config)
        result = detector.detect(frame)

        if result['detected']:
            print(f"偏移: {result['offset']:.1f}px, 角度: {result['angle']:.1f}°")
    """
    def __init__(self, config=None):
        """
        初始化检测器
        Args:
            config: dict, 配置参数字典.
                    未提供时使用 main_config.py 中的 DEFAULT_CONFIG.
                    建议传入 BOTTOM_HSV 配置段 (含 HSV 阈值/形态学/角度消抖参数).
                    支持运行时通过 update_config() 更新.
        """
        if config is not None:
            self.config = copy.deepcopy(config)
        else:
            self.config = copy.deepcopy(DEFAULT_CONFIG)

        # 缓存上一帧的检测结果, 用于连续帧丢失时的降级策略
        self._last_valid_result = None

        # 角度消抖: 滑动窗口历史队列
        self._angle_history = []
        self._work_scale = 1.0   # 当前工作图缩放系数 (hsv_work_scale, 1.0=不缩放)

    # ================================================================
    # 公开接口
    # ================================================================

    def detect(self, frame, preprocessed=False):
        """
        检测引导线 (主入口)

        Args:
            frame: BGR 图像 (numpy ndarray)
            preprocessed: True 表示 frame 已由外部统一预处理过, 跳过内部
                          preprocess 直接复用 (避免同一帧被重复预处理).

        Returns:
            dict: {
                'offset':   float,   引导线中心偏离垂直中心线的像素数
                                     正值=偏右, 负值=偏左
                'angle':    float,   引导线与垂直方向的夹角 (度)
                'detected': bool,    是否成功检测到引导线
                'center':   (x, y),  引导线重心在画面中的坐标
            }
        """
        # ---- 0. 可选下采样: 小图执行 CLAHE/形态学, 大幅降 CPU ----
        scale = float(self.config.get('hsv_work_scale', 1.0))
        self._work_scale = scale
        work = frame
        if 0 < scale < 1.0:
            h, w = frame.shape[:2]
            work = cv2.resize(frame, (max(1, int(w * scale)),
                                      max(1, int(h * scale))),
                              interpolation=cv2.INTER_LINEAR)

        # ---- 1. 预处理 (调用共享模块) ----
        if preprocessed:
            processed = work          # 外部已统一预处理, 直接复用, 不再二次处理
        else:
            processed = preprocess(work, self.config)

        # ---- 2. HSV 颜色分割 ----
        mask = self._color_segment(processed)

        # ---- 3. 形态学处理 ----
        mask = self._morphology_process(mask)

        # ---- 4. 提取引导线, 计算偏移量和角度 ----
        result = self._extract_line(mask)

        # 下采样时把 center/offset 还原回原图分辨率 (angle 与分辨率无关)
        if 0 < scale < 1.0 and result['detected']:
            result['center'] = (result['center'][0] / scale,
                                result['center'][1] / scale)
            result['offset'] = result['offset'] / scale

        # 降级策略: 如果当前帧未检测到但上一帧有效, 保留上一帧结果供参考
        if not result['detected'] and self._last_valid_result is not None:
            result['last_valid'] = self._last_valid_result

        if result['detected']:
            # ---- 5. 角度消抖 (死区 + 滑动窗口滤波) ----
            result['angle'] = self._debounce_angle(result['angle'])

            self._last_valid_result = {
                'offset': result['offset'],
                'angle': result['angle'],
                'center': result['center'],
            }

        return result

    def update_config(self, new_config):
        """
        运行时更新配置 (例如测试工具滑动条回调)

        Args:
            new_config: dict, 需要更新的参数
        """
        self.config.update(new_config)

    def draw_result(self, frame, result, show_mask=False, fps=None):
        """
        在图像上叠加检测结果 (调试/可视化用)

        Args:
            frame:     原始 BGR 图像
            result:    detect() 返回的结果字典
            show_mask: 是否在右侧拼接显示二值掩码
            fps:       当前处理帧率 (float, 可选)

        Returns:
            vis: 叠加了标注的图像 (BGR)
        """
        vis = frame.copy()
        h, w = vis.shape[:2]
        cx_img = w // 2  # 图像垂直中心线位置

        if result['detected']:
            cx, cy = result['center']
            cx, cy = int(cx), int(cy)

            # 绘制垂直中心线 (绿色虚线, 作为参考)
            for y in range(0, h, 20):
                cv2.line(vis, (cx_img, y), (cx_img, y + 10),
                         (0, 255, 0), 1)

            # 绘制引导线重心 (红色圆点)
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)

            # 绘制偏移量箭头
            cv2.arrowedLine(vis, (cx_img, cy), (cx, cy),
                            (255, 255, 0), 2, tipLength=0.3)

            # 绘制拟合的引导线方向
            length = 80
            angle_rad = np.radians(result['angle'])
            dx = int(length * np.sin(angle_rad))
            dy = int(length * np.cos(angle_rad))
            cv2.line(vis,
                     (cx - dx, cy - dy),
                     (cx + dx, cy + dy),
                     (255, 0, 255), 2)

            # 文字信息
            cv2.putText(vis, f"Offset: {result['offset']:+.1f}px",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)
            cv2.putText(vis, f"Angle:  {result['angle']:+.1f}deg",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)
        else:
            # 未检测到引导线
            cv2.putText(vis, "NO LINE DETECTED",
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255), 2)

        # 可选: 帧率显示 (右上角, 避免与左上角文字重叠)
        if fps is not None:
            fps_text = f"FPS: {fps:.1f}"
            (tw, _), _ = cv2.getTextSize(
                fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.putText(vis, fps_text,
                        (w - tw - 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)

        # 可选: 拼接掩码图
        if show_mask and 'mask' in result:
            mask_vis = cv2.cvtColor(result['mask'], cv2.COLOR_GRAY2BGR)
            vis = np.hstack([vis, mask_vis])

        return vis

    # ================================================================
    # 内部检测流水线
    # ================================================================

    def _color_segment(self, frame):
        """
        HSV 颜色空间橙红色分割

        橙红色在 HSV 空间中的位置跨 0° 边界:
        - 低H段: [0 ~ 25]  (橙-红色)
        - 高H段: [150 ~ 179] (红-品色, 跨边界)

        两段合并得到完整的橙红色掩码.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 主段 (橙-红色)
        lower1 = np.array(self.config['hsv_lower'])
        upper1 = np.array(self.config['hsv_upper'])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        # 跨边界段 (红色)
        if self.config.get('enable_red_wrap', True):
            lower2 = np.array(self.config['hsv_lower2'])
            upper2 = np.array(self.config['hsv_upper2'])
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1

        return mask

    def _morphology_process(self, mask):
        """
        形态学后处理: 开运算去噪 + 闭运算连接断线

        引导线在水下可能因反光、遮挡、水质浑浊而断裂,
        闭运算可弥合小断口, 提高轮廓的连续性.
        """
        # 下采样时按 scale 折算核尺寸, 保持与原分辨率等效的去噪/连接效果
        s = getattr(self, '_work_scale', 1.0)
        open_k = max(1, int(round(self.config['morph_open_kernel'] * s)) | 1)
        close_k = max(1, int(round(self.config['morph_close_kernel'] * s)) | 1)

        kernel_open = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_k, open_k))
        kernel_close = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_k, close_k))

        # 开运算: 先腐蚀再膨胀 → 去除细小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        # 闭运算: 先膨胀再腐蚀 → 连接邻近断裂
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        return mask

    def _extract_line(self, mask):
        """
        从二值掩码中提取引导线, 计算偏移量和角度

        步骤:
          1. (可选) ROI 限制
          2. 查找轮廓, 取最大轮廓
          3. 面积过滤
          4. 计算重心 → offset
          5. 拟合直线 → angle
        """
        h, w = mask.shape
        image_center_x = w / 2.0

        # ---- ROI 限制: 只关注画面中间区域 ----
        if self.config.get('roi_enabled', False):
            roi = self.config['roi_ratio']
            y1, y2 = int(h * roi[0]), int(h * roi[1])
            x1, x2 = int(w * roi[2]), int(w * roi[3])
            # 将 ROI 外区域置零
            roi_mask = np.zeros_like(mask)
            roi_mask[y1:y2, x1:x2] = 255
            mask = cv2.bitwise_and(mask, roi_mask)

        # ---- 查找轮廓 ----
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return self._empty_result()

        # ---- 取最大轮廓 ----
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # 下采样时面积阈值按 scale² 折算 (面积随分辨率平方缩小)
        s = getattr(self, '_work_scale', 1.0)
        if area < self.config['min_contour_area'] * s * s:
            return self._empty_result()

        # ---- 计算重心 (图像矩) ----
        M = cv2.moments(largest)
        if M['m00'] < 1e-6:
            return self._empty_result()

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']

        # ---- 偏移量: 引导线中心到图像垂直中心线的水平距离 ----
        offset = cx - image_center_x

        # ---- 角度: 拟合直线与垂直方向的夹角 ----
        angle = self._calc_vertical_angle(largest)

        return {
            'offset': round(float(offset), 2),
            'angle': round(float(angle), 2),
            'detected': True,
            'center': (round(float(cx), 2), round(float(cy), 2)),
        }

    def _calc_vertical_angle(self, contour):
        """
        计算轮廓拟合直线的方向与垂直方向 (y轴) 的夹角

        使用自实现的总体最小二乘法 (PCA-TLS) 拟合直线, 替代 cv2.fitLine.
        垂直方向向量为 (0, 1), 拟合直线方向为 (vx, vy).
        atan2(vx, vy) = 偏离垂直方向的角度.

        Args:
            contour: OpenCV 轮廓, 形状为 (N, 1, 2)

        Returns:
            float: 角度 (度), 正值=线向右偏, 负值=线向左偏
        """
        # 提取轮廓点坐标, 形状 (N, 2)
        points = contour.reshape(-1, 2).astype(np.float64)

        # 自实现最小二乘拟合, 返回方向向量 (vx, vy)
        vx, vy, _, _ = self._least_squares_fit_line(points)

        angle_rad = np.arctan2(vx, vy)
        angle_deg = np.degrees(angle_rad)

        return float(angle_deg)

    @staticmethod
    def _least_squares_fit_line(points):
        """
        基于 PCA 的总体最小二乘 (Total Least Squares) 直线拟合.

        最小化所有点到直线的正交距离平方和, 等价于求协方差矩阵
        最大特征值对应的特征向量 (数据方差最大方向 = 直线方向).
        直线必经过所有点的质心.

        Args:
            points: (N, 2) numpy 数组, 每行一个点 (x, y)

        Returns:
            (vx, vy, x0, y0): 方向向量 (已统一 vy>0) + 直线上点(质心)
        """
        n = points.shape[0]

        # 点数不足时返回默认垂直方向
        if n < 2:
            return 0.0, 1.0, 0.0, 0.0

        # Step 1: 计算质心
        x_mean = np.mean(points[:, 0])
        y_mean = np.mean(points[:, 1])

        # Step 2: 中心化数据
        x_centered = points[:, 0] - x_mean
        y_centered = points[:, 1] - y_mean

        # Step 3: 构建 2x2 协方差矩阵
        sxx = np.sum(x_centered ** 2)
        syy = np.sum(y_centered ** 2)
        sxy = np.sum(x_centered * y_centered)
        cov = np.array([[sxx, sxy],
                        [sxy, syy]])

        # Step 4: 特征值分解 (eigh 针对实对称矩阵优化, 特征值升序)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # 最大特征值对应最后一列特征向量 = 直线方向向量
        vx = float(eigvecs[0, 1])
        vy = float(eigvecs[1, 1])

        # 统一方向: 确保 vy > 0, 避免拟合方向反转导致角度跳变
        if vy < 0:
            vx, vy = -vx, -vy

        return vx, vy, float(x_mean), float(y_mean)

    def _debounce_angle(self, angle):
        """
        角度消抖: 小角度死区 + 滑动窗口滤波.

        解决引导线静止时角度仍持续波动的问题. 两级处理:
          1. 死区: |angle| < angle_deadzone 时直接输出 0.0
          2. 滑动窗口: 取最近 N 帧角度的中值/均值, 抑制突发跳变

        Args:
            angle: 当前帧原始角度 (度)

        Returns:
            float: 消抖后的角度 (度)
        """
        # 总开关
        if not self.config.get('enable_angle_debounce', True):
            return angle

        # ---- 第一级: 小角度死区 ----
        deadzone = float(self.config.get('angle_deadzone', 3.0))
        if abs(angle) < deadzone:
            angle = 0.0

        # ---- 第二级: 滑动窗口滤波 ----
        window = int(self.config.get('angle_smooth_window', 5))
        if window <= 1:
            return round(angle, 2)

        # 入队并保持窗口大小
        self._angle_history.append(angle)
        if len(self._angle_history) > window:
            self._angle_history = self._angle_history[-window:]

        method = self.config.get('angle_smooth_method', 'median')
        if method == 'mean':
            smoothed = float(np.mean(self._angle_history))
        else:
            # 默认中值滤波: 对突发异常值(如单帧轮廓提取错误)鲁棒
            smoothed = float(np.median(self._angle_history))

        return round(smoothed, 2)

    @staticmethod
    def _empty_result():
        """返回空检测结果"""
        return {
            'offset': 0.0,
            'angle': 0.0,
            'detected': False,
            'center': (0.0, 0.0),
        }



# =====================================================================
# ② YOLO 检测器 (YoloDetector + YoloDetect + 通用辅助)
# =====================================================================
"""
YOLO 检测模块 (module_YOLO.py)

从 main.py 拆分出的 YOLO 目标检测相关代码:
  - 通用辅助: normalize_name / resolve_model_path / load_labels
  - 可视化:   draw_detections / format_detection
  - 默认计时: NullTimer / NULL_TIMER (未传入 StageTimer 时使用)
  - YoloDetectConfig / YoloDetect : hbm (RDK 板端 BPU) 推理封装
  - YoloDetector                  : hbm / ultralytics 双后端适配层

依赖: cv2 / numpy / dataclasses / typing / os / utils.py_utils (preprocess/postprocess)
"""

import os

import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple

# YOLO 预处理/后处理工具 (utils/py_utils, 与 main.py 同级)
try:
    import utils.py_utils.preprocess as pre_utils
    import utils.py_utils.postprocess as post_utils
except ImportError:
    pre_utils = post_utils = None
    print("[警告] 未找到 utils/py_utils (需要从 v4.1(board) 目录运行 main.py), YOLO 功能将不可用")


# ======================
# 通用辅助函数
# ======================

def normalize_name(name):
    """标准化类别名, 兼容 大小写/空格/下划线/短横线 差异."""
    return str(name).strip().lower().replace('_', ' ').replace('-', ' ')


def resolve_model_path(model_path):
    """解析模型/标签路径: 相对路径基于 main.py 所在目录 (与 hbm 模型同级) 展开."""
    _base_dir = os.path.dirname(os.path.abspath(__file__))
    if not os.path.isabs(model_path):
        model_path = os.path.join(_base_dir, model_path)
    return model_path


def load_labels(label_file):
    """读取类别名称文件 (每行一个类名), 未配置/不存在时返回空列表."""
    if not label_file or not os.path.exists(label_file):
        return []
    with open(label_file, 'r', encoding='utf-8') as f:
        return [line.strip() for line in f if line.strip()]


# ======================
# 默认计时器 (未传入 StageTimer 时使用)
# ======================

class NullTimer:
    """空计时器: 关闭阶段统计时使用 (接口与 StageTimer 一致, 近零开销)."""

    def measure(self, stage):
        """返回一个空测量上下文管理器（不记录任何耗时）。"""
        return _NullMeasure()


class _NullMeasure:
    """空测量上下文管理器：与 NullTimer 配套，未启用阶段统计时 measure() 返回它，近零开销且接口兼容 StageTimer。"""
    __slots__ = ()

    def __enter__(self):
        """进入上下文：返回自身。"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文：不吞异常，原样返回 False。"""
        return False


NULL_TIMER = NullTimer()   # 默认计时器 (不使用阶段统计时)


# ======================
# 可视化辅助
# ======================

def draw_detections(frame, detections):
    """在帧上绘制检测结果 (外接框 + 中心点 + 类别置信度)."""
    vis = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        cx, cy = det['center']
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)   # 黄色外接框
        cv2.circle(vis, (cx, cy), 4, (0, 255, 0), -1)              # 绿色中心点
        cv2.putText(vis, f"{det['label']} {det['score']:.2f}",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return vis


def format_detection(det, mark_point, idx):
    """格式化单个 YOLO 检测: 标号 + label + 中心点 + 相对相机标识点的偏移.

    相机标识点来自各相机配置的 mark_point (画面参考点像素坐标);
    相对偏移 = 目标中心 - 标识点, x 正=目标偏右, y 正=目标偏下.
    """
    cx, cy = det['center']
    if mark_point:
        mx, my = mark_point
        rel = f"相对标记点=({cx - mx:+d},{cy - my:+d})"
    else:
        rel = '相对标记点=(-,-)'
    return (f"#{idx} {det['label']} 中心=({cx},{cy}) {rel} conf={det['score']:.2f}")


# ======================
# YOLO 检测类
# ======================

@dataclass
class YoloDetectConfig:
    """YoloDetect 模型初始化配置。

    该数据类保存模型路径以及 YOLO 检测流水线中预处理、推理、后处理所需的
    全部运行时参数，适用于所有基于 DFL 的 YOLO 检测模型（v5u、v8、v11、v12）。

    属性说明：
        model_path: 编译好的 YOLO 检测模型 `.hbm` 路径。
        classes_num: 检测类别数量，默认 80（COCO）。
        resize_type: 预处理时的图像缩放方式。
            - 0: 直接拉伸缩放。
            - 1: 保持宽高比的 letterbox 填充。
        score_thres: 过滤检测结果的置信度阈值。
        nms_thres: 非极大值抑制（NMS）使用的 IoU 阈值。
        reg: 每个框边 DFL 回归的离散分箱数量，默认 16。
        strides: 各检测尺度的特征图下采样倍率。
        anchor_sizes: 各检测尺度的特征图网格尺寸（像素）。
    """
    model_path: str
    classes_num: int = 80
    resize_type: int = 1
    score_thres: float = 0.25
    nms_thres: float = 0.45
    reg: int = 16
    strides: list = field(default_factory=lambda: [8, 16, 32])
    anchor_sizes: list = field(default_factory=lambda: [80, 40, 20])


class YoloDetect:
    """基于 HB_HBMRuntime 的 YOLO DFL 检测封装。

    该类为 DFL 型 YOLO 检测模型（v5u、v8、v11、v12）提供统一的推理流水线，
    包括输入预处理、模型执行以及后处理步骤（无锚框 DFL 框解码、置信度过滤、
    非极大值抑制 NMS 等）。

    说明:
        hbm_runtime 为 RDK 板端运行时库, 采用延迟导入:
        - 板端 (已安装 hbm_runtime): 正常使用
        - 开发机 (未安装): 可在 YoloDetector 适配层改用 ultralytics 后端
    """

    def __init__(self, config: YoloDetectConfig, timer=None):
        """根据给定配置初始化 YoloDetect 模型。

        Args:
            config: 模型配置 (YoloDetectConfig)
            timer:  StageTimer 实例; 用于统计 预处理/推理/后处理 各阶段耗时
        """
        self.timer = timer or NULL_TIMER
        try:
            import hbm_runtime
        except ImportError as exc:
            raise RuntimeError(
                'hbm 后端需要 RDK 板端环境 (hbm_runtime); '
                '开发机请在配置中将 backend 改为 "ultralytics"'
            ) from exc

        # 加载模型并提取元数据
        self.model = hbm_runtime.HB_HBMRuntime(config.model_path)

        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]

        # 模型输入分辨率 (H, W)，从输入张量形状推断
        self.input_h = self.input_shapes[self.input_names[0]][1]
        self.input_w = self.input_shapes[self.input_names[0]][2]

        # DFL 权重：形状 (1, 1, reg)，用于计算期望框偏移
        self.weights_static = np.arange(config.reg, dtype=np.float32)[np.newaxis, np.newaxis, :]

        # 保存配置
        self.cfg = config

    def set_scheduling_params(self,
                              priority: Optional[int] = None,
                              bpu_cores: Optional[list] = None) -> None:
        """配置推理调度参数。

        Args:
            priority: 推理优先级，取值范围 [0, 255]。
            bpu_cores: 用于推理的 BPU 核心索引列表。
        """
        kwargs = {}
        if priority is not None:
            kwargs["priority"] = {self.model_name: priority}
        if bpu_cores is not None:
            kwargs["bpu_cores"] = {self.model_name: bpu_cores}

        if kwargs:
            self.model.set_scheduling_params(**kwargs)

    def pre_process(self,
                    img: Optional[np.ndarray] = None,
                    image_format: Optional[str] = "BGR",
                    nv12: Optional[np.ndarray] = None,
                    ) -> Dict[str, Dict[str, np.ndarray]]:
        """将输入图像预处理为模型所需的张量格式。

        image_format="BGR" 时 img 为 BGR 帧 (传统路径, resize + 转 NV12);
        image_format="NV12" 时 nv12 为 JPU 解出的原始 NV12 (2D H*3//2 x W),
        直接对 Y/UV 平面 letterbox 喂 BPU, 省掉两次颜色转换.
        """
        if pre_utils is None:
            raise RuntimeError('utils.py_utils 未加载, 无法执行模型预处理')
        if image_format == "BGR":
            if img is None:
                raise ValueError('image_format=BGR 时必须提供 img')
            resize_img = pre_utils.resized_image(
                img, self.input_w, self.input_h, self.cfg.resize_type)
            y, uv = pre_utils.bgr_to_nv12_planes(resize_img)
        elif image_format in ("NV12", "nv12"):
            if nv12 is None:
                raise ValueError('image_format=NV12 时必须提供 nv12 数组')
            ori_h, ori_w = nv12.shape[0] * 2 // 3, nv12.shape[1]
            y, uv = pre_utils.preprocess_nv12(
                nv12, ori_w, ori_h,
                self.input_w, self.input_h)
        else:
            raise ValueError(f"不支持的图像格式: {image_format}")
        # 统一补 batch/channel 维: images_y[1,640,640,1], images_uv[1,320,320,2]
        if y.ndim == 2:
            y = y[np.newaxis, :, :, np.newaxis]
        if uv.ndim == 3:
            uv = uv[np.newaxis, :, :, :]

        return {
            self.model_name: {
                self.input_names[0]: y,
                self.input_names[1]: uv
            }
        }

    def forward(self, input_tensor: Dict[str, Dict[str, np.ndarray]]) -> Dict[str, np.ndarray]:
        """执行模型推理。"""
        outputs = self.model.run(input_tensor)
        return outputs

    def post_process(self,
                     outputs: Dict[str, Dict[str, np.ndarray]],
                     ori_img_w: int,
                     ori_img_h: int,
                     score_thres: Optional[float] = None,
                     nms_thres: Optional[float] = None,
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """将模型原始输出转换为最终检测结果。

        该步骤包括无锚框 DFL 框解码、置信度过滤、非极大值抑制（NMS）
        以及将坐标缩放回原始图像分辨率。
        """
        if post_utils is None:
            raise RuntimeError('utils.py_utils 未加载, 无法执行模型后处理')
        score_thres = score_thres if score_thres is not None else self.cfg.score_thres
        nms_thres = nms_thres if nms_thres is not None else self.cfg.nms_thres

        # 计算原始 logit 过滤所需的逆 sigmoid 阈值
        conf_thres_raw = -np.log(1.0 / score_thres - 1)

        # 第一步：解码各检测尺度的 分类输出 与 框分布输出
        model_outputs = outputs[self.model_name]
        all_boxes = []
        all_scores = []
        all_ids = []
        for i, (stride, anchor_size) in enumerate(
                zip(self.cfg.strides, self.cfg.anchor_sizes)):
            cls_key = self.output_names[2 * i]      # 分类 logits 输出
            box_key = self.output_names[2 * i + 1]  # DFL 框分布输出

            # 在 sigmoid 之前先按原始 logit 阈值过滤
            scores, ids, valid_indices = post_utils.filter_classification(
                model_outputs[cls_key], conf_thres_raw)

            # 对有效预测解码 DFL 边界框
            dbboxes = post_utils.decode_boxes(
                model_outputs[box_key], valid_indices,
                anchor_size, stride, self.weights_static)

            all_boxes.append(dbboxes)
            all_scores.append(scores)
            all_ids.append(ids)

        # 第二步：拼接所有检测尺度的结果
        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        cls_ids = np.concatenate(all_ids, axis=0)

        # 第三步：非极大值抑制
        keep = post_utils.NMS(boxes, scores, cls_ids, nms_thres)

        # 第四步：将框坐标缩放回原始图像尺寸
        xyxy = post_utils.scale_coords_back(
            boxes[keep], ori_img_w, ori_img_h,
            self.input_w, self.input_h, self.cfg.resize_type)

        return xyxy, scores[keep], cls_ids[keep]

    def predict(self,
                img: Optional[np.ndarray] = None,
                image_format: str = "BGR",
                nv12: Optional[np.ndarray] = None,
                score_thres: Optional[float] = None,
                nms_thres: Optional[float] = None,
                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """在单张图像上运行完整的检测流水线。

        image_format="NV12" 时 img 传 None、nv12 传 JPU 解出的原始 NV12 (2D H*3//2 x W).
        """
        if image_format in ("NV12", "nv12"):
            ori_img_h, ori_img_w = nv12.shape[0] * 2 // 3, nv12.shape[1]
        else:
            ori_img_h, ori_img_w = img.shape[:2]

        # 1) 预处理 (阶段耗时分阶段统计: YOLO预处理/推理/后处理)
        with self.timer.measure('YOLO预处理'):
            input_tensor = self.pre_process(img, image_format=image_format,
                                            nv12=nv12)

        # 2) 推理
        with self.timer.measure('推理'):
            outputs = self.forward(input_tensor)

        # 3) 后处理
        with self.timer.measure('后处理'):
            boxes, scores, cls_ids = self.post_process(
                outputs, ori_img_w, ori_img_h, score_thres, nms_thres)

        return boxes, scores, cls_ids

    def __call__(self,
                 img: Optional[np.ndarray] = None,
                 image_format: str = "BGR",
                 nv12: Optional[np.ndarray] = None,
                 score_thres: Optional[float] = None,
                 nms_thres: Optional[float] = None,
                 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """检测流水线的可调用接口 (等价于 predict())."""
        return self.predict(img, image_format=image_format, nv12=nv12,
                            score_thres=score_thres,
                            nms_thres=nms_thres)


class YoloDetector:
    """YOLO 检测器适配层: 统一 hbm (RDK 板端 BPU) / ultralytics (开发机) 两种后端.

    对上层提供统一接口:
      detect(frame) -> detections: [{'label','score','bbox','center'}, ...]
    """

    def __init__(self, cfg: dict, timer=None):
        """初始化检测器：解析后端(hbm/ultralytics)、预处理模式、类别名与目标过滤，并按 backend 加载对应模型（hbm 走 BPU，ultralytics 走开发机 .pt）。"""
        self.cfg = cfg
        self.backend = cfg.get('backend', 'hbm')
        self.timer = timer or NULL_TIMER   # 阶段计时 (推理/后处理)
        # 预处理模式 (config: preprocess_mode)
        self.preprocess_mode = str(cfg.get('preprocess_mode', 'auto')).lower()
        # 类别名列表: 优先 config 内联 class_names, 其次 label_file 文件
        self.labels = self._resolve_labels(cfg)
        # 目标类别过滤: target_class_ids (数字) 或 target_class_names (名称), 二选一
        self.target_ids = [int(i) for i in (cfg.get('target_class_ids') or [])]
        names = cfg.get('target_class_names') or []
        self.targets = {normalize_name(n) for n in names} if names else set()
        if self.targets:
            if not self.labels:
                print('[警告] 配置了 target_class_names 但未提供类别名 '
                      '(class_names / label_file), 无法按名称过滤, 将保留全部检测结果')
            else:
                label_set = {normalize_name(l) for l in self.labels}
                missing = [n for n in names if normalize_name(n) not in label_set]
                if missing:
                    raise ValueError(
                        f"target_class_names 中的类别 {missing} 不在类别列表 {self.labels} 中; "
                        '请检查 class_names / label_file 的顺序与模型类别 id 一致')

        if self.backend == 'hbm':
            self._init_hbm()
        elif self.backend == 'ultralytics':
            self._init_ultralytics()
        else:
            raise ValueError(f'未知的检测后端: {self.backend}')

    # ---------- 类别名与过滤 ----------

    @staticmethod
    def _resolve_labels(cfg):
        """解析类别名列表: 优先 config 内联 class_names, 其次 label_file."""
        class_names = cfg.get('class_names') or []
        labels = [str(n).strip() for n in class_names if str(n).strip()]
        if labels:
            return labels
        label_file = cfg.get('label_file')
        if label_file:
            return load_labels(resolve_model_path(label_file))
        return []

    def _is_target_cls(self, cls_id, label):
        """按类别 id 或名称过滤; 均未配置时保留全部.

        优先级: target_class_ids (数字) > target_class_names (名称).
        """
        if self.target_ids:
            return int(cls_id) in self.target_ids
        if self.targets:
            return normalize_name(label) in self.targets
        return True

    # ---------- 后端初始化 ----------

    def _init_hbm(self):
        """加载 hbm 模型 (YoloDetect, 需 RDK 板端 hbm_runtime)."""
        model_path = resolve_model_path(self.cfg['model_path'])
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'模型文件不存在: {model_path}')
        config = YoloDetectConfig(
            model_path=model_path,
            score_thres=self.cfg['score_thres'],
            nms_thres=self.cfg['nms_thres'],
            strides=self.cfg['strides'],
        )
        self.model = YoloDetect(config, timer=self.timer)
        # 特征图网格大小按模型实际输入分辨率自动计算 (320x320 -> [40,20,10])
        self.model.cfg.anchor_sizes = [self.model.input_h // s
                                       for s in self.model.cfg.strides]
        self.model.set_scheduling_params(
            priority=self.cfg.get('priority', 0),
            bpu_cores=self.cfg.get('bpu_cores', [0]))

    def _init_ultralytics(self):
        """加载 ultralytics .pt 模型 (开发机测试)."""
        model_path = resolve_model_path(self.cfg['model_path'])
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'模型文件不存在: {model_path}')
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('未安装 ultralytics, 请先执行: pip install ultralytics') from exc
        self.model = YOLO(model_path)
        # config 未提供类别名时, 自动从模型读取 (ultralytics 模型自带 names)
        if not self.labels and hasattr(self.model, 'names'):
            names = self.model.names
            if isinstance(names, dict):
                self.labels = [names[i] for i in sorted(names.keys())]
            elif isinstance(names, list):
                self.labels = list(names)

    # ---------- 统一检测接口 ----------

    def detect(self, frame, nv12=None):
        """对一帧执行 YOLO, 返回统一格式检测列表. 空帧/异常返回 [].

        nv12: JPU 硬解保留的原始 NV12 (2D H*3//2 x W). preprocess_mode 允许
              ('auto'/'nv12') 且为 hbm 后端时走 NV12 直通预处理 (省两次颜色转换);
              BGR 路径仅在无 NV12 源时使用.
        """
        if frame is None and nv12 is None:
            return []
        if self.backend == 'ultralytics':
            return self._detect_ultralytics(frame)
        return self._detect_hbm(frame, nv12)

    def _detect_hbm(self, frame, nv12=None):
        """hbm 后端: 单帧 predict (BPU 推理). NV12 优先直通."""
        use_nv12 = (nv12 is not None and self.preprocess_mode in ('auto', 'nv12'))
        if use_nv12:
            boxes, scores, cls_ids = self.model.predict(
                None, image_format='NV12', nv12=nv12)
        else:
            if frame is None and nv12 is not None:
                frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            boxes, scores, cls_ids = self.model.predict(frame)
        return self._format_detections(boxes, scores, cls_ids)

    def _detect_ultralytics(self, frame):
        """ultralytics 后端: 单帧 predict.

        该后端一次 predict 内含 预处理+推理+后处理 无法拆分,
        统一计入 '推理' 阶段.
        """
        with self.timer.measure('推理'):
            preds = self.model.predict(frame, conf=self.cfg['score_thres'],
                                       iou=self.cfg['nms_thres'], verbose=False)
        dets = []
        if preds and preds[0].boxes is not None:
            names = preds[0].names
            for box in preds[0].boxes:
                cls_id = int(box.cls.item())
                label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else names[cls_id]
                if not self._is_target_cls(cls_id, label):
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                dets.append(self._make_detection(label, float(box.conf.item()),
                                                 x1, y1, x2, y2))
        return dets

    def _format_detections(self, boxes, scores, cls_ids):
        """把 hbm 后处理的 (boxes, scores, cls_ids) 转为统一格式."""
        dets = []
        for box, score, cls_id in zip(boxes, scores, cls_ids):
            cls_id = int(cls_id)
            label = self.labels[cls_id] if self.labels and cls_id < len(self.labels) else str(cls_id)
            if not self._is_target_cls(cls_id, label):
                continue
            x1, y1, x2, y2 = [float(v) for v in box]
            dets.append(self._make_detection(label, float(score), x1, y1, x2, y2))
        return dets

    @staticmethod
    def _make_detection(label, score, x1, y1, x2, y2):
        """构造统一格式的单个检测结果字典."""
        return {
            'label': label,
            'score': float(score),
            'bbox': (int(x1), int(y1), int(x2), int(y2)),
            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2)),
        }

    def warmup(self, width=640, height=480):
        """首次推理预热 (BPU 上下文与算子初始化很慢, 不预热会拖慢启动)."""
        black = np.zeros((height, width, 3), dtype=np.uint8)
        try:
            self.detect(black)
        except Exception as exc:
            print(f'[警告] 模型预热失败: {exc!r}')



# =====================================================================
# ③ 水下图像预处理 (preprocess / _underwater_color_correct / _apply_clahe)
# =====================================================================
"""
预处理模块 (module_preprocess.py)

从 main.py 拆分出的通用水下图像预处理函数:
  - preprocess                 : 总入口 (颜色校正 + 高斯去噪 + CLAHE)
  - _underwater_color_correct  : 灰度世界白平衡 + 红色通道增强 (LUT 加速)
  - _apply_clahe               : LAB 空间 CLAHE 自适应直方图均衡化

依赖: cv2 / numpy
"""

import cv2
import numpy as np


def preprocess(frame, config=None):
    """
    通用水下图像预处理

    Args:
        frame: 输入图像 (BGR, numpy ndarray)
        config: 配置字典, 支持以下键 (未提供的键使用默认值):

            颜色校正:
            - enable_color_correct: bool, 是否颜色校正, 默认 True
            - red_boost:     float, 红色通道增强系数, 默认 1.2

            高斯去噪:
            - enable_gaussian:  bool, 是否高斯去噪, 默认 True
            - gaussian_kernel:  int, 高斯核大小 (奇数), 默认 5

            CLAHE:
            - enable_clahe:  bool, 是否 CLAHE 增强, 默认 True
            - clahe_clip:    float, CLAHE 对比度限幅, 默认 2.0
            - clahe_tile:    tuple, CLAHE 网格大小, 默认 (8, 8)

    Returns:
        processed: 预处理后的 BGR 图像 (numpy ndarray)
    """
    # ---- 合并默认配置 ----
    cfg = {
        # 颜色校正
        'enable_color_correct': True,
        'red_boost': 1.2,
        # 高斯去噪
        'enable_gaussian': True,
        'gaussian_kernel': 5,
        # CLAHE
        'enable_clahe': True,
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
    }

    #修正配置参数
    if config is not None:
        cfg.update(config)

    # 不拷贝原图: 以下各步骤 (颜色校正/高斯/CLAHE) 均返回新数组, 不会原地修改
    # frame, 直接复用输入引用即可省去一次全图拷贝.
    result = frame

    # ============================================================
    # 步骤 1: 水下颜色校正
    # ============================================================
    if cfg['enable_color_correct']:
        result = _underwater_color_correct(result, cfg['red_boost'])

    # ============================================================
    # 步骤 2: 高斯去噪
    # ============================================================
    if cfg['enable_gaussian']:
        ksize = cfg['gaussian_kernel']
        if ksize % 2 == 0:
            ksize += 1  # 确保为奇数
        result = cv2.GaussianBlur(result, (ksize, ksize), 0)

    # ============================================================
    # 步骤 3: CLAHE 自适应直方图均衡化
    # ============================================================
    if cfg['enable_clahe']:
        result = _apply_clahe(
            result,
            clip_limit=cfg['clahe_clip'],
            tile_grid_size=cfg['clahe_tile'],
        )

    return result


# ================================================================
# 辅助函数
# ================================================================

def _underwater_color_correct(frame, red_boost=1.2):
    """
    水下颜色校正

    原理: 水体对红光的吸收远大于蓝绿光, 导致水下图像偏蓝绿.
    本函数使用 "灰度世界假设" 做白平衡, 并额外增强红色通道.

    实现说明 (LUT 加速版):
      每通道增益是标量, 输出 = 输入 x 增益, 恰好构成一张 256 项查表 (LUT).
      用一次 cv2.LUT 全 uint8 完成, 相比原 float32 实现 (转 float、split、
      多次 np.mean、逐通道标量乘、clip、merge, 十余次全图遍历) 省掉绝大部分
      遍历, 且结果与原实现完全等价.

    Args:
        frame:     BGR 图像 (uint8)
        red_boost: 红色通道增强倍数, >1.0 增强, <1.0 抑制

    Returns:
        校正后的 BGR 图像 (uint8)
    """
    # cv2.mean 一次调用返回四通道均值 (C 层单遍扫描), 免去 split + 多次 np.mean
    mean_b, mean_g, mean_r, _ = cv2.mean(frame)
    gray_mean = (mean_b + mean_g + mean_r) / 3.0

    def _gain(m):
        """灰度世界增益; 通道均值过小 (接近纯色画面) 时退化为 1.0, 防止除零."""
        return gray_mean / m if m > 1e-6 else 1.0

    gb, gg, gr = _gain(mean_b), _gain(mean_g), _gain(mean_r) * red_boost

    # 每通道一张 256 项映射表, 合成 256x1x3 后一次查表完成三通道校正
    lut = np.empty((256, 1, 3), dtype=np.uint8)
    lut[:, 0, 0] = np.clip(np.arange(256) * gb, 0, 255)
    lut[:, 0, 1] = np.clip(np.arange(256) * gg, 0, 255)
    lut[:, 0, 2] = np.clip(np.arange(256) * gr, 0, 255)
    return cv2.LUT(frame, lut)


def _apply_clahe(frame, clip_limit=2.0, tile_grid_size=(8, 8)):
    """
    自适应直方图均衡化 (CLAHE)

    在 LAB 色彩空间的 L (亮度) 通道上操作, 保持色彩不失真.
    用于应对水下光照不均匀的情况.

    Args:
        frame:         BGR 图像
        clip_limit:    对比度限幅阈值, 越大对比度越强
        tile_grid_size: 网格划分, 越小局部性越强

    Returns:
        增强后的 BGR 图像
    """
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)

    clahe = cv2.createCLAHE(
        clipLimit=clip_limit,
        tileGridSize=tile_grid_size,
    )
    l_channel = clahe.apply(l_channel)

    lab = cv2.merge([l_channel, a_channel, b_channel])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
