"""
function.py — 板端核心库（YOLO 检测 + 水下预处理）

从顶到底两节，建议按需跳读：

    ① YOLO 检测器 (YoloDetector + YoloDetect + 通用辅助)
         - front.py / bottom.py 实际使用的检测入口
         - 适配 hbm (RDK 板端 BPU) / ultralytics (开发机) 两种后端
         - 上层调用: detector = YoloDetector(CFG['FRONT_YOLO']); detector.detect(frame, nv12=...)

    ② 水下图像预处理 (preprocess / _underwater_color_correct / _apply_clahe)
         - 颜色校正 + 高斯去噪 + CLAHE 三个独立开关
         - 被 front.py / bottom.py 复用

外部依赖:
    cv2 / numpy
    main_config.DEFAULT_CONFIG                (来自 config/main_config.py)
    utils.py_utils.preprocess / postprocess   (来自 src/utils/)
"""

import os  # 路径解析与文件存在性判断
import sys  # 模块搜索路径注入
import time

# ---- 路径注入: config/ + src/ ----
_HERE = os.path.dirname(os.path.abspath(__file__))     # src/
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 项目根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE):  # 依次注入 config/ 与 src/
    if _p not in sys.path:  # 已存在则不重复插入
        sys.path.insert(0, _p)  # 插到最前，保证本地模块优先

import cv2  # OpenCV：颜色空间转换、滤波、CLAHE、绘制
import numpy as np  # 数值数组与查找表
from dataclasses import dataclass, field  # 配置数据类与可变默认值工厂
from typing import Optional, Dict, Tuple  # 类型标注

from main_config import DEFAULT_CONFIG  # 导入即校验配置模块可用

# YOLO 预处理/后处理工具 (utils/py_utils)
try:  # 开发机上可能没有 utils 包
    import utils.py_utils.preprocess as pre_utils  # 板端预处理：缩放、BGR 转 NV12 平面
    import utils.py_utils.postprocess as post_utils  # 板端后处理：阈值过滤、DFL 解码、NMS、坐标还原
except ImportError:  # 导入失败则功能降级
    pre_utils = post_utils = None  # 置空，后面对应位置会明确报错
    print("[警告] 未找到 utils/py_utils, YOLO 功能将不可用")  # 提示环境缺失


# =====================================================================
# ① YOLO 检测器 (YoloDetector + YoloDetect + 通用辅助)
# =====================================================================

# ======================
# 通用辅助函数
# ======================

def normalize_name(name):  # 统一类别名写法，便于按名称匹配
    """标准化类别名, 兼容 大小写/空格/下划线/短横线 差异."""
    return str(name).strip().lower().replace('_', ' ').replace('-', ' ')  # 去空白、转小写、下划线短横线统一为空格


def resolve_model_path(model_path):  # 解析模型或标签文件路径
    """解析模型/标签路径: 相对路径基于本文件所在目录展开."""
    _base_dir = os.path.dirname(os.path.abspath(__file__))  # 以本文件目录作为相对路径基准
    if not os.path.isabs(model_path):  # 相对路径才需要拼接
        model_path = os.path.join(_base_dir, model_path)  # 拼成绝对路径
    return model_path  # 返回可直接使用的路径


def load_labels(label_file):  # 读取类别名文件
    """读取类别名称文件 (每行一个类名), 未配置/不存在时返回空列表."""
    if not label_file or not os.path.exists(label_file):  # 未配置或文件不存在
        return []  # 返回空列表，后续用类别 id 兜底
    with open(label_file, 'r', encoding='utf-8') as f:  # 只读打开，退出自动关闭
        return [line.strip() for line in f if line.strip()]  # 去掉空行与首尾空白


# ======================
# 默认计时器 (未传入 StageTimer 时使用)
# ======================

class NullTimer:  # 空计时器，未开启阶段统计时替换真实计时器
    """空计时器: 关闭阶段统计时使用 (接口与 StageTimer 一致, 近零开销)."""

    def measure(self, stage):  # 返回一个空的计时上下文
        return _NullMeasure()  # 空测量对象，什么都不做


class _NullMeasure:  # 空测量上下文管理器，与 NullTimer 配套
    """空测量上下文管理器：与 NullTimer 配套，近零开销。"""
    __slots__ = ()  # 不生成 __dict__，进一步降低内存与开销

    def __enter__(self):  # 进入计时段
        return self  # 返回自身，with ... as 可用

    def __exit__(self, exc_type, exc_val, exc_tb):  # 退出计时段
        return False  # 返回 False 表示不吞异常


NULL_TIMER = NullTimer()  # 全局共享的空计时器实例


# ======================
# 可视化辅助
# ======================

def draw_detections(frame, detections):  # 在帧上绘制检测框、中心点与标签
    """在帧上绘制检测结果 (外接框 + 中心点 + 类别置信度)."""
    vis = frame.copy()  # 拷贝一份再画，避免污染原始帧
    for det in detections:  # 逐条检测结果绘制
        x1, y1, x2, y2 = det['bbox']  # 取外接框左上与右下角
        cx, cy = det['center']  # 取目标中心点
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)  # 画黄色外接框，线宽 2
        cv2.circle(vis, (cx, cy), 4, (0, 255, 0), -1)  # 画绿色实心中心点
        cv2.putText(vis, f"{det['label']} {det['score']:.2f}",  # 写类别名与置信度
                    (x1, max(20, y1 - 8)),  # 文本位置，靠上边界时下压防止出画
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)  # 黄色小字号文本
    return vis  # 返回绘制后的图像

def format_detection(det, mark_point, idx):  # 格式化单条检测为一行日志文本
    """格式化单个 YOLO 检测: 标号 + label + 中心点 + 相对相机标识点的偏移."""
    cx, cy = det['center']  # 目标中心坐标
    if mark_point:  # 配置了相机画面标记点
        mx, my = mark_point  # 标记点坐标
        rel = f"相对标记点=({cx - mx:+d},{cy - my:+d})"  # 计算带符号的相对偏移
    else:  # 未配置标记点
        rel = '相对标记点=(-,-)'  # 用占位符表示无偏移
    return (f"#{idx} {det['label']} 中心=({cx},{cy}) {rel} conf={det['score']:.2f}")  # 返回完整描述行


# ======================
# YOLO 检测类
# ======================

@dataclass  # 自动生成 __init__/__repr__ 等样板
class YoloDetectConfig:  # YOLO 模型初始化参数集合
    """YoloDetect 模型初始化配置。"""
    model_path: str  # 模型文件路径
    classes_num: int = 80  # 类别数，默认 COCO 80 类
    resize_type: int = 1  # 缩放方式，1 表示保持长宽比
    score_thres: float = 0.25  # 置信度阈值，低于此值的候选丢弃
    nms_thres: float = 0.45  # NMS 的 IoU 阈值，越大保留越多重叠框
    reg: int = 16  # DFL 积分桶数量
    strides: list = field(default_factory=lambda: [8, 16, 32])  # 三个检测头的下采样步长
    anchor_sizes: list = field(default_factory=lambda: [80, 40, 20])  # 各检测头对应的 anchor 尺寸


class YoloDetect:  # 基于 HB_HBMRuntime 的 YOLO 检测封装
    """基于 HB_HBMRuntime，兼容 DFL 分布与直接 LTRB 距离输出。"""

    def __init__(self, config: YoloDetectConfig, timer=None):  # 加载模型并读取输入输出元信息
        self.timer = timer or NULL_TIMER  # 未传计时器则用空实现
        try:  # 板端运行时只在 RDK 上存在
            import hbm_runtime  # 板端 BPU 运行时
        except ImportError as exc:  # 开发机没有该模块
            raise RuntimeError(  # 转换成更明确的错误提示
                'hbm 后端需要 RDK 板端环境 (hbm_runtime); '  # 说明缺少的依赖
                '开发机请在配置中将 backend 改为 "ultralytics"'  # 给出解决办法
            ) from exc  # 保留原始异常链

        self.model = hbm_runtime.HB_HBMRuntime(config.model_path)  # 加载模型文件
        self.model_name = self.model.model_names[0]  # 取模型名，多模型时取第一个
        self.input_names = self.model.input_names[self.model_name]  # 输入节点名列表
        self.output_names = self.model.output_names[self.model_name]  # 输出节点名列表
        self.input_shapes = self.model.input_shapes[self.model_name]  # 输入形状表

        self.input_h = self.input_shapes[self.input_names[0]][1]  # 模型输入高度
        self.input_w = self.input_shapes[self.input_names[0]][2]  # 模型输入宽度

        self.weights_static = np.arange(config.reg, dtype=np.float32)[np.newaxis, np.newaxis, :]  # DFL 积分权重 0..reg-1，便于广播
        self.cfg = config  # 保存配置供前后处理使用

    def set_scheduling_params(self,  # 设置 BPU 调度优先级与核心绑定
                              priority: Optional[int] = None,  # 推理任务优先级
                              bpu_cores: Optional[list] = None) -> None:  # 允许使用的 BPU 核编号
        kwargs = {}  # 按模型名组织的调度参数
        if priority is not None:  # 指定了优先级
            kwargs["priority"] = {self.model_name: priority}  # 以模型名为键
        if bpu_cores is not None:  # 指定了核心
            kwargs["bpu_cores"] = {self.model_name: bpu_cores}  # 以模型名为键
        if kwargs:  # 有参数才下发，避免空调用报错
            self.model.set_scheduling_params(**kwargs)  # 下发给底层 runtime

    def pre_process(self,  # 把输入图像转成模型需要的 Y/UV 张量
                    img: Optional[np.ndarray] = None,  # BGR 原图，BGR 模式下必填
                    image_format: Optional[str] = "BGR",  # 输入格式，BGR 或 NV12
                    nv12: Optional[np.ndarray] = None,  # NV12 原始帧，NV12 模式下必填
                    ) -> Dict[str, Dict[str, np.ndarray]]:  # 返回按模型名与输入名组织的字典
        if pre_utils is None:  # 预处理工具未加载
            raise RuntimeError('utils.py_utils 未加载, 无法执行模型预处理')  # 明确报错
        if image_format == "BGR":  # BGR 输入分支
            if img is None:  # 缺少图像数据
                raise ValueError('image_format=BGR 时必须提供 img')  # 参数校验
            resize_img = pre_utils.resized_image(  # 缩放到模型输入尺寸
                img, self.input_w, self.input_h, self.cfg.resize_type)  # 按配置的缩放方式
            y, uv = pre_utils.bgr_to_nv12_planes(resize_img)  # BGR 转 NV12 的 Y 与 UV 平面
        elif image_format in ("NV12", "nv12"):  # NV12 输入分支，大小写都兼容
            if nv12 is None:  # 缺少 NV12 数据
                raise ValueError('image_format=NV12 时必须提供 nv12 数组')  # 参数校验
            ori_h, ori_w = nv12.shape[0] * 2 // 3, nv12.shape[1]  # NV12 高度为画面的三分之二，据此还原原图宽高
            y, uv = pre_utils.preprocess_nv12(  # 直接在 NV12 上缩放，省一次颜色转换
                nv12, ori_w, ori_h,  # 原图宽高
                self.input_w, self.input_h)  # 模型输入宽高
        else:  # 其它格式
            raise ValueError(f"不支持的图像格式: {image_format}")  # 拒绝未知格式
        if y.ndim == 2:  # Y 平面缺批量与通道维
            y = y[np.newaxis, :, :, np.newaxis]  # 补上 batch 与 channel 维
        if uv.ndim == 3:  # UV 只缺批量维
            uv = uv[np.newaxis, :, :, :]  # 补上 batch 维
        return {  # 组装成 runtime 要求的双层字典
            self.model_name: {  # 以模型名为第一层键
                self.input_names[0]: y,  # Y 分量输入
                self.input_names[1]: uv  # UV 分量输入
            }
        }

    def forward(self, input_tensor):  # 执行一次 BPU 前向推理
        return self.model.run(input_tensor)  # 调用底层 runtime 并返回输出

    def post_process(self,  # 解码输出、阈值过滤、NMS 并还原到原图坐标
                     outputs,  # 模型原始输出
                     ori_img_w: int,  # 原图宽度
                     ori_img_h: int,  # 原图高度
                     score_thres: Optional[float] = None,  # 覆盖默认置信度阈值
                     nms_thres: Optional[float] = None,  # 覆盖默认 NMS 阈值
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:  # 返回框、分数、类别 id
        if post_utils is None:  # 后处理工具未加载
            raise RuntimeError('utils.py_utils 未加载, 无法执行模型后处理')  # 明确报错
        score_thres = score_thres if score_thres is not None else self.cfg.score_thres  # 未指定则用配置值
        nms_thres = nms_thres if nms_thres is not None else self.cfg.nms_thres  # 未指定则用配置值

        conf_thres_raw = -np.log(1.0 / score_thres - 1)  # sigmoid 反函数，把概率阈值换算成 logit 阈值

        model_outputs = outputs[self.model_name]  # 取出本模型的输出
        all_boxes = []  # 各检测头的框
        all_scores = []  # 各检测头的分数
        all_ids = []  # 各检测头的类别 id
        for i, (stride, anchor_size) in enumerate(  # 逐检测头解码
                zip(self.cfg.strides, self.cfg.anchor_sizes)):  # 步长与 anchor 一一对应
            cls_key = self.output_names[2 * i]  # 分类分支输出名
            box_key = self.output_names[2 * i + 1]  # 回归分支输出名

            scores, ids, valid_indices = post_utils.filter_classification(  # 按阈值筛出有效候选
                model_outputs[cls_key], conf_thres_raw)  # 传入分类输出与 logit 阈值

            dbboxes = post_utils.decode_boxes(  # 按回归通道数选择 DFL 或直接 LTRB 解码
                model_outputs[box_key], valid_indices,  # 回归输出与有效下标
                anchor_size, stride, self.weights_static)  # anchor、步长与积分权重

            all_boxes.append(dbboxes)  # 收集框
            all_scores.append(scores)  # 收集分数
            all_ids.append(ids)  # 收集类别

        boxes = np.concatenate(all_boxes, axis=0)  # 合并所有检测头的框
        scores = np.concatenate(all_scores, axis=0)  # 合并分数
        cls_ids = np.concatenate(all_ids, axis=0)  # 合并类别 id

        keep = post_utils.NMS(boxes, scores, cls_ids, nms_thres)  # 非极大值抑制，返回保留下标

        xyxy = post_utils.scale_coords_back(  # 把坐标从模型尺寸还原到原图
            boxes[keep], ori_img_w, ori_img_h,  # 保留的框与原图宽高
            self.input_w, self.input_h, self.cfg.resize_type)  # 模型输入尺寸与缩放方式

        return xyxy, scores[keep], cls_ids[keep]  # 只返回 NMS 保留下来的结果

    def predict(self,  # 预处理、推理、后处理串起来的完整流程
                img: Optional[np.ndarray] = None,  # BGR 原图
                image_format: str = "BGR",  # 输入格式
                nv12: Optional[np.ndarray] = None,  # NV12 原始帧
                score_thres: Optional[float] = None,  # 置信度阈值
                nms_thres: Optional[float] = None,  # NMS 阈值
                ):
        self.last_timing_ms = {}
        if image_format in ("NV12", "nv12"):  # NV12 输入时从数组尺寸反推原图大小
            ori_img_h, ori_img_w = nv12.shape[0] * 2 // 3, nv12.shape[1]  # NV12 高度乘三分之二还原
        else:  # BGR 输入
            ori_img_h, ori_img_w = img.shape[:2]  # 直接取图像高宽

        started = time.perf_counter()
        with self.timer.measure('YOLO预处理'):  # 计时：预处理阶段
            input_tensor = self.pre_process(img, image_format=image_format, nv12=nv12)  # 生成模型输入
        prepared = time.perf_counter()
        self.last_timing_ms['yolo_input'] = (prepared-started)*1000

        with self.timer.measure('推理'):  # 计时：BPU 推理阶段
            outputs = self.forward(input_tensor)  # 执行推理
        inferred = time.perf_counter()
        self.last_timing_ms['bpu_inference'] = (inferred-prepared)*1000

        with self.timer.measure('后处理'):  # 计时：后处理阶段
            boxes, scores, cls_ids = self.post_process(  # 解码并还原坐标
                outputs, ori_img_w, ori_img_h, score_thres, nms_thres)  # 传入原图尺寸与阈值
        self.last_timing_ms['yolo_decode_nms'] = (time.perf_counter()-inferred)*1000

        return boxes, scores, cls_ids  # 返回最终检测结果

    def __call__(self, img=None, image_format="BGR", nv12=None,  # 让实例可以像函数一样直接调用
                 score_thres=None, nms_thres=None):  # 阈值参数
        return self.predict(img, image_format=image_format, nv12=nv12,  # 直接转发给 predict
                            score_thres=score_thres, nms_thres=nms_thres)  # 透传阈值


class YoloDetector:  # 统一 hbm 与 ultralytics 两种后端的高层检测入口
    """YOLO 检测器适配层: 统一 hbm (RDK 板端 BPU) / ultralytics (开发机) 两种后端."""

    def __init__(self, cfg: dict, timer=None):  # 按配置初始化后端、类别表与过滤条件
        self.cfg = cfg  # 保存原始配置
        self.backend = cfg.get('backend', 'hbm')  # 后端类型，缺省用板端 BPU
        self.timer = timer or NULL_TIMER  # 计时器，缺省用空实现
        self.preprocess_mode = str(cfg.get('preprocess_mode', 'auto')).lower()  # auto 时优先用 NV12
        self.labels = self._resolve_labels(cfg)  # 类别名列表，可能为空
        self.target_ids = [int(i) for i in (cfg.get('target_class_ids') or [])]  # 需要保留的类别 id
        names = cfg.get('target_class_names') or []  # 需要保留的类别名
        self.targets = {normalize_name(n) for n in names} if names else set()  # 归一化后的目标类别名集合
        if self.targets:  # 配了名称过滤才做一致性校验
            if not self.labels:  # 没有类别名表，无法按名称过滤
                print('[警告] 配置了 target_class_names 但未提供类别名 '  # 提示前半句
                      '(class_names / label_file), 无法按名称过滤, 将保留全部检测结果')  # 提示后半句与后果
            else:  # 有类别名表，校验配置是否有效
                label_set = {normalize_name(l) for l in self.labels}  # 已有类别名的归一化集合
                missing = [n for n in names if normalize_name(n) not in label_set]  # 找出配置里不存在的类别
                if missing:  # 存在匹配不上的类别
                    raise ValueError(  # 配置与模型不符，直接报错避免静默漏检
                        f"target_class_names 中的类别 {missing} 不在类别列表 {self.labels} 中; "  # 指出问题类别
                        '请检查 class_names / label_file 的顺序与模型类别 id 一致')  # 给出排查方向

        if self.backend == 'hbm':  # 板端后端
            self._init_hbm()  # 初始化 BPU 模型
        elif self.backend == 'ultralytics':  # 开发机后端
            self._init_ultralytics()  # 初始化 ultralytics 模型
        else:  # 未知后端
            raise ValueError(f'未知的检测后端: {self.backend}')  # 拒绝非法配置

    @staticmethod  # 不依赖实例状态的工具方法
    def _resolve_labels(cfg):  # 确定类别名列表
        class_names = cfg.get('class_names') or []  # 优先取配置里的类别名列表
        labels = [str(n).strip() for n in class_names if str(n).strip()]  # 去空白并丢弃空项
        if labels:  # 配置里有就直接返回
            return labels  # 返回配置类别名
        label_file = cfg.get('label_file')  # 回退读取标签文件
        if label_file:  # 配置了标签文件
            return load_labels(resolve_model_path(label_file))  # 解析路径后读取
        return []  # 都没有则返回空表，后续用类别 id 显示

    def _is_target_cls(self, cls_id, label):  # 判断该类别是否属于关注目标
        if self.target_ids:  # 配了 id 过滤，优先级最高
            return int(cls_id) in self.target_ids  # 按类别 id 判定
        if self.targets:  # 其次按名称过滤
            return normalize_name(label) in self.targets  # 按归一化名称判定
        return True  # 都没配置则保留全部类别

    def _init_hbm(self):  # 初始化板端 hbm 后端
        model_path = resolve_model_path(self.cfg['model_path'])  # 解析模型绝对路径
        if not os.path.exists(model_path):  # 模型文件缺失
            raise FileNotFoundError(f'模型文件不存在: {model_path}')  # 明确报错，避免底层难懂的异常
        config = YoloDetectConfig(  # 组装底层检测配置
            model_path=model_path,  # 模型路径
            score_thres=self.cfg['score_thres'],  # 置信度阈值
            nms_thres=self.cfg['nms_thres'],  # NMS 阈值
            strides=self.cfg['strides'],  # 检测头步长
        )
        self.model = YoloDetect(config, timer=self.timer)  # 创建底层检测器
        self.model.cfg.anchor_sizes = [self.model.input_h // s  # 按模型输入尺寸自动推算 anchor，避免手填出错
                                       for s in self.model.cfg.strides]  # 每个步长对应一个 anchor
        self.model.set_scheduling_params(  # 设置 BPU 调度参数
            priority=self.cfg.get('priority', 0),  # 未配置时优先级为 0
            bpu_cores=self.cfg.get('bpu_cores', [0]))  # 未配置时只使用 0 号核

    def _init_ultralytics(self):  # 初始化开发机 ultralytics 后端
        model_path = resolve_model_path(self.cfg['model_path'])  # 解析模型绝对路径
        if not os.path.exists(model_path):  # 模型文件缺失
            raise FileNotFoundError(f'模型文件不存在: {model_path}')  # 明确报错
        try:  # ultralytics 是可选依赖
            from ultralytics import YOLO  # 导入 ultralytics 的 YOLO 类
        except ImportError as exc:  # 未安装
            raise RuntimeError('未安装 ultralytics, 请先执行: pip install ultralytics') from exc  # 给出安装命令
        self.model = YOLO(model_path)  # 加载模型
        if not self.labels and hasattr(self.model, 'names'):  # 未配类别名时从模型自带名字补全
            names = self.model.names  # 模型自带的类别名表
            if isinstance(names, dict):  # 字典形式，键为类别 id
                self.labels = [names[i] for i in sorted(names.keys())]  # 按 id 升序排成列表
            elif isinstance(names, list):  # 列表形式
                self.labels = list(names)  # 直接拷贝成列表

    def detect(self, frame, nv12=None):  # 统一检测入口，按后端分派
        self.last_timing_ms = {}
        started = time.perf_counter()
        try:
            if frame is None and nv12 is None:  # 没有任何输入数据
                return []
            if self.backend == 'ultralytics':
                return self._detect_ultralytics(frame)
            return self._detect_hbm(frame, nv12)
        finally:
            self.last_timing_ms['total_ms'] = (time.perf_counter()-started)*1000

    def _detect_hbm(self, frame, nv12=None):  # 板端推理，能直接用 NV12 就省一次转换
        use_nv12 = (nv12 is not None and self.preprocess_mode in ('auto', 'nv12'))  # 有 NV12 且模式允许
        if use_nv12:  # 直接用 NV12 输入
            boxes, scores, cls_ids = self.model.predict(  # 调底层推理
                None, image_format='NV12', nv12=nv12)  # BGR 传 None，只给 NV12
        else:  # 只能走 BGR 输入
            if frame is None and nv12 is not None:  # 只有 NV12 时先转成 BGR
                frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)  # NV12 转 BGR
            boxes, scores, cls_ids = self.model.predict(frame)  # 用 BGR 帧推理
        self.last_timing_ms.update(self.model.last_timing_ms)
        started = time.perf_counter()
        detections = self._format_detections(boxes, scores, cls_ids)
        self.last_timing_ms['format_detections'] = (time.perf_counter()-started)*1000
        return detections

    def _detect_ultralytics(self, frame):  # ultralytics 后端推理
        started = time.perf_counter()
        with self.timer.measure('推理'):  # 计时：整段推理
            preds = self.model.predict(frame, conf=self.cfg['score_thres'],  # 传入置信度阈值
                                       iou=self.cfg['nms_thres'], verbose=False)  # 传入 IoU 阈值并关闭冗长日志
        inferred = time.perf_counter()
        self.last_timing_ms['predict'] = (inferred-started)*1000
        if preds:
            self.last_timing_ms.update({'ultralytics_'+name: float(value)
                                       for name, value in (getattr(preds[0], 'speed', None) or {}).items()
                                       if value is not None})
        dets = []  # 结果列表
        if preds and preds[0].boxes is not None:  # 有预测结果且含检测框
            names = preds[0].names  # 类别名表
            for box in preds[0].boxes:  # 逐个检测框处理
                cls_id = int(box.cls.item())  # 取出类别 id
                label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else names[cls_id]  # 取类别名，缺失用 id
                if not self._is_target_cls(cls_id, label):  # 非关注目标
                    continue  # 跳过
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]  # 取出左上右下坐标
                dets.append(self._make_detection(label, float(box.conf.item()),  # 组装统一格式的结果
                                                 x1, y1, x2, y2))  # 传入框坐标
        self.last_timing_ms['format_detections'] = (time.perf_counter()-inferred)*1000
        return dets  # 返回检测结果列表

    def _format_detections(self, boxes, scores, cls_ids):  # 把底层数组转成统一的字典列表
        dets = []  # 结果列表
        for box, score, cls_id in zip(boxes, scores, cls_ids):  # 逐条配对
            cls_id = int(cls_id)  # 转成 Python int，便于查表与序列化
            label = self.labels[cls_id] if self.labels and cls_id < len(self.labels) else str(cls_id)  # 越界或无表则用 id
            if not self._is_target_cls(cls_id, label):  # 非关注目标
                continue  # 跳过
            x1, y1, x2, y2 = [float(v) for v in box]  # 取出框坐标并转浮点
            dets.append(self._make_detection(label, float(score), x1, y1, x2, y2))  # 组装并收集
        return dets  # 返回统一格式列表

    @staticmethod  # 纯函数，不依赖实例
    def _make_detection(label, score, x1, y1, x2, y2):  # 生成单条检测结果字典
        return {  # 统一结果结构
            'label': label,  # 类别名
            'score': float(score),  # 置信度
            'bbox': (int(x1), int(y1), int(x2), int(y2)),  # 整数外接框
            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2)),  # 框中心点
        }

    def warmup(self, width=640, height=480):  # 预热模型，消除首帧额外延迟
        black = np.zeros((height, width, 3), dtype=np.uint8)  # 构造全黑测试帧
        try:  # 预热失败不应阻断启动
            self.detect(black)  # 跑一次推理触发模型加载
        except Exception as exc:  # 捕获任何预热异常
            print(f'[警告] 模型预热失败: {exc!r}')  # 只提示不抛出


# =====================================================================
# ② 水下图像预处理 (preprocess / _underwater_color_correct / _apply_clahe)
# =====================================================================

def preprocess(frame, config=None):  # 水下图像预处理总入口，三个开关可独立控制
    """
    通用水下图像预处理: 颜色校正 + 高斯去噪 + CLAHE
    """
    cfg = {  # 处理参数默认值
        'enable_color_correct': True,  # 默认开启颜色校正
        'red_boost': 1.2,  # 红色通道增强系数，补偿水下红光衰减
        'enable_gaussian': True,  # 默认开启高斯去噪
        'gaussian_kernel': 5,  # 高斯核边长
        'enable_clahe': True,  # 默认开启 CLAHE
        'clahe_clip': 2.0,  # CLAHE 对比度限制，越大增强越猛
        'clahe_tile': (8, 8),  # CLAHE 分块网格大小
    }
    if config is not None:  # 有外部配置
        cfg.update(config)  # 用外部值覆盖默认值

    result = frame  # 逐级处理，全部关闭时原样返回

    if cfg['enable_color_correct']:  # 开启颜色校正
        result = _underwater_color_correct(result, cfg['red_boost'])  # 先做白平衡与红通道增强

    if cfg['enable_gaussian']:  # 开启高斯去噪
        ksize = cfg['gaussian_kernel']  # 取核大小
        if ksize % 2 == 0:  # 高斯核必须为奇数
            ksize += 1  # 偶数则加一修正
        result = cv2.GaussianBlur(result, (ksize, ksize), 0)  # 高斯模糊，sigmaX 为 0 表示按核自动推算

    if cfg['enable_clahe']:  # 开启对比度增强
        result = _apply_clahe(  # 最后做局部直方图均衡
            result,  # 待处理图像
            clip_limit=cfg['clahe_clip'],  # 对比度限制
            tile_grid_size=cfg['clahe_tile'],  # 分块网格
        )

    return result  # 返回处理后的图像


def _underwater_color_correct(frame, red_boost=1.2):  # 水下颜色校正
    """水下颜色校正: 灰度世界白平衡 + 红色通道增强 (LUT 加速)."""
    mean_b, mean_g, mean_r, _ = cv2.mean(frame)  # 求 BGR 三通道各自的均值
    gray_mean = (mean_b + mean_g + mean_r) / 3.0  # 三通道均值作为灰度世界基准

    def _gain(m):  # 计算单通道增益
        return gray_mean / m if m > 1e-6 else 1.0  # 通道过暗时不放大，同时避免除零

    gb, gg, gr = _gain(mean_b), _gain(mean_g), _gain(mean_r) * red_boost  # 红通道额外乘增强系数

    lut = np.empty((256, 1, 3), dtype=np.uint8)  # 建 256 级查找表，避免逐像素运算
    lut[:, 0, 0] = np.clip(np.arange(256) * gb, 0, 255)  # 蓝通道映射曲线，需截断防溢出
    lut[:, 0, 1] = np.clip(np.arange(256) * gg, 0, 255)  # 绿通道映射曲线
    lut[:, 0, 2] = np.clip(np.arange(256) * gr, 0, 255)  # 红通道映射曲线，含增强系数
    return cv2.LUT(frame, lut)  # 用查找表一次性完成整帧映射


def _apply_clahe(frame, clip_limit=2.0, tile_grid_size=(8, 8)):  # 在 LAB 空间做 CLAHE
    """LAB 空间 CLAHE 自适应直方图均衡化."""
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)  # 转 LAB，只对亮度均衡可保住颜色
    l_channel, a_channel, b_channel = cv2.split(lab)  # 拆出亮度与两个色度通道

    clahe = cv2.createCLAHE(  # 创建 CLAHE 对象
        clipLimit=clip_limit,  # 限制对比度放大幅度，抑制噪声被放大
        tileGridSize=tile_grid_size,  # 局部均衡的分块大小
    )
    l_channel = clahe.apply(l_channel)  # 只对亮度通道做均衡

    lab = cv2.merge([l_channel, a_channel, b_channel])  # 合并回 LAB 图像
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # 转回 BGR 供后续使用
