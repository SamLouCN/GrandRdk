"""
main_config.py — vp6.0 板端唯一配置入口。

main.py 启动时 import 本模块的 DEFAULT_CONFIG, 所有开关与参数
（相机、YOLO、预处理、空闲超时/切换、队列/线程、计时/日志等）都在这
里统一修改。改完保存后重新运行 main.py 即可生效，无需改任何业务逻辑
代码。每个字段后的 # 注释即"这里改什么/影响什么"。

相对 vp5.0 的差异：
  - 去掉 STREAM 推流段（vp6.0 不做推流）；
  - 相机由"前视/下视双路并行"改为"cam1 -> (空闲超时) -> cam2 单向切换"，
    新增 CAM1_IDLE_TIMEOUT / CAM2_IDLE_TIMEOUT 控制切换时机；
  - HSV 巡线段保留为备用（LINE_HSV，主循环暂未接入，接入时传给
    function.LineDetector(config=LINE_HSV) 使用）。
"""
# ======================
# 库导入
# ======================
import argparse
import importlib.util
import os
import sys
import cv2
import numpy as np

DEFAULT_CONFIG = {

# ======================
# 全局开关（每个功能都有独立开关，统一在此配置）
# ======================
    # -- 相机开关：关闭某路则跳过该路初始化与采集 --
    'ENABLE_CAM1': True,       # cam1 (默认 /dev/video0)：开机先启用的一路
    'ENABLE_CAM2': True,       # cam2 (默认 /dev/video2)：cam1 空闲超时后单向切换过去

    # -- 功能开关 --
    'ENABLE_CAM1_YOLO': True,  # cam1 YOLO 识别
    'ENABLE_CAM2_YOLO': True,  # cam2 YOLO 识别

    # -- 预处理开关（YOLO 前可选做水下预处理：颜色校正/高斯/CLAHE）--
    'ENABLE_PREPROCESS_CAM1': False,  # cam1：预处理 -> YOLO（默认关，实时性优先）
    'ENABLE_PREPROCESS_CAM2': False,  # cam2：预处理 -> YOLO（默认关）

# ======================
# 空闲计时 / 单向切换（vp6.0 核心逻辑）
# ======================
    # 语义：cam1 启动时空闲计时器从 0 开始累计；每一帧检测到目标
    #       （经 CAM1_YOLO.target_* 过滤后仍有结果）计时器归零；
    #       连续无目标累计 >= CAM1_IDLE_TIMEOUT 秒 -> 关闭 cam1 与计时器，
    #       启用 cam2（单向，之后不再切回 cam1）。
    'CAM1_IDLE_TIMEOUT': 30.0,   # cam1 连续无目标超过该秒数即切换；单位 秒
    'CAM2_IDLE_TIMEOUT': 10.0,   # cam2 的空闲阈值（单向切换后仅作参考/日志，
                                 # 不再触发切回；保留字段便于以后改策略）
    'IDLE_CHECK_INTERVAL': 0.2,  # 空闲计时监督线程轮询间隔（秒），越小切换越及时
    'SWITCH_LOG': True,          # True: 切换瞬间在终端/日志打印一行切换原因

# ======================
# cam1 摄像头参数 (默认 /dev/video0)
# ======================
    'CAM1_CAMERA': {
        'device': '/dev/video0',   # 设备路径；置 None 时改用 index
        'index': 0,                # 相机索引（Windows/开发机常用）
        'width': 640,
        'height': 480,
        'fps': 250,                # 请求帧率（MJPG 链路可上高帧率；实际以相机为准）
        'format': 'MJPG',          # MJPG 才能上高帧率；置 None 用相机默认
        'backend': 'auto',         # 'auto' / 'msmf'(Windows) / 'v4l2'(Linux)
        'hardware_decode': True,  # 板端 JPU 硬件解码 (MJPG)。vp6.0 目录当前未附带
                                   # libmjpg_hw.so / hw_camera.py，默认 False 走 cv2
                                   # 软解；若拷贝入 hw_camera.py 与 .so 可置 True 提速
        'mark_point': [320, 240],  # 相机标识点(画面参考点)像素坐标: 检测目标相对它的
                                   # 偏移输出到终端/日志 (正x=目标偏右, 正y=目标偏下)
    },

# ======================
# cam1 预处理功能参数
# ======================
    'CAM1_PREPROCESS': {
        'enable_color_correct': True,  # 水下颜色校正（红光补偿，LUT 加速）
        'red_boost': 1.2,
        'enable_gaussian': False,      # 高斯去噪（实时控制下关闭提速）
        'gaussian_kernel': 5,
        'enable_clahe': False,         # CLAHE（实时控制下关闭提速）
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
    },

# ======================
# cam1 YOLO 功能参数
# ======================
    'CAM1_YOLO': {
        'backend': 'hbm',            # 推理后端: 'hbm'(RDK板端BPU) / 'ultralytics'(开发机)
        'preprocess_mode': 'auto',   # YOLO 输入预处理: 'auto'(有NV12直通BPU,否则BGR)
                                     # / 'nv12' / 'bgr'
        'model_path': 'test_nashe_640x640_nv12.hbm',   # 相对 main.py 所在目录
        'score_thres': 0.6,          # 置信度阈值
        'nms_thres': 0.45,           # NMS IoU 阈值
        'strides': [8, 16, 32],      # 检测头各尺度下采样倍率（一般不改）
        'priority': 0,               # BPU 推理优先级 (0~255)
        'bpu_cores': [0],            # BPU 核心；单模型场景用默认核即可；多模型并发
                                     # 时再评估（S100 上部分核号会触发调度失败，勿乱试）
        'class_names': ['door', 'red-ball', 'yellow-ball'],  # 模型类别名列表 (与模型
                                     # 类别 id 顺序一致)。test_nashe_640x640_nv12.hbm
                                     # 沿用 vp5.0 的类别，请按实际模型类别调整！
        'label_file': None,          # 类别名文件 (与 class_names 二选一, 优先 class_names)
        'target_class_names': ['red-ball'],    # 只识别这些类别, 命中即视为"检测到目标"并复位
                                     # 空闲计时器；空列表 = 识别全部类别（任何检测框都
                                     # 算命中）。例如 ['red-ball']
        'target_class_ids': [],      # 只识别的类别 id (数字, 与名称二选一; 配置后优先)
    },

# ======================
# cam2 摄像头参数 (默认 /dev/video2)
# ======================
    # 注: 当前板端只枚举到 /dev/video0、/dev/video1（video1 为同一物理相机的
    # 伴生元数据节点）。若现场只有一台相机做时序验证，可把 device 临时改为
    # '/dev/video0'（cam1 释放后再开，需等待 2-5s 释放窗口）；若插了两台相机
    # 则保持 '/dev/video2'。
    'CAM2_CAMERA': {
        'device': '/dev/video2',
        'index': 2,
        'width': 640,
        'height': 480,
        'fps': 250,
        'format': 'MJPG',
        'backend': 'auto',
        'hardware_decode': True,    # 同 cam1：vp6.0 默认 cv2 软解
        'mark_point': [320, 240],    # 相机标识点(画面参考点)
    },

# ======================
# cam2 预处理功能参数
# ======================
    'CAM2_PREPROCESS': {
        'enable_color_correct': True,
        'red_boost': 1.2,
        'enable_gaussian': False,
        'gaussian_kernel': 5,
        'enable_clahe': False,
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
    },

# ======================
# cam2 YOLO 功能参数
# ======================
    'CAM2_YOLO': {
        'backend': 'hbm',
        'preprocess_mode': 'auto',
        'model_path': 'test_nashe_640x640_nv12.hbm',
        'score_thres': 0.6,
        'nms_thres': 0.45,
        'strides': [8, 16, 32],
        'priority': 0,
        'bpu_cores': [0],
        'class_names': ['door', 'red-ball', 'yellow-ball'],  # 按实际模型类别调整！
        'label_file': None,
        'target_class_names': ['yellow-ball'],    # 例如 ['yellow-ball']；空 = 识别全部
        'target_class_ids': [],
    },

# ======================
# HSV 巡线功能参数（备用，主循环未接入）
# ======================
    # function.LineDetector 使用示例（main.py 如需接入巡线）:
    #   from function import LineDetector
    #   detector = LineDetector(config=DEFAULT_CONFIG['LINE_HSV'])
    #   result = detector.detect(frame)   # -> {'detected','offset','angle','center'}
    'LINE_HSV': {
        # -- 预处理（LineDetector 内部自带）--
        'enable_color_correct': False,   # 巡线靠颜色分割，通常关颜色校正提速
        'red_boost': 1.2,
        'enable_gaussian': False,
        'gaussian_kernel': 5,
        'enable_clahe': False,
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
        # -- HSV 颜色分割 (OpenCV: H∈[0,179], S∈[0,255], V∈[0,255]) --
        'hsv_lower': [0, 80, 80],        # 橙红色 HSV 下界 [H, S, V]
        'hsv_upper': [25, 255, 255],     # 橙红色 HSV 上界
        'hsv_lower2': [150, 80, 80],     # 红色第二段（跨 180° 边界）下界
        'hsv_upper2': [179, 255, 255],   # 红色第二段上界
        'enable_red_wrap': True,         # 是否启用红色跨边界检测
        # -- 形态学 --
        'hsv_work_scale': 0.5,           # HSV 工作图缩放 (1.0=不缩放)
        'morph_open_kernel': 3,          # 开运算核（去除小噪点）
        'morph_close_kernel': 7,         # 闭运算核（连接断裂的引导线）
        # -- 检测 --
        'min_contour_area': 500,         # 最小轮廓面积（像素²）
        'roi_enabled': False,            # 是否只关注画面中间区域
        'roi_ratio': [0.2, 0.9, 0.1, 0.9],  # [y_start, y_end, x_start, x_end] 比例
        # -- 角度消抖 --
        'enable_angle_debounce': True,
        'angle_deadzone': 3.0,           # |angle| 小于此值视为 0
        'angle_smooth_window': 5,        # 滑动窗口大小（帧）
        'angle_smooth_method': 'median', # 'median'(抗跳变) / 'mean'
    },

# ======================
# 主循环/采集参数
# ======================
    'LOOP_SLEEP': 0.001,       # 主循环 sleep 秒数，让出 GIL（可提升采集帧率）
    # -- 处理并发（多线程多实例并行，榨 BPU 算力；参考论坛 34996）--
    # vp6.0 一次只跑一路相机，进程可用满整机 CPU；N_WORKERS 默认 6 对齐 6 核，
    # 每 worker 线程一个独立 YoloDetector/BPU 实例。若出现 hbUCPSubmitTask 类
    # 调度失败或内存压力，把该值调低（如 3）再试。
    'N_WORKERS': 6,             # 每路相机检测 worker 线程数
    'CAMERA_QUEUE_SIZE': 12,    # 每路相机采集队列容量（有界，满丢最旧）

    # -- 时间统计/帧率计时 开关 --
    'ENABLE_TIMING': True,      # True: 打印各相机运行时间/帧率 + 阶段耗时
    'TIMING_INTERVAL': 100,     # 每 N 帧打印一次滚动统计；启动/结束时各一次

    # -- 日志保存开关 --
    'ENABLE_LOG': True,         # True: 每帧检测结果保存到 ./result/log/ 下
    'LOG_DIR': 'result/log',    # 日志目录（相对 main.py 所在目录）；每路一个文件
                                # cam1.log / cam2.log

    # -- 输出开关 --
    'SHOW': True,              # 是否显示图像窗口（板端无显示器时保持 False）

}
