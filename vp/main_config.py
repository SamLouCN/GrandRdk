"""
main_config.py — vp5.0 板端唯一配置入口。

front.py / bottom.py 启动时 import 本模块的 DEFAULT_CONFIG, 所有开关与参数
（相机、YOLO、HSV 巡线、预处理、队列/线程、计时/日志/推流 STREAM 等）都在这里
统一修改。改完保存后重新运行 run.sh（或其单进程入口）即可生效，无需改任何
业务逻辑 .py 代码。每个字段后的 # 注释即"这里改什么/影响什么"。
"""
#======================
#库导入
#======================
import argparse
import importlib.util
import os
import sys
import cv2
import numpy as np

DEFAULT_CONFIG = {

#======================
#全局开关（每个功能都有独立开关，统一在此配置）
#======================
    # -- 相机开关：关闭某路相机则跳过初始化与采集，不影响另一路 --
    'ENABLE_FRONT_CAM': True,      # 前视摄像头 (video0)
    'ENABLE_BOTTOM_CAM': True,     # 下视摄像头 (video1)
    'ENABLE_FLOW_SHARE': False,        # 帧共享桥: front 是否写共享帧 (光流测速已改用下视, 默认 False 省一路 memcpy)
                                        # (True=front 也写 /dev/shm/momo_flow_front.bin, 需前视光流时开启)
    'ENABLE_FLOW_SHARE_BOTTOM': True,  # 下视是否写共享帧: 光流测速读 bottom(下视) 帧, 默认 True(写)
                                        # (bottom 仅由本开关独立控制; False=下视不写, 光流进程会无帧)

    # -- 功能开关 --
    'ENABLE_FRONT_YOLO': True,     # 前视 YOLO 识别
    'ENABLE_BOTTOM_HSV': False,     # 下视 HSV 识别（引导线巡线）
    'ENABLE_BOTTOM_YOLO': True,    # 下视 YOLO 识别

    # -- 预处理开关：两个摄像头都可在各自算法前做水下预处理 --
    'ENABLE_PREPROCESS_FRONT': False,   # 前视：预处理 -> YOLO
    'ENABLE_PREPROCESS_BOTTOM':False,   # 下视：预处理 -> HSV / YOLO
                                       # 开启时下视原帧按 BOTTOM_PREPROCESS 统一预处理一次,
                                       # HSV 与 YOLO 共用结果 (HSV 跳过内部预处理);
                                       # 关闭时 HSV 走自身内部预处理 (参数见 BOTTOM_HSV 段),
                                       # YOLO 不做预处理

#======================
#前视摄像头参数 (video0)
#======================
    'FRONT_CAMERA': {
        'device': '/dev/video0',   # 设备路径；置 None 时改用 index
        'index': 0,                # 相机索引（Windows/开发机常用）
        'width': 640,
        'height': 480,
        'fps': 250,                # 请求帧率（MJPG 链路实测 ~204fps，见 test/README.md）
        'format': 'MJPG',          # MJPG 才能上高帧率；置 None 用相机默认
        'backend': 'auto',         # 'auto' / 'msmf'(Windows) / 'v4l2'(Linux)
        'hardware_decode': True,   # 板端 JPU 硬件解码 (MJPG)，失败自动回退 cv2 软解
        'mark_point': [320, 240],  # 相机标识点(画面参考点)像素坐标: 检测目标相对它的偏移
                                   # 输出到终端/日志 (正x=目标偏右, 正y=目标偏下)
    },

#======================
#前视预处理功能参数
#======================
    'FRONT_PREPROCESS': {
        'enable_color_correct': True,  # 水下颜色校正（红光补偿，LUT 加速）
        'red_boost': 1.2,
        'enable_gaussian': False,      # 高斯去噪（实时控制下关闭提速）
        'gaussian_kernel': 5,
        'enable_clahe': False,         # CLAHE（实时控制下关闭提速）
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
    },

#======================
#前视摄像头YOLO功能参数
#======================
    'FRONT_YOLO': {
        'backend': 'hbm',            # 推理后端: 'hbm'(RDK板端BPU) / 'ultralytics'(开发机)
        'preprocess_mode': 'auto',   # YOLO 输入预处理: 'auto'(JPU硬解有NV12时直通BPU, 否则走BGR) / 'nv12' / 'bgr'
        'model_path': 'test_nashe_640x640_nv12.hbm',   # 相对 main.py 所在目录
        'score_thres': 0.6,         # 置信度阈值
        'nms_thres': 0.45,           # NMS IoU 阈值
        'strides': [8, 16, 32],      # 检测头各尺度下采样倍率（一般不改）
        'priority': 0,               # BPU 推理优先级 (0~255)
        'bpu_cores': [0,1,2,3],            # BPU 核心（前视用核0，下视用核1 避免并发冲突）
        'class_names': ['door','red-ball','yellow-ball'],           # 模型类别名列表 (与模型类别 id 顺序一致), 例如:
                                     #   ['people', 'ball']
                                     # 未配置时可用 label_file 指定类别名文件 (每行一个类名)
        'label_file': None,          # 类别名文件 (相对 main.py 所在目录; 与 class_names 二选一, 优先 class_names)
        'target_class_names': ['red-ball'],    # 只识别的类别名, 例如 ['ball']; 空列表 = 识别全部类别
        'target_class_ids': [],      # 只识别的类别 id (数字, 与名称二选一; 配置后优先于名称)
    },

#======================
#下视摄像头参数 (video2)
#======================
    'BOTTOM_CAMERA': {
        'device': '/dev/video2',
        'index': 2,
        'width': 640,
        'height': 480,
        'fps': 250,
        'format': 'MJPG',
        'backend': 'auto',
        'hardware_decode': True,   # 板端 JPU 硬件解码 (MJPG)，失败自动回退 cv2 软解
        'mark_point': [320, 240],  # 相机标识点(画面参考点)像素坐标: 检测目标相对它的偏移
                                   # 输出到终端/日志 (正x=目标偏右, 正y=目标偏下)
    },

#======================
#下视预处理功能参数
#======================
    'BOTTOM_PREPROCESS': {
        'enable_color_correct': True,  # 水下颜色校正（红光补偿，LUT 加速）
        'red_boost': 1.2,
        'enable_gaussian': False,      # 高斯去噪（实时控制下关闭提速）
        'gaussian_kernel': 5,
        'enable_clahe': False,         # CLAHE（实时控制下关闭提速）
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
    },

#======================
#下视摄像头HSV功能参数
#======================
    # 注意: vp5.0 的 bottom.py 当前未接入巡线 (主用 BOTTOM_YOLO);
    #       本段为保留配置 (LineDetector 在 function.py), 供 vp4.x 或自行接入时使用。
    'BOTTOM_HSV': {
        # -- 预处理（LineDetector 内部自带，见 pre_process/preprocessor.py）--
        'enable_color_correct': False,   # 巡线靠颜色分割，通常关颜色校正提速
        'enable_gaussian': False,        # 高斯去噪
        'enable_clahe': False,           # CLAHE（实时控制下关闭提速）
        'clahe_clip': 2.0,
        'clahe_tile': (8, 8),
        # -- HSV 颜色分割 (OpenCV: H∈[0,179], S∈[0,255], V∈[0,255]) --
        'hsv_lower': [0, 80, 80],        # 橙红色 HSV 下界 [H, S, V]
        'hsv_upper': [25, 255, 255],     # 橙红色 HSV 上界
        'hsv_lower2': [150, 80, 80],     # 红色第二段（跨 180° 边界）下界
        'hsv_upper2': [179, 255, 255],   # 红色第二段上界
        'enable_red_wrap': True,         # 是否启用红色跨边界检测
        # -- 形态学 --
        'hsv_work_scale': 0.5,            # HSV 工作图缩放 (1.0=不缩放; <1.0 在小图上做 CLAHE/形态学/轮廓/PCA, 面积阈值与核尺寸自动折算, 加速显著)
        'morph_open_kernel': 3,          # 开运算核（去除小噪点）
        'morph_close_kernel': 7,         # 闭运算核（连接断裂的引导线）
        # -- 检测 --
        'min_contour_area': 500,         # 最小轮廓面积（像素²），小于此值视为噪声
        'roi_enabled': False,            # 是否只关注画面中间区域
        'roi_ratio': [0.2, 0.9, 0.1, 0.9],  # [y_start, y_end, x_start, x_end] 比例
        # -- 角度消抖 --
        'enable_angle_debounce': True,   # 小角度死区 + 滑动窗口滤波
        'angle_deadzone': 3.0,           # |angle| 小于此值视为 0
        'angle_smooth_window': 5,        # 滑动窗口大小（帧）
        'angle_smooth_method': 'median', # 'median'(抗跳变) / 'mean'
    },

#======================
#下视摄像头YOLO功能参数
#======================
    'BOTTOM_YOLO': {
        'backend': 'hbm',
        'preprocess_mode': 'auto',   # YOLO 输入预处理: 'auto'(JPU硬解有NV12时直通BPU, 否则走BGR) / 'nv12' / 'bgr'
        'model_path': 'test_nashe_640x640_nv12.hbm',
        'score_thres': 0.6,
        'nms_thres': 0.45,
        'strides': [8, 16, 32],
        'priority': 0,
        'bpu_cores': [0,1,2,3],            # 板端实测: [1] 会触发 hbUCPSubmitTask failed (S100 不接受该调度),
                                     # 先用默认核 [0]; 多模型并发时再评估核分配
        'class_names': ['door','red-ball','yellow-ball'],           # 模型类别名列表 (与模型类别 id 顺序一致), 例如:
                                     #   ['people', 'ball']
                                     # 未配置时可用 label_file 指定类别名文件 (每行一个类名)
        'label_file': None,          # 类别名文件 (相对 main.py 所在目录; 与 class_names 二选一, 优先 class_names)
        'target_class_names': ['yellow-ball'],    # 只识别的类别名, 例如 ['ball']; 空列表 = 识别全部类别
        'target_class_ids': [],      # 只识别的类别 id (数字, 与名称二选一; 配置后优先于名称)
    },

#======================
#主循环/采集参数
#======================
    'LOOP_SLEEP': 0.001,       # 主循环 sleep 秒数，让出 GIL（实测可显著提升采集帧率）
    # -- 处理并发（提速: 多线程多实例并行, 榨 BPU 算力, 参考论坛 34996） --
 
    'N_WORKERS': 3,             # 每进程检测 worker 线程数（front/bottom 各自生效；每线程独立 BPU 实例）
    'CAMERA_QUEUE_SIZE': 12,    # 每路相机采集队列容量（有界，满丢最旧，保证延迟恒定）:
                               #   4->8: 多 worker 并发消费时加大缓冲, 减少丢帧
    # -- 时间统计/帧率计时 开关（默认 False：保持最原始检测输出，无日志无计时） --
    'ENABLE_TIMING': True,         # True: 打印各相机总运行时间/帧率 + 各阶段耗时（采集/检测/显示）
    'TIMING_INTERVAL': 100,          # 每 N 帧打印一次滚动统计；同时启动/结束时各打印一次
    'SIMPLE_TIMING': True,          # 简化时间统计（在 ENABLE_TIMING=True 基础上生效）:
                                      #   True = 只打印/统计帧率(FPS), 不做采集/检测/显示各阶段计时;
                                      #   False = 原有完整统计（帧率 + 各阶段耗时）

    # -- 日志保存开关（默认 False：不落盘，保持纯检测输出） --
    'ENABLE_LOG': False,            # True: 将每一帧的检测结果保存到 ./result/log/ 下
    'LOG_DIR': 'result/log',        # 日志目录（相对 main 所在目录）；每进程一个文件 front.log / bottom.log
    
    # -- 输出开关 --
    'SHOW': False,                  # 是否显示图像窗口

    'SHOW_INTERVAL': 1.0 / 60, # 显示节流：每 SHOW_INTERVAL 秒显示最新一帧（~60fps）

    # -- 上位机推流 (STREAM)：YOLO 画框帧 -> JPEG 编码 -> TCP 发给上位机显示 --
    #    front/bottom 各自进程内监听一个端口, 上位机作为 client 连接 (协议见 streamer.py)
    'STREAM': {
        'enable': False,            # vp5.1 默认关闭推流 (TCP/HTTP MJPEG 桥暂不需要, 减负载并避开 JPU 干扰);
                                    # 临时打开改 True; 或命令行 ./run.sh 加 --no-stream 保持关闭
        'bind_host': '0.0.0.0',     # 监听地址 (网线直连时也可写 eth IP, 如 192.168.127.10)
        'front_port': 9000,         # front 进程推流端口
        'bottom_port': 9001,        # bottom 进程推流端口
        'fps': 60,                  # 发送帧率上限 (显示用, 不必追 200fps 采集)
        'jpeg_quality':100,         # JPEG 质量 (1-100), 千兆直连可适当调高
    },

}