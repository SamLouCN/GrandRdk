# -*- coding: utf-8 -*-
"""quick_config.py — 日常调参入口 (相机节点 / 目标类别 / 阈值 / 性能等)        # 模块文档字符串: 说明本文件用途与不承载哪些键

不在此处的键:                                                              # 注释: 引导读者去其它配置文件的键
  - 共享内存路径 / 帧头格式 / 光流: config/main_config.py                  # 注释: 这些键在 main_config.py
  - 中位机协议 / 模式 / 静默策略: config/to32_config.py                    # 注释: 这些键在 to32_config.py

惯例: 与设备/相机相关的真值不在 quick_config 写死; 通过 camera_ports.py 按 USB 物理口解析。  # 注释: 配置按惯例说明
"""                                                                          # 注释: 文档字符串结束

# YOLO模型：前视使用新 NV12 门模型；下视保留原配置。
YOLO_MODEL_FRONT  = 'door_6_nashe_640x640_nv12.hbm'  # 前视/穿门模型，只识别 door
YOLO_MODEL_BOTTOM = 'door_3_640x640.hbm'  # 下视模型: 同 door_3, 只识别 door(2026-10-08 回退)
YOLO_MODEL        = 'test_nashe_640x640_nv12.hbm'  # 旧单模型, 两路不再引用(兼容/回退参考)

# === 相机(按 USB 物理口绑定, 见 camera_ports.py) ===                          # 注释: 分组标题
# 2026-09-23: 改为按 USB 物理口固定(见 camera_ports.py), 设备节点号不再写死。  # 注释: 改动日期与原因
#   cam1 前视 = USB 口 3-2 ; cam2 下视 = USB 口 1-2 ; cam3 = USB 口 1-1        # 注释: 通道与物理口对应
import os  # 引入 os 标准库: 下面要拿到本文件路径, 把 camera_ports.py 加进 import 路径  # 注释: 引入原因
import sys  # 引入 sys 标准库: 准备修改模块搜索路径  # 注释: 引入原因

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 把 config/ 目录加入模块搜索路径(优先于其它)  # 注释: 操作目的
from camera_ports import device_for  # 导入 camera_ports.device_for(): 按通道名解析成具体节点  # 注释: 函数作用

FRONT_DEVICE    = device_for('cam1')  # 前视节点 = 按 USB 物理口 3-2 实时解析 (cam1)  # 注释: 前视设备节点来源
BOTTOM_DEVICE   = device_for('cam2')  # 下视节点 = 按 USB 物理口 1-2 实时解析 (cam2)  # 注释: 下视设备节点来源
FRONT_FPS       = 250  # 前视相机请求帧率(实测低于此值, 受 USB 带宽与解码限制)  # 注释: 帧率上限
BOTTOM_FPS      = 250  # 下视相机请求帧率(同上)  # 注释: 帧率上限

# === 识别目标(模型类别顺序在各路 CLASS_NAMES, 这里只列出"实际想筛出谁") ===         # 注释: 分组标题
# 两路基线类别均只有 door；任务阶段按 stage_model 切换。
#   ⚠ 影响: t_search_ball 的"下视见 ball 即完成"快路径、捡球方案的 poll('bottom','ball') 将永不成立。
FRONT_TARGETS   = ['door']                 # 前视目标类别: 只 door(过门)
BOTTOM_TARGETS  = ['door']                 # 下视目标类别: 只 door(2026-10-08 回退)
FRONT_CLASS_NAMES  = ['door']              # 前视模型类别(索引0=door)
BOTTOM_CLASS_NAMES = ['door']              # 下视同 door_3, 索引0=door(2026-10-08 回退)
CLASS_NAMES     = ['door', 'red-ball', 'yellow-ball']     # 旧单模型类别(兼容/回退参考)

# === 检测阈值 ===                                                              # 注释: 分组标题
SCORE_THRES     = 0.6  # 置信度阈值: 低于此分数的检测结果直接丢弃(过滤噪声)  # 注释: 阈值作用
NMS_THRES       = 0.45  # NMS 阈值: 重叠框 IoU 大于此值时只保留分数最高的  # 注释: 阈值作用

# === 功能开关 ===                                                              # 注释: 分组标题
ENABLE_FRONT        = True  # 前视相机总开关(总硬关, 即使 YOLO 关闭仍会读相机)  # 注释: 总开关语义
ENABLE_BOTTOM       = True  # 下视相机总开关(同上)  # 注释: 总开关语义
ENABLE_FRONT_YOLO   = True  # 前视是否启用 YOLO 检测(可单独关掉跑纯相机)  # 注释: 子开关
ENABLE_BOTTOM_YOLO  = True  # 下视是否启用 YOLO 检测(同上)  # 注释: 子开关

# === 输出 / 调试 ===                                                          # 注释: 分组标题
SHOW            = False  # 是否调用 cv2.imshow(板端无桌面, 保持 False)  # 注释: 调试输出
ENABLE_TIMING   = True  # 命令行环境下打印每 N 帧的统计信息  # 注释: 时延统计
SIMPLE_TIMING   = True  # 命令行环境下打印简化版统计(只有帧率)  # 注释: 简化统计
ENABLE_LOG      = False  # 是否把帧结果写到 logs/ 目录  # 注释: 日志落盘

# === 性能 ===                                                                  # 注释: 分组标题
N_WORKERS          = 3  # 每个检测进程内的 worker 线程数(并行做前处理/推理/后处理)  # 注释: 并行度
CAMERA_QUEUE_SIZE  = 12  # 相机采集线程到 worker 的队列长度(防止内存暴涨的缓冲)  # 注释: 队列容量

FRONT_BPU_CORES    = [0, 1, 2, 3]  # 前视 YOLO 推理可用的 BPU 核列表(同板两路共用, 调度时复用)  # 注释: BPU 核分配
BOTTOM_BPU_CORES   = [0, 1, 2, 3]  # 下视 YOLO 推理可用的 BPU 核列表(同上)  # 注释: BPU 核分配
