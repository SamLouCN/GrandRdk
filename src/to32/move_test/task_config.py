# -*- coding: utf-8 -*-
"""任务参数 —— GrandRDKv2.5 重写版任务代码的全部可调参数集中在这里

约束：
  1. 本文件自包含，不 import to32_config —— 板端 to32_config 不用改。
     AUV_* 各键的默认值刻意与 v2.2 时期 to32_config 保持一致，行为可对齐。
  2. 任务代码（mission/mode_auv/obs）只从本文件取参，getattr 兜底照旧，
     防止板端旧配置段缺失时 ImportError。
  3. 占位值（标 TODO 的）上车前必须实测标定。
"""
import os

# ---------------- 观测源（共享内存 JSON，格式沿用 v2.2，写端不改） ----------------
AUV_SHM_DIR = '/dev/shm'                       # 共享内存目录；台架测试可注入临时目录
AUV_DET_FRONT = 'momo_det_front.json'          # 前视检测（front.py 写）
AUV_DET_BOTTOM = 'momo_det_bottom.json'        # 下视检测（bottom.py 写）
AUV_DET_STALE_S = 0.5                          # 检测 JSON 超期秒数，超期按无目标
AUV_MIN_SCORE = 0.5                            # 检测置信度门槛
AUV_IMG_W = 640.0                              # 画面宽（归一化偏差 ex 的分母）
AUV_IMG_H = 480.0                              # 画面高
AUV_CLIP_MARGIN_PX = 4.0                       # 贴边判定余量（px），贴边时 w/h 不可信
AUV_DEPTH_FILE = 'momo_depth.json'             # 融合深度（depth_kalman 写，20Hz）
AUV_DEPTH_STALE_S = 1.0                        # 深度 JSON 超期秒数
AUV_SIGMA_D_MAX = 0.05                         # 深度标准差上限（m），超限判 ok=False

# ---------------- 运动控制（占位值，TODO 上车标定） ----------------
AUV_SPEED_MPS = 0.25                           # TODO: 巡航速度对应 surge 档位需实测
AUV_YAW_RATE_DPS = 30.0                        # TODO: 定向转速率
AUV_POOL_DEPTH_CM = 130                        # TODO: 池深对应的目标深度(cm)
AUV_LOG_EVERY_S = 1.0                          # 0x09 日志节流秒数
YAW_MIRROR = True                              # 抵消固件对 Yaw 的取负归一化（勿乱关）

# ---------------- 任务阶段表 ----------------
# 重写任务时在这里排阶段顺序，例如：
#   STAGE_TABLE = [Dive, SeekGate, PassGate, ...]  （类名见 mission.py 的注册方式）
STAGE_TABLE = []                               # 空表 = 状态机开机即 DONE（安全停推）
