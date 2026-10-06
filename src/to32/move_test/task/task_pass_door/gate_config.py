#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_config.py — 过门对准算法参数（方案 v4 §四，2026-10-06）

全部参数集中在此，实机整定只改这个文件。
"""

# ---- 对准环（位置式 PD）----
GATE_KP_YAW    = 30.0    # e=0.1 → Δψ 含 3° P 分量
GATE_KD_YAW    = 12.0    # 阻尼项, 配遥测 ω(°/s)
GATE_KP_SWAY   = 0.8     # e=0.1 → sway 0.08
GATE_SWAY_DEAD = 0.03    # sway 死区
GATE_E_DEAD    = 0.04    # 对准判定死区（raw 口径 ≈13px）
GATE_T_ALIGN   = 2.0     # 死区需连续保持的时长 s
GATE_KI        = 0.0     # 默认不给, 代码路径保留
GATE_I_MAX     = 1.0     # 积分限幅(∫e·dt), 仅 KI>0 时生效
GATE_PSI_MAX   = 0.0     # |Δψ| 限幅(°), 0=不限(默认关)

# ---- 盲跑 ----
GATE_SURGE     = 0.35    # 前进推力
GATE_T_BLIND   = 8.0     # 盲跑时长 s, 过门判定接入后替换

# ---- 滤波 ----
GATE_EF_TYPE   = 'none'  # none=直通(仿真/回放) | viskf(实机) | complementary
GATE_EF_ALPHA  = 0.3     # complementary 一阶低通系数
GATE_VISKF_PATH = '/dev/shm/momo_viskf.json'   # viskf 输出

# ---- 视觉源 ----
GATE_DET_PATH  = '/dev/shm/momo_det_front.json'
GATE_IMG_W     = 640
GATE_IMG_H     = 480
GATE_STALE_S   = 0.5     # 检测/滤波文件 mtime 超时 → 视同丢帧
GATE_LABEL     = 'door'  # 参与的类别, 多个同标签取 score 最高
GATE_W_PASS    = 0.85    # 门框宽占比 → 到位出口(留桩)

# ---- 流程与保护 ----
GATE_LOCK_FRAMES = 2     # 连续多少拍见门才从等门转对准(去单帧误检)
GATE_LOST_WAIT   = 2.0   # 丢帧冻结等待 s, 超时回等门态
GATE_WAIT_TIMEOUT = 30.0 # 一直没门 → 保持等门并告警(不自行运动)

# ---- 符号（现场各标定一次写死）----
GATE_E_SIGN    = +1      # e 符号: 门中心在画面右侧为正 → 机内"右为正"
GATE_GYRO_SIGN = +1      # ω 符号: 陀螺仪 z 轴正方向 → 机内"右转为正"
