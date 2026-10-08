#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_config.py — 过门对准算法参数（方案 v4 §四，2026-10-06）

全部参数集中在此，实机整定只改这个文件。
"""

# ---- 对准环（位置式 PD）----
# ★ 2026-10-08 用户指令: PID 全部归零, 先跑通状态机, 增益手动测
#   （中位机 $TASKPID,gate,P,I,D# 可在线改 P/I/D; sway/psi 增益改本文件）
GATE_KP_YAW    = 0.0     # e 的航向 P 增益(°/unit)
GATE_KD_YAW    = 0.0     # 阻尼项(° per °/s), 配遥测 ω
GATE_KP_SWAY   = 0.0     # e 的横移 P 增益
GATE_SWAY_DEAD = 0.03    # sway 死区
GATE_E_DEAD    = 0.04    # 对准判定死区（raw 口径 ≈13px）
GATE_T_ALIGN   = 2.0     # 死区需连续保持的时长 s
GATE_KI        = 0.0     # 积分, 默认关
GATE_I_MAX     = 1.0     # 积分限幅(∫e·dt), 仅 KI>0 时生效
GATE_PSI_MAX   = 0.0     # |Δψ| 限幅(°), 0=不限

# ---- 盲跑 ----
GATE_SURGE     = 0.35    # 前进推力
GATE_T_BLIND   = 8.0     # 盲跑时长 s, 过门判定接入后替换

# ---- 滤波 ----
GATE_EF_TYPE   = 'none'  # none=直通(仿真/回放) | viskf(实机) | complementary
GATE_EF_ALPHA  = 0.3     # complementary 一阶低通系数
GATE_VISKF_PATH = '/dev/shm/momo_viskf.json'   # viskf 输出

# ---- 视觉源 ----
GATE_DET_PATH  = '/dev/shm/momo_det_front.json'
GATE_IMG_W     = 1280
GATE_IMG_H     = 720
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

# ==================================================================
# quad_cv 落地增补（quad_cv过门落地方案_2026-10-08.md §六，2026-10-08）
# ==================================================================

# ---- S0 找门扫描（渐进增幅: 不从大角度起扫）----
GATE_SCAN_AMP    = 30.0   # 扫描半幅角上限(°, 相对进态时实际航向, 锁存为扫心)
GATE_SCAN_AMP_START = 10.0  # 起始半幅角(°): 第一段只扫 ±10°, 没找到再慢慢加大
GATE_SCAN_AMP_STEP  = 5.0   # 每次折返幅角增量(°) → ±10 → ±15 → ±20 → ... 封顶 SCAN_AMP
GATE_SCAN_DWELL  = 1.0    # 每端驻留时长 s（~10 个检测帧, 够 LOCK_FRAMES 确认）
GATE_SCAN_TOL    = 5.0    # 判"到位"的航向误差带(°), 到位后才开始驻留计时

# ---- 三相位切换 ----
GATE_SURGE_FAR   = 0.30   # S1 APPROACH 逼近推力
GATE_W_FAR       = 0.35   # S1→S2: 屏占比(bbox宽/画幅宽)阈值, 且 CV 就绪才切
GATE_W_BLIND     = 0.55   # S2→S3 回正盲冲的距离门(屏占比), 避免远距盲冲误差累积

# ---- CV 通道（quad_cv_det 找红杆）----
GATE_CV_ENABLE   = True   # 总开关; False 时行为与 v4 完全一致(纯 bbox)
GATE_CV_FRAME    = '/dev/shm/momo_frame_front_cv.bin'  # 干净帧(front.py 双写, 无黄框叠加)
GATE_CV_PERIOD_S = 0.12   # CV 节流 s（ARM 上解码+detect 约 30~60ms/次）
GATE_CV_STALE_S  = 0.30   # 干净帧超期 → 本拍按无 CV 观测
GATE_CV_LOCK_FRAMES = 3   # CV_READY: 连续多少拍 lvl>=3 才允许 S1→S2（滞回防抖）
GATE_CV_LOST_BACK_S = 6.0  # S2 中连续无 lvl4 超此时长 → 退回 v4 纯 bbox 判据（行为不劣于现状）
GATE_CV_FX_PX    = 637.6   # 像素焦距占位（640×480 的 318.8 按 1280×720 中心裁切换算）; 仅 psi_deg/日志用
GATE_CV_OPTS     = None    # quad_cv_det.DEFAULTS 覆盖字典（调参唯一入口, None=全默认）

# ---- psi 正对环 ----
GATE_PSI_KP      = 0.0    # °/unit psi。★归零, 手动测
GATE_PSI_DEAD    = 0.04   # 正对死区(无量纲; fx 标定后才可换算 psi_deg 口径)
GATE_PSI_HOLD_S  = 1.0    # psi 沿用窗: 距上次 lvl=4 超此时长视为失效
GATE_PSI_SIGN    = -1.0   # psi>0=机头偏右应左转; 右转为正 → 系数 -1; ★上车一次标定
GATE_PSI_KF_R    = 0.04   # psi KF 量测方差(psi², σ≈0.2)
GATE_PSI_KF_Q    = 0.50   # psi KF 过程噪声加速度谱密度
GATE_PSI_SIGMA_MAX = 0.25 # psi KF trust 门限: 状态不确定度超此值不喂舵
GATE_CV4_MIN     = 5      # 近 1s 内 lvl=4 拍数下限（~8.3 拍/s 上限的 60%）, 不足视为 psi 不可信
GATE_RECT_PAR_MAX = 8.0   # 矩形确认位: 左右竖边不平行度 par_h 门限(°), 只做门禁不做控制量
