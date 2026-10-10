# -*- coding: utf-8 -*-
"""任务参数 —— GrandRDKv2.5 重写版任务代码的全部可调参数集中在这里

约束：
  1. 本文件自包含，不 import to32_config —— 板端 to32_config 不用改。
     AUV_* 各键的默认值刻意与 v2.2 时期 to32_config 保持一致，行为可对齐。
  2. 任务代码（mission/mode_auv/obs）只从本文件取参，getattr 兜底照旧，
     防止板端旧配置段缺失时 ImportError。
  3. 占位值（标 TODO 的）上车前必须实测标定。
  4. ★ 2026-10-11 整理：**所有任务的参数一律收敛到本文件** —— 任务脚本里
     不许再留"只有 getattr 兜底、配置里没有"的数值（改参只改这里）。
     task/task_door/config.py 只包含独立视觉参数，不包含任务控制参数。

分区索引（按 Ctrl+F 搜"===== 区"跳转；编号即文件中的阅读顺序）：
  区1  观测源（共享内存 JSON / 画面尺寸 / 检测门槛）
  区2  运动原语通用（t_function：深度换算常数 / 推力档 / 方向符号 / 到位判据）
  区3  Task1 / Task2 / Task4 / Return / SearchBall
  区4  HitBall（task/task_hit_ball/，正式序列用；AUV_HIT_* 键）
  区5  HitBall_v2（task/t_hit_ball_v2.py，根目录版；AUV_HIT_V2_* 键）
  区6  Exit 上浮（撞球 v2 用）/ IMU 触壁检测（通用）
  区7  PickRing（捡环，纯开环）
  区8  PassDoor_v2（task/t_pass_door_v2.py）
  区9  阶段表（STAGE_TABLE 与各任务注册表）
  区10 卡尔曼进程托管（★ 深度路已按用户要求移除；仅 viskf 共用同名机制）
"""
import os

# ===== 区1 观测源（共享内存 JSON，格式沿用 v2.2，写端不改）=====
AUV_SHM_DIR = '/dev/shm'                       # 共享内存目录；台架测试可注入临时目录
AUV_DET_FRONT = 'momo_det_front.json'          # 前视检测（front.py 写）
AUV_DET_BOTTOM = 'momo_det_bottom.json'        # 下视检测（bottom.py 写）
AUV_DET_STALE_S = 0.5                          # 检测 JSON 超期秒数，超期按无目标
AUV_MIN_SCORE = 0.5                            # 检测置信度门槛
AUV_IMG_W = 640.0                             # 前摄画面宽（归一化偏差 ex 的分母）
AUV_IMG_H = 480.0                             # 前摄画面高
AUV_BOTTOM_IMG_W = 1280.0                     # 下摄仍为 1280×720，独立归一化
AUV_BOTTOM_IMG_H = 720.0
# ★ 2026-10-10 口径说明（前摄为什么必须是 640×480）：
#   ① quad_cv_kit/camera_correction_params.json 前摄标定就是 640×480（内参 cx=320,cy=240）；
#   ② task_door/perception.py 在帧尺寸 ≠ image_width/height 时**直接判观测无效**（PassGate 会看不到门）；
#   ③ 全项目其它地方也一律按 (320,240) 是画面中心：auv_config.AUV_AIM_*、main_config.mark_point、
#      viskf_config.VISKF_X0_PX、web/index.html 的 `cx-320`、捡球方案 `dx=cx-320`。
#   原来前摄配 1280 时：任务侧中心算成 640 → ex=cx-640 **恒为负** → 过门横移只朝一个方向推、永远对不正。
#   ★ 下摄保持 1280×720（AUV_BOTTOM_IMG_*）：obs.VisionIF 按相机分别取尺寸，前摄降分辨率不影响下摄。
AUV_CLIP_MARGIN_PX = 4.0                       # 贴边判定余量（px），贴边时 w/h 不可信
# ★ [2026-10-11 用户要求] **深度卡尔曼已从板端移除** —— 以下 3 键对应已停用的融合深度
#   通道（obs.DepthIF 不再被注入、不再有进程写 momo_depth.json）。
#   当前**所有深度判定一律用固件深度计遥测 actual_depth_cm**（$TEL 帧内，
#   与下发 depth_cm 是同一个固件帧，可直接相减）。保留键仅为兼容旧引用。
AUV_DEPTH_FILE = 'momo_depth.json'             # （已停用）融合深度（depth_kalman 写，20Hz）
AUV_DEPTH_STALE_S = 1.0                        # （已停用）深度 JSON 超期秒数
AUV_SIGMA_D_MAX = 0.05                         # （已停用）融合深度标准差上限（m）
AUV_ALT_FILE = 'momo_alt.json'                 # 高度计原始通道（read_altimeter.py 5Hz 落盘；AltIF 读）
AUV_ALT_STALE_S = 1.5                          # 高度计数据超期秒数（与 depth_config 一致）

# ===== 区2 运动原语通用（t_function：深度换算常数 / 推力档 / 方向符号 / 到位判据）=====
# 占位值（标 TODO 的）上车前必须实测标定。
AUV_SPEED_MPS = 0.25                           # （未使用·预留）巡航速度对应 surge 档位 TODO 实车实测
AUV_YAW_RATE_DPS = 30.0                        # （未使用·预留）定向转速率 TODO 实车实测
AUV_POOL_DEPTH_CM = 106                        # ★ 实测水深(cm)，必须实测。2026-10-08 两次独立推算
                                               #   ①融合 H=1.068m（momo_depth.json）
                                               #   ②下发 50cm 时稳定离底净空只有 35.8cm ⇒ 水深
                                               #     ≈ 35.8+50+19.5 = 105.3cm
                                               #   原值 130（旧"至少 1.3m"占位）会让所有「离底 X cm」
                                               #   偏深 ~24cm（要 60 实际只到 36）→ Dive 判据
                                               #   （净空 60±8）永不满足，任务卡在第一个定深子步骤。
AUV_SURF_SAFE_CM = 25.0                        # 离面安全余量(cm)：定深目标下限（防露头）
AUV_LOG_EVERY_S = 1.0                          # 0x09 日志节流秒数
YAW_MIRROR = True                              # 抵消固件对 Yaw 的取负归一化（勿乱关；只在下发侧生效一次）

# ---- 到位判据（2026-10-11 从 t_function.py 模块常量上收）----
# [2026-10-11 整理] 以下 6 个键原先只写在 t_function.py 的模块常量里、靠 getattr 兜底，
#   现在上收到本文件统一管理（值 = 原代码常量，行为不变）。所有任务共用同一套到位判据。
AUV_BODY_HEIGHT_CM = 20.0                      # 机体高度(cm)：target_depth_cm 换算常数（Task.md §4.1）
AUV_DEFAULT_HEIGHT_CM = 60.0                   # 定深目标缺省值(cm)：原语首拍未显式传 target_height_cm 时沿用
AUV_DEPTH_TOL_CM = 8                           # Dive 到位判据带半宽(cm)：|离底净空−目标| ≤ 此值计到位
                                               #   ⚠ 融合 D 与固件深度计口径差 ≈19.5cm，混用前先统一（见 t_function 注释）
AUV_DEPTH_HOLD_N = 15                          # Dive 带内保持拍数(20Hz≈0.75s) → 判到位
AUV_YAW_TOL_DEG = 3.0                          # Turn 到位容差(°)：|实际−目标| ≤ 此值
AUV_YAW_HOLD_N = 10                            # Turn 到位保持拍数(20Hz≈0.5s) → 判到位

# ★ 推力方向标定键（2026-10-08）：正 surge = 前进 / 正 sway = 右移 是**任务系**口径；
#   换算到固件方向由输出侧（mode_auv.tick / test_runner.tick）各施加一次。
#   目前没有任何一处做额外取反 ⇒ AUV 的前进 == 上位机手柄前推的同一个值（ROV 已验证口径）；
#   实车若方向反了，改这里一个键即可（改完前后自动覆盖 直行/横移/撞球 ram·back/过门 surge）。
AUV_SURGE_THRUST = 0.5                         # 直行/前冲默认推力幅度 [-1,1]（原为代码内隐式默认 0.5）
AUV_SURGE_SIGN = 1.0                           # 1.0 = 正 surge 前进；实车反向改 -1.0
AUV_SWAY_SIGN = 1.0                            # 1.0 = 正 sway 右移；实车反向改 -1.0

# ---------------- 任务阶段表 ----------------
# 重写任务时在这里排阶段顺序，例如：
#   STAGE_TABLE = [Dive, SeekGate, PassGate, ...]  （类名见 mission.py 的注册方式）
# ===== 区3 基础任务（Task1 / Task2 / Task4 / Return / SearchBall）=====
# Task1（task/t_task1.py，2026-10-06）：整体为一个阶段，定深 60cm(离底) → 前进 3s

AUV_TASK1_DEPTH_CM = 60.0        # Task1 定深目标：距池底高度(cm)
AUV_TASK1_FWD_S = 3.0            # Task1 直行时长(s)

# Task2（task/t_task2.py，2026-10-06）：整体为一个阶段，右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底)
AUV_TASK2_DEPTH_CM = 60.0        # Task2 行进定深目标：距池底高度(cm)
AUV_TASK2_FWD_S = 3.0            # Task2 直行时长(s)
AUV_TASK2_TURN_DEG = 120.0       # Task2 转向：相对当前 yaw 值**右转**角度(°，右为正)
AUV_TASK2_TURN_NEG_DEG = -60.0   # Task2 补转向：相对当前 yaw 值**左转**角度(°，右为正，负=左)
AUV_TASK2_DIVE_CM = 55.0         # Task2 定深目标：距池底高度(cm)

# ===== 区4 HitBall（task/task_hit_ball/t_hit_ball.py，正式序列 STAGE_TABLE 用的那版）=====
# [2026-10-11 整理] 以下 AUV_HIT_* 原先全部只写在 t_hit_ball.py 的 self._f/_s 兜底里，
#   配置里一个都没有 —— 现场想调只能改代码。现全部上收到本文件（值 = 原代码兜底，行为不变）。
#   ⚠ 与区6 的 AUV_HIT_V2_*（根目录版 t_hit_ball_v2.py）是**两套独立参数**，别混改。
#      两版差异：本版是"跟随+盲冲+撞后确认+失败重试"轮次循环（yaw 递推靠近），
#      区6 版是"横移对准 + 开环全速冲撞"（不动 yaw）。正式序列 STAGE_TABLE 挂的是本版。
# ---- 视觉与轮次 ----
AUV_HIT_CAM = 'front'              # 前视相机
AUV_HIT_WANT = 'red-ball'          # 检测目标（CANON red-ball→ball）；代码兜底是 'ball'，此处按实际红球写
AUV_HIT_ROUNDS = 3                 # 最多尝试轮次：轮内 TRACK→RAM→CONFIRM，失败重试
AUV_HIT_FAIL_ACTION = 'ascend'     # 轮尽处置：'ascend'=上浮 ASCEND_HOLD_S 秒后置 ball_lost；'skip'=直接置 ball_lost
AUV_HIT_ASCEND_HOLD_S = 6.0        # 轮尽上浮时长(s)
AUV_HIT_HEIGHT_CM = 60.0           # 全程保持的距池底高度(cm)
# ---- 完成判据 / 超时 ----
AUV_HIT_TIMEOUT_S = 15.0           # 单轮 TRACK 超时(s)：超时按本轮失败处理
AUV_HIT_LOST_S = 2.0               # 真丢球阈值(s)：>此值判丢球（KF 冻结→重捕）
AUV_HIT_KF_COAST_S = 0.5           # KF 滑行窗口(s)：≤此值仅预测不喂舵
AUV_HIT_W_EMA = 0.3                # 球宽 EMA 系数：近场判据用（w_ema / 画面宽）
AUV_HIT_RAM_W_RATIO = 0.45         # 近场判据：w_ema/画面宽 ≥ 此值（且连续 RAM_SEEN_N 拍）→ 切盲冲
AUV_HIT_RAM_SEEN_N = 5             # 近场判据连续拍数
# ---- 相位时长 / 推力 ----
AUV_HIT_TRACK_SURGE = 0.35         # TRACK 跟随档推力（-1~1）
AUV_HIT_RAM_S = 1.0                # RAM 盲冲时长(s)：定时必结束（forward_step）
AUV_HIT_RAM_SURGE = 0.9            # RAM 盲冲推力档
AUV_HIT_CONFIRM_S = 0.8            # CONFIRM 撞后确认窗口(s)★ 必要件：窗内球仍命中≥CONFIRM_SEEN_N 帧 → 判未撞上
AUV_HIT_CONFIRM_SEEN_N = 2         # 窗内命中帧数阈值
AUV_HIT_BACK_S = 2.0               # BACK 后退时长(s)（硬顶；球重现≥BACK_SEEN_N 帧或触壁提前结束）
AUV_HIT_BACK_SURGE = -0.4          # BACK 后退推力档（负=倒退）；置 0 = 原地等球（Plan B）
AUV_HIT_BACK_SEEN_N = 2            # 球重现连续帧数 → 提前结束 BACK
AUV_HIT_YAW_SIGN = 1.0             # yaw 递推符号：1.0 = dx>0 → 右转（2026-10-07 已拍板）
# ---- KF1D（球心 cx 滤波，vservo.KF1D）----
AUV_HIT_KF_R_PX2 = 225.0           # 量测方差 (15px)²，TODO 实测回填
AUV_HIT_KF_Q_ACC = 800.0           # 过程噪声加速度谱密度
AUV_HIT_KF_GATE_NSIGMA = 3.0       # 新息门限 σ
AUV_HIT_KF_RESET_N = 5             # 连续拒收 N 帧 → 重置重捕
AUV_HIT_TRUST_AGE_S = 0.2          # trust 门禁：喂舵新鲜度(≈2 个写帧周期)
AUV_HIT_KF_SIGMA_MAX = 60.0        # trust 门禁：滤波 σ 上限(px)，TODO 实测标定
# ---- PID（yaw 递推环，vservo.PID）----
AUV_HIT_PID_KP = 25.0              # 比例增益
AUV_HIT_PID_KI = 0.0               # 积分增益
AUV_HIT_PID_KD = 0.0               # 微分增益
AUV_HIT_PID_OUT_MAX = 10.0         # 输出限幅(°)：单拍 yaw 增量上限
AUV_HIT_PID_I_MAX = 5.0            # 积分限幅

# ===== 区5 HitBall_v2（task/t_hit_ball_v2.py，根目录版）=====
# HitBall_v2（task/t_hit_ball_v2.py，2026-10-08 用户新流程：找球 → 对准 → 冲撞）
#   [2026-10-09] Step1 摇摆找球已**注释停用**（Task1 完成后球应已在前视视野）；
#   Step2 改为**横移对准**（不动 yaw，sway 伺服，参数见下方 AUV_SWAY_ALIGN_*）。

# Step1 摇摆找球：相对当前 yaw 沿 YAW_TRAJ 轨迹分小步来回摆，前视见球即退出摇摆。
#   未挂 STAGE_TABLE（撞球 v2 落地中）；测试用 TEST_TABLE = HIT_BALL_TABLE 引本表。
AUV_HIT_V2_CAM = 'front'                # 撞球用前视摄像头
AUV_HIT_V2_WANT = 'red-ball'            # 撞红球（CANON red-ball→ball）
AUV_HIT_V2_YAW_TRAJ = [5, 10, 15, 10, 5, -5, -10, -15, -10, -5]  # (停用)相对 yaw 摇摆轨迹 —— Step1 已注释，恢复时用
AUV_HIT_V2_TURN_TOL_DEG = 3.0      # 单小步转向到位容差(°)
AUV_HIT_V2_TURN_HOLD_N = 10        # 单小步到位保持拍数(20Hz≈0.5s)
AUV_HIT_V2_HEIGHT_CM = 60.0        # 摇摆/对准期间保持的距池底高度(cm)

# Step2 对准（横移伺服，2026-10-09 改版：**不动 yaw**，KF 滤 cx → 像素误差 × 比例增益 → sway 横移；
#   误差≤容差 连续保持 hold 帧 → 到位。原 yaw 递推参数 FOV_DEG/YAW_SIGN/ALIGN_* 停用保留）
AUV_SWAY_ALIGN_KP = 1.0          # 横移对准比例增益(归一化 ex→sway)：sway=clamp(KP·ex/(0.5·画面宽),±AUV_SWAY_THRUST)
                                 #   临时值待标定；方向反了调符号（任务系右为正）
AUV_SWAY_ALIGN_PX_TOL = 20.0     # 横移对准到位容差(px)：滤波后目标心距画面中心 ≤ 此值
AUV_SWAY_ALIGN_HOLD_N = 20       # 到位保持拍数(20Hz≈1s)：误差带内连续 N 拍 → 锁定完成
AUV_HIT_V2_LOST_S = 3.0          # Step2 对准中丢球超时(s)：连续超时无球帧 → 切 Exit(停推+上浮)
                                 #   并终止整链（不再回 Step1 重摇）；调试期可改
# （停用保留）旧 yaw 递推对准参数：
AUV_HIT_V2_FOV_DEG = 120.0       # (停用)前视水平视场角(°) —— Step2 已改横移，不再用于 yaw 换算
AUV_HIT_V2_ALIGN_PX_TOL = 20.0   # (停用)原 yaw 对准容差 —— 由 AUV_SWAY_ALIGN_PX_TOL 承接
AUV_HIT_V2_ALIGN_HOLD_N = 20     # (停用)原 yaw 对准保持 —— 由 AUV_SWAY_ALIGN_HOLD_N 承接
AUV_HIT_V2_YAW_SIGN = 1.0        # (停用)原 dx>0→右转符号键 —— yaw 递推已废弃
AUV_HIT_V2_KF_R_PX2 = 225.0      # KF 量测方差 (15px)²，TODO 实测回填
AUV_HIT_V2_KF_Q_ACC = 800.0      # KF 过程噪声加速度谱密度
AUV_HIT_V2_KF_GATE_NSIGMA = 3.0  # KF 新息门限
AUV_HIT_V2_KF_RESET_N = 5        # KF 连续拒收 N 帧 → 重置重捕
AUV_HIT_V2_KF_TRUST_AGE_S = 0.2  # trust 门禁：喂舵新鲜度(≈2 个写帧周期)
AUV_HIT_V2_KF_SIGMA_MAX = 60.0   # trust 门禁：滤波 σ 上限(px)，TODO 实测标定
# Step3 冲撞（直接开环，最高速度前进；撞到球判据：ACCx 突降）
AUV_HIT_V2_RUSH_SURGE = 1.0      # 冲撞推力档位：1.0 = 最高速度（-1~1 钳位）
AUV_HIT_V2_RUSH_DUR_S = 30.0     # 超时兜底(s)：RUSH 期间没撞到东西 → 直接完成本阶段衔接下一任务（不上浮），可修改
AUV_HIT_V2_RUSH_ACCX_DROP = 1.0  # ACCx 突降阈值(临时值,单位 m/s²)：acc_x < 基线−此值 → 判撞到球；
                               #   0.15(旧≈g 口径)在 m/s² 下属噪声级会误触发；1.0 起步，TODO 实车标定
# ===== 区6 Exit 上浮 / IMU 触壁检测（通用原语）=====
# Exit 上浮（t_function.exit_step，2026-10-09 新增）：停止运动 + 自动上浮至水面安全区
AUV_EXIT_TOL_CM = 5              # Exit 到位容差(cm)：actual_depth_cm ≤ (离水面余量+此值) 算到位
AUV_EXIT_HOLD_N = 15             # Exit 带内保持拍数(20Hz≈0.75s)

# SearchBall（task/t_search_ball.py，2026-10-06）：整体为一个阶段，转向 60° → 前进 2s → zigzag 扫描找球
AUV_SEARCH_BALL_TURN_DEG = 60.0     # 先转向：相对当前 yaw 值**右转**角度(°，右为正)
AUV_SEARCH_BALL_FWD_S = 2.0         # 转向后前进时长(s)
AUV_SEARCH_BALL_HEIGHT_CM = 60.0    # 任务全程保持的距池底高度(cm)
AUV_SEARCH_BALL_SWAY_S = 1.0        # 扫描段单次横移时长(s)
AUV_SEARCH_BALL_FWD_STEP_S = 1.0    # 扫描段单次前进时长(s)
AUV_SWAY_THRUST = 0.5               # 横移推力幅度 [-1,1]（TODO 上车标定）


# Task4（task/t_task4.py，2026-10-06）：整体为一个阶段，定深 65cm(离底) → 前进 2s → 悬停 3s
AUV_TASK4_DEPTH_CM = 65.0        # Task4 定深目标：距池底高度(cm)
AUV_TASK4_FWD_S = 2.0            # Task4 直行时长(s)
AUV_TASK4_HOVER_S = 3.0          # Task4 悬停时长(s)：定深保持 + 锁航向 + 零推力

# Return（task/t_return.py，2026-10-06）：整体为一个阶段，右转 90° → 前进 5s（触壁提前结束）
AUV_RETURN_TURN_DEG = 90.0       # Return 转向：相对当前 yaw 值**右转**角度(°，右为正)
AUV_RETURN_FWD_S = 5.0           # Return 前进时长(s)，touch_wall=True：触壁(IMU 加速度突降)提前结束
AUV_RETURN_HEIGHT_CM = 60.0      # Return 任务全程保持的距池底高度(cm)

# ===== 区7 PickRing（task/t_pick_ring.py，纯开环扫描，不用视觉）=====
# 动作序列：定深 → N×[前进 → 右转180° → 右移 → 前进 → 右转180° → 左移]
#   [2026-10-11 整理] 以下 5 键原先只写在 t_pick_ring.py 的 getattr 兜底里，现上收到本文件。
#   默认不入主序列：跑它把 STAGE_TABLE / TEST_TABLE 换成 PICK_RING_TABLE。
AUV_PICK_RING_HEIGHT_CM = 40.0   # 定深目标：距池底高度(cm)，全任务保持
AUV_PICK_RING_FWD_S = 1.0        # 每个前进子步骤时长(s)
AUV_PICK_RING_TURN_DEG = 180.0   # 每次转向角度(°，右为正，相对当前航向)
AUV_PICK_RING_SWAY_S = 0.5       # 每个横移子步骤时长(s)
AUV_PICK_RING_CYCLES = 3         # 循环次数（总子步骤 = 1 + 6×此值 = 19）

# 触壁检测（IMU 三轴加速度，2026-10-06 新增；阈值待实车标定）
#   被 t_function.forward_step(touch_wall=True)（Return）与撞球 v2 Step3 的 ACCx 突变判据复用
AUV_TOUCH_ACC_BASE_N = 10        # 触壁基线窗口：进入检测后前 N 拍平均幅值作基线(锁定)
AUV_TOUCH_ACC_DROP_RATIO = 0.5   # 触壁判据：当前幅值 < 基线×(1-ratio) 计一次命中
AUV_TOUCH_ACC_HIT_N = 3          # 连续命中拍数 → 判触壁（20Hz 下 ≈0.15s）

# ===== 区8 PassDoor_v2（task/t_pass_door_v2.py）=====
# PassDoor_v2（task/t_pass_door_v2.py，2026-10-11 当前为**单门流程**：一次性走完一颗门）
#   流程（内部 idx 0..4 = Step0..4）：
#     Step0 视觉就绪：等前视检测文件新鲜连续 N 拍（不读模型名）
#     Step1 转向定深：转 TURN_DEG（相对当前航向，右为正）→ 定深 HEIGHT_CM（距池底）
#     Step2 横移对中：假设转完角度后门已在视野；sway = KP·(框中心x − 画面中心x)/半宽，
#                     限幅 SWAY_THRUST、保底 SWAY_MIN_THRUST；丢帧按上一帧方向继续；
#                     |差值| ≤ PX_TOL 连续 HOLD_N 个有效帧 → 完成
#     Step3 前冲过门：锁航向纯前进 SURGE；高度计 B/C 均突变后 DONE_DELAY_S → 完成；
#                     兜底 RUSH_DUR_S（15s）未检出突变也自动完成
#     Step4 占位：直接完成，交接下一阶段
#   ★ 四门循环版已被重写为单门流程：AUV_PASS_DOOR_V2_GATES 与随之而来的
#     SWAY_DIR / SWAY_HYST_PX / ALIGN_EARLY / LOST_S / MODEL_* 均**当前未使用**（保留备用）。
#   ★ 无兜底口径：Step1 判据失效即不完成；Step2 丢帧不设超时；唯一兜底是 Step3 的 RUSH_DUR_S。
AUV_PASS_DOOR_V2_STAGE = 'PassDoorV2'      # 本任务阶段名（决定 YOLO 模型映射，勿与正式序列 PassGate 混淆）
AUV_PASS_DOOR_V2_TURN_DEG = 60.0           # ★ 单门流程 Step1 转向角（相对当前航向，右为正）—— 测第几门就改这里
AUV_PASS_DOOR_V2_HEIGHT_CM = 45.0          # ★ 单门流程 Step1 定深目标（距池底 cm）—— 低门 45 / 高门 65
AUV_PASS_DOOR_V2_GATES = [                 # 四门参数表（**当前单门流程未使用**，4 门循环版遗留；改上面两个键即可）
    {'turn': +60.0, 'height': 45.0},       # 门1：右转60°，定深45cm（低门）
    {'turn': -90.0, 'height': 65.0},       # 门2：左转90°，定深65cm（高门）
    {'turn': +60.0, 'height': 45.0},       # 门3：右转60°，定深45cm（低门）
    {'turn': -30.0, 'height': 65.0},       # 门4：左转30°，定深65cm（高门）
]
AUV_PASS_DOOR_V2_CAM = 'front'             # 前视相机（YOLO 检测 door → canonical gate）
AUV_PASS_DOOR_V2_WANT = 'gate'             # 检测目标 canonical 名
AUV_PASS_DOOR_V2_SWAY_DIR = 1.0            # （Step2 简化版未使用）缺省横移方向（+1 右 / -1 左）：
                                           #   之后一律由视觉误差决定方向（见下面 SWAY_KP/HYST）
AUV_PASS_DOOR_V2_SWAY_THRUST = 0.3         # 横移推力档（**对齐时的限幅上限**，也是无框找门的幅度，-1~1）
AUV_PASS_DOOR_V2_SWAY_KP = 1.0             # ★ 对中比例增益：sway = clamp(KP·ex/(0.5·画面宽), ±SWAY_THRUST)
                                           #   ex = 框中心 x − 画面中心 x（框偏右为正 → 右移）
AUV_PASS_DOOR_V2_SWAY_MIN_THRUST = 0.15    # 输出幅度保底（避免比例输出过小推不动）
AUV_PASS_DOOR_V2_SWAY_HYST_PX = 24.0       # （Step2 简化版未使用）换向滞回带(px)：
                                           #   → 保证"朝一个方向持续横移到对中"，不在中心附近来回抖
AUV_PASS_DOOR_V2_ALIGN_EARLY = True        # （单门流程未使用）看到门就允许对中：
                                           #   默认 True = sub=0 转向到位后，**只要已识别到门就立刻进入
                                           #   sub=1 横移对中**，不必等定深到位（定深目标仍每拍下发，下潜不中断）。
                                           #   动机：定深判据依赖融合深度（depth_kalman/池深标定），判据不可用时
                                           #   按"无兜底"口径永不完成 → 门就在正前方也永远轮不到对准。
                                           #   False = 恢复"先定深到位再对中"的老顺序。

# ---- step0 相关（★ 当前 t_pass_door_v2.py 的 Step0 只等「检测文件新鲜连续 3 拍」，不读模型名。
#      以下 4 键是上一版"主动切模型"方案遗留，**当前未使用**，保留备用）----
AUV_PASS_DOOR_V2_MODEL = 'door_6'          # ★ step0 要切到的前视模型名（**子串**匹配 det JSON 的 model_path）
AUV_PASS_DOOR_V2_MODEL_STAGE = 'PassDoorV2'  # step0 发布/重申的 stage 名（stage_model.STAGE_MODELS 的键）。
                                           #   默认 = 本阶段名；★ 若板端 stage_model 没把 PassDoorV2 指到
                                           #   door_6，改成本地已知映射到 door_6 的 stage 名（如 'PassGate'）
AUV_PASS_DOOR_V2_MODEL_READY_N = 3          # 就绪判据：model_path 匹配 + 检测帧新鲜，**连续 N 拍**
AUV_PASS_DOOR_V2_MODEL_WAIT_S = 0.0         # 等待上限秒：0 = 无限等（默认，守"宁停勿猜"口径）；
                                           #   >0 = 超时打 WARN 并放行进 step1
AUV_PASS_DOOR_V2_LOST_S = 0.5              # （Step2 简化版未使用）丢帧窗口(s)：见过门后短暂无有效目标帧(≤此值)保持上一帧 sway 输出，
                                           #   用下一有效帧修复；超过此值视为真丢门 → 固定方向横移找门（ok_cnt 清零）
AUV_PASS_DOOR_V2_PX_TOL = 20.0             # 对准容差(px)：门中心 x 距画面中心 ≤ 此值（口径=AUV_IMG_W 的检测空间）
AUV_PASS_DOOR_V2_HOLD_N = 10               # 对准稳定帧数（带内连续 N 个新检测帧）
AUV_PASS_DOOR_V2_SURGE = 0.5               # 前冲推力幅度（-1~1）
AUV_PASS_DOOR_V2_RUSH_DUR_S = 15.0         # 每门前冲兜底时长(s)：B/C 突变未检出 → 到时自动过门
AUV_PASS_DOOR_V2_DONE_DELAY_S = 1.0        # B、C 均突变后延迟(s) → 判过门完成
AUV_PASS_DOOR_V2_ALT_DROP_MM = 80.0        # 突变判据：读数 < 基线−此值(mm) 算一次命中（需现场标定）
AUV_PASS_DOOR_V2_ALT_BASE_N = 10           # 突变基线窗口（拍，@5Hz≈2s；基线锁存后才开始判）
AUV_PASS_DOOR_V2_ALT_HIT_N = 3             # 连续命中拍数 → 该通道判"突变发生"（@5Hz≈0.6s）



try:                                             # Task1 脚本在 task/ 子目录；move_test 在 sys.path 时引用
    from task import t_task1
    TASK1_TABLE = t_task1.TASK1_TABLE
except ImportError:                              # 找不到脚本 → 该表置空，不影响另一任务
    TASK1_TABLE = []

try:
    from task import t_task2
    TASK2_TABLE = t_task2.TASK2_TABLE
except ImportError:
    TASK2_TABLE = []

try:
    from task import t_search_ball
    SEARCH_BALL_TABLE = t_search_ball.SEARCH_BALL_TABLE
except ImportError:
    SEARCH_BALL_TABLE = []

try:
    from task import t_task4
    TASK4_TABLE = t_task4.TASK4_TABLE
except ImportError:
    TASK4_TABLE = []

try:
    from task import t_return
    RETURN_TABLE = t_return.RETURN_TABLE
except ImportError:
    RETURN_TABLE = []

try:
    from task.task_hit_ball import t_hit_ball          # 文件在 task/task_hit_ball/ 子目录（vservo 同目录自包含）
    HIT_BALL_TABLE = t_hit_ball.HIT_BALL_TABLE
except ImportError:
    HIT_BALL_TABLE = []

try:
    from task.task_pick_ball import t_pick_ball        # 文件在 task/task_pick_ball/ 子目录
    PICK_BALL_TABLE = t_pick_ball.PICK_BALL_TABLE
except ImportError:
    PICK_BALL_TABLE = []

# ===== 区9 阶段表（STAGE_TABLE 与各任务注册表）=====
# 阶段注册（2026-10-06）：每个任务各自整体为一个阶段，可单独测试
#   跑 Task1：       STAGE_TABLE = TASK1_TABLE       （定深 60cm(离底) → 前进 3s）
#   跑 Task2：       STAGE_TABLE = TASK2_TABLE       （右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底)）
#   跑搜索球：       STAGE_TABLE = SEARCH_BALL_TABLE （转向 60° → 前进 2s → zigzag 扫描找球）
#   跑 Task4：       STAGE_TABLE = TASK4_TABLE       （定深 65cm(离底) → 前进 2s → 悬停 3s）
#   跑 Return：      STAGE_TABLE = RETURN_TABLE      （右转 90° → 前进 5s，触壁提前结束）
#   单独调试任意段： 把 STAGE_TABLE 换成对应单表即可（测试模式走 TEST_TABLE，互不影响）
# 空表 = 开机即 DONE（安全停推）；任务脚本缺失时该表 try-import 置空，拼接时自动跳过。
# [2026-10-07 接回主链路] 正式比赛全序列（Task.md §3 直译）：
#   DIVE(60)+FWD(x1)=Task1   STRIKE_BALL=HitBall   TURN(θ1)+FWD(x2)=Task2
#   GATE×N=（穿门段待重写, 2026-10-08 旧 task_pass_door 已整体清理）
#   GRAB(坐底抓球)=PickBall   DROP=缺（0x09 无合爪位，未来挂点）
#   TURN(θ3)+FWD(x5,触壁)=Return；Task4 为旧测试段，不入正式序列。
# task_door 仅保留视觉后端；空表兼容已有配置引用，不注册运动阶段。
DOOR_TABLE = []

try:
    from task import t_pass_door_v2                 # 新过门 v2（对中→前冲+高度计突变判完成）
    PASS_DOOR_V2_TABLE = t_pass_door_v2.PASS_DOOR_V2_TABLE
except ImportError:
    PASS_DOOR_V2_TABLE = []

try:
    from task import t_pick_ring                    # 捡环（纯开环，不用视觉/不用 TC 参数；[2026-10-10] 补挂点）
    PICK_RING_TABLE = t_pick_ring.PICK_RING_TABLE
except ImportError:
    PICK_RING_TABLE = []

try:
    from task import t_hit_ball_v2                  # 撞球 v2（task/ 根目录版，NAME=HitBall_v2；
    HIT_BALL_V2_TABLE = t_hit_ball_v2.HIT_BALL_TABLE  # 与子目录版 t_hit_ball.HIT_BALL_TABLE 同名不同文件，故另立表名）
except ImportError:
    HIT_BALL_V2_TABLE = []

# task_door 的控制阶段已删除，其视觉链路由 front.py 独立启用。
STAGE_TABLE = (TASK1_TABLE + HIT_BALL_TABLE + TASK2_TABLE
               + SEARCH_BALL_TABLE + PICK_BALL_TABLE + TASK4_TABLE + RETURN_TABLE)

# 注：测试模式（test_mode/）的测试表在 test_mode/test_config.py 的 TEST_TABLE 配置，
#     写法与本文件 STAGE_TABLE 相同（Stage 类列表表达式，可单阶段/多阶段任意拼），
#     与本文件的 STAGE_TABLE 互不影响。（原 TEST_CUSTOM_TABLE 挂点已并入该写法，删除。）

# ===== 区10 卡尔曼进程托管（move_test/kalman_launcher.py）=====
# ★ [2026-10-11 用户要求] **深度卡尔曼已从板端移除**：
#   ① mode_auv / test_runner 不再 import DepthKalmanLauncher、不再自动拉起；
#   ② AUV_KALMAN_AUTOSTART 置 False（它本身就是深度路的总开关）；
#   ③ 板端 run.sh 默认不启动 depth_kalman（见 run.sh 的深度卡尔曼段）；
#   ④ 所有深度判定改走固件深度计遥测 actual_depth_cm。
#   以下 AUV_KALMAN_* 键保留：图像卡尔曼（viskf）仍共用同名机制，将来若要恢复深度路可直接复用。
AUV_KALMAN_AUTOSTART = False                   # 进 AUV 时自动拉起【深度】卡尔曼（已置 False = 停用）
AUV_KALMAN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                            'kalman', 'depth_kalman'))  # 跟随实际部署目录
AUV_KALMAN_PY = 'python3'                      # ★[整理新增] 拉起子进程用的解释器
AUV_KALMAN_CONFIG_DIR = ''                     # ★[整理新增] 显式指定配置目录；空 = 自动推导宿主 config/
AUV_KALMAN_START_WAIT_S = 3.0                  # 退出时等优雅退出的秒数
AUV_KALMAN_FRESH_S = 1.5                       # momo_depth.json 比这新 → 视为已在跑
AUV_VISKF_AUTOSTART = False                    # viskf 待穿门任务接入时再开
AUV_VISKF_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                             'kalman', 'camera_kalman'))  # ★[整理修正] 原硬编码
                                               #   '/userdata/GrandRDKv2.5/src/kalman/camera_kalman'
                                               #   已随部署目录改名失效 → 改为与 KALMAN_DIR 同款相对推导
AUV_VISKF_FILE = 'momo_viskf.json'             # ★[整理新增] 图像卡尔曼输出文件名
AUV_VISKF_CONFIG_DIR = ''                      # ★[整理新增] 显式指定配置目录；空 = 自动推导
AUV_VISKF_START_WAIT_S = 3.0
AUV_VISKF_FRESH_S = 1.5
