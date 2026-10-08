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
AUV_IMG_W = 1280.0                              # 画面宽（归一化偏差 ex 的分母）
AUV_IMG_H = 720.0                              # 画面高
AUV_CLIP_MARGIN_PX = 4.0                       # 贴边判定余量（px），贴边时 w/h 不可信
AUV_DEPTH_FILE = 'momo_depth.json'             # 融合深度（depth_kalman 写，20Hz）
AUV_DEPTH_STALE_S = 1.0                        # 深度 JSON 超期秒数
AUV_SIGMA_D_MAX = 0.05                         # 深度标准差上限（m），超限判 ok=False

# ---------------- 运动控制（占位值，TODO 上车标定） ----------------
AUV_SPEED_MPS = 0.25                           # TODO: 巡航速度对应 surge 档位需实测
AUV_YAW_RATE_DPS = 30.0                        # TODO: 定向转速率
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
# Task1（task/t_task1.py，2026-10-06）：整体为一个阶段，定深 60cm(离底) → 前进 3s
AUV_TASK1_DEPTH_CM = 60.0        # Task1 定深目标：距池底高度(cm)
AUV_TASK1_FWD_S = 3.0            # Task1 直行时长(s)

# Task2（task/t_task2.py，2026-10-06）：整体为一个阶段，右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底)
AUV_TASK2_DEPTH_CM = 60.0        # Task2 行进定深目标：距池底高度(cm)
AUV_TASK2_FWD_S = 3.0            # Task2 直行时长(s)
AUV_TASK2_TURN_DEG = 120.0       # Task2 转向：相对当前 yaw 值**右转**角度(°，右为正)
AUV_TASK2_TURN_NEG_DEG = -60.0   # Task2 补转向：相对当前 yaw 值**左转**角度(°，右为正，负=左)
AUV_TASK2_DIVE_CM = 55.0         # Task2 定深目标：距池底高度(cm)

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

# 触壁检测（IMU 三轴加速度，2026-10-06 新增；阈值待实车标定）
AUV_TOUCH_ACC_BASE_N = 10        # 触壁基线窗口：进入检测后前 N 拍平均幅值作基线(锁定)
AUV_TOUCH_ACC_DROP_RATIO = 0.5   # 触壁判据：当前幅值 < 基线×(1-ratio) 计一次命中
AUV_TOUCH_ACC_HIT_N = 3          # 连续命中拍数 → 判触壁（20Hz 下 ≈0.15s）

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
    from task import t_hit_ball
    HIT_BALL_TABLE = t_hit_ball.HIT_BALL_TABLE
except ImportError:
    HIT_BALL_TABLE = []

try:
    from task import t_pick_ball
    PICK_BALL_TABLE = t_pick_ball.PICK_BALL_TABLE
except ImportError:
    PICK_BALL_TABLE = []

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
# 实际序列（2026-10-08, 穿门清空后）：
#   Task1 → SearchBall → Task2 → Return。
STAGE_TABLE = (TASK1_TABLE + SEARCH_BALL_TABLE + HIT_BALL_TABLE + TASK2_TABLE
               + PICK_BALL_TABLE + RETURN_TABLE)

# 注：测试模式（test_mode/）的测试表在 test_mode/test_config.py 的 TEST_TABLE 配置，
#     写法与本文件 STAGE_TABLE 相同（Stage 类列表表达式，可单阶段/多阶段任意拼），
#     与本文件的 STAGE_TABLE 互不影响。（原 TEST_CUSTOM_TABLE 挂点已并入该写法，删除。）

# ---------------- 卡尔曼进程托管（[2026-10-08] 进 AUV 自动拉起，老版思路重加） ----------------
AUV_KALMAN_AUTOSTART = True                    # 进 AUV 时自动拉起【深度】卡尔曼
AUV_KALMAN_DIR = '/userdata/GrandRDKv2.5/src/kalman/depth_kalman'   # 工程根（main.py 平铺）
AUV_KALMAN_START_WAIT_S = 3.0                  # 退出时等优雅退出的秒数
AUV_KALMAN_FRESH_S = 1.5                       # momo_depth.json 比这新 → 视为已在跑
AUV_VISKF_AUTOSTART = False                    # viskf 待穿门任务接入时再开
AUV_VISKF_DIR = '/userdata/GrandRDKv2.5/src/kalman/camera_kalman'
AUV_VISKF_START_WAIT_S = 3.0
AUV_VISKF_FRESH_S = 1.5
