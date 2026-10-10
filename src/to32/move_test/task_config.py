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
AUV_IMG_W = 640.0                             # 前摄画面宽（归一化偏差 ex 的分母）
AUV_IMG_H = 480.0                             # 前摄画面高
AUV_BOTTOM_IMG_W = 1280.0                     # 下摄仍为 1280×720，独立归一化
AUV_BOTTOM_IMG_H = 720.0
AUV_CLIP_MARGIN_PX = 4.0                       # 贴边判定余量（px），贴边时 w/h 不可信
AUV_DEPTH_FILE = 'momo_depth.json'             # 融合深度（depth_kalman 写，20Hz）
AUV_DEPTH_STALE_S = 1.0                        # 深度 JSON 超期秒数
AUV_SIGMA_D_MAX = 0.05                         # 深度标准差上限（m），超限判 ok=False
AUV_ALT_FILE = 'momo_alt.json'                 # 高度计原始通道（read_altimeter.py 5Hz 落盘；AltIF 读）
AUV_ALT_STALE_S = 1.5                          # 高度计数据超期秒数（与 depth_config 一致）

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

# 触壁检测（IMU 三轴加速度，2026-10-06 新增；阈值待实车标定）
AUV_TOUCH_ACC_BASE_N = 10        # 触壁基线窗口：进入检测后前 N 拍平均幅值作基线(锁定)
AUV_TOUCH_ACC_DROP_RATIO = 0.5   # 触壁判据：当前幅值 < 基线×(1-ratio) 计一次命中
AUV_TOUCH_ACC_HIT_N = 3          # 连续命中拍数 → 判触壁（20Hz 下 ≈0.15s）

# PassDoor_v2（task/t_pass_door_v2.py，2026-10-11 四门过门：门循环 + 高度计突变判完成）
#   流程：Step0 视觉就绪（仅一次）→ 四门循环，每门 = 转向定深 → 横移对中 → 前冲+B/C突变判完成
#   过门判定四门统一：B/C 均突变（读数 < 基线−ALT_DROP_MM，连续 ALT_HIT_N 拍）后 DONE_DELAY_S；
#   每门 Step3 均有 RUSH_DUR_S（15s）兜底；整体无兜底（门4 Step3 的 15s 兜底即全任务兜底）。
#   ★ 无 Step2 超时：60s→HOLD_FAULT 已按 2026-10-11 要求移除，无门帧持续横移找门、永不超时。
#   模型由 stage_model 按 Stage.NAME 热切换（PassDoorV2 已注册 → gate）。
AUV_PASS_DOOR_V2_STAGE = 'PassDoorV2'      # 本任务阶段名（决定 YOLO 模型映射，勿与正式序列 PassGate 混淆）
AUV_PASS_DOOR_V2_GATES = [                 # 四门参数：turn=相对当前航向（右正左负），height=定深距池底(cm)
    {'turn': +60.0, 'height': 45.0},       # 门1：右转60°，定深45cm（低门）
    {'turn': -90.0, 'height': 65.0},       # 门2：左转90°，定深65cm（高门）
    {'turn': +60.0, 'height': 45.0},       # 门3：右转60°，定深45cm（低门）
    {'turn': -30.0, 'height': 65.0},       # 门4：左转30°，定深65cm（高门）
]
AUV_PASS_DOOR_V2_CAM = 'front'             # 前视相机（YOLO 检测 door → canonical gate）
AUV_PASS_DOOR_V2_WANT = 'gate'             # 检测目标 canonical 名
AUV_PASS_DOOR_V2_SWAY_DIR = 1.0            # 无门帧横移方向：+1 右移 / -1 左移（找门）
AUV_PASS_DOOR_V2_SWAY_THRUST = 0.3         # 无门帧横移推力幅度（找门速度档，-1~1）
AUV_PASS_DOOR_V2_LOST_S = 0.5              # 丢帧窗口(s)：见过门后短暂无有效目标帧(≤此值)保持上一帧 sway 输出，
                                           #   用下一有效帧修复；超过此值视为真丢门 → 固定方向横移找门（ok_cnt 清零）
AUV_PASS_DOOR_V2_PX_TOL = 20.0             # 对准容差(px)：门中心 x 距画面中心 ≤ 此值
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
# 穿门参数单独集中于 task/task_door/config.py，不在此复制。
from task.task_door.t_door import DOOR_TABLE

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

# Task1 → SearchBall → HitBall → Task2 → Door → PickBall → Return。
STAGE_TABLE = (TASK1_TABLE + HIT_BALL_TABLE + TASK2_TABLE + DOOR_TABLE
               + SEARCH_BALL_TABLE + PICK_BALL_TABLE + TASK4_TABLE + RETURN_TABLE)

# 注：测试模式（test_mode/）的测试表在 test_mode/test_config.py 的 TEST_TABLE 配置，
#     写法与本文件 STAGE_TABLE 相同（Stage 类列表表达式，可单阶段/多阶段任意拼），
#     与本文件的 STAGE_TABLE 互不影响。（原 TEST_CUSTOM_TABLE 挂点已并入该写法，删除。）

# ---------------- 卡尔曼进程托管（[2026-10-08] 进 AUV 自动拉起，老版思路重加） ----------------
AUV_KALMAN_AUTOSTART = True                    # 进 AUV 时自动拉起【深度】卡尔曼
AUV_KALMAN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                            'kalman', 'depth_kalman'))  # 跟随实际部署目录
AUV_KALMAN_START_WAIT_S = 3.0                  # 退出时等优雅退出的秒数
AUV_KALMAN_FRESH_S = 1.5                       # momo_depth.json 比这新 → 视为已在跑
AUV_VISKF_AUTOSTART = False                    # viskf 待穿门任务接入时再开
AUV_VISKF_DIR = '/userdata/GrandRDKv2.5/src/kalman/camera_kalman'
AUV_VISKF_START_WAIT_S = 3.0
AUV_VISKF_FRESH_S = 1.5
