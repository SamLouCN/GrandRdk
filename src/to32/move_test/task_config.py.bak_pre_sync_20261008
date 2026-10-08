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
AUV_POOL_DEPTH_CM = 130                        # TODO: 池深对应的目标深度(cm)
AUV_LOG_EVERY_S = 1.0                          # 0x09 日志节流秒数
YAW_MIRROR = True                              # 抵消固件对 Yaw 的取负归一化（勿乱关）

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
AUV_SWAY_SIGN = 1.0                 # sway 符号：1 = 正推力为右移；实车方向反了改 -1

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

# HitBall（task/task_hit_ball/t_hit_ball.py，2026-10-07 v3.1 落地；方案源 strike_ball_plan.md）
# 键源：strike v2.1 的 AUV_PID_*/AUV_BALL_*/AUV_RAM_* 改名归入（vservo 统一规划键空间）；
#      公共层 task/vservo.py 的 KF1D/PID 构造注入这些键，参数独立不串任务。
AUV_HIT_CAM = 'front'            # 撞球用前视摄像头
AUV_HIT_WANT = 'ball'            # 撞红球（CANON red-ball→ball）；蓝球待模型实装后换名即接
AUV_HIT_HEIGHT_CM = 60.0         # 撞球工作高度（距池底），track/ram/confirm 全程沿用
AUV_HIT_ROUNDS = 3               # 总尝试轮数（含首轮）：失败后退找球重试
AUV_HIT_TIMEOUT_S = 15.0         # 单轮 track 超时 → 本轮失败（绝不死等）
AUV_HIT_PID_KP = 25.0            # PID 比例（°/单位ex），TODO 上车调参
AUV_HIT_PID_KI = 0.0             # PID 积分（实装但默认关，I 限幅见下）
AUV_HIT_PID_KD = 0.0             # PID 微分
AUV_HIT_PID_OUT_MAX = 10.0       # 单拍输出限幅（°）
AUV_HIT_PID_I_MAX = 5.0          # 积分项限幅（°）
AUV_HIT_YAW_SIGN = 1.0           # ★已拍板：dx>0→右转（yaw 增，右为正）→ 默认 +1；上车确认项
AUV_HIT_TRACK_SURGE = 0.35       # 跟随段前进推力
AUV_HIT_RAM_W_RATIO = 0.45       # 近场判据：w_ema/画面宽占比 ≥ 此值
AUV_HIT_RAM_SEEN_N = 5           # 近场判据连续确认拍数（20Hz ≈0.25s）
AUV_HIT_RAM_SURGE = 0.9          # 盲冲推力
AUV_HIT_RAM_S = 1.0              # 盲冲时长（定时必结束）
AUV_HIT_CONFIRM_S = 0.8          # 撞后确认观察窗（s）：球仍在画面 → 判未撞上
AUV_HIT_CONFIRM_SEEN_N = 2       # 确认窗内球命中 ≥N 帧判"未撞上"（防单帧闪现）
AUV_HIT_BACK_S = 2.0             # 每轮后退上限（s）：球重现提前结束；超窗未重现=本轮失败
AUV_HIT_BACK_SURGE = -0.4        # 倒退推力（负=后退，⚠ 上车首测项；不可用改 0=原地等球 Plan B）
AUV_HIT_BACK_SEEN_N = 2          # 后退中球重现连续帧确认（原始帧计数，不引 KF）
AUV_HIT_FAIL_ACTION = 'ascend'   # 轮尽处置：'ascend'=先上浮再置ball_lost | 'skip'=直接置ball_lost
AUV_HIT_ASCEND_HOLD_S = 6.0      # 上浮保持时长（s，纯定时非闭环）
AUV_HIT_KF_R_PX2 = 225.0         # KF 量测方差 (15px)²，TODO 实测回填
AUV_HIT_KF_Q_ACC = 800.0         # KF 过程噪声加速度谱密度，TODO 标定
AUV_HIT_KF_GATE_NSIGMA = 3.0     # KF 新息门限（+floor≈√R）
AUV_HIT_KF_RESET_N = 5           # KF 连续拒收 N 帧 → 重置重捕
AUV_HIT_KF_COAST_S = 0.5         # KF 纯预测滑行窗（≈写端 10Hz 连丢 5 帧）
AUV_HIT_TRUST_AGE_S = 0.2        # trust 门禁：喂舵新鲜度（≈2 个写帧周期）
AUV_HIT_KF_SIGMA_MAX = 60.0      # trust 门禁：滤波 σ 上限（px），TODO 实测标定
AUV_HIT_LOST_S = 2.0             # 真丢处置门限（s）：≥此值本轮失败
AUV_HIT_W_EMA = 0.3              # 框宽 EMA 系数（近场判据平滑用）

# PickRing（task/t_pick_ring.py，2026-10-08）：纯开环捡环扫描（不碰视觉，固定编排）
#   定深40cm(离底) → N×[前进1s→转180°→右移0.5s→前进1s→转180°→左移0.5s]；
#   车身系下朝南"右移"与朝北"左移"地理同向 → 每循环净横移 2×0.5s 蛇形梯级推进
AUV_PICK_RING_HEIGHT_CM = 40.0   # 全程定深：距池底高度(cm)
AUV_PICK_RING_FWD_S = 1.0        # 单段前进时长(s)
AUV_PICK_RING_TURN_DEG = 180.0   # 旋转角：相对当前 yaw **右转**(°，右为正，负=左)
AUV_PICK_RING_SWAY_S = 0.5       # 单段横移时长(s)
AUV_PICK_RING_CYCLES = 3         # 扫描循环次数（整段任务 = 1+6×N 个子步骤）

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
    from task.task_hit_ball import t_hit_ball          # 文件在 task/task_hit_ball/ 子目录
    HIT_BALL_TABLE = t_hit_ball.HIT_BALL_TABLE
except ImportError:
    HIT_BALL_TABLE = []

try:
    from task.task_pass_door import t_pass_gate
    PASS_GATE_TABLE = t_pass_gate.PASS_GATE_TABLE
except ImportError:
    PASS_GATE_TABLE = []

try:
    from task.task_pick_ball import t_pick_ball        # 文件在 task/task_pick_ball/ 子目录
    PICK_BALL_TABLE = t_pick_ball.PICK_BALL_TABLE
except ImportError:
    PICK_BALL_TABLE = []

try:
    from task import t_pick_ring                       # 文件在 task/ 平级（2026-10-08）
    PICK_RING_TABLE = t_pick_ring.PICK_RING_TABLE
except ImportError:
    PICK_RING_TABLE = []

# 阶段注册（2026-10-06）：每个任务各自整体为一个阶段，可单独测试
#   跑 Task1：       STAGE_TABLE = TASK1_TABLE       （定深 60cm(离底) → 前进 3s）
#   跑 Task2：       STAGE_TABLE = TASK2_TABLE       （右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底)）
#   跑搜索球：       STAGE_TABLE = SEARCH_BALL_TABLE （转向 60° → 前进 2s → zigzag 扫描找球）
#   跑 Task4：       STAGE_TABLE = TASK4_TABLE       （定深 65cm(离底) → 前进 2s → 悬停 3s）
#   跑 Return：      STAGE_TABLE = RETURN_TABLE      （右转 90° → 前进 5s，触壁提前结束）
#   跑捡环（开环）： STAGE_TABLE = PICK_RING_TABLE   （定深40cm → 3×蛇形梯级扫描；默认不入主序列）
#   单独调试任意段： 把 STAGE_TABLE 换成对应单表即可（测试模式走 TEST_TABLE，互不影响）
# 空表 = 开机即 DONE（安全停推）；任务脚本缺失时该表 try-import 置空，拼接时自动跳过。
# [2026-10-07 接回主链路] 正式比赛全序列（Task.md §3 直译）：
#   DIVE(60)+FWD(x1)=Task1   STRIKE_BALL=HitBall   TURN(θ1)+FWD(x2)=Task2
#   GATE×N=PassGate   GRAB(坐底抓球)=PickBall   DROP=缺（0x09 无合爪位，未来挂点）
#   TURN(θ3)+FWD(x5,触壁)=Return；Task4 为旧测试段，不入正式序列。
# 当前 t_hit_ball 已落地（2026-10-07，公共层 task/vservo.py + 轮次重试）；t_pick_ball
# 仍空骨架（try-import 置空自动跳过）；PassGateAll 已随 task_pass_door 接入
# （其导入链依赖 to32 根目录的 task_pid_controller）。
# 实际序列（2026-10-07 板端等价路径实测导入，HitBall 入链后 6 段）：
#   Task1 → SearchBall → HitBall → Task2 → PassGate → Return。
STAGE_TABLE = (TASK1_TABLE + SEARCH_BALL_TABLE + HIT_BALL_TABLE + TASK2_TABLE
               + PASS_GATE_TABLE + PICK_BALL_TABLE + RETURN_TABLE)

# 注：测试模式（test_mode/）的测试表在 test_mode/test_config.py 的 TEST_TABLE 配置，
#     写法与本文件 STAGE_TABLE 相同（Stage 类列表表达式，可单阶段/多阶段任意拼），
#     与本文件的 STAGE_TABLE 互不影响。（原 TEST_CUSTOM_TABLE 挂点已并入该写法，删除。）
