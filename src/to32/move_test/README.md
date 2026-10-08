# move_test —— AUV 任务代码（v2.5 重写，2026-10-08 随 PID v3.8 联动）

**状态（2026-10-08）：6 段正式序列已落地（含撞球 HitBall）；AuvMode 已接回主链路（三态切换）。**

- `task_config.STAGE_TABLE` = **全序列拼接**（Task.md §3 直译）：
  `TASK1 + SEARCH_BALL + HIT_BALL + TASK2 + PASS_GATE + PICK_BALL + RETURN`。
  PickBall 仍空骨架（try-import 置空自动跳过）。**实际序列 = Task1 → SearchBall → HitBall → Task2 → PassGate → Return（6 段）**；Task4 为旧练习段，不入正式序列（测试模式可用）。
- AUV 模式位（`mode=1`）**三态**：
  - `test_mode/TEST_MODE_ENABLED=True`（当前默认）→ **测试模式**：`TestMode` 接管 AUV 位，按 `TEST_TABLE` 只测选定的几个阶段，随时切回 ROV，退出自动停推；
  - `TEST_MODE_ENABLED=False` → **正式 AuvMode**：切 AUV 即按 `STAGE_TABLE` 依次自动完成 6 段（on_exit 发停推帧）；
  - `test_config.py` 加载异常（开关状态未知）或 AuvMode 导入失败 → 回退内联 `AuvModeStub`（静止不发 0x09，journal 打印原因，宁停勿跑）。
- 主链路（ROV / IDLE）不受影响。

## 目录内容（顶层 6 文件 + `task/` + `test_mode/`）

| 文件 | 职责 |
|---|---|
| `task_config.py` | 任务全部参数集中地（不 import to32_config，自包含）；`STAGE_TABLE`=全序列拼接（见上），各任务表 try-import 置空自动跳过；含触壁检测参数 `AUV_TOUCH_ACC_*` |
| `obs.py` | 观测接口：VisionIF（momo_det_*.json）+ DepthIF（momo_depth.json）；CANON 表归一化模型 label（door→gate、red-ball→ball） |
| `mission.py` | 任务状态机：Stage 注册制 + Mission 编排器（阶段表在 task_config）；支持 `task_pids` 注入（PassGate 用） |
| `mode_auv.py` | 正式 AUV 模式壳：0x04 切模式 → tick 驱动状态机 → 组 0x09 下发；on_exit 带停推帧；自带 sys.path 注入 |
| `Task.md` | 2026 巡游任务阶段划分与定深口径（设计依据） |
| `README.md` | 本文件 |
| `task/` | 任务脚本与公共层（详见下表） |
| `test_mode/` | `test_config.py`（`TEST_MODE_ENABLED=True`、`TEST_LOOP=False`、`TEST_TABLE`=5 任务整表串联）+ `test_runner.py`（TestMode 接管 AUV 位，on_exit 发停推 0x09，支持 TEST_LOOP 重跑） |

### task/ 内容

| 文件/目录 | 内容 |
|---|---|
| `t_function.py` | 运动原语库：`dive_step / forward_step / turn_step / sway_step / hover_step` + 工具（`_cmd / _yaw_hold / _depth_out / _touch_wall_detect`）。**无超时兜底**（判据失效即持续下发）；`forward_step(touch_wall=True)` 启用 IMU 加速度触壁判据（见下） |
| `t_task1.py` | `Task1All`（TASK1_TABLE）：定深 60cm(离底) → 前进 3s |
| `t_task2.py` | `Task2All`（TASK2_TABLE）：右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底) |
| `t_search_ball.py` | `SearchBallAll`（SEARCH_BALL_TABLE）：右转 60° → 前进 2s → zigzag 扫描，下视见球提前结束 |
| `t_task4.py` | `Task4All`（TASK4_TABLE）：定深 65cm(离底) → 前进 2s → 悬停 3s（练习段，不入正式序列） |
| `t_return.py` | `ReturnAll`（RETURN_TABLE）：右转 90° → 前进 5s **touch_wall=True**（触壁提前结束） |
| `task_pass_door/` | 穿门全套（**已落地**）：`t_pass_gate.py`（PassGateAll，复用 `TaskPidController` 的 S100 GatePid）+ `gate_config/gate_filter/gate_pid/gate_mission/gate_vision/replay_gate_a` |
| `task_hit_ball/` | 撞球（**2026-10-07 v3.1 已落地，文件夹自包含**）：`t_hit_ball.py`（HitBallAll，五相位轮次重试 TRACK→RAM→CONFIRM，轮间 BACK）+ `vservo.py`（视觉伺服公共层：KF1D/KF2D/PID/yaw_servo_step/loss_tier/wrap_deg，**零 task_config 依赖**，捡球/穿门可复用）+ `README_hitball.md`；AUV_HIT_* 参数全在 task_config |
| `task_pick_ball/` | 捡球（**空骨架**）：`t_pick_ball.py`，`PICK_BALL_TABLE=[]`（实现后替换 `[PickBallAll]`） |

数据格式与 v2.2 写端**完全兼容**（front/bottom/depth_kalman 不用改）。
v2.2 旧版全套在 `D:\RC\S100\综合\GrandRDKv2.2` 里随时可查。

## 触壁检测（IMU 加速度，2026-10-06 用户指定例外）

唯一允许的"兜底"判据——用户口径是移除超时兜底，但 **Return 前进段与 HitBall BACK 段**明确要求"触壁即结束"：

- `t_function._touch_wall_detect`：读遥测三轴加速度 `acc_x/acc_y/acc_z`（link_stm32 已还原）幅值，相对**锁定基线**（进入检测后前 `AUV_TOUCH_ACC_BASE_N`(10) 拍均值）下降超 `AUV_TOUCH_ACC_DROP_RATIO`(0.5) 计一次命中，连续 `AUV_TOUCH_ACC_HIT_N`(3) 拍判触壁；
- 无加速度遥测 → 退化为纯定时；
- 参数在 `task_config.py`（`AUV_TOUCH_ACC_*`），**阈值待实车标定**。

## 检测链路（2026-10-08 双模型，config/ 层）

- 前视 `FRONT_YOLO` → `models/door_3_640x640.hbm`：只识别 `door`（CANON → `gate`，穿门用）；
- 下视 `BOTTOM_YOLO` → `models/bottom.hbm`：识别 `ring/red-ball/yellow-ball`，**只筛 `red-ball`**（CANON → `ball`，撞球/捡球用）；
- 任务侧 canonical 名 `'ball'/'gate'/'ball_y'` 不受模型影响；换模型只需同步 `quick_config` 类别表与 `obs.CANON`。

## PID 中继联动（2026-10-07 v3.8 新增）

`$TASKPID,gate,P,I,D#` 由中位机 `src/to32/task_pid_controller.py` 处理 → 更新 `TaskPidController` 内的真实 `GatePid` 实例；穿门阶段 `PassGateAll`（`task/task_pass_door/t_pass_gate.py`）经 `create_gate_mission` **共用同一对象** → 过门 PID 调参即时生效。参数更新**不启动任务、不切模式**；撞球/捡球不走此链路（HitBall 用 vservo 构造注入 AUV_HIT_*，参数独立）。验证：补丁包 `源码\GrandRdk-2.5\tests\test_task_pid.py`（14 项无硬件回归全过；脚本未拷入本项目）。

## 重写任务的开发流程

1. 在 `mission.py`（或新建模块）写阶段类，继承 `Stage`：
   - `NAME`：阶段名（日志/展示用）
   - `enter(ctx, now)`：进入时一次（初始化计时器）
   - `step(ctx, now, dt)`：每拍一次，返回 cmd dict；返回 `None` = 本阶段完成
2. 在 `task_config.STAGE_TABLE` 按顺序排阶段类，如 `[Dive, SeekGate, PassGate, ...]`
3. 改完必须：`cd /userdata/GrandRDK && ./stop.sh && ./run.sh`

cmd dict 字段（mode_auv 据此组 0x09，全部有默认值）：
`stage / note / yaw(°绝对角) / depth(cm) / surge / sway / stop`

## 硬约束（重写时不许违反）

- 阶段内**不许直接碰共享内存**，观测一律走 `obs.VisionIF.poll()` / `obs.DepthIF.read()`
- 观测返回 `None` / `ok=False` 是常态：Dive/Turn 原语已按用户口径**移除超时兜底**
  （判据失效即持续下发、永不完成）；任务阶段如需超时安全退出，须在 Stage 层自行实现
- **触壁判据是唯一例外**（用户指定）：仅 `forward_step(touch_wall=True)` 模式启用 IMU 加速度判据，见上节
- 每个 JSON 读端都要容忍「读到半个 JSON」（写端覆盖写）——obs 已处理，别绕过它
- 目标贴边（`clip=True`）时 `w/h` 不可信，判距/穿门判定必须避开
- 阶段内抛异常会被 Mission 捕获并**立即停推收尾**（ABORT）——别依赖异常做正常流程
- `YAW_MIRROR=True` 是抵消固件 Yaw 取负的，别乱关 —— ★ 但它**只在下发侧生效一次**
  （`mode_auv.tick` / `test_runner.tick` 组 0x09 时镜像）。判据侧（`t_function.yaw_err_deg` /
  `_yaw_hold`）**一律不镜像**：遥测 `actual_yaw` 原始数值系就是任务系（与 ROV 的
  `target_yaw_deg` 锚定口径一致）。**两侧都镜像 = 闭环符号反转 = err 恒不归零，
  任务永久卡在第一个转向子步骤**（2026-10-08 板端实锤：0x09 yaw 下发 −61.1°、
  遥测 +62.0°、err 恒 123°、t=442s 仍 ok=0/10）
- **深度两个帧别混比**（2026-10-08）：下发目标走**固件深度计帧**（`pool − 高度 − 机体20`，
  深度计装机体上部）；融合的 `D` 是**探头帧**（探头在舱底，比深度计低约一个机体高度
  ≈19.5cm）。判据一律用融合 `clearance`（离底净空）或同帧 `D vs (pool − 高度)`。

## 接回主链路（2026-10-07 已完成）

`src/to32/mode_dispatcher.py` **三态注册**（`_register_modes()`）：`TEST_MODE_ENABLED=True` → `TestMode`；`False` → `_load_auv_mode()` 动态加载本目录 `AuvMode`（按 `STAGE_TABLE` 自动跑全序列）；`test_config.py` 加载异常（开关状态未知）或 `AuvMode` 导入失败 → 回退内联 `AuvModeStub`（静止，journal 打印原因）。`mode_auv.py` 自带 `sys.path` 注入，`run.sh` 的 PYTHONPATH **不需要加 move_test 段**；`AuvMode.on_exit` 已带停推帧（急停闩锁时让路）。

- 跑正式任务：`test_mode/test_config.py` 置 `TEST_MODE_ENABLED=False` → 切 `mode=1` 即依次自动完成 6 段（Task1→SearchBall→HitBall→Task2→PassGate→Return）。
- 测试单段：开关置 `True` → `TEST_TABLE` 任意拼（当前默认 = Task1/Task2/SearchBall/Task4/Return，不含 HitBall/PassGate——要测它们加 `HIT_BALL_TABLE` / `PASS_GATE_TABLE` 进表达式即可）。
- `AuvModeStub` 仅剩回退用途：出现即说明配置有错，看 journal 排查。

## 相对 v2.2 移除的能力（需要时按旧版思路重加）

- `kalman_launcher.py`（进 AUV 自动托管两路卡尔曼进程）
- `auv_report.py`（$AUV 状态 5Hz UDP 回传 :8085）
- `viskf_if.py`（过门滤波接口；v2.5 用 VisionIF 直读 + vservo KF，要旧滤波时再重加）
