# move_test —— AUV 任务代码（v2.5 重写，2026-10-09 全序列含穿门 + YOLO 按阶段热切换）

**状态（2026-10-09）：8 段正式序列已落地（含撞球 HitBall、穿门 DoorTask）；AuvMode 已接回主链路（三态切换）；任务运行时经 `mission._set_stage` 发布 `momo_stage.json` 驱动视觉 YOLO 模型热切换（见 `config/stage_model.py`）。**

- `task_config.STAGE_TABLE` = **全序列拼接**（Task.md §3 直译）：
  `TASK1 + HIT_BALL + TASK2 + DOOR_TABLE + SEARCH_BALL + PICK_BALL + TASK4 + RETURN`。
  PickBall 仍空骨架（try-import 置空自动跳过）。**实际序列 = Task1 → HitBall → Task2 → PassGate（穿门）→ SearchBall → Return（8 段）**；Task4 为旧练习段，不入正式序列（测试模式可用）。撞球 v2（`task/t_hit_ball_v2.py`，Sway 找球→对准→冲撞→Exit 上浮）已落地但**暂未挂 STAGE_TABLE**。
- AUV 模式位（`mode=1`）**三态**：
  - `test_mode/TEST_MODE_ENABLED=True`（当前默认）→ **测试模式**：`TestMode` 接管 AUV 位，按 `TEST_TABLE` 只测选定的几个阶段，随时切回 ROV，退出自动停推；
  - `TEST_MODE_ENABLED=False` → **正式 AuvMode**：切 AUV 即按 `STAGE_TABLE` 依次自动完成 8 段（on_exit 发停推帧，并自动拉起深度卡尔曼）；
  - `test_config.py` 加载异常（开关状态未知）或 AuvMode 导入失败 → 回退内联 `AuvModeStub`（静止不发 0x09，journal 打印原因，宁停勿跑）。
- 主链路（ROV / IDLE）不受影响。

## 目录内容（顶层 6 文件 + `task/` + `test_mode/`）

| 文件 | 职责 |
|---|---|
| `task_config.py` | 任务全部参数集中地（不 import to32_config，自包含）；`STAGE_TABLE`=全序列拼接（见上，**2026-10-09 含 `DOOR_TABLE`**），各任务表 try-import 置空自动跳过；含触壁检测参数 `AUV_TOUCH_ACC_*`、**卡尔曼托管开关 `AUV_KALMAN_AUTOSTART=True` / `AUV_VISKF_AUTOSTART=False`**、`AUV_POOL_DEPTH_CM=106`（2026-10-08 实测） |
| `obs.py` | 观测接口：VisionIF（momo_det_*.json）+ DepthIF（momo_depth.json）；CANON 表归一化模型 label（door→gate、red-ball→ball） |
| `mission.py` | 任务状态机：Stage 注册制 + Mission 编排器（阶段表在 task_config）；支持 `task_pids` 注入（PassGate 用）；**每阶段 `_set_stage` 发布 `/dev/shm/momo_stage.json` 驱动视觉换模型**；STOP 哨兵 |
| `mode_auv.py` | 正式 AUV 模式壳：0x04 切模式 → tick 驱动状态机 → 组 0x09 下发；on_exit 带停推帧 + **停掉自己拉起的卡尔曼**；自带 sys.path 注入 |
| `kalman_launcher.py` | ★（2026-10-08 恢复）`DepthKalmanLauncher`/`ViskfLauncher` 两路托管：幂等 `ensure_started()` + 只停自己起的 `stop()` |
| `Task.md` | 2026 巡游任务阶段划分与定深口径（设计依据） |
| `README.md` | 本文件 |
| `task/` | 任务脚本与公共层（详见下表） |
| `test_mode/` | `test_config.py`（`TEST_MODE_ENABLED=True`、`TEST_LOOP=False`、**`TEST_TABLE`=当前 `TASK2_TABLE+RETURN_TABLE`**）+ `test_runner.py`（TestMode 接管 AUV 位，on_exit 发停推 0x09，支持 TEST_LOOP 重跑） |

### task/ 内容

| 文件/目录 | 内容 |
|---|---|
| `t_function.py` | 运动原语库：`dive_step / forward_step / turn_step / sway_step / hover_step` + 工具（`_cmd / _yaw_hold / _depth_out / _touch_wall_detect`）。**无超时兜底**（判据失效即持续下发）；`forward_step(touch_wall=True)` 启用 IMU 加速度触壁判据（见下） |
| `t_task1.py` | `Task1All`（TASK1_TABLE）：定深 60cm(离底) → 前进 3s |
| `t_task2.py` | `Task2All`（TASK2_TABLE）：右转 120° → 前进 3s → 左转 60° → 定深 55cm(离底) |
| `t_search_ball.py` | `SearchBallAll`（SEARCH_BALL_TABLE）：右转 60° → 前进 2s → zigzag 扫描，下视见球提前结束 |
| `t_task4.py` | `Task4All`（TASK4_TABLE）：定深 65cm(离底) → 前进 2s → 悬停 3s（练习段，不入正式序列） |
| `t_return.py` | `ReturnAll`（RETURN_TABLE）：右转 90° → 前进 5s **touch_wall=True**（触壁提前结束） |
| `task_door/` | ★ **穿门全套（2026-10-09 重构落地，`DOOR_TABLE` 已入 `STAGE_TABLE`）**：`t_door.py`（`DoorTask(Stage)` `NAME='PassGate'`，状态机 ACQUIRE→YOLO_ALIGN→APPROACH_50→CV_ALIGN→APPROACH_80→CV_FINAL→BLIND→ACQUIRE，无门走 SEARCH_TURN/SEARCH_OBSERVE→RETURN_HEADING→EXIT）+ `config.py`（穿门参数）+ `perception/observation/front_pipeline/overlay`（front.py `PassGate` 分支复用）+ `test_door.py`；旧 `task_pass_door/t_pass_gate.py`（PassGateAll）2026-10-09 已整体清理 |
| `task_hit_ball/` | 撞球 v1（**2026-10-07 v3.1 已落地，文件夹自包含**）：`t_hit_ball.py`（HitBallAll，五相位轮次重试 TRACK→RAM→CONFIRM，轮间 BACK）+ `vservo.py`（视觉伺服公共层：KF1D/KF2D/PID/yaw_servo_step/loss_tier/wrap_deg，**零 task_config 依赖**，捡球/穿门可复用）+ `README_hitball.md`；AUV_HIT_* 参数全在 task_config。**撞球 v2（`task/t_hit_ball_v2.py`，`HitBallAll` `NAME='HitBall_v2'`：Sway 找球→转向对准→冲撞→Exit 上浮）已落地但暂未挂 STAGE_TABLE** |
| `task_pick_ball/` | 捡球（**空骨架**）：`t_pick_ball.py`，`PICK_BALL_TABLE=[]`（实现后替换 `[PickBallAll]`）；方案见 `捡球任务方案_2026-10-07.md`。另 `task/t_pick_ring.py`（试抓环）未挂表 |

数据格式与 v2.2 写端**完全兼容**（front/bottom/depth_kalman 不用改）。
v2.2 旧版全套在 `D:\RC\S100\综合\GrandRDKv2.2` 里随时可查。

## 触壁检测（IMU 加速度，2026-10-06 用户指定例外）

唯一允许的"兜底"判据——用户口径是移除超时兜底，但 **Return 前进段与 HitBall BACK 段**明确要求"触壁即结束"：

- `t_function._touch_wall_detect`：读遥测三轴加速度 `acc_x/acc_y/acc_z`（link_stm32 已还原）幅值，相对**锁定基线**（进入检测后前 `AUV_TOUCH_ACC_BASE_N`(10) 拍均值）下降超 `AUV_TOUCH_ACC_DROP_RATIO`(0.5) 计一次命中，连续 `AUV_TOUCH_ACC_HIT_N`(3) 拍判触壁；
- 无加速度遥测 → 退化为纯定时；
- 参数在 `task_config.py`（`AUV_TOUCH_ACC_*`），**阈值待实车标定**。

## 检测链路（2026-10-08 回退单模型 + 2026-10-09 stage_model 按阶段热切换）

- **基线（IDLE/DONE）**：`quick_config.py` 的 `YOLO_MODEL_FRONT`/`YOLO_MODEL_BOTTOM` 均 `models/door_3_640x640.hbm`，只识别 `door`（CANON → `gate`，穿门用）；
- **AUV 任务运行时**：`config/stage_model.py` 的 `StageDetector` 按 `/dev/shm/momo_stage.json` 热切换——
  - `ball`（`test_nashe_640x640_nv12.hbm`，door/red-ball/yellow-ball，筛 red-ball → `ball`）：Task1 / SearchBall / HitBall / HitBall_v2；
  - `gate`（`door_4_nashe_1280x1280_nv12.hbm`，door → `gate`）：Task2 / PassGate；
  - `pick`（`bottom.hbm`，ring/red-ball/yellow-ball，筛 red-ball → `ball`）：PickBall；
- 任务侧 canonical 名 `'ball'/'gate'/'ball_y'` 不受模型影响；换模型只需同步 `stage_model.MODELS` 类别表与 `obs.CANON`（不必再动 quick_config 的 target/class_names，基线仍用它们）。

### 下视相机按赛段门控（2026-10-09 新增）

- `config/stage_model.py` 新增 `BOTTOM_ACTIVE_STAGES = {'SearchBall', 'PickBall'}`：仅这些阶段 `src/bottom.py` 才打开下视相机采集，其余阶段 V4L2 release；
- 实现：`bottom.py` 的 `producer` 采集线程循环顶部读 `momo_stage.json`（`stage_model.read_stage()`）→ stage 入集且相机未开则 `open_camera()` 重开（失败 1s 重试 + 告警）；出集则 `cap.release()` 置 None；关闭态 0.2s 低频轮询（省 CPU）。**进程常驻不退出**（`run.sh` 的 `wait $PID_BOTTOM` 依赖进程存活，整体退出会触发 run.sh 收尾全链路）；
- `main()` 初始按当前 stage 决定是否立刻开相机（非启用阶段延迟到第一个启用阶段再开）；`--frames N` 有限帧测试模式跳过门控，保持"启动即开相机"原行为；
- 相机所有权移交 `producer` 线程（关闭/重开都在采集循环内），`main` 的 finally 不再重复 release；重开后按新相机类型重算 NV12 通道；
- 副作用：关闭期间 `momo_frame_bottom.bin` / `momo_det_bottom.json` 停更 → `/cam2` 画面冻结、`obs.poll('bottom',...)` 因 mtime 超期自动返回 None（不会误判有球）；开相机约 0.3~1s 延迟，SearchBall 前 2s 转向/前进不用下视，天然覆盖。

## PID 中继联动（2026-10-07 v3.8 新增）

`$TASKPID,gate,P,I,D#` 由中位机 `src/to32/task_pid_controller.py` 处理 → 更新 `TaskPidController` 内的真实 `GatePid` 实例；穿门阶段 `DoorTask`（`task/task_door/t_door.py`）经 `create_gate_mission` **共用同一对象** → 过门 PID 调参即时生效。参数更新**不启动任务、不切模式**；撞球/捡球不走此链路（HitBall 用 vservo 构造注入 AUV_HIT_*，参数独立）。验证：补丁包 `源码\GrandRdk-2.5\tests\test_task_pid.py`（14 项无硬件回归全过；脚本未拷入本项目）。

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

- 跑正式任务：`test_mode/test_config.py` 置 `TEST_MODE_ENABLED=False` → 切 `mode=1` 即依次自动完成 8 段（Task1→HitBall→Task2→PassGate（穿门）→SearchBall→Return；进 AUV 自动拉起深度卡尔曼）。
- 测试单段：开关置 `True` → `TEST_TABLE` 任意拼（当前默认 = **Task2+Return**——要测其它段加 `TASK1_TABLE` / `HIT_BALL_TABLE` / `DOOR_TABLE` / `SEARCH_BALL_TABLE` 进表达式即可）。
- `AuvModeStub` 仅剩回退用途：出现即说明配置有错，看 journal 排查。

## 相对 v2.2 移除的能力（需要时按旧版思路重加）

- ~~`kalman_launcher.py`~~ → **2026-10-08 已恢复**：`DepthKalmanLauncher`/`ViskfLauncher` 两路托管（`AUV_KALMAN_AUTOSTART=True` 深度自动拉起、`AUV_VISKF_AUTOSTART=False` 默认关），见目录表
- `auv_report.py`（$AUV 状态 5Hz UDP 回传 :8085，仍已删）
- `viskf_if.py`（过门滤波接口；v2.5 用 VisionIF 直读 + vservo KF / `task_door` 前端管线，要旧滤波时再重加）
