# move_test —— AUV 任务代码（v2.5 重写，2026-10-07 随 PID v3.8 联动）

**状态（2026-10-07）：任务阶段已落地；正式 AuvMode 已接回主链路（测试模式开关三态切换）。**

- `task_config.STAGE_TABLE` 当前 = **全序列拼接**（Task.md §3 直译：Task1+SearchBall+HitBall+Task2+PassGate+PickBall+Return；未落地任务的表 try-import 置空自动跳过。2026-10-07 实测导入：实际序列 = **Task1→SearchBall→Task2→PassGate→Return**，HitBall/PickBall 表为空自动跳过）。
- AUV 模式位（`mode=1`）**三态**：
  - `test_mode/TEST_MODE_ENABLED=True`（当前默认）→ **测试模式**：`TestMode` 接管 AUV 位，按 `TEST_TABLE` 只测选定的一个/几个阶段，行为不变；
  - `TEST_MODE_ENABLED=False` → **正式 AuvMode**：切 AUV 即按 `STAGE_TABLE` 依次自动完成所有阶段（on_exit 发停推帧）；
  - `test_config.py` 加载异常（开关状态未知）或 AuvMode 导入失败 → 回退内联 `AuvModeStub`（静止不发 0x09，journal 打印原因，宁停勿跑）。
- 主链路（ROV / IDLE）不受影响。

## 目录内容（顶层 6 文件 + `task/` + `test_mode/`，v2.2 的 7 个旧文件已全部删除）

| 文件 | 职责 |
|---|---|
| `task_config.py` | 任务全部参数集中地（不 import to32_config，自包含）；`STAGE_TABLE`=全序列拼接（Task.md §3 直译），各任务表 try-import 置空自动跳过 |
| `obs.py` | 观测接口：VisionIF（momo_det_*.json）+ DepthIF（momo_depth.json） |
| `mission.py` | 任务状态机：Stage 注册制 + Mission 编排器（阶段表在 task_config）；**2026-10-07 支持 `task_pids` 注入** |
| `mode_auv.py` | AUV 模式壳：0x04 切模式 → tick 驱动状态机 → 组 0x09 下发；自带 sys.path 注入 |
| `Task.md` | 2026 巡游任务阶段划分与定深口径（设计依据） |
| `README.md` | 本文件 |
| `task/` | 任务阶段脚本：`t_task1/t_task2/t_search_ball/t_task4/t_return`（5 任务，对应 `TEST_TABLE`）+ `t_function.py`（未入表）+ `task_pass_door/`（穿门，`PassGateAll` 已就位：gate_config/gate_filter/gate_pid/gate_mission/gate_vision/t_pass_gate/replay_gate_a）+ `task_hit_ball/`、`task_pick_ball/`（撞球/捡球，**未接线**） |
| `test_mode/` | `test_config.py`（`TEST_MODE_ENABLED=True`、`TEST_LOOP=False`、`TEST_TABLE`=5 任务整表串联）+ `test_runner.py`（TestMode 接管 AUV 位，on_exit 发停推 0x09，支持 TEST_LOOP 重跑） |

数据格式与 v2.2 写端**完全兼容**（front/bottom/depth_kalman 不用改）。
v2.2 旧版全套在 `D:\RC\S100\综合\GrandRDKv2.2` 里随时可查。

## PID 中继联动（2026-10-07 v3.8 新增）

`$TASKPID,gate,P,I,D#` 由中位机 `src/to32/task_pid_controller.py` 处理 → 更新 `TaskPidController` 内的真实 `GatePid` 实例；穿门阶段 `PassGateAll`（`task/task_pass_door/t_pass_gate.py`）经 `create_gate_mission` **共用同一对象** → 过门 PID 调参即时生效。参数更新**不启动任务、不切模式**；撞球/捡球未实现，`$TASKPID` 明确拒绝。验证：补丁包 `源码\GrandRdk-2.5\tests\test_task_pid.py`（14 项无硬件回归全过；脚本未拷入本项目）。

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
- 每个 JSON 读端都要容忍「读到半个 JSON」（写端覆盖写）——obs 已处理，别绕过它
- 目标贴边（`clip=True`）时 `w/h` 不可信，判距/穿门判定必须避开
- 阶段内抛异常会被 Mission 捕获并**立即停推收尾**（ABORT）——别依赖异常做正常流程
- `YAW_MIRROR=True` 是抵消固件 Yaw 取负的，别乱关

## 接回主链路（2026-10-07 已完成）

`src/to32/mode_dispatcher.py` 已改为**三态注册**（`_register_modes()`）：测试模式开关开 → `TestMode`；开关关 → `_load_auv_mode()` 动态加载本目录 `AuvMode`（按 `STAGE_TABLE` 自动跑全序列）；`test_config.py` 加载异常（开关状态未知）或 `AuvMode` 导入失败 → 回退内联 `AuvModeStub`（静止，journal 打印原因）。`mode_auv.py` 自带 `sys.path` 注入，`run.sh` 的 PYTHONPATH **不需要加 move_test 段**；`AuvMode.on_exit` 已带停推帧（急停闩锁时让路）。

- 跑正式任务：`test_mode/test_config.py` 置 `TEST_MODE_ENABLED=False` → 切 `mode=1` 即依次自动完成 `STAGE_TABLE` 全部阶段。
- 测试单段：开关置 `True` → `TEST_TABLE` 任意拼。
- `AuvModeStub` 仅剩回退用途：出现即说明配置有错，看 journal 排查。

## 相对 v2.2 移除的能力（需要时按旧版思路重加）

- `kalman_launcher.py`（进 AUV 自动托管两路卡尔曼进程）
- `auv_report.py`（$AUV 状态 5Hz UDP 回传 :8085）
- `viskf_if.py`（过门滤波接口；v2.5 用 VisionIF 直读，要滤波时再重加）
