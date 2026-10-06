# move_test —— AUV 任务代码（v2.5 全新重写骨架）

**状态：骨架已就位，任务阶段表为空 —— 状态机开机即 DONE，不会发任何运动指令。**
主链路（ROV / IDLE）不受影响：`mode_dispatcher.py` 注册的仍是内联 `AuvModeStub`。

## 目录内容（5 个文件，v2.2 的 7 个旧文件已全部删除）

| 文件 | 职责 |
|---|---|
| `task_config.py` | 任务全部参数集中地（不 import to32_config，自包含） |
| `obs.py` | 观测接口：VisionIF（momo_det_*.json）+ DepthIF（momo_depth.json） |
| `mission.py` | 任务状态机：Stage 注册制 + Mission 编排器（阶段表在 task_config） |
| `mode_auv.py` | AUV 模式壳：0x04 切模式 → tick 驱动状态机 → 组 0x09 下发 |
| `README.md` | 本文件 |

数据格式与 v2.2 写端**完全兼容**（front/bottom/depth_kalman 不用改）。
v2.2 旧版全套在 `D:\RC\S100\综合\GrandRDKv2.2` 里随时可查。

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

## 接回主链路（想让 AUV 真正动起来时）

`mode_auv.py` 已自带 `sys.path` 注入，`run.sh` 的 PYTHONPATH **不需要加 move_test 段**。
在 `src/to32/mode_dispatcher.py`：

1. 顶部加 `from mode_auv import AuvMode`
2. 把 `register(AuvModeStub)`（或等价注册行）换成 `register(AuvMode)`

不接回时一切照旧：上位机切 `mode=1` 只会让下位机进 AUV 语义，机器人不动。

## 相对 v2.2 移除的能力（需要时按旧版思路重加）

- `kalman_launcher.py`（进 AUV 自动托管两路卡尔曼进程）
- `auv_report.py`（$AUV 状态 5Hz UDP 回传 :8085）
- `viskf_if.py`（过门滤波接口；v2.5 用 VisionIF 直读，要滤波时再重加）
