# task_pass_door — 穿门任务（方案 v4 + 过门对准算法包，2026-10-06）

方案文档（本目录内）：
- `过门任务方案_从零规划_2026-10-06.md`（定稿 v4：三态流程 / viskf 决策 / 陀螺仪增强）
- `PID对门横向控制方案_2026-10-06.md`（早期方案存档，供追溯）
- 任务骨架：`t_pass_gate.py`（GrandRDKv2.5 task 框架占位，PASS_GATE_TABLE 待实现）

## 算法模块

| 文件 | 职责 | I/O |
|---|---|---|
| gate_config.py | 全部参数（与方案 §四一致） | — |
| gate_pid.py | 位置式 PD(+I备用)：e,ω → Δψ 与 sway | 纯计算 |
| gate_vision.py | 读 momo_det_front.json → e_raw/w_px/可见性 | 读文件 |
| gate_filter.py | e 滤波适配：none 直通 / viskf(momo_viskf.json) / complementary | 读文件 |
| gate_mission.py | 三态流程：等门→对准→关视觉盲跑；锚定/冻结/丢帧 | 注入 tel/send 回调 |
| replay_gate_a.py | 方案A回放：真机视频检测数据开环验证 | 读 dets.jsonl |

## 设计约定

- **纯算法与执行分离**：本包不碰串口、不发 0x09。`GateMission` 通过注入的
  `tel()`（遥测：实际航向/陀螺仪 ω）与 `send(ψ_target, surge, sway)` 回调与外界交互，
  接入 task 框架（t_pass_gate.py 的 Stage）时只写适配器。
- 纯计算模块（gate_pid）仿真与实机共用同一份代码。
- 符号：e 右为正（E_SIGN），ω 右转为正（GYRO_SIGN），都在入口处乘符号。

## 已做验证

- 方案A回放（replay_out/）：2025 帧真机门视频检测数据开环跑通，
  量级/死区/冻结/状态迁移正常；符号待实机 GATE_E_SIGN 标定。
- 方案C（真检测+虚拟机体闭环）待实机窗口做。

## 板端部署（待实机）

随 GrandRDKv2.5 整体上传；实机接线：tel() 接 parse_telemetry 输出，
send() 接 link_stm32 下行组帧，滤波用 `GATE_EF_TYPE='viskf'`。
