# move_test —— AUV 自主运动逻辑（2026-10-05 已接回主链路）

**状态：已接回。** 主链路 dispatcher 注册的是真 `AuvMode`，`mode_auv.py` 驱动
`auv_task.TaskRunner`（任务化执行器）。上位机切 `mode=1` 后机器人**会自主作业**。

## 变更时间线

| 日期 | 变更 |
|---|---|
| 2026-10-04 | 一整套自主运动逻辑从 `src/to32/` 搬到本目录，主链路改注册内联 `AuvModeStub`（只切模式，不发 0x09） |
| **2026-10-05** | **重构为任务化框架 `auv_task/` 并接回主链路**：`mission.py`（805 行单体）→ 14 个独立任务类覆盖 24 个阶段；`mode_dispatcher.py` 换回真 `AuvMode`；`run.sh` 的 PYTHONPATH 补第五段；**删掉 `mission.py`**、**开启测试模式**（`auv_task/test_config.py` 的 `TEST_ENABLED=True`） |

## 目录内容

| 文件 / 目录 | 作用 |
|---|---|
| **`auv_task/`** | ★ **任务化执行框架**（当前在用）。一个阶段 = 一个 `Task` 子类，14 个类覆盖 24 个阶段。详见 **`auv_task/README.md`** |
| `auv_task/test_config.py` | ★ **测试阶段唯一入口**：选段 + 各模块参数 + 全局覆盖 + 离线剧本。改测试配置**只改它**，不用碰 `config/auv_config.py` |
| `auv_task/tests/` | 离线假件 + 台架 + **34 项回归**（板端/Windows 都能跑，不碰串口相机） |
| `mode_auv.py` | AUV 模式壳：`on_enter` 建 `TaskRunner`，`tick` 驱动它并组装 `0x09` 下发；上浮后按 `pop_mode_request()` 请求切回有线 ROV |
| `vision_if.py` | 视觉接口：读 `momo_det_*.json` |
| `depth_if.py` | 深度接口：读 `momo_depth.json` → `D`/`v_z`/`clearance` |
| `viskf_if.py` | 过门滤波接口：读 `momo_viskf.json`（判可用看 **`trust`**，不看 `gate_visible`） |
| `auv_report.py` | `$AUV` 状态回传线程，5 Hz 推 `192.168.127.100:8085`（**重构未改动**） |
| `kalman_launcher.py` | 两路卡尔曼（深度 + 图像）进程托管 |
| ~~`mission.py`~~ | 2026-10-05 **已删除**（旧 805 行单体状态机，无任何 import 引用）。<br>回滚拷贝在 `/userdata/_bak_auvtask_20260925_002758/src/to32/move_test/mission.py` |
| `apply_auv_task_patch.py` | 接入补丁（备份 + 逐项报告 + 幂等），已执行过 |

## 主链路现在是什么样

`src/to32/mode_dispatcher.py`：

- 顶部 `from mode_auv import AuvMode`（真身，从 `move_test` 取）
- `_register_modes()` 里构造 `AuvMode` 而不是 `AuvModeStub`
- 所有模式都回挂了 `m.dispatcher = self`（★ 没有它 AUV 上浮后切不回 ROV —— `mode_base` 只有 `ctx`）

`run.sh` 的 PYTHONPATH 已是**五段**：

```bash
export PYTHONPATH="$CONFIG_DIR:$SRC_DIR:$SRC_DIR/utils:$SRC_DIR/to32:$SRC_DIR/to32/move_test"
```

## 上电前自检

```bash
export PYTHONPATH=/userdata/GrandRDK/config:/userdata/GrandRDK/src:/userdata/GrandRDK/src/utils:/userdata/GrandRDK/src/to32:/userdata/GrandRDK/src/to32/move_test

# 1) 任务化回归（不碰串口/相机）
cd /userdata/GrandRDK/src/to32/move_test/auv_task/tests && python3 test_auv_task.py   # 期望 34/34

# 2) 看测试配置 / 列阶段 / 单段干跑
python3 run_task.py --cfg
python3 run_task.py --list
python3 run_task.py --stage SIT_BOTTOM

# 3) 主链自检（模式注册/切换）
cd /userdata/GrandRDK/src/to32 && python3 selftest_modes.py                            # 期望 77/77
```

⚠ `selftest_modes.py` 里有两条断言与"AUV 是否接回"**强绑定**：
「AUV 期间持续下发 0x09」「AUV 注册的是真 AuvMode」。
若哪天摘回占位壳，这两条要一起翻回去（文件里已注明）。

## 水池测试 vs 验收（只改 `auv_task/test_config.py`）

| 场景 | `TEST_ENABLED` | `TEST_STAGES` |
|---|---|---|
| 单段调试 | `True` | `['SIT_BOTTOM']` |
| 一类模块全测 | `True` | `['task:PassGateTask']` |
| 全流程联调 | `True` | `[]` |
| **验收/比赛** | **`False`** | 任意；并清空 `GLOBAL_PARAMS` |

★ `SURFACE` / `DONE` 强制保留，任何配置都删不掉 —— 上浮不允许被关闭。

## 相关备份

- 本次改动：`/userdata/_bak_auvtask_*`（move_test 整目录 + dispatcher + auv_config + run.sh）
- 补丁自备份：`move_test/mode_auv.py.bak_auvtask_*`、`src/to32/mode_dispatcher.py.bak_auvtask_*`、`run.sh.bak_auvtask_*`
- 更早的摘除备份：`/userdata/_bak_auvmove_*`

## 现场必须标定的占位值（🔴）

`AUV_POOL_DEPTH_CM`(130) / `AUV_YAW_RATE_DPS`(30) / `AUV_TURN_RIGHT_SIGN`(+1) /
`AUV_SPEED_MPS`(0.25) / `AUV_RAM_W_PX`(220) / `AUV_GATE_COUNT`(4) / `AUV_GATE_SEQ`。
标定方法见 `auv_task/README.md` §4 各模块说明与《AUV任务化重构方案_2026-10-04.md》§16.10.1。
