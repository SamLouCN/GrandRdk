# GrandRDK v2.5 · 板端视觉与感知总控

> **v3.8 PID修正**：STM32二进制PID默认只转发0～3；4～7是参考补偿环，需核实后另行启用。
> 新增 `$TASKPID` 命名S100参数及确认，当前仅gate；直接更新实际GatePid，PASS_GATE_TABLE可选阶段共用同一对象。未实现的撞球/捡球明确拒绝。参数更新不会启动任务或切模式，默认任务排序保持原样。下方原说明为历史版本记录。

> **2026-10-09 YOLO 模型按阶段动态切换（stage_model）**：新增 `config/stage_model.py`——
> 前视/下视 worker 改用 `StageDetector`，每帧读 `/dev/shm/momo_stage.json`（中位机
> `mission._set_stage` 原子发布），按 AUV 阶段在 `ball`(test_nashe)/`gate`(door_3)/`pick`(bottom)
> 三套模型配置间**热切换**，未映射阶段（IDLE/DONE）用 `quick_config` 基线；
> 切换即丢弃推理期间过时结果、加载失败 1s 重试。**2026-10-08 基线已回退单模型**：
> 前后视均 `door_3_640x640.hbm`、只找 `door`；撞球/捡球阶段由 stage_model 切到对应模型，
> `obs.CANON` 别名归一化（door→gate、red-ball→ball）不受影响。

> **2026-10-07 PID中继适配**：本工作副本已接通ROV控制站v3.7的UDP二进制PID。
> `CD 0A 01 CH KP_LE KI_LE KD_LE DC`（11B，**默认放行 CH 0～3；4~7 为参考补偿环、待核实后另行启用**，与上行 v3.8 修正一致）先校验后原字节转发STM32；
> 文本控制继续原路径，旧文本PID仍丢弃。`config/to32_config.py` 的
> `PID_BINARY_PASSTHRU=True` / `PID_BINARY_MAX_CH=3` 控制新中继。
> 串口采用整帧互斥，失败/短写/空跑分别记录；转发不切模式或解除急停。
> 下文PID旧说明为原包历史记录；真实固件通道和参数生效尚需台架核对。

RDK S100 上跑的水下机器人（ROV/AUV）**板端软件总线**：两路相机采集 → BPU 跑 YOLO → 超声波测高 → 经 Web/UDP 把画面和遥测交给上位机与中位机。（光流测速已于 2026-10-04 整体停用；AUV 自主运动 2026-10-04 从主链路摘除、2026-10-06 在 `move_test/` 重写为 v2.5 骨架，**2026-10-08 已接回主链路**：三态注册 + 卡尔曼托管恢复 + 遥测落盘，2026-10-09 全序列含穿门并支持 YOLO 按阶段热切换。）

> 本机信息：hostname `neos100`，IP `192.168.127.10`（eth1；eth0=192.168.128.10），Ubuntu 22.04.5 LTS，内核 `6.1.158-rt58`（PREEMPT_RT 实时内核）
> **文档更新：2026-10-06**（依 v2.5 镜像实测重整；同日晚间追加深度卡尔曼 2026-10-06 实测落档：B/C 安装参数 + 失效硬拒 + 耦合配对修正 + H_M 标定法）
> **文档更新：2026-10-07**（PID 中继 v3.8 落地：新增《PID 调参链路》一节；`src/to32/`、`src/to32/move_test/`、`src/to32/move_test/task/task_pass_door/` 的 README 同步更新）
> **文档更新：2026-10-09**（同步 2026-10-08 模型回退单模型 + `kalman_launcher` 托管恢复 + `tel_shm_sink` 遥测落盘 + 三态注册（AuvMode 已可注册）；2026-10-09 `stage_model` 按阶段热切换 + 穿门重构为 `task/task_door/`（`t_door.py` `DoorTask`，`DOOR_TABLE` 已入 `STAGE_TABLE`） + 撞球 v2 + STOP 哨兵）
> 自写 Python 约 **13.2k 行** = 根级脚本 775 + `src/` 主工程约 11394（含 `src/to32/` 中位机 3508（内测试脚本 931）、`src/kalman/` 两路卡尔曼 2283、`src/to32/move_test/` 任务代码 2686）+ `config/` 约 1070；**2026-10-07 更新**：to32/move_test 行数为补丁后本地实测（原 v2.5 镜像口径 3640/930/510 已替换）；
> 另有 vendored 库 `src/utils/py_utils` 2299 行（第三方，**别改**）。
> ★ v2.5 结构变化：① **`src/to32/move_test/` 全新重写**（v2.2 的 7 文件全删，换成 `task_config/obs/mission/mode_auv` 骨架 + `Task.md`；**2026-10-08 起 `AuvMode` 已可按开关注册回主链路、`kalman_launcher.py` 已恢复（深度自动托管）**）；② **`run.sh` 的 `read_cfg` stdout 污染已修复**（2026-10-04，旧 README 误记为未修）；③ **新增 `config/udev/99-rover-cameras.rules`**（⚠ 与 `camera_ports.py` 的 cam1/cam2 物理口记载相反，待现场核对）；④ 前视目标含 `red-ball`（撞球要靠前视找球，2026-09-26 改动，**2026-10-08 回退单模型后基线只找 `door`，球类由 stage_model 切换**）。

> 本 README 依据板端源码实测整理；若发现与代码不一致，**以代码注释为准**并顺手改回这里。

---

## ★ 当前功能状态一览（2026-10-09，先看这里）

| 功能 | 状态 | 说明 |
|---|---|---|
| 相机采集 + YOLO 检测（front/bottom） | ✅ 运行中 | 基线（IDLE/DONE）cam1/cam2 都找 `door`（2026-10-08 回退单模型）；**AUV 任务运行时由 `stage_model` 按阶段热切换**（撞球→test_nashe 找球、穿门→door_3、捡球→bottom.hbm），画框后写共享内存 |
| YOLO 模型按阶段动态切换 | ✅ 已接线 | `config/stage_model.py` `StageDetector`：读 `momo_stage.json` → 模型配置变化即热重载；切换丢旧结果、失败 1s 重试。详见文件地图 |
| 图像回传 `/cam1` `/cam2`（:5000） | ✅ 运行中 | web_server 读共享内存发 MJPEG，**带检测框**，是上位机画面唯一来源；与光流/AUV 完全无关 |
| 检测状态接口 `/api/status` `/api/detections` `/ws/status` `/healthz` | ✅ 运行中 | |
| 监控网页（Nginx :80） | ✅ 运行中 | 浏览器用，反代 :5000 |
| CAM3 内窥镜按需推流 :8084 | ✅ 运行中 | 取流才开相机、空闲 5s 释放 |
| 高度计读取 `read_altimeter.py` | ⚠ 运行但无数据 | 五路持续 no-reply（自动降 2s 低频探测）；`$ALT` 照常推上位机 :8082。深度卡尔曼侧已加**失效硬拒**（净空出界 / 姿态超限直接拒，2026-10-06） |
| 中位机（UDP 控制 / 遥测转发） | ✅ 运行中 | 上电停 **IDLE 待命**；行为见下节《模式与状态》 |
| PID 参数下发（二进制中继 v3.8） | ✅ 软件链路已通（**未实机**） | 上位机 v3.8 发 11B 二进制 PID → S100 校验后**原字节转发** STM32（默认仅 CH 0~3）；`$TASKPID,gate,P,I,D#` 更新过门参数并 ACK；旧文本 `$PID` 仍丢弃。详见下节《PID 调参链路》 |
| ROV 手动遥控 | ✅ 可用 | 摇杆 → `0x09` 四轴直通；需上位机显式发 `mode=0` 激活 |
| AUV 自主任务（v2.5 阶段注册制状态机） | ⚠ **已接回主链路；当前 AUV 位由测试模式接管** | 2026-10-08 起 `mode_dispatcher` 三态注册：`TEST_MODE_ENABLED=True`（当前默认）→ `TestMode` 跑 `TEST_TABLE`（**当前=Task2+Return**）；`False` → 正式 `AuvMode` 按 `STAGE_TABLE` 跑**全序列**（Task1→撞球→Task2→**穿门 `DOOR_TABLE`**→SearchBall→捡球→Task4→Return）；测试配置异常 → `AuvModeStub` 兜底宁停勿跑 |
| 两路卡尔曼自动拉起 | ✅ 深度自动拉起（viskf 默认关） | `move_test/kalman_launcher.py` 已恢复：`AUV_KALMAN_AUTOSTART=True`（进 AUV 即带起深度卡尔曼）；`AUV_VISKF_AUTOSTART=False`（viskf 默认不拉起，需要时置 True） |
| `$AUV` 状态上报 :8085 | ⛔ 已删 | `auv_report.py` 已随 v2.2 重写删除（需要时按旧版思路重加） |
| 遥测落盘 `momo_telemetry.json` | ✅ 已接回 | `to32/tel_shm_sink.py`：`mode_dispatcher.on_stm32_telemetry` 每帧写，深度卡尔曼等观测工程的遥测输入源（2026-10-08） |
| 光流测速 :8000 | ⛔ **已停用** | 计算+共享全停（`FLOW_DISABLED=1` + 两个配置开关 False）；代码与参数保留，一键可恢复 |
| 中位机自带图像回传 `video.py` | ⛔ 停用（长期） | 2026-09-21 起 `VIDEO_PATHS={}`；上位机画面来自 web_server，与它无关 |

---

## ★ 模式与状态：会发生什么（2026-10-08 口径，三态注册）

中位机（`src/to32/`）有三个模式。**上电 `./run.sh` 后停在 IDLE，等上位机指令**：

| 模式 | id | 怎么进 | 发 0x04 模式帧 | 发 0x09 运动帧 | 发 0x0C 遥测请求 | `$TEL` | 会发生什么 |
|---|---|---|---|---|---|---|---|
| **IDLE 待命** | -1 | 上电默认；`--mode idle` | ❌ | ❌ | ❌ | 41 字段全 0 占位帧 | 静默待机：机器人不动，下位机收不到任何帧；上位机凭全 0 `$TEL` 知道板子在线 |
| **ROV 手动** | 0 | `$CMD mode=0`；`--mode rov` | 进入时一次 | ✅ 每帧直通 | ✅ 周期轮询 | 真遥测 | 摇杆即推力（四轴直通，四轴全 0 才判回中）；图像/检测照常 |
| **AUV 自主** | 1 | `$CMD mode=1`；`--mode 1` | 进入时一次（0x05） | ⚠ 视接管者 | ✅ 周期轮询 | 真遥测 | **2026-10-08 三态口径**：① `TEST_MODE_ENABLED=True`（当前默认）→ AUV 位被 `TestMode` 接管，跑 `TEST_TABLE`（**当前=Task2+Return**）下发 0x09；② `False` → 正式 `AuvMode`（`move_test/mode_auv.py`，按 `STAGE_TABLE` 全序列含穿门跑）并**自动拉起深度卡尔曼**；③ `test_config` 异常/`AuvMode` 导入失败 → `AuvModeStub` 兜底（只切模式、不动）。无 `$AUV` 上报 |

切换语义（重要，逐条都是实测结论）：

1. **每一帧带 `mode` 字段的 `$CMD` 都会立即切模式**——没有来源锁定、没有防抖。上位机 `build_cmd` 恒发 9 字段（含 `mode`），所以"上位机一发 `$CMD` 就离开 IDLE"是预期行为。
2. IDLE 待命态**只认显式带 mode 的帧**（老 6 段 `$CMD` 不会误激活）；`protocol.parse_cmd` 用 `mode_explicit = len(p)>6` 判别。
3. ⚠ **双源并发会高频抖动**：只要有两个来源同时发带 mode 的 `$CMD`（例如上位机 + 板端调试注入），模式就逐帧互抢——实测 12 秒内切换 **464 次**、向下位机灌了 463 帧 `0x04`。现象是 `to32_main.log` 疯狂刷"进入 AUV/退出 AUV/进入 ROV"。**调试注入与上位机不要同时发帧。**
4. **图像回传与模式无关**：任何模式下 `/cam1` `/cam2` 都正常供流。
5. 急停：自动急停+锁存**默认关闭**，要启用 `./run.sh --to32-estop`。

想让 AUV 真正动起来（v2.5 两步，**2026-10-08 起第二步已内置**）：
1. **填阶段**：按 `src/to32/move_test/Task.md`（2026 巡游任务阶段划分）在 `task/` 里写 Stage 子类，排进 `task_config.STAGE_TABLE`（当前全序列：Task1→撞球→Task2→穿门→SearchBall→捡球→Task4→Return）；
2. **接回主链路**：`mode_dispatcher._register_modes()` 已是三态——把 `test_mode/test_config.py` 的 `TEST_MODE_ENABLED` 置 `False` 即注册正式 `AuvMode`（`move_test/mode_auv.py` 自带 sys.path 注入，run.sh 不用改）。
   ⚠ 当前默认 `TEST_MODE_ENABLED=True`，AUV 位由 `TestMode` 接管跑 `TEST_TABLE`（当前=Task2+Return）；要跑正式 `STAGE_TABLE` 先把开关置 `False`，二选一，不要同时生效。

---

## ★ PID 调参链路（2026-10-07 v3.8 落地）

上位机 v3.8 起，PID 增益不再走旧文本 `$PID`（仍丢弃），改走 **11B 二进制帧**：

```
CD 0A 01 CH KP_LE KI_LE KD_LE DC
```

（`CH`=通道号；增益 ×100、int16 小端、±327.68 边界）。中位机 `link_pc` 先判二进制 → `protocol.parse_binary_pid` 严格校验（帧长/头/功能码/编号上限/帧尾）→ `mode_dispatcher` **原字节转发** STM32（独立于模式切换、不解除急停、不保存）；文本 `$CMD/$VID/PING` 路径不变。

- **通道口径**：默认只放行 **CH 0~3**（Pitch/Yaw/Roll/Depth）；4~7 是参考固件的补偿环，`PID_BINARY_MAX_CH=3` 写死，待台架核实后另行放宽。
- **`$TASKPID,<req>,gate,P,I,D#`**（命名任务参数，仅 S100 处理）：直接更新过门 `GatePid` 实例并回 ACK（UDP 8081）；**不启动任务、不切模式**；撞球/捡球未实现，明确拒绝。
- **配置开关**（`config/to32_config.py`）：`PID_BINARY_PASSTHRU=True` / `PID_BINARY_MAX_CH=3` / `TASK_PID_ENABLED=True`。
- **已落地验证**：补丁包 `源码\GrandRdk-2.5\tests\test_pid_relay.py`（16 项）+ `test_task_pid.py`（14 项）无硬件回归全过（脚本**未拷入项目/未入 git**，从补丁包目录直接可跑）；**未实机**——STM32 对 0~3 的真实接受/保存、4~7 补偿环存在性，待实体联调核对。
- **上位机配套**：PC 侧必须用 `ROV_ControlStation_v3.8`（二进制 PID 面板 + `$TASKPID` 确认）。

---

## 30 秒快速开始

```bash
cd /userdata/GrandRDK
./run.sh          # 一键启动（相机 + 检测 + Web + 高度计 + 中位机 + Nginx；光流已停用）
./status.sh       # 看各进程是否 ON
./logs.sh front   # 看某一路日志
./stop.sh         # 全部停止

# ★ 中位机模式（上电默认 IDLE 待命，需显式切换）
./run.sh --to32-args "--mode rov"   # 切到 ROV 手动模式
./run.sh --to32-args "--mode 1"     # 切到 AUV（2026-10-09：默认 TestMode 接管跑 TEST_TABLE=Task2+Return；
                                    #   正式任务：test_mode/test_config.py 置 False 即注册 AuvMode 跑 STAGE_TABLE 全序列）
./run.sh --to32-args "--mode 1 --stm32 sim"   # 干跑（不下发真实指令）
```

> ★ **中位机上电停在 IDLE 待命态（2026-10-04 变更）**：`./run.sh` 起来后中位机**不自动进入 ROV/AUV**，
> 也不发 `0x04`（模式）/`0x09`（运动）/`0x0C`（遥测请求），只回一份全 0 占位 `$TEL`
> 让上位机知道在线。上位机下发**带 `mode` 字段**的 `$CMD` 后才切到对应模式
> （`config/to32_config.py` 的 `START_MODE = MODE_IDLE`）。

> ⚠ **卡尔曼托管状态（2026-10-08 恢复）**：托管方 `src/to32/move_test/kalman_launcher.py` 已恢复，
> 挂在 `move_test/mode_auv.on_enter/on_exit` —— **进 AUV 自动带起、退出自动停（只停自己起的那个）**。
> 当前开关（`move_test/task_config.py`）：`AUV_KALMAN_AUTOSTART=True`（深度，自动拉起）、
> `AUV_VISKF_AUTOSTART=False`（viskf，默认关，需要时置 True）。单独调试仍可用各自的 `./run.sh --daemon`：

| 卡尔曼 | 目录 | 读 | 写 |
|---|---|---|---|
| 深度 | `src/kalman/depth_kalman/` | `momo_alt.json` + `momo_telemetry.json` | `momo_depth.json` |
| 图像（穿门） | `src/kalman/camera_kalman/` | `momo_det_front.json` | `momo_viskf.json` |

跑起来后：

| 想看什么 | 打开哪里 |
|---|---|
| 监控网页 | `http://<板端IP>/`（Nginx :80） |
| 前视画面 /cam1、下视 /cam2 | `http://<板端IP>:5000/cam1`、`:5000/cam2` |
| 检测状态 JSON | `http://<板端IP>:5000/api/status` |
| ~~光流测速调试画面~~ | ⛔ 已停用（2026-10-04），:8000 不再监听 |
| 第三路相机 CAM3 | `http://<板端IP>:8084/stream` |
| 高度计（UDP，非 HTTP） | 板端主动推 `$ALT` 帧到上位机 `:8082` |

> 🧪 **hwless_tests/ 不在本 v2.5 镜像中**（开发机工程独有，板端也没有）：无外接硬件全流程测试
> `python3 hwless_tests/run_all.py -v`（2026-10-01 实测 93/93 PASS 约 68s）与遗产自检 `run_legacy.py`（4/4 子套件）
> 都在开发机工程的 `hwless_tests/` 里；要在本机跑需先取该目录。

---

## 文件地图

| 路径 | 作用 |
|---|---|
| `run.sh` | 统一启动脚本，参数见下节 |
| `status.sh` / `stop.sh` / `logs.sh` | 状态查询 / 停止 / 看日志 |
| `src/kalman/depth_kalman/` | ★ 深度卡尔曼（2026-10-01 归位到此）。**源码平铺**在本目录（`main/kf/model/sources/sinks/samples/fusion/cfgutil` 8 文件 1397 行），**配置在 `config/depth_config.py`**。只读 `momo_telemetry.json`/`momo_alt.json`，只写 `momo_depth.json`。✅ **2026-10-08 起由 AUV 模式自动拉起**（`move_test/kalman_launcher.py` `DepthKalmanLauncher`，`AUV_KALMAN_AUTOSTART=True`）；单独调试用它自己的 `./run.sh --daemon` |
| `src/kalman/camera_kalman/` | ★ **图像卡尔曼（viskf）**：`viskf.py`（886 行）平铺在本目录，**配置在 `config/viskf_config.py`**。读 `momo_det_front.json`，写 `momo_viskf.json`。托管方已恢复但**当前默认不拉起**（`AUV_VISKF_AUTOSTART=False`），需要时置 True |
| `config/depth_config.py` | ★ 深度卡尔曼的**全部可调参数**（2026-10-01 从卡尔曼包里搬上来，与 `auv_config.py` 同级） |
| `config/viskf_config.py` | ★ 图像卡尔曼的**全部可调参数**（同上，键名前缀 `VISKF_`） |
| `config/udev/99-rover-cameras.rules` | ★ udev 物理口↔相机绑定规则（SYMLINK `cam1/cam2/cam3`，只认 index=0 采集节点）。⚠ **其 cam1=口1-2 / cam2=口3-2 与 `camera_ports.py`（cam1=3-2 / cam2=1-2）记载相反**，二选一必有一个过时，启用前现场核对 |
| `config/auv_config.py` | AUV 旧参数段（`to32_config.py` 末尾 `from auv_config import *` 并入）。⚠ **v2.5 任务参数已迁 `src/to32/move_test/task_config.py`**（自包含），调任务改那边 |
| `nginx_setup.sh` | Nginx 安装与站点配置 |
| `config/quick_config.py` | **日常调参入口**：相机节点、目标类别、阈值、开关、BPU 核。⚠ **2026-10-08 回退单模型**：`YOLO_MODEL_FRONT`/`YOLO_MODEL_BOTTOM` 均 `door_3_640x640.hbm`、`FRONT_TARGETS`/`BOTTOM_TARGETS` 均 `['door']` |
| `config/stage_model.py` | ★（2026-10-09 新增）**YOLO 模型按阶段动态切换**：`MODELS` 三套配置（ball=test_nashe / gate=door_3 / pick=bottom.hbm）+ `STAGE_MODELS` 阶段映射 + `StageDetector`（每帧读 `momo_stage.json`，配置变化即热重载、`is_current()` 丢过时结果、失败 1s 重试） |
| `config/main_config.py` | 全局配置：路径、共享内存、光流、Web；从 quick_config 合成 `DEFAULT_CONFIG`。⚠ 光流两开关（`FLOW_ENABLE` / `ENABLE_FLOW_SHARE_BOTTOM`）**已 False** |
| `config/to32_config.py` | **中位机配置**：端口、串口、模式、PID、映射量纲、图像回传；`START_MODE = MODE_IDLE` |
| `config/nginx/vp.conf` | Nginx 站点（静态页 + `/cam1` `/cam2` `/api` `/ws` 反代） |
| `src/front.py` | 前视进程：采集 → YOLO 检测（**2026-10-09 起经 `StageDetector` 按阶段热切换模型**；基线只找 `door`）→ 写共享内存；`PassGate` 阶段走穿门视觉跟踪管线 |
| `src/bottom.py` | 下视进程：采集 → YOLO 检测（同上 `StageDetector`；基线只找 `door`）→ 写共享内存（原供光流的 NV12 帧已停写，2026-10-04） |
| `src/function.py` | YOLO 检测器适配（hbm / ultralytics）+ 水下图像预处理 |
| `src/hw_camera.py` | USB MJPG 相机封装：ctypes 调 JPU 硬件解码 |
| `src/shm_writer.py` / `shm_reader.py` | `/dev/shm` 帧与 JSON 的读写封装 |
| `src/web_server.py` | FastAPI 服务 `:5000`，读共享内存对外供流与接口；★ **上位机图像回传唯一来源** |
| `src/flow_speed.py` | ⛔ **光流测速，已停用（2026-10-04）**：`run.sh` 不再拉起，文件保留只加横幅；`:8000` 不再监听。恢复方法见《变更记录》 |
| `src/show_cam.py` | 第三路相机 CAM3，**按需**采集推流 `:8084` |
| `src/read_altimeter.py` | CH348 A–E 口读 DYP-L08 超声波高度计（Modbus-RTU）→ UDP `$ALT` 推上位机 `:8082` |
| `src/to32/` | **中位机源码**：`main.py` 入口、`link_pc/link_stm32` 双链路、`mode_idle/mode_rov` 模式层、`mode_dispatcher`（三态注册：TestMode/AuvMode/AuvModeStub）、`protocol`/`tel_builder`/`video`，+ **2026-10-07 `task_pid_wire.py`/`task_pid_controller.py`（PID 中继 + `$TASKPID`）** + **2026-10-08 `tel_shm_sink.py`（遥测落盘 `momo_telemetry.json`）**（详见 **`src/to32/README.md`**） |
| `src/to32/move_test/` | ★（2026-10-06 v2.5 全新重写，**2026-10-08 接回主链路、2026-10-09 全序列含穿门**）**AUV 任务代码**：`task_config.py`=任务参数+`STAGE_TABLE`（全序列：Task1→撞球→Task2→穿门 `DOOR_TABLE`→SearchBall→捡球→Task4→Return；含 `AUV_KALMAN_AUTOSTART`/`AUV_VISKF_AUTOSTART` 卡尔曼托管开关）；`obs.py`=`VisionIF`+`DepthIF`（`CANON` 别名归一化）；`mission.py`=Stage 基类+Mission 编排器（**每阶段 `_set_stage` 发布 `momo_stage.json` 驱动视觉换模型**，支持 `task_pids` 注入、STOP 哨兵）+ `kalman_launcher.py`（**深度自动托管，已恢复**）+ `mode_auv.py`=AUV 模式壳 + `Task.md` + `task/`（`t_task1/t_task2/t_search_ball/t_task4/t_return/t_hit_ball/t_hit_ball_v2/t_pick_ring` + **穿门 `task/task_door/`（`t_door.py` `DoorTask`，`DOOR_TABLE`）**）+ `test_mode/`（**`TEST_MODE_ENABLED=True`，切 mode=1 即跑 `TEST_TABLE`=Task2+Return**）。⚠ 当前 AUV 位由 TestMode 接管；v2.2 旧 7 文件全套在 `D:\RC\S100\综合\GrandRDKv2.2` 可查 |
| `src/to32/` | （2026-09-22 由 `docs/` 合并迁入）中位机文档：`README.md`、`README_中位机.md`、`ROV_指令与V2协议对应关系.md`、`上位机通讯协议.md` |
| `logs/to32_mode_state.json` | 中位机模式记忆文件（原子写；当前 `MODE_PERSIST=False` 不写） |
| `src/utils/flow_share.py` | 帧共享桥，flock 互斥避免读写撕裂（光流停用后暂无消费者，保留） |
| `src/utils/py_utils/` | **第三方 vendored 库**（非本项目代码，别改） |
| `models/*.hbm` | BPU 模型（3 个）：`door_3_640x640.hbm`（基线/穿门，前后视默认）、`test_nashe_640x640_nv12.hbm`（撞球/寻球）、`bottom.hbm`（捡球）——由 `stage_model.py` 按阶段选用 |
| `libs/libmjpg_hw.so` | JPU 硬件解码动态库 |
| `web/index.html` | 监控页面（Nginx 实际分发，**Win8.1 磁贴风**） |
| `web/index_v2.html` | 新版页面，**尚未接入**（Nginx `index index.html`，代码里没人引用） |
| ~~`hwless_tests/`~~ | **本 v2.5 镜像未包含**（开发机工程独有，板端也没有）：无硬件全流程测试 `run_all.py -v`（2026-10-01 实测 93/93 PASS 约 68s）与遗产自检 `run_legacy.py`（4/4 子套件）都在开发机工程的该目录里，需要时从那里取 |

**备份文件**：2026-10-04 起的板端改动备份在 `/userdata/_bak_flowoff_*`、`/userdata/_bak_auvmove_*`、`/userdata/_bak_idle_*`、`/userdata/_bak_to32_*`；更早的 `.bak*` 已于 2026-10-01 清理干净。

> 上面是"看一眼"的速查表；**每个文件的完整说明见下一节《各目录逐文件作用》**。

---

## 各目录逐文件作用（详细）

按目录逐个说明。行数 / 体积取自 2026-10-06 v2.5 镜像实测。`__pycache__/` 为 Python 自动生成的字节码缓存，不算源码。

### 1. 根目录 —— 运维脚本

| 文件 | 行数 | 作用 |
|---|---|---|
| `run.sh` | 444 | **总启动脚本**。检查依赖 → 准备 Nginx → 按序拉起 front/bottom/read_altimeter/中位机/show_cam/web_server（光流段被 `FLOW_DISABLED=1` 短路跳过）→ 前台 `wait` 检测进程；任一检测进程退出就 kill 全部子进程后自己退出。★ `read_cfg` 的 stdout 污染**已修复**（2026-10-04：import 期 stdout 改道 stderr + `tail -1` 兜底） |
| `status.sh` | 127 | 巡检：逐进程打印 ON/OFF，附共享内存文件与端口占用（光流已标"停用"） |
| `stop.sh` | 36 | 停止全部。uvicorn 在被上位机占流时优雅关闭会挂住，故有超时后 SIGKILL 兜底 |
| `logs.sh` | 14 | 看日志的薄封装：`./logs.sh front\|bottom\|web\|alt\|to32\|flow\|showcam\|all` |
| `nginx_setup.sh` | 154 | 安装 nginx、写入 `config/nginx/vp.conf`、启动站点（`:80` 静态页 + 反代 `:5000`） |
| `README.md` | — | 本文件（整工程说明） |
| `.gitignore` | 287 B | 忽略 `__pycache__/`、`logs/`、`*.pyc` 等 |

### 2. `config/` —— 配置（两层 + 中位机）

| 文件 | 行数 | 作用 |
|---|---|---|
| `quick_config.py` | 56 | **日常调参入口**：相机节点（`device_for('cam1')`/`device_for('cam2')`）、目标类别、阈值、推理开关、BPU 核。⚠ **2026-10-08 回退单模型**：`YOLO_MODEL_FRONT=YOLO_MODEL_BOTTOM='door_3_640x640.hbm'`、`FRONT_TARGETS=BOTTOM_TARGETS=['door']`、`FRONT_CLASS_NAMES=BOTTOM_CLASS_NAMES=['door']`（旧 `YOLO_MODEL` 单键保留注释作回退参考） |
| `stage_model.py` | 143 | ★（2026-10-09 新增）**YOLO 按阶段热切换**：`MODELS` 三套配置（`ball`→test_nashe 找 red-ball / `gate`→door_3 找 door / `pick`→bottom.hbm 找 red-ball）+ `STAGE_MODELS` 阶段映射（Task1/SearchBall/HitBall→ball；Task2/PassGate→gate；PickBall→pick）+ `publish_stage()`/`StageDetector`（读 `/dev/shm/momo_stage.json`，模型配置变化即重载，推理期间切走丢旧结果，加载失败 1s 重试） |
| `main_config.py` | 216 | 全局配置：路径、`/dev/shm` 键名、光流 `FLOW_*`、Web。⚠ `FLOW_ENABLE=False`、`ENABLE_FLOW_SHARE_BOTTOM=False`（2026-10-04 光流停用） |
| `to32_config.py` | 134 | **中位机配置**：UDP 8080/8081、串口、模式表、PID 系数、量纲映射、图像回传开关（停用）；`MODE_IDLE=-1`、`START_MODE=MODE_IDLE`（待命）；末尾 `from auv_config import *` |
| `auv_config.py` | 216 | AUV 旧参数段（被 `to32_config.py` 并入）。⚠ **v2.5 任务参数已迁 `src/to32/move_test/task_config.py`**（自包含、不 import to32_config；`AUV_SPEED_MPS`/`AUV_YAW_RATE_DPS`/`AUV_POOL_DEPTH_CM` 等占位值待上车标定） |
| `depth_config.py` | 231 | ★ **深度卡尔曼**的全部可调参数。🔴 `H_M=1.3` 仍是**占位值**（2026-10-06 现场实测水深约 1.2 m，上机前按"水面标定法"重标，预计 ≈1.2：机器人放水面、深度计归零、读锚路 B 净空即 H_M）。2026-10-06 已落档：B/C 安装参数（C 比 B 低 20mm，`ALT_MOUNT['C'].dz=0.02`）、失效硬拒门限（`ALT_MIN/MAX_VALID_M` 0.02–2.5、`ALT_MAX_TILT_DEG=30`）、观测方程耦合配对修正（y↔俯仰、x↔横滚；pitch/roll 锁 0 期间无影响） |
| `viskf_config.py` | 109 | ★ **图像卡尔曼**的全部可调参数，键名前缀 `VISKF_` |
| `camera_ports.py` | 106 | 相机按 USB 物理口绑定的真值表（`CAM_PORTS`：cam1=3-2、cam2=1-2、cam3=1-1）+ `device_for()` 解析。⚠ 与 `config/udev/99-rover-cameras.rules` 的 cam1/cam2 记载相反，见已知问题 |
| `udev/99-rover-cameras.rules` | 18 | udev 规则：按 KERNELS 物理口建 `cam1/cam2/cam3` 软链（只作用于 index=0 采集节点）。**启用前必须先核对与 `camera_ports.py` 的矛盾** |
| `nginx/vp.conf` | 1.2 KB | Nginx 站点：静态页 + `/cam1` `/cam2` `/api` `/ws` 反代到 `:5000` |

> ★ 两个卡尔曼的**配置已收进本目录**，源码留在 `src/kalman/<名字>/`。
> 代价是它们的 PYTHONPATH 必须是**两段**：`/userdata/GrandRDK/config:<工程根>`。
> 各自的 `run.sh` 已自动算好；**手动起进程忘了第一段会直接 `ModuleNotFoundError: depth_config`**。

### 3. `src/` —— 主工程源码（前视/下视/Web/高度计 + 卡尔曼）

| 文件 | 行数 | 作用 |
|---|---|---|
| `front.py` | 511 | **前视进程**：cam1（USB 物理口 3-2）采集 → YOLO 检测（**2026-10-09 起经 `stage_model.StageDetector` 按阶段热切换**；基线只找 `door`；`PassGate` 阶段走穿门视觉跟踪）→ 叠字 + 写共享内存。绑核 CPU 0-2 |
| `bottom.py` | 543 | **下视进程**：cam2（USB 物理口 1-2）采集 → YOLO 检测（同上 `StageDetector`；基线只找 `door`）→ 写共享内存。绑核 CPU 3-5。⚠ 原供光流的 NV12 共享帧已由 `FLOW_SHARE_DISABLED=True` 硬开关停写 |
| `function.py` | 509 | YOLO 检测器适配层（`hbm_runtime` / `ultralytics` 两种后端）+ 水下图像预处理，被 front/bottom 共用 |
| `show_cam.py` | 406 | **第三路相机 CAM3**：cam3（USB 物理口 1-1 内窥镜），**按需采集**（HTTP 常驻但不开相机，取流才开、空闲 5s 释放），推流 `:8084/stream` |
| `flow_speed.py` | 338 | ⛔ **光流测速，已停用（2026-10-04）**：读 bottom 的 NV12 帧、Mali GPU 稀疏 LK 光流、调试画面 `:8000` —— 全部不再运行；文件保留只加停用横幅 |
| `web_server.py` | 298 | FastAPI 服务 `:5000`：读共享内存对外供 `/cam1` `/cam2` MJPEG、`/api/status` `/api/detections`、`/ws/status`、`/healthz`；推流有节流。★ **上位机图像回传的唯一来源（带检测框）** |
| `read_altimeter.py` | 347 | **超声波高度计**：CH348 A–E 口读 DYP-L08（Modbus-RTU）→ 组 `$ALT,<通道>,<mm>,<状态>#` 帧 UDP 推上位机 `:8082`；同时落 `/dev/shm/momo_alt.json`；无应答自动降 2s 低频探测 |
| `hw_camera.py` | 148 | USB MJPG 相机封装：ctypes 调 `libs/libmjpg_hw.so` 走 JPU 硬件解码出 BGR/NV12 |
| `shm_writer.py` | 85 | `/dev/shm` 写端：JPEG 帧（`momo_frame_*.bin`）+ det/stats JSON 的统一写入 |
| `shm_reader.py` | 91 | `/dev/shm` 读端：web_server 等消费者用；**新增消费者只读共享内存，绝不直接开相机** |
| **`kalman/`** | **2283** | ★ **两路卡尔曼**：`depth_kalman/`（深度，8 文件 1397，写 `momo_depth.json`）与 `camera_kalman/`（图像 viskf，`viskf.py` 886，写 `momo_viskf.json`）。源码**平铺**在各子目录内，**配置已收进 `config/`**，各自带 `run.sh`/`stop.sh`/`README.md`。✅ **2026-10-08 起深度由 AUV 自动拉起**（`move_test/kalman_launcher.py`，`AUV_KALMAN_AUTOSTART=True`）；viskf 默认关（`AUV_VISKF_AUTOSTART=False`）。完整说明见各子目录 `README.md` 与 **`src/README_src.md` §五** |
| `README_src.md` | 372 | **`src/` 目录说明**（2026-10-09 同步 v2.5 重写：move_test 结构、stage_model、遥测落盘、卡尔曼托管） |

### 4. `src/utils/` —— 帧共享 + vendored 库

| 文件 | 行数 | 作用 |
|---|---|---|
| `flow_share.py` | 148 | **自写**。帧共享桥：把检测进程解出的帧经 `/dev/shm/momo_flow_<name>.bin` 给光流进程，header 32B（magic `MFS1` + seq + w/h + fmt 共 24B，紧跟 8B ts_us），`flock` 互斥防读写撕裂。⚠ 光流停用后暂无消费者，代码保留 |
| `py_utils/` | 约 2299 行 | **第三方 vendored 库**（D-Robotics 官方 sample 工具，Apache-2.0）——**不是本项目代码，别改** |

`py_utils/` 内部（体积为文件大小）：

| 文件 | 体积 | 作用 |
|---|---|---|
| `__init__.py` | 190 B | 一次性 `from .xxx import *` 导出全部工具 |
| `preprocess.py` | 9.1 KB | 推理前处理：resize / letterbox / 输入格式转换 |
| `postprocess.py` | 39.2 KB | 推理后处理：解码、阈值过滤、NMS、坐标映射（最大的一个） |
| `visualize.py` | 20.6 KB | 结果渲染：分类 / 检测 / 分割 / 关键点画框 |
| `inspect.py` | 8.4 KB | 调试检查：Tensor / 模型输出信息打印 |
| `file_io.py` | 5.7 KB | 文件与资源读写：图像加载、标签/词表读取 |
| `nn_math.py` | 2.1 KB | 数值工具：sigmoid / softmax / 归一化 |
| `README.md` | 1.8 KB | 该 vendored 库自带的说明 |

### 5. `src/to32/` —— 中位机（2026-09-21 迁入）

**源码**：

| 文件 | 行数 | 作用 |
|---|---|---|
| `main.py` | 294 | **入口**：argparse 参数 → 装配 link_pc / link_stm32 / mode_dispatcher / video 各线程并常驻；`--mode idle/-1/待命` 可直接进待命态 |
| `link_pc.py` | 144※ | 上位机链路：UDP :8080 收 `$CMD`/`$PID`/`$VID`/`PING`，:8081 发 `$TEL`；**2026-10-07 收包先判 11B 二进制 PID（命中走独立中继），并新增 `$TASKPID` ACK 出口** |
| `link_stm32.py` | 348※ | 下位机链路：串口（CH348 F 口 `ttyCH9344USB5`）发 `0x09` 控制帧、收 `0x0C` 48 字节遥测帧并解析；**`poll_suspended=True` 时兜底 0x0C 轮询被抑制（IDLE 待命期）；2026-10-07 `send()` 整帧互斥写锁 + 短写检查，兜底轮询走同一 `send`，新增 PID 帧常量** |
| `protocol.py` | 107※ | V2 协议组帧 / 校验的纯函数；`parse_cmd` 输出 `mode_explicit` 标记"帧里有没有第 7 段"；**2026-10-07 新增 `parse_binary_pid`（11B 二进制 PID 严格校验）** |
| `tel_builder.py` | 167※ | 把下位机遥测拼成上位机的 `$TEL` 帧 |
| `mode_base.py` | 92※ | `ModeBase` 抽象基类，定义 10 个回调（tick / 载荷处理 / 进入退出等） |
| `mode_idle.py` | 99 | ★（2026-10-04）**IDLE 待命模式**：上电默认进入，**不发 0x04/0x09/0x0C**，只回全 0 占位 `$TEL`；等上位机下发**带 mode 的** `$CMD` 才切到 ROV/AUV |
| `mode_rov.py` | 198※ | **ROV 手动模式**：四轴直通；空闲静默语义为四轴全 0 才判回中 |
| `mode_dispatcher.py` | 641※ | **调度核心**：模式切换与记忆、看门狗、急停锁存、抑制危险帧；**待命态只认显式带 `mode` 的 `$CMD`**（老格式 6 段帧不误激活），并在此进/出 IDLE 时置 `link_stm32.poll_suspended`。**2026-10-08 三态注册**：`TEST_MODE_ENABLED=True`→`TestMode` 接管 AUV 位（跑 `TEST_TABLE`）；`False`→正式 `AuvMode`（动态加载 `move_test/mode_auv.py`）；`test_config` 异常/导入失败→`AuvModeStub` 兜底宁停勿跑。另有 **PID 二进制中继（`_on_pid_binary`/`_on_task_pid`）**、`task_pids` 注入与 **`tel_shm_sink` 遥测落盘回调** |
| `task_pid_wire.py` | 81※ | **2026-10-07 新增**：`$TASKPID` 命名任务参数编解码与 ACK 回帧（仅 S100 处理） |
| `task_pid_controller.py` | 29※ | **2026-10-07 新增**：S100 真实 `GatePid` 对象 + 过门任务工厂；`$TASKPID gate` 更新即时生效 |
| `tel_shm_sink.py` | 81 | **2026-10-08 接回**：下位机遥测落盘 `/dev/shm/momo_telemetry.json`（写 raw 协议值：depth_raw=cm×100 等，与 depth_kalman 模型约定对齐）；`mode_dispatcher.on_stm32_telemetry` 每帧回调写一次，20Hz 节流、原子写、异常吞掉绝不拖垮主链路 |
| `verify_idle_mode.py` | **0（空文件）** | ⚠ 本 v2.5 镜像中为**空文件**（2026-10-04 版有 IDLE 专项自检 17/17，在 v2.2 工程/开发机里；要用从那边取） |
| `move_test/`（任务包） | 核心 6 模块 + task/ 脚本 + test_mode/ 278※ | ★（2026-10-06 v2.5 重写，2026-10-08 接回主链路，2026-10-09 全序列含穿门）**AUV 任务代码**：`task_config.py`（任务参数 + `STAGE_TABLE` **全序列**：Task1→撞球→Task2→穿门 `DOOR_TABLE`→SearchBall→捡球→Task4→Return；含卡尔曼托管开关 `AUV_KALMAN_AUTOSTART=True`/`AUV_VISKF_AUTOSTART=False`、`AUV_POOL_DEPTH_CM=106` 实测值）/ `obs.py`（`VisionIF`+`DepthIF`，`CANON` 别名归一化）/ `mission.py`（Stage 基类 + Mission 编排器：**每阶段 `_set_stage` 发布 `momo_stage.json` 驱动视觉热切换**；STOP 哨兵；支持 `task_pids` 注入）/ `kalman_launcher.py`（**深度自动托管，2026-10-08 恢复**）/ `mode_auv.py`（AUV 模式壳）+ `Task.md` + `task/`（`t_task1/t_task2/t_search_ball/t_task4/t_return/t_hit_ball/t_hit_ball_v2/t_pick_ring` + **穿门 `task/task_door/`（`t_door.py` `DoorTask`，`DOOR_TABLE`）**）+ `test_mode/`（`test_config.py` + `test_runner.py`，**`TEST_MODE_ENABLED=True` → 切 AUV 即跑 `TEST_TABLE`=Task2+Return**）。⚠ 当前 AUV 位由 TestMode 接管；旧 task_pass_door（t_pass_gate.py）2026-10-09 已整体清理改由 task_door 承接。v2.2 旧 7 文件全套在 `D:\RC\S100\综合\GrandRDKv2.2` 可查 |
| `video.py` | 303※ | ⚠ **个人测试代码，非交付链路** —— 中位机自带图像回传（自己开相机直编 MJPEG，**不经过 YOLO ⇒ 画面无检测框**）。**2026-09-21 已停用**（`VIDEO_PATHS={}`、`VIDEO_ENABLED_AT_START=False`），`run.sh` 固定 `--no-video --video-port 0`。**上位机看到的带框画面来自 `web_server.py`，与它无关。** 保留仅供个人调试 |
| `port_probe.py` | 74 | 串口探测（CH348 A~H 通道确认；`main.py --list` 是另一条独立实现，不依赖本文件） |
| `selftest_modes.py` | 537 | 模式层离线自检（板端保留；开发机镜像归口 `hwless_tests/legacy_to32/`）。口径：**AUV 期间 0x09 必须不增加**（运动逻辑已摘除）。板端跑法：`PYTHONPATH=/userdata/GrandRDK/config:/userdata/GrandRDK/src:/userdata/GrandRDK/src/utils:/userdata/GrandRDK/src/to32 python3 src/to32/selftest_modes.py` |
| `make_test_frame.py` / `test_v2_frames.py` | 197 / 196 | V2 测试帧构造 / 帧解析测试（板端保留；`test_v2_frames` 需先 `make_test_frame.py --bin /tmp/v2_telemetry_5frames.bin` 造数据，19/19 PASS） |

> **测试脚本归属（2026-10-04 澄清）**：开发机镜像中，`selftest_modes` / `make_test_frame` /
> `test_v2_frames` 已归口 `hwless_tests/legacy_to32/`；**板端（运行环境）没有 `hwless_tests/`，
> 这几个脚本仍留在 `src/to32/`** 供板上自检。
>
> ※ 行数 = 2026-10-07 PID v3.8 补丁后**本地工作副本**实测；未标 ※ 的行仍为 2026-10-06 v2.5 镜像口径。板端部署后以实际为准。

**文档**（2026-09-22 由 `docs/` 合并迁入）：

| 文件 | 行数 | 作用 |
|---|---|---|
| `README.md` | 584+ | **中位机目录导航**：文件速查、线程模型、协议速查、安全机制、排障表。**2026-10-09 已同步三态注册、`tel_shm_sink`、`task_door` 与 stage_model 联动** |
| `README_中位机.md` | 355 | 中位机完整设计说明（含完整协议字段表） |
| `ROV_指令与V2协议对应关系.md` | 237 | 上位机指令 ↔ 下位机 V2 功能码对照 |
| `上位机通讯协议.md` | 130 | 上位机侧 UDP 报文格式 |

### 6. `web/` `models/` `libs/` `logs/`

| 目录 | 文件 | 说明 |
|---|---|---|
| `web/` | `index.html`（594 行 / 16 KB） | **在用的监控页面**，Win8.1 磁贴风，Nginx 实际分发 |
| | `index_v2.html`（803 行 / 30 KB） | 新版页面，**尚未接入**（Nginx 配 `index index.html`，代码里无人引用，`ENABLE_WEB_NEW` 也没人读） |
| `models/` | `door_3_640x640.hbm`（6.7 MB）+ `test_nashe_640x640_nv12.hbm`（6.6 MB）+ `bottom.hbm` | **三个 BPU 模型**（stage_model 按阶段选用：门/球/环），占工程体积绝大部分；传代码不想带模型时 `tar --exclude="models"` |
| `libs/` | `libmjpg_hw.so`（22 KB） | JPU 硬件解码动态库，`hw_camera.py` 用 ctypes 调它 |
| `logs/` | `front.log` / `bottom.log` | 两路检测进程日志（含 fps、识别结果） |
| | `bottom_retry.log` / `web_server_retry.log` / `show_cam_ondemand_test.log` | 历史排查期遗留日志（retry 调试、按需采集测试），当前链路不写 |
| | `flow_speed.log` | 光流日志（**历史遗留**，2026-10-04 起不再更新） |
| | `web_server.log` | Web 服务日志 |
| | `show_cam.log` | CAM3 按需采集日志 |
| | `altimeter.log` | 高度计各通道读取与在线状态 |
| | `to32_main.log` | 中位机主日志（$CMD 计数、下发/回帧统计、**模式切换计数**） |
| | `to32_mode_state.json` | 中位机模式记忆（原子写；当前 `MODE_PERSIST=False` 不写） |
| | `legacy_bridge.log` | **历史遗留**：旧链路已于 2026-09-21 删除，仅剩日志 |

---

## 进程与端口

| 进程 | 绑核 / nice | 端口 | 说明 |
|---|---|---|---|
| `front.py` | taskset CPU 0-2 | — | 写 `momo_frame_front.bin` 等 |
| `bottom.py` | taskset CPU 3-5 | — | 写 `momo_frame_bottom.bin` 等（原另写的光流 NV12 帧已停写） |
| `read_altimeter.py` | nice 5 | **UDP 推上位机 :8082** | `$ALT,<通道>,<mm>,<状态>#` |
| ~~`flow_speed.py`~~ | — | ~~:8000~~ | ⛔ **已停用（2026-10-04），不启动** |
| `show_cam.py` | nice 5 | :8084 | 空闲不占相机，上位机取流才开 |
| `web_server.py` | nice 10 | :5000 | `/cam1` `/cam2` `/api/status` `/api/detections` `/ws/status` `/healthz`。★ **上位机图像回传的唯一来源（带检测框）**。⚠ 绑定具体网卡 IP（192.168.127.10），板端本地探测别用 127.0.0.1 |
| `src/to32/main.py` | nice 5 | UDP 8080/8081 + 串口 F 口 | 中位机；**已停用自带图像回传**，不碰相机；上电停 IDLE；**2026-10-07 起支持 11B 二进制 PID 中继与 `$TASKPID`**；**2026-10-08 起遥测落盘 `momo_telemetry.json`**；AUV 位三态（TestMode/AuvMode/AuvModeStub，见《模式与状态》）；AUV 运行时由 `mission` 发布 stage 驱动视觉换模型 |
| Nginx | — | :80 | 静态页 + 反代到 :5000（供浏览器，非上位机） |

> `legacy_bridge.py`（旧图像链路 `:9000/:9001`）**已于 2026-09-21 删除**，`--with-legacy` 不再存在，只剩 `logs/legacy_bridge.log` 的历史日志。

---

## 数据流

```
cam1（USB 物理口 3-2）──────> front.py（StageDetector 按阶段热切换模型）──> /dev/shm/momo_frame_front.bin  ─┐
                          (基线 door；撞球/捡球阶段切 test_nashe/bottom)  momo_det_front.json        │
                                                  momo_stats_front.json      ├─> web_server.py :5000 ──> 上位机（直连 :5000，带检测框）
cam2（USB 物理口 1-2）──────> bottom.py（同上 StageDetector）────> /dev/shm/momo_frame_bottom.bin ─┘                    └─> Nginx :80（浏览器用）
                                                  momo_flow_bottom.bin ──✕ 光流已停用（2026-10-04）：NV12 帧不再写出，无消费者
CH348 A–E    超声波高度计 ───> read_altimeter.py ──> UDP $ALT ─────────────> 上位机 :8082
                                        └──────────> /dev/shm/momo_alt.json
                                                          │
                                                          v
                                       src/kalman/depth_kalman/（深度卡尔曼 —— ✅ 2026-10-08 起由 AUV 模式自动拉起）
                                                          │  D / v_z / clearance / sigma
                                                          v
                                                 /dev/shm/momo_depth.json
                                                          │
                                                          v
                              src/to32/move_test/obs.py DepthIF ──> mission.py（AUV 定深 / 坐底判据，已接回）

momo_det_front.json ──> src/kalman/camera_kalman/（图像卡尔曼 viskf —— 托管方已恢复但默认不拉起
                                                          │   （AUV_VISKF_AUTOSTART=False）；穿门主闭环在 front.py 内）
                                                          │  e_x / de_x / sig_x / el / s_n / trust
                                                          v
                                                 /dev/shm/momo_viskf.json
                                                          │
                                                          v
                              src/to32/move_test/obs.py VisionIF ──> mission.py（撞球 / 穿门视觉判据，已接回）
                                                          （直接读 momo_det_*.json，不过卡尔曼）

★ YOLO 模型动态切换链（2026-10-09）：
  mission._set_stage(阶段) ──publish_stage()──> /dev/shm/momo_stage.json ──> front/bottom 的
  StageDetector.detect() 每帧读 → model_config(stage, base_cfg) → cfg 变化即重载模型（日志
  [stage_model] <stage> -> <model_path>）；is_current() 丢弃推理期间已切走的过时结果。
```

> 中位机侧遥测落盘（2026-10-08 接回）：`to32/tel_shm_sink.py` 在 `on_stm32_telemetry` 每帧写
> `/dev/shm/momo_telemetry.json`（20Hz 节流、原子写）——`depth_kalman` 等观测工程的遥测输入源。

> **图像回传两条出口（别搞混）**：上游都是 front/bottom 进程写进 `/dev/shm` 的
> **已画检测框 JPEG**，由 `web_server.py` 统一读出来发 MJPEG：
> - **上位机**（`pc_main2.py` 的 `VideoThread`）→ **直连 `http://192.168.127.10:5000/cam1`**，
>   **不经 Nginx**。这是交付给操作员的带框画面。
> - **Nginx `:80/cam1`** 反代到 `:5000`，**只服务浏览器**。
>
> `src/to32/video.py` 是**个人测试代码**（自己 `cv2.VideoCapture` 开相机直编 MJPEG，**不过 YOLO ⇒ 无检测框**），
> 与交付链路无关，`run.sh` 固定 `--no-video` 关掉它。

> 高度计现在**同时**推 UDP 与落共享内存（默认就落，`--no-shm` 才关）。
> 卡尔曼是 AUV 的深度源（**2026-10-08 起进 AUV 自动拉起深度，退出自动停**）——遥测 **2026-10-04 实测已通**
> （0x0C 150 帧、`yaw` 有真值），但**上位机深度显示 0.0cm 仍降级**。
> 🔴 `config/depth_config.py` 里的 `H_M=1.3` 仍是**占位值**，**必须实测**（错了融合深度
> 整体平移同量且不报警，仿真：差 0.3 m → 稳态误差 0.3008 m，σ_D 看不出来）。
> **水面标定法**（比量池底可靠）：机器人放水面、深度计归零，读锚路 B 的净空即 H_M
> （公式口径 `H_M = 水深 − dz_锚路`，当前 dz_B=0 ⇒ H_M=水深）；2026-10-06 现场实测
> 水深约 1.2 m，重标预计 ≈1.2。⚠ 这个文件 2026-10-01 已从卡尔曼包里搬到 `config/`。
> ★ 深度卡尔曼 2026-10-06 三项更新：① B/C 安装参数落档（B 比 C 高 20 mm ⇒
> `ALT_MOUNT['C'].dz=0.02`，修掉 C 恒带 2 cm 系统偏差进融合的旧问题）；② **失效硬拒**
> （高度计失效会读 -3/超程大数，净空出界 [0.02, 2.5] m 或 |pitch|/|roll|>30° 在门限**之前**
> 直接拒、不喂 P 放大看门狗，`fusion.py` 新增 `rej_range`/`rej_tilt` 计数）；③ **观测方程
> 耦合配对修正**（前向偏移 y 耦合俯仰 θ、右向偏移 x 耦合横滚 φ，旧式配反；pitch/roll 锁 0
> 期间无影响，未来姿态解锁前必须用新式）。
>
> ⚠ 图像卡尔曼的两条口径（v2.2 实测经验；其消费者 `viskf_if.py` 已删，将来重加时必须沿用）：
>   ① `e_x` 归一化口径恒定 `(cx−x0−dx0)/门框宽`，贴边也不许换成按画面宽（会造出 0.209 的假跳变 =
>   正常帧间变化 50 倍）；② 判"能不能用"看 **`trust`** 不看 `gate_visible`（门一丢滤波器还会自由外推
>   0.5~2s，实测 `e_x` 能飘到 2.38）。
> ⚠ viskf 的 `el`（仰角）依赖姿态：遥测中断时会恒 `att_degraded` 而不可信
> （2026-10-04 遥测已恢复，该降级暂不会触发）。2026-10-09 起穿门主闭环已移到 `front.py`
> （`PassGate` 阶段走门跟踪管线，直接消费 `momo_det_front.json`），viskf 暂未接入新链路。

另一条独立链路（控制/遥测，代码在本仓库的 `src/to32/`）：

```
上位机 --UDP 8080 $CMD--> src/to32/main.py --UART V2 (/dev/ttyCH9344USB5, CH348 F 口)--> 下位机 STM32
上位机 <--UDP 8081 $TEL-- src/to32/main.py <-------- 0x0C 48 字节遥测帧 --------------
```

⚠ 模式语义见《模式与状态》：**每帧带 mode 的 `$CMD` 都即时切模式，无来源锁定**。

---

## 常用配置（`config/quick_config.py` + `config/stage_model.py`）

| 键 | 当前值 | 含义 |
|---|---|---|
| `YOLO_MODEL_FRONT` / `YOLO_MODEL_BOTTOM` | `door_3_640x640.hbm` / `door_3_640x640.hbm` | **前后视独立** BPU 模型文件名（2026-10-08 回退单模型；旧单键 `YOLO_MODEL` 保留注释） |
| `FRONT_DEVICE` | `device_for('cam1')` | 前视相机节点（按 USB 物理口绑定，见 `config/camera_ports.py`） |
| `BOTTOM_DEVICE` | `device_for('cam2')` | 下视相机节点（按 USB 物理口绑定） |
| `FRONT_TARGETS` / `BOTTOM_TARGETS` | `['door']` / `['door']` | 基线检测目标（2026-10-08 回退；撞球/捡球时由 stage_model 切模型自动扩目标） |
| `FRONT_CLASS_NAMES` / `BOTTOM_CLASS_NAMES` | `['door']` / `['door']` | 模型类别顺序，改模型必须同步改 |
| `stage_model.MODELS` | `ball`=test_nashe / `gate`=door_3 / `pick`=bottom.hbm | ★（2026-10-09）三套按阶段切换的模型配置 |
| `stage_model.STAGE_MODELS` | Task1/SearchBall/HitBall→`ball`；Task2/PassGate→`gate`；PickBall→`pick` | 阶段名 → 模型配置映射；未列出（IDLE/DONE）用基线 |
| `SCORE_THRES` / `NMS_THRES` | `0.6` / `0.45` | 置信度 / NMS 阈值 |
| `ENABLE_FRONT` / `ENABLE_BOTTOM` | `True` / `True` | 两路相机总开关 |
| `ENABLE_FRONT_YOLO` / `ENABLE_BOTTOM_YOLO` | `True` / `True` | 推理开关 |
| `FRONT_FPS` / `BOTTOM_FPS` | `250` / `250` | 相机请求帧率（实测低于此值） |
| `FRONT_BPU_CORES` / `BOTTOM_BPU_CORES` | `[0,1,2,3]` | 两路各自可用的 BPU 核 |
| `N_WORKERS` / `CAMERA_QUEUE_SIZE` | `3` / `12` | 进程内 worker 数与队列长度 |
| `SHOW` | `False` | 板端无桌面，保持关闭 |

> 光流参数在 `config/main_config.py` 的 `FLOW_*` 段（含角点、抗噪、标定）——
> **2026-10-04 起整段停用**（`FLOW_ENABLE=False`、`ENABLE_FLOW_SHARE_BOTTOM=False`），参数保留以便恢复。

---

## 相机节点对照（**按 USB 物理口绑定，不再写死节点**）

设备节点号由内核枚举顺序决定，上电/插拔一变就漂移——本板 2026-09-23 实测曾出现"节点与角色互换"的错位。**改为按 USB 物理口绑定**，由 `config/camera_ports.py` 解析：

| 通道 | 角色 | USB 物理口 | 进程 / 出口 |
|---|---|---|---|
| `cam1` | **前视** | `3-2` | `front.py` → `/cam1`（HTTP :5000） |
| `cam2` | **下视** | `1-2` | `bottom.py` → `/cam2`（HTTP :5000） |
| `cam3` | 第三路（内窥镜） | `1-1` | `show_cam.py` → `:8084/stream` |

`config/quick_config.py` 用 `FRONT_DEVICE = device_for('cam1')` / `BOTTOM_DEVICE = device_for('cam2')` 取节点，不再硬编码节点号。

> 换 USB 口（或经 Hub，口名变成 `1-1.2`）时只改 `config/camera_ports.py` 一行（`CAM_PORTS` 字典）。查节点号当前叫什么：`v4l2-ctl --list-devices`；查占用：`fuser <设备节点>`。

## run.sh 常用参数

```bash
./run.sh                      # 真实相机，全部启动（光流除外——已停用）
./run.sh --door-sim            # task_door_sim/run.py -> front.py 写处理后的共享帧 -> :5000/cam1（日志 logs/front.log）
./run.sh --door-sim --to32-args "--mode rov"  # ROV 启动，同时立即运行门视觉测试
./run.sh --door-sim --door-sim-args "--contrast 1.3 --sharpen 0.8"  # 门视觉测试参数透传
./run.sh --frames 100         # 每路只跑 100 帧（透传给检测进程）
./run.sh --no-web             # 只跑检测，不起 Web / Nginx
./run.sh --no-flow            # （兼容保留）光流已整体停用，此参数无实际效果
./run.sh --no-showcam         # 不启动第三路相机
./run.sh --no-altimeter       # 不读高度计
./run.sh --alt-channels A,B   # 高度计只读指定口（默认 A,B,C,D,E）
./run.sh --alt-no-udp         # 高度计不推 UDP，只打终端日志
./run.sh --alt-args "--reg 0x0100 --interval 0.3"   # 高度计参数透传
./run.sh --no-to32            # 不拉起中位机
./run.sh --to32-dir /path     # 中位机目录（默认 <项目根>/src/to32）
./run.sh --to32-estop         # 打开中位机的自动急停+锁存（默认关闭）
./run.sh --to32-args "--mode rov"   # 中位机参数透传（--mode idle/rov/1）
./run.sh --no-nginx           # 不动 Nginx
./run.sh --setup-only         # 只做依赖检查与 Nginx 配置
```

已移除的参数：`--virtual`（2026-09-21 起不再支持虚拟帧）、`--with-legacy`（旧链路已删）、`--no-range`（测距已移除，2026-09-23）。

看日志：`./logs.sh front|bottom|web|alt|to32|showcam|all`

---

## 实测参考

**2026-10-04**：

| 项 | 实测 |
|---|---|
| `front.py` / `bottom.py` | 正常出帧、检测正常，`/cam1` `/cam2` 供流正常 |
| 光流 | 未启动（已停用），`:8000` 不监听 |
| 高度计 | 五路仍 no-reply → 自动降频 2s 探测 |
| 中位机 | `$CMD` 正常收；**遥测已通：0x0C 150 帧、`yaw=-144.2` 有真值**（此前长期 0 帧，疑换过设备/固件）；`深度 0.0cm` 仍降级 |
| 模式 | IDLE 待命正常；待命期 `0x04/0x09/0x0C` 均为 0；双源注入实测 12s 模式切换 464 次（见排障） |

**2026-10-06**：现场实测水深约 **1.2 m**（深度卡尔曼 `H_M` 重标依据，见《数据流》与《变更记录》）；B/C 高度计实测布置：前后相距 ~15cm、B 比 C 高 20mm、左右间距可忽略。

历史参考（2026-09-22）：front 157~164 / bottom 135~165 / flow 31~43 fps（光流数据为停用前实测）。

---

## 排障速查

| 现象 | 常见原因 | 处理 |
|---|---|---|
| `真实相机打开失败` | 相机没插 / 节点被别的进程占着 | `v4l2-ctl --list-devices` 核对；改 `quick_config.py` 的节点 |
| `/cam1` `/cam2` 黑屏 | 写端没起来，`/dev/shm` 无数据 | `./status.sh` 看共享内存；先看 `front.log` / `bottom.log` |
| 上位机连不上 :5000 | 监听地址或防火墙；或 run.sh 的 `read_cfg` 被 stdout 污染跳过了 web_server | 用 `ss -lntp \| grep 5000` 确认在听；板端本地探测**别用 127.0.0.1**（uvicorn 绑网卡 IP），refused 多为上游长连接占满 backlog，非故障 |
| `stop.sh` 后进程还在 | 上位机占着 MJPEG 流，uvicorn 优雅关闭会挂住 | stop.sh 带宽限期后 SIGKILL 兜底，属已知行为 |
| **上位机切 AUV 机器人不动** | 先看 `test_mode/test_config.py`：`TEST_MODE_ENABLED=True`（默认）→ 测试模式跑 `TEST_TABLE`（当前=Task2+Return，**不是正式全序列**）；`False` → 正式 `AuvMode` 按 `STAGE_TABLE` 全序列跑；`test_config` 异常 → `AuvModeStub` 兜底（只切模式不动，**预期行为**，journal 有 print） | 想跑正式任务：`TEST_MODE_ENABLED=False` 后重启中位机；`logs/to32_main.log` 看 journal 提示 |
| **YOLO 模型没有按阶段切换** | `momo_stage.json` 没被写（任务没在跑，或 stage 未映射）→ 一直用基线模型 | 看 `front.log`/`bottom.log` 的 `[stage_model] <stage> -> <model_path>` 行；确认任务已进入 SearchBall/PassGate/PickBall 等阶段 |
| **日志疯狂刷"进入 AUV/退出 AUV/进入 ROV"** | 两个来源同时发带 mode 的 `$CMD`（如上位机 + 调试注入）互抢模式 | 只保留一个 `$CMD` 来源；`to32_main.log` 的"模式切换"计数可确认规模 |
| 光流相关问题（:8000 无画面、`moving=0%`、m/s 不对） | **光流已整体停用（2026-10-04）** | 恢复：`run.sh` 的 `FLOW_DISABLED=0` + `main_config.py` 两个开关 True；启动顺序仍须**先 bottom 后 flow_speed** |
| `[!!] [x] 读取失败(no-reply)` | 该串口没接高度计 | 自动降 2s 低频探测，接上自动恢复，无需处理 |
| 上位机收不到高度数据 | 没开 UDP 推送或监听端口错 | 默认推 `:8082`；`--alt-no-udp` 会关掉推送 |
| 整栈突然全停 | **run.sh 在检测进程退出时会收尾杀掉全部子进程** | 先看 `logs/front.log` / `logs/bottom.log` 谁先挂（多为相机掉线），修根因再 `./run.sh` |
| `[jpu] too many bad jpeg frames` | 相机节点被别的进程占着 / 节点不对 | `fuser <设备节点>` 查占用；按 USB 物理口核对 `camera_ports.py` |

---

## 已知问题

1. ~~**相机节点冲突**~~（**2026-09-23 已改为物理口绑定**）：详见 `config/camera_ports.py`。教训：**节点号会随枚举漂移，必须按物理口绑定**。
2. ~~**光流标定是占位值**~~（**2026-10-04 光流已停用，此条失效**；恢复前仍需标定 `FLOW_CALIB_*`）。
3. **Web 服务停止会挂**：上位机占流时 uvicorn 不退出，靠 stop.sh 强杀，属设计内兜底。
4. **高度计持续离线**：2026-10-01 复测 A–E 五路全部 no-reply（接线已确认），下一步查 Modbus 参数（`--addr 0x01`/`--reg 0x0101`/115200）与供电；与 STM32 共用同一颗 CH348。深度卡尔曼侧已按 2026-10-06 实机行为加**失效硬拒**（失效读数 -3/超程大数直接拒，见《数据流》），高度计来数即可安全接入。
5. **`web/index_v2.html` 未接入**：Nginx `index index.html`，代码里也没有引用它。想启用需改 nginx 配置或自行接入。
6. **遥测状态曾反复**：2026-09 期间 0x0C 恒 0 帧；**2026-10-04 实测已通**（150 帧、yaw 有真值），疑换过设备/固件。**上位机深度 0.0cm 仍降级**。改相关代码前先实测确认。
7. **AUV 任务已接回主链路，但当前默认走测试模式（2026-10-08/09 口径）**：`mode_dispatcher` 已是三态注册——`TEST_MODE_ENABLED=True`（当前默认）时 AUV 位由 `TestMode` 接管，跑 `TEST_TABLE`=Task2+Return（**不是正式全序列**）；置 `False` 即注册正式 `AuvMode` 跑 `STAGE_TABLE` 全序列（Task1→撞球→Task2→穿门 `DOOR_TABLE`→SearchBall→捡球→Task4→Return），并自动拉起深度卡尔曼（`AUV_KALMAN_AUTOSTART=True`）。无 `$AUV` 上报（`auv_report.py` 已删）；viskf 默认不拉起（`AUV_VISKF_AUTOSTART=False`）。任务侧待标定参数（`AUV_SPEED_MPS`、`AUV_YAW_RATE_DPS`、`AUV_POOL_DEPTH_CM=106` 已按 2026-10-08 实测更新）在 `move_test/task_config.py`；旧 AUV 参数段在 `config/auv_config.py`；深度卡尔曼 `H_M=1.3` 仍须实测。
8. **模式切换无来源锁定**：任意带 mode 的 `$CMD` 都即时生效、无防抖——双源并发会高频抖动（实测 12s 切 464 次）。运行期确保只有一个 `$CMD` 来源。
9. **udev 规则与 `camera_ports.py` 的 cam1/cam2 物理口记载相反（2026-10-06 发现）**：`config/udev/99-rover-cameras.rules` 写 cam1=口1-2 / cam2=口3-2，`camera_ports.py` 写 cam1=3-2 / cam2=1-2——两颗 Realtek 型号/序列号相同，只能靠物理口区分，二者必有一个过时。**启用 udev 软链前必须现场实测核对**（`v4l2-ctl --list-devices` + 试开确认画面），并同步改另一个文件，否则前视/下视会错位。

---

## 相关工程

- **中位机 To32**：2026-09-21 迁入本工程（`src/to32/` + `config/to32_config.py` + 文档），`run.sh` 默认拉起它，可用 `--to32-dir` 指外部目录。**子目录内自有 README：`src/to32/README.md`**。
- **`/userdata/To32`（早期独立副本，2026-10-04 更新）**：自带 `video.py` HTTP MJPEG，可自提供 `:5000 /cam1 /cam2`（上位机取流地址不变，零改动）；已修好可独立跑（`start.sh`/`stop.sh`，`setsid` 启动）。**与主工程的 `src/to32` 互斥**（都占 `/dev/video*` 与 `:5000`），别同时跑。
- **`/userdata/momo_pwmnet*`**：光流测速的前身，2026-09-17 迁入 `src/flow_speed.py`，原目录仅作备份。
- 其他同级目录：`/userdata/vp`、`/userdata/RC`、`/userdata/USART`。

---

## 变更记录（从代码注释整理）

| 日期 | 变更 |
|---|---|
| 2026-09-17 | `flow_speed.py` 自 momo_pwmnet 迁入；删除 legacy 假遥测 `$TEL`（改由中位机提供真数据）；Web 监听改 `0.0.0.0`；关闭旧图像链路 `:9000/:9001` |
| 2026-09-18 | `show_cam.py` 迁入本目录，纳入 run.sh / stop.sh / status.sh 统一托管 |
| 2026-09-19 | show_cam 改为**按需采集**：HTTP 常驻但不开相机，取流才开、空闲 5s 自动释放 |
| 2026-09-21 | **中位机 To32 迁入本工程**（`src/to32/` + `config/to32_config.py` + 文档），三脚本路径同步更新；下视改 cam2（物理口 1-2）、CAM3 保留 cam3（物理口 1-1 内窥镜），解除节点冲突 |
| 2026-09-21 | **移除 `--virtual` / `make_frame()` 合成帧**（front/bottom 只读真实相机） |
| 2026-09-21 | 删除 `legacy_bridge.py` 与 `--with-legacy`；停用中位机自带图像回传（cam1/cam2），避免抢相机与 `:5000` |
| 2026-09-21 | 中位机 `mode_rov` 空闲静默语义修订：**四轴全 0 才判回中**，持续推杆必须每帧下发 `0x09` |
| 2026-09-22 | 中位机文档由 `docs/` 合并迁入 `src/to32/`；根 README 新增《各目录逐文件作用》章节 |
| 2026-09-23 | **移除前视单目测距**：删除 `src/range.py` 及全部 `RANGE_*` 配置与 `--no-range` 参数；前视目标**保留 `door`** |
| 2026-09-23 | **相机节点改按 USB 物理口绑定**：真值在 `config/camera_ports.py` |
| 2026-09-22 | 前视目标改为 `door`、下视改为 `red-ball` |
| 2026-09-25 | **AUV 自主任务脚本部署**：`config/auv_config.py`（唯一调参入口）+ `src/to32/{vision_if,depth_if,mission,mode_auv,auv_report}.py`；`to32_config.py` 的 AUV 段换成 `from auv_config import *` |
| 2026-09-25 | **深度卡尔曼迁入本工程**，按"遥测 0 帧"调成无遥测档；`read_altimeter.py` 新增 `momo_alt.json` 落盘 |
| 2026-10-01 | **卡尔曼改为 AUV 模式托管**：新增 `depth_launcher.py`（后通用化为 `kalman_launcher.py`），起停挂在 `mode_auv.on_enter/on_exit`；**删除 `start_auv.sh` / `stop_auv.sh`** |
| 2026-10-01 | **两路卡尔曼统一归位 `src/kalman/`**；图像卡尔曼 viskf 首次上板；新增 `src/to32/viskf_if.py`，过门闭环（PASS_GATE）接上滤波量 |
| 2026-10-01 | ★ **两个卡尔曼按本工程分类拆开**（定稿）：配置进 `config/`、源码平铺进 `src/kalman/<名字>/`（不摊 `src/` 顶层——`main.py` 会和 `src/to32/main.py` 撞名）；PYTHONPATH 必须两段 `<宿主>/config + <工程根>` |
| 2026-10-01 | ★ **测试代码全工程归口 `hwless_tests/`**（开发机）：`src/to32/{selftest_modes,make_test_frame,test_v2_frames}.py` → `hwless_tests/legacy_to32/`；新增 `run_legacy.py` 一键回归（4/4）。⚠ **板端没有 `hwless_tests/`，这几个脚本在板上仍留 `src/to32/`** |
| 2026-10-01 | ★ 11 处生产源码的 `GRDK_*` 测试接缝注释规范化（取值表达式一字未改） |
| 2026-10-01 | ⚠ 发现 `selftest_modes.py` 由 68/68 退为 66/68（AUV 主动发 0x09 与旧断言冲突）——该问题随后被 2026-10-04 的 AUV 摘除一并解决 |
| 2026-10-04 | ★ **中位机上电改停 IDLE 待命态**（`./run.sh` 不再自动进 ROV/AUV）：新增 `src/to32/mode_idle.py`；`to32_config.py` 新增 `MODE_IDLE=-1`，`START_MODE`/`DEFAULT_MODE` 改 `MODE_IDLE`。待命态**不发 `0x04`/`0x09`/`0x0C`**，只回 41 字段全 0 占位 `$TEL`；只认显式带 `mode` 的 `$CMD`（`parse_cmd` 新增 `mode_explicit`）；`link_stm32.poll_suspended` 抑制兜底 0x0C。新增 `verify_idle_mode.py`（17 用例）。备份 `/userdata/_bak_idle_20260925_003705` |
| 2026-10-04 | ★ **光流整体停用（计算 + 共享），图像回传不受影响**：`run.sh` 加 `FLOW_DISABLED=1` 短路启动段（`--no-flow` 兼容保留）；`main_config.py` 的 `ENABLE_FLOW_SHARE_BOTTOM`/`FLOW_ENABLE` 改 `False`；`bottom.py` 加 `FLOW_SHARE_DISABLED=True` 硬开关停写 `momo_flow_bottom.bin`；`src/flow_speed.py` 保留未删只加横幅。**关键口径：图像回传（JPEG 帧）与光流链（NV12 帧）是两条独立共享内存，互不影响。** 恢复 = `FLOW_DISABLED=0` + 两个开关 True。备份 `/userdata/_bak_flowoff_20261004_165530` |
| 2026-10-04 | ★ **AUV 运动逻辑整体移入 `src/to32/move_test/`（主链路解耦）**：7 文件（`mission`/`mode_auv`/`vision_if`/`depth_if`/`viskf_if`/`auv_report`/`kalman_launcher`）搬家，`mode_dispatcher.py` 删 `from mode_auv import AuvMode`，改注册内联 **`AuvModeStub`**（`on_enter` 只发 `0x04=0x05`、`tick` 只泵 `0x0C`，**绝不发 0x09**）⇒ AUV 期间无自主运动、卡尔曼不被拉起、无 `$AUV` 上报。`selftest_modes.py` 更新断言后 **77/77**；`test_v2_frames.py` 19/19。接回办法见 `move_test/README_move_test.md`。备份 `/userdata/_bak_auvmove_20260925_003115` |
| 2026-10-04 | 遥测实测恢复：0x0C 150 帧、`yaw` 有真值（此前长期 0 帧）；`深度 0.0cm` 仍降级 |
| 2026-10-04 | **本 README 全面更新**：新增《当前功能状态一览》《模式与状态：会发生什么》两节；同步光流停用 / AUV 摘除 / IDLE 待命三项变更；澄清测试脚本归属（板端 `src/to32/` 仍保留 4 个测试脚本）；补双源模式抖动与 `read_cfg` 污染两条已知问题 |
| 2026-10-04 | `run.sh` 的 `read_cfg` stdout 污染**修复落码**（import 期 stdout 改道 stderr + `tail -1` 兜底；此前 README 误记为"未修"，本次更正） |
| 2026-09-26 | `quick_config.py` 前视目标加 `red-ball`（撞球阶段要靠前视找球；此改动此前未记入本 README，2026-10-06 补档） |
| 2026-10-06 | ★ **`src/to32/move_test/` 全新重写为 v2.5 骨架**：v2.2 的 7 文件（mission/mode_auv/vision_if/depth_if/viskf_if/auv_report/kalman_launcher）全部删除，换成 4 模块 `task_config.py`(36)/`obs.py`(211)/`mission.py`(142)/`mode_auv.py`(121) + `Task.md`(2026 巡游阶段划分，含定深口径公式 `depth_cm=实测水深−目标高度−机体高度20cm`) + `README.md`。`STAGE_TABLE=[]` 空表 = 开机即 DONE；主链路仍注册 `AuvModeStub`，接回 = dispatcher 两行改动（见 `move_test/README.md`）。能力移除待需要时重加：卡尔曼自动托管、`$AUV` 5Hz 回传、viskf 过门滤波接口。新增空目录 `task/`、`test_mode/` |
| 2026-10-06 | 新增 `config/udev/99-rover-cameras.rules`（udev 物理口绑定）；⚠ 其 cam1/cam2 物理口与 `camera_ports.py` 相反，列入已知问题待现场核对。`verify_idle_mode.py` 在本镜像为空文件。各文件行数按 v2.5 镜像实测校准（front 511 / bottom 543 / read_altimeter 347 / mode_dispatcher 537 / selftest 537 / kalman 2283 等）；本 README 依镜像实况全面更新 |
| 2026-10-06（晚） | ★ **深度卡尔曼 2026-10-06 实测落档**：`config/depth_config.py`（210→231 行）——① B/C 安装参数落档（前后相距 ~15cm、B 比 C 高 20mm ⇒ `ALT_MOUNT['C'].dz=0.02`，修掉 C 恒带 2cm 系统偏差进融合的旧问题；y 全 0）；② 新增**失效硬拒**门限 `ALT_MIN/MAX_VALID_M=0.02/2.5`、`ALT_MAX_TILT_DEG=30`（实机高度计失效会读 -3/超程大数）；③ **观测方程耦合配对修正**（前向偏移 y 耦合俯仰 θ、右向偏移 x 耦合横滚 φ，旧式配反；pitch/roll 锁 0 期间无影响，姿态解锁前必须用新式）；④ `H_M` 水面标定法与公式口径 `H_M = 水深 − dz_锚路`（2026-10-06 实测水深约 1.2m，1.3 仍占位、重标预计 ≈1.2）；⑤ `R_ALT_BASE` 0.008→0.010（无遥测档放宽）。配套代码：`fusion.py`(342) 在自适应门限**之前**硬拒且不喂 P 放大看门狗、snapshot 新增 `rej_range`/`rej_tilt` 计数；`model.py`(302) `obs_alt`/`clearance_of` 同步修正耦合配对。⚠ 已知债务：深度计装机体**上部**，当前 D 语义 = B 探头深度（≈舱底上方 2cm），深度计回灌后 O1/O2 恒差 ~0.18m 由 b_d 吸收；完整迁移（dz_B=0.18 等）须连下游「池深当 D 目标」一起改 |
| 2026-10-07 | ★ **PID 中继 v3.8 落地（本地工作副本，已提交）**：`protocol.py` 新增 `parse_binary_pid`；`link_pc.py` 先判二进制再解码文本（并修复 `_stop` 覆盖）；`link_stm32.py` `send()` 整帧互斥写锁 + 短写检查；`mode_dispatcher.py` 独立 PID 中继（不切模式/不解急停/不保存）+ `$TASKPID` 处理 + `task_pids` 注入；新增 `task_pid_wire.py`、`task_pid_controller.py`；`move_test` 联动（mission/mode_auv/test_runner/task_config/t_pass_gate/gate_pid/gate_mission）；`config/to32_config.py` 新增 `PID_BINARY_PASSTHRU=True` / `PID_BINARY_MAX_CH=3` / `TASK_PID_ENABLED=True`；无硬件回归 30 项（16+14）**从补丁包 `源码\GrandRdk-2.5\tests\` 复跑全过**（脚本未拷入项目/未入 git）。⚠ 未实机：STM32 通道接受/4~7 补偿环待联调；PC 侧需配套上位机 v3.8 |
| 2026-10-07 | 本 README 更新：《PID 调参链路》新节；功能状态表 / 模式表 / 文件地图 / 进程表 / 排障 / 已知问题同步 2026-10-07 口径；`src/to32/`、`move_test/`、`task_pass_door/` 的 README 同步更新 |
| 2026-10-08 | ★ **AUV 接回主链路 + 卡尔曼托管恢复 + 模型回退**：`mode_dispatcher._register_modes()` 改**三态注册**（`TEST_MODE_ENABLED=True`→`TestMode` 接管 AUV 位跑 `TEST_TABLE`；`False`→正式 `AuvMode` 动态加载；配置异常→`AuvModeStub` 兜底宁停勿跑）。恢复 `move_test/kalman_launcher.py`（`mode_auv.on_enter/on_exit` 托管，`AUV_KALMAN_AUTOSTART=True` 自动拉起深度卡尔曼、`AUV_VISKF_AUTOSTART=False` 默认不拉起 viskf）。新增 `to32/tel_shm_sink.py`：下位机遥测落盘 `momo_telemetry.json`（depth_kalman 等观测工程遥测输入源，`on_stm32_telemetry` 每帧写、20Hz 节流、原子写）。**模型回退单模型**：`quick_config.py` 改 `YOLO_MODEL_FRONT/BOTTOM`（均 `door_3_640x640.hbm`）、`FRONT_TARGETS/BOTTOM_TARGETS=['door']`、`FRONT_CLASS_NAMES/BOTTOM_CLASS_NAMES=['door']`；`task_config.py` 全序列 STAGE_TABLE（Task1→撞球→Task2→SearchBall→捡球→Task4→Return）、`AUV_POOL_DEPTH_CM=106`（实测）、撞球 v2 参数 `AUV_HIT_V2_*`、STOP 哨兵；旧 6 段序列与 `AUVMODE_ENABLED` 键清理。`AuvMode` 已注册（文档口径同步） |
| 2026-10-09 | ★ **YOLO 模型按阶段动态切换（`config/stage_model.py`）**：新增 `MODELS` 三套配置（ball=test_nashe/gate=door_3/pick=bottom.hbm）+ `STAGE_MODELS` 阶段映射 + `StageDetector`（front/bottom worker 每帧读 `/dev/shm/momo_stage.json`，模型配置变化即热重载、`is_current()` 丢弃推理期间切走的结果、加载失败 1s 重试）；`mission._set_stage` 每阶段原子发布 stage。**穿门重构为 `task/task_door/`**（`t_door.py` `DoorTask(Stage)` `NAME='PassGate'`，状态机 ACQUIRE→YOLO_ALIGN→APPROACH_50→CV_ALIGN→APPROACH_80→CV_FINAL→BLIND→ACQUIRE，无门走 SEARCH→RETURN_HEADING→EXIT；`DOOR_TABLE` 已入 `STAGE_TABLE`；旧 `task_pass_door`（t_pass_gate.py）整体清理）。`front.py` PassGate 分支接穿门视觉跟踪管线（`task_door/front_pipeline`）；撞球 v2（`t_hit_ball_v2.py`，摇摆寻球→对准→冲撞→Exit 上浮）落地但暂未挂 STAGE_TABLE；`test_config.py` `TEST_TABLE = TASK2_TABLE + RETURN_TABLE`（不再是 5 任务整表）。各 README 同步更新 |
