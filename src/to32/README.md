# To32 中位机（`src/to32/`）

> 本目录是 **2026-09-21 从 `/userdata/To32` 整体迁入**的中位机程序，现在是 GrandRDK 工程的一部分。
> 详细协议与历史设计决策见 `README_中位机.md`、`ROV_指令与V2协议对应关系.md`、`上位机通讯协议.md`；
> 本文件是**代码导航 + 排障速查**，不重复协议的完整字段表。

## 0. 它是什么

中位机是三机链路的中间层：**上位机（PC UI） ↔ 中位机 ↔ 下位机（STM32）**，跑在 RDK S100 上，由 GrandRDK 的 `run.sh` 一并拉起。

```
   上位机 PC UI                中位机 src/to32/                下位机 STM32
 ┌──────────────┐          ┌────────────────────────┐      ┌──────────────┐
 │ $CMD  20Hz   │ UDP 8080 │ link_pc.py  收→归类→入队 │      │              │
 │ $PID / $VID  ├─────────▶│        ↓ queue          │      │              │
 │ PING         │          │ mode_dispatcher.py 编排 │ V2   │  串口 @921600 │
 │              │ UDP 8081 │   ├ mode_rov.py         ├─────▶│  /dev/ttyCH…  │
 │ $TEL 41 字段 │◀─────────┤   └ mode_auv.py         │◀─────┤   (CH348 F口) │
 │ PONG         │          │ link_stm32.py 组帧/收帧 │      │              │
 └──────────────┘          └────────────────────────┘      └──────────────┘
```

一句话：**上面说文本（`$`帧），下面说二进制（V2 帧），中间负责翻译、编排和安全。**

| 项 | 值 |
|---|---|
| 当前路径 | `/userdata/GrandRDK/src/to32/` |
| 配置 | `/userdata/GrandRDK/config/to32_config.py` |
| 模式记忆文件 | `/userdata/GrandRDK/logs/to32_mode_state.json` |
| 日志 | `/userdata/GrandRDK/logs/to32_main.log` |
| 上行端口 | UDP **8080**（收 `$CMD`/`$PID`/`$VID`） |
| 下行/遥测端口 | UDP **8081**（收 PING、发 `$TEL`/PONG） |
| 下位机串口 | `/dev/ttyCH9344USB5`（CH348 F 口），921600 |
| 进程识别串 | `src/to32/main.py`（`status.sh` / `stop.sh` 用它匹配） |
| 规模 | 14 个 .py，约 2850 行 |

## 1. 文件功能速查

### 入口与两条链路

| 文件 | 行数 | 职责 | 关键接口 |
|---|---|---|---|
| `main.py` | 288 | **入口**。解析命令行 → 构造配置视图 → 装配三件套 → 启动并阻塞等待；注册 SIGTERM 优雅退出 | `main()`、`build_runtime()`、`build_cfg()`、`list_stm32_ports()`、`preflight_ports()` |
| `link_pc.py` | 113 | **上位机链路**（线程）。绑 8080/8081，收字节→解析归类→入队；PING→PONG；`$TEL` 出口 | `PcLink`（Thread）：`open/stop/run/_dispatch/send_telem/pc_ip` |
| `link_stm32.py` | 295 | **下位机链路**。V2 组帧/收帧（`CD LEN FUNC DATA DC`）+ 0x0C 遥测轮询线程 | `build_frame`、`frame_mode`、`frame_motion`、`frame_set_pid`、`frame_target_depth`、`FrameParser`、`parse_telemetry`、`Stm32Link` |

> `link_pc.py` / `link_stm32.py` **只做传输**，不含任何模式逻辑。

### 模式层

| 文件 | 行数 | 职责 |
|---|---|---|
| `mode_base.py` | 89 | **模式基类** `ModeBase`，定义 10 个回调；新增模式只需继承它 |
| `mode_dispatcher.py` | 461 | **编排核心**：模式注册/切换、`$CMD` 事件派发、安全三件套、tick 调度、`$TEL` 上行、5s 状态汇报 |
| `mode_rov.py` | 163 | **ROV 遥控**：`$CMD` 杆位 → `0x09` 推力槽；heave→深度积分、yaw→航向积分；急停锁存期间抑制 `0x09` |
| `mode_auv.py` | 77 | **AUV 自主**（骨架）：进模式时下发 `0x02` 预编程，忽略手动杆位（只记录不动作），自主逻辑留 `tick()` |

`ModeBase` 回调一览（`mode_base.py`）：

| 回调 | 何时调用 | 用途 |
|---|---|---|
| `on_enter(prev_id)` | 切进来时一次 | 下发模式命令、初始化积分量 |
| `on_exit(next_id)` | 切走时一次 | 收尾、清零下发状态 |
| `on_cmd(cmd)` | 收到 `$CMD`（20Hz） | 主遥控输入 |
| `on_pid(pid)` | 收到 `$PID` | PID 透传（当前默认丢弃） |
| `on_vid(on)` | 收到 `$VID` | 开/关图像回传（板端拦截，不下发） |
| `on_downlink(tel)` | 收到 V2 遥测 | 闭环（ROV 目前用于锚定与遥测展示） |
| `tick(now, dt)` | 周期（`TICK_HZ`=20Hz） | 积分、超时、状态机 |
| `build_telemetry()` | 每 `$TEL` 周期 | 返回 `$TEL` 文本；`None` = 本周期不发 |
| `send_downlink(frame)` / `pump_telemetry(now)` | 模式内部 | 下发 V2 帧 / 触发一次遥测请求 |

### 协议与遥测映射

| 文件 | 行数 | 职责 | 关键接口 |
|---|---|---|---|
| `protocol.py` | 84 | **上位机文本协议**解析与组帧 | `parse_cmd`、`parse_pid`、`parse_vid`、`is_ping`、`build_tel`、`TEL_FIELD_COUNT=41` |
| `tel_builder.py` | 76 | **V2 遥测 → `$TEL` 41 字段**的**唯一**映射 | `tel_values(tel)`、`build_tel(tel)` |

> `$TEL` 的 41 个索引在上位机是**硬编码**的，缺失字段必须补 0 占位、不能省略，否则整帧错位。任何模式都不要再自己造一份映射，用 `tel_builder`。

### 图像回传（默认关闭，见第 7 节 WARNING）

| 文件 | 行数 | 职责 |
|---|---|---|
| `video.py` | 295 | USB 相机 → HTTP MJPEG（`:5000` `/cam1` `/cam2`）。懒加载开相机、`$VID,0#` 释放；HTTP 服务常驻；某路打不开则发占位帧不拖垮其它路。**2026-09-21 起 cam1/cam2 已停用**（默认不挂任何相机，见第 7 节） |

### 调试 / 自测工具（不参与运行）

> **2026-10-01（R8）**：`selftest_modes.py` / `make_test_frame.py` / `test_v2_frames.py`
> **已迁出本目录**，落到 **`hwless_tests/legacy_to32/`**（板端约定：测试代码只在 `hwless_tests/`）。
> 三者都补了自足的路径引导，**在仓库根目录下直接跑**即可（不再需要 `cd src/to32`）。
> 批量回归用 `python3 hwless_tests/run_legacy.py`。下表保留说明，路径已更新。

| 文件（新位置） | 行数 | 职责 | 用法 |
|---|---|---|---|
| `hwless_tests/legacy_to32/selftest_modes.py` | 440 | **模式框架自测**（无需下位机/上位机）：初始模式、`$CMD.mode` 切换、0x09 抑制、急停锁存与解除、`$TEL` 41 字段、PING→PONG、模式记忆 | `cd /userdata/GrandRDK && python3 hwless_tests/legacy_to32/selftest_modes.py`（日志 `/tmp/selftest_modes.log`）。⚠ 当前 **66/68**，2 例为已知偏差（见 `hwless_tests/README_hwless_tests.md` §九） |
| `hwless_tests/legacy_to32/make_test_frame.py` | 197 | 生成 **5 个场景的 V2 遥测回帧（48B）**，并自校验能被 `link_stm32`→`tel_builder` 原样解析 | `python3 hwless_tests/legacy_to32/make_test_frame.py` / `--hex-only` / `--bin f.bin` |
| `hwless_tests/legacy_to32/test_v2_frames.py` | 196 | **端到端测试**：把上面生成的 bin 逐字节喂进真代码路径，校验 A~E 五项判据 | `cd /userdata/GrandRDK && python3 hwless_tests/legacy_to32/test_v2_frames.py [bin]` → **19/19 PASS** |
| `port_probe.py` | 74 | **CH348 通道确认**（**留本目录**：运维排障工具，非测试代码）：往串口发 ASCII 标记，确认 PC 助手看到的是哪个物理口 | `python3 src/to32/port_probe.py --sweep`（**须先停 main.py**） |

## 2. 运行时线程模型

```
PcLink (daemon Thread)        收上位机包 → rx_queue
Stm32Link._poll_loop (Thread) 按 POLL_HZ 发 0x0C + 收遥测回帧 → on_telemetry 回调
Dispatcher._bridge_loop       rx_queue → on_pc_event → 各模式回调；$CMD.mode 触发切模式
Dispatcher._tick_loop         TICK_HZ(20Hz) → mode.tick() + 冷启动锚定 + $TEL(TEL_HZ) 上行
                              + _check_deadman() 看门狗
VideoService（若启用）        每路 Source 一个采集线程 + HTTP 服务线程
```

三者由 `main.build_runtime()` 装配，`Dispatcher` 持有 `pc_link` / `link_stm32`，并把 `stm32.on_telemetry` 回填成 `disp.on_stm32_telemetry`。

## 3. 协议速查

### 上位机侧（UDP 8080 收 / 8081 发）

| 帧 | 方向 | 说明 |
|---|---|---|
| `$CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#\r\n` | PC→中位机 | 20Hz（有线）/5Hz（无线）；`mode` 0=ROV、1=AUV |
| `$PID,ch,p,i,d#\r\n` | PC→中位机 | 逐通道下发，默认 `PID_PASSTHRU=False` 丢弃计数 |
| `$VID,1#` / `$VID,0#` | PC→中位机 | 图像回传开关，**板端拦截、不下发下位机** |
| `PING` | PC→8081 | 中位机回 `PONG` 到源端口 |
| `$TEL,<41 字段>#\r\n` | 中位机→PC | 按 `TEL_HZ` 上行 |
| `$ESTOP#` / `$ESTOP,0#` | PC→中位机 | 人工兜底急停/解除（v3.2 UI 不发，留作手动） |

### 下位机侧（V2 串口帧）

帧：`CD | LEN | FUNC | DATA(n) | DC`，`LEN = n+3`，整帧 ≤ 64B，多字节**小端**。

| FUNC | 含义 | 组帧函数 | 备注 |
|---|---|---|---|
| `0x01` | 设置 PID | `frame_set_pid` | 通道仅 0~3（Pitch/Yaw/Roll/Depth） |
| `0x04` | 模式 | `frame_mode` | `00`启动 `01`急停 `02`预编程 `03`ROV `04`测试 |
| `0x09` | 摇杆综合运动（15B） | `frame_motion` | 目标 Pitch/Yaw/Roll + 目标深度 + surge/sway + FLAG |
| `0x0A` | 目标深度 | `frame_target_depth` | — |
| `0x0C` | 遥测请求 | `frame_telemetry_request` | **唯一会回帧的帧**（1:1，48B） |

> **红线**：`0x09` **没有 STANDBY 守卫** —— 急停后再发 `0x09` 会静默退出急停。因此**急停锁存只能由中位机负责**，这也是 `mode_rov` 在锁存期间硬性抑制 `0x09` 的原因。

## 4. 模式机制

- 当前模式：`MODE_ROV=0`、`MODE_AUV=1`（`to32_config.py`）。
- 切换来源：上位机 `$CMD` 的 `mode` 字段；`Dispatcher.switch_mode()` 负责 `on_exit`→`on_enter` 与模式记忆。
- **模式记忆**：`MODE_PERSIST=True` 时，上位机最后一次切换的模式写入 `logs/to32_mode_state.json`，下次启动沿用。
  - `--mode rov|auv` 强制覆盖（走 `MODE_FORCE_START`）；
  - `--no-mode-persist` 本次不记忆。

**新增一个模式（三步）**

1. 新建 `mode_xxx.py`，继承 `mode_base.ModeBase`，填 `id` / `name` / `desc`，实现需要的回调；
2. 在 `config/to32_config.py` 登记 `MODE_XXX`（即 `$CMD.mode` 的取值）；
3. 在 `mode_dispatcher.py` 的 `_register_modes()` 里注册。

## 5. 安全三件套（`mode_dispatcher.py`）

| 机制 | 触发 | 行为 |
|---|---|---|
| **下行失联看门狗** | 上位机静默 > `ESTOP_TIMEOUT_S`（默认 1.0s） | 下发 `0x04 0x01` 并锁存（`0`=关闭） |
| **急停锁存** | 看门狗触发 / 上位机 `$ESTOP#` | `estop_latch=True`，**抑制一切 `0x09`** |
| **解除** | `$ESTOP,0#`；或 `ESTOP_LATCH=False` 时收到新 `$CMD` 自动解除 | 重置 `last_cmd_ts`，deadman 重新起算 |

> ⚠️ **`run.sh` 默认把这些关掉**：实际启动参数是 `--estop-timeout 0 --estop-no-latch`（避免联调时频繁误急停）。
> 需要真正的自动急停：**`./run.sh --to32-estop`**（保留默认 1s + 锁存）。

## 6. 配置在哪改

**改中位机参数请改 `/userdata/GrandRDK/config/to32_config.py`**（不是 `config/quick_config.py`，那是视觉链路的）。

| 组 | 关键项 |
|---|---|
| 通信 | `CMD_PORT=8080`、`TELEM_PORT=8081`、`SERIAL_PORT=/dev/ttyCH9344USB5`、`BAUD=921600`、`PC_BIND_IP` |
| 频率 | `POLL_HZ=10`、`TEL_HZ=10`、`TICK_HZ=20`、`REPORT_S=5` |
| 模式 | `MODE_ROV/AUV`、`START_MODE`、`MODE_PERSIST`、`MODE_STATE_PATH`、`MODE_FORCE_START` |
| ROV 映射量纲 | `SURGE_FULL_SCALE=127`、`YAW_RATE_DPS=60`、`DEPTH_RATE_CMS=20`、`HEAVE_SIGN=1`、`YAW_MIRROR=True`、`DEPTH_MAX_CM=200` |
| PID | `PID_PASSTHRU=False`、`PID_MAX_CH=3`、`PID_VALUE_LIMIT=327.67` |
| 静默 | `MOTION_SILENT_WHEN_IDLE=True`、`POLL_IDLE_SILENT=False`、`IDLE_SILENT_AFTER_S=1.0`（**2026-09-21 修订**：静默判定是"surge/sway/heave/yaw **四轴全为 0**"，不是"载荷未变"；旧语义下持续推杆会只发 1 帧，已在 `selftest_modes.py` 加了回归用例） |
| 安全 | `ESTOP_TIMEOUT=1.0`、`ESTOP_LATCH=True`（**可被 run.sh 命令行覆盖**） |
| 图像 | `VIDEO_ENABLED_AT_START=True`、`VIDEO_HTTP_PORT=5000`、`VIDEO_WIDTH/HEIGHT/FPS/QUALITY`、`VIDEO_PATHS` |

## 7. 怎么跑

### 随整栈拉起（推荐）

```bash
cd /userdata/GrandRDK
./run.sh                    # 含中位机，实际参数: main.py --no-video --video-port 0 --estop-timeout 0 --estop-no-latch
./run.sh --to32-estop       # 打开自动急停 + 锁存
./run.sh --no-to32          # 不拉中位机
./run.sh --to32-args "--mode auv"   # 附加参数透传
```

`run.sh` 会先 `pkill -f "src/to32/main.py"` 再拉，并把 stdout/stderr 重定向到 `logs/to32_main.log`。

### 单独手动跑

```bash
cd /userdata/GrandRDK
export PYTHONPATH="config:src:src/utils:src/to32"

python3 src/to32/main.py --list            # 列出候选串口后退出
python3 src/to32/main.py --stm32 sim       # 无硬件联调：只跑协议状态机，无遥测源
python3 src/to32/main.py --stm32 off       # 只跑上位机侧，不碰串口
python3 src/to32/main.py --mode auv        # 强制以 AUV 启动
python3 src/to32/main.py --no-mode-persist # 本次不记忆模式
python3 src/to32/main.py --no-video        # ★ 手动跑一定带上，理由见下
```

### 图像回传：cam1 / cam2 已于 2026-09-21 停用（不再碰任何相机）

中位机自带一套 MJPEG 回传（`video.py`，HTTP `:5000` `/cam1` `/cam2`），但本板的图像服务实际由 `web_server.py` 提供，双方的取流路径同名、且都要独占摄像头设备节点。为彻底消除抢设备的隐患，**默认关闭**：

| 位置 | 改动 |
|---|---|
| `config/to32_config.py` | `VIDEO_PATHS = {}`（原 cam1/cam2 节点映射已注释留痕）→ `VideoService.sources` 为空，不开任何相机 |
| `config/to32_config.py` | `VIDEO_ENABLED_AT_START = False`（原 `True`）→ 即使 `$VID,1#` 之前也不主动采集 |
| `src/to32/video.py` | `getattr(cfg,"VIDEO_PATHS", {})` 的**兜底改成空 `{}`**；这里的兜底比配置更关键——键一旦缺失就会回落到去开前视相机 |
| `src/to32/video.py` | 独立调试入口 `python3 video.py` 的 `--cam1/--cam2` 默认值改为 `None`（不指定就是不开） |
| `run.sh` | 仍然传 `--no-video --video-port 0`（连 `:5000` 都不绑，彻底与 `web_server.py` 无争） |

**仍可临时起用**：`python3 src/to32/main.py --cam1 <cam1 节点路径>`（命令行 `--cam1/--cam2` 覆盖通道保留，节点号可由 `v4l2-ctl --list-devices` 现场查）。但注意它还要绑 `:5000`，所以**必须先停 `web_server.py`**，且相机节点不能与 front/bottom 冲突。

> ⚠️ 一句话结论：**平时不用管它，`run.sh` 已经全关；手启 `main.py` 时如果发现它在蹭 `:5000`，补一个 `--no-video`。**

## 8. 排障速查

| 现象 | 查什么 | 处理 |
|---|---|---|
| 起来就退出 / 日志报端口被占 | `logs/to32_main.log` 里 `preflight` 段 | `:8080/:8081` 被其它实例占；`./stop.sh` 后重来 |
| 看不到中位机进程 | `pgrep -fa "src/to32/main.py"` | `status.sh` 靠这个串匹配；手动启动要用**绝对路径** `src/to32/main.py` 才会被识别 |
| 有 `$CMD` 但下位机没反应 | 日志 `report` 段的下行帧计数、`0x09` 计数 | 串口是 CH348 **F 口**（`USB5`）；用 `--list` 核对，或 `port_probe.py --sweep` 确认实际通道 |
| `$TEL` 一直不上行 | 日志是否有「尚无下位机遥测」 | `$TEL` 源是 V2 遥测；下位机不回 `0x0C` 就没有上行，先解决串口/供电 |
| **改了 `to32_config.py` 没生效** | 是否在哪儿写成 `import config` | 本项目根目录有 `config/` 目录，`import config` 会被解析成**空的命名空间包**且不报错，配置静默退回默认值。**必须 `import to32_config`** |
| 挪动/拆分子目录后自测崩 | `hwless_tests/legacy_to32/selftest_modes.py` 是否报 `ModuleNotFoundError` | **2026-10-01 起不再是"必须同目录"**：源码守卫已改成**绝对路径** `<宿主>/src/to32`，脚本自带路径引导。若仍报错，检查宿主结构是否为 `<宿主>/{config,src,src/to32}` |
| `port_probe.py` 报打不开串口 | 是否还开着 `main.py` | 串口独占，**先停中位机**再探测 |
| 数值对不上（上位机显示的数 vs 串口里的数） | 跑一遍 `make_test_frame.py` | 它用同一套代码算出 HEX 与对应 `$TEL`，两边天然对齐 |

## 9. 相关文档

| 文档 | 内容 |
|---|---|
| `README_中位机.md` | 中位机完整设计、协议背景、历史决策（29KB） |
| `ROV_指令与V2协议对应关系.md` | 上位机指令 ↔ V2 帧字段的逐项对照（18KB） |
| `上位机通讯协议.md` | 上位机文本协议实测稿（`$CMD`/`$PID`/`$VID`/`$TEL` 字段定义） |
| 本文件末尾附录 | 原 `docs/README.md`（2026-09-15 完整设计文档），2026-09-22 合并保留 |
| `/userdata/GrandRDK/README.md` | GrandRDK 主工程的 README（进程表、端口表、整栈排障） |

---

**变更记录（本目录）**

| 日期 | 变更 |
|---|---|
| 2026-09-21 | 从 `/userdata/To32` 整体迁入 `src/to32/`；配置拆到 `config/to32_config.py`；`import config` 修正为 `import to32_config`；`MODE_STATE_PATH` 改指向 `logs/`；`run.sh`/`stop.sh`/`status.sh` 路径更新 |
| 2026-09-21 | **停用 cam1/cam2 图像回传**：`VIDEO_PATHS = {}`、`VIDEO_ENABLED_AT_START = False`，`video.py` 的兜底默认值改空、独立调试入口不再默认开相机；`--cam1/--cam2` 覆盖通道保留 |
| 2026-09-21 | `mode_rov` 空闲静默语义修订：四轴全 0 才判回中，持续推杆每帧必发 `0x09`；`selftest_modes.py` 增加三组回归用例（持续推杆 / yaw / heave） |
| 2026-10-01 | ★ **R8：测试代码迁出本目录** —— `selftest_modes.py` / `make_test_frame.py` / `test_v2_frames.py` → `hwless_tests/legacy_to32/`（补自足路径引导 + 源码守卫改绝对路径）；`port_probe.py` 作为运维排障工具**留原地**。本目录此后**不含任何测试脚本** |
| 2026-10-01 | ⚠ **已知偏差**：`selftest_modes.py` 当前 66/68（"AUV 期间无 0x09" 2 例）。非 R8 引入，用迁移前原版在当前代码上复跑结果相同；根因是 AUV 模式已改为主动下发运动帧 |

---

# 附录：原 `docs/README.md`（2026-09-15 完整设计文档）

> 2026-09-22 由 `docs/` 合并入本文件，原文保留；与上文「代码导航」部分有内容重叠。

# To32 —— ROV 中位机（RDK S100 中间层控制程序）

三端互通里的**中位机**：把上位机（PC UI）的文本指令翻译成下位机（STM32）的 V2 二进制帧，
再把下位机回传的遥测翻译成上位机的 `$TEL` 文本帧，并自带图像回传与安全逻辑。

```
   上位机 PC UI                       中位机 RDK S100                     下位机 STM32
 ┌───────────────┐   UDP :8080       ┌──────────────────────┐  UART 115200      ┌──────────────┐
 │ ROV控制站      │ ─── $CMD/$PID/$VID ──▶ │  main.py             │ ── V2 二进制 ──▶ │ XLB-V1.0-    │
 │  (PyQt5)      │   UDP :8081       │   ├ link_pc.py        │  /dev/ttyCH9344USB5│ Servo(V2)    │
 │               │ ◀── $TEL/PONG ──────── │   ├ link_stm32.py    │ ◀── 0x0C 48B ──── │              │
 └───────────────┘   HTTP :5000      │   ├ mode_dispatcher   │                   └──────────────┘
        ▲              /cam1 /cam2   │   └ video.py (MJPEG)  │
        └─────────── cv2.VideoCapture ┘──────────────────────┘
```

- 链路 A：上位机 ⇄ 中位机 = **UDP 文本协议**（`$CMD` / `$PID` / `$VID` / `$TEL` / `PING`）
- 链路 B：中位机 ⇄ 下位机 = **V2 二进制**（`CD LEN FUNC DATA DC`，CH348 **F 口**）
- 链路 C：中位机 → 上位机 = **HTTP MJPEG**（`:5000/cam1`、`/cam2`）

> 本 README 基于对 `/userdata/GrandRDK` 全部 15 个 `.py` + 3 份文档的逐行阅读整理（2026-09-15），
> 并对照了下位机固件源码 `XLB-V1.0-Servo/firmware/JXZK_XLB_lib/`。文中的行为都已实测或源码核对。

---

## 1. 快速开始

```bash
cd /userdata/GrandRDK

python3 main.py --list                            # 列出候选串口（CH348 A~H 口）
python3 main.py                                   # 正式跑：config.SERIAL_PORT（CH348 F 口）
python3 main.py --stm32 sim                  # 无硬件联调：仅跑协议状态机, 无遥测源
python3 main.py --stm32 sim --no-video --log /tmp/mid.log   # 空跑并落日志
python3 main.py --mode auv                        # 强制以 AUV 启动（忽略模式记忆）
python3 main.py --no-video --pc-port 18080        # 不占 :5000 / 不抢 8080（与别的脚本并存）
```

启动横幅会打印：上位机端口、下位机串口、初始模式、图像回传状态、**启动模式来源**（命令行/记忆/config）。

### 运行前必看：端口与设备互斥

| 资源 | 谁在抢 | 说明 |
|---|---|---|
| UDP **8080 / 8081** | 本程序、`To32_1/relay.py`、`cmd_watch.py` | 三者不能同时跑；调试期建议 `--pc-port 18080 --tel-port 18081` |
| 串口 `/dev/ttyCH9344USB5` | 本程序、`port_probe.py` | 串口独占：用 `port_probe.py` 前先停 `main.py` |
| 摄像头设备节点 | 本程序内置 MJPEG（`:5000`）、`vp5.1`（front/bottom + YOLO → `:9000/:9001`） | **互斥**，不要同时启动 |
| UDP 8082 | 高度计 `read_altimeter.py`（另一路） | 与中位机无关，别占 |

---

## 2. 文件地图（15 个 .py + 3 份文档）

### 运行核心（11 个）

| 文件 | 行数级别 | 职责 |
|---|---|---|
| `main.py` | ~340 | 入口：命令行解析、日志（含 sim 折叠）、`build_cfg()` 配置视图、端口预检、信号处理与优雅退出 |
| `config.py` | ~110 | **所有可调参数的唯一落点**（端口/串口/节拍/模式/映射/PID/静默/量纲/视频） |
| `protocol.py` | ~80 | 上位机文本协议：`$CMD` 解析、`$TEL` 41 字段组帧、`PING/PONG` |
| `link_pc.py` | ~113 | 上位机 UDP 链路：收指令线程 → 队列、发遥测、回 PONG、记录对端 `first_cmd_logged`/`pc_ip()` |
| `link_stm32.py` | ~295 | V2 编解码：`FrameParser`（CD/LEN/DC 定界 + 失步重同步）、`frame_*` 组帧、`parse_telemetry`、串口/sim 链路 + 兜底轮询 |
| `tel_builder.py` | ~76 | V2 遥测 → `$TEL` **唯一映射**（含 `MOTOR_SCALE`、V2 缺失字段补 0） |
| `mode_base.py` | ~90 | `ModeBase` 骨架：`send_downlink` / `pump_telemetry`（0x0C 节拍）/ `build_telemetry` |
| `mode_dispatcher.py` | ~460 | **编排核心**：模式切换与记忆、急停锁存与 deadman、冷启动锚定、`tick` 循环、`$TEL` 上行、周期汇报、收包线程桥接 |
| `mode_rov.py` | ~165 | 遥控模式：`$CMD` → `0x09`（含空闲静默、参数读 config）、`$PID` 丢弃计数、遥测转发 |
| `mode_auv.py` | ~77 | 自主模式：不发 `0x09`（避免抢控制权），仅遥测与安全 |
| `video.py` | ~295 | 内置图像回传：`Source` 采集线程 + MJPEG HTTP（`:5000`），离线/关闭时发占位帧 |

### 工具与测试（`port_probe.py` 留本目录；其余 3 个已迁 `hwless_tests/legacy_to32/`）

| 文件 | 用途 | 当前基线 |
|---|---|---|
| `hwless_tests/legacy_to32/selftest_modes.py` | 框架自测：启动/切换/0x09/急停/遥测锚定/链路/模式记忆/**空闲静默** | ⚠ **66/68**（2 例已知偏差，见 `hwless_tests/README_hwless_tests.md` §九） |
| `hwless_tests/legacy_to32/test_v2_frames.py` | 5 条 V2 遥测测试帧的端到端测试（状态机→解析→编排→`$TEL`） | **19/19 通过** |
| `hwless_tests/legacy_to32/make_test_frame.py` | 生成 5 个场景的 48B 遥测回帧（HEX / `--bin` 导二进制 / 兼容旧的两参数用法） | 自校验过 `FrameParser`+`parse_telemetry` |
| `port_probe.py` | 往 CH348 各口写可识别 ASCII，**确认 `/dev/ttyCH9344USB5` 是不是真的 F 口** | 需独占串口 |

验证命令（改动后必跑；测试脚本已迁，路径见下）：
```bash
cd /userdata/GrandRDK && python3 -m py_compile src/to32/*.py && \
python3 hwless_tests/run_legacy.py --fast     # 含 selftest_modes + 造帧 + V2 端到端
```

### 文档（3 份，各有分工）

| 文档 | 内容 | 关系 |
|---|---|---|
| `README.md` | **本文件**：架构、文件地图、协议表、运行/验证、坑与待办 | 入口 |
| `README_中位机.md` | 三端互通状态审计：未完成清单 U1–U20（逐条带 file:line）、已验证清单、证据台账、变更记录 | 问题追踪 |
| `ROV_指令与V2协议对应关系.md` | 上位机报文 ↔ V2 帧的逐条对应、缺口分析、验收判据 | 协议对照 |
| `上位机通讯协议.md` | 上位机侧 `$TEL` 41 字段索引定义（上位机硬编码） | 上位机口径 |

---

## 3. 链路 B：V2 二进制协议（下位机）

### 3.1 通用帧格式

```
CD  LEN  FUNC  DATA(n)  DC          LEN = n + 3      整帧 = LEN + 1 ≤ 64 字节
                                   多字节一律**小端**
```

收帧状态机：`CD` 定界 → 读 `LEN` → 收满 → 校验尾字节必须 `DC`；不合法则丢 1 字节继续找 `CD`（失步重同步）。
`LEN < 3` 或整帧 > 64 一律丢弃。

### 3.2 命令帧（中位机 → 下位机）

| FUNC | 名称 | DATA | 说明 |
|---|---|---|---|
| `0x01` | PID 参数 | 7 | `[0]ch(0~3)` + Kp/Ki/Kd 各 int16LE×100。**待机忽略**；超范围由 `frame_set_pid` 钳位 |
| `0x02` | 测试油门 | 3 | `[0]电机0~7` + throttle int16LE 原值。**写入即切 TEST 模式** ⚠️ |
| `0x03` | 目标姿态角 | 3 | `[0]轴(0=Pitch,1=Yaw,2=Roll)` + 角度 int16LE×100。**待机忽略** |
| `0x04` | 模式 | 1 | `0x00启动/恢复 0x01急停 0x02预编程 0x03ROV 0x04测试` |
| `0x05` | 保存参数 | 0 | 固件侧无动作（仅占位） |
| `0x08` | 读角度 | 0 | 结果走遥测帧回传 |
| **`0x09`** | **摇杆综合运动** | **11** | 见 3.3，**ROV 遥控的唯一控制通道** |
| `0x0A` | 目标深度 | 2 | uint16LE，cm×100，钳位 0~200(=2.00m)。**待机忽略** |
| `0x0B` | 读深度 | 0 | 结果走遥测帧回传 |
| `0x0C` | 遥测请求/返回 | 0 | **唯一会回包的帧**（严格 1:1） |

时序约定：`0x0C` 按 `config.STM32_POLL_HZ`（10Hz）轮询；`0x01~0x0B` **只执行不回包**，不要写成"发命令等 ACK"。

### 3.3 `0x09` 摇杆综合运动（11 字节 DATA）

| 偏移 | 字段 | 编码 | 固件侧行为（`JXZK_XLB_Protocol.c:743-788`） |
|---|---|---|---|
| 0..1 | 目标 Pitch | int16LE 度×100 | `normalize(±90)` |
| 2..3 | 目标 Yaw | int16LE 度×100 | **固件自己取负** `Target_Yaw = -normalize(±180)` |
| 4..5 | 目标 Roll | int16LE 度×100 | `normalize(±90)` |
| 6..7 | 目标深度 | uint16LE cm×100 | 钳位 ≤ 200（2.00m） |
| 8 | surge 推力 | int8 | 固件 `Forward_Thrust = surge × 2` |
| 9 | sway 推力 | int8 | 固件 `Sidesway_Thrust = sway × 2` |
| 10 | FLAG | 位域 | bit0=1 → 推力清零且 `Motion_Flag=0` |

⚠️ **两个必须知道的固件行为**（源码核对）：
1. **收到 0x09 会自动切到 ROV 模式**（非 ROV 时），且**没有 STANDBY 守卫** →
   急停期间中位机必须在软件侧抑制 0x09（`mode_rov.on_cmd` 的 `estop_latch` 检查），固件拦不住。
2. `Motion_Flag` / `Motion_Time` **只写不读**（全项目无读取）→ **不存在"靠周期帧维持"的看门狗**，
   所以"摇杆静止就不发帧"是安全的；但**回中瞬间必须补发一帧**把 `surge/sway` 归零，否则固件会保持上一次推力。

### 3.4 遥测返回帧（下位机 → 中位机）

```
CD 2F 0C <44B DATA> DC     整帧 48 字节（LEN=0x2F=44+3）
```
| DATA 偏移 | 字段 | 编码 |
|---|---|---|
| 0..11 | 目标 / 实际 P、Y、R | 各 int16LE 度×100 |
| 12..17 | 角速度 P、Y、R | int16LE 度/秒×100 |
| 18..23 | 加速度 X、Y、Z | int16LE m/s²×100 |
| 24..39 | 电机 0~7 目标转速 | 8×int16LE，**原值**（不除 100） |
| 40..41 | 目标深度 | uint16LE cm×100 |
| 42..43 | 实际深度 | uint16LE cm×100（**负值固件强制 0**） |

---

## 4. 链路 A：上位机文本协议

### 4.1 下行（上位机 → 中位机，UDP 8080）

- `$CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#` —— **20Hz 循环帧**，唯一运动+状态载体（无独立状态帧）
  - `mode`：`0`=ROV 遥控，`1`=AUV 自主
  - `led1/led2/grab/store`：V2 **无对应落点** → 中位机丢弃并计数（改记"上升沿"，避免 20Hz 刷爆）
- `$PID,ch,p,i,d#` —— PID 调试面板；当前**决策 D：暂不打通**（丢弃 + 计数 + 首帧提示，`config.PID_PASSTHRU`）
- `$VID,1# / $VID,0#` —— 图像回传开关（板端拦截，不下发下位机）
- `$ESTOP#` / `$ESTOP,0#` —— 显式急停 / 解除锁存（板端拦截）
- `PING` → 回 `PONG`（链路活性）

### 4.2 上行（中位机 → 上位机，UDP 8081）

- `$TEL,` + **41 个字段** + `#`，默认 10Hz（`config.TEL_HZ`）
  索引定义见 `上位机通讯协议.md`；映射实现**只有一份**：`tel_builder.tel_values()`
- V2 没有的字段（线速度 vx/vy/vz、目标角速度、电池/舱温、高度 alt、9~12 路电机）**必须补 0 占位**，否则上位机整帧错位
- 深度以**米**上报（`actual_depth_cm / 100`）；电机以 `MOTOR_SCALE`（默认 1000）归一

---

## 5. 关键机制

### 5.1 模式与记忆
`START_MODE`（默认 ROV）→ 若 `MODE_PERSIST=True`，上位机最后一次切换写入 `mode_state.json`，下次启动沿用 →
命令行 `--mode rov|auv` 强制覆盖（`MODE_FORCE_START=True`），`--no-mode-persist` 临时关闭记忆。启动横幅会打印"模式来源"。

### 5.2 安全（急停 / deadman）
- **deadman**：连续 `config.ESTOP_TIMEOUT`（默认 1.0s）没有上位机下行 → 自动下发 `0x04 0x01` 进 STANDBY
- **锁存**：`ESTOP_LATCH=True`（默认）时急停后**抑制 0x09**，必须 `$ESTOP,0#` 或 `--estop-no-latch` 才能恢复
- **显式急停**：`$ESTOP#` 立即进 STANDBY 并锁存；`$ESTOP,0#` 解除（板上锁存后**唯一**的解锁手段）
- 上电/进入 ROV 先发 `0x04 0x03` 就位，但**不发 0x09**

### 5.3 空闲静默（2026-09-15 需求）
`0x09` **只在载荷字节发生变化时下发**（与上一帧逐字节比对），摇杆在中间且目标量未变 → F 口无 0x09。
- 实测：20Hz 全零 `$CMD` 连续喂 10s（184 帧）→ 实际只下发 **1 帧**（建立基准那帧）
- **两帧必须保留**：① 回中瞬间补发一帧（推力归零）；② 急停解除后补发一帧
- 开关 `config.MOTION_SILENT_WHEN_IDLE`（默认 True）
- 若要求"连遥测请求也不发"：`config.POLL_IDLE_SILENT=True`（**代价：`$TEL` 断流**，上位机看不到数据）

### 5.4 冷启动锚定
首帧遥测到达时，把模式内部的目标量（`target_depth_cm` / `target_yaw_deg`）**锚定到实测值**，
避免一上电/切模式就"回水面、转向 0°"。日志出现 `[ANCHOR] 首帧遥测 -> …` 即生效。

### 5.5 ROV 的"降级语义"（重要）
V2 没有 heave / yaw **推力**槽位，只有 0x09 的"目标角/目标深度 + surge/sway"。因此中位机把：
- `yaw` 摇杆 → 按 `YAW_RATE_DPS` 积分成**目标航向**（再按 `YAW_MIRROR` 抵消固件取负）
- `heave` 摇杆 → 按 `DEPTH_RATE_CMS` 积分成**目标深度**（`HEAVE_SIGN` 定方向，上限 `DEPTH_MAX_CM`）

**控制律不在中位机**，在下位机固件里；中位机只做"摇杆量 → V2 帧"的翻译。

### 5.6 图像回传（`:5000`）
HTTP 服务常驻（即使视频关着），`$VID,1#` 懒加载打开相机并采集，`$VID,0#` 释放相机；
相机打不开（如 cam2 节点是元数据节点）→ 该路只发**占位帧**，不黑屏不断流。
与 `vp5.1`（带 YOLO 的重链路）**互斥**。

---

## 6. 参数速查（`config.py`）

| 分类 | 键 | 默认 | 说明 |
|---|---|---|---|
| 网络 | `CMD_PORT` / `TELEM_PORT` | 8080 / 8081 | 上位机 UDP 端口 |
| 串口 | `SERIAL_PORT` / `BAUD` | `/dev/ttyCH9344USB5` / 115200 | CH348 **F 口**（A~H=USB0~7） |
| 节拍 | `POLL_HZ` / `TEL_HZ` / `REPORT_S` | 10 / 10 / 5 | 0x0C 请求 / `$TEL` 上行 / 状态汇报 |
| 安全 | `ESTOP_TIMEOUT` / `ESTOP_LATCH` | 1.0 / True | deadman 秒数 / 锁存 |
| 模式 | `START_MODE` / `MODE_PERSIST` / `MODE_STATE_PATH` / `MODE_FORCE_START` | 0 / True / `mode_state.json` / False | 见 5.1 |
| 映射 | `SURGE_FULL_SCALE` / `YAW_RATE_DPS` / `DEPTH_RATE_CMS` / `HEAVE_SIGN` / `YAW_MIRROR` / `DEPTH_MAX_CM` | 127 / 60 / 20 / 1 / True / 200 | 见 5.5 |
| PID | `PID_PASSTHRU` / `PID_MAX_CH` / `PID_VALUE_LIMIT` | False / 3 / 327.67 | 决策 D |
| 静默 | `MOTION_SILENT_WHEN_IDLE` / `POLL_IDLE_SILENT` / `IDLE_SILENT_AFTER_S` | True / False / 1.0 | 见 5.3 |
| 量纲 | `MOTOR_SCALE` | 1000 | V2 电机原值 → [-1,1] |
| 视频 | `VIDEO_ENABLED_AT_START` / `VIDEO_HTTP_PORT` / `VIDEO_PATHS` / `VIDEO_WIDTH/HEIGHT/FPS/QUALITY` | True / 5000 / cam1,cam2 / 1280×720@15 q70 | 见 5.6 |

> 协议码（`HEAD/TAIL/MAX_FRAME/FUNC_*/MODE_*`）的**唯一来源是 `link_stm32.py`**，`config.py` 里不再有副本。

---

## 7. 常见任务 → 改哪里

| 想做的事 | 改哪里 |
|---|---|
| 换串口 / 波特率 | `config.SERIAL_PORT` / `BAUD`（或 `--stm32` / `--baud`） |
| 换上位机 IP/端口 | `--pc-bind` / `--pc-port` / `--tel-port` |
| 改摇杆灵敏度 | `config.YAW_RATE_DPS` / `DEPTH_RATE_CMS` / `SURGE_FULL_SCALE` |
| 改深度上限 | `config.DEPTH_MAX_CM`（固件硬钳 200cm） |
| 改急停超时 / 取消锁存 | `config.ESTOP_TIMEOUT` / `ESTOP_LATCH`（或 `--estop-timeout` / `--estop-no-latch`） |
| 静止时连遥测也停 | `config.POLL_IDLE_SILENT = True` |
| 加一路相机 | `config.VIDEO_PATHS` / `--cam1` / `--cam2` |
| 加一个新 V2 命令帧 | `link_stm32.py` 加 `frame_*`（协议码同文件），模式里调用 |
| 改 `$TEL` 字段含义 | **只改** `tel_builder.tel_values()`（唯一映射） |

---

## 8. 排查手册

| 现象 | 先看/先做 |
|---|---|
| 进程在跑但下位机没反应 | 日志有没有 `[ANCHOR]` 与 `遥测 N 帧` 增长；**串口打不开会静默降级为 sim**，只看"进程活着"必然误判 |
| 上位机连接标签变红 | 是否 `POLL_IDLE_SILENT=True`（停遥测）；`ss -lunp` 看 8081 有没有在发 |
| 收不到 `$CMD` | 端口被 `relay.py`/`cmd_watch.py` 抢了；或 `--pc-bind` 绑错网卡 |
| F 口到底是不是 F 口 | 停 `main.py` → `python3 port_probe.py --sweep`，看 PC 串口助手收到哪个 `<<CH348 USBn=X>>` |
| 摄像头没画面 | `v4l2-ctl --list-devices` 查节点号；`vp5.1` 是否在跑（互斥）；`:5000/cam1` 是否是占位帧（相机离线） |
| 板上备份 `cp -a` 会挂起 | 改用 `tar czf /userdata/backup/To32_<主题>.tar.gz -C /userdata To32`（见 README_中位机 U20） |

---

## 9. 当前边界与待办（不要在 README 里假装已通）

| # | 事项 | 状态 |
|---|---|---|
| 1 | **实体 STM32 联调** | ❌ **从未跑通**（历史都是 `--stm32 sim` 空跑）。这是唯一的硬件级缺口 |
| 2 | `$PID` 打通 | 暂缓（决策 D）。需先统一"上位机 12 通道"与"固件 ch0~3"的口径 |
| 3 | `led1/led2/grab/store` | 无 V2 落点，只能丢弃计数；要走通需固件新增 FUNC |
| 4 | 角速度/姿态**方向**与手感 | 未实船核对（`$TEL[3..5]` 方向约定） |
| 5 | `0x02/0x03/0x05/0x08/0x0B` 组帧 | `link_stm32.py` 已定义协议码；`frame_target_depth()` 已实现但**暂无调用方**（规范帧，等按协议规范重写时接入） |
| 6 | 规范文档勘误 | `上位机对接说明_V2.md` §6「目标深度 1.00 m = `CD 05 0A 64 00 DC`」**是错的**（`0x0064`=1.00 **cm**；1.00m 应为 `10 27`）。另：0x09 的 DATA 布局该文档未定义，实以固件为准 |
| 7 | 实体相机第二路 | `/cam2` 目前是占位流 |

---

## 10. 本次整理记录（2026-09-15）

**删除/隔离的"不再需要"内容**（全部有引用扫描证据，见下）：

| 对象 | 证据 | 处理 |
|---|---|---|
| `config.py` 的 V2 常量块（`V2_HEAD/V2_TAIL/V2_MAX_FRAME/FUNC_*/MODE_CMD_*` 共 18 个键） | 与 `link_stm32.py` 重复定义，全项目 **0 引用**（`grep -E "\b(cfg\|C\|config)\.(V2_\|FUNC_\|MODE_CMD_)"` 无命中） | 删除，留指针注释指向 `link_stm32.py` |
| `config.PC_IP / MAIN_LOOP_S / MODE_NAMES / DEPTH_SCALE` | 全项目 0 引用 | 删除 |
| `protocol.is_zero_cmd()` | 0 调用（且对应"用全零 $CMD 判急停"的已否决方案） | 删除 |
| `main.py.bak_20260913_152832`（11 日旧备份，13KB） | 已被当前 `main.py`（30KB）取代 | 移出工作目录 |
| `mid.log` / `mid.out`（11 日陈旧日志） | 与当前运行无关 | 移出工作目录 |
| `__pycache__/`（11 个 .pyc） | 字节码缓存，自动重建 | 删除 |
| `config.PID_MAX_CH` / `POLL_IDLE_SILENT` | **是我自己加的却没人读**的"空头承诺" | 已接线：`PID_MAX_CH` 进 `on_pid` 提示；`POLL_IDLE_SILENT` 在 `mode_base.pump_telemetry` 真正生效（新增 `IDLE_SILENT_AFTER_S`） |

**保留但标记**：`link_stm32.py` 的规范帧码（`FUNC_TEST_THROTTLE/FUNC_TARGET_ANGLE/FUNC_SAVE/FUNC_READ_ANGLE/FUNC_READ_DEPTH/MODE_START/MODE_TEST`）与 `frame_target_depth()` 当前 0 引用 —— 它们是 V2 规范帧，**待 §9-5 协议重写时启用**，故不删。

**回滚点**：
- `/userdata/backup/To32_cleanup_20260915/`（编辑前快照：`config.py`/`mode_base.py`/`mode_rov.py`/`protocol.py` + 移出的 3 个陈旧文件）
- `/userdata/backup/To32_before_estopB_20260915/`（2026-09-15 上一轮改动前的 7 个文件）

**整理后回归验证（全部实测）**：

```
python3 -m py_compile src/to32/*.py            → OK
python3 hwless_tests/legacy_to32/selftest_modes.py  → 66/68（2 例已知偏差，见 hwless_tests/README §十）
python3 hwless_tests/legacy_to32/test_v2_frames.py  → 19/19 PASS
main.py --stm32 sim 冒烟 5s                     → 无 Traceback / ERR（退出码 124 = 被 timeout 正常终止）
死代码复扫                                       → 未用 import 0 | 死方法 0 | config 孤岛键 0（仅剩保留的协议码）
```

> 以上命令的脚本位置已于 2026-10-01 更新（测试脚本迁至 `hwless_tests/legacy_to32/`）；
> 数值列的是**当前实测**，历史记录（64/64、46/46 等）见本文件"变更记录"一节。

---

## 11. 相关路径

- 板端项目：`/userdata/GrandRDK`（本目录）；备份：`/userdata/backup/`
- 上位机：`D:\RC\ROV控制站_v3.3`（`pc_main2.py` / `protocol.py` / `README_zxq.md`）
- 下位机固件：`XLB-V1.0-Servo/firmware/JXZK_XLB_lib`（`Src/JXZK_XLB_Protocol.c` 是协议权威）
- 规范：`上位机对接说明_V2.md`（§4 命令 / §5 遥测 / §6 示例——**§6 深度示例有误**）
