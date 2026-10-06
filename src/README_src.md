# `src/` —— 源码目录说明

RDK S100 板端工程 `/userdata/GrandRDK/src/` 的全部源码：**两路相机采集 + BPU 推理 → 共享内存 → Web/UDP 对外**，外加中位机（控制/遥测）、超声波高度计，以及 2026-10-01 迁入的两路卡尔曼（`kalman/`）。

> **文档更新：2026-10-01**（两路卡尔曼按分类归位后重写；同日 R8 把测试代码全部迁出本目录）
> 本目录规模：自写 **约 9870 行** = 顶层 10 个模块 3226 + `utils/flow_share.py` 144 + `to32/` 中位机 **4004**（*不含*已迁出的测试脚本，原 4881）+ **`kalman/` 两路卡尔曼 2256**；另有 vendored 库 `utils/py_utils/` **2299 行**（第三方，**别改**）。
> **本目录不含任何测试代码**：原先散在 `to32/` 与 `kalman/depth_kalman/tests/` 的自检脚本已全部归口到 `hwless_tests/`（`legacy_to32/` + `legacy_depth_kalman/`，见 `hwless_tests/README_hwless_tests.md`）。各卡尔曼的 `run.sh --selftest/--mock` 仍可用，`TESTS` 变量已指向新落点。
> 上级说明见 `/userdata/GrandRDK/README.md`（整工程：端口表、启停、排障、变更记录）；中位机详见 **`src/to32/README.md`**；两路卡尔曼各有一份自己的 `README.md`。
> 若本文档与代码注释冲突，**以代码注释为准**并顺手改回这里。

---

## 30 秒速览

```
cam1 前视(USB 口 3-2) ─> front.py  ──┬─> momo_frame_front.bin  (JPEG)
                                     ├─> momo_det_front.json    (检测框)
                                     └─> momo_stats_front.json  (fps/耗时)   ──> web_server.py :5000 ──> Nginx :80

cam2 下视(USB 口 1-2) ─> bottom.py ──┬─> momo_frame_bottom.bin / det / stats ──> web_server.py
                                     └─> momo_flow_bottom.bin   (NV12)      ──> flow_speed.py  :8000

cam3 内窥镜(USB 口 1-1) ─> show_cam.py ──> :8084/stream（按需采集，空闲释放相机）

CH348 A–E 超声波 ─> read_altimeter.py ──> UDP $ALT :8082  +  momo_alt.json
                                              │
                                              └─> kalman/depth_kalman ──> momo_depth.json ──> to32/depth_if

momo_det_front.json ──> kalman/camera_kalman(viskf) ──> momo_viskf.json ──> to32/viskf_if
                                                                                  └─> mission（PASS_GATE 闭环）

上位机 ──$CMD :8080──> to32/main.py ──UART──> STM32；遥测 ──$TEL :8081──> 上位机；AUV 状态 ──$AUV :8085──> 上位机
   └─ mode=1 进 AUV ──> to32/kalman_launcher.py ──> 拉起上面两个卡尔曼（退出 AUV 时停掉自己起的）
```

**三条铁律**

1. **消费者只读共享内存，绝不直接开相机** —— 相机由 `front.py` / `bottom.py` 独占，抢设备会互相打掉。
2. **进程间只通过 `/dev/shm/momo_*` 打交道**（tmpfs，重启即清空，不落磁盘）。
3. **`utils/py_utils/` 是第三方 vendored 库，不要改**；要改行为在 `function.py` 或各进程里做适配。

---

## 一、目录总览

| 层 | 内容 | 行数 | 说明 |
|---|---|---|---|
| **顶层** | 10 个 `.py` | 3226 | 采集 / 推理 / 共享内存 / Web / 光流 / 高度计，见第二节 |
| `utils/` | `flow_share.py` | 144 | 自写，帧共享桥（给光流用） |
| `utils/py_utils/` | 7 个 `.py` | 2299 | **vendored 第三方库（别改）** |
| `to32/` | 20 个 `.py` + 4 个 `.md` | 4881 | 中位机：上位机/下位机双链路 + 模式层 + AUV 任务，见第四节 |
| **`kalman/`** | 2 个子工程 | **2495** | ★ 2026-10-01 迁入：深度卡尔曼 + 图像（视觉）卡尔曼，见第五节 |

---

## 二、顶层模块逐个说明

### 1. `front.py`（498 行）—— 前视进程

| 项 | 内容 |
|---|---|
| 相机 | `cam1` = USB 物理口 **3-2**（`config/camera_ports.py` 按物理口绑定，不写死节点号） |
| 职责 | 采集 → YOLO 找 `door` + `red-ball`（2026-09-26 加球，撞球阶段要靠前视找球）→ 叠字 → 写共享内存 |
| 写 | `momo_frame_front.bin`(JPEG)、`momo_det_front.json`、`momo_stats_front.json` |
| 架构 | **producer 线程采集 → `queue` → N 个 worker 并行推理**（`N_WORKERS`，见 `quick_config.py`） |
| 调度 | `run.sh` 用 `taskset` 绑 **CPU 0-2** |
| 参数 | `--device` / `--index` / `--no-show` / `--workers N` / `--frames N`（跑 N 帧就退，调试用）/ `--timing` `--no-timing` / `--log` `--no-log` |

> 2026-09-21 起已删除 `--virtual` 与 `make_frame()` 合成帧逻辑 —— **只读真实相机，不再有伪造帧**。

### 2. `bottom.py`（512 行）—— 下视进程

| 项 | 内容 |
|---|---|
| 相机 | `cam2` = USB 物理口 **1-2** |
| 职责 | 采集 → YOLO 找 `red-ball` → 写共享内存；**额外写一帧 NV12 供光流测速** |
| 写 | `momo_frame_bottom.bin` / `momo_det_bottom.json` / `momo_stats_bottom.json` / **`momo_flow_bottom.bin`(NV12)** |
| 调度 | `taskset` 绑 **CPU 3-5** |
| 参数 | 同 `front.py` |

> ⚠ **启动顺序硬约束：先 bottom，后 flow_speed**。写端重启后 seq 归零会被读端判成旧帧而**静默丢帧**；bottom 重启必须一并重启光流。

### 3. `function.py`（509 行）—— 检测与预处理核心库

被 `front.py` / `bottom.py` 共用，两节：

| 节 | 内容 |
|---|---|
| ① YOLO 检测器 | `YoloDetector` / `YoloDetect`，适配 **两种后端**：`hbm_runtime`（板端 BPU）与 `ultralytics`（开发机）。上层用法：`detector = YoloDetector(CFG['FRONT_YOLO']); detector.detect(frame, nv12=...)` |
| ② 水下图像预处理 | 颜色校正 + 高斯去噪 + CLAHE，**三个独立开关** |

> 依赖 `config/main_config.DEFAULT_CONFIG` 与 `utils/py_utils/{preprocess,postprocess}`。

### 4. `hw_camera.py`（148 行）—— USB MJPG 相机封装（JPU 硬解）

- `HwMjpgCamera(device, width, height, fps)`，**接口故意与 `cv2.VideoCapture` 对齐**（`.read()` / `.get()` / `.release()` / `.isOpened()`），可直接替换 `VideoCapture`。
- 底层：`ctypes` 调 `libs/libmjpg_hw.so` —— V4L2 抓 MJPG 原始帧 + **JPU 硬件解码**出 NV12，再一次 `cvtColor` 转 BGR。
- 出 BGR 或 NV12（`bottom.py` 要 NV12 给光流）。

### 5. `shm_writer.py`（85 行）/ 6. `shm_reader.py`（91 行）—— 共享内存读写

| 文件 | 谁用 | 内容 |
|---|---|---|
| `shm_writer.py` | `front.py` / `bottom.py` | 写 JPEG 帧 + det/stats JSON，`mmap` 零拷贝 |
| `shm_reader.py` | `web_server.py` 等消费者 | 读端，校验 `HDR_MAGIC` 后按 `HDR_SIZE` 偏移取数据 |

帧头格式（定义在 `config/main_config.py`）：

| 常量 | 值 | 说明 |
|---|---|---|
| `HDR_MAGIC` | `b'MFS1'` | 魔数，读端据此校验合法性 |
| `HDR_FMT` | `'<4sIIIQ'` | 魔数 4s + 3×uint32 + uint64 微秒时间戳，**共 24 B** |
| `HDR_SIZE` | `struct.calcsize(HDR_FMT)` | 读写双方按此偏移取数据 |
| `MAX_JPEG_BYTES` | 2 MB | 单帧 JPEG 上限，超出视为非法帧丢弃 |
| `MAX_JSON_BYTES` | 64 KB | 单个 det/stats JSON 上限 |

### 7. `web_server.py`（298 行）—— FastAPI 对外服务 `:5000`

监听 `0.0.0.0`（**不要改回 127.0.0.1**），读共享内存对外供流：

| 路由 | 说明 |
|---|---|
| `GET /cam1` / `GET /cam2` | 前视 / 下视 MJPEG 流（有节流） |
| `GET /api/status` | 两路检测 + 统计（REST） |
| `GET /api/detections` | 检测框列表 |
| `GET /healthz` | 健康检查 |
| `GET /` | 根路径 |
| `WS /ws/status` | WebSocket 状态推送 |

> 无命令行参数，端口与节流由 `config/` 决定。上位机占着 MJPEG 流时 uvicorn 优雅关闭会挂住，`stop.sh` 的超时后 SIGKILL 是**设计内兜底，不是 bug，别"修"**。

### 8. `flow_speed.py`（332 行）—— 光流测速

- 数据源：`momo_flow_bottom.bin`（NV12），由 `ENABLE_FLOW_SHARE_BOTTOM=True` 控制（前视默认不写，省内存）。
- 算法：**Mali GPU（OpenCL `UMat`）稀疏金字塔 LK 光流** —— 不占 BPU、不开相机，不影响 YOLO 帧率。
- 输出：图像平面速度 `px/帧` + `moving%`；`FLOW_CALIB_ENABLE=True` 时按针孔模型换算 `m/s`：`v = 位移(px/帧) × (range_m / focal_px) / dt`。
- 调试画面：`:8000`。参数：`--fps` / `--port`。
- 🔴 **`FLOW_CALIB_*` 是未标定的占位值**：输出的 m/s **只能看趋势，不能当真值**。

### 9. `show_cam.py`（406 行）—— 第三路相机 CAM3（按需采集）

- 相机：`cam3` = USB 物理口 **1-1**（内窥镜）。推流 `:8084/stream`（上位机 v3.3 默认取这个地址；`/cam3` 同义）。
- **按需采集**（2026-09-19 改造）：HTTP 常驻但**不开相机**；只有上位机真的连上取流才打开 `cam3`，最后一个客户端断开后空闲 `IDLE_RELEASE_SEC` 秒自动释放 V4L2 句柄。
- 无 argparse，端口/相机由配置决定。

### 10. `read_altimeter.py`（347 行）—— 超声波高度计（Modbus-RTU）

| 项 | 内容 |
|---|---|
| 硬件 | DYP-L08(-V3.0)，挂 **CH348 A–E 口**：A→`ttyCH9344USB0`、B→`USB1`、C→`USB2`、D→`USB3`、E→`USB4`（A–H 顺序对应 USB0–7） |
| 并发 | **每通道 = 一条独立线程 + 独立串口句柄 + 独立统计**，互不阻塞 |
| 输出 ① | UDP 推 `$ALT,<通道>,<mm>,<状态>#` → 上位机 **:8082** |
| 输出 ② | ★（2026-09-26 新增）原子写 `/dev/shm/momo_alt.json` —— **深度卡尔曼的唯一输入源**，默认就落，`--no-shm` 才关 |
| 降级 | 连续 **10 次**无应答 → 判定该口未接设备，降 **2 s** 低频探测；接上后自动恢复正常节奏 |
| 参数 | `--channels A,B,C,D,E` / `--baud 115200` / `--addr 0x01` / `--reg 0x0101` / `--interval 0.2` / `--pc-ip 192.168.127.100` / `--pc-port 8082` / `--no-udp` / `--shm-dir /dev/shm` / `--no-shm` |

⚠ **三个已知风险**

1. **与 STM32 共用同一颗 CH348 / 同一条 USB** —— 整条 USB 掉线，高度计和 STM32 一起挂。
2. **tty 编号按枚举顺序分配**，拔插换口后会漂移 → 上板先 `ls -l /dev/ttyCH9344*` 核对 `PORT_OF` 映射。
3. **5 探头同频并发读有"声学串扰"风险** → 上电先做「悬停静止 30 s，看 B−C 是否恒定、有无几十 cm 尖峰」验证，必要时各线程错开采样相位。

> 🔴 2026-10-01 实测：**A–E 五路仍全部 no-reply**（09-27 已确认探头物理上都插着、B/C 朝下）。既然五路**全**无应答，更像 Modbus 参数（`--addr` / `--reg` / 波特率）或供电问题，不像个别探头坏。

---

## 三、`utils/`

| 文件 | 行数 | 说明 |
|---|---|---|
| `flow_share.py` | 144 | **自写。** 帧共享桥：把检测进程解出的帧经 `/dev/shm/momo_flow_<name>.bin` 给光流进程，避免为测速再开第三颗相机。布局 `[header 32 B][数据区]`，header = magic `MFS1` + ver + seq + w + h + fmt(0=NV12/1=BGR) 共 24 B，**再紧跟一个独立 8 B 小端 ts_us**（合计 32 B）；并发用 `fcntl.flock`（写端 EX / 读端 SH）防撕裂。写一次约 0.1 ms，对 200 fps 采集的额外开销 **< 2% 单核** |
| `py_utils/` | 2299 | **第三方 vendored 库**（D-Robotics 官方 sample 工具，Apache-2.0）—— **不是本项目代码，别改** |

`py_utils/` 内部：`postprocess.py` 1043（解码/阈值/NMS/坐标映射，最大）、`visualize.py` 525（画框）、`inspect.py` 238、`preprocess.py` 248（resize/letterbox/格式转换）、`file_io.py` 176、`nn_math.py` 60、`__init__.py` 9。

> ⚠ **注意头部是两套机制，别混淆**：`shm_writer/shm_reader` 的帧头是 **24 B**（`<4sIIIQ`：magic/seq/length/wh/ts_us），`flow_share.py` 的光流帧头是 **32 B**（`HDR<4sIIIII>` 24 B + `HDR_TS<Q>` 8 B，多出 w/h/fmt）。两者都以 `MFS1` 开头但布局不同。
>
> 📌 **2026-10-01 修正**：此前文档与本文件注释均写 48 B，属笔误；`_HEADER_SIZE` 实测为 **32**，已同步改注释并在 `hwless_tests/test_02_flow.py` 里写成断言锁定。

---

## 四、`to32/` —— 中位机（4881 行，2026-09-21 迁入）

**扁平结构，不要拆子目录**（`selftest_modes.py` 会读同目录源码做守卫）。完整说明见 **`src/to32/README.md`**（线程模型、协议表、排障）。

| 文件 | 行数 | 职责 |
|---|---|---|
| `main.py` | 288 | 入口：装配双链路 + 模式层并常驻 |
| `link_pc.py` | 114 | 上位机链路：UDP :8080 收 `$CMD`/`$PID`/`$VID`/`PING`，:8081 发 `$TEL` |
| `link_stm32.py` | 296 | 下位机链路：串口（CH348 F 口 `ttyCH9344USB5`）发 `0x09` 控制帧、收 `0x0C` 48 字节遥测帧 |
| `protocol.py` / `tel_builder.py` | 84 / 76 | V2 协议组帧校验 / 拼 `$TEL` |
| `mode_base.py` | 90 | `ModeBase` 抽象基类，10 个回调 |
| `mode_rov.py` | 192 | ROV 手动模式：四轴直通；空闲静默 = 四轴全 0 才判回中 |
| `mode_auv.py` | 178 | ★ AUV 自主模式：20 Hz 驱动 `mission.step()`；`on_enter/on_exit` 托管**两路**卡尔曼与状态上报 |
| `mode_dispatcher.py` | 461 | 调度核心：模式切换与记忆、看门狗、急停锁存、抑制危险帧 |
| `mission.py` | 805 | ★ AUV 任务状态机，17 阶段（撞球→过门→捡球→触壁→上浮），`_st_<阶段名小写>` 由 `getattr` 分发 |
| `vision_if.py` / `depth_if.py` | 181 / 112 | ★ AUV 的视觉接口（`e_x`/可见性/`trust`）与深度接口（`D`/`v_z`/`clearance`/`sigma`） |
| `viskf_if.py` | 129 | ★（2026-10-01 新增）读 `momo_viskf.json`，给过门阶段供滤波后的 `e_x` / `s_n` |
| `kalman_launcher.py` | 350 | ★（2026-10-01 新增，取代并删除 `depth_launcher.py`）**两路**卡尔曼进程托管：幂等 `ensure_started()` + 只停自己起的 `stop()`；`DepthKalmanLauncher` + `ViskfLauncher` 各管一路，**互不影响** |
| `auv_report.py` | 271 | ★（2026-09-30 新增）独立线程 5 Hz 推 `$AUV` → 上位机 :8085，非阻塞，失败 3 次永久放弃 |
| `video.py` | 303 | 中位机自带图像回传，**2026-09-21 已停用**（`VIDEO_PATHS={}`、`VIDEO_ENABLED_AT_START=False`） |
| `port_probe.py` | 74 | 串口探测（CH348 A~H 通道确认）：往各口发 ASCII 标记，看 PC 助手收到哪个；**运维排障工具，非测试代码** |
| `*.md`（4 个） | — | `README.md`、`README_中位机.md`、`ROV_指令与V2协议对应关系.md`、`上位机通讯协议.md` |

> **2026-10-01（R8）**：本目录**不再有任何测试/自检脚本**。原 `selftest_modes.py` /
> `make_test_frame.py` / `test_v2_frames.py`（及 `kalman/depth_kalman/tests/`）已全工程归口
> 到 `hwless_tests/legacy_to32/`、`hwless_tests/legacy_depth_kalman/`。
> 因此上一行"扁平结构不要拆子目录"的说法**已不适用于测试脚本**（源码目录仍建议扁平，
> 但源码守卫已改为绝对路径指向本目录，见 `hwless_tests/README_hwless_tests.md` §九）。

---

## 五、`kalman/` —— 两路卡尔曼（2256 行，2026-10-01 迁入）

**按 GrandRDK 的分类拆开存放**（这是本节最重要的约定，改路径前先读）：

| 分类 | 落点 |
|---|---|
| **配置** | `/userdata/GrandRDK/config/depth_config.py`、`config/viskf_config.py` |
| **源码** | `/userdata/GrandRDK/src/kalman/<名字>/*.py` —— **平铺**，不再各套一层 `src/` |

⚠ **源码不摊到 `src/` 顶层**：`depth_kalman` 的主程序叫 `main.py`，而板端已有 `src/to32/main.py`，
摊平后 `pkill`/`grep` 的 `src/main.py` 会误伤中位机进程。所以统一收在 `src/kalman/<名字>/` 这一层。

⚠ **代价：PYTHONPATH 必须是两段** —— `<宿主>/config` **加上** `<工程根>`：

```bash
export PYTHONPATH="/userdata/GrandRDK/config:/userdata/GrandRDK/src/kalman/depth_kalman"
```

两个 `run.sh` 已经自己算好了（往上级 `../../..` 找宿主根）；**迁移后**各自的 `tests/` 已不在包内，
`run.sh` 的 `TESTS` 变量指向 `hwless_tests/legacy_depth_kalman/`（找不到时回退老位置 `./tests/`）。
**手动起进程忘了第一段 → `ModuleNotFoundError: depth_config`**（viskf 会静默退回内置默认值，更难查）。

| 子工程 | 行数 | 读 | 写 | 说明 |
|---|---|---|---|---|
| `depth_kalman/` | 1370 | `momo_alt.json`、`momo_telemetry.json` | `momo_depth.json` | 深度卡尔曼：EKF 融合深度计 + 两路朝下高度计 + 加速度，50 Hz |
| `camera_kalman/` | 886 | `momo_det_front.json` | `momo_viskf.json` | 图像卡尔曼（viskf）：**不是 EKF**，3 个并联的 2 维 CV KF（`e_x`/`el`/`s_n`），纯 stdlib，50 Hz 循环 / 20 Hz 输出 |

> **2026-10-01（R8）**：各自的 `tests/` 已**整体迁出包目录**（源码不引用它们），统一落在
> `hwless_tests/legacy_depth_kalman/`（4 个文件）；生产机上整个 `hwless_tests/` 可删，不影响运行。
> ⚠ `camera_kalman` 的 `tests/test_viskf.py` **从未存在**（`./run.sh --selftest` 会报缺文件），
> 是既有缺口，与本次迁移无关。

### 5.1 生命周期 —— 挂在 AUV 模式上，没有独立启停脚本

```
上位机 "$CMD mode=1"
   → mode_auv.on_enter → kalman_launcher 各路 ensure_started()   （幂等：已在跑就复用）
上位机切回 ROV / 退出
   → mode_auv.on_exit  → 各路 stop()                             （只停自己起的那个）
```

- 幂等三判据：自己起的进程活着 / `logs/*.pid` 活着 / 输出 JSON 的 mtime < 1.5 s。
- **两路互不影响**：一路起不来另一路照常跑，任务各自走降级路径，绝不外抛异常。
- ⚠ 手动 `./run.sh --daemon` 起的实例，AUV 退出时**不会**替你停（不抢别人的进程）。

### 5.2 三条"别乱动"的口径（都是踩过的坑）

1. **`e_x` 归一化口径必须恒定**（`cx−x0−dx0)/门框宽`），门框贴边也不许换成按画面宽 ——
   换口径会在切换处产生 0.209 的观测跳变（正常帧间变化的 50 倍），滤波抖动被放大 3.9 倍，最后外推 runaway。
2. **判"能不能用"看 `trust`，不看 `gate_visible`** —— 门一丢 `gate_visible` 立刻 False，
   但滤波器还会自由外推 0.5~2 s，实测这期间 `e_x` 飘到 2.38 / `sig_x` 2.40（正常 +0.60 / ≤0.50）。
3. **`AUV_GATE_CROSS_TOL` 必须 > 0** —— `0x09` 的 surge 是 int8×127（分辨率 ≈0.004），
   P 控制越近推力越小，小推力量化成 0 就永远等不到 `s_n ≥ s_star`，实测差 0.003 就能干等到 30 s 超时。

### 5.3 各自的回归自检（改完 config 必跑）

```bash
cd /userdata/GrandRDK/src/kalman/depth_kalman && ./run.sh --selftest   # → 11 个场景全达标
cd /userdata/GrandRDK/src/kalman/camera_kalman && ./run.sh --selftest  # → ⚠ tests/ 缺失（既有缺口，见上）
# 测试代码已迁到 hwless_tests/；也可直接批量回归：
python3 /userdata/GrandRDK/hwless_tests/run_legacy.py
```

---

## 六、共享内存契约（新增消费者必读）

全部键名定义在 `config/main_config.py`，根目录 `/dev/shm`：

| 键 | 文件 | 写端 | 读端 |
|---|---|---|---|
| `SHM_FRAME_FRONT` | `momo_frame_front.bin` | `front.py` | `web_server.py` |
| `SHM_DET_FRONT` | `momo_det_front.json` | `front.py` | `web_server.py`、**`to32/vision_if.py`** |
| `SHM_STATS_FRONT` | `momo_stats_front.json` | `front.py` | `web_server.py` |
| `SHM_FRAME_BOTTOM` / `SHM_DET_BOTTOM` / `SHM_STATS_BOTTOM` | 同名 `_bottom` | `bottom.py` | `web_server.py` |
| `SHM_FLOW_BOTTOM` | `momo_flow_bottom.bin`(NV12) | `bottom.py` | `flow_speed.py` |
| `SHM_TELEM` | `momo_telemetry.json` | — | 汇总两路的统一遥测 JSON |
| （外部） | `momo_alt.json` | `read_altimeter.py` | **`kalman/depth_kalman`** |
| （外部） | `momo_depth.json` | `kalman/depth_kalman` | **`to32/depth_if.py`** |
| （外部） | `momo_viskf.json` | `kalman/camera_kalman` | **`to32/viskf_if.py`** |

**新增消费者模板**：只 `import shm_reader`，读现成的 `momo_*.json/bin`，**不要碰 `hw_camera.py`、不要开 `/dev/video*`**。

---

## 七、进程模型与手动启动

| 进程 | 调度 | 出口 |
|---|---|---|
| `front.py` | `taskset` CPU **0-2** | 共享内存 |
| `bottom.py` | `taskset` CPU **3-5** | 共享内存 + 光流帧 |
| `flow_speed.py` | `nice 19` | `:8000` |
| `show_cam.py` | `nice 5` | `:8084` |
| `web_server.py` | `nice 10` | `:5000` |
| `read_altimeter.py` | `nice 5` | UDP `:8082` + `momo_alt.json` |
| `to32/main.py` | `nice 5` | UDP 8080/8081/**8085** + 串口 F 口 |
| `kalman/depth_kalman/main.py` | 由 AUV 模式拉起 | `momo_depth.json` |
| `kalman/camera_kalman/viskf.py` | 由 AUV 模式拉起 | `momo_viskf.json` |

**日常启停走工程根脚本**（`run.sh` / `status.sh` / `logs.sh` / `stop.sh`），不要手搓。
两路卡尔曼**不进 `run.sh`** —— 那套 wait/kill 逻辑会放大故障；它们由 AUV 模式托管（见 §5.1）。

**单进程手动调试**（必须自带 `PYTHONPATH`，否则 import 失败）：

```bash
cd /userdata/GrandRDK
PYTHONPATH=config:src:src/utils:src/to32 python3 src/front.py  --frames 100 --no-show
PYTHONPATH=config:src:src/utils:src/to32 python3 src/bottom.py --frames 100 --no-show
PYTHONPATH=config:src:src/utils:src/to32 python3 src/read_altimeter.py --channels B,C --interval 0.2
PYTHONPATH=config:src:src/utils:src/to32 python3 src/flow_speed.py --fps 30 --port 8000
PYTHONPATH=config:src:src/utils:src/to32 python3 src/show_cam.py
```

后台常驻用 `nohup` 并自行带 `taskset`/`nice`。

**单独调某一路卡尔曼**（不用进 AUV）：

```bash
cd /userdata/GrandRDK/src/kalman/depth_kalman  && ./run.sh --daemon   # 停：./stop.sh
cd /userdata/GrandRDK/src/kalman/camera_kalman && ./run.sh --daemon   # 停：./stop.sh
```

（新布局下不用手给 PYTHONPATH，`run.sh` 会往上找到宿主的 `config/`。）

---

## 八、排障速查

| 现象 | 常见原因 | 处理 |
|---|---|---|
| `真实相机打开失败` | 相机没插 / 节点被别的进程占着 | `v4l2-ctl --list-devices` 核对；`fuser <设备节点>` 查占用；按物理口核对 `config/camera_ports.py` |
| `[jpu] too many bad jpeg frames` | 相机节点被占 / 节点不对 | 同上（节点号会随枚举漂移，**必须按物理口绑定**） |
| `/cam1` `/cam2` 黑屏 | 写端没起来，`/dev/shm` 无数据 | 先看 `logs/front.log` / `logs/bottom.log` |
| 光流一直 `moving=0%` | 先起 flow_speed 后起 bottom，写端 seq 归零被判旧帧 | **先 bottom，后 flow_speed**；bottom 重启后一并重启光流 |
| 光流 m/s 不对 | `FLOW_CALIB_*` 是占位值 | m/s 只能看趋势；待标定 |
| 高度计 `[!!] 读取失败(no-reply)` | 该口没接上 / 参数不对 | 自动降 2 s 探测；五路全无应答 → 查 `--addr`/`--reg`/波特率与供电 |
| 高度计与 STM32 同时掉线 | 共用同一颗 CH348 / 同一条 USB | 查 `lsusb` 与 `ls -l /dev/ttyCH9344*` |
| `momo_alt.json 落盘失败 FileNotFoundError` | `/dev/shm` 不可写 / 被改了 `--shm-dir` | `ls -ld /dev/shm`；落盘失败**不影响 UDP 推送与采集** |
| 整栈突然全停 | `run.sh` 在检测进程退出时会收尾杀全部子进程 | 先看 `logs/front.log` / `logs/bottom.log` 谁先挂（多为相机掉线），修根因再启动 |
| `stop.sh` 后进程还在 | 上位机占着 MJPEG 流，uvicorn 优雅关闭会挂住 | `stop.sh` 带宽限期后 SIGKILL 兜底，属**已知设计** |
| `ModuleNotFoundError: depth_config` / viskf 报找不到配置 | 手动起卡尔曼时 PYTHONPATH 少给了宿主 `config/` 那一段 | 按 §五 给两段；或直接用 `./run.sh`（已处理） |
| 卡尔曼日志写到 `src/kalman/logs/` 去了 | `main.py` 的 `ROOT` 还按老布局取两层 dirname | 源码已平铺，`ROOT` 只能取一层 |
| 进了 AUV 但 `momo_depth.json` 不更新 | 高度计没数（A–E 全 no-reply）或卡尔曼没起来 | `./run.sh --selftest` 先证代码没坏；再看 `logs/depth.log` |
| 过门时机器人贴着框不动 | `AUV_GATE_CROSS_TOL ≤ 0`，surge 被 int8 量化成 0 | 该值必须 > 0（默认 0.02） |

---

## 九、变更记录（本目录相关）

| 日期 | 变更 |
|---|---|
| 2026-09-17 | `flow_speed.py` 由 `/userdata/momo_pwmnet` 迁入；Web 监听改 `0.0.0.0`；删除 legacy 假 `$TEL` |
| 2026-09-18 | `show_cam.py` 迁入本目录，纳入 `run.sh/stop.sh/status.sh/logs.sh` 统一托管 |
| 2026-09-19 | `show_cam.py` 改为**按需采集**：HTTP 常驻但不开相机，取流才开、空闲自动释放 |
| 2026-09-21 | **中位机 `to32/` 迁入本目录**；下视改 cam2、CAM3 保留 cam3，解除节点冲突；删除 `--virtual` / `make_frame()` 合成帧；停用中位机自带图像回传 |
| 2026-09-23 | 相机节点改按 **USB 物理口绑定**（`config/camera_ports.py`）；移除前视单目测距（`src/range.py`） |
| 2026-09-25 | `read_altimeter.py` 新增 `momo_alt.json` 落盘（卡尔曼输入源）；`to32/` 增加 AUV 四件套（`vision_if`/`depth_if`/`mission`/`auv_report`） |
| 2026-09-26 | `front.py` 目标加 `red-ball`（撞球阶段靠前视找球）；`mission.py` 17 阶段状态机定稿 |
| 2026-09-30 | `to32/auv_report.py` 新增：5 Hz 推 `$AUV` → 上位机 :8085 |
| 2026-10-01 | ★ **两路卡尔曼迁入 `kalman/` 并按分类拆开**：配置进 `config/`（`depth_config.py` / `viskf_config.py`），源码平铺在 `src/kalman/<名字>/`。`to32/depth_launcher.py`(221) 被 `kalman_launcher.py`(350) 取代并删除 —— 一路托管变**两路**；新增 `to32/viskf_if.py`(129)；`mission.py` 715→805（过门闭环接 viskf）；`mode_auv.py` 168→178。本目录自写 7893 → **10746 行** |
