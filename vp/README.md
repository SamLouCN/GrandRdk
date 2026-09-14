# vp5.1 — 双相机双进程 YOLO 检测

RDK S100 (80TOPS) 板端实现：**两个独立进程（front / bottom）并行跑 YOLO 检测**，
每个进程绑定 3 个 CPU 核并开启多线程，回归最原始的检测输出逻辑
（默认无日志、无计时，仅输出检测结果）。

## 版本记录（更新时间线，新→旧）

| 日期 | 版本 | 改动摘要 | 详见 |
|---|---|---|---|
| 2026-09-09 | **光流测速改用下视（bottom）** | 光流测速输入从前视切到下视：`flow_speed.py` 固定读 `/dev/shm/momo_flow_bottom.bin`（下视），`run.sh`/注释同步；`main_config.py` 两个开关解耦——`ENABLE_FLOW_SHARE`（front 写，默认 False，省一路 memcpy）、`ENABLE_FLOW_SHARE_BOTTOM`（bottom 写，默认 True，光流测速数据源）。回滚：`ENABLE_FLOW_SHARE=True` + `ENABLE_FLOW_SHARE_BOTTOM=True` 恢复双写，或改 `flow_speed.py` 读回 `bottom`→`front` | 「帧共享桥（FLOW_SHARE）」章节 |
| 2026-09-09 | **vp5.1 文档同步复查（仅文档）** | 对照代码修正 README：config 当前值（ENABLE_TIMING=True / TIMING_INTERVAL=100 / SIMPLE_TIMING=True / ENABLE_LOG=False / SHOW=False，STREAM fps=60·q100、相机 fps 请求 250）、模型软链真实路径 `/userdata/vp/vp4.5/`、目录结构补全 STREAM/TEL/FLOW_SHARE 相关文件 | 本文档各章节 |
| 2026-09-07 | **vp5.1 联合改动 v4（GPU 光流测速）** | 光流测速整链路迁 Mali-G78AE GPU（OpenCL/UMat）：`flow_speed.py` 由「CPU DIS 光流 + BPU 深度 m/s」重构为「GPU 稀疏金字塔 LK + px/帧 + moving%」；`run.sh` 启动行加 `nice -n 19` + `OMP/OPENBLAS/MKL=1`（不抢 front/bottom 核），`FLOW_FPS` 默认 20→50。YOLO 双进程绑核（front 0-2 / bottom 3-5）与 BPU 配置零改动。回滚：`/userdata/momo_pwmnet/flow_speed_cpu.py.bak` 还原 + `run.sh` 去掉 nice/改回旧启动行 | 「帧共享桥（FLOW_SHARE）」章节 |
| 2026-09-06 | **vp5.1 联合改动** | 光流只用前视摄像头：`main_config.py` 新增 `ENABLE_FLOW_SHARE_BOTTOM=False`（bottom 默认不再写共享帧，省一路 memcpy）；光流进程 `/userdata/momo_pwmnet/flow_speed.py` 移除 `--cam` 参数、固定读 front 共享帧；两侧 README 同步。回滚：`ENABLE_FLOW_SHARE_BOTTOM=True` 恢复双写 | 「帧共享桥（FLOW_SHARE）」章节 |
| 2026-09-07 | **vp5.1 联合改动 v2** | `run.sh` 同步启动光流测速：默认一并拉起 `/userdata/momo_pwmnet/flow_speed.py`（Web 推流 `:8080`、抽帧 20fps，与图像同启同停）；`--virtual` 模式自动跳过；`FLOW_PORT` / `FLOW_FPS` 环境变量覆盖端口/抽帧率。回滚：在 `run.sh` 中删除光流测速启动块（详见「帧共享桥（FLOW_SHARE）」章节使用与验证） | 「帧共享桥（FLOW_SHARE）」章节 |
| 2026-09-07 | **vp5.1 联合改动 v3** | 简化时间统计开关：`main_config.py` 新增 `SIMPLE_TIMING`（默认 False）；`front.py`/`bottom.py` 的 `StageStats` 支持 `simple` 模式（只计帧数、不累加各阶段耗时），滚动统计只输出「帧数 / 运行时间 / 平均帧率」。回滚：`SIMPLE_TIMING` 改回 False 即恢复原行为 | 「时间统计功能（ENABLE_TIMING）」章节 |
| 2026-09-06 | vp5.1 | 帧共享桥 FLOW_SHARE：front/bottom 把最新帧写入 `/dev/shm/momo_flow_*.bin`，供独立光流测速进程读取（免第三相机） | 「帧共享桥（FLOW_SHARE）」章节 |
| 2026-09-03 | vp5.0 | STREAM 上位机推流（TCP 9000/9001 → HTTP :5000）、TEL 遥测滚动上传（UDP 8081、10Hz），均并入 run.sh 同一生命周期 | 「上位机推流（STREAM）」「遥测滚动上传（TEL）」章节 |
| 2026-09-02 前 | vp4.6A | 双相机双进程 YOLO 检测基线：JPU 硬解 / NV12 直通 / 每 worker 独立 BPU 实例 / 绑核并发 | 本文档主体 |

## 目录结构

```
/userdata/vp/vp5.1/
├── run.sh              # 启动脚本：同时拉起 front / bottom 两进程，各绑 3 核
├── front.py            # 前视进程入口（FRONT_CAMERA + FRONT_YOLO）
├── bottom.py           # 下视进程入口（BOTTOM_CAMERA + BOTTOM_YOLO）
├── main_config.py      # 配置文件（所有开关/参数集中管理）
├── function.py         # 算法函数库（YoloDetector / LineDetector / draw_detections 等）
├── hw_camera.py        # JPU 硬件解码封装（HwMjpgCamera，接口对齐 cv2.VideoCapture）
├── libmjpg_hw.so       # JPU 硬解 .so（V4L2 抓 MJPG + hb_media_codec 硬解 → NV12）
├── streamer.py         # 画框帧 TCP JPEG 推流线程（front :9000 / bottom :9001）
├── mjpeg_bridge.py     # TCP JPEG → HTTP MJPEG 桥（:5000 /cam1 /cam2，供上位机取流）
├── telem_sender.py     # 遥测滚动上传独立进程（UDP 8081，run.sh 拉起）
├── utils/
│   ├── py_utils/       # YOLO 预处理/后处理工具（resize / NV12 / DFL 解码 / NMS）
│   └── flow_share.py   # 帧共享桥（/dev/shm/momo_flow_bottom.bin，bottom 写 / 光流进程读）
├── result/log/              # 帧日志输出目录（ENABLE_LOG=True 时自动创建）
│   ├── front.log            # 前视每帧处理结果（一行一帧，文本）
│   └── bottom.log           # 下视每帧处理结果
└── test_nashe_640x640_nv12.hbm   # 模型软链 → /userdata/vp/vp4.5/test_nashe_640x640_nv12.hbm
```

> 目录树外另有 `run.sh.bak_20260903`（run.sh 旧版备份）与 `__pycache__/`（运行缓存，可忽略）。

## 架构：两进程 · 三线程组 · 每 worker 独立 BPU 实例

```
run.sh
├── taskset -c 0-2  python3 front.py     # front 进程：绑 CPU 0-2（3 核）
│   ├── 采集线程(producer)   : cap.read() / 虚拟帧合成 → 有界队列 (fid, frame, nv12)
│   ├── worker×N_WORKERS      : 每线程独立 YoloDetector 实例 → BPU 推理 → 输出/写日志
│   └── 显示线程(display)    : imshow + waitKey（SHOW 开启时）
└── taskset -c 3-5  python3 bottom.py    # bottom 进程：绑 CPU 3-5（3 核）
    ├── 采集线程(producer)
    ├── worker×N_WORKERS（每线程独立 detector 实例）
    └── 显示线程(display)
```

关键设计决策：

1. **两进程独立**：front / bottom 各用自己的相机（config 中 `FRONT_CAMERA.device` /
   `BOTTOM_CAMERA.device`），互不共享、互不影响。任一相机掉线，**只停它自己的进程**，
   另一路继续跑。
2. **每 worker 独立 YoloDetector 实例**：实测多个线程共享同一个 BPU 模型实例会
   触发并发死锁（全部线程 futex 等待）；改为每 worker 独立实例后 3 路并发推理正常
   （单帧 ~0.04s，3 worker 并发总耗时 ~0.11s/5帧×3）。config 注释中的
   "多线程多实例并行，榨 BPU 算力" 即此方案。
3. **绑核 + 多线程**：S100 共 6 个 A78AE 大核，front 绑 0-2、bottom 绑 3-5；
   进程内 1 采集线程 + N_WORKERS worker + 1 显示线程，并用
   `OMP_NUM_THREADS=3` 等环境变量让 OpenCV/numpy 内部也并行。
4. **N_WORKERS 可配置**：worker 数量由 `main_config.N_WORKERS` 控制（默认 3），
   也可用 CLI `--workers N` 临时覆盖。线程数对效率有上限——详见末尾"瓶颈与可优化点"。
5. **有界队列背压**：采集线程按 `CAMERA_QUEUE_SIZE`（默认 12）有界入队，
   队列满丢当前帧不阻塞采集，保证延迟恒定。
6. **掉线即停**：相机打开失败 / 连续 5 次读帧失败（select 超时）→ 打印一行提示后
   该进程退出（return 1），不降级、不卡死、不拖累另一路。

## 配置文件说明（main_config.py）

所有开关集中在 `DEFAULT_CONFIG`，front/bottom 入口逐项联动。代码读取默认值
（括号中标"默认"），config 中可改：

| 字段 | 代码默认 | 作用 |
|---|---|---|
| `ENABLE_FRONT_CAM` / `ENABLE_BOTTOM_CAM` | True | 各路相机总开关 |
| `ENABLE_FRONT_YOLO` / `ENABLE_BOTTOM_YOLO` | True | 各路 YOLO 开关 |
| `FRONT_CAMERA` / `BOTTOM_CAMERA` | — | device / index / width / height / fps / format(MJPG) / backend / **hardware_decode** / mark_point |
| `FRONT_YOLO` / `BOTTOM_YOLO` | — | backend(hbm) / model_path / score_thres / nms_thres / strides / priority / bpu_cores / class_names / target_class_names |
| `N_WORKERS` | 3 | 每进程检测 worker 线程数；CLI `--workers N` 可临时覆盖 |
| `CAMERA_QUEUE_SIZE` | 12 | 每路采集队列容量（有界，满丢最旧） |
| `LOOP_SLEEP` | 0.001 | 主循环 sleep，让出 GIL |
| `SHOW` | True | 是否开 imshow 显示窗口 |
| `ENABLE_TIMING` | False | 时间统计/帧率计时开关（见下） |
| `TIMING_INTERVAL` | 30 | 每 N 帧打印一次滚动统计 |
| `SIMPLE_TIMING` | False | 简化时间统计（仅在 `ENABLE_TIMING=True` 时生效）：True = 滚动统计只打帧率，不做采集/检测/显示各阶段计时与输出；False = 原有完整统计 |
| `ENABLE_LOG` | False | 帧日志保存开关：每帧处理结果写入 `./result/log/` |
| `LOG_DIR` | `result/log` | 日志目录（相对 main 目录；每进程一个文件 front.log / bottom.log） |

> 注意：当前板子上的 `main_config.py` 里：`ENABLE_TIMING=True`、`TIMING_INTERVAL=100`、
> `SIMPLE_TIMING=True`、`ENABLE_LOG=False`、`SHOW=False`。
> 实际生效：运行会打印**简化**滚动统计（只含帧数 / 运行时间 / 平均帧率，见下方
> "简化模式"）与结束统计，**不落盘、不弹窗**。`result/log/` 下的 `front.log` /
> `bottom.log` 为历史运行残留，`ENABLE_LOG=False` 时不再追加新行。
> 若想保持"最原始检测输出"（终端零统计），把 `ENABLE_TIMING` 与 `SIMPLE_TIMING`
> 都改回 `False` 即可（`ENABLE_LOG` / `SHOW` 本来就是关的）。
> 文档表格里的"代码默认"指的是代码中 fallback 时的值，不一定是 config 当前值。

## JPU 硬件解码（hardware_decode）

> 背景：MJPG 相机的 CPU 软解（cv2 内部 libjpeg）是采集链路最大 CPU 开销之一，
> 早前 README"瓶颈与可优化点"列过"JPU 硬解"方向，本版已落地。

**选型结论（重要）**：先尝试了社区开源库 **BCDL**（ruisv/bcdl，forum 35346，
conda 预编译包 `bcdl=0.7.1`，已装于 `/root/miniconda3/envs/bcdl`），但其
`JpegDecoder`/`jpeg_decode` 的 JPU 硬解路径**只支持 4:2:0 采样的 JPEG**；本机
LRCP S400 相机（MJPG 640×480@200fps）实际输出 **4:2:2（Y 采样 2×1）**，硬解被
`RuntimeError: ... supports only 4:2:0 JPEGs ... got 2x1` 拒绝，且 `VideoDecoder`
仅支持 H264/H265、`out_format` 只读——**BCDL 路线对这台相机不可行**。

因此沿用 vp4.5 已验证的**官方 hb_media_codec JPU 硬解封装**（等价于 BCDL
JpegCodec 思路、官方栈实现，能解 4:2:2 帧）：

| 文件 | 说明 |
|---|---|
| `libmjpg_hw.so` | C 库：V4L2 mmap 抓原始 MJPG 帧（不经 OpenCV）→ `hb_mm_mc_*` JPU 硬解 → NV12。依赖 `/usr/hobot/lib` 的 `libmultimedia/libhbmem/libalog` |
| `hw_camera.py` | ctypes 封装 `HwMjpgCamera(device, width, height, fps)`，接口对齐 `cv2.VideoCapture`：`read()→(ok, bgr)` / `isOpened()` / `get()` / `release()`，另有 `grab_raw_nv12()` 供 NV12 直通 |

**接线方式**（改动收敛在"解码方式"，producer/worker/检测/日志/计时全部未动）：

- `main_config.py`：`FRONT_CAMERA` / `BOTTOM_CAMERA` 各加 `'hardware_decode': True`
- `front.py` / `bottom.py` 的 `open_camera(device, index)`：`hardware_decode=True`
  时先尝试 `HwMjpgCamera`（打印 `启用 JPU 硬件解码`），**打开失败/异常自动回退
  原 `cv2.VideoCapture` 软解**（打印 `回退 cv2 软解`），相机链路不中断
- 单路禁用：把对应相机配置的 `hardware_decode` 改为 `False`，或 CLI `--device` 指定
  其它路径（仍走硬解）；纯软解联调可临时改配置后运行

**性能口径（vp4.5 实测，同板同相机）**：JPU 纯解码 0.574ms/帧 ≈ 1742FPS，
cv2 软解 486FPS（约 3.6×）；实时链路 200FPS 打满相机上限；采集阶段 2.46ms/帧
（软解 3.0ms）。JPU 与 cv2 解码像素差 mean≈7.29（JPEG 解码器量化差异，可接受）。

**已知注意**：
- 本板 USB 相机（LRCP S400）存在热插拔抖动（dmesg `device descriptor read error
  -110`、节点 `/dev/video*` 随之时有时无），会导致打开瞬间 busy/节点消失；
  与硬解代码无关（软解同样会失败），属物理链路问题，优先查线缆/供电/USB 口。
- 退出释放时可能出现 `dequeue_out failed` 提示，是 `--check` 类立即释放的清理噪音。
- 模型软链 `test_nashe_640x640_nv12.hbm` 指向 vp4.5；打包（tar.gz）默认保留软链，
  不内嵌模型本体，换机部署需一并带上
  `/userdata/vp/vp4.5/test_nashe_640x640_nv12.hbm` 或改软链。

## NV12 直通预处理（2026-09-02 新增；GPU 分支 2026-09-03 已移除）

> 结论不变：**S100 的 Mali-G78AE GPU 用 OpenCV UMat 做这套 640×480 小图预处理，
> 实测反而慢 ~10 倍**（见下），GPU 加速分支已于 2026-09-03 从代码中整体移除
> （`gpu_accel` / `use_gpu` / `cv2.UMat` / OpenCL 相关），保留的只有
> 「JPU 解出 NV12 → 直接 letterbox 喂 BPU」的 **NV12 直通预处理**，
> 把老链路两次颜色转换省掉，预处理 CPU 减半。

### GPU（OpenCV UMat / Mali）实测 —— 路线被数据否定（历史记录，代码已移除）

以下为移除前的实测记录，用于说明为何否决该路线、避免后人重试：
S100 确有 Mali-G78AE GPU，OpenCV 4.11 带 OpenCL 且能识别到（backend =
`Mali-G78AE r0p1`）。但对 640×480 逐帧小图，host↔GPU 搬运与核调度开销远大于并行收益：

| 操作（640×480 帧） | CPU (Mat) | GPU (UMat) | GPU/CPU |
|---|---|---|---|
| `cv2.resize` 单次 | 0.09 ms | 1.63 ms | ~18× 慢 |
| `cv2.cvtColor` 单次 | 0.13 ms | 0.87 ms | ~7× 慢 |
| 完整 NV12 letterbox（Y/UV 三平面链式，单次上传/下载） | 0.31 ms | 3.25 ms | ~10× 慢 |

整条 `detect()` 链路（含 BPU 推理）实测（合成红球帧直连 BPU）：
BGR 老路径 **3.86 ms/帧**，NV12 直通 **3.64 ms/帧**，NV12+GPU(Mali) **6.74 ms/帧**。
GPU 路径虽功能可用但更慢，故该分支已整体移除。

### 实际优化：NV12 直通预处理（`preprocess_mode='auto'` 默认生效）

老链路每次白白做两次颜色转换 + 一次整图 resize：

`JPU 解 NV12 → cvtColor 转 BGR → BGR letterbox resize → cvtColor 转 YUV420 再拆 Y/UV` ≈ 0.62 ms/帧

新链路：

`JPU 解 NV12 → 对 Y/UV 平面直接 letterbox` ≈ 0.31 ms/帧

检测结果与老路径一致（合成帧验证：`red-ball` 中心均为 `(53,249)`、conf 0.84）。

改动文件：
- `hw_camera.py`：新增 `_grab()`/`grab_raw_nv12()`/`read_both()`，单次抓帧即可
  NV12 直出，避免重复转 BGR。
- `utils/py_utils/preprocess.py`：新增 `preprocess_nv12()`（CPU 实现；
  UMat/Mali 对比分支已于 2026-09-03 移除）。
- `function.py`：`YoloDetector.detect(frame, nv12=None)` 支持 NV12 直通，
  读 `preprocess_mode` 开关；`YoloDetect.pre_process/predict`
  增加 NV12 输入分支。
- `front.py` / `bottom.py`：队列项改为 `(fid, frame, nv12)`；producer 按
  `prefer_nv12`/`want_bgr` 在 `grab_raw_nv12()`（纯 NV12）/ `read_both()`（BGR+NV12）/
  `cap.read()`（BGR）间选择。
- `main_config.py`：两路 YOLO 配置各加两个开关。

### 开关

| 字段（`FRONT_YOLO` / `BOTTOM_YOLO` 内） | 默认 | 作用 |
|---|---|---|
| `preprocess_mode` | `'auto'` | `'auto'`：JPU 硬解有 NV12 时直通，软解回退/虚拟相机走 BGR；`'nv12'`：强制 NV12（无 NV12 源回退 BGR）；`'bgr'`：强制老 BGR 路径 |

- `hardware_decode=True` + `preprocess_mode='auto'` + `SHOW=False`：采集线程只调
  `grab_raw_nv12()` 拿 NV12（全链路不再产生 BGR）。
- `SHOW=True` 时 producer 改用 `read_both()`（一份 BGR 供画框 + 一份 NV12 供模型），只多一次转换。
- 软解回退 / 虚拟相机无 NV12 源：自动走原 BGR 路径，行为与 vp4.6 一致。

## 使用方式

```bash
cd /userdata/vp/vp5.1

# 正常双相机运行（无限，Ctrl-C 结束两个进程；默认走 JPU 硬件解码）
./run.sh

# 跑固定帧数后自动退出（验收用）
./run.sh --frames 90

# 开启时间统计（相机运行时间/帧率 + 各阶段耗时，固定窗口增量，不会虚增）
./run.sh --timing --frames 90

# 关闭时间统计（保持"最原始检测输出"，终端只剩目标行）
./run.sh --no-timing

# 无显示窗口模式（headless / SSH / VNC 无桌面时）
./run.sh --no-show

# 无实体相机联调（两路都用合成帧，自动 cap.read 失败不卡死）
./run.sh --virtual --frames 90

# 开启帧日志保存（每帧处理结果写入 ./result/log/）
./run.sh --log --frames 90

# 临时改 worker 数量（不改 config）
./run.sh --workers 4 --frames 90

# 单独指定某路相机设备
FRONT_DEVICE=/dev/video0 BOTTOM_DEVICE=/dev/video2 ./run.sh
```

单路入口也支持：`python3 front.py --timing --frames 90`（参数同 run.sh 透传）。

启动后应看到每路打印一次 `[*] [front/bottom] 相机 /dev/videoX: 启用 JPU 硬件解码`，
随后是 YOLO 检测行（开启计时时还有滚动统计）。若打印的是 `[警告] ... 回退 cv2 软解`
则说明硬解打开失败走了软解（查相机节点是否稳定/硬件解码是否可用）。

## 检测输出格式（终端）

```
[front] #0 red-ball 中心=(42,240) 相对标记点=(-278,+0) conf=0.85
[bottom] #0 yellow-ball 中心=(600,202) 相对标记点=(+280,-38) conf=0.90
```

- `#N`：该进程内检测序号
- `中心=(x,y)`：目标中心像素坐标
- `相对标记点=(dx,dy)`：目标中心相对相机标识点（config `mark_point`）的偏移，
  正 x = 目标偏右，正 y = 目标偏下
- `conf`：置信度（已按 `score_thres` 过滤，`target_class_names` 过滤类别）

**默认只在检测到目标时打印这一行**——如果该帧无检测目标，无输出。要让终端每帧都
"发声"（含空帧），见下面的帧日志；要在终端看统计行（帧率/耗时），开启
`ENABLE_TIMING` 或 `--timing`。

## 帧日志保存（ENABLE_LOG）

默认关闭（不落盘）。开启后每个进程把**每一帧**的处理结果以文本追加写入
`./result/log/front.log` / `./result/log/bottom.log`（目录自动创建），一行一帧：

```
[11:43:23][0.825s][frame:0] [done]: red-ball conf=0.85 bbox=(4, 204, 79, 277) center=(42, 240)
[11:43:24][2.050s][frame:40] [done]: none
[11:43:25][3.120s][frame:85] [dropped]: dropped(queue_full)
[11:43:26][4.001s][frame:120] [no_yolo]: no_yolo
[11:43:27][4.102s][frame:121] [no_data]: no_data
```

字段说明：
- `[hh:mm:ss]`：本地时间（时分秒）
- `[运行时间]`：从进程启动到本帧的秒数，**三位小数**（`{elapsed:.3f}s`）
- `[frame:N]`：帧号（从 0 开始，worker 取队列里 `(fid, frame, nv12)` 元组的 fid）
- `[status]`：处理状态
  - `done`：worker 正常处理了一帧（无论是否检出目标都会写一行）
  - `no_data`：真实相机读帧失败（瞬时失败；失败帧也占 fid，保证帧号连续）
  - `dropped(queue_full)`：producer 队列满丢弃，由 producer 端如实写一行
  - `no_yolo`：YOLO 开关关闭时跳过检测
- 冒号后为检测结果：多个检测用 `; ` 分隔，每个含
  `label`（类别名）/ `conf`（置信度）/ `bbox`（[x1,y1,x2,y2]）/ `center`（中心点）；
  该帧无检测时为 `none`

**如实写入**：worker 每处理一帧就调用 `FrameLog.write(fid, dets, status)`，
**与该帧是否检测到目标无关**——无目标的帧会写 `[done]: none`，不会因为"没目标"
就跳过。区别于终端 `print(format_detection(dets))` 只在有目标时输出。
多 worker 并发写同一文件用锁串行化，保证每行完整不交错；文件以追加模式打开，
多次运行结果累积。可用 `--log` / `--no-log` 在命令行覆盖 config 开关。

## 时间统计功能（ENABLE_TIMING）

> 历史 Bug 修复记录：早期版本的 `format_timing` 用**进程累计耗时 ÷ 当前窗口帧数**
> 计算各阶段 ms/帧，分子全程累计、分母是当前窗口——导致数值随运行时间线性膨胀，
> 给人"检测越来越慢"的错觉。已改为窗口增量口径。

代码默认 `False`（保持最原始检测输出，零日志零计时开销）。开启后输出：

- **滚动统计**（每 `TIMING_INTERVAL` 帧）：当前窗口帧数 / 运行时间 / 平均帧率 /
  采集 ms/帧 / 检测 ms/帧 / 显示 ms/帧
- **结束统计**：进程总运行时间 / 总帧数 / 总检测帧 / 平均帧率

### 简化模式（`SIMPLE_TIMING=True`，2026-09-07 新增）

仅在 `ENABLE_TIMING=True` 时生效（**板上当前已开启**，见上）。开启后：
- `StageStats(simple=True)` 跳过各阶段秒数累加（只保留帧数计数）；
- 滚动统计行只打印「帧数 / 运行时间 / 平均帧率」，**不输出**采集/检测/显示各阶段 ms/帧；
- 结束统计不变（总运行/总帧数/总检测帧/平均帧率）。

适用场景：长期运行、只需看吞吐（fps）而不关心各阶段耗时分布时；可省去逐阶段 perf_counter 调用与日志行长度。

```bash
# main_config.py（板上当前 True）
'SIMPLE_TIMING': True   # True=只记录帧率, 不记录各阶段耗时; False=原有完整统计

# 滚动统计输出对比（同一段 --virtual --frames 250 限帧跑）
# SIMPLE_TIMING=False（原行为）
# [front] 时间统计@100帧: 帧数=100 运行=2.56s 平均帧率=39.1fps | 采集=8.73ms/帧 检测=4.55ms/帧 显示=0.00ms/帧
# SIMPLE_TIMING=True（简化）
# [front] 简化时间统计@100帧: 帧数=100 运行=2.56s 平均帧率=39.0fps
```

三个计时点（代码中均有注释说明测什么任务）：

| 计时点 | 位置 | 测什么 |
|---|---|---|
| 采集阶段 | `producer()` 内 `cap.read()`/`make_frame()` 前后 | 相机取一帧的耗时（USB 解码/合成链路） |
| 检测阶段 | `worker()` 内 `detector.detect()` 前后 | 完整 YOLO 链路（预处理+BPU 推理+后处理） |
| 显示阶段 | `worker()` 内画框+入显示队列前后 | 可视化开销对主链路的挤占 |

**统计口径（窗口增量，已修复）**：
- `StageStats.mark_window()` 在 `main()` 启动时记录基准（_base_capture/detect/display）
- producer 每打印一次统计行，调用 `delta_window()` 取两次快照之间的增量
- `format_timing(elapsed, frames, d_capture, d_detect, d_display, title)` 用
  `窗口增量 / 窗口帧数` 计算每阶段单帧耗时——**数值稳定反映"当前"各阶段负担**，
  不会随时间虚增
- 帧率 `fps = 窗口帧数 / 窗口墙钟`，始终是真实吞吐

> 注：hardware_decode=True 时，采集计时覆盖的是 V4L2 抓帧 + JPU 硬解 + NV12→BGR
> 全链路的耗时（`HwMjpgCamera.read()` 内部），与软解时的 `cap.read()` 同口径。

## 相机掉线处理（两路独立）

| 场景 | 行为 |
|---|---|
| 相机打开失败（`isOpened()=False`） | 打印 `真实相机打开失败: device=... index=... 进程退出`，该进程 return 1 |
| 运行中读帧连续 5 次失败（select 超时） | 打印 `相机掉线/无数据(select超时)，本进程退出`，置 stop 事件退出 |
| 另一路相机正常 | 不受影响，继续检测输出 |

> 说明：USB 相机掉线通常伴随 dmesg 中 `Failed to set UVC probe control : -71` /
> `USB disconnect`（线缆、供电或带宽问题）。读帧超时已通过
> `CAP_PROP_READ_TIMEOUT_MSEC=1000` 从默认 5s 收紧到 1s，加速掉线判定。
> 本板曾观察到 LRCP S400 反复热插拔（dmesg `device descriptor read/8, error -110`），
> 现象是 `/dev/video*` 节点时有时无、打开瞬间 busy——先查物理链路（线缆/供电/USB口），
> 再判断是否为软件问题。

## 模型

- `test_nashe_640x640_nv12.hbm` 为软链，指向
  `/userdata/vp/vp4.5/test_nashe_640x640_nv12.hbm`
  （实际模型名 `best_nashe_640x640_nv12`）。
- 输入：`images_y[1,640,640,1]` + `images_uv[1,320,320,2]`（NV12）。
- 输出：3 个尺度 DFL 解耦头（P3/P4/P5，stride 8/16/32），类别
  `['door','red-ball','yellow-ball']`。
- 前视只输出 `red-ball`，下视只输出 `yellow-ball`（config `target_class_names` 控制）。
- 两路 `bpu_cores` 当前均配 `[0,1,2,3]`：S100 实测只给 `[1]` 会触发
  `hbUCPSubmitTask failed`，不要手动改成单核调度（详见 main_config.py 注释）。

## 帧率说明（为什么"两个帧率"差 3 倍）

代码里"平均帧率"= producer 采集侧节拍：

| 模式 | 帧率上限 | 实测 | 备注 |
|---|---|---|---|
| 虚拟（`--virtual`） | **60fps**（FPS 封顶 `min(fps,60)`） | ~56fps | 由虚拟帧 `sleep(1/60)` 限速 |
| 真实相机（video0/video2，MJPG 640×480） | 取决于相机+USB带宽 | 软解 ~165-172fps / **硬解可打满 ~200fps** | config `fps=250`（请求值；相机/带宽上限实测 ~200-204fps） |

如果用 `--frames 90` 等固定帧数测一次，看到的 fps ≈ 真实模式上限；
如果用 `--virtual`，fps 被人为限到 ~56fps。这是 3 倍差的根因，**不是 bug**——
是不同模式的供给端限制。如果想去掉虚拟模式 60fps 封顶，看代码顶部 `FPS = max(1, min(int(CAM.get('fps', 60)), 60))` 改为不限速。

真实相机实测（300 帧自然跑完，固定 --frames；**软解基线**，硬解后采集/帧率应
更优——采集从 ~3.0ms 降到 ~2.46ms，纯解码 1742FPS vs 486FPS）：

| 指标 | front | bottom |
|---|---|---|
| 稳定帧率（软解基线） | 171.9 fps | 165.5 fps |
| 采集单帧（软解基线） | ~2.3 ms | ~2.3 ms |
| 检测单帧 | ~4.3 ms | ~4.3 ms |
| 进程 CPU | 85~123% | 61~102% |
| BPU ratio 峰值 | 26% | （同） |

**旧瓶颈在 CPU，不在 BPU**——MJPEG 软解 + 预处理（resize/BGR→NV12）吃掉了大量 CPU，
BPU 还有余量。**本版已把 MJPEG 解码切换到 JPU 硬解**（见"JPU 硬件解码"章节），
把解码从 CPU 卸载到硬件单元；若仍想进一步降 CPU，可降低预处理工作分辨率。

## 已验证行为（2026-09-02）

- 单进程虚拟相机 30 帧：30 行检测输出，EXIT=0
- 3 worker 并发独立实例：每线程 5 帧全检出，无死锁（共享实例会死锁，已改）
- run.sh 双进程：front/bottom 各自绑核、独立退出、互不影响
- 掉线独立性：front 相机打不开时 front 进程退出，bottom 照常跑完并输出统计
- 时间统计（已修 bug）：每窗口用增量口径，数值稳定反映"当前"各阶段耗时
- 帧日志：worker 处理每帧都写一行（包括无目标的 `none`/`done`），多 worker 锁串行化
- 真实相机（video0/video2，640×480 MJPG）软解基线：稳定 ~165-172fps，
  真实单帧 采集 ~2.3ms / 检测 ~4.3ms（恒定），CPU 占用 85-123%（多核并行）、
  BPU 占用峰值 26% → 瓶颈在 CPU（MJPEG 软解 + 预处理/后处理），BPU 远未打满
- **JPU 硬解接线（2026-09-02 新增）**：BCDL（forum 35346）conda 预编译包
  0.7.1 安装/import 成功，但 JpegDecoder 仅支持 4:2:0 JPEG，对 LRCP S400 的
  4:2:2 MJPG 帧硬解被拒 → 改用官方 hb_media_codec 封装（vp4.5 已验证）；
  `main_config.py` 双相机 `hardware_decode=True`，`front.py`/`bottom.py`
  `open_camera` 硬解优先、失败回退 cv2；`/usr/bin/python3 -m py_compile` 通过；
  独立 smoke：`HwMjpgCamera('/dev/video0',640,480,200)` isOpened=True，
  `read()` 返回 (True, 480×640 BGR)。全链路端到端验收因该相机 USB 热插拔抖动
  （节点时有时无）暂未完成，待物理链路稳定后 `./run.sh --frames 100 --no-show`
  应打印两路 `启用 JPU 硬件解码` 与滚动统计

- **GPU/NV12 直通（2026-09-02 新增；2026-09-03 移除 GPU 分支）**：
  - GPU 实测（t0）：Mali-G78AE + OpenCV UMat 对小图预处理比 CPU 慢 ~10×，
    「GPU 加速预处理」路线被数据否定；2026-09-03 已将
    `gpu_accel` / `use_gpu` / UMat / OpenCL 分支从代码中整体移除，
    仅保留 NV12 直通。
  - NV12 直通：合成红球帧直连 BPU，BGR 老路径 vs NV12 直通检测一致
    （`red-ball` 中心 `(53,249)`、conf 0.84），单帧 `3.86ms → 3.64ms`。
  - 虚拟相机双进程 30 帧回归 EXIT=0，front/bottom 均正常出检测。
  - 真实相机（`/dev/video2` LRCP S400）`bottom.py` 120 帧：打印
    `启用 JPU 硬件解码`，show 关闭时队列无 BGR（NV12 直通），检测 ~3.99ms/帧，EXIT=0。




## 上位机推流（STREAM）2026-09-03 新增

把 YOLO **画框后的画面**通过网线实时传到上位机 ROV 控制站显示。

### 链路

```
front.py (YOLO 画框) --TCP:9000(JPEG,长度前缀)--> mjpeg_bridge.py --HTTP MJPEG /cam1--> PC(cv2.VideoCapture)
bottom.py( YOLO 画框) --TCP:9001 -->  mjpeg_bridge.py --HTTP MJPEG /cam2--> PC(cv2.VideoCapture)
```

- `streamer.py`：进程内推流线程（JpegStreamer）。协议：每帧 = `struct.pack('<I', 帧长)` + JPEG bytes；
  以 `STREAM.fps` 节流发**最新一帧**（队列满丢旧，低延迟）；断连自动回到 accept。
- `mjpeg_bridge.py`：纯 stdlib 的 HTTP MJPEG 桥，`/cam1 <- 9000`（front）、`/cam2 <- 9001`（bottom），
  兼容 rc 站 `pc_main2.py` 的 `cv2.VideoCapture("http://<板IP>:5000/cam1"|"/cam2")`。
- `run.sh` 会自动拉起 bridge（`/tmp/mjpeg_bridge.log`），Ctrl-C 时一并清理。

### 配置（main_config.py STREAM 段）

| 字段 | 默认 | 说明 |
|---|---|---|
| enable | True | 启动即监听；`--no-stream` 临时关闭 |
| bind_host | 0.0.0.0 | 监听地址（网线直连可写 eth IP） |
| front_port / bottom_port | 9000 / 9001 | 两路 TCP 推流端口 |
| fps | 60（代码 fallback 30） | 发送帧率上限（显示用，不必追 200fps 采集） |
| jpeg_quality | 100（代码 fallback 80） | JPEG 质量 |

无客户端连接时 worker 不画框不编码（`streamer.connected` 门控），对检测主链路零开销。

### 实测（2026-09-03，真实相机 + 上位机同时拉流）

- 板端：JPU 硬解双路稳定 ~200fps（采集 ~2.0ms/帧、检测 ~4.3-4.9ms/帧），开启推流并连接客户端后
  「显示」阶段仅 0.04-0.08ms/帧，检测帧率几乎不受影响。
- 上位机（Python 3.13 + opencv 4.13，与 pc_main2 相同取流方式）：
  `/cam1` 27.5fps、`/cam2` 26.8fps，均 640×480，单帧平均 ~12KB（q80）。
  （该实测对应当时 config `fps=30`/`q80`；当前 config 为 `fps=60`/`q100`，
  帧率与单帧体积会相应上升，以现场实测为准。）
- 带宽估算：30fps × 12KB ≈ 0.36MB/s ≈ 3Mbps/路，千兆直连富余。
- 接入地址：`http://192.168.127.10:5000/cam1` | `/cam2`（板端 eth1 网线直连上位机 192.168.127.100）。

### 上位机侧

- `ROV控制站_v3.1/protocol.py`：`X5_IP` 已指向 `192.168.127.10`（原 192.168.5.11）。
- 依赖补装：`python -m pip install pygame`（station 缺 pygame 会启动即退出）。
- 直接运行 `python pc_main2.py` 即可在双路视频区看到 vp5.0 的画框画面。


## 遥测滚动上传（TEL）2026-09-03 新增

对应临时任务③「AI 生成数据滚动上传, 上位机读取参数」, 与 ROV 控制站 protocol.py 的
`$TEL` v3.1 协议完全兼容 (41 字段: 10 实际量 + 10 目标量 + 5 电池/温度 + 12 推进器 + 4 加速度/高度)。

- `telem_sender.py`: 板端 10Hz 向上位机 UDP 8081 单播滚动参数 (正弦波+噪声模拟);
  同时在板端监听 UDP 8081 应答上位机 PING -> PONG (GUI RTT 延迟显示)。
- 验证: 上位机按 pc_main2 TelemetryThread 同路径绑定 8081 + `parse_telemetry`,
  40 帧全解析, 10/10 采样字段滚动变化, 12 路推进器齐全。

用法:
```bash
python3 telem_sender.py --dst 192.168.127.100 --port 8081 --hz 10
```

## 帧共享桥（FLOW_SHARE）2026-09-06 新增（vp5.1）

把 front/bottom 每帧解出的**最新帧**经共享内存交给独立的光流测速进程
（`/userdata/momo_pwmnet/flow_speed.py`）读取——光流测速**无需第三路相机**。

### 链路

```
bottom.py producer --FlowShareWriter('bottom', W,H, fmt=0)--> /dev/shm/momo_flow_bottom.bin
                    --FlowShareReader('bottom')--------> momo_pwmnet/flow_speed.py
front.py  producer 默认不写共享帧 (ENABLE_FLOW_SHARE=False, 光流只用下视)
```

### 实现（utils/flow_share.py，vp5.1 新增）

- **写端**：`bottom.py` 的 producer 每次取帧后调用
  `flow_share_w.write(nv12 if nv12 is not None else frame)`——JPU 硬解时共享
  **NV12 直通帧**（fmt=0），软解/虚拟回退共享 BGR（fmt=1）；
  `front.py` 默认不写（`ENABLE_FLOW_SHARE=False`，省一路 memcpy），需前视光流时改 True。
- **读端**：`FlowShareReader('bottom')`，`read()` 返回 `(seq, frame)`；无新帧时返回
  `(last_seq, None)`；初始化时最多等写端 5 秒，超时抛错。
- **格式**：`/dev/shm/momo_flow_bottom.bin` = 定长头部 + 数据区
  （容量 `w*h*3`，按 BGR 最大容量分配，NV12 帧只写 `w*h*3//2` 字节）。
  头部 = magic `'MFS1'` + ver/seq/w/h/fmt + ts_us（代码内
  `HDR.size + HDR_TS.size` = 32B；模块顶部注释写的 48B 未随实现更新，以代码为准）。
  并发用 `fcntl.flock` 互斥（写端 EX、读端 SH），写端先拷数据、后更新 seq/ts，
  读者见到的总是完整新帧，无撕裂。
- **开销**（模块注释记录）：写一次 ~0.1ms，200fps 采集下额外 <2% 单核。

### 开关

| 字段（main_config.py 顶层） | 默认 | 作用 |
|---|---|---|
| `ENABLE_FLOW_SHARE` | False（2026-09-09 起） | front 是否写共享帧 `/dev/shm/momo_flow_front.bin`（光流测速已改读下视，默认 False 省一路 memcpy）；需前视光流时改 True |
| `ENABLE_FLOW_SHARE_BOTTOM` | True（2026-09-09 起） | bottom 是否写共享帧 `/dev/shm/momo_flow_bottom.bin`（光流测速的数据源，默认 True）；False = 下视不写，光流进程会无帧退出 |

- 开启条件（两路各自独立）：bottom 需要 `ENABLE_FLOW_SHARE_BOTTOM=True` **且非 `--virtual`**（虚拟相机模式自动跳过）；front 需要 `ENABLE_FLOW_SHARE=True`。
- 初始化成功打印：仅 bottom：`[*] [bottom] 帧共享已开启: /dev/shm/momo_flow_bottom.bin (NV12)`。
- 初始化失败只打一行 `[警告] 帧共享初始化失败 ... 继续原有流程`，**不中断检测**。

### 使用与验证

```bash
cd /userdata/vp/vp5.1
# 1) 确认 main_config.py: ENABLE_FLOW_SHARE=False（front 不写）、ENABLE_FLOW_SHARE_BOTTOM=True（bottom 写，光流数据源）
# 2) 一行启动：front/bottom + 推流桥 + 遥测 + 光流测速同启同停（Ctrl-C 一次带走）
./run.sh --no-show
# 3) 确认共享帧已建立（约 900KB，随帧更新）
ls -l /dev/shm/momo_flow_bottom.bin
# 4) 浏览器看光流测速画面（speed/motion 叠加）：http://<板卡IP>:${FLOW_PORT:-8080}/
# 5) 光流测速日志：tail -f /tmp/flow_speed.log
# 备选：脱离 run.sh 单独跑光流测速（不联调上位机/推流时，固定读 bottom 共享帧；
#   v4 起默认 --fps 50 --port 8080，亦可显式指定）
#    python3 /userdata/momo_pwmnet/flow_speed.py
```

### 影响面（vp5.1 相对 vp5.0 的全部改动）

`main_config.py`（新增 `ENABLE_FLOW_SHARE`、`ENABLE_FLOW_SHARE_BOTTOM`）、`utils/flow_share.py`（新增）、
`front.py`（producer 增加共享写入旁路）、`bottom.py`（共享写入新增独立开关，默认关闭）、
`run.sh`（同生命周期启动光流测速进程；`--virtual` 跳过；`FLOW_PORT`/`FLOW_FPS` 覆盖）。
YOLO / 推流（STREAM）/ 遥测（TEL）/ `flow_speed.py` 内部实现均未改动。

> **合并说明（2026-09-06）**：原同目录 `README_zxq.md`（vp5.1 板端修改说明：
> 帧共享桥 改了什么/怎么跑/怎么回滚）内容与本章节一致，已并入本 README，
> **本目录不再单独维护 README_zxq.md**。
>
> 补充要点：
> - 默认 `ENABLE_FLOW_SHARE=True`（front 写共享帧）、`ENABLE_FLOW_SHARE_BOTTOM=False`（bottom 不写）；
>   两开关都关掉时 `bash run.sh` 行为与 vp5.0 完全一致；`--virtual` 同样跳过帧共享。
> - `run.sh` 同生命周期：`./run.sh` 默认会一并拉起光流测速进程（Web :8080/抽帧默认 50fps，
>   v2 时为 20fps，v4 起改为 50），Ctrl-C 一次带走；`--virtual` 自动跳过；端口/抽帧率可用
>   `FLOW_PORT` / `FLOW_FPS` 覆盖。
> - `/dev/shm` 是 tmpfs：共享文件随重启自动清空，无需清理、不占磁盘。
> - **回滚**：把 `ENABLE_FLOW_SHARE` 与 `ENABLE_FLOW_SHARE_BOTTOM` 都改回 `False` 即回到 vp5.0 行为；
>   vp5.0 完整版仍在 `/userdata/vp/vp5.0/`（含其修改说明）可对照。
>
> **vp5.1 联合改动 v2（2026-09-07）**：`run.sh` 同步启动光流测速 ——
> 板端默认一并拉起 `/userdata/momo_pwmnet/flow_speed.py`（`:8080`/抽帧 20fps，
> 与 front/bottom/bridge/telem 同启同停，Ctrl-C 一次带走）；`--virtual` 模式自动跳过；
> 端口/抽帧率可用 `FLOW_PORT` / `FLOW_FPS` 环境变量覆盖。
>
> **2026-09-09 起**：光流测速已改用下视（读 bottom 共享帧，front 默认不再写），当前行为以版本记录顶部新条目与开关表为准；以下 2026-09-06 为历史表述。
>
> > **vp5.1 联合改动（2026-09-06）**：光流只用前视摄像头 —— vp5.1 侧新增
> `ENABLE_FLOW_SHARE_BOTTOM=False`（bottom 默认不再写共享帧，省一路 memcpy）；
> 光流侧 `/userdata/momo_pwmnet/flow_speed.py` 移除 `--cam` 参数，固定读
> front 共享帧；两侧 README 已同步。
