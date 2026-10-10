# task_door_sim

独立的纯视觉实验，参考 `move_test/task/task_door` 的前视处理和共享帧发布方式，
复用工程根目录 `quad_cv_kit`。`sim` 表示视觉实验，不生成假检测，不控制推进器。

流程：相机/视频 BGR → 相机校正 → 仅亮度锐化 0.6 →
整门 YOLO → 最大门/贴边目标连续性选择 → ROI 内 OpenCV 红杆拟合和短时跟踪 →
增强图叠加黄色 YOLO 框、绿色 CV 边及校正坐标下的视觉回正提示 → JPEG 共享帧。
YOLO 和 CV 使用同一张增强图，干净校正图保留为 CV 颜色/白角验证依据。
预处理不再做对比度、CLAHE 或饱和度增强，锐化强度集中在 `config.py`。
OpenCL 用单个融合内核完成亮度提取、sigma=1.2 的高斯模糊和锐化；有效邻域
加权避免校正黑边产生亮边，原 BGR 通道同比例缩放。`--sharpen 0` 关闭锐化。
默认 `cv_backend=auto` 优先选择 Mali GPU，接入当前 `quad_cv_kit` 的 OpenCL 校正、
增强、颜色、Hough、线拟合/修剪与边支持；不可用时在日志和同帧 JSON 中明确记录 CPU 回退。
板端默认参数为 `--gpu-device Mali --cv-hough opencl --cv-blur pyramid --cv-quality fast`
及 `--cv-search adaptive --opencv-threads 3`，与优化后的录像入口一致。
颜色 ROI 保留高斯上下文，LSD 双通道与 GPU Hough 并行，几何验证复用交点和重复边支持。
自适应搜索使用当前图像光流及四边搜索带，失败同帧回退，周期性完整搜索；没有卡尔曼路径。
CV 完整搜索默认每 3 个处理帧运行一次，间隔帧使用当前图像跟踪并验证边线；
跟踪失败立即重新搜索。YOLO 仍逐处理帧运行，JPEG 默认质量为 85。
姿态与像素等效位移仅供显示，不发送控制指令。

输出与现有前视回传兼容：

- `momo_frame_front.bin`：增强并标注后的 JPEG，`src/web_server.py` 经 `/cam1` 回传。
- `momo_det_front.json`：同次处理的 `frame/capture_ts/dets/door_sim`。
  `door_sim` 包含当前 YOLO 框、CV 边、跟踪状态、视觉提示与增强参数；
  `stage=DoorSim`，不发布正式控制使用的 `door` 字段。

`perception.py` 负责图像处理；`front_pipeline.py` 的 `DoorSimFrontPipeline.process`
可接收已有采集线程的 BGR 帧及共享写端。重复帧与采集时已过期的帧跳过，
多个调用共用一个串行跟踪器。`run.py` 仅负责配置模型和图像算法，再调用 `front.main()`；
相机采集（含 JPU 硬解）、队列、JPEG/JSON/统计共享写入全部由 `front.py` 管理。
测试前视使用单个检测 worker，保证跟踪与共享写入顺序。
实时相机只保留最新一张待处理帧，CV 变慢时覆盖旧帧，不让 12 帧队列积压画面；
录像仍按顺序处理。`door_sim.timing_ms` 分列校正、增强、YOLO、CV、绘制耗时，
`door_sim.stream` 记录 JPEG 大小、编码、共享写入及采集到发布的延迟。
`door_sim.cv_backend` 记录实际后端/回退原因；OpenCL 模式增加 `door_sim.gpu`，
包括设备、内核与操作耗时、传输字节及 Hough 工作量。GPU 校正图和增强图与录像入口相同，
增强图输入 YOLO/CV，干净校正图提供颜色证据。退出时等待算法线程并释放 OpenCL。
启用时间统计时每 2 秒打印一条 DoorSim 处理统计；原简化 fps 仍是采集帧率。

### OpenCV 内部性能定位

DoorSim 默认开启内部诊断；颜色、几何和跟踪判定阈值沿用原值。每个处理帧记录
`door_sim.yolo.cv_profile`，完整内容可从 `/api/detections` 的前视数据读取。
`[front][DoorSim][CVProfile]` 日志包含两类记录：

- `event=slow`：CV 耗时至少 100ms 的帧，最多每秒打印一次，包含最慢的 6 个子阶段及候选数量。
- `event=window`：每 2 秒分别汇总 `search`（完整搜索）、`track`（图像跟踪）、`idle`（无搜索/无成功跟踪）
  的均值、P95、最大值，并保留该窗口最慢帧的细节，避免周期采样漏掉卡顿。
  分布最多保留最近 256 个样本，窗口最慢帧单独保留。

`stages_ms` 为顺序分段耗时，可相加，不重复累计父子阶段。
`thread_cpu_ms` / `stages_thread_cpu_ms` 为调用线程的 CPU 时间，不包括 OpenCV 的其他工作线程；
它们与墙钟时间的差值只能辅助判断调度/等待，不能直接等同于 CPU 总使用率。
统计范围为 `RedGateTracker.update`，外层 `CV=` 还包含 YOLO 目标关联等少量开销。
慢帧日志的 `search_reason=tracking_failed` 表示因跟踪失败重新搜索，
`scheduled` 为定期搜索，`no_previous` 为未持有上一目标，`hold_expired` 为跟踪时效已过。

| 子阶段或计数 | 用于定位 |
| --- | --- |
| `detect.color.*` / `track.color.*` | 颜色转换、三尺度高斯模糊、阈值及颜色证据合并 |
| `lines.lsd_*` / `lines.hough_join` | 快速 GPU 配置运行 mask/chroma 两路 LSD；汇合阶段仅记录剩余等待 |
| `lines.prefilter` / `lines.fit_filter` / `lines.merge` | 廉价预筛选、保留线段的拟合、嵌套合并 |
| `lines.contrast_full_image` | 每次搜索构造干净图/增强图的红绿对数比缓存，最多两次 |
| `models.geometry` / `models.valid_mask` / `models.color_support` | 四边组合几何、有效区和逐边颜色证据检查 |
| `track.features` / `track.optical_flow` / `track.ransac` / `track.revalidate` | 跟踪各阶段 |
| `raw_segments` / `merge_comparisons` | 输入线段数量、合并比较次数 |
| `quad_combinations` / `quad_color_checks` | 枚举组合数、进入四边颜色检查的模型数 |
| `raw_segments` / `prefilter_segments` / `fitted_lines` | 预筛选前、昂贵拟合前、拟合通过的线段数 |
| `model_side_checks` | 实际执行的逐边验证次数；遇到不支持的边立即淘汰该组合 |
| `contrast_full_image_calls` / `color_evidence_calls` / `color_evidence_reuses` | 对比度缓存构造、颜色证据计算及同帧复用次数 |
| `line_extraction_size` | LSD/Hough 实际处理尺寸；输出坐标仍属于 640×360 画布 |
| `color_processing_size` / `line_extract_parallel` / `lsd_worker_ms` | 颜色实际尺寸、线提取并行状态和各 LSD 通道完整时间 |

OpenCL 模式 `enqueue_*` 只记录 CPU 提交时间，`readback` 包含剩余 GPU 等待；
Hough 完整耗时看 `door_sim.gpu.operation_ms.hough`。并行工作线程时间不能相加作为帧耗时，
`thread_cpu_ms` 不包含这些工作线程，实际性能以当前相机画面测量为准。

OpenCL 搜索现在将有序合线、断口拆分、对比度采样和完整模型几何/有效区检查
批量放在 GPU；残缺门框的交点与逐线颜色证据也使用 GPU。日志通过
`search_execution/model_execution/partial_execution` 标识实际执行，完整几何计时
合并为 `models.gpu_batch`。CPU 仍保留 LSD、连通域、图选择和跟踪。
实现边界、BPU 适用性及板端对照命令见 [GPU_SEARCH.md](../../../quad_cv_kit/GPU_SEARCH.md)。

诊断仍随原启动命令运行；需要关闭时使用 `--door-sim-args "--no-cv-profile"`。
离线诊断入口不需要模型/BPU，可以用固定原图 ROI 检查慢帧或录像：

```bash
python3 quad_cv_kit/demo/profile_red_gate.py --source /path/slow.jpg --roi 100 80 1200 710 --frames 8 --out /tmp/cv_profile.json
python3 quad_cv_kit/demo/profile_red_gate.py --source /path/door.mp4 --frames 100 --out /tmp/cv_profile.json
# 固定种子的复杂线段场景，用于复现组合开销，不能代替板端现场结论：
python3 quad_cv_kit/demo/profile_red_gate.py --synthetic clutter --frames 8 --opencv-threads 3 --out /tmp/cv_profile.json
```

离线入口在未校正输入上运行 CV，增强参数为 DoorSim 默认值；不执行 YOLO，
默认搜索全图，`--roi` 为原图坐标。`--python-profile /tmp/cv.prof` 可额外保存 cProfile 数据，
该选项会增加测量开销。板端定位以实际 DoorSim 输出为准。

### 已实施的 CV 搜索优化

- 干净图、增强图的红绿对数比按搜索帧懒加载，每张图只计算一次，各候选杆共用。
- 批量排除长度/方向不合格的线段；积分图只排除采样覆盖区完全无红像素的线段，
  保留弱红、断续红杆进入原有采样验证。减少采样坐标张量和合并循环内的重复分配。
- LSD/Hough 实际裁剪到搜索区域及 16px 邻域，并将线段坐标还原到原画布。
  边界向外对齐到 5px 网格，保持 LSD 默认 0.8 缩放的采样位置。
  CLAHE 保留完整画布上下文，颜色 ROI 保留高斯邻域，原图尺度、管宽、校正有效区不变。
  `precise` 保留三路 LSD，`fast` 使用 mask/chroma 两路，支持贴边、缺边和弱色门框。
- 三尺度高斯模糊使用连续的 LAB a 通道，逐次原位取最大值，减少通道复制和临时数组；
  跟踪与重新搜索复用同一次 `update` 的颜色证据和预处理图，不跨帧缓存。
- 四边模型发现一条边缺乏颜色支持就立即淘汰，避免继续采样其余三边；通过条件不变。

开发机 OpenCV 5、3 线程、相同合成图，预热后交替运行优化前后各 7 次的耗时中位数：

| 场景 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 简单完整门搜索 | 39.36ms | 33.00ms |
| 复杂线段全图搜索 | 532.42ms | 251.16ms |
| 复杂线段 ROI 搜索（面积约 34.7%） | 263.70ms | 130.38ms |
| 简单门跟踪后定期搜索 | 60.64ms | 35.69ms |

复杂全图/ROI 场景的整图对比度计算次数分别从 86/42 降到 2；
跟踪后搜索的颜色证据计算次数从 2 降到 1。以上不包括 YOLO、校正、增强、JPEG 和网络，
也不代表 RDK 板端绝对耗时。板端复测重点比较 `search` 的 P95/最大值、
`lines.fit_filter`、`lines.hough`、`lines.contrast_full_image` 和采集到发布延迟。

不依赖 `momo_stage.json`、AUV 模式或任务表，也不写模式/阶段状态。
输入尺寸从实际帧取得；尺寸变化时重建校正和跟踪状态。

## 通过 run.sh 启动

```bash
./run.sh --door-sim
# 同时将中位机以 ROV 模式启动：
./run.sh --door-sim --to32-args "--mode rov"
# 可选：增强参数透传
./run.sh --door-sim --door-sim-args "--sharpen 0.8"
# 调整 CV 搜索频率与推流压缩：
./run.sh --door-sim --door-sim-args "--cv-every 3 --jpeg-quality 85"
# 显式要求 Mali OpenCL，不可用则报错：
./run.sh --door-sim --door-sim-args "--cv-backend opencl --gpu-device Mali --cv-quality fast --cv-blur pyramid --cv-hough opencl"
# CPU 对照：
./run.sh --door-sim --door-sim-args "--cv-backend cpu"
```

加 `--door-sim` 后，启动链路为 `task_door_sim/run.py → front.main()`，
相机沿用 `front.py` 前视配置。`front.py` 将增强、YOLO/CV 标注后的图编码为 JPEG，
写入 `main_config.SHM_FRAME_FRONT`，`web_server` 从同一路径读取并经 `:5000/cam1` 回传。
测试在脚本启动后立即运行，
不等待切到 ROV/AUV；中位机默认仍为 IDLE，需要启动即进入 ROV 时加上述 `--to32-args`。
不加 `--door-sim` 时沿用原前视流程。下视、Web 和中位机仍按原启动参数运行。
`--frames` 同时作用于测试前视和下视；测试专用参数通过 `--door-sim-args` 按空白切分透传。
日志为 `logs/front.log`，`status.sh` 可查询，`stop.sh`、Ctrl+C 和重新运行
`run.sh` 都会清理测试进程，避免与普通前视争用同一相机和共享文件。

## 单独手动运行

在工程根执行，板端默认复用 `config/stage_model.py` 中的整门 HBM 配置：

```bash
python3 src/to32/task_door_sim/run.py --source /dev/video0
python3 src/web_server.py
```

上位机继续访问 `http://<板端IP>:5000/cam1`。也可沿用已启动的 `web_server`。
本入口内部调用 `front.py` 写前视帧；同一个相机和前视共享文件只能有一个生产者，
运行实验前停止原前视采集。`to32/main.py` 内置的直接相机视频服务与
`web_server` 同占 5000 端口，运行此链路时中位机使用 `--no-video`。

开发机录像实验（需要 OpenCV、NumPy、Ultralytics；默认权重为 `quad_cv_kit/model/best.pt`）：

```bash
python3 src/to32/task_door_sim/run.py --backend ultralytics --source /path/door.mp4 --frames 100
```

可用 `--model` 指定整门模型，`--sharpen` 调整锐化（0 关闭）；已移除 `--contrast`。
`--no-correction` 直接在原图检测，不提供依赖校正内参的姿态提示。
`--shm-dir` 可指定测试输出目录；读端须使用相同目录（`GRDK_SHM_DIR`）。
正常退出、Ctrl+C 或 SIGTERM 会释放输入和共享写端。

## 当前模式切换与任务入口

以当前源码为准（旧 README 中的下位机模式码和 AUV 空壳描述已过时）：

1. `config/to32_config.py`：默认 `START_MODE=-1`（IDLE），`MODE_PERSIST=False`。
   只有显式带 `mode` 字段的 `$CMD` 才从 IDLE 激活 ROV/AUV。
   命令行 `python3 src/to32/main.py --mode rov|auv|idle` 可指定启动模式。
   启用记忆后优先级为命令行 > 记忆文件 > `START_MODE`。
2. 上位机向 UDP 8080 周期发送
   `$CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#\r\n`。
   `mode=0` 是 ROV，`mode=1` 是 AUV。例如：
   `$CMD,0,0,0,0,0,0,1,0,0#\r\n` 请求进入 AUV。
   同一模式的后续帧不会重启任务；再次切回 ROV 后再切 AUV 才从头开始。
   已激活时，旧格式省略 mode 会被按默认 0 解释，可能切回 ROV。
3. `mode_dispatcher.py` 调用旧模式 `on_exit` → 更新 mode →
   先下发 START（功能码 `0x04`、值 `0x00`）→ 新模式 `on_enter`。
   当前下位机有线 ROV 模式值为 `0x06`，AUV 为 `0x05`，
   与上位机的 0/1 编号不同。
4. **当前 `move_test/test_mode/test_config.py` 的 `TEST_MODE_ENABLED=True`**：
   切 AUV 实际进入 `TestMode`，运行 `TEST_TABLE=TASK2_TABLE + RETURN_TABLE`。
   当前测试表没有 `DOOR_TABLE`；切 AUV 不会直接启动 `task_door`。
   开关只在中位机启动注册模式时读取，修改后需重启中位机。
5. 测试开关关闭后，切 AUV 进入正式 `move_test/mode_auv.py`，
   重建 Mission 并按 `task_config.STAGE_TABLE` 从头运行；其中包含 `DOOR_TABLE`。
   到 `PassGate` 阶段时 `front.py` 启用正式门视觉流程，`DoorTask` 读取其 `door` 观测。
   切回 ROV 会停推并关闭任务；上位机在 AUV 下的手动杆位由该模式忽略。

本次未将 `task_door_sim` 注册进 ROV/AUV，也未修改正式/测试任务表。
模式与两种门任务的启动关系留待用户决策。`$VID` 是独立的视频开关，
不能选择两种门任务；`$TASKPID` 更新参数也不启动任务、不切模式。

## 无硬件验证

```bash
python3 -m unittest discover -s src/to32/task_door_sim -p test_door_sim.py -v
python3 -m unittest discover -s quad_cv_kit/tests -p test_yolo_red_gate.py -v
```

使用合成图与检测器桩验证增强输入、干净颜色证据、真实 OpenCV 拟合、
共享 JPEG/JSON 读写与录像入口；不打开相机或串口。
