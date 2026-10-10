# task_door_sim

独立的门视觉实验，经 `src/front.py` 采集前摄，复用 `quad_cv_kit` v7 常驻 GPU CV。
不控制推进器，不发布正式门任务的 `door` 控制观测。

## 当前链路

前摄原生 **640×480 MJPG** → JPU 解码 BGR → GPU 相机校正 →
干净校正图供 BPU YOLO 和常驻 GPU CV → 绘制 YOLO/CV/姿态提示 → JPEG 共享帧 → `:5000/cam1`。
对比度、CLAHE、饱和度增强和锐化均已关闭，`enhancement=0`；没有增强图副本。
默认模型为 `models/door_6_nashe_640x640_nv12.hbm`，由 `config/stage_model.py` 的 `gate`
配置选择；普通前视基线也使用此新模型。BPU 输入尺寸从 HBM 元数据读取。
640×480 图像保持原尺寸，上下各填充 80 像素灰边成为 640×640，再生成 NV12 Y/UV。
后处理去掉填充，检测框还原到 640×480 校正图。

相机分辨率、画面中心 `(320,240)`、正式门任务尺寸、前视观测归一化、视卡尔曼图像尺寸
与共享帧头统一为 640×480。校正采用 `quad_cv_kit/camera_correction_params.json` 的
640×480 标定；下摄尺寸和基线模型独立配置。
驱动实际输出不符合请求尺寸时，前摄停止并报错，避免使用错误标定。

CV 内部保持 **640×360 标准画布**，640×480 输入按比例 0.75 映射为 480×360，
左右各留 80 像素。YOLO 框、有效掩膜、CV 边和姿态坐标按同一变换处理，输出还原到
640×480 校正图。没有把 4:3 画面拉伸成 16:9，也没有改动颜色/模型判定阈值。

默认 `cv_backend=auto`，有 GPU 时使用 `cv_execution=resident`；GPU 不可用时明确记录
CPU/hybrid 回退。显式选择 `--cv-backend opencl --gpu-device Mali` 时不可用会报错。
常驻路径的颜色、位压缩形态学、行游程八连通区域、光流/RANSAC、Hough、稳定峰值排序/NMS、
线段拟合/合并、完整/残缺模型和位姿全部使用 OpenCL。
输入图上传一次，校正图保留在 GPU；同帧 CV 中间图像和候选结果不回读。
稳态每帧的图像上传为 921,600 字节，外部 BPU/绘制需要的校正 BGR 回读为 921,600 字节，
另上传小型 YOLO 结果并回读 CV 最终状态 800 字节。初始化时会额外上传标定映射、查表和有效掩膜。
CPU 负责外部 BPU 输入转换/YOLO 后处理、OpenCL 提交和最终绘制/JPEG 编码。

CV 搜索默认每 3 个处理帧运行一次，间隔帧根据当前图像跟踪，失败同帧重新搜索。
YOLO 逐处理帧运行，JPEG 默认质量 85。实时相机单 worker、只保留最新待处理帧；录像顺序处理。
详情见 [MALI_RESIDENT.md](../../../quad_cv_kit/MALI_RESIDENT.md)。

## 启动和查看

在 RDK 工程根目录执行：

```bash
./run.sh --door-sim --door-sim-args "--cv-backend opencl --gpu-device Mali --cv-execution resident"
```

启动器调用 `task_door_sim/run.py → front.main()`，前摄设备沿用 `camera_ports.py` 的 cam1
USB 物理口配置。Web 服务也由 `run.sh` 启动，上位机访问：

```text
http://<板端IP>:5000/cam1
```

处理后的 JPEG 写入 `/dev/shm/momo_frame_front.bin`，同次检测/尺寸/诊断写入
`momo_det_front.json`，统计写入 `momo_stats_front.json`。
`web_server.py` 读取同一共享帧；`/api/detections` 可查看 `door_sim` 诊断。
不加 `--door-sim` 时使用普通前视流程。测试启动即运行，不等待 ROV/AUV 阶段。

```bash
# 调整搜索频率与 JPEG 质量：
./run.sh --door-sim --door-sim-args "--cv-every 3 --jpeg-quality 85"
# CPU 对照：
./run.sh --door-sim --door-sim-args "--cv-backend cpu"
# 启动中位机 ROV：
./run.sh --door-sim --to32-args "--mode rov"
```

日志为 `logs/front.log`。`status.sh` 查询状态，`stop.sh` 或 Ctrl+C 停止。
`run.sh` 自动避免普通前摄与 DoorSim 同时占用相机/共享文件。
测试专用参数通过 `--door-sim-args` 按空白切分透传。

单独手动运行时，先停止其他前视生产者，在两个终端分别执行：

```bash
python3 src/to32/task_door_sim/run.py --cv-backend opencl --gpu-device Mali --cv-execution resident
python3 src/web_server.py
```

默认使用新模型，也可用 `--model /path/model.hbm` 覆盖。
`--no-correction` 在原图检测，关闭依赖校正内参的姿态提示。
`--shm-dir` 指定共享输出目录时，Web 读端须设置相同的 `GRDK_SHM_DIR`。
中位机直接相机视频服务与 `web_server.py` 同占 5000 端口，此链路使用 `--no-video`。
`--sharpen` 和 `--contrast` 已移除。

板端用视频检查同一条前视处理和发布链路：

```bash
python3 src/to32/task_door_sim/run.py --source /path/door.mp4 --frames 100 --cv-backend opencl --gpu-device Mali
```

视频尺寸取实际帧，并适配标定；验证新的相机输入条件时应使用 640×480 录像。

## 性能诊断

`door_sim.timing_ms` 分列校正、增强、YOLO、CV、绘制耗时；`door_sim.stream` 给出 JPEG 大小、
编码、共享写入和采集到发布延迟。采集线程打印的简化 fps 是采集帧率，不能当作视觉处理帧率。
`door_sim.yolo.cv_profile` 给出 CV 总墙钟/调用线程 CPU 时间和各 GPU 阶段事件时间。
`door_sim.gpu` 给出实际设备、内核时间、传输字节/次数及 `pipeline_version=7`。
`cpu_compute_stages=[]`、`intermediate_readbacks=0` 表示 CV 核心走常驻 GPU 路径，
并不表示绘制、JPEG 或外部 BPU 预处理也在 GPU 上。
设备事件时间与等待墙钟时间不能重复相加。初始化、模型预热和首帧耗时需单独看待。

默认每 2 秒打印 `[front][DoorSim][CVProfile] event=window`，分别汇总 search/track/idle
均值、P95、最大值及最慢帧；达到 100ms 的慢帧额外限频记录。
`--no-cv-profile` 关闭内部诊断。实际性能需要 Mali-G78AE 板端录像/相机复测。

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
python3 -m unittest discover -s src -p test_yolo_postprocess.py -v
python3 -m unittest discover -s src -p test_front_stream.py -v
python3 -m unittest discover -s src -p test_vision_dimensions.py -v
```

验证真实 NV12 预处理/框还原、合成图 CV、共享 JPEG/JSON 和实际 `/cam1` ASGI 路由。
HBM 推理使用运行时桩；OpenCL C 内核通过 CPU 正确性测试执行，不使用开发机 GPU，
不把测试时长作为 Mali 性能结论。物理相机协商和新模型的实际 BPU 输出仍需板端验证。
