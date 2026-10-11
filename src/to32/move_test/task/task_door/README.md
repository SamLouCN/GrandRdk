# 门视觉链路与 cx/cy 航向深度测试

本目录保留相机校正、YOLO/OpenCV 后端、UI 标注和共享发布，另外整合下载版
`gate_yaw_cx_ctrl.py` 和 `gate_depth_ctrl.py`，在同一个任务中测试 cx 航向和 cy 深度计算。
此前的完整穿门状态机、扫视、接近、盲冲、机器人坐标变换和控制重置握手仍已删除。
`DOOR_TABLE` 在 `gate_yaw_cx_ctrl.py` 中导出，由 `task_config.py` 导入，
不加入正式任务序列；其他任务（包括 `t_pass_door_v2`）独立保留。

## 启动

从工程根目录运行前摄纯视觉入口：

```bash
python3 src/front.py --door-vision --no-show
```

该参数固定选择 `config/stage_model.py` 中的门模型
`door_6_nashe_640x640_nv12.hbm`，不依赖或改写 `momo_stage.json`，不启动 to32。
摄像头仍由 `front.py` 采集，输入为 640×480 BGR，校正后重新生成模型 NV12。
单 worker 按顺序处理；实时采集只保留最新待处理帧。
不要同时运行另一个占用前摄或前视共享写端的进程。

在另一个终端启动 Web 服务（已运行时无需再启动）：

```bash
python3 src/web_server.py
```

通过 `http://<RDK-IP>:5000/cam1` 查看带 YOLO/CV 标注的画面。
前视与 Web 使用相同的 `main_config.SHM_DIR` / `GRDK_SHM_DIR`。

## 数据链路与输出

```text
front.py 采集
  → DoorFrontPipeline
  → DoorFrameProcessor：校正 → BPU YOLO → GPU resident CV
  → original_geometry：还原校正图像坐标
  → draw_door_overlay
  → momo_frame_front.bin → web_server.py → /cam1
  → momo_det_front.json（同帧结构化结果）
```

| 文件 | 职责 |
| --- | --- |
| `config.py` | 图像尺寸、标定、后端、选门连续性和 UI 参数 |
| `perception.py` | 调用 YOLO/CV，构造并返回 OpenCV 几何与 UI 姿态结果 |
| `front_pipeline.py` | 串行处理、丢弃积压/乱序帧、写共享 JPEG 和 JSON |
| `overlay.py` | 黄色 YOLO 框、绿色实测边、浅绿延长线、红色角点和姿态文字 |
| `gate_yaw_cx_ctrl.py` | 下载版航向控制器、合并航向/深度的 Stage 包装和 `DOOR_TABLE` 注册 |
| `gate_depth_ctrl.py` | 下载版深度控制器，保持原公式、增益和 step 流程 |
| `servo.py` | 替代缺失的 `auv_task.servo`：wrap180、实际航向/深度遥测读取 |
| `control_observation.py` | 同帧读取选中门框 cx/cy 与 CV Z 角，校验新鲜度/尺寸并去重 |
| `test_vision.py` | 无相机、无串口的后端及共享数据链路测试 |
| `test_control.py` | 航向计算、观测接线、任务注册及二进制运动帧测试 |

`DoorFrameProcessor.process(frame, detector, now)` 返回
`(corrected_bgr, detections, observation)`。OpenCV 对外输出位于 `observation`：

- `geometry`：框、实测边线、模型边线、角点、完整性、检测/跟踪状态和边支持度；
  坐标已从内部 640×360 画布还原为校正后的 640×480 图像像素。
- `guidance`：UI 模式、缺边方向、角点/延长线及 `alignment` 姿态估计。
- `guidance.alignment.rotation_xyz_deg`：三轴旋转，单位度。
- `guidance.alignment.translation_scaled_xyz`：三轴像素等效位移，Z 为虚拟尺度，非米制距离。
- `cv_profile` / `gpu`：阶段耗时和 GPU 传输诊断。

无当前目标时 `geometry` / `guidance` 为 `None`；几何或姿态不可用时按状态/原因显示。
姿态明确标记 `ui_only=True`，不产生运动指令。坐标约定为相机 X右/Y下/Z前。
GPU 路径使用算法库的平面单应姿态；CPU 回退使用算法库的四点 IPPE 显示。
GPU 算法库的门框参考尺寸为 0.70×0.50，UI 不把其输出声明为可靠米制距离。
`config.py` 的门尺寸字段用于 CPU 姿态显示。

`front_pipeline.py` 将观测写入 `/dev/shm/momo_det_front.json` 的 `door` 字段，
与顶层 `frame` / `capture_ts` / `dets` 一起发布（路径可由共享目录配置覆盖）。
例如 `door.geometry.corners` 和 `door.guidance.alignment.rotation_xyz_deg`。
共享 JPEG 在校正图副本上绘制，不污染后端输入或结构化数据。

稳态 GPU 路径只上传一次 921,600 字节输入图，回读校正 BGR 供 BPU/绘制，
CV 核心仅回读 800 字节最终状态，不回读中间图像。
不做锐化、对比度或其他图像增强。

## 同任务测试 cx 航向和 cy 深度控制

原控制核心保持下载版逻辑与增益：

```python
target_yaw = wrap180(actual_yaw + K_PSI_DEG * (320.0 - cx))
K_PSI_DEG = 1.0
target_depth_cm = actual_depth_cm + K_DEPTH * (240.0 - cy)
K_DEPTH = 1.0
```

`wrap180` 范围为 `(-180, 180]`；实际航向来自解码后的 0x0C 遥测
`actual_yaw`，不在控制器内镜像。航向镜像仍由现有 AUV/测试输出壳施加一次。
增益按用户要求保留在 `GateYawController.K_PSI_DEG` 和
`GateDepthController.K_DEPTH`，未增加限幅、滤波或到位完成判据。
两个控制器的目标都基于本拍实际遥测，不基于上一次下发目标递推。

任务包装从同一条 `momo_det_front.json` 读取
`door.center_px[0]`（cx）、`door.center_px[1]`（cy）和
`door.guidance.alignment.rotation_xyz_deg[2]`。
cx/cy 是选中 YOLO 框中心，坐标为校正后的 640×480 图像像素；不混用 CV 四角均值。
Z 是相机轴的滚转角，单位度；仅作为日志/诊断观测，不参与 cx 航向公式。
无有效 OpenCV 姿态时 Z 记为不可用，仍可用当前 YOLO cx/cy 计算航向和深度。
日志包含 `frame / cx / cy / CV_Z / target_yaw / target_depth`。

任务同时调整航向和深度，两者合并为同一条 0x09；不执行前冲或横移（`surge=sway=0`）。
不额外注册串行深度阶段，`DOOR_TABLE` 仍只含一个 `GateYawCxTask`。
任一控制器缺输入、观测过期或重复帧时返回既有 `paused` 契约，本拍不发新的 0x09；
这不会清除固件此前的航向/深度目标。此计算器没有任务完成判据，持续运行到切出模式。
控制器/任务包装不钳位原深度公式结果；现有 `frame_motion` 按
`DEPTH_MAX_CM` 将最终下发深度限定到协议范围 `[0, DEPTH_MAX_CM]`。

启用测试前，在 `src/to32/move_test/test_mode/test_config.py` 手动选择：

```python
TEST_MODE_ENABLED = True
TEST_LOOP = False
TEST_TABLE = DOOR_TABLE
```

上述活动配置本次未自动修改。随后从工程根目录运行：

```bash
./run.sh --to32-args "--mode auv"
```

Mission 进入 `PassGate` 后自动通知普通 `front.py` 使用门模型和门视觉链路。
无需另外启动 `front.py --door-vision`，以免两个进程争用前摄。
同一任务也兼容独立纯视觉入口；仅看视频时继续使用前面的 `--door-vision` 命令。

## 逐帧耗时日志

`./run.sh` 默认将视觉进程输出写入 `logs/front.log`。每个处理帧输出一条
`[front][PassGate][Timing]` JSON，不受 `--timing/--no-timing` 周期统计开关影响。
普通 YOLO 和 DoorSim 入口也输出对应阶段的 `[Timing]` 记录。

```bash
tail -f logs/front.log | grep --line-buffered '\[Timing\]'
```

`frame` 对应处理帧号；`timing_ms` 为主机墙钟耗时（毫秒），包含校正、YOLO、
CV、几何/姿态整理、绘制、JPEG、共享帧及结果写入。
`yolo_timing_ms` 包含输入预处理、BPU 推理、解码/NMS、结果格式化及总耗时；
`model_load_ms` 单独标出首帧/切模型时的加载时间。
`cv_profile` 原样保留全部已有 CV 阶段、主线程 CPU 时间、最终等待和事件收集时间；
`gpu` 原样保留 GPU 内核、操作、传输及同步诊断。各字段中的毫秒值沿用后端原有精度。
设备事件时间与主机时间存在包含关系，不能直接将所有字段相加。
无目标时 `cv_mode=skipped-no-target`、`opencv=0`、`cv_profile=null`，避免复用上一帧数据。
日志本身不增加 GPU 同步或中间图像回读；完整逐帧 JSON 会增加日志写入量。

## 验证

```bash
python3 -m unittest discover -s src/to32/move_test/task/task_door -p test_vision.py -v
python3 -m unittest discover -s src/to32/move_test/task/task_door -p test_control.py -v
python3 -m unittest discover -s src -p test_front_stream.py -v
```

OpenCL host harness 只检查内核正确性，不用于 M4 性能测试或 Mali 耗时估计。
