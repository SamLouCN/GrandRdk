# 门视觉链路

本目录只包含相机校正、YOLO/OpenCV 后端、UI 标注和共享发布。
运动状态机、扫视、定深、对中、接近、盲冲、机器人坐标变换和控制观测握手已删除。
`task_config.DOOR_TABLE=[]` 仅为旧配置的兼容空表；不会注册门运动阶段。
其他任务（包括 `t_pass_door_v2`）独立保留。

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
| `test_vision.py` | 无相机、无串口的后端及共享数据链路测试 |

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

## 验证

```bash
python3 -m unittest discover -s src/to32/move_test/task/task_door -p test_vision.py -v
python3 -m unittest discover -s src -p test_front_stream.py -v
```

OpenCL host harness 只检查内核正确性，不用于 M4 性能测试或 Mali 耗时估计。
