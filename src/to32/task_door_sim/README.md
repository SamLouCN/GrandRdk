# task_door_sim

独立的纯视觉实验，参考 `move_test/task/task_door` 的前视处理和共享帧发布方式，
复用工程根目录 `quad_cv_kit`。`sim` 表示视觉实验，不生成假检测，不控制推进器。

流程：相机/视频 BGR → 相机校正 → 对比度增强 1.2 + 亮度锐化 0.6 →
整门 YOLO → 最大门/贴边目标连续性选择 → ROI 内 OpenCV 红杆拟合和短时跟踪 →
增强图叠加黄色 YOLO 框、绿色 CV 边及校正坐标下的视觉回正提示 → JPEG 共享帧。
YOLO 和 CV 使用同一张增强图，干净校正图保留为 CV 颜色/白角验证依据。
CLAHE 和额外饱和度增强默认关闭，参数集中在 `config.py`。
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
不依赖 `momo_stage.json`、AUV 模式或任务表，也不写模式/阶段状态。
输入尺寸从实际帧取得；尺寸变化时重建校正和跟踪状态。

## 通过 run.sh 启动

```bash
./run.sh --door-sim
# 同时将中位机以 ROV 模式启动：
./run.sh --door-sim --to32-args "--mode rov"
# 可选：增强参数透传
./run.sh --door-sim --door-sim-args "--contrast 1.3 --sharpen 0.8"
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

可用 `--model` 指定整门模型，`--contrast/--sharpen` 调整增强。
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
