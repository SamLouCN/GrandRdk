# S100 的 BPU 与批量 GPU 门框检测

新视频默认路径是 `--cv-execution resident`，其完整 GPU 数据流、算法差异、
Mali-G78AE 的估计耗时和板端测试命令见 [MALI_RESIDENT.md](MALI_RESIDENT.md)。
本文以下批量搜索及 CPU 阶段说明对应旧的 `hybrid` 路径；DoorSim 仍使用该路径。

BPU 执行厂商工具链编译的神经网络。工程通过 `src/function.py` 中的
`hbm_runtime.HB_HBMRuntime` 加载 `.hbm` 并执行 YOLO。OpenCV 的 Mat/UMat、
LSD、Hough、动态合线、连通域和 Python 几何循环没有直接切换到 BPU 的执行入口。
模型编译器能否支持某个算子，取决于对应 S100 SDK、输入形状、精度和编译报告；
不能把整个 OpenCV 函数调用交给模型运行时。

可以训练门杆分割模型，输出杆的像素掩码；也可以训练门角关键点模型，输出角点
及可见性。这能把部分颜色/候选搜索交给 BPU，但需要覆盖褪色、遮挡、贴边、
残缺近门、白支撑腿等样本，重新验证准确率。几何、连续性选择与缺边拒绝仍需
后处理。把固定卷积写成网络虽可能利用 BPU，却不能自然覆盖动态合线和候选枚举。

## 当前 GPU 实现

此改动重写 `red-gate` 检测路径的搜索热点，不替换系统 OpenCV 安装。
`src/gpu_search.py` 调度 `src/kernels/search.cl`；搜索、颜色和驻留 OpenCL 源码一起编译，
仍只依赖厂商 OpenCL 1.2 ICD，不增加 PyOpenCL 或 BPU 模型。
`task_door_sim` 默认的 OpenCL 后端自动使用新路径；CPU 后端保留原检测规则。

| 工作 | 执行位置与数据流 |
| --- | --- |
| 拟合后的有序合线 | GPU 的一个 64 线程工作组按原种子顺序处理，并行比较已合线，选择第一个匹配项；避免改变重叠杆归属 |
| 线段占用采样与断口闭运算 | 全部线段/采样点批量运行，掩码及占用数组留在 GPU |
| 连续段提取 | GPU 按线段、断口顺序压缩输出，不读回占用数组 |
| 干净图/增强图的对比度验证 | 每段每通道一个工作组，沿线五点采样、有效点计数、局部排序计算精确中位数；CPU 不创建每段采样矩阵 |
| 完整模型 | 连续数组表示候选组合；交点、凸性、面积、边长、有效区 33 点采样、逐边颜色证据在 GPU 连续运行 |
| 模型输出 | GPU 压缩通过验证的模型；数量与最多 64 条输出一批读回，按数量切片；超过 64 条时追加读取，保留全部候选 |
| 残缺模型证据 | GPU 批量计算竖横线交点、端点距离，以及每条线 60/80 点颜色证据；CPU 保留图连通、锚点归属和排序 |
| 传输复用 | 一次搜索中拟合、拆分、裁剪、模型验证复用相同掩码；裁剪的数量/结果成批读回 |

搜索局部缓存随搜索结束清除，帧级驻留关联保留到整帧结束。下一帧即使复用并修改
同一个 NumPy 图像数组，也会重新上传；静态只读查表单独缓存。嵌套回退使可覆盖的
命名缓存失效，独立驻留槽继续保留，防止复用已被覆盖的设备数据。
中间像素数组不读回，但变长阶段仍有少量计数读回，不能称为整帧零同步。

CPU 仍运行 LSD 输入的 LAB/CLAHE、连通域、Hough 峰值排序、裁剪后去重、
残缺模型图选择及光流/RANSAC。颜色证据的 LAB 已迁移到 GPU；裁剪后去重使用
可选原生双精度实现，没有 C 编译器时保留 Python 路径。移植其余功能需要准确率/板端收益验证；
纯 GPU 执行不等于端到端最快。现有图像阈值、缺边规则、合线顺序和质量配置保持，
OpenCL float32 在临界值/半像素附近可能与 CPU float64 有差异。

## demo_video.py 与帧内驻留

入口相对路径是 `quad_cv_kit/demo/demo_video.py`。在工程根目录运行：

```bash
python3 quad_cv_kit/demo/demo_video.py quad_cv_kit/res/test.mp4 \
  --pipeline red-gate --backend hbm \
  --weights models/door_4_nashe_1280x1280_nv12.hbm --class-names door \
  --cv-backend opencl --gpu-device Mali --cv-execution hybrid --cv-hough opencl \
  --cv-quality fast --cv-blur pyramid --opencv-threads 3 --cv-budget-ms 40 \
  --out quad_cv_kit/runs/gpu_resident \
  --perf-log quad_cv_kit/logs/gpu_resident.log
```

必须选择 `--pipeline red-gate`；默认 `legacy` 不使用此 GPU 路径。输出 `after.mp4`、
`rows.jsonl`，性能日志逐帧记录 `gpu.transfer_bytes/transfer_calls`，可按缓冲区检查
CPU→GPU 上传和 GPU→CPU 读回；`gpu.residency=frame` 表示该帧已启用驻留范围。

`src/gpu_residency.py` 管理从校正/增强到 CV 搜索/跟踪的整帧设备生命周期：

- 校正、增强和缩小后的图像保留 GPU 副本；720p→640×360 的整数面积缩放在 GPU
  上生成同样的画布。其他缩放比例沿用 CPU INTER_AREA，并在 GPU 首次需要时上传小图。
- 干净图/增强图的 HSV 与 LAB a 在一个 GPU 内核转换，复用 BGR 设备输入；
  LAB 使用本机 OpenCV 生成的 16 MiB 精确字节查表，不再逐帧上传 CPU 色度。
- 颜色掩码和分数读回供 CPU 消费后，GPU 原件继续保留；GPU 上完成 ROI 展开、
  图像清零裁剪、有效区过滤、Hough 区域裁剪和掩码相与，后续拟合/裁剪/验证不重新上传。
- 校正、增强、颜色分数和最终掩码直接写入帧内唯一槽，只读主机副本共享设备输入；
  避免为了保存结果再执行整图 `resident_copy`。ROI/画布按像素处理，各通道共享坐标计算。
  帧内唯一槽避免临时工作缓冲区在嵌套搜索中覆盖驻留数据；下一帧
  清除数组身份关联，防止复用被修改的 NumPy 数据。调用范围内关联的主机数组须只读；
  内部掩码变换返回新数组并同步注册其设备版本。
- 只读校正表、有效区、HSV 和 LAB 查表跨帧保留。线段、计数、Hough 峰值与 CPU 连通域
  筛选结果仍在各自接口上传/读回；不能称为整帧零拷贝。

CPU LSD、光流和视频/BPU 输入接口仍需要主机视图，因此部分读回量不会减少。
要继续消除这些接口，需要移植对应算法或更换输入 API；当前日志能直接指出剩余传输。

## 无增强的视频预处理

`red-gate` 视频入口已删除 CPU/GPU 锐化和 `--sharpen` 参数，仅保留可选旋转
及相机矫正。YOLO/CV 直接使用干净矫正图，BPU/绘制需要的 BGR 只读回一次，
设备副本继续供 CV 使用。`details_ms.enhancement=0`、
`gpu.enhancement_mode=none`，内核列表不再有 `sharpen_only`。
矫正 wall time 包含必要的 BGR 输出等待，remap 事件耗时仍独立记录。
库中独立锐化/增强对照函数仍可显式调用，但不参与此视频流程。
此次仅修改 quad_cv_kit；DoorSim 需另行接入。

resident 峰值选择采用并行排名、邻近角度的稀疏抑制列表和轻量贪心输出，
保留原来的排序/配额/抑制规则。实际收益应比较
`rg_peak_prepare + rg_peak_order + rg_peak_select` 的事件总时长，
日志 `gpu.peak_selector.total_gpu_ms` 已汇总；详见 [MALI_RESIDENT.md](MALI_RESIDENT.md)。

## 队列和搜索调度

新一轮改造以 CV 40ms 为板端验收目标，未以减少检测次数、缩小画布或删除候选来
降低工作量。每个帧内操作不再强制 `clFinish`，批量读回使用最后一次阻塞读保证
前面的主机数组已就绪，错误路径仍完成已提交读回，防止主机内存提前释放。
合线输出容量按种子数预分配，未使用记录在 GPU 清零；合线和拆分计数一起读回，
不在两阶段之间等待合线数量。

小半径高斯把水平和垂直两遍融合在局部内存中；颜色高斯的最后一遍、金字塔上采样
直接计算局部色度证据，省掉全图模糊中间值的写入/读取与单独 `local_contrast` 调用。
残缺杆颜色验证只在 60/80 个查询点计算颜色，不再为了少量采样转换整张 HSV。
运动后的占用率与逐边支持验证在 GPU 批量执行，共享一次读回；CPU 保留光流与 RANSAC。

日志以 `gpu.pipeline_version=2`、`color_conversion=fused-hsv-exact-lab-a` 和
`residency_strategy=direct-outputs-readonly-aliases` 标明新路径。`synchronization`
记录实际 `clFinish` 与读回等待次数，`dispatch` 记录内核提交及参数设置次数；
不变参数不重复调用 `clSetKernelArg`。`profiling_cpu_ms` 记录收集事件的 CPU 开销，
`tracker.gpu_finalize` 把延后的计时收集纳入 CV 总耗时，避免移出计时范围产生虚假提速。
操作计时不再包含各操作独立收集事件的开销，跨版本应比较整帧 CV 与总运行时间。

首次启动生成精确 LAB 表并缓存在 `runs/color_cache`，每次启动上传一次；额外常驻
GPU 存储为 16 MiB，启动成本计入初始化时间，预热后的逐帧统计不包含它。

以下历史传输审计在仅锐化改造之前进行：使用同一 1280×720 合成图、恒等校正、
模拟 YOLO 和完整搜索，将上一版整帧驻留与第二轮驻留实现比较。
生产内核在主机测试环境运行，仅统计传输接口的字节，
不测 S100 的 GPU 性能：

| 每帧传输 | 上一版整帧驻留 | 第二轮驻留（改为仅锐化前） |
| --- | ---: | ---: |
| CPU→GPU 字节 | 4,773,528 | 2,976,600 |
| GPU→CPU 字节 | 6,955,896 | 6,955,928 |

上传量进一步减少约 37.6%，选中检测框一致。额外 32 字节读回是迁移到 GPU 的
跟踪占用率/支持度结果。剩余上传主要是新输入帧 2,764,800 字节和 CPU 连通域
筛选后的掩码 198,720 字节；其余为小型参数和线段。真实视频的组合数量、ROI
和读回容量会变化，不能直接套用此合成场景数据；完整图像读回仍是后续优化空间。

## 验证和板端测量

本机测试把生产 OpenCL 源码编译成主机可执行代码，验证计算、工作组屏障及输出顺序。
这些测试不能证明 Mali 驱动兼容性、GPU 性能或现场准确率。

在 S100 的工程根目录执行：

```bash
python3 quad_cv_kit/demo/check_opencl.py --gpu-device Mali \
  --cv-quality fast --cv-blur pyramid --opencv-threads 3 \
  --benchmark-search-frames 12 --benchmark-frames 12 \
  --report quad_cv_kit/runs/gpu_search_check.json
```

`search_benchmark_summary` 对照旧 CPU 搜索阶段与新 GPU 搜索阶段，使用同一输入、
相同 GPU 颜色/Hough/拟合和质量配置，输出墙钟均值/P95、调用线程 CPU 均值及选中
几何。合成场景只用于定位，实际帧率须比较同一段现场录像的搜索/跟踪统计与发布数量。
`benchmark_summary` 分别测 GPU Hough 和 CPU Hough 配置下完整跟踪器的搜索/跟踪分布。

正常 `task_door_sim` 的 JSON 与 CVProfile 日志新增 `search_execution=gpu-batched`、
`model_execution=gpu-batched`、`partial_execution=gpu-batched-evidence`，只有实际执行
该阶段时才出现。完整几何合并计时为 `models.gpu_batch`，不要再以旧的
`models.geometry/models.valid_mask` 衡量该路径。GPU 日志记录内核和含传输的操作耗时。
新的完整模型不复用旧 CPU 的逐边缓存：`model_side_checks` 是通过几何/有效区检查
的模型边数，`unique_model_side_checks` 不再输出；不能跨版本直接比较这两个计数。

`--cv-budget-ms 40` 仅统计 40ms 内的帧占比，不丢帧、不截断计算。没有 S100
实测数据时，不承诺已达到 40ms、固定 FPS 或提速倍数。
