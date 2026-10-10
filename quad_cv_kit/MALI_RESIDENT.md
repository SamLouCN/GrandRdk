# Mali-G78AE 常驻 GPU CV

`demo/demo_video.py --pipeline red-gate --cv-backend opencl` 默认使用新的
`resident` 路径。库调用需要显式设置 `YoloRedGateTracker(...,
cv_backend=gpu, cv_execution='resident')`；旧调用保持 `hybrid` 行为。
GrandRdk 的 `task_door_sim` 已接入 v7 resident 路径，前摄为 640×480，
默认门模型为 `door_6_nashe_640x640_nv12.hbm`。启动命令见
[task_door_sim/README.md](../src/to32/task_door_sim/README.md)。

## 执行位置与数据流

解码后的 BGR 上传一次 → GPU 可选旋转/矫正 →
矫正 BGR 回读供外部 YOLO/BPU 和输出绘制使用。干净的矫正图保留在
GPU 上，通过只包含 shape 的只读引用交给 CV，不回读后再上传。

CV 内部以下算法全部在 OpenCL 上执行：

| 阶段 / 日志名 | GPU 工作 |
|---|---|
| `input_and_target` | YOLO 框关联、边界判断、ROI、搜索调度、面积缩放、灰度 |
| `color_evidence` | HSV/LAB-a 查表、多尺度局部颜色、红绿信号、阈值、形态学 |
| `motion` | 灰度金字塔、分块角点、64 个特征排序、四层 LK、256 个相似变换 RANSAC 假设、颜色重验证 |
| `components` | 红色/白色区域的 8 连通 union-find、区域统计和筛选、白色肘部开运算 |
| `line_extraction` | 颜色 Hough 投票、峰值排序/NMS、并行线段扫描、采样拟合、排序、合并、颜色对比度和端点验证 |
| `models` | 784 组四边组合、侧边支持、残缺门框连接、最近门框选择 |
| `guidance` | 完整门框的平面单应性位姿、重投影验证 |

搜索是否执行由设备上的状态决定；跟踪失败时同一帧在设备上启动搜索。
跟踪成功时 Hough、拟合、完整/残缺模型和连通区域内核跳过主要计算，
仍有少量提交、清零和条件检查开销。灰度金字塔和门框状态跨帧保留。

CV 核心最后一次批量等待读取 800 字节：门框/跟踪/目标状态 512 字节、
控制信息 128 字节、位姿 160 字节。中间图像、线段、Hough 峰值、
组件统计、候选门框、光流和 RANSAC 结果均不回读。
在 1280×720 视频路径中，稳态输出回读约为 2,764,800 + 800 字节：
前者是外部 BPU/绘制需要的干净 BGR，后者是 CV 最终输出。
640×480 DoorSim 路径的稳态 BGR 回读相应为 921,600 + 800 字节。
这不是将整个解码、BPU 输入转换、YOLO 后处理或编码都改为 GPU。

CPU 仍负责一次性的查表/相机映射初始化、OpenCL 提交、外部检测输入、
最终字典格式化、绘制和编码；常驻 CV 不调用 CPU 的 LAB、LSD、CCL、
角点、LK、RANSAC 或 PnP 算法。Mali 采用共享系统内存，数据回读仍会
触发同步/缓存处理，并不意味着没有代价。

## 历史 v4/v5 优化

用户提供的 v4 第 16 帧，1280×720 输入、640×360 画布：整帧
81.883ms，CV wall time 56.230ms、主线程 CPU 4.211ms。
GPU 分组事件总时长 51.495ms；final_join=52.374ms 是等待尚未完成的
GPU 工作与最终读回，不能再加到事件总时长上。这是单帧，非均值/P95。

| 阶段 | v4 Mali 实测 ms | 本轮 v5 处理 |
|---|---:|---|
| input_and_target | 1.300 | 保持 |
| color_evidence | 13.859 | 保持 |
| motion | 4.091 | 保持 |
| components | 8.185 | 保持 |
| line_extraction | 22.660 | 优化峰值抑制与线段 TopK |
| models | 1.247 | 保持 |
| guidance | 0.153 | 保持 |

视频预处理保持无锐化，enhancement=0。v3 的 19.450ms 峰值选择已在
v4 改为三个内核，总成本 7.681ms（prepare=0.342、order=1.903、
select=5.436）；比较时必须统计三个内核的总和。

### v5：三个字的精确贪心峰值抑制

1. `rg_peak_prepare`：保持原 float32 余弦/距离阈值，逐峰检查 ±4
   角度桶、每桶 8 个峰值。72 个连续循环槽位最多触及三个 32 位字，
   每个候选只存基址与三个抑制掩码（16 字节）。
2. `rg_peak_order`：保持 v4 并行排名，票数降序、原始索引升序。
3. `rg_peak_select`：保持一遍扫描和贪心依赖，每个接受峰值用三次 OR
   更新局部 bitset，替代原来的 72 次邻居读取/判断/位更新。

抑制表从 829,440 字节（810KiB）降到 46,080 字节（45KiB）。
局部 bitset 仍为 360 字节。0/π 环绕、同分顺序、横竖配额及
A→B→C 抑制链语义保持；没有缩减峰值上限或降低搜索频率。
日志 `peak_selector.total_gpu_ms` 包含以上三个内核；
`selected=device-parallel-rank-mask-greedy-nms`，`suppression_words_per_peak=3`。

### v5：线段分块稳定 TopK

v4 `rg_rank` 总计 5.443ms；最多 5,632 个拟合槽位，每个有效槽位
扫描全数组并重复计算长度×sqrt(管宽)×支持率。v5 保留原 float32
得分公式与同分时原槽位升序，改为：

1. `rg_rank_tiles`：每块 256 槽位，一个 64 线程工作组；有效行仅
   计算一次得分，在 2KiB 局部存储中稳定 bitonic 排序，压紧有效行，
   每块只保留 Top128 的得分/原始索引，线段正文不反复搬运。
2. `rg_rank_merge`：两两合并有序 Top128，输出的每个排名独立通过
   二分分区定位。22 块按 22→11→6→3→2→1 合并，奇数尾块直接复制。
   一块内低于 Top128 的行不可能成为全局 Top128，因此没有近似截断。
3. `rg_rank_gather`：最后只搬运入选 128 行，空缺明确写零。
4. `rg_rank`：最终 128→每方向 Top8 的小排序保留稳定排名，工作组
   缓存得分，每个有效行也只开方一次。输出方向配额与旧实现一致。

跟踪帧由设备 control 跳过排序主计算。所有得分、索引与合并状态
均常驻 GPU，无新增 CPU 计算、计数回读或中间图像传输。
日志 `line_ranker.total_gpu_ms` 汇总四种内核（含全部合并轮次及两次
方向筛选），不能只比较新 `rg_rank` 的时间。
整个路径标记为 `pipeline_version=5`。

v5 板端第 22 帧：峰值选择合计 3.384ms，线段排序合计 0.412ms。
CV 48.587ms，整帧 74.442ms；相比 v4 提供的第 16 帧分别下降 55.9%、
92.4%、13.6%。不是相同帧的受控对比，也不能代表稳定均值或 P95。

## v6：精确形态学、块连通区域和每角度 Top8

本轮只改变整数计算的组织方式，不改变阈值、ROI、分辨率、连通性、
候选容量、同分顺序、跟踪/搜索间隔或 float32 颜色/拟合/位姿计算。

### 形态学融合

`morph3_fused` 每组 64 个工作项处理 16×16 输出及完整 halo。两步
闭运算和四步闭/开运算分别在一次内核中完成；中间结果在两个局部
字节图块之间切换，不再逐遍写出/读入整图。

逐步保留原始 3×3 最大值/最小值和图像边界语义：膨胀边界为 0，
腐蚀边界为 255，每个中间图像也重新设置下一步所需的边界值。
只延伸一次输入边界会在混合运算时改变角落，因此没有采用这种近似。
ROI/valid-mask 周边的图内零像素仍作为实际像素处理，不将 ROI 边缘
误当成图像边缘。四步运算只需两个 24×24 字节局部缓冲，合计 1,152B。

### 精确八连通与组件统计

- `rg_cc_tile`：16×16 块内使用局部原子 union-find 处理全部内部边，
  同时初始化全局统计。局部合并后的标签映射为原图最小像素索引。
- `rg_cc_boundary`：仅跨块边执行全局合并，包括上下/左右以及两条
  对角边，覆盖四块相交的角落，不使用固定轮次的近似标签传播。
- `rg_cc_stats_hash`：每 256 个像素用 512 桶局部哈希表累计面积及
  包围盒，每个根节点每块只提交一份全局统计。最多 256 个唯一根，
  碰撞线性探测不会耗尽容量，不丢弃组件、不限制组件数量。
- `rg_cc_filter` 与 `rg_white_open2` 的筛选/白色开运算规则保持。

哈希表占 12KiB 局部内存，低于 Mali-G78AE 的 32KiB 限制。
保持原八连通筛选规范，准确统计面积及包围盒；全局根为组件最小像素索引。
局部/全局合并使用 CAS，仅联结仍为根节点的标签；竞争失败后重新找根，
避免覆盖另一线程已安装的父边而分裂组件。测试会重复执行真实线程
交错下的标签/统计对照，不只检验单次运行。

### 每角度精确 Top8

`rg_peak_top_tiled` 对每个 rho 只检查一次原 3×3 局部极大值规则。
每个工作项保留自己负责的 rho 中准确的 Top8，再以六层局部并行合并
得到该角度 Top8。每组 64 项，双缓冲得分/索引共 8KiB 局部内存。
工作项内低于 Top8 的峰值不可能进入整个角度 Top8，因此没有近似剪枝。
保留票数降序、rho 升序、相邻角度的索引同分抑制、角度资格判断，
以及无峰值/跳过搜索时的全部输出字段。

旧 `morph3`、CCL 和八次扫描峰值内核保留供精度对照；resident 视频
不提交旧 CCL/峰值内核，形态学由六次提交变为两次，无新增中间回读。
整体标记为 `pipeline_version=6`，板端日志重点比较以下完整成本：

| 成本 | v5 第 22 帧实测 ms | v6 日志字段 |
|---|---:|---|
| 六遍形态学 | 5.224 | `gpu.morphology.total_gpu_ms` |
| 全部连通区域及白色开运算 | 8.531 | `gpu.components.total_gpu_ms` |
| 每角度峰值提取 | 3.512 | `gpu.angle_peak_top.total_gpu_ms` |
| CV wall time | 48.587 | `cv_profile.total_ms` |

用户提供的 v6 第 34 帧：CV 46.081ms，整帧 71.695ms；形态学
4.797ms、组件含白色开运算 8.920ms、每角度 Top8 1.842ms。
Top8 改善明显，形态学收益有限，组件成本未改善。v5/v6 来自不同帧，
不构成同帧均值/P95 对照。此前 38–42ms 估算偏乐观，未达到该目标。
未运行 M4 OpenCL，也未使用 M4 性能数据调参。

## v7：二值位打包、游程连通区域和完整峰值排序

### 二值形态学

`morph_pack` 将每行 32 个 0/255 像素打包成一个 uint，行尾不足 32 位
单独处理。`morph_packed1/2/3/4` 使用固定步数，在 256×8 输出图块中
融合逐遍运算：水平位移加跨字进位，垂直三行 OR/AND，最后直接解包
为 0/255 图像。两遍/四遍常驻操作分别使用该组内核，不再提交字节
`morph3_fused`。包含打包，均为两次提交；成本必须把两次相加。

每遍仍重新应用膨胀 0、腐蚀 255 的图像边界，包括行尾无效位；图内
ROI/valid-mask 的零像素仍为实际零像素。横向一整字 halo 的 32 位
宽度超过四遍操作的传播距离；纵向保留逐遍完整 halo。
四遍使用两个 10×16 uint 缓冲，共 1,280B，不依赖 subgroup 扩展。
仅在调用方确认二值掩码时启用 `binary=True`，灰度形态学保持原实现。
阈值后的颜色/严格红色掩码符合该条件，所有步骤仍在 GPU 上。

### 精确游程八连通

1. `rg_cc_runs`：每行一个 64 项工作组，提取全部连续前景区间；用
   局部前缀和得到稳定的行内次序，建立像素到游程的映射。每行容量
   为 `ceil(width/2)`，覆盖交替前景的最大数量，没有丢弃/近似截断。
2. `rg_cc_run_link`：二分定位上一行第一个可能相交的游程，连接所有
   区间相交或横向相距一像素的游程，准确保持八连通。根连接仍使用
   CAS 重试，无固定传播轮次；最终标签为组件最小像素索引。
3. `rg_cc_run_stats`：按游程长度累计面积与边界框，每个游程贡献一份
   统计，取代每个像素的统计哈希操作。只初始化真实游程起点的统计，
   不再逐像素初始化五个统计字段，也不再初始化 12KiB 局部哈希表。
4. `rg_cc_run_filter`：把准确组件标签和原筛选结果映射回全部像素。

白色 2×2 开运算 `rg_white_open2` 规则保持。棋盘格有大量短游程，
收益可能低于稀疏长线段；必须按真实视频分布测量，不能声称通用加速。

### 完整稳定峰值排序

`rg_peak_sort_tiles` 对 12 个 256 槽位块做稳定整数元组排序，排序规则
为票数降序、key 升序、原槽位升序；每组局部存储 3KiB。
`rg_peak_sort_merge` 通过并行 merge-path 做完整合并，块数
12→6→3→2→1、块长度 256→512→1024→2048→4096。
`rg_peak_sort_order` 输出全部 2,880 槽位的顺序，无效槽位写 -1。
不再执行平方复杂度的 `rg_peak_order`；整数票数不转换为 float。

NMS 保留原贪心抑制链、三字抑制记录、余弦/距离阈值和方向配额。
排序不能预先只留 Top256：高分峰被抑制后，较弱峰仍可能需要入选。
所有排序/计数状态均留在设备，没有新增中间回读。

`pipeline_version=7`；当前没有 v7 Mali 性能实测，也不保证已达到 30ms。
以下成本汇总包含整个新算法，不能仅比较同名旧内核与某个新内核：

| 日志字段 | 包含的成本 | v6 单帧基线 |
|---|---|---:|
| `gpu.morphology.total_gpu_ms` | 打包与全部融合/解包调用 | 4.797ms |
| `gpu.components.total_gpu_ms` | 两次游程 CCL 与白色开运算 | 8.920ms |
| `gpu.peak_selector.total_gpu_ms` | prepare、分块排序、全部合并、顺序输出、NMS | 3.536ms |
| `cv_profile.total_ms` | 完整 CV wall time | 46.081ms |

精度检查覆盖二值逐位/灰度逐字节对照、跨 32 位字/256 像素块、奇数
宽度、逐遍图像边界、ROI 边缘；组件最小索引标签/面积/边界框/筛选
像素与 OpenCV 八连通对照，包括棋盘格容量上限和多游程连接一条长
游程。峰值的全部排序槽位用独立整数排序校验，NMS 回归包含抑制链、
角度环绕和方向配额。验证专用回读不会进入实时视频路径。

## 算法差异与质量限制

这是面向 GPU 的新检测器，不是 OpenCV LSD 的数值等价移植。
使用干净图像的一份颜色证据，LSD/CLAHE 被颜色 Hough 替代；拟合使用
128 个横截面，搜索峰值每角度最多 8 个、总共最多 256 个，合并前
最多 128 个拟合线段，完整/残缺各保留每方向 8 条线。
固定容量限制了最坏工作量，也可能漏掉拥挤画面中的弱门框。
不会静默回退到 CPU LSD。`candidate_capacity` 和 `candidate_generator`
记录这些限制，`cv_quality=resident` 表示独立算法而非 precise 的等价结果。
最近门框选择保留 12% 管宽容差、上一帧重叠优先、可见长度打破同宽平局。

完整门框位姿使用平面单应性分解和重投影拒绝，尚未实现 IPPE 的
双解歧义检验；残缺门框只输出观测边，不生成位姿或推测补全。
应在 S100 上比较实际视频的漏检、误检、角点误差和目标切换。
合成场景回归通过不能代表实际场景精度已经一致。

## 仅在 S100/Mali 上采集性能

在 GrandRdk 根目录先运行板端自检，再测试视频：

```bash
python3 quad_cv_kit/demo/check_opencl.py \
  --gpu-device Mali --resident-only --benchmark-resident-frames 30 \
  --report quad_cv_kit/logs/mali-resident-check.json

python3 quad_cv_kit/demo/demo_video.py quad_cv_kit/res/test.mp4 \
  --pipeline red-gate --backend hbm \
  --weights models/door_4_nashe_1280x1280_nv12.hbm --class-names door \
  --cv-backend opencl --gpu-device Mali --cv-execution resident \
  --cv-blur pyramid --cv-budget-ms 30 \
  --out quad_cv_kit/runs/mali_v7 --perf-log quad_cv_kit/logs/mali_v7.log
```

自检先执行 v7 精度检查：形态学与旧 GPU 逐遍内核像素对照、组件标签/
面积/包围盒与 OpenCV 八连通对照、全部峰值记录与旧 GPU 八次扫描对照，
以及 NMS 前全部峰值的稳定排序对照。
这些验证专用的中间回读不参与视频或随后 benchmark 的统计。检查失败
时自检退出非零；应先通过再测视频。

自检使用合成场景，并排除 YOLO/预处理/编码；真实视频日志更能反映
复杂场景。视频和模型路径需替换为板端实际文件。
`--cv-budget-ms 30` 只统计达标率，不截断计算，不保证时限。
比较旧路径时换成 `--cv-execution hybrid`，输出到另一个目录和日志。

检查 `device=Mali-G78AE`、`device_type=gpu`、
`cv_profile.timing_kind=device-events`，以及：

- `cv_profile.total_ms` / `timing_ms.opencv`：CV wall time；
- `cv_profile.stages_ms`：按工作阶段分组的 GPU 事件时长，非 CPU wall checkpoints；
- `cv_profile.final_join_ms`：等待尚未完成的 GPU 工作和最终读回，不能再加到事件总时长上；
- `gpu.kernel_ms`：更细的内核事件耗时；
- `gpu.pipeline_version=7` 与本轮 `total_gpu_ms`：确认位打包形态学、游程 CCL 和完整峰值排序的成本；
- `gpu.peak_selector.total_gpu_ms`、`gpu.line_ranker.total_gpu_ms`：继续监控 v5 优化的两个成本；
- `gpu.transfer_bytes`：应无 mask、score、peaks、partial-mask 等中间回读；
- `run_summary.cv_modes`：搜索/跟踪的 mean、P95、max、30ms 达标率。

矫正 wall checkpoint 包含矫正、必要的 BGR 回读与等待；`details_ms.enhancement`
固定为 0，`gpu.enhancement_mode=none`。实际 remap 内核耗时查看
`gpu.kernel_ms.remap_bgr`，完整预处理查看 `timing_ms.preprocess`。
视频入口已移除 `--sharpen` 参数，旧命令需删除该参数。

本机仅通过 `tests/opencl_host.py/.cpp` 在 CPU 上编译/执行实际 OpenCL C
源代码以检查算术、屏障、连通性、跟踪和输出传输。该方式的时间标为
`host-kernel-emulation`，不能用来估计 Mali 性能，不进行 M4 GPU 调参。

v5 验证：全套 202 项测试，200 项通过、2 项因 FFmpeg 不可用跳过；
生产内核通过 OpenCL C 1.2 语法检查。新增峰值测试将全部 2,880 个
候选的压缩抑制掩码与同表达式的 OpenCL 穷举结果逐字比较，覆盖角度
环绕、位边界和贪心抑制链。排序覆盖 5,632 个密集/稀疏槽位、跨块
同分、奇数合并层、空输入、方向配额和跳过搜索。视频/完整 CV 回归
继续确认无中间回读、最终 CV 输出为 800 字节。

v6 验证：全套 209 项，207 项通过、2 项因 FFmpeg 不可用跳过；生产
内核通过 OpenCL C 1.2 语法检查。新增逐像素形态学/全部峰值记录对照，
八连通最小索引标签、面积、包围盒、筛选像素对照和哈希碰撞检查。
组件检查重复 12 轮；修复竞争根连接后，针对性随机场景另连续 100 轮
通过。完整/残缺、弱红色、白接头、嵌套门框与跨帧跟踪的全状态输出
通过 v5 参考路径对照。视频与板端自检入口回归确认验证回读不污染
实时统计，CV 核心仍仅最终 800B 输出。

v7 验证：全套 211 项，209 项通过、2 项因 FFmpeg 不可用跳过；
游程端点提取的最后一次并行调整后，重新执行的 9 项基元/完整流程
精度检查全部通过。最终生产内核通过 OpenCL C 1.2 语法检查。
这些运行仅为 CPU 主机执行实际 OpenCL C 的正确性验证，未运行
M4 GPU/OpenCL 性能测试，也没有据此报告 Mali 耗时。
