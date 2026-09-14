# 光流算法项目（/userdata/momo_pwmnet）

RDK S100 开发板上的**实时光流算法研究与测速实验**项目。核心能力：把相机画面里的运动矢量算出来并做运动统计。

- `optical_flow_demo.py` / `speed_flow_demo.py`：算法实验室 + **独立演示链路**（CPU 光流 + BPU 单目深度 m/s，Depth Anything V2，无 torch 依赖，板端直跑）。
- `flow_speed.py`（与 vp5.1 联合、2026-09-07 起）：**GPU 光流测速独立进程**——读 vp5.1 **bottom（下视）** 共享帧（NV12），在 Mali-G78AE GPU（OpenCL/UMat）上做稀疏金字塔 LK，输出图像平面速度 **px/帧 + moving%**；**已移除深度链路**（不再加载 DAV2/BPU，BPU 全部让给 vp5.1 双 YOLO）。2026-09-09 起支持按 `[calib]` 标定输出近似 **m/s**（占位标定，真实标定后替换 focal_px/range_m）。

---

## 1. 目录结构

```
/userdata/momo_pwmnet/
├── optical_flow_demo.py      # 光流演示主程序（算法调度框架，无算法实现）
├── speed_flow_demo.py        # 光流 + BPU 深度 测速(m/s) 主程序
├── flow_speed.py             # 独立测速进程: 读 vp5.1 下视(bottom)共享帧(不占相机), GPU(UMat) LK 光流 px/帧 + [calib] m/s
├── flow_speed_cpu.py.bak     # 旧 CPU DIS 光流 + BPU 深度版备份 (2026-09-07 GPU 化前, 回滚用)
├── config.ini                # 全部参数（算法/摄像头/可视化/深度/测速）
├── algos/                    # 光流算法包: algo_common + 可选算法(见下)
│   ├── algo_common.py        # 共享组件: HSV 光流渲染 + 运动统计(纯函数)
│   ├── algo_dense.py         # 算法1: Farneback 稠密光流  (mode=dense)
│   ├── algo_dis.py           # 算法2: DIS 稠密光流(推荐)  (mode=dis)
│   ├── algo_sparse.py        # 算法3: PyrLK 稀疏角点跟踪  (mode=sparse)
│   └── algo_tvl1.py          # 算法4: DualTV-L1 稠密光流 (mode=tvl1)
├── flow_frame.png            # 早期光流渲染样图（HSV 编码）
├── depth/                    # (仅独立演示用; flow_speed.py 已不依赖, 2026-09-07 起弃用)
│   ├── depth_anything.py     # Depth Anything V2 BPU 推理封装（hbm_runtime）
│   ├── speed_fusion.py       # DepthSpeedMeter：深度+光流 -> m/s 融合器
│   ├── calibrate_depth.py    # 深度仿射标定工具（capture/sample/solve/apply/once）
│   ├── model/depth_any.hbm   # 官方预编译 BPU 模型（121MB）
│   └── rel_view.png          # 标定参考图（左原图 | 右深度热图，由工具生成）
└── archive/                  # 第一代实现（旧版 flow_*.py，无统一接口，已废弃，勿用）
    ├── algo_farneback.py / algo_pyrlk.py / flow_dis.py
    ├── flow_farneback.py / flow_pyrlk.py / flow_viz.py
```

---

## 2. 硬件环境

| 项 | 值 |
|---|---|
| 开发板 | RDK S100（80 TOPS BPU，A78AE×6） |
| 主相机 | `/dev/video0`：通用 USB Camera（MJPG 640×480 @ ~29fps 实测） |
| 其它视频源 | `/dev/video2/3`：LRCP S400（摄像头阵列，本项目未使用） |
| 板端 IP | 192.168.127.10（SSH）；demo 会自动探测并打印 Web 地址 |
| 显示 | 无桌面（headless）；可视化一律走 **Web MJPEG 推流** |

> ⚠️ 不要用 MobaXterm X11 转发看 cv2.imshow 弹窗：板端无图形桌面，窗口小且全黑、还拖慢主循环。MobaXterm 只当命令行用，画面用浏览器看 `http://<板端IP>:8080/`。

---

## 3. 运行方式

无命令行参数，全部配置在 `config.ini`：

```bash
cd /userdata/momo_pwmnet

# 光流演示（看彩色光流场 + 运动统计）
python3 optical_flow_demo.py

# 测速演示（光流 + BPU 深度 -> m/s）
python3 speed_flow_demo.py
```

浏览器打开 `http://<板端IP>:8080/` 看实时画面（标题叠加 speed / motion 信息）。
板端有桌面（HDMI/VNC + DISPLAY）时也可开窗口；无桌面自动降级为仅 Web。

---

## 4. 两个程序的分工

### 4.1 optical_flow_demo.py —— 光流算法实验室

- **框架/算法分离**：主程序只做采集、可视化、统计；算法在 `algos/` 子目录的 `algo_*.py` 纯函数模块
- **统一接口**：每个算法暴露 `process(prev_gray, gray, frame, cfg, state=None) -> (viz, mean_motion, moving_ratio, state)`
  - 稠密算法（dense/dis/tvl1）无跨帧状态，state 恒为 None
  - 稀疏算法（sparse）的角点经 state 显式传入传出
- **换算法**：改 `config.ini → [algorithm].mode = dense|dis|sparse|tvl1`
- **加算法**：复制 algos/ 下 algo_*.py 改名、实现同签名 process()、mode 指向它，零侵入

| 算法 | 特点 | 适用 |
|---|---|---|
| `dis`（推荐） | Dense Inverse Search，快且准 | 实时、测速 |
| `dense` | Farneback 经典稠密 | 对比实验 |
| `tvl1` | DualTV-L1 变分法，抗噪好 | 大位移/噪声场景 |
| `sparse` | PyrLK 角点稀疏跟踪 | 只需少数特征点 |

输出统计：`mean_motion`（平均位移 px/帧）、`moving_ratio`（运动像素占比）。

### 4.2 speed_flow_demo.py —— 光流 + 深度测速

**链路**：DIS 稠密光流(px/帧) → BPU Depth Anything V2(相对深度 rel) → 仿射转米 → 融合

```
v(m/s) = |flow|(px/帧) × Z_m / focal_px × real_fps
Z_m    = depth_a × rel + depth_b      # 仿射标定
```

- 深度约每 10 帧刷新一次（DAV2 BPU 推理 ~147ms/帧 @518×686）
- 左上角叠加 `speed: x.xx m/s` 与运动占比；静止时显示 `--` 或 0

**抗噪三层（config.ini [motion] 段，2026-09-06 加入）**：
1. `blur_ksize`：光流前对灰度帧高斯平滑（默认 3），压暗光噪点伪流
2. `motion_threshold`：位移小于阈值的像素不算运动（默认 1.5 px/帧）
3. `min_motion_ratio`：运动像素占比低于 2% 判静止，速度直接置 0

---

### 4.4 flow_speed.py —— 与 vp5.1 联合（GPU 光流 · 无深度 · YOLO 资源零占用）

`flow_speed.py` 是**独立测速进程**（vp5.1 run.sh 一并拉起，与图像同启同停）：
不打开摄像头，直接读 vp5.1 **bottom（下视）** 进程写入的共享帧 `/dev/shm/momo_flow_bottom.bin`（NV12），
在 **Mali-G78AE GPU（OpenCV UMat / OpenCL）** 上做稀疏金字塔 LK 光流 → 图像平面速度
**px/帧 + moving%**，不再融合深度；2026-09-09 起读 `config.ini [calib]` 标定输出近似 **m/s**。

```bash
# 与 vp5.1 run.sh 一起起（推荐，由 run.sh 一行拉起并 nice -n 19）
python3 flow_speed.py [--fps 50] [--port 8080]    # 默认 fps=50 (vs 旧版 20)
```

**关键设计 (2026-09-07)**：

- **光流整链路迁 GPU**：`goodFeaturesToTrack + calcOpticalFlowPyrLK` 走 UMat(OCL)；
  实测 (640×480 合成) CPU 整链路 248% CPU → GPU 整链路 27% CPU，前视/下视的
  front/bottom YOLO 不再被光流测速抢 CPU 核。
- **移除深度链路**：`DepthSpeedMeter / DAV2 / depth_any.hbm` 不再加载；
  BPU 推理资源全部让给 vp5.1 双 YOLO（双 YOLO 绑核 0-2 / 3-5 不变）。
- **速度输出**：默认 px/帧 + moving%；2026-09-09 起 `config.ini [calib]` 支持物理标定
  （focal_px 焦距 + range_m 相机到场景平面距离），按针孔横向平移近似输出 m/s——
  `v(m/s) = px位移 × (range_m/focal_px) / dt`；当前为常见值占位（focal≈554、range=1.0m），
  真实标定后替换；`[calib].enable=false` 则保持纯 px/帧。旧 `[speed].px_to_m` 仅显示近似，
  已移交 [calib] 不参与换算。
- **进程独立 + 低优先级**：`nice -n 19`，`OMP/OPENBLAS/MKL=1`（run.sh 注入），不绑核。
- **测速与推流解耦**：测速循环 40-50 fps 全速，Web MJPEG 独立 ~12 fps 节流，
  JPEG 编码不拖累测速。
- **DIS / TV-L1 排除**：cv2 的 DIS OCL kernel 在地平线 OpenCL 栈编译失败
  （依赖 `cl_khr_subgroups`），恰好与默认选稀疏 LK 一致。

**前置条件**：vp5.1 `run.sh` 运行中（`main_config.ENABLE_FLOW_SHARE_BOTTOM=True`，且非 `--virtual`），
**bottom（下视）** 每帧写共享帧；front 默认不写（`ENABLE_FLOW_SHARE=False`，光流只用下视，
2026-09-09 起）。独立跑（脱离 run.sh）也可：`FLOW_PORT=8080 python3 /userdata/momo_pwmnet/flow_speed.py`，
但需自己保证 bottom 在写共享帧。

**回滚**：
```bash
# 1) 还原旧 CPU DIS + BPU 深度版
cp /userdata/momo_pwmnet/flow_speed_cpu.py.bak /userdata/momo_pwmnet/flow_speed.py
# 2) run.sh 启动行去掉 nice -n 19 与 OMP=1 注入（恢复原"普通后台进程"）
```

**与 `speed_flow_demo.py` 的区别**：后者自己开相机（/dev/video0），会与 vp5.1 抢摄像头，
且只演示用（CPU 光流 + DAV2 m/s）。本进程零相机占用，与 vp5.1 同启同停，是产品链路。

---

## 5. 深度标定（重要：当前未标定；仅作用于独立演示）

> ⚠️ 2026-09-07 起：`flow_speed.py`（vp5.1 联合链路）**不再使用深度**，本节只影响
> `speed_flow_demo.py` 独立演示。产品链路测速输出为图像平面速度 px/帧，与深度无关。

**现状**：`depth_a=1.0, depth_b=0.0` —— DAV2 输出的相对深度 rel **直接当米**，数值无物理意义。
实测 rel 值域 **8.3 ~ 18.0**（中位数 14.6，近大远小），被放大 ~14 倍，这就是"静止也测出 2 m/s"的放大器。

**标定原理**：单目深度无尺度，用两已知真实距离参照物解仿射方程：
`a = (Z₁-Z₂)/(r₁-r₂)，b = Z₁ - a·r₁`

**工具**（已就绪并验证）：`depth/calibrate_depth.py`

```bash
cd /userdata/momo_pwmnet/depth
python3 calibrate_depth.py capture          # 抓帧生成 rel_view.png（左原图|右热图）
python3 calibrate_depth.py once <x1> <y1> <Z1_m> <x2> <y2> <Z2_m> --apply
# 例: 参照物A(像素123,200,真实0.5m)、B(像素400,260,真实1.5m)
#     -> 自动解 a/b、验算、写回 config.ini [depth]
```

建议摆 3 个距离点（如 0.5/1.0/1.5 m）做最小二乘，更稳。
标定后 rel 热图 + 原图对照：在 RDK Studio 文件面板或下载后看 `depth/rel_view.png`。

**测速换算另两个参数**（也影响绝对值）：
- `speed.focal_px`：焦距（像素），需内参标定；无标定按 FOV 估算（640 宽 @70° ≈ 457）
- `speed.real_fps`：真实采集帧率，务必填实测值（本项目 ~29，勿填摄像头标称 400）

---

## 6. 测试方法（自检清单）

### 光流 demo（optical_flow_demo.py）
1. 静止画面 10s：`mean_motion` 应 < 1 px、`moving_ratio` < 10%（理想 <5%）
2. 挥手/推物体：`mean_motion` 明显上升、颜色编码出现方向变化
3. 切换 mode（dense/dis/sparse/tvl1）对比帧率与质量

### 测速 demo（speed_flow_demo.py）
1. **静止基线**（画面完全不动）：`moving%` 应 <5%、`median_speed` ≈ 0 或 `--`
2. **标定后**挥手/推物体：速度随运动快慢变化、随距离变远而变小
3. 若静止仍报速度：先查画面是否过暗/无纹理（亮度均值 <30 基本没救），再查曝光是否自动闪烁

> 画面亮度和内容质量是光流测速的前提：**全黑/无纹理场景下任何光流输出都是噪声**。

---

## 7. 已知问题与修复记录

| 日期 | 问题 | 修复 |
|---|---|---|
| 09-07 | vp5.1 联合链路 flow_speed.py 光流仍吃 CPU（248%）且依赖 BPU 深度，抢核/占 BPU | 整链路迁 Mali-G78AE GPU（UMat/OCL）：CPU 占用降至 ~27%（合成帧实测）；深度链路移除（DAV2 不再加载，BPU 全让 YOLO）；速度改输出 px/帧 + moving%；旧 CPU 版备份 flow_speed_cpu.py.bak |
| 09-06 | speed_flow_demo 启动即崩（TypeError） | 深度未就绪时 `moving_ratio=None` 未兜底 → 日志/summary 加 None→0 处理 |
| 09-06 | 静止画面测出 1.2~4.8 m/s 假速度 | 根因=近全黑画面(亮度16/255)噪点伪位移(2~8px) × 未标定 rel(8~18)放大；加抗噪三层 + 提示标定 |
| 09-06 | config fps=400 与实际 29 不符 | 400→30 |
| 09-06 | `/etc/profile.d/resize_screen.sh` 重定向顺序错，SSH/SFTP 通道被 `resize: can't open terminal` 污染，破坏文件下载/apply_patch | 修为 `resize > /dev/null 2>&1`（备份 /tmp/resize_screen.sh.bak） |
| 09-06 | calibrate_depth.py 引用类名作用域错误 | 局部 import DepthAnythingV2 修复 |
| 09-06 | 文件头出现 `resize: can't open terminal` 残留（工具通道污染，磁盘文件本身干净） | resize bug 修复后消失 |

**踩坑备忘**：
- `sed -i 's#pat#repl&#'` 的替换串里 `&` = 整个匹配串，会写坏文件 → 用 python 精确写或转义 `\&`
- 摄像头资源释放慢：停 demo 后 2~5s 再重开，别把"第一次 busy"当硬件坏了
- 板上无 tty 时 `bash -lc`（login shell）可能挂起占死持久 SSH，避免用

---

## 8. 当前状态与待办

**已完成**：
- 4 算法统一框架（optical_flow_demo / speed_flow_demo 独立演示）
- **vp5.1 联合链路 flow_speed.py GPU 化（2026-09-07）**：Mali-G78AE GPU（UMat/OCL）稀疏金字塔 LK，
  输出 px/帧 + moving%；深度链路移除（DAV2/BPU 零占用，BPU 全给 vp5.1 双 YOLO）；
  CPU 占用 248%→27%（合成帧实测）；run.sh `nice -n 19` 最低优先级启动；
  旧 CPU 版备份 flow_speed_cpu.py.bak 可回滚
- 抗噪三层、标定工具就绪、resize 系统 bug 修复、下载通道恢复

**待办**（针对独立演示 speed_flow_demo.py）：
1. **深度标定**：摆参照物 → `once --apply` 解真实 depth_a/b（当前仍是相对单位）
2. **静止复测验收**：正常光照有纹理场景下，确认静止 moving% <5%、speed 归 0
3. （可选）focal_px 内参标定，让 m/s 绝对误差收敛

**一句总结**：产品链路（vp5.1 + flow_speed.py）已 GPU 化、去深度、不抢 BPU；独立演示链路
的"真实性"仍取决于两件事——给摄像头一个正常场景，给深度一个标定。
