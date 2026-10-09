# camera_kalman — 视觉穿门卡尔曼滤波（viskf）

用 YOLO 门框的**中心 x/y + 面积 S** 做 AUV 穿门的视觉伺服滤波，输出偏航 / 升降 / 前向
三个通道的估计量与变化率。

- 方案文档：本工作区 `视觉穿门卡尔曼滤波方案.md`（本次修订见 §11，**§3.1 的"出框兜底"与 §3.3 的重捕口径已被 §11 覆盖，以 §11 为准**）
- 部署位置（2026-10-01 起）：**`/userdata/GrandRDK/src/kalman/camera_kalman`**
  （与深度卡尔曼 `src/kalman/depth_kalman` 并列；老的 `/userdata/kalman/camera_kalman` 是空壳，可删）
- 依赖：**纯 Python 标准库**（无 numpy / cv2 / 串口 / 相机），Python 3.8+
- ★ **按宿主工程的分类拆开**（2026-10-01 定稿，**不各保一套 `config/` + `src/`**）：
  - **配置** → `<宿主>/config/viskf_config.py`（即 `/userdata/GrandRDK/config/viskf_config.py`）
  - **源码** → 平铺在本目录（`viskf.py` 与 `run.sh` 同级，**不再有 `src/`**）
  - 代价：PYTHONPATH 必须是 `<宿主>/config` + `<本目录>` **两段**（`run.sh` 已自动处理）
  - `viskf.py` 的配置搜索顺序：显式 `--config` → 环境变量 `VISKF_CONFIG` →
    `<工程根>/config/` → 往上 4 级逐个找 `<宿主>/config/` → `quick_config.py`

## 0. 滤波器是什么（先看这里，别猜）

**3 个并联的、互相不耦合的 2 维匀速（CV）线性卡尔曼 —— 不是 EKF。**
三个通道 `f_x / f_el / f_s` 完全独立，每个都是：

```
状态 x = [z, ż]
F = [[1, dt], [0, 1]]                          匀速模型（dt 变步长，clamp 到 dt_min~dt_max）
H = [1, 0]                                     只观测位置，速度不可观测
Q = q · [[dt³/3, dt²/2], [dt²/2, dt]]          连续白噪声加速度模型（密度 q）
更新用 **Joseph 形式** P' = (I−KH)P(I−KH)ᵀ + KRKᵀ（数值稳定，不易失去正定性）
带 nσ 新息门限，且**门限判定在改状态之前**（被拒的野点不会污染状态）
```

所有非线性（roll 像面反旋转、`atan2`、`√(w·h)`）都在 `measure()` 里**前置**做完，
所以进滤波器的已经是物理量，F/H 都是常数矩阵 —— **不需要雅可比、不会线性化发散**。
代价：线性化误差被折进观测噪声 R（实测残差是观测 σ 的 1.5~3.5 倍的原因之一）。

> 要真 EKF（`H=∂h/∂x` 每拍传入）的参照：同级的 `depth_kalman/kf.py` 的 `EKF` 类。

**输出可信度**：`trust` 位 + `age` 字段。门离场后 `gate_visible` 会立刻变 False，
但 `coasting` 窗口（0.5~2s）内 `track` **仍是 1**、滤波器**仍在往外推**——
实测未修复前 `e_x` 能从正常上限 +0.60 漂到 **+2.38**（修复后同场景只到 **+0.49**）。
所以控制侧不要只看 `gate_visible`，要看 `trust`（详见 §4）。

> **当前状态**：滤波器可独立运行，输出 `/dev/shm/momo_viskf.json`。
> 已在真机视频上做过离线评测并按结论优化过一轮（`D:/RC/Test_any/README_测试报告.md`）。
> **2026-10-09 起穿门主闭环已移到 `front.py` 前端管线**（`task/task_door/front_pipeline` 直接消费
> `momo_det_front.json`），viskf 目前**仍无消费者** —— 先跑起来录数据，接回时沿用 §8 两条口径。

---

## 1. 目录

| 路径 | 作用 | 迁移时 |
|---|---|---|
| **`<宿主>/config/viskf_config.py`** | **唯一调参入口**（2026-10-01 从本目录搬上去了） | ✅ 必带 |
| `viskf.py` | 主程序：三个 CV 卡尔曼 + 数据源 + 主循环，**纯生产代码**（平铺，与 `run.sh` 同级） | ✅ 必带 |
| `run.sh` / `stop.sh` | 启停（自动把宿主 `config/` 加进 PYTHONPATH） | ✅ 必带 |
| `tests/test_viskf.py` | 11 项自检 + 离线端到端 | ❌ 可不带（删掉不影响运行，只是 `--selftest` / `--mock` 会给人话提示） |
| `logs/viskf.log` | 运行日志（自动建） | ❌ |

生产代码与测试**严格分离**：`viskf.py` 里对 `tests/` 零 import（只有注释和一句提示语）。
拷 `viskf.py + run.sh + stop.sh` 三个文件、再把 `viskf_config.py` 放进宿主的 `config/`，
就是一个完整可用的滤波器。

⚠ **源码不摊到 `src/` 顶层**：统一放 `src/kalman/<名字>/` 这一层，避免与
`src/to32/main.py` 之类的同名文件在 `pkill`/`grep` 时互相误伤。

## 2. 快速开始

```bash
cd /userdata/GrandRDK/src/kalman/camera_kalman

./run.sh --status                # 看实际命中了哪个配置文件 + 数据源新鲜度
./run.sh --selftest              # 11 项无硬件自检（改完参数先跑这个）
./run.sh --mock                  # 离线端到端：合成数据走一遍真实文件读写链路

./run.sh --daemon                # 真机后台跑，日志 logs/viskf.log
tail -f logs/viskf.log
./stop.sh                        # 停止

./run.sh                         # 前台跑（Ctrl-C 退出），调试用
```

其它参数（`--config` / `--shm-dir` / `--loop-hz`）原样透传给 `viskf.py`。

## 3. 数据流

本工程**只读共享内存、只写一个 JSON**，不开相机、不开串口、不碰 `run.sh`。

| 方向 | 路径 | 谁写 | 缺了会怎样 |
|---|---|---|---|
| 读 | `/dev/shm/momo_det_front.json` | `front.py`（前视 YOLO） | 没有检测 → 滑行 → `lost`，输出 `track=0` |
| 读 | `/dev/shm/momo_telemetry.json` | 中位机 To32 `shm_sink` | **自动降级**：当机体水平跑，`flags.att_degraded=True`（不报错、不阻塞） |
| 写 | `/dev/shm/momo_viskf.json` | 本进程 | 原子写（tmp + `os.replace`），读端不会读到半个文件 |

**姿态源（2026-10-08 已接通）**：`momo_telemetry.json` 由 `to32/tel_shm_sink.py` 落盘（每帧写、20Hz 节流）。
缺姿态时 e_x / s_n 两条通道完全正常（它们不依赖姿态），只有 `el`（俯仰补偿后的水平系仰角）会退化成"按机体水平算"。

## 4. 输出字段（`momo_viskf.json`）

| 字段 | 含义 |
|---|---|
| `track` | 0/1，当前是否有效跟踪（注意：`coasting` 期间 `track` 仍为 1） |
| **`trust`** | **0/1，这个数现在能不能信** = `track` 且 `age≤VISKF_TRUST_AGE_S` 且 `sig_x≤VISKF_MAX_SIG_X`。**控制侧应该判它** |
| **`age`** | 距最近一次有效观测的秒数（无观测时持续增长）——最直接的"还能不能用"判据 |
| `gate_visible` | 本帧是否真的看到门（0/1） |
| `e_x`, `de_x`, `sig_x` | 横向偏差（占门宽比例，无量纲，**与距离无关**）及其变化率、标准差 |
| `el`, `del`, `sig_el` | 俯仰补偿后的水平系仰角 (rad) 及其变化率、标准差 |
| `s_n`, `ds_n`, `sig_s` | `√(w·h)/画面宽` 归一化尺度及其变化率、标准差 |
| `flags.clip_lr` / `clip_tb` | 框贴左边或右边 / 贴上边或下边（分开报；R 按方向分开放大） |
| `flags.clip` | 上面两个的"或"（兼容旧消费方） |
| `flags.coasting` | 无观测，纯预测滑行中（≥ `coast_s`） |
| `flags.lost` | 丢失超时（≥ `lost_s`），再出现会整体重置 |
| `flags.att_ok` / `att_hold` / `att_degraded` | 姿态新鲜 / 用保持值 / 当水平跑 |
| `outliers`, `reinits` | 累计拒收帧数、重捕次数 |
| `n_clip_lr`, `n_clip_tb`, `n_reacquire` | 累计贴边次数（分方向）、重捕跳门限次数 —— 回放排查用 |
| `obs{...}` | 本帧原始观测（cx/cy/w/h/x1/y1/x2/y2/score/clip/clip_l/clip_r/clip_t/clip_b） |

三个通道的物理含义：`e_x` → 偏航，`s_n` → 前向，`el` → 升降（只做"到位确认"，不直接闭环）。

**两条口径约定（改代码前必读）**

1. **`e_x` 的归一化口径恒定**：永远 `(cx−x0−dx0)/w_bbox`，**贴边时也不切换**。
   老实现在框贴边时改成"按画面宽归一化"，与非贴边帧相差 1.5~2.1 倍 ——
   真机实测在切换处造出 **0.209** 的观测跳变（正常帧间只有 0.004），
   周期性击穿 3σ 门限，把滤波输出抖动放大到观测的 **3.9 倍**。
   贴边只用**放大 R** 表达质量下降（软信息），不换单位（硬切换）。
2. **贴边分边放大 R**：左右贴边 → 只放大 `e_x`（框宽略偏，实测小 7.6%）；
   上下贴边 → 只放大 `s_n` 与 `el`（框高被裁 ~10%、中心偏移）。
   老实现是"任一边贴边就三通道全放大 4 倍"。

## 5. 配置

改 **`<宿主>/config/viskf_config.py`** 一个文件就够（即 `/userdata/GrandRDK/config/viskf_config.py`）。
搜索顺序（命中即用）：

1. `--config <文件>` / 环境变量 `VISKF_CONFIG`
2. `<工程根>/config/viskf_config.py` ← **独立部署**时走这条
3. `<宿主根>/config/viskf_config.py` ← ★ **并入 GrandRDK 后走这条**（宿主根 = 工程根往上 3 级）
4. 同上但叫 `quick_config.py` ← 回落（读其 `VISKF_*` 段）

**没写的键用内置缺省**（完整清单见 `viskf.py` 的 `DEFAULTS`）。
配置文件存在却加载失败（语法错、缺依赖）会**打印警告**，不会静默用缺省——
`./run.sh --status` 第一行就是实际命中的配置来源。

## 6. 🔴 上真机前必须做的

| 参数 / 事项 | 怎么定 |
|---|---|
| `VISKF_FOCAL_PX = 554.0` | **占位值**。板端对着已知距离的门量框宽，`f = w_px · z / W_door` |
| `VISKF_SIG_EX/EL/S` | **已用真机视频粗标**：`0.0177 / 0.0036 / 0.0043`（排除出框帧）。仍建议静止对门录 30s 复核 —— 且**改动 σ 要一起改 `VISKF_GATE_FLOOR_*`**（≈同量级），否则门限变纯观测噪声、野点反而变多 |
| `VISKF_GATE_FLOOR_X/EL/S` | 新息门限的**附加底噪**（模型误差：机动 / 线性化 / 姿态补偿残差）。当前取值直接抄 σ，**缺实测依据**，有 30s 静止数据后应重估（做法：干净段残差 std ÷ 观测 σ） |
| `VISKF_PITCH_SIGN / ROLL_SIGN` | 对着固定门**手持摇机体**，补偿后 `e_x` / `el` 应基本不动；动则翻号 |
| `VISKF_TARGET_LABEL` | 与 `front.py` 实际筛出的类别一致（当前 `door`） |
| `VISKF_DX0_PX` | 相机相对机体轴线有横向偏置时才非 0 |
| `VISKF_Q_EX/EL/S` | **本次未动**（真机数据量不足）。要定它得录一段缓慢平移，看残差是否白噪声 |
| 遥测率 | 姿态源 < 20Hz 时 `el` 的抑制比会塌到 ~1.2x（滞后误差是**系统性**的，滤不掉）。板端 `POLL_HZ` 默认 10Hz，闭环前必须提到 50Hz |

## 7. 排障速查

| 现象 | 先看这里 |
|---|---|
| `track=0` 且一直 `lost` | `momo_det_front.json` 的 mtime 是否在刷新（`front.py` 在跑吗）；`--status` 会报年龄 |
| `e_x` 一直是 0 | 门上没有 `door` 类别的框，或分数低于 `VISKF_MIN_SCORE` |
| `flags.att_degraded` 恒真 | 遥测无数据（`momo_telemetry.json` 没有在刷新 = 下位机没回 `0x0C`，落盘链路 2026-10-08 已通） |
| 提前 `clip` 然后 `lost` | 正常：门贴满视野 → bbox 被裁 → 检测丢。靠滑行 + 盲走穿门 |
| `trust=0` 但 `track=1` | 正常，这就是 `coasting` 窗口：`age` 超 0.2s 或 `sig_x` 超 1.0。控制侧此时**不能用这个数** |
| `n_clip_lr` 涨得比 `n_clip_tb` 快 | 说明门是**左右**出画（少见，真机数据里只占 10%）。若是这样，框宽 Δw 才真的不可信 |
| 参数改了没生效 | `./run.sh --status` 看**配置来源**那行；再确认键名是 `VISKF_` 前缀 |
| `--selftest` 报缺文件 | `tests/` 没传上去（生产运行不需要它） |

## 8. 与 GrandRDK 的关系（2026-10-01 更新：已并入并接上闭环）

**代码仍然完全解耦**：本工程不 import GrandRDK 的任何模块，也不出现在 `run.sh` 的启动序列里。
联系只有三层：

| 层 | 内容 |
|---|---|
| 数据 | 共享内存约定文件名（§3）：读 `momo_det_front.json`（+ 可选 `momo_telemetry.json`），写 `momo_viskf.json` |
| 生命周期 | 由 `GrandRDK/src/to32/move_test/kalman_launcher.py` 的 `ViskfLauncher` 托管：上位机切 AUV 模式（`mode_auv.on_enter`）自动拉起，退出时（`on_exit`）停掉自己起的那个。配置在 `GrandRDK/src/to32/move_test/task_config.py` 的 `AUV_VISKF_*` 段；**当前 `AUV_VISKF_AUTOSTART=False`（默认不拉起，需要时置 True）** |
| 消费 | ~~`GrandRDK/src/to32/viskf_if.py`~~（已删）。穿门主闭环 **2026-10-09 起在 `front.py` 内**（`task/task_door/front_pipeline` 直接消费 `momo_det_front.json`）；viskf 若要接回，沿用下方两条硬约定 |

⚠ 消费侧的两条硬约定（改本工程前必读）：
① `e_x` 归一化口径**恒定**为 `(cx−x0−dx0)/门框宽`，贴边也不许换成按画面宽；
② 判"能不能用"看 **`trust`**，不看 `gate_visible`（外推期间 `track` 仍为 1）。
