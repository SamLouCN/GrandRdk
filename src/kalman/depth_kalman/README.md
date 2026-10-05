# depth_kalman · 深度卡尔曼滤波

融合 **深度计 + 两路朝下超声波高度计 + 三轴加速度**，输出更精确的深度值。

> **定位：独立模块。** 不接入 `GrandRDK/run.sh`、不改 `To32`、**不开任何串口**、
> 不写别人产出的共享内存。只做两件事：读约定的落点文件 → 算 → 写自己的输出。
> 数据落点还没做（P0 未通），所以**现在它读不到任何真实数据**，只能用合成数据验证。
>
> **📁 2026-10-01 归位**：已按宿主工程 GrandRDK 的分类拆开 ——
> 配置进 `/userdata/GrandRDK/config/depth_config.py`，源码平铺在
> `/userdata/GrandRDK/src/kalman/depth_kalman/`（详见 §2）。
> 生命周期由 `src/to32/kalman_launcher.py` 的 `DepthKalmanLauncher` 托管
> （上位机切 AUV 模式自动带起，退出自动停自己起的那个）。
>
> **🧪 2026-10-01（R8）测试归口**：本模块的 `tests/` 已迁到
> `/userdata/GrandRDK/hwless_tests/legacy_depth_kalman/`（全工程测试代码只留 `hwless_tests/` 一处）。
> `./run.sh --selftest` / `--mock` 仍照常用（内部 `TESTS` 变量已指向新位置）。

配套方案文档：工作区 `深度卡尔曼滤波方案.md`（含物理约束推导、分期、回灌设计）。

---

## 0. 结论速览

| 项 | 结论 |
|---|---|
| 现在能跑吗 | ✅ 能。`./run.sh --selftest` 全绿；`./run.sh --mock` 端到端通 |
| 有真实数据吗 | ❌ 没有。F 口遥测 0 帧、A–E 高度计全部 no-reply（见方案文档 §1） |
| 融合有用吗 | ✅ 有用，但**价值在抗漂移，不在降噪**：深度计漂 8 cm 时误差从 4.5 cm 降到 1 mm（**8.8x**）；纯降噪只有 1.1~1.6x |
| 最大风险 | 🔴 加速度单位未标定；`H_M` / `ALT_MOUNT` / 深度计零点全是占位值 |

---

## 1. 快速开始

```bash
cd /userdata/GrandRDK/src/kalman/depth_kalman

./run.sh --selftest        # 合成真值自检（无硬件/无串口/不写共享内存）—— 改完参数先跑它
./run.sh --mock            # 离线端到端：起 mock 喂数 + 前台跑滤波（走 /tmp/dk_mock）
./run.sh                   # 前台真机模式（读 /dev/shm）
./run.sh --daemon          # 后台真机模式
./stop.sh                  # 停止
```

自检的另外两种跑法（等价，不经过 `run.sh`）：

```bash
# 2026-10-01 起 tests/ 迁到 hwless_tests/legacy_depth_kalman/（板端约定：测试代码只在 hwless_tests/）
cd /userdata/GrandRDK && python3 hwless_tests/legacy_depth_kalman/test_depth_kalman.py   # 自带路径引导，cwd 任意
python3 -m pytest -p no:anyio hwless_tests/legacy_depth_kalman/ -q                       # 装了 pytest 时按用例跑
# 老布局下（tests/ 仍在包内）等价于：
#   python3 tests/test_depth_kalman.py
```

> ⚠ 板端的 pytest 环境是**坏的**：`anyio` 的 pytest 插件与 pytest 6.2.5 不兼容，
> 不加 `-p no:anyio` 时**连空目录都跑不起来**（`ModuleNotFoundError: _pytest.scope`）。
> 这不是本工程的问题 —— 要么加 `-p no:anyio`（已实测 11 passed），要么直接用第一种跑法。

真机模式下的常用开关：

```bash
./run.sh --channels B,C          # 高度计通道白名单（空串 = 不用高度计）
./run.sh --h-m 2.90              # H 标定值 = 水面处锚路高度计读数
./run.sh --depth-zero 0.00       # 深度计水面零点
./run.sh --accel-unit m/s^2      # 加速度单位标定后填（unknown 时加速度自动关闭）
./run.sh --no-ba                 # 关掉加速度零偏状态
./run.sh --dry-run               # 只算不落盘
./run.sh --once                  # 跑一拍就退出（冒烟）
```

> `--mock` 会自动补 `--h-m 2.9`（mock 真值 H=3.0、锚路 dz=0.10）。
> **参数不对时高度计会被门限全拒，融合退化**——这是刻意保留的真实行为，不是 bug。

---

## 2. 目录结构

> ⚠ **2026-10-01 已按 GrandRDK 的分类拆开归位**（原来是一个自洽的独立包：
> `<包根>/config/` + `<包根>/src/）。现在：
> - **配置** → 宿主工程的 `config/depth_config.py`（与 `auv_config.py` 等统一）
> - **源码** → 平铺在本目录（**不再套 `src/`**），与 `run.sh` 同级
>
> 所以 **生产代码 = `<宿主>/config/depth_config.py` + 本目录的 `*.py` + `run.sh`**。
> 测试代码已于 **2026-10-01（R8）迁出本目录** → `<宿主>/hwless_tests/legacy_depth_kalman/`
> （`run.sh` 的 `TESTS` 变量指过去，找不到时回退老位置 `./tests/`）。删掉 `hwless_tests/` 程序照样跑。

```
/userdata/GrandRDK/                            ← 宿主工程根
├── config/
│   ├── auv_config.py          AUV 任务的唯一调参入口
│   └── depth_config.py        ★ 本模块的全部可调参数（从包里搬上来的）
└── src/kalman/depth_kalman/                   ← 本目录
    ├── README.md              ← 本文件
    ├── run.sh / stop.sh       独立启停（PID 文件在 logs/）
    ├── kf.py                  EKF 内核（预测 / Joseph 标量更新 / 夹紧 / 重置）
    ├── model.py               状态与观测模型、姿态扣重力、量纲换算
    ├── fusion.py              一拍编排：预测 → 逐路更新 → 自适应门限 → 保护 → 组包
    ├── sources.py             读 momo_telemetry.json / momo_alt.json（mtime 判新鲜度 + 解析缓存）
    ├── sinks.py               原子写 momo_depth.json + 轮转日志
    ├── samples.py             样本容器
    ├── cfgutil.py             配置装载与派生规则（单位未知强制关加速度等）
    ├── main.py                入口循环
    └── logs/                  depth.log / *.pid

/userdata/GrandRDK/hwless_tests/legacy_depth_kalman/   ← 测试（2026-10-01 R8 迁入）
    ├── test_depth_kalman.py   11 个场景的合成真值自检（无硬件）
    ├── simtruth.py            合成真值 + 传感器模型（自检与 mock 共用）
    ├── mock_feed.py           离线喂数器（按真机相同 JSON schema 写文件）
    └── conftest.py            只有 pytest 才加载的路径引导
```

**为什么源码不摊到 `src/` 顶层**：主程序叫 `main.py`，而板端已有 `src/to32/main.py`，
摊平后 `pkill`/`grep` 的 `src/main.py` 会误伤中位机进程，所以统一放 `src/kalman/<名字>/` 一层。

**PYTHONPATH 必须是两段**（`run.sh` 已处理好，手动跑要照抄）：

```bash
# ROOT = src/kalman/depth_kalman，宿主根 = ROOT 往上 3 级
export PYTHONPATH="/userdata/GrandRDK/config:/userdata/GrandRDK/src/kalman/depth_kalman"
```

源码里 `import depth_config`（cfgutil.py）与 `import cfgutil/fusion/sinks/sources`（main.py）
都靠这两段解析。本目录 `*.py` 的任何文件**都不会 import 测试代码** —— 已用 grep 核实过
（测试已迁到 `hwless_tests/legacy_depth_kalman/`，迁移后其路径引导显式指向本目录，见该目录文件头）。

---

## 3. 数据接口

### 3.1 输入 A：`momo_telemetry.json`（**该落点还不存在，需 To32 侧新增**）

```json
{ "ts": 1790133164.5,
  "depth_raw": 15000,          // uint16，raw = cm×100；负值被固件钳成 0
  "acc_x": 12, "acc_y": -30, "acc_z": -976,   // int16，×100 编码；单位见 ACCEL_UNIT
  "pitch": 120, "roll": -30,   // 度×100
  "seq": 1234 }
```

字段解析是**宽容**的：`depth_raw` / `depth_m` 都认，`acc_*` 与 `acc:{x,y,z}` 都认，
缺字段就退化成"本次不更新"，文件被写坏也只是本次跳过。

### 3.2 输入 B：`momo_alt.json`（**该落点还不存在，需 read_altimeter 侧新增**）

```json
{ "ts": 1790133164.5,
  "ch": { "B": {"mm": 534, "status": "OK"},
          "C": {"mm": 536, "status": "OK"} } }
```

* `mm` = **探头面到池底的净空**（不是深度）；
* 只认 `status == "OK"`（大小写不敏感），`EMPTY/ERR/DOWN` 一律丢；
* 建议**全部口都落盘**，谁进观测由 `ALT_CHANNELS` 决定 —— 这样改通道不用动脚本。

### 3.3 输出：`momo_depth.json`

```json
{ "ts":…, "valid":true, "D":1.5231, "v_z":-0.012, "b_d":0.0031, "b_a":null, "H":2.9038,
  "sigma":{"D":0.0085,"v_z":…,"b_d":…,"H":…},
  "clearance":{"B":1.3800,"C":1.3812},      // 融合后的离底净空 → 可直接给贴底防撞
  "anchor":"B",
  "counters":{"depth":1234,"alt":432,"rej_depth":0,"rej_alt":12,"sat":35,"reset":0,"inflate":0},
  "sources":{"telem_age":0.03,"telem_stale":false,"alt_age":{"B":0.21},"alt_ok":["B"]},
  "flags":{"degraded":false,"accel_used":true,"level_assumed":false,"depth_sat":false,
           "h_mode":"known","ba_enabled":true,"bd_enabled":true,"accel_enabled":true},
  "stats":{"loop_hz":48.2,"steps":12345} }
```

**下游必须看 `valid` 和 `flags.degraded`**：`valid=false` = 还没吃到过任何观测，
数值没有意义；`degraded=true` = 当前没有一路高度计在场。

---

## 4. 模型（一页版）

```
状态 x = [ D , v_z , (b_d) , (b_a) , H ]
  D   深度（水面向下为正）               m
  v_z 垂向速度（向下为正）               m/s
  b_d 深度计零偏/慢漂移（OU 过程）        m       ← 有高度计且 H 已知时才进状态
  b_a 加速度零偏                         m/s²    ← 默认开，需要单位已标定
  H   锚路的有效池底深度                  m       ← = 水面处锚路高度计读数
```

**预测**

```
D   ← D + v_z·dt + 0.5·(a_dn − b_a)·dt²
v_z ← v_z + (a_dn − b_a)·dt
a_dn = g − sinθ·f_x + cosθ·sinφ·f_y + cosθ·cosφ·f_z      ← 比力扣重力，θ=pitch φ=roll
```

**观测**

```
O1 深度计 : z = D + b_d          σ = R_DEPTH
O2 高度计 : z = (H + dz_锚 − dz_i) − D − x_i·θ + y_i·φ
O3 加速度 : 只进预测，不做观测
```

**三道保护**：自适应新息门限（砍野值不砍慢偏）→ 分量夹紧 → 协方差发散重置。
另有一个兜底看门狗：同一路连续被拒 25 次就放大 `P`（应对真正的阶跃）。

---

## 5. 关键参数（详见 `config/depth_config.py`）

| 参数 | 占位值 | 说明 |
|---|---|---|
| `ALT_CHANNELS` | `['B','C']` | 进观测的通道白名单。`[]` / `['B']` / `['B','C']` 都合法，改通道只动这一行 |
| `ALT_ANCHOR` | `''` | 锚路，空 = 取白名单第一个。**只是 H 的数学参考零点**，锚路没数据也照样能算 |
| 🔴 `ALT_MOUNT` | 全 0 | 每路 `(x 右, y 前, dz 下)`。`dz` = 探头面相对**深度计参考点**的竖直偏移（正=探头更低），**不是相对机壳底** |
| 🔴 `H_M` | `1.0` | **水面处锚路高度计的读数**（深度计已归零时）。等于 `H_true − dz_锚` |
| 🔴 `H_SIGMA_KNOWN` | `0.05` | H 的先验 σ。**它直接决定融合深度的绝对精度上限** |
| 🔴 `ACCEL_UNIT` | `unknown` | `m/s^2` 或 `g`。静置读 `acc_z` raw：≈981→m/s²，≈100→g。unknown 时加速度**自动关闭** |
| 🔴 `ACCEL_SIGN` | `(1,1,1)` | 逐轴符号，按实测对齐 |
| 🔴 `DEPTH_ZERO_OFFSET` | `0.0` | 深度计水面零点标定值 |
| 🔴 `R_DEPTH` / `R_ALT_BASE` | `0.01` / `0.008` | 观测 σ，按实测噪声调 |
| `ENABLE_BD` | `True` | 深度计零偏状态（H 已知时才真的生效） |
| `SIGMA_BD` / `TAU_BD` | `0.10` / `60` | b_d 的 OU 稳态 σ 与相关时间 |
| `GATING_N_SIGMA` / `GATING_ADAPT_ALPHA` | `3.0` / `0.1` | 自适应新息门限 |

---

## 6. 自检结果（2026-09-23，11 场景全 PASS）

```
场景                              fused RMSE   raw RMSE      提升
C1 双路 + 加速度(m/s^2) + b_a          0.0059 m   0.0099 m   1.66x
C2 只有单路高度计(B)                    0.0070 m   0.0103 m   1.46x
C3 加速度单位=g                        0.0060 m   0.0099 m   1.64x
C4 加速度关闭(两源融合)                  0.0090 m   0.0101 m   1.13x   ← 两源增益有限
C5 H 走随机游走(rw)                    0.0055 m   0.0099 m   1.78x
C6 无高度计(降级)                       0.0064 m   0.0102 m   1.59x
C7 野值10% + 两段4s掉线                 0.0068 m   0.0103 m   1.51x
C8 深度计全程饱和(只靠高度计)              0.0073 m      NA       —
C9 深度计慢漂移(0→8cm)                 0.0059 m   0.0529 m   8.97x  ★ 真正的价值
C10 H 标定错+5cm 且深度计偏-5cm          0.0487 m   0.0505 m   1.04x  ← 不可分辨，不报警
C11 无任何数据                           不崩、不 NaN、valid=false
```

**这张表本身就是三条重要结论：**

1. **融合对"慢漂移"价值巨大（8.8x），对"纯噪声"只有 1.1~1.6x。**
   因为两路高度计的单次噪声（σ≈1 cm）并不比深度计（σ≈1 cm）小；它们的真正作用是
   提供**第二条绝对基准**，把深度计的长期漂移/零点误差揪出来（靠 `b_d` 状态）。
   所以别指望它能让你在静水里把读数变"更稳"，它保的是**长航时的绝对准确性**。
2. **加速度关闭后只剩 1.12x。** 加速度的价值是让滤波器在两次观测之间能"推算运动"，
   从而把深度计的测量噪声做长时间平均。单位没标定就关掉是**正确的默认行为**，
   但也就别期待降噪效果。
3. **`H_M` 标错会整体平移，而且滤波器不会报警。**（C10）——H 与深度计零点的误差
   符号相反时在数学上不可分辨，融合值会带着系统性偏差、而 `σ_D` 依然很小。
   **所以 H_M 必须在现场用水面读数标定，别拿设计值填。**

---

## 7. 与方案文档的 3 处修订（实施时发现，已改进代码）

| # | 方案文档原文 | 实际实现 | 原因 |
|---|---|---|---|
| 1 | §3.2 `a_dn = g + sinθ·f_x − cosθ·sinφ·f_y − cosθ·cosφ·f_z` | `a_dn = g − sinθ·f_x + cosθ·sinφ·f_y + cosθ·cosφ·f_z` | 文档把 `a_up` 与 `(R·f)_z` 混了，整体差一个负号。原式会让静止时算出 19.6 m/s² |
| 2 | §3.4 新息门限按 `√S` 判定 | 改成 `√(EWMA(ν²) + S + FLOOR²)` | 只按 S 判定时，深度计漂 5 cm 就会让高度计新息持续超限 → 高度计被**自己的门限永久拒掉** |
| 3 | §2.1「不要给深度计设零偏状态」 | 增加 `b_d` 状态，`ENABLE_BD=True` | 文档结论的前提是 **H 当随机游走**。H 固定且已知时 `b_d` 与 `D` **是可分辨的**（高度计给出独立绝对 D，差值即 b_d），且没有它时慢漂移会被 v_z / H 瓜分 |

---

## 8. 排障速查

| 现象 | 先查什么 |
|---|---|
| `valid=false` 一直不变 | 两个落点文件是否存在、mtime 是否在更新（`ls -l /dev/shm/momo_*`）。本项目只读，不负责通数据 |
| `degraded=true` | `momo_alt.json` 没有 `status=OK` 的通道；或全被门限拒了（看 `rej_alt`） |
| 高度计 `rej_alt` 一直涨 | ① `H_M` 对不对（最常见）② `ALT_MOUNT` 的 `dz` ③ 通道口插错 ④ 超声波真被气泡/浊水干扰 |
| `b_d` 一直往一个方向跑 | 深度计确实在漂（正常，融合会把它扣掉）；若跑满 ±0.5 说明标定或量纲有问题 |
| 深度读数在近水面附近是 0 | 固件把负值钳成 0。本项目**主动丢弃** `raw<1`（记 `sat`），这是设计行为 |
| 日志里反复出现「连续被拒 N 次」 | 传感器与模型的系统性偏差已超出门限尺度 → 检查 `H_M` / `ALT_MOUNT` |
| 输出 `σ_D` 很大（>0.1） | 观测没进来，或在漂 → 看 `counters` |
| 想验证"代码没被改坏" | `./run.sh --selftest`（或 `python3 hwless_tests/legacy_depth_kalman/test_depth_kalman.py`），11 场景全 PASS 才算好 |
| 迁移到别的工程 | 拷本目录 `*.py` + `run.sh`/`stop.sh`/`README.md`，再把 `depth_config.py` 放进宿主的 `config/`；`hwless_tests/legacy_depth_kalman/` 不用带 |
| `./run.sh --selftest` 报"找不到 …/legacy_depth_kalman/test_depth_kalman.py" | 测试已迁到 `<宿主>/hwless_tests/legacy_depth_kalman/`；本包只拷了生产代码时属正常。把整个 `hwless_tests/` 带上即可 |
| `ModuleNotFoundError: depth_config` | PYTHONPATH 少给了一段。新布局下必须是 `<宿主>/config` **加上** `<本目录>` 两段（见 §2） |
| 日志写到别的地方去了 | `main.py` 的 `ROOT` 只取一层 `dirname`；源码平铺后若还写成两层会指到 `src/kalman/logs/` |

---

## 9. 边界（明确不做的事）

* ❌ 不开串口。`USB0-4` 归 `read_altimeter`，`USB5` 归 `To32`，谁都不能抢。
* ❌ 不写别人的落点文件。本模块只写 `momo_depth.json`。
* ❌ 不改 `To32` / `GrandRDK` 的任何代码 —— 落点改造是**另一件事**（方案文档 §4）。
* ❌ 不回灌 `$TEL`、不接深度环（级一/级二）—— 等 P2 拿到实测对比数据再谈。
