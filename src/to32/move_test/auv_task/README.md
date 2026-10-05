# auv_task —— AUV 自主运动「任务化」执行框架

替代旧的 `mission.py`（805 行 / 17 阶段单体状态机）。**一个阶段 = 一个独立任务类**，
14 个任务类覆盖 24 个阶段，每个都能单独跑、单独测。

> 水池时间很贵。**能在桌上验的（跳转顺序、超时、判据、选段过滤）别下水验**；
> 下水只验标定类东西（推力↔速度、转向速率、池深、撞击阈值）。

**导航**：§1 三句话 · §2 三种跑法 · **§3 ★启动门控/接管/状态提示** · §4 `test_config` 配置 ·
§5 各任务模块作用 · **★ §6 测试手册（可以测哪些阶段 + 每段测什么 + 推荐下水顺序）** ·
§7 阶段表一览 · §9 `$AUVCTL` 监督 · §10 硬规则 · §11 排障速查 · §12 与旧代码对接

---

## 0. 目录结构

| 文件 | 作用 |
|---|---|
| `test_config.py` | ★ **测试阶段唯一入口** —— 选模块 + 各模块参数 + 全局覆盖 + 离线剧本（改它就够了） |
| `servo.py` | 伺服律（**纯函数**，从旧 `mission.py` 逐行平移，逻辑不许改） |
| `ctx.py` | `TaskCtx` 任务上下文：观测 / 计时 / 参数 / 出口的唯一窄接口 |
| `base.py` | `Task` 基类：参数化 + `enter/tick/exit` 三段生命周期 |
| `servo_if.py` | 投放舵机接口（`ServoStub` 占位，实装后只换工厂） |
| `testcfg.py` | 测试模式：`TestCfg` 合并 test_config 与 `AUV_TEST_*`；`SURFACE` 强制保留 |
| `plan.py` | 阶段表（**改流程只改这里**；自动套 `STAGE_PARAMS` 覆盖） |
| `runner.py` | `TaskRunner` 执行器：顺序 / 跳转裁决 / 预算 / 监督 / `kill()`（零改动对接 mode_auv） |
| `gate.py` | ★ 启动门控：AUV 只能由上位机 `$CMD` 切入（纯函数，可离线测） |
| `notify.py` | ★ 状态提示回传 `$MSG` 帧：当前阶段 / 警告 / 报错 → 上位机终端 |
| `tasks/` | 14 个任务类（每个都能单独跑、单独测） |
| `tests/` | 离线假件 + 台架 + **59 项回归**（Windows 直接跑，不用连板子） |

---

## 1. 三句话记住这个框架

1. **一个阶段 = 一个 `Task` 子类**，只管自己那一段，跳转由参数 `next` / `fail` 声明；
2. 任务**只通过 `ctx` 看世界**（vision / depth / viskf 全是可注入的假实现）→ 能离线跑；
3. 跳转**只放信号**（`ctx.goto` / `abort` / `finish`），真正跳不跳由 `runner` 裁决
   （测试模式过滤、`SURFACE` 强制保留、DONE 收尾都在那儿）。

---

## 2. 快速上手：三种跑法

### 2.1 本机离线（不连板子，改完先跑这个）

```bash
cd src/to32/move_test/auv_task/tests
python run_task.py --cfg          # ★ 打印当前 test_config 摘要（先看这个）
python run_task.py --list         # 列出 24 个阶段 + 任务类 + 生效的参数覆盖
python run_task.py                # 完全按 test_config.py 跑一遍
python run_task.py --full         # 全流程干跑（忽略测试选段）
python run_task.py --stage SIT_BOTTOM -v   # 单段调试（-v 打印每条日志）
python test_auv_task.py           # 59 项回归（改完 plan/runner/gate/notify/任务必跑）
```

### 2.2 板端水池测试（只改 `test_config.py`，不用碰 `auv_config.py`）

```python
# src/to32/move_test/auv_task/test_config.py
TEST_ENABLED = True
TEST_STAGES  = ['SIT_BOTTOM']            # 这次下水只验坐底
STAGE_PARAMS = {'SIT_BOTTOM': {'RATE_CMS': 10.0, 'TIMEOUT_S': 15.0}}
```

★ **怎么开始**：上电后中位机停在 IDLE，**AUV 不会自己跑**。
用上位机点「自主航行(AUV)」（= 发一帧 `$CMD.mode=1`）→ 执行器从这一段开始；
中途点回「有线遥控(ROV)」→ **立刻杀死任务、交还遥控**（§3）。
改完选段想重跑，就是"切回 ROV 再切 AUV"（每次进 AUV 都重建执行器，从头跑）。

→ 跑完进 `SURFACE`（强制，删不掉）→ `DONE` → 请求切回有线 ROV。

★ **每一段具体测什么、常调哪些参数、判成功的标志** → 见 **§6 测试手册**（24 段逐段说明 + 推荐下水顺序）。

启动日志会打这四行，**下水前确认它们跟你写的一致**：

```
[AUV] 任务执行器就绪：24 个阶段，运行模式 = 测试模式(仅 SIT_BOTTOM, 保持判据≤5.0s)
[AUV] 测试模式实际保留：SIT_BOTTOM,SURFACE,DONE
[AUV] test_config 阶段参数覆盖：SIT_BOTTOM←{'RATE_CMS': 10.0, 'TIMEOUT_S': 15.0}
[AUV] test_config 全局覆盖 2 项: AUV_POOL_DEPTH_CM,AUV_SPEED_MPS
```

### 2.3 板端验收 / 比赛

```python
TEST_ENABLED = False        # ★ 验收前必须置回 False（= 进"真实作业模式"）
GLOBAL_PARAMS = {}          # ★ 清空临时覆盖
# AUV_CMD_POLICY 不用改：真实作业模式会被 AUV_ACCEPTANCE_LOCK 自动强制成 ignore（§3.6）
```

进 AUV 后认这两行日志（**模式配错是最贵的错误**）：

```
[AUV] 运行模式 = 真实作业模式（验收）：跑全部 24 段，不接受上位机干预（仅"切模式"生效）
[AUV] 监督指令策略 = ignore（$AUVCTL 全部拒绝）
```

---

---

## 3. ★ 启动门控 / 上位机接管 / 状态提示回传（2026-10-05）

三条规矩，一条比一条关键（**都是水池测试的安全底线**）：

| # | 规矩 | 落地在哪 |
|---|---|---|
| ① | **AUV（作业 / 测试都一样）只能由上位机 `$CMD` 切入** —— 上电默认、`--mode auv` 命令行、模式记忆文件，一律不许自己跑起来 | `gate.py` + dispatcher 补丁 B5/B7 |
| ② | 测试/作业中上位机一切回 ROV → **立刻杀死自主任务**，交还遥控 | `runner.kill()` + `mode_auv.on_exit` 补丁 A5 |
| ③ | 正在跑哪一段、降级/超时/中止原因，**实时显示在上位机终端** | `notify.py` 的 `$MSG` 帧 + 上位机 `MsgThread` |

### 3.1 启动门控（AUV 怎么才算"被允许开始"）

四种模式来源（`mode_dispatcher.mode_source`）：

| 来源 | 含义 | 能进 AUV 吗 |
|---|---|---|
| `pc_cmd` | 上位机 `$CMD` 的 mode 字段 | ✅ **唯一合法来源** |
| `start` | 上电 / `config.START_MODE` | ❌ 回落 IDLE |
| `forced` | 命令行 `--mode auv` | ❌ 回落 IDLE |
| `saved` | 模式记忆文件 | ❌ 回落 IDLE（且 AUV 根本不会被写进记忆） |

★ 失效方向是**安全侧**：`auv_task.gate` 导入失败 → **不进 AUV**（停在 IDLE 等上位机）。
宁可不动，也不能没人看着就自己跑起来。

⚠ 门控只管**启动那一次**（`switch_mode(initial=True)`）。运行期上位机随时能切进/切出 AUV ——
否则上位机就再也切不进 AUV 了。

⚠ `AUV_LOCK_MODE` 必须保持 **`False`**：它一旦为 True 就会忽略 `$CMD` 的 mode 字段，
上位机切不回 ROV，与规矩 ② 直接冲突。

### 3.2 上位机接管：kill ≠ abort

| | 谁触发 | 之后还发 0x09 吗 | 会上浮吗 |
|---|---|---|---|
| `abort()` | 任务自己判"不行了" | ✅ 继续发（把上浮走完） | ✅ 强制上浮 |
| `kill('PC_TAKEBACK')` | **人**（切回 ROV / `$AUVCTL KILL`） | ❌ 一帧都不发 | ❌ 立刻停手 |

`mode_auv.on_exit` 里（补丁 A5）的顺序：`kill()` → 补一帧零推力停推（`AUV_STOP_FRAME_ON_EXIT`，
急停闩锁期间不发）→ 关提示通道。**下次上位机再切回 AUV 会重建执行器，从头开始跑**。

### 3.3 状态提示回传：`$MSG` 帧

另开一条**纯文本**通道，专门送"给人看的一句话"
（`$TEL` 是 41 字段定长、上位机硬编码解析，加字段会破坏老上位机；`$AUV` 是数值快照，装不下中文）。

```
$MSG,<ts>,<level>,<code>,<stage>,<text>#\r\n
  ts    发送时刻 HH:MM:SS
  level INFO / WARN / ERROR     → 上位机终端染色：白 / 黄 / 红
  code  STAGE / ABORT / TIMEOUT / SKIP / DEGRADE / BUDGET / TEST /
        KILLED / PC_TAKEBACK / MODE / SERVO / VISION / HB
  stage 当前阶段名（没有就 '-'）
  text  中文一句话（内部逗号已转 ';'、'#' 已剔除，绝不破坏帧结构；超 160 字截断）
```

显示效果（上位机终端页）：

```
[11:05:12] [INFO] [AUV][STAGE] SEEK_BALL_F: 进入 SEEK_BALL_F（前视找球）
[11:05:30] [WARN] [AUV][DEGRADE] SIT_BOTTOM: 深度源不可用; 定深走定时放行（判据只剩时间）
[11:06:02] [ERR ] [AUV][ABORT] SURFACE: 中止: 找不到球 -> 强制上浮
[11:06:30] [WARN] [AUV][KILLED] RAM_BALL: 自主任务已终止: PC_TAKEBACK（停止下发; 交还遥控）
```

发送策略（防刷屏）：ERROR / WARN **立即发**；INFO 按 `AUV_MSG_HZ` 节流（默认 2Hz）；
相同 `(level, code, text)` 在 `AUV_MSG_DEDUP_S` 秒内不重复；静默 `AUV_MSG_HEARTBEAT_S` 秒后播一次心跳。

与 `AuvReport` 同样四层隔离（**回传绝不能影响主功能**）：独立线程 + 独立 socket、非阻塞、
异常全消化、**连续失败 `AUV_MSG_MAX_FAIL` 次永久放弃**。
★ 无缆验收时上位机 IP 学不到 → 一帧都不发、零开销（不是故障）。

### 3.4 相关配置（`config/auv_config.py`）

| 键 | 默认 | 说明 |
|---|---|---|
| `AUV_REQUIRE_PC_CMD` | `True` | 启动门控总开关；置 `False` = 允许上电/记忆/命令行直接进 AUV（**只在无上位机的验收场景用**） |
| `AUV_LOCK_MODE` | `False` | ⚠ 必须 False；True 会让上位机切不回 ROV |
| `AUV_STOP_FRAME_ON_EXIT` | `True` | 被接管时补一帧零推力停推（急停闩锁期间自动跳过） |
| `AUV_MSG_ENABLED` | `True` | 状态提示总开关（False → 静默，调用方不用判空） |
| `AUV_MSG_IP` / `AUV_MSG_PORT` | `AUV_REPORT_IP` / `8085` | 与 `$AUV` 共用通道，靠帧头区分 |
| `AUV_MSG_HZ` | `2.0` | INFO 节流频率（WARN/ERROR 不受限） |
| `AUV_MSG_MIN_LEVEL` | `INFO` | 低于该级别的提示直接丢 |
| `AUV_MSG_DEDUP_S` | `2.0` | 相同提示的去重窗（秒） |
| `AUV_MSG_HEARTBEAT_S` | `5.0` | 心跳周期；`0` = 关 |
| `AUV_MSG_MAX_FAIL` | `3` | 连续发送失败多少次后永久放弃 |

测试期想临时改，写在 `test_config.py` 的 `NOTIFY` 段（**不用碰 auv_config.py**）：

```python
NOTIFY = {'enabled': True, 'min_level': 'INFO', 'hz': 2.0,
          'heartbeat_s': 5.0, 'dedup_s': 2.0}
```

### 3.5 上位机侧怎么接（`/d/RC/ROV控制站_v3.3`）

1. `protocol.py`：`MSG_PORT = 8085` + `parse_msg()`（已加）；
2. `pc_main2.py`：`MsgThread`（UDP 8085 收帧）+ `_on_msg()` → `_term()` 按级别染色（已加）；
3. `$AUV` 数值快照共用 8085，靠帧头 `$MSG` / `$AUV` 分发，老解析完全不受影响。

⚠ 帧字段数**恒为 6（含帧头 `$MSG`）**；去掉帧头与帧尾 `#` 后是 5 段。
解析时按 6 取会永远返回 None（这个坑踩过一次）。

### 3.6 ★ 两种运行模式：真实作业（验收）vs 调试测试

**由 `test_config.TEST_ENABLED`（或 `auv_config.AUV_TEST_MODE`）一个开关决定**，进 AUV 时自动生效：

| | **真实作业模式（验收）** | **调试测试模式** |
|---|---|---|
| 开关 | `TEST_ENABLED = False` 且 `AUV_TEST_MODE = False` | `TEST_ENABLED = True`（或 `AUV_TEST_MODE = True`） |
| 跑什么 | **全部 24 段**，从 `DIVE` 从头跑 | 按 `TEST_STAGES` 选段，从选中的那段开始 |
| 保持判据 | 原样（不压缩） | 压到 ≤ `TEST_HOLD_MAX_S` |
| 上位机杆位 `$CMD` | ❌ 不执行（只记日志） | ❌ 不执行（只记日志） |
| `$AUVCTL`（HOLD/SKIP/ABORT/KILL…） | ❌ **全部拒绝**（强制 `policy=ignore`） | ✅ 按 `AUV_CMD_POLICY` 受理（默认 `supervise`） |
| `$VID`（图像开关） | ❌ 拒绝（补丁 B8：`return`，连 `set_enabled` 都不调） | ✅ 生效（可随手开关） |
| `$PID`（调参） | ❌ 不下发（只计数） | ❌ 不下发（只计数） |
| `$ESTOP`（急停） | ✅ **永远生效**（安全件，不受任何锁限制） | ✅ 永远生效 |
| **上位机切模式**（`$CMD.mode`） | ✅ 生效（切回 ROV = 立刻终止任务） | ✅ 生效（切回 ROV = 立刻终止任务） |
| 图像 / 数据 / 终端回传 | 无缆 → 本来就没有（不是故障） | ✅ 自动打开图像回传（`:5000`），`$TEL`/`$AUV`/`$MSG` 全开 |
| 启动提示（终端） | `INFO …真实作业模式（验收）：跑全部阶段…` | `WARN …调试测试模式：…` |

⚠ 两边是**或**语义：`TEST_ENABLED` 与 `AUV_TEST_MODE` 任一个为 True 就进测试模式
（防止"一边开一边关、你以为在验收其实在测试"）。

★ 真实模式的"不接受干预"是**硬锁**（`AUV_ACCEPTANCE_LOCK=True`）：
即使 `AUV_CMD_POLICY='supervise'`，非测试模式下也会被强制成 `ignore`。
要临时解锁改 `AUV_ACCEPTANCE_LOCK = False`（不建议在比赛机上做）。

★ 但**"切模式"永远有效** —— 那是 dispatcher 层面的事，不归 `policy` 管：
上位机点回「有线遥控(ROV)」→ `kill()` → 一帧 0x09 都不再发，遥控立刻回到手里。

下水前看这两行，一眼确认没配错：

```
[AUV] 运行模式 = 调试测试模式（测试模式(仅 SIT_BOTTOM, 保持判据≤5.0s)）；上位机可随时切回 ROV 接管
[AUV] 监督指令策略 = supervise（$AUVCTL 受理）
```

## 4. `test_config.py` 配置说明（★ 改测试配置只改这个文件）

### 4.1 六个配置段

| 段 | 管什么 | 板端用 | 离线用 |
|---|---|---|---|
| `TEST_ENABLED` / `TEST_STAGES` / `TEST_HOLD_MAX_S` / `TEST_LOOP` | 测试总开关与选段 | ✅ | ✅ |
| `NOTIFY` | 状态提示回传（开关 / 级别 / 节流 / 心跳 / 去重），见 §3.4 | ✅ | — |
| `STAGE_PARAMS` | **每个功能模块自己的参数**（超时/推力/容差…） | ✅ | ✅ |
| `GLOBAL_PARAMS` | 覆盖任何 `AUV_*` 全局配置键（增益/阈值/池深/门数） | ✅ | ✅ |
| `SIM` | 仿真节奏（dt / 池深 / 深度源 / 舵机 / viskf） | — | ✅ |
| `VISION_SCRIPT` | 离线"什么时候看得见目标"的剧本 | — | ✅ |

### 4.2 选段怎么写（`TEST_STAGES`）

| 写法 | 含义 |
|---|---|
| `['SEEK_BALL_F']` | 按**阶段名**选（精确，推荐） |
| `['task:PassGateTask']` | 按**任务类**选 → 一次选中 4 个 `PASS_GATE_n` |
| `['SEEK_GATE_*']` | 通配（`*` `?`） |
| `[]` | 空 = 全部跑（但仍受 `TEST_HOLD_MAX_S` 压缩） |

★ `SURFACE` / `DONE` 在 `FORCED` 里，**任何配置都删不掉** —— 上浮不允许被关闭。
★ 选段里写了不存在的名字 → 启动打 `✗ 选段里有未知名字 [...]` 并**自动退回全流程**
（Fail-Safe：宁可跑全程，也不让你以为"这段没问题"）。

### 4.3 参数覆盖的优先级

```
STAGE_PARAMS 精确阶段名  >  task:<类名>  >  通配 '*'  >  plan.py 里的默认值
GLOBAL_PARAMS           >  auv_config.py / to32_config 里的同名键
```

⚠ 例外（**或**语义，防"一边开一边关"的错觉）：
`TEST_ENABLED` 与 `AUV_TEST_MODE` **任一个为 True 就进测试模式**。
`TEST_HOLD_MAX_S = None`（默认）= 沿用 `AUV_TEST_HOLD_MAX_S`（默认 5.0）。

### 4.4 参数写法速记（★ 最容易踩的坑）

| 写法 | 含义 | 例 |
|---|---|---|
| 大写键 + 字符串值 | 值 = **配置键名**（不含 `AUV_` 前缀）→ 运行时查 `AUV_<键名>` | `'SURGE': 'SURGE_SEEK'` → `AUV_SURGE_SEEK` |
| 大写键 + 数字值 | **字面量**（该阶段专属，不进全局配置） | `'TIMEOUT_S': 10.0` |
| 小写键 | **字面量**，`lit()` 取，绝不当配置键查 | `'cam': 'front'`、`'scan': True` |

⚠ `next` / `fail` / `cam` 必须走 `lit()`。用 `gp()` 会把 `next='SEEK_BALL_F'`
当成配置键去查 `AUV_SEEK_BALL_F`，查不到就回落默认值 → **跳转静默失效**（台架上踩过一次）。

### 4.5 常见配方（直接抄）

```python
# ① 只验"前视能不能找到球"，别干等 30s
TEST_STAGES  = ['SEEK_BALL_F']
STAGE_PARAMS = {'SEEK_BALL_F': {'TIMEOUT_S': 10.0, 'SETTLE_S': 0.5}}

# ② 一次性把 4 个穿门都调一遍（选类不选名）
TEST_STAGES  = ['task:PassGateTask']
STAGE_PARAMS = {'task:PassGateTask': {'SURGE': 0.35, 'TIMEOUT_S': 20.0}}

# ③ 现场量完池深 / 数完门，临时覆盖（试好再写回 auv_config.py）
GLOBAL_PARAMS = {'AUV_POOL_DEPTH_CM': 132.0, 'AUV_GATE_COUNT': 3}

# ④ 只想快速过一遍流程（不真下水，把预算压到 2 分钟）
GLOBAL_PARAMS = {'AUV_BUDGET_S': 120.0}

# ⑤ 离线测"找不到球 → 中止上浮"这条路径
VISION_SCRIPT['never_see'] = ['SEEK_BALL_F']
```

---

## 5. 各任务功能模块的作用

### 5.1 深度 / 上浮（`tasks/t_depth.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `DepthTask` | DIVE、RELEASE_ASCEND | 下潜 / 定深：把目标深度改到指定值，等它稳定后进下一阶段 |
| `SurfaceTask` | **SURFACE（FORCED，删不掉）** | 上浮收尾：把目标深度改到水面，到位（或超时）就结束 |

`DepthTask`
- **目标深度三种写法**（优先级从高到低）：`height_cm`（离底高度 → 按池深换算，**推荐**，与赛事规则口径一致）／`target_cm`（直接给深度 cm）／`depth_m_key`（配置键名，单位 m）。
- **完成判据**：`|当前深度 − 目标| < TOL_CM(8)` 连续保持 `HOLD_S(2)s`。
- **降级**：无深度源时不能判"到位"，走 `FALLBACK_S(12s)` 定时放行 —— 别原地死等。
- **参数**：`height_cm / target_cm / depth_m_key / SURGE / TOL_CM / HOLD_S / FALLBACK_S / TIMEOUT_S(30)`
- ⚠ 0x09 帧**没有 heave 推力槽位**，垂直运动只能改**目标深度**让固件闭环。

`SurfaceTask`
- **完成判据**：深度 ≤ `DONE_CM(15cm)` 保持 1s；`TIMEOUT_S(30s)` 超时**也收尾**
  （宁可"没浮到位就结束"，也不能永远卡在水下发 0x09）。
- 正常完成与中止路径共用它，区别只在 `ctx.abort_reason` 是否为空。

### 5.2 搜索（`tasks/t_seek.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `SeekTask` | SEEK_BALL_F、SEEK_GATE_1..4、SEEK_BALL_B | 找一个目标；找不到就超时，按 `fail` 处置 |

- **流程**：稳定期 `SETTLE_S(1.5s)`（刚下潜/刚转向水面扰动大，先别信检测）→ 边 `SURGE` 前进边找 → 命中即进下一阶段。
- **扫深**（`scan=True`）：找不到时以进入时深度为基准上下扫一段（正弦）。
  赛事只给"球距底 60cm""矮门中心距底 45cm"这类**离底高度**，池深却是现场量的、相机还可能装俯仰 —— 扫深比闷头平扫命中率高得多。
- **参数**：`cam('front'/'bottom') / label_key / aim_key / scan / SURGE / SETTLE_S(1.5) / TIMEOUT_S(30)`
- **出口**：找到 → `next`；超时 → `fail`（球 = `ABORT` 上浮止损，门 = **下一个门**，跳过别整局报废）。

### 5.3 撞球 30 分（`tasks/t_ram_ball.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `RamBallTask` | RAM_BALL | 视觉对准 + 直冲 + 四条撞击判据 |

- **伺服**：`servo_depth`（把球拉到瞄准点高度）+ `servo_yaw`（机头正对）+ `sway`（横向修正）。
- **撞击判据**（任一命中即算撞上，逐行平移自旧 mission，不许改顺序）：
  1. **IMU** 加速度超 `AUV_IMPACT_ACC`（默认关 `AUV_USE_IMU=False`，标定后才开）
  2. **尺度突增**（主判据）：`max_w > AUV_RAM_W_PX(220)` —— 贴脸就是撞上了
  3. **贴脸后消失**：`max_w > AUV_RAM_W_NEAR_PX(150)` 且目标消失 > `AUV_RAM_LOST_S(0.8)`
  4. **垂向扰动**：`dz > AUV_RAM_DZ(0.08)` 或 `vz > AUV_RAM_VZ(0.15)` ——
     ⚠ **开了垂向伺服就必须禁用**（伺服自己在改深度，分不清"撞上了"还是"自己在动"）
- **武装延迟 `ARM_S(0.8)`**：避开进入瞬间自身加速度/水花扰动，防止一进门就误判。
- **参数**：`cam / label_key / aim_key / SURGE(0.40) / ARM_S(0.8) / TIMEOUT_S(20)`

### 5.4 穿门 30 分/个（`tasks/t_pass_gate.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `PassGateTask` | PASS_GATE_1..4（高矮门通用） | 横向对中 + 机头转正 + 直冲穿门 |

- **伺服双路**：`viskf.trust` 为真 → 用滤波量 `e_x/de_x/s_n` 伺服（噪声小、带速度、与距离无关）；
  否则退回原始像素伺服。两条路**共用同一套"已穿过"判定**，关掉 viskf 行为完全不变。
- **"已穿过"判定必须放在"目标消失"分支里**：门贴近到出画才会消失，写在"看得见"分支里永远等不到。
  判据：`max_w > AUV_GATE_PASS_W_RATIO(0.80)×AUV_IMG_W(640)` 或 `s_n ≥ AUV_GATE_S_STAR(0.85)×(1−AUV_GATE_CROSS_TOL)`，
  且消失 > `AUV_GATE_LOST_S(0.5)`。
- **高矮门**：进入时按 `height_cm` 先把深度对到门中心高度（`GATE_LOW_HEIGHT_CM=45` / `GATE_HIGH_HEIGHT_CM=65`），再由视觉微调。
- **参数**：`cam / label_key / aim_key / height_cm / SURGE(0.30) / TIMEOUT_S(30)`
- ⚠ 一次性状态 `max_w / last_seen` **必须在 `enter()` 里复位** —— 4 个门共用本类，不复位会直接误判"已穿过"。

### 5.5 捡球 100 分（`tasks/t_center_ball.py` + `tasks/t_sit_bottom.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `CenterBallTask` | CENTER_BALL | 下视把球对到收集框正上方 |
| `SitBottomTask` | SIT_BOTTOM | 下沉积底，用收集框把球兜进去 |

`CenterBallTask`
- 无机械手，靠**收集框坐底兜球**（30×40cm），球必须在框正上方 —— 框只有这么大，对不准 = 白坐一次。
- **伺服**：`sway` 修横向（`ex`）、`surge` 修纵向（`ey`，注意是这个任务唯一用 surge 修纵向的地方）。
- **判据**：`|dx| < TOL_PX(25)` 且 `|dy| < TOL_PX` 保持 `HOLD_S(1.5)`。
- **超时/目标丢失都不中止**，直接进坐底：捡球是"尽力而为"，对不准也比原地耗到整局失败强。
- **参数**：`cam / label_key / aim_key / TOL_PX / HOLD_S / KP_SURGE(0.40) / SURGE_MAX(0.30) / TIMEOUT_S(15)`

`SitBottomTask`
- 每拍把目标深度 `+RATE_CMS(15)×dt`，上限 **直接取 `AUV_POOL_DEPTH_CM`（不走 `clamp_depth`）**。
  ⚠ 走 `clamp_depth` 会被扣掉 `AUV_DEPTH_BOTTOM_MARGIN_CM(10)` → 净空永远到不了 6cm → 只能靠 25s 超时兜底（干跑实测踩出来的）。
- **判据**：① 离底净空 `clearance×100 < CLEAR_CM(6)` 保持 `CLEAR_HOLD_S(1.0)`（主，绝对量最可靠）；
  ② 深度停滞 + `|v_z| < VZ(0.02)` 保持 `STALL_S(2.0)`（没有 clearance 时的退路）；③ 超时按到底处理。
- **参数**：`RATE_CMS / CLEAR_CM / CLEAR_HOLD_S / STALL_CM / STALL_S / VZ / TIMEOUT_S(25)`

### 5.6 保持 / 定时（`tasks/t_hold.py`、`tasks/t_timed.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `HoldTask` | BOTTOM_HOLD、RELEASE_HOVER | 原地悬停 N 秒，**零推力** |
| `TimedTask` | （备用） | 以固定推力开环推进 N 秒 |

- `HoldTask` 只一个参数 `DUR_S`（→ `AUV_BOTTOM_HOLD_S=3` / `AUV_RELEASE_HOVER_S=3`）。
  BOTTOM_HOLD 是给球滚进框的时间；RELEASE_HOVER 是吃掉落点前的余速。
- 零推力悬停是**最安全的默认动作**（急停 / 暂停 / 降级都往它落）。
- `TimedTask` 是**开环**的：光流已停用，没有速度反馈 —— 🔴 `AUV_SPEED_MPS` 未标定前只能验结构不能验距离。
  与 `HoldTask` 差别 = 有推力；与 `DeadReckonTask` 差别 = 只认时间不认距离。

### 5.7 转向（`tasks/t_turn.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `TurnTask` | TURN_120、RELEASE_TURN | 原地转向 N 度 |

- **目标航向 = 进入时的航向估计 + `DEG × AUV_TURN_RIGHT_SIGN`**，在 `enter()` 里算**一次**
  （每拍重算会把"正在转过去的量"冲掉，导致永远转不到位）。
- **判到位**：开环需时 `|DEG| / AUV_YAW_RATE_DPS + SETTLE_S(1.0)`；有遥测 yaw 真值时自动升级为真闭环（角差 < `TOL_DEG(3)`）。
- **参数**：`DEG / SETTLE_S / TOL_DEG / TIMEOUT_S(10)`
- 🔴 `AUV_YAW_RATE_DPS(30)` 是占位值，必须水池标定（发满舵 → 计时 → 量角度）；未标定时"转 120°"实际可能是 100~140°。

### 5.8 航位推算 / 回出发区（`tasks/t_dead_reckon.py`、`tasks/t_go_home.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `DeadReckonTask` | RELEASE_MOVE | 沿当前航向推进到**推算**的目标距离 |
| `GoHomeTask` | GO_HOME | 回出发区并顶到池壁（30 分） |

`DeadReckonTask`
- **主估 = 标称速度 × 时间**：`AUV_SPEED_MPS(0.25) × AUV_SPEED_K_SURGE(1.0) × (surge/0.3)` 累加。
- **IMU（STM32 遥测应答，g=9.81）当前只做"是不是还在动"的健康检查** ——
  加速度二次积分在无姿态补偿下漂移极快（几十秒漂出几米），
  🔴 IMU 标定完成前**不要**拿它当位移主源；标定后把 `AUV_DR_USE_IMU` 打开即切换主源，本类不用改。
- 推力给着但加速度长期 ≈ 0 → 打 WARN "可能被顶住"（`AUV_DR_STILL_G` / `AUV_DR_STUCK_S`）。
- **参数**：`DIST_M / SURGE / yaw_abs / height_cm / TIMEOUT_S(30)`

`GoHomeTask`
- 评分口径是"**触碰出发区任意一侧池壁**"即得分，不要求停准 → 策略是"顶到墙"而不是"走到坐标点"，
  即便航位推算不准也能拿分。
- **判据**：① IMU `AUV_WALL_IMPACT_ACC`（未标定 = 0 → 不启用）；② **超时**（⚠ 无 IMU 时它就是实际判据）。
- **参数**：`SURGE(0.35) / yaw_abs / turn_deg / height_cm / TIMEOUT_S(20)`

### 5.9 投放 100 分 / 结束（`tasks/t_servo_drop.py`、`tasks/t_done.py`）

| 类 | 用在 | 作用 |
|---|---|---|
| `ServoDropTask` | RELEASE_DROP | 悬停 → `ARM_S` 后触发舵机投放 → 保持到 `TOTAL_S` |
| `DoneTask` | DONE | 首拍发 `stop=True` 停推，之后恒 `None` |

`ServoDropTask`
- 舵机**还没实装**：`ctx.servo` 现在是 `ServoStub`，`ready()` 恒 False、`drop()` 只打一条 WARN。
  → **不中止**任务，照跑（投放 100 分重要，但"没舵机就整局报废"更糟）；实装后只换 `make_servo()` 的实现，本任务一行不用改。
- **幂等**：`drop()` 会被 20Hz 反复调用，真正触发只允许一次（接口内部保证 + `fired` 双保险）；
  ⚠ `fired` 必须在 `enter()` 里复位，放 `__init__` 的话第二次跑就不会触发了。
- **参数**：`ARM_S(1.0) / TOTAL_S(3.0)`；全程零推力悬停（任何平移都会让球落偏）。

`DoneTask`
- 返回 `None` 是"让 mode_auv 停止下发 0x09"的信号 —— 结束后**不能再持续发同一帧**
  （会顶掉上位机手动控制，也会让下位机一直保持推力）。

---

## 6. 测试手册：可以测哪些阶段，每个阶段到底测什么

> ★ 一句话用法：`test_config.py` 里 `TEST_STAGES = ['XXX']` 选中这一段 → 重启中位机 →
> 执行器**直接从这一段开始跑**，跑完强制进 `SURFACE` → `DONE` → 请求切回有线 ROV。
> 每段结束都会上浮，**不需要你手动救**。
>
> 分级标注：**〔离线〕** = 本机 `run_task.py` 就能验（不用下水） ·
> **〔下水〕** = 必须真机水池验 · **〔待标定〕** = 依赖还没标定的量，先验标定值再谈效果。

### 6.0 一段的标准测试动作（每次下水照抄这 5 步）

| # | 动作 | 说明 |
|---|---|---|
| 1 | 离线先跑 `python run_task.py --stage X -v` | 确认这段的**入口/出口/超时/参数覆盖**是你想要的 |
| 2 | `test_config.py` 设 `TEST_STAGES = ['X']` + `STAGE_PARAMS` | ★ 超时一律先压小（10~15s），别在水里干等 30s |
| 3 | 上传 → 重启中位机 → 上位机切 AUV | **盯启动四行日志**：模式 / 保留段 / 阶段参数覆盖 / 全局覆盖 |
| 4 | 记录 | 用时、日志关键量、`$TEL` 曲线、水面视频 |
| 5 | 收敛 | 好使 → 参数**写回 `auv_config.py`** 并清空 `GLOBAL_PARAMS`；不好使 → 改了再来 |

⚠ 每次下水**只测一段**（最多 3 段连测）。一次测太多，出问题分不清是哪段的锅。
⚠ `TEST_ENABLED` 现在是 **True**（测试模式）。验收/上场前必须改回 `False` 并清空 `GLOBAL_PARAMS`。

### 6.1 快速对照：哪些能在桌上验，哪些必须下水

| 类别 | 验什么 | 在哪验 |
|---|---|---|
| 结构类 | 跳转顺序、超时、选段过滤、参数覆盖优先级、中止路径、DONE 停推 | 〔离线〕`run_task.py` |
| 判据类 | 各段的"到位/命中/穿过"条件写得对不对、会不会卡死 | 〔离线〕改 `VISION_SCRIPT` / `SIM` |
| 标定类 | 推力↔速度、转向速率与方向、池深、撞击阈值、坐底净空 | 〔下水〕 |
| 感知类 | YOLO 到底有没有框、框准不准、前视/下视有没有串台 | 〔下水〕（看 `$TEL` 的 det） |

---

### 6.2 下潜与上浮（DIVE / SURFACE）

**① `DIVE` · `DepthTask` · 10 分 · 〔下水〕**

- **测什么**：深度源通不通；定深闭环灵不灵；能不能稳定停在"离底 60cm"（= 深度 `池深 − 60`）。
- **判成功**：`$TEL` 深度稳定在目标 ±`TOL_CM(8)` 且连续保持 `HOLD_S(2.0)s` → 日志出现「定深到位 err=…cm」→ 进 `SEEK_BALL_F`。
- **常用参数**：`height_cm`(→`AUV_BALL_HEIGHT_CM`=60) / `SURGE`(→`AUV_SURGE_SEEK`=0.25) / `TOL_CM`(8) / `HOLD_S`(2.0) / `FALLBACK_S`(12) / `TIMEOUT_S`(30)
- **全局**：`AUV_POOL_DEPTH_CM`(130 〔待标定〕**到场第一件事就是量它**) / `AUV_DIVE_DEPTH_M`(1.0，没配离底高度时的退路)
- **失败表现**：30s 超时**也放行**去 `SEEK_BALL_F`（fail 就是它）；深度源掉线 → 12s 定时放行（不会原地死等）。
- **先查这个**：板端曾实测深度恒 0.0cm（高度计五路 no-reply）。**深度源没修好前这段验不了真判据**，只会走 12s 定时放行。
- **选段**：`TEST_STAGES = ['DIVE']`

**② `SURFACE` · `SurfaceTask` · FORCED 删不掉 · 〔下水〕**

- **测什么**：上浮能不能真的浮出水面；超时兜底会不会生效（你要靠它"浮起来看问题"）。
- **判成功**：深度 ≤ `DONE_CM(15cm)` 保持 1s；`TIMEOUT_S(30s)` 超时**也收尾**（宁可没浮到位就结束，也不能永远卡在水下发 0x09）。
- **正常完成与中止路径共用它**，区别只在 `ctx.abort_reason` 是否为空 —— 所以**任何选段最后都会走到这里**，随便测。
- **不用选**：它在 `FORCED` 里，写不写都在。

---

### 6.3 搜索（SEEK_BALL_F / SEEK_GATE_n / SEEK_BALL_B）

**③ `SEEK_BALL_F` · `SeekTask` · 前视找球 · 〔下水〕+〔离线〕**

- **测什么**：前视 YOLO 能不能看见球；扫深（`scan=True`）有没有用；找不到时的**中止上浮**路径。
- **判成功**：稳定期 `SETTLE_S(1.5s)` 后检测到 `AUV_LABEL_BALL`（现模型里叫 `red-ball`）→ 进 `RAM_BALL`；`TIMEOUT_S(30)` → `ABORT` 上浮止损。
- **常用参数**：`cam='front'`(小写字面量) / `label_key`(→`AUV_LABEL_BALL`) / `aim_key`(→`AUV_AIM_RAM`=(320,240)) / `scan`(True) / `SURGE`(0.25) / `SETTLE_S`(1.5) / `TIMEOUT_S`(30)
- **下水配方**：`STAGE_PARAMS = {'SEEK_BALL_F': {'TIMEOUT_S': 10.0, 'SETTLE_S': 0.5}}`（别干等 30s）
- **离线验中止**：`VISION_SCRIPT['never_see'] = ['SEEK_BALL_F']` → 走超时 → ABORT → SURFACE
- **〔待标定〕**：撞球 YOLO **还没训练**，模型输出的类别名必须和 `AUV_LABEL_BALL` 对得上，否则永远"看不见"。

**④ `SEEK_GATE_1..4` · `SeekTask` · 前视找门 · 〔下水〕**

- **测什么**：前视能不能看见门（`AUV_LABEL_GATE`，模型里叫 `door`）；找不到时是否**跳到下一个门**而不是整局报废。
- **判成功**：同 ③；`fail = SEEK_GATE_{n+1}`（最后一个门 fail = `SEEK_BALL_B`）。
- **参数**：同 ③，`label_key='LABEL_GATE'`、`aim_key='AIM_GATE'`、`scan=True`。
- **结构**：门数 `AUV_GATE_COUNT`（3 或 4，现场数）→ 段数随之变（4 门 = 24 段 / 3 门 = 22 段）；高矮顺序 `AUV_GATE_SEQ='low,high,low,high'`。
- **选段**：`['SEEK_GATE_1']` 单门调试 / `['SEEK_GATE_*']` 四个门全测 /
  ⚠ `['task:SeekTask']` 会把 `SEEK_BALL_F` / `SEEK_BALL_B` 也一起选中（同一个类）。

**⑤ `SEEK_BALL_B` · `SeekTask` · 下视找待捡球 · 〔下水〕**

- **测什么**：下视相机能不能看见球（`AUV_LABEL_PICK`）；下视视野够不够。
- **判成功**：找到 → `CENTER_BALL`；`TIMEOUT_S(30)` → **ABORT**（捡球 100 分是核心，找不到就上浮止损）。
- **参数**：`cam='bottom'` / `scan=False`（坐底阶段不扫深）/ `TIMEOUT_S(30 → 先压 10)`。
- **⚠ 相机物理口**：cam1 前视 = USB3-2、cam2 下视 = USB1-2。**别用 `/dev/camN` 软链**（板端 udev 规则是反的，会前视/下视串台）。

---

### 6.4 撞球 30 分（`RAM_BALL`）

**⑥ `RAM_BALL` · `RamBallTask` · 30 分 · 〔下水〕**

- **测什么**：视觉伺服对不对得准；四条撞击判据哪条先触发；到底撞上没有。
- **判据顺序**（逐行平移自旧 mission，**不许改顺序**）：
  1. IMU 加速度超 `AUV_IMPACT_ACC`（默认 `AUV_USE_IMPACT_ACC=False`，标定后才开）
  2. **尺度突增（主判据）**：`max_w > AUV_RAM_W_PX(220)` —— 贴脸就是撞上了
  3. 贴脸后消失：`max_w > AUV_RAM_W_NEAR_PX(150)` 且目标消失 > `AUV_RAM_LOST_S(0.8)`
  4. 垂向扰动：`dz > AUV_RAM_DZ(0.08)` 或 `vz > AUV_RAM_VZ(0.15)` ⚠ **开了垂向伺服就必须禁用**（分不清"撞上了"还是"自己在动"）
- **参数**：`SURGE`(0.40 → `AUV_SURGE_RAM`) / `ARM_S`(0.8，武装延迟，避开进入瞬间扰动) / `TIMEOUT_S`(20)
- **〔待标定〕`AUV_RAM_W_PX`**：拍下贴脸那一刻的球宽（px），填进去。
- **失败表现**：`fail = TURN_120` —— 没撞上也继续（保住过门的 30×N 分）。
- **选段**：`TEST_STAGES = ['RAM_BALL']`，`STAGE_PARAMS = {'RAM_BALL': {'SURGE': 0.35, 'TIMEOUT_S': 12.0}}`

---

### 6.5 转向（`TURN_120` / `RELEASE_TURN`）—— ★ 优先标定项

**⑦ `TURN_120` · `TurnTask` · 〔下水〕+〔待标定〕**

- **测什么**：转向**方向**和**角度**准不准 —— 后面所有"转向找门/转向投放"都靠它。
- **判成功**：开环需时 `|DEG| / AUV_YAW_RATE_DPS + SETTLE_S(1.0)` 后放行；`$TEL` 有 yaw 真值时自动升级为**真闭环**（角差 < `TOL_DEG(3)`）。
- **参数**：`DEG`(→`AUV_TURN1_DEG`=120) / `SETTLE_S`(1.0) / `TOL_DEG`(3.0) / `TIMEOUT_S`(10)
- **标定手法**（水面附近最安全，建议第一个测）：
  1. `TEST_STAGES = ['TURN_120']`，先设 `DEG: 90.0`；
  2. 发满舵 → 计时 → 量实际转过多少度 → 反算 `AUV_YAW_RATE_DPS`（度/秒）；
  3. 看 yaw 是增还是减 → 定 `AUV_TURN_RIGHT_SIGN`（+1 或 −1）。
- **未标定时**：说"转 120°"实际可能 100~140°，别信。

**⑧ `RELEASE_TURN` · `TurnTask` · 投放前转向 · 〔待标定〕**

- **测什么**：转向投放方向（`AUV_RELEASE_TURN_DEG`=90 是占位值，方向待现场定）。
- **前提**：先做完 ⑦ 的标定，否则这段测了也没意义。

---

### 6.6 穿门 30 分/个（`PASS_GATE_1..4`）

**⑨ `PASS_GATE_n` · `PassGateTask` · 30×N 分 · 〔下水〕+〔离线〕**

- **测什么**：横向对中 + 机头转正 + 直冲；**"已穿过"判定**灵不灵（最容易卡死的地方）。
- **判成功**：门贴近到**出画消失**，且（`max_w > AUV_GATE_PASS_W_RATIO(0.80)×AUV_IMG_W(640)` 或
  `s_n ≥ AUV_GATE_S_STAR(0.85)×(1−AUV_GATE_CROSS_TOL)`），消失持续 > `AUV_GATE_LOST_S(0.5)` → 进下一个门。
- **参数**：`height_cm`(矮门→`AUV_GATE_LOW_HEIGHT_CM`=45 / 高门→`AUV_GATE_HIGH_HEIGHT_CM`=65) / `SURGE`(0.30) / `TIMEOUT_S`(30)
- **关键**：`AUV_GATE_CROSS_TOL` **必须 > 0** —— int8 量化会把小推力归零，机器人卡死到超时。
- **viskf 双路**：`viskf.trust` 为真 → 用滤波量 `e_x/de_x/s_n` 伺服（噪声小、与距离无关）；否则退回像素伺服。
  离线把 `SIM['viskf_ok'] = True` 就能走滤波路径验一遍。
- **⚠ 一次性状态**：`max_w / last_seen` 必须在 `enter()` 里复位（4 个门共用本类，不复位会直接误判"已穿过"）。
- **选段（推荐）**：`TEST_STAGES = ['task:PassGateTask']` —— 一次选中 4 个门，参数一次调完：
  `STAGE_PARAMS = {'task:PassGateTask': {'SURGE': 0.35, 'TIMEOUT_S': 20.0}}`

---

### 6.7 捡球 100 分（`CENTER_BALL` / `SIT_BOTTOM` / `BOTTOM_HOLD`）

**⑩ `CENTER_BALL` · `CenterBallTask` · 100 分 · 〔下水〕**

- **测什么**：下视能否把球对到**收集框正上方**（框只有 30×40cm，对不准 = 白坐一次）。
- **判成功**：`|dx| < TOL_PX(25)` 且 `|dy| < TOL_PX` 保持 `HOLD_S(1.5)`。
- **参数**：`TOL_PX`(25) / `HOLD_S`(1.5) / `KP_SURGE`(0.40) / `SURGE_MAX`(0.30) / `TIMEOUT_S`(15)
- **超时/目标丢失都不中止**，直接进坐底 —— 捡球是"尽力而为"，对不准也比原地耗到整局失败强。
- **手法**：先悬在半空用下视看球，盯日志里 `ex/ey` 有没有收敛到 0。

**⑪ `SIT_BOTTOM` · `SitBottomTask` · 〔下水〕+〔待标定〕**

- **测什么**：能不能真坐到底（净空判据）；下沉速率合不合适（太猛会砸、太慢会超时）。
- **判成功**（三取一）：
  ① 净空 `clearance×100 < CLEAR_CM(6)` 保持 `CLEAR_HOLD_S(1.0)`（主判据，绝对量最可靠）
  ② 深度停滞 + `|v_z| < VZ(0.02)` 保持 `STALL_S(2.0)`（没有 clearance 净空量时的退路）
  ③ `TIMEOUT_S(25)` 超时按到底处理
- **参数**：`RATE_CMS`(15，下沉速率 cm/s) / `CLEAR_CM`(6) / `CLEAR_HOLD_S`(1.0) / `STALL_CM` / `STALL_S`(2.0) / `VZ`(0.02) / `TIMEOUT_S`(25)
- **全局**：`AUV_POOL_DEPTH_CM`(130 〔待标定〕) —— 目标深度**直接取池深**，不走 `clamp_depth`
  （⚠ 走 clamp 会被扣掉池底余量 → 净空永远到不了 → 只能靠 25s 超时兜底，干跑实测踩出来的）。
- **观测**：日志里的 `cl`（净空）有没有持续变小。一直不变小 = 池深填错或深度源没数据。

**⑫ `BOTTOM_HOLD` · `HoldTask` · 〔下水〕**

- **测什么**：坐底后**零推力**保持 `AUV_BOTTOM_HOLD_S`(3s)，给球滚进框的时间。
- **判成功**：纯计时；真正的验收是上浮后看框里**有没有球**。
- **组合测（推荐）**：`TEST_STAGES = ['CENTER_BALL', 'SIT_BOTTOM', 'BOTTOM_HOLD']` 一次验完整个捡球动作。

---

### 6.8 投放 100 分（`RELEASE_ASCEND` / `_MOVE` / `_HOVER` / `_DROP`）

**⑬ `RELEASE_ASCEND` · `DepthTask` · 〔下水〕**
- **测什么**：带着球从池底升到投放高度（离底 110cm ≈ 水深 20cm）；配平变化会不会让姿态跑掉。
- **参数**：`height_cm`(→`AUV_RELEASE_HEIGHT_CM`=110) / `TIMEOUT_S`(30)

**⑭ `RELEASE_MOVE` · `DeadReckonTask` · 〔待标定〕**
- **测什么**：航位推算推进 `AUV_RELEASE_DIST_M`(2.0m)。★ 未标定前**只能验结构不能验距离**。
- **主估**：`AUV_SPEED_MPS`(0.25) × `AUV_SPEED_K_SURGE`(1.0) × (surge/0.3) 累加。
- **参数**：`DIST_M`(→2.0) / `SURGE`(0.30) / `TIMEOUT_S`(30)
- **IMU 现状**：STM32 遥测应答（g=9.81）当前**只做"是不是还在动"的健康检查**，不当位移主源
  （加速度二次积分无姿态补偿，几十秒漂几米）；标定后把 `AUV_DR_USE_IMU` 打开即切主源，本类不用改。
- **健康告警**：推力给着但加速度长期 ≈ 0 → 打 WARN「可能被顶住」。
- **标定手法**：选这段跑固定时长 → 水面量实际距离 → 反算 `AUV_SPEED_MPS`。

**⑮ `RELEASE_HOVER` · `HoldTask` · 〔下水〕**
- **测什么**：悬停吃掉余速 —— 投放准不准全看它。`DUR_S`(→`AUV_RELEASE_HOVER_S`=3)。

**⑯ `RELEASE_DROP` · `ServoDropTask` · 100 分 · 〔离线〕（舵机未实装）**
- **测什么**：投放时序（进入 → `ARM_S(1.0)` → 触发舵机 → 保持到 `TOTAL_S(3.0)`）；未实装时是否**照跑不中止**。
- **现状**：`ctx.servo` 是 `ServoStub`，`ready()` 恒 False、`drop()` 只打一条 WARN → **不中止任务**
  （投放 100 分没了，也不能因为没舵机整局报废）。
- **幂等**：`drop()` 会被 20Hz 反复调用，真正触发只允许一次；⚠ `fired` 在 `enter()` 复位。
- **实装后**：只换 `make_servo()` 的实现，**本任务一行不用改**。

---

### 6.9 回出发区 30 分（`GO_HOME`）

**⑰ `GO_HOME` · `GoHomeTask` · 30 分 · 〔下水〕**

- **测什么**：能不能顶到出发区池壁。评分口径是"**触碰出发区任意一侧池壁**"即得分，不要求停准
  → 策略就是"顶到墙"而不是"走到坐标点"，即便航位推算不准也能拿分。
- **判成功**：① IMU `AUV_WALL_IMPACT_ACC`（未标定 = 0 → 不启用）；② **超时**（⚠ 无 IMU 时它就是实际判据）。
- **参数**：`SURGE`(0.35) / `turn_deg`(→`AUV_HOME_TURN_DEG`=90 〔待标定〕) / `TIMEOUT_S`(20)
- **标定手法**：先在水面手操顶墙，记下要多久 → 反推超时与航向。

---

### 6.10 结束（`DONE`）

**⑱ `DONE` · `DoneTask` · 〔离线〕+〔下水〕**

- **测什么**：结束后是否**停推**（返回 `None` = 让 mode_auv 停止下发 0x09）→ 上位机能手动接管；
  上浮后是否请求切回有线 ROV。
- **⚠ 硬规则**：上浮后切回 ROV 必须用 **`cfg.MODE_ROV`(=0)**。写成 `link_stm32.MODE_ROV`(0x03)
  → 0x04 一帧都发不出去，机器人留在 AUV，只在 stderr 多一行。

---

### 6.11 推荐下水顺序（先标定、后功能；先安全、后冒险）

| 轮 | `TEST_STAGES` | 目的 | 要记下的数 |
|---|---|---|---|
| 1 | `['DIVE']` | 确认深度源真的有数据（否则后面全是降级路径） | 深度是否非 0、到位耗时 |
| 2 | `['TURN_120']` | 标定转向 | `AUV_YAW_RATE_DPS`、`AUV_TURN_RIGHT_SIGN` |
| 3 | `['RELEASE_MOVE']` | 标定巡航速度（水面最好量） | `AUV_SPEED_MPS` |
| 4 | `['SEEK_BALL_F']` | 前视 YOLO 有没有框 | 延迟、框位置、类别名 |
| 5 | `['RAM_BALL']` | 撞击阈值 | 贴脸时球宽 → `AUV_RAM_W_PX` |
| 6 | `['SIT_BOTTOM']` | 池深 + 坐底净空 | `AUV_POOL_DEPTH_CM`、`RATE_CMS` |
| 7 | `['CENTER_BALL','SIT_BOTTOM','BOTTOM_HOLD']` | 完整捡球动作 | 框里有没有球 |
| 8 | `['task:PassGateTask']` | 四个门一次调完 | 穿过判据是否触发 |
| 9 | `['RELEASE_ASCEND','RELEASE_HOVER','RELEASE_DROP']` | 投放链路（舵机未实装只验时序） | WARN 有没有打出来 |
| 10 | `['GO_HOME']` | 顶墙 | 超时该设多少 |
| 11 | `[]`（全跑） | 全流程联调 | 总耗时 vs `AUV_BUDGET_S`(900s) |

### 6.12 每次下水的最小记录表（抄到纸上手填最快）

| 段 | 耗时(s) | 日志关键量 | 实际现象 | 这次改了什么 | 结论 |
|---|---|---|---|---|---|
| DIVE | | err= / D= | | | |
| SIT_BOTTOM | | cl= / 超时? | | | |
| RAM_BALL | | max_w 峰值 | | | |

---

## 7. 阶段表一览（24 段）

| # | 阶段 | 任务类 | 分值 | 说明 |
|---|---|---|---|---|
| 0 | DIVE | DepthTask | 10 | 下潜到撞球高度（离底 60cm） |
| 1 | SEEK_BALL_F | SeekTask | | 前视找球（扫深），超时 → **ABORT** |
| 2 | RAM_BALL | RamBallTask | 30 | 对准直冲，四条撞击判据 |
| 3 | TURN_120 | TurnTask | | 转向找门（`AUV_TURN1_DEG`） |
| 4-11 | SEEK_GATE_n / PASS_GATE_n | SeekTask / PassGateTask | 30×4 | 4 个门，高矮交替；找不到 → 跳下一个门 |
| 12 | SEEK_BALL_B | SeekTask | | 下视找待捡球，超时 → **ABORT** |
| 13 | CENTER_BALL | CenterBallTask | 100 | 把球对到收集框正上方（对不准也坐底） |
| 14 | SIT_BOTTOM | SitBottomTask | | 坐底，框兜球 |
| 15 | BOTTOM_HOLD | HoldTask | | 坐底保持，等球滚进框 |
| 16 | RELEASE_ASCEND | DepthTask | | 上升到投放高度（离底 110cm） |
| 17 | RELEASE_TURN | TurnTask | | 转向投放方向 |
| 18 | RELEASE_MOVE | DeadReckonTask | | 航位推算推进 `AUV_RELEASE_DIST_M` |
| 19 | RELEASE_HOVER | HoldTask | | 悬停吃掉余速 |
| 20 | RELEASE_DROP | ServoDropTask | 100 | 舵机投放（未实装则跳过，不中止） |
| 21 | GO_HOME | GoHomeTask | 30 | 顶到出发区池壁 |
| 22 | **SURFACE** | SurfaceTask | | **上浮（FORCED，删不掉）** |
| 23 | DONE | DoneTask | | 停推结束 |

门数可配：`AUV_GATE_COUNT`（4 门 = 24 段，3 门 = 22 段）；高矮顺序可配：`AUV_GATE_SEQ='low,high,low,high'`。

---

## 8. 怎么加一个阶段

1. 在 `tasks/` 下新建 `t_xxx.py`，继承 `Task`，实现 `tick()`（一次性初始化放 `enter()`）；
2. 在 `tasks/__init__.py` 导出；
3. 在 `plan.py` 的 `build_plan()` 里加一行 `_sp('名字', XxxTask, ...)`；
4. 跑一遍 `tests/test_auv_task.py`。

**执行器和别的代码一行都不用改**；`STAGE_PARAMS` 覆盖由 `_sp()` 自动套用，新增阶段也不用自己管。

---

## 9. 监督通道 `$AUVCTL`（水池保命）

AUV 模式**不执行**上位机杆位（`on_cmd` 只记日志），但流程要能被遥控：

| 指令 | 作用 |
|---|---|
| `HOLD` | 暂停：保持当前深度/航向，零推力悬停 |
| `RESUME` | 恢复 |
| `SKIP` | 跳过当前阶段 |
| `GOTO <阶段名>` | 强制跳段（测试模式下只允许跳到已启用的段） |
| `ABORT` | 中止 → 强制上浮 |
| `KILL` | ★ 立即停手（**不上浮**，一帧 0x09 都不再发）—— 水池里要"马上不动"用这个 |
| `RESET` | 从头重跑（同时解除 KILL 的终止态） |

`AUV_CMD_POLICY`：`ignore`（验收/比赛，全部拒绝）/ `supervise`（默认）/ `passthrough`。

⚠ `AUV_LOCK_MODE` 必须保持 **`False`**：为 True 时会忽略 `$CMD` 的 mode 字段，
上位机就切不回 ROV 了，与 §3.2「上位机接管即终止」直接冲突。

---

## 10. 硬规则（改代码前必读）

| # | 规则 | 后果 |
|---|---|---|
| 1 | **上浮不允许被配置关闭**：`SURFACE`/`DONE` 在 `TestCfg.FORCED` | 任何选段都会走到上浮 |
| 2 | 舵机未实装时**不中止**任务，只打 WARN | 投放 100 分没了也不报废整局 |
| 3 | 全局预算 `AUV_BUDGET_S=900s` 到点强制上浮 | 赛事 15 分钟硬上限 |
| 4 | `AUV_GATE_CROSS_TOL` 必须 > 0 | int8 量化会把小推力归零 → 卡死到超时 |
| 5 | viskf 判可用看 `trust`，**不看** `gate_visible` | 看错字段会全程走降级路径 |
| 6 | `e_x` 归一化恒用 `(cx−x0−dx0)/w_bbox` | 裁剪时换分母会让伺服发散 |
| 7 | 每个任务的一次性状态**必须在 `enter()` 里复位** | 4 个门共用 `PassGateTask`，`max_w` 不复位会直接误判穿过 |
| 8 | 坐底目标**不能**走 `clamp_depth` | 扣掉池底余量后净空判据永远达不到，只能超时兜底 |
| 9 | ★★ 上浮后切回 ROV 用 **cfg.MODE_ROV(=0)** | 写成 `link_stm32.MODE_ROV(0x03)` → 0x04 一帧都发不出去，机器人留在 AUV |
| 9b | ★★ **真实作业模式**强制 `policy=ignore`（除切模式外拒绝一切） | 验收时被误发干预指令打断整局 |
| 9c | ★★ **"切模式"永远有效**，不受 policy 约束 | 水池里出事必须能一秒接管 |
| 10 | ★★ AUV 只能由上位机 `$CMD` 切入（`AUV_REQUIRE_PC_CMD=True`） | 上电/记忆/`--mode` 一律回落 IDLE；门控失效时也**不许**自己跑 |
| 11 | `AUV_LOCK_MODE` 必须 False | 为 True 时上位机切不回 ROV，接管不生效 |
| 12 | 回传（`$TEL`/`$AUV`/`$MSG`）一律"失败即放弃"，绝不重试到卡住 | 影响主功能 = 直接废掉一局；无缆时零帧是正常现象 |
| 13 | `$MSG` 帧字段数恒为 6（含帧头） | 解析按 6 取不到：去帧头+帧尾后是 5 段 |

---

## 11. 排障速查

| 现象 | 先看 | 改哪个 |
|---|---|---|
| **上电后 AUV 不自己跑** | 启动日志"[MODE] 启动门控" | ✅ 预期行为：用上位机点「自主航行(AUV)」才会开始 |
| 上位机点了 AUV 没反应 | 门控摘要行 + `[MODE] 待命态收到 $CMD.mode=1` | `$CMD` 是否带 mode 字段（≥7 段）；`AUV_REQUIRE_PC_CMD` |
| **切回 ROV 后机器人还在自己动** | 日志"任务执行已终止" | `mode_auv.on_exit` 补丁 A5 是否打上；`AUV_STOP_FRAME_ON_EXIT` |
| 上位机终端没有 AUV 提示 | 板端 `[AUV-MSG] 状态提示回传已启动` | `AUV_MSG_ENABLED` / IP 是否学到（无缆时本来就没有）；端口 8085 |
| `$MSG` 收到但解析不出 | 原始帧文本 | 字段按 5 段取（去掉 `$MSG,` 与 `#`） |
| 一进 AUV 就"回水面 + 转向" | 首帧锚定日志 | 深度源是否 ok；`AUV_SURFACE_DEPTH_M` |
| 搜索永远找不到 | `$TEL` 里 det 有没有框 | `SETTLE_S` / `scan` / 相机俯仰 / `AUV_LABEL_*` |
| 撞球判不上 | 日志里 `max_w` 峰值 | `AUV_RAM_W_PX`（拍下贴脸时的球宽取阈值） |
| 穿门卡住不动 | `s_n` / `w_max` | `AUV_GATE_CROSS_TOL` 是否为 0；`AUV_GATE_S_STAR` |
| 坐底要等 25s 才过 | 净空 `cl` 有没有变小 | `AUV_POOL_DEPTH_CM` 是否量对；`RATE_CMS` |
| 转向不到位 | 遥测 yaw 有没有真值 | `AUV_YAW_RATE_DPS`（标定）/ `AUV_TURN_RIGHT_SIGN` |
| 上浮后没切回 ROV | 日志"请求切回有线 ROV" | 只能是 `cfg.MODE_ROV`；别写成 0x03 |
| 选段没生效 | 启动四行日志 | 名字写错会 Fail-Safe 退回全流程 |

---

## 12. 与旧代码的对接

`TaskRunner` 与旧 `Mission` **对外契约完全一致**，`mode_auv.py` 只改 2 处：

```python
from auv_task import TaskRunner, apply_yaw_mirror            # 原 from mission import ...
self.mission = TaskRunner(C, vision=..., depth=..., viskf=..., log=self.log)
```

`step(now, dt, tel)` 返回的 dict 字段集不变：`depth / yaw / surge / sway / stop / stage / note`。
`AuvReport.make_snapshot` 需要的属性（`t0 / stage / seq / last_dep / last_obs / last_obs_cam /
last_obs_ts / last_vk / abort_reason`）在 runner 上全部代理好了 → **`auv_report.py` 一行都不用改**。

板端接入用根目录的 `apply_auv_task_patch.py`（自动备份 + 逐项报告 + 幂等），
并记得给 `run.sh` 的 PYTHONPATH 补第五段 `$SRC_DIR/to32/move_test`。

### 12.1 补丁规则清单（2026-10-05 版）

`mode_auv.py`：A1 import / A2 Mission→TaskRunner / A3 上浮后切回 ROV / A4 viskf import /
**A5 `on_exit` 里 kill + 零推力停推** / A6 import make_notifier / A7 TaskRunner 传 notify /
A8 `self.notify = make_notifier(C)` / A9 `notify.start()` + 启动提示 / A10 `on_cmd` 提示 /
A11 `on_exit` 里 `notify.stop()`。

`mode_dispatcher.py`：B1 import / B2 Mission→TaskRunner（若有） / B3 回挂 `m.dispatcher = self` /
B4 AUV 注册 / **B5 启动门控（只拦 `initial=True` 那次）** / B6 启动日志打门控摘要 /
**B7 模式记忆不写 AUV** / **B8 真实作业模式拒绝 `$VID`**。

⚠ B8 必须插在 `on = bool(payload)` **之后**：这行之后 dispatcher 才去 `video.set_enabled(on)`，
插在它前面没用、插在 `set_enabled` 之后就晚了。B8 用 `return` 直接退出 `on_pc_event`，
连 `mode().on_vid()` 都不调 —— 这样"除切模式外一律不接受"才是真的。

★ 必须回挂 `m.dispatcher = self`（`mode_base.py` 只有 `ctx`，没有 dispatcher 引用），
否则上浮后"切回 ROV"的请求传不出去。
