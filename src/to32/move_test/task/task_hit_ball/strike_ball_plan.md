# 「撞球」任务阶段方案（HitBall）v3 —— 落地依据（唯一方案源）

状态：**方案定稿待落地** · 更新：2026-10-07 11:15（v3：对齐 vservo 统一规划 v1.1；yaw 符号与失败退出机制已拍板）
落点：实现 `src/to32/move_test/task/t_hit_ball.py`（现空骨架，`HIT_BALL_TABLE=[]`）；公共层 `task/vservo.py` 先行（vservo 规划 §5 步 1）
前置总纲：`task/vservo_统一架构与落地规划_2026-10-07.md`（v1.1：公共层零件、三任务参数独立、盲段统一走既有原语）
对应工程：GrandRDKv2.5（板端 `/userdata/GrandRDK` 镜像）
参考工程：`D:\RC\S100\综合\GrandRDKv2.2`（viskf 真机经验，见 §4）

---

## 0. 落地须知（给实现者/下一次会话：读这一节即可动工）

**测试红线：用户未明确下令前，不写任何测试（冒烟/单元/自检都不写）。**

### 0.1 v3 相对 v2.1 的变更（全部已拍板，2026-10-07）

| # | 变更 | 依据 |
|---|---|---|
| 1 | **删除 SearchBall 章**（v2.1 §3 整章作废）：搜索由用户 `t_search_ball.py`（下视 zigzag）承担，撞球从"前视已见球"起步 | vservo v1.1 §4.1 |
| 2 | **键改名**：v2.1 的 `AUV_PID_* / AUV_BALL_* / AUV_RAM_*` 全部改名归入 `AUV_HIT_*`（§6 键表即终稿）；v2.1 计划的 9 个 `AUV_SEARCH_*` 键废弃（从未入过 task_config，勿再加） | vservo v1.1 §3/§4.1 |
| 3 | **机构替换**：v2.1 的 `_BallKF/_PID` → 公共层 `vservo.KF1D / vservo.PID`（构造注入本任务键，零全局默认） | vservo v1.1 §1 |
| 4 | **yaw 递推公式定稿**：`yaw_ref = wrap(yaw_ref + AUV_HIT_YAW_SIGN·PID输出)`，★**dx>0 → 右转**（task 系 yaw 右为正）→ 参数默认 +1，从"上车未知"降级为"上车确认项"；v2.1 的 `yaw_ref − SIGN·out` 公式作废 | 用户 2026-10-07 11:07 |
| 5 | **ram 盲冲改用既有原语**：`t_function.forward_step(AUV_HIT_RAM_S, surge=AUV_HIT_RAM_SURGE, target_height_cm=AUV_HIT_HEIGHT_CM, stage='Ram')`，不写新原语 | vservo v1.1 §2 |
| 6 | **失败处置参数化（退出机制）**：所有失败路径汇入 `_fail_exit()`，动作由 `AUV_HIT_FAIL_ACTION` 一键决定（'skip'/'ascend'/'retry'）；用户当前倾向"炸了直接上浮重开下一轮"，正式定夺后改键即生效，不动代码 | 用户 2026-10-07 11:12 |
| 7 | track 丢失 ≥2s 的处置从 v2.1 的"警告"改为与超时同款退出机制 | vservo v1.1 §4.1 |

### 0.2 复用清单（全部 import 复用，不改对方）

| 复用对象 | 来自 | 用途 |
|---|---|---|
| `Stage` 基类契约：填 `NAME`，`enter(now)` + `step(now, dt) -> cmd \| None` | `move_test/mission.py` | HitBallAll 骨架（整体一个 Stage，同构 t_task1/t_search_ball） |
| `vservo.KF1D / vservo.PID / trust_ok / 丢失节拍` | `task/vservo.py`（步 1 先建） | 滤波/PID/门禁，构造注入 AUV_HIT_* 键 |
| `Ctx`：`ctx.vision / ctx.tel / ctx.say()` | `mission.py` | 观测与日志；`ball_lost` 用 `setattr(ctx,...)` 挂载，不改 Ctx 类 |
| `VisionIF.poll('front','ball',now)` | `move_test/obs.py` | 返回 dx/dy/ex/ey/w/h/clip*/score/frame；同帧去重、0.5s 超期、score≥0.5 |
| `t_function.forward_step / _depth_out / _cmd / _say_throttled` | `move_test/task/t_function.py` | ram 盲冲、定深沿用（绝不发 0）、cmd 组装、日志节流 |
| `task_config` 各键（getattr 兜底） | `move_test/task_config.py` | 本方案 §6 键表 |

### 0.3 实现顺序（= vservo 规划 §5，撞球先行；每步交付即停等验收）

1. `task/vservo.py`（KF1D/KF2D/PID/yaw_servo_step/工具 ~140 行，零 task_config 依赖）；
2. `task_config.py` 追加 §6 的 AUV_HIT_* 键（只追加不改动现有键；STAGE_TABLE 不动）；
3. `task/t_hit_ball.py` 实现 HitBallAll（~120 行）+ 本方案状态行收尾；STAGE_TABLE/TEST_TABLE 编排始终留给用户。

## 1. 目标与范围

实现 STAGE_TABLE 中的 `HitBall`（撞球段）：前视画面已见球的前提下，视觉 PID + 图像卡尔曼跟随球心（track），近场后盲冲撞球（ram），全程防露头、绝不死等。

- **只闭 yaw**：纵向偏差由定深消化，不做深度/纵倾闭环。
- **在序列中的位置**：`Task1 → SearchBall（下视 zigzag 找球，用户自写）→ HitBall → Task2 → …`。
- **起步假设**：进入本阶段时前视画面可见目标球；若不可见，track 段丢失节拍 2s 内快速失败 → 走 §3.3 退出机制，不会死等。SearchBall（下视交棒）与 HitBall（前视接力）的衔接是上车首测项。
- **规则红线**（Task.md §4.2）：撞球必须最先完成；挂碰/碰线不得分 → 对准精度不能省（近场判据连续 5 拍确认、PID 无 I 积累防超调）。

## 2. 接口快照（2026-10-06/07 实测核实）

| 需求 | 现状 | 结论 |
|---|---|---|
| 前视检测结果 | `src/front.py` 写 `/dev/shm/momo_det_front.json`（bbox/center/label/score），~10Hz | 任务侧只读，front.py 不动 |
| 目标框相对位置 | `obs.VisionIF.poll('front','ball')` 返回 dx/dy/ex/ey/w/h/clip*/score/frame | 直接可用（obs.py L67/L140-144 核实） |
| 丢帧/烂帧防御 | 同帧去重、mtime 超 0.5s 判无目标、半 JSON 兜空、score≥0.5、贴边 clip 标志 | obs 层已封装 |
| 距离/速度 | VisionIF 无距离字段；V2 遥测无 vx | 撞到判据只能用像素近场代理 + 定时兜底 |
| CANON | `red-ball → 'ball'`（obs.py L23）；**无蓝球**（模型 CLASS_NAMES 无 blue，抽中蓝球属模型侧债务） | `AUV_HIT_WANT` + CANON 换名即接新模型 |
| Stage 契约 | `enter(now)` / `step(now,dt)`，`self.ctx` 由 Mission 注入 | mission.py L51-61 核实 |

数据链路：

```
front.py(不动) → momo_det_front.json(~10Hz)
  → VisionIF.poll('front','ball') → cx 像素
  → vservo.KF1D（20Hz tick：新帧 update / 丢帧 predict）
  → vservo.PID(ex_kf) → 递推 yaw_ref（yaw_servo_step 语义）→ cmd → 0x09
```

## 3. HitBallAll —— 整体一个 Stage：track → ram → 退出机制

同构 t_task1/t_search_ball：STAGE_TABLE 一项 = 整个撞球任务；内部子状态机 track/ram/fail_exit；
所有状态对象在 `enter()` 重建（KF/PID 实例构造注入本任务键）——测试模式循环重跑天然安全。

### 3.1 track（跟随）

- 每拍 `poll('front','ball')`：命中帧喂 `KF1D.update(cx)`（clip 贴边帧量测方差 ×4），丢帧走 `predict(dt)` 三级节拍（§4.2）。
- 控制律（**仅 trust 输出可用时**更新 yaw_ref）：
  - `err = ex_kf = (cx_kf − W/2)/(W/2)`（归一化口径恒定，全程除画面宽不切换——v2.2 教训）
  - `out = PID(err)`，限幅 ±AUV_HIT_PID_OUT_MAX=10°/拍
  - `yaw_ref = wrap(yaw_ref + AUV_HIT_YAW_SIGN·out)` 递推目标角（★已拍板 dx>0→右转→默认 +1；固件闭 yaw 内环，视觉 PID 是外环；ex→0 时 yaw_ref 自动锁定）
  - surge = AUV_HIT_TRACK_SURGE=0.35 保持前进
- dy 不管：纵向偏差由定深消化；depth 走 t_function._depth_out 沿用（AUV_HIT_HEIGHT_CM=60cm 距底，绝不发 0）。

### 3.2 ram（盲冲）

- 切入：`w_ema / AUV_IMG_W ≥ AUV_HIT_RAM_W_RATIO(0.45)` **连续 AUV_HIT_RAM_SEEN_N(5) 拍**（w 走 EMA 0.3 防单帧抖动误触发）；
- 动作：`t_function.forward_step(duration_s=AUV_HIT_RAM_S(1.0), surge=AUV_HIT_RAM_SURGE(0.9), target_height_cm=AUV_HIT_HEIGHT_CM, stage='Ram')` —— 锁航向 + 定深 + 定时，**必然结束**；
- 辅助完成：近场后目标出画/丢失 ≥0.5s → 也判撞过（防冲过头漏判）。

### 3.3 退出机制（★ v3 新增：失败处置参数化，2026-10-07 用户拍板）

**设计**：所有失败路径（KF 丢失 ≥2s / track 总超时 AUV_HIT_TIMEOUT_S=15s / retry 额度用尽）
统一汇入内部 `_fail_exit()`，处置动作由**一个键**决定——用户后续定夺时改键即生效，不动代码：

| `AUV_HIT_FAIL_ACTION` | 行为 | 语义 |
|---|---|---|
| `'skip'`（当前默认） | 置 `ctx.ball_lost=True` → 本阶段完成（返回 None）进下一阶段 | v1.1 拍板口径；下游需要球的 Stage（捡球）enter 秒退 |
| `'ascend'` | 先上浮子段（depth=clamp_depth_cm 下限即离面安全值、surge=0、纯定时 AUV_HIT_ASCEND_HOLD_S=6s 非闭环）→ 走完同 'skip' | "炸了上浮重开"的半实现：本阶段负责上浮安全，重开交给整轮编排 |
| `'retry'` | 重置内部状态机（KF 重建、yaw_ref=当前航向、计数清零）重新 track；额度 AUV_HIT_RETRIES=1，用尽 → 回退 'skip' 语义 | 适用"球还在画面但跟炸了"（误检拉飞/瞬时遮挡） |

- **retry 边界（写死）**：不倒退回 SearchBall（STAGE_TABLE 不回头）；画面无球时 retry 会在 2s 内再失败烧掉额度——额度上限保证必有出口。
- `ball_lost` 语义不变：置位后下游捡球 Stage 在 enter 检查直接秒退，避免没球乱跑。
- 用户当前倾向：炸了直接上浮、重新进行下一轮（≈ 'ascend'）；正式定夺后改 `AUV_HIT_FAIL_ACTION` 一个键。

## 4. 图像卡尔曼（vservo.KF1D）与 track 节拍推导

### 4.1 设计（继承 v2.1 §4 全部真机教训）

- 1D 常速 CV KF，状态 `[cx, vcx]`，F=[[1,dt],[0,1]]，量测 z=cx；vservo.KF1D 手写 2×2，不引 numpy；
- 参数构造注入：R=AUV_HIT_KF_R_PX2=15²（TODO 实测回填）、Q=AUV_HIT_KF_Q_ACC（偏大=信观测）、
  新息门限 3σ+floor（floor≈√R；v2.2 教训：无 floor 好观测被误拒，拒收 20→35 拍）、
  连续 AUV_HIT_KF_RESET_N=5 帧拒收 → KF 重置重捕；
- 贴边防护：clip 帧量测方差 ×4（v2.2 分边放大的简化版）；w 通道 EMA 0.3 平滑仅供 ram 判据；
- **trust 门禁**：仅"新帧 age ≤ 0.2s 且 σ ≤ 上限"的输出才进 PID——绝不拿长时间外推值控舵
  （v2.2 实测：门离场 6.68s 外推把 e_x 推到 +2.38，正常上限 +0.60——外推垃圾喂舵 = 满舵乱转）；
- 归一化口径恒定：全程 `ex=(cx−W/2)/(W/2)`，不切换（v2.2 教训：口径切换 0.209 假跳变）。

### 4.2 track 节拍是怎么算的（2026-10-07 用户问，答录）

**核心量**：`age = now − last_hit_ts`（KF 最近一次成功吃进新帧的时刻），每拍（20Hz，dt≈0.05s）计算分档。
**三级节拍是粗节拍，trust 门禁是 coast 段内的细门禁**，两层叠加后的实际行为：

| age 区间 | 档位 | 行为 |
|---|---|---|
| 0 ~ 0.2s | coast · 新鲜 | KF 每拍 predict + 外推值仍够新鲜 → **PID 继续更新** yaw_ref |
| 0.2 ~ 0.5s | coast · 过期 | KF 继续滚（保状态温热，新帧到来无缝衔接）但 trust 门禁挡住外推值 → **PID 停更，yaw_ref 锁住** |
| 0.5 ~ 2.0s | 冻结 | 明确冻结档：yaw_ref 保持上拍 + surge 继续（保底前进），不依赖 KF 输出 |
| ≥ 2.0s | 处置 | `_fail_exit()` → 按 AUV_HIT_FAIL_ACTION 处置 |

任意时刻新帧到来 → age 归零回正常 track。

**三个门限的推导（都不是拍脑门）：**

| 门限 | 值 | 推导 |
|---|---|---|
| coast | 0.5s | 前视写端 ~10Hz → 0.5s ≈ **连丢 5 帧**；球横向近似匀速时 CV 外推 5 帧内漂移可控（速度状态由最近观测撑住）；再长，转向机动下外推就是"猜"→ 封顶转冻结。v2.2 同参数真机验证过 |
| trust age | 0.2s | ≈ **2 个检测写帧周期**：正常 10Hz 下 age 峰值 ~0.1s（一拍 50ms + 半个写帧周期），0.2s 给 2 帧余量；连丢 2 帧起不许外推值进 PID。它不是丢失判据，是防"外推垃圾喂舵"的新鲜度门禁 |
| lost | 2.0s | 区分**瞬时丢**（遮挡/框抖/短暂出画，通常 1s 内回来）与**真丢**（球出 FOV/检测持续失败）；连续 2s 无新帧继续跟无意义 → 触发处置。v2.2 真机参数 |

## 5. 完成判据与兜底总表

| 段/情形 | 主判据 | 兜底/出口 |
|---|---|---|
| track 跟随 | w_ema/W≥0.45 连续 5 拍 → 切 ram | KF 丢失节拍（§4.2）+ 总超时 15s → `_fail_exit()` |
| ram 冲撞 | 定时 1.0s 必然结束 | 近场后出画/丢失≥0.5s 也判完成 |
| 误检防御 | —— | 3σ+floor 拒收 + 连续 5 帧重置 + score≥0.5 前置 + trust 门禁 |
| 失败处置 | —— | `AUV_HIT_FAIL_ACTION` 参数化（skip/ascend/retry），retry 额度上限保证必有出口 |
| 深度安全 | 每帧 0x09 带正 depth_cm | clamp_depth_cm 抬到离面 25cm 之上（防露头红线） |
| 框架层 | —— | Stage 异常 → Mission 立即停推收尾；脚本缺失 → 空表开机即 DONE |
| ★无死等面 | —— | 全段完成判据 = 定时/像素代理/标志位，不依赖 Dive 融合深度带、不依赖 Turn yaw 到位（yaw 闭环在固件），不存在"判据失效永不完成"死等面 |

## 6. 文件改动清单与配置键

| 文件 | 动作 | 内容 |
|---|---|---|
| `task/vservo.py` | **新建** ~140 行（vservo 规划步 1） | KF1D/KF2D/PID/yaw_servo_step/trust_ok/丢失节拍工具；零 TC 依赖 |
| `task/t_hit_ball.py` | **实现** ~120 行（现空骨架） | HitBallAll(Stage)：track/ram/_fail_exit 子状态机 |
| `task_config.py` | 追加 | 下表 ~26 键（只追加不改动现有键；HIT_BALL_TABLE try-import 已预挂） |
| `mission.py / mode_auv.py / obs.py / t_function.py / front.py / t_search_ball.py` | **不动** | 只被 import 复用（ball_lost 用 setattr 挂 Ctx） |

```python
# --- HitBall 撞球（v3，键源 strike v2.1 改名归入；只追加） ---
AUV_HIT_CAM            = 'front'
AUV_HIT_WANT           = 'ball'      # 撞红球；蓝球待模型实装后换名即接
AUV_HIT_HEIGHT_CM      = 60.0        # 撞球工作高度（距池底），track/ram 全程沿用
AUV_HIT_PID_KP         = 25.0        # °/单位ex，TODO 上车调参
AUV_HIT_PID_KI         = 0.0         # 实装但默认关（I 限幅 AUV_HIT_PID_I_MAX）
AUV_HIT_PID_KD         = 0.0
AUV_HIT_PID_OUT_MAX    = 10.0        # 单拍输出限幅（°）
AUV_HIT_PID_I_MAX      = 5.0
AUV_HIT_YAW_SIGN       = 1.0         # ★已拍板：dx>0→右转（yaw 增）→ 默认 +1；上车确认项
AUV_HIT_TRACK_SURGE    = 0.35        # 跟随段前进推力
AUV_HIT_RAM_W_RATIO    = 0.45        # 近场判据：w_ema/画面宽占比
AUV_HIT_RAM_SEEN_N     = 5           # 近场连续确认拍数
AUV_HIT_RAM_SURGE      = 0.9         # 冲撞推力
AUV_HIT_RAM_S          = 1.0         # 冲撞时长（定时必结束）
AUV_HIT_TIMEOUT_S      = 15.0        # track 总超时 → _fail_exit
AUV_HIT_KF_R_PX2       = 225.0       # 量测方差 (15px)²，TODO 实测回填
AUV_HIT_KF_Q_ACC       = 800.0       # 过程噪声加速度谱密度，TODO 标定
AUV_HIT_KF_GATE_NSIGMA = 3.0         # 新息门限（+floor≈√R）
AUV_HIT_KF_RESET_N     = 5           # 连续拒收 N 帧重置重捕
AUV_HIT_KF_COAST_S     = 0.5         # 纯预测滑行窗（≈写端 5 帧）
AUV_HIT_TRUST_AGE_S    = 0.2         # 喂舵新鲜度门禁（≈2 个写帧周期）
AUV_HIT_LOST_S         = 2.0         # 真丢处置门限
AUV_HIT_W_EMA          = 0.3         # 框宽 EMA 系数
# --- 退出机制（2026-10-07 新增，处置策略参数化） ---
AUV_HIT_FAIL_ACTION    = 'skip'      # 'skip'=置ball_lost完成进下阶段 | 'ascend'=先上浮再skip | 'retry'=重置重跟
AUV_HIT_RETRIES        = 1           # retry 额度，用尽回退 skip 语义
AUV_HIT_ASCEND_HOLD_S  = 6.0         # ascend 上浮保持时长（纯定时非闭环）
```

## 7. 已知风险

| 风险 | 缓解 |
|---|---|
| KF 预测滑行漂移（球机动/转向时外推错） | coast 窗仅 0.5s + trust 门禁（0.2s 起 PID 停更）+ 超窗冻结；v2.2 同参数真机验证 |
| 检测误检把 KF/舵拉飞 | 3σ+floor 拒收 + 连续 5 帧重置 + score≥0.5 前置 + FAIL_ACTION='retry' 可快速重置重跟 |
| 起步时前视无球（SearchBall 下视交棒 → 前视接力的口径差） | track 丢失节拍 2s 快速失败 → _fail_exit，绝不死等；衔接是上车首测项 |
| 模型尚无蓝球类别 | 控制链路先行；AUV_HIT_WANT + CANON 换名即接新模型 |
| 近场框宽抖动误触发 ram | w EMA + 连续 5 拍 + ram 定时 + 总超时四层兜底 |
| ascend 上浮 6s 可能未到面 | 纯定时保底不闭环；上浮不足也显著离面，风险可控 |
| Kp=25/R/Q 均先验值 | 全部集中 AUV_HIT_* 键，上车调参不改代码 |

## 8. 决策演进记录

| 版本 | 决策 | 理由 |
|---|---|---|
| v1 | 拆 SearchBall+StrikeBall 两 Stage；front.py 不动走 VisionIF | 用户"找到球进入下一阶段"字面即 Stage 语义 |
| v2 | 摆动相对 yaw0；撞球必须完成+上浮兜底；内嵌轻量 KF（不复活独立 viskf 进程） | 用户拍板修正；独立进程改造成本高、延迟大 |
| v2.1 | 移入工程 task_hit_ball/ + 自包含落地须知 | 唯一方案源约定 |
| **v3** | 删 SearchBall 章（t_search_ball 承担）；键改名 AUV_HIT_*；机构 vservo.KF1D/PID；ram=forward_step 显式 surge；**符号拍板 dx>0→右转（正号递推公式）**；**失败处置参数化 AUV_HIT_FAIL_ACTION（skip 默认 / ascend / retry）**，用户倾向 ascend 上浮重开，定夺后改一键 | vservo 统一规划 v1.1（2026-10-07）+ 用户 11:07 / 11:12 拍板 |
