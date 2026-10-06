# 「搜索撞球」任务阶段方案（StrikeBall）v2.1 —— 落地依据（唯一方案源）

状态：**方案定稿待落地** · 更新：2026-10-06 21:59（v2.1：移入工程 + 自包含落地须知）
落点：`src/to32/move_task_plan/strike_ball_plan.md` → 实现目标文件 `src/to32/move_test/task/strike_ball.py`
对应工程：GrandRDKv2.5（板端 `/userdata/GrandRDK` 镜像）
参考工程：`D:\RC\S100\综合\GrandRDKv2.2`（viskf 真机经验，见 §4）

---

## 0. 落地须知（给实现者/下一次会话：读这一节即可动工）

**测试红线：用户未明确下令前，不写任何测试（冒烟/单元/自检都不写）。**

### 0.1 与现有代码的复用清单（全部 import 复用，不改对方）

| 复用对象 | 来自 | 用途 |
|---|---|---|
| `Stage` 基类契约：子类填 `NAME`，`enter(now)` + `step(now, dt) -> cmd \| None` | `move_test/mission.py` | SearchBall/StrikeBall 的骨架 |
| `apply_yaw_mirror(yaw, mirror)` | `mission.py` | 任务系 ↔ 固件系镜像（自逆）；遥测读回比较也用它 |
| `Ctx`：`ctx.cfg / vision / depth / tel / say()` | `mission.py` | 观测与日志入口；`ball_lost` 标志用 `setattr(ctx,...)` 挂上去，**不改 Ctx 类** |
| `VisionIF.poll('front', want, now)` | `move_test/obs.py` | 返回 `dx/dy/ex/ey/w/h/clip*/score/frame`；同帧去重（None≠没球）、0.5s 超期、score≥0.5 |
| `t_function.turn_step / forward_step` | `move_test/task/t_function.py` | 搜索摆动逐段转向、重试换位直行（完成判据已是 2026-10-06 口径） |
| `t_function._depth_out / _cmd / _say_throttled / _yaw_hold / yaw_err_deg` | 同上 | 定深沿用（绝不发 0）、cmd 组装、日志节流、锁航向、误差计算 |
| `task_config` 各键（getattr 兜底） | `move_test/task_config.py` | 本方案新增键见 §7 |

### 0.2 实现顺序建议

1. `task_config.py` 追加 §7 配置键（先有参数骨架）；
2. `task/strike_ball.py`：`_BallKF`（§4）→ `_PID`（§5）→ `StrikeBall`（§5，依赖 KF+PID）→ `SearchBall`（§3，依赖 t_function）；
3. `obs.py` CANON 追加蓝球别名一行（§7）；
4. `task_config.STAGE_TABLE` 暂不动——等全部阶段齐了再由用户排序（撞球段插在 Dive 之后，位置待 Task.md 全序列定）。

### 0.3 决策演进记录（为什么是现在这个设计）

| 版本 | 决策 | 理由 |
|---|---|---|
| v1 | front.py 不动，走 VisionIF 读 momo_det_front.json | 检测结果 JSON 已含 bbox/center/label/score，dx/dy 现成 |
| v1 | 拆 SearchBall + StrikeBall 两个 Stage（STAGE_TABLE 两行） | 用户"找到球后进入下一阶段"字面即 Stage 语义；职责清晰 |
| v1 | PID：Kp=25/Ki=0/Kd=0，输出限幅 ±10°/拍，仅新帧更新 | 去重+丢帧环境积分易炸；视觉噪声大不上 D；固件已有 yaw 内环，视觉 PID 是外环递推目标角 |
| **v2 修正 1** | 摆动是**相对 yaw0** 的偏移序列，下发 wrap(yaw0+offset) | 用户明确：不是绝对角 [15,-15,15]；找到球不回 yaw0；第二轮 yaw0 重置为换位后当前航向 |
| **v2 修正 2** | 撞球必须完成：首轮搜索失败→前进 2s→第二轮搜索→仍败→ASCEND 上浮兜底 + `ctx.ball_lost=True` | 用户明确撞球是必须任务；ball_lost 秒退零框架改动（不改 mission.py） |
| **v2 修正 3** | 引入图像卡尔曼：任务内嵌轻量 1D CV KF（不复活独立 viskf 进程） | 用户要求"尝试结合图像卡尔曼"；球跟随只有横向一个自由度，独立进程（v2.2 形态）改造成本高、延迟大；Gate 阶段将来若也要视觉卡尔曼再统一复活 |
| v2 | 丢失三级节拍 coast 0.5s 预测→冻结保持→2s 处置 | 对齐 v2.2 viskf 真机参数（coast 0.5 / trust age 0.2 / lost 2.0） |

### 0.4 待拍板问题（落地前用户确认；未确认时按"默认"实现）

1. 撞哪个球：默认红球（`AUV_BALL_WANT='ball'`）。
2. 摆动先右后左 `[+15,−15,+15]` 还是先左：默认先右。
3. 近场判据"平滑框宽占比 ≥0.45 + 定时冲刺"：默认接受。
4. 首轮搜索原地摆（surge=0）：默认原地；重试段已定前进 2s 换位。
5. KF 路线确认内嵌轻量版：默认内嵌。
6. track 丢失 2s 处置：默认"完成进下一阶段 + 警告"。
7. ball_lost 秒退机制：默认采用。

---

## 1. 目标与范围

实现 Task.md 阶段序列中的 `STRIKE_BALL`：摆动搜索前视画面中的目标球 → 找到后用视觉 PID + 图像卡尔曼跟随球心，保持前进直至撞球。

- **只闭 yaw**：画面纵向偏差（dy）由定深消化，不做深度/纵倾闭环。
- **撞球是必须完成任务**：搜索自带一轮重试，终极兜底上浮。
- **守 Task.md §5 红线**：全程防露头定深、每段定时兜底、绝不死等。

## 2. 接口快照（2026-10-06 实测，落地时以此为准）

| 需求 | 现状 | 结论 |
|---|---|---|
| 前视检测结果 | `src/front.py` 写 `/dev/shm/momo_det_front.json`：`{'frame','ts','dets':[{bbox,center,label,score}]}`，写端 ~10Hz | 任务侧只读，不碰 front.py |
| 目标框与画面中心相对位置 | `obs.VisionIF.poll('front','ball',now)` 现成返回 `dx/dy`（带符号像素差）+ 归一化 `ex/ey` | 直接可用 |
| 丢帧/烂帧防御 | 同帧号去重（同帧不出两次）、mtime 超 0.5s 判无目标、半 JSON 兜空、score≥`AUV_MIN_SCORE`=0.5、贴边 clip 标志 | obs.py 已封装 |
| 距离/速度 | VisionIF ★无距离字段；V2 遥测无 vx | 撞到判据只能用像素近场代理 + 定时兜底 |
| CANON 现状 | `red-ball → 'ball'`（撞球目标）；**无蓝球**；`yellow-ball → 'ball_y'`；`door → 'gate'` | 蓝球占表位待模型实装 |

数据链路（内嵌 KF 版）：

```
front.py(不动) → momo_det_front.json(~10Hz)
  → VisionIF.poll('front','ball') → dx 像素差
  → _BallKF 图像卡尔曼（任务内嵌，20Hz tick：新帧 update / 丢帧 predict）
  → PID(横向偏差) → 递推 yaw_ref → cmd → 0x09
```

## 3. SearchBall —— 相对摆动搜索（一轮重试 + 上浮兜底）

### 3.1 相对 yaw 口径（★ 修正 1）

- `enter` 记基准航向 `yaw0`（mirror 后任务系角，取自遥测 `actual_yaw` 经 `apply_yaw_mirror`；无遥测时按 0 并节流警告）。
- 摆动序列存**相对偏移**：`AUV_SEARCH_PATTERN = [+15, −15, +15]`（°，相对 yaw0）。
- 每段下发角 = `wrap(yaw0 + offset)`，交 `t_function.turn_step()` 闭环（0x09 语义是绝对角，
  但**序列定义与调参一律用相对偏移**，绝无绝对角 [15,−15,15] 的歧义）。
- 找到球交棒时**不回 yaw0**：StrikeBall 从交棒时的当前实际航向起步（PID 初值 yaw_ref = 当前航向）。
- 第二轮搜索的 yaw0 **重置为重试前进结束时的当前实际航向**（机体已移位，旧基准失效）。

### 3.2 状态机

```
[SEQ_TURN i] --到位--> [SEQ_DWELL i] --窗内命中≥N帧--> 交棒 StrikeBall
     ^                      | 全窗无命中
     |                      v
     +---- i 未完 <---- [SEQ_NEXT]
                            | 3 段跑完仍无球
                            v
                    [RETRY_FWD] 前进 AUV_RETRY_FORWARD_S=2.0s（surge=0.4 巡航、yaw 跟随保持、定深）
                            |
                    重试额度未用(默认 1 轮) → 重置 yaw0=当前航向 → 回 [SEQ_TURN 0]
                            | 重试额度用完仍无球
                            v
                    [ASCEND] 兜底上浮：depth=离面安全目标、surge=0、
                             保持 AUV_ASCEND_HOLD_S=6s → 完成并置 ctx.ball_lost=True
```

- 命中判定：观察窗（`AUV_SEARCH_DWELL_S`=1.5s）内 `poll('front','ball')` 视觉命中
  ≥ `AUV_SEARCH_SEEN_N`=3 帧（按帧不按拍，写端 10Hz 下冗余足够）。搜索段用原始帧计数，
  不引 KF（观察窗短，简单可靠）。
- ⚠ 坑：poll 同帧去重返回 None ≠ 没球；"没球"只能判"整个观察窗命中数为 0"。
- RETRY_FWD 段：`forward_step` 语义（定时、锁当前航向、定深沿用），2s 短促换位。
- ASCEND 段：depth 目标 = `clamp_depth_cm` 下限（离面安全 `AUV_SURF_SAFE_CM`=25cm 对应值），
  纯定时保持 6s（按 1.2m 水深、~0.15m/s 上浮估），不判融合深度到位。
- `ctx.ball_lost=True`（setattr 到 Ctx）：后续需要球的 Stage 在 `enter` 里检查该标志
  直接返回 None 秒退，等效"跳过剩余阶段直奔收尾"——避免没球还乱跑。

## 4. 图像卡尔曼（★ 修正 3）

### 4.1 路线：任务内嵌轻量 KF（推荐），不复活独立 viskf 进程

| | A. 复活 v2.2 viskf 独立进程 | B. 内嵌轻量 KF（采用） |
|---|---|---|
| 形态 | 独立进程 50Hz，写 momo_viskf.json | strike_ball.py 内 ~60 行，Stage 生命周期内 |
| 改造成本 | e_x 按"门宽"归一化，球无门宽参照，口径必须重写；姿态补偿对球跟随无用 | 直接滤像素 cx，无语义改造 |
| 延迟 | 跨进程 + JSON 往返 | 同进程 20Hz tick，无延迟 |
| 适用面 | 将来过门（Gate）也要视觉卡尔曼时统一复用 | 只服务撞球 |
| 结论 | 备选：Gate 阶段立项时再统一复活 | **本方案采用** |

v2.2 参考文件（要用时再翻）：`GrandRDKv2.2/src/kalman/camera_kalman/viskf.py`（滤波器本体）、
`config/viskf_config.py`（参数与真机教训注释）、`src/to32/move_test/viskf_if.py`（读端 trust 门禁）。

### 4.2 设计（借鉴 v2.2 真机教训）

- **模型**：一维常速（CV）KF，状态 `[cx, vcx]`（像素、像素/s），F=[[1,dt],[0,1]]，量测 z=cx。
  手写标量/2×2 运算，不引 numpy 依赖。
- **量测噪声**：`AUV_BALL_KF_R_PX2 = 15²`（检测框中心抖动 ~15px，TODO 实测回填；
  v2.2 门框 e_x 实测 σ=0.0177 归一化 ≈ 11px 可作先验）。
- **过程噪声**：`AUV_BALL_KF_Q_ACC`（加速度谱密度，默认偏大 = 信观测；跟随段球近似匀速）。
- **新息门限**：`|z − cx_pred| > 3σ + floor`（floor ≈ √R；v2.2 教训：不加 floor 好观测被误拒，
  σ 回填后拒收从 20 涨到 35 拍）→ 拒收按丢帧处理；连续拒收 `AUV_BALL_KF_RESET_N`=5 帧 → KF 重置重捕。
- **贴边防护**：检测框贴边（clip 标志）时该帧量测方差 ×4（v2.2 分边放大的简化版）。
- **w 通道**：框宽用 EMA 平滑（`AUV_BALL_W_EMA`=0.3），近场判据用平滑值，防单帧抖动误触发 ram。
- **丢失三级节拍**（对齐 v2.2：coast 0.5s / trust age 0.2s / lost 2.0s）：
  1. 丢帧 ≤ `AUV_BALL_KF_COAST_S`=0.5s：**KF 纯预测滑行**，用预测 cx 继续闭环（方向由 vcx 外推）；
  2. coast 超窗但 < `AUV_BALL_LOST_S`=2.0s：**冻结** yaw_ref 保持上拍 + surge 继续（保底）；
  3. ≥ 2.0s：丢失处置（默认完成进下一阶段 + 警告，见 §0.4-6）。
- **trust 门禁**（v2.2 最大的坑）：只有"新帧 age ≤ 0.2s 且滤波 σ ≤ 上限"的输出才允许进 PID；
  外推值 σ 持续膨胀，**绝不拿长时间外推值控舵**（v2.2 实测：门离场 6.68s 外推把 e_x 推到 +2.38，
  正常上限 +0.60，σ 到 2.40——外推垃圾喂舵 = 满舵乱转）。
- 归一化口径**恒定**：`ex_kf = (cx_kf − W/2)/(W/2)`，全程除画面宽，不切换
  （v2.2 教训：口径切换造成 0.209 假跳变，是真机踩出来的）。

## 5. StrikeBall —— PID 跟随 + 冲撞（track → ram）

**track（跟随）**

- 每拍 `poll('front','ball')` → 命中帧喂 `_BallKF.update(cx)`，丢帧走 `predict(dt)` 三级节拍（§4.2）。
- 控制律（**仅 trust 输出可用时**更新）：
  - `err = ex_kf`（归一化横向偏差）
  - PID → 输出限幅 ±`AUV_PID_OUT_MAX`=10°/拍
  - `yaw_ref = wrap(yaw_ref − SIGN·out)` 递推目标角（固件闭 yaw 内环，视觉 PID 是外环；
    ex_kf→0 时 yaw_ref 自动锁定）；初值 = 交棒时当前航向
  - surge = `AUV_TRACK_SURGE`=0.35 保持前进
- dy 不管：纵向偏差由定深消化。

**ram（冲撞）**

- 切入：平滑框宽占比 `w_ema / AUV_IMG_W ≥ AUV_RAM_W_RATIO`=0.45 持续 `AUV_RAM_SEEN_N`=5 拍；
- 动作：surge=0.9 满推 + 锁 yaw_ref，持续 `AUV_RAM_S`=1.0s → 完成；
- 辅助完成：近场后目标出画/丢失 ≥ 0.5s → 也判撞过（防冲过头漏判）。

**PID 参数**：Kp=25（°/单位ex）起步，Ki=0（实装带 I 限幅 5° 与丢失冻结，默认关），Kd=0。
⚠ `AUV_BALL_YAW_SIGN=±1` 上车一次实测（dx>0 → 任务系航向往哪转，纸面定不了）。

## 6. 完成判据与兜底总表

| 段 | 主判据 | 兜底 |
|---|---|---|
| SearchBall 单段 | 观察窗内视觉命中 ≥3 帧 | 下一段/下一轮 |
| SearchBall 轮 | —— | 首轮失败 → 前进 2s 重试一轮 |
| SearchBall 终极 | —— | **上浮**（depth=离面安全，6s）+ ball_lost 标志 |
| StrikeBall.track | 框宽占比 ≥0.45 连续 5 拍 | 总超时 `AUV_STRIKE_TIMEOUT_S`=15s |
| StrikeBall.ram | 定时 1.0s | 近场后出画也算完成 |
| KF 丢失 | —— | coast 0.5s 预测 → 冻结保持 → 2s 处置 |
| 深度 | 融合深度（Dive 口径，t_function 已定） | 融合源失效 → 定时兜底 |

## 7. 文件改动清单与配置键

| 文件 | 动作 | 内容 |
|---|---|---|
| `task/strike_ball.py` | **新建** ~260 行 | `_BallKF`（~60 行）+ `_PID` + `SearchBall(Stage)` + `StrikeBall(Stage)` |
| `task_config.py` | 追加 | 下表 ~24 键 |
| `obs.py` | 追加 1 行 | CANON 登记蓝球别名 `blue-ball/blue_ball → 'ball_b'`（模型未实装，占表位） |
| `front.py` / `mission.py` / `mode_auv.py` / `t_function.py` | **不动** | 只被 import 复用（ball_lost 用 setattr 挂 Ctx，不改 Ctx 类） |

```python
# --- SearchBall（相对 yaw 口径）---
AUV_SEARCH_PATTERN   = [15.0, -15.0, 15.0]  # 相对 yaw0 的偏移序列（°）
AUV_SEARCH_DWELL_S   = 1.5                  # 每顶点观察窗
AUV_SEARCH_SEEN_N    = 3                    # 窗内视觉命中帧数门槛
AUV_SEARCH_TIMEOUT_S = 20.0                 # 单轮搜索超时（含 3 段摆动）
AUV_SEARCH_SURGE     = 0.0                  # 搜索时前进推力（0=原地摆）
AUV_SEARCH_RETRIES   = 1                    # 搜索失败重试轮数
AUV_RETRY_FORWARD_S  = 2.0                  # 重试前换位前进时长
AUV_RETRY_SURGE      = 0.4                  # 重试前进推力
AUV_ASCEND_HOLD_S    = 6.0                  # 终极兜底上浮保持时长
# --- StrikeBall / PID ---
AUV_BALL_CAM         = 'front'
AUV_BALL_WANT        = 'ball'               # 撞红球；区分红蓝待模型实装
AUV_PID_KP           = 25.0
AUV_PID_KI           = 0.0                  # 实装但默认关
AUV_PID_KD           = 0.0
AUV_PID_OUT_MAX      = 10.0                 # 单拍输出限幅（°）
AUV_PID_I_MAX        = 5.0
AUV_BALL_YAW_SIGN    = 1.0                  # ⚠ 上车标定
AUV_TRACK_SURGE      = 0.35
AUV_RAM_W_RATIO      = 0.45
AUV_RAM_SEEN_N       = 5
AUV_RAM_SURGE        = 0.9
AUV_RAM_S            = 1.0
AUV_STRIKE_TIMEOUT_S = 15.0
# --- 图像卡尔曼 ---
AUV_BALL_KF_R_PX2    = 225.0                # 量测方差（15px）²，TODO 实测回填
AUV_BALL_KF_Q_ACC    = 800.0                # 过程噪声加速度谱密度，TODO 标定
AUV_BALL_KF_GATE_NSIGMA = 3.0               # 新息门限
AUV_BALL_KF_RESET_N  = 5                    # 连续拒收 N 帧重置
AUV_BALL_KF_COAST_S  = 0.5                  # 预测滑行窗（对齐 v2.2）
AUV_BALL_TRUST_AGE_S = 0.2                  # trust 门禁：距最近有效观测
AUV_BALL_LOST_S      = 2.0                  # 丢失处置门限
AUV_BALL_W_EMA       = 0.3                  # 框宽 EMA 系数
```

## 8. 已知风险

| 风险 | 缓解 |
|---|---|
| KF 预测滑行漂移（球机动/转向时外推错） | coast 窗仅 0.5s + trust 门禁（σ 膨胀即拒用）+ 超窗冻结，v2.2 同参数真机验证过 |
| 检测误检把 KF/舵拉飞 | 新息门限 3σ+floor 拒收 + 连续 5 帧重置重捕 + score≥0.5 前置 |
| 模型尚无球类别（blue/pink ball、ring 不在 CLASS_NAMES） | 控制链路先行；`AUV_BALL_WANT` + CANON 换名即接新模型 |
| yaw-图像符号不确定 | `AUV_BALL_YAW_SIGN` 上车首测项 |
| 近场框宽抖动误触发 ram | w 走 EMA + 连续 5 拍确认 + ram 定时 + 总超时四层兜底 |
| 重试前进 2s 可能顶到障碍物 | 巡航推力 0.4 短促换位，池内环境可接受；无 vx 判据，接受此风险 |
| 上浮速率估计不准（6s 可能未到面） | 纯定时保底不闭环；上浮不足也显著离底，风险可控 |
