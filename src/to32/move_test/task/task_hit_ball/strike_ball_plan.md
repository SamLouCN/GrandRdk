# 「撞球」任务阶段方案（HitBall）v3.1 —— 落地依据（唯一方案源）

状态：**v3.1 已落地**（2026-10-07 17:40：vservo.py + t_hit_ball.py + task_config AUV_HIT_* 30 键 + README_hitball；板端等价导入实测 HitBall 入链 6 段）
落点：`src/to32/move_test/task/vservo.py`（公共层，新建）/ `task/task_hit_ball/t_hit_ball.py`（状态机，已实现）/ `task_config.py`（AUV_HIT_* 键，已追加）
前置总纲：`task/vservo_统一架构与落地规划_2026-10-07.md`（v1.1：公共层零件、三任务参数独立、盲段统一走既有原语）
使用说明：`task/task_hit_ball/README_hitball.md`（模块划分/状态机/参数速查/上车清单）
对应工程：GrandRDKv2.5（板端 `/userdata/GrandRDK` 镜像）
参考工程：`D:\RC\S100\综合\GrandRDKv2.2`（viskf 真机经验，见 §3）

---

## 0. 落地须知（给维护者：读这一节即可上手）

**测试红线：用户未明确下令前，不写任何测试（冒烟/单元/自检都不写）。**

### 0.1 版本变更记录（全部已拍板）

| 版本 | 变更 | 依据 |
|---|---|---|
| v3（对齐 v2.1） | ①删 SearchBall 章（t_search_ball 承担）②键改名 AUV_HIT_* ③机构 vservo.KF1D/PID ④ram=forward_step 显式 surge ⑤yaw 递推公式定稿 `yaw_ref=wrap(yaw_ref+AUV_HIT_YAW_SIGN·PID)`，**dx>0→右转**（右为正）默认 +1 | vservo v1.1 §4.1 + 用户 11:07 |
| **v3.1** | ⑥**轮次重试机制**：失败后先**后退 ≤2s 找球**（球重现即止），重现后重新跟随冲撞，共 `AUV_HIT_ROUNDS=3` 轮；⑦**撞后确认**：ram 后悬停观察 0.8s，球仍在画面判未撞上（★必要件：没有它 ram 恒判成功、重试永不触发）；⑧轮尽处置默认 **'ascend' 直接上浮**（'skip' 降为调试档，原 'retry' 档被内置轮次循环取代） | 用户 11:29 提议 + 17:32「落地吧」 |

### 0.2 模块化分工（v3.1 实际形态）

| 模块 | 职责 | 约束 |
|---|---|---|
| `task/vservo.py` | 公共零件：KF1D/KF2D/PID/yaw_servo_step/Consec/loss_tier/wrap_deg | **零 task_config 依赖**，参数全部构造注入；不含运动原语 |
| `task/task_hit_ball/t_hit_ball.py` | HitBallAll 五相位状态机（track/ram/confirm/back/ascend） | 只写状态机；每轮重建 KF/PID（测试模式循环重跑天然安全） |
| `task_config.py` | AUV_HIT_* 30 键（唯一参数来源） | getattr 兜底；HIT_BALL_TABLE try-import 挂 `task.task_hit_ball` |
| `t_function.py` | forward_step（ram 盲冲/back 后退，显式 surge+touch_wall）、hover_step（confirm/ascend）、定深链路 | 不动 |
| `obs.py` / `mission.py` / `front.py` / `t_search_ball.py` | VisionIF.poll('front','ball') / Stage 契约 / 检测写端 / 搜索段 | 不动 |

数据链路：`front.py → momo_det_front.json(~10Hz) → VisionIF.poll → KF1D(cx) → trust 门禁 → PID(ex_kf) → yaw_servo_step 递推 yaw_ref → cmd → 0x09`

### 0.3 验收记录（2026-10-07）

- `py_compile` vservo.py / t_hit_ball.py / task_config.py 全过；
- 板端等价路径（cwd=to32 根、PYTHONPATH=../../config）导入实测：`STAGE_TABLE = Task1+SearchBall+HitBall+Task2+PassGate+Return`（6 段，HitBall 首次入正式链路）；
- ★顺带修 bug：task_config 预挂的 `from task import t_hit_ball/t_pick_ball` 路径缺子目录段，try-import 静默置空——已改为 `from task.task_hit_ball import t_hit_ball` / `from task.task_pick_ball import t_pick_ball`。

## 1. 目标与范围

实现 STAGE_TABLE 中的 `HitBall`（撞球段）：前视画面已见球的前提下，视觉 PID + 图像卡尔曼跟随球心（track），近场盲冲（ram），撞后确认，失败后退找球重试，轮尽上浮兜底。全程防露头、绝不死等。

- **只闭 yaw**：纵向偏差由定深消化。
- **序列位置**：`Task1 → SearchBall（下视 zigzag，用户自写）→ HitBall → Task2 → …`（实测 6 段）。
- **起步假设**：进入时前视可见球；不可见则丢失节拍 2s 内快速失败 → 后退重试/退出。SearchBall（下视交棒）与 HitBall（前视接力）的衔接是上车首测项。
- **规则红线**：撞球必须最先完成；挂碰/碰线不得分 → 近场判据连续 5 拍确认 + PID 无 I 积累防超调。

## 2. 状态机与轮次结构

```
第 1 轮: [TRACK 跟随] → [RAM 盲冲 1s] → [CONFIRM 撞后确认 0.8s]
            │失败（丢失≥2s / 单轮超时15s）      ├─ 球已消失/出画 → DONE 撞球完成
            v                                  └─ 球仍在画面(≥2帧) → 判未撞上 ↓
第 2 轮: [BACK 后退 ≤2s] → [TRACK(重建KF/PID)] → [RAM] → [CONFIRM]
            │球重现(连续≥2帧)提前结束；超窗未重现 = 本轮失败
            v
第 3 轮: [BACK] → [TRACK] → [RAM] → [CONFIRM]
            │失败
            v
         [ASCEND 上浮 6s] → 置 ball_lost → 完成进下一阶段
         （AUV_HIT_FAIL_ACTION='skip' 时跳过上浮直接置标志——调试档）
```

各相位要点：

| 相位 | 行为 | 完成/出口 |
|---|---|---|
| TRACK | KF1D 滤 cx；trust 门禁（age≤0.2s 且 σ≤上限）内 PID 更新 yaw_ref，否则锁住上拍；surge=0.35 | 近场 w_ema/W≥0.45 连续 5 拍 → RAM；丢失≥2s / 超时 15s → 本轮失败 |
| RAM | `forward_step(surge=0.9, target_height_cm=60, 1.0s)` 锁航向定深盲冲 | 定时必结束 → CONFIRM |
| CONFIRM | 悬停 0.8s 数球帧 | 窗内命中 <2 帧 → **DONE**；≥2 帧 → 本轮失败 |
| BACK | `forward_step(surge=-0.4, touch_wall=True, 2.0s)` 后退找球 | 球重现连续 ≥2 帧 → 下一轮 TRACK；超窗 → 本轮失败 |
| ASCEND | `hover_step` 到离面安全深度（clamp 下限）6s 纯定时 | 完成置 `ctx.ball_lost=True` → 阶段完成 |

yaw 递推公式（★已拍板）：`yaw_ref ← wrap(yaw_ref + AUV_HIT_YAW_SIGN·PID(ex_kf))`，dx>0（球在右）→ yaw_ref 增 → 右转；参数默认 +1，上车确认项。

## 3. 图像卡尔曼（vservo.KF1D）与 track 节拍

### 3.1 设计（继承 v2.2 真机教训）

- 1D 常速 CV KF，状态 `[cx, vcx]`，手写 2×2 不引 numpy；参数构造注入：
  R=AUV_HIT_KF_R_PX2=15²（TODO 回填）、Q=AUV_HIT_KF_Q_ACC（偏大=信观测）、
  新息门限 3σ+floor（floor≈√R）、连续 5 帧拒收 → 重置重捕、clip 贴边帧量测方差 ×4；
- w 通道 EMA 0.3 平滑仅供近场判据；
- **trust 门禁**：仅"新帧 age ≤ 0.2s 且 σ ≤ AUV_HIT_KF_SIGMA_MAX"的输出进 PID
  （v2.2 实测：门离场 6.68s 外推 e_x 被推到 +2.38——外推垃圾喂舵=满舵乱转）；
- 归一化口径恒定：`ex=(cx−W/2)/(W/2)`，全程除画面宽不切换。

### 3.2 track 节拍推导（2026-10-07 用户问，答录）

核心量：`age = now − last_hit_ts`（KF 最近一次成功吃帧时刻），每拍（20Hz）分档。
**三级节拍是粗节拍，trust 门禁是 coast 段内的细门禁**，两层叠加：

| age 区间 | 档位 | 行为 |
|---|---|---|
| 0 ~ 0.2s | coast · 新鲜 | KF 每拍 predict + 外推值仍够新鲜 → PID 继续更新 yaw_ref |
| 0.2 ~ 0.5s | coast · 过期 | KF 继续滚（保状态温热）但 trust 挡住外推值 → PID 停更，yaw_ref 锁住 |
| 0.5 ~ 2.0s | freeze | 明确冻结档：yaw_ref 保持上拍 + surge 继续 |
| ≥ 2.0s | lost | 本轮失败 → 后退重试 / 轮尽退出 |

任意时刻新帧到达 → age 归零回正常。门限推导：coast 0.5s ≈ 写端 10Hz 连丢 5 帧（CV 外推 5 帧内漂移可控）；trust 0.2s ≈ 2 个写帧周期（防外推喂舵）；lost 2.0s 区分瞬时丢与真丢。均为 v2.2 真机同参数。

## 4. 完成判据与兜底总表

| 段/情形 | 主判据 | 兜底/出口 |
|---|---|---|
| TRACK 跟随 | 近场 w_ema/W≥0.45 连续 5 拍 | 丢失节拍 + 单轮超时 15s → 本轮失败 |
| RAM 盲冲 | 定时 1.0s 必结束 | —— |
| CONFIRM 确认 | 定时 0.8s 必结束 | 命中 <2 帧 = 撞上 DONE；≥2 帧 = 本轮失败 |
| BACK 后退 | 球重现连续 2 帧提前结束 | 定时 2.0s 硬顶 + IMU 触壁提前结束；超窗 = 本轮失败 |
| 轮次 | —— | 3 轮用尽 → ASCEND 上浮（默认）/ skip 调试档 |
| 误检防御 | —— | 3σ+floor 拒收 + 连续 5 帧重置 + score≥0.5 前置 + trust 门禁 |
| 深度安全 | 每帧 0x09 带正 depth_cm | clamp_depth_cm 抬到离面 25cm 之上（防露头） |
| 框架层 | —— | Stage 异常 → Mission 立即停推收尾；ball_lost 下游秒退 |
| ★无死等面 | —— | 全部判据=定时/像素代理/标志位，不依赖 Dive/Turn 的无兜底判据 |

最坏时间预算：3×(后退2s+track15s+ram1s+确认0.8s) ≈ 55s + 上浮 6s ≈ **61s**（撞球必须最先完成、总时长 15min，可接受；嫌长可缩重试轮 track 超时）。

## 5. 配置键（task_config.py，AUV_HIT_* 30 键）

```python
AUV_HIT_CAM='front'  AUV_HIT_WANT='ball'  AUV_HIT_HEIGHT_CM=60.0
AUV_HIT_ROUNDS=3              # 总尝试轮数（含首轮）
AUV_HIT_TIMEOUT_S=15.0        # 单轮 track 超时
AUV_HIT_PID_KP=25.0  KI=0.0  KD=0.0  OUT_MAX=10.0  I_MAX=5.0
AUV_HIT_YAW_SIGN=1.0          # ★已拍板 dx>0→右转；上车确认项
AUV_HIT_TRACK_SURGE=0.35
AUV_HIT_RAM_W_RATIO=0.45  RAM_SEEN_N=5  RAM_SURGE=0.9  RAM_S=1.0
AUV_HIT_CONFIRM_S=0.8  CONFIRM_SEEN_N=2
AUV_HIT_BACK_S=2.0  BACK_SURGE=-0.4  BACK_SEEN_N=2   # BACK_SURGE 改 0 = Plan B 原地等球
AUV_HIT_FAIL_ACTION='ascend'  ASCEND_HOLD_S=6.0      # 'ascend'|'skip'
AUV_HIT_KF_R_PX2=225.0  Q_ACC=800.0  GATE_NSIGMA=3.0  RESET_N=5
AUV_HIT_KF_COAST_S=0.5  TRUST_AGE_S=0.2  SIGMA_MAX=60.0  LOST_S=2.0  W_EMA=0.3
```

## 6. 已知风险

| 风险 | 缓解 |
|---|---|
| 负 surge 倒退不干净（掉头/侧漂） | 上车首测项；不可用改 `AUV_HIT_BACK_SURGE=0` 退化为原地等球（Plan B，零代码改动） |
| 后退对"球横移出画"无效 | 自然边界：3 轮用尽上浮；后续真需要再加微摆扫视 |
| 撞后确认可能补刀（球漂得慢） | 多撞不扣分只费时间，接受 |
| 起步前视无球（下视→前视交棒口径差） | 丢失节拍 2s 快速失败 → 后退重试 → 轮尽上浮，绝不死等；衔接上车首测 |
| Kp=25/R/Q/σ 上限均先验值 | 全部集中 AUV_HIT_* 键，上车调参不改代码 |
| 模型无蓝球类别 | 控制链路先行；AUV_HIT_WANT + CANON 换名即接新模型 |
| 近场框宽抖动误触发 ram | w EMA + 连续 5 拍 + ram 定时 + 确认窗四层兜底 |

## 7. 决策演进记录

| 版本 | 决策 | 理由 |
|---|---|---|
| v1 | 拆 SearchBall+StrikeBall 两 Stage；front.py 不动走 VisionIF | 用户"找到球进入下一阶段"字面即 Stage 语义 |
| v2 | 摆动相对 yaw0；内嵌轻量 KF（不复活独立 viskf 进程） | 用户拍板修正；独立进程改造成本高 |
| v2.1 | 移入工程 task_hit_ball/ + 自包含落地须知 | 唯一方案源约定 |
| v3 | 删 SearchBall 章；键改名 AUV_HIT_*；机构 vservo.KF1D/PID；ram=forward_step；符号拍板 dx>0→右转；失败处置参数化 | vservo 统一规划 v1.1 + 用户 11:07 拍板 |
| **v3.1** | **轮次重试：失败后退 ≤2s 找球（球重现即止）→ 重跟重冲，共 3 轮；加撞后确认 0.8s（必要件）；轮尽默认直接上浮**；task_config 挂点路径修正（task.task_hit_ball） | 用户 11:29 提议 + 17:32 下令落地；导入实测发现挂点 bug 顺带修复 |
