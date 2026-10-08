# README_hitball —— 撞球任务（HitBall）模块说明

状态：**已落地**（2026-10-07 v3.1） · 方案源：`strike_ball_plan.md`（本目录，唯一方案源）
一句话：前视已见球 → KF+PID 跟随球心（只闭 yaw）→ 近场盲冲 → 撞后确认；失败后退找球重试，共 3 轮，轮尽上浮兜底。

---

## 1. 在整个任务序列里的位置

```
Task1(下潜+直行) → SearchBall(下视zigzag找球) → 【HitBall 撞球】 → Task2(转向+直行) → PassGate(过门) → Return(回出发点)
```

STAGE_TABLE 实测 6 段（板端等价路径导入验证，2026-10-07）。HitBall 必须最先完成（规则红线：撞球前不得做过门/抓取）。

## 2. 模块划分（模块化编程，三层）

| 层 | 文件 | 职责 | 关键约束 |
|---|---|---|---|
| **公共零件层** | `task/vservo.py` | `KF1D`（1D CV 卡尔曼）/ `KF2D`（捡球备用糖）/ `PID` / `yaw_servo_step`（误差→PID→递推目标角）/ `Consec`（连续命中计数）/ `loss_tier`（丢失三级节拍）/ `wrap_deg` | **零 task_config 依赖**，参数全部构造注入；不含任何推力原语；捡球/穿门将来直接复用 |
| **任务状态机层** | `task/task_hit_ball/t_hit_ball.py` | `HitBallAll(Stage)`：五相位轮次循环（track / ram / confirm / back / ascend） | 只写状态机；滤波数学在 vservo、运动原语在 t_function；每轮重建 KF/PID 实例 |
| **参数层** | `task_config.py` | `AUV_HIT_*` 30 键（唯一参数来源） | getattr 兜底；调参不改代码；与其他任务键空间隔离 |

复用（不动）：`t_function.py`（forward_step=盲冲/后退、hover_step=确认/上浮、定深链路）、`obs.py`（VisionIF.poll）、`mission.py`（Stage 契约 + apply_yaw_mirror）、`front.py`（检测写端）、`t_search_ball.py`（搜索段）。

数据流：

```
front.py → /dev/shm/momo_det_front.json(~10Hz)
  → VisionIF.poll('front','ball')  [cx / w / clip / score]
  → vservo.KF1D 滤 cx              [20Hz：新帧 update / 丢帧 predict]
  → trust 门禁(age≤0.2s 且 σ≤上限) → vservo.PID(ex_kf)
  → vservo.yaw_servo_step 递推 yaw_ref（dx>0 → 右转）
  → cmd{yaw, depth, surge} → mode_auv → 0x09
```

## 3. 状态机（轮次循环）

```
第 1 轮: [TRACK 跟随] → [RAM 盲冲 1s] → [CONFIRM 撞后确认 0.8s]
            │失败（丢失≥2s / 单轮超时15s）      ├─ 球消失/出画 → DONE 撞球完成
            v                                  └─ 球仍在画面(≥2帧) → 判未撞上 ↓
第 2 轮: [BACK 后退 ≤2s] → [TRACK(KF/PID重建)] → [RAM] → [CONFIRM]
            │球重现(连续≥2帧)提前结束；超窗未重现 = 本轮失败
            v
第 3 轮: [BACK] → [TRACK] → [RAM] → [CONFIRM]
            │失败
            v
         [ASCEND 上浮 6s] → 置 ball_lost → 完成进下一阶段
```

| 相位 | 干什么 | 怎么算完成 |
|---|---|---|
| TRACK | KF 滤球心 cx → PID → 递推目标角，surge 0.35 跟进 | 近场（框宽占比≥0.45 连续 5 拍）→ 切 RAM；丢失≥2s 或超时 15s → 本轮失败 |
| RAM | `forward_step(surge=0.9, 1.0s)` 锁航向定深盲冲 | 定时必结束 → CONFIRM |
| CONFIRM | 悬停 0.8s 数球帧（★必要件，防"冲偏了也算成功"） | 球命中 <2 帧 = 撞上 DONE；≥2 帧 = 本轮失败 |
| BACK | `forward_step(surge=-0.4, 触壁检测, 2.0s)` 后退拉距离 | 球重现连续 2 帧 → 下一轮 TRACK；超窗 = 本轮失败 |
| ASCEND | 悬停上浮到离面安全深度，6s 纯定时 | 完成置 `ball_lost` → 进下一阶段（捡球会秒退） |

**track 丢失三级节拍**（age = 距最近有效帧的时间，KF 内部记）：

| age | 行为 |
|---|---|
| 0~0.2s | KF 外推仍新鲜 → PID 继续更新（外推喂舵） |
| 0.2~0.5s | KF 继续滚但 trust 门禁挡住外推 → PID 停更、yaw_ref 锁住 |
| 0.5~2.0s | 冻结：yaw_ref 保持上拍 + surge 继续 |
| ≥2.0s | 本轮失败 → BACK 重试 / 轮尽退出 |

推导：0.5s ≈ 检测写端 10Hz 连丢 5 帧；0.2s ≈ 2 个写帧周期；2.0s 区分瞬时丢/真丢（v2.2 真机同参数）。

## 4. 兜底速查（排障时先看这）

| 症状 | 机制 | 出口 |
|---|---|---|
| 进入后一直没球 | 丢失节拍 2s 判真丢 | 后退重试 → 3 轮（约 12s）→ 上浮 |
| 跟着跟着球没了 ≤0.5s | KF 外推滑行，方向由速度状态撑 | 自动恢复 |
| 球没了 0.5~2s | 冻结目标角继续前进 | 新帧回来自动接上 |
| 跟踪总不进近场 | 单轮 15s 超时 | 本轮失败 → 后退重试 |
| 盲冲后球还在画面 | 撞后确认判未撞上 | 后退重试 |
| 后退 2s 还没看到球 | 定时硬顶 | 本轮失败（额度-1） |
| 3 轮全失败 | FAIL_ACTION='ascend' | 上浮 6s → ball_lost → 进下一阶段 |
| 深度安全 | 每帧 depth 经 clamp | 离面 ≥25cm，绝不露头 |
| 阶段代码异常 | Mission 层捕获 | 立即停推收尾 |

最坏全程 ≈ 61s。日志节流 1s/条，看 `logs/` 里 stage 名 `HitBall`。

## 5. 参数速查（task_config.py，AUV_HIT_*）

| 分组 | 键 | 默认 | 说明 |
|---|---|---|---|
| 目标 | CAM / WANT / HEIGHT_CM | front / ball / 60 | 前视红球，工作高度距底 60cm |
| 轮次 | ROUNDS / TIMEOUT_S | 3 / 15 | 总机会数 / 单轮跟随超时 |
| PID | PID_KP / KI / KD / OUT_MAX / I_MAX | 25 / 0 / 0 / 10 / 5 | 外环，输出°/拍 |
| 符号 | YAW_SIGN | +1 | ★已拍板 dx>0→右转；**上车确认项** |
| 跟随 | TRACK_SURGE / W_EMA | 0.35 / 0.3 | 前进推力 / 框宽平滑 |
| 近场 | RAM_W_RATIO / RAM_SEEN_N | 0.45 / 5 | 框宽占比判据 |
| 盲冲 | RAM_SURGE / RAM_S | 0.9 / 1.0 | 冲撞推力/时长 |
| 确认 | CONFIRM_S / CONFIRM_SEEN_N | 0.8 / 2 | 撞后观察窗 |
| 后退 | BACK_S / BACK_SURGE / BACK_SEEN_N | 2.0 / -0.4 / 2 | ⚠ 负 surge 首测项；**改 0 = Plan B 原地等球** |
| 退出 | FAIL_ACTION / ASCEND_HOLD_S | ascend / 6.0 | 'ascend'/'skip'（调试档） |
| KF | KF_R_PX2 / Q_ACC / GATE_NSIGMA / RESET_N | 225 / 800 / 3.0 / 5 | R=15px²，TODO 回填 |
| 节拍 | KF_COAST_S / TRUST_AGE_S / KF_SIGMA_MAX / LOST_S | 0.5 / 0.2 / 60 / 2.0 | σ 上限 TODO 标定 |

## 6. 上车首测清单（按顺序）

1. **AUV_HIT_YAW_SIGN 确认**：手动摆球在画面右侧，看机头是否右转；反了改 -1
2. **负 surge 倒退**：BACK 段是否干净后退；不干净改 `AUV_HIT_BACK_SURGE=0`（Plan B 原地等球）
3. **Kp=25 手感**：跟随是否振荡/迟钝，调 `AUV_HIT_PID_KP`
4. **R/Q/SIGMA_MAX 回填**：录一段检测 JSON 算 cx 抖动方差回填 R；σ 上限照实测放宽/收紧
5. **RAM_W_RATIO=0.45 标定**：球在"该冲"的距离上量框宽像素，算占比改阈值
6. **SearchBall→HitBall 交棒衔接**：下视报"找到球"后前视是否真看得见（视场角差）
7. `AUV_POOL_DEPTH_CM` 按 实测水深（~120）更新，影响工作高度换算与上浮目标

## 7. 已知边界（接受项）

- 后退只拉距离不扫方位：球横移出画时重试大概率再失败 → 3 轮用尽上浮（要救再加微摆扫视）
- 撞后确认可能补刀（球漂得慢再撞一次）：多撞不扣分，只费时间
- 模型暂无蓝球：抽中蓝球需换模型 + `AUV_HIT_WANT` 换名（CANON 加一行）
- 起步前视无球会在 2s 内快速失败（设计如此，绝不死等）

## 8. 单独调试入口

```python
# test_mode/test_config.py 里把 TEST_TABLE 指向撞球单表（不影响正式 STAGE_TABLE）：
TEST_TABLE = HIT_BALL_TABLE
```

测试红线照旧：用户不下令，不写任何测试脚本。
