# -*- coding: utf-8 -*-
"""t_function.py —— 基础运动原语函数库(定深 / 前进 / 转向)

定位(Task.md §2 的运动原语,供任务阶段与 Stage 子类直接调用)：
    dive_step()    下潜定深到「距池底 target_height_cm」
    forward_step() 定时直行(恒 surge + 定深 + 锁航向,可选触壁语义)
    sway_step()    定时横移(恒 sway + 定深 + 锁死航向)   [2026-10-06 新增]
    hover_step()   定时悬停(定深保持 + 锁死航向 + 零推力)  [2026-10-06 新增]
    turn_step()    定向旋转到绝对航向(可选同时调深 —— Turn 兼职变深)
    sway_align_step() 横移对准(锁死航向 + 按像素误差比例输出 sway,把目标摆到画面中心)
                     [2026-10-09 新增,撞球 v2 Step2 改版口径:对准不动 yaw,用横移]
    ★ yaw 锁死口径(2026-10-09 用户指令)：除显式指定转向的动作(turn_step /
      yaw 递推伺服)外,其余所有原语一律锁死 yaw —— _yaw_hold 首拍锁存进入动作时的
      航向,每拍固定下发(不随遥测刷新,底层持续纠偏)。

调用契约(与 mission.Stage 对齐)：
    cmd = t_function.dive_step(ctx, st, now, dt, target_height_cm=60)
    - ctx     mission.Ctx(用 ctx.tel / ctx.depth / ctx.say)
    - st      本阶段的持久状态 dict(Stage.__init__ 里建 {} 传入,函数自管键)
    - now/dt  Mission.step 透传的时间戳与拍间隔
    - 返回 cmd dict(mode_auv 据此组 0x09)；返回 None = 本原语完成

状态外置：函数本身无全局状态,计时器/稳定计数/锁存航向全存在调用者传入的
st dict 里(键见各函数 docstring)——同一函数可被多个阶段实例安全复用。

★ 必守的口径(Task.md §4.1 / §5,违者翻车)：
  1. 定深换算：depth_cm = 实测水深 − 目标高度(距底) − 机体高度(20cm)。
     本文件所有 target_height_cm 参数都是「距池底高度」,下发前经 target_depth_cm() 换算。
  2. 水面安全红线：换算结果再经 clamp_depth_cm() 抬到离面安全余量之上(防露头)。
  3. AUV 模式下 0x09 每帧都必须带正的 depth_cm(frame_motion 会把 0 原样下发,
     固件把它当"目标深度 0 = 水面")。原语在未显式给定深时**沿用上一拍的定深目标**
     (st['last_height_cm']),首拍无沿用值时用 AUV_DEFAULT_HEIGHT_CM。
  4. yaw 坐标系(★ 2026-10-08 板端实锤修正，勿再镜像两次)：
     **遥测 actual_yaw 的原始数值系 == 任务系**(与 ROV 模式 target_yaw_deg 锚定口径、
     任务阶段直接读 tel['actual_yaw'] 的口径一致)。cmd['yaw'] 给该系的绝对角；
     固件对 Yaw 取负，只在**下发侧**由 mode_auv.tick 的 apply_yaw_mirror 抵消**一次**。
     本文件判据侧(yaw_err_deg / _yaw_hold)一律**不做镜像**。
     两侧都镜像 = 闭环符号反转：实测 err 恒等于「目标 + 当前」(日志里 +123°)、
     ok_cnt 永远 0 → 任务永久卡在第一个转向子步骤(2026-10-08 to32_main.log 实锤：
     0x09 yaw 下发 -61.1°，遥测 actual_yaw +62.0°，镜像后 err 恒 123°，t=442s 仍 ok=0/10)。
  5. 推力坐标系(2026-10-08)：cmd['surge']/cmd['sway'] 给**任务系**值
     (正 surge = 前进/前冲, 正 sway = 右移)。固件方向由输出侧
     (mode_auv.tick / test_runner.tick) 按 AUV_SURGE_SIGN / AUV_SWAY_SIGN **各施加一次**
     —— 原语内不许再乘符号(否则双重取反)，加推力的任务(撞球/过门/捡球)也走同一处。
  6. 完成判据(2026-10-06 用户口径)：Dive/Turn **无超时兜底** —— 判据失效
     (融合深度缺失 / 无 yaw 遥测)即永不完成、持续下发；Forward/Sway/Hover
     定时即完成方式(touch_wall 模式可被 IMU 触壁判据提前结束)。

已知限制(TODO 上车前处理)：
  - 深度判据(★ 2026-10-11 改)：Dive / Exit 只看**固件深度计遥测 actual_depth_cm**，
    与下发的 depth_cm 是同一个固件帧 → 直接相减判带内；无遥测期间不判
    → **永不完成(无超时兜底)**。
    深度卡尔曼(depth_kalman → obs.DepthIF)已按用户要求**从板端移除**，
    融合深度/离底净空路径不再参与任何判定。
  - touch_wall 触壁判据(2026-10-06)：接 **IMU 三轴加速度**(遥测 acc_x/acc_y/acc_z,
    link_stm32 已还原)幅值突降判触壁；阈值(AUV_TOUCH_ACC_*)待实车标定，
    遥测无加速度时退化为纯定时。V2 无线速度,不接 vx 判据。
"""
try:                                                    # move_test 在 sys.path(mode_auv 注入)时直接平级 import
    import task_config as TC
    from mission import apply_yaw_mirror
except ImportError:                                     # 直接以 task/ 为工作目录运行时的兜底
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import task_config as TC
    from mission import apply_yaw_mirror

# ------------------------------------------------------------------ 常量
BODY_HEIGHT_CM = 20.0        # 机体高度(Task.md §4.1 固定常数；换算用)

_YAW_TOL_DEG = 3.0           # Turn 默认到位容差(°)
_YAW_HOLD_N = 10             # 到位保持拍数(20Hz 下 ≈0.5s)
_DEPTH_TOL_CM = 8         # Dive 默认定深带半宽(cm)。⚠ 融合 D 与深度计口径差 ~0.18m,混用前先统一
_DEPTH_HOLD_N = 15           # 定深带内保持拍数(20Hz 下 ≈0.75s)
_SURF_SAFE_CM = 25.0         # 离面安全余量(cm)：换算结果低于它一律抬到它 —— Task.md §5-1 防露头


# ------------------------------------------------------------------ 工具函数
def target_depth_cm(target_height_cm):
    """距池底高度(cm) → 固件定深目标 depth_cm(Task.md §4.1 公式)。

    depth_cm = 实测水深(AUV_POOL_DEPTH_CM) − 目标高度 − 机体高度(20cm)
    例：水深 130、目标高度 60 → depth_cm = 50。
    ★ 该目标是**固件深度计帧**（深度计装在机体上部，故减机体高度）；判据侧若用
    融合值（探头帧、比深度计低约一个机体高度）比差，必须换成 clearance 或
    「pool − 目标高度」——两帧直接相减恒差 ~20cm（见 dive_step）。
    ⚠ AUV_POOL_DEPTH_CM 必须是**实测水深**：2026-10-08 融合实测 H≈1.068m，
    而配置仍写 130cm ⇒ 定深目标整体偏深 ~23cm（离底 60cm 实际只到 ~37cm）。
    """
    pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 106))
    body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', BODY_HEIGHT_CM))
    return pool - float(target_height_cm) - body


def clamp_depth_cm(depth_cm):
    """定深目标安全钳位：下限 = 离面安全余量(防露头红线),上限 = 贴底。"""
    pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 106))
    body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', BODY_HEIGHT_CM))
    surf = float(getattr(TC, 'AUV_SURF_SAFE_CM', _SURF_SAFE_CM))
    lo = surf
    hi = max(lo, pool - body)
    return max(lo, min(hi, float(depth_cm)))


def yaw_err_deg(actual_yaw_tel, target_yaw_deg):
    """遥测航向(任务系)与目标角的带符号误差(°),wrap 到 [-180,180]。

    ★ 判据侧**不做任何镜像**(2026-10-08 修正)：actual_yaw 原始数值即任务系，
    target_yaw_deg 也是任务系 → 直接相减。固件取负由 mode_auv.tick 在**下发侧**
    镜像一次抵消；这里再镜像 = 双重镜像 = 符号反转，判据永不满足(见文件头口径 4)。
    """
    e = float(target_yaw_deg) - float(actual_yaw_tel)
    while e > 180.0:
        e -= 360.0
    while e < -180.0:
        e += 360.0
    return e


def _say_throttled(ctx, st, now, msg):
    """日志节流：同阶段内按 AUV_LOG_EVERY_S 限频,避免 20Hz 刷爆日志。"""
    gap = float(getattr(TC, 'AUV_LOG_EVERY_S', 1.0))
    if (now - st.get('_log_ts', 0.0)) >= gap:
        st['_log_ts'] = now
        ctx.say(msg)


def _tel_f(ctx, key, default=None):
    """从遥测 dict 安全取浮点字段；无遥测/字段缺失返回 default。"""
    tel = ctx.tel or {}
    try:
        v = tel.get(key)
        return float(v) if v is not None else default
    except (TypeError, ValueError):
        return default


def _depth_out(st, target_height_cm):
    """解析本拍定深目标(cm)：显式参数 > st 沿用 > 默认工作高度；写回 st 并换算钳位。

    ★ AUV 模式每帧 0x09 都必须有合理 depth_cm(发 0 = 目标水面),故"沿用上一拍"
    是安全默认,而不是发 0。
    """
    h = target_height_cm
    if h is None:
        h = st.get('last_height_cm')
    if h is None:
        h = float(getattr(TC, 'AUV_DEFAULT_HEIGHT_CM', 60.0))
    h = float(h)
    st['last_height_cm'] = h
    return clamp_depth_cm(target_depth_cm(h))


def _yaw_hold(st, ctx):
    """「锁死当前航向」(2026-10-09 用户口径)：首拍锁存进入时的实际航向，之后固定下发。

    非转向原语(dive/forward/sway/hover/exit)统一走这里 —— 除显式指定转向的动作
    (turn_step / yaw 递推伺服)外，其余动作全部**锁死 yaw**：首拍读遥测锁存进入时刻的
    航向角，后续每拍**固定下发该值**(不随遥测刷新) —— 底层按此目标持续纠偏，
    水流/扰动把船推偏了会自己转回来，而不是放任漂移。

    返回 yaw 目标(**任务系 = 遥测原始数值系，不镜像**；下发侧由 mode_auv 镜像一次)；
    无遥测时锁存不成功,返回上次锁定值；从未有过则 None
    (调用侧回落 0.0 并节流警告 —— 无遥测时航向锁死不可靠,运行期须保证 $TEL 正常)。
    """
    if 'yaw_ref' not in st:                  # ★ 锁死语义：只锁存一次(进入动作时的航向)
        y = _tel_f(ctx, 'actual_yaw')
        if y is not None:
            st['yaw_ref'] = float(y)
            st['yaw_ok'] = True
    return st.get('yaw_ref')


def _acc_mag(ctx):
    """遥测三轴加速度幅值(link_stm32 还原后的 acc_x/acc_y/acc_z)；缺字段返回 None。

    只比相对变化,单位 m/s² 或 g 均可 —— 触壁判据只看「幅值相对基线突降」。
    """
    tel = ctx.tel or {}
    ax = tel.get('acc_x')
    ay = tel.get('acc_y')
    az = tel.get('acc_z')
    if ax is None or ay is None or az is None:
        return None
    try:
        return (float(ax) ** 2 + float(ay) ** 2 + float(az) ** 2) ** 0.5
    except (TypeError, ValueError):
        return None


def _touch_wall_detect(st, ctx):
    """触壁检测(touch_wall 模式用)：IMU 加速度幅值相对基线突然下降 → 判触壁。

    基线 = 进入检测后前 AUV_TOUCH_ACC_BASE_N(缺省 10)拍的平均幅值,**锁定不再滑动**
    (防缓慢漂移把突变吸收掉)；之后当前幅值 < 基线×(1 − AUV_TOUCH_ACC_DROP_RATIO,
    缺省 0.5) 计一次命中,连续 AUV_TOUCH_ACC_HIT_N(缺省 3)拍命中 → 返回 True。
    无加速度遥测 → 恒 False(退化纯定时)。
    """
    a = _acc_mag(ctx)
    if a is None:
        return False
    base_n = int(getattr(TC, 'AUV_TOUCH_ACC_BASE_N', 10))
    ratio = float(getattr(TC, 'AUV_TOUCH_ACC_DROP_RATIO', 0.5))
    hit_n = int(getattr(TC, 'AUV_TOUCH_ACC_HIT_N', 3))
    buf = st.setdefault('touch_acc_buf', [])
    buf.append(a)
    if len(buf) > 30:
        buf.pop(0)
    if st.get('touch_base') is None:
        if len(buf) >= base_n:
            st['touch_base'] = sum(buf[-base_n:]) / base_n
        else:
            return False
    if a < st['touch_base'] * (1.0 - ratio):
        st['touch_hit'] = st.get('touch_hit', 0) + 1
    else:
        st['touch_hit'] = 0
    return st['touch_hit'] >= hit_n


def _cmd(stage, note, yaw, depth, surge=0.0, sway=0.0):
    """组装 mission 契约的 cmd dict(字段缺一不可,mode_auv 据此组 0x09)。"""
    return {'stage': stage, 'note': note,
            'yaw': float(yaw) if yaw is not None else 0.0,
            'depth': float(depth),
            'surge': max(-1.0, min(1.0, float(surge))),
            'sway': max(-1.0, min(1.0, float(sway))),
            'stop': 0}


# ------------------------------------------------------------------ 子步骤等待 / 相对转向锁存
def wait_cmd(stage='Wait', note='等遥测'):
    """「本拍不下发 0x09」的占位 cmd（mode_auv / test_runner 见 paused 立即 return）。

    用途：子步骤判据所依赖的遥测尚未到位时**等待** —— Mission 不会把本拍当成
    「阶段完成」(必须返回 None 才算完成)，链路也不会发出危险目标。
    """
    return {'stage': stage, 'note': note, 'paused': True}


def lock_turn_target(ctx, st, now, deg, key='turn_tgt', stage='Turn'):
    """相对转向目标角锁存（Turn 族共用）：当前 yaw + deg（**同系直接相加，不镜像**）。

    返回 float 目标角(任务系)；**未拿到 yaw 遥测 → 返回 None**，调用方必须回
    wait_cmd()（本拍不下发），等遥测到位后再锁存。★ 绝不允许以 0 兜底：
    锁 0 = 命令转到绝对航向 0°（错误且危险）；NaN 还会让 0x09 组帧
    int(round(nan*100)) 抛异常 → 阶段被 Mission 判为异常直接收尾。
    锁存后本子步骤内不再变化（一次切入只锁一次）。deg 语义：右为正（用户口径）。
    """
    if st.get(key) is None:
        y = _tel_f(ctx, 'actual_yaw')
        if y is None:
            _say_throttled(ctx, st, now,
                           '%s 切入时无 yaw 遥测 → 本拍不下发，等遥测（不以 0 兜底）' % stage)
            return None
        st[key] = float(y) + float(deg)
    return st[key]


# ------------------------------------------------------------------ 
# 原语 1：下潜定深
def dive_step(ctx, st, now, dt, target_height_cm,
              tol_cm=None, hold_n=None, stage='Dive'):
    """下潜定深到「距池底 target_height_cm」。

    行为(Task.md §4.1 Dive)：surge/sway=0,depth_cm 闭环(固件内),yaw 锁死当前航向
    (进入时首拍锁存,固定下发 —— _yaw_hold)。
    完成(★ 2026-10-11 口径)：**固件深度计遥测 actual_depth_cm** 连续 hold_n 拍落在
    [下发目标 depth_cm ± tol_cm] 内 —— 下发与判据是**同一个固件帧**，可直接相减。
    这是深度卡尔曼从板端移除后的**唯一**深度判据（融合 clearance / 融合 D 路径已删除）。
    （历史：2026-10-08 曾用融合 clearance 判，并踩过"融合探头帧 vs 固件深度计帧"恒定
      差 19.5cm 的跨帧混比坑；改用同一固件帧后该坑自然消失。）
    ★ 无兜底(2026-10-06 用户口径)：无 actual_depth_cm 遥测、或未入带期间**永不完成**，
      持续下发定深指令（要完成须先保证 $TEL 遥测正常）。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        target_height_cm:  float —— 目标**距池底高度**(cm,必填)。下发/钳位走
                           depth_cm = AUV_POOL_DEPTH_CM − 目标高度 − 机体高度(20cm)
                           （固件深度计装在机体上部,故减去机体高度）
        tol_cm:            float | None —— 定深带半宽(cm)。None = AUV_DEPTH_TOL_CM(缺省 5)
        hold_n:            int | None —— 带内保持拍数。None = AUV_DEPTH_HOLD_N(缺省 15,20Hz≈0.75s)
        stage:             str —— 阶段名(日志/展示用,默认 'Dive')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / ok_cnt / yaw_ref / yaw_ok / last_height_cm / _log_ts
    """
    st.setdefault('t0', now)
    st.setdefault('ok_cnt', 0)
    if tol_cm is None:
        tol_cm = float(getattr(TC, 'AUV_DEPTH_TOL_CM', _DEPTH_TOL_CM))
    if hold_n is None:
        hold_n = int(getattr(TC, 'AUV_DEPTH_HOLD_N', _DEPTH_HOLD_N))

    d_target = clamp_depth_cm(target_depth_cm(target_height_cm))
    elapsed = now - st['t0']

    # 完成判据(★ 2026-10-11)：固件深度计遥测 actual_depth_cm vs 下发目标 depth_cm
    #   —— 两者是同一个固件帧，可直接相减（深度卡尔曼已移除，融合路径不再参与）。
    a_d = _tel_f(ctx, 'actual_depth_cm')
    if a_d is not None:
        try:
            a_d = float(a_d)
        except (TypeError, ValueError):
            a_d = None
    if a_d is not None:
        in_band = abs(a_d - d_target) <= tol_cm
        _say_throttled(ctx, st, now,
                       '%s 深度=%.1fcm 目标=%.1fcm(距底%.0fcm) ok=%d/%d t=%.1fs'
                       % (stage, a_d, d_target, target_height_cm, st['ok_cnt'], hold_n, elapsed))
    else:
        in_band = False
        _say_throttled(ctx, st, now,
                       '%s 无 actual_depth_cm 遥测 → 不判带内(无兜底，持续下发定深)' % stage)
    st['ok_cnt'] = st['ok_cnt'] + 1 if in_band else 0
    done = (st['ok_cnt'] >= hold_n)
    why = ('带内保持 %d 拍(深度 %s/%.1fcm)'
           % (st['ok_cnt'], ('%.1f' % a_d) if a_d is not None else 'N/A', d_target))

    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if done:
        ctx.say('%s 完成：目标高度 %.0fcm(下发 depth=%.0fcm),%s'
                % (stage, target_height_cm, d_target, why))
        return None

    return _cmd(stage, '定深%.0f(距底%.0f)' % (d_target, target_height_cm),
                yaw=yaw_ref, depth=d_target)


# ------------------------------------------------------------------ 
# 原语 2：定时直行
def forward_step(ctx, st, now, dt, duration_s,
                 target_height_cm=None, surge=None, touch_wall=False, stage='Forward'):
    """定时直行 duration_s 秒：surge 恒定推力 + 定深 + 锁当前航向。

    行为(Task.md §4.1 Forward)：定深目标可用 target_height_cm 显式给,
    缺省沿用上一拍(st['last_height_cm'],首拍 AUV_DEFAULT_HEIGHT_CM)——
    AUV 每帧 0x09 必须带正 depth_cm,绝不允许发 0(= 目标水面,露头)。
    完成(2026-10-06 口径)：定时 —— elapsed >= duration_s 即完成；
    touch_wall(触壁模式)：**额外启用 IMU 加速度触壁判据** —— 遥测三轴加速度
    (acc_x/acc_y/acc_z)幅值相对基线(进入检测后前 AUV_TOUCH_ACC_BASE_N 拍均值)
    突然下降超过 AUV_TOUCH_ACC_DROP_RATIO,连续 AUV_TOUCH_ACC_HIT_N 拍命中 →
    判触壁提前结束(不等时长跑满)；无加速度遥测时退化为纯定时。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        duration_s:        float —— 直行时长(秒),唯一完成判据
        target_height_cm:  float | None —— 行进定深(距池底 cm)。None = 沿用上一拍
                           (st['last_height_cm'],首拍 AUV_DEFAULT_HEIGHT_CM)
        surge:             float | None —— 前后推力 [-1,1]。None = AUV_SURGE_THRUST(缺省 0.5)
        touch_wall:        bool —— 触壁模式(True 启用 IMU 加速度触壁判据：
                           幅值突降连续 AUV_TOUCH_ACC_HIT_N 拍即提前完成；
                           遥测无 acc_x/acc_y/acc_z 时退化为纯定时)
        stage:             str —— 阶段名(日志/展示用,默认 'Forward')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / yaw_ref / yaw_ok / last_height_cm / _log_ts
    """
    st.setdefault('t0', now)
    if surge is None:
        surge = float(getattr(TC, 'AUV_SURGE_THRUST', 0.5))   # TODO: 巡航推力档上车实测
    duration_s = float(duration_s)
    elapsed = now - st['t0']

    d_target = _depth_out(st, target_height_cm)
    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if elapsed >= duration_s:
        ctx.say('%s 完成：直行 %.1fs 到时%s'
                % (stage, duration_s, '(触壁模式)' if touch_wall else ''))
        return None

    # 触壁兜底(touch_wall=True 时启用)：IMU 加速度幅值突然下降 → 判触壁提前结束
    if touch_wall and _touch_wall_detect(st, ctx):
        ctx.say('%s 触壁（IMU 加速度突降），提前结束' % stage)
        return None

    _say_throttled(ctx, st, now, '%s t=%.1f/%.1fs surge=%.2f depth=%.0fcm'
                   % (stage, elapsed, duration_s, surge, d_target))
    return _cmd(stage, '直行%.0fs%s' % (duration_s, '/触壁' if touch_wall else ''),
                yaw=yaw_ref, depth=d_target, surge=surge)


# ------------------------------------------------------------------ 
# 原语 3：定向旋转
def turn_step(ctx, st, now, dt, target_yaw_deg,
              target_height_cm=None, tol_deg=None, hold_n=None, stage='Turn'):
    """定向旋转到任务系绝对航向 target_yaw_deg(可选同时调深 —— Turn 兼职变深)。

    行为(Task.md §4.1 Turn)：yaw_deg 闭环在固件内(0x09 直接给目标绝对角),
    surge/sway=0；下发值 = 任务系目标(mode_auv 在下发侧镜像**一次**——判据侧不镜像)。
    完成(2026-10-08 修正口径)：**只看输入的目标航向** —— |actual_yaw − target|
    连续 hold_n 拍 < tol_deg 即完成(actual_yaw 原始数值即任务系)。
    ★ 无兜底(2026-10-06 用户口径)：遥测无 yaw → 无法判到位,**永不完成**,
    持续下发转向指令(不按时间接受当前航向)。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        target_yaw_deg:    float —— 目标航向(**任务系**绝对角,°)，用
                           t_function.lock_turn_target() 从遥测锁存当前 yaw + 转角得到
        target_height_cm:  float | None —— 可选调深(距池底 cm)。None = 沿用上一拍
        tol_deg:           float | None —— 到位容差(°)。None = AUV_YAW_TOL_DEG(缺省 3.0)
        hold_n:            int | None —— 到位保持拍数。None = AUV_YAW_HOLD_N(缺省 10,≈0.5s)
        stage:             str —— 阶段名(日志/展示用,默认 'Turn')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / ok_cnt / last_height_cm / _log_ts
    """
    st.setdefault('t0', now)
    st.setdefault('ok_cnt', 0)
    if tol_deg is None:
        tol_deg = float(getattr(TC, 'AUV_YAW_TOL_DEG', _YAW_TOL_DEG))
    if hold_n is None:
        hold_n = int(getattr(TC, 'AUV_YAW_HOLD_N', _YAW_HOLD_N))
    target_yaw_deg = float(target_yaw_deg)
    elapsed = now - st['t0']

    yaw_tel = _tel_f(ctx, 'actual_yaw')
    if yaw_tel is not None:
        err = yaw_err_deg(yaw_tel, target_yaw_deg)
        if abs(err) <= tol_deg:
            st['ok_cnt'] += 1
        else:
            st['ok_cnt'] = 0
        done = (st['ok_cnt'] >= hold_n)
        why = '到位保持 %d 拍(err=%.1f°)' % (st['ok_cnt'], err)
    else:
        # 无兜底(2026-10-06 用户口径)：无 yaw 遥测 → 无法判到位,永不完成
        done = False
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,无法判到位(无兜底,持续旋转)' % stage)

    d_target = _depth_out(st, target_height_cm)

    if done:
        ctx.say('%s 完成：目标航向 %.1f°(0x09 yaw=%.1f°),%s'
                % (stage, target_yaw_deg,
                   apply_yaw_mirror(target_yaw_deg, bool(getattr(TC, 'YAW_MIRROR', True))), why))
        return None

    _say_throttled(ctx, st, now, '%s err=%s ok=%d/%d t=%.1fs depth=%.0fcm'
                   % (stage, ('%.1f°' % yaw_err_deg(yaw_tel, target_yaw_deg)) if yaw_tel is not None else 'None',
                      st['ok_cnt'], hold_n, elapsed, d_target))
    return _cmd(stage, '转%.1f°%s' % (target_yaw_deg, '/调深' if target_height_cm is not None else ''),
                yaw=target_yaw_deg, depth=d_target)


# ------------------------------------------------------------------ 
# 原语 4：定时横移（2026-10-06 新增，与 forward_step 对称）
def sway_step(ctx, st, now, dt, duration_s, direction=1.0,
              target_height_cm=None, sway=None, stage='Sway'):
    """定时横移 duration_s 秒：sway 恒定推力 + 定深 + 锁当前航向。

    行为(与 Forward 对称)：定深目标可用 target_height_cm 显式给,缺省沿用上一拍；
    yaw 锁死当前航向(与直行同款 _yaw_hold：首拍锁存,固定下发)；只发 sway 推力,surge=0。
    完成(2026-10-06 口径)：**只看输入的时间** —— elapsed >= duration_s 即完成。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        duration_s:        float —— 横移时长(秒),唯一完成判据
        direction:         float —— 横移方向：+1 = 右移,-1 = 左移（**任务系**）。
                           符号标定键 AUV_SWAY_SIGN 不在本函数乘，统一在输出侧
                           (mode_auv/test_runner) 随 surge 一起施加一次 —— 别两处都乘
        target_height_cm:  float | None —— 行进定深(距池底 cm)。None = 沿用上一拍
                           (st['last_height_cm'],首拍 AUV_DEFAULT_HEIGHT_CM)
        sway:              float | None —— 横移推力幅度 [-1,1]。None = AUV_SWAY_THRUST(缺省 0.5)
        stage:             str —— 阶段名(日志/展示用,默认 'Sway')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / yaw_ref / yaw_ok / last_height_cm / _log_ts
    """
    st.setdefault('t0', now)
    if sway is None:
        sway = float(getattr(TC, 'AUV_SWAY_THRUST', 0.5))   # TODO: 横移推力档上车实测
    sign = 1.0 if float(direction) >= 0 else -1.0           # 任务系方向（+1 = 右移）
    duration_s = float(duration_s)
    elapsed = now - st['t0']

    d_target = _depth_out(st, target_height_cm)
    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if elapsed >= duration_s:
        ctx.say('%s 完成：横移 %.1fs 到时(%s)'
                % (stage, duration_s, '右' if sign >= 0 else '左'))
        return None

    _say_throttled(ctx, st, now, '%s t=%.1f/%.1fs sway=%.2f depth=%.0fcm'
                   % (stage, elapsed, duration_s, sign * float(sway), d_target))
    return _cmd(stage, '横移%.0fs%s' % (duration_s, '右' if sign >= 0 else '左'),
                yaw=yaw_ref, depth=d_target, sway=sign * float(sway))


# ------------------------------------------------------------------ 
# 原语 5：定时悬停（2026-10-06 新增）
def hover_step(ctx, st, now, dt, duration_s,
               target_height_cm=None, stage='Hover'):
    """定时悬停 duration_s 秒：定深保持 + 锁当前航向 + surge/sway=0。

    行为：depth_cm 闭环(固件内)、yaw 锁死当前航向(_yaw_hold：首拍锁存,固定下发)、
    零推力 —— 纯保持位姿等待,常用于「到位后稳定观察/交接」。定深目标可用
    target_height_cm 显式给,缺省沿用上一拍(st['last_height_cm'])。
    完成(2026-10-06 口径)：**只看输入的时间** —— elapsed >= duration_s 即完成。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        duration_s:        float —— 悬停时长(秒),唯一完成判据
        target_height_cm:  float | None —— 悬停保持的距池底高度(cm)。None = 沿用上一拍
                           (st['last_height_cm'],首拍 AUV_DEFAULT_HEIGHT_CM)
        stage:             str —— 阶段名(日志/展示用,默认 'Hover')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / yaw_ref / yaw_ok / last_height_cm / _log_ts
    """
    st.setdefault('t0', now)
    duration_s = float(duration_s)
    elapsed = now - st['t0']

    d_target = _depth_out(st, target_height_cm)
    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if elapsed >= duration_s:
        ctx.say('%s 完成：悬停 %.1fs 到时' % (stage, duration_s))
        return None

    _say_throttled(ctx, st, now, '%s t=%.1f/%.1fs depth=%.0fcm'
                   % (stage, elapsed, duration_s, d_target))
    return _cmd(stage, '悬停%.0fs' % duration_s, yaw=yaw_ref, depth=d_target)


# ------------------------------------------------------------------ 
# 原语 6：上浮退出（2026-10-09 新增：撞球超时兜底 / 异常收尾用）
def exit_step(ctx, st, now, dt, surf_margin_cm=None, tol_cm=None, hold_n=None, stage='Exit'):
    """上浮退出：停止运动(surge/sway=0) + 自动上浮至水面安全区 + 深度到位判完成。

    行为：下发的 depth_cm = 离水面 surf_margin_cm（默认 AUV_SURF_SAFE_CM=25，
    即防露头红线 —— 不贴水面，上浮到离水面 25cm 的安全位置即算到位）；
    yaw 锁死当前航向(_yaw_hold：首拍锁存,固定下发)；surge/sway 恒 0（停止运动，不前进不横移）。
    完成(2026-10-09 口径)：**固件深度计遥测 actual_depth_cm** 连续 hold_n 拍
    ≤ (surf_margin_cm + tol_cm) 即完成 —— 与下发 depth_cm 同一固件口径(可直比)。
    无 actual_depth_cm 遥测 → 无法判到位，**永不完成**，持续下发上浮指令
    （安全语义：即使不判完成，船也在停推/上浮，不会失控；到位与否只影响任务状态机推进）。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        surf_margin_cm:    float | None —— 上浮目标 = 离水面距离(cm)。
                           None = AUV_SURF_SAFE_CM(缺省 25,防露头红线)
        tol_cm:            float | None —— 到位容差(cm)。None = AUV_EXIT_TOL_CM(缺省 10)
        hold_n:            int | None —— 带内保持拍数。None = AUV_EXIT_HOLD_N(缺省 15,≈0.75s)
        stage:             str —— 阶段名(日志/展示用,默认 'Exit')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：t0 / ok_cnt / yaw_ref / yaw_ok / _log_ts
    """
    st.setdefault('t0', now)
    st.setdefault('ok_cnt', 0)
    if surf_margin_cm is None:
        surf_margin_cm = float(getattr(TC, 'AUV_SURF_SAFE_CM', _SURF_SAFE_CM))
    if tol_cm is None:
        tol_cm = float(getattr(TC, 'AUV_EXIT_TOL_CM', 10.0))
    if hold_n is None:
        hold_n = int(getattr(TC, 'AUV_EXIT_HOLD_N', 15))

    d_target = clamp_depth_cm(float(surf_margin_cm))   # 上浮目标固件深度 = 离水面余量（钳位防越界）
    elapsed = now - st['t0']

    # 完成判据：固件深度计遥测 actual_depth_cm（与下发 depth_cm 同口径）到水面安全带内
    a_d = _tel_f(ctx, 'actual_depth_cm')
    if a_d is not None:
        in_band = a_d <= (float(surf_margin_cm) + tol_cm)
        st['ok_cnt'] = st['ok_cnt'] + 1 if in_band else 0
        _say_throttled(ctx, st, now,
                       '%s 深度=%.1fcm 目标=%.0fcm ok=%d/%d t=%.1fs'
                       % (stage, a_d, surf_margin_cm, st['ok_cnt'], hold_n, elapsed))
    else:
        st['ok_cnt'] = 0
        _say_throttled(ctx, st, now, '%s 无 actual_depth_cm 遥测 → 不判到位(持续上浮)' % stage)
    done = (st['ok_cnt'] >= hold_n)

    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if done:
        ctx.say('%s 完成：已上浮至水面安全区(深度 %.0fcm)' % (stage, d_target))
        return None

    return _cmd(stage, '上浮至水面(深度目标 %.0fcm)' % d_target,
                yaw=yaw_ref, depth=d_target)


# ------------------------------------------------------------------
# 原语 7：横移对准（sway 伺服，2026-10-09 用户口径：对准不动 yaw，用横移；可复用）
def sway_align_step(ctx, st, now, dt, ex_px, target_height_cm, yaw_ref,
                    sway_kp=None, sway_max=None, px_tol=None, hold_n=None, stage='SwayAlign'):
    """横移对准：保持航向(yaw 固定下发) + 按像素误差比例输出 sway，把目标摆到画面中心。

    行为(2026-10-09 撞球 v2 Step2 改版口径)：yaw 不动(由调用方起步锁存,本原语每拍
    固定下发 yaw_ref)；sway = clamp(sway_kp · ex_px / (0.5·画面宽), ±sway_max)：
        目标在画面右半(ex_px>0) → sway 正(右移,任务系右为正) → 目标向左回中心；
    比例控制：静止目标无稳态误差(ex→0 则 sway→0 停稳)，收敛后 |ex_px| ≤ px_tol
    连续 hold_n 拍 → None(到位,由调用方切下一动作)。
    ★ 无兜底(2026-10-06 用户口径)：ex_px=None(无有效误差/滤波不可信) → 不判到位,
    持续下发(保持上拍 sway)；丢球等上层处置由调用方负责(如撞球 Step2 丢球超时 → Exit)。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel / ctx.say)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        ex_px:             float | None —— 滤波后目标心距画面中心的像素误差(px,右正)；
                           None = 无有效误差(不判到位,保持上拍 sway 持续下发)
        target_height_cm:  float | None —— 对准期间定深(距池底 cm)。None = 沿用上一拍
        yaw_ref:           float | None —— 锁存航向(任务系,每拍固定下发不递推)。
                           None = 沿用上拍(yaw 保持)
        sway_kp:           float | None —— 比例增益(归一化 ex→sway)。None = AUV_SWAY_ALIGN_KP(缺省 1.0)
        sway_max:          float | None —— sway 输出限幅。None = AUV_SWAY_THRUST(缺省 0.5)
        px_tol:            float | None —— 到位容差(px)。None = AUV_SWAY_ALIGN_PX_TOL(缺省 20.0)
        hold_n:            int | None —— 到位保持拍数。None = AUV_SWAY_ALIGN_HOLD_N(缺省 20,≈1s)
        stage:             str —— 阶段名(日志/展示用,默认 'SwayAlign')

    返回：cmd dict(mode_auv 据此组 0x09)；None = 本原语完成

    st 键：ok_cnt / last_ex / last_sway / last_height_cm / _log_ts
    """
    st.setdefault('ok_cnt', 0)
    if sway_kp is None:
        sway_kp = float(getattr(TC, 'AUV_SWAY_ALIGN_KP', 1.0))
    if sway_max is None:
        sway_max = float(getattr(TC, 'AUV_SWAY_THRUST', 0.5))
    if px_tol is None:
        px_tol = float(getattr(TC, 'AUV_SWAY_ALIGN_PX_TOL', 20.0))
    if hold_n is None:
        hold_n = int(getattr(TC, 'AUV_SWAY_ALIGN_HOLD_N', 20))

    if ex_px is None:                                  # 无有效误差 → 不判到位,保持上拍 sway
        done = False
        sway_out = st.get('last_sway', 0.0)
        _say_throttled(ctx, st, now, '%s 无有效误差,不判到位(保持 sway=%.2f 持续下发)' % (stage, sway_out))
    else:                                              # 有效误差 → 比例输出 + 到位判据
        ex_px = float(ex_px)
        st['last_ex'] = ex_px
        w = float(getattr(TC, 'AUV_IMG_W', 640.0))   # 兜底=640（前摄画面口径，见 task_config 注释）
        sway_out = (sway_kp * ex_px / (0.5 * w)) if w > 0 else 0.0
        sway_out = max(-sway_max, min(sway_max, sway_out))   # 限幅 ±sway_max
        if abs(ex_px) <= px_tol:
            st['ok_cnt'] += 1
        else:
            st['ok_cnt'] = 0
        done = (st['ok_cnt'] >= hold_n)
    st['last_sway'] = sway_out

    d_target = _depth_out(st, target_height_cm)
    yaw_out = yaw_ref if yaw_ref is not None else _yaw_hold(st, ctx)

    if done:
        ctx.say('%s 完成：误差 %.0fpx 保持 %d 拍,sway=%.2f'
                % (stage, st['last_ex'], hold_n, sway_out))
        return None

    _say_throttled(ctx, st, now, '%s ex=%s ok=%d/%d sway=%.2f depth=%.0fcm'
                   % (stage, ('%.0fpx' % st['last_ex']) if 'last_ex' in st else 'None',
                      st['ok_cnt'], hold_n, sway_out, d_target))
    return _cmd(stage, '横移对准 ex=%.0fpx sway=%.2f' % (st.get('last_ex', 0.0), sway_out),
                yaw=yaw_out, depth=d_target, sway=sway_out)
