# -*- coding: utf-8 -*-
"""t_function.py —— 基础运动原语函数库(定深 / 前进 / 转向)

定位(Task.md §2 的运动原语,供任务阶段与 Stage 子类直接调用)：
    dive_step()    下潜定深到「距池底 target_height_cm」
    forward_step() 定时直行(恒 surge + 定深 + 锁航向,可选触壁语义)
    sway_step()    定时横移(恒 sway + 定深 + 锁航向)   [2026-10-06 新增]
    hover_step()   定时悬停(定深保持 + 锁航向 + 零推力)  [2026-10-06 新增]
    turn_step()    定向旋转到绝对航向(可选同时调深 —— Turn 兼职变深)

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
  4. yaw 坐标系：cmd['yaw'] 一律给**任务系**绝对角；固件系镜像由 mode_auv.tick
     的 apply_yaw_mirror 统一处理(本文件读回遥测时也用同一函数镜像回来,自逆)。
  5. 完成判据(2026-10-06 用户口径)：Dive/Turn **无超时兜底** —— 判据失效
     (融合深度缺失 / 无 yaw 遥测)即永不完成、持续下发；Forward/Sway/Hover
     定时即完成方式(touch_wall 模式可被 IMU 触壁判据提前结束)。

已知限制(TODO 上车前处理)：
  - 深度判据(2026-10-06 用户口径)：Dive 的完成/退出**只看 kalman 融合深度**
    (obs.DepthIF ← /dev/shm/momo_depth.json),不与固件深度计混判；融合源 !ok
    (depth_kalman 未起/超期/σ 超限)期间不判带内 → **永不完成(无超时兜底)**。
    注意口径：融合 D=B 探头语义,与深度计差 ~0.18m(config/depth_config.py H_M
    段"已知债务"),定深目标换算须与它配套。
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
    """
    pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 130))
    body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', BODY_HEIGHT_CM))
    return pool - float(target_height_cm) - body


def clamp_depth_cm(depth_cm):
    """定深目标安全钳位：下限 = 离面安全余量(防露头红线),上限 = 贴底。"""
    pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 130))
    body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', BODY_HEIGHT_CM))
    surf = float(getattr(TC, 'AUV_SURF_SAFE_CM', _SURF_SAFE_CM))
    lo = surf
    hi = max(lo, pool - body)
    return max(lo, min(hi, float(depth_cm)))


def yaw_err_deg(actual_yaw_tel, target_yaw_deg):
    """遥测航向(固件系)与任务系目标角的带符号误差(°),wrap 到 [-180,180]。

    遥测的 actual_yaw 是固件坐标系(固件对 Yaw 取负),用 apply_yaw_mirror
    镜像回任务系再比差(mirror 自逆,下发侧 mode_auv 已统一处理)。
    """
    mirror = bool(getattr(TC, 'YAW_MIRROR', True))
    a = apply_yaw_mirror(actual_yaw_tel, mirror)
    e = float(target_yaw_deg) - a
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
    """「保持当前航向」：每拍用遥测刷新任务系锁定角(跟随式,抗漂移)。

    返回任务系 yaw 目标；无遥测时返回上次锁定值,从未有过则 None
    (调用侧回落 0.0 并节流警告 —— 无遥测时航向保持不可靠,运行期须保证 $TEL 正常)。
    """
    y = _tel_f(ctx, 'actual_yaw')
    if y is not None:
        st['yaw_ref'] = apply_yaw_mirror(y, bool(getattr(TC, 'YAW_MIRROR', True)))
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


# ------------------------------------------------------------------ 
# 原语 1：下潜定深
def dive_step(ctx, st, now, dt, target_height_cm,
              tol_cm=None, hold_n=None, stage='Dive'):
    """下潜定深到「距池底 target_height_cm」。

    行为(Task.md §4.1 Dive)：surge/sway=0,depth_cm 闭环(固件内),yaw 保持当前航向。
    完成(2026-10-06 口径)：**kalman 融合深度**(ctx.depth → DepthIF)连续 hold_n 拍
    落在 [目标±tol_cm] 带内 —— 融合值是唯一判据,不与固件深度计混判。
    ★ 无兜底(2026-10-06 用户口径)：融合源 !ok(depth_kalman 未起/超期/σ 超限)
    或未入带期间**永不完成**,持续下发定深指令(要完成须先拉起 depth_kalman)。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        target_height_cm:  float —— 目标**距池底高度**(cm,必填)。下发前换算
                           depth_cm = 实测水深 − 目标高度 − 机体高度(20cm),
                           再经 clamp_depth_cm 钳位
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

    # 完成判据：只看 kalman 融合深度(DepthIF)落带才计数；
    # 融合源 !ok(depth_kalman 未起/超期/σ 超限)不判带内 → 永不完成(无兜底)。
    depth_if = getattr(ctx, 'depth', None)
    d_now = None
    if depth_if is not None:
        d = depth_if.read(now)                 # DepthIF 永不抛；!ok 时 D 为 None
        if d.get('ok') and d.get('D') is not None:
            try:
                d_now = float(d['D']) * 100.0  # 融合深度 m → cm
            except (TypeError, ValueError):
                d_now = None
    if d_now is not None and abs(d_now - d_target) <= tol_cm:
        st['ok_cnt'] += 1
    else:
        st['ok_cnt'] = 0
    done = (st['ok_cnt'] >= hold_n)
    why = '融合深度带内保持 %d 拍' % st['ok_cnt']

    yaw_ref = _yaw_hold(st, ctx)
    if yaw_ref is None:
        _say_throttled(ctx, st, now, '%s 无 yaw 遥测,航向保持不可靠(下发 0)' % stage)

    if done:
        ctx.say('%s 完成：目标高度 %.0fcm(depth=%.0fcm),%s'
                % (stage, target_height_cm, d_target, why))
        return None

    _say_throttled(ctx, st, now, '%s d_now=%s 目标=%.0fcm ok=%d/%d t=%.1fs'
                   % (stage, ('%.1f' % d_now) if d_now is not None else 'None',
                      d_target, st['ok_cnt'], hold_n, elapsed))
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
    surge/sway=0；下发值 = 任务系目标(mode_auv 统一镜像到固件系)。
    完成(2026-10-06 口径)：**只看输入的目标航向** —— |mirror(actual_yaw) − target|
    连续 hold_n 拍 < tol_deg 即完成。
    ★ 无兜底(2026-10-06 用户口径)：遥测无 yaw → 无法判到位,**永不完成**,
    持续下发转向指令(不按时间接受当前航向)。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        target_yaw_deg:    float —— 目标航向(**任务系**绝对角,°)。固件系镜像由
                           mode_auv 统一处理(本文件读回遥测时用 apply_yaw_mirror 镜像回来)
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
        ctx.say('%s 完成：目标航向 %.1f°(下发 %.1f°),%s'
                % (stage, target_yaw_deg, apply_yaw_mirror(target_yaw_deg, bool(getattr(TC, 'YAW_MIRROR', True))), why))
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
    yaw 跟随当前航向(与直行同款 _yaw_hold)；只发 sway 推力,surge=0。
    完成(2026-10-06 口径)：**只看输入的时间** —— elapsed >= duration_s 即完成。

    参数说明：
        ctx:               mission.Ctx —— 上下文(ctx.tel 遥测 / ctx.depth 融合深度 / ctx.say 日志)
        st:                dict —— 本阶段持久状态(函数自管键,见"st 键")
        now:               float —— 当前时间戳(Mission.step 透传)
        dt:                float —— 拍间隔秒数(Mission.step 透传)
        duration_s:        float —— 横移时长(秒),唯一完成判据
        direction:         float —— 横移方向：+1 = 右移(sway>0),-1 = 左移。
                           最终下发 = AUV_SWAY_SIGN × direction × 推力幅度
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
    sign = float(getattr(TC, 'AUV_SWAY_SIGN', 1.0)) * float(direction)
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

    行为：depth_cm 闭环(固件内)、yaw 跟随当前航向(_yaw_hold)、零推力 ——
    纯保持位姿等待,常用于「到位后稳定观察/交接」。定深目标可用
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
