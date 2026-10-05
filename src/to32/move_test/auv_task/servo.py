# -*- coding: utf-8 -*-
"""伺服律 —— 从旧 mission.py【逐行平移】出来的纯函数集

★★ 硬约束（改这个文件前先读三遍）★★
  1. 这里的每一条判据、每一个符号约定都在台架/水池上验过（有些是踩坑后才改对的），
     重构时**逻辑一行都不能改**，只是把 `self._gf(...)` 换成 `ctx.gf(...)`、
     把 `self.depth_cm / self.yaw_deg` 换成显式入参/返回值（便于离线单测）。
  2. 三条既有口径**不许动**：
       (a) e_x 归一化恒用 `(cx − x0 − dx0) / w_bbox`，任何时候都不要在裁剪时换分母；
       (b) viskf 判可用看 `trust`，**不看** `gate_visible`；
       (c) `AUV_GATE_CROSS_TOL` 必须 > 0（int8×127 量化分辨率 1/254≈0.004，
           越接近门推力越小，掉到 0.004 以下会被量化成 0 → 机器人卡死到超时）。
  3. 本模块**不 import 任何板端模块**（不碰串口/共享内存/配置模块），
     所以能在 Windows 上直接跑离线回归测试。

单位约定：长度 cm，角度 deg，时间 s，推力归一化 [-1,1]。
"""
import math  # 数学库: 正弦扫描与平方根


# ---------------------------------------------------------------- 角度 / 符号
def wrap180(deg):
    """把角度归一到 (-180, 180]；所有角度差都必须过这个函数，别手写 if"""
    d = float(deg)
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def sign(v):
    """取符号：>0 → +1.0，<0 → -1.0，==0 → 0.0

    不用 math.copysign 是为了让 0 明确返回 0（copysign(1.0, 0.0)=+1，
    会把"不需要推力"变成"往前推"）。
    """
    return 1.0 if v > 0 else (-1.0 if v < 0 else 0.0)


def apply_yaw_mirror(yaw_deg, mirror):
    """固件对 Yaw 取负归一化 → 下发前镜像取反抵消

    抽成纯函数是为了能离线测符号端到端（mode_auv 依赖串口库，台架跑不了）。
    """
    return -float(yaw_deg) if mirror else float(yaw_deg)


# ---------------------------------------------------------------- 深度换算
def depth_of_height(ctx, h_cm):
    """离底高度 → 目标深度（cm）

    赛事规则给的是"目标距池底多少 cm"，而固件只认"目标深度 cm"，两者必须换算：
        depth = 池深 − 离底高度
    ⚠ 池深 AUV_POOL_DEPTH_CM 是**现场实测值**（规则只给"约 1.3m"，实测前先量）。
    结果会被 AUV_MIN_DEPTH_CM 兜住，绝不下出比水面余量还浅的目标。
    """
    pool = ctx.gf('AUV_POOL_DEPTH_CM', ctx.gf('AUV_POOL_DEPTH_M', 1.3) * 100.0)
    mn = ctx.gf('AUV_MIN_DEPTH_CM', ctx.gf('AUV_DEPTH_MIN_CM', 10.0))
    d = pool - float(h_cm)
    return max(mn, d)


def clamp_depth(ctx, d_cm):
    """目标深度限幅：不浅于水面余量、不深于「池底 − 安全余量」

    伺服是直接改目标深度的，没有这道钳制，目标一直在画面下方就会把机器人压到池底。
    """
    lo = ctx.gf('AUV_DEPTH_MIN_CM', 10.0)
    hi = ctx.gf('AUV_POOL_DEPTH_CM', ctx.gf('AUV_POOL_DEPTH_M', 1.3) * 100.0) \
        - ctx.gf('AUV_DEPTH_BOTTOM_MARGIN_CM', 10.0)
    if hi < lo:
        hi = lo
    return max(lo, min(hi, float(d_cm)))


# ---------------------------------------------------------------- 推力
def thrust(ctx, x, limit):
    """推力限幅 + int8 量化死区补偿

    surge/sway 是 int8(×127)，|x| 太小时量化后直接归零 → 机器人原地不动。
    所以「需要推力但量太小」时抬到 AUV_MIN_THRUST，而不是让它归零。
    """
    lim = abs(float(limit))
    v = max(-lim, min(lim, float(x)))
    if v == 0.0:
        return 0.0
    mn = ctx.gf('AUV_MIN_THRUST', 0.10)
    if abs(v) < mn:
        v = sign(v) * mn
    return v


# ---------------------------------------------------------------- 视觉伺服
def servo_depth(ctx, obs, dt, depth_cm):
    """★ 纵向视觉伺服：用前视画面里目标的纵向偏差驱动【目标深度】

    heave 没有推力槽位，垂直运动只能改目标深度让固件闭环。
    目标在画面下方(dy>0) → 它比我低 → 下潜（深度增大）；反之亦然。

    ⚠ 贴上下边时 bbox 被裁、cy 不可信，直接不动 —— 交给尺度判据收尾
      （撞球最后一段球必然贴边，此时保持深度直冲即可）。
    """
    if obs is None or not ctx.g('AUV_VERT_SERVO', True):
        return depth_cm
    if obs.get('clip_t') or obs.get('clip_b'):
        return depth_cm
    if abs(float(obs.get('dy') or 0.0)) <= ctx.gf('AUV_VERT_DEADBAND_PX', 15.0):
        return depth_cm                                  # 死区内不动，防抖
    s = ctx.gf('AUV_DEPTH_SIGN', 1.0)
    rate = ctx.gf('AUV_KP_DEPTH_CMS', 40.0) * float(obs.get('ey') or 0.0)
    mx = ctx.gf('AUV_DEPTH_RATE_MAX_CMS', 30.0)
    rate = max(-mx, min(mx, rate))
    return clamp_depth(ctx, depth_cm + s * rate * max(0.0, float(dt)))


def servo_yaw(ctx, obs, dt, yaw_deg):
    """★ 姿态伺服：用横向偏差驱动【目标航向】，让机头正对目标

    ⚠ 贴左右边时 cx 不可信，不动 yaw。
    ⚠ yaw_deg 是绝对目标角；这里做的是增量修正，YawEst 会自动跟着积分过去。
    ★ 方向约定（用户 2026-09-30 定）：目标在画面右侧 → 右转。
      "右转"到底是 yaw 增大还是减小取决于 AUV_TURN_RIGHT_SIGN，所以这里乘上它 ——
      转向符号标定完，伺服自动跟着走，不会一个改了另一个反。
    """
    if obs is None or not ctx.g('AUV_YAW_SERVO', True):
        return yaw_deg
    if obs.get('clip_l') or obs.get('clip_r'):
        return yaw_deg
    if abs(float(obs.get('dx') or 0.0)) <= ctx.gf('AUV_YAW_DEADBAND_PX', 15.0):
        return yaw_deg
    s = ctx.gf('AUV_YAW_SIGN', 1.0) * ctx.gf('AUV_TURN_RIGHT_SIGN', 1.0)
    rate = ctx.gf('AUV_KP_YAW_DPS', 25.0) * float(obs.get('ex') or 0.0)
    mx = ctx.gf('AUV_YAW_RATE_MAX_DPS', 20.0)
    rate = max(-mx, min(mx, rate))
    return wrap180(yaw_deg + s * rate * max(0.0, float(dt)))


def servo_gate_yaw(ctx, vk, dt, yaw_deg):
    """★ 过门偏航伺服：用滤波后的横向偏差 e_x（**不是原始像素**）驱动目标航向

    与 servo_yaw 的差别：
      * 输入 e_x = (cx−x0−dx0)/门框宽 —— **无量纲且与距离无关**，远近用同一套增益；
      * 多一个 de_x（滤波器白送的速度）当 D 项，不再靠"上一拍差分"这种带噪的东西；
      * 贴边不再"直接不动"，由 viskf 自己放大 R 表达质量下降（软信息，不硬切口径）。

    ⚠ 方向约定与 servo_yaw 完全一致：AUV_TURN_RIGHT_SIGN 一改，这里自动跟着走。
    """
    if vk is None:
        return yaw_deg
    e_x = float(vk.get('e_x') or 0.0)
    de_x = float(vk.get('de_x') or 0.0)
    if abs(e_x) <= ctx.gf('AUV_GATE_YAW_DEADBAND', 0.03):
        return yaw_deg
    s = ctx.gf('AUV_YAW_SIGN', 1.0) * ctx.gf('AUV_TURN_RIGHT_SIGN', 1.0)
    rate = (ctx.gf('AUV_GATE_YAW_KP_DPS', 40.0) * e_x
            + ctx.gf('AUV_GATE_YAW_KD_DPS', 8.0) * de_x)
    mx = ctx.gf('AUV_GATE_YAW_RATE_MAX_DPS', 20.0)
    rate = max(-mx, min(mx, rate))
    return wrap180(yaw_deg + s * rate * max(0.0, float(dt)))


def servo_gate_surge(ctx, vk):
    """★ 过门前向推力：越接近门推力越小（surge ∝ 剩余距离），配 CROSS_TOL 收尾

    ⚠ **必须配容差**：越近推力越小，而 surge 是 int8×127（分辨率 ≈0.004），
      推力掉到 0.004 以下会被**直接量化成 0**，机器人停在 s_star 下方几个千分点处
      再也不动 → 硬阈值 s_n ≥ s_star 永远等不到，干等到超时。
    """
    if vk is None:
        return ctx.gf('AUV_SURGE_GATE', 0.30)
    s_n = float(vk.get('s_n') or 0.0)
    s_star = ctx.gf('AUV_GATE_S_STAR', 0.85)
    mn = ctx.gf('AUV_GATE_SURGE_MIN', 0.25)
    if s_n >= s_star * (1.0 - ctx.gf('AUV_GATE_CROSS_TOL', 0.05)):
        return mn                                        # 已到位：保持最小推力直冲过去
    u = ctx.gf('AUV_GATE_SURGE_K', 1.2) * (s_star - s_n)
    # 下限用 GATE_SURGE_MIN 而不是 AUV_MIN_THRUST：前向要对抗水阻，
    # 0.10 这种"刚好不被量化成 0"的推力在真机上等于没推。
    return max(mn, min(ctx.gf('AUV_GATE_SURGE_MAX', 0.85), u))


# ---------------------------------------------------------------- 搜索期扫深
def seek_scan(ctx, now, t0, base_cm):
    """★ 搜索期的深度扫描：目标高度未知，在当前深度附近缓慢上下扫

    正弦扫，基准是「进入本搜索阶段时的深度」（不是写死 1m —— 撞完球后深度就在球的高度，
    从那儿接着扫才合理）。一旦看到目标就转进下一阶段，深度停住，交给 servo_depth 接管。
    """
    if not ctx.g('AUV_SEEK_SCAN', True):
        return base_cm
    rng = ctx.gf('AUV_SCAN_RANGE_CM', 30.0)
    per = ctx.gf('AUV_SCAN_PERIOD_S', 12.0)
    if rng <= 0 or per <= 0:
        return base_cm
    ph = ((float(now) - float(t0)) % per) / per
    return clamp_depth(ctx, float(base_cm) + rng * math.sin(2.0 * math.pi * ph))


# ---------------------------------------------------------------- 遥测取值
def tel_val(tel, keys):
    """按候选字段名列表依次取值，第一个能转 float 的就返回；全都没有返回 None

    遥测字段名在不同固件版本里不一致（actual_depth_cm / depth_cm / depth / depth_m ...），
    宽容取值是刻意的：宁可"取不到降级"，也不要因为改名整个任务崩掉。
    """
    if not isinstance(tel, dict):
        return None
    for k in keys:
        if k in tel and tel[k] is not None:
            try:
                return float(tel[k])
            except (TypeError, ValueError):
                continue
    return None


def tel_depth_cm(tel):
    """从遥测取深度（cm）"""
    return tel_val(tel, ('actual_depth_cm', 'depth_cm', 'depth', 'depth_m'))


def tel_yaw(tel):
    """从遥测取航向（deg）；取不到返回 None（此时转向只能开环）"""
    return tel_val(tel, ('actual_yaw', 'yaw_deg', 'yaw', 'yaw_now'))


def tel_acc(tel):
    """加速度模长（g）；取不到返回 None（遥测不通时 IMU 判据自动失效）

    优先用固件直接给的 acc_mag；没有再用 acc_x/acc_y/acc_z 合成。
    g 取 9.81（用户 2026-10-04 定：IMU 来自 STM32 遥测应答，g 恒 9.81）。
    """
    if not isinstance(tel, dict):
        return None
    mag = tel_val(tel, ('acc_mag', 'accel_mag', 'acc'))
    if mag is not None:
        return abs(mag)
    ax = tel_val(tel, ('acc_x', 'ax'))
    ay = tel_val(tel, ('acc_y', 'ay'))
    az = tel_val(tel, ('acc_z', 'az'))
    if None in (ax, ay, az):
        return None
    return math.sqrt(ax * ax + ay * ay + az * az)
