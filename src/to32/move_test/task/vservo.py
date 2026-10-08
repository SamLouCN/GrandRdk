# -*- coding: utf-8 -*-
"""vservo.py —— 视觉伺服公共层（KF1D/KF2D/PID/yaw_servo_step/丢失节拍工具）

定位（vservo_统一架构与落地规划_2026-10-07 v1.1 §1）：
    撞球/捡球/穿门三任务共用的"零件库"——只给零件，不给参数：
    - 本文件 **零 task_config 依赖**（不出现任何 getattr(TC, ...)），
      全部参数由任务文件在 enter()/重建时构造注入（AUV_<任务>_* 键随任务走）。
    - 运动原语不在这里：盲段统一复用 t_function 的 forward_step/hover_step
      （2026-10-07 09:23 用户拍板，本文件不含任何推力指令）。

零件清单：
    KF1D            1D 常速(CV)卡尔曼：滤像素坐标 cx（撞球/穿门）或 cy（捡球）
    KF2D            两个 KF1D 组合（捡球 cx,cy 专用糖）
    PID             通用 PID（kp/ki/kd + 输出限幅 + I 限幅，构造注入）
    yaw_servo_step  前视族共用「误差→PID→递推目标角」一步（撞/门同一语义）
    Consec          连续命中计数器（近场确认/球重现判定用）
    loss_tier       丢失三级节拍：按 age 分 coast / freeze / lost 档
    wrap_deg        角度 wrap 到 [-180, 180)

★ 设计红线（v2.2 viskf 真机教训，详见 task_hit_ball/strike_ball_plan.md §4）：
    1. trust 门禁：只有"新帧 age ≤ trust_age_s 且 σ ≤ sigma_max"的输出才准进
       PID——绝不拿长时间外推值控舵（外推垃圾喂舵 = 满舵乱转）。
    2. 新息门限 = nsigma·sqrt(S) + floor(sqrt(R))（无 floor 好观测会被误拒，
       v2.2 实测拒收从 20 涨到 35 拍）；连续拒收 reset_n 帧 → 重置重捕。
    3. 归一化口径由调用方保证恒定（全程除同一分母，不切换——v2.2 教训：
       口径切换造成 0.209 假跳变）。
"""
import math


def wrap_deg(a):
    """角度 wrap 到 [-180, 180)"""
    a = float(a) % 360.0
    if a >= 180.0:
        a -= 360.0
    return a


class KF1D(object):
    """1D 常速(CV)卡尔曼：状态 [x, vx]，量测 z = x。手写 2x2，不引 numpy。

    用法（20Hz 控制拍）：
        命中帧:  kf.update(z, now, clip=贴边)   → True = 接受（含重捕）
        丢帧拍:  kf.predict(now)                → 把状态滚到 now
        喂舵前:  kf.trust_ok(now, sigma_max)    → 只有 True 才准更新 PID

    st 无外置状态：滤波状态全在实例里；测试模式循环重跑须重建实例
    （任务文件在每轮进入伺服时重建，天然安全）。
    """

    def __init__(self, r, q_acc, gate_nsigma=3.0, reset_n=5, trust_age_s=0.2):
        self.r = float(r)                    # 量测方差 (px²)
        self.q_acc = float(q_acc)            # 过程噪声加速度谱密度
        self.gate_nsigma = float(gate_nsigma)
        self.reset_n = max(1, int(reset_n))  # 连续拒收 N 帧 → 重置重捕
        self.trust_age_s = float(trust_age_s)
        self.x = None                        # [x, vx]；None = 未初始化
        self.p = None                        # 协方差扁平 2x2 [p00,p01,p10,p11]
        self.t = None                        # 最近一次成功 update 的时刻
        self.rej = 0                         # 连续拒收计数

    def reset(self):
        """清空状态（重捕前调用）"""
        self.x = None
        self.p = None
        self.t = None
        self.rej = 0

    def _predict(self, dt):
        """状态/协方差向前滚 dt 秒（CV 模型 + 分段白加速度过程噪声）"""
        if dt <= 0.0:
            return
        x0, x1 = self.x
        p00, p01, p10, p11 = self.p
        q00 = self.q_acc * dt ** 4 / 4.0
        q01 = self.q_acc * dt ** 3 / 2.0
        q11 = self.q_acc * dt ** 2
        n00 = p00 + dt * (p10 + p01) + dt * dt * p11 + q00
        n01 = p01 + dt * p11 + q01
        n10 = p10 + dt * p11 + q01
        n11 = p11 + q11
        self.x = [x0 + x1 * dt, x1]
        self.p = [n00, n01, n10, n11]

    def predict(self, now):
        """把状态滚到 now（丢帧拍调用；未初始化时为空操作）"""
        if self.x is None or self.t is None:
            return
        self._predict(float(now) - self.t)
        self.t = float(now)

    def update(self, z, now, clip=False):
        """吃一帧量测。返回 True = 接受（含重捕初始化）；False = 被新息门限拒收。

        clip=True（检测框贴边）时本帧量测方差 ×4（v2.2 分边放大的简化版）。
        """
        z = float(z)
        r = self.r * (4.0 if clip else 1.0)
        if self.x is None:                   # 首帧 / 重捕：直接初始化
            self.x = [z, 0.0]
            self.p = [self.r, 0.0, 0.0, self.r]
            self.t = float(now)
            self.rej = 0
            return True
        self.predict(now)
        p00, p01, p10, p11 = self.p
        s = p00 + r                          # 新息方差
        y = z - self.x[0]                    # 新息
        if abs(y) > self.gate_nsigma * math.sqrt(s) + math.sqrt(self.r):
            self.rej += 1                    # floor = √R：防好观测被误拒（v2.2 教训）
            if self.rej >= self.reset_n:
                self.reset()
                return self.update(z, now, clip)   # 重置重捕：按本帧重新锁定
            return False
        k0 = p00 / s
        k1 = p10 / s
        self.x[0] += k0 * y
        self.x[1] += k1 * y
        self.p = [(1.0 - k0) * p00, (1.0 - k0) * p01,
                  p10 - k1 * p00, p11 - k1 * p01]
        self.t = float(now)
        self.rej = 0
        return True

    def age(self, now):
        """距最近一次成功 update 的秒数；从未 update 过返回 inf"""
        if self.t is None:
            return float('inf')
        return max(0.0, float(now) - self.t)

    @property
    def sigma(self):
        """x 估计标准差（px）；未初始化返回 inf"""
        if self.p is None:
            return float('inf')
        return math.sqrt(max(0.0, self.p[0]))

    def trust_ok(self, now, sigma_max):
        """trust 门禁：新帧 age ≤ trust_age_s 且 σ ≤ sigma_max 才准进 PID（★红线 1）"""
        return (self.x is not None
                and self.age(now) <= self.trust_age_s
                and self.sigma <= float(sigma_max))


class KF2D(object):
    """两个 KF1D 组合（捡球 cx,cy 专用糖）：同一节奏 update/predict/trust"""

    def __init__(self, r, q_acc, gate_nsigma=3.0, reset_n=5, trust_age_s=0.2):
        self.kx = KF1D(r, q_acc, gate_nsigma, reset_n, trust_age_s)
        self.ky = KF1D(r, q_acc, gate_nsigma, reset_n, trust_age_s)

    def update(self, zx, zy, now, clip=False):
        return (self.kx.update(zx, now, clip),
                self.ky.update(zy, now, clip))

    def predict(self, now):
        self.kx.predict(now)
        self.ky.predict(now)

    def trust_ok(self, now, sigma_max):
        return (self.kx.trust_ok(now, sigma_max)
                and self.ky.trust_ok(now, sigma_max))


class PID(object):
    """通用 PID：构造注入增益与限幅。I 积分钳位防饱和；kd=0/ki=0 时零开销。"""

    def __init__(self, kp, ki=0.0, kd=0.0, out_max=None, i_max=None):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_max = None if out_max is None else abs(float(out_max))
        self.i_max = None if i_max is None else abs(float(i_max))
        self.i = 0.0
        self.prev = None

    def reset(self):
        self.i = 0.0
        self.prev = None

    def step(self, err, dt):
        """一步 PID：返回输出（已按 out_max 限幅）"""
        err = float(err)
        dt = max(0.0, float(dt))
        self.i += self.ki * err * dt
        if self.i_max is not None:
            self.i = max(-self.i_max, min(self.i_max, self.i))
        d = 0.0
        if self.kd != 0.0 and self.prev is not None and dt > 0.0:
            d = self.kd * (err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.i + d
        if self.out_max is not None:
            out = max(-self.out_max, min(self.out_max, out))
        return out


def yaw_servo_step(st, pid, err, dt, sign=1.0):
    """前视族共用的一步航向伺服：err → PID → 递推目标角（撞/门同一语义）。

    st['yaw_ref'] 由调用方在进入伺服前初始化（= 交棒/起步时的当前实际航向，
    ★ 取遥测 actual_yaw 原始值，**不镜像** —— 原始值即任务系；固件取负由下发侧
    mode_auv 镜像一次抵消，这里再镜像 = 伺服符号反转，永远追不上目标）；
    本函数每拍递推：
        yaw_ref ← wrap(yaw_ref + sign · PID(err))
    返回 (yaw_ref, out)。

    sign 语义（★ 2026-10-07 用户拍板）：err>0（目标在画面右半）→ yaw_ref 增 =
    右转（task 系 yaw 右为正）→ sign 默认 +1；上车实测反了改 -1。
    """
    out = pid.step(err, dt)
    st['yaw_ref'] = wrap_deg(st.get('yaw_ref', 0.0) + float(sign) * out)
    return st['yaw_ref'], out


class Consec(object):
    """连续命中计数器：连续 n 次 feed(True) → True；一次 False 即清零。

    用于近场判据（w 占比连续 N 拍）、球重现判定（原始帧连续 N 帧）等。
    """

    def __init__(self, n):
        self.n = max(1, int(n))
        self.c = 0

    def feed(self, ok):
        self.c = self.c + 1 if ok else 0
        return self.c >= self.n

    def reset(self):
        self.c = 0


def loss_tier(age, coast_s, lost_s):
    """丢失三级节拍（粗节拍，对齐 v2.2 真机参数口径）：

        age ≤ coast_s          → 'coast'   KF 纯预测滑行（PID 是否更新另由
                                             KF1D.trust_ok 细门禁管）
        coast_s < age < lost_s → 'freeze'  冻结：yaw_ref 保持上拍 + surge 继续
        age ≥ lost_s           → 'lost'    真丢：触发调用方处置（重试/退出）

    推导（strike_ball_plan.md §4.2）：coast=0.5s ≈ 写端 10Hz 连丢 5 帧；
    lost=2.0s 区分瞬时丢与真丢。门限由调用方传参，本函数不读配置。
    """
    if age >= float(lost_s):
        return 'lost'
    if age > float(coast_s):
        return 'freeze'
    return 'coast'
