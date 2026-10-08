# -*- coding: utf-8 -*-
"""t_hit_ball.py —— 撞球任务脚本（整体为一个阶段）：跟随球心 → 近场盲冲 → 撞后确认，失败轮次重试

定位（与 t_task1/t_search_ball 同构；方案源 task_hit_ball/strike_ball_plan.md v3.1）：
    - STAGE_TABLE 一项 = 整个撞球任务，可单独测试（TEST_TABLE 引 HIT_BALL_TABLE）
    - 模块化分工：滤波/PID/递推环用公共层 task/vservo.py（构造注入 AUV_HIT_* 键，
      参数独立）；盲段（后退/盲冲/悬停确认/上浮）复用 t_function 原语；
      本文件只写状态机，不含滤波数学、不含推力原语
    - 只闭 yaw：dy 由定深消化；yaw 递推公式
      yaw_ref ← wrap(yaw_ref + AUV_HIT_YAW_SIGN · PID(ex_kf))
      （★已拍板 2026-10-07：dx>0 → 右转，task 系 yaw 右为正 → 参数默认 +1）

轮次结构（2026-10-07 用户拍板）：
    第 1 轮: [TRACK 跟随] → [RAM 盲冲 1s] → [CONFIRM 撞后确认 0.8s]
    第 2+ 轮: [BACK 后退 ≤2s 或球重现≥2帧] → [TRACK(重建)] → [RAM] → [CONFIRM]
    共 AUV_HIT_ROUNDS(3) 轮；轮尽 → AUV_HIT_FAIL_ACTION 处置：
        'ascend'（默认）= 上浮 AUV_HIT_ASCEND_HOLD_S(6s) → 置 ball_lost → 完成
        'skip'          = 直接置 ball_lost → 完成
    ball_lost 置位后，下游需要球的 Stage（捡球）在 enter 检查秒退。

完成判据与兜底（绝不死等）：
    - TRACK：近场判据 w_ema/W ≥ 0.45 连续 5 拍 → RAM；丢失三级节拍
      （age≤0.5s KF 滑行+trust 门禁 / 0.5~2s 冻结 / ≥2s 判真丢）；
      单轮超时 AUV_HIT_TIMEOUT_S(15s)
    - RAM：定时 AUV_HIT_RAM_S(1s) 必结束（forward_step 锁航向定深盲冲）
    - CONFIRM：定时 AUV_HIT_CONFIRM_S(0.8s)；窗内球仍命中 ≥2 帧 → 判未撞上
      （★ 没有它 RAM 恒判成功，重试轮永远不会触发）
    - BACK：定时 AUV_HIT_BACK_S(2s) 硬顶 + IMU 触壁提前结束（touch_wall）；
      球重现连续 AUV_HIT_BACK_SEEN_N(2) 帧提前结束；倒退不可用改
      AUV_HIT_BACK_SURGE=0 退化为原地等球（Plan B）
    - 每帧 0x09 带正 depth_cm（_depth_out 沿用 + clamp_depth_cm 防露头）
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                          # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC
from mission import Stage, apply_yaw_mirror
import t_function
import vservo


class HitBallAll(Stage):
    """撞球任务（整体为一个阶段）：轮次循环 [TRACK→RAM→CONFIRM]，轮间 [BACK]，轮尽退出。

    五相位状态机（self.phase）：
        track    跟随：KF1D 滤 cx → trust 门禁 → PID → yaw_servo_step 递推目标角
        ram      盲冲：forward_step(surge=AUV_HIT_RAM_SURGE, 定时必结束)
        confirm  撞后确认：悬停观察窗内数球帧，球仍在 → 本轮失败
        back     后退找球：forward_step(负 surge, touch_wall)，球重现提前结束
        ascend   上浮兜底：hover 到离面安全深度，完成后置 ball_lost

    每轮进入 track 时重建 KF/PID/递推状态（构造注入本任务 AUV_HIT_* 键，
    vservo 零参数耦合）——测试模式循环重跑天然安全。
    """

    NAME = 'HitBall'

    # ------------------------------------------------------------ 生命周期
    def enter(self, now):
        self.round = 1                      # 当前轮（1 起，含首轮）
        self.phase = 'track'
        self.next_phase = None              # 相位完成后的去向；None = 整个阶段完成
        self.sts = {'track': {}, 'ram': {}, 'confirm': {}, 'back': {}, 'ascend': {}}
        self.confirm_hits = 0               # 撞后确认窗内球命中帧数
        self.back_reacquired = False        # 后退段是否等到球重现
        self.track_failed = False           # 本轮 track 是否失败（丢失/超时）
        self.round_t0 = None                # 本轮 track 起点（单轮超时判据）
        self.w_ema = None                   # 框宽 EMA（px）
        self.ram_gate = vservo.Consec(int(self._f('AUV_HIT_RAM_SEEN_N', 5)))
        self.back_gate = vservo.Consec(int(self._f('AUV_HIT_BACK_SEEN_N', 2)))
        self.servo = {'yaw_ref': 0.0}
        self._build_servo(now)              # 首轮滤波/PID/递推初值
        self.round_t0 = now
        self.ctx.say('%s 进入撞球任务（共 %d 轮机会，退出口径 %s）'
                     % (self.NAME, int(self._f('AUV_HIT_ROUNDS', 3)),
                        self._s('AUV_HIT_FAIL_ACTION', 'ascend')))

    def step(self, now, dt):
        """每拍：执行当前相位 → 相位完成则按 next_phase 级联切换（同拍可连跳）"""
        guard = 0
        while True:
            guard += 1
            if guard > 8:                   # 相位级联上限（防异常死循环）
                return None
            res = getattr(self, '_ph_' + self.phase)(now, dt)
            if res is not None:
                return res                  # 本拍下发这一帧
            if self.next_phase is None:
                return None                 # 整个阶段完成
            self.phase = self.next_phase
            self.next_phase = None
            self._enter_phase(now)

    # ------------------------------------------------------------ 相位切换
    def _enter_phase(self, now):
        if self.phase == 'track':
            self._build_servo(now)          # 每轮重建（新 KF/新 PID/新初值）
            self.round_t0 = now
            self.track_failed = False
            self.w_ema = None
            self.ram_gate.reset()
        elif self.phase == 'confirm':
            self.confirm_hits = 0
        elif self.phase == 'back':
            self.back_reacquired = False
            self.back_gate.reset()

    def _after_round_fail(self):
        """本轮失败 → 下一轮从 back 起步；轮尽按 AUV_HIT_FAIL_ACTION 处置"""
        if self.round < int(self._f('AUV_HIT_ROUNDS', 3)):
            self.round += 1
            return 'back'
        action = self._s('AUV_HIT_FAIL_ACTION', 'ascend').lower()
        if action == 'skip':
            setattr(self.ctx, 'ball_lost', True)
            self.ctx.say('%s %d 轮尝试全部失败（skip）→ 置 ball_lost → 进下一阶段'
                         % (self.NAME, int(self._f('AUV_HIT_ROUNDS', 3))))
            return None
        return 'ascend'

    # ------------------------------------------------------------ 相位 1：跟随
    def _ph_track(self, now, dt):
        st = self.sts['track']

        # ① 单轮超时兜底（从本轮 track 起点算，绝不死等）
        if (self.round_t0 is not None
                and (now - self.round_t0) >= self._f('AUV_HIT_TIMEOUT_S', 15.0)):
            self.ctx.say('%s 第 %d 轮 track 超时 %.0fs → 本轮失败'
                         % (self.NAME, self.round, self._f('AUV_HIT_TIMEOUT_S', 15.0)))
            self.track_failed = True
            self.next_phase = self._after_round_fail()
            return None

        # ② 观测：命中帧喂 KF（贴边帧方差×4），丢帧拍 predict 滚状态
        r = self._poll(now)
        if r is not None:
            clip = bool(r.get('clip_l') or r.get('clip_r'))
            self.kf.update(float(r['cx']), now, clip=clip)
            if not clip and r.get('w'):
                w = float(r['w'])
                a = self._f('AUV_HIT_W_EMA', 0.3)
                self.w_ema = w if self.w_ema is None else self.w_ema + a * (w - self.w_ema)
        else:
            self.kf.predict(now)

        # ③ 丢失三级节拍（粗）+ trust 门禁（细）：真丢 → 本轮失败
        tier = vservo.loss_tier(self.kf.age(now),
                                self._f('AUV_HIT_KF_COAST_S', 0.5),
                                self._f('AUV_HIT_LOST_S', 2.0))
        if tier == 'lost':
            self.ctx.say('%s 第 %d 轮球丢失 ≥%.1fs → 本轮失败'
                         % (self.NAME, self.round, self._f('AUV_HIT_LOST_S', 2.0)))
            self.track_failed = True
            self.next_phase = self._after_round_fail()
            return None

        # ④ trust 门禁内的外环 PID：ex_kf → 递推 yaw_ref（dx>0 → 右转）
        if self.kf.trust_ok(now, self._f('AUV_HIT_KF_SIGMA_MAX', 60.0)):
            half_w = self._f('AUV_IMG_W', 1280.0) / 2.0
            err = (self.kf.x[0] - half_w) / half_w
            yaw_ref, out = vservo.yaw_servo_step(self.servo, self.pid, err, dt,
                                                 sign=self._f('AUV_HIT_YAW_SIGN', 1.0))
        else:
            yaw_ref = self.servo['yaw_ref']  # 外推过期/σ 膨胀 → 锁住上拍目标

        # ⑤ 近场判据 → 切盲冲（w EMA 占比连续 N 拍）
        if self.w_ema is not None and (self.w_ema / self._f('AUV_IMG_W', 1280.0)
                                       ) >= self._f('AUV_HIT_RAM_W_RATIO', 0.45):
            if self.ram_gate.feed(True):
                self.ctx.say('%s 第 %d 轮近场(w_ema=%.0fpx) → 盲冲'
                             % (self.NAME, self.round, self.w_ema))
                self.next_phase = 'ram'
                return None
        else:
            self.ram_gate.feed(False)

        # ⑥ 下发：跟随推力 + 递推目标角 + 定深沿用
        t_function._say_throttled(self.ctx, st, now,
                                  '%s r%d 跟随 tier=%s w=%s'
                                  % (self.NAME, self.round, tier,
                                     ('%.0f' % self.w_ema) if self.w_ema is not None else 'None'))
        d = t_function._depth_out(st, self._f('AUV_HIT_HEIGHT_CM', 60.0))
        return t_function._cmd(self.NAME, '跟随r%d' % self.round,
                               yaw=yaw_ref, depth=d,
                               surge=self._f('AUV_HIT_TRACK_SURGE', 0.35))

    # ------------------------------------------------------------ 相位 2：盲冲
    def _ph_ram(self, now, dt):
        cmd = t_function.forward_step(self.ctx, self.sts['ram'], now, dt,
                                      duration_s=self._f('AUV_HIT_RAM_S', 1.0),
                                      surge=self._f('AUV_HIT_RAM_SURGE', 0.9),
                                      target_height_cm=self._f('AUV_HIT_HEIGHT_CM', 60.0),
                                      stage='Ram')
        if cmd is None:
            self.ctx.say('%s 第 %d 轮盲冲完成 → 撞后确认' % (self.NAME, self.round))
            self.next_phase = 'confirm'
        return cmd

    # ------------------------------------------------------------ 相位 3：撞后确认
    def _ph_confirm(self, now, dt):
        """悬停观察窗：球仍在画面(≥N 帧) → 判未撞上进下一轮；球消失 → 撞球完成。

        ★ 必要件：没有这一步 RAM 恒判成功，轮次重试永远不会触发。
        """
        r = self._poll(now)
        if r is not None:
            self.confirm_hits += 1
        cmd = t_function.hover_step(self.ctx, self.sts['confirm'], now, dt,
                                    duration_s=self._f('AUV_HIT_CONFIRM_S', 0.8),
                                    target_height_cm=self._f('AUV_HIT_HEIGHT_CM', 60.0),
                                    stage='Confirm')
        if cmd is None:                      # 观察窗结束
            seen_n = int(self._f('AUV_HIT_CONFIRM_SEEN_N', 2))
            if self.confirm_hits >= seen_n:
                self.ctx.say('%s 第 %d 轮确认：球仍在画面(%d帧) → 判未撞上'
                             % (self.NAME, self.round, self.confirm_hits))
                self.next_phase = self._after_round_fail()
            else:
                self.ctx.say('%s 撞球完成（第 %d 轮，窗内球命中 %d 帧）'
                             % (self.NAME, self.round, self.confirm_hits))
                self.next_phase = None       # 整个阶段完成
        return cmd

    # ------------------------------------------------------------ 相位 4：后退找球
    def _ph_back(self, now, dt):
        """后退 ≤2s（球重现≥N 帧提前结束；touch_wall 防倒车撞壁）。

        ⚠ 负 surge 倒退效果是上车首测项；不可用把 AUV_HIT_BACK_SURGE 改 0
        = 原地等球重现（Plan B，零代码改动）。
        """
        r = self._poll(now)
        if r is None:
            self.back_gate.feed(False)
        elif self.back_gate.feed(True):
            self.back_reacquired = True
            self.ctx.say('%s 第 %d 轮后退中球重现 → 重新跟随' % (self.NAME, self.round))
            self.next_phase = 'track'
            return None
        cmd = t_function.forward_step(self.ctx, self.sts['back'], now, dt,
                                      duration_s=self._f('AUV_HIT_BACK_S', 2.0),
                                      surge=self._f('AUV_HIT_BACK_SURGE', -0.4),
                                      target_height_cm=self._f('AUV_HIT_HEIGHT_CM', 60.0),
                                      touch_wall=True,
                                      stage='Back')
        if cmd is None:
            self.ctx.say('%s 第 %d 轮后退 %.1fs 未重现球 → 本轮失败'
                         % (self.NAME, self.round, self._f('AUV_HIT_BACK_S', 2.0)))
            self.next_phase = self._after_round_fail()
        return cmd

    # ------------------------------------------------------------ 相位 5：上浮兜底
    def _ph_ascend(self, now, dt):
        cmd = self._ascend_step(now, dt)
        if cmd is None:                      # 上浮保持完成
            setattr(self.ctx, 'ball_lost', True)
            self.ctx.say('%s %d 轮尝试全部失败 → 已上浮 %.0fs，置 ball_lost → 进下一阶段'
                         % (self.NAME, int(self._f('AUV_HIT_ROUNDS', 3)),
                            self._f('AUV_HIT_ASCEND_HOLD_S', 6.0)))
            self.next_phase = None
        return cmd

    def _ascend_step(self, now, dt):
        """上浮子段：depth = 离面安全余量对应值、零推力、锁航向、纯定时（复用 hover_step）。"""
        pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 130))
        body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', 20.0))
        surf = float(getattr(TC, 'AUV_SURF_SAFE_CM', 25.0))
        height = max(0.0, pool - body - surf)     # 距底高度 ↔ depth_cm = 离面安全值
        return t_function.hover_step(self.ctx, self.sts['ascend'], now, dt,
                                     duration_s=self._f('AUV_HIT_ASCEND_HOLD_S', 6.0),
                                     target_height_cm=height,
                                     stage='Ascend')

    # ------------------------------------------------------------ 工具
    def _build_servo(self, now):
        """每轮重建滤波/PID/递推状态（构造注入 AUV_HIT_* 键——三任务参数独立原则）。"""
        self.kf = vservo.KF1D(r=self._f('AUV_HIT_KF_R_PX2', 225.0),
                              q_acc=self._f('AUV_HIT_KF_Q_ACC', 800.0),
                              gate_nsigma=self._f('AUV_HIT_KF_GATE_NSIGMA', 3.0),
                              reset_n=int(self._f('AUV_HIT_KF_RESET_N', 5)),
                              trust_age_s=self._f('AUV_HIT_TRUST_AGE_S', 0.2))
        self.pid = vservo.PID(kp=self._f('AUV_HIT_PID_KP', 25.0),
                              ki=self._f('AUV_HIT_PID_KI', 0.0),
                              kd=self._f('AUV_HIT_PID_KD', 0.0),
                              out_max=self._f('AUV_HIT_PID_OUT_MAX', 10.0),
                              i_max=self._f('AUV_HIT_PID_I_MAX', 5.0))
        tel = self.ctx.tel or {}
        y = tel.get('actual_yaw')
        if y is not None:
            self.servo['yaw_ref'] = apply_yaw_mirror(float(y), bool(getattr(TC, 'YAW_MIRROR', True)))
        else:
            self.servo['yaw_ref'] = 0.0      # 无遥测按 0（v2.1 口径）；上车保证 $TEL 正常
            self.ctx.say('%s 无 yaw 遥测，递推初值按 0' % self.NAME)

    def _poll(self, now):
        """前视查球（AUV_HIT_CAM/AUV_HIT_WANT）；视觉缺失/异常一律按没看见"""
        v = getattr(self.ctx, 'vision', None)
        if v is None:
            return None
        try:
            return v.poll(self._s('AUV_HIT_CAM', 'front'),
                          self._s('AUV_HIT_WANT', 'ball'), now)
        except Exception:
            return None

    @staticmethod
    def _f(key, default):
        """task_config 浮点键读取（getattr 兜底，键缺失/类型坏都不炸）"""
        try:
            return float(getattr(TC, key, default))
        except (TypeError, ValueError):
            return float(default)

    @staticmethod
    def _s(key, default):
        """task_config 字符串键读取"""
        return str(getattr(TC, key, default))


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个撞球任务；空表 = 开机即 DONE）
HIT_BALL_TABLE = [HitBallAll]
