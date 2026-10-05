# -*- coding: utf-8 -*-
"""航位推算任务：DeadReckonTask（用高精度 IMU / 标称速度推算走了多远）

★ 为什么需要它（方案 §16.5.1）：
  赛事方的引导线（橙红 10×35cm 沉底）质量很差、很难用 —— 用户拍板**不用引导线**，
  改用高精度 IMU 做航位推算。所以"走到哪儿了"这件事必须靠推算，而不是看图。

⚠ 现状与诚实边界：
  * 主估 = **标称速度 × 时间**（AUV_SPEED_MPS，🔴 未标定前只验结构不能验距离）；
  * IMU（STM32 遥测应答，g=9.81）当前只用来做"是不是还在动"的健康检查 ——
    加速度二次积分在无姿态补偿下漂移极快（几十秒就能漂出几米），
    🔴 在 IMU 五步标定（方案 §16.10.1）完成前**不要**拿它当位移主源；
  * 标定完成后，把 AUV_DR_USE_IMU 打开即切换主源，本类结构与判据不用改。

健康检查的作用：推力给着但加速度长期≈0 → 大概率被池壁/球/水草顶住了，打 WARN 提示。
"""
from ..base import Task            # 任务基类
from ..servo import wrap180        # 角度归一


class DeadReckonTask(Task):
    """推算推进：沿当前（或指定）航向推进到目标距离"""

    name = 'DEAD_RECKON'

    def enter(self, ctx, now):
        self.dist = self.gpf(ctx, 'DIST_M', 2.0)                 # 目标距离 m
        self.surge = self.gpf(ctx, 'SURGE', 0.30)                # 推进推力
        self.s = 0.0                                             # 已推算距离 m
        self.stuck_s = 0.0                                       # 疑似被顶住的累计时长
        self.warned = False                                      # WARN 只打一次
        # 可选：进入时把航向设成绝对角（不写 = 沿用当前航向）
        y = self.gp(ctx, 'yaw_abs', None)
        if y is not None:
            ctx.yaw_deg = wrap180(float(y))
        # 可选：进入时把深度设到离底高度（不写 = 沿用当前深度）
        h = self.gp(ctx, 'height_cm', None)
        if h is not None:
            from ..servo import depth_of_height, clamp_depth
            ctx.depth_cm = clamp_depth(ctx, depth_of_height(ctx, h))
        ctx.log('[AUV] %s 推算推进 %.2fm（surge=%.2f, yaw*=%.0f°）'
                % (self.name, self.dist, self.surge, ctx.yaw_deg))

    def tick(self, ctx, now, dt):
        dt = max(0.0, float(dt))

        # ---- 距离推算：主源 = 标称速度 × 时间（🔴 AUV_SPEED_MPS 待标定）
        speed = ctx.gf('AUV_SPEED_MPS', 0.25)                    # 标称巡航速度 m/s
        k = ctx.gf('AUV_SPEED_K_SURGE', 1.0)                     # 推力→速度系数（🔴 待标定）
        v = speed * k * (abs(self.surge) / 0.3 if self.surge else 0.0)
        self.s += v * dt

        # ---- IMU 健康检查（不是位移主源，只报"可能被顶住了"）
        if ctx.g('AUV_DR_USE_IMU', False) or ctx.g('AUV_USE_IMU', False):
            acc = ctx.tel_acc()
            if acc is not None and abs(acc - ctx.gf('AUV_G', 9.81)) < ctx.gf('AUV_DR_STILL_G', 0.05):
                self.stuck_s += dt
                if self.stuck_s > ctx.gf('AUV_DR_STUCK_S', 2.0) and not self.warned:
                    self.warned = True
                    ctx.log('[AUV] ⚠ %s 推力已给但 IMU 无响应 %.1fs —— 可能被顶住'
                            % (self.name, self.stuck_s))
            else:
                self.stuck_s = 0.0

        if self.s >= self.dist:
            return self.done(ctx, now, '推算到达 %.2f/%.2fm' % (self.s, self.dist))

        r = self.timeout(ctx, now, 'TIMEOUT_S', 30.0, '推算推进')
        if r is not None:
            return r
        return ctx.out(surge=self.surge,
                       note='推算推进 %.2f/%.2fm' % (self.s, self.dist))
