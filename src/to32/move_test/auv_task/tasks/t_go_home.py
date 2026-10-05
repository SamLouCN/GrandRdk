# -*- coding: utf-8 -*-
"""回出发区任务：GoHomeTask（回到出发区并触碰池壁，30 分）

评分口径（2025 国赛规则）：**触碰出发区任意一侧池壁**即得分，不要求停准。
所以实现策略是"顶到墙"而不是"走到坐标点" —— 即便航位推算不准也能拿分。

流程：设定回家航向 → 推算推进到接近出发区 → 持续顶推直到触壁判据命中或超时。
⚠ 触壁判据现状：**只有超时**（无 IMU）。
  AUV_USE_IMU=True 且 AUV_WALL_IMPACT_ACC 标定后才会启用加速度判据（🔴 待标定）。
"""
from ..base import Task          # 任务基类
from ..servo import wrap180      # 角度归一


class GoHomeTask(Task):
    """回出发区：航向对准 → 顶推触壁"""

    name = 'GO_HOME'

    def enter(self, ctx, now):
        self.surge = self.gpf(ctx, 'SURGE', 0.35)
        # 航向：优先用绝对角 `yaw_abs`；否则用相对当前航向的转角 `turn_deg`（×方向符号）
        y = self.gp(ctx, 'yaw_abs', None)
        if y is not None:
            ctx.yaw_deg = wrap180(float(y))
        else:
            deg = self.gpf(ctx, 'turn_deg', 0.0) * ctx.gf('AUV_TURN_RIGHT_SIGN', 1.0)
            ctx.yaw_deg = wrap180(ctx.yaw_est.now() + deg)
        h = self.gp(ctx, 'height_cm', None)
        if h is not None:
            from ..servo import depth_of_height, clamp_depth
            ctx.depth_cm = clamp_depth(ctx, depth_of_height(ctx, h))
        self.s = 0.0
        ctx.log('[AUV] %s 回出发区（yaw*=%.0f°, D*=%.0fcm）'
                % (self.name, ctx.yaw_deg, ctx.depth_cm))

    def tick(self, ctx, now, dt):
        dt = max(0.0, float(dt))
        self.s += ctx.gf('AUV_SPEED_MPS', 0.25) * dt

        # ---- 触壁判据 ① IMU（遥测通 + 阈值标定后才启用）
        if ctx.g('AUV_USE_IMU', False):
            acc = ctx.tel_acc()
            thr = ctx.gf('AUV_WALL_IMPACT_ACC', 0.0)
            if acc is not None and thr > 0 and acc > thr:
                ctx.log('[AUV] 触壁(IMU) acc=%.2f' % acc)
                return self.done(ctx, now, '触壁(IMU)')

        # ---- 触壁判据 ② 超时（⚠ 无 IMU 时它就是实际判据）
        if ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 20.0):
            ctx.log('[AUV] 触壁(超时) —— 按已触壁处理（IMU 未启用/未标定时这是实际判据）')
            return self.done(ctx, now, '触壁(超时)')

        return ctx.out(surge=self.surge, sway=0.0,
                       note='回出发区 %.1fs / 推算 %.2fm' % (ctx.elapsed(now), self.s))
