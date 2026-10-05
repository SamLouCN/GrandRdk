# -*- coding: utf-8 -*-
"""转向任务：TurnTask（原地转向 N 度）

★ 现状：转向是**开环定时**的（方案主线：无航向反馈时靠 AUV_YAW_RATE_DPS 积分）。
  有遥测（yaw 真值）时自动升级为真闭环 —— 判到位用的是遥测角，不再只靠时长。

🔴 AUV_YAW_RATE_DPS 必须水池标定（发满舵 → 计时 → 量角度），当前是占位值，
   未标定时"转 120°"实际可能是 100~140°。

⚠ 目标航向 `yaw_deg` 是**绝对角**不是角速度，且下发前会被 `apply_yaw_mirror` 取反，
   这里不要自己再取反（符号统一由 AUV_TURN_RIGHT_SIGN + YAW_MIRROR 管）。
"""
from ..base import Task                       # 任务基类
from ..servo import wrap180, tel_yaw as _tel_yaw  # 角度归一 / 遥测取航向


class TurnTask(Task):
    """原地转向：进入时设定目标航向，转到位（或超时）后进下一阶段"""

    name = 'TURN'

    def enter(self, ctx, now):
        """★ 目标航向 = 当前航向估计 + 角度 × 方向符号

        放在 enter（而不是每拍）是因为：每拍重算会把"正在转过去的量"冲掉，
        导致 yaw 目标被反复重置、永远转不到位。
        """
        deg = self.gpf(ctx, 'DEG', 120.0)
        s = ctx.gf('AUV_TURN_RIGHT_SIGN', 1.0)
        self.deg = deg * s
        ctx.yaw_deg = wrap180(ctx.yaw_est.now() + self.deg)
        ctx.log('[AUV] %s 转向 %+.0f° → 目标 yaw=%.0f°' % (self.name, self.deg, ctx.yaw_deg))

    def tick(self, ctx, now, dt):
        rate = ctx.gf('AUV_YAW_RATE_DPS', 30.0)                # 标称角速度（🔴 待标定）
        settle = self.gpf(ctx, 'SETTLE_S', 1.0)                # 转完的稳舵时间
        need = (abs(self.deg) / rate if rate > 0 else 0.0) + settle
        done = ctx.elapsed(now) >= need

        ty = _tel_yaw(ctx.tel)                                 # 有遥测 → 真闭环判到位
        if ty is not None and ctx.yaw_est.src == 'tel':
            tol = self.gpf(ctx, 'TOL_DEG', 3.0)
            done = done or abs(wrap180(ty - ctx.yaw_deg)) < tol

        if done or ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 10.0):
            ctx.goto(self.next_of(ctx))
            return ctx.out(surge=self.gpf(ctx, 'SURGE_SEEK', 0.25),
                           note='转向完成(开环需 %.1fs)' % need)
        return ctx.out(surge=0.0,
                       note='转向中 %.1f/%.1fs yaw*=%.0f°'
                            % (ctx.elapsed(now), need, ctx.yaw_deg))
