# -*- coding: utf-8 -*-
"""对中任务：CenterBallTask（把下视画面里的球对到收集框正上方）

★ 为什么必须先对中再坐底：收集框是**坐底兜球**（无机械手，方案 §16f 用户拍板），
  球必须在框的正上方，坐下去才兜得到。框只有 30×40cm，对不准 = 白坐一次。

★ 超时/目标丢失都**不中止**，直接进坐底：
  捡球是"尽力而为"，对不准也比原地耗到整局失败强（这条是旧版就定下的策略，别改）。
"""
from ..base import Task  # 任务基类


class CenterBallTask(Task):
    """下视对中：sway 修横向、surge 修纵向，进容差保持 hold_s 即完成"""

    name = 'CENTER_BALL'

    def enter(self, ctx, now):
        self.label = self.gp(ctx, 'label_key', 'ball')
        self.aim = self.gp(ctx, 'aim_key', None)
        self.cam = self.lit('cam', 'bottom')
        self.sit_d0 = None                                  # 传给坐底阶段的起始深度
        ctx.log('[AUV] %s 对中开始（容差 %.0fpx）'
                % (self.name, self.gpf(ctx, 'TOL_PX', 25.0)))

    def tick(self, ctx, now, dt):
        obs = ctx.see(self.cam, self.label, now, aim=self.aim)
        tol = self.gpf(ctx, 'TOL_PX', 25.0)

        if obs is None:
            ctx.clear_hold('centered')                      # 目标丢失 → 清计时，别"攒够就过"
            if ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 15.0):
                ctx.goto(self.next_of(ctx))
                return ctx.out(note='对中超时/目标丢失，直接坐底')
            return ctx.out(note='对中：目标丢失')

        sway = ctx.thrust(ctx.gf('AUV_SWAY_SIGN', 1.0) * ctx.gf('AUV_KP_SWAY', 0.60)
                          * float(obs.get('ex') or 0.0), ctx.gf('AUV_SWAY_MAX', 0.50))
        surge = ctx.thrust(ctx.gf('AUV_SURGE_SIGN', 1.0)
                           * self.gpf(ctx, 'KP_SURGE', 0.40)
                           * float(obs.get('ey') or 0.0),
                           self.gpf(ctx, 'SURGE_MAX', 0.30))
        ok = (abs(float(obs.get('dx') or 0.0)) < tol) and (abs(float(obs.get('dy') or 0.0)) < tol)
        if ctx.holding('centered', ok, now, ctx.hold_s(self.gpf(ctx, 'HOLD_S', 1.5))):
            return self.done(ctx, now, '对中完成')

        if ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 15.0):
            ctx.goto(self.next_of(ctx))
            return ctx.out(note='对中超时，直接坐底')
        return ctx.out(surge=surge, sway=sway,
                       note='对中 dx=%+.0f dy=%+.0f'
                            % (obs.get('dx') or 0.0, obs.get('dy') or 0.0))
