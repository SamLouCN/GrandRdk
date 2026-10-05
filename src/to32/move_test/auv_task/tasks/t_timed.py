# -*- coding: utf-8 -*-
"""定时推进任务：TimedTask（按固定时长做一件"开环"的事）

适用：水池里最常用的一类调试动作 —— "往前推 3 秒看看走多远"。
⚠ 它是**开环**的：没有速度反馈（光流已整体停用，方案主线），
  所以"走多远"完全取决于推力与水体，🔴 AUV_SPEED_MPS 未标定前只能验结构不能验距离。

与 HoldTask 的差别：HoldTask 恒零推力；TimedTask 可给 surge/sway，也能顺手改目标深度。
与 DeadReckonTask 的差别：TimedTask 只认时间；DeadReckonTask 认"推算距离"。
"""
from ..base import Task  # 任务基类
from ..servo import clamp_depth  # 深度限幅


class TimedTask(Task):
    """定时推进：以固定推力走 dur_s 秒（可选：进入时改一次目标深度）"""

    name = 'TIMED'

    def enter(self, ctx, now):
        self.dur = self.gpf(ctx, 'DUR_S', 3.0)                 # 持续时长 s
        self.surge = self.gpf(ctx, 'SURGE', 0.25)              # 前向推力
        self.sway = self.gpf(ctx, 'SWAY', 0.0)                 # 横向推力
        # 可选：进入时把深度改到指定值（不写 = 沿用当前目标深度）
        h = self.gp(ctx, 'height_cm', None)
        if h is not None:
            from ..servo import depth_of_height
            ctx.depth_cm = clamp_depth(ctx, depth_of_height(ctx, h))
        ctx.log('[AUV] %s 推进 %.1fs surge=%.2f sway=%.2f'
                % (self.name, self.dur, self.surge, self.sway))

    def tick(self, ctx, now, dt):
        if ctx.elapsed(now) >= self.dur:
            return self.done(ctx, now, '推进完成 %.1fs' % self.dur)
        return ctx.out(surge=ctx.thrust(self.surge, ctx.gf('AUV_SURGE_MAX', 1.0)),
                       sway=ctx.thrust(self.sway, ctx.gf('AUV_SWAY_MAX', 0.50)),
                       note='%s 推进 %.1f/%.1fs' % (self.name, ctx.elapsed(now), self.dur))
