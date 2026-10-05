# -*- coding: utf-8 -*-
"""保持类任务：HoldTask（原地悬停 N 秒，零推力）

用途（方案 §18.2 阶段表）：
  * BOTTOM_HOLD：坐底保持（收集框兜球需要时间，球要滚进框里）
  * RELEASE_HOVER：投放前悬停（把余速吃掉，舵机投放才准）

为什么单独一个类而不是"定时前进"的参数变体：
  零推力悬停是**最安全的默认动作**（急停/暂停/降级都往它落），
  单独成类可以让 runner 在暂停、监督 HOLD 时直接复用它，不必再造一个出口。
"""
from ..base import Task  # 任务基类


class HoldTask(Task):
    """原地悬停 dur_s 秒，全程 surge=sway=0（只保持当前目标深度/航向）"""

    name = 'HOLD'

    def enter(self, ctx, now):
        """进入时记一条日志（含时长，便于水池对照"到底停了多久"）"""
        self.dur = self.gpf(ctx, 'DUR_S', 3.0)
        ctx.log('[AUV] %s 悬停 %.1fs' % (self.name, self.dur))

    def tick(self, ctx, now, dt):
        if ctx.elapsed(now) >= self.dur:
            return self.done(ctx, now, '悬停完成 %.1fs' % self.dur)
        return ctx.out(surge=0.0, sway=0.0,
                       note='%s 悬停 %.1f/%.1fs' % (self.name, ctx.elapsed(now), self.dur))
