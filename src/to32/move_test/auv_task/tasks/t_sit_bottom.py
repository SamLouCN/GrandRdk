# -*- coding: utf-8 -*-
"""坐底任务：SitBottomTask（下沉积底，用收集框把球兜进去）

★ 判据优先级（逐行平移自旧 mission，不许改）：
  ① 离底净空 clearance 连续小于阈值（**绝对量**，比"深度停滞"可靠得多）
  ② 深度停滞 + 垂速接近 0（深度源没有 clearance 时的退路）
  ③ 超时 → 按到底处理（宁可错判"到底"，也不要在水下一直耗）

为什么坐底：无机械手，收集框（30×40cm，距底 20-60cm 红框）朝下装，
坐到池底把球罩进框里 —— 这是方案 §16f 拍板的捡球方式。
"""
from ..base import Task           # 任务基类
from ..servo import clamp_depth   # 深度限幅


class SitBottomTask(Task):
    """坐底：持续加深目标深度直到判定到底"""

    name = 'SIT_BOTTOM'

    def enter(self, ctx, now):
        self.rate = self.gpf(ctx, 'RATE_CMS', 15.0)     # 下沉速率 cm/s（改目标深度的斜率）
        self.d0 = ctx.dep.get('D') if ctx.depth_ok() else None
        # ⚠ 坐底目标上限**不能**走 clamp_depth（它会扣掉 AUV_DEPTH_BOTTOM_MARGIN_CM）：
        #   留了 10cm 余量就永远到不了"净空 < 6cm"，坐底只能靠 25s 超时兜底 ——
        #   这是重构干跑时实测出来的配置冲突（SIT_BOTTOM 停留 25.1s = 超时）。
        #   坐底就是要贴底，所以上限直接用池深本身。
        self.max_cm = ctx.gf('AUV_POOL_DEPTH_CM', ctx.gf('AUV_POOL_DEPTH_M', 1.3) * 100.0)
        ctx.log('[AUV] %s 坐底开始（目标上限 %.0fcm）' % (self.name, self.max_cm))

    def tick(self, ctx, now, dt):
        ctx.depth_cm = min(self.max_cm, ctx.depth_cm + self.rate * max(0.0, float(dt)))
        cl = (ctx.dep or {}).get('clearance')

        if ctx.depth_ok() and cl is not None:                       # ① 净空判据（主）
            if ctx.holding('sit_cl', float(cl) * 100.0 < self.gpf(ctx, 'CLEAR_CM', 6.0),
                           now, ctx.hold_s(self.gpf(ctx, 'CLEAR_HOLD_S', 1.0))):
                return self.done(ctx, now, '净空 %.1fcm 判定坐底' % (float(cl) * 100.0))
        elif ctx.depth_ok():                                        # ② 深度停滞判据（退路）
            vz = abs(float(ctx.dep.get('v_z') or 0.0))
            stall = False
            if self.d0 is not None:
                stall = abs(float(ctx.dep['D']) - float(self.d0)) \
                    < (self.gpf(ctx, 'STALL_CM', 2.0) / 100.0)
            if vz < self.gpf(ctx, 'VZ', 0.02) and stall:
                if ctx.holding('sit_stall', True, now,
                               ctx.hold_s(self.gpf(ctx, 'STALL_S', 2.0))):
                    return self.done(ctx, now, '深度停滞判定坐底')
            else:
                ctx.clear_hold('sit_stall')
                self.d0 = ctx.dep['D']

        if ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 25.0):     # ③ 超时
            ctx.goto(self.next_of(ctx))
            return ctx.out(note='坐底超时，按到底处理')
        return ctx.out(note='下沉中 D*=% .0fcm cl=%s' % (ctx.depth_cm, cl))
