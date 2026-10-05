# -*- coding: utf-8 -*-
"""投放任务：ServoDropTask（舵机投放高尔夫球，100 分）

★ 舵机**还没实装**（2026-10-04 用户拍板）：ctx.servo 当前是 ServoStub，
  `ready()` 恒 False、`drop()` 只打一条 WARN。所以：
    * 舵机不可用 → **不中止**，照跑（投放 100 分很重要，但"没舵机就整局报废"更糟）；
    * 实装后只换 make_servo() 的返回实现，本任务一行都不用改。

⚠ 幂等：`drop()` 会被 20Hz 反复调用，真正触发只允许一次（接口内部保证 + 这里 fired 双保险）。
★ `fired` 必须在 enter() 里复位 —— 放在 __init__ 的话第二次跑（重进 AUV）就不会触发了。
"""
from ..base import Task  # 任务基类


class ServoDropTask(Task):
    """投放：悬停 → arm_s 后触发舵机 → 再保持到 total_s 后进下一阶段"""

    name = 'SERVO_DROP'

    def enter(self, ctx, now):
        self.fired = False                                  # ★ 必须每轮复位
        self.arm_s = self.gpf(ctx, 'ARM_S', 1.0)            # 进入后多久触发（等机体稳住）
        self.total_s = self.gpf(ctx, 'TOTAL_S', 3.0)        # 本阶段总时长
        self.ready = bool(ctx.servo is not None and ctx.servo.ready())
        if not self.ready:
            ctx.log('[AUV] ⚠ %s 舵机不可用（未实装）—— 投放动作将被跳过，任务继续'
                    % self.name)
            nz = getattr(ctx, 'notify', None)               # ★ 上位机终端要看到"投放 100 分没了"
            if nz is not None:
                nz.warn('DEGRADE', '舵机未实装/不可用：投放动作跳过，任务继续（投放 100 分拿不到）',
                        stage=ctx.stage)
        else:
            ctx.log('[AUV] %s 投放准备（%.1fs 后触发）' % (self.name, self.arm_s))

    def tick(self, ctx, now, dt):
        if (not self.fired) and ctx.elapsed(now) >= self.arm_s:
            self.fired = True
            ok = False
            if ctx.servo is not None:
                try:
                    ok = bool(ctx.servo.drop())             # 幂等接口，重复调用安全
                except Exception as e:                      # 舵机异常绝不冒泡到任务主循环
                    ctx.log('[AUV] ✗ %s 投放异常: %s' % (self.name, e))
                    ok = False
            ctx.log('[AUV] %s 投放触发 %s' % (self.name, '成功' if ok else '失败/未实装'))

        if ctx.elapsed(now) >= self.total_s:
            return self.done(ctx, now, '投放阶段结束')
        # 全程零推力悬停：投放要求机体稳，任何平移都会让球落偏
        return ctx.out(surge=0.0, sway=0.0,
                       note='[DROP] 保持悬停 %.1f/%.1fs' % (ctx.elapsed(now), self.total_s))
