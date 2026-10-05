# -*- coding: utf-8 -*-
"""搜索任务：SeekTask（前视/下视找一个目标，找不到就超时或跳过）

一个任务覆盖所有"找东西"的阶段（方案 §18.2）：
  SEEK_BALL_F(前视撞球目标) / SEEK_GATE_1..4(前视门) / SEEK_BALL_B(下视待捡球)
差别只在参数：cam / label_key / aim_key / 是否扫深 / 超时后是中止还是跳过。

★ 搜索期为什么要"扫深"（seek_scan）：
  赛事规则只给"球距底 60cm""矮门中心距底 45cm"这类**离底高度**，
  但池深是现场量出来的，且相机俯仰装偏一点，目标在画面里的高度就对不上。
  所以找不到时在当前深度上下扫一段（正弦），比"闷头平扫"命中率高得多。
"""
from ..base import Task        # 任务基类
from ..servo import seek_scan  # 搜索期深度扫描（纯函数，逐行平移自旧 mission）


class SeekTask(Task):
    """搜索目标：稳定期 → 边前进边找 → 命中进下一阶段 / 超时按 fail 处置"""

    name = 'SEEK'

    def enter(self, ctx, now):
        self.cam = self.lit('cam', 'front')                 # front / bottom
        self.label = self.gp(ctx, 'label_key', 'ball')          # 目标类别（走配置键）
        self.aim = self.gp(ctx, 'aim_key', None)                # 瞄准点偏移（可 None）
        self.scan = self.gpb(ctx, 'scan', True)                 # 是否扫深
        self.surge = self.gpf(ctx, 'SURGE', 0.25)               # 搜索推进力
        self.base_cm = ctx.depth_cm                             # ★ 扫深基准 = 进入时的深度
        ctx.log('[AUV] %s 搜索[%s] %s（cam=%s, 扫深=%s）'
                % (self.name, self.label, ctx.stage, self.cam, self.scan))

    def tick(self, ctx, now, dt):
        # ---- 稳定期：刚转向/刚下潜时水面与自身扰动大，先别信检测
        if ctx.elapsed(now) < self.gpf(ctx, 'SETTLE_S', 1.5):
            return ctx.out(surge=self.surge, note='搜索稳定期')

        obs = ctx.see(self.cam, self.label, now, aim=self.aim)
        if obs is not None:
            ctx.goto(self.next_of(ctx))
            return ctx.out(surge=self.surge,
                           note='发现 %s score=%.2f' % (self.label, obs.get('score') or 0.0))

        r = self.timeout(ctx, now, 'TIMEOUT_S', 30.0, '搜索 %s' % self.label)
        if r is not None:
            return r

        if self.scan:
            ctx.depth_cm = seek_scan(ctx, now, ctx.t0, self.base_cm)
        return ctx.out(surge=self.surge,
                       note='搜索 %s(扫深 D*=%.0fcm)' % (self.label, ctx.depth_cm))
