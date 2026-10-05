# -*- coding: utf-8 -*-
"""任务基类 Task —— 所有阶段任务的统一骨架

设计要点：
  * 一个任务 = 一个小类，**只管自己那一段**（搜索 / 撞击 / 穿门 / 坐底 / 定时 / ...），
    阶段之间的跳转由参数 `next` / `fail` 声明，任务内部不认识"下一个任务是谁"。
  * 参数全部走 `gp()`：**字符串 = 去配置里查 `AUV_<字符串>`，数字 = 字面量**。
    这样一个 `SeekTask` 既能当前视搜球、也能当下视搜球，只是参数不同
    （方案 §16.5/§18.2：13 个任务文件覆盖 24 个阶段，靠的就是参数化）。
  * 生命周期 `enter() → tick() → exit()`：一次性初始化放 enter，**绝不放在 __init__**，
    因为同一个任务实例在一次任务里可能被执行多次（比如 4 个门共用 2 个类）。

★ 唯一出口是返回一个控制量 dict（由 ctx.out() 生成），
  或设置 ctx.pending 让执行器跳段。任务自己**不许**直接改 ctx.stage。
"""
import time  # 时间库: enter 的默认时刻兜底

from .notify import CODE_ABORT, CODE_TIMEOUT, CODE_SKIP  # 上位机终端提示的机器码


class Task(object):
    """任务基类：参数化 + 三段生命周期"""

    name = 'UNNAMED'      # 任务类名（子类覆盖，用于日志/报错定位）

    def __init__(self, **params):
        """params 是 plan.py 里写死的阶段参数（字符串 = 配置键名，数字 = 字面量）"""
        self.p = dict(params)     # 参数字典
        self.entered = False      # 本轮是否已 enter（由执行器复位，任务不要自己改）

    # ---------------------------------------------------------------- 参数解析
    def gp(self, ctx, key, default=None):
        """取参数：字符串 → 从配置查 `AUV_<字符串>`；其它类型 → 原样返回

        为什么要这套：水池调参时改的是 auv_config.py，不是代码；
        而"转 120°"这种阶段专属量不该进全局配置，就直接写数字。
        """
        v = self.p.get(key, None)
        if v is None:
            v = default
        if isinstance(v, str):
            return ctx.g('AUV_' + v, default)
        return v

    def gpf(self, ctx, key, default=0.0):
        """取参数并强制转 float（配置里可能被写成字符串/int，转失败回落缺省）"""
        try:
            return float(self.gp(ctx, key, default))
        except (TypeError, ValueError):
            return float(default)

    def gpb(self, ctx, key, default=False):
        """取参数并强制转 bool"""
        return bool(self.gp(ctx, key, default))

    def lit(self, key, default=None):
        """★ 取**字面量**参数：字符串**不**当配置键查，原样返回

        ⚠ 必须用它来取 next / fail / cam 这类"本身就是字符串"的参数 ——
          走 gp() 会把 `next='SEEK_BALL_F'` 当成配置键去查 `AUV_SEEK_BALL_F`，
          查不到就回落成默认，跳转会静默失效（这个坑已经在台架上踩过一次）。
        """
        v = self.p.get(key, None)
        return default if v is None else v

    def next_of(self, ctx):
        """成功出口（阶段名或 'DONE'）—— 字面量，不走配置"""
        return self.lit('next', 'DONE')

    def fail_of(self, ctx):
        """失败/超时出口：阶段名、'ABORT'（中止上浮）或 'DONE' —— 字面量"""
        return self.lit('fail', 'ABORT')

    # ---------------------------------------------------------------- 出口
    def done(self, ctx, now, note=''):
        """成功 → 跳 next"""
        ctx.goto(self.next_of(ctx))
        return ctx.out(note=note or ('%s 完成' % self.name))

    def bail(self, ctx, now, reason):
        """失败/超时 → 按 fail 参数处置

        fail='ABORT'  → ctx.abort(reason)：打标记后强制走 SURFACE（**上浮不可关闭**）
        fail='<阶段名>' → 跳过这一段继续跑（门没找到就找下一个门，别整局报废）

        ★ 顺手给上位机终端发一条提示（$MSG）：人不用盯板子日志也能知道"为什么跳了"。
        """
        f = self.fail_of(ctx)
        nz = getattr(ctx, 'notify', None)
        if f == 'ABORT':
            ctx.abort(reason)
            if nz is not None:
                nz.error(CODE_ABORT, '%s：%s → 上浮' % (ctx.stage, reason), stage=ctx.stage)
        else:
            ctx.log('[AUV] %s ✗ %s → 跳过，转 %s' % (self.name, reason, f))
            ctx.goto(f)
            if nz is not None:
                code = CODE_TIMEOUT if ('超时' in str(reason)) else CODE_SKIP
                nz.warn(code, '%s：%s → 跳过，转 %s' % (ctx.stage, reason, f), stage=ctx.stage)
        return ctx.out(note='%s ✗ %s' % (self.name, reason))

    def timeout(self, ctx, now, key, default_s, reason):
        """超时判定 + 处置的合并写法（超时秒数也走参数化，便于单测把时间缩到 0.1s）"""
        if ctx.elapsed(now) > self.gpf(ctx, key, default_s):
            return self.bail(ctx, now, '%s %.1fs 超时'
                             % (reason, self.gpf(ctx, key, default_s)))
        return None

    # ---------------------------------------------------------------- 生命周期
    def enter(self, ctx, now):
        """进入本阶段时执行一次：在这里复位**本轮**状态

        ★ 所有"只属于这一轮"的变量必须在这里（而不是 __init__）复位 ——
          4 个门共用同一个 PassGateTask 类，第 2 个门若沿用第 1 个门的 max_w 会直接误判穿过。
        """
        return None

    def tick(self, ctx, now, dt):
        """每拍执行一次：返回控制量 dict，或设置 ctx.pending 让执行器跳段"""
        raise NotImplementedError('%s.tick() 未实现' % self.name)

    def exit(self, ctx, now):
        """离开本阶段时执行一次（收尾/关舵机/记日志）"""
        return None

    # ---------------------------------------------------------------- 工具
    @staticmethod
    def _now():
        """取当前时刻（抽出来是为了单测能注入假时钟）"""
        return time.time()
