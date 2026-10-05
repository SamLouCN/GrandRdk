# -*- coding: utf-8 -*-
"""深度类任务：DepthTask（下潜/定深）与 SurfaceTask（上浮）

两者结构同构（"把目标深度改到某值 → 等它到位"），但**必须分成两个类**，原因：
  * 上浮是**硬性收尾动作**，判据与降级路径都更激进（超时也要收尾，绝不能卡在水下）；
  * 上浮阶段被 TestCfg.FORCED 永久保留，单独一个类便于 runner 和校验脚本识别它。

★ 垂直运动的物理约束：0x09 帧**没有 heave 推力槽位**，
  所以"上浮/下潜"只能改**目标深度**让固件闭环 —— 本任务全程不碰 surge 以外的垂直量。
"""
from ..base import Task          # 任务基类: 参数化与出口封装
from ..servo import depth_of_height, clamp_depth  # 高度→深度换算 / 深度限幅


class DepthTask(Task):
    """下潜 / 定深：把目标深度改到指定值，等它稳定后进入下一阶段

    目标深度的三种写法（优先级从高到低）：
      * `height_cm`  离底高度 cm → 由池深换算（**推荐**，与赛事规则口径一致）
      * `target_cm`  直接给目标深度 cm
      * `depth_m_key` 配置键名（如 'DIVE_DEPTH_M' → 读 AUV_DIVE_DEPTH_M，单位 m）

    判据：深度进入容差并**连续保持** hold_s 秒（防抖）。
    ⚠ 无深度源时不能判"到位"，走 `fallback_s` 定时放行 —— 别原地死等。
    """

    name = 'DEPTH'

    def enter(self, ctx, now):
        """算一次目标深度并下发（不要每拍重算配置，浪费且易被改配置的瞬间抖到）"""
        self.target = self._target_cm(ctx)
        self.surge = self.gpf(ctx, 'SURGE', 0.25)          # 边下潜边低速前进，省时间
        ctx.log('[AUV] %s 定深目标 %.0fcm（surge=%.2f）' % (self.name, self.target, self.surge))

    def _target_cm(self, ctx):
        """解析目标深度（三种写法取其一，最后统一限幅）"""
        h = self.gp(ctx, 'height_cm', None)
        if h is not None:
            return clamp_depth(ctx, depth_of_height(ctx, h))
        t = self.gp(ctx, 'target_cm', None)
        if t is not None:
            return clamp_depth(ctx, float(t))
        key = self.gp(ctx, 'depth_m_key', 'DIVE_DEPTH_M')
        return clamp_depth(ctx, ctx.gf('AUV_' + str(key), 1.0) * 100.0)

    def tick(self, ctx, now, dt):
        ctx.depth_cm = self.target
        tol = self.gpf(ctx, 'TOL_CM', 8.0)
        hold_s = ctx.hold_s(self.gpf(ctx, 'HOLD_S', 2.0))
        surge = self.surge

        if ctx.depth_ok():                                  # 有深度源 → 真判据
            err = abs(ctx.depth_cm_now() - self.target)
            if ctx.holding('dep_ok', err < tol, now, hold_s):
                return self.done(ctx, now, '定深到位 err=%.1fcm' % err)
        else:                                               # 无深度源 → 定时放行
            fb = self.gpf(ctx, 'FALLBACK_S', 12.0)
            if ctx.elapsed(now) > fb:
                nz = getattr(ctx, 'notify', None)           # ★ 上位机终端要看到"深度源不可用"
                if nz is not None:
                    nz.warn('DEGRADE', '深度源不可用：定深走定时 %.0fs 放行（判据只剩时间）' % fb,
                            stage=ctx.stage)
                return self.done(ctx, now, '无深度源，定时 %.0fs 放行' % fb)

        r = self.timeout(ctx, now, 'TIMEOUT_S', 30.0, '定深')
        if r is not None:
            return r
        return ctx.out(surge=thrust_of(ctx, surge),
                       note='定深中 D=%s → % .0fcm' % (fmt_d(ctx), self.target))


class SurfaceTask(Task):
    """★ 上浮收尾：把目标深度改到水面，到位后结束（或超时也结束）

    ★ 硬规则（方案 §16c 用户拍板）：**上浮不允许被配置关闭**。
      正常完成和中止路径共用本阶段，区别只在 ctx.abort_reason 是否为空。
    ⚠ 超时也要收尾：宁可"没浮到位就结束"，也不能永远卡在水下发 0x09。
    """

    name = 'SURFACE'

    def enter(self, ctx, now):
        self.target = clamp_depth(ctx, ctx.gf('AUV_SURFACE_DEPTH_M', 0.0) * 100.0)
        ctx.log('[AUV] 上浮 → 目标 %.0fcm%s'
                % (self.target, ('（中止：%s）' % ctx.abort_reason) if ctx.abort_reason else ''))

    def tick(self, ctx, now, dt):
        ctx.depth_cm = self.target
        done_cm = self.gpf(ctx, 'DONE_CM', 15.0)
        if ctx.depth_ok() and ctx.depth_cm_now() <= done_cm:
            if ctx.holding('surf', True, now, ctx.hold_s(1.0)):
                return self.done(ctx, now, '已浮出水面')
        else:
            ctx.clear_hold('surf')                          # 没到位就把计时清掉，别"攒够 1s 就过"
        if ctx.elapsed(now) > self.gpf(ctx, 'TIMEOUT_S', 30.0):
            ctx.log('[AUV] ⚠ 上浮超时，按已浮出处理（宁可早收尾也不卡在水下）')
            return self.done(ctx, now, '上浮超时')
        return ctx.out(surge=0.0, note='上浮中 D=%s' % fmt_d(ctx))


# ---------------------------------------------------------------- 小工具
def fmt_d(ctx):
    """当前深度的字符串（不可用显示 '-'），日志用"""
    d = ctx.depth_cm_now()
    if d is None:
        return '-'
    return '%.0fcm' % d


def thrust_of(ctx, v):
    """前向推力限幅（纵向要对抗水阻，下限比 AUV_MIN_THRUST 更实在）"""
    return ctx.thrust(v, ctx.gf('AUV_SURGE_MAX', 1.0))
