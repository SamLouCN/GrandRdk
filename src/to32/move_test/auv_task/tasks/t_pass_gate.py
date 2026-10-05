# -*- coding: utf-8 -*-
"""穿门任务：PassGateTask（横向对中 + 机头转正 + 直冲穿门）

★ 视觉来源二选一（逐行平移自旧 mission）：
  * viskf 可用（`trust=1`）→ 用滤波量 e_x / de_x / s_n 做伺服（噪声小、带速度、与距离无关）
  * 否则 → 退回原始像素伺服（servo_depth / servo_yaw / sway 平移）
  两条路**共用同一套"已穿过"判定**，所以关掉 viskf 行为完全不变。

★ "已穿过"判定必须放在**目标消失**分支里：门贴近到出画才会消失，
  写在"看得见"分支里永远等不到（台架实测踩出来的真 bug）。

★ 门有高矮两种（矮门中心距底 45cm / 高门 65cm，赛事规则高矮交替）：
  进入时按 `height_cm` 先把深度对到门中心高度，再由视觉伺服微调。
"""
from ..base import Task  # 任务基类
from ..servo import (servo_depth, servo_yaw, servo_gate_yaw, servo_gate_surge,
                     clamp_depth, depth_of_height)  # 平移过来的伺服律


class PassGateTask(Task):
    """穿门：滤波/像素双路伺服 + 消失判定"""

    name = 'PASS_GATE'

    def enter(self, ctx, now):
        """★ 每轮必须复位 max_w / last_seen（4 个门共用本类，不复位会直接误判穿过）"""
        self.label = self.gp(ctx, 'label_key', 'gate')
        self.aim = self.gp(ctx, 'aim_key', None)
        self.cam = self.lit('cam', 'front')
        self.max_w = 0.0
        self.last_seen = now
        h = self.gp(ctx, 'height_cm', None)     # 字符串 = 配置键名（查 AUV_<键名>）
        if h is not None:
            ctx.depth_cm = clamp_depth(ctx, depth_of_height(ctx, h))
        ctx.log('[AUV] %s 穿门开始（目标深度 %.0fcm）' % (self.name, ctx.depth_cm))
        # ★ 图像卡尔曼不可用要说出来：穿门会退回像素伺服，判据灵敏度不一样
        if ctx.viskf is None:
            nz = getattr(ctx, 'notify', None)
            if nz is not None:
                nz.warn('DEGRADE', '图像卡尔曼不可用：穿门退回原始像素伺服', stage=ctx.stage)

    def tick(self, ctx, now, dt):
        obs = ctx.see(self.cam, self.label, now, aim=self.aim)
        surge = self.gpf(ctx, 'SURGE', 0.30)
        sway = 0.0
        vk = ctx.vk(now)                        # None = 不可用 → 走原始像素伺服（设计内降级）

        if obs is not None:
            self.last_seen = now
            self.max_w = max(self.max_w, float(obs.get('w') or 0.0))
            ctx.depth_cm = servo_depth(ctx, obs, dt, ctx.depth_cm)
            if vk is not None:
                ctx.yaw_deg = servo_gate_yaw(ctx, vk, dt, ctx.yaw_deg)
                surge = servo_gate_surge(ctx, vk)
                if ctx.g('AUV_GATE_SWAY_WITH_VISKF', False):
                    sway = ctx.thrust(ctx.gf('AUV_SWAY_SIGN', 1.0)
                                      * ctx.gf('AUV_KP_SWAY', 0.60)
                                      * float(obs.get('ex') or 0.0),
                                      ctx.gf('AUV_SWAY_MAX', 0.50))
                # ⚠ 升降只用原始像素伺服：el 依赖焦距与姿态补偿，而遥测 0 帧 → 姿态全程降级，
                #   拿 el 反修深度是拿不可信的量控不可信的环（滤波器文档也只让它做"到位确认"）
            else:
                ctx.yaw_deg = servo_yaw(ctx, obs, dt, ctx.yaw_deg)
                sway = ctx.thrust(ctx.gf('AUV_SWAY_SIGN', 1.0) * ctx.gf('AUV_KP_SWAY', 0.60)
                                  * float(obs.get('ex') or 0.0),
                                  ctx.gf('AUV_SWAY_MAX', 0.50))
        else:
            # ★ "穿过"判定：门宽曾经超阈值（或 s_n 到位）且现在消失了 → 判定已穿过
            thr = ctx.gf('AUV_GATE_PASS_W_RATIO', 0.80) * ctx.gf('AUV_IMG_W', 640.0)
            by_w = self.max_w > thr
            by_s = False
            if ctx.g('AUV_VISKF_USE', True) and isinstance(ctx.last_vk, dict):
                s_n = ctx.last_vk.get('s_n')
                by_s = (s_n is not None
                        and float(s_n) >= ctx.gf('AUV_GATE_S_STAR', 0.85)
                        * (1.0 - ctx.gf('AUV_GATE_CROSS_TOL', 0.05)))
            if (by_w or by_s) and (now - self.last_seen) > ctx.gf('AUV_GATE_LOST_S', 0.5):
                ctx.log('[AUV] 已穿门（%s）'
                        % ('门宽 %.0f > %.0f' % (self.max_w, thr) if by_w
                           else 's_n 到位 %s' % ctx.last_vk.get('s_n')))
                return self.done(ctx, now, '已穿门')

        r = self.timeout(ctx, now, 'TIMEOUT_S', 30.0, '穿门')
        if r is not None:
            return r

        if vk is not None:
            return ctx.out(surge=surge, sway=sway,
                           note='穿门(滤波) e_x=%+.3f s_n=%s w_max=%.0f'
                                % (float(vk.get('e_x') or 0.0), vk.get('s_n'), self.max_w))
        return ctx.out(surge=surge, sway=sway, note='穿门中 w_max=%.0f' % self.max_w)
