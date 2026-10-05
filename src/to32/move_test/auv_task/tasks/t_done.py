# -*- coding: utf-8 -*-
"""结束任务：DoneTask（置停推位并停止下发）

★ 返回 None 是"让 mode_auv 停止下发 0x09"的信号：
  任务结束后**不能再持续发同一帧**（会顶掉上位机的手动控制，也会让下位机一直保持推力）。
  所以这里先发一拍带 stop=True 的帧（让固件明确停推），之后恒返回 None。
"""
from ..base import Task  # 任务基类


class DoneTask(Task):
    """结束：首拍发 stop=True，之后恒 None（mode_auv 据此停止下发）"""

    name = 'DONE'

    def enter(self, ctx, now):
        self.sent = False                                   # ★ 每轮都要复位（runner 可重跑）
        ctx.log('[AUV] 任务结束（%s），停推' % (ctx.abort_reason or '正常完成'))

    def tick(self, ctx, now, dt):
        if not self.sent:
            self.sent = True                                # 只发一拍停推帧
            return ctx.out(surge=0.0, sway=0.0, stop=True, note='结束停推')
        return None                                         # 之后不再下发
