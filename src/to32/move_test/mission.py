# -*- coding: utf-8 -*-
"""任务状态机 —— GrandRDKv2.5 全新重写的骨架（阶段注册制，阶段表为空）

设计（v2.5 重写版，替代 v2.2 的 mission.py 17 阶段 getattr 分发）：
  - 每个任务阶段 = Stage 子类：enter(ctx, now) 进入时一次 + step(ctx, now, dt) 每拍
  - step 返回 cmd dict（本拍控制量）；返回 None 表示本阶段完成 → Mission 切下一阶段
  - 阶段表在 task_config.STAGE_TABLE 排序；**当前为空表 = 开机即 DONE**
  - Mission.step(now, dt, tel) -> cmd | None（None = 任务结束，不再下发 0x09）
    该契约与 mode_auv 对齐；cmd 字段: stage/note/yaw/depth/surge/sway/stop

cmd 字段语义（mode_auv 据此组 0x09）：
  stage  阶段名（日志与 $AUV 展示用）      note  本拍说明文字
  yaw    目标航向绝对角(°)                 depth 目标深度(cm，固件内闭环)
  surge  前后推力 [-1,1]                   sway  横向推力 [-1,1]
  stop   FLAG bit0：仅任务结束停推时置 1

约束：
  - 阶段内绝不直接碰共享内存/链路 —— 观测一律走 obs.VisionIF / obs.DepthIF
  - 阶段完成判据由原语/阶段自定；t_function 的 Dive/Turn 已按用户口径移除超时
    兜底（判据失效即持续执行），如需超时安全退出须在 Stage 层自行实现
"""
import task_config as TC


class Ctx(object):
    """传给每个阶段的上下文：配置 + 观测接口 + 遥测 + 日志"""

    def __init__(self, cfg, vision, depth, log=None, task_pids=None):
        self.cfg = cfg          # task_config
        self.vision = vision    # obs.VisionIF
        self.depth = depth      # obs.DepthIF
        self.log = log          # 日志函数，可为 None
        self.tel = None         # 最近一帧下位机遥测（每拍由 Mission 注入）
        self.task_pids = task_pids

    def say(self, msg):
        if self.log:
            self.log('[mission] ' + msg)


def apply_yaw_mirror(yaw, mirror):
    """固件对 Yaw 取负归一化，此处镜像取反抵消；并归一化到 [-180, 180]"""
    y = -float(yaw) if mirror else float(yaw)
    while y > 180.0:
        y -= 360.0
    while y < -180.0:
        y += 360.0
    return y


class Stage(object):
    """阶段基类：子类填 NAME，覆写 enter/step"""
    NAME = '?'

    def __init__(self, ctx):
        self.ctx = ctx

    def enter(self, now):
        """进入本阶段时调用一次：初始化计时器等"""

    def step(self, now, dt):
        """每拍调用：返回 cmd dict；返回 None = 本阶段完成"""
        raise NotImplementedError


class _StopCmd(object):
    """任务结束时的统一收尾：全零推力 + stop=1（只发一拍，之后 Mission 返回 None）"""

    @staticmethod
    def make(stage='DONE'):
        return {'stage': stage, 'note': '任务结束，停推', 'yaw': 0.0,
                'depth': 0.0, 'surge': 0.0, 'sway': 0.0, 'stop': 1}


class Mission(object):
    """任务状态机：按 STAGE_TABLE 顺序跑各阶段

    用法（重写任务时）：
      1. 在本文件（或独立模块）写 Stage 子类，如 class Dive(Stage)
      2. task_config.STAGE_TABLE = [Dive, SeekGate, ...] 排好顺序
      3. mode_auv.on_enter 会重建 Mission，每次切进 AUV 都从头跑
    """

    def __init__(self, cfg=None, vision=None, depth=None, log=None, task_pids=None):
        self.cfg = cfg or TC
        self.ctx = Ctx(self.cfg, vision, depth, log=log, task_pids=task_pids)
        self.table = list(getattr(self.cfg, 'STAGE_TABLE', []) or [])
        self.idx = -1                  # -1 = 尚未开始
        self.current = None            # 当前 Stage 实例
        self.done = False              # True = 全部阶段跑完
        self._stop_sent = False        # 收尾 stop 帧只发一拍
        self.stage = 'IDLE'            # 对外展示的当前阶段名
        self.ctx.say('任务状态机就绪（阶段数=%d%s）'
                     % (len(self.table), '，空表=开机即停' if not self.table else ''))

    # ------------------------------------------------------------ 主循环
    def step(self, now, dt, tel=None):
        """跑一拍。返回 cmd dict（组 0x09 下发）或 None（任务结束，不下发）

        单拍内允许多个阶段连续完成（前一阶段 step 返回 None 即切下一个），
        直到某个阶段给出 cmd、或阶段表跑完/异常收尾。循环次数有界（≤ 阶段数+1）。
        """
        self.ctx.tel = tel                       # 遥测注入：深度/航向判据从这取
        if self.done:
            if not self._stop_sent:              # 收尾：先发一拍 stop，之后彻底静默
                self._stop_sent = True
                return _StopCmd.make()
            return None

        while True:
            if self.current is None:             # 需要切阶段：首个 / 上一个已完成
                self.idx += 1
                if self.idx >= len(self.table):  # 阶段表跑完
                    self.done = True
                    self._stop_sent = True     # 收尾帧随本拍返回，不再重发
                    self.stage = 'DONE'
                    self.ctx.say('全部阶段完成')
                    return _StopCmd.make()
                cls = self.table[self.idx]
                self.current = cls(self.ctx)
                self.stage = getattr(cls, 'NAME', cls.__name__)
                self.current.enter(now)
                self.ctx.say('→ 阶段 %d/%d: %s'
                             % (self.idx + 1, len(self.table), self.stage))
            try:
                cmd = self.current.step(now, dt)
            except Exception as e:               # 阶段内部异常不许带崩整个任务
                self.ctx.say('阶段 %s 异常: %r —— 立即停推收尾' % (self.stage, e))
                self.done = True
                self._stop_sent = True           # 收尾帧随本拍返回，不再重发
                self.stage = 'DONE'
                return _StopCmd.make('ABORT')
            if cmd is None:                      # 本阶段完成 → 同拍切下一阶段
                self.ctx.say('阶段 %s 完成' % self.stage)
                self.current = None
                continue
            cmd.setdefault('stage', self.stage)
            cmd.setdefault('note', '')
            cmd.setdefault('yaw', 0.0)
            cmd.setdefault('depth', 0.0)
            cmd.setdefault('surge', 0.0)
            cmd.setdefault('sway', 0.0)
            cmd.setdefault('stop', 0)
            return cmd
