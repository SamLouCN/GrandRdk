"""S100任务PID的真实算法对象与参数更新；不自动启动任何任务。"""
import os
import sys

_GATE = os.path.join(os.path.dirname(__file__), 'move_test', 'task', 'task_pass_door')
if _GATE not in sys.path:
    sys.path.insert(0, _GATE)
import gate_config as GC
from gate_pid import GatePid
from gate_mission import GateMission


class TaskPidController:
    def __init__(self):
        self.gate_pid = GatePid(GC.GATE_KP_YAW, GC.GATE_KD_YAW, GC.GATE_KP_SWAY,
                                GC.GATE_SWAY_DEAD, GC.GATE_KI, GC.GATE_I_MAX, GC.GATE_PSI_MAX)

    def update(self, loop, p, i, d):
        if loop != 'gate':
            raise ValueError('UNSUPPORTED')
        # P=航向比例，I=航向积分，D=角速度阻尼；横移比例保持既有配置。
        self.gate_pid.set_gains(p, i, d)
        return self.gate_pid.gains()

    def create_gate_mission(self, vision, efilter, tel, send, log=None, allow_blind=True):
        """任务调度接入时使用此工厂，共用正在被调参的GatePid实例。"""
        cfg = {k: getattr(GC, k) for k in dir(GC) if k.startswith('GATE_')}
        self.gate_pid.reset()
        return GateMission(cfg, vision, efilter, self.gate_pid, tel, send, log, allow_blind)
