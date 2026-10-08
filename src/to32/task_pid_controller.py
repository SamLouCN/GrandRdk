"""S100 任务 PID 通道（占位）。

2026-10-08：原 gate PID（GatePid/GateMission/gate_config）随穿门任务 task_pass_door
一并清理，本文件收成占位——`$TASKPID,<loop>,P,I,D#` 的解析/ACK 通道
（task_pid_wire + protocol + mode_dispatcher）保持可用，但当前没有任何可调 loop。

重写任务时在这里接新的 PID 对象：
    __init__ 建 PID 实例  →  update(loop, p, i, d) 识别 loop 名并 set_gains
    mode_dispatcher 的 ctx.task_pids 会把它发给任务阶段（Stage 里用 ctx.task_pids）。
"""


class TaskPidController(object):

    def __init__(self):
        pass

    def update(self, loop, p, i, d):
        # 没有已注册的 loop → 上位机收到 UNSUPPORTED
        raise ValueError('UNSUPPORTED')
