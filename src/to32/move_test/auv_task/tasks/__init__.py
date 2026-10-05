# -*- coding: utf-8 -*-
"""任务库 —— 13 个任务类覆盖 24 个阶段（方案 §18.2/§18.3）

每个任务都是**独立可测**的小类：只管自己那一段，跳转由参数 next/fail 声明。
新增阶段 = 加一个 t_*.py + 在 plan.py 里加一行，不用改执行器。
"""
from .t_depth import DepthTask, SurfaceTask            # 定深 / 上浮
from .t_seek import SeekTask                           # 搜索（前视/下视通用）
from .t_ram_ball import RamBallTask                    # 撞球
from .t_pass_gate import PassGateTask                  # 穿门（高矮门通用）
from .t_center_ball import CenterBallTask              # 下视对中
from .t_sit_bottom import SitBottomTask                # 坐底兜球
from .t_hold import HoldTask                           # 零推力悬停
from .t_timed import TimedTask                         # 定时推进
from .t_turn import TurnTask                           # 转向
from .t_dead_reckon import DeadReckonTask              # 航位推算推进
from .t_go_home import GoHomeTask                      # 回出发区触壁
from .t_servo_drop import ServoDropTask                # 舵机投放
from .t_done import DoneTask                           # 结束停推

__all__ = [
    'DepthTask', 'SurfaceTask', 'SeekTask', 'RamBallTask', 'PassGateTask',
    'CenterBallTask', 'SitBottomTask', 'HoldTask', 'TimedTask', 'TurnTask',
    'DeadReckonTask', 'GoHomeTask', 'ServoDropTask', 'DoneTask',
]
