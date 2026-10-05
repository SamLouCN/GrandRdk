# -*- coding: utf-8 -*-
"""auv_task —— AUV 自主运动**任务化**执行框架（替代旧 mission.py 单体状态机）

目录结构:
    servo.py      伺服律（纯函数，从旧 mission 逐行平移，逻辑不许改）
    ctx.py        任务上下文 TaskCtx（观测/计时/参数/出口的唯一窄接口）
    base.py       任务基类 Task（参数化 + enter/tick/exit 三段生命周期）
    servo_if.py   投放舵机接口（ServoStub 占位，实装后换实现即可）
    testcfg.py    测试模式（总开关 + 次级选段；SURFACE 强制保留）
    test_config.py ★ 测试阶段唯一入口（选模块 + 各模块参数 + 全局覆盖 + 离线剧本）
    plan.py       阶段表（24 段，改流程只改这里）
    runner.py     执行器 TaskRunner（顺序/跳转裁决/预算/监督，零改动对接 mode_auv）
    tasks/        13 个独立任务类（每个都能单独跑、单独测）

★ 三句话记住这个框架:
  1. 一个阶段 = 一个 Task 子类，只管自己那一段，跳转由参数 next/fail 声明；
  2. 任务只通过 ctx 看世界（vision/depth/viskf 全是可注入的假实现）→ 能离线跑；
  3. 跳转只放信号（ctx.goto/abort/finish），真正跳不跳由 runner 裁决
     （测试模式过滤、SURFACE 强制保留、DONE 收尾都在那儿）。

★ 测试阶段要改的东西**全在 test_config.py**（不用碰 auv_config.py）:
    TEST_ENABLED / TEST_STAGES / STAGE_PARAMS / GLOBAL_PARAMS
"""
from .servo import (wrap180, sign, apply_yaw_mirror, depth_of_height, clamp_depth,
                    thrust, servo_depth, servo_yaw, servo_gate_yaw, servo_gate_surge,
                    seek_scan, tel_val, tel_depth_cm, tel_yaw, tel_acc)
from .ctx import TaskCtx, YawEst
from .base import Task
from .servo_if import ServoIF, ServoStub, ServoSerial, make_servo
from .testcfg import TestCfg, FORCED, apply_global
from .plan import build_plan, names, pairs, index_of
from .runner import TaskRunner, SURFACE, DONE
from .notify import (AuvMsg, NoMsg, make_notifier, build_msg_frame,
                     describe, STAGE_DESC, INFO, WARN, ERROR)
from .gate import (auv_initial_allowed, sanitize_persist_mode, guard_initial_mode,
                   is_pc_source, SRC_PC, SRC_START, SRC_FORCED, SRC_SAVED)
from . import tasks
from . import test_config  # ★ 测试阶段唯一入口（改它，别改 auv_config.py）

__all__ = [
    'wrap180', 'sign', 'apply_yaw_mirror', 'depth_of_height', 'clamp_depth',
    'thrust', 'servo_depth', 'servo_yaw', 'servo_gate_yaw', 'servo_gate_surge',
    'seek_scan', 'tel_val', 'tel_depth_cm', 'tel_yaw', 'tel_acc',
    'TaskCtx', 'YawEst', 'Task',
    'ServoIF', 'ServoStub', 'ServoSerial', 'make_servo',
    'TestCfg', 'FORCED', 'apply_global',
    'build_plan', 'names', 'pairs', 'index_of',
    'TaskRunner', 'SURFACE', 'DONE',
    'AuvMsg', 'NoMsg', 'make_notifier', 'build_msg_frame', 'describe', 'STAGE_DESC',
    'INFO', 'WARN', 'ERROR',
    'auv_initial_allowed', 'sanitize_persist_mode', 'guard_initial_mode',
    'is_pc_source', 'SRC_PC', 'SRC_START', 'SRC_FORCED', 'SRC_SAVED',
    'test_config', 'tasks',
]
