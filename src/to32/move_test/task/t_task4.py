# -*- coding: utf-8 -*-
"""t_task4.py —— 任务 4 脚本（整体为一个阶段）：定深 65cm（离底）→ 前进 2s → 悬停 3s

定位（GrandRDKv2.5 重写版任务脚本，与 t_task1.py / t_task2.py 同构）：
    - Task4 **整体封装为一个 Stage**：STAGE_TABLE 里一项就是整个任务，
      可单独测试（task_config 里切 TASK4_TABLE 即可）
    - 内部子状态机依次调用 t_function 运动原语，复用同一套原语语义
    - 完成判据走 2026-10-06 用户口径：Dive **无超时兜底**（判据失效即永不完成）；
      Forward/Hover 定时即完成

动作序列（内部子步骤）：
    1. dive_step    定深到「距池底 AUV_TASK4_DEPTH_CM」（默认 65cm）
    2. forward_step 恒 surge 直行 AUV_TASK4_FWD_S（默认 2s），保持定深 65
    3. hover_step   悬停 AUV_TASK4_HOVER_S（默认 3s）：定深保持 + 锁航向 + 零推力

接口契约（与 mission.Stage / mode_auv 对齐，勿破坏）：
    - 继承 mission.Stage：enter(ctx, now) 进入一次；step(now, dt) 每拍返回
      cmd dict（mode_auv 据此组 0x09）；返回 None = 本阶段完成 → Mission 切下一阶段
    - 每个子步骤有独立 st dict，交给原语函数存计时器/计数/锁存
    - 参数全部走 task_config（AUV_TASK4_*），遵循「任务参数全部进 task_config」

运行前提（无兜底口径，判据失效即死等）：
    - 定深子步骤完成依赖 depth_kalman 融合深度（obs.DepthIF ← /dev/shm/momo_depth.json）
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                           # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC
from mission import Stage
import t_function


class Task4All(Stage):
    """Task4 整体为一个阶段：定深 65cm（离底）→ 前进 2s → 悬停 3s。

    内部子状态机：子步骤返回 None（完成）即切下一个；全部完成 → 返回 None
    （整个任务阶段完成）。每个子步骤独立 st dict，无跨子步骤状态共享。
    """

    NAME = 'Task4'

    def enter(self, now):
        self.idx = 0                       # 当前内部子步骤
        self.sts = [{} for _ in range(3)]  # 每个子步骤独立的 st dict

    def step(self, now, dt):
        while self.idx < 3:
            i = self.idx
            st = self.sts[i]
            if i == 0:                     # ① 定深 65cm（离底）
                cmd = t_function.dive_step(
                    self.ctx, st, now, dt,
                    target_height_cm=float(getattr(TC, 'AUV_TASK4_DEPTH_CM', 65.0)))
            elif i == 1:                   # ② 前进 2s，保持定深 65
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_TASK4_FWD_S', 2.0)),
                    target_height_cm=float(getattr(TC, 'AUV_TASK4_DEPTH_CM', 65.0)))
            else:                          # ③ 悬停 3s：定深保持 + 锁航向 + 零推力
                cmd = t_function.hover_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_TASK4_HOVER_S', 3.0)),
                    target_height_cm=float(getattr(TC, 'AUV_TASK4_DEPTH_CM', 65.0)))
            if cmd is None:
                self.idx += 1              # 本子步骤完成 → 切下一个
                continue
            return cmd                     # 本子步骤未完成 → 下发这一拍
        return None                        # 全部子步骤完成 = 任务阶段完成


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个 Task4；空表 = 开机即 DONE）
TASK4_TABLE = [Task4All]
