# -*- coding: utf-8 -*-
"""t_task2.py —— 任务 2 脚本（整体为一个阶段）：右转 120° → 前进 3s → 左转 60° → 定深 55cm

定位（GrandRDKv2.5 重写版任务的第二个任务脚本，与 t_task1.py 同构）：
    - Task2 **整体封装为一个 Stage**：STAGE_TABLE 里一项就是整个任务，
      可单独测试（task_config 里切 TASK1_TABLE / TASK2_TABLE 即可）
    - 内部子状态机依次调用 t_function 运动原语，复用同一套原语语义
    - 完成判据走 2026-10-06 用户口径：Turn/Dive **无超时兜底**（判据失效即永不完成）

动作序列（内部子步骤）：
    1. turn_step    相对当前 yaw **右转** AUV_TASK2_TURN_DEG（默认 120°，右为正；
                    目标角 = 当前 yaw 值 + 右转量，直接相加不做镜像 —— 用户口径）
    2. forward_step 恒 surge 直行 AUV_TASK2_FWD_S（默认 3s），行进保持定深
                    （AUV_TASK2_DEPTH_CM，默认 60cm 离底）
    3. turn_step    相对当前 yaw **左转** AUV_TASK2_TURN_NEG_DEG（默认 -60°，右为正；
                    目标角 = 当前 yaw 值 + 左转量，直接相加不做镜像 —— 用户口径）
    4. dive_step    定深到「距池底 AUV_TASK2_DIVE_CM」（默认 55cm）

接口契约（与 mission.Stage / mode_auv 对齐，勿破坏）：
    - 继承 mission.Stage：enter(ctx, now) 进入一次；step(now, dt) 每拍返回
      cmd dict（mode_auv 据此组 0x09）；返回 None = 本阶段完成 → Mission 切下一阶段
    - 每个子步骤有独立 st dict，交给原语函数存计时器/计数/锁存
    - 参数全部走 task_config（AUV_TASK2_*），遵循「任务参数全部进 task_config」

运行前提（无兜底口径，判据失效即死等）：
    - 转向子步骤完成依赖 yaw 遥测（$TEL actual_yaw），且切入时能读到当前航向
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


class Task2All(Stage):
    """Task2 整体为一个阶段：右转 120° → 前进 3s → 左转 60° → 定深 55cm。

    内部子状态机：子步骤返回 None（完成）即切下一个；全部完成 → 返回 None
    （整个任务阶段完成）。每个子步骤独立 st dict；转向子步骤在切入该子步骤
    时读当前 yaw 遥测锁存目标角（当前 yaw + 转向量，直接相加不做镜像），
    读不到遥测 → 该子步骤永不完成（无兜底）。
    """

    NAME = 'Task2'

    def enter(self, now):
        self.idx = 0                       # 当前内部子步骤
        self.sts = [{} for _ in range(4)]  # 每个子步骤独立的 st dict
        self.turn_tgt = {}                 # 转向子步骤索引 → 锁存的目标角

    def step(self, now, dt):
        while self.idx < 4:
            i = self.idx
            st = self.sts[i]
            if i == 0:                     # ① 右转 120°
                cmd = t_function.turn_step(
                    self.ctx, st, now, dt,
                    target_yaw_deg=self._turn_target(i, float(getattr(TC, 'AUV_TASK2_TURN_DEG', 120.0))),
                    target_height_cm=float(getattr(TC, 'AUV_TASK2_DEPTH_CM', 60.0)))
            elif i == 1:                   # ② 前进 3s，保持定深 60
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_TASK2_FWD_S', 3.0)),
                    target_height_cm=float(getattr(TC, 'AUV_TASK2_DEPTH_CM', 60.0)))
            elif i == 2:                   # ③ 左转 60°
                cmd = t_function.turn_step(
                    self.ctx, st, now, dt,
                    target_yaw_deg=self._turn_target(i, float(getattr(TC, 'AUV_TASK2_TURN_NEG_DEG', -60.0))),
                    target_height_cm=float(getattr(TC, 'AUV_TASK2_DEPTH_CM', 60.0)))
            else:                          # ④ 定深 55cm（离底）
                cmd = t_function.dive_step(
                    self.ctx, st, now, dt,
                    target_height_cm=float(getattr(TC, 'AUV_TASK2_DIVE_CM', 55.0)))
            if cmd is None:
                self.idx += 1              # 本子步骤完成 → 切下一个
                continue
            return cmd                     # 本子步骤未完成 → 下发这一拍
        return None                        # 全部子步骤完成 = 任务阶段完成

    def _turn_target(self, i, deg):
        """转向子步骤目标角：切入时读当前 yaw，锁存 当前yaw + 转向量（直接相加不镜像）。"""
        if i not in self.turn_tgt:
            tel = self.ctx.tel or {}
            y = tel.get('actual_yaw')
            if y is not None:
                self.turn_tgt[i] = float(y) + deg
            else:
                self.turn_tgt[i] = None    # 无兜底：目标角未定 → 永不完成
                self.ctx.say('%s 子步骤%d 切入时无 yaw 遥测，无法定目标角（无兜底，持续下发）'
                             % (self.NAME, i + 1))
        return self.turn_tgt[i] if self.turn_tgt[i] is not None else 0.0


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个 Task2；空表 = 开机即 DONE）
TASK2_TABLE = [Task2All]
