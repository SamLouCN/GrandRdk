# -*- coding: utf-8 -*-
"""t_return.py —— Return 任务脚本（整体为一个阶段）：右转 90° → 前进 5s（触壁提前结束）

定位（GrandRDKv2.5 重写版任务脚本，与 t_task1.py / t_task4.py 同构）：
    - Return **整体封装为一个 Stage**：STAGE_TABLE 里一项就是整个任务，
      可单独测试（task_config 里切 RETURN_TABLE 即可）
    - 内部子状态机依次调用 t_function 运动原语，复用同一套原语语义
    - 完成判据走 2026-10-06 用户口径：Turn **无超时兜底**（判据失效即永不完成）；
      前进段定时完成，且 touch_wall=True 启用 **IMU 加速度触壁兜底** ——
      前进中若发生触壁（遥测 acc_x/acc_y/acc_z 幅值突然降低，判据见 t_function
      _touch_wall_detect 与 task_config AUV_TOUCH_ACC_*）则提前结束本任务

动作序列（内部子步骤）：
    1. turn_step    相对当前 yaw **右转** AUV_RETURN_TURN_DEG（默认 90°，右为正；
                    目标角 = 当前 yaw 值 + 右转量，直接相加不做镜像 —— 用户口径）
    2. forward_step 恒 surge 直行 AUV_RETURN_FWD_S（默认 5s），保持定深
                    （AUV_RETURN_HEIGHT_CM，默认 60cm 离底）；
                    touch_wall=True：IMU 加速度突降判触壁 → 提前结束

接口契约（与 mission.Stage / mode_auv 对齐，勿破坏）：
    - 继承 mission.Stage：enter(ctx, now) 进入一次；step(now, dt) 每拍返回
      cmd dict（mode_auv 据此组 0x09）；返回 None = 本阶段完成 → Mission 切下一阶段
    - 每个子步骤有独立 st dict，交给原语函数存计时器/计数/锁存
    - 参数全部走 task_config（AUV_RETURN_*），遵循「任务参数全部进 task_config」

运行前提（无兜底口径，判据失效即死等）：
    - 转向子步骤完成依赖 yaw 遥测（$TEL actual_yaw），且切入时能读到当前航向
    - 触壁判据依赖 IMU 三轴加速度（$TEL acc_x/acc_y/acc_z）；无加速度遥测时
      前进段退化为纯定时 5s
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


class ReturnAll(Stage):
    """Return 任务（整体为一个阶段）：右转 90° → 前进 5s（触壁提前结束）。

    内部子状态机：子步骤返回 None（完成）即切下一个；全部完成 → 返回 None
    （整个任务阶段完成）。前进段 touch_wall=True，IMU 加速度突降判触壁 →
    前进子步骤提前返回 None（本任务提前结束，Mission 进入下一任务）。

    每个子步骤独立 st dict；转向子步骤在切入时读当前 yaw 遥测锁存目标角
    （当前 yaw + 右转量，直接相加不做镜像 —— 用户口径），读不到遥测 →
    该子步骤永不完成（无兜底）。
    """

    NAME = 'Return'

    def enter(self, now):
        self.idx = 0                       # 当前内部子步骤
        self.sts = [{} for _ in range(2)]  # 每个子步骤独立的 st dict
        self.turn_tgt = None               # 转向子步骤锁存的目标角

    def step(self, now, dt):
        while self.idx < 2:
            i = self.idx
            st = self.sts[i]
            if i == 0:                     # ① 右转 90°
                tgt = self._turn_target(float(getattr(TC, 'AUV_RETURN_TURN_DEG', 90.0)))
                cmd = t_function.turn_step(
                    self.ctx, st, now, dt,
                    target_yaw_deg=tgt,
                    target_height_cm=float(getattr(TC, 'AUV_RETURN_HEIGHT_CM', 60.0)))
            else:                          # ② 前进 5s，触壁提前结束
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_RETURN_FWD_S', 5.0)),
                    target_height_cm=float(getattr(TC, 'AUV_RETURN_HEIGHT_CM', 60.0)),
                    touch_wall=True)
            if cmd is None:
                self.idx += 1              # 本子步骤完成 → 切下一个
                continue
            return cmd                     # 本子步骤未完成 → 下发这一拍
        return None                        # 全部子步骤完成 = 任务阶段完成

    def _turn_target(self, deg):
        """转向目标角：切入时读当前 yaw，锁存 当前yaw + 右转量（直接相加不镜像）。"""
        if self.turn_tgt is None:
            tel = self.ctx.tel or {}
            y = tel.get('actual_yaw')
            if y is not None:
                self.turn_tgt = float(y) + float(deg)
            else:
                self.turn_tgt = float('nan')   # 无兜底：目标角未定 → 永不完成
                self.ctx.say('%s 切入时无 yaw 遥测，无法定目标角（无兜底，持续下发）' % self.NAME)
        return self.turn_tgt


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个 Return 任务；空表 = 开机即 DONE）
RETURN_TABLE = [ReturnAll]
