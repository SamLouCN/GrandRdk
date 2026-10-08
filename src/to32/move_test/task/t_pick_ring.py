# -*- coding: utf-8 -*-
"""t_pick_ring.py —— 捡环任务脚本（纯开环扫描，整体为一个阶段）：
定深 40cm(离底) → 3×[前进 1s → 转 180° → 右移 0.5s → 前进 1s → 转 180° → 左移 0.5s]

定位（GrandRDKv2.5 重写版任务脚本，与 t_task1.py / t_search_ball.py 同构）：
    - 本任务 **整体封装为一个 Stage**：STAGE_TABLE 里一项就是整个任务，
      可单独测试（STAGE_TABLE = PICK_RING_TABLE；默认不入主序列，同 Task4 模式）
    - **纯开环**：全程不碰视觉（不用 obs.VisionIF，无 CLASS_NAMES 依赖），
      动作是固定编排；唯一闭环是固件内定深 + 转向到位判据
    - 内部子状态机依次调用 t_function 运动原语，复用同一套原语语义

动作序列（内部子步骤，1 + 6×N = 19 步）：
    0. dive_step    定深到距池底 40cm（融合深度带内保持，无兜底）
    每循环 6 步（×AUV_PICK_RING_CYCLES）：
    1. forward_step 前进 1s
    2. turn_step    相对当前 yaw **右转** 180°（目标 = 当前 yaw + 转角，
                    直接相加不镜像 —— 用户口径；180° 在 wrap 边界，
                    yaw_err_deg 取 |err| 不受符号翻转影响）
    3. sway_step    右移 0.5s（direction=+1）
    4. forward_step 前进 1s
    5. turn_step    再右转 180°（回到原航向）
    6. sway_step    左移 0.5s（direction=-1）
    ★ 车身系方向语义（2026-10-08 方案确认项）：转向后朝向反转，朝南时"右移"
      与朝北时"左移"地理同向 → 每循环净横移 2×0.5s 同向，构成蛇形梯级扫描

接口契约（与 mission.Stage / mode_auv 对齐，勿破坏）：
    - 继承 mission.Stage：enter(now) 进入一次；step(now, dt) 每拍返回
      cmd dict（mode_auv 据此组 0x09）；返回 None = 本阶段完成 → Mission 切下一阶段
    - enter 时把动作序列展开为 plan 列表（1+6×N 个子步骤元组），每个子步骤
      独立 st dict（计时器/锁存互不串）；turn 的目标角锁存进该子步骤的 st['_tgt']

运行前提（无兜底口径，判据失效即死等）：
    - 定深子步骤依赖 kalman 融合深度（ctx.depth → DepthIF），融合源 !ok 即永不完成
    - 转向子步骤依赖 yaw 遥测（$TEL actual_yaw），切入时读不到 → 永不完成（无兜底）
    - ★ 每个子步骤都显式传 target_height_cm：各子步骤 st 全新，原语"沿用上一拍"
      机制首拍会回落 AUV_DEFAULT_HEIGHT_CM(60)，必须逐子步骤显式传参规避
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


class PickRingAll(Stage):
    """捡环任务（纯开环，整体为一个阶段）：
    定深 40cm(离底) → N×[前进 1s → 右转 180° → 右移 0.5s → 前进 1s → 右转 180° → 左移 0.5s]。

    enter 时展开计划表（plan：1+6×N 个 ('kind', ...) 元组），step 每拍按
    self.idx 取当前子步骤分发到 t_function 原语；子步骤返回 None（完成）
    即切下一个，全部完成 → 返回 None（整个任务阶段结束）。
    """

    NAME = 'PickRing'

    def enter(self, now):
        h = float(getattr(TC, 'AUV_PICK_RING_HEIGHT_CM', 40.0))
        fw = float(getattr(TC, 'AUV_PICK_RING_FWD_S', 1.0))
        td = float(getattr(TC, 'AUV_PICK_RING_TURN_DEG', 180.0))
        sw = float(getattr(TC, 'AUV_PICK_RING_SWAY_S', 0.5))
        self.n = int(getattr(TC, 'AUV_PICK_RING_CYCLES', 3))
        cyc = [('fwd', fw), ('turn', td), ('sway', 1.0, sw),
               ('fwd', fw), ('turn', td), ('sway', -1.0, sw)]
        self.plan = [('dive', h)] + cyc * self.n   # 1 + 6×N 个子步骤
        self.sts = [{} for _ in self.plan]         # 每个子步骤独立 st dict
        self.idx = 0                               # 当前子步骤下标

    def step(self, now, dt):
        h = float(getattr(TC, 'AUV_PICK_RING_HEIGHT_CM', 40.0))
        while self.idx < len(self.plan):
            act = self.plan[self.idx]
            st = self.sts[self.idx]
            if act[0] == 'dive':                   # ⓪ 定深（离底 h cm）
                cmd = t_function.dive_step(self.ctx, st, now, dt,
                                           target_height_cm=act[1],
                                           stage='PickRing/Dive')
            elif act[0] == 'fwd':                  # 前进 1s
                cmd = t_function.forward_step(self.ctx, st, now, dt,
                                              duration_s=act[1],
                                              target_height_cm=h,
                                              stage='PickRing/Forward')
            elif act[0] == 'turn':                 # 右转 180°（目标 = 当前 yaw + 转角）
                cmd = t_function.turn_step(self.ctx, st, now, dt,
                                           target_yaw_deg=self._turn_target(st, act[1]),
                                           target_height_cm=h,
                                           stage='PickRing/Turn')
            else:                                  # 横移 0.5s（+1 右 / -1 左）
                cmd = t_function.sway_step(self.ctx, st, now, dt,
                                           duration_s=act[2],
                                           direction=act[1],
                                           target_height_cm=h,
                                           stage='PickRing/Sway')
            if cmd is None:
                self.idx += 1                      # 本子步骤完成 → 切下一个
                continue
            return cmd                             # 本子步骤未完成 → 下发这一拍
        self.ctx.say('%s 开环扫描完成（%d 循环），阶段结束' % (self.NAME, self.n))
        return None                                # 全部子步骤完成 = 任务阶段完成

    def _turn_target(self, st, deg):
        """转向目标角：本子步骤首拍读当前 yaw 锁存 当前yaw+右转量（直接相加不镜像）。

        锁存进本子步骤自己的 st dict（各次转向互不串）；无 yaw 遥测 →
        NaN（turn_step 无兜底：目标角未定 → 永不完成，持续下发转向指令，
        与 t_search_ball._turn_target 同款既有口径）。
        """
        if st.get('_tgt') is None:
            tel = self.ctx.tel or {}
            y = tel.get('actual_yaw')
            if y is not None:
                st['_tgt'] = float(y) + float(deg)
            else:
                st['_tgt'] = float('nan')
                self.ctx.say('%s 转向切入时无 yaw 遥测，无法定目标角（无兜底，持续下发）' % self.NAME)
        return st['_tgt']


# 阶段注册表：task_config.PICK_RING_TABLE 引用（一项 = 整个捡环任务；空表 = 开机即 DONE）
PICK_RING_TABLE = [PickRingAll]
