# -*- coding: utf-8 -*-
"""t_search_ball.py —— 搜索球任务脚本（整体为一个阶段）：转向 60° → 前进 2s → zigzag 扫描找球

定位（GrandRDKv2.5 重写版任务脚本，与 t_task1.py / t_task2.py 同构）：
    - 本任务 **整体封装为一个 Stage**：STAGE_TABLE 里一项就是整个任务，
      可单独测试（task_config 里切 SEARCH_BALL_TABLE 即可）
    - 内部子状态机依次调用 t_function 运动原语，复用同一套原语语义
    - 完成判据走 2026-10-06 用户口径：Turn **无超时兜底**（判据失效即永不完成）；
      扫描段以下视识别到球为提前结束判据，扫完仍未找到则自然结束

动作序列（内部子步骤）：
    1. turn_step    相对当前 yaw **右转** AUV_SEARCH_BALL_TURN_DEG（默认 60°，右为正；
                    目标角 = 当前 yaw 值 + 右转量，直接相加不做镜像 —— 用户口径）
    2. forward_step 恒 surge 直行 AUV_SEARCH_BALL_FWD_S（默认 2s），保持定深
                    （AUV_SEARCH_BALL_HEIGHT_CM，默认 60cm 离底）
    3. 扫描搜索球（zigzag，每拍用**下视摄像头**查球，识别到即提前结束）：
       a. sway_step  右横移 AUV_SEARCH_BALL_SWAY_S（默认 1s）
       b. forward_step 前进 AUV_SEARCH_BALL_FWD_STEP_S（默认 1s）
       c. sway_step  左横移 AUV_SEARCH_BALL_SWAY_S（默认 1s）
       d. forward_step 前进 AUV_SEARCH_BALL_FWD_STEP_S（默认 1s）

接口契约（与 mission.Stage / mode_auv 对齐，勿破坏）：
    - 继承 mission.Stage：enter(ctx, now) 进入一次；step(now, dt) 每拍返回
      cmd dict（mode_auv 据此组 0x09）；返回 None = 本阶段完成 → Mission 切下一阶段
    - 每个子步骤有独立 st dict，交给原语函数存计时器/计数/锁存
    - 参数全部走 task_config（AUV_SEARCH_BALL_*），遵循「任务参数全部进 task_config」

运行前提（无兜底口径，判据失效即死等）：
    - 转向子步骤完成依赖 yaw 遥测（$TEL actual_yaw），且切入时能读到当前航向
    - 扫描段识球依赖下视摄像头（obs.VisionIF ← /dev/shm/momo_det_bottom.json，
      canonical 名 'ball'）；视觉缺失时按"找不到球"跑完 zigzag 自然结束
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


class SearchBallAll(Stage):
    """搜索球任务（整体为一个阶段）：转向 60° → 前进 2s → zigzag 扫描找球。

    内部子状态机：子步骤返回 None（完成）即切下一个；全部完成 → 返回 None
    （整个任务阶段完成）。扫描段（子步骤 3~6）每拍先查下视摄像头是否识别到球
    （ctx.vision.poll('bottom', 'ball', now)），识别到 → 本阶段立即完成
    （返回 None，Mission 进入下一任务）；扫完 zigzag 仍未找到 → 自然结束。

    每个子步骤独立 st dict；转向子步骤在切入时读当前 yaw 遥测锁存目标角
    （当前 yaw + 右转量，直接相加不做镜像 —— 用户口径），读不到遥测 →
    本拍不发 0x09 并等待（不以 0 兜底）；遥测始终不来 → 永不完成。
    """

    NAME = 'SearchBall'

    def enter(self, now):
        self.idx = 0                       # 当前内部子步骤
        self.sts = [{} for _ in range(6)]  # 每个子步骤独立的 st dict

    def step(self, now, dt):
        while self.idx < 6:
            i = self.idx
            # 扫描段（i>=2）每拍先查下视球：识别到 → 整个阶段提前完成
            if i >= 2 and self._ball_found(now):
                self.ctx.say('%s 下视识别到球，扫描提前结束 → 进入下一任务' % self.NAME)
                return None
            st = self.sts[i]
            if i == 0:                     # ① 右转 60°
                tgt = self._turn_target(now, st, float(getattr(TC, 'AUV_SEARCH_BALL_TURN_DEG', 60.0)))
                if tgt is None:            # 未拿到 yaw 遥测 → 本拍不下发 0x09（等待，不推进）
                    return t_function.wait_cmd(self.NAME, '等 yaw 遥测(Turn 目标未锁存)')
                cmd = t_function.turn_step(
                    self.ctx, st, now, dt,
                    target_yaw_deg=tgt,
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            elif i == 1:                   # ② 前进 2s
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_SEARCH_BALL_FWD_S', 2.0)),
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            elif i == 2:                   # ③ 扫描：右横移 1s
                cmd = t_function.sway_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_SEARCH_BALL_SWAY_S', 1.0)),
                    direction=1.0,
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            elif i == 3:                   # ④ 扫描：前进 1s
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_SEARCH_BALL_FWD_STEP_S', 1.0)),
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            elif i == 4:                   # ⑤ 扫描：左横移 1s
                cmd = t_function.sway_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_SEARCH_BALL_SWAY_S', 1.0)),
                    direction=-1.0,
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            else:                          # ⑥ 扫描：前进 1s
                cmd = t_function.forward_step(
                    self.ctx, st, now, dt,
                    duration_s=float(getattr(TC, 'AUV_SEARCH_BALL_FWD_STEP_S', 1.0)),
                    target_height_cm=float(getattr(TC, 'AUV_SEARCH_BALL_HEIGHT_CM', 60.0)))
            if cmd is None:
                self.idx += 1              # 本子步骤完成 → 切下一个
                continue
            return cmd                     # 本子步骤未完成 → 下发这一拍
        return None                        # 全部子步骤完成 = 任务阶段完成

    def _turn_target(self, now, st, deg):
        """转向目标角：切入时读当前 yaw，锁存 当前yaw + 右转量（同系直接相加不镜像）。

        未拿到 yaw 遥测 → 返回 None（调用方回 wait_cmd：本拍不下发、不推进）；
        绝不以 0 兜底（锁 0 = 命令转到绝对航向 0°），也不给 NaN（0x09 组帧会抛异常）。
        """
        return t_function.lock_turn_target(self.ctx, st, now, deg, stage=self.NAME)

    def _ball_found(self, now):
        """下视摄像头是否识别到球（canonical 'ball'）。视觉缺失/异常一律按没找到。"""
        v = getattr(self.ctx, 'vision', None)
        if v is None:
            return False
        try:
            return v.poll('bottom', 'ball', now) is not None
        except Exception:
            return False


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个搜索球任务；空表 = 开机即 DONE）
SEARCH_BALL_TABLE = [SearchBallAll]
