# -*- coding: utf-8 -*-
"""test_config.py —— 测试模式总开关与测试表（test_mode/ 唯一配置入口）

生效链路（2026-10-06）：
    to32/mode_dispatcher._register_modes() 启动时 import 本文件：
      TEST_MODE_ENABLED=True  → AUV 模式位被 test_runner.TestMode 接管：
                                上位机切换成 AUV 模式（$CMD.mode=1）自动进入测试模式；
                                测试模式运行中，上位机随时切回 ROV（$CMD.mode=0）即退出
                                （发一帧停推 → ROV 接管）；再切 AUV 又从头进测试模式。
      TEST_MODE_ENABLED=False → AUV 位 = 正式 AuvMode（2026-10-07 接回主链路，
                                按 task_config.STAGE_TABLE 依次自动跑完全部阶段）。
    ⚠ 本文件会 import 任务链（task_config / task 脚本）：写错导致 import 失败时，
      dispatcher 捕获异常自动回退占位 AuvModeStub（宁停勿跑，journal 有 print 提示），
      不拖垮中位机。

★★ 测试表 TEST_TABLE（本文件核心）：Stage 类的**列表表达式**，写法与
   task_config.STAGE_TABLE 完全一致 —— 既可以只测一个阶段，也可以任意拼多阶段：

       TEST_TABLE = [t_task1.Task1All]                    # 只测一个阶段
       TEST_TABLE = TASK1_TABLE + TASK2_TABLE              # 两张任务表整表拼
       TEST_TABLE = [t_task2.Task2All, EchoObs]            # 单阶段 + 内置回显混拼
       TEST_TABLE = []                                     # 空表 = 进入即 DONE（安全停推）

   可用的名字（本文件已 import 好，直接写进表达式）：
       TASK1_TABLE / TASK2_TABLE / SEARCH_BALL_TABLE / TASK4_TABLE / RETURN_TABLE
       task_config 里各任务表的整表（t_task*.py 导出）
       t_task1.* / t_task2.* / ...   各任务脚本的 Stage 类（Task1All/Task2All/...）
       EchoObs                     内置观测回显（不动船：悬停保持 + 节流打印融合深度/
                                   前视检测/遥测摘要 —— 上水前验视觉与深度链路用）

   各阶段参数仍在 task_config.py（AUV_TASKx_*），遵循「任务参数全部进 task_config」；
   本文件只放测试框架自身的配置（开关 / 表 / 循环 / 回显时长）。

新增测试功能代码：写 task/t_taskX.py（Stage 子类表，封装方式参照 t_task1.py），
在这里 `from task import t_taskX` 后把它的类或整表写进 TEST_TABLE 即可。
"""
import os
import sys

# ---- 路径注入：move_test/ + task/（同 mode_auv / t_task* 约定）----
_HERE = os.path.dirname(os.path.abspath(__file__))          # move_test/test_mode
_MV = os.path.dirname(_HERE)                                 # move_test
_TASK = os.path.join(_MV, 'task')                            # move_test/task
for _p in (_TASK, _MV):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import t_function             # 运动原语与工具（EchoObs 中性保持帧复用）
from mission import Stage     # 测试项基类契约
import task_config as TC      # 任务参数 + 各任务表来源
from task import t_task1, t_task2, t_search_ball, t_task4, t_return   # 各任务脚本的 Stage 类

# task_config 里已 try-import 拼好的整表（引过来方便在 TEST_TABLE 里直接写）
TASK1_TABLE = TC.TASK1_TABLE
TASK2_TABLE = TC.TASK2_TABLE
SEARCH_BALL_TABLE = TC.SEARCH_BALL_TABLE
TASK4_TABLE = TC.TASK4_TABLE
RETURN_TABLE = TC.RETURN_TABLE
DOOR_TABLE = TC.DOOR_TABLE  # 单独测试门任务时设置 TEST_TABLE = DOOR_TABLE
PASS_DOOR_V2_TABLE = TC.PASS_DOOR_V2_TABLE  # 过门 v2（task_config try-import；TEST_TABLE = PASS_DOOR_V2_TABLE 即单测）
HIT_BALL_TABLE = TC.HIT_BALL_TABLE          # 撞球（子目录版 task_hit_ball/t_hit_ball.py，NAME=HitBall）
HIT_BALL_V2_TABLE = TC.HIT_BALL_V2_TABLE    # 撞球 v2（task/ 根目录 t_hit_ball_v2.py，NAME=HitBall_v2）
PICK_BALL_TABLE = TC.PICK_BALL_TABLE        # 捡球（当前为空表 = 进入即 DONE，任务未实现）
PICK_RING_TABLE = TC.PICK_RING_TABLE        # 捡环（纯开环，可无视觉单测）

# ---------------- 测试框架自身配置 ----------------
TEST_MODE_ENABLED = True       # 总开关：True = 切 AUV 即进测试模式；False = 现状不变
TEST_LOOP = False              # True = 测试表跑完自动从头重跑（间隔 0.5s，便于重复观测）
TEST_ECHO_S = 10.0             # EchoObs 观测回显时长(s)；<=0 = 一直跑到切模式


class EchoObs(Stage):
    """内置观测回显测试项：不动船（中性悬停保持），节流打印三条观测链路读数。

    用途：上水前验证 VisionIF（前视检测 JSON）/ DepthIF（融合深度）/ 遥测链路是否在线。
    深度目标贴当前实测（≈悬停不调深）；无遥测时回落 t_function 沿用口径。
    """

    NAME = 'Echo'

    def enter(self, now):
        self.st = {}

    def step(self, now, dt):
        st = self.st
        st.setdefault('t0', now)
        elapsed = now - st['t0']

        # ---- 观测回显（1s 节流）：融合深度 / 前视 ball+gate / 遥测摘要 ----
        d = self.ctx.depth.read(now)                       # DepthIF 永不抛；!ok 时 D=None
        v_ball = self.ctx.vision.poll('front', 'ball', now)
        v_gate = self.ctx.vision.poll('front', 'gate', now)
        tel = self.ctx.tel or {}
        msg = ('Echo t=%.1fs 融合D=%s ball=%s gate=%s | yaw=%.1f° 深度计=%.1fcm'
               % (elapsed,
                  ('%.3fm(ok)' % d['D']) if d.get('ok') and d.get('D') is not None else '无',
                  ('dx=%.0fpx w=%.0f' % (v_ball['dx'], v_ball['w'])) if v_ball else '无',
                  ('dx=%.0fpx' % v_gate['dx']) if v_gate else '无',
                  float(tel.get('actual_yaw', 0.0) or 0.0),
                  float(tel.get('actual_depth_cm', 0.0) or 0.0)))
        t_function._say_throttled(self.ctx, st, now, msg)

        # ---- 到时完成（TEST_ECHO_S<=0 = 一直跑到切模式）----
        if TEST_ECHO_S > 0 and elapsed >= TEST_ECHO_S:
            self.ctx.say('%s 完成：观测回显 %.0fs' % (self.NAME, TEST_ECHO_S))
            return None

        # ---- 中性保持帧：深度贴当前实测（≈悬停）、锁当前航向、零推力 ----
        tel_d = tel.get('actual_depth_cm')
        depth = float(tel_d) if tel_d is not None else t_function._depth_out(st, None)
        yaw = t_function._yaw_hold(st, self.ctx)
        return t_function._cmd(self.NAME, '观测回显',
                               yaw if yaw is not None else 0.0, depth)


# ★★ 测试表：Stage 类列表（单阶段/多阶段/任意组合，写法同 task_config.STAGE_TABLE）★★
# 2026-10-06 仿真：已实现的 5 个任务（Task1/Task2/SearchBall/Task4/Return）整表串联；
# 改这一行即可切测试内容。
# [2026-10-10] 当前目标 = 单测过门 v2；要测别的任务换这一行即可（可用名见上方别名区）。
TEST_TABLE = (PASS_DOOR_V2_TABLE)
