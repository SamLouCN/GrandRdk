# -*- coding: utf-8 -*-
"""t_pick_ball.py —— 捡球任务脚本（空骨架，待实现）

定位：捡球任务（拾取/抓取目标球）。
    ⚠ 空内容占位：本文件只导出空表 PICK_BALL_TABLE = []（空表 = 开机即 DONE 安全停推），
    实现时按 t_task1.py / t_search_ball.py 同构补充 Stage 类并替换表内容。
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                           # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 空骨架：实现后替换为 [PickBallAll]（整体为一个阶段，参照 t_task1.py 同构写法）
PICK_BALL_TABLE = []
