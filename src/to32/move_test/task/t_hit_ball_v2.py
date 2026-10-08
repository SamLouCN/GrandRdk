
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                          # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC
from mission import Stage
import t_function

class HitBallAll(Stage):

    NAME = "HitBall_v2"

    #======Step1. Sway to find the ball==============
    '''
    与上一级任务（task1）衔接，完成前进后，开始撞球任务
    第一步：
    左右摇摆，相对yaw的角度，幅度为±15°，且并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度，

    退出机制：
    一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
    '''
    def step1_find_ball(self):
        self.log("Step1. Sway to find the ball") #
        # 1. 左右摇摆，幅度为±15°，并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度
        # 2. 一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
        yaw_naw = self.get_yaw_naw()
        



# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个撞球任务；空表 = 开机即 DONE）
HIT_BALL_TABLE = [HitBallAll]