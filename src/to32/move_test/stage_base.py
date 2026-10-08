"""无配置依赖的阶段契约和输出坐标工具，避免任务注册循环导入。"""


def apply_yaw_mirror(yaw, mirror):
    """抵消固件航向取负，并归一化到 [-180,180]；只在输出侧调用。"""
    y = -float(yaw) if mirror else float(yaw)
    while y > 180.0:
        y -= 360.0
    while y < -180.0:
        y += 360.0
    return y


class Stage:
    NAME = '?'

    def __init__(self, ctx):
        self.ctx = ctx

    def enter(self, now):
        """进入阶段，初始化持久状态。"""

    def step(self, now, dt):
        """返回控制字典；None 表示本阶段完成。"""
        raise NotImplementedError
