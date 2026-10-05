# -*- coding: utf-8 -*-
"""投放舵机接口 —— 物理投放（高尔夫球）的唯一抽象层

现状（2026-10-04 用户拍板）：**舵机还没实装**。
所以这里先定义接口 + 一个打 WARN 的 Stub，任务代码按接口写死，
等实装后只换 make_servo() 的返回值，任务一行都不用改。

★ 为什么必须走接口而不是直接在任务里写 GPIO：
  0x09 运动帧**没有舵机槽位**（字段只有 pitch/yaw/roll/depth/surge/sway/stick_stop），
  舵机走的是另一条路（STM32 扩展口 / 独立 PWM）。把它封在接口里，
  将来换成"串口扩展帧"或"GPIO"都只在 make_servo() 里改。

幂等性约定：`drop()` 可能被重复调用（任务 tick 是 20Hz 循环），
实现必须保证**只真正触发一次**（用 self.fired 之类的标志）。
"""


class ServoIF(object):
    """舵机接口（抽象基类，只定义契约）"""

    def ready(self):
        """舵机是否可用（False = 没实装/没上电，任务必须走降级路径）

        返回 False 时任务**不许**当作失败：投放分 100 分很重要，
        但"没舵机就整局中止"是不可接受的 —— 应该是照跑 + 打 WARN。
        """
        raise NotImplementedError

    def drop(self):
        """触发投放。幂等：重复调用只触发一次。返回 True = 已发出触发"""
        raise NotImplementedError

    def done(self):
        """投放动作是否已完成（用于日志/上报；没实装时返回 False）"""
        raise NotImplementedError


class ServoStub(ServoIF):
    """占位实现：没实装时的兜底（打 WARN + 假装成功，任务照跑）"""

    def __init__(self, log=None):
        self.log = log or (lambda m: None)   # 日志回调
        self.fired = False                   # 是否已触发（幂等标志）
        self._warned = False                 # WARN 只打一次，别 20Hz 刷屏

    def ready(self):
        """未实装 → 恒 False"""
        return False

    def drop(self):
        """未实装 → 打一条 WARN，返回 True 让流程继续"""
        if not self.fired:
            self.fired = True
            if not self._warned:
                self._warned = True
                self.log('[AUV][SERVO] ⚠ 舵机未实装（ServoStub）—— 投放动作被跳过，任务继续')
        return True

    def done(self):
        """未实装 → 恒 False"""
        return False


class ServoSerial(ServoIF):
    """串口舵机实现**骨架**（未接线，先留结构）

    等 STM32 侧把舵机帧定下来，只要补 `S.frame_servo(...)` 的实现和端口即可；
    本类已经把幂等/只触发一次/失败重试上限这些坑处理好了。
    """
    def __init__(self, log=None, send=None, retry=3):
        self.log = log or (lambda m: None)
        self.send = send                     # 下发回调 fn(frame_bytes) -> bool
        self.retry = int(retry)              # 最多尝试几次
        self.fired = False                   # 幂等：已成功触发过就不再发
        self.tries = 0                       # 已尝试次数

    def ready(self):
        return self.send is not None

    def drop(self):
        if self.fired:                       # 幂等
            return True
        if self.send is None:
            self.log('[AUV][SERVO] 无下发通道，投放失败')
            return False
        self.tries += 1
        ok = False
        try:
            ok = bool(self.send(self.tries))  # 由注入的 send 决定帧内容
        except Exception as e:                # 任何异常都消化在接口内，绝不冒泡到任务
            self.log('[AUV][SERVO] 投放异常: %s' % e)
            ok = False
        if ok:
            self.fired = True
        elif self.tries >= self.retry:
            self.log('[AUV][SERVO] 投放连续 %d 次失败 -> 放弃（任务继续）' % self.tries)
        return ok

    def done(self):
        return self.fired


def make_servo(cfg, log=None):
    """舵机工厂：按配置选实现。目前只有 Stub，实装后在这里加分支

    配置键：
      AUV_SERVO_IMPL = 'stub'（默认） | 'serial'
      AUV_SERVO_SEND = 可调用对象（仅 serial 用；一般由 move_test/mode_auv.py 注入）
    """
    impl = str(getattr(cfg, 'AUV_SERVO_IMPL', 'stub') or 'stub').lower()
    if impl == 'serial':
        return ServoSerial(log=log, send=getattr(cfg, 'AUV_SERVO_SEND', None),
                           retry=int(getattr(cfg, 'AUV_SERVO_RETRY', 3)))
    return ServoStub(log=log)
