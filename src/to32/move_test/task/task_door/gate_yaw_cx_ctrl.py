# -*- coding: utf-8 -*-
"""门框穿越控制 —— 目标航向计算（用 cx）

按下面公式算出目标航向：
    target_yaw = actual_yaw + k_psi_deg * (320 - cx)

    actual_yaw  实际偏航角(deg)，取自 0x0C 遥测回帧
    cx          目标框中心的横向像素坐标，由视觉程序给出
    k_psi_deg   偏航角增益
    target_yaw  目标航向(deg)
"""
from .servo import wrap180, tel_yaw  # 角度归一化到 (-180,180] / 从遥测取实际航向


def calc_target_yaw(actual_yaw, cx, k_psi_deg):  # 目标航向计算
    """目标航向 = 实际航向 + k × (320 − 目标框中心横向像素)"""
    return wrap180(float(actual_yaw) - float(k_psi_deg) * (320.0 - float(cx)))  # 求和后做角度归一化


class GateYawController(object):  # 目标航向计算器
    """把视觉给的目标框中心横向坐标转成目标航向"""

    K_PSI_DEG = -0.1  # ★ 调整 k 值：改这一行的数值

    def step(self, tel, cx):  # 一拍：算目标航向
        """返回本拍算出的目标航向(deg)；条件不满足时返回 None"""
        actual_yaw = tel_yaw(tel)      # 从遥测取实际偏航角
        if actual_yaw is None:         # 取不到实际航向
            return None                # 本拍不输出
        if cx is None:                 # 没有目标框横向坐标
            return None                # 本拍不输出
        target_yaw = calc_target_yaw(actual_yaw, cx, self.K_PSI_DEG)  # 算出目标航向角
        return target_yaw              # 返回目标航向角


# ---- GrandRdk 接线层：以上控制公式、增益和 step 流程保持下载版原样 ----
from stage_base import Stage
from .control_observation import GateControlObservation
from .gate_depth_ctrl import GateDepthController
import os


class GateYawCxTask(Stage):
    """同帧运行下载版 cx 航向和 cy 深度控制；CV Z 角只读取/记录。"""
    NAME = 'PassGate'

    def __init__(self, ctx, observation=None):
        super().__init__(ctx)
        shm_dir = (getattr(getattr(ctx, 'vision', None), 'shm_dir', None)
                   or getattr(ctx.cfg, 'AUV_SHM_DIR', None)
                   or os.environ.get('GRDK_SHM_DIR') or '/dev/shm')
        self.observation = observation or GateControlObservation(shm_dir)
        self.controller = GateYawController()
        self.depth_controller = GateDepthController()

    def enter(self, now):
        self.observation.reset()
        self.last_observation = None
        self.last_log_at = None
        self.ctx.say('PassGate: cx/cy 航向+深度测试，k_yaw=%.3f k_depth=%.3f；surge=sway=0；CV Z仅观测'
                     % (self.controller.K_PSI_DEG, self.depth_controller.K_DEPTH))

    def step(self, now, dt):
        sample = self.observation.read(now)
        self.last_observation = sample
        if sample is None:
            return self._wait('等当前有效门框观测')
        if not sample['fresh']:
            return self._wait('等新视觉帧；不重复计算航向和深度目标')
        tel = self.ctx.tel
        target_yaw = self.controller.step(tel, sample['cx'])
        target_depth = self.depth_controller.step(tel, sample['cy'])
        if target_yaw is None or target_depth is None:
            return self._wait('等实际航向/深度遥测或目标框中心；本拍不下发')
        z_text = 'unavailable' if sample['z_deg'] is None else '%+.3fdeg' % sample['z_deg']
        note = 'frame=%s cx=%.2f cy=%.2f CV_Z=%s target_yaw=%+.2fdeg target_depth=%.2fcm' % (
            sample['frame'], sample['cx'], sample['cy'], z_text, target_yaw, target_depth)
        if self.last_log_at is None or now-self.last_log_at >= 1.0:
            self.ctx.say('PassGate: ' + note)
            self.last_log_at = now
        # Mission/TestMode apply the existing firmware yaw mirror exactly once.
        # This controller has no completion criterion; test ends on mode exit.
        return dict(stage=self.NAME, note=note, yaw=target_yaw, depth=target_depth,
                    surge=0.0, sway=0.0, stop=0)

    def _wait(self, reason):
        # None means stage complete to Mission, so represent the controller's
        # "no output this tick" with the existing paused command contract.
        return dict(stage=self.NAME, note=reason, paused=True)


DOOR_TABLE = [GateYawCxTask]
