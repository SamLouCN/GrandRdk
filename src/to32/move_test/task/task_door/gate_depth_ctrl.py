# -*- coding: utf-8 -*-
"""门框穿越控制 —— 第 2 步：目标深度计算

按下面公式算出目标深度：
    target_depth_cm = actual_depth_cm + k_depth * (240 - cy)

    actual_depth_cm  实际深度(cm)，取自 0x0C 遥测回帧
    cy               目标框中心的纵向像素坐标，由视觉程序给出
    k_depth          深度增益
    target_depth_cm  目标深度(cm)
"""
from .servo import tel_depth_cm  # 从遥测取实际深度


def calc_target_depth(actual_depth_cm, cy, k_depth):  # 目标深度计算
    """目标深度 = 实际深度 + k × (240 − 目标框中心纵向像素)"""
    return float(actual_depth_cm) - float(k_depth) * (240.0 - float(cy))  # 按公式求和


class GateDepthController(object):  # 目标深度计算器
    """把视觉给的目标框中心纵向坐标转成目标深度"""

    K_DEPTH = -3  # ★ 调整 k 值：改这一行的数值

    def step(self, tel, cy):  # 一拍：算目标深度
        """返回本拍算出的目标深度(cm)；条件不满足时返回 None"""
        actual_depth_cm = tel_depth_cm(tel)  # 从遥测取实际深度
        if actual_depth_cm is None:          # 取不到实际深度
            return None                      # 本拍不输出
        if cy is None:                       # 没有目标框纵向坐标
            return None                      # 本拍不输出
        target_depth_cm = calc_target_depth(actual_depth_cm, cy, self.K_DEPTH)  # 算出目标深度
        return target_depth_cm               # 返回目标深度
