"""下载版控制器依赖的接口适配；直接读取实际航向与深度遥测。"""
import math


def wrap180(angle):
    """归一化到 (-180, 180]，与下载版接口约定一致。"""
    result = (float(angle) + 180.0) % 360.0 - 180.0
    return 180.0 if result == -180.0 else result


def tel_yaw(tel):
    """从已解码的 0x0C 遥测取实际航向；无有效遥测返回 None。"""
    try:
        value = float(tel['actual_yaw'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def tel_depth_cm(tel):
    """从已解码的 0x0C 遥测取实际深度（cm）；无有效遥测返回 None。"""
    try:
        value = float(tel['actual_depth_cm'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None
