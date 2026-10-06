# -*- coding: utf-8 -*-
"""V2 遥测 -> 上位机 $TEL（41 字段）的统一映射。

为什么单独一个文件：
  $TEL 的 41 个索引是上位机**硬编码**的（见《上位机通讯协议.md》），谁都不该各写一份；
  这里给出唯一一份「V2 遥测字段 -> 上位机索引」映射，模式里没有特殊需求时直接用本模块。

输入 = link_stm32.parse_telemetry() 的 dict；输出 = 可直接 sendto 的 $TEL 文本帧。
V2 没有的字段（线速度 vx/vy/vz、目标角速度、电池/舱温、高度 alt、9~12 路电机）一律补 0 占位，
**不能省略**，否则上位机整帧错位。

v3.6 追加（2026-10-06）：append_fusion_fields() —— 把 depth_kalman 的融合深度/离底净空
  （/dev/shm/momo_depth.json）追加到 $TEL 文本帧**尾部**（索引 41=融合深度 m、42=离底净空 m）。
  旧 41 字段一字不动；上位机按序号取值，尾部多出的字段向后兼容（v3.1 的 ax/ay/az/alt 先例）。
  融合数据无效/超期/缺失时**原样返回**，绝不影响 $TEL 主链。挂载点：mode_dispatcher._send_tel。
"""
from __future__ import annotations

import json
import os
import time

import protocol as P

try:
    import to32_config as C
    MOTOR_SCALE = float(getattr(C, "MOTOR_SCALE", 1000.0)) or 1000.0
except Exception:
    MOTOR_SCALE = 1000.0

TEL_FIELD_COUNT = int(getattr(P, "TEL_FIELD_COUNT", 41))


# ---------------------------------------------------------------------- #
# v3.6 融合值上屏：$TEL 帧尾追加 depth_kalman 融合深度/离底净空
# ---------------------------------------------------------------------- #
FUSION_PATH = "/dev/shm/momo_depth.json"   # depth_kalman 原子写，~20Hz
FUSION_STALE_S = 1.0                       # mtime 超期（写端挂了）→ 不追加
_fusion_cache = {"mtime": None, "obj": None}


def _read_fusion():
    """mtime 缓存读 momo_depth.json -> (obj, mtime)；失败 -> (None, None)。

    写端 ~20Hz 覆盖写、组帧 ~10-20Hz 直读会撞上半截 JSON ——
    stat 命中同一 mtime 直接复用上次解析结果，变了才重读；
    读失败本拍放弃、缓存保留（下一拍 mtime 变了自然会重读）。
    """
    try:
        st = os.stat(FUSION_PATH)
    except OSError:
        _fusion_cache["mtime"] = None
        _fusion_cache["obj"] = None
        return None, None
    m = st.st_mtime
    if m == _fusion_cache["mtime"] and _fusion_cache["obj"] is not None:
        return _fusion_cache["obj"], m
    try:
        with open(FUSION_PATH, "r") as f:
            obj = json.load(f)
    except Exception:
        return None, None
    if not isinstance(obj, dict):
        return None, None
    _fusion_cache["mtime"] = m
    _fusion_cache["obj"] = obj
    return obj, m


def fusion_ext_fields(now=None):
    """融合结果 -> [融合深度m, 离底净空m]；无效/超期/缺失 -> None。

    - 净空口径与 move_test/obs.py 的 DepthIF 一致：各通道有效值取最小
    - 永不抛异常：组帧路径上的任何抖动都不许影响 $TEL 主链
    """
    obj, mtime = _read_fusion()
    if obj is None or not obj.get("valid"):
        return None
    if abs((time.time() if now is None else now) - mtime) > FUSION_STALE_S:
        return None
    try:
        d = float(obj.get("D"))
    except (TypeError, ValueError):
        return None
    if d != d or abs(d) > 100.0:          # NaN / 明显离谱值
        return None
    vals = []
    for v in (obj.get("clearance") or {}).values():
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if fv == fv and fv > -1e-9:       # 排除 NaN / 负值
            vals.append(fv)
    out = [d]
    if vals:
        out.append(min(vals))
    return out


def append_fusion_fields(frame, now=None):
    """$TEL 文本帧尾部追加融合字段（41 -> 42/43）；非 $TEL / 无有效融合 -> 原样返回。

    索引 41=融合深度(m)、42=离底净空(m)；净空拿不到时只追加深度（PC 端按帧长守卫）。
    """
    s = (frame or "").rstrip("\r\n")
    if not (s.startswith("$TEL,") and s.endswith("#")):
        return frame
    ext = fusion_ext_fields(now)
    if not ext:
        return frame
    return s[:-1] + "," + ",".join("%.2f" % v for v in ext) + "#\r\n"


def tel_values(tel):
    """V2 遥测 dict -> 41 个数值（顺序即上位机索引）；无遥测返回 None。"""
    if not tel:
        return None
    f = [0.0] * TEL_FIELD_COUNT

    def put(idx, key, scale=1.0):
        try:
            f[idx] = float(tel.get(key, 0.0)) / scale
        except (TypeError, ValueError):
            f[idx] = 0.0

    # 0-5  实际姿态 + 角速度（注意上位机顺序是 roll, pitch, yaw）
    put(0, "actual_roll")
    put(1, "actual_pitch")
    put(2, "actual_yaw")
    put(3, "gyro_roll")
    put(4, "gyro_pitch")
    put(5, "gyro_yaw")
    # 6    实际深度(m)
    put(6, "actual_depth_cm", 100.0)
    # 7-9  vx/vy/vz：V2 无（后续接光流/多普勒时在此填）
    # 10-12 目标姿态
    put(10, "target_roll")
    put(11, "target_pitch")
    put(12, "target_yaw")
    # 13-15 目标角速度：V2 无
    # 16   目标深度(m)
    put(16, "target_depth_cm", 100.0)
    # 17-19 目标线速度：V2 无
    # 20-24 电池电压/电流/SOC/温度/舱温：V2 无
    # 25-32 推进器 thr0..thr7（V2 电机原值 -> [-1,1]）
    motors = tel.get("motors") or []
    for i in range(min(8, len(motors))):
        try:
            f[25 + i] = float(motors[i]) / MOTOR_SCALE
        except (TypeError, ValueError):
            f[25 + i] = 0.0
    # 33-36 thr8..thr11：V2 只有 8 路
    # 37-39 加速度
    put(37, "acc_x")
    put(38, "acc_y")
    put(39, "acc_z")
    # 40    alt：由高度计（USART / UDP 8082）单独提供，V2 无
    return f


def build_tel(tel):
    """-> '$TEL,...,#\\r\\n' 文本帧；无遥测返回 None。"""
    vals = tel_values(tel)
    if vals is None:
        return None
    return P.build_tel(vals)