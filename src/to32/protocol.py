# -*- coding: utf-8 -*-
"""上位机协议（PC ↔ 中位机）：文本控制 + v3.7 二进制 PID。

实测依据: /userdata/GrandRDK/src/to32/上位机通讯协议.md（2026-09-11 全量按键实测）

帧型:
  $CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#\r\n   20Hz(有线)/5Hz(无线) 循环
  CD 0A 01 CH KP_LE KI_LE KD_LE DC                         v3.7 PID，11B，增益×100
  $PID,ch,p,i,d#\r\n                                        旧文本 PID，仅保留识别/丢弃
  $VID,1# / $VID,0#\r\n                                     视频开关（板端拦截）
  PING                                                      发往 8081，需回 PONG 到源端口
  $TEL,<41字段>#\r\n                                        中位机 → 上位机
"""
import struct

from link_stm32 import HEAD, TAIL, FUNC_SET_PID, PID_FRAME_LEN, PID_REFERENCE_MAX_INDEX
from task_pid_wire import parse_task_pid, build_task_pid_ack

TEL_FIELD_COUNT = 41   # 上位机索引硬编码：缺字段必须补 0 占位，不能省略


def parse_cmd(frame):
    """$CMD → dict；非 $CMD 或字段不足返回 None"""
    s = frame.strip()
    if not (s.startswith("$CMD,") and s.endswith("#")):
        return None
    try:
        p = s[5:-1].split(",")
        if len(p) < 6:
            return None
        return {
            "surge": float(p[0]),
            "sway": float(p[1]),
            "heave": float(p[2]),
            "yaw": float(p[3]),
            "led1": int(float(p[4])),
            "led2": int(float(p[5])),
            # mode: 缺字段时给出 0，但用 mode_explicit 标记"帧里到底有没有第 7 字段"。
            # [2026-10-04] 待命态(IDLE)只认显式带 mode 的帧，避免老格式被误当成 ROV。
            "mode": int(float(p[6])) if len(p) > 6 else 0,
            "mode_explicit": len(p) > 6,
            "grab": int(float(p[7])) if len(p) > 7 else 0,
            "store": int(float(p[8])) if len(p) > 8 else 0,
        }
    except (ValueError, IndexError):
        return None


def parse_binary_pid(frame):
    """严格解析一条 UDP 数据报中的 V2 PID；不修改原始整数编码。"""
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        return None
    raw = bytes(frame)
    if len(raw) != PID_FRAME_LEN:
        return None
    if raw[:3] != bytes((HEAD, PID_FRAME_LEN - 1, FUNC_SET_PID)) or raw[-1] != TAIL:
        return None
    ch, kp, ki, kd = struct.unpack("<Bhhh", raw[3:-1])
    if ch > PID_REFERENCE_MAX_INDEX:
        return None
    return {"ch": ch, "p": kp / 100.0, "i": ki / 100.0, "d": kd / 100.0}


def parse_pid(frame):
    """$PID → {'ch','p','i','d'}；否则 None"""
    s = frame.strip()
    if not (s.startswith("$PID,") and s.endswith("#")):
        return None
    try:
        p = s[5:-1].split(",")
        if len(p) < 4:
            return None
        return {"ch": int(float(p[0])), "p": float(p[1]),
                "i": float(p[2]), "d": float(p[3])}
    except (ValueError, IndexError):
        return None


def parse_vid(frame):
    """$VID → True/False；否则 None"""
    s = frame.strip()
    if not (s.startswith("$VID,") and s.endswith("#")):
        return None
    try:
        return int(float(s[5:-1])) == 1
    except ValueError:
        return None


def is_ping(frame):
    return frame.strip() == "PING"


def _fmt(x):
    try:
        return "%.2f" % float(x)
    except (TypeError, ValueError):
        return "0.00"


def build_tel(values):
    """按 41 字段组装 $TEL；values 不足自动补 0（不可省略，否则上位机整帧丢弃）"""
    v = [0.0] * TEL_FIELD_COUNT
    for i, x in enumerate(list(values)[:TEL_FIELD_COUNT]):
        v[i] = x
    return "$TEL," + ",".join(_fmt(x) for x in v) + "#\r\n"

