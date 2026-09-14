#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telem_sender.py — 板端遥测滚动上传 (对应 ROV 控制站 protocol.py 的 $TEL v3.1)

功能:
  1) 以 10Hz 向 PC(默认 192.168.127.100:8081) 单播 $TEL v3.1 帧,
     41 字段全部滚动变化(AI 生成的模拟参数: 姿态/角速度/深度/目标量/电池/12路推进器/加速度/高度)
  2) 在板端监听 UDP 8081, 响应上位机的 PING -> PONG (供 GUI RTT 延迟显示)

用法: python3 telem_sender.py [--dst 192.168.127.100] [--port 8081] [--hz 10]
帧格式与 x5_server.py 遥测模拟完全兼容, PC 端 parse_telemetry 可直接解析。
"""
import argparse
import math
import random
import socket
import struct
import time

# ====================== 端口 ======================
TELEM_PORT = 8081

# ====================== 帧格式 ======================
# $TEL,roll,pitch,yaw,gx,gy,gz,depth,vx,vy,vz,          (10 实际量)
#      t_roll,t_pitch,t_yaw,t_gx,t_gy,t_gz,t_depth,
#      t_vx,t_vy,t_vz,                                 (10 目标量)
#      batt_v,batt_a,batt_soc,batt_temp,cabin_temp,     (5 电池/温度)
#      thr0..thr11,                                     (12 推进器)
#      ax,ay,az,alt#                                    (4 v3.1 加速度/高度)
N_FIELDS = 41


class Rolling:
    """滚动参数源: 慢波 + 噪声 + 可设定基线, 让数据'真实地动起来'."""

    def __init__(self, base, amp, period, noise=0.02, phase=None):
        """初始化滚动参数源：基线(base) + 正弦波动(amp/period) + 均匀噪声(noise)。phase 缺省时随机取 0~2pi，保证多个实例波形互不相同。"""
        self.base = base
        self.amp = amp
        self.period = period
        self.noise = noise
        self.phase = phase if phase is not None else random.uniform(0, 6.28)

    def at(self, t):
        """取 t 时刻的模拟值: 基线 + 慢正弦 + 均匀噪声(-noise..noise)。"""
        v = self.base + self.amp * math.sin(2 * math.pi * t / self.period + self.phase)
        return v + random.uniform(-self.noise, self.noise)


def build_frame(t):
    """按 protocol.py v3.1 字段顺序生成 $TEL 帧 (41 字段)."""
    # --- 实际量: 姿态角/角速度/深度/线速度 (10) ---
    roll = Rolling(2.0, 6.0, 8.0).at(t)
    pitch = Rolling(-1.5, 4.0, 11.0).at(t)
    yaw = Rolling(135.0, 90.0, 25.0).at(t)
    gx = Rolling(0.0, 20.0, 4.0).at(t)
    gy = Rolling(0.0, 15.0, 5.0).at(t)
    gz = Rolling(0.0, 12.0, 6.0).at(t)
    depth = Rolling(2.5, 0.8, 20.0).at(t)
    vx = Rolling(0.0, 0.4, 7.0).at(t)
    vy = Rolling(0.0, 0.3, 9.0).at(t)
    vz = Rolling(0.0, 0.25, 5.0).at(t)
    actual = [roll, pitch, yaw, gx, gy, gz, depth, vx, vy, vz]
    # --- 目标量 (10) ---
    t_roll = Rolling(2.0, 6.0, 8.0, phase=1.0).at(t)
    t_pitch = Rolling(-1.5, 4.0, 11.0, phase=1.3).at(t)
    t_yaw = Rolling(135.0, 90.0, 25.0, phase=1.6).at(t)
    t_gx = Rolling(0.0, 20.0, 4.0, phase=0.4).at(t)
    t_gy = Rolling(0.0, 15.0, 5.0, phase=0.7).at(t)
    t_gz = Rolling(0.0, 12.0, 6.0, phase=1.1).at(t)
    t_depth = Rolling(2.5, 0.8, 20.0, phase=0.9).at(t)
    t_vx = Rolling(0.0, 0.4, 7.0, phase=1.4).at(t)
    t_vy = Rolling(0.0, 0.3, 9.0, phase=0.5).at(t)
    t_vz = Rolling(0.0, 0.25, 5.0, phase=2.0).at(t)
    target = [t_roll, t_pitch, t_yaw, t_gx, t_gy, t_gz, t_depth, t_vx, t_vy, t_vz]
    # --- 电池/温度 (5) ---
    batt_v = Rolling(24.5, 0.3, 60.0).at(t)
    batt_a = Rolling(3.2, 1.5, 12.0).at(t)
    batt_soc = Rolling(87.0, 4.0, 45.0).at(t)
    batt_temp = Rolling(34.0, 1.5, 40.0).at(t)
    cabin_temp = Rolling(28.0, 1.0, 35.0).at(t)
    batt = [batt_v, batt_a, batt_soc, batt_temp, cabin_temp]
    # --- 12 路推进器油门 [-1,1] (12) ---
    thrusters = [max(-1.0, min(1.0, Rolling(0.0, 0.55, 6.0 + i * 0.7).at(t)))
                 for i in range(12)]
    # --- v3.1 加速度/高度 (4) ---
    ax = Rolling(0.0, 0.9, 5.0).at(t)
    ay = Rolling(0.0, 0.7, 6.0).at(t)
    az = Rolling(9.8, 0.4, 30.0).at(t)
    alt = Rolling(0.0, 0.6, 18.0).at(t)
    acc = [ax, ay, az, alt]

    all_vals = actual + target + batt + thrusters + acc
    assert len(all_vals) == N_FIELDS, len(all_vals)
    body = ",".join(f"{v:.2f}" for v in all_vals)
    return f"$TEL,{body}#\r\n"


def ping_responder():
    """监听 8081, 收到 b'PING' 回 b'PONG' (与 x5_server.ping_responder 一致)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", TELEM_PORT))
    s.settimeout(0.5)
    print(f"[PONG] 监听 UDP :{TELEM_PORT} (PING->PONG)", flush=True)
    while True:
        try:
            data, addr = s.recvfrom(64)
            if data == b"PING":
                s.sendto(b"PONG", addr)
        except socket.timeout:
            continue
        except OSError:
            time.sleep(0.2)


def main():
    """CLI 入口：解析 --dst/--port/--hz，启动 PING 应答并在 10Hz 循环发包。

    参数:
      --dst    上位机 IP（板端遥测 UDP 目标）
      --port   上位机遥测端口（默认 8081，与 ROV 控制站 protocol.py 对齐）
      --hz     发送频率；--no-pong 可关闭 PING 应答线程
    """
    ap = argparse.ArgumentParser(description='vp5.0 遥测滚动上传 (协议兼容 x5_server)')
    ap.add_argument('--dst', default='192.168.127.100', help='上位机 IP')
    ap.add_argument('--port', type=int, default=TELEM_PORT, help='上位机遥测端口')
    ap.add_argument('--hz', type=float, default=10.0, help='发送频率')
    ap.add_argument('--no-pong', action='store_true', help='不启动 PING 应答')
    args = ap.parse_args()

    if not args.no_pong:
        import threading
        threading.Thread(target=ping_responder, daemon=True).start()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / max(0.5, args.hz)
    print(f'[TEL] 滚动上传 -> {args.dst}:{args.port} @ {args.hz:.1f}Hz '
          f'($TEL v3.1, {N_FIELDS} 字段)', flush=True)
    t0 = time.time()
    n = 0
    try:
        while True:
            frame = build_frame(time.time() - t0)
            s.sendto(frame.encode(), (args.dst, args.port))
            n += 1
            if n % 100 == 0:
                print(f'[TEL] 已发送 {n} 帧', flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f'[TEL] 停止, 共发送 {n} 帧', flush=True)


if __name__ == '__main__':
    main()