#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
read_altimeter.py — 多路持续读取 DYP-L08(-V3.0) 超声波高度计（Modbus-RTU）

硬件接线：
  CH348 A–E 口 <-> 各高度计 UART(TTL 5V)，板端设备：
    A -> /dev/ttyCH9344USB0   B -> /dev/ttyCH9344USB1   C -> /dev/ttyCH9344USB2
    D -> /dev/ttyCH9344USB3   E -> /dev/ttyCH9344USB4   （A–H 顺序对应 USB0–7）

设计要点（每口完全独立）：
  - 每个通道 = 一条独立线程 + 独立串口句柄 + 独立统计，互不阻塞
  - 某口未接设备：打印一次提示后降为 2s 低频探测，不影响其它通道；
    接上高度计后自动恢复正常读取节奏
  - 某口打开失败（节点不存在）：仅该通道跳过，其余照常运行
  - 5 口中任意只有 1 个也能正常运行

协议（详见《DYP-RD 产品规格书 L08-V3.0》3.3 节）：
  Modbus-RTU, CRC-16/MODBUS, 默认 115200-8N1, 从机地址默认 0x01
  常用只读寄存器（单位 mm）：0x0100 处理值(≈135ms) / 0x0101 实时值(≈20ms, 默认)
  输出统一使用 mm 原生单位，不再换算 m/cm。

用法：
  python3 read_altimeter.py                          # 默认读 A–E 五口，每 0.2s
  python3 read_altimeter.py --channels B             # 只读 B 口（单口模式）
  python3 read_altimeter.py --channels A,C,E         # 只读指定口
  python3 read_altimeter.py --reg 0x0100 --interval 0.3   # 改读处理值
  python3 read_altimeter.py --addr 0x02              # 各口从机地址都改为 0x02
  Ctrl-C 结束并打印各通道汇总（有效/失败/断开/min~max/avg）
"""
import argparse
import binascii
import socket
import struct
import sys
import threading
import time

import serial

# ------------------------- 协议常量 -------------------------
BAUD_DEFAULT = 115200          # 手册默认波特率（寄存器 0x0201 = 0x09）
ADDR_DEFAULT = 0x01            # 手册默认从机地址
REG_DEFAULT = 0x0101           # 实时值（≈20ms 响应）；0x0100 为处理值(≈135ms)
FUNC_READ = 0x03               # Modbus 读保持寄存器功能码

# CH348 物理口 -> 板端 tty 节点（A–H 顺序对应 USB0–7；本脚本只用到 A–E）
PORT_OF = {ch: f'/dev/ttyCH9344USB{i}' for i, ch in enumerate('ABCDEFGH')}

# 连续多少次无响应后判定该口"未接设备"，进入降频探测（2s），恢复后自动回到正常节奏
NO_REPLY_SWITCH = 10
SLOW_PROBE_INTERVAL = 2.0
# v3.2.1: UDP 推送(PC 上位机"全部"页高度计显示区消费)
ALT_PORT_DEFAULT = 8082
PC_IP_DEFAULT = "192.168.127.100"   # 有线直连拓扑中上位机 PC 的 IP(与 vp5.0 telem_sender 一致)


# ------------------------- 工具函数 -------------------------

def modbus_crc16(data: bytes) -> bytes:
    """计算 Modbus-RTU CRC-16（多项式 0xA001，初值 0xFFFF），返回低字节在前的 2 字节。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


def build_read_frame(addr: int, reg: int, count: int = 1) -> bytes:
    """构造 Modbus 读保持寄存器请求：地址+功能码+寄存器地址(2B)+数量(2B)+CRC(2B)。"""
    body = bytes([addr, FUNC_READ]) + struct.pack('>HH', reg, count)
    return body + modbus_crc16(body)


def parse_response(resp: bytes, addr: int):
    """校验并解析从机响应，返回原始值(int)或 None（长度/地址/功能码/字节数/CRC 任一不符）。"""
    if len(resp) < 7:                      # 最短合法响应 = 1+1+1+2+2 = 7 字节
        return None
    if resp[0] != addr or resp[1] != FUNC_READ or resp[2] != 2:
        return None
    if modbus_crc16(resp[:-2]) != resp[-2:]:
        return None
    return int.from_bytes(resp[3:5], 'big', signed=True)   # 高字节在前（手册备注 1）


def make_alt_frame(ch: str, mm, status: str) -> bytes:
    """构建上位机显示帧: $ALT,<通道>,<mm>,<状态># （与上位机 protocol.py 的 parse_alt 对应）
    mm 为 None 时输出空值; 状态: OK / EMPTY / ERR / DOWN"""
    v = '' if mm is None else str(int(mm))
    return f'$ALT,{ch},{v},{status}#\r\n'.encode()


# ------------------------- 单通道统计 -------------------------

class ChanStat:
    """单个通道的读取统计（各通道独立维护，主线程 Ctrl-C 后统一打印）。"""

    def __init__(self, name: str):
        self.name = name
        self.open_ok = False       # 串口是否成功打开
        self.no_device = False     # 是否已被判定为"未接设备"（降频探测中）
        self.ok_n = 0              # 有效帧数
        self.fail_n = 0            # 失败帧数（超时/无响应/CRC 错）
        self.vals = []             # 有效原始值（mm）
        self.lock = threading.Lock()

    def note_fail(self):
        with self.lock:
            self.fail_n += 1

    def note_ok(self, v: int):
        with self.lock:
            self.ok_n += 1
            self.vals.append(v)


# ------------------------- 通道读取线程 -------------------------

def channel_loop(name: str, args, stop: threading.Event, st: ChanStat, udp=None):
    """单个通道的读取循环（线程体）：打开串口 -> 按节奏发查询 -> 校验并打印。

    独立性保证：
      - 串口打开失败：打印提示后直接结束本线程（不影响其它线程）
      - 连续 NO_REPLY_SWITCH 次无响应：判定未接设备，降为 SLOW_PROBE_INTERVAL
        低频探测；一旦恢复（收到有效帧）立刻回到正常 interval 节奏
    """
    dev = PORT_OF[name]
    try:
        ser = serial.Serial(dev, args.baud, timeout=0.3)
        st.open_ok = True
    except Exception as exc:
        print(f'[!!] [{name}] 打开失败 {dev}: {exc!r} —— 该通道跳过，不影响其它通道', flush=True)
        if udp:
            udp.sendto(make_alt_frame(name, None, 'DOWN'), args.pc_addr)
        return

    frame = build_read_frame(args.addr, args.reg)
    resp_wait = max(0.05, args.interval * 0.5)   # 读超时须大于从机响应时间
    consec_fail = 0
    print(f'[*] [{name}] {dev} @ {args.baud} 8N1 就绪（addr=0x{args.addr:02X} '
          f'reg=0x{args.reg:04X}，请求 {binascii.hexlify(frame).decode()}）', flush=True)
    try:
        while not stop.is_set():
            ser.reset_input_buffer()             # 清旧字节，避免残留帧被误当响应
            ser.write(frame)
            ser.flush()
            deadline = time.time() + resp_wait
            resp = b''
            while time.time() < deadline and len(resp) < 9:
                chunk = ser.read(64)
                if chunk:
                    resp += chunk

            raw = parse_response(resp, args.addr)
            if raw is None:
                consec_fail += 1
                st.note_fail()
                if consec_fail == NO_REPLY_SWITCH:
                    st.no_device = True
                    print(f'[!!] [{name}] 连续 {NO_REPLY_SWITCH} 次无响应，判定该口未接高度计；'
                          f'降为 {SLOW_PROBE_INTERVAL}s 低频探测，接上设备后自动恢复', flush=True)
                    if udp:
                        udp.sendto(make_alt_frame(name, None, 'EMPTY'), args.pc_addr)
                if consec_fail == 1:
                    print(f'[!!] [{name}] 读取失败(no-reply): {binascii.hexlify(resp).decode()!r}', flush=True)
            else:
                if consec_fail >= NO_REPLY_SWITCH:
                    # 降频探测期间收到有效帧 -> 设备已接上，恢复正常节奏
                    consec_fail = 0
                    st.no_device = False
                    print(f'[*] [{name}] 设备恢复，回到正常读取节奏', flush=True)
                else:
                    consec_fail = 0
                st.note_ok(raw)
                print(f'[OK] [{name}] {time.strftime("%H:%M:%S")} '
                      f'raw=0x{raw:04X} -> {raw} mm', flush=True)
                if udp:
                    udp.sendto(make_alt_frame(name, raw, 'OK'), args.pc_addr)

            # 调度：未接设备时慢探，正常时按 interval
            sleep_s = SLOW_PROBE_INTERVAL if consec_fail >= NO_REPLY_SWITCH else args.interval
            if sleep_s > 0:
                stop.wait(sleep_s)
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ------------------------- 汇总打印 -------------------------

def print_summary(addr, reg, ch_stats, frame_hex):
    """打印所有通道的最终统计表（按 通道名 排序，便于对照物理口）。"""
    print('\n==================== 汇总 ====================')
    print(f'请求帧(hex): {frame_hex} | addr=0x{addr:02X} reg=0x{reg:04X} '
          f'({"实时值" if reg == 0x0101 else "处理值" if reg == 0x0100 else "自定义"})')
    print(f'{"通道":<4}{"设备节点":<26}{"状态":<10}{"有效/失败":<12}数值范围(mm)')
    print('-' * 68)
    for st in ch_stats:
        dev = PORT_OF[st.name]
        if not st.open_ok:
            status, val = '打开失败', '-'
        elif st.no_device:
            status, val = '未接设备', '-'
        elif st.ok_n:
            status = '正常'
            val = f'{min(st.vals)}~{max(st.vals)} (avg {sum(st.vals) / len(st.vals):.1f})'
        else:
            status, val = '无数据', '-'
        print(f'{st.name:<4}{dev:<26}{status:<10}{st.ok_n}/{st.fail_n:<8}{val}')
    print('===============================================')


# ------------------------- 主流程 -------------------------

def main():
    ap = argparse.ArgumentParser(description='多路持续读取 DYP-L08 超声波高度计 (Modbus-RTU, A–E 口独立)')
    ap.add_argument('--channels', default='A,B,C,D,E',
                    help='要读取的通道（逗号分隔，默认 A,B,C,D,E；例如 B 或 A,C,E）')
    ap.add_argument('--baud', type=int, default=BAUD_DEFAULT, help=f'波特率（默认 {BAUD_DEFAULT}）')
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=ADDR_DEFAULT,
                    help=f'Modbus 从机地址（默认 0x{ADDR_DEFAULT:02X}，可用 0x01 或 1 写法）')
    ap.add_argument('--reg', type=lambda x: int(x, 0), default=REG_DEFAULT,
                    help=f'寄存器地址（默认 0x{REG_DEFAULT:04X}=实时值; 0x0100=处理值）')
    ap.add_argument('--interval', type=float, default=0.2,
                    help='读取间隔秒（默认 0.2；须大于从机响应时间：实时值~20ms，处理值~135ms）')
    ap.add_argument('--pc-ip', default=PC_IP_DEFAULT,
                    help=f'上位机 PC 的 IP（默认 {PC_IP_DEFAULT}，UDP 推送目标；配合 PC 端 8082 监听）')
    ap.add_argument('--pc-port', type=int, default=ALT_PORT_DEFAULT,
                    help=f'上位机监听端口（默认 {ALT_PORT_DEFAULT}）')
    ap.add_argument('--no-udp', action='store_true', help='关闭 UDP 推送（纯终端模式）')
    args = ap.parse_args()
    args.pc_addr = None if args.no_udp else (args.pc_ip, args.pc_port)

    names = [c.strip().upper() for c in args.channels.split(',') if c.strip().upper() in PORT_OF]
    if not names:
        print(f'[!] 没有有效通道（可选 A–H），输入: {args.channels}')
        return 1

    frame_hex = binascii.hexlify(build_read_frame(args.addr, args.reg)).decode()
    udp = None if args.no_udp else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f'[*] 高度计多路读取: {len(names)} 通道 {names} @ {args.baud} 8N1 | '
          f'addr=0x{args.addr:02X} reg=0x{args.reg:04X} | 每通道间隔 {args.interval}s | '
          f'请求帧={frame_hex}', flush=True)
    if udp:
        print(f'[*] UDP 推送已开启: {args.pc_ip}:{args.pc_port} ($ALT 帧 → PC 上位机"全部"页)', flush=True)
    else:
        print('[*] UDP 推送已关闭 (--no-udp)', flush=True)

    stop = threading.Event()
    stats = [ChanStat(n) for n in names]
    threads = [threading.Thread(target=channel_loop, args=(n, args, stop, st, udp), daemon=True)
               for n, st in zip(names, stats)]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(0.5)
            if not any(t.is_alive() for t in threads):   # 所有通道都结束（如全部打开失败）
                break
    except KeyboardInterrupt:
        print('\n[!] Ctrl-C 中断，正在汇总…', flush=True)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=1.0)

    if udp:
        try:
            udp.close()
        except Exception:
            pass

    print_summary(args.addr, args.reg, stats, frame_hex)
    return 0 if any(s.open_ok and s.ok_n for s in stats) else 1


if __name__ == '__main__':
    sys.exit(main())
