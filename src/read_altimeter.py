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
import argparse  # 命令行参数解析，用于 --channels/--baud/--reg 等覆盖项
import binascii  # 帧内容转十六进制，便于日志打印请求帧与原始响应
import json  # [AUV-MISSION 2026-09-26] 落盘 momo_alt.json 供 depth_kalman 消费
import os  # [AUV-MISSION 2026-09-26] 共享内存目录与原子写
import socket  # UDP 套接字，向 PC 上位机推送 $ALT 帧
import struct  # 大/小端打包，用于构造 Modbus 帧与 CRC
import sys  # 取 main() 返回值作为进程退出码
import threading  # 每通道一条独立线程，另加全局停止事件
import time  # 响应超时判定、间隔等待与时间戳

import serial  # pyserial，读写 CH348 各口对应的 tty 节点

# ------------------------- 协议常量 -------------------------
BAUD_DEFAULT = 115200          # 手册默认波特率（寄存器 0x0201 = 0x09）
ADDR_DEFAULT = 0x01            # 手册默认从机地址
REG_DEFAULT = 0x0101           # 实时值（≈20ms 响应）；0x0100 为处理值(≈135ms)
FUNC_READ = 0x03               # Modbus 读保持寄存器功能码

# CH348 物理口 -> 板端 tty 节点（A–H 顺序对应 USB0–7；本脚本只用到 A–E）
PORT_OF = {ch: f'/dev/ttyCH9344USB{i}' for i, ch in enumerate('ABCDEFGH')}  # 通道名到串口节点的映射表，A 对应 USB0

# 连续多少次无响应后判定该口"未接设备"，进入降频探测（2s），恢复后自动回到正常节奏
NO_REPLY_SWITCH = 10  # 连续 10 次无响应才判定未接设备，避免偶发丢帧误判
SLOW_PROBE_INTERVAL = 2.0  # 降频探测间隔(秒)，判定未接设备后按此节奏重试
# v3.2.1: UDP 推送(PC 上位机"全部"页高度计显示区消费)
ALT_PORT_DEFAULT = 8082  # UDP 推送目标端口，PC 上位机在 8082 监听
PC_IP_DEFAULT = "192.168.127.100"   # 有线直连拓扑中上位机 PC 的 IP(与 vp5.0 telem_sender 一致)

# ==================== [AUV-MISSION 2026-09-26 新增段] 共享内存落盘 ====================
# 目的：depth_kalman 读 momo_alt.json 作为高度计输入源（此前本脚本只往 PC 推 UDP $ALT，
#       板端没有任何落盘 → 深度卡尔曼拿不到数据 → AUV 坐底/定深判据无米下锅）。
# 格式（与 depth_kalman/config/depth_config.py 的 SHM_ALT 约定一致）:
#   {"ts": 1790133164.5, "ch": {"B": {"mm": 534, "status": "OK"}, "C": {"mm": 536, "status": "OK"}}}
#   * mm = 探头面到池底的**净空**（不是深度）；status 只认 "OK"，EMPTY/ERR/DOWN 会被丢弃
#   * 建议全部口都落盘，谁进观测由 depth_kalman 的 ALT_CHANNELS 决定
ALT_SHM_FILE = 'momo_alt.json'  # 落盘文件名（写在 ALT_SHM_DIR 下）
ALT_SHM_DIR = '/dev/shm'        # 默认落盘目录；空串/None 或 --no-shm 即关闭
_ALT_LATEST = {}                # ch -> {'mm':..,'status':..}，全局快照（所有通道合并写）
_ALT_LOCK = threading.Lock()    # 多线程并发保护：每个通道一条线程
_ALT_ERR_LOGGED = False         # 写盘失败只提示一次，避免每轮刷屏


def alt_shm_note(ch, mm, status):
    """记录一次读数并原子写 momo_alt.json（任何异常都不影响采集主流程）"""
    global ALT_SHM_DIR, _ALT_ERR_LOGGED
    if not ALT_SHM_DIR:  # 未开启落盘
        return
    try:
        with _ALT_LOCK:  # 加锁合并各通道最新值，避免互相覆盖
            _ALT_LATEST[ch] = {'mm': (None if mm is None else int(mm)), 'status': str(status)}
            obj = {'ts': time.time(), 'ch': dict(_ALT_LATEST)}
        path = os.path.join(ALT_SHM_DIR, ALT_SHM_FILE)  # 目标文件
        tmp = path + '.tmp'  # 临时文件：先写后换，读端永远不会读到半个 JSON
        with open(tmp, 'w') as f:  # 写临时文件
            json.dump(obj, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)  # 原子替换
    except Exception as exc:  # 落盘失败绝不能拖垮采集
        if not _ALT_ERR_LOGGED:  # 只提示一次
            _ALT_ERR_LOGGED = True
            print('[!!] momo_alt.json 落盘失败（采集继续）: %r' % (exc,), flush=True)
# ==================== 新增段结束（原有内容在下方继续） ====================


# ------------------------- 工具函数 -------------------------

def modbus_crc16(data: bytes) -> bytes:  # 计算 Modbus-RTU CRC-16 校验值
    """计算 Modbus-RTU CRC-16（多项式 0xA001，初值 0xFFFF），返回低字节在前的 2 字节。"""
    crc = 0xFFFF  # CRC 初值，协议规定为全 1
    for b in data:  # 逐字节参与迭代计算
        crc ^= b  # 当前字节与 CRC 低 8 位异或
        for _ in range(8):  # 每字节展开 8 次移位
            if crc & 1:  # 最低位为 1 时需再异或多项式
                crc = (crc >> 1) ^ 0xA001  # 右移一位并异或反向多项式 0xA001
            else:  # 最低位为 0，仅移位
                crc >>= 1  # 右移一位
    return struct.pack('<H', crc)  # 按低字节在前返回 2 字节


def build_read_frame(addr: int, reg: int, count: int = 1) -> bytes:  # 组装读保持寄存器请求帧
    """构造 Modbus 读保持寄存器请求：地址+功能码+寄存器地址(2B)+数量(2B)+CRC(2B)。"""
    body = bytes([addr, FUNC_READ]) + struct.pack('>HH', reg, count)  # 从机地址+功能码，再大端拼寄存器地址与数量
    return body + modbus_crc16(body)  # 追加 CRC 形成完整 8 字节请求帧


def parse_response(resp: bytes, addr: int):  # 解析从机响应，任何一项不符即返回 None
    """校验并解析从机响应，返回原始值(int)或 None（长度/地址/功能码/字节数/CRC 任一不符）。"""
    if len(resp) < 7:                      # 最短合法响应 = 1+1+1+2+2 = 7 字节
        return None  # 长度不足，必然不是完整应答
    if resp[0] != addr or resp[1] != FUNC_READ or resp[2] != 2:  # 校验从机地址、功能码与数据字节数
        return None  # 不是本机要的应答（可能是异常帧或别家从机）
    if modbus_crc16(resp[:-2]) != resp[-2:]:  # 对前面所有字节重算 CRC 并与帧尾比对
        return None  # CRC 不符，判定传输有误并丢弃
    return int.from_bytes(resp[3:5], 'big', signed=True)   # 高字节在前（手册备注 1）


def make_alt_frame(ch: str, mm, status: str) -> bytes:  # 生成上位机 $ALT 文本帧
    """构建上位机显示帧: $ALT,<通道>,<mm>,<状态># （与上位机 protocol.py 的 parse_alt 对应）
    mm 为 None 时输出空值; 状态: OK / EMPTY / ERR / DOWN"""
    v = '' if mm is None else str(int(mm))  # 无读数时留空占位，有读数取整转字符串
    return f'$ALT,{ch},{v},{status}#\r\n'.encode()  # 按上位机协议封装，CRLF 结尾转字节


# ------------------------- 单通道统计 -------------------------

class ChanStat:  # 单个通道的统计容器，各通道独立实例
    """单个通道的读取统计（各通道独立维护，主线程 Ctrl-C 后统一打印）。"""

    def __init__(self, name: str):  # 初始化各统计字段
        """建一个通道的统计容器；自带锁，因为每个通道一条线程并发读写"""
        self.name = name  # 通道名（A~H）
        self.open_ok = False       # 串口是否成功打开
        self.no_device = False     # 是否已被判定为"未接设备"（降频探测中）
        self.ok_n = 0              # 有效帧数
        self.fail_n = 0            # 失败帧数（超时/无响应/CRC 错）
        self.vals = []             # 有效原始值（mm）
        self.lock = threading.Lock()  # 保护计数与列表，避免多线程竞争

    def note_fail(self):  # 记录一次读取失败
        """失败帧 +1（超时/无响应/CRC 错都算）"""
        with self.lock:  # 加锁后再改计数
            self.fail_n += 1  # 失败帧数累加

    def note_ok(self, v: int):  # 记录一次有效读数
        """有效帧 +1 并留样（mm），供退出时统计 min/max/avg"""
        with self.lock:  # 加锁后再改计数与列表
            self.ok_n += 1  # 有效帧数累加
            self.vals.append(v)  # 保存毫米值，供汇总算 min/max/avg


# ------------------------- 通道读取线程 -------------------------

def channel_loop(name: str, args, stop: threading.Event, st: ChanStat, udp=None):  # 单通道读取线程体
    """单个通道的读取循环（线程体）：打开串口 -> 按节奏发查询 -> 校验并打印。

    独立性保证：
      - 串口打开失败：打印提示后直接结束本线程（不影响其它线程）
      - 连续 NO_REPLY_SWITCH 次无响应：判定未接设备，降为 SLOW_PROBE_INTERVAL
        低频探测；一旦恢复（收到有效帧）立刻回到正常 interval 节奏
    """
    dev = PORT_OF[name]  # 取出该通道对应的串口节点路径
    try:  # 打开串口可能失败（节点不存在或被占用）
        ser = serial.Serial(dev, args.baud, timeout=0.3)  # 打开串口，读超时固定 0.3s
        st.open_ok = True  # 标记打开成功，汇总表据此显示状态
    except Exception as exc:  # 捕获所有打开异常，只影响本通道
        print(f'[!!] [{name}] 打开失败 {dev}: {exc!r} —— 该通道跳过，不影响其它通道', flush=True)  # 打印失败原因
        if udp:  # 开启 UDP 时才上报
            udp.sendto(make_alt_frame(name, None, 'DOWN'), args.pc_addr)  # 推 DOWN 帧告知上位机该口掉线
        alt_shm_note(name, None, 'DOWN')  # [AUV-MISSION 2026-09-26 改动 ②] 落盘 DOWN，读端会丢弃该路
        return  # 直接结束本线程，其余通道照常运行

    frame = build_read_frame(args.addr, args.reg)  # 预生成请求帧，循环内重复发送
    resp_wait = max(0.05, args.interval * 0.5)   # 读超时须大于从机响应时间
    consec_fail = 0  # 连续失败计数，达到阈值即触发降频
    print(f'[*] [{name}] {dev} @ {args.baud} 8N1 就绪（addr=0x{args.addr:02X} '
          f'reg=0x{args.reg:04X}，请求 {binascii.hexlify(frame).decode()}）', flush=True)  # 打印就绪信息与请求帧十六进制
    try:  # 包裹主循环，保证退出时一定关串口
        while not stop.is_set():  # 收到停止事件才退出读取循环
            ser.reset_input_buffer()             # 清旧字节，避免残留帧被误当响应
            ser.write(frame)  # 发送本轮查询帧
            ser.flush()  # 等待数据真正写出到硬件
            deadline = time.time() + resp_wait  # 计算本轮响应的截止时刻
            resp = b''  # 清空本轮响应缓冲
            while time.time() < deadline and len(resp) < 9:  # 在截止前尽量收齐 9 字节响应
                chunk = ser.read(64)  # 一次最多读 64 字节
                if chunk:  # 没读到数据时 chunk 为空
                    resp += chunk  # 累积已收到的响应字节

            raw = parse_response(resp, args.addr)  # 解析出毫米原始值，失败返回 None
            if raw is None:  # 本轮没有有效应答
                consec_fail += 1  # 连续无应答计数，超阈值后自动降低该通道探测频率
                st.note_fail()  # 记入失败统计
                if consec_fail == NO_REPLY_SWITCH:  # 刚好达到阈值，判定动作只触发一次
                    st.no_device = True  # 标记未接设备，后续按低频探测
                    print(f'[!!] [{name}] 连续 {NO_REPLY_SWITCH} 次无响应，判定该口未接高度计；'
                          f'降为 {SLOW_PROBE_INTERVAL}s 低频探测，接上设备后自动恢复', flush=True)  # 提示已降为低频探测
                    if udp:  # 开启 UDP 时才上报
                        udp.sendto(make_alt_frame(name, None, 'EMPTY'), args.pc_addr)  # 推 EMPTY 帧表示无设备
                    alt_shm_note(name, None, 'EMPTY')  # [AUV-MISSION 2026-09-26 改动 ③] 落盘 EMPTY
                if consec_fail == 1:  # 仅首次失败打印，避免每轮刷屏
                    print(f'[!!] [{name}] 读取失败(no-reply): {binascii.hexlify(resp).decode()!r}', flush=True)  # 打印原始字节便于排查
            else:  # 本轮解析成功，拿到毫米读数
                if consec_fail >= NO_REPLY_SWITCH:  # 说明此前正处于降频探测状态
                    # 降频探测期间收到有效帧 -> 设备已接上，恢复正常节奏
                    consec_fail = 0  # 清零连续失败，回到正常间隔
                    st.no_device = False  # 清除未接设备标记
                    print(f'[*] [{name}] 设备恢复，回到正常读取节奏', flush=True)  # 提示设备已重新接上
                else:  # 本来就是正常状态
                    consec_fail = 0  # 清零连续失败计数
                st.note_ok(raw)  # 记入有效统计
                print(f'[OK] [{name}] {time.strftime("%H:%M:%S")} '
                      f'raw=0x{raw:04X} -> {raw} mm', flush=True)  # 打印带时间戳的原始值与毫米数
                if udp:  # 开启 UDP 时才上报
                    udp.sendto(make_alt_frame(name, raw, 'OK'), args.pc_addr)  # 推 OK 帧带上最新读数
                alt_shm_note(name, raw, 'OK')  # [AUV-MISSION 2026-09-26 改动 ④] 落盘有效净空(mm)

            # 调度：未接设备时慢探，正常时按 interval
            sleep_s = SLOW_PROBE_INTERVAL if consec_fail >= NO_REPLY_SWITCH else args.interval  # 未接设备用 2s 慢探，否则用 --interval
            if sleep_s > 0:  # 间隔为 0 表示全速轮询
                stop.wait(sleep_s)  # 可被停止事件提前唤醒的休眠
    finally:  # 正常退出或异常都要释放串口
        try:  # 关闭句柄也可能抛异常
            ser.close()  # 释放串口
        except Exception:  # 忽略关闭失败
            pass  # 不做任何处理


# ------------------------- 汇总打印 -------------------------

def print_summary(addr, reg, ch_stats, frame_hex):  # 打印各通道最终统计表
    """打印所有通道的最终统计表（按 通道名 排序，便于对照物理口）。"""
    print('\n==================== 汇总 ====================')  # 打印汇总表标题
    print(f'请求帧(hex): {frame_hex} | addr=0x{addr:02X} reg=0x{reg:04X} '
          f'({"实时值" if reg == 0x0101 else "处理值" if reg == 0x0100 else "自定义"})')  # 标注读的是实时值还是处理值
    print(f'{"通道":<4}{"设备节点":<26}{"状态":<10}{"有效/失败":<12}数值范围(mm)')  # 打印表格表头
    print('-' * 68)  # 打印表头分隔线
    for st in ch_stats:  # 逐通道输出一行
        dev = PORT_OF[st.name]  # 取该通道对应的串口节点
        if not st.open_ok:  # 串口压根没打开成功
            status, val = '打开失败', '-'  # 状态列显示打开失败
        elif st.no_device:  # 已判定为未接设备
            status, val = '未接设备', '-'  # 状态列显示未接设备
        elif st.ok_n:  # 打开成功且有有效读数
            status = '正常'  # 状态列显示正常
            val = f'{min(st.vals)}~{max(st.vals)} (avg {sum(st.vals) / len(st.vals):.1f})'  # 输出最小值、最大值与平均值
        else:  # 打开成功但一帧有效数据都没有
            status, val = '无数据', '-'  # 状态列显示无数据
        print(f'{st.name:<4}{dev:<26}{status:<10}{st.ok_n}/{st.fail_n:<8}{val}')  # 输出该通道汇总行
    print('===============================================')  # 打印汇总表结束线


# ------------------------- 主流程 -------------------------

def main():  # 命令行入口：解析参数并拉起各通道线程
    """解析命令行 → 按 --channels 给每个物理口拉一条读取线程 → 主线程等 Ctrl-C。

    ⚠ global ALT_SHM_DIR 必须写在函数体最前面：它是模块级全局，供各通道线程读写，
    若在函数里先用后声明会触发 SyntaxError（此前踩过）。
    """
    global ALT_SHM_DIR  # [AUV-MISSION 2026-09-26] 落盘目录是模块级全局，供各通道线程读写
    ap = argparse.ArgumentParser(description='多路持续读取 DYP-L08 超声波高度计 (Modbus-RTU, A–E 口独立)')  # 创建参数解析器
    ap.add_argument('--channels', default='A,B,C,D,E',
                    help='要读取的通道（逗号分隔，默认 A,B,C,D,E；例如 B 或 A,C,E）')  # 注册 --channels，指定要读的物理口
    ap.add_argument('--baud', type=int, default=BAUD_DEFAULT, help=f'波特率（默认 {BAUD_DEFAULT}）')  # 注册 --baud 波特率
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=ADDR_DEFAULT,
                    help=f'Modbus 从机地址（默认 0x{ADDR_DEFAULT:02X}，可用 0x01 或 1 写法）')  # 注册 --addr，支持 0x 与十进制写法
    ap.add_argument('--reg', type=lambda x: int(x, 0), default=REG_DEFAULT,
                    help=f'寄存器地址（默认 0x{REG_DEFAULT:04X}=实时值; 0x0100=处理值）')  # 注册 --reg 寄存器地址
    ap.add_argument('--interval', type=float, default=0.2,
                    help='读取间隔秒（默认 0.2；须大于从机响应时间：实时值~20ms，处理值~135ms）')  # 注册 --interval 读取间隔
    ap.add_argument('--pc-ip', default=PC_IP_DEFAULT,
                    help=f'上位机 PC 的 IP（默认 {PC_IP_DEFAULT}，UDP 推送目标；配合 PC 端 8082 监听）')  # 注册 --pc-ip 上位机地址
    ap.add_argument('--pc-port', type=int, default=ALT_PORT_DEFAULT,
                    help=f'上位机监听端口（默认 {ALT_PORT_DEFAULT}）')  # 注册 --pc-port 上位机端口
    ap.add_argument('--no-udp', action='store_true', help='关闭 UDP 推送（纯终端模式）')  # 注册 --no-udp 开关
    # [AUV-MISSION 2026-09-26 改动 ⑤] 共享内存落盘开关（depth_kalman 的输入源）
    ap.add_argument('--shm-dir', default=ALT_SHM_DIR,
                    help=f'momo_alt.json 落盘目录（默认 {ALT_SHM_DIR}）')  # 注册 --shm-dir 目录
    ap.add_argument('--no-shm', action='store_true',
                    help='关闭 momo_alt.json 落盘（只推 UDP）')  # 注册 --no-shm 开关
    args = ap.parse_args()  # 解析命令行参数
    args.pc_addr = None if args.no_udp else (args.pc_ip, args.pc_port)  # 合成 UDP 目标地址，关闭推送时为 None
    ALT_SHM_DIR = None if args.no_shm else args.shm_dir  # 关闭时置空，alt_shm_note 会直接返回

    names = [c.strip().upper() for c in args.channels.split(',') if c.strip().upper() in PORT_OF]  # 拆分通道名并过滤非法项
    if not names:  # 一个有效通道都没剩下
        print(f'[!] 没有有效通道（可选 A–H），输入: {args.channels}')  # 提示输入不合法
        return 1  # 返回非零退出码

    frame_hex = binascii.hexlify(build_read_frame(args.addr, args.reg)).decode()  # 预生成请求帧十六进制串供日志使用
    udp = None if args.no_udp else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 按需创建 UDP 套接字
    print(f'[*] 高度计多路读取: {len(names)} 通道 {names} @ {args.baud} 8N1 | '
          f'addr=0x{args.addr:02X} reg=0x{args.reg:04X} | 每通道间隔 {args.interval}s | '
          f'请求帧={frame_hex}', flush=True)  # 打印启动信息：通道数、波特率、地址、间隔、请求帧
    if udp:  # UDP 推送已开启
        print(f'[*] UDP 推送已开启: {args.pc_ip}:{args.pc_port} ($ALT 帧 → PC 上位机"全部"页)', flush=True)  # 打印推送目标
    else:  # 未开启 UDP 推送
        print('[*] UDP 推送已关闭 (--no-udp)', flush=True)  # 提示当前为纯终端模式
    # [AUV-MISSION 2026-09-26 改动 ⑥] 落盘状态提示
    if ALT_SHM_DIR:  # 开启了共享内存落盘
        print(f'[*] 共享内存落盘已开启: {os.path.join(ALT_SHM_DIR, ALT_SHM_FILE)} '
              f'(depth_kalman 输入源)', flush=True)  # 提示落盘路径
    else:  # 关闭了落盘
        print('[*] 共享内存落盘已关闭 (--no-shm)：depth_kalman 将没有高度计输入', flush=True)

    stop = threading.Event()  # 停止事件，Ctrl-C 时统一通知各通道线程
    stats = [ChanStat(n) for n in names]  # 为每个通道各建一份统计对象
    threads = [threading.Thread(target=channel_loop, args=(n, args, stop, st, udp), daemon=True)
               for n, st in zip(names, stats)]  # 建守护线程列表，主线程退出即随之结束
    for t in threads:  # 逐个启动通道线程
        t.start()  # 启动线程，各口开始独立轮询

    try:  # 主线程原地等待，期间可被 Ctrl-C 打断
        while True:  # 持续运行直到中断或线程全部结束
            time.sleep(0.5)  # 每 0.5s 检查一次线程存活情况
            if not any(t.is_alive() for t in threads):   # 所有通道都结束（如全部打开失败）
                break  # 没有存活线程则退出等待
    except KeyboardInterrupt:  # 用户按下 Ctrl-C
        print('\n[!] Ctrl-C 中断，正在汇总…', flush=True)  # 提示正在收尾
    finally:  # 无论正常退出还是中断都要停止线程
        stop.set()  # 置位停止事件，通知各线程退出循环
        for t in threads:  # 逐个等待线程结束
            t.join(timeout=1.0)  # 最多等 1s，避免卡住进程退出

    if udp:  # 存在 UDP 套接字才需要关闭
        try:  # 关闭也可能抛异常
            udp.close()  # 释放 UDP 套接字
        except Exception:  # 忽略关闭失败
            pass  # 不做任何处理

    print_summary(args.addr, args.reg, stats, frame_hex)  # 打印最终各通道汇总表
    return 0 if any(s.open_ok and s.ok_n for s in stats) else 1  # 任一通道读到有效数据即算成功


if __name__ == '__main__':  # 仅直接运行本文件时执行
    sys.exit(main())  # 用 main 的返回值作为进程退出码
