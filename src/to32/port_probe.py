# -*- coding: utf-8 -*-
"""CH348 通道确认工具：往指定串口写一段"一眼能认出"的 ASCII 标记。

用途：确认 PC 串口助手上看到的数据，到底来自 CH348 的哪一个物理通道
（也就是确认 /dev/ttyCH9344USB5 是不是真的 F 口）。

用法（**先停掉 main.py，串口是独占的**）:
    python3 port_probe.py --sweep                  # 依次扫 USB0..USB7，每口发自己的标记
    python3 port_probe.py /dev/ttyCH9344USB5       # 只测一个口
    python3 port_probe.py /dev/ttyCH9344USB5 "HI" 5   # 自定义文本 / 重复次数

sweep 模式下每个口发的标记不同，PC 助手收到哪个标记，就说明那个口是接在 PC 上的：
    USB0(A) ... USB7(H)
"""
import sys
import time

DEV_FMT = "/dev/ttyCH9344USB%d"
PORT_NAMES = "ABCDEFGH"
BAUD = 115200


def probe(dev, text, times=3, gap=0.08):
    try:
        import serial
    except ImportError:
        print("缺 pyserial：请先 pip3 install pyserial")
        return False
    try:
        ser = serial.Serial(dev, BAUD, timeout=0.1, write_timeout=1.0)
    except Exception as e:
        print("  %s 打开失败: %s" % (dev, e))
        return False
    try:
        for i in range(times):
            ser.write(text.encode("ascii", "replace"))
            ser.flush()
            time.sleep(gap)
        print("  %s  <- 已发送 %d 次: %r" % (dev, times, text))
        return True
    except Exception as e:
        print("  %s 写失败: %s" % (dev, e))
        return False
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main():
    args = [a for a in sys.argv[1:]]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        return
    if args[0] == "--sweep":
        print("依次扫描 8 个 CH348 通道，请盯住 PC 串口助手的接收区：")
        for i in range(8):
            dev = DEV_FMT % i
            # 标记里同时带索引和字母，肉眼最好认
            probe(dev, "\r\n<<CH348 USB%d=%s>>\r\n" % (i, PORT_NAMES[i]), times=4, gap=0.05)
        print()
        print("看到哪个 <<CH348 USBn=X>>，那个 /dev/ttyCH9344USBn 就是接在 PC 上的通道。")
        print("若一个都没看到：接线(TX/RX 需交叉、GND 共地)、电平(TTL/RS232)或 PC 侧串口号不对。")
        return

    dev = args[0]
    text = args[1] if len(args) > 1 else "\r\n<<PROBE %s>>\r\n" % dev
    times = int(args[2]) if len(args) > 2 else 3
    probe(dev, text, times=times)


if __name__ == "__main__":
    main()
