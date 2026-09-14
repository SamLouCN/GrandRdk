#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_watch.py — PC 上位机 → S100 上行帧接收检测器（被动旁路）

作用：把电脑上位机(ROV控制站 v3.2)通过网线发到本板的 UDP 帧逐条显示/记录，
     方便确认"点按钮/推摇杆到底发了什么帧"。纯只读：不转发、不转 V2、不改链路。

用法:
  python3 cmd_watch.py                      # 默认监听 UDP 8080（$CMD/$PID/$VID 等指令帧）
  python3 cmd_watch.py --port 8081          # 改为监听 8081（PC 的 PING 探测）
  python3 cmd_watch.py --ports 8080,8081    # 同时监听多个口
  python3 cmd_watch.py --log /userdata/cmd_watch.log   # 追加写日志
  python3 cmd_watch.py --hex                # 每帧额外打印 HEX 字节

配合使用:
  nohup python3 cmd_watch.py --log /userdata/cmd_watch.log > /userdata/cmd_watch.out 2>&1 &
  tail -f /userdata/cmd_watch.log

注意: 若 8080 被 relay.py/vp5.1 占用会拒绝启动；此时先停那一边再用本工具。
"""
import argparse
import datetime
import select
import socket
import sys
import time


def parse_frame(text):
    """轻量解析上行文本帧 → 可读摘要（不依赖 protocol.py）。"""
    s = text.strip()
    if not s:
        return "(空行)"
    if s.startswith("$CMD,") and s.endswith("#"):
        p = s[5:-1].split(",")
        if len(p) >= 6:
            mode = "AUV" if (len(p) > 6 and p[6] == "1") else "ROV"
            return ("$CMD surge=%s sway=%s heave=%s yaw=%s led1=%s led2=%s mode=%s"
                    % (p[0], p[1], p[2], p[3], p[4], p[5], mode))
        return "$CMD(字段不足): " + s
    if s.startswith("$PID,") and s.endswith("#"):
        p = s[5:-1].split(",")
        if len(p) >= 4:
            return "$PID ch=%s p=%s i=%s d=%s" % (p[0], p[1], p[2], p[3])
        return "$PID(字段不足): " + s
    if s.startswith("$VID,") and s.endswith("#"):
        v = s[5:-1].strip()
        return "$VID 视频=%s" % ("开(1)" if v == "1" else "关(0)" if v == "0" else v)
    if s.startswith("$ESTOP"):
        return "$ESTOP 急停帧: " + s
    if s.startswith("$ALT,"):
        return "$ALT 高度计帧(板上行): " + s[:60]
    if s == "PING":
        return "PING (PC 延迟探测)"
    return s[:80]


def main():
    ap = argparse.ArgumentParser(description="PC→S100 上行帧接收检测器(被动旁路)")
    ap.add_argument("--ports", default="8080", help="逗号分隔的 UDP 端口列表, 默认 8080")
    ap.add_argument("--port", type=int, default=None, help="等价于 --ports 单口")
    ap.add_argument("--log", default="", help="追加写入的日志文件")
    ap.add_argument("--hex", action="store_true", help="每帧额外打印 HEX 字节")
    args = ap.parse_args()

    if args.port is not None:
        ports = [args.port]
    else:
        ports = [int(x) for x in args.ports.split(",") if x.strip()]
    if not ports:
        print("[FATAL] 未指定有效端口"); sys.exit(2)

    socks = []
    for p in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", p))
        except OSError as e:
            print("[FATAL] 端口 %d 绑定失败: %s —— 可能被 relay.py/vp5.1 占用, 先停那边再试" % (p, e))
            sys.exit(2)
        socks.append((p, s))
        print("[OK] 监听 UDP 0.0.0.0:%d" % p)
        sys.stdout.flush()

    logf = open(args.log, "a", buffering=1) if args.log else None

    def emit(line):
        print(line, flush=True)
        if logf:
            logf.write(line + "\n")

    print("[WARN] 只读旁路: 不转发、不转 V2；看板端↔STM32 请用 relay.py")
    print("[NOW ] 等待 PC 上位机操作… (Ctrl+C 退出)")
    sys.stdout.flush()

    counter = {}
    while True:
        try:
            r, _, _ = select.select([s for _, s in socks], [], [], 3600)
            if not r:
                continue
            for s in r:
                data, addr = s.recvfrom(2048)
                port = dict((id(x), p) for p, x in socks)[id(s)]
                text = data.decode("utf-8", errors="replace")
                ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                kind = (text.split(",")[0].strip() or "?")[:12]
                counter[kind] = counter.get(kind, 0) + 1
                line = ("[%s] :%d ≤ %s:%d (%dB) 累计=%s"
                        % (ts, port, addr[0], addr[1], len(data), dict(counter)))
                line += "\n      " + repr(parse_frame(text))
                if args.hex:
                    line += "\n      HEX: " + " ".join("%02X" % b for b in data)
                emit(line)
        except KeyboardInterrupt:
            print("\n[END] 累计: " + str(counter))
            sys.stdout.flush()
            if logf:
                logf.close()
            sys.exit(0)
        except Exception as e:  # noqa: BLE001
            print("[ERR] %s" % e, flush=True)
            time.sleep(0.5)


if __name__ == "__main__":
    main()