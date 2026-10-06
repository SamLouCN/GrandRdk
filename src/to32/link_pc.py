# -*- coding: utf-8 -*-
"""上位机链路：UDP 8080 收指令 / UDP 8081 收 PING + 发 $TEL

只做"收字节、解析归类、入队"；业务派发交给主循环与 ModeManager。
"""
import queue
import select
import socket
import threading
import time

import protocol as P


class PcLink(threading.Thread):
    def __init__(self, cfg, log, rx_queue):
        super().__init__(name="PcLink", daemon=True)
        self.cfg = cfg
        self.log = log
        self.q = rx_queue
        self.pc_addr = None          # 从下行帧源地址学习（上位机从不自报 IP）
        self.first_cmd_logged = False  # 首个下行帧只打一次源地址（对端身份可追溯，U2）
        self.last_cmd_ts = 0.0
        self.stats = {"cmd": 0, "pid": 0, "vid": 0, "ping": 0, "unknown": 0}
        self._stop = threading.Event()
        self._sock_cmd = None
        self._sock_tel = None

    # ---------- 生命周期 ----------
    def open(self):
        self._sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_cmd.bind((self.cfg.PC_BIND_IP, self.cfg.CMD_PORT))
        self._sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_tel.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_tel.bind((self.cfg.PC_BIND_IP, self.cfg.TELEM_PORT))

    def stop(self):
        self._stop.set()
        for s in (self._sock_cmd, self._sock_tel):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass

    def run(self):
        try:
            self.open()
        except OSError as e:
            self.log("[PC] 端口绑定失败: %s（8080/8081 被占用？先停 cmd_watch.py / relay.py）" % e)
            return
        self.log("[PC] 监听 UDP :%d（指令） / :%d（PING→PONG + $TEL 上行）"
                 % (self.cfg.CMD_PORT, self.cfg.TELEM_PORT))
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self._sock_cmd, self._sock_tel], [], [], 0.2)
            except (OSError, ValueError):
                break
            for s in r:
                try:
                    data, addr = s.recvfrom(2048)
                except OSError:
                    continue
                text = data.decode("utf-8", errors="replace")
                if s is self._sock_cmd:
                    self.pc_addr = addr
                    self.last_cmd_ts = time.time()
                    if not self.first_cmd_logged:
                        self.first_cmd_logged = True
                        self.log("[PC] 首个下行帧来自 %s:%d（对端身份已记录）"
                                 % (addr[0], addr[1]))
                    self._dispatch(text)
                elif P.is_ping(text):
                    self.stats["ping"] += 1
                    try:
                        self._sock_tel.sendto(b"PONG", addr)   # 必须回 PING 的源端口
                    except OSError:
                        pass

    # ---------- 解析归类 ----------
    def _dispatch(self, text):
        cmd = P.parse_cmd(text)
        if cmd is not None:
            self.stats["cmd"] += 1
            self.q.put(("cmd", cmd))
            return
        pid = P.parse_pid(text)
        if pid is not None:
            self.stats["pid"] += 1
            self.q.put(("pid", pid))
            return
        vid = P.parse_vid(text)
        if vid is not None:
            self.stats["vid"] += 1
            self.q.put(("vid", vid))
            return
        if text.strip():
            self.stats["unknown"] += 1
            self.q.put(("raw", text.strip()))

    # ---------- 发送 ----------
    def send_telem(self, text):
        """把 $TEL 发给上位机:8081（地址由下行帧学习）"""
        if not text or self.pc_addr is None or self._sock_tel is None:
            return False
        try:
            self._sock_tel.sendto(text.encode(), (self.pc_addr[0], self.cfg.TELEM_PORT))
            return True
        except OSError:
            return False

    def pc_ip(self):
        return self.pc_addr[0] if self.pc_addr else "-"