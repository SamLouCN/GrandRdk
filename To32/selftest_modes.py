#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中位机模式框架自测 —— 不需要真实下位机，也不需要上位机 UI。

覆盖:
  1) 启动：初始模式(ROV)向 0x04 下发 ROV
  2) 模式切换：$CMD.mode 0->1->0 触发 0x04(预编程/ROV) 与 on_enter/on_exit
  3) 各模式下行：ROV 有 0x09；AUV 期间 0x09 必须不增加
  4) 安全：下行静默 > ESTOP_TIMEOUT_S -> 0x04 0x01 急停锁存；锁存期间抑制 0x09；$ESTOP,0# 解除
  5) 遥测/上行：合成 0x0C 遥测 -> 锚定 + $TEL(41 字段) 上行
  6) 链路：PING -> PONG
  7) 启动模式：START_MODE / --mode 强制 / 上位机切换后的模式记忆（重启沿用）

运行: cd /userdata/To32 && python3 selftest_modes.py      详细日志: /tmp/selftest_modes.log
"""
from __future__ import annotations

import json
import os
import queue
import socket
import struct
import sys
import threading
import time

import link_stm32 as S
import main as M

CMD_PORT = 18080        # 用非默认端口，避免与正在跑的 cmd_watch.py / relay.py 抢 8080/8081
TEL_PORT = 18081
STATE_PATH = "/tmp/selftest_mode_state.json"   # 模式记忆自测临时文件（绝不动生产 mode_state.json）
ESTOP_TIMEOUT = 0.4


class RecLink(S.Stm32Link):
    """sim 链路 + 记录每一帧下行内容。"""

    def __init__(self, cfgv, log):
        super().__init__(cfgv, log, port_spec="sim", on_telemetry=None)
        self.frames = []

    def send(self, frame, note=""):
        if frame:
            self.frames.append((time.time(), note, bytes(frame)))
        return super().send(frame, note)

    def func_frames(self, func):
        return [f for (_t, _n, f) in self.frames if len(f) > 2 and f[2] == func]

    def mode_codes(self):
        return [f[3] for f in self.func_frames(S.FUNC_MODE) if len(f) > 3]


def telemetry_frame(depth_cm=55.0, target_cm=50.0):
    """合成一帧 48B 0x0C 遥测（与真实下位机同布局）。"""
    data = struct.pack("<12h",
                       0, 0, 0,
                       int(1.5 * 100), int(0.8 * 100), int(-0.5 * 100),
                       int(0.3 * 100), int(0.1 * 100), int(-0.2 * 100),
                       int(0.05 * 100), int(0.02 * 100), int(9.8 * 100))
    data += struct.pack("<8h", *[int(100 * (i % 4)) for i in range(8)])
    data += struct.pack("<HH", int(target_cm * 100), int(depth_cm * 100))
    return S.build_frame(S.FUNC_TELEMETRY, data)


class Test(object):
    def __init__(self):
        self.results = []

    def check(self, name, ok, detail=""):
        self.results.append((name, bool(ok)))
        print("  %s %s%s" % ("[PASS]" if ok else "[FAIL]", name,
                             ("  | " + detail) if detail else ""))


def run():
    t = Test()
    log = M.make_logger("/tmp/selftest_modes.log", quiet=True)
    cfgv = M.build_cfg({
        "CMD_PORT": CMD_PORT, "TELEM_PORT": TEL_PORT, "PC_BIND_IP": "127.0.0.1",
        "ESTOP_TIMEOUT_S": ESTOP_TIMEOUT, "ESTOP_LATCH": True, "REPORT_S": 0.0,
        "STM32_POLL_HZ": 20.0, "TICK_HZ": 20.0, "TEL_HZ": 10.0,
        # 与"模式记忆"隔离：自测不读不写生产记忆文件, 启动模式恒为 START_MODE
        "MODE_PERSIST": False, "MODE_STATE_PATH": STATE_PATH,
        "START_MODE": 0, "MODE_FORCE_START": True,
    }, warn=log)

    rx = queue.Queue()
    pc = M.PC.PcLink(cfgv, log, rx)
    link = RecLink(cfgv, log)
    disp = M.Dispatcher(cfgv, pc, link, log, rx_queue=rx)
    link.on_telemetry = disp.on_stm32_telemetry
    sent_tel = []
    pc.send_telem = lambda text: (sent_tel.append(text), True)[1]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 模拟上位机手柄：20Hz 持续 $CMD（真实 UI 有线模式就是 20Hz），状态由各步骤改写
    state = {"mode": 0, "surge": 0.0, "yaw": 0.0, "sending": True, "quit": False}

    def heartbeat():
        while not state["quit"]:
            if state["sending"]:
                line = "$CMD,%.2f,0.00,0.00,%.2f,0,0,%d,0,0#\r\n" % (
                    state["surge"], state["yaw"], state["mode"])
                try:
                    sock.sendto(line.encode(), ("127.0.0.1", CMD_PORT))
                except OSError:
                    return
            time.sleep(0.05)

    def feed_telemetry():
        for func, data in S.FrameParser().feed(telemetry_frame()):
            if func == S.FUNC_TELEMETRY:
                tel = S.parse_telemetry(data)
                if tel:
                    disp.on_stm32_telemetry(tel)
                    return tel
        return None

    print("[1] 启动与初始模式")
    disp.start()
    threading.Thread(target=heartbeat, daemon=True).start()
    time.sleep(0.25)
    t.check("上位机 UDP 链路通（已收到 $CMD）", disp.stats["cmd"] > 0, "cmd=%d" % disp.stats["cmd"])
    t.check("初始模式 = ROV", disp.mode_id == cfgv.MODE_ROV, "mode_id=%s" % disp.mode_id)
    t.check("启动即下发 0x04 ROV(0x03)", S.MODE_ROV in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    n0c_rov = len(link.func_frames(S.FUNC_TELEMETRY))
    t.check("处于/切到 ROV 即请求回传(0x0C)", n0c_rov > 0, "0x0C 帧数=%d" % n0c_rov)

    print("[2] ROV 模式：$CMD -> 0x09")
    state["surge"] = 0.5
    time.sleep(0.3)
    n09_rov = len(link.func_frames(S.FUNC_MOTION))
    t.check("ROV 下发 0x09", n09_rov > 0, "0x09 帧数=%d" % n09_rov)

    print("[3] 遥测 -> 锚定 + $TEL 上行")
    feed_telemetry()
    time.sleep(0.35)
    t.check("首帧遥测后已锚定", disp.anchored)
    t.check("$TEL 已上行", len(sent_tel) > 0, "条数=%d" % len(sent_tel))
    fields = sent_tel[-1].strip().rstrip("#").split(",")[1:] if sent_tel else []
    t.check("$TEL 字段数 = 41", len(fields) == 41, "实际=%d" % len(fields))
    if fields:
        t.check("$TEL 深度(m) 合理", abs(float(fields[6]) - 0.55) < 0.5, "fields[6]=%s" % fields[6])

    print("[4] 切到 AUV：0x04 预编程 + 停止 0x09")
    state["mode"] = 1
    time.sleep(0.2)
    n09_before_auv = len(link.func_frames(S.FUNC_MOTION))
    t.check("模式切到 AUV", disp.mode_id == cfgv.MODE_AUV, "mode_id=%s" % disp.mode_id)
    t.check("下发 0x04 预编程(0x02)", S.MODE_PREPROGRAM in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    state["surge"] = 1.0
    state["yaw"] = 1.0
    time.sleep(0.3)
    n09_in_auv = len(link.func_frames(S.FUNC_MOTION))
    t.check("AUV 期间无 0x09（摇杆满偏也不发）", n09_in_auv == n09_before_auv,
            "0x09 帧数 %d -> %d" % (n09_before_auv, n09_in_auv))
    n0c_in_auv = len(link.func_frames(S.FUNC_TELEMETRY))
    time.sleep(0.35)
    n0c_after_wait = len(link.func_frames(S.FUNC_TELEMETRY))
    t.check("AUV 期间 ROV 停止请求回传(0x0C 不再增长)", n0c_after_wait == n0c_in_auv,
            "0x0C 帧数 %d -> %d" % (n0c_in_auv, n0c_after_wait))

    print("[5] 切回 ROV：0x09 恢复")
    state["mode"] = 0
    state["surge"] = 0.3
    state["yaw"] = 0.0
    time.sleep(0.3)
    n09_back = len(link.func_frames(S.FUNC_MOTION))
    t.check("切回 ROV 后 0x09 恢复", n09_back > n09_before_auv,
            "0x09 帧数 %d -> %d" % (n09_before_auv, n09_back))
    t.check("切回 ROV 后恢复请求回传(0x0C 继续增长)",
            len(link.func_frames(S.FUNC_TELEMETRY)) > n0c_after_wait,
            "0x0C 帧数 %d -> %d" % (n0c_after_wait, len(link.func_frames(S.FUNC_TELEMETRY))))

    print("[6] PING -> PONG")
    ping = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ping.bind(("127.0.0.1", 0))
    ping.settimeout(1.0)
    ping.sendto(b"PING", ("127.0.0.1", TEL_PORT))
    pong = b""
    try:
        pong, _ = ping.recvfrom(64)
    except socket.timeout:
        pass
    t.check("PING 收到 PONG", pong.startswith(b"PONG"), "收到=%r" % pong)
    ping.close()

    print("[7] 安全：下行静默 -> 急停锁存 -> $ESTOP,0# 解除")
    state["sending"] = False                      # 模拟上位机/链路静默
    time.sleep(ESTOP_TIMEOUT + 0.6)
    t.check("静默超时后已锁存", disp.estop_latch, "reason=%s" % disp.estop_reason)
    t.check("已下发 0x04 急停(0x01)", S.MODE_ESTOP in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    n09_latched = len(link.func_frames(S.FUNC_MOTION))
    state["surge"] = 1.0
    state["sending"] = True                       # 恢复下行：锁存状态下也不该有 0x09
    time.sleep(0.3)
    n09_after_latch = len(link.func_frames(S.FUNC_MOTION))
    t.check("锁存期间抑制 0x09", n09_after_latch == n09_latched,
            "0x09 帧数 %d -> %d" % (n09_latched, n09_after_latch))
    sock.sendto(b"$ESTOP,0#\r\n", ("127.0.0.1", CMD_PORT))
    time.sleep(0.25)
    t.check("$ESTOP,0# 解除锁存", not disp.estop_latch)
    state["surge"] = 0.4
    time.sleep(0.3)
    t.check("解除后 0x09 恢复", len(link.func_frames(S.FUNC_MOTION)) > n09_after_latch,
            "0x09 帧数=%d" % len(link.func_frames(S.FUNC_MOTION)))

    print("[8] 收尾")
    state["quit"] = True
    disp.stop()
    n_cmd = disp.stats["cmd"]
    sock.sendto(b"$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#\r\n", ("127.0.0.1", CMD_PORT))
    time.sleep(0.3)
    sock.close()
    t.check("stop() 后编排线程已退出", all(not th.is_alive() for th in disp._threads))
    t.check("stop() 后不再处理上位机帧", disp.stats["cmd"] == n_cmd,
            "$CMD %d -> %d" % (n_cmd, disp.stats["cmd"]))

    print("[9] 启动模式配置 + 模式记忆")
    t.check("--mode 解析 rov/auv/0/1/非法",
            M.parse_mode_arg("rov") == 0 and M.parse_mode_arg("AUV") == 1
            and M.parse_mode_arg("0") == 0 and M.parse_mode_arg("1") == 1
            and M.parse_mode_arg("xx") is None,
            "rov=%s auv=%s 0=%s 1=%s xx=%s" % (
                M.parse_mode_arg("rov"), M.parse_mode_arg("AUV"), M.parse_mode_arg("0"),
                M.parse_mode_arg("1"), M.parse_mode_arg("xx")))
    if os.path.exists(STATE_PATH):
        os.remove(STATE_PATH)

    def make_disp(extra):
        """独立 cfg + 编排实例（不启动线程、不占端口），用于验证启动模式解析"""
        base = {"CMD_PORT": CMD_PORT, "TELEM_PORT": TEL_PORT, "PC_BIND_IP": "127.0.0.1",
                "REPORT_S": 0.0, "MODE_STATE_PATH": STATE_PATH}
        base.update(extra)
        cv = M.build_cfg(base, warn=log)
        rq = queue.Queue()
        return cv, M.Dispatcher(cv, M.PC.PcLink(cv, log, rq), RecLink(cv, log), log, rx_queue=rq)

    cv_a, d_a = make_disp({"MODE_PERSIST": True, "START_MODE": 0, "MODE_FORCE_START": False})
    t.check("无记忆文件 -> 按 START_MODE(ROV) 启动", d_a.mode_id == cv_a.MODE_ROV,
            "mode_id=%s" % d_a.mode_id)
    d_a.switch_mode(cv_a.MODE_AUV)
    try:
        with open(STATE_PATH, "r", encoding="utf-8") as f:
            saved = json.load(f).get("mode")
    except Exception as e:
        saved = "读取失败: %s" % e
    t.check("切到 AUV 后已写入记忆文件(mode=1)", saved == 1, "文件值=%r" % (saved,))

    cv_b, d_b = make_disp({"MODE_PERSIST": True, "START_MODE": 0, "MODE_FORCE_START": False})
    t.check("下次启动沿用记忆模式(AUV)", d_b.mode_id == cv_b.MODE_AUV,
            "mode_id=%s" % d_b.mode_id)

    cv_c, d_c = make_disp({"MODE_PERSIST": True, "START_MODE": 0, "MODE_FORCE_START": True})
    t.check("--mode 强制覆盖记忆(ROV)", d_c.mode_id == 0, "mode_id=%s" % d_c.mode_id)

    cv_d, d_d = make_disp({"MODE_PERSIST": False, "START_MODE": 1, "MODE_FORCE_START": False})
    t.check("关闭记忆时按 START_MODE(AUV) 启动", d_d.mode_id == 1, "mode_id=%s" % d_d.mode_id)

    with open(STATE_PATH, "w", encoding="utf-8") as f:
        f.write("{ 坏文件 ")
    cv_e, d_e = make_disp({"MODE_PERSIST": True, "START_MODE": 0, "MODE_FORCE_START": False})
    t.check("记忆文件损坏 -> 回退 START_MODE(ROV)", d_e.mode_id == 0, "mode_id=%s" % d_e.mode_id)
    if os.path.exists(STATE_PATH):
        os.remove(STATE_PATH)

    print("[10] 回传链路：0x0C 回帧 -> 遥测解析 -> $TEL 41 字段映射")
    raw = telemetry_frame(depth_cm=55.0, target_cm=50.0)
    t.check("0x0C 回帧整帧 = 48B (LEN=0x2F)", len(raw) == S.TELEM_FRAME_LEN == 48,
            "len=%d LEN=0x%02X" % (len(raw), raw[1]))
    pairs = S.FrameParser().feed(raw)
    tel10 = None
    if len(pairs) == 1 and pairs[0][0] == S.FUNC_TELEMETRY:
        tel10 = S.parse_telemetry(pairs[0][1])
    t.check("回帧可解析为遥测 dict", isinstance(tel10, dict) and bool(tel10),
            "键数=%s" % (len(tel10) if tel10 else 0))
    t.check("遥测键名 = actual_depth_cm / target_depth_cm（非旧名）",
            bool(tel10) and "actual_depth_cm" in tel10 and "target_depth_cm" in tel10
            and "depth_actual_cm" not in tel10 and "depth_target_cm" not in tel10,
            "深度相关键=%s" % sorted(k for k in (tel10 or {}) if "depth" in k))
    if tel10:
        t.check("回帧深度值 = 实测 55.0cm / 目标 50.0cm",
                abs(tel10["actual_depth_cm"] - 55.0) < 0.01
                and abs(tel10["target_depth_cm"] - 50.0) < 0.01,
                "actual=%.1f target=%.1f" % (tel10["actual_depth_cm"], tel10["target_depth_cm"]))

    import tel_builder as TB
    fv = TB.tel_values(tel10) if tel10 else None
    t.check("$TEL 字段数 = 41", fv is not None and len(fv) == TB.TEL_FIELD_COUNT == 41,
            "len=%s" % (len(fv) if fv else None))
    if fv:
        t.check("$TEL[6] 实际深度(m) = 0.55（修复前恒 0）",
                abs(fv[6] - 0.55) < 0.002, "f[6]=%.4f" % fv[6])
        t.check("$TEL[16] 目标深度(m) = 0.50（修复前恒 0）",
                abs(fv[16] - 0.50) < 0.002, "f[16]=%.4f" % fv[16])
        t.check("$TEL[0..2] 姿态 roll/pitch/yaw = -0.50/1.50/0.80",
                abs(fv[0] + 0.5) < 0.002 and abs(fv[1] - 1.5) < 0.002 and abs(fv[2] - 0.8) < 0.002,
                "%.2f/%.2f/%.2f" % (fv[0], fv[1], fv[2]))
        t.check("$TEL[3..5] 角速度 gx/gy/gz = -0.20/0.30/0.10",
                abs(fv[3] + 0.2) < 0.002 and abs(fv[4] - 0.3) < 0.002 and abs(fv[5] - 0.1) < 0.002,
                "%.2f/%.2f/%.2f" % (fv[3], fv[4], fv[5]))
        t.check("$TEL[25..32] 推进器 8 路 = 0/.1/.2/.3 循环",
                all(abs(fv[25 + i] - (i % 4) / 10.0) < 1e-6 for i in range(8)),
                ",".join("%.2f" % fv[25 + i] for i in range(8)))
        t.check("$TEL[33..36] 推进器 9~12 路 = 0（V2 仅 8 路占位）",
                all(abs(fv[33 + i]) < 1e-9 for i in range(4)),
                ",".join("%.2f" % fv[33 + i] for i in range(4)))
        t.check("$TEL[37..39] 加速度 = 0.05/0.02/9.80",
                abs(fv[37] - 0.05) < 0.002 and abs(fv[38] - 0.02) < 0.002 and abs(fv[39] - 9.8) < 0.01,
                "%.2f/%.2f/%.2f" % (fv[37], fv[38], fv[39]))
        txt10 = TB.build_tel(tel10)
        body10 = txt10.strip()[5:-1].split(",") if (txt10 and txt10.strip().endswith("#")) else []
        t.check("$TEL 文本帧 = $TEL,<41 字段>#\\r\\n",
                bool(txt10) and txt10.startswith("$TEL,") and txt10.endswith("#\r\n")
                and len(body10) == 41,
                "字段数=%d 尾部=%r" % (len(body10), txt10[-3:] if txt10 else None))

    bad = []
    _here = os.path.dirname(os.path.abspath(__file__))
    for fn in ("tel_builder.py", "mode_dispatcher.py"):
        with open(os.path.join(_here, fn), encoding="utf-8") as fh:
            src10 = fh.read()
        for key in ('"depth_actual_cm"', '"depth_target_cm"'):
            if key in src10:
                bad.append("%s 残留 %s" % (fn, key))
    t.check("旧字段名已彻底清除（源码守卫）", not bad, "残留=%s" % (bad or "无"))

    cv_f, d_f = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True})
    if tel10:
        d_f.on_stm32_telemetry(tel10)
    m_f = d_f.mode()
    t.check("冷启动锚定取实测值 target_depth_cm=55.0 / target_yaw_deg=0.8（修复前深度锚定失效）",
            abs(getattr(m_f, "target_depth_cm", -1) - 55.0) < 0.01
            and abs(getattr(m_f, "target_yaw_deg", -1) - 0.8) < 0.01,
            "depth=%s yaw=%s" % (getattr(m_f, "target_depth_cm", None),
                                 getattr(m_f, "target_yaw_deg", None)))
    txt_rov = m_f.build_telemetry()
    rf = ([float(x) for x in txt_rov.strip()[5:-1].split(",")]
          if (txt_rov and txt_rov.strip().endswith("#")) else [])
    t.check("ROV build_telemetry = 41 字段, [6]=0.55, [16]=0.50（修复前 [16] 写死 0）",
            len(rf) == 41 and abs(rf[6] - 0.55) < 0.002 and abs(rf[16] - 0.50) < 0.002,
            "len=%d f6=%s f16=%s" % (len(rf), rf[6] if len(rf) > 6 else "-",
                                     rf[16] if len(rf) > 16 else "-"))

    passed = sum(1 for _n, ok in t.results if ok)
    total = len(t.results)
    print("\n结果: %d/%d 通过" % (passed, total))
    print("详细日志: /tmp/selftest_modes.log")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(run())