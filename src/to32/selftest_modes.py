#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中位机模式框架自测 —— 不需要真实下位机，也不需要上位机 UI。

覆盖:
  1) 启动：初始模式(ROV)向 0x04 下发 ROV
  2) 模式切换：$CMD.mode 0->1->0 触发 0x04(有线ROV 0x06 / AUV 0x05) 与 on_enter/on_exit
  3) 各模式下行：ROV 有 0x09；AUV 期间 0x09 必须不增加（运动逻辑已摘除，见 move_test/）
  4) 安全：下行静默 > ESTOP_TIMEOUT_S -> 0x04 0x01 急停锁存；锁存期间抑制 0x09；$ESTOP,0# 解除
  5) 遥测/上行：合成 0x0C 遥测 -> 锚定 + $TEL(41 字段) 上行
  6) 链路：PING -> PONG
  7) 启动模式：START_MODE / --mode 强制 / 上位机切换后的模式记忆（重启沿用）

运行: cd /userdata/GrandRDK && python3 src/to32/selftest_modes.py      详细日志: /tmp/selftest_modes.log

注: 该命令依赖 run.sh 导出的 PYTHONPATH（含 config/ 目录）。若手动单独运行，
    需自行补: PYTHONPATH=<根>/config:<根>/src:<根>/src/to32
    在扁平部署（如 /userdata/To32，配置与代码同目录）下直接 cd 进去跑即可。
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
import protocol as P

CMD_PORT = 18080        # 用非默认端口，避免与正在跑的 cmd_watch.py / relay.py 抢 8080/8081
TEL_PORT = 18081
STATE_PATH = "/tmp/selftest_mode_state.json"   # 模式记忆自测临时文件（绝不动生产 mode_state.json）
ESTOP_TIMEOUT = 0.4

# [2026-10-04] 遥测请求(0x0C)暂停开关。
#   与 mode_base.pump_telemetry / link_stm32._poll_loop / mode_rov.on_enter 里
#   的 0x0C 下发点保持同步。
#   注意：/userdata/To32 与 GrandRDK/src/to32 两份部署在 2026-10-04 起状态不同——
#         /userdata/To32 已恢复下发（本文件置 False），
#         GrandRDK/src/to32 仍暂停（那里的副本应置 True）。
TELEMETRY_PAUSED = False


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
    t.check("已记录对端身份（首个下行帧源地址）", getattr(pc, "first_cmd_logged", False))
    t.check("初始模式 = ROV（有线遥控）", disp.mode_id == cfgv.MODE_ROV, "mode_id=%s" % disp.mode_id)
    # [2026-10-03 协议换代] 进有线 ROV 下发的是 0x06（原先 0x03，现为 S100 禁发的"无线ROV"）
    t.check("启动即下发 0x04 有线ROV(0x06)", S.MODE_ROV_TETHERED in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    t.check("启动未发禁发码(0x02 预编程 / 0x03 无线ROV)",
            S.MODE_PREPROGRAM not in link.mode_codes() and S.MODE_ROV not in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    n0c_rov = len(link.func_frames(S.FUNC_TELEMETRY))
    # [2026-10-04] 遥测暂停期间应恒为 0；恢复后应 > 0
    t.check("处于/切到 ROV 即请求回传(0x0C)", (n0c_rov == 0) if TELEMETRY_PAUSED else (n0c_rov > 0),
            "0x0C 帧数=%d (暂停=%s)" % (n0c_rov, TELEMETRY_PAUSED))

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

    print("[4] 切到 AUV：0x04 AUV(0x05)；运动逻辑已摘除 —— 不再自发 0x09")
    state["mode"] = 1
    time.sleep(0.2)
    n09_before_auv = len(link.func_frames(S.FUNC_MOTION))
    t.check("模式切到 AUV", disp.mode_id == cfgv.MODE_AUV, "mode_id=%s" % disp.mode_id)
    # [2026-10-03 协议换代] AUV 下发 0x05（原先 0x02 预编程，属 S100 禁发码）
    t.check("下发 0x04 AUV(0x05)", S.MODE_AUV in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    # [2026-10-05 auv_task 任务化框架接回] 自主运动逻辑回来了：
    #   主链路注册的是真 mode_auv.AuvMode（内部跑 auv_task.TaskRunner），AUV 期间会**持续下发 0x09**。
    #   因此断言从"0x09 必须不增长"翻回"0x09 必须增长"。
    #   ⚠ 这两条与"AUV 有没有接回主链路"强绑定：再摘除要同步翻回 ==，否则回归假红/假绿。
    time.sleep(0.5)
    n09_in_auv = len(link.func_frames(S.FUNC_MOTION))
    t.check("AUV 期间持续下发 0x09（任务化执行器已接回，自主运动在跑）",
            n09_in_auv > n09_before_auv,
            "0x09 帧数 %d -> %d" % (n09_before_auv, n09_in_auv))
    # AUV 模式实例应为真 AuvMode（内部 TaskRunner），而非占位壳 AuvModeStub
    t.check("AUV 注册的是真 AuvMode（任务化执行器）",
            type(disp.mode()).__name__ == "AuvMode",
            "实例=%s" % type(disp.mode()).__name__)
    # AUV 自己掌握遥测节拍（pump_telemetry），0x0C 应继续增长
    n0c_in_auv = len(link.func_frames(S.FUNC_TELEMETRY))
    time.sleep(0.35)
    n0c_after_wait = len(link.func_frames(S.FUNC_TELEMETRY))
    # [2026-10-04] 遥测暂停期间应保持不增长
    t.check("AUV 期间继续请求回传(0x0C 增长)",
            (n0c_after_wait == n0c_in_auv) if TELEMETRY_PAUSED else (n0c_after_wait > n0c_in_auv),
            "0x0C 帧数 %d -> %d (暂停=%s)" % (n0c_in_auv, n0c_after_wait, TELEMETRY_PAUSED))

    print("[5] 切回 ROV：0x09 恢复")
    state["mode"] = 0
    state["surge"] = 0.3
    state["yaw"] = 0.0
    time.sleep(0.3)
    n09_back = len(link.func_frames(S.FUNC_MOTION))
    t.check("切回 ROV 后 0x09 恢复", n09_back > n09_before_auv,
            "0x09 帧数 %d -> %d" % (n09_before_auv, n09_back))
    n0c_back = len(link.func_frames(S.FUNC_TELEMETRY))
    # [2026-10-04] 遥测暂停期间应保持不增长
    t.check("切回 ROV 后恢复请求回传(0x0C 继续增长)",
            (n0c_back == n0c_after_wait) if TELEMETRY_PAUSED else (n0c_back > n0c_after_wait),
            "0x0C 帧数 %d -> %d (暂停=%s)" % (n0c_after_wait, n0c_back, TELEMETRY_PAUSED))

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
    # [2026-10-03] 解除后必须先发 START(0x00) 让固件退出 STANDBY，再恢复 0x09
    t.check("解除后先下发 START(0x00)", S.MODE_START in link.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in link.mode_codes()])
    state["surge"] = 0.4
    time.sleep(0.3)
    t.check("解除后 0x09 恢复", len(link.func_frames(S.FUNC_MOTION)) > n09_after_latch,
            "0x09 帧数=%d" % len(link.func_frames(S.FUNC_MOTION)))

    print("[7.5] 协议换代守卫：S100 禁发码必须被拒")
    # 《多源控制协议设计.md》§5.3：S100 不得发 0x02 预编程 / 0x03 无线ROV
    t.check("frame_mode(0x03 无线ROV) 被拒发（返回 None）",
            S.frame_mode(S.MODE_ROV) is None)
    t.check("frame_mode(0x02 预编程) 被拒发（返回 None）",
            S.frame_mode(S.MODE_PREPROGRAM) is None)
    # 正常码仍可组帧
    t.check("frame_mode(0x05 AUV) 正常组帧",
            S.frame_mode(S.MODE_AUV) is not None
            and S.frame_mode(S.MODE_AUV)[2] == S.FUNC_MODE
            and S.frame_mode(S.MODE_AUV)[3] == S.MODE_AUV)
    t.check("frame_mode(0x06 有线ROV) 正常组帧",
            S.frame_mode(S.MODE_ROV_TETHERED) is not None
            and S.frame_mode(S.MODE_ROV_TETHERED)[3] == S.MODE_ROV_TETHERED)
    # 全项目不可能发出 0x02 测试油门（组帧函数根本不存在）
    t.check("不存在测试油门组帧函数（0x02 水下禁发通路已物理删除）",
            not hasattr(S, "frame_test_throttle"))

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
    # [auv_task 补丁 B7] 记忆文件**不写 AUV**：写进去的必须被改写成 IDLE(-1)。
    #   否则下次上电（手动 ./run.sh 没带 --mode）会自动进 AUV 跑起来 —— 危险。
    #   ⚠ 这两条断言与"B7 有没有打"强绑定：摘掉 B7 要同步翻回来。
    t.check("切到 AUV 后记忆文件被改写成 IDLE(-1)（B7：不写 AUV）", saved == -1,
            "文件值=%r" % (saved,))

    cv_b, d_b = make_disp({"MODE_PERSIST": True, "START_MODE": 0, "MODE_FORCE_START": False})
    t.check("下次启动**不会**沿用 AUV（记忆里是 IDLE）", d_b.mode_id == cv_b.MODE_IDLE,
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

    print("[11] 2026-09-15 变更：显式急停帧 / 参数单一来源 / $TEL 单一映射 / PID 组帧安全")
    cv_h, d_h = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "ESTOP_LATCH": True, "YAW_RATE_DPS": 90.0, "DEPTH_RATE_CMS": 30.0})
    m_h = d_h.mode()
    t.check("ROV 速率读自 config（YAW_RATE_DPS=90 / DEPTH_RATE_CMS=30）",
            abs(m_h.yaw_rate - 90.0) < 1e-9 and abs(m_h.depth_rate - 30.0) < 1e-9,
            "yaw_rate=%s depth_rate=%s" % (m_h.yaw_rate, m_h.depth_rate))
    m_h.target_depth_cm = 100.0
    m_h._last_t = time.time() - 0.5              # dt 会被 wait_dt_max(0.25) 截断
    m_h.on_cmd(P.parse_cmd("$CMD,0.00,0.00,1.00,0.00,0,0,0,0,0#"))
    t.check("heave 积分速率跟随 config（0.25s 满偏 = depth_rate×0.25 cm 上浮）",
            abs((100.0 - m_h.target_depth_cm) - m_h.depth_rate * 0.25) < 0.01,
            "target_depth_cm=%.4f" % m_h.target_depth_cm)

    n09_h = len(d_h.link_stm32.func_frames(S.FUNC_MOTION))
    d_h.on_pc_event("raw", "$ESTOP#")
    t.check("$ESTOP# 立即急停并锁存", d_h.estop_latch, "reason=%s" % d_h.estop_reason)
    t.check("$ESTOP# 下发了 0x04 急停(0x01)", S.MODE_ESTOP in d_h.link_stm32.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in d_h.link_stm32.mode_codes()])
    for _ in range(3):                           # 持续 $CMD（满杆位）仍不应有 0x09
        d_h.on_pc_event("cmd", P.parse_cmd("$CMD,1.00,1.00,1.00,1.00,0,0,0,0,0#"))
    t.check("$ESTOP# 锁存期间持续 $CMD 仍抑制 0x09",
            len(d_h.link_stm32.func_frames(S.FUNC_MOTION)) == n09_h,
            "0x09 帧数 %d -> %d" % (n09_h, len(d_h.link_stm32.func_frames(S.FUNC_MOTION))))
    d_h.on_pc_event("raw", "$ESTOP,0#")
    t.check("$ESTOP,0# 解除 $ESTOP# 造成的锁存", not d_h.estop_latch)
    t.check("解除后先下发 START(0x00)",
            S.MODE_START in d_h.link_stm32.mode_codes(),
            "0x04 序列=%s" % [hex(x) for x in d_h.link_stm32.mode_codes()])
    d_h.on_pc_event("cmd", P.parse_cmd("$CMD,0.50,0.00,0.00,0.00,0,0,0,0,0#"))
    t.check("解除后 0x09 恢复",
            len(d_h.link_stm32.func_frames(S.FUNC_MOTION)) > n09_h,
            "0x09 帧数=%d" % len(d_h.link_stm32.func_frames(S.FUNC_MOTION)))

    t.check("ROV build_telemetry == tel_builder 统一映射（单一来源）",
            bool(tel10) and m_f.build_telemetry() == TB.build_tel(tel10))

    try:
        f_over = S.frame_set_pid(0, 1e6, -1e6, 500.0)
        f_lim = S.frame_set_pid(0, 327.67, -327.67, 327.67)
        t.check("frame_set_pid 超范围钳位且不抛异常（= int16 边界帧）",
                bytes(f_over) == bytes(f_lim) and len(f_over) == 11 and f_over[1] == 0x0A,
                "帧=%s" % " ".join("%02X" % b for b in f_over))
    except Exception as e:                        # noqa: BLE001 - 自测要覆盖异常路径
        t.check("frame_set_pid 超范围钳位且不抛异常", False, "异常=%s" % e)

    print("[12] 空闲静默：摇杆回中不发 0x09（2026-09-15 需求）")
    cv_i, d_i = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "MOTION_SILENT_WHEN_IDLE": True})
    m_i = d_i.mode()
    zero_i = P.parse_cmd("$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#")
    n09_i0 = len(d_i.link_stm32.func_frames(S.FUNC_MOTION))
    d_i.on_pc_event("cmd", zero_i)                    # 第 1 帧：建立基准，必发
    n09_i1 = len(d_i.link_stm32.func_frames(S.FUNC_MOTION))
    t.check("静止第 1 帧发出（建立基准）", n09_i1 == n09_i0 + 1, "0x09=%d" % n09_i1)
    for _ in range(5):
        d_i.on_pc_event("cmd", zero_i)                # 静止持续 5 帧
    n09_idle = len(d_i.link_stm32.func_frames(S.FUNC_MOTION))
    t.check("静止后续 5 帧全部静默（F 口无 0x09）", n09_idle == n09_i1,
            "0x09 %d -> %d" % (n09_i1, n09_idle))
    d_i.on_pc_event("cmd", P.parse_cmd("$CMD,0.60,0.00,0.00,0.00,0,0,0,0,0#"))
    t.check("推杆立即下发（载荷变化）",
            len(d_i.link_stm32.func_frames(S.FUNC_MOTION)) == n09_idle + 1)
    d_i.on_pc_event("cmd", zero_i)
    n09_center = len(d_i.link_stm32.func_frames(S.FUNC_MOTION))
    t.check("回中补发一帧（把推力归零）", n09_center == n09_idle + 2, "0x09=%d" % n09_center)
    last_i = d_i.link_stm32.func_frames(S.FUNC_MOTION)[-1]
    t.check("回中帧的 surge/sway 归零", last_i[11] == 0 and last_i[12] == 0,
            "surge=%d sway=%d" % (last_i[11], last_i[12]))
    for _ in range(5):
        d_i.on_pc_event("cmd", zero_i)
    t.check("回中之后再次静默",
            len(d_i.link_stm32.func_frames(S.FUNC_MOTION)) == n09_center)
    t.check("静默帧数有计数（5s 汇报可见）", m_i.idle_skipped >= 10,
            "idle_skipped=%d" % m_i.idle_skipped)

    # 2026-09-21 回归: "持续前推左摇杆(载荷恒定但非中位) → 必须每帧下发 0x09"
    # 旧字节比对逻辑在 surge/sway 恒定时会静默; 新"仅回中静默"逻辑必须放行。
    push_i = P.parse_cmd("$CMD,0.60,0.00,0.00,0.00,0,0,0,0,0#")
    cv_p, d_p = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "MOTION_SILENT_WHEN_IDLE": True,
                           "MOTION_SILENT_WHEN_IDLE_AXES_ONLY": True})
    n09_p0 = len(d_p.link_stm32.func_frames(S.FUNC_MOTION))
    for _ in range(5):                                  # 持续前推 5 帧(载荷不变)
        d_p.on_pc_event("cmd", push_i)
    t.check("[2026-09-21] 持续推杆 5 帧(载荷不变) → 全部下发",
            len(d_p.link_stm32.func_frames(S.FUNC_MOTION)) == n09_p0 + 5,
            "0x09 %d -> %d" % (n09_p0, len(d_p.link_stm32.func_frames(S.FUNC_MOTION))))
    yaw_i = P.parse_cmd("$CMD,0.00,0.00,0.00,0.30,0,0,0,0,0#")
    cv_y, d_y = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "MOTION_SILENT_WHEN_IDLE": True,
                           "MOTION_SILENT_WHEN_IDLE_AXES_ONLY": True})
    n09_y0 = len(d_y.link_stm32.func_frames(S.FUNC_MOTION))
    for _ in range(5):
        d_y.on_pc_event("cmd", yaw_i)
    t.check("[2026-09-21] yaw 持续非 0(转向) 5 帧 → 全部下发",
            len(d_y.link_stm32.func_frames(S.FUNC_MOTION)) == n09_y0 + 5,
            "0x09 %d -> %d" % (n09_y0, len(d_y.link_stm32.func_frames(S.FUNC_MOTION))))
    heave_i = P.parse_cmd("$CMD,0.00,0.00,0.50,0.00,0,0,0,0,0#")
    cv_h, d_h = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "MOTION_SILENT_WHEN_IDLE": True,
                           "MOTION_SILENT_WHEN_IDLE_AXES_ONLY": True})
    n09_h0 = len(d_h.link_stm32.func_frames(S.FUNC_MOTION))
    for _ in range(5):
        d_h.on_pc_event("cmd", heave_i)
    t.check("[2026-09-21] heave 持续非 0(上浮) 5 帧 → 全部下发",
            len(d_h.link_stm32.func_frames(S.FUNC_MOTION)) == n09_h0 + 5,
            "0x09 %d -> %d" % (n09_h0, len(d_h.link_stm32.func_frames(S.FUNC_MOTION))))
    # 旧字节比对回退验证: 持续推杆 5 帧 → 仅发 1 帧(已知 bug, 仅留作回退)
    cv_legacy, d_legacy = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                                      "MOTION_SILENT_WHEN_IDLE": True,
                                      "MOTION_SILENT_WHEN_IDLE_AXES_ONLY": False})
    n09_l0 = len(d_legacy.link_stm32.func_frames(S.FUNC_MOTION))
    for _ in range(5):
        d_legacy.on_pc_event("cmd", push_i)
    t.check("[2026-09-21] 旧逻辑回退: 持续推杆 5 帧 → 仅发 1 帧(已知 bug, 仅作回退)",
            len(d_legacy.link_stm32.func_frames(S.FUNC_MOTION)) == n09_l0 + 1,
            "0x09 %d -> %d" % (n09_l0, len(d_legacy.link_stm32.func_frames(S.FUNC_MOTION))))

    cv_j, d_j = make_disp({"MODE_PERSIST": False, "START_MODE": 0, "MODE_FORCE_START": True,
                           "MOTION_SILENT_WHEN_IDLE": False})
    n09_j0 = len(d_j.link_stm32.func_frames(S.FUNC_MOTION))
    for _ in range(3):
        d_j.on_pc_event("cmd", zero_i)
    t.check("开关关闭时维持旧行为（静止也每帧发）",
            len(d_j.link_stm32.func_frames(S.FUNC_MOTION)) == n09_j0 + 3,
            "0x09=%d" % len(d_j.link_stm32.func_frames(S.FUNC_MOTION)))

    passed = sum(1 for _n, ok in t.results if ok)
    total = len(t.results)
    print("\n结果: %d/%d 通过" % (passed, total))
    print("详细日志: /tmp/selftest_modes.log")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(run())