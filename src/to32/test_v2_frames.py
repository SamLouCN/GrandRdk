#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""5 条 V2 遥测测试帧的**端到端测试**（不需要真实下位机、不需要上位机 UI）

被测对象 = 中位机自己的代码路径（不是测试脚本自说自话）：
    link_stm32.FrameParser（收帧状态机，含跨包切分）
      -> link_stm32.parse_telemetry（按 V2 §5 布局解析）
      -> mode_dispatcher.on_stm32_telemetry（编排 + 冷启动锚定）
      -> mode_rov/build_telemetry -> tel_builder（41 字段统一映射）
      -> pc_link.send_telem（上行给上位机）

测试数据 = make_test_frame.py --bin 生成的 5×48B 二进制（默认 /tmp/v2_telemetry_5frames.bin）

判据：
  A 文件就是 5 帧，每帧 48B、LEN=0x2F、CD…DC
  B 逐字节喂入状态机能完整还原（等价于串口被拆包到达）
  C 每帧解析出的关键字段 == 期望值（深度/目标深度/姿态/电机）
  D 走完编排层：收到 5 帧遥测、上行 5 条 $TEL、关键索引值正确、41 字段齐全
  E 冷启动锚定生效（首帧把内部目标量锚到实测值）

运行: cd /userdata/GrandRDK && python3 src/to32/test_v2_frames.py [bin 路径]
"""
from __future__ import annotations

import queue
import sys
import time

import link_stm32 as S
import main as M
import protocol as P

BIN = sys.argv[1] if len(sys.argv) > 1 else "/tmp/v2_telemetry_5frames.bin"
STATE_PATH = "/tmp/test_v2_frames_state.json"

results = []


def check(name, ok, detail=""):
    ok = bool(ok)
    results.append(ok)
    print("  %s %s%s" % ("[PASS]" if ok else "[FAIL]", name, ("  | " + detail) if detail else ""))


class RecLink(S.Stm32Link):
    """sim 链路 + 记录下行帧（与 selftest_modes.py 同款桩）"""

    def __init__(self, cfgv, log):
        super().__init__(cfgv, log, port_spec="sim", on_telemetry=None)
        self.frames = []

    def send(self, frame, note=""):
        if frame:
            self.frames.append((time.time(), note, bytes(frame)))
        return super().send(frame, note)

    def func_frames(self, func):
        return [f for (_t, _n, f) in self.frames if len(f) > 2 and f[2] == func]


EXPECT = [
    # 帧1 水面待机
    dict(depth=0.0, tdepth=0.0, roll=0.0, pitch=0.0, yaw=0.0, motors=[0] * 8),
    # 帧2 浅水下潜中
    dict(depth=55.0, tdepth=50.0, roll=-0.5, pitch=1.5, yaw=30.0,
         motors=[0, 100, 200, 300, 0, 100, 200, 300]),
    # 帧3 定深巡航
    dict(depth=152.0, tdepth=152.0, roll=0.30, pitch=-0.80, yaw=-45.20, motors=[120] * 8),
    # 帧4 边界
    dict(depth=200.0, tdepth=200.0, roll=-8.90, pitch=8.90, yaw=-179.99,
         motors=[-32767, 32767, 0, 0, -1000, 1000, 0, 0]),
    # 帧5 负深度按 0 + 电机饱和
    dict(depth=0.0, tdepth=0.0, roll=15.0, pitch=15.0, yaw=0.0, motors=[32767] * 8),
]


def fields(txt):
    """$TEL 文本 -> float 列表（丢帧头与结尾 #）"""
    return [float(x) for x in txt.strip().rstrip("#").split(",")[1:]]


def main():
    print("V2 遥测测试帧 端到端测试 | 数据: %s" % BIN)
    print("=" * 74)

    # ---------- A 文件级 ----------
    try:
        raw = open(BIN, "rb").read()
    except OSError as e:
        print("  [FAIL] 无法读取测试数据: %s" % e)
        print("  提示: 先运行 python3 make_test_frame.py --bin %s" % BIN)
        return 2
    check("A1 文件 = 5 帧 × 48B = 240B", len(raw) == 240, "%d 字节" % len(raw))
    frames_raw = [raw[i * 48:(i + 1) * 48] for i in range(len(raw) // 48)]
    check("A2 每帧 48B / LEN=0x2F / FUNC=0x0C / 头 CD / 尾 DC",
          len(frames_raw) == 5 and all(
              len(f) == 48 and f[0] == 0xCD and f[1] == 0x2F
              and f[2] == S.FUNC_TELEMETRY and f[-1] == 0xDC for f in frames_raw))

    # ---------- B 收帧状态机（逐字节 = 最苛刻的拆包） ----------
    parser = S.FrameParser()
    got = []
    for i in range(len(raw)):
        got.extend(parser.feed(raw[i:i + 1]))
    check("B1 逐字节喂入还原出 5 帧", len(got) == 5, "解析出 %d 帧" % len(got))
    check("B2 还原的 DATA 与源文件逐字节相同",
          len(got) == 5 and all(got[i][1] == frames_raw[i][3:-1] for i in range(5)))
    check("B3 全部是 0x0C 遥测帧", all(f == S.FUNC_TELEMETRY for f, _d in got))

    # ---------- C 字段级 ----------
    tels = []
    for i, (func, data) in enumerate(got):
        tel = S.parse_telemetry(data)
        e = EXPECT[i]
        if not tel:
            check("C%d 帧%d 解析" % (i + 1, i + 1), False, "parse_telemetry 返回空")
            return 1
        ok = (abs(tel["actual_depth_cm"] - e["depth"]) < 1e-6
              and abs(tel["target_depth_cm"] - e["tdepth"]) < 1e-6
              and abs(tel["actual_roll"] - e["roll"]) < 1e-6
              and abs(tel["actual_pitch"] - e["pitch"]) < 1e-6
              and abs(tel["actual_yaw"] - e["yaw"]) < 1e-6
              and list(tel["motors"]) == e["motors"])
        check("C%d 帧%d 关键字段 == 期望" % (i + 1, i + 1), ok,
              "深度=%.2f/目标%.2f roll=%.2f pitch=%.2f yaw=%.2f 电机=%s"
              % (tel["actual_depth_cm"], tel["target_depth_cm"], tel["actual_roll"],
                 tel["actual_pitch"], tel["actual_yaw"], list(tel["motors"])[:3]))
        tels.append(tel)

    # ---------- D/E 编排层 + $TEL 上行 ----------
    log = M.make_logger("/tmp/test_v2_frames.log", quiet=True)
    cfgv = M.build_cfg({
        "CMD_PORT": 18090, "TELEM_PORT": 18091, "PC_BIND_IP": "127.0.0.1",
        "ESTOP_TIMEOUT_S": 0.0,            # 测试内不触发 deadman（避免干扰计数）
        "ESTOP_LATCH": True, "REPORT_S": 0.0,
        "STM32_POLL_HZ": 20.0, "TICK_HZ": 20.0,
        "TEL_HZ": 0.01,                    # 关掉自动 $TEL 节拍，本测试手动触发以便精确计数
        "MODE_PERSIST": False, "MODE_STATE_PATH": STATE_PATH,
        "START_MODE": 0, "MODE_FORCE_START": True,
    }, warn=log)

    rx = queue.Queue()
    pc = M.PC.PcLink(cfgv, log, rx)
    link = RecLink(cfgv, log)
    disp = M.Dispatcher(cfgv, pc, link, log, rx_queue=rx)
    link.on_telemetry = disp.on_stm32_telemetry
    sent = []
    pc.send_telem = lambda text: (sent.append(text), True)[1]

    disp.start()
    try:
        for tel in tels:
            link.on_telemetry(tel)      # 等价于真实链路收满一帧后的回调
            disp._send_tel()            # 触发 $TEL 上行（真实由 TEL_HZ 节拍驱动）
        check("D1 编排层累计收到 5 帧遥测", disp.stats["tel"] == 5, "tel=%d" % disp.stats["tel"])
        check("D2 上行 5 条 $TEL", len(sent) == 5, "上行=%d tel_tx=%d" % (len(sent), disp.stats["tel_tx"]))
        check("D3 每条 $TEL 都是 41 字段", len(sent) == 5 and all(len(fields(x)) == 41 for x in sent),
              "字段数=%s" % [len(fields(x)) for x in sent])
        if len(sent) == 5:
            f2 = fields(sent[1])
            check("D4 帧2 -> $TEL[6]深度=0.55m, [2]yaw=30.00, [1]pitch=1.50, [25..32]推进器",
                  abs(f2[6] - 0.55) < 1e-6 and abs(f2[2] - 30.0) < 1e-6
                  and abs(f2[1] - 1.50) < 1e-6
                  and [round(f2[25 + i], 3) for i in range(8)] == [0.0, 0.1, 0.2, 0.3, 0.0, 0.1, 0.2, 0.3],
                  "深度=%s yaw=%s" % (f2[6], f2[2]))
            f3 = fields(sent[2])
            check("D5 帧3 -> 目标=实际=1.52m（到位）",
                  abs(f3[6] - 1.52) < 1e-6 and abs(f3[16] - 1.52) < 1e-6,
                  "深度=%s 目标=%s" % (f3[6], f3[16]))
            f4 = fields(sent[3])
            check("D6 帧4 -> 深度=2.00m（固件上限）+ 推进器超量程 -32.767/32.767",
                  abs(f4[6] - 2.0) < 1e-6 and abs(f4[25] + 32.767) < 0.01 and abs(f4[26] - 32.767) < 0.01,
                  "深度=%s thr0=%s thr1=%s" % (f4[6], f4[25], f4[26]))
            f5 = fields(sent[4])
            check("D7 帧5 -> 负深度按 0（线上 0.00m）",
                  abs(f5[6]) < 1e-9, "深度=%s" % f5[6])
        m = disp.mode()
        check("E1 冷启动锚定：首帧后 anchored=True 且目标深度锚到实测 0.00cm",
              bool(disp.anchored) and abs(float(getattr(m, "target_depth_cm", -1)) - 0.0) < 1e-6,
              "anchored=%s target_depth=%.2f" % (disp.anchored, float(getattr(m, "target_depth_cm", -1))))
        check("E2 编排层未下发 0x09（本测试未喂 $CMD，静止不应有运动帧）",
              len(link.func_frames(S.FUNC_MOTION)) == 0,
              "0x09 帧数=%d" % len(link.func_frames(S.FUNC_MOTION)))
    finally:
        disp.stop()
        pc.stop()

    ok = sum(1 for r in results if r)
    print("=" * 74)
    print("结果: %d/%d PASS" % (ok, len(results)))
    print("结论: %s" % ("全部通过 —— 5 条测试帧可被中位机正确接收、解析并上行" if ok == len(results) else "存在失败项，见上"))
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
