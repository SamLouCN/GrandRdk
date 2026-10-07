# -*- coding: utf-8 -*-
"""合并版 v3.8 协议级回归测试（替代缺失的 test_v36/test_v38）。

运行: python -X utf8 -B tests/test_v39.py
只测协议与静态装配, 不打开网络/串口/手柄/摄像头/UI。
"""
import io
import os
import sys
import struct
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import protocol
from protocol import (build_cmd, build_emergency_stop, build_pid, parse_pid,
                      build_estop, build_estop_release, parse_telemetry,
                      parse_msg, parse_alt, parse_vid,
                      ReferenceTelemetryBuffer, PID_REFERENCE_MAX_INDEX)
import task_pid_wire
from task_pid_wire import (build_task_pid, parse_task_pid,
                           build_task_pid_ack, parse_task_pid_ack)


class BinaryPidTests(unittest.TestCase):
    """参考 0x01 二进制 PID 帧（v3.7）。"""

    def test_frame_layout_exact_bytes(self):
        frame = build_pid(3, 1.0, 0.5, 0.25)
        self.assertEqual(frame[:3], b"\xcd\x0a\x01")
        self.assertEqual(frame[-1], 0xdc)
        self.assertEqual(len(frame), 11)
        ch, p, i, d = struct.unpack("<Bhhh", frame[3:-1])
        self.assertEqual((ch, p, i, d), (3, 100, 50, 25))

    def test_percent_scale_and_round_half_up(self):
        frame = build_pid(0, 1.234, 0, 0)
        self.assertEqual(struct.unpack("<h", frame[4:6])[0], 123)  # 123.4 -> 123
        frame = build_pid(0, -1.5, 0, 0)
        self.assertEqual(struct.unpack("<h", frame[4:6])[0], -150)

    def test_parse_roundtrip(self):
        for idx in (0, 1, 2, 3, 7):
            parsed = parse_pid(build_pid(idx, 12.34, -5.67, 0.01))
            self.assertEqual(parsed["ch"], idx)
            self.assertAlmostEqual(parsed["p"], 12.34)
            self.assertAlmostEqual(parsed["i"], -5.67)
            self.assertAlmostEqual(parsed["d"], 0.01)

    def test_rejects_old_text_and_bad_frames(self):
        self.assertIsNone(parse_pid(b"$PID,0,1.0000,0.0100,0.0020#\r\n"))
        self.assertIsNone(parse_pid(b"\xcd\x0a\x01\x00\x64\x00\x00\x00\x00\x00"))  # 少 DC
        self.assertIsNone(parse_pid(b"x" * 11))
        self.assertIsNone(parse_pid(b""))

    def test_index_range_validation(self):
        self.assertRaises(ValueError, build_pid, 8, 1, 0, 0)       # 超 7
        self.assertRaises(ValueError, build_pid, -1, 1, 0, 0)
        self.assertRaises(ValueError, build_pid, True, 1, 0, 0)    # bool 不算 int
        self.assertRaises(ValueError, build_pid, 0, 400, 0, 0)     # 超 ±327.68
        self.assertRaises(ValueError, build_pid, 0, float("nan"), 0, 0)


class ReferenceBufferTests(unittest.TestCase):
    """48B 参考遥测应答提取（容忍拆包/粘包/调试文本）。"""

    def _frame(self):
        return b"\xcd\x2f\x0c" + bytes(range(44)) + b"\xdc"  # 3 + 44 + 1 = 48

    def test_single_frame_and_split_packets(self):
        buf = ReferenceTelemetryBuffer()
        frame = self._frame()
        self.assertEqual(buf.feed(frame[:5]), [])
        self.assertEqual(buf.feed(frame[5:]), [frame])

    def test_stuck_frames_and_garbage(self):
        buf = ReferenceTelemetryBuffer()
        frame = self._frame()
        self.assertEqual(len(buf.feed(b"debug text" + frame + frame + b"tail")), 2)

    def test_truncated_frame_waits(self):
        buf = ReferenceTelemetryBuffer()
        frame = self._frame()
        self.assertEqual(buf.feed(frame[:47]), [])
        self.assertEqual(buf.feed(frame[47:]), [frame])


class TaskPidWireTests(unittest.TestCase):
    """$TASKPID / $TASKPID_ACK（S100 过门参数）。"""

    def test_build_and_parse_task_pid(self):
        frame = build_task_pid("req-abc_1", "gate", 30.0, 0.0, 12.0)
        self.assertEqual(frame, b"$TASKPID,req-abc_1,gate,30.00,0.00,12.00#\r\n")
        parsed = parse_task_pid(frame)
        self.assertEqual(parsed["request"], "req-abc_1")
        self.assertEqual(parsed["loop"], "gate")
        self.assertEqual(parsed["p"], 30.0)

    def test_ack_roundtrip_ok_and_err(self):
        ok = build_task_pid_ack("req1", "gate", gains=(30.0, 0.0, 12.0))
        self.assertEqual(ok, "$TASKPID_ACK,req1,gate,OK,30.00,0.00,12.00#\r\n")
        parsed = parse_task_pid_ack(ok)
        self.assertTrue(parsed["ok"])
        self.assertEqual(parsed["gains"], (30.0, 0.0, 12.0))
        err = build_task_pid_ack("req1", "gate", error="BAD_PARAM")
        parsed = parse_task_pid_ack(err)
        self.assertFalse(parsed["ok"])
        self.assertEqual(parsed["error"], "BAD_PARAM")

    def test_rejects_foreign_frames(self):
        self.assertIsNone(parse_task_pid_ack(b"$TEL,...#\r\n"))
        self.assertIsNone(parse_task_pid_ack(b"$TASKPID_ACK,x,gate,OK,1#\r\n"))  # 字段数错
        self.assertRaises(ValueError, build_task_pid, "bad id!", "gate", 1, 0, 0)
        self.assertRaises(ValueError, build_task_pid, "req", "GATE", 1, 0, 0)  # loop 必须小写


class TelemetryFusedDepthTests(unittest.TestCase):
    """板端融合值（v3.6 合并回归）。"""

    def _tel(self, fields):
        return "$TEL," + ",".join(str(v) for v in fields) + "#\r\n"

    def test_long_frame_carries_fused_depth_and_clearance(self):
        # 0..40 为 v3.1 既有字段, 41/42 为板端融合深度/离底净空
        fields = [0.0] * 43
        fields[0], fields[1], fields[2] = 0.1, 0.2, 0.3      # roll/pitch/yaw
        fields[3:6] = (0.0, 0.0, 0.0)                        # gx/gy/gz
        fields[6] = 3.5                                      # depth
        fields[40] = 2.5                                     # alt
        fields[41] = 3.46                                    # fused_depth
        fields[42] = 0.84                                    # clearance
        t = parse_telemetry(self._tel(fields))
        self.assertIsNotNone(t)
        self.assertAlmostEqual(t["fused_depth"], 3.46)
        self.assertAlmostEqual(t["clearance"], 0.84)
        self.assertAlmostEqual(t["alt"], 2.5)

    def test_short_frame_has_no_fused_keys(self):
        fields = [0.0] * 41  # 到 alt 为止(0..40), 没有 41/42
        fields[0], fields[1], fields[2] = 0.1, 0.2, 0.3
        fields[6] = 3.5
        t = parse_telemetry(self._tel(fields))
        self.assertIsNotNone(t)
        self.assertNotIn("fused_depth", t)
        self.assertNotIn("clearance", t)

    def test_legacy_seven_field_frame_still_parses(self):
        t = parse_telemetry("$TEL,0.1,0.2,0.3,0,0,0,3.5#\r\n")
        self.assertIsNotNone(t)
        self.assertAlmostEqual(t["depth"], 3.5)


class MiscProtocolTests(unittest.TestCase):
    """其余协议帧快速回归。"""

    def test_cmd_and_estop(self):
        self.assertEqual(build_cmd(1, 0, 0, 0, 0, 0), "$CMD,1.00,0.00,0.00,0.00,0,0,0,0,0#\r\n")
        self.assertEqual(build_emergency_stop(), "$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#\r\n")
        self.assertEqual(build_estop(), "$ESTOP#\r\n")
        self.assertEqual(build_estop_release(), "$ESTOP,0#\r\n")

    def test_msg_and_alt_and_vid(self):
        msg = parse_msg("$MSG,12:00:00,WARN,DEGRADE,SIT_BOTTOM,深度源不可用#\r\n")
        self.assertEqual(msg["level"], "WARN")
        self.assertEqual(msg["code"], "DEGRADE")
        self.assertEqual(msg["stage"], "SIT_BOTTOM")
        alt = parse_alt("$ALT,B,533,OK#\r\n")
        self.assertEqual(alt, {"ch": "B", "mm": 533, "status": "OK"})
        self.assertEqual(parse_vid("$VID,1#\r\n"), True)


class MergedSourceAssemblyTests(unittest.TestCase):
    """合并后的 pc_main2.py 静态装配检查（不导入 PyQt）。"""

    def test_fused_depth_rows_present_in_pc_main2(self):
        with io.open(os.path.join(ROOT, "pc_main2.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn('("fdepth","板端融合深度")', src)
        self.assertIn('("clr","离底净空")', src)
        self.assertIn('"fused_depth":"板端融合深度","clearance":"离底净空"', src)
        self.assertIn("板端融合值上屏", src)          # docstring 条目22

    def test_pid_modules_wired_in_pc_main2(self):
        with io.open(os.path.join(ROOT, "pc_main2.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn("from pid_panel import ControlPidPanel", src)
        self.assertIn("from task_pid_wire import build_task_pid, parse_task_pid_ack", src)
        self.assertIn('"PID 调参"', src)

    def test_mode_button_releases_estop_latch(self):
        """方案A: _set_mode 在急停待机态先发 $ESTOP,0# 解除板端锁存再切模式。"""
        with io.open(os.path.join(ROOT, "pc_main2.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn("急停锁存中按模式按钮", src)
        self.assertIn("jt.send_estop_release()", src)
        self.assertIn("已发 $ESTOP,0# 解除锁存", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
