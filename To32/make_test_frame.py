# -*- coding: utf-8 -*-
"""生成一帧"假下位机"遥测回帧（48B），供 PC 串口助手发给中位机。

用法:
    python3 make_test_frame.py                  # 默认演示值（实际深度55cm / 航向30°）
    python3 make_test_frame.py 123 45           # 自定义: 实际深度(cm) 实际航向(度)

自校验（本脚本的价值就在这）:
    造出的字节会真的过一遍 link_stm32.FrameParser -> parse_telemetry -> tel_builder.tel_values，
    确认中位机能原样收下并解析，再打印它在 $TEL 41 字段里对应的索引值。
    这样"发的字节"和"上位机该显示的数字"是同一份代码算出来的，不会对不上。
"""
import struct
import sys

import link_stm32 as L
import tel_builder


def build_demo(actual_depth_cm=55.0, actual_yaw_deg=30.0):
    """按 V2 遥测 44B DATA 布局打包一个小端帧（字段序 = link_stm32.parse_telemetry 的字段序）。"""
    d = struct.pack(
        "<12h8hHH",
        # ---- 12h: 姿态/角速度/加速度（全部 ×100）----
        0,                            # 0..1  目标 Pitch  0.00°
        int(round(actual_yaw_deg * 100)),   # 2..3  目标 Yaw    与实测同（模拟"已到位"）
        0,                            # 4..5  目标 Roll   0.00°
        int(round(1.50 * 100)),       # 6..7  实际 Pitch  1.50°
        int(round(actual_yaw_deg * 100)),   # 8..9  实际 Yaw    可自定
        int(round(-0.50 * 100)),      # 10..11 实际 Roll  -0.50°
        int(round(0.20 * 100)),       # 12..13 角速度 P    0.20°/s
        int(round(0.30 * 100)),       # 14..15 角速度 Y    0.30°/s
        int(round(-0.10 * 100)),      # 16..17 角速度 R   -0.10°/s
        int(round(0.05 * 100)),       # 18..19 加速度 X    0.05
        int(round(0.02 * 100)),       # 20..21 加速度 Y    0.02
        int(round(9.80 * 100)),       # 22..23 加速度 Z    9.80
        # ---- 8h: 电机 0~7 目标转速（原值，不除100）----
        0, 100, 200, 300, 0, 100, 200, 300,
        # ---- HH: 目标深度 / 实际深度（uint16LE, cm×100）----
        50 * 100,                                        # 40..41 目标深度 50.00cm
        int(round(actual_depth_cm * 100)),               # 42..43 实际深度
    )
    assert len(d) == 44, "DATA 必须 44 字节，实际 %d" % len(d)
    return L.build_frame(L.FUNC_TELEMETRY, d)


def show(frame, tag="帧"):
    """自校验：过一遍真解析路径，并打印 $TEL 关键索引。"""
    hexs = " ".join("%02X" % b for b in frame)
    print("=== %s ===" % tag)
    print("整帧 %d 字节 (LEN=0x%02X=0x2F? %s)  CD=%s DC=%s"
          % (len(frame), frame[1], frame[1] == 0x2F,
             "OK" if frame[0] == 0xCD else "BAD",
             "OK" if frame[-1] == 0xDC else "BAD"))
    print("HEX(带空格): %s" % hexs)
    print("HEX(无空格): %s" % hexs.replace(" ", ""))

    # 真的用中位机的收帧状态机走一遍
    frames = L.FrameParser().feed(frame)
    assert len(frames) == 1, "解析出的帧数应为 1，实际 %d" % len(frames)
    func, data = frames[0]
    assert func == L.FUNC_TELEMETRY, "FUNC 应为 0x0C，实际 0x%02X" % func
    tel = L.parse_telemetry(data)
    assert tel, "parse_telemetry 返回空（DATA 不足 44B）"

    vals = tel_builder.tel_values(tel)
    print("--> 中位机解析结果（上位机 $TEL 索引 = 值）:")
    for name, idx in (("roll", 0), ("pitch", 1), ("yaw", 2),
                      ("gyro gx(roll)", 3), ("gyro gy(pitch)", 4), ("gyro gz(yaw)", 5),
                      ("深度 m", 6), ("目标深度 m", 16), ("alt", 40)):
        print("      $TEL[%2d] %-14s = %.2f" % (idx, name, vals[idx]))
    print("      $TEL[25..32] 推进器0~7 = %s"
          % ", ".join("%.1f" % vals[25 + i] for i in range(8)))
    print("      $TEL[33..36] 推进器8~11 = %s   <- V2 只有 8 路，恒 0"
          % ", ".join("%.1f" % vals[33 + i] for i in range(4)))
    print("      $TEL[37..39] 加速度 ax/ay/az = %.2f / %.2f / %.2f"
          % (vals[37], vals[38], vals[39]))
    print("      完整 $TEL: %s" % tel_builder.build_tel(tel).strip())
    print()


def main():
    if len(sys.argv) >= 3:
        depth = float(sys.argv[1])
        yaw = float(sys.argv[2])
    else:
        depth, yaw = 55.0, 30.0

    print("目标: 实际深度 %.1fcm / 实际航向 %.2f°  (目标深度固定 50.0cm)" % (depth, yaw))
    print()
    show(build_demo(depth, yaw), "帧1: 实际深度 %.1fcm, 航向 %.1f°" % (depth, yaw))
    # 第二帧换个值，方便你确认"上位机数字真的跟着串口变"，而不是卡在某个缓存值
    show(build_demo(123.0, -45.0), "帧2: 实际深度 123.0cm, 航向 -45°（用来验证数字会变）")


if __name__ == "__main__":
    main()
