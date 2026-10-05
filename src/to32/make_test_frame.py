#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成符合 V2 协议的**遥测回传帧**（48B）测试数据 —— 5 个场景

规范依据：
  1) `上位机对接说明_V2.md` §5 / §6（遥测返回帧 0x0C，固定 48 字节）
  2) 固件源码 `JXZK_XLB_lib/Src/JXZK_XLB_Protocol.c:910-959`（Protocol_SendTelemetryV2）

帧格式：`CD 2F 0C <44B DATA> DC`   整帧 48B，LEN=0x2F=44+3，多字节**小端**
DATA 布局（偏移相对 DATA；整帧偏移 = DATA 偏移 + 3）：
   0..1  目标 Pitch   2..3  目标 Yaw    4..5  目标 Roll      int16LE 度×100
   6..7  实际 Pitch   8..9  实际 Yaw   10..11 实际 Roll      int16LE 度×100
  12..13 角速度 Pitch 14..15 角速度 Yaw 16..17 角速度 Roll     int16LE 度/秒×100
  18..19 加速度 X    20..21 加速度 Y   22..23 加速度 Z       int16LE m/s²×100
  24..39 电机 0~7 目标转速（8×int16LE，**原值**，不除 100）
  40..41 目标深度      uint16LE cm×100
  42..43 实际深度      uint16LE cm×100（**固件对负值强制为 0**）

自校验（本脚本的价值所在）：
  造出的字节会真的过一遍 `link_stm32.FrameParser → parse_telemetry → tel_builder.tel_values`，
  确认中位机能原样收下并解析，再打印它对应的 `$TEL` 41 字段。
  —— "发出去的字节"与"上位机该显示的数字"由同一份代码算出，不会对不上。

用法：
    python3 make_test_frame.py               # 打印 5 个场景（HEX + 解码 + $TEL）
    python3 make_test_frame.py --hex-only    # 只打印 5 行紧凑 HEX（粘串口助手用）
    python3 make_test_frame.py --bin f.bin   # 追加写二进制帧（喂给中位机/假下位机）
    python3 make_test_frame.py 123 45        # 兼容旧用法：自定义 实际深度(cm) 实际航向(度)
"""
import struct
import sys

import link_stm32 as L
import tel_builder


def _i16(v):
    """度/秒、m/s² 等小数 → int16LE ×100（按规范）"""
    return int(round(float(v) * 100))


def _u16_cm(v):
    """cm → uint16LE ×100；负值按固件规则强制 0"""
    x = float(v)
    if x < 0:
        return 0
    return int(round(x * 100))


def build(scn):
    """按 V2 遥测 44B DATA 布局打包（字段序 = link_stm32.parse_telemetry 的字段序）"""
    data = struct.pack(
        "<12h8hHH",
        _i16(scn["tp"]), _i16(scn["ty"]), _i16(scn["tr"]),          # 目标 P/Y/R
        _i16(scn["ap"]), _i16(scn["ay"]), _i16(scn["ar"]),          # 实际 P/Y/R
        _i16(scn["gp"]), _i16(scn["gy"]), _i16(scn["gr"]),          # 角速度 P/Y/R
        _i16(scn["ax"]), _i16(scn["acy"]), _i16(scn["acz"]),        # 加速度 X/Y/Z
        *[int(m) for m in scn["motors"]],                           # 电机 0~7（原值）
        _u16_cm(scn["td_cm"]),                                      # 目标深度
        _u16_cm(scn["ad_cm"]),                                      # 实际深度
    )
    assert len(data) == 44, "DATA 必须 44 字节，实际 %d" % len(data)
    return L.build_frame(L.FUNC_TELEMETRY, data)


SCENARIOS = [
    dict(
        name="1 水面待机（上电初始 / 尚未锚定）",
        tp=0, ty=0, tr=0, ap=0, ay=0, ar=0,
        gp=0, gy=0, gr=0, ax=0, acy=0, acz=9.80,
        motors=[0] * 8, td_cm=0.0, ad_cm=0.0,
        note="全零姿态 + 深度 0 + 电机全停：验证上位机不会把'静止'当异常；也是冷启动锚定前的典型值",
    ),
    dict(
        name="2 浅水下潜中（常规跟踪）",
        tp=0, ty=30.0, tr=0, ap=1.50, ay=30.0, ar=-0.50,
        gp=0.20, gy=0.30, gr=-0.10, ax=0.05, acy=0.02, acz=9.80,
        motors=[0, 100, 200, 300, 0, 100, 200, 300], td_cm=50.0, ad_cm=55.0,
        note="目标/实际深度差 5cm，姿态带小噪声：看上位机深度与姿态曲线是否跟随",
    ),
    dict(
        name="3 定深巡航（已到位，只剩余噪声）",
        tp=0, ty=-45.0, tr=0, ap=-0.80, ay=-45.2, ar=0.30,
        gp=-0.15, gy=0.05, gr=0.12, ax=-0.03, acy=0.06, acz=9.81,
        motors=[120, 120, 120, 120, 120, 120, 120, 120], td_cm=152.0, ad_cm=152.0,
        note="实际=目标（152cm），电机对称小值：验证'到位'显示与推进器通道对应关系",
    ),
    dict(
        name="4 边界：最大深度 + 极端姿态",
        tp=0, ty=-179.99, tr=0, ap=8.90, ay=-179.99, ar=-8.90,
        gp=1.00, gy=-1.00, gr=0.50, ax=0.00, acy=0.00, acz=9.80,
        motors=[-32767, 32767, 0, 0, -1000, 1000, 0, 0], td_cm=200.0, ad_cm=200.0,
        note="深度 200.00cm = 固件钳位上限(2.00m)；Yaw 接近 ±180°；电机接近 int16 满量程",
    ),
    dict(
        name="5 异常态：负深度按 0 + 电机饱和",
        tp=0, ty=0, tr=0, ap=15.00, ay=0.00, ar=15.00,
        gp=2.50, gy=0.00, gr=-2.50, ax=0.10, acy=-0.10, acz=9.70,
        motors=[32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767],
        td_cm=0.0, ad_cm=-12.5,
        note="指定实际深度 -12.5cm：固件会把负值强制为 0，故本帧发出的是 0；电机全饱和 ±32767",
    ),
]


def show(frame, scn, idx=None, total=None):
    tag = ("帧 %s/%s  " % (idx, total) if idx else "") + scn["name"]
    hexs = " ".join("%02X" % b for b in frame)
    print("=" * 78)
    print(tag)
    print("=" * 78)
    print("整帧 %d 字节 | LEN=0x%02X(%s) | 帧头 CD %s | 帧尾 DC %s"
          % (len(frame), frame[1], "OK" if frame[1] == 0x2F else "BAD",
             "OK" if frame[0] == 0xCD else "BAD",
             "OK" if frame[-1] == 0xDC else "BAD"))
    print("HEX : %s" % hexs)
    if scn.get("note"):
        print("场景: %s" % scn["note"])

    # —— 真的用中位机的收帧状态机走一遍 ——
    frames = L.FrameParser().feed(frame)
    assert len(frames) == 1, "解析出的帧数应为 1，实际 %d" % len(frames)
    func, data = frames[0]
    assert func == L.FUNC_TELEMETRY, "FUNC 应为 0x0C，实际 0x%02X" % func
    tel = L.parse_telemetry(data)
    assert tel, "parse_telemetry 返回空（DATA 不足 44B）"

    print("解码(中位机视角):")
    print("   目标 P/Y/R = %7.2f / %7.2f / %7.2f 度" % (tel["target_pitch"], tel["target_yaw"], tel["target_roll"]))
    print("   实际 P/Y/R = %7.2f / %7.2f / %7.2f 度" % (tel["actual_pitch"], tel["actual_yaw"], tel["actual_roll"]))
    print("   角速度 P/Y/R = %7.2f / %7.2f / %7.2f 度/秒" % (tel["gyro_pitch"], tel["gyro_yaw"], tel["gyro_roll"]))
    print("   加速度 X/Y/Z = %7.2f / %7.2f / %7.2f m/s²" % (tel["acc_x"], tel["acc_y"], tel["acc_z"]))
    print("   目标深度 = %.2f cm | 实际深度 = %.2f cm" % (tel["target_depth_cm"], tel["actual_depth_cm"]))
    print("   电机 0~7 = %s" % ", ".join(str(m) for m in tel["motors"]))

    vals = tel_builder.tel_values(tel)
    print("→ 中位机将发给上位机的 $TEL（41 字段）关键索引:")
    for name, i in (("roll", 0), ("pitch", 1), ("yaw", 2),
                    ("gx", 3), ("gy", 4), ("gz", 5),
                    ("深度(m)", 6), ("t_roll", 10), ("t_pitch", 11), ("t_yaw", 12),
                    ("t_depth(m)", 16),
                    ("ax", 37), ("ay", 38), ("az", 39), ("alt", 40)):
        print("      $TEL[%2d] %-10s = %.4f" % (i, name, vals[i]))
    print("      $TEL[25..32] 推进器 0~7   = %s" % ", ".join("%.3f" % vals[25 + i] for i in range(8)))
    print("      $TEL[33..36] 推进器 8~11  = %s   <- V2 只有 8 路，恒 0"
          % ", ".join("%.2f" % vals[33 + i] for i in range(4)))
    print("      完整 $TEL: %s" % tel_builder.build_tel(tel).strip())
    print()


def main():
    argv = sys.argv[1:]

    # 兼容旧用法：两个数字参数 = 自定义 深度/航向
    if len(argv) >= 2 and argv[0].replace(".", "").replace("-", "").isdigit() \
            and argv[1].replace(".", "").replace("-", "").isdigit():
        depth, yaw = float(argv[0]), float(argv[1])
        scn = dict(SCENARIOS[1])
        scn.update(name="自定义：实际深度 %.1fcm / 实际航向 %.1f°" % (depth, yaw),
                   ad_cm=depth, ay=yaw, ty=yaw, note="")
        show(build(scn), scn)
        return 0

    if "--hex-only" in argv:
        print("# 5 条 V2 遥测回帧（无空格 HEX，可直接粘串口助手按 HEX 发送）")
        print("# 帧格式 CD 2F 0C <44B DATA> DC，整帧 48B")
        for i, scn in enumerate(SCENARIOS, 1):
            b = build(scn)
            print("# %d. %s" % (i, scn["name"]))
            print("".join("%02X" % x for x in b))
        return 0

    if "--bin" in argv:
        idx = argv.index("--bin")
        path = argv[idx + 1] if len(argv) > idx + 1 else "v2_telemetry_5frames.bin"
        with open(path, "wb") as f:
            for scn in SCENARIOS:
                f.write(build(scn))
        print("已写入 %s（5 帧 × 48B = %d 字节）" % (path, 5 * 48))
        print("喂给中位机/假下位机：按 48B 边界逐帧发送即可（CD 开头、DC 结尾）")
        return 0

    print("V2 遥测回传帧测试数据 —— 共 %d 个场景" % len(SCENARIOS))
    print("规范: 上位机对接说明_V2.md §5 | 固件: JXZK_XLB_Protocol.c:910-959")
    print()
    for i, scn in enumerate(SCENARIOS, 1):
        show(build(scn), scn, i, len(SCENARIOS))

    print("=" * 78)
    print("紧凑 HEX 汇总（可直接粘串口助手）:")
    for i, scn in enumerate(SCENARIOS, 1):
        print("%d. %s" % (i, "".join("%02X" % x for x in build(scn))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
