#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# calibrate_depth.py - 单目深度 DAV2 仿射标定工具
# ============================================================
# 用法:
#   1) 摆两个已知真实距离的参照物 A、B (卷尺量到摄像头镜头的距离 Z1, Z2)
#   2) 在画面中看清 A、B 的中心像素坐标 (x1, y1), (x2, y2)
#   3) 跑一次无参命令, 抓当前帧生成色彩化深度图 /tmp/rel_view.png
#      与左侧原图拼接, 保存到 /userdata/momo_pwmnet/depth/rel_view.png
#      (RDK Studio 文件面板可直接看)
#   4) 用图像软件(画图 / 看图)对照左原图找出两参照物中心像素坐标
#   5) 正式标定:
#         python3 calibrate_depth.py sample x1 y1 Z1
#         python3 calibrate_depth.py sample x2 y2 Z2
#         python3 calibrate_depth.py solve           # 打印 a, b 并预览
#         python3 calibrate_depth.py apply           # 写入 config.ini [depth]
# 现场两步走(更快):
#         python3 calibrate_depth.py once x1 y1 Z1 x2 y2 Z2 [--apply]
# ============================================================
import json
import os
import sys

import cv2
import numpy as np

_PROJ = "/userdata/momo_pwmnet"
_SAMPLE_CACHE = "/tmp/calib_samples.json"
_MODEL_PATH = f"{_PROJ}/depth/model/depth_any.hbm"


def _load_depth_model():
    sys.path.insert(0, f"{_PROJ}/depth")
    from depth_anything import DepthAnythingV2, DepthAnythingV2Config
    return DepthAnythingV2(DepthAnythingV2Config(_MODEL_PATH))


def _sample_rel(model, frame, x, y, ksize=5):
    """以 (x,y) 为中心取 5x5 均值 rel, 容错越界。"""
    h, w = frame.shape[:2]
    half = ksize // 2
    x0, y0 = max(0, x - half), max(0, y - half)
    x1, y1 = min(w, x + half + 1), min(h, y + half + 1)
    rel = model.predict(frame)
    return float(rel[y0:y1, x0:x1].mean())


def _grab_frame():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(10):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        sys.exit("read frame FAIL")
    return frame


def _save_colorized(model, frame, path):
    rel = model.predict(frame)
    sys.path.insert(0, f"{_PROJ}/depth")
    from depth_anything import DepthAnythingV2
    color = DepthAnythingV2.colorize(rel)
    cv2.imwrite(path, np.hstack([frame, color]))


def cmd_capture():
    """无参: 抓帧 + 深度热图保存."""
    model = _load_depth_model()
    frame = _grab_frame()
    out = f"{_PROJ}/depth/rel_view.png"
    _save_colorized(model, frame, out)
    print(f"saved {out} (左:原图 右:深度热图, 亮=近 暗=远)")


def _load_samples():
    if not os.path.exists(_SAMPLE_CACHE):
        return []
    with open(_SAMPLE_CACHE) as f:
        return json.load(f)


def _save_samples(samples):
    with open(_SAMPLE_CACHE, "w") as f:
        json.dump(samples, f, indent=2)


def cmd_sample(x, y, z_m):
    """采样一个参照点: 像素 (x,y), 真实距离 Z_m (米)."""
    samples = _load_samples()
    model = _load_depth_model()
    frame = _grab_frame()
    rel = _sample_rel(model, frame, int(x), int(y))
    samples.append({"x": int(x), "y": int(y), "z_m": float(z_m), "rel": rel})
    _save_samples(samples)
    print(f"sample #{len(samples)}: x={x} y={y} Z={z_m}m -> rel={rel:.4f}")
    for i, s in enumerate(samples, 1):
        print(f"  [{i}] x={s['x']} y={s['y']} Z={s['z_m']}m rel={s['rel']:.4f}")


def cmd_solve():
    """根据已采样点解仿射 a, b (Z_m = a*rel + b)."""
    samples = _load_samples()
    if len(samples) < 2:
        sys.exit("need >=2 samples; use 'sample x y Z_m' first")
    # 最小二乘 / 直接解(两点)
    if len(samples) == 2:
        s1, s2 = samples[0], samples[1]
        if abs(s1["rel"] - s2["rel"]) < 1e-6:
            sys.exit("two samples have same rel, 不能定标")
        a = (s1["z_m"] - s2["z_m"]) / (s1["rel"] - s2["rel"])
        b = s1["z_m"] - a * s1["rel"]
        print(f"linear solve: Z_m = {a:.6f} * rel + {b:.6f}")
        # 残差
        for s in samples:
            z_pred = a * s["rel"] + b
            print(f"  ({s['x']},{s['y']}) Z_real={s['z_m']:.3f} Z_pred={z_pred:.3f} "
                  f"err={z_pred-s['z_m']:+.3f} m")
        print(f"\napply with: calibrate_depth.py apply")
        print(f"or  python3 calibrate_depth.py apply  -> 写入 config.ini [depth]")
        return a, b
    # 多点最小二乘 (z = a*rel + b)
    A = np.array([[s["rel"], 1.0] for s in samples], dtype=np.float64)
    z = np.array([s["z_m"] for s in samples], dtype=np.float64)
    a, b = np.linalg.lstsq(A, z, rcond=None)[0]
    print(f"lstsq solve: Z_m = {a:.6f} * rel + {b:.6f}")
    for s in samples:
        z_pred = a * s["rel"] + b
        print(f"  ({s['x']},{s['y']}) Z_real={s['z_m']:.3f} Z_pred={z_pred:.3f} "
              f"err={z_pred-s['z_m']:+.3f} m")
    return float(a), float(b)


def cmd_apply():
    """根据已采样点解 a, b 并直接写回 config.ini [depth]."""
    samples = _load_samples()
    if len(samples) < 2:
        sys.exit("need >=2 samples")
    if len(samples) == 2:
        s1, s2 = samples[0], samples[1]
        a = (s1["z_m"] - s2["z_m"]) / (s1["rel"] - s2["rel"])
        b = s1["z_m"] - a * s1["rel"]
    else:
        A = np.array([[s["rel"], 1.0] for s in samples], dtype=np.float64)
        z = np.array([s["z_m"] for s in samples], dtype=np.float64)
        a, b = np.linalg.lstsq(A, z, rcond=None)[0]
    cfg_path = f"{_PROJ}/config.ini"
    with open(cfg_path) as f:
        lines = f.readlines()
    out = []
    section = None
    for line in lines:
        s = line.strip()
        if s.startswith("[") and s.endswith("]"):
            section = s[1:-1]
            out.append(line)
        elif section == "depth" and s.startswith("depth_a"):
            out.append(f"depth_a = {a:.6f}\n")
        elif section == "depth" and s.startswith("depth_b"):
            out.append(f"depth_b = {b:.6f}\n")
        else:
            out.append(line)
    with open(cfg_path, "w") as f:
        f.writelines(out)
    print(f"wrote config.ini [depth]: depth_a={a:.6f} depth_b={b:.6f}")


def cmd_once(x1, y1, z1, x2, y2, z2, apply=False):
    """一步: 抓一帧 + 两点采样 + 解 a/b (+ 可选写 config)."""
    model = _load_depth_model()
    frame = _grab_frame()
    r1 = _sample_rel(model, frame, int(x1), int(y1))
    r2 = _sample_rel(model, frame, int(x2), int(y2))
    a = (float(z1) - float(z2)) / (r1 - r2)
    b = float(z1) - a * r1
    print(f"rel@({x1},{y1}) = {r1:.4f}  Z1={z1}m")
    print(f"rel@({x2},{y2}) = {r2:.4f}  Z2={z2}m")
    print(f"-> Z_m = {a:.6f} * rel + {b:.6f}")
    print(f"   verify: rel={r1:.4f} -> {a*r1+b:.3f}m (expect {z1}m)")
    print(f"           rel={r2:.4f} -> {a*r2+b:.3f}m (expect {z2}m)")
    if apply:
        cfg_path = f"{_PROJ}/config.ini"
        with open(cfg_path) as f:
            lines = f.readlines()
        out, section = [], None
        for line in lines:
            s = line.strip()
            if s.startswith("[") and s.endswith("]"):
                section = s[1:-1]
                out.append(line)
            elif section == "depth" and s.startswith("depth_a"):
                out.append(f"depth_a = {a:.6f}\n")
            elif section == "depth" and s.startswith("depth_b"):
                out.append(f"depth_b = {b:.6f}\n")
            else:
                out.append(line)
        with open(cfg_path, "w") as f:
            f.writelines(out)
        print(f"wrote config.ini [depth]: depth_a={a:.6f} depth_b={b:.6f}")
    return a, b


def usage():
    print(__doc__)
    sys.exit(1)


def main(argv):
    if len(argv) < 2:
        usage()
    cmd = argv[1]
    if cmd == "capture":
        cmd_capture()
    elif cmd == "sample":
        cmd_sample(argv[2], argv[3], argv[4])
    elif cmd == "solve":
        cmd_solve()
    elif cmd == "apply":
        cmd_apply()
    elif cmd == "once":
        if len(argv) < 8:
            usage()
        apply = "--apply" in argv[8:]
        cmd_once(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7], apply)
    else:
        usage()


if __name__ == "__main__":
    main(sys.argv)