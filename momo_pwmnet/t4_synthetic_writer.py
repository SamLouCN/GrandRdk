#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
import sys, time
import numpy as np
import cv2
sys.path.insert(0, "/userdata/vp/vp5.1/utils")
from flow_share import FlowShareWriter

W, H = 640, 480
SCENE_W = 1920            # 大场景宽度: 裁窗模拟相机平移, 纹理永不枯竭

# 固定"场景"纹理 (非周期! 模糊随机场 + 随机方块, 避免 LK 周期混淆)
def _make_scene():
    rng = np.random.default_rng(7)
    field = rng.normal(128, 40, (H, SCENE_W)).astype(np.float32)
    field = cv2.GaussianBlur(field, (0, 0), 12)
    img = np.clip(field, 0, 255).astype(np.uint8)
    # 随机实心方块 (强角点, 增强 GFtT 稳定性)
    for _ in range(24):
        x = int(rng.integers(40, SCENE_W - 60)); y = int(rng.integers(40, H - 60))
        s = int(rng.integers(14, 34))
        v = int(rng.integers(0, 256))
        cv2.rectangle(img, (x, y), (x + s, y + s), v, -1)
    return img

_SCENE = _make_scene()

def make_nv12(move_px, pos):
    """从大场景上按累计位移 pos 裁 640px 窗口(模拟相机平移)。
    move_px=0 时位置不变(静止); move_px>0 时 pos 每次 += move_px(真实运动)。
    """
    pos = min(pos, SCENE_W - W)          # 裁窗上限保护
    base = _SCENE[:, pos:pos + W]
    nv12 = np.zeros((H * 3 // 2, W), np.uint8)
    nv12[:H, :] = base                          # Y 平面
    # UV 平面给中性色即可(光流只取 Y)
    nv12[H:, :] = 128
    return nv12

def main():
    fps = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    dur = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    sh = FlowShareWriter("front", W, H, fmt=0)   # NV12
    print(f"[writer] /dev/shm/momo_flow_front.bin 写入中: {W}x{H} NV12 @ {fps}fps x {dur}s", flush=True)
    n = 0
    pos = 0
    t0 = time.time()
    while time.time() - t0 < dur:
        t = time.time() - t0
        # 场景段(注意: flow_speed 采样有追赶延迟, 静止段放最前段让采样必然命中运动段)
        if t < 5:
            move = 0                              # 静止
        elif t < 15:
            move = 4                              # 匀速右移 4px/帧(模拟相机运动)
        else:
            move = 4 if int(t) % 4 < 2 else 0     # 2s 动 2s 静
        if move:
            pos += move
        frame = make_nv12(move, pos)
        sh.write(frame)
        n += 1
        time.sleep(1.0 / fps)
        if n % 300 == 0:
            print(f"[writer] {n} frames ({n/(time.time()-t0):.0f} fps)", flush=True)
    sh.close()
    print(f"[writer] done: {n} frames in {time.time()-t0:.1f}s", flush=True)

if __name__ == "__main__":
    main()