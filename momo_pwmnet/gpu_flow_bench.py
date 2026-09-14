#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T2: GPU(OpenCL/UMat) 光流单帧基准 —— 验证稀疏 PyrLK 是否真走 Mali-G78AE,
以及 320x240 / 640x480 下的耗时, 与 CPU 版对比。
不依赖 shm, 用合成/静态帧; 不超过 run.sh 生命周期。
用法: python3 gpu_flow_bench.py
"""
import time
import numpy as np
import cv2

# ---------- 合成帧(带纹理, 第二帧整体平移 3px 模拟运动) ----------
def make_pair(w, h, shift=3):
    rng = np.random.default_rng(42)
    base = (rng.integers(0, 255, (h, w, 3), dtype=np.uint8))
    # 加棋盘网格, 保证角点充足
    g = np.uint8(np.indices((h, w)).sum(axis=0) * 255 // (w + h))
    base = cv2.addWeighted(base, 0.6, cv2.merge([g, g, g]), 0.4, 0)
    M = np.float32([[1, 0, shift], [0, 1, int(shift * 0.6)]])
    f2 = cv2.warpAffine(base, M, (w, h))
    return cv2.cvtColor(base, cv2.COLOR_BGR2GRAY), cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY)

def bench(name, prev, cur, pts, win=21, max_level=2, iterations=5, warmup=3):
    prev_um, cur_um = cv2.UMat(prev), cv2.UMat(cur)

    # ---- GPU (UMat + PyrLK OCL) ----
    try:
        nxt_um = cv2.UMat(np.zeros((len(pts), 2), dtype=np.float32))
        st_um = None
        for _ in range(warmup):
            nxt_um, st_um, err_um = cv2.calcOpticalFlowPyrLK(
                prev_um, cur_um, pts, None, winSize=(win, win), maxLevel=max_level)
        t0 = time.perf_counter()
        for _ in range(iterations):
            nxt_um, st_um, err_um = cv2.calcOpticalFlowPyrLK(
                prev_um, cur_um, pts, None, winSize=(win, win), maxLevel=max_level)
        t1 = time.perf_counter()
        gpu_ms = (t1 - t0) / iterations * 1000
        nxt = cv2.UMat.get(nxt_um) if isinstance(nxt_um, cv2.UMat) else nxt_um
        ok = int(np.sum(1 if (st_um is None or cv2.UMat.get(st_um) is None) else 0))
        tracked = None
        try:
            st_mat = cv2.UMat.get(st_um) if isinstance(st_um, cv2.UMat) else st_um
            tracked = int(np.sum(st_mat)) if st_mat is not None else -1
        except Exception:
            tracked = -1
        gpu_note = f"tracked={tracked}"
    except Exception as exc:
        gpu_ms, gpu_note = float('nan'), f'ERR {exc!r}'

    # ---- CPU 对比 ----
    nxt_c = None
    t0 = time.perf_counter()
    for _ in range(iterations):
        nxt_c, st_c, err_c = cv2.calcOpticalFlowPyrLK(
            prev, cur, pts, None, winSize=(win, win), maxLevel=max_level)
    t1 = time.perf_counter()
    cpu_ms = (t1 - t0) / iterations * 1000

    print(f'[{name}] w={cur.shape[1]} h={cur.shape[0]} pts={len(pts)} | '
          f'GPU(UMat/OCL)={gpu_ms:6.2f}ms {gpu_note} | CPU={cpu_ms:6.2f}ms | '
          f'speedup={cpu_ms/gpu_ms if gpu_ms==gpu_ms and gpu_ms>0 else float("nan"):5.2f}x')

def main():
    print('cv2:', cv2.__version__, '| haveOpenCL:', cv2.ocl.haveOpenCL(),
          '| useOpenCL:', cv2.ocl.useOpenCL())
    # 尝试取默认 OCL 设备名
    try:
        dev = cv2.ocl.Device.getDefault()
        print('OCL default device:', dev.name(), '| CU:', dev.maxComputeUnits(),
              '| clock:', dev.clockFrequency(), 'MHz')
    except Exception as exc:
        print('OCL device query 失败:', exc)

    for w, h, shift in ((640, 480, 3), (320, 240, 3)):
        prev, cur = make_pair(w, h, shift)
        t0 = time.perf_counter()
        pts = cv2.goodFeaturesToTrack(prev, maxCorners=200, qualityLevel=0.01,
                                      minDistance=10, blockSize=7)
        t1 = time.perf_counter()
        pts = pts.reshape(-1, 1, 2).astype(np.float32)
        print(f'--- {w}x{h} 特征检测(CPU)={ (t1-t0)*1000:.2f}ms 点数={len(pts)} ---')
        bench(f'pyrlk-L{2}', prev, cur, pts, max_level=2, iterations=8)
        bench(f'pyrlk-L{1}', prev, cur, pts, max_level=1, iterations=8)

if __name__ == '__main__':
    main()