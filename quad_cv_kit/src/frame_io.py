# -*- coding: utf-8 -*-
"""red_pole_cv 帧读取 —— 解析 `/dev/shm/momo_frame_front.bin`（MFS1）

格式（板端 `config/main_config.py` `HDR_FMT='<4sIIIQ'`，写入端 `src/shm_writer.py`）：

    offset 0   4B  magic   b'MFS1'
    offset 4   4B  seq     uint32   ← ★ 靠它与 momo_det_front.json 的 frame 对齐
    offset 8   4B  jpeg_len uint32
    offset 12  4B  wh      uint32   （= w*h，诊断用）
    offset 16  8B  ts_us   uint64
    offset 24  N   jpeg    N = jpeg_len

整块容量 = HDR_SIZE(24) + MAX_JPEG_BYTES(2MB)。

★ 纯标准库解析（`struct`），**不 import cv2** ⇒ 本文件可以离线用合成 bin 单测；
  解码是可选的（`decode=True` 时才需要 cv2）。
★ 读到的是**"最后一次写完"的整块**（写端先写头再写体），本函数只做长度/魔数校验，
  **不做加锁**（与既有 `src/shm_reader.py` 同口径）。
"""
from __future__ import annotations

import os
import struct

HDR_SIZE = 24
HDR_FMT = '<4sIIIQ'
MAGIC = b'MFS1'
MAX_JPEG_BYTES = 2 * 1024 * 1024


def parse(buf):
    """bytes → dict(seq, jpeg_len, wh, ts_us, jpeg) | None"""
    if not buf or len(buf) < HDR_SIZE:
        return None
    magic, seq, jlen, wh, ts_us = struct.unpack(HDR_FMT, buf[:HDR_SIZE])
    if magic != MAGIC:
        return None
    if jlen <= 0 or jlen > MAX_JPEG_BYTES:
        return None
    if len(buf) < HDR_SIZE + jlen:
        return None                                     # 写端刚写一半
    return {'seq': int(seq), 'jpeg_len': int(jlen), 'wh': int(wh),
            'ts_us': int(ts_us), 'jpeg': buf[HDR_SIZE:HDR_SIZE + jlen]}


def read_latest(path, max_bytes=None):
    """读共享内存帧 → parse 结果 | None（文件不存在/空/坏都返回 None，不抛）"""
    try:
        lim = int(max_bytes or (HDR_SIZE + MAX_JPEG_BYTES))
        with open(path, 'rb') as f:
            buf = f.read(lim)
    except OSError:
        return None
    return parse(buf)


def decode(jpeg_bytes, scale=1.0):
    """JPEG → BGR ndarray（需要 cv2）；scale=0.5 走半分辨率解码（★ ψ 尺度无关，几乎无损）"""
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    if not jpeg_bytes:
        return None
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    flag = cv2.IMREAD_COLOR
    s = float(scale)
    # ★ 档位映射：IMREAD_REDUCED_COLOR_2 = 1/2、_4 = 1/4、_8 = 1/8
    if s < 0.19:
        flag = cv2.IMREAD_REDUCED_COLOR_8
    elif s < 0.38:
        flag = cv2.IMREAD_REDUCED_COLOR_4
    elif s < 0.76:
        flag = cv2.IMREAD_REDUCED_COLOR_2
    return cv2.imdecode(arr, flag)


def write_frame(path, seq, jpeg_bytes, wh=0, ts_us=0):
    """★ 仅测试用：按同一格式造一个合成 bin（板端不会调用它）"""
    hdr = struct.pack(HDR_FMT, MAGIC, int(seq), len(jpeg_bytes), int(wh), int(ts_us))
    with open(path, 'wb') as f:
        f.write(hdr)
        f.write(jpeg_bytes)
    return os.path.getsize(path)


if __name__ == '__main__':                           # pragma: no cover
    import sys
    print('frame_io: HDR_SIZE=%d FMT=%s' % (HDR_SIZE, HDR_FMT))
    sys.exit(0)
