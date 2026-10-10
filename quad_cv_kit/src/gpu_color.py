"""Exact LAB a lookup, generated once from the local OpenCV build.

The detector only consumes a: a 16 MiB byte table covers every BGR value.
Steady-frame conversion runs on the GPU with unchanged 8-bit thresholds.
"""
from functools import lru_cache
import hashlib
from pathlib import Path
import tempfile

import cv2
import numpy as np


@lru_cache(maxsize=1)
def lab_a_table():
    signature = (cv2.__version__, cv2.getBuildInformation(), cv2.useOptimized(), 'bgr-a-v1')
    key = hashlib.sha256(repr(signature).encode()).hexdigest()[:20]
    cache = Path(__file__).resolve().parents[1]/'runs'/'color_cache'
    path = cache/f'lab_a_{key}.npy'
    try:
        table = np.load(path, allow_pickle=False)
        if table.shape == (256, 256, 256) and table.dtype == np.uint8:
            table.setflags(write=False)
            return table
    except (OSError, ValueError):
        pass
    plane = np.empty((256, 256, 3), np.uint8)
    plane[:, :, 1] = np.arange(256, dtype=np.uint8)[:, None]
    plane[:, :, 2] = np.arange(256, dtype=np.uint8)[None, :]
    table = np.empty((256, 256, 256), np.uint8)
    for blue in range(256):
        plane[:, :, 0] = blue
        table[blue] = cv2.cvtColor(plane, cv2.COLOR_BGR2LAB)[:, :, 1]
    try:
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache, prefix='lab-build-') as folder:
            temporary = Path(folder)/'table.npy'
            np.save(temporary, table, allow_pickle=False)
            temporary.replace(path)
    except OSError:
        pass  # Read-only deployments generate the table in memory at startup.
    table.setflags(write=False)
    return table
