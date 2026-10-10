"""Small CPU joins use packed doubles; keep Python as compiler-free fallback."""
import ctypes
import hashlib
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

import numpy as np


class NativePostprocess:
    def __init__(self):
        source = Path(__file__).parent/'native'/'trim_merge.c'
        flags = ['-O3', '-std=c99', '-shared', '-fPIC', '-ffp-contract=off', '-fno-fast-math']
        key = hashlib.sha256(source.read_bytes()+repr((platform.machine(), platform.system(), flags)).encode()).hexdigest()[:20]
        cache = source.parents[2]/'runs'/'native_cache'
        library = cache/f'trim_merge_{key}.so'
        if not library.exists():
            compiler = shutil.which('cc')
            if compiler is None:
                raise OSError('No C compiler; use Python trim merge')
            cache.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=cache, prefix='trim-build-') as folder:
                output = Path(folder)/'trim_merge.so'
                subprocess.run([compiler, *flags, str(source), '-o', str(output), '-lm'],
                               check=True, capture_output=True, timeout=30)
                output.replace(library)
        self.library = ctypes.CDLL(str(library))
        self.function = self.library.quad_merge_trimmed
        matrix = np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='C_CONTIGUOUS')
        integers = np.ctypeslib.ndpointer(dtype=np.int32, ndim=1, flags='C_CONTIGUOUS')
        self.function.argtypes = [matrix, ctypes.c_int, integers]
        self.function.restype = ctypes.c_int

    def merge_trimmed(self, items):
        if not items:
            return []
        rows = np.ascontiguousarray([[*line['d'], *line['n'], line['b'], line['lo'], line['hi'],
                                     line['width'], line['support'], line['vertical'],
                                     line['strong_lo'], line['strong_hi']] for line in items], np.float64)
        indices = np.empty(len(items), np.int32)
        count = self.function(rows, len(items), indices)
        result = []
        for index in indices[:count]:
            line = items[index]
            line.update(lo=float(rows[index, 5]), hi=float(rows[index, 6]),
                        strong_lo=float(rows[index, 10]), strong_hi=float(rows[index, 11]))
            result.append(line)
        return sorted(result, key=lambda line: (line['hi']-line['lo'])*np.sqrt(line['width'])*line['support'], reverse=True)


def create_native_postprocess():
    try:
        return NativePostprocess(), dict(selected='native-float64')
    except (OSError, subprocess.SubprocessError) as exc:
        return None, dict(selected='python', reason=str(exc))
