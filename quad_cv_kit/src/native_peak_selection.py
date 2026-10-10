"""Optional exact CPU suppression; all compiler artifacts stay in quad_cv_kit."""
import ctypes
import hashlib
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

import numpy as np


class NativePeakSelector:
    def __init__(self):
        source = Path(__file__).parent/'native'/'hough_peaks.c'
        flags = ['-O3', '-std=c99', '-shared', '-fPIC', '-ffp-contract=off', '-fno-fast-math']
        key = hashlib.sha256(source.read_bytes()+repr((platform.machine(), platform.system(), flags)).encode()).hexdigest()[:20]
        cache = source.parents[2]/'runs'/'native_cache'
        library = cache/f'peak_select_{key}.so'
        if not library.exists():
            compiler = shutil.which('cc')
            if compiler is None:
                raise OSError('No C compiler; use Python peak suppression')
            cache.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=cache, prefix='peak-build-') as folder:
                output = Path(folder)/'peak_select.so'
                subprocess.run([compiler, *flags, str(source), '-o', str(output)],
                               check=True, capture_output=True, timeout=30)
                output.replace(library)
        self.library = ctypes.CDLL(str(library))
        self.function = self.library.quad_select_peaks
        vector = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C_CONTIGUOUS')
        matrix = np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='C_CONTIGUOUS')
        integers = np.ctypeslib.ndpointer(dtype=np.int32, ndim=1, flags='C_CONTIGUOUS')
        self.function.argtypes = [matrix, vector, integers, ctypes.c_int, ctypes.c_int, ctypes.c_double, integers]
        self.function.restype = ctypes.c_int

    def select(self, normals, offsets, horizontal, limit, cosine_limit):
        normals = np.ascontiguousarray(normals, np.float64)
        offsets = np.ascontiguousarray(offsets, np.float64)
        horizontal = np.ascontiguousarray(horizontal, np.int32)
        result = np.empty(min(len(offsets), limit), np.int32)
        count = self.function(normals, offsets, horizontal, len(offsets), limit, cosine_limit, result)
        if count < 0:
            raise MemoryError('Native peak suppression allocation failed')
        return result[:count]


def create_native_selector():
    try:
        return NativePeakSelector(), {'selected': 'native-float64'}
    except (OSError, subprocess.SubprocessError) as exc:
        return None, {'selected': 'python-angle-indexed', 'reason': str(exc)}
