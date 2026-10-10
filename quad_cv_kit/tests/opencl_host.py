"""Run compiled OpenCL kernels on host threads for arithmetic regression tests."""
import ctypes as C
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

import numpy as np

from src.opencl_runtime import Buffer, LocalMemory


class Arg(C.Structure):
    _fields_ = [('pointer', C.c_void_p), ('integer', C.c_int), ('real', C.c_float)]


class HostRuntime:
    folder = lib = None

    @classmethod
    def compile(cls):
        if cls.lib is not None:
            return
        if not shutil.which('clang') or not shutil.which('clang++'):
            raise RuntimeError('Host kernel tests require clang and clang++')
        cls.folder = tempfile.TemporaryDirectory(prefix='quad-opencl-host-')
        output = Path(cls.folder.name)
        kernel_dir = Path(__file__).resolve().parents[1]/'src'/'kernels'
        kernels = output/'kernels.cl'
        kernels.write_text('\n'.join((kernel_dir/name).read_text()
                                     for name in ('red_gate.cl', 'search.cl', 'residency.cl', 'color.cl', 'sharpen.cl')))
        subprocess.run(['clang', '-x', 'cl', '-D__kernel=', '-cl-std=CL1.2', '-O2',
                        '-c', str(kernels), '-o', str(output/'kernels.o')], check=True, capture_output=True)
        subprocess.run(['clang++', '-std=c++11', '-O2', '-shared', '-fPIC', '-pthread',
                        str(Path(__file__).with_suffix('.cpp')), str(output/'kernels.o'),
                        '-o', str(output/'host.so')], check=True, capture_output=True)
        cls.lib = C.CDLL(str(output/'host.so'))
        cls.lib.launch.argtypes = [C.c_char_p, C.c_size_t, C.c_size_t, C.POINTER(Arg)]
        cls.lib.launch.restype = C.c_int

    def __init__(self, *args, **kwargs):
        self.compile()
        self.arrays = {}
        self.info = dict(selected='host-kernel-test', device='CPU test harness', vendor='test',
                         driver='test', version='OpenCL C 1.2 source', device_type='host')
        self.reset_stats()

    def reset_stats(self):
        self.kernel_ms = {}
        self.upload_bytes = self.download_bytes = 0
        self.transfer_bytes, self.transfer_calls = {}, {}
        self._buffer_names = getattr(self, '_buffer_names', {})

    def record_transfer(self, direction, name, nbytes):
        key = direction+':'+name
        self.transfer_bytes[key] = self.transfer_bytes.get(key, 0)+int(nbytes)
        self.transfer_calls[key] = self.transfer_calls.get(key, 0)+1

    def buffer(self, name, nbytes):
        nbytes = max(4, int(nbytes))
        if name not in self.arrays or self.arrays[name].nbytes < nbytes:
            self.arrays[name] = np.zeros(nbytes, np.uint8)
        array = self.arrays[name]
        self._buffer_names[array.ctypes.data] = name
        return Buffer(array.ctypes.data, array.nbytes)

    def upload(self, name, array):
        array = np.ascontiguousarray(array)
        buf = self.buffer(name, array.nbytes)
        C.memmove(buf.handle, array.ctypes.data, array.nbytes)
        self.upload_bytes += array.nbytes
        self.record_transfer('upload', name, array.nbytes)
        return buf

    def read(self, buf, shape, dtype):
        array = np.empty(shape, dtype)
        if array.nbytes > buf.capacity:
            raise RuntimeError('Host read exceeds buffer')
        C.memmove(array.ctypes.data, buf.handle, array.nbytes)
        self.download_bytes += array.nbytes
        self.record_transfer('download', self._buffer_names[buf.handle], array.nbytes)
        return array

    def run(self, name, size, args, local=None):
        values, scratch = (Arg*len(args))(), []
        for i, value in enumerate(args):
            if isinstance(value, Buffer):
                values[i].pointer = value.handle
            elif isinstance(value, LocalMemory):
                array = np.zeros(value.size, np.uint8)
                scratch.append(array)
                values[i].pointer = array.ctypes.data
            elif isinstance(value, (float, np.floating)):
                values[i].real = value
            else:
                values[i].integer = value
        started = time.perf_counter()
        code = self.lib.launch(name.encode(), size, local or 0, values)
        if code:
            raise RuntimeError(f'Unknown host kernel: {name}')
        self.kernel_ms[name] = self.kernel_ms.get(name, 0.)+(time.perf_counter()-started)*1000

    def read_many(self, requests):
        return [self.read(buf, shape, dtype) for buf, shape, dtype in requests]

    def finish(self):
        pass

    def close(self):
        self.arrays.clear()
