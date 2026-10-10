"""Small OpenCL 1.2 host API. Uses the vendor ICD, no PyOpenCL dependency."""
import ctypes as C
import ctypes.util
from dataclasses import dataclass
import os
import struct
import time

import numpy as np


class OpenCLError(RuntimeError):
    pass


@dataclass
class Buffer:
    handle: int
    capacity: int


@dataclass
class LocalMemory:
    size: int


class OpenCLRuntime:
    def __init__(self, source, device_name=None, device_type='gpu'):
        self.context = self.queue = self.program = None
        self.buffers, self.kernels, self.events = {}, {}, []
        self.kernel_arguments = {}
        self.pending_uploads = []
        self._pending_commands = False
        self.reset_stats()
        path = os.environ.get('GRDK_OPENCL_LIBRARY') or ctypes.util.find_library('OpenCL') or 'libOpenCL.so.1'
        try:
            self.lib = C.CDLL(path)
        except OSError as exc:
            raise OpenCLError(f'Cannot load the OpenCL ICD ({path}): {exc}') from exc
        try:
            self._bind()
        except AttributeError as exc:
            raise OpenCLError(f'The OpenCL ICD lacks a required 1.2 host API: {exc}') from exc
        try:
            self.device, self.info = self._select_device(device_name, device_type)
            error = C.c_int()
            devices = (C.c_void_p*1)(self.device)
            self.context = self.lib.clCreateContext(None, 1, devices, None, None, C.byref(error))
            self.check(error.value, 'create context')
            self.queue = self.lib.clCreateCommandQueue(self.context, self.device, 2, C.byref(error))
            self.check(error.value, 'create profiling queue')
            encoded = source.encode()
            sources, sizes = (C.c_char_p*1)(encoded), (C.c_size_t*1)(len(encoded))
            self.program = self.lib.clCreateProgramWithSource(self.context, 1, sources, sizes, C.byref(error))
            self.check(error.value, 'create program')
            code = self.lib.clBuildProgram(self.program, 1, devices, b'-cl-std=CL1.2', None, None)
            if code:
                size = C.c_size_t()
                self.lib.clGetProgramBuildInfo(self.program, self.device, 0x1183, 0, None, C.byref(size))
                log = C.create_string_buffer(size.value)
                self.lib.clGetProgramBuildInfo(self.program, self.device, 0x1183, size, log, None)
                raise OpenCLError(f'OpenCL build failed ({code}): {log.value.decode(errors="replace")}')
        except BaseException:
            self.close()
            raise

    def _bind(self):
        p, i, u, z, flags = C.c_void_p, C.c_int, C.c_uint, C.c_size_t, C.c_uint64
        pp, pi, pu, pz = C.POINTER(p), C.POINTER(i), C.POINTER(u), C.POINTER(z)
        signatures = {
            'clGetPlatformIDs': (i, [u, pp, pu]),
            'clGetDeviceIDs': (i, [p, flags, u, pp, pu]),
            'clGetDeviceInfo': (i, [p, u, z, p, pz]),
            'clCreateContext': (p, [p, u, pp, p, p, pi]),
            'clCreateCommandQueue': (p, [p, p, flags, pi]),
            'clCreateProgramWithSource': (p, [p, u, C.POINTER(C.c_char_p), pz, pi]),
            'clBuildProgram': (i, [p, u, pp, C.c_char_p, p, p]),
            'clGetProgramBuildInfo': (i, [p, p, u, z, p, pz]),
            'clCreateKernel': (p, [p, C.c_char_p, pi]),
            'clCreateBuffer': (p, [p, flags, z, p, pi]),
            'clSetKernelArg': (i, [p, u, z, p]),
            'clEnqueueWriteBuffer': (i, [p, p, u, z, z, p, u, pp, pp]),
            'clEnqueueReadBuffer': (i, [p, p, u, z, z, p, u, pp, pp]),
            'clEnqueueNDRangeKernel': (i, [p, p, u, pz, pz, pz, u, pp, pp]),
            'clGetEventProfilingInfo': (i, [p, u, z, p, pz]),
            'clFinish': (i, [p]),
        }
        for name in ('Context', 'CommandQueue', 'Program', 'Kernel', 'MemObject', 'Event'):
            signatures['clRelease'+name] = (i, [p])
        for name, (restype, argtypes) in signatures.items():
            fn = getattr(self.lib, name)
            fn.restype, fn.argtypes = restype, argtypes

    @staticmethod
    def check(code, operation):
        if code:
            raise OpenCLError(f'OpenCL {operation} failed (error {code})')

    def _device_string(self, device, key):
        size = C.c_size_t()
        self.check(self.lib.clGetDeviceInfo(device, key, 0, None, C.byref(size)), 'device info')
        value = C.create_string_buffer(size.value)
        self.check(self.lib.clGetDeviceInfo(device, key, size, value, None), 'device info')
        return value.value.decode(errors='replace')

    def _device_value(self, device, key, dtype):
        value = dtype()
        self.check(self.lib.clGetDeviceInfo(device, key, C.sizeof(value), C.byref(value), None), 'device limits')
        return value.value

    def _select_device(self, name, kind):
        count = C.c_uint()
        self.check(self.lib.clGetPlatformIDs(0, None, C.byref(count)), 'enumerate platforms')
        platforms = (C.c_void_p*count.value)()
        self.check(self.lib.clGetPlatformIDs(count, platforms, None), 'enumerate platforms')
        found = []
        for platform in platforms:
            count = C.c_uint()
            code = self.lib.clGetDeviceIDs(platform, 4 if kind == 'gpu' else 0xFFFFFFFF, 0, None, C.byref(count))
            if code == -1:
                continue
            self.check(code, 'enumerate devices')
            devices = (C.c_void_p*count.value)()
            self.check(self.lib.clGetDeviceIDs(platform, 4 if kind == 'gpu' else 0xFFFFFFFF, count, devices, None), 'enumerate devices')
            for device in devices:
                label = self._device_string(device, 0x102B)
                vendor = self._device_string(device, 0x102C)
                if name and name.lower() not in (label+' '+vendor).lower():
                    continue
                found.append((device, label, vendor))
        if not found:
            raise OpenCLError(f'No OpenCL {kind} device matching {name or "any GPU"}')
        found.sort(key=lambda row: 'mali' not in row[1].lower())
        device, label, vendor = found[0]
        return device, dict(selected='opencl', device=label, vendor=vendor,
                            driver=self._device_string(device, 0x102D),
                            version=self._device_string(device, 0x102F), device_type=kind,
                            max_work_group_size=self._device_value(device, 0x1004, C.c_size_t),
                            local_memory_bytes=self._device_value(device, 0x1023, C.c_uint64),
                            max_allocation_bytes=self._device_value(device, 0x1010, C.c_uint64))

    def buffer(self, name, nbytes):
        nbytes = max(4, int(nbytes))
        if nbytes > self.info['max_allocation_bytes']:
            raise OpenCLError(f'{name} requires {nbytes} bytes, exceeding the GPU allocation limit')
        old = self.buffers.get(name)
        if old is not None and old.capacity >= nbytes:
            return old
        if old is not None:
            self.finish()
            self.buffers.pop(name)
            self.lib.clReleaseMemObject(old.handle)
            getattr(self, '_buffer_names', {}).pop(old.handle, None)
        error = C.c_int()
        handle = self.lib.clCreateBuffer(self.context, 1, nbytes, None, C.byref(error))
        self.check(error.value, f'allocate {name} ({nbytes} bytes)')
        result = self.buffers[name] = Buffer(handle, nbytes)
        if not hasattr(self, '_buffer_names'):
            self._buffer_names = {}
        self._buffer_names[handle] = name
        return result

    def upload(self, name, array):
        array = np.ascontiguousarray(array)
        buf = self.buffer(name, array.nbytes)
        if array.nbytes:
            self.check(self.lib.clEnqueueWriteBuffer(self.queue, buf.handle, 0, 0, array.nbytes,
                        C.c_void_p(array.ctypes.data), 0, None, None), f'upload {name}')
            # Nonblocking writes must own their host memory until completion.
            self.pending_uploads.append(array)
            self._pending_commands = True
            self.upload_bytes += array.nbytes
            self.record_transfer('upload', name, array.nbytes)
        return buf

    def read(self, buf, shape, dtype):
        result = np.empty(shape, dtype)
        if result.nbytes > buf.capacity:
            raise OpenCLError('Read exceeds device buffer capacity')
        if result.nbytes:
            self.check(self.lib.clEnqueueReadBuffer(self.queue, buf.handle, 1, 0, result.nbytes,
                        C.c_void_p(result.ctypes.data), 0, None, None), 'download')
            self.download_bytes += result.nbytes
            self.record_transfer('download', getattr(self, '_buffer_names', {}).get(buf.handle, 'external'), result.nbytes)
            self.read_wait_calls += 1
            self.pending_uploads.clear()
            self._pending_commands = False
        return result

    def run(self, name, size, args, local=None):
        if size < 1 or (local is not None and (local < 1 or size % local)):
            raise ValueError('OpenCL launch requires positive sizes and complete work groups')
        if local is not None and local > self.info['max_work_group_size']:
            raise OpenCLError(f'{name}: work group {local} exceeds the device limit')
        local_bytes = sum(arg.size for arg in args if isinstance(arg, LocalMemory))
        if local_bytes > self.info['local_memory_bytes']:
            raise OpenCLError(f'{name}: scratch requires {local_bytes} local bytes, exceeding the device limit')
        kernel = self.kernels.get(name)
        if kernel is None:
            error = C.c_int()
            kernel = self.lib.clCreateKernel(self.program, name.encode(), C.byref(error))
            self.check(error.value, f'create kernel {name}')
            self.kernels[name] = kernel
        for index, arg in enumerate(args):
            key = (name, index)
            signature = ('local', arg.size) if isinstance(arg, LocalMemory) else (
                ('buffer', arg.handle) if isinstance(arg, Buffer) else
                ('float', struct.pack('=f', C.c_float(float(arg)).value)) if isinstance(arg, (float, np.floating)) else
                ('int', int(arg)))
            if self.kernel_arguments.get(key) == signature:
                continue
            if isinstance(arg, LocalMemory):
                self.check(self.lib.clSetKernelArg(kernel, index, arg.size, None), f'arg {name}[{index}]')
            else:
                value = C.c_void_p(arg.handle) if isinstance(arg, Buffer) else (
                    C.c_float(float(arg)) if isinstance(arg, (float, np.floating)) else C.c_int(int(arg)))
                self.check(self.lib.clSetKernelArg(kernel, index, C.sizeof(value), C.byref(value)), f'arg {name}[{index}]')
            self.kernel_arguments[key] = signature
            self.argument_set_calls += 1
        sizes = (C.c_size_t*1)(int(size))
        group = None if local is None else (C.c_size_t*1)(int(local))
        event = C.c_void_p()
        self.check(self.lib.clEnqueueNDRangeKernel(self.queue, kernel, 1, None, sizes, group,
                    0, None, C.byref(event)), f'launch {name}')
        self.events.append((name, event))
        self._pending_commands = True
        self.kernel_launch_calls += 1

    def read_many(self, requests):
        """Queue independent readbacks, then wait once with host arrays alive."""
        prepared = []
        for buf, shape, dtype in requests:
            result = np.empty(shape, dtype)
            if result.nbytes > buf.capacity:
                raise OpenCLError('Read exceeds device buffer capacity')
            prepared.append((buf, result))
        nonempty = [(buf, result) for buf, result in prepared if result.nbytes]
        try:
            for index, (buf, result) in enumerate(nonempty):
                # The last blocking read fences earlier reads on the in-order
                # queue. Profiling stays deferred until frame completion.
                blocking = int(index == len(nonempty)-1)
                self.check(self.lib.clEnqueueReadBuffer(self.queue, buf.handle, blocking, 0, result.nbytes,
                           C.c_void_p(result.ctypes.data), 0, None, None), 'download batch')
                self.download_bytes += result.nbytes
                self.record_transfer('download', getattr(self, '_buffer_names', {}).get(buf.handle, 'external'), result.nbytes)
                self._pending_commands = not bool(blocking)
        except BaseException:
            # Also finish after a failed enqueue, so an earlier asynchronous
            # read cannot write into a host array after its lifetime ends.
            self.finish()
            raise
        if nonempty:
            self.read_wait_calls = getattr(self, 'read_wait_calls', 0)+1
            self.pending_uploads.clear()
        return [result for _, result in prepared]

    def finish(self):
        pending, self.events = self.events, []
        started_cpu = time.thread_time()
        try:
            if getattr(self, '_pending_commands', True):
                self.check(self.lib.clFinish(self.queue), 'finish')
                self.finish_calls = getattr(self, 'finish_calls', 0)+1
                self._pending_commands = False
            self.pending_uploads.clear()
            for name, event in pending:
                start, end = C.c_uint64(), C.c_uint64()
                self.check(self.lib.clGetEventProfilingInfo(event, 0x1282, 8, C.byref(start), None), 'profile start')
                self.check(self.lib.clGetEventProfilingInfo(event, 0x1283, 8, C.byref(end), None), 'profile end')
                self.kernel_ms[name] = self.kernel_ms.get(name, 0.)+(end.value-start.value)/1e6
        finally:
            for _, event in pending:
                self.lib.clReleaseEvent(event)
            self.profiling_cpu_ms = getattr(self, 'profiling_cpu_ms', 0.)+(time.thread_time()-started_cpu)*1000

    def reset_stats(self):
        self.kernel_ms = {}
        self.upload_bytes = self.download_bytes = 0
        self.transfer_bytes, self.transfer_calls = {}, {}
        self.finish_calls = self.read_wait_calls = 0
        self.kernel_launch_calls = self.argument_set_calls = 0
        self.profiling_cpu_ms = 0.

    def record_transfer(self, direction, name, nbytes):
        # Also works with minimal runtime instances used by failure-path tests.
        if not hasattr(self, 'transfer_bytes'):
            self.transfer_bytes, self.transfer_calls = {}, {}
        key = direction+':'+name
        self.transfer_bytes[key] = self.transfer_bytes.get(key, 0)+int(nbytes)
        self.transfer_calls[key] = self.transfer_calls.get(key, 0)+1

    def close(self):
        if self.queue:
            self.lib.clFinish(self.queue)
        self.pending_uploads.clear()
        for _, event in self.events:
            self.lib.clReleaseEvent(event)
        self.events.clear()
        for kernel in self.kernels.values():
            self.lib.clReleaseKernel(kernel)
        self.kernels.clear()
        for buffer in self.buffers.values():
            self.lib.clReleaseMemObject(buffer.handle)
        self.buffers.clear()
        for attr, release in (('program', 'clReleaseProgram'), ('queue', 'clReleaseCommandQueue'), ('context', 'clReleaseContext')):
            value = getattr(self, attr, None)
            if value:
                getattr(self.lib, release)(value)
                setattr(self, attr, None)
