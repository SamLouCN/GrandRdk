"""Cached dispatch still updates changed buffers, values and local scratch."""
import ctypes as C
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.opencl_runtime import Buffer, LocalMemory, OpenCLRuntime


class DispatchTests(unittest.TestCase):
    def runtime(self):
        rt = OpenCLRuntime.__new__(OpenCLRuntime)
        rt.queue, rt.kernels, rt.events, rt.kernel_arguments = 1, {'probe': 9}, [], {}
        rt.info = dict(max_work_group_size=512, local_memory_bytes=32768)
        rt.pending_uploads = []
        rt._pending_commands = False
        rt.reset_stats()
        rt.lib = Mock()
        rt.lib.clSetKernelArg.return_value = 0
        rt.lib.clEnqueueNDRangeKernel.return_value = 0
        return rt

    def test_changed_inputs_update_args_while_unchanged_dispatch_reuses_them(self):
        rt = self.runtime()
        arguments = [Buffer(11, 16), 4, 1.25, LocalMemory(32)]
        rt.run('probe', 64, arguments, local=64)
        self.assertEqual(rt.lib.clSetKernelArg.call_count, 4)
        rt.run('probe', 64, arguments, local=64)
        self.assertEqual(rt.lib.clSetKernelArg.call_count, 4)
        rt.run('probe', 64, [Buffer(12, 16), 5, 1.5, LocalMemory(64)], local=64)
        self.assertEqual(rt.lib.clSetKernelArg.call_count, 8)
        self.assertEqual(rt.lib.clEnqueueNDRangeKernel.call_count, 3)

    def test_negative_zero_is_a_distinct_kernel_argument(self):
        rt = self.runtime()
        rt.run('probe', 1, [0.0])
        rt.run('probe', 1, [-0.0])
        self.assertEqual(rt.lib.clSetKernelArg.call_count, 2)

    def test_completed_commands_collect_events_without_another_queue_finish(self):
        rt = self.runtime()
        rt.events = [('probe', C.c_void_p(123))]
        def profile(event, key, size, output, unused):
            C.cast(output, C.POINTER(C.c_uint64)).contents.value = 100 if key == 0x1282 else 1100100
            return 0
        rt.lib.clGetEventProfilingInfo.side_effect = profile
        rt.finish()
        rt.lib.clFinish.assert_not_called()
        self.assertEqual(rt.kernel_ms, {'probe': 1.1})
        rt.lib.clReleaseEvent.assert_called_once()
