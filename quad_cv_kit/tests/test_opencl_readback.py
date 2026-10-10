"""Batch readback owns host memory through completion and failed enqueues."""
import ctypes
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.opencl_runtime import Buffer, OpenCLError, OpenCLRuntime


class BatchReadbackTests(unittest.TestCase):
    def runtime(self, fail_second=False):
        runtime = OpenCLRuntime.__new__(OpenCLRuntime)
        runtime.queue, runtime.download_bytes = 1, 0
        runtime.pending_uploads = [np.array([9], np.uint8)]
        runtime.lib = Mock()
        pending = []
        source = {11: np.array([3, 4], np.int32), 12: np.array([[2., 3.]], np.float32)}
        def enqueue(queue, handle, blocking, offset, nbytes, address, *unused):
            if fail_second and handle == 12:
                return -5
            pending.append((address.value, source[handle]))
            if blocking:
                finish()
            return 0
        def finish():
            for address, data in pending:
                ctypes.memmove(address, data.ctypes.data, data.nbytes)
        runtime.lib.clEnqueueReadBuffer.side_effect = enqueue
        runtime.finish = Mock(side_effect=finish)
        return runtime

    def test_multiple_arrays_are_ready_after_one_completion(self):
        runtime = self.runtime()
        values = runtime.read_many([(Buffer(11, 8), (2,), np.int32), (Buffer(12, 8), (1, 2), np.float32)])
        np.testing.assert_array_equal(values[0], [3, 4])
        np.testing.assert_array_equal(values[1], [[2., 3.]])
        runtime.finish.assert_not_called()
        self.assertEqual([call.args[2] for call in runtime.lib.clEnqueueReadBuffer.call_args_list], [0, 1])
        self.assertEqual(runtime.pending_uploads, [])
        self.assertEqual(runtime.read_wait_calls, 1)
        self.assertEqual(runtime.download_bytes, 16)

    def test_failed_enqueue_finishes_earlier_reads_before_host_arrays_die(self):
        runtime = self.runtime(fail_second=True)
        with self.assertRaises(OpenCLError):
            runtime.read_many([(Buffer(11, 8), (2,), np.int32), (Buffer(12, 8), (1, 2), np.float32)])
        runtime.finish.assert_called_once()
        self.assertEqual(runtime.download_bytes, 8)

    def test_all_capacities_are_validated_before_any_read_is_queued(self):
        runtime = self.runtime()
        with self.assertRaises(OpenCLError):
            runtime.read_many([(Buffer(11, 8), (2,), np.int32), (Buffer(12, 4), (1, 2), np.float32)])
        runtime.lib.clEnqueueReadBuffer.assert_not_called()


if __name__ == '__main__':
    unittest.main()
