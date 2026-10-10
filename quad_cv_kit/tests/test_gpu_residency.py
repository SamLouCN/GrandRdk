"""Production device kernels must reuse pixels without stale host-array caches."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opencl_host import HostRuntime
from src import detect_red_gate as D, gate_models as M
from src.opencl_backend import OpenCLBackend
from src.opencl_validation import check_resident_chain


class ResidencyTests(unittest.TestCase):
    def setUp(self):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            self.backend = OpenCLBackend(quality='fast', blur_mode='pyramid', hough_backend='cpu')
        self.addCleanup(self.backend.close)

    def assert_resident_pixels(self, array):
        buffer = self.backend.resident_buffer(array)
        self.assertIsNotNone(buffer)
        np.testing.assert_array_equal(self.backend.runtime.read(buffer, array.shape, array.dtype), array)

    def test_roi_mask_and_score_do_not_return_to_host_upload(self):
        rng = np.random.default_rng(31)
        reference = rng.integers(0, 256, (100, 160, 3), dtype=np.uint8)
        enhanced = cv2.convertScaleAbs(reference, alpha=1.1)
        bounds = (25, 20, 135, 85)
        with self.backend.frame_batch():
            mask, score = M.combined_evidence(enhanced, reference, backend=self.backend, bounds=bounds)
            before = self.backend.runtime.upload_bytes
            self.assert_resident_pixels(mask)
            self.assert_resident_pixels(score)
            copied = self.backend.copy_host(mask)
            roi = self.backend.restrict_mask(copied, bounds=(30, 25, 120, 70))
            self.assert_resident_pixels(roi)
            self.backend.search_input('mask', roi, np.uint8)
            self.backend.search_input('score', score, np.float32)
            self.assertEqual(self.backend.runtime.upload_bytes, before)

    def test_board_validator_checks_resident_kernel_bytes(self):
        check_resident_chain(self.backend)

    def test_crop_blackout_and_expansion_preserve_bytes_for_bgr_and_float(self):
        rng = np.random.default_rng(42)
        for source in (rng.integers(0, 256, (21, 34, 3), dtype=np.uint8),
                       rng.uniform(-10, 10, (21, 34)).astype(np.float32)):
            with self.backend.frame_batch():
                self.backend.device_input('source', source)
                before = self.backend.runtime.upload_bytes
                crop = np.ascontiguousarray(source[3:18, 5:27])
                self.backend.register_roi(crop, source, (5, 3, 27, 18))
                expanded = np.zeros_like(source)
                expanded[3:18, 5:27] = crop
                self.backend.register_roi(expanded, crop, (0, 0, 22, 15), (5, 3))
                self.assert_resident_pixels(crop)
                self.assert_resident_pixels(expanded)
                self.assertEqual(self.backend.runtime.upload_bytes, before)

    def test_integer_area_canvas_matches_cpu_and_does_not_upload_again(self):
        rng = np.random.default_rng(91)
        for shape in ((720, 1280, 3), (720, 960, 3), (360, 640, 3)):
            source = rng.integers(0, 256, shape, dtype=np.uint8)
            with self.backend.frame_batch():
                self.backend.device_input('source', source)
                before = self.backend.runtime.upload_bytes
                canvas = self.backend.prepare_canvas(source)[0]
                self.assert_resident_pixels(canvas)
                self.assertEqual(self.backend.runtime.upload_bytes, before)

    def test_mutable_frame_is_refreshed_and_nested_search_cannot_overwrite_input(self):
        source = np.zeros((19, 32), np.uint8)
        for value in (7, 231):
            source[:] = value
            with self.backend.frame_batch():
                self.backend.device_input('source', source)
                copied = self.backend.copy_host(source)
                with self.backend.frame_batch(), self.backend.search_batch():
                    self.backend.device_input('source', np.zeros_like(source))
                self.assert_resident_pixels(copied)
                self.assert_resident_pixels(source)
            self.assertIsNone(self.backend.resident_buffer(source))

    def test_readonly_valid_canvas_uploads_once_across_frames(self):
        valid = np.ones((720, 1280), bool)
        valid[:, :12] = False
        valid.setflags(write=False)
        for index in range(2):
            self.backend.reset_stats()
            with self.backend.frame_batch():
                reduced = self.backend.prepare_valid_canvas(valid, 1280, 720, .5, (0, 0))
                self.assert_resident_pixels(reduced)
                self.assertEqual(self.backend.runtime.upload_bytes, reduced.nbytes if index == 0 else 0)

    def test_full_search_has_no_color_mask_or_score_host_reupload(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (145, 75), (500, 285), (50, 60, 230), 8)
        valid = np.ones(frame.shape[:2], bool)
        with self.backend.frame_batch():
            candidates = D.detect(frame, search_bbox=[120, 55, 525, 310],
                                  valid_mask=valid, backend=self.backend)[0]
            self.assertTrue(candidates)
            keys = self.backend.runtime.transfer_bytes
            self.assertFalse(any(key.startswith('upload:') and
                                 ('search_mask' in key or 'search_score' in key or 'hough_mask' in key)
                                 for key in keys))
            self.assertIn('resident_roi', self.backend.runtime.kernel_ms)

    def test_preprocess_to_canvas_reuses_device_images(self):
        source = np.full((720, 1280, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(source, (290, 150), (1000, 570), (50, 60, 230), 16)
        valid = np.ones(source.shape[:2], bool)
        valid.setflags(write=False)
        with self.backend.frame_batch():
            fixed, enhanced, _, _ = self.backend.preprocess(source, valid)
            before = self.backend.runtime.upload_bytes
            first = self.backend.prepare_canvas(fixed)[0]
            second = self.backend.prepare_canvas(enhanced)[0]
            self.assert_resident_pixels(first)
            self.assert_resident_pixels(second)
            self.assertEqual(self.backend.runtime.upload_bytes, before)

    def test_stable_outputs_and_readonly_aliases_need_no_device_copies(self):
        source = np.full((720, 1280, 3), (120, 90, 40), np.uint8)
        y, x = np.indices(source.shape[:2], dtype=np.float32)
        with self.backend.frame_batch():
            fixed, enhanced, _, _ = self.backend.preprocess(source, None, maps=(x, y))
            canvas = self.backend.prepare_canvas(fixed)[0]
            extra = self.backend.prepare_canvas(enhanced)[0]
            mask, score = self.backend.combined_evidence(extra, canvas)
            alias = self.backend.copy_host(mask)
            self.assertEqual(self.backend.resident_buffer(mask).handle,
                             self.backend.resident_buffer(alias).handle)
            for array in (fixed, enhanced, mask, score):
                self.assert_resident_pixels(array)
            self.assertNotIn('resident_copy', self.backend.runtime.kernel_ms)

    def test_gpu_color_is_exact_for_random_pixels_and_cached_without_upload(self):
        rng = np.random.default_rng(987)
        source = rng.integers(0, 256, (117, 193, 3), dtype=np.uint8)
        self.backend.reset_stats()
        with self.backend.frame_batch():
            hsv, chroma = self.backend.color_device(source, 'random')
            rt = self.backend.runtime
            actual = rt.read(chroma, source.shape[:2], np.float32)
            expected = cv2.cvtColor(source, cv2.COLOR_BGR2LAB)[:, :, 1]
            np.testing.assert_array_equal(actual, expected)
            before = rt.upload_bytes
            self.assertEqual(self.backend.color_device(source, 'second'), (hsv, chroma))
            self.assertEqual(rt.upload_bytes, before)
            self.assertFalse(any('chroma' in key and key.startswith('upload:') for key in rt.transfer_bytes))

    def test_frame_operations_defer_finish_and_preserve_readable_outputs(self):
        source = np.full((36, 64, 3), (120, 90, 40), np.uint8)
        with patch.object(self.backend.runtime, 'finish', wraps=self.backend.runtime.finish) as finish:
            with self.backend.frame_batch():
                self.backend.combined_evidence(source, source)
                self.backend.red_mask(source)
                finish.assert_not_called()
            finish.assert_called_once()

    def test_two_immutable_valid_masks_do_not_overwrite_first_device_view(self):
        first = np.ones((360, 640), bool)
        second = first.copy()
        first[:, :100] = False
        second[:100] = False
        first.setflags(write=False)
        second.setflags(write=False)
        with self.backend.frame_batch():
            a = self.backend.prepare_valid_canvas(first, 640, 360, 1, (0, 0))
            b = self.backend.prepare_valid_canvas(second, 640, 360, 1, (0, 0))
            self.assert_resident_pixels(a)
            self.assert_resident_pixels(b)


if __name__ == '__main__':
    unittest.main()
