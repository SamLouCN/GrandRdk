"""ROI context and overlapping line extraction must preserve observations."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D, gate_models as M
from src.cv_profile import CvFrameProfile
from src.opencl_backend import OpenCLBackend


class CompressionTests(unittest.TestCase):
    def test_parallel_lsd_matches_serial_on_mask_chroma_and_gray_crops(self):
        rng = np.random.default_rng(101)
        frame = rng.integers(0, 256, (360, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (160, 80), (490, 290), (50, 60, 230), 8)
        channels = [M.tube_evidence(frame)[0], cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                    cv2.createCLAHE(2., (8, 8)).apply(cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[:, :, 1])]
        regions = [(145, 65, 505, 305), (0, 0, 105, 130)]
        backend = OpenCLBackend.__new__(OpenCLBackend)
        backend._lsd_executor = None
        try:
            futures = [backend.submit_lsd_regions(image, regions) for image in channels]
            detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
            for image, future in zip(channels, futures):
                expected = []
                for x0, y0, x1, y1 in regions:
                    found = detector.detect(np.ascontiguousarray(image[y0:y1, x0:x1]))[0]
                    if found is not None:
                        expected.extend(found.reshape(-1, 4)+[x0, y0, x0, y0])
                rows, elapsed = future.result()
                np.testing.assert_array_equal(rows, expected)
                self.assertGreater(elapsed, 0)
        finally:
            if backend._lsd_executor is not None:
                backend._lsd_executor.shutdown(wait=True)

    def test_roi_colors_keep_gaussian_context_and_clear_outside(self):
        rng = np.random.default_rng(37)
        clean = rng.integers(0, 256, (360, 640, 3), dtype=np.uint8)
        extra = cv2.convertScaleAbs(clean, alpha=1.05)
        full_mask, full_score = M.combined_evidence(extra, clean)
        for bounds in ((230, 100, 400, 260), (0, 3, 130, 127), (501, 244, 640, 360)):
            mask, score = M.combined_evidence(extra, clean, bounds=bounds)
            x0, y0, x1, y1 = bounds
            np.testing.assert_array_equal(mask[y0:y1, x0:x1], full_mask[y0:y1, x0:x1])
            np.testing.assert_array_equal(score[y0:y1, x0:x1], full_score[y0:y1, x0:x1])
            mask[y0:y1, x0:x1] = 0
            self.assertFalse(mask.any())

    def test_strict_partial_crop_preserves_boundary_components(self):
        rng = np.random.default_rng(81)
        for bounds in ((230, 100, 400, 260), (0, 3, 130, 127), (501, 244, 640, 360)):
            frame = np.zeros((360, 640, 3), np.uint8)
            x0, y0, x1, y1 = bounds
            frame[y0:y1, x0:x1] = rng.integers(0, 256, (y1-y0, x1-x0, 3), dtype=np.uint8)
            np.testing.assert_array_equal(D.red_mask(frame, bounds=bounds), D.red_mask(frame))

    def test_parallel_hough_preserves_order_and_joins_before_gpu_fit(self):
        class Backend:
            quality = 'fast'
            executor = ThreadPoolExecutor(max_workers=1)
            joined = False

            def hough_segments(self, mask):
                return cv2.HoughLinesP(mask, 1, np.pi/720, 35, minLineLength=55, maxLineGap=10)

            def submit_hough_regions(self, mask, regions):
                def extract():
                    rows = []
                    for x0, y0, x1, y1 in regions:
                        found = self.hough_segments(np.ascontiguousarray(mask[y0:y1, x0:x1]))
                        if found is not None:
                            rows.extend(found.reshape(-1, 4)+[x0, y0, x0, y0])
                    self.joined = True
                    return rows
                return self.executor.submit(extract)

            def fit_segments(self, mask, segments, support):
                assert self.joined
                return [q for s in segments if (q := D.line_from_segment(mask, s, support)) is not None]

            def contrast_signal(self, frame):
                return D.contrast_signal(frame)

        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (220, 100), (410, 280), (50, 60, 230), 8)
        mask, _ = M.combined_evidence(frame, frame)
        backend = Backend()
        seeds = {};profile = CvFrameProfile(True)
        try:
            D.get_lines(frame, mask, profile=profile, backend=backend, seed_cache=seeds)
            found = backend.hough_segments(mask)
            np.testing.assert_array_equal(seeds['hough'], found.reshape(-1, 4))
            self.assertTrue(profile.meta['line_extract_parallel'])
            self.assertIn('lines.hough_join', profile.stages)
        finally:
            backend.executor.shutdown(wait=True)

    def test_lsd_exception_waits_for_hough_before_returning(self):
        class Backend:
            quality = 'fast'
            executor = ThreadPoolExecutor(max_workers=1)
            finished = False

            def submit_hough_regions(self, mask, regions):
                def work():
                    self.finished = True
                    return []
                return self.executor.submit(work)

        frame = np.zeros((360, 640, 3), np.uint8)
        backend = Backend()
        try:
            with patch.object(cv2, 'createLineSegmentDetector') as factory:
                factory.return_value.detect.side_effect = ValueError('LSD failed')
                with self.assertRaisesRegex(ValueError, 'LSD failed'):
                    D.get_lines(frame, frame[:, :, 0], backend=backend)
            self.assertTrue(backend.finished)
        finally:
            backend.executor.shutdown(wait=True)


if __name__ == '__main__':
    unittest.main()
