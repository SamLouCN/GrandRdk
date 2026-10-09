"""Optimization regressions: color evidence, exact rejections and cropped geometry."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D
from src import gate_models as M
from src.cv_profile import CvFrameProfile
from src.gate_line_geometry import samples


class CvOptimizationTests(unittest.TestCase):
    def scene(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (220, 100), (410, 280), (50, 60, 230), 8)
        # A weak top and white elbows must survive crop offsets and padding.
        cv2.line(frame, (220, 100), (410, 100), (105, 105, 140), 8)
        for point in ((220, 100), (410, 100), (410, 280), (220, 280)):
            cv2.circle(frame, point, 6, (220, 220, 220), -1)
        return frame

    def test_sampling_matches_full_probe_tensor_at_boundaries(self):
        rng = np.random.default_rng(31)
        mask = np.uint8(rng.random((75, 110)) > .7)*255
        for radius in (0, 5, 10, 16):
            for _ in range(25):
                a, b = rng.uniform([-25, -25], [135, 100], (2, 2))
                points, hits, normal = samples(mask, a, b, radius)
                probes = points[:, None, :]+np.arange(-radius, radius+1)[None, :, None]*normal
                xs, ys = np.rint(probes).astype(int).transpose(2, 0, 1)
                valid = (xs >= 0) & (xs < 110) & (ys >= 0) & (ys < 75)
                expected = (mask[np.clip(ys, 0, 74), np.clip(xs, 0, 109)] > 0) & valid
                np.testing.assert_array_equal(hits, expected)

    def test_prefilter_never_drops_a_supported_segment(self):
        frame = self.scene()
        mask, _ = M.tube_evidence(frame)
        rng = np.random.default_rng(12)
        segments = rng.uniform([-30, -30, -30, -30], [670, 390, 670, 390], (500, 4))
        segments = np.vstack([segments, [220, 100, 220, 280], [220, 280, 410, 280],
                              [410, 100, 410, 280], [410, 100, 220, 100]])
        retained = D.eligible_segments(mask, segments)
        accepted = 0
        for segment in segments:
            if D.line_from_segment(mask, segment, .48) is not None:
                self.assertTrue(np.any(np.all(retained == segment, axis=1)))
                accepted += 1
        self.assertGreater(accepted, 3)
        self.assertLess(len(retained), len(segments))
        self.assertEqual(D.eligible_segments(mask, []).shape, (0, 4))
        self.assertEqual(len(D.eligible_segments(np.zeros_like(mask), segments)), 0)

    def test_cached_contrast_is_exact_and_built_once_per_source(self):
        frame = self.scene()
        enhanced = cv2.convertScaleAbs(frame, alpha=1.2)
        mask, _ = M.combined_evidence(enhanced, frame)
        profile = CvFrameProfile(True)
        vertical, horizontal = D.get_lines(enhanced, mask, frame, profile)
        self.assertGreaterEqual(len(vertical)+len(horizontal), 4)
        self.assertEqual(profile.finish()['counts']['contrast_full_image_calls'], 2)
        blue, green, red = cv2.split(frame.astype(np.float32))
        expected = np.log((red+10)/(green+10))
        np.testing.assert_array_equal(D.contrast_signal(frame), expected)
        for line in vertical+horizontal:
            self.assertEqual(D.tube_contrast(frame, line),
                             D.tube_contrast(frame, line, signal=expected))

    def test_color_blur_memory_optimization_preserves_weak_evidence(self):
        rng = np.random.default_rng(27)
        for frame in (self.scene(), rng.integers(0, 256, (120, 170, 3), dtype=np.uint8)):
            actual_mask, actual_score = M.tube_evidence(frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            chroma = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)[:, :, 1]
            local = np.maximum.reduce([chroma-cv2.GaussianBlur(chroma, (0, 0), sigma)
                                       for sigma in (3, 9, 18)])
            hue, saturation, value = cv2.split(hsv)
            colored = (((hue < 14) | (hue > 118)) & (saturation > 4)) | (
                (saturation < 60) & (chroma > 126) & (local > 3))
            red = colored & (chroma > 125) & (value > 18) & ((local > 1.3) | (chroma > 136))
            expected_mask = cv2.morphologyEx(np.uint8(red)*255, cv2.MORPH_CLOSE,
                                             np.ones((3, 3), np.uint8))
            expected_score = np.float32(np.clip((chroma-125)/15, 0, 1)*np.clip(local/4, 0, 1)*colored)
            np.testing.assert_array_equal(actual_mask, expected_mask)
            np.testing.assert_array_equal(actual_score, expected_score)

    def test_cropped_search_keeps_weak_gate_scale_offset_and_full_mask(self):
        # Nonaligned ROIs exercise LSD's 0.8 sampling grid. The 4:3 source
        # exercises the additional 80px offset in the processing canvas.
        for size in ((640, 360), (1280, 720), (480, 360)):
            frame = cv2.resize(self.scene(), size)
            sx, sy = size[0]/640, size[1]/360
            roi = np.array([193*sx, 73*sy, 438*sx, 312*sy])
            profile = CvFrameProfile(True)
            candidates, _, mask = D.detect(frame, search_bbox=roi, profile=profile)
            complete = [candidate for candidate in candidates if candidate['complete']]
            self.assertTrue(complete, size)
            geometry = D.original_geometry(frame, complete[0])
            np.testing.assert_allclose(geometry['bbox'],
                [220*sx, 100*sy, 410*sx, 280*sy], atol=5)
            self.assertEqual(mask.shape, (360, 640))
            extraction = profile.finish()['line_extraction_size']
            self.assertLess(extraction[0]*extraction[1], 640*360)
            self.assertEqual(extraction[0] % 5, 0)
            self.assertEqual(extraction[1] % 5, 0)

    def test_tracker_reuse_matches_standalone_search_and_does_not_keep_old_color(self):
        frame = self.scene()
        enhanced = cv2.convertScaleAbs(frame, alpha=1.2)
        roi = [193, 73, 438, 312]
        tracker = D.RedGateTracker(detect_every=1, profile=True)
        first, _ = tracker.update(enhanced, roi, reference_frame=frame)
        self.assertIsNotNone(first)
        with patch.object(D, 'move_with_image', return_value=None):
            actual, _ = tracker.update(enhanced, roi, reference_frame=frame)
        expected = D.select_nearest(D.detect(enhanced, roi, reference_frame=frame)[0])
        self.assertEqual(D.original_geometry(frame, actual), D.original_geometry(frame, expected))
        self.assertEqual(tracker.last_status['cv_profile']['counts']['color_evidence_calls'], 1)
        self.assertEqual(tracker.last_status['cv_profile']['counts']['color_evidence_reuses'], 1)
        water = np.full_like(frame, (120, 90, 40))
        missing, _ = tracker.update(water, roi, reference_frame=water)
        self.assertIsNone(missing)

    def test_clutter_roi_retains_four_observed_sides(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (220, 130), (390, 275), (50, 60, 230), 5)
        rng = np.random.default_rng(17)
        for _ in range(50):
            x, y = rng.integers([50, 30], [590, 330])
            length = int(rng.integers(35, 130))
            end = ((min(639, int(x)+length), int(y)) if rng.random() < .5
                   else (int(x), min(359, int(y)+length)))
            cv2.line(frame, (int(x), int(y)), end, (50, 60, 230), int(rng.integers(2, 7)))
        # This crop lost the complete gate when its origin was not aligned
        # to LSD's resize grid, even with the same full-frame color mask.
        candidates, _, _ = D.detect(frame, search_bbox=[120, 65, 510, 270])
        complete = [candidate for candidate in candidates if candidate['complete']]
        self.assertTrue(complete)
        self.assertTrue(all(len(candidate['side_support']) == 4 for candidate in complete))


if __name__ == '__main__':
    unittest.main()
