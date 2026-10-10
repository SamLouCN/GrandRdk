"""Local search must retain measured sides and reacquire in the same frame."""
from pathlib import Path
import json
import shutil
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D
from src.cv_profile import CvFrameProfile


class PredictiveSearchTests(unittest.TestCase):
    def test_full_roi_and_band_profiles_are_native_json_serializable(self):
        frame, _ = self.scene()
        tracker = D.RedGateTracker(profile=True)
        scopes = []
        for _ in range(4):
            tracker.update(frame, search_bbox=[130, 50, 520, 320])
            record = json.loads(json.dumps(tracker.last_status, allow_nan=False))
            profile = record['cv_profile']
            if profile['mode'] == 'search':
                scopes.append(profile['search_scope'])
                for region in tracker.last_status['cv_profile']['line_extraction_regions']:
                    self.assertTrue(all(type(value) is int for value in region))
        self.assertEqual(scopes, ['full', 'bands'])

    def scene(self, shift=(0, 0), weak=False, missing=False):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        corners = np.array([[160, 80], [490, 80], [490, 290], [160, 290]], float)+shift
        for i, a in enumerate(corners):
            if missing and i == 2:
                continue
            b = corners[(i+1) % 4]
            cv2.line(frame, tuple(a.astype(int)), tuple(b.astype(int)),
                     (105, 105, 140) if weak and i == 0 else (50, 60, 230), 8)
        return frame, corners

    def prediction(self, frame):
        candidates, _, _ = D.detect(frame)
        return dict(D.select_nearest(candidates), motion_inlier_ratio=1., motion_rms=0., motion_px=0.)

    def local(self, frame, prediction, **kwargs):
        profile = CvFrameProfile(True)
        result = D.detect(frame, profile=profile, _prediction=prediction, **kwargs)
        return result, profile.finish()

    def assert_full_equivalent(self, frame, prediction, **kwargs):
        actual, profile = self.local(frame, prediction, **kwargs)
        expected = D.detect(frame, **kwargs)
        self.assertEqual(profile['search_scope'], 'full')
        self.assertEqual(len(actual[0]), len(expected[0]))
        self.assertEqual(D.original_geometry(frame, D.select_nearest(actual[0])),
                         D.original_geometry(frame, D.select_nearest(expected[0])))
        np.testing.assert_array_equal(actual[2], expected[2])
        return profile

    def test_strong_isolated_gate_skips_chroma_and_refines_dense_current_geometry(self):
        frame, corners = self.scene()
        prediction = self.prediction(frame)
        result, profile = self.local(frame, prediction)
        self.assertEqual(profile['search_scope'], 'bands')
        self.assertEqual(profile['lsd_policy'], 'mask-only')
        self.assertNotIn('lines.lsd_chroma', profile['stages_ms'])
        self.assertIn('detect.band_dense_refine', profile['stages_ms'])
        candidate = D.select_nearest(result[0])
        np.testing.assert_allclose(candidate['corners'], corners, atol=1.5)
        self.assertTrue(candidate['geometry_validated'])
        self.assertEqual(min(candidate['side_support']), 1.)
        # Validate actual widths as well as centerline positions.
        self.assertTrue(all(8 <= line['width'] <= 10 for line in candidate['lines']))

    def test_weak_side_requests_chroma_and_reuses_mask_hough_seeds(self):
        frame, corners = self.scene(weak=True)
        prediction = self.prediction(frame)
        with patch.object(D, 'strong_color_on_sides', return_value=False):
            result, profile = self.local(frame, prediction)
        self.assertIn('lines.lsd_chroma', profile['stages_ms'])
        self.assertGreaterEqual(profile['counts']['line_seed_cache_reuses'], 2)
        np.testing.assert_allclose(D.select_nearest(result[0])['corners'], corners, atol=3)

    def test_mask_failure_supplements_chroma_before_acceptance(self):
        frame, _ = self.scene()
        original = D.get_lines
        def fail_mask(*args, **kwargs):
            lines = original(*args, **kwargs)
            return ([], []) if kwargs.get('lsd_mode') == 'mask' else lines
        with patch.object(D, 'get_lines', side_effect=fail_mask):
            result, profile = self.local(frame, self.prediction(frame))
        self.assertTrue(result[0])
        self.assertEqual(profile['lsd_policy'], 'supplemented')
        self.assertGreaterEqual(profile['counts']['line_seed_cache_reuses'], 2)

    def test_competing_gate_outside_bands_runs_original_full_search(self):
        frame, _ = self.scene()
        prediction = self.prediction(frame)
        cv2.rectangle(frame, (230, 130), (400, 245), (50, 60, 230), 12)
        profile = self.assert_full_equivalent(frame, prediction)
        self.assertEqual(profile['band_fallback_reason'], 'outside_evidence')

    def test_fragmented_extra_red_rod_triggers_full_search(self):
        frame, _ = self.scene()
        prediction = self.prediction(frame)
        for x in range(230, 300, 8):
            cv2.line(frame, (x, 160), (x+3, 160), (50, 60, 230), 3)
        profile = self.assert_full_equivalent(frame, prediction)
        self.assertEqual(profile['band_fallback_reason'], 'outside_evidence')

    def test_target_reset_reacquires_new_gate_immediately(self):
        frame, _ = self.scene()
        tracker = D.RedGateTracker(profile=True)
        tracker.update(frame)
        tracker.reset()
        shifted, corners = self.scene(shift=(40, 0))
        result, _ = tracker.update(shifted)
        self.assertEqual(tracker.last_status['cv_profile']['search_scope'], 'full')
        np.testing.assert_allclose(result['corners'], corners, atol=3)

    def test_small_motion_matches_measured_ground_truth(self):
        tracker = D.RedGateTracker(profile=True)
        baseline = D.RedGateTracker(profile=True, adaptive_search=False)
        for i in range(10):
            frame, corners = self.scene(shift=(i, 0))
            actual, _ = tracker.update(frame)
            expected, _ = baseline.update(frame)
            self.assertEqual(actual['complete'], expected['complete'])
            np.testing.assert_allclose(actual['corners'], corners, atol=2)
            np.testing.assert_allclose(actual['corners'], expected['corners'], atol=2)
            self.assertAlmostEqual(D.apparent_pipe_width(actual), D.apparent_pipe_width(expected), delta=.5)

    def test_missing_side_cannot_be_restored_from_prediction(self):
        before, _ = self.scene()
        current, _ = self.scene(missing=True)
        profile = self.assert_full_equivalent(current, self.prediction(before))
        self.assertEqual(profile['band_fallback_reason'], 'local_model_not_certified')
        self.assertEqual(profile['counts']['band_search_fallbacks'], 1)

    def test_overconfident_wrong_prediction_falls_back_same_frame(self):
        frame, _ = self.scene()
        prediction = self.prediction(frame)
        shifted, _ = self.scene(shift=(35, 0))
        self.assert_full_equivalent(shifted, prediction)

    def test_partial_predictions_and_large_motion_never_enable_band_search(self):
        frame, _ = self.scene()
        tracker = D.RedGateTracker(profile=True)
        tracker.update(frame)
        tracker.index = 3
        unreliable = dict(tracker.previous, motion_inlier_ratio=.9, motion_rms=0., motion_px=30.)
        with patch.object(D, 'move_with_image', return_value=unreliable):
            tracker.update(frame)
        self.assertEqual(tracker.last_status['cv_profile']['search_scope'], 'full')
        tracker.index = 6
        unreliable.update(motion_px=0., complete=False)
        with patch.object(D, 'move_with_image', return_value=unreliable):
            tracker.update(frame)
        self.assertEqual(tracker.last_status['cv_profile']['search_scope'], 'full')

    def test_periodic_full_search_reset_and_full_mode(self):
        frame, _ = self.scene()
        tracker = D.RedGateTracker(fps=30, profile=True)
        scopes = {}
        for i in range(16):
            tracker.update(frame)
            profile = tracker.last_status['cv_profile']
            if profile['mode'] == 'search':
                scopes[i] = profile['search_scope']
        self.assertEqual(scopes, {0: 'full', 3: 'bands', 6: 'bands', 9: 'bands', 12: 'bands', 15: 'full'})
        tracker.reset()
        tracker.update(frame)
        self.assertEqual(tracker.last_status['cv_profile']['search_scope'], 'full')
        baseline = D.RedGateTracker(profile=True, adaptive_search=False)
        for _ in range(4):
            baseline.update(frame)
        self.assertEqual(baseline.last_status['cv_profile']['search_scope'], 'full')

    def test_valid_mask_hole_forces_original_full_validation(self):
        frame, _ = self.scene()
        prediction = self.prediction(frame)
        valid = np.ones(frame.shape[:2], bool)
        valid[75:88, 310:335] = False
        self.assert_full_equivalent(frame, prediction, valid_mask=valid)

    def test_anchor_rejection_is_preserved_after_dense_refinement(self):
        frame, _ = self.scene()
        prediction = self.prediction(frame)
        self.assert_full_equivalent(frame, prediction, anchor_bbox=[250, 140, 350, 220])

    def test_crop_grid_and_canvas_offsets_are_preserved(self):
        for size in ((1280, 720), (480, 360)):
            base, _ = self.scene()
            frame = cv2.resize(base, size)
            prediction = self.prediction(frame)
            result, profile = self.local(frame, prediction)
            self.assertTrue(result[0])
            small_mask = result[2]
            regions, _ = D.prediction_bands(prediction, (0, 0, 640, 360), small_mask.shape)
            for region in regions:
                self.assertTrue(all(value % 5 == 0 for value in region))
            actual = D.original_geometry(frame, D.select_nearest(result[0]))
            expected = D.original_geometry(frame, prediction)
            np.testing.assert_allclose(actual['corners'], expected['corners'], atol=3)


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Compiled kernel checks require clang')
class CompiledPredictiveSearchTests(unittest.TestCase):
    def test_fast_backend_local_search_uses_dense_final_validation(self):
        from opencl_host import HostRuntime
        from src.opencl_backend import OpenCLBackend
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            backend = OpenCLBackend(quality='fast', blur_mode='pyramid')
        self.addCleanup(backend.close)
        scenarios = PredictiveSearchTests()
        frame, corners = scenarios.scene()
        result, profile = scenarios.local(frame, scenarios.prediction(frame), backend=backend)
        self.assertEqual(profile['search_scope'], 'bands')
        self.assertEqual(profile['lsd_policy'], 'mask-only')
        self.assertIn('hough_runs_fast', backend.runtime.kernel_ms)
        np.testing.assert_allclose(D.select_nearest(result[0])['corners'], corners, atol=1.5)


if __name__ == '__main__':
    unittest.main()
