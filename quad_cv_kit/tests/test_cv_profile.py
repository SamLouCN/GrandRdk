"""Timing accounting, spike retention and unchanged gate detection/tracking."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D
from src.cv_profile import CvFrameProfile, CvProfileReporter


class CvProfileTests(unittest.TestCase):
    def scene(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (140, 80), (480, 280), (50, 60, 230), 8)
        return frame

    def test_stage_times_do_not_double_count_nested_work(self):
        with patch('src.cv_profile.time.perf_counter', side_effect=[0, .010, .025, .040]), \
             patch('src.cv_profile.time.thread_time', side_effect=[0, .004, .010, .015]):
            profile = CvFrameProfile(True)
            profile.mark('color')
            profile.mark('lines')
            result = profile.finish()
        self.assertEqual(result['stages_ms'], {'color': 10, 'lines': 15, 'tracker.finalize': 15})
        self.assertEqual(sum(result['stages_ms'].values()), result['total_ms'])
        self.assertEqual(sum(result['stages_thread_cpu_ms'].values()), result['thread_cpu_ms'])
        self.assertIsNone(CvFrameProfile().finish())

    def test_window_keeps_unsampled_slow_frame_and_separates_search_from_idle(self):
        reporter = CvProfileReporter()

        def sample(value, mode='search'):
            return dict(total_ms=value, thread_cpu_ms=value, mode=mode, search_reason='no_previous',
                        stages_ms={'models.color_support': value}, stages_thread_cpu_ms={'models.color_support': value},
                        counts={'quad_combinations': 2025})

        self.assertEqual(reporter.add(sample(570), 1, now=0)[0]['event'], 'slow')
        self.assertEqual(reporter.add(sample(620), 2, now=.5), [])
        report = reporter.add(sample(1, 'idle'), 3, now=2)[0]
        self.assertEqual(report['event'], 'window')
        self.assertEqual(report['worst']['frame'], 2)
        self.assertEqual(report['modes']['search']['n'], 2)
        self.assertEqual(report['modes']['search']['max_ms'], 620)
        self.assertEqual(report['modes']['idle']['mean_ms'], 1)

    def test_profiling_preserves_candidates_geometry_masks_and_tracking(self):
        plain, timed = D.RedGateTracker(), D.RedGateTracker(profile=True)
        frame = self.scene()
        # Image-supported tracking, a scheduled fresh search and disabled detection.
        for allow in (True, True, True, True, False):
            expected, _ = plain.update(frame, allow_detect=allow, reference_frame=frame)
            actual, _ = timed.update(frame, allow_detect=allow, reference_frame=frame)
            self.assertEqual(D.original_geometry(frame, actual), D.original_geometry(frame, expected))
            status = dict(timed.last_status)
            profile = status.pop('cv_profile')
            self.assertEqual(status, plain.last_status)
            self.assertAlmostEqual(sum(profile['stages_ms'].values()), profile['total_ms'], delta=.1)
            if profile['mode'] == 'search':
                self.assertGreater(profile['counts']['raw_segments'], 0)
                self.assertIn('models.color_support', profile['stages_ms'])
        plain_result = D.detect(frame, reference_frame=frame)
        profile = CvFrameProfile(True)
        timed_result = D.detect(frame, reference_frame=frame, profile=profile)
        np.testing.assert_array_equal(plain_result[2], timed_result[2])
        self.assertEqual(len(plain_result[0]), len(timed_result[0]))

    def test_failed_tracking_search_reuses_current_frame_evidence(self):
        tracker = D.RedGateTracker(profile=True)
        tracker.update(self.scene())
        with patch.object(D, 'move_with_image', return_value=None):
            tracker.update(self.scene())
        profile = tracker.last_status['cv_profile']
        self.assertEqual(profile['search_reason'], 'tracking_failed')
        self.assertEqual(profile['counts']['color_evidence_calls'], 1)
        self.assertEqual(profile['counts']['color_evidence_reuses'], 1)


if __name__ == '__main__':
    unittest.main()
