"""Hybrid regressions: ROI isolation, clipped continuity, switching and YOLO expiry."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D
from src.yolo_red_gate import YoloRedGateTracker, touches_boundary, search_region


def door(box, score=.9):
    return dict(class_id=0, label='door', score=score, bbox=box)


class FakeDetector:
    target_ids = {0}

    def __init__(self, sequence):
        self.sequence = iter(sequence)

    def detect_boxes(self, frame):
        return next(self.sequence)


class HybridTests(unittest.TestCase):
    def scene(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (220, 130), (390, 275), (50, 60, 230), 5)
        cv2.line(frame, (5, 55), (535, 55), (50, 60, 230), 14)
        cv2.line(frame, (535, 55), (535, 359), (50, 60, 230), 14)
        cv2.circle(frame, (535, 55), 9, (220, 220, 220), -1)
        return frame

    def test_normal_yolo_roi_excludes_thicker_unselected_door(self):
        tracker = YoloRedGateTracker(FakeDetector([[door([210, 120, 400, 285])]]))
        selected, _ = tracker.update(self.scene())
        self.assertIsNotNone(selected)
        points = np.concatenate(selected['segments'])
        self.assertLess(float(points[:, 0].max()), 410)
        self.assertGreater(float(points[:, 1].min()), 115)
        self.assertLess(D.apparent_pipe_width(selected), 10)

    def test_clipped_roi_allows_actual_edges_outside_yolo_box(self):
        tracker = YoloRedGateTracker(FakeDetector([[door([440, 40, 545, 360])]]))
        selected, _ = tracker.update(self.scene())
        self.assertIsNotNone(selected)
        self.assertTrue(tracker.last_status['target_clipped'])
        self.assertLess(float(np.concatenate(selected['segments'])[:, 0].min()), 430)
        self.assertGreater(D.apparent_pipe_width(selected), 10)

    def test_matching_clipped_foreground_beats_larger_background(self):
        first = door([5, 40, 545, 360])
        partial = door([400, 40, 545, 360])
        background = door([150, 80, 470, 330])
        tracker = YoloRedGateTracker(FakeDetector([[first], [partial, background]]), detect_every=1)
        tracker.update(self.scene())
        selected, _ = tracker.update(self.scene())
        self.assertEqual(tracker.last_status['target_bbox'], partial['bbox'])
        self.assertEqual(tracker.last_status['selection_reason'], 'clipped-target-continuity')
        self.assertGreater(D.apparent_pipe_width(selected), 10)

    def test_no_yolo_does_not_start_cv(self):
        tracker = YoloRedGateTracker(FakeDetector([[]]))
        with patch.object(D, 'detect', side_effect=AssertionError('CV must be gated')):
            selected, _ = tracker.update(self.scene())
        self.assertIsNone(selected)
        self.assertFalse(tracker.last_status['cv_enabled'])

    def test_yolo_gap_expires_even_if_image_tracking_keeps_succeeding(self):
        tracker = YoloRedGateTracker(FakeDetector([[door([210, 120, 400, 285])], [], [], []]),
                                    fps=10, hold_seconds=.2)
        first, _ = tracker.update(self.scene())
        self.assertIsNotNone(first)
        with patch.object(D, 'move_with_image', return_value=dict(first, tracked=True)):
            for _ in range(2):
                selected, _ = tracker.update(self.scene())
                self.assertIsNotNone(selected)
                self.assertFalse(tracker.last_status['cv_enabled'])
            expired, _ = tracker.update(self.scene())
            self.assertIsNone(expired)
            self.assertIsNone(tracker.last_status['target_bbox'])

    def test_target_switch_clears_old_pipe_tracks(self):
        tracker = YoloRedGateTracker(FakeDetector([[door([210, 120, 400, 285])],
                                                  [door([5, 40, 545, 360])]]), detect_every=3)
        old, _ = tracker.update(self.scene())
        with patch.object(D, 'move_with_image', side_effect=AssertionError('Old target leaked')):
            new, _ = tracker.update(self.scene())
        self.assertTrue(tracker.last_status['target_switched'])
        self.assertGreater(D.apparent_pipe_width(new), D.apparent_pipe_width(old))

    def test_corrected_valid_pixel_boundary_enables_overflow(self):
        valid = np.ones((360, 640), bool)
        valid[:, :40] = False
        box = [35, 100, 300, 300]
        self.assertTrue(touches_boundary(box, (640, 360), valid))
        ordinary = search_region(box, (640, 360), False)
        expanded = search_region(box, (640, 360), True)
        self.assertLess(expanded[1], ordinary[1])
        self.assertEqual(expanded[0], 0)

    def test_roi_does_not_rescale_720p_pipe_width(self):
        frame = cv2.resize(self.scene(), (1280, 720))
        tracker = YoloRedGateTracker(FakeDetector([[door([880, 80, 1090, 720])]]))
        selected, _ = tracker.update(frame)
        self.assertIsNotNone(selected)
        geometry = D.original_geometry(frame, selected)
        self.assertGreater(geometry['pipe_width_px'], 20)
        self.assertLess(geometry['pipe_width_px'], 40)


if __name__ == '__main__':
    unittest.main()
