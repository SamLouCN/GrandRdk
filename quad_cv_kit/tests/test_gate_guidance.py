"""Synthetic known-pose, inferred edge intervals, boundary direction and CV input tests."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.gate_guidance import (enhance_cv_contrast, complete_gate_edges, estimate_alignment,
                               build_gate_guidance, draw_gate_guidance)
from src.yolo_red_gate import YoloRedGateTracker, boundary_sides


class GuidanceTests(unittest.TestCase):
    matrix = np.array([[900., 0, 640], [0, 900, 360], [0, 0, 1]])

    def geometry(self):
        corners = np.array([[400, 200], [800, 200], [800, 500], [400, 500]], float)
        edges = [[a+(b-a)*.12, a+(b-a)*.88]
                 for a, b in zip(corners, np.roll(corners, -1, axis=0))]
        return dict(segments=np.asarray(edges).tolist(), observation='detected')

    def status(self):
        return dict(target_bbox=[385, 185, 815, 515], target_clipped=False,
                    yolo_age_frames=0, target_boundary_sides=[])

    def test_contrast_preserves_black_validity_input_and_hue(self):
        hsv = np.zeros((40, 100, 3), np.uint8)
        hsv[:, :, 0] = 170
        hsv[:, :, 1] = 150
        hsv[:, :, 2] = np.tile(np.linspace(50, 180, 100).astype(np.uint8), (40, 1))
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        frame[:, :10] = 0
        original = frame.copy()
        enhanced = enhance_cv_contrast(frame)
        before = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        after = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        self.assertGreater(after[:, 10:, 2].std(), before[:, 10:, 2].std())
        self.assertLess(np.abs(after[:, 10:, 0].astype(float)-before[:, 10:, 0]).max(), 2)
        np.testing.assert_array_equal(enhanced[:, :10], frame[:, :10])
        np.testing.assert_array_equal(frame, original)

    def test_yolo_gets_clean_frame_cv_gets_contrast_frame(self):
        class Detector:
            target_ids = {0}
            def detect_boxes(inner, frame):
                self.assertIs(frame, original)
                return [dict(class_id=0, score=.9, bbox=[100, 100, 500, 300])]
        original = np.tile(np.arange(640, dtype=np.uint16)%150+50, (360, 1))
        original = np.repeat(original[:, :, None], 3, axis=2).astype(np.uint8)
        tracker = YoloRedGateTracker(Detector())
        with patch.object(tracker.cv, 'update', return_value=(None, [])) as update:
            tracker.update(original)
        self.assertFalse(np.array_equal(update.call_args.args[0], original))

    def test_extensions_are_only_missing_intervals(self):
        completed, _ = complete_gate_edges(self.geometry(), self.status()['target_bbox'], (1280, 720))
        self.assertEqual(len(completed['segments']), 8)
        np.testing.assert_allclose(completed['corners'], [[400, 200], [800, 200], [800, 500], [400, 500]])
        guidance = dict(mode='pose-unavailable', reason='test', completion=completed, alignment=None)
        frame = np.zeros((720, 1280, 3), np.uint8)
        drawn = draw_gate_guidance(frame, guidance, self.matrix)
        self.assertEqual(drawn[200, 410].tolist(), [150, 255, 150])
        self.assertEqual(drawn[200, 600].tolist(), [0, 0, 0])

    def test_missing_edge_or_unbounded_extrapolation_is_rejected(self):
        geometry = self.geometry()
        geometry['segments'] = geometry['segments'][:3]
        self.assertIsNone(complete_gate_edges(geometry, self.status()['target_bbox'], (1280, 720))[0])
        geometry = self.geometry()
        geometry['segments'][0] = [[590, 200], [620, 200]]
        self.assertIsNone(complete_gate_edges(geometry, self.status()['target_bbox'], (1280, 720))[0])

    def test_clipped_direction_and_conflicting_sides(self):
        status = dict(self.status(), target_clipped=True, target_boundary_sides=['left', 'up'])
        result = build_gate_guidance(status, self.geometry(), self.matrix, (1280, 720))
        np.testing.assert_allclose(result['direction_xy'], [-2**-.5, -2**-.5])
        self.assertIsNone(result['completion'])
        self.assertIsNone(result['alignment'])
        status['target_boundary_sides'] = ['left', 'right']
        self.assertIsNone(build_gate_guidance(status, None, self.matrix, (1280, 720))['direction_xy'])

    def test_yolo_gap_does_not_emit_direction_or_pose(self):
        status = dict(self.status(), yolo_age_frames=1)
        result = build_gate_guidance(status, self.geometry(), self.matrix, (1280, 720))
        self.assertEqual(result['mode'], 'yolo-gap')
        self.assertIsNone(result['completion'])

    def test_internal_valid_boundary_direction(self):
        valid = np.ones((720, 1280), bool)
        valid[:, :100] = False
        self.assertEqual(boundary_sides([80, 200, 500, 500], (1280, 720), valid), ['left'])

    def pose(self, angles=(12, -20, 8), translation=(.25, -.12, 2.5)):
        x, y, z = np.radians(angles)
        rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
        ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
        rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
        rotation = rz@ry@rx
        object_points = np.array([[-.35, -.25, 0], [.35, -.25, 0], [.35, .25, 0], [-.35, .25, 0]])
        corners = cv2.projectPoints(object_points, cv2.Rodrigues(rotation)[0],
                                    np.asarray(translation, float), self.matrix, np.zeros(5))[0].reshape(4, 2)
        return corners, rotation

    def test_known_pose_angles_and_normal_axis_translation(self):
        translation = np.array([.25, -.12, 2.5])
        corners, rotation = self.pose(translation=translation)
        pose, reason = estimate_alignment(corners, self.matrix)
        self.assertIsNotNone(pose, reason)
        np.testing.assert_allclose(pose['rotation_xyz_deg'], [12, -20, 8], atol=1e-5)
        movement = np.asarray(pose['alignment_translation_camera_m'])
        np.testing.assert_allclose(translation-movement, (translation@rotation[:, 2])*rotation[:, 2], atol=1e-6)
        np.testing.assert_allclose(pose['translation_pixel_equivalent_xyz'], 900*movement/2.5, atol=1e-5)
        # After translating then rotating, the center lies on the new forward axis.
        aligned = rotation.T@(translation-movement)
        np.testing.assert_allclose(aligned[:2], [0, 0], atol=1e-6)

    def test_frontal_centered_pose_zero_adjustment(self):
        corners, _ = self.pose(angles=(0, 0, 0), translation=(0, 0, 2.5))
        pose, reason = estimate_alignment(corners, self.matrix)
        self.assertIsNotNone(pose, reason)
        np.testing.assert_allclose(pose['rotation_xyz_deg'], [0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(pose['translation_pixel_equivalent_xyz'], [0, 0, 0], atol=1e-6)

    def test_bad_rectangle_fit_has_no_pose(self):
        corners, _ = self.pose()
        corners[2] += [150, 40]
        self.assertIsNone(estimate_alignment(corners, self.matrix)[0])


if __name__ == '__main__':
    unittest.main()
