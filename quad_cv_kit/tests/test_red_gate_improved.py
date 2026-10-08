"""Nearest-gate regression: partial foreground, source coordinates, tracking expiry."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import detect_red_gate as D
from src.camera_correction import adapt_camera_params, create_corrector, load_camera_params


class RedGateTests(unittest.TestCase):
    def scene(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        # Far complete gate; foreground left upright and bottom bar are out of view.
        cv2.rectangle(frame, (220, 130), (390, 275), (50, 60, 230), 5)
        cv2.line(frame, (5, 55), (535, 55), (50, 60, 230), 14)
        cv2.line(frame, (535, 55), (535, 359), (50, 60, 230), 14)
        cv2.circle(frame, (535, 55), 9, (220, 220, 220), -1)
        return frame

    def nearest(self, frame):
        candidates, _, _ = D.detect(frame)
        result = D.select_nearest(candidates)
        self.assertIsNotNone(result)
        return result

    def test_partial_near_gate_beats_complete_far_gate(self):
        result = self.nearest(self.scene())
        self.assertFalse(result['complete'])
        self.assertGreater(result['apparent_width'], 10)
        points = np.concatenate(result['segments'])
        self.assertGreater(float(points[:, 0].max()), 520)
        self.assertLess(float(points[:, 1].min()), 70)

    def test_single_near_pipe_beats_complete_far_gate(self):
        frame = self.scene()
        frame[40:360, 520:550] = (120, 90, 40)
        result = self.nearest(frame)
        self.assertEqual(len(result['segments']), 1)
        self.assertGreater(result['apparent_width'], 10)

    def test_720p_geometry_and_green_edges_are_in_source_pixels(self):
        frame = cv2.resize(self.scene(), (1280, 720))
        result = self.nearest(frame)
        geometry = D.original_geometry(frame, result)
        self.assertGreater(geometry['bbox'][2], 1040)
        self.assertAlmostEqual(geometry['pipe_width_px'], result['apparent_width']*2)
        annotated = D.draw_nearest(frame, result)
        self.assertEqual(annotated.shape, frame.shape)
        midpoint = np.mean(geometry['segments'][0], axis=0)
        x, y = np.rint(midpoint).astype(int)
        self.assertEqual(annotated[y, x].tolist(), [0, 255, 0])
        x1, _, x2, y2 = np.rint(geometry['bbox']).astype(int)
        x = (x1 + x2) // 2
        np.testing.assert_array_equal(annotated[y2, x], frame[y2, x])

    def test_pillarbox_offset_is_removed_from_4_by_3_coordinates(self):
        frame = cv2.resize(self.scene(), (480, 360))
        result = self.nearest(frame)
        geometry = D.original_geometry(frame, result)
        # 640x360 processing adds 80px of padding on each side of this image.
        self.assertGreater(geometry['bbox'][2], 390)
        self.assertLess(geometry['bbox'][2], 420)

    def test_tracking_cannot_extend_without_fresh_observation(self):
        frame = self.scene()
        candidate = self.nearest(frame)
        tracker = D.RedGateTracker(fps=10, detect_every=100, hold_seconds=.2)
        with patch.object(D, 'detect', side_effect=[([candidate], ([], []), None),
                                                   ([], ([], []), None)]), \
             patch.object(D, 'move_with_image', return_value=dict(candidate, tracked=True)):
            first, _ = tracker.update(frame)
            self.assertFalse(first['tracked'])
            for _ in range(2):
                followed, _ = tracker.update(frame)
                self.assertTrue(followed['tracked'])
            expired, _ = tracker.update(frame)
            self.assertIsNone(expired)
            self.assertEqual(tracker.last_status['observation'], 'missing')

    def test_uniform_water_produces_no_gate(self):
        frame = np.full((720, 1280, 3), (120, 90, 40), np.uint8)
        result, _ = D.RedGateTracker().update(frame)
        self.assertIsNone(result)

    def test_corrected_edge_maps_to_a_raw_curve_with_existing_camera(self):
        camera, _ = adapt_camera_params(load_camera_params(), 1280, 720)
        corrector = create_corrector(camera)
        geometry = dict(segments=[[[150, 170], [1100, 170]]], corners=[],
                        complete=False, observation='detected')
        projected = D.project_gate_geometry(geometry, corrector.corrected_to_raw, (1280, 720))
        self.assertEqual(len(projected['edge_paths']), 1)
        points = np.asarray(projected['edge_paths'][0])
        self.assertGreater(len(points), 50)
        expected = corrector.corrected_to_raw(np.column_stack([
            np.linspace(150, 1100, len(points)), np.full(len(points), 170)]))
        np.testing.assert_allclose(points, expected)
        direction = points[-1] - points[0]
        normal = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction)
        self.assertGreater(float(np.max(np.abs((points-points[0]) @ normal))), 1)
        frame = np.zeros((720, 1280, 3), np.uint8)
        drawn = D.draw_gate_geometry(frame, projected)
        x, y = np.rint(points[len(points)//2]).astype(int)
        self.assertEqual(drawn[y, x].tolist(), [0, 255, 0])

    def test_invalid_mapped_regions_do_not_create_connecting_edges(self):
        geometry = dict(segments=[[[20, 100], [300, 100]]], corners=[[160, 100]],
                        complete=False, observation='tracked')

        def mapper(points):
            result = np.asarray(points, float).copy()
            result[(result[:, 0] >= 140) & (result[:, 0] <= 180)] = np.nan
            return result

        projected = D.project_gate_geometry(geometry, mapper, (640, 360))
        self.assertEqual(len(projected['edge_paths']), 2)
        self.assertEqual(projected['corners'], [])
        drawn = D.draw_gate_geometry(np.zeros((360, 640, 3), np.uint8), projected)
        self.assertEqual(drawn[100, 160].tolist(), [0, 0, 0])

    def test_no_gate_maps_to_no_gate(self):
        self.assertIsNone(D.project_gate_geometry(None, lambda p: p, (1280, 720)))


if __name__ == '__main__':
    unittest.main()
