"""Regression: background rods must not pull the target's centerlines."""
import sys
from pathlib import Path
import unittest
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import quad_cv_det as Q
from src.pole_lines import verify_quad


class PoleTests(unittest.TestCase):
    def scene(self, scale=1):
        frame = np.full((480, 640, 3), (110, 90, 50), np.uint8)
        corners = np.array([[130, 110], [510, 80], [515, 360], [125, 380]], float)
        cv2.polylines(frame, [corners.astype(np.int32)], True, (40, 70, 220), 10)
        # Background gate overlaps the broad original side bands.
        cv2.rectangle(frame, (205, 180), (440, 440), (40, 60, 160), 7)
        for y in range(150, 280, 9):
            cv2.circle(frame, (165 + (y % 5), y), 3, (40, 60, 200), -1)
        if scale != 1:
            frame = cv2.resize(frame, None, fx=scale, fy=scale)
        return frame, corners*scale, np.array([115, 65, 530, 395])*scale

    def test_background_poles_do_not_pull_target_lines(self):
        for scale in (1, 2):
            frame, truth, bbox = self.scene(scale)
            result = Q.detect(frame, bbox)
            self.assertEqual(result['lvl'], 4)
            error = np.linalg.norm(np.asarray(result['corners'])-truth, axis=1)
            self.assertLess(float(error.max()), 5*scale)
            self.assertTrue(result['diag']['polygon_supported'])

    def test_bbox_jitter_does_not_rotate_real_poles(self):
        frame, truth, bbox = self.scene()
        for delta in (-3, 0, 3):
            result = Q.detect(frame, bbox + [delta, -delta, delta, -delta])
            self.assertEqual(result['lvl'], 4)
            self.assertLess(float(np.linalg.norm(np.asarray(result['corners'])-truth, axis=1).max()), 5)

    def test_reject_unsupported_and_extrapolated_polygons(self):
        mask = np.zeros((480, 640), np.uint8)
        corners = [[130, 110], [510, 80], [515, 360], [125, 380]]
        cv2.polylines(mask, [np.asarray(corners, np.int32)], True, 255, 8)
        valid, _ = verify_quad(mask, corners, [115, 65, 530, 395], 5)
        self.assertTrue(valid)
        for bad in ([[0, 110], [510, 80], [515, 360], [125, 380]],
                    [[130, 110], [510, 80], [515, 280], [125, 280]]):
            valid, _ = verify_quad(mask, bad, [115, 65, 530, 395], 5)
            self.assertFalse(valid)


if __name__ == '__main__':
    unittest.main()
