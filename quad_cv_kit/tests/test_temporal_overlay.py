"""Real optical-flow checks for bounded and labeled video stabilization."""
import copy
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.temporal_overlay import TemporalOverlay
from src.quad_geom import edge_metrics


class OverlayTests(unittest.TestCase):
    def setUp(self):
        self.base = np.random.default_rng(11).integers(20, 45, (480, 640, 3), dtype=np.uint8)
        self.corners = np.array([[130, 110], [510, 90], [515, 360], [125, 380]], float)
        cv2.polylines(self.base, [self.corners.astype(np.int32)], True, (30, 50, 220), 12)
        for a, b in zip(self.corners, np.roll(self.corners, -1, axis=0)):
            for t in np.linspace(.1, .9, 9):
                p = tuple(np.rint(a + t*(b-a)).astype(int))
                cv2.circle(self.base, p, 2, (200, 220, 240), -1)
        self.tracker = TemporalOverlay(fps=30)

    def frame(self, dx=0):
        return cv2.warpAffine(self.base, np.array([[1, 0, dx], [0, 1, 0]], float), (640, 480))

    def detection(self, dx=0, with_quad=True):
        corners = (self.corners + [dx, 0]).tolist()
        d = dict(label='door', class_id=0, score=.9, bbox=[115+dx, 75, 530+dx, 395], selected=True)
        if with_quad:
            d['quad'] = dict(lvl=4, geometry_valid=True, corners=corners,
                             edges=edge_metrics(corners), fields={'psi': .2}, diag={})
        return d

    def test_first_frame_immediate_and_raw_observation_unchanged(self):
        observed = [self.detection()]
        before = copy.deepcopy(observed)
        result = self.tracker.update(self.frame(), observed)
        self.assertTrue(result[0]['observed'])
        self.assertEqual(result[0]['quad']['corners'], before[0]['quad']['corners'])
        self.assertEqual(observed, before)

    def test_yolo_gap_uses_current_motion_instead_of_freezing(self):
        self.tracker.update(self.frame(), [self.detection()])
        result = self.tracker.update(self.frame(3), [])
        self.assertEqual(len(result), 1)
        self.assertFalse(result[0]['observed'])
        self.assertEqual(result[0]['bbox_source'], 'optical-flow')
        self.assertAlmostEqual(result[0]['bbox'][0], 118, delta=.6)
        self.assertEqual(result[0]['quad']['observation'], 'tracked')
        self.assertFalse(result[0]['quad']['geometry_valid'])
        self.assertNotIn('psi', result[0]['quad']['fields'])

    def test_hold_expires_even_with_successful_optical_flow(self):
        self.tracker.update(self.frame(), [self.detection()])
        for i in range(1, 7):
            self.assertTrue(self.tracker.update(self.frame(i*2), []))
        self.assertEqual(self.tracker.update(self.frame(14), []), [])
        self.assertEqual(self.tracker.update(self.frame(16), []), [])
        result = self.tracker.update(self.frame(18), [self.detection(18)])
        self.assertTrue(result[0]['observed'])
        self.assertEqual(result[0]['quad']['observation'], 'detected')
        self.assertEqual(result[0]['tracking']['bbox_age_frames'], 0)

    def test_cv_gap_uses_red_support_and_expires_independently(self):
        self.tracker.update(self.frame(), [self.detection()])
        for i in range(1, 7):
            result = self.tracker.update(self.frame(i*2), [self.detection(i*2, False)])
            self.assertEqual(result[0]['quad']['observation'], 'tracked')
        result = self.tracker.update(self.frame(14), [self.detection(14, False)])
        self.assertTrue(result[0]['observed'])
        self.assertNotIn('quad', result[0])

    def test_scene_cut_never_reuses_old_box_or_quad(self):
        self.tracker.update(self.frame(), [self.detection()])
        self.assertEqual(self.tracker.update(np.zeros_like(self.base), []), [])
        new = self.detection()
        new['bbox'] = [10, 10, 80, 80]
        new.pop('quad')
        result = self.tracker.update(self.frame(), [new])
        self.assertEqual(result[0]['bbox'], new['bbox'])
        self.assertNotIn('quad', result[0])

    def test_smoothing_reduces_jitter_and_measurements_match_display(self):
        self.tracker.update(self.frame(), [self.detection()])
        d = self.detection()
        d['bbox'] = (np.array(d['bbox']) + [6, 0, 6, 0]).tolist()
        d['quad']['corners'] = (np.array(d['quad']['corners']) + [3, 0]).tolist()
        result = self.tracker.update(self.frame(), [d])[0]
        self.assertLess(result['bbox'][0], d['bbox'][0])
        self.assertEqual(result['quad']['observation'], 'smoothed')
        self.assertEqual(result['quad']['edges'], edge_metrics(result['quad']['corners']))


if __name__ == '__main__':
    unittest.main()
