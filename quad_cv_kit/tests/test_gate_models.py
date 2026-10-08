"""White elbows, genuine four-side evidence and current-frame tracking regression."""
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from src import detect_red_gate as D
from src import gate_models as M


class GateModelTests(unittest.TestCase):
    def scene(self, gap=12, missing=None):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        corners = np.array([[140, 80], [480, 80], [480, 280], [140, 280]], float)
        for index, a in enumerate(corners):
            if index == missing:
                continue
            b = corners[(index+1) % 4]
            direction = (b-a)/np.linalg.norm(b-a)
            cv2.line(frame, tuple(np.rint(a+gap*direction).astype(int)),
                     tuple(np.rint(b-gap*direction).astype(int)), (50, 60, 230), 8)
        return frame, corners

    def complete(self, frame):
        candidates, _, _ = D.detect(frame)
        return [candidate for candidate in candidates if candidate.get('geometry_validated')]

    def test_elbow_gap_without_white_segmentation_still_forms_four_corners(self):
        frame, corners = self.scene()
        with patch.object(D, 'elbow_mask', return_value=np.zeros(frame.shape[:2], np.uint8)):
            models = self.complete(frame)
        self.assertTrue(models)
        np.testing.assert_allclose(models[0]['corners'], corners, atol=3)
        self.assertEqual(len(models[0]['side_support']), 4)
        self.assertIs(D.polygon_edges(models[0]), models[0])
        for line, segment in zip(models[0]['lines'], models[0]['segments']):
            self.assertLess(np.linalg.norm(np.diff(line['observed_segment'], axis=0)),
                            np.linalg.norm(np.diff(segment, axis=0)))

    def test_white_support_legs_do_not_extend_gate_bottom(self):
        frame, corners = self.scene()
        for x in (140, 480):
            cv2.line(frame, (x, 280), (x, 345), (235, 235, 235), 8)
            cv2.circle(frame, (x, 280), 12, (235, 235, 235), -1)
        models = self.complete(frame)
        self.assertTrue(models)
        np.testing.assert_allclose(models[0]['corners'], corners, atol=3)

    def test_missing_fourth_side_is_not_manufactured(self):
        frame, _ = self.scene(missing=2)
        for x in (140, 480):
            cv2.line(frame, (x, 280), (x, 345), (235, 235, 235), 8)
        self.assertFalse(self.complete(frame))

    def test_warm_white_leg_does_not_extend_partial_gate(self):
        frame, _ = self.scene(missing=1)
        cv2.line(frame, (140, 280), (140, 345), (160, 148, 160), 8)
        candidates, _, _ = D.detect(frame)
        nearest = D.select_nearest(candidates)
        self.assertIsNotNone(nearest)
        self.assertFalse(nearest['complete'])
        measured = [line.get('observed_segment', segment)
                    for line, segment in zip(nearest['lines'], nearest['segments'])]
        self.assertLess(np.concatenate(measured)[:, 1].max(), 292)

    def test_short_red_patch_on_white_foot_does_not_hide_near_partial_gate(self):
        frame, _ = self.scene(missing=1)
        cv2.rectangle(frame, (250, 130), (370, 230), (50, 60, 230), 3)
        cv2.line(frame, (480, 280), (480, 345), (160, 148, 160), 8)
        cv2.line(frame, (480, 296), (480, 315), (50, 60, 230), 3)
        candidates, _, _ = D.detect(frame)
        nearest = D.select_nearest(candidates)
        self.assertIsNotNone(nearest)
        self.assertFalse(nearest['complete'])
        self.assertGreater(nearest['apparent_width'], 6)
        measured = [line.get('observed_segment', segment)
                    for line, segment in zip(nearest['lines'], nearest['segments'])]
        self.assertLess(np.concatenate(measured)[:, 1].max(), 292)

    def test_excessive_end_gaps_are_rejected(self):
        frame, _ = self.scene(gap=55)
        self.assertFalse(self.complete(frame))

    def test_middle_only_background_support_is_rejected(self):
        frame, _ = self.scene()
        frame[65:96, 140:481] = (120, 90, 40)
        cv2.line(frame, (255, 80), (365, 80), (50, 60, 230), 8)
        self.assertFalse(self.complete(frame))

    def test_short_mid_rod_gap_is_recovered(self):
        frame, _ = self.scene()
        frame[65:96, 300:313] = (120, 90, 40)
        self.assertTrue(self.complete(frame))

    def test_faded_top_still_supports_measured_four_line_model(self):
        frame, corners = self.scene()
        cv2.line(frame, (152, 80), (468, 80), (130, 110, 120), 8)
        models = self.complete(frame)
        self.assertTrue(models)
        np.testing.assert_allclose(models[0]['corners'], corners, atol=3)

    def test_offset_overlapping_gates_keep_near_identity(self):
        frame = np.full((360, 640, 3), (145, 115, 55), np.uint8)
        cv2.rectangle(frame, (300, 95), (575, 270), (55, 65, 190), 6)
        cv2.rectangle(frame, (125, 55), (440, 305), (55, 65, 190), 12)
        candidates, _, _ = D.detect(frame)
        nearest = D.select_nearest(candidates)
        self.assertTrue(nearest['complete'])
        np.testing.assert_allclose(nearest['corners'],
                                   [[125, 55], [440, 55], [440, 305], [125, 305]], atol=8)

    def test_missing_near_side_does_not_borrow_distant_fourth_side(self):
        frame, _ = self.scene(missing=2)
        cv2.rectangle(frame, (250, 130), (370, 230), (50, 60, 230), 3)
        candidates, _, _ = D.detect(frame)
        nearest = D.select_nearest(candidates)
        self.assertIsNotNone(nearest)
        self.assertFalse(nearest['complete'])
        self.assertGreater(nearest['apparent_width'], 6)
        for model in candidates:
            if model.get('geometry_validated'):
                self.assertLess(np.ptp(np.array(model['corners'])[:, 0]), 150)

    def test_valid_mask_and_roi_reject_corner_outside_observation(self):
        frame, _ = self.scene()
        valid = np.ones(frame.shape[:2], bool)
        valid[65:95, 125:155] = False
        candidates, _, _ = D.detect(frame, valid_mask=valid)
        self.assertFalse(any(candidate.get('geometry_validated') for candidate in candidates))
        candidates, _, _ = D.detect(frame, search_bbox=[200, 60, 510, 310])
        self.assertFalse(any(candidate.get('geometry_validated') for candidate in candidates))

    def test_enhancement_cannot_create_red_far_from_clean_reference(self):
        reference = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        enhanced, _ = self.scene()
        mask, _ = M.combined_evidence(enhanced, reference)
        self.assertFalse(mask.any())
        candidates, _, _ = D.detect(enhanced, reference_frame=reference)
        self.assertFalse(candidates)

    def test_current_image_missing_side_downgrades_tracked_complete_model(self):
        frame, _ = self.scene()
        previous = self.complete(frame)[0]
        next_frame, _ = self.scene(missing=2)
        mask, _ = M.tube_evidence(next_frame)
        points = np.arange(20, dtype=np.float32).reshape(10, 1, 2)+100
        identity = np.array([[1., 0., 0.], [0., 1., 0.]])
        with patch.object(cv2, 'goodFeaturesToTrack', return_value=points), \
             patch.object(cv2, 'calcOpticalFlowPyrLK', return_value=(points, np.ones((10, 1)), np.zeros((10, 1)))), \
             patch.object(cv2, 'estimateAffinePartial2D', return_value=(identity, np.ones((10, 1)))):
            moved = D.move_with_image(previous, frame[:, :, 0], next_frame[:, :, 0], mask, next_frame)
        self.assertIsNotNone(moved)
        self.assertFalse(moved['complete'])
        self.assertEqual(len(moved['segments']), 3)
        self.assertEqual(len(moved['side_support']), 3)


if __name__ == '__main__':
    unittest.main()
