"""Geometry and pipeline contracts, without requiring torch or model inference."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.quad_geom import edge_metrics
from src.yolo_quad import YoloQuadDetector, detect_in_box, draw_detections
from test_quad_cv import make_door_img


class Tensor:
    def __init__(self, values):
        self.values = values

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class Boxes:
    def __init__(self, bboxes, scores, ids):
        self.xyxy, self.conf, self.cls = Tensor(bboxes), Tensor(scores), Tensor(ids)
        self.size = len(bboxes)

    def __len__(self):
        return self.size


class PipelineTests(unittest.TestCase):
    def test_boxes_only_does_not_run_legacy_cv(self):
        frame = np.zeros((720, 1280, 3), np.uint8)
        boxes = Boxes([[100, 150, 900, 650]], [.9], [0])
        model = SimpleNamespace(names={0: 'door'},
                                predict=Mock(return_value=[SimpleNamespace(boxes=boxes)]))
        detector = YoloQuadDetector(model=model)
        with patch('src.yolo_quad.detect_in_box', side_effect=AssertionError('Unexpected CV')):
            detections = detector.detect_boxes(frame)
        self.assertEqual(detections[0]['bbox'], [100, 150, 900, 650])
        self.assertNotIn('quad', detections[0])
        self.assertIs(model.predict.call_args.kwargs['source'], frame)

    def test_rectangle_lengths_and_angles(self):
        edges = edge_metrics([(10, 10), (110, 10), (110, 60), (10, 60)])
        for name in ('top', 'bottom'):
            self.assertEqual(edges[name], dict(length_px=100.0, angle_deg=0.0))
        for name in ('left', 'right'):
            self.assertEqual(edges[name], dict(length_px=50.0, angle_deg=90.0))

    def test_tilt_sign_and_endpoint_invariance(self):
        quad = [(0, 10), (10, 0), (20, 10), (10, 20)]
        edges = edge_metrics(quad)
        self.assertAlmostEqual(edges['top']['length_px'], math.sqrt(200))
        self.assertAlmostEqual(edges['top']['angle_deg'], 45)
        self.assertAlmostEqual(edges['right']['angle_deg'], -45)
        reversed_edges = edge_metrics(list(reversed(quad)))
        self.assertEqual(edges['top'], reversed_edges['bottom'])
        self.assertEqual(edges['right'], reversed_edges['right'])

    def test_degenerate_and_missing_corners(self):
        self.assertEqual(edge_metrics(None), {})
        self.assertEqual(edge_metrics([(0, 0)] * 4), {})
        self.assertEqual(edge_metrics([(0, 0), None, (1, 1), (0, 1)]), {})

    def test_yolo_coordinates_and_clean_frame(self):
        frame, _, bbox = make_door_img(yaw=12, z=2)
        clean = frame.copy()
        boxes = Boxes([list(bbox), [0, 0, 10, 10]], [0.95, 0.80], [0, 1])
        model = SimpleNamespace(names={0: 'door', 1: 'red-ball'},
                                predict=Mock(return_value=[SimpleNamespace(boxes=boxes)]))
        detector = YoloQuadDetector(model=model)
        result = detector.detect(frame)
        self.assertEqual(result[0]['quad']['lvl'], 4)
        self.assertTrue(result[0]['quad']['geometry_valid'])
        self.assertEqual(len(result[0]['quad']['edges']), 4)
        self.assertNotIn('quad', result[1])
        self.assertEqual(result[0]['bbox'], list(bbox))
        self.assertIs(model.predict.call_args.kwargs['source'], frame)
        vis = draw_detections(frame, result)
        np.testing.assert_array_equal(frame, clean)
        self.assertFalse(np.array_equal(vis, clean))

    def test_invalid_structure_has_metrics_but_no_usable_psi(self):
        frame, _, bbox = make_door_img(yaw=0, z=2)
        quad = detect_in_box(frame, bbox, opts={'aspect': 5.0})
        self.assertEqual(quad['lvl'], 3)
        self.assertFalse(quad['geometry_valid'])
        self.assertEqual(len(quad['edges']), 4)
        self.assertIsNone(quad['fields']['psi'])
        self.assertIsNone(quad['fields']['psi_deg'])

    def test_empty_yolo_results(self):
        model = SimpleNamespace(names={0: 'door'}, predict=Mock(
            return_value=[SimpleNamespace(boxes=Boxes([], [], []))]))
        self.assertEqual(YoloQuadDetector(model=model).detect(np.zeros((32, 32, 3), np.uint8)), [])

    def test_class_selection_and_mismatch(self):
        model = SimpleNamespace(names={0: 'target', 1: 'red-ball'})
        with self.assertRaisesRegex(ValueError, 'No target class'):
            YoloQuadDetector(model=model)
        self.assertEqual(YoloQuadDetector(model=model, classes=['0']).target_ids, {0})

    def model_for(self, bboxes, scores, ids=None):
        ids = ids or [0] * len(bboxes)
        return SimpleNamespace(names={0: 'door'}, predict=Mock(
            return_value=[SimpleNamespace(boxes=Boxes(bboxes, scores, ids))]))

    def test_corner_checkpoint_is_rejected(self):
        model = SimpleNamespace(names={0: 'upper-left', 1: 'upper-right',
                                      2: 'lower-left', 3: 'lower-right'})
        with self.assertRaisesRegex(ValueError, 'whole-door'):
            YoloQuadDetector(model=model)
        with self.assertRaisesRegex(ValueError, 'Corner checkpoints'):
            YoloQuadDetector(model=model, classes=['0'])

    def test_largest_area_wins_over_confidence(self):
        frame, _, bbox = make_door_img(yaw=12, z=2)
        model = self.model_for([list(bbox), [20, 20, 40, 40]], [0.5, 0.99])
        result = YoloQuadDetector(model=model).detect(frame)
        self.assertTrue(result[0]['selected'])
        self.assertEqual(result[0]['quad']['lvl'], 4)
        self.assertNotIn('selected', result[1])
        self.assertNotIn('quad', result[1])
        self.assertEqual(len(result), 2)

    def test_cv_runs_on_first_and_every_detected_frame(self):
        frame, _, bbox = make_door_img(yaw=12, z=2)
        detector = YoloQuadDetector(model=self.model_for([list(bbox)], [0.9]))
        with patch('src.yolo_quad.detect_in_box', wraps=detect_in_box) as cv:
            for index in range(3):
                result = detector.detect(frame)
                self.assertEqual(result[0]['quad']['lvl'], 4)
                self.assertTrue(detector.last_status['cv_enabled'])
                self.assertEqual(cv.call_count, index + 1)

    def test_target_loss_and_return_have_no_stale_geometry_or_wait(self):
        frame, _, bbox = make_door_img(yaw=0, z=2)
        model = self.model_for([list(bbox)], [0.9])
        detector = YoloQuadDetector(model=model)
        self.assertIn('quad', detector.detect(frame)[0])
        model.predict.return_value = [SimpleNamespace(boxes=Boxes([], [], []))]
        self.assertEqual(detector.detect(frame), [])
        self.assertEqual(detector.last_status['state'], 'searching')
        self.assertFalse(detector.last_status['cv_enabled'])
        model.predict.return_value = [SimpleNamespace(boxes=Boxes([list(bbox)], [0.9], [0]))]
        self.assertIn('quad', detector.detect(frame)[0])

    def test_largest_target_switch_runs_cv_immediately(self):
        frame, _, bbox = make_door_img(yaw=0, z=2)
        model = self.model_for([list(bbox)], [0.9])
        detector = YoloQuadDetector(model=model)
        detector.detect(frame)
        other = [0, 0, 600, 450]
        model.predict.return_value = [SimpleNamespace(boxes=Boxes(
            [list(bbox), other], [0.99, 0.3], [0, 0]))]
        with patch('src.yolo_quad.detect_in_box', wraps=detect_in_box) as cv:
            result = detector.detect(frame)
            self.assertTrue(result[1]['selected'])
            self.assertIn('quad', result[1])
            self.assertNotIn('quad', result[0])
            self.assertEqual(cv.call_count, 1)
            self.assertEqual(cv.call_args.args[1], other)

if __name__ == '__main__':
    unittest.main()


