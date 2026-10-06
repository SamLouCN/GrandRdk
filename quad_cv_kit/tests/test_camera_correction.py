"""Validate refraction coordinates and corrected-only paired-video inference."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.camera_correction import (FlatPortCorrector, create_corrector, load_camera_params,
                                   prepare_frame, line_straightness, DEFAULT_PARAMS_PATH,
                                   adapt_camera_params)
from src.camera_correction import correct_image
from src.yolo_quad import GateQuadProcessor, project_detection_geometry, draw_detections
from src.media_demo import run_video


class CorrectionTests(unittest.TestCase):
    def test_default_uses_root_document(self):
        camera = json.loads(DEFAULT_PARAMS_PATH.read_text(encoding='utf-8-sig'))
        self.assertEqual(load_camera_params(), camera)
        corrector = create_corrector()
        self.assertEqual((corrector.w, corrector.h), (640, 480))
        self.assertAlmostEqual(corrector.f_out, 474.70, places=2)
        self.assertEqual(corrector.output_matrix[0, 0], corrector.f_out)

    def test_maps_match_annotation_mapping_and_cache(self):
        corrector = create_corrector()
        mx, my = corrector.maps(0.98)
        points = np.array([[320, 240], [100, 100], [500, 350], [639, 479]])
        mapped = corrector.corrected_to_raw(points, 0.98)
        for p, q in zip(points, mapped):
            if np.isfinite(q).all():
                np.testing.assert_allclose(q, [mx[p[1], p[0]], my[p[1], p[0]]], atol=3e-5)
            else:
                self.assertEqual([mx[p[1], p[0]], my[p[1], p[0]]], [-1, -1])
        np.testing.assert_allclose(mapped[0], [320, 240], atol=1e-8)
        self.assertIs(corrector.maps(0.98)[0], mx)
        self.assertIsNot(corrector.maps(None)[0], mx)

    def test_air_no_lens_distortion_is_identity(self):
        corrector = FlatPortCorrector(640, 480, [[318.8, 0, 320], [0, 318.8, 240], [0, 0, 1]],
                                       n_water=1.0)
        points = np.array([[10, 20], [320, 240], [630, 450]], dtype=float)
        np.testing.assert_allclose(corrector.corrected_to_raw(points), points, atol=1e-4)
        frame = np.random.default_rng(42).integers(0, 256, (480, 640, 3), dtype=np.uint8)
        np.testing.assert_array_equal(corrector.undistort(frame), frame)

    def test_nonzero_lens_coefficients_are_applied(self):
        corrector = FlatPortCorrector(640, 480, [[320, 0, 320], [0, 320, 240], [0, 0, 1]],
                                       distortion=[0.1, 0, 0, 0, 0], n_water=1.0)
        # x=0.5 normalized -> x_distorted=0.5125 -> 484px
        np.testing.assert_allclose(corrector.corrected_to_raw([[480, 240]]), [[484, 240]], atol=1e-4)

    def test_resize_and_wrong_aspect(self):
        large = np.zeros((960, 1280, 3), np.uint8)
        self.assertEqual(prepare_frame(large).shape, (480, 640, 3))
        frame = np.zeros((480, 640, 3), np.uint8)
        self.assertIs(prepare_frame(frame), frame)
        with self.assertRaises(ValueError):
            prepare_frame(np.zeros((720, 1280, 3), np.uint8))

    def test_invalid_parameters_and_distance(self):
        with self.assertRaises(ValueError):
            FlatPortCorrector(640, 480, np.zeros((3, 3)))
        for distance in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                create_corrector().maps(distance)

    def test_direct_image_correction_keeps_resolution_and_caches_intrinsics(self):
        corrector = create_corrector()
        frame = np.zeros((720, 1280, 3), np.uint8)
        self.assertEqual(correct_image(frame, corrector).shape, frame.shape)
        adapted = corrector.for_frame(frame)
        self.assertIs(adapted, corrector.for_frame(frame))
        np.testing.assert_array_equal(adapted.dist, corrector.dist)
        self.assertEqual((adapted.fx, adapted.fy, adapted.cx, adapted.cy),
                         (637.6, 637.6, 640, 360))

    def test_widescreen_crop_preserves_focal_scale_and_shifts_principal_point(self):
        camera, info = adapt_camera_params(load_camera_params(), 1280, 720)
        self.assertEqual((camera['width'], camera['height']), (1280, 720))
        np.testing.assert_allclose(camera['matrix'], [[637.6, 0, 640],
                                                     [0, 637.6, 360], [0, 0, 1]])
        self.assertTrue(info['aspect_changed'])
        raw = np.zeros((720, 1280, 3), np.uint8)
        corrector = create_corrector(camera)
        self.assertEqual(corrector.undistort(raw).shape, (720, 1280, 3))
        np.testing.assert_allclose(corrector.corrected_to_raw([[640, 360]]), [[640, 360]])

    def test_recording_modes_and_small_input(self):
        original = load_camera_params()
        camera, _ = adapt_camera_params(original, 1280, 960)
        np.testing.assert_allclose(camera['matrix'], [[637.6, 0, 640], [0, 637.6, 480], [0, 0, 1]])
        resized, _ = adapt_camera_params(original, 1280, 720, 'resize')
        self.assertAlmostEqual(resized['matrix'][1][1], 318.8 * 1.5)
        self.assertAlmostEqual(resized['matrix'][1][2], 360)
        small, _ = adapt_camera_params(original, 320, 240)
        self.assertEqual((small['width'], small['height']), (320, 240))
        self.assertAlmostEqual(small['matrix'][0][0], 159.4)
        odd, _ = adapt_camera_params(original, 323, 241)
        self.assertEqual(prepare_frame(np.zeros((241, 323, 3), np.uint8),
                                      odd['width'], odd['height']).shape, (241, 323, 3))
        with self.assertRaises(ValueError):
            adapt_camera_params(original, 1280, 720, 'strict')

    def test_raw_annotation_curves_preserve_original_detections(self):
        detector = GateQuadProcessor()
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(frame, (120, 100), (520, 380), (20, 30, 230), 13)
        dets = detector.process(frame, [dict(label='door', score=0.9, bbox=[115, 95, 525, 385])])
        original = copy.deepcopy(dets)
        corrector = create_corrector()
        geometry = project_detection_geometry(dets, corrector.corrected_to_raw)
        self.assertEqual(dets, original)
        self.assertEqual(len(geometry), len(dets))
        self.assertEqual(len(geometry[0]['bbox_paths'][0]), 33)
        self.assertGreater(line_straightness(geometry[0]['bbox_paths'][0])[0], 1)
        self.assertEqual(geometry[0]['edge_labels'][0]['metric'], dets[0]['quad']['edges']['top'])
        vis = draw_detections(frame, dets, geometry)
        self.assertFalse(np.array_equal(frame, vis))

    def test_image_straightness_diagnostic_and_missing_evidence(self):
        corrector = create_corrector()
        frame = np.zeros((480, 640, 3), np.uint8)
        self.assertEqual(corrector.assess_distortion(frame, None)['status'], 'insufficient-evidence')
        cv2.rectangle(frame, (120, 100), (520, 380), (20, 30, 230), 5)
        diag = corrector.assess_distortion(frame, [[120, 100], [520, 100], [520, 380], [120, 380]])
        self.assertFalse(diag['parameter_estimation'])
        self.assertEqual(len(diag['edges']), 4)
        self.assertEqual(diag['status'], 'curvature-reduced')
        self.assertGreater(diag['before_relative'], diag['after_relative'])

    def test_invalid_projection_regions_do_not_join(self):
        detections = [dict(label='door', score=0.9, bbox=[-1000, -1000, 1000, 1000])]
        geom = project_detection_geometry(detections, create_corrector().corrected_to_raw)
        # JSON must not contain NaN even where projection is undefined.
        json.dumps(geom, allow_nan=False)
        draw_detections(np.zeros((480, 640, 3), np.uint8), detections, geom)

    def test_two_videos_share_one_corrected_inference_per_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source, output = folder / 'input.avi', folder / 'out'
            output.mkdir()
            frame = np.zeros((720, 1280, 3), np.uint8)
            cv2.rectangle(frame, (240, 150), (1040, 650), (20, 30, 230), 26)
            writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'MJPG'), 30, (1280, 720))
            self.assertTrue(writer.isOpened())
            for _ in range(3):
                writer.write(frame)
            writer.release()
            reader = cv2.VideoCapture(str(source))
            _, decoded = reader.read()
            reader.release()
            camera, _ = adapt_camera_params(load_camera_params(), 1280, 720)
            expected = create_corrector(camera).undistort(decoded)
            processor = GateQuadProcessor()
            observed = []
            def infer(image, *args):
                observed.append(image.copy())
                return processor.process(image, [dict(label='door', score=0.9, bbox=[200, 120, 1080, 680])])
            with patch('src.media_demo._setup', return_value=(processor, None)), \
                    patch('src.media_demo._detect', side_effect=infer) as detect:
                run_video([str(source), '--out', str(output), '--bbox-source', 'red'])
                self.assertEqual(detect.call_count, 3)
            np.testing.assert_array_equal(observed[0], expected)
            self.assertFalse(np.array_equal(observed[0], decoded))
            records = [json.loads(line) for line in (output / 'rows.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual(len(records), 3)
            for rec in records:
                self.assertEqual(rec['coordinate_space'], 'corrected')
                self.assertEqual(len(rec['detections']), len(rec['raw_geometry']))
                self.assertEqual(len(rec['detections'][0]['quad']['corners']), 4)
                self.assertEqual(len(rec['raw_geometry'][0]['quad_paths']), 4)
                for edge in rec['raw_geometry'][0]['edge_labels']:
                    self.assertEqual(edge['metric'], rec['detections'][0]['quad']['edges'][edge['name']])
                self.assertEqual(len(rec['display_detections']), len(rec['display_raw_geometry']))
                for displayed, geometry in zip(rec['display_detections'], rec['display_raw_geometry']):
                    for edge in geometry['edge_labels']:
                        self.assertEqual(edge['metric'], displayed['quad']['edges'][edge['name']])
                self.assertEqual(rec['temporal_status']['state'], 'observed')
            for name in ('before_correction.mp4', 'after_correction.mp4'):
                cap = cv2.VideoCapture(str(output / name))
                count = 0
                while True:
                    ok, image = cap.read()
                    if not ok:
                        break
                    count += 1
                    self.assertEqual(image.shape, (720, 1280, 3))
                cap.release()
                self.assertEqual(count, 3)


if __name__ == '__main__':
    unittest.main()
