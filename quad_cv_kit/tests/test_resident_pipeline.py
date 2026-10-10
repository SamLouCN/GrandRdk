"""Production kernels: CPU-free core, connectivity, motion, target hold and pose."""
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opencl_host import HostRuntime
from src.opencl_backend import OpenCLBackend
from src.opencl_runtime import LocalMemory
from src.gpu_pipeline import ResidentGatePipeline


def scene(dx=0, dy=0, partial=False):
    frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    points = np.array([[170, 70], [470, 70], [470, 290], [170, 290]])+[dx, dy]
    for k in range(3 if partial else 4):
        cv2.line(frame, tuple(points[k]), tuple(points[(k+1)%4]), (50, 60, 230), 8)
    return frame


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Production kernel harness requires clang')
class ResidentPipelineTests(unittest.TestCase):
    def test_candidate_reports_measured_endpoints_separately_from_completed_sides(self):
        row = np.zeros(128, np.float32)
        row[0], row[2], row[3], row[7] = 1, 1, 1, 4
        row[16:28] = [1, 0, 50, 120, 280, 5, 1, 0, 120, 280, 5, 1]
        row[64:68] = [100, 50, 300, 50]
        candidate = ResidentGatePipeline.candidate(row)
        np.testing.assert_array_equal(candidate['segments'][0], [[100, 50], [300, 50]])
        np.testing.assert_array_equal(candidate['lines'][0]['observed_segment'], [[120, 50], [280, 50]])

    @classmethod
    def setUpClass(cls):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            cls.gpu = OpenCLBackend(quality='fast', blur_mode='pyramid')

    @classmethod
    def tearDownClass(cls):
        cls.gpu.close()

    def run_frame(self, pipeline, frame, boxes=None, valid=None):
        self.gpu.reset_stats()
        boxes = [dict(class_id=0, score=.9, bbox=[155, 55, 485, 305])] if boxes is None else boxes
        with self.gpu.frame_batch():
            return pipeline.update(frame, frame, boxes, {0}, valid)[0]

    def test_complete_gate_has_only_final_output_readback_and_no_cpu_algorithms(self):
        pipeline = ResidentGatePipeline(self.gpu)
        names = ('cvtColor', 'createCLAHE', 'createLineSegmentDetector', 'connectedComponentsWithStats',
                 'goodFeaturesToTrack', 'calcOpticalFlowPyrLK', 'estimateAffinePartial2D', 'solvePnPGeneric')
        from contextlib import ExitStack
        frame = scene()
        with ExitStack() as stack:
            for name in names:
                stack.enter_context(patch.object(cv2, name, side_effect=AssertionError('CPU '+name)))
            result = self.run_frame(pipeline, frame)
        self.assertIsNotNone(result)
        self.assertTrue(result['complete'])
        np.testing.assert_allclose(np.asarray(result['corners']), [[170, 70], [470, 70], [470, 290], [170, 290]], atol=3)
        self.assertEqual(self.gpu.runtime.download_bytes, 512+128+160)
        self.assertEqual(len([k for k in self.gpu.runtime.transfer_calls if k.startswith('download:')]), 3)
        self.assertEqual(pipeline.last_status['cv_profile']['intermediate_readbacks'], 0)
        self.assertEqual(pipeline.last_status['cv_profile']['timing_kind'], 'host-kernel-emulation')

    def test_motion_tracks_translation_and_yolo_gap_cannot_renew_detection(self):
        pipeline = ResidentGatePipeline(self.gpu, fps=30, detect_every=3, hold_seconds=.2)
        first = self.run_frame(pipeline, scene())
        second = self.run_frame(pipeline, scene(3, 2))
        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertTrue(second['tracked'])
        np.testing.assert_allclose(np.asarray(second['corners'])-np.asarray(first['corners']), np.tile([3, 2], (4, 1)), atol=.7)
        self.assertFalse(pipeline.last_status['detection_ran'])
        for _ in range(7):
            result = self.run_frame(pipeline, scene(3, 2), boxes=[])
            self.assertFalse(pipeline.last_status['detection_ran'])
        self.assertIsNone(result)
        self.assertIsNone(pipeline.last_status['target_bbox'])

    def test_partial_invalid_mask_and_nonred_texture(self):
        partial = self.run_frame(ResidentGatePipeline(self.gpu), scene(partial=True))
        self.assertIsNotNone(partial)
        self.assertFalse(partial['complete'])
        self.assertEqual(len(partial['lines']), 3)
        self.assertIsNone(self.run_frame(ResidentGatePipeline(self.gpu), scene(), valid=np.zeros((360, 640), bool)))
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (170, 70), (470, 290), (190, 90, 40), 8)
        self.assertIsNone(self.run_frame(ResidentGatePipeline(self.gpu), frame))

    def test_weak_red_white_elbows_and_nested_gate_keep_outer_geometry(self):
        weak = scene()
        weak[np.all(weak == (50, 60, 230), axis=2)] = (50, 60, 95)
        elbows = scene()
        for x, y in ((170, 70), (470, 70), (470, 290), (170, 290)):
            cv2.rectangle(elbows, (x-9, y-9), (x+9, y+9), (235, 235, 235), -1)
        nested = scene()
        cv2.rectangle(nested, (225, 115), (415, 245), (50, 60, 230), 5)
        expected = [[170, 70], [470, 70], [470, 290], [170, 290]]
        for name, frame in (('weak red', weak), ('white elbows', elbows), ('nested', nested)):
            with self.subTest(scene=name):
                result = self.run_frame(ResidentGatePipeline(self.gpu), frame)
                self.assertIsNotNone(result)
                self.assertTrue(result['complete'])
                np.testing.assert_allclose(result['corners'], expected, atol=4)

    def test_gpu_components_match_eight_connected_cpu_reference(self):
        rt = self.gpu.runtime
        rng = np.random.default_rng(114)
        mask = np.uint8(rng.random((33, 61)) > .8)*255
        mask[3:24, 5:17] = 255
        mask[27, 40] = mask[28, 41] = mask[29, 42] = 255
        h, w = mask.shape
        n = mask.size
        src, parents, stats, output = (rt.upload('cc_fixture', mask), rt.buffer('cc_parents', n*4),
                                      rt.buffer('cc_stats', n*20), rt.buffer('cc_output', n))
        control = rt.upload('cc_control', np.array([0, 1], np.float32))
        rt.run('rg_cc_init', n, [src, parents, stats, n, control])
        rt.run('rg_cc_link', n, [src, parents, w, h, control])
        rt.run('rg_cc_stats', ((n+255)//256)*64, [src, parents, stats, w, h, LocalMemory(256*4), control], local=64)
        rt.run('rg_cc_filter', n, [src, parents, stats, output, n, 3, 3, 0, control])
        _, labels, measurements, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = (measurements[:, 4] >= 3) & (np.maximum(measurements[:, 2], measurements[:, 3]) >= 3)
        keep[0] = False
        np.testing.assert_array_equal(rt.read(output, mask.shape, np.uint8), np.uint8(keep[labels])*255)

    def test_device_only_preprocess_rotation_keeps_clean_pixels_resident(self):
        frame = scene()
        h, w = frame.shape[:2]
        maps = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        valid = np.ones((h, w), bool)
        valid.setflags(write=False)
        self.gpu.reset_stats()
        with self.gpu.frame_batch():
            fixed, enhanced, _, _ = self.gpu.preprocess(
                frame, valid, maps=maps, device_reference=True, rotate_180=True)
            np.testing.assert_array_equal(enhanced, frame[::-1, ::-1])
            self.assertFalse(fixed.flags.writeable)
            self.assertIsNotNone(self.gpu.resident_buffer(fixed))
            before = self.gpu.runtime.upload_bytes
            self.assertEqual(self.gpu.device_input('clean_consumer', fixed), self.gpu.resident_buffer(fixed))
            self.assertEqual(self.gpu.runtime.upload_bytes, before)
            self.assertEqual(self.gpu.runtime.download_bytes, frame.nbytes)
            self.assertNotIn('resident_copy', self.gpu.runtime.kernel_ms)
            self.assertNotIn('sharpen_only', self.gpu.runtime.kernel_ms)
        with self.assertRaises(ValueError):
            self.gpu.preprocess(frame, valid, device_reference=True)
        # Older integrations pass strength and maps positionally. The argument
        # remains accepted, but cannot re-enable the deleted preprocessing step.
        self.gpu.reset_stats()
        fixed, output, _, enhancement_ms = self.gpu.preprocess(frame, valid, .6, maps)
        np.testing.assert_array_equal(fixed, frame)
        np.testing.assert_array_equal(output, frame)
        self.assertEqual(enhancement_ms, 0.)
        self.assertNotIn('sharpen_only', self.gpu.runtime.kernel_ms)

    def test_white_opening_matches_reference_and_width_tie_keeps_visible_length(self):
        rt = self.gpu.runtime
        mask = np.zeros((360, 640), np.uint8)
        mask[0:8, 0:8] = 255
        mask[23:28, 30:37] = 255
        mask[39, 49] = 255
        control = np.zeros(32, np.float32)
        control[1] = 1
        c = rt.upload('selection_control', control)
        src, out = rt.upload('white_fixture', mask), rt.buffer('white_fixture_open', mask.size)
        rt.run('rg_white_open2', mask.size, [src, out, c])
        np.testing.assert_array_equal(rt.read(out, mask.shape, np.uint8),
                                      cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)))
        # Similar widths use observed length; a clearly thicker object wins.
        models = np.zeros((3, 128), np.float32)
        for i, (width, length) in enumerate(((8., 200.), (8.5, 100.), (6., 400.))):
            models[i, 0] = 1
            models[i, 2] = 1
            models[i, 5] = width
            models[i, 64:68] = [10, 10, 10+length, 10]
        m = rt.upload('selection_models', models)
        prediction = rt.upload('selection_prediction', np.zeros(128, np.float32))
        state = rt.upload('selection_state', np.zeros(128, np.float32))
        rt.run('rg_select', 64, [m, prediction, state, c, 3, LocalMemory(256), LocalMemory(256)], local=64)
        self.assertEqual(rt.read(state, (128,), np.float32)[5], 8.)
        models[2, 5] = 12.
        m = rt.upload('selection_models', models)
        rt.run('rg_select', 64, [m, prediction, state, c, 3, LocalMemory(256), LocalMemory(256)], local=64)
        self.assertEqual(rt.read(state, (128,), np.float32)[5], 12.)

    def test_pose_recovers_known_projection_and_rejects_invalid_state(self):
        rt = self.gpu.runtime
        matrix = np.array([[500, 0, 320], [0, 510, 180], [0, 0, 1]], np.float32)
        obj = np.array([[-.35, -.25, 0], [.35, -.25, 0], [.35, .25, 0], [-.35, .25, 0]], np.float32)
        rotation, translation = np.array([.12, -.2, .07]), np.array([.06, -.03, 1.5])
        points = cv2.projectPoints(obj, rotation, translation, matrix, np.zeros(5))[0].reshape(4, 2)
        state = np.zeros(128, np.float32)
        state[0] = state[3] = 1
        state[8:16] = points.reshape(-1)
        s, k, out = rt.upload('pose_fixture', state), rt.upload('pose_camera', matrix), rt.buffer('pose_output', 160)
        rt.run('rg_pose', 1, [s, k, out, 1., 0, 0, 1])
        pose = rt.read(out, (40,), np.float32)
        self.assertEqual(pose[0], 1)
        np.testing.assert_allclose(pose[12:15], translation, atol=2e-4)
        np.testing.assert_allclose(pose[3:12].reshape(3, 3), cv2.Rodrigues(rotation)[0], atol=2e-4)
        self.assertLess(pose[1], .01)
        from src.gate_guidance import draw_gate_guidance
        guidance = ResidentGatePipeline.guidance(dict(target_bbox=[100, 50, 500, 320],
            yolo_age_frames=0, target_clipped=False, observation='detected'), pose)
        self.assertEqual(guidance['mode'], 'align')
        self.assertEqual(draw_gate_guidance(scene(), guidance, matrix).shape, (360, 640, 3))
        state[3] = 0
        rt.run('rg_pose', 1, [rt.upload('pose_fixture', state), k, out, 1., 0, 0, 1])
        self.assertEqual(rt.read(out, (40,), np.float32)[0], 0)


if __name__ == '__main__':
    unittest.main()
