"""无相机、无串口测试：独立门模型、视觉后端和共享 JPEG/JSON。"""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import json
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path[:0] = [str(ROOT), str(ROOT/'src'), str(ROOT/'config'),
               str(ROOT/'quad_cv_kit/tests')]

import front
from to32.move_test.task.task_door.config import CONFIG
from to32.move_test.task.task_door.front_pipeline import DoorFrontPipeline, DoorDetector
from to32.move_test.task.task_door.perception import DoorFrameProcessor
from shm_writer import ShmFrameWriter, ShmJsonWriter
from shm_reader import ShmFrameReader, ShmJsonReader


class VisionTests(unittest.TestCase):
    def test_fixed_door_model_ignores_motion_stage_and_does_not_write_it(self):
        with tempfile.TemporaryDirectory() as directory:
            stage = Path(directory)/'momo_stage.json'
            stage.write_text('{"stage": "Task1"}')
            configs = []
            def factory(cfg):
                configs.append(cfg)
                return SimpleNamespace(detect=lambda frame, nv12=None: [])
            detector = DoorDetector({}, factory, directory, log=lambda _: None)
            detector.detect(np.zeros((480, 640, 3), np.uint8))
            self.assertEqual(detector.current_stage(), 'PassGate')
            self.assertTrue(detector.is_current() and detector.ready)
            self.assertTrue(configs[0]['model_path'].endswith('door_6_nashe_640x640_nv12.hbm'))
            self.assertEqual(json.loads(stage.read_text()), {'stage': 'Task1'})

    def test_front_worker_routes_pure_vision_without_stage_publication(self):
        image = np.zeros((480, 640, 3), np.uint8)
        q = queue.Queue()
        q.put((12, image, None, front.time.time()))
        q.put(None)
        seen = []
        detector = SimpleNamespace(current_stage=lambda: 'PassGate', is_current=lambda: True,
                                   ready=True)
        pipeline = SimpleNamespace(process=lambda *args: (seen.append(args) or (image, [])))
        with patch('to32.move_test.task.task_door.front_pipeline.DoorDetector', return_value=detector), \
             patch.object(front, 'door_pipeline', return_value=pipeline), \
             patch.object(front, 'YOLO_ENABLED', True):
            front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                         False, None, None, None, door_vision=True)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][0], 12)

    def test_shared_publication_keeps_frame_contract_and_drops_backlog(self):
        image = np.zeros((480, 640, 3), np.uint8)
        pipeline = DoorFrontPipeline()
        pipeline.processor = SimpleNamespace(process=lambda frame, detector, now:
            (frame, [], dict(valid=True, has_target=False, geometry=None, guidance=None)))
        detector = SimpleNamespace(stage='PassGate', ready=True, is_current=lambda: True,
                                   cfg={'model_path': 'door.hbm'})
        with tempfile.TemporaryDirectory() as directory:
            fw = ShmFrameWriter(str(Path(directory)/'frame.bin'), 640, 480)
            dw = ShmJsonWriter(str(Path(directory)/'momo_det_front.json'))
            reader = ShmFrameReader(fw.path)
            try:
                with patch('to32.move_test.task.task_door.front_pipeline.time.time', return_value=100.):
                    self.assertIsNotNone(pipeline.process(10, image, detector, 99.9, fw, dw, 85))
                    record = ShmJsonReader(dw.path).read()
                    self.assertEqual((record['frame'], record['capture_ts']), (10, 99.9))
                    self.assertIn('guidance', record['door'])
                    _, jpeg = reader.read_latest()
                    self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape,
                                     image.shape)
                    self.assertIsNone(pipeline.process(9, image, detector, 99.9, fw, dw, 85))
                    self.assertIsNone(pipeline.process(11, image, detector, 98., fw, dw, 85))
                    self.assertEqual(ShmJsonReader(dw.path).read()['frame'], 10)
            finally:
                fw.close()
                dw.close()
                reader.close()

    def test_target_continuity_and_input_validation(self):
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='cpu'))
        near = dict(label='door', score=.7, bbox=[50, 60, 600, 440])
        far = dict(label='door', score=.99, bbox=[250, 180, 350, 280])
        self.assertEqual(processor._select([near, far], 0)['bbox'], near['bbox'])
        self.assertIsNone(processor._select([far], .1))
        self.assertEqual(processor._select([far], 2)['bbox'], far['bbox'])
        with self.assertRaisesRegex(ValueError, '640×480'):
            processor.process(np.zeros((720, 1280, 3), np.uint8), None, 3.)
        with self.assertRaisesRegex(ValueError, 'uint8 BGR'):
            processor.process(None, None, 3.)
        processor.close()

    def test_cpu_backend_keeps_visual_pose_without_robot_control_fields(self):
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='cpu'))
        matrix = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
        corners = np.array([[145., 115.], [495., 115.], [495., 365.], [145., 365.]])
        geometry = dict(observation='detected', complete=True, corners=corners.tolist(),
                        segments=[[a.tolist(), b.tolist()] for a, b in zip(corners, np.roll(corners, -1, axis=0))])
        processor.corrector = SimpleNamespace(output_matrix=matrix, undistort=lambda image, _: image)
        processor.valid_mask = np.ones((480, 640), bool)
        detector = SimpleNamespace(detect=lambda image, nv12=None:
            [dict(label='door', score=.9, bbox=[135, 105, 505, 375])])
        try:
            with patch('to32.move_test.task.task_door.perception.create_backend', return_value=(None, {'selected': 'cpu'})), \
                 patch.object(processor.cv, 'update', return_value=({}, [])), \
                 patch('to32.move_test.task.task_door.perception.original_geometry', return_value=geometry):
                _, _, obs = processor.process(np.zeros((480, 640, 3), np.uint8), detector, 100.)
            self.assertEqual(obs['geometry'], geometry)
            self.assertTrue(obs['guidance']['ui_only'])
            self.assertIsNotNone(obs['guidance']['alignment'])
            self.assertNotIn('pose', obs)
            self.assertNotIn('control_pose_ms', obs)
            self.assertNotIn('reset_token', obs)
        finally:
            processor.close()

    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'OpenCL C host correctness harness requires clang')
    def test_resident_backend_preserves_geometry_ui_and_frame_residency(self):
        sys.path.insert(0, str(ROOT/'quad_cv_kit'))
        from opencl_host import HostRuntime
        from quad_cv_kit.src.opencl_backend import OpenCLBackend
        from quad_cv_kit.src.opencl_runtime import LocalMemory
        matrix = np.array([[500., 0, 320], [0, 510., 240], [0, 0, 1]])
        frame = np.full((480, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (145, 113), (495, 368), (50, 60, 230), 6)
        yy, xx = np.indices((480, 640), dtype=np.float32)
        with patch('quad_cv_kit.src.opencl_backend.OpenCLRuntime', HostRuntime):
            gpu = OpenCLBackend(quality='fast', blur_mode='pyramid')
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='opencl'))
        processor.corrector = SimpleNamespace(output_matrix=matrix, maps=lambda _: (xx, yy))
        processor.valid_mask = np.ones((480, 640), bool)
        def detect(fixed, nv12=None):
            self.assertIsNone(nv12)
            np.testing.assert_array_equal(fixed, frame)
            return [dict(label='door', score=.9, bbox=[135, 100, 505, 380])]
        detector = SimpleNamespace(stage='PassGate', ready=True, is_current=lambda: True,
                                   cfg={'model_path': 'door_6_nashe_640x640_nv12.hbm'}, detect=detect)
        try:
            with tempfile.TemporaryDirectory() as directory, \
                 patch('opencl_host.LocalMemory', LocalMemory), \
                 patch('to32.move_test.task.task_door.front_pipeline.time.time', return_value=100.), \
                 patch('to32.move_test.task.task_door.perception.create_backend', return_value=(gpu, gpu.runtime.info)), \
                 patch('to32.move_test.task.task_door.perception.RedGateTracker.update', side_effect=AssertionError('CPU image CV forbidden')):
                pipeline = DoorFrontPipeline()
                pipeline.processor = processor
                fw = ShmFrameWriter(str(Path(directory)/'frame.bin'), 640, 480)
                dw = ShmJsonWriter(str(Path(directory)/'momo_det_front.json'))
                try:
                    for index in (1, 2):
                        self.assertIsNotNone(pipeline.process(index, frame, detector, 99.9, fw, dw, 85))
                        obs = ShmJsonReader(dw.path).read()['door']
                        self.assertTrue(obs['geometry']['complete'])
                        self.assertEqual(len(obs['geometry']['corners']), 4)
                        self.assertTrue(obs['guidance']['ui_only'])
                        self.assertIsNotNone(obs['guidance']['alignment'])
                        self.assertEqual(obs['cv_profile']['intermediate_readbacks'], 0)
                        self.assertEqual(obs['gpu']['download_bytes'], 640*480*3+800)
                        self.assertNotIn('pose', obs)
                        if index == 2:
                            self.assertEqual(obs['gpu']['upload_bytes'], 640*480*3+24)
                    pipeline.close()
                    pipeline.close()
                finally:
                    fw.close()
                    dw.close()
        finally:
            processor.close()

    def test_mission_import_has_no_removed_door_controller(self):
        code = ('import sys; sys.path.insert(0, sys.argv[1]); '
                'import mission, task_config; from test_mode import test_config; '
                'assert task_config.DOOR_TABLE == []; '
                'assert test_config.DOOR_TABLE == []; '
                'assert task_config.PASS_DOOR_V2_TABLE; '
                'assert all(cls.__name__ != "DoorTask" for cls in task_config.STAGE_TABLE)')
        result = subprocess.run([sys.executable, '-c', code, str(ROOT/'config')], cwd=ROOT/'src/to32/move_test',
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
