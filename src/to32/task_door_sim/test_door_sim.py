"""纯视觉回归：检测输入、真实 CV 和现有共享读端的兼容性。"""
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT, ROOT/'src', ROOT/'config'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from to32.task_door_sim.config import CONFIG
from to32.task_door_sim.perception import DoorSimFrameProcessor
from to32.task_door_sim.front_pipeline import DoorSimFrontPipeline
from quad_cv_kit.src.gate_guidance import enhance_cv_contrast
from shm_writer import ShmFrameWriter, ShmJsonWriter
from shm_reader import ShmFrameReader, ShmJsonReader


def scene():
    frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    cv2.rectangle(frame, (220, 130), (390, 275), (50, 60, 230), 5)
    return frame


class FakeDetector:
    def __init__(self, detections=None):
        self.detections = (detections if detections is not None else
                           [dict(label='door', score=.9, bbox=[210, 120, 400, 285],
                                 center=[305, 202])])
        self.inputs = []

    def detect(self, frame, nv12=None):
        if nv12 is not None:
            raise AssertionError('不能复用原始 NV12')
        self.inputs.append(frame.copy())
        return self.detections


class DoorSimTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CONFIG, correction_enabled=False)

    def test_yolo_cv_share_enhanced_input_and_clean_color_evidence(self):
        raw = scene()
        original = raw.copy()
        detector = FakeDetector()
        processor = DoorSimFrameProcessor(detector, self.cfg)
        processor._prepare(640, 360)
        expected = enhance_cv_contrast(raw, 1.2, processor.valid_mask, 0, 0, .6, 1)
        self.assertFalse(np.array_equal(raw, expected))
        with patch.object(processor.tracker.cv, 'update', wraps=processor.tracker.cv.update) as update:
            display, dets, obs = processor.process(raw)
        np.testing.assert_array_equal(detector.inputs[0], expected)
        np.testing.assert_array_equal(update.call_args.args[0], expected)
        np.testing.assert_array_equal(update.call_args.kwargs['reference_frame'], original)
        np.testing.assert_array_equal(raw, original)
        self.assertTrue(dets[0]['selected'])
        self.assertIsNotNone(obs['geometry'])
        self.assertEqual(obs['geometry']['observation'], 'detected')
        self.assertFalse(np.array_equal(display, expected))

    def test_only_door_detections_enable_cv(self):
        detector = FakeDetector([dict(label='red-ball', score=.99, bbox=[210, 120, 400, 285])])
        processor = DoorSimFrameProcessor(detector, self.cfg)
        with patch('quad_cv_kit.src.detect_red_gate.detect', side_effect=AssertionError('不应启动 CV')):
            _, detections, obs = processor.process(scene())
        self.assertEqual(detections, [])
        self.assertFalse(obs['has_target'])
        self.assertFalse(obs['yolo']['cv_enabled'])
        self.assertIsNone(obs['geometry'])

    def test_corrected_coordinates_and_resolution_change(self):
        processor = DoorSimFrameProcessor(FakeDetector([]))
        for size in ((640, 360), (1280, 720)):
            display, _, obs = processor.process(cv2.resize(scene(), size))
            self.assertEqual(display.shape[:2], size[::-1])
            self.assertEqual((obs['img_w'], obs['img_h']), size)
            self.assertEqual(obs['coordinate_space'], 'corrected')
            self.assertTrue(obs['guidance']['ui_only'])
            self.assertEqual(processor.tracker.index, 1)

    def test_shared_jpeg_json_roundtrip_has_no_control_observation(self):
        pipeline = DoorSimFrontPipeline(FakeDetector(), self.cfg)
        with tempfile.TemporaryDirectory() as directory:
            frame_path = str(Path(directory)/'momo_frame_front.bin')
            det_path = str(Path(directory)/'momo_det_front.json')
            frame_writer = ShmFrameWriter(frame_path, 640, 360)
            det_writer = ShmJsonWriter(det_path)
            frame_reader = ShmFrameReader(frame_path)
            try:
                captured_at = time.time()
                result = pipeline.process(17, scene(), captured_at, frame_writer, det_writer)
                seq, jpeg = frame_reader.read_latest()
                self.assertEqual(seq, 1)
                self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape,
                                 result[0].shape)
                ok, expected = cv2.imencode('.jpg', result[0], [cv2.IMWRITE_JPEG_QUALITY, self.cfg.jpeg_quality])
                self.assertTrue(ok)
                self.assertEqual(jpeg, expected.tobytes())
                data = ShmJsonReader(det_path).read()
                self.assertEqual(data['frame'], 17)
                self.assertEqual(data['capture_ts'], captured_at)
                self.assertEqual(data['stage'], 'DoorSim')
                self.assertNotIn('door', data)
                self.assertEqual(data['door_sim']['geometry'], result[2]['geometry'])
            finally:
                frame_reader.close()
                det_writer.close()
                frame_writer.close()

    def test_stale_duplicate_and_out_of_order_frames_skip_inference(self):
        detector = FakeDetector([])
        pipeline = DoorSimFrontPipeline(detector, self.cfg)
        raw = scene()
        self.assertIsNone(pipeline.process(1, raw, time.time()-2, None, None))
        self.assertIsNotNone(pipeline.process(2, raw, time.time(), None, None))
        self.assertIsNone(pipeline.process(2, raw, time.time(), None, None))
        self.assertIsNone(pipeline.process(0, raw, time.time(), None, None))
        self.assertEqual(len(detector.inputs), 1)

    def test_video_runner_publishes_without_stage_switch(self):
        from to32.task_door_sim.run import main
        import front
        with tempfile.TemporaryDirectory() as directory:
            source = str(Path(directory)/'door.avi')
            writer = cv2.VideoWriter(source, cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 360))
            self.assertTrue(writer.isOpened())
            for _ in range(3):
                writer.write(scene())
            writer.release()
            with patch.multiple(front.MC, SHM_FRAME_FRONT=str(Path(directory)/'momo_frame_front.bin'),
                                SHM_DET_FRONT=str(Path(directory)/'momo_det_front.json'),
                                SHM_STATS_FRONT=str(Path(directory)/'momo_stats_front.json')):
                with patch.object(front, 'YoloDetector', return_value=FakeDetector()), \
                     patch.object(front, 'main', wraps=front.main) as front_main:
                    self.assertEqual(main(['--source', source, '--no-correction', '--frames', '2',
                                           '--no-log', '--no-timing']), 0)
                    self.assertEqual(front_main.call_count, 1)
            data = ShmJsonReader(str(Path(directory)/'momo_det_front.json')).read()
            self.assertEqual(data['frame'], 1)
            self.assertTrue(data['door_sim']['has_target'])
            self.assertFalse((Path(directory)/'momo_stage.json').exists())

    def test_front_worker_writes_processed_frame_and_keeps_stream_on_vision_error(self):
        import front
        import queue
        import threading
        from types import SimpleNamespace
        raw = scene()
        display = np.full_like(raw, (12, 155, 220))
        processor = SimpleNamespace(process=lambda frame: (display, [], dict(valid=True)))
        with tempfile.TemporaryDirectory() as directory:
            frame_path = str(Path(directory)/'momo_frame_front.bin')
            det_path = str(Path(directory)/'momo_det_front.json')
            fw, dw = ShmFrameWriter(frame_path), ShmJsonWriter(det_path)
            reader = ShmFrameReader(frame_path)
            try:
                for fail in (False, True):
                    if fail:
                        def failed(frame):
                            raise ValueError('CV unavailable')
                        processor.process = failed
                    q = queue.Queue()
                    q.put((8, raw, None, time.time()))
                    q.put(None)
                    front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                                 False, None, fw, dw, processor, 'gate.hbm')
                    _, jpeg = reader.read_latest()
                    _, expected = cv2.imencode('.jpg', raw if fail else display,
                                                [cv2.IMWRITE_JPEG_QUALITY, front.MC.WEB_MJPEG_QUALITY])
                    self.assertEqual(jpeg, expected.tobytes())
                    data = ShmJsonReader(det_path).read()
                    self.assertEqual(data['stage'], 'DoorSim')
                    self.assertEqual(data['door_sim']['valid'], not fail)
                    self.assertNotIn('door', data)
            finally:
                reader.close()
                dw.close()
                fw.close()

    def test_camera_launcher_uses_front_main_and_gate_model(self):
        import front
        from to32.task_door_sim.run import main
        from stage_model import model_config
        with patch.object(front, 'YoloDetector', return_value=FakeDetector()) as factory, \
             patch.object(front, 'main', return_value=0) as entry, \
             patch.object(cv2, 'VideoCapture', side_effect=AssertionError('启动器不自行开相机')):
            self.assertEqual(main(['--device', '/dev/video9', '--workers', '3', '--frames', '20']), 0)
        self.assertEqual(factory.call_args.args[0]['model_path'],
                         model_config('PassGate', front.YOLO_CFG)['model_path'])
        self.assertEqual(entry.call_args.args[0],
                         ['--no-show', '--device', '/dev/video9', '--workers', '3', '--frames', '20'])
        self.assertIsNotNone(entry.call_args.kwargs['door_sim_processor'])


    def test_processed_front_jpeg_is_served_by_cam1_http_route(self):
        try:
            from fastapi.testclient import TestClient
        except ImportError:
            self.skipTest('HTTP 验证需要 fastapi/httpx')
        import front
        import queue
        import threading
        import web_server

        raw = scene()
        processor = DoorSimFrameProcessor(FakeDetector(), self.cfg)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'momo_frame_front.bin')
            fw = ShmFrameWriter(path, 640, 360)
            dw = ShmJsonWriter(str(Path(directory)/'momo_det_front.json'))
            reader = ShmFrameReader(path)
            try:
                q = queue.Queue()
                q.put((1, raw, None, time.time()))
                q.put(None)
                front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                             False, None, fw, dw, processor, 'gate.hbm')
                _, jpeg = reader.read_latest()
                _, raw_jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 100])
                self.assertNotEqual(jpeg, raw_jpeg.tobytes())
                generator = web_server.mjpeg_generator

                def one_frame(frame_reader):
                    stream = generator(frame_reader)
                    try:
                        yield next(stream)
                    finally:
                        stream.close()

                with patch.object(web_server, 'front_frames', reader), \
                     patch.object(web_server, 'mjpeg_generator', one_frame), \
                     TestClient(web_server.app) as client:
                    response = client.get('/cam1')
                self.assertEqual(response.status_code, 200)
                self.assertIn('multipart/x-mixed-replace; boundary=frame',
                              response.headers['content-type'])
                http_jpeg = response.content.split(b'\r\n\r\n', 1)[1][:-2]
                self.assertEqual(http_jpeg, jpeg)
                decoded = cv2.imdecode(np.frombuffer(http_jpeg, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded.shape, raw.shape)
                self.assertEqual(web_server.MC.WEB_PORT, 5000)
            finally:
                reader.close()
                dw.close()
                fw.close()


if __name__ == '__main__':
    unittest.main()
