"""纯视觉回归：检测输入、真实 CV 和现有共享读端的兼容性。"""
from dataclasses import replace
from pathlib import Path
import sys
import shutil
import struct
import asyncio
import threading
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
from shm_writer import ShmFrameWriter, ShmJsonWriter
from shm_reader import ShmFrameReader, ShmJsonReader


def scene():
    frame = np.full((480, 640, 3), (120, 90, 40), np.uint8)
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
        self.cfg = replace(CONFIG, correction_enabled=False, cv_backend='cpu')

    def test_yolo_cv_share_clean_input_without_any_enhancement(self):
        raw = scene()
        original = raw.copy()
        detector = FakeDetector()
        processor = DoorSimFrameProcessor(detector, self.cfg)
        processor._prepare(640, 480)
        expected = original
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
        self.assertEqual(obs['enhancement_mode'], 'none')
        self.assertEqual(obs['timing_ms']['enhancement'], 0.)

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
        processor = DoorSimFrameProcessor(FakeDetector([]), replace(CONFIG, cv_backend='cpu'))
        for size in ((640, 480), (1280, 720)):
            display, _, obs = processor.process(cv2.resize(scene(), size))
            self.assertEqual(display.shape[:2], size[::-1])
            self.assertEqual((obs['img_w'], obs['img_h']), size)
            self.assertEqual(obs['coordinate_space'], 'corrected')
            self.assertTrue(obs['guidance']['ui_only'])
            self.assertEqual(processor.tracker.index, 1)
            if size == (640, 480):
                np.testing.assert_array_equal(obs['camera_adaptation']['reference_to_processing'], np.eye(3))

    def test_shared_jpeg_json_roundtrip_has_no_control_observation(self):
        pipeline = DoorSimFrontPipeline(FakeDetector(), self.cfg)
        with tempfile.TemporaryDirectory() as directory:
            frame_path = str(Path(directory)/'momo_frame_front.bin')
            det_path = str(Path(directory)/'momo_det_front.json')
            frame_writer = ShmFrameWriter(frame_path, 640, 480)
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
            writer = cv2.VideoWriter(source, cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))
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
                                           '--cv-backend', 'cpu', '--no-log', '--no-timing']), 0)
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
        self.assertEqual(Path(factory.call_args.args[0]['model_path']).name,
                         'door_6_nashe_640x640_nv12.hbm')
        self.assertEqual(entry.call_args.args[0],
                         ['--no-show', '--device', '/dev/video9', '--workers', '3', '--frames', '20'])
        self.assertIsNotNone(entry.call_args.kwargs['door_sim_processor'])
        self.assertEqual(entry.call_args.kwargs['door_sim_processor'].cfg.cv_execution, 'resident')
        self.assertEqual((front.W, front.H), (640, 480))

    def test_launcher_passes_gpu_settings_and_closes_processor_on_front_failure(self):
        import front
        from to32.task_door_sim.run import main
        with patch.object(front, 'YoloDetector', return_value=FakeDetector()), \
             patch.object(front, 'main', side_effect=RuntimeError('capture failed')) as entry:
            with self.assertRaisesRegex(RuntimeError, 'capture failed'):
                main(['--cv-backend', 'opencl', '--gpu-device', 'Mali', '--cv-hough', 'cpu',
                      '--cv-blur', 'exact', '--cv-quality', 'precise', '--cv-search', 'full',
                      '--opencv-threads', '2', '--device', '/dev/video9'])
        processor = entry.call_args.kwargs['door_sim_processor']
        self.assertEqual(processor.cfg.cv_backend, 'opencl')
        self.assertEqual(processor.cfg.cv_quality, 'precise')
        self.assertEqual(processor.cfg.cv_search, 'full')
        self.assertEqual(processor.cfg.opencv_threads, 2)
        self.assertTrue(processor.closed)
        self.assertEqual(entry.call_args.args[0], ['--no-show', '--device', '/dev/video9'])

    def test_auto_cpu_fallback_is_published_with_reason(self):
        cfg = replace(self.cfg, cv_backend='auto')
        processor = DoorSimFrameProcessor(FakeDetector(), cfg)
        with patch('to32.task_door_sim.perception.create_backend',
                   return_value=(None, {'requested': 'auto', 'selected': 'cpu', 'fallback_reason': 'no GPU'})):
            _, _, obs = processor.process(scene())
        self.assertEqual(obs['cv_backend']['fallback_reason'], 'no GPU')
        self.assertEqual(obs['yolo']['cv_profile']['cv_backend'], 'cpu')
        self.assertNotIn('gpu', obs)
        processor.close()
        processor.close()
        with self.assertRaisesRegex(RuntimeError, '已关闭'):
            processor.process(scene())

    def test_close_waits_for_active_frame_before_releasing_gpu(self):
        from types import SimpleNamespace
        processor = DoorSimFrameProcessor(FakeDetector(), self.cfg)
        started, release, released = threading.Event(), threading.Event(), threading.Event()
        processor.cv_backend = SimpleNamespace(close=released.set)

        def slow(frame):
            started.set()
            release.wait(2)

        processor._process = slow
        worker = threading.Thread(target=processor.process, args=(scene(),))
        worker.start()
        self.assertTrue(started.wait(1))
        closer = threading.Thread(target=processor.close)
        closer.start()
        try:
            self.assertFalse(released.wait(.05))
        finally:
            release.set()
            worker.join(2); closer.join(2)
        self.assertTrue(released.is_set())
        self.assertFalse(worker.is_alive())
        self.assertFalse(closer.is_alive())

    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'GPU arithmetic harness requires clang')
    def test_opencl_resident_pipeline_shares_clean_input_and_releases_backend(self):
        self._assert_resident_pipeline(correction=False)

    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'GPU arithmetic harness requires clang')
    def test_opencl_corrected_pipeline_keeps_cv_resident(self):
        self._assert_resident_pipeline(correction=True)

    def _assert_resident_pipeline(self, correction):
        sys.path.insert(0, str(ROOT/'quad_cv_kit'))
        sys.path.insert(0, str(ROOT/'quad_cv_kit/tests'))
        from opencl_host import HostRuntime
        from quad_cv_kit.src.opencl_backend import OpenCLBackend
        from quad_cv_kit.src.opencl_runtime import LocalMemory
        cfg = replace(self.cfg, cv_backend='opencl', correction_enabled=correction)
        with patch('quad_cv_kit.src.opencl_backend.OpenCLRuntime', HostRuntime):
            gpu = OpenCLBackend(quality='fast', blur_mode='pyramid')
        processor = DoorSimFrameProcessor(FakeDetector(), cfg)
        local_memory = patch('opencl_host.LocalMemory', LocalMemory)
        local_memory.start()
        try:
            with patch('to32.task_door_sim.perception.create_backend',
                       return_value=(gpu, dict(requested='opencl', **gpu.runtime.info))) as factory, \
                 patch('quad_cv_kit.src.detect_red_gate.detect', side_effect=AssertionError('CPU CV forbidden')), \
                 patch('to32.task_door_sim.perception.build_gate_guidance', side_effect=AssertionError('CPU guidance forbidden')):
                display, detections, obs = processor.process(scene())
                self.assertEqual(factory.call_args.args, ('opencl', 'Mali', 'opencl', 'pyramid', 'fast'))
                self.assertIs(processor.tracker.cv.backend, gpu)
                self.assertEqual(processor.tracker.cv_execution, 'resident')
                self.assertIsNotNone(obs['geometry'])
                self.assertEqual(obs['gpu']['cv_quality'], 'resident')
                self.assertEqual(obs['gpu']['pipeline_version'], 7)
                self.assertIn('rg_hough_vote', obs['gpu']['kernel_ms'])
                self.assertEqual(obs['yolo']['cv_profile']['intermediate_readbacks'], 0)
                output_bytes = 640*480*3 if correction else 0
                self.assertEqual(obs['gpu']['download_bytes'], output_bytes+800)
                self.assertEqual(sum(value for key, value in obs['gpu']['transfer_bytes'].items()
                                     if key.startswith('upload:resident_') and
                                     key.endswith('_remap_source' if correction else '_preprocess_source')),
                                 640*480*3)
                self.assertEqual(obs['yolo']['ratio'], .75)
                self.assertEqual(obs['yolo']['offset'], [80, 0])
                if correction:
                    # Device descriptors carry shape only, never host pixels.
                    self.assertEqual(processor.tracker.last_cv_frame.shape, scene().shape)
                    self.assertFalse(processor.tracker.last_cv_frame.flags.writeable)
                    resident_pixels = next(array[:640*480*3].reshape(scene().shape)
                                           for name, array in gpu.runtime.arrays.items()
                                           if name.endswith('_remap_output'))
                    np.testing.assert_array_equal(processor.detector.detector.inputs[0], resident_pixels)
                else:
                    np.testing.assert_array_equal(processor.detector.detector.inputs[0], processor.tracker.last_cv_frame)
                self.assertEqual(display.shape, scene().shape)
                self.assertTrue(detections[0]['selected'])
                if correction:
                    self.assertIsNotNone(obs['guidance'])
                    np.testing.assert_array_equal(obs['camera_adaptation']['reference_to_processing'], np.eye(3))
                _, _, steady = processor.process(scene())
                self.assertEqual(steady['gpu']['upload_bytes'], 640*480*3+24)
                self.assertEqual(steady['gpu']['download_bytes'], output_bytes+800)
                self.assertEqual(steady['yolo']['cv_profile']['cpu_compute_stages'], [])
                processor.process(cv2.resize(scene(), (1280, 720)))
                self.assertEqual(factory.call_count, 1)
            with patch.object(gpu, 'close', wraps=gpu.close) as close:
                processor.close(); processor.close()
                self.assertEqual(close.call_count, 1)
        finally:
            processor.close()
            local_memory.stop()


    def test_processed_front_jpeg_is_served_by_cam1_http_route(self):
        try:
            import web_server
        except ImportError:
            self.skipTest('HTTP 验证需要 fastapi')
        import front
        import queue
        import threading
        import web_server

        raw = scene()
        processor = DoorSimFrameProcessor(FakeDetector(), self.cfg)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'momo_frame_front.bin')
            fw = ShmFrameWriter(path, 1280, 720)  # Simulate a writer with legacy dimensions.
            dw = ShmJsonWriter(str(Path(directory)/'momo_det_front.json'))
            reader = ShmFrameReader(path)
            try:
                q = queue.Queue()
                q.put((1, raw, None, time.time()))
                q.put(None)
                front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                             False, None, fw, dw, processor, 'gate.hbm')
                _, jpeg = reader.read_latest()
                header = struct.unpack(front.MC.HDR_FMT, reader._mm[:front.MC.HDR_SIZE])
                self.assertEqual(header[3], 640*480)
                payload = ShmJsonReader(dw.path).read()
                self.assertEqual((payload['img_w'], payload['img_h']), (640, 480))
                _, raw_jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 100])
                self.assertNotEqual(jpeg, raw_jpeg.tobytes())
                generator = web_server.mjpeg_generator

                def one_frame(frame_reader):
                    stream = generator(frame_reader)
                    try:
                        yield next(stream)
                    finally:
                        stream.close()

                messages = []

                async def request():
                    # Exercise the actual ASGI route without depending on
                    # board-specific httpx/TestClient version combinations.
                    scope = dict(type='http', asgi={'version': '3.0', 'spec_version': '2.3'},
                                 method='GET', path='/cam1', raw_path=b'/cam1', root_path='',
                                 query_string=b'', headers=[], scheme='http', http_version='1.1',
                                 client=('127.0.0.1', 1234), server=('127.0.0.1', 5000))

                    async def receive():
                        await asyncio.Event().wait()

                    async def send(message):
                        messages.append(message)

                    await web_server.app(scope, receive, send)

                with patch.object(web_server, 'front_frames', reader), \
                     patch.object(web_server, 'mjpeg_generator', one_frame):
                    asyncio.run(request())
                response = next(m for m in messages if m['type'] == 'http.response.start')
                self.assertEqual(response['status'], 200)
                self.assertIn(b'multipart/x-mixed-replace; boundary=frame',
                              dict(response['headers'])[b'content-type'])
                body = b''.join(m.get('body', b'') for m in messages if m['type'] == 'http.response.body')
                http_jpeg = body.split(b'\r\n\r\n', 1)[1][:-2]
                self.assertEqual(http_jpeg, jpeg)
                decoded = cv2.imdecode(np.frombuffer(http_jpeg, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded.shape, raw.shape)
                self.assertEqual(web_server.MC.WEB_PORT, 5000)
                self.assertEqual(raw.shape, (480, 640, 3))
            finally:
                reader.close()
                dw.close()
                fw.close()


if __name__ == '__main__':
    unittest.main()
