"""实时帧不积压、共享 JPEG 不撕裂、CV 间隔帧仍验证当前图像。"""
from dataclasses import replace
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'config')]

import front
from shm_reader import ShmFrameReader
from shm_writer import ShmFrameWriter
from to32.task_door_sim.config import CONFIG
from to32.task_door_sim.perception import DoorSimFrameProcessor
from to32.task_door_sim.test_door_sim import FakeDetector, scene


class FrontStreamTests(unittest.TestCase):
    def test_front_closes_and_discards_shared_formal_gpu_pipeline(self):
        from unittest.mock import Mock
        pipeline = Mock()
        with patch.object(front, '_door_pipeline', pipeline):
            front.close_door_pipeline()
            front.close_door_pipeline()
            pipeline.close.assert_called_once_with()
            self.assertIsNone(front._door_pipeline)

    def test_front_camera_and_reference_geometry_use_vga(self):
        from to32.move_test.task.task_door.config import CONFIG as door_config
        from quad_cv_kit.src.camera_correction import load_camera_params
        self.assertEqual((front.W, front.H), (640, 480))
        self.assertEqual(front.MARK_POINT, [320, 240])
        self.assertEqual((door_config.image_width, door_config.image_height), (640, 480))
        camera = load_camera_params(CONFIG.camera_params_path)
        self.assertEqual((camera['width'], camera['height']), (640, 480))

    def test_camera_negotiating_wrong_resolution_stops_before_publication(self):
        from types import SimpleNamespace
        camera = SimpleNamespace(read=lambda: (True, np.zeros((720, 1280, 3), np.uint8)))
        q, stop, stats = queue.Queue(), threading.Event(), front.StageStats()
        front.producer(camera, q, 1, stop, 1, stats, False, None, None,
                       expected_size=(640, 480))
        self.assertTrue(stop.is_set())
        self.assertIn('640×480', stats.capture_error)
        self.assertIn('1280×720', stats.capture_error)
        self.assertTrue(q.empty())

    def test_camera_requests_vga_on_hardware_and_opencv_paths(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        hardware = Mock()
        with patch.dict(sys.modules, {'hw_camera': SimpleNamespace(HwMjpgCamera=hardware)}):
            with patch.dict(front.CAM, hardware_decode=True):
                front.open_camera('/dev/front', 0)
        self.assertEqual(hardware.call_args.args[:3], ('/dev/front', 640, 480))
        capture = Mock()
        with patch.dict(front.CAM, hardware_decode=False), patch.object(cv2, 'VideoCapture', return_value=capture):
            front.open_camera('/dev/front', 0)
        capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def test_slow_consumer_receives_latest_camera_frame_without_capture_backpressure(self):
        finished_capture = threading.Event()

        class Camera:
            def read(self):
                return True, scene()

        class LiveQueue(queue.Queue):
            def put(self, item, *args, **kwargs):
                if item is None:
                    finished_capture.set()
                return super().put(item, *args, **kwargs)

        q = LiveQueue(maxsize=1)
        stop = threading.Event()
        thread = threading.Thread(target=front.producer,
                                  args=(Camera(), q, 15, stop, 1, front.StageStats(), False,
                                        None, None, False, True, True), daemon=True)
        thread.start()
        try:
            self.assertTrue(finished_capture.wait(2), '采集不能因检测积压而阻塞')
            self.assertEqual(q.get(timeout=1)[0], 14)
            self.assertIsNone(q.get(timeout=1))
        finally:
            stop.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())

    def test_jpeg_header_is_published_only_after_payload_and_visible_in_other_process(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'frame.bin')
            writer = ShmFrameWriter(path)
            reader = ShmFrameReader(path)
            observed = []
            mm = writer._mm

            class ObservedMapping:
                def __setitem__(self, key, value):
                    mm[key] = value
                    observed.append(reader.read_latest())

            try:
                _, encoded = cv2.imencode('.jpg', scene())
                jpeg = encoded.tobytes()
                with patch.object(writer, '_mm', ObservedMapping()):
                    writer.write(jpeg)
                self.assertEqual(observed[:2], [(-1, None), (-1, None)])
                self.assertEqual(observed[2], (1, jpeg))
                # 写端尚未 close/flush，另一个进程也必须读到完整帧。
                script = ('import sys,io,contextlib; '
                          'exec("with contextlib.redirect_stdout(io.StringIO()):\\n from shm_reader import ShmFrameReader"); '
                          'r=ShmFrameReader(sys.argv[1]); seq,jpeg=r.read_latest(); '
                          'assert seq==1; sys.stdout.buffer.write(jpeg); r.close()')
                actual = subprocess.check_output([sys.executable, '-c', script, path], cwd=ROOT/'src')
                self.assertEqual(actual, jpeg)
            finally:
                reader.close()
                writer.close()

    def test_reader_skips_old_jpeg_copy_and_rejects_frame_changed_during_read(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = ShmFrameWriter(str(Path(directory)/'frame.bin'))
            reader = ShmFrameReader(writer.path)
            try:
                writer.write(b'first-jpeg')
                reader._ensure()
                mm = reader._mm
                payload_reads = []

                class ConcurrentMapping:
                    def __getitem__(self, key):
                        result = mm[key]
                        if key.start == front.MC.HDR_SIZE:
                            payload_reads.append(key)
                            writer.write(b'second-jpeg')
                        return result

                with patch.object(reader, '_mm', ConcurrentMapping()):
                    self.assertEqual(reader.read_latest(after_seq=1), (1, None))
                    self.assertEqual(payload_reads, [])
                    self.assertEqual(reader.read_latest(), (-1, None))
                self.assertEqual(reader.read_latest(after_seq=1), (2, b'second-jpeg'))
            finally:
                reader.close()
                writer.close()

    def test_cv_interval_tracks_current_image_and_researches_when_tracking_fails(self):
        processor = DoorSimFrameProcessor(FakeDetector(), replace(CONFIG, correction_enabled=False, cv_backend='cpu'))
        raw = scene()
        statuses = [processor.process(raw)[2] for _ in range(4)]
        self.assertEqual([s['yolo']['detection_ran'] for s in statuses], [True, False, False, True])
        self.assertEqual(statuses[1]['geometry']['observation'], 'tracked')
        self.assertEqual(len(processor.detector.detector.inputs), 4)
        with patch('quad_cv_kit.src.detect_red_gate.move_with_image', return_value=None):
            observation = processor.process(raw)[2]
        self.assertTrue(observation['yolo']['detection_ran'])
        self.assertEqual(observation['geometry']['observation'], 'detected')
        self.assertEqual(set(observation['timing_ms']), {'correction', 'enhancement', 'yolo', 'cv', 'overlay'})

    def test_front_publishes_configured_compression_and_stream_timing(self):
        from types import SimpleNamespace

        display = scene()
        processor = SimpleNamespace(cfg=replace(CONFIG, jpeg_quality=70),
                                    process=lambda frame: (display, [], {'valid': True}))
        frames, payloads = [], []
        frame_writer = SimpleNamespace(write=frames.append)
        q = queue.Queue()
        q.put((1, display, None, front.time.time()))
        q.put(None)
        with patch.object(front.MC, 'WEB_MJPEG_QUALITY', 100):
            front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                         False, None, frame_writer, SimpleNamespace(write=payloads.append), processor)
        _, encoded = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self.assertEqual(frames[0], encoded.tobytes())
        metrics = payloads[0]['door_sim']['stream']
        self.assertEqual(metrics['jpeg_quality'], 70)
        self.assertEqual(metrics['jpeg_bytes'], len(frames[0]))
        self.assertGreaterEqual(metrics['capture_to_publish_ms'], 0)


if __name__ == '__main__':
    unittest.main()
