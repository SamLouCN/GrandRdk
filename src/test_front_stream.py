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
        processor = DoorSimFrameProcessor(FakeDetector(), replace(CONFIG, correction_enabled=False))
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
