"""无板端硬件回归：四通道门模型、DFL 模型和 front 的异常回传。"""
import contextlib
import io
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'config'))

class YoloPostprocessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 开发机缺少 BPU runtime；数值解码和图像处理仍运行真实实现。
        try:
            import hbm_runtime
        except ImportError:
            runtime_patch = patch.dict(sys.modules, {'hbm_runtime': SimpleNamespace(QuantParams=object)})
            runtime_patch.start()
            cls.addClassCleanup(runtime_patch.stop)
        global function, decode_boxes
        import function
        from utils.py_utils.postprocess import decode_boxes
        if function.post_utils is None:
            import importlib
            importlib.reload(function)

    def setUp(self):
        self.weights = np.arange(16, dtype=np.float32)[None, None, :]

    def test_direct_ltrb_uses_selected_grid_cells_and_stride(self):
        output = np.zeros((1, 4, 4, 4), dtype=np.float32)
        output[0, 1, 2] = [1, 2, 3, 4]
        output[0, 3, 0] = [.5, 1.5, 2.5, 3.5]
        for stride in (8, 16, 32):
            with self.subTest(stride=stride):
                boxes = decode_boxes(output, np.array([6, 12]), 4, stride, self.weights)
                expected = np.array([[1.5, -.5, 5.5, 5.5], [0, 2, 3, 7]]) * stride
                np.testing.assert_allclose(boxes, expected)

    def test_dfl_preserves_expected_distances_and_configured_bins(self):
        for bins in (8, 16):
            with self.subTest(bins=bins):
                # 大 logit 差使四边的期望距离分别为 1、2、3、4。
                output = np.full((1, 4, 4, 4, bins), -100, dtype=np.float32)
                for side, distance in enumerate((1, 2, 3, 4)):
                    output[0, 1, 2, side, distance] = 100
                weights = np.arange(bins, dtype=np.float32)[None, None, :]
                boxes = decode_boxes(output.reshape(1, 4, 4, 4*bins),
                                     np.array([6]), 4, 8, weights)
                np.testing.assert_allclose(boxes, [[12, -4, 44, 44]])

    def test_empty_and_invalid_outputs(self):
        for channels in (4, 64):
            boxes = decode_boxes(np.zeros((1, 4, 4, channels), np.float32),
                                 np.array([], dtype=int), 4, 8, self.weights)
            self.assertEqual(boxes.shape, (0, 4))
        for indices in (np.array([], dtype=int), np.array([6])):
            with self.assertRaisesRegex(ValueError, 'expected 4 LTRB channels or 64 DFL channels'):
                decode_boxes(np.zeros((1, 4, 4, 1), np.float32),
                             indices, 4, 8, self.weights)

    def test_640_nv12_hbm_model_through_door_sim_front_worker(self):
        import front
        from to32.task_door_sim.config import CONFIG
        from to32.task_door_sim.perception import DoorSimFrameProcessor
        from dataclasses import replace

        outputs = {}
        for index, size in enumerate((80, 40, 20)):
            outputs[f'cls{index}'] = np.full((1, size, size, 1), -10, np.float32)
            outputs[f'box{index}'] = np.zeros((1, size, size, 4), np.float32)
        outputs['cls0'][0, 35, 38, 0] = 4
        outputs['box0'][0, 35, 38] = [12.25, 10.5, 11.5, 10.125]
        submitted = []
        def infer(tensor):
            submitted.append(tensor)
            return {'door': outputs}
        runtime = SimpleNamespace(
            model_names=['door'], input_names={'door': ['y', 'uv']},
            input_shapes={'door': {'y': (1, 640, 640, 1), 'uv': (1, 320, 320, 2)}},
            output_names={'door': list(outputs)},
            run=infer, set_scheduling_params=lambda **kwargs: None)
        with patch.dict(sys.modules, {'hbm_runtime': SimpleNamespace(
                HB_HBMRuntime=lambda path: runtime)}):
            cfg = dict(front.YOLO_CFG, model_path=str(ROOT/'models/door_6_nashe_640x640_nv12.hbm'),
                       class_names=['door'], target_class_names=['door'], target_class_ids=[])
            detector = function.YoloDetector(cfg)
        processor = DoorSimFrameProcessor(detector, replace(CONFIG, correction_enabled=False, cv_backend='cpu'))
        raw = np.full((480, 640, 3), (120, 90, 40), np.uint8)
        q = queue.Queue()
        q.put((8, raw, None, time.time()))
        q.put(None)
        published = []
        writer = SimpleNamespace(write=published.append)
        front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                     False, None, None, writer, processor, cfg['model_path'])
        self.assertEqual(published[0]['status'], 'done')
        self.assertTrue(published[0]['door_sim']['valid'])
        np.testing.assert_allclose(published[0]['dets'][0]['bbox'], [210, 120, 400, 285])
        self.assertAlmostEqual(published[0]['dets'][0]['score'], 1/(1+np.exp(-4)), places=6)
        self.assertEqual(submitted[0]['door']['y'].shape, (1, 640, 640, 1))
        self.assertEqual(submitted[0]['door']['uv'].shape, (1, 320, 320, 2))
        from utils.py_utils.preprocess import resized_image, bgr_to_nv12_planes
        letterboxed = resized_image(raw, 640, 640)
        np.testing.assert_array_equal(letterboxed[80:560], raw)
        self.assertTrue(np.all(letterboxed[:80] == 127))
        self.assertTrue(np.all(letterboxed[560:] == 127))
        y, uv = bgr_to_nv12_planes(letterboxed)
        np.testing.assert_array_equal(submitted[0]['door']['y'], y)
        np.testing.assert_array_equal(submitted[0]['door']['uv'], uv)

    def test_repeated_vision_errors_log_traceback_and_keep_publishing(self):
        import front

        def failed(frame):
            raise ValueError('bad output shape')

        q = queue.Queue()
        raw = np.zeros((16, 16, 3), np.uint8)
        for fid in range(3):
            q.put((fid, raw, None, time.time()))
        q.put(None)
        published = []
        stderr = io.StringIO()
        with patch.object(front.time, 'monotonic', return_value=10), contextlib.redirect_stderr(stderr):
            front.worker(0, q, queue.Queue(), False, threading.Event(), front.StageStats(),
                         False, None, None, SimpleNamespace(write=published.append),
                         SimpleNamespace(process=failed), 'gate.hbm')
        self.assertEqual(len(published), 3)
        self.assertTrue(all(p['status'] == 'vision_error' for p in published))
        self.assertEqual(stderr.getvalue().count('视觉处理失败'), 1)
        self.assertIn('Traceback (most recent call last)', stderr.getvalue())
        self.assertIn('model=gate.hbm', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
