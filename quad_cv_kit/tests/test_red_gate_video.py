"""Offline video export and stage accounting without BPU/PyTorch dependencies."""
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import red_gate_video as V
from src.camera_correction import adapt_camera_params, create_corrector, load_camera_params
from src.video_output import VideoOutput


class FakeDetector:
    names = {0: 'door'}
    target_ids = {0}

    def __init__(self, empty=False, fail=False):
        self.frames = []
        self.empty, self.fail = empty, fail

    def detect_boxes(self, frame):
        self.frames.append(frame.copy())
        if self.fail:
            raise RuntimeError('inference failed')
        return [] if self.empty else [dict(class_id=0, label='door', score=.9, bbox=[210, 90, 420, 290])]


class RedGateVideoTests(unittest.TestCase):
    def test_json_log_converts_nested_numpy_values_without_losing_numbers(self):
        handle = io.StringIO()
        V._log(handle, frame=np.int64(7), status=dict(
            region=np.array([[170, 50, 460, 330]], dtype=np.int64),
            flags=[np.bool_(True)], score=np.float32(.5)))
        self.assertEqual(json.loads(handle.getvalue()), dict(frame=7, status=dict(
            region=[[170, 50, 460, 330]], flags=[True], score=.5)))

    def test_json_log_keeps_invalid_numbers_and_unknown_objects_as_errors(self):
        for value in (np.float32(np.nan), np.float32(np.inf), np.array([np.nan])):
            handle = io.StringIO()
            with self.assertRaises(ValueError):
                V._log(handle, value=value)
            self.assertEqual(handle.getvalue(), '')
        with self.assertRaises(TypeError):
            V._log(io.StringIO(), value=object())

    def source(self, folder, frames=5):
        path = folder/'source.avi'
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 12., (640, 360))
        self.assertTrue(writer.isOpened())
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (220, 100), (410, 280), (50, 60, 230), 8)
        for _ in range(frames):
            writer.write(frame)
        writer.release()
        return path

    def arguments(self, folder, source, *extra):
        weights = folder/'door.hbm'
        weights.touch()
        return [str(source), '--backend', 'hbm', '--weights', str(weights),
                '--out', str(folder/'runs'), '--perf-log', str(folder/'logs'/'perf.log'), *extra]

    def records(self, path):
        return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]

    def assert_video(self, path, frames):
        cap = cv2.VideoCapture(str(path))
        try:
            self.assertTrue(cap.isOpened())
            self.assertAlmostEqual(cap.get(cv2.CAP_PROP_FPS), 12., places=2)
            count = 0
            while True:
                ok, image = cap.read()
                if not ok:
                    break
                count += 1
                self.assertEqual(image.shape, (360, 640, 3))
            self.assertEqual(count, frames)
        finally:
            cap.release()

    def test_export_real_cv_and_account_every_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder)
            detector = FakeDetector()
            cap = cv2.VideoCapture(str(source))
            _, decoded = cap.read()
            cap.release()
            camera, _ = adapt_camera_params(load_camera_params(), 640, 360)
            corrector = create_corrector(camera)
            fixed = corrector.undistort(decoded)
            expected = fixed
            with patch.object(V, 'create_detector', return_value=detector):
                self.assertEqual(V.run_video(self.arguments(folder, source)), 0)
            self.assertEqual(len(detector.frames), 5)
            np.testing.assert_array_equal(detector.frames[0], expected)
            self.assert_video(folder/'runs'/'after.mp4', 5)
            records = self.records(folder/'logs'/'perf.log')
            self.assertEqual(records[0]['event'], 'run_start')
            self.assertEqual(records[-1]['event'], 'run_summary')
            self.assertEqual(records[-1]['frames'], 5)
            for mode in records[-1]['cv_modes'].values():
                self.assertEqual(mode['budget_ms'], 30.)
                self.assertLessEqual(mode['within_budget'], mode['n'])
                self.assertGreaterEqual(mode['within_budget_pct'], 0)
                self.assertLessEqual(mode['within_budget_pct'], 100)
            frames = [record for record in records if record['event'] == 'frame']
            self.assertEqual([frame['frame'] for frame in frames], list(range(1, 6)))
            self.assertEqual(frames[0]['cv_mode'], 'search')
            self.assertIn('track', {frame['cv_mode'] for frame in frames})
            for frame in frames:
                self.assertEqual(set(frame['timing_ms']), set(V.STAGES))
                self.assertTrue(all(value >= 0 for value in frame['timing_ms'].values()))
                self.assertAlmostEqual(sum(frame['timing_ms'].values()), frame['total_ms'], delta=.004)
                self.assertGreater(frame['timing_ms']['yolo'], 0)
                self.assertGreater(frame['details_ms']['video_write'], 0)
            rows = self.records(folder/'runs'/'rows.jsonl')
            self.assertEqual(len(rows), 5)
            self.assertIsNotNone(rows[0]['geometry'])
            self.assertLess(rows[0]['status']['cv_profile']['line_extraction_size'][0], 640)
            self.assertIn('encoder_finalized', [record['event'] for record in records])
            self.assertIn('warmup', [record['event'] for record in records])

    def test_cpu_encoder_fallback_no_target_frame_limit_and_log_append(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder)
            # Exercise the OpenCV encoder path even if FFmpeg is installed.
            with patch.object(V, 'create_detector', return_value=FakeDetector(empty=True)), \
                    patch('src.video_output.shutil.which', return_value=None):
                V.run_video(self.arguments(folder, source, '--no-correction', '--max-frames', '2'))
                self.assert_video(folder/'runs'/'after.mp4', 2)
                V.run_video(self.arguments(folder, source, '--no-correction', '--max-frames', '1'))
            records = self.records(folder/'logs'/'perf.log')
            self.assertEqual(len([r for r in records if r['event'] == 'run_start']), 2)
            self.assertEqual([r['frames'] for r in records if r['event'] == 'run_summary'], [2, 1])
            self.assertEqual({r['cv_mode'] for r in records if r['event'] == 'frame'}, {'idle'})

    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Host kernel checks require clang/clang++')
    def test_compiled_kernel_video_export_and_diagnostics(self):
        from opencl_host import HostRuntime
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder, frames=2)
            with patch('src.opencl_backend.OpenCLRuntime', HostRuntime), \
                    patch.object(V, 'create_detector', return_value=FakeDetector()):
                V.run_video(self.arguments(folder, source, '--cv-backend', 'opencl', '--cv-execution', 'hybrid'))
            self.assert_video(folder/'runs'/'after.mp4', 2)
            records = self.records(folder/'logs'/'perf.log')
            frames = [r for r in records if r['event'] == 'frame']
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[0]['cv_backend']['selected'], 'host-kernel-test')
            self.assertEqual(frames[0]['gpu']['device_type'], 'host')
            self.assertIn('hough_vote_compacted', frames[0]['gpu']['kernel_ms'])
            self.assertIn('fit_moments', frames[0]['gpu']['kernel_ms'])
            self.assertIn('remap_bgr', frames[0]['gpu']['kernel_ms'])
            self.assertIn('bgr_hsv', frames[0]['gpu']['kernel_ms'])
            self.assertNotIn('median_parameters', frames[0]['gpu']['kernel_ms'])
            self.assertNotIn('contrast_device', frames[0]['gpu']['kernel_ms'])
            self.assertNotIn('sharpen_only', frames[0]['gpu']['kernel_ms'])
            self.assertEqual(frames[0]['gpu']['enhancement_mode'], 'none')
            self.assertIn('trim_sections', frames[0]['gpu']['kernel_ms'])
            self.assertIn('trim_extract_fast', frames[0]['gpu']['kernel_ms'])
            self.assertIn('hough_runs_fast', frames[0]['gpu']['kernel_ms'])
            self.assertEqual(frames[0]['gpu']['cv_quality'], 'fast')
            self.assertNotIn('lines.lsd_gray', frames[0]['cv_profile']['stages_ms'])
            self.assertEqual(frames[0]['gpu']['blur_mode'], 'pyramid')
            self.assertIn('warmup', [r['event'] for r in records])
            self.assertGreater(frames[0]['gpu']['upload_bytes'], 0)
            self.assertIsNotNone(self.records(folder/'runs'/'rows.jsonl')[0]['geometry'])

    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Host kernel checks require clang/clang++')
    def test_resident_video_export_uses_final_records_and_one_image_output(self):
        from opencl_host import HostRuntime
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder, frames=2)
            with patch('src.opencl_backend.OpenCLRuntime', HostRuntime), \
                    patch.object(V, 'create_detector', return_value=FakeDetector()):
                V.run_video(self.arguments(folder, source, '--cv-backend', 'opencl'))
            self.assert_video(folder/'runs'/'after.mp4', 2)
            records = self.records(folder/'logs'/'perf.log')
            frames = [r for r in records if r['event'] == 'frame']
            for frame in frames:
                self.assertEqual(frame['gpu']['cv_execution'], 'resident')
                self.assertEqual(frame['gpu']['pipeline_version'], 7)
                self.assertEqual(frame['gpu']['cpu_stages'], [])
                self.assertEqual(frame['gpu']['peak_selector']['selected'], 'device-tiled-stable-sort-mask-greedy-nms')
                self.assertEqual(frame['gpu']['components']['connectivity'], 8)
                self.assertEqual(frame['gpu']['angle_peak_top']['selected'], 'exact-lane-top8-merge')
                self.assertIn('morph_packed4', frame['gpu']['kernel_ms'])
                self.assertIn('rg_cc_runs', frame['gpu']['kernel_ms'])
                self.assertIn('rg_peak_sort_merge', frame['gpu']['kernel_ms'])
                self.assertNotIn('morph3_fused', frame['gpu']['kernel_ms'])
                self.assertNotIn('rg_peak_order', frame['gpu']['kernel_ms'])
                self.assertNotIn('rg_cc_stats_hash', frame['gpu']['kernel_ms'])
                self.assertNotIn('morph3', frame['gpu']['kernel_ms'])
                self.assertNotIn('rg_cc_link', frame['gpu']['kernel_ms'])
                self.assertNotIn('rg_peak_top', frame['gpu']['kernel_ms'])
                self.assertEqual(frame['gpu']['download_bytes'], 640*360*3+800)
                self.assertEqual(frame['cv_profile']['timing_kind'], 'host-kernel-emulation')
                self.assertEqual(frame['cv_profile']['intermediate_readbacks'], 0)
                self.assertIn('components', frame['cv_profile']['stages_ms'])
                self.assertNotIn('contrast_device', frame['gpu']['kernel_ms'])
                self.assertNotIn('sharpen_only', frame['gpu']['kernel_ms'])
                self.assertEqual(frame['details_ms']['enhancement'], 0.)
                self.assertEqual(frame['gpu']['enhancement_mode'], 'none')
            self.assertTrue(all(row['geometry'] is not None for row in self.records(folder/'runs'/'rows.jsonl')))

    def test_gpu_startup_failure_logged_without_loading_yolo(self):
        from src.opencl_runtime import OpenCLError
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder, frames=1)
            with patch.object(V, 'create_backend', side_effect=OpenCLError('No Mali GPU')), \
                    patch.object(V, 'create_detector') as detector:
                with self.assertRaisesRegex(OpenCLError, 'No Mali GPU'):
                    V.run_video(self.arguments(folder, source, '--cv-backend', 'opencl'))
            detector.assert_not_called()
            records = self.records(folder/'logs'/'perf.log')
            self.assertEqual(records[-1]['event'], 'error')
            self.assertIn('No Mali GPU', records[-1]['error'])
            self.assertFalse((folder/'runs'/'after.mp4').exists())

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'FFmpeg is unavailable')
    def test_ffmpeg_reader_and_custom_output(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder, frames=2)
            output = folder/'custom'/'after.mp4'
            with patch.object(V, 'create_detector', return_value=FakeDetector(empty=True)):
                V.run_video(self.arguments(folder, source, '--reader', 'ffmpeg', '--output', str(output)))
            self.assert_video(output, 2)

    def test_failure_is_logged_and_input_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = self.source(folder)
            with patch.object(V, 'create_detector', return_value=FakeDetector(fail=True)):
                with self.assertRaisesRegex(RuntimeError, 'inference failed'):
                    V.run_video(self.arguments(folder, source))
            records = self.records(folder/'logs'/'perf.log')
            self.assertEqual(records[-1]['event'], 'error')
            self.assertEqual(records[-1]['completed_frames'], 0)
            original = source.read_bytes()
            with self.assertRaisesRegex(ValueError, 'paths must differ'):
                V.run_video(self.arguments(folder, source, '--output', str(source)))
            self.assertEqual(source.read_bytes(), original)

    def test_hbm_adapter_preserves_class_ids_and_passes_bgr_to_existing_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            args = V.parser().parse_args(['test.mp4', '--backend', 'hbm',
                '--weights', str(Path(directory)/'door.hbm'), '--class-names', 'ball', 'door'])
            captured = {}
            class Runtime:
                def detect(self, frame, nv12):
                    captured.update(frame=frame, nv12=nv12)
                    for stage in ('YOLO预处理', '推理', '后处理'):
                        with captured['timer'].measure(stage):
                            pass
                    return [dict(label='door', score=.9, bbox=(10, 20, 30, 40)),
                            dict(label='ball', score=.8, bbox=(1, 2, 3, 4))]
            def factory(cfg, timer):
                captured.update(cfg=cfg, timer=timer)
                return Runtime()
            with patch.object(V, '_board_detector_factory', side_effect=factory):
                detector = V.HbmBoxDetector(args)
            frame = np.zeros((64, 80, 3), np.uint8)
            boxes = detector.detect_boxes(frame)
            self.assertEqual(detector.target_ids, {1})
            self.assertEqual(boxes[0]['class_id'], 1)
            self.assertEqual(len(boxes), 1)
            self.assertIs(captured['frame'], frame)
            self.assertIsNone(captured['nv12'])
            self.assertEqual(captured['cfg']['target_class_ids'], [1])
            self.assertEqual(set(detector.timer.ms), {'YOLO预处理', '推理', '后处理'})

    def test_demo_entry_preserves_legacy_and_routes_optimized_pipeline(self):
        spec = importlib.util.spec_from_file_location('quad_demo_video', ROOT/'demo'/'demo_video.py')
        demo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(demo)
        with patch.object(demo, 'run_video', return_value=0) as legacy, \
                patch.object(V, 'run_video', return_value=0) as optimized:
            self.assertEqual(demo.main(['test.mp4']), 0)
            legacy.assert_called_once_with(['test.mp4'])
            self.assertEqual(demo.main(['--pipeline', 'red-gate', 'test.mp4', '--backend', 'hbm']), 0)
            optimized.assert_called_once_with(['test.mp4', '--backend', 'hbm'])

    def test_embedded_ffmpeg_without_x264_falls_back_and_preserves_odd_size(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'odd.mp4'
            with patch('src.video_output.shutil.which', return_value='/mock/ffmpeg'), \
                    patch('src.video_output.subprocess.run', return_value=MagicMock(stdout=' V..... mpeg4 encoder')), \
                    patch('src.video_output.subprocess.Popen') as encoder:
                writer = VideoOutput(output, 12., (161, 121))
                writer.write(np.full((121, 161, 3), 110, np.uint8))
                writer.close()
                encoder.assert_not_called()
            cap = cv2.VideoCapture(str(output))
            try:
                ok, frame = cap.read()
                self.assertTrue(ok)
                self.assertEqual(frame.shape, (122, 162, 3))
                self.assertFalse(cap.read()[0])
            finally:
                cap.release()

    def test_failed_ffmpeg_finalization_is_reported(self):
        encoder = MagicMock()
        encoder.wait.return_value = 1
        with patch('src.video_output.shutil.which', return_value='/mock/ffmpeg'), \
                patch('src.video_output.subprocess.run', return_value=MagicMock(stdout=' V....D libx264 H264')), \
                patch('src.video_output.subprocess.Popen', return_value=encoder):
            writer = VideoOutput(Path('/unused/after.mp4'), 12., (640, 360))
            writer.write(np.zeros((360, 640, 3), np.uint8))
            with self.assertRaisesRegex(RuntimeError, 'encoding failed'):
                writer.close()
            encoder.stdin.close.assert_called_once()
            encoder.wait.assert_called_once()
            writer.close()


if __name__ == '__main__':
    unittest.main()
