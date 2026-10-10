"""Execute production search kernels on the host, checking CPU equivalence.

Host execution proves arithmetic/ordering, not Mali throughput or driver behavior.
"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opencl_host import HostRuntime
from src import detect_red_gate as D, gate_models as M
from src.cv_profile import CvFrameProfile
from src.gate_line_geometry import endpoints
from src.opencl_backend import OpenCLBackend
from src.opencl_validation import check_batched_search, benchmark_search, check_parallel_sides


class GPUSearchTests(unittest.TestCase):
    def setUp(self):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            self.backend = OpenCLBackend(quality='fast')
        self.addCleanup(self.backend.close)

    def test_split_contrast_matches_reference_in_both_quality_modes(self):
        check_batched_search(self.backend)

    def test_parallel_side_sampling_matches_serial_on_gaps_borders_and_reversed_edges(self):
        check_parallel_sides(self.backend)

    def test_gpu_geometry_matches_cpu_for_valid_missing_and_invalid_edges(self):
        frame = np.full((180, 260, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (30, 25), (225, 155), (50, 60, 230), 7)
        mask = M.combined_evidence(frame, frame)[0]
        lines = [D.line_from_segment(mask, np.asarray(seed, float), .48)
                 for seed in ((30, 25, 225, 25), (225, 25, 225, 155),
                              (30, 155, 225, 155), (30, 25, 30, 155))]
        self.assertTrue(all(line is not None for line in lines))
        vertical = [line for line in lines if line['vertical']]
        horizontal = [line for line in lines if not line['vertical']]
        for missing in (False, True):
            source = mask.copy()
            if missing:
                source[149:162, 25:231] = 0
            for invalid in (False, True):
                valid = np.ones(mask.shape, bool)
                if invalid:
                    valid[:36, :45] = False
                expected = M.complete_models(vertical, horizontal, source, (0, 0, 260, 180), valid)
                actual = M.complete_models(vertical, horizontal, source, (0, 0, 260, 180), valid,
                                           backend=self.backend)
                self.assertEqual(len(actual), len(expected))
                for cpu, gpu in zip(expected, actual):
                    np.testing.assert_allclose(gpu['quad'], cpu['quad'], atol=.01, rtol=0)
                    np.testing.assert_allclose(gpu['side_support'], cpu['side_support'], atol=.002, rtol=0)
        self.assertEqual(M.complete_models(vertical[:1], horizontal, mask, backend=self.backend), [])

    def test_merge_expanding_short_seeds_sizes_samples_for_final_span(self):
        frame = np.full((120, 260, 3), (120, 90, 40), np.uint8)
        cv2.line(frame, (10, 60), (245, 60), (50, 60, 230), 5)
        mask = M.combined_evidence(frame, frame)[0]
        lines = [D.line_from_segment(mask, np.asarray([x, 60, x+45, 60], float), .48)
                 for x in range(10, 211, 40)]
        actual = self.backend.merge_split_lines(lines, mask, frame, frame, CvFrameProfile(True))[1]
        self.assertEqual(len(actual), 1)
        self.assertGreater(actual[0]['hi']-actual[0]['lo'], 230)
        self.assertGreater(actual[0]['contrast'], .035)

    def test_search_input_reuses_within_batch_and_refreshes_mutable_next_frame(self):
        mask = np.zeros((12, 19), np.uint8)
        rt = self.backend.runtime
        with self.backend.search_batch():
            first = self.backend.search_input('mask', mask, np.uint8)
            before = rt.upload_bytes
            second = self.backend.search_input('mask', mask, np.uint8)
            self.assertEqual(first.handle, second.handle)
            self.assertEqual(before, rt.upload_bytes)
        mask[:] = 255
        with self.backend.search_batch():
            result = self.backend.search_input('mask', mask, np.uint8)
            np.testing.assert_array_equal(rt.read(result, mask.shape, np.uint8), mask)

    def test_partial_joint_and_color_batches_preserve_partial_selection(self):
        frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        rods = [((-8, 55), (535, 55)), ((535, 55), (535, 359)),
                ((40, 240), (130, 240))]
        for a, b in rods:
            cv2.line(frame, a, b, (50, 60, 230), 9)
        mask = M.combined_evidence(frame, frame)[0]
        lines = [D.line_from_segment(mask, np.asarray((*a, *b), float), .48) for a, b in rods]
        vertical = [line for line in lines if line['vertical']]
        horizontal = [line for line in lines if not line['vertical']]
        ordered = vertical+horizontal
        colors, _ = self.backend.partial_measurements(ordered, len(vertical), frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for line, actual in zip(ordered, colors):
            expected = []
            for count in (60, 80):
                xy = np.rint(np.linspace(*endpoints(line), count)).astype(int)
                valid = (xy[:, 0]>=0)&(xy[:, 0]<640)&(xy[:, 1]>=0)&(xy[:, 1]<360)
                xy = xy[valid]
                pixels = frame[xy[:, 1], xy[:, 0]].astype(float)
                expected.append(float(np.median(np.log((pixels[:, 2]+10)/(pixels[:, 1]+10)))))
                if count == 80:
                    samples = hsv[xy[:, 1], xy[:, 0]]
                    expected.append(float(np.mean(((samples[:, 0]<30)|(samples[:, 0]>143))&(samples[:, 1]>35))))
            np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=0)
        expected = D.pipe_groups(vertical, horizontal, frame)
        actual = D.pipe_groups(vertical, horizontal, frame, backend=self.backend)
        self.assertEqual(len(actual), len(expected))
        for cpu, gpu in zip(expected, actual):
            self.assertEqual(len(cpu['lines']), len(gpu['lines']))
            np.testing.assert_allclose(gpu['segments'], cpu['segments'], atol=.01, rtol=0)
            self.assertEqual(gpu['score'], cpu['score'])

    def test_model_pipeline_uploads_mask_once_and_reports_gpu_work(self):
        # Four long rods form several rejected combinations too. Geometry and
        # support do not download intermediate corners or per-side samples.
        frame = np.full((120, 190, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (20, 20), (165, 100), (50, 60, 230), 7)
        mask = M.combined_evidence(frame, frame)[0]
        lines = [D.line_from_segment(mask, np.asarray(seed, float), .48)
                 for seed in ((20, 20, 165, 20), (165, 20, 165, 100),
                              (20, 100, 165, 100), (20, 20, 20, 100))]
        profile = CvFrameProfile(True)
        with self.backend.search_batch():
            self.backend.search_input('mask', mask, np.uint8)
            before = self.backend.runtime.upload_bytes
            models = M.complete_models([lines[1], lines[3]], [lines[0], lines[2]], mask,
                (0, 0, 190, 120), profile=profile, backend=self.backend)
            # Four packed line records + one pair + disabled valid-mask byte.
            self.assertEqual(self.backend.runtime.upload_bytes-before, 4*12*4+4*4+1)
        self.assertEqual(len(models), 1)
        self.assertEqual(profile.meta['model_execution'], 'gpu-batched')
        self.assertIn('search_quads', self.backend.runtime.kernel_ms)

    def test_board_comparison_runs_previous_and_new_search_paths(self):
        # Host timings are deliberately not asserted or reported as GPU speed.
        self.backend.hough_backend = 'cpu'
        records = []
        summaries = benchmark_search(self.backend, frames=1, warmup=0, report=records.append)
        self.assertEqual(len(summaries), 2)
        frames = [item for item in records if item['event'] == 'search_benchmark_frame']
        self.assertEqual(len(frames), 2)
        self.assertNotIn('search_merge', frames[0]['gpu']['kernel_ms'])
        self.assertIn('search_merge', frames[1]['gpu']['kernel_ms'])
        self.assertNotIn('search_quads', frames[0]['gpu']['kernel_ms'])
        self.assertIn('search_quads', frames[1]['gpu']['kernel_ms'])
        self.assertTrue(all(item['selected'] is not None for item in summaries))
        np.testing.assert_allclose(summaries[0]['selected']['bbox'], summaries[1]['selected']['bbox'], atol=2)

    def test_tracking_batch_preserves_occupancy_and_side_acceptance(self):
        mask = np.zeros((180, 260), np.uint8)
        rods = [((12, 25), (238, 25)), ((25, 60), (25, 161)),
                ((70, 90), (231, 110)), ((238, 140), (30, 140))]
        for a, b in rods:
            cv2.line(mask, a, b, 255, 7)
        mask[18:33, 95:125] = 0
        mask[75:95, 20:31] = 0
        lines = [D.line_from_segment(mask, np.asarray((*a, *b), float), .48) for a, b in rods]
        segments = [endpoints(line) for line in lines]
        occupancy, support = self.backend.track_support(mask, lines, segments, True)
        from src.gate_line_geometry import samples
        for line, segment, actual, side in zip(lines, segments, occupancy, support):
            expected = samples(mask, *segment, max(4, int(line['width']*.6)))[1].any(axis=1).mean()
            self.assertAlmostEqual(float(actual), expected, delta=.005)
            cpu_side = M.side_evidence(mask, line, segment)
            self.assertEqual(side >= 0, cpu_side is not None)
            if cpu_side is not None:
                self.assertAlmostEqual(float(side), cpu_side, delta=.005)


if __name__ == '__main__':
    unittest.main()
