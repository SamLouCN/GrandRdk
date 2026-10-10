"""Production OpenCL TopK parity with the former float32 stable ranking."""
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opencl_host import HostRuntime
from src.gpu_pipeline import ResidentGatePipeline
from src.opencl_backend import OpenCLBackend


def reference(lines, capacity, per_orientation=False):
    scores = (lines[:, 4]-lines[:, 3])*np.sqrt(np.maximum(lines[:, 5], np.float32(0)))*lines[:, 6]
    result = np.zeros((capacity, 12), np.float32)
    for orientation in ((1, 0) if per_orientation else (None,)):
        ids = [i for i, row in enumerate(lines) if row[11] and (orientation is None or row[7] == orientation)]
        ids.sort(key=lambda i: (-float(scores[i]), i))
        limit = capacity//2 if per_orientation else capacity
        offset = limit if orientation == 0 else 0
        chosen = ids[:limit]
        result[offset:offset+len(chosen)] = lines[chosen]
    return result


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Production kernel harness requires clang')
class ResidentRankTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            cls.backend = OpenCLBackend(quality='fast', blur_mode='pyramid')
        cls.pipeline = ResidentGatePipeline(cls.backend)

    @classmethod
    def tearDownClass(cls):
        cls.backend.close()

    def fixture(self, n, seed=361):
        rng = np.random.default_rng(seed)
        lines = rng.normal(size=(n, 12)).astype(np.float32)
        lines[:, 3] = rng.uniform(-100, 100, n)
        lines[:, 4] = lines[:, 3]+rng.uniform(35, 700, n)
        lines[:, 5] = rng.uniform(.5, 20, n)
        lines[:, 6] = rng.uniform(.5, 1, n)
        lines[:, 7] = rng.integers(0, 2, n)
        lines[:, 10] = np.arange(n)
        lines[:, 11] = rng.random(n) > .55
        return lines

    def compare(self, lines, capacity=128, per_orientation=False, search=True):
        rt = self.backend.runtime
        rt.reset_stats()
        source = rt.upload('rank_fixture', lines)
        control = np.zeros(32, np.float32)
        control[1] = search
        control = rt.upload('rank_fixture_control', control)
        out = self.pipeline.floats('rank_fixture_output', capacity*12, clear=True)
        self.pipeline._rank_lines(source, out, len(lines), capacity, control, per_orientation)
        self.assertEqual(rt.download_bytes, 0)
        actual = rt.read(out, (capacity, 12), np.float32)
        expected = reference(lines, capacity, per_orientation) if search else np.zeros((capacity, 12), np.float32)
        np.testing.assert_array_equal(actual, expected)
        return actual

    def test_dense_full_capacity_and_original_score_formula(self):
        lines = self.fixture(5632)
        lines[:, 11] = 1
        self.compare(lines)

    def test_empty_single_tile_odd_merge_levels_and_sparse_invalid_rows(self):
        for n in (0, 1, 255, 256, 257, 513, 5632):
            with self.subTest(n=n):
                self.compare(self.fixture(n))
        # Reusing all scratch/output buffers after a dense call cannot retain rows.
        self.compare(np.zeros((5632, 12), np.float32))
        self.compare(self.fixture(5632), search=False)

    def test_stable_ties_across_tiles_and_short_non_power_two_topk(self):
        lines = self.fixture(1301)
        lines[:, 3:7] = [0, 80, 9, .75]
        for capacity in (1, 7, 16, 128, 256):
            with self.subTest(capacity=capacity):
                self.compare(lines, capacity)

    def test_final_orientation_quotas_ties_and_missing_orientation(self):
        lines = self.fixture(128)
        self.compare(lines, 16, True)
        self.compare(lines, 16, True, search=False)
        lines[:, 3:7] = [0, 80, 9, .75]
        self.compare(lines, 16, True)
        lines[:, 7] = 1
        self.compare(lines, 16, True)
        lines[:, 11] = 0
        self.compare(lines, 16, True)
