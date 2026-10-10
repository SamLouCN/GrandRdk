"""Sparse resident peak selection preserves the former greedy float32 policy."""
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


def reference(peaks, angles, radius, limit, suppressions=None):
    """Former exhaustive suppression, including quotas and stable vote ties."""
    ids = [i for i in range(2880) if peaks[i, 2] > 0]
    ids.sort(key=lambda i: (-int(peaks[i, 2]), int(peaks[i, 3]), i))
    normals = angles[peaks[:, 0]]
    offsets = np.float32(peaks[:, 1]-radius) - (
        normals[:, 0]*np.float32(319.5)+normals[:, 1]*np.float32(179.5))
    suppressed = np.zeros(2880, bool)
    counts, chosen = [0, 0], []
    for i in ids:
        horizontal = int(abs(normals[i, 0]) < abs(normals[i, 1])*np.float32(.7))
        if suppressed[i] or counts[horizontal] >= limit//2:
            continue
        chosen.append(i)
        counts[horizontal] += 1
        cosine = normals[:, 0]*normals[i, 0]+normals[:, 1]*normals[i, 1]
        distance = offsets*np.where(cosine < 0, np.float32(-1), np.float32(1))-offsets[i]
        if suppressions is None:
            suppressed |= (np.abs(cosine) > np.float32(.9993908)) & (np.abs(distance) < 4)
        else:
            js = np.arange(2880)
            suppressed |= ((suppressions[i, js//32] >> (js%32)) & 1).astype(bool)
        if len(chosen) == limit:
            break
    result = np.zeros((limit, 4), np.int32)
    result[:len(chosen)] = peaks[chosen]
    return result


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Production kernel harness requires clang')
class ResidentPeakTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            cls.backend = OpenCLBackend(quality='fast', blur_mode='pyramid')
        cls.pipeline = ResidentGatePipeline(cls.backend)
        cls.angles = cls.backend.angle_values[::2]
        cls.radius = 735

    @classmethod
    def tearDownClass(cls):
        cls.backend.close()

    def fixture(self):
        rows = np.zeros((2880, 4), np.int32)
        rows[:, 0] = np.repeat(np.arange(360), 8)
        rows[:, 3] = np.arange(2880)
        return rows

    def compare(self, rows, limit=256, search=True, compiled_reference=False):
        rt, pipeline = self.backend.runtime, self.pipeline
        pipeline.peak_limit = limit
        rt.reset_stats()
        control = np.zeros(32, np.float32)
        control[1] = search
        src = rt.upload('peak_fixture', rows)
        c = rt.upload('peak_control', control)
        output = rt.buffer('peak_fixture_output', limit*16)
        pipeline._select_peaks(src, output, c, self.radius)
        # Selection is entirely device-resident, even its sort/count metadata.
        self.assertEqual(rt.download_bytes, 0)
        actual = rt.read(output, (limit, 4), np.int32)
        suppressions = None
        if compiled_reference:
            masks = rt.buffer('test_exhaustive_peak_masks', 2880*90*4)
            rt.run('rg_test_peak_suppression', 2880, [src, self.backend.fast_angles, masks, self.radius])
            suppressions = rt.read(masks, (2880, 90), np.uint32)
            self.reference_suppressions = suppressions
        expected = reference(rows, self.angles, self.radius, limit, suppressions) if search else np.zeros((limit, 4), np.int32)
        np.testing.assert_array_equal(actual, expected)
        return actual

    def test_random_dense_clutter_vote_ties_and_both_orientation_quotas(self):
        rng = np.random.default_rng(617)
        rows = self.fixture()
        normal = self.angles[rows[:, 0]]
        eligible = (abs(normal[:, 1]) < abs(normal[:, 0])*.65) | (abs(normal[:, 0]) < abs(normal[:, 1])*.7)
        rows[:, 1] = rng.integers(0, 1471, 2880)
        rows[:, 2] = np.where(eligible, rng.integers(18, 25, 2880), 0)
        for limit in (16, 256):
            with self.subTest(limit=limit):
                selected = self.compare(rows, limit)
                normals = self.angles[selected[:, 0]]
                horizontal = abs(normals[:, 0]) < abs(normals[:, 1])*.7
                self.assertEqual(int(horizontal.sum()), limit//2)
                self.assertTrue(np.all(selected[:, 2] > 0))

    def test_suppression_chain_keeps_peak_rejected_neighbor_would_suppress(self):
        rows = self.fixture()
        rows[:3, 1] = [self.radius+300, self.radius+303, self.radius+306]
        rows[:3, 2] = [100, 90, 80]
        selected = self.compare(rows)
        np.testing.assert_array_equal(selected[:2], rows[[0, 2]])

    def test_all_bitset_alignments_and_circular_word_wrap(self):
        rows = self.fixture()
        rng = np.random.default_rng(843)
        normal = self.angles[rows[:, 0]]
        rows[:, 1] = np.rint(self.radius+normal[:, 0]*np.float32(319.5)+normal[:, 1]*np.float32(179.5))
        rows[:, 2] = rng.integers(18, 99, 2880)
        self.compare(rows, compiled_reference=True)
        rt, pipeline = self.backend.runtime, self.pipeline
        masks = rt.read(pipeline.buffer('peak_masks', 2880*16), (2880, 4), np.uint32)
        # Verify each compressed record, including all four 8-slot alignments,
        # against exhaustive suppression on the full 0/pi circular angle grid.
        for i in range(2880):
            actual = np.zeros(90, np.uint32)
            base = int(masks[i, 0])
            for k in range(3):
                word = (base+k)%90
                actual[word] = masks[i, k+1]
            np.testing.assert_array_equal(actual, self.reference_suppressions[i], err_msg=f'peak {i}')

    def test_angle_wraparound_threshold_neighbors_empty_and_skipped_search(self):
        rows = self.fixture()
        angles = [0, 1, 3, 4, 5, 6, 354, 355, 356, 357, 359, 175, 179, 180, 184, 185]
        for a in angles:
            for k in range(8):
                i = a*8+k
                normal = self.angles[a]
                offset = k*3-10
                rows[i, 1] = round(self.radius+normal[0]*319.5+normal[1]*179.5+offset)
                rows[i, 2] = 100
                rows[i, 3] = a*1471+rows[i, 1]
        self.compare(rows)
        self.compare(rows, search=False)
        self.compare(self.fixture())
