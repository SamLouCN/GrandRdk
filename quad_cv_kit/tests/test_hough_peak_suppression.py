"""Angle indexing must preserve the former exhaustive peak suppression."""
from pathlib import Path
import sys
import shutil
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.opencl_backend import OpenCLBackend


def exhaustive_peaks(backend, rows, radius, w, h):
    normals = backend.angle_values[::2][rows[:, 0]].astype(float)
    vertical = np.abs(normals[:, 1]) < np.abs(normals[:, 0])*.65
    horizontal = np.abs(normals[:, 0]) < np.abs(normals[:, 1])*.7
    ids = np.flatnonzero(vertical | horizontal)
    ids = ids[np.lexsort((rows[ids, 1], rows[ids, 0], -rows[ids, 2]))]
    ids = ids[(vertical[ids] & (np.cumsum(vertical[ids]) <= 2048)) |
              (horizontal[ids] & (np.cumsum(horizontal[ids]) <= 2048))]
    rows, normals, horizontal = rows[ids], normals[ids], horizontal[ids]
    offsets = rows[:, 1]-radius-normals[:, 0]*((w-1)/2)-normals[:, 1]*((h-1)/2)
    selected, counts = [], [0, 0]
    suppressed = np.zeros(len(rows), bool)
    for i in range(len(rows)):
        orientation = int(horizontal[i])
        if suppressed[i] or counts[orientation] >= backend.hough_peak_limit//2:
            continue
        selected.append(i)
        counts[orientation] += 1
        dot = normals[:, 0]*normals[i, 0]+normals[:, 1]*normals[i, 1]
        suppressed |= ((np.abs(dot) > np.cos(np.deg2rad(2))) &
                       (np.abs(offsets*np.where(dot < 0, -1, 1)-offsets[i]) < 4))
        if len(selected) >= backend.hough_peak_limit:
            break
    return np.ascontiguousarray(rows[selected])


class PeakSuppressionTests(unittest.TestCase):
    def setUp(self):
        self.backend = OpenCLBackend.__new__(OpenCLBackend)
        angles = np.arange(720)*np.pi/720
        self.backend.angle_values = np.float32(np.column_stack([np.cos(angles), np.sin(angles)]))
        self.backend.work_counts = {}
        self.backend.hough_peak_limit = 256

    def compare(self, rows, w=500, h=355):
        radius = int(np.ceil(np.hypot(w-1, h-1)))+1
        expected = exhaustive_peaks(self.backend, rows, radius, w, h)
        actual = self.backend._diverse_peaks(rows, radius, w, h)
        np.testing.assert_array_equal(actual, expected)

    def test_random_clutter_and_orientation_quotas_match_exactly(self):
        rng = np.random.default_rng(710)
        for count in (0, 1, 50, 2473, 6000, 12000):
            for w, h in ((270, 210), (500, 355)):
                rows = np.column_stack((rng.integers(0, 360, count), rng.integers(0, 1200, count),
                                        rng.integers(18, 160, count), np.arange(count))).astype(np.int32)
                with self.subTest(count=count, size=(w, h)):
                    self.compare(rows, w, h)

    def test_wraparound_threshold_neighbors_and_tied_votes_match(self):
        angles = np.array([0, 1, 3, 4, 5, 6, 354, 355, 356, 357, 359,
                           175, 176, 177, 179, 180, 181, 183, 184, 185])
        rows = np.array([[a, r, 100, i] for i, (a, r) in enumerate(
            (a, r) for a in angles for r in range(540, 552))], np.int32)
        self.compare(rows)
        self.backend.hough_peak_limit = 16
        self.compare(rows)

    @unittest.skipUnless(shutil.which('cc'), 'Native peak checks require a C compiler')
    def test_native_float64_preserves_exhaustive_outputs(self):
        from src.native_peak_selection import NativePeakSelector
        self.backend._native_peak_selector = NativePeakSelector()
        self.test_random_clutter_and_orientation_quotas_match_exactly()
        self.test_wraparound_threshold_neighbors_and_tied_votes_match()


if __name__ == '__main__':
    unittest.main()
