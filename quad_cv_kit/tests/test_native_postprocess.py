"""Native joins preserve ordered identity, mutation and threshold decisions."""
import copy
from pathlib import Path
import shutil
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.gate_models import merge_trimmed
from src.native_postprocess import NativePostprocess


@unittest.skipUnless(shutil.which('cc'), 'Native merge checks require a C compiler')
class NativePostprocessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = NativePostprocess()

    def compare(self, rows):
        first, second = copy.deepcopy(rows), copy.deepcopy(rows)
        expected = merge_trimmed(first)
        actual = self.native.merge_trimmed(second)
        self.assertEqual([line['index'] for line in actual], [line['index'] for line in expected])
        for cpu, native in zip(expected, actual):
            for field in ('lo', 'hi', 'strong_lo', 'strong_hi'):
                self.assertAlmostEqual(cpu[field], native[field], delta=1e-12)
        self.assertTrue(all(any(line is original for original in second) for line in actual))

    def line(self, index, angle, offset, low, high, width=5):
        d = np.array([np.cos(angle), np.sin(angle)])
        return dict(index=index, d=d, n=np.array([-d[1], d[0]]), b=offset,
                    lo=low, hi=high, width=width, support=.85,
                    vertical=abs(d[0]) < abs(d[1])*.65, strong_lo=low+2, strong_hi=high-2)

    def test_random_overlaps_and_directions_preserve_first_match_order(self):
        rng = np.random.default_rng(902)
        for size in (0, 1, 10, 70, 200):
            rows = [self.line(i, (i%2)*np.pi/2+rng.uniform(-.04, .04),
                              rng.choice([40, 80, 120])+rng.uniform(-3, 3),
                              low := rng.uniform(0, 300), low+rng.uniform(24, 100),
                              rng.uniform(3, 15)) for i in range(size)]
            self.compare(rows)

    def test_gap_distance_and_angle_thresholds_and_reversed_rods(self):
        for gap in (19.999999, 20, 20.000001):
            for distance in (1.999999, 2, 2.000001):
                rows = [self.line(0, 0, 40, 10, 100, 3),
                        self.line(1, 0, 40+distance, 100+gap, 180+gap, 3),
                        self.line(2, np.pi, -40, -180, -20, 3)]
                self.compare(rows)
