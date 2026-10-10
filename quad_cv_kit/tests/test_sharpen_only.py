"""Sharpening changes local detail, preserving flat brightness and colour."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.gate_guidance import sharpen_frame


class SharpenOnlyTests(unittest.TestCase):
    def test_constant_colours_preserve_brightness_even_at_invalid_boundaries(self):
        for colour in ((30, 80, 120), (50, 60, 230), (120, 120, 120)):
            frame = np.full((41, 73, 3), colour, np.uint8)
            valid = np.ones(frame.shape[:2], bool)
            valid[:7] = False
            frame[:7] = 0
            np.testing.assert_array_equal(sharpen_frame(frame, 2, valid), frame)

    def test_local_edge_sharpens_and_all_channels_keep_original_ratios(self):
        frame = np.full((41, 73, 3), (40, 80, 160), np.uint8)
        frame[:, 35:] = (50, 100, 200)
        result = sharpen_frame(frame, .6)
        self.assertLess(int(result[20, 34, 2]), 160)
        self.assertGreater(int(result[20, 35, 2]), 200)
        np.testing.assert_array_equal(result[:, :20], frame[:, :20])
        np.testing.assert_array_equal(result[:, 50:], frame[:, 50:])
        np.testing.assert_allclose(result[:, :, 1], result[:, :, 2]/2, atol=.5, rtol=0)

    def test_zero_strength_and_all_invalid_return_original_pixels(self):
        rng = np.random.default_rng(41)
        frame = rng.integers(0, 256, (17, 33, 3), dtype=np.uint8)
        np.testing.assert_array_equal(sharpen_frame(frame, 0), frame)
        np.testing.assert_array_equal(sharpen_frame(frame, 2, np.zeros(frame.shape[:2], bool)), frame)
        for amount in (-1, 3, float('nan')):
            with self.assertRaises(ValueError):
                sharpen_frame(frame, amount)
