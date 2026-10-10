"""Resident integer stages: exact borders, connectivity, statistics and peak ties."""
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
from src.opencl_runtime import LocalMemory
from src.resident_accuracy import check_morphology, check_components, check_angle_peaks, check_peak_order, legacy_morph


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Production kernel harness requires clang')
class ResidentPrimitiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            cls.backend = OpenCLBackend(quality='fast', blur_mode='pyramid')

    @classmethod
    def tearDownClass(cls):
        cls.backend.close()

    def test_all_fused_pass_combinations_match_original_bytes_at_borders(self):
        self.assertTrue(check_morphology(self.backend, exhaustive=True)['passed'])

    def test_labels_stats_filtering_and_diagonal_tile_boundaries_are_exact(self):
        # Repeated real thread interleavings exercise competing root links;
        # atomically overwriting an already-linked root would silently split it.
        for _ in range(12):
            self.assertTrue(check_components(self.backend)['passed'])

    def test_all_angle_top8_records_match_old_kernel_for_ties_and_empty(self):
        self.assertTrue(check_angle_peaks(self.backend)['passed'])

    def test_board_self_check_runs_precision_checks_and_isolates_validation_readbacks(self):
        from src.resident_validation import check_resident
        result = check_resident(self.backend)
        self.assertTrue(result['passed'])
        self.assertEqual(result['final_output_bytes'], 800)
        self.assertEqual(len(result['primitive_checks']), 4)
        self.assertTrue(all(check['passed'] for check in result['primitive_checks']))
        self.assertEqual(self.backend.runtime.download_bytes, 800)

    def test_large_roi_and_valid_mask_edges_morphology(self):
        rt = self.backend.runtime
        rt.reset_stats()
        mask = np.zeros((360, 640), np.uint8)
        rng = np.random.default_rng(6010)
        mask[13:285, 127:577] = np.uint8(rng.random((272, 450)) > .45)*255
        mask[0, :] = mask[-1, :] = mask[:, 0] = mask[:, -1] = 255
        source = rt.upload('morph_large_source', mask)
        for steps in ((True, False), (True, False, False, True)):
            actual = self.backend._morph(source, 640, 360, steps, 'morph_large_fused', binary=True)
            expected = legacy_morph(self.backend, source, 640, 360, steps, 'morph_large_old')
            self.assertEqual(rt.download_bytes, 0)
            np.testing.assert_array_equal(rt.read(actual, mask.shape, np.uint8), rt.read(expected, mask.shape, np.uint8))
            rt.reset_stats()

    def test_complete_peak_order_including_keys_slots_padding_and_skipped_search(self):
        self.assertTrue(check_peak_order(self.backend)['passed'])

    def test_packed_word_tile_boundaries_and_every_pass_combination(self):
        import itertools
        rt = self.backend.runtime
        rng = np.random.default_rng(7010)
        for w in (31, 32, 33, 255, 256, 257, 511, 513, 641):
            mask = np.uint8(rng.random((17, w)) > .58)*255
            mask[0, :] = mask[-1, :] = mask[:, 0] = mask[:, -1] = 255
            source = rt.upload('packed_boundary_source', mask)
            for length in (1, 2, 3, 4):
                for steps in itertools.product((False, True), repeat=length):
                    actual = self.backend._morph(source, w, 17, steps, 'packed_boundary', binary=True)
                    expected = legacy_morph(self.backend, source, w, 17, steps, 'packed_boundary_old')
                    np.testing.assert_array_equal(rt.read(actual, mask.shape, np.uint8),
                                                  rt.read(expected, mask.shape, np.uint8),
                                                  err_msg=f'packed width={w} steps={steps}')

    def test_hash_reduction_never_drops_collisions_or_threshold_boundary_components(self):
        rt = self.backend.runtime
        rng = np.random.default_rng(6011)
        pipeline = ResidentGatePipeline(self.backend)
        pipeline.width, pipeline.height = 81, 49
        mask = np.uint8(rng.random((49, 81)) > .9)*255
        mask[2:9, 1:6] = 255  # exactly 35 pixels, exact extent boundary
        mask[2:9, 7:12] = 255
        mask[10:12, 14:39] = 255
        control = rt.upload('hash_control', np.array([0, 1], np.float32))
        source = rt.upload('hash_source', mask)
        for minimum, extent, both in ((35, 25, False), (35, 7, False), (35, 5, True), (1, 1, False)):
            result = pipeline._cc(source, 'hash', minimum, extent, control, both)
            import cv2
            _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            size = np.minimum(stats[:, 2], stats[:, 3]) if both else np.maximum(stats[:, 2], stats[:, 3])
            keep = (stats[:, 4] >= minimum) & (size >= extent);keep[0] = False
            np.testing.assert_array_equal(rt.read(result, mask.shape, np.uint8), np.uint8(keep[labels])*255)
        # Explicitly force 256 unique roots into a 512-entry hash table; some
        # buckets collide, and all component statistics must still be preserved.
        n = 256
        roots = np.arange(n, dtype=np.int32)
        stats = np.tile(np.array([0, n, 1, -1, -1], np.int32), (n, 1))
        src = rt.upload('hash_isolated_source', np.full(n, 255, np.uint8))
        parent, measure = rt.upload('hash_isolated_roots', roots), rt.upload('hash_isolated_stats', stats)
        rt.run('rg_cc_stats_hash', 64, [src, parent, measure, n, 1, control, LocalMemory(512*6*4)], local=64)
        expected = np.column_stack((np.ones(n, np.int32), roots, np.zeros(n, np.int32), roots, np.zeros(n, np.int32)))
        np.testing.assert_array_equal(rt.read(measure, (n, 5), np.int32), expected)

    def test_complete_partial_weak_nested_and_tracking_match_v5_outputs(self):
        from test_resident_pipeline import scene
        import cv2

        class LegacyPipeline(ResidentGatePipeline):
            def _cc(self, source, tag, minimum, extent, control, both=False):
                n, rt = self.width*self.height, self.rt
                parents, stats = self.buffer(tag+'_parents', n*4), self.buffer(tag+'_stats', n*20)
                out = self.buffer(tag+'_filtered', n)
                rt.run('rg_cc_init', n, [source, parents, stats, n, control])
                rt.run('rg_cc_link', n, [source, parents, self.width, self.height, control])
                rt.run('rg_cc_stats', ((n+255)//256)*64,
                       [source, parents, stats, self.width, self.height, LocalMemory(256*4), control], local=64)
                rt.run('rg_cc_filter', n, [source, parents, stats, out, n, minimum, extent, int(both), control])
                return out

            def _peak_top(self, votes, peaks, control, nrhos, threshold):
                self.rt.run('rg_peak_top', 360*64, [votes, peaks, control, self.backend.fast_angles,
                            nrhos, threshold, LocalMemory(64*4), LocalMemory(64*4)], local=64)

        backend, rt = self.backend, self.backend.runtime
        weak, nested, elbows = scene(), scene(), scene()
        weak[np.all(weak == (50, 60, 230), axis=2)] = (50, 60, 95)
        cv2.rectangle(nested, (225, 115), (415, 245), (50, 60, 230), 5)
        for x, y in ((170, 70), (470, 70), (470, 290), (170, 290)):
            cv2.rectangle(elbows, (x-9, y-9), (x+9, y+9), (235, 235, 235), -1)
        for tag, frame in (('complete', scene()), ('partial', scene(partial=True)), ('weak', weak),
                           ('nested', nested), ('elbows', elbows), ('translated', scene(3, 2))):
            with self.subTest(scene=tag):
                # Translation follows a complete-frame observation to exercise
                # tracking and subsequent scheduled search with identical state.
                modern, legacy = ResidentGatePipeline(backend), LegacyPipeline(backend)
                frames = [scene(), frame] if tag == 'translated' else [frame]
                for image in frames:
                    boxes = [dict(class_id=0, score=.9, bbox=[155, 55, 485, 305])]
                    rt.reset_stats()
                    with backend.frame_batch():
                        modern.update(image, image, boxes, {0})
                    self.assertEqual(rt.download_bytes, 800)
                    expected_state = rt.read(modern.state, (128,), np.float32)
                    rt.reset_stats()
                    with patch.object(backend, '_morph', side_effect=lambda *args, **kwargs: legacy_morph(backend, *args)):
                        with backend.frame_batch():
                            legacy.update(image, image, boxes, {0})
                    self.assertEqual(rt.download_bytes, 800)
                    np.testing.assert_array_equal(rt.read(legacy.state, (128,), np.float32), expected_state)
                    self.assertEqual(legacy.last_status['observation'], modern.last_status['observation'])
