"""Execute production OpenCL source on host; never present this as a GPU test."""
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opencl_host import HostRuntime
from src.opencl_backend import OpenCLBackend, create_backend
from src.opencl_runtime import OpenCLError
from src.opencl_validation import CHECKS


@unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'Host kernel checks require clang/clang++')
class CompiledKernelTests(unittest.TestCase):
    def setUp(self):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            self.backend = OpenCLBackend()
        self.addCleanup(self.backend.close)

    def test_blur(self):
        CHECKS['blur'](self.backend)

    def test_color(self):
        CHECKS['color'](self.backend)

    def test_fused_color_blur_and_gpu_tracking_join(self):
        CHECKS['gpu_pipeline'](self.backend)

    def test_enhancement(self):
        CHECKS['enhancement'](self.backend)

    def test_sharpen_only_fused_kernel_and_invalid_border_handling(self):
        CHECKS['sharpen_only'](self.backend)

    def test_remap(self):
        CHECKS['remap'](self.backend)

    def test_fitting(self):
        CHECKS['fitting'](self.backend)

    def test_hough(self):
        CHECKS['hough'](self.backend)

    def test_geometry(self):
        CHECKS['geometry'](self.backend)

    def test_resident_preprocess_and_gpu_color_conversion(self):
        CHECKS['preprocess'](self.backend)

    def test_gpu_trim_and_side_support(self):
        CHECKS['trim_and_sides'](self.backend)

    def test_batched_search_preserves_reference_acceptance(self):
        CHECKS['batched_search'](self.backend)

    def test_pyramid_approximation_has_bounded_fixture_drift_and_valid_geometry(self):
        from src.opencl_validation import check_pyramid
        check_pyramid(self.backend)

    def test_validator_reports_exact_and_approximate_checks_separately(self):
        from src import opencl_validation as V
        self.backend.blur_mode = 'pyramid'
        seen = []
        def probe(backend):
            seen.append(backend.blur_mode)
            return {}
        with patch.object(V, 'CHECKS', {'exact_probe': probe}), patch.object(V, 'check_pyramid', probe):
            results = V.validate_backend(self.backend)
        self.assertEqual(seen, ['exact', 'pyramid'])
        self.assertEqual([r['gpu']['blur_mode'] for r in results], seen)
        self.assertEqual(self.backend.blur_mode, 'pyramid')

    def test_pyramid_and_gpu_sampling_preserve_gate_evidence_scenarios(self):
        from test_gate_models import GateModelTests
        from src import detect_red_gate as D
        self.backend.blur_mode = 'pyramid'
        original = D.detect
        scenarios = GateModelTests()
        names = ('test_elbow_gap_without_white_segmentation_still_forms_four_corners',
                 'test_faded_top_still_supports_measured_four_line_model',
                 'test_missing_fourth_side_is_not_manufactured',
                 'test_white_support_legs_do_not_extend_gate_bottom',
                 'test_excessive_end_gaps_are_rejected',
                 'test_offset_overlapping_gates_keep_near_identity',
                 'test_short_red_patch_on_white_foot_does_not_hide_near_partial_gate',
                 'test_valid_mask_and_roi_reject_corner_outside_observation')
        def accelerated(*args, **kwargs):
            kwargs['backend'] = self.backend
            return original(*args, **kwargs)
        with patch.object(D, 'detect', side_effect=accelerated):
            for name in names:
                with self.subTest(scene=name):
                    getattr(scenarios, name)()

    def test_parallel_greedy_consumption_matches_serial(self):
        CHECKS['cooperative_selection'](self.backend)

    def test_fast_hough_and_parallel_trim_preserve_supported_geometry(self):
        from src.opencl_validation import check_fast
        check_fast(self.backend)

    def test_fast_hough_bounds_clutter_work_and_recovers_border_rods(self):
        import cv2
        import numpy as np
        self.backend.quality = 'fast'
        mask = np.zeros((180, 260), np.uint8)
        rods = [((0, 12), (250, 12)), ((259, 0), (259, 179)), ((9, 53), (230, 82))]
        for a, b in rods:
            cv2.line(mask, a, b, 255, 1)
        fitted = self.backend.fit_segments(mask, self.backend.hough_segments(mask))
        for a, b in rods:
            center = np.mean([a, b], axis=0)
            self.assertTrue(any(abs(center@line['n']-line['b']) < 2 and
                                line['hi']-line['lo'] > np.linalg.norm(np.subtract(b, a))*.8
                                for line in fitted), (a, b))
        rng = np.random.default_rng(44)
        clutter = np.uint8(rng.random((180, 260)) < .2)*255
        cv2.rectangle(clutter, (50, 30), (220, 150), 255, 6)
        self.backend.hough_segments(clutter)
        counts = self.backend.diagnostics()['work_counts']
        self.assertGreater(counts['hough_peak_count'], counts['hough_scanned_peaks'])
        self.assertLessEqual(counts['hough_scanned_peaks'], 256)
        self.assertIn('hough_runs_fast', self.backend.runtime.kernel_ms)
        self.assertNotIn('hough_select_parallel', self.backend.runtime.kernel_ms)

    def test_fast_sampling_preserves_gate_evidence_scenarios(self):
        self.backend.quality = 'fast'
        self.test_pyramid_and_gpu_sampling_preserve_gate_evidence_scenarios()

    def test_validator_checks_requested_fast_pyramid_combination_and_restores_settings(self):
        from src import opencl_validation as V
        self.backend.quality, self.backend.blur_mode = 'fast', 'pyramid'
        seen = []
        def probe(backend):
            seen.append((backend.quality, backend.blur_mode))
            return {}
        with patch.object(V, 'CHECKS', {'precise_probe': probe}), \
                patch.object(V, 'check_fast', probe), patch.object(V, 'check_pyramid', probe):
            V.validate_backend(self.backend)
        self.assertEqual(seen, [('precise', 'exact'), ('fast', 'pyramid'), ('precise', 'pyramid')])
        self.assertEqual((self.backend.quality, self.backend.blur_mode), ('fast', 'pyramid'))

    def test_cpu_hough_keeps_gpu_fitting_and_reports_placement(self):
        self.backend.hough_backend = 'cpu'
        CHECKS['geometry'](self.backend)
        diagnostics = self.backend.diagnostics()
        self.assertEqual(diagnostics['hough_backend'], 'cpu')
        self.assertNotIn('hough_vote', diagnostics['kernel_ms'])
        self.assertIn('fit_sections', diagnostics['kernel_ms'])

    def test_benchmark_reports_full_cv_modes_and_restores_placement(self):
        from src.opencl_validation import benchmark_cv
        records = []
        results = benchmark_cv(self.backend, frames=3, warmup=0, report=records.append)
        self.assertEqual(self.backend.hough_backend, 'opencl')
        self.assertEqual([result['hough_backend'] for result in results], ['opencl', 'cpu'])
        self.assertEqual(len([r for r in records if r['event'] == 'benchmark_frame']), 6)
        for result in results:
            self.assertEqual(sum(mode['n'] for mode in result['cv_modes'].values()), 3)
            self.assertIn('search', result['cv_modes'])
            self.assertIn('track', result['cv_modes'])
            self.assertEqual(result['selected_frames'], 3)
            self.assertEqual(result['budget_ms'], 100.)


class BackendSelectionTests(unittest.TestCase):
    def test_cpu_does_not_load_opencl(self):
        with patch('src.opencl_backend.OpenCLBackend') as factory:
            backend, info = create_backend('cpu')
        factory.assert_not_called()
        self.assertIsNone(backend)
        self.assertEqual(info['selected'], 'cpu')

    def test_strict_gpu_failure_and_explicit_auto_startup_fallback(self):
        with patch('src.opencl_backend.OpenCLBackend', side_effect=OpenCLError('GPU unavailable')):
            with self.assertRaisesRegex(OpenCLError, 'GPU unavailable'):
                create_backend('opencl', 'Mali')
            backend, info = create_backend('auto', 'Mali')
        self.assertIsNone(backend)
        self.assertEqual(info['selected'], 'cpu')
        self.assertIn('GPU unavailable', info['fallback_reason'])

    def test_runtime_failure_is_not_silently_replaced_with_cpu(self):
        with patch('src.opencl_backend.OpenCLRuntime', HostRuntime):
            backend = OpenCLBackend()
        self.addCleanup(backend.close)
        import numpy as np
        with patch.object(backend.runtime, 'run', side_effect=OpenCLError('GPU launch failed')):
            with self.assertRaisesRegex(OpenCLError, 'GPU launch failed'):
                backend.contrast_signal(np.zeros((8, 8, 3), np.uint8))


if __name__ == '__main__':
    unittest.main()
