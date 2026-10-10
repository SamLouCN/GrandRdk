"""Deterministic arithmetic and geometry checks for the explicit OpenCL backend.

Called on the real device by demo/check_opencl.py and against compiled kernels
by host regression tests. These are correctness checks, not video accuracy or
throughput benchmarks.
"""
import time

import cv2
import numpy as np

from . import detect_red_gate as D, gate_models as M
from .gate_guidance import enhance_cv_contrast, sharpen_frame
from .gate_line_geometry import endpoints
from .opencl_runtime import LocalMemory


def check_blur(backend):
    rng = np.random.default_rng(812)
    worst = 0.
    # Small dimensions also exercise repeated reflection for the large kernel.
    for shape in ((1, 19), (19, 1), (37, 61)):
        source = rng.uniform(0, 255, shape).astype(np.float32)
        h, w = shape
        for sigma in (1.2, 3, 9, 18):
            buf = backend.runtime.upload('validation_source', source)
            out = backend._blur(buf, w, h, sigma, 'validation')
            actual = backend.runtime.read(out, shape, np.float32)
            expected = cv2.GaussianBlur(source, (0, 0), sigma)
            np.testing.assert_allclose(actual, expected, atol=.002, rtol=0)
            worst = max(worst, float(np.max(np.abs(actual-expected))))
    return dict(max_abs_error=worst)


def check_color(backend):
    rng = np.random.default_rng(91)
    frame = rng.integers(0, 256, (72, 128, 3), dtype=np.uint8)
    enhanced = cv2.convertScaleAbs(frame, alpha=1.2)
    cv2.line(frame, (10, 32), (115, 42), (50, 60, 230), 5)
    worst = 0.
    for extra in (frame, enhanced):
        mask, score = M.combined_evidence(extra, frame)
        actual_mask, actual_score = backend.combined_evidence(extra, frame)
        # Random fixture is away from calibrated threshold boundaries.
        np.testing.assert_array_equal(actual_mask, mask)
        np.testing.assert_allclose(actual_score, score, atol=.0001, rtol=0)
        worst = max(worst, float(np.max(np.abs(actual_score-score))))
    np.testing.assert_allclose(backend.contrast_signal(frame), D.contrast_signal(frame), atol=1e-6, rtol=0)
    np.testing.assert_array_equal(backend.red_mask(frame), D.red_mask(frame))
    return dict(max_score_error=worst)


def check_enhancement(backend):
    rng = np.random.default_rng(19)
    frame = rng.integers(0, 256, (36, 64, 3), dtype=np.uint8)
    frame[3:6] = 0
    valid = np.ones(frame.shape[:2], bool)
    valid[:3] = False
    worst = 0
    for clip, blend, amount, saturation in ((0, 0, .6, 1), (0, 0, 0, 1.25), (2, .6, .6, 1.25)):
        expected = enhance_cv_contrast(frame, 1.2, valid, clip, blend, amount, saturation)
        actual = backend.enhance(frame, 1.2, valid, clip, blend, amount, saturation)
        error = int(np.max(np.abs(actual.astype(int)-expected.astype(int))))
        # Float32 rather than CPU float64 can change rounding by one level.
        if error > 2:
            raise AssertionError(f'Enhancement pixel error {error} > 2')
        np.testing.assert_array_equal(actual[~valid], frame[~valid])
        np.testing.assert_array_equal(actual[3:6], frame[3:6])
        worst = max(worst, error)
    np.testing.assert_array_equal(backend.enhance(frame, 1, valid, 0, 0, 0, 1), frame)
    np.testing.assert_array_equal(backend.enhance(frame, valid_mask=np.zeros_like(valid)), frame)
    return dict(max_pixel_error=worst)


def check_remap(backend):
    rng = np.random.default_rng(71)
    frame = rng.integers(0, 256, (33, 57, 3), dtype=np.uint8)
    y, x = np.indices(frame.shape[:2], dtype=np.float32)
    worst = 0
    for mx, my in ((x, y), (x+.23, y-.71), (x*1.23-4.4, y*.92+2.8)):
        # The kernel uses OpenCV's documented 5-bit interpolation table. Some
        # SIMD builds interpolate float maps with finer precision; compare the
        # same canonical map quantization and report the float-map difference.
        fixed, fractions = cv2.convertMaps(mx, my, cv2.CV_16SC2)
        expected = cv2.remap(frame, fixed, fractions, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        actual = backend.remap(frame, mx, my)
        np.testing.assert_array_equal(actual, expected)
        float_result = cv2.remap(frame, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        worst = max(worst, int(np.max(np.abs(actual.astype(int)-float_result.astype(int)))))
    # Map cache invalidates when the caller supplies different map arrays.
    return dict(fixed_map_max_pixel_error=0, float_map_max_pixel_error=worst)


def check_fitting(backend):
    mask = np.zeros((180, 260), np.uint8)
    rods = [((12, 25), (238, 40)), ((25, 70), (25, 161)),
            ((70, 90), (231, 90)), ((111, 122), (239, 146))]
    seeds = []
    for a, b in rods:
        cv2.line(mask, a, b, 255, 5)
        seeds.extend([(*a, *b), (*b, *a)])
    mask[77:94, 21:30] = 0  # Supported line with a real gap.
    seeds += [(40, 60, 230, 60), (0, 0, 9, 0), (0, 0, 170, 170)]
    # Compare one seed at a time so that rejections can be checked too.
    accepted, worst = 0, 0.
    for seed in np.asarray(seeds, np.float32):
        expected = D.line_from_segment(mask, seed, .48)
        actual = backend.fit_segments(mask, [seed], .48)
        if bool(actual) != (expected is not None):
            raise AssertionError(f'Fit acceptance differs for {seed}')
        if expected is not None:
            np.testing.assert_allclose(endpoints(actual[0]), endpoints(expected), atol=.2, rtol=0)
            if abs(actual[0]['width']-expected['width']) > 1:
                raise AssertionError('Fitted width differs by more than one pixel')
            worst = max(worst, float(np.max(np.abs(endpoints(actual[0])-endpoints(expected)))))
            accepted += 1
    batch = backend.fit_segments(mask, np.asarray(seeds, np.float32), .48)
    if len(batch) != accepted:
        raise AssertionError('Batch fit differs from independent fits')
    if backend.fit_segments(mask, []):
        raise AssertionError('Empty seeds should return no lines')
    return dict(accepted=accepted, rejected=len(seeds)-accepted, max_endpoint_error=worst)


def check_hough(backend):
    mask = np.zeros((190, 280), np.uint8)
    rods = [((12, 22), (120, 22)), ((150, 22), (263, 22)),
            ((35, 60), (35, 174)), ((87, 78), (258, 106))]
    for a, b in rods:
        cv2.line(mask, a, b, 255, 3)
    segments = backend.hough_segments(mask)
    fitted = backend.fit_segments(mask, segments)
    for a, b in rods:
        midpoint = np.mean([a, b], axis=0)
        direction = np.subtract(b, a).astype(float)
        direction /= np.linalg.norm(direction)
        found = any(abs(midpoint@line['n']-line['b']) < 2.5
                    and abs(line['d']@direction) > .995
                    and line['hi']-line['lo'] > np.linalg.norm(np.subtract(b, a))*.8 for line in fitted)
        if not found:
            raise AssertionError(f'GPU Hough did not recover rod {a}->{b}')
    for segment in segments:
        ends = segment.reshape(2, 2)
        if np.max(np.abs(ends[:, 1]-22)) < 4 and ends[:, 0].min() < 100 and ends[:, 0].max() > 170:
            raise AssertionError('Hough bridged the 30px gap with max_gap=10')
    empty = backend.hough_segments(np.zeros_like(mask))
    if empty.shape != (0, 4):
        raise AssertionError('Empty Hough input should produce no segments')
    # Repeatability includes sorting peaks despite atomic compaction order.
    np.testing.assert_array_equal(segments, backend.hough_segments(mask))
    return dict(segments=len(segments), fitted=len(fitted), rods_recovered=len(rods))


def check_cooperative_selection(backend):
    """Compare parallel consumption to the former serial ACTUAL GPU kernel."""
    mask = np.zeros((115, 170), np.uint8)
    cv2.line(mask, (10, 27), (158, 36), 255, 9)
    cv2.line(mask, (65, 4), (70, 109), 255, 7)
    mask[20:45, 95:110] = 0
    actual = backend.hough_segments(mask, min_length=35)
    rt = backend.runtime
    count_buf = rt.buffer('peak_count', 4)
    npeaks = int(rt.read(count_buf, (1,), np.uint32)[0])
    maxruns = int(np.ceil(np.hypot(169, 114)/35))+2
    old_mask = rt.upload('validation_old_mask', mask)
    out = rt.buffer('validation_old_selected', npeaks*maxruns*16)
    count = rt.buffer('validation_old_count', 4)
    rt.run('hough_select', 1, [old_mask, rt.buffer('hough_segments', npeaks*maxruns*16),
            rt.buffer('hough_run_counts', npeaks*4), out, count, 170, 115, npeaks, maxruns, 35])
    n = int(rt.read(count, (1,), np.uint32)[0])
    expected = rt.read(out, (n, 4), np.float32)
    np.testing.assert_array_equal(actual, expected)
    return dict(segments=n, serial_parallel_identical=True)


def check_geometry(backend):
    frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    cv2.rectangle(frame, (220, 100), (410, 280), (50, 60, 230), 8)
    cv2.line(frame, (220, 100), (410, 100), (105, 105, 140), 8)
    for point in ((220, 100), (410, 100), (410, 280), (220, 280)):
        cv2.circle(frame, point, 6, (220, 220, 220), -1)
    source = cv2.resize(frame, (1280, 720))
    candidates, _, _ = D.detect(source, search_bbox=[386, 146, 876, 624], backend=backend)
    complete = [item for item in candidates if item['complete']]
    if not complete:
        raise AssertionError('Weak complete gate was lost after GPU/ROI migration')
    box = D.original_geometry(source, complete[0])['bbox']
    np.testing.assert_allclose(box, [440, 200, 820, 560], atol=10)
    # No fourth side may be supplied by unrelated evidence or Hough peaks.
    missing = frame.copy()
    missing[266:295, 215:416] = (120, 90, 40)
    candidates, _, _ = D.detect(missing, backend=backend)
    if any(item['complete'] for item in candidates):
        raise AssertionError('A missing fourth side was manufactured')
    # Closest partially visible thick frame must beat the complete far frame.
    partial = frame.copy()
    cv2.line(partial, (5, 55), (535, 55), (50, 60, 230), 14)
    cv2.line(partial, (535, 55), (535, 359), (50, 60, 230), 14)
    nearest = D.select_nearest(D.detect(partial, backend=backend)[0])
    if nearest is None or nearest['complete'] or nearest['apparent_width'] <= 10:
        raise AssertionError('Foreground partial gate selection changed')
    return dict(complete_bbox=box, missing_side_rejected=True, partial_foreground_selected=True)


def check_preprocess(backend):
    rng = np.random.default_rng(31)
    frame = rng.integers(0, 256, (37, 61, 3), dtype=np.uint8)
    valid = np.ones(frame.shape[:2], bool)
    valid[:3] = False
    valid.setflags(write=False)
    y, x = np.indices(frame.shape[:2], dtype=np.float32)
    maps = (x+.23, y-.71)
    fixed, enhanced, _, _ = backend.preprocess(frame, valid, maps=maps)
    m1, m2 = cv2.convertMaps(*maps, cv2.CV_16SC2)
    np.testing.assert_array_equal(fixed, cv2.remap(frame, m1, m2, cv2.INTER_LINEAR))
    np.testing.assert_array_equal(enhanced, fixed)
    # Immutable maps remain cached; no corrected-image reupload is permitted.
    backend.reset_stats()
    _, output, _, enhancement_ms = backend.preprocess(frame, valid, maps=maps)
    np.testing.assert_array_equal(output, fixed)
    if backend.runtime.upload_bytes != frame.nbytes:
        raise AssertionError('Preprocessing redundantly uploaded maps or corrected frame')
    forbidden = ('bgr_hsv', 'histogram_value', 'median_parameters', 'contrast_device',
                 'sharpen_bgr', 'sharpen_only')
    if any(name in backend.runtime.kernel_ms for name in forbidden) or enhancement_ms != 0:
        raise AssertionError('Correction-only preprocessing ran an enhancement stage')
    return dict(max_pixel_error=0, steady_upload_bytes=backend.runtime.upload_bytes,
                corrected_frame_reupload=False, enhancement_mode='none')


def check_sharpen_only(backend):
    """Compare fused sharpening including tiny images and masked borders."""
    rng = np.random.default_rng(743)
    worst = 0
    forbidden = ('bgr_hsv', 'histogram_value', 'median_parameters', 'contrast_device', 'sharpen_bgr')
    for shape in ((1, 19), (19, 1), (3, 7), (37, 61)):
        frame = rng.integers(0, 256, (*shape, 3), dtype=np.uint8)
        frame.flat[:6] = 0
        valid = rng.random(shape) > .2
        for amount in (0, .6, 2):
            backend.reset_stats()
            with backend.frame_batch():
                actual = backend.sharpen(frame, amount, valid)
                expected = sharpen_frame(frame, amount, valid)
                error = int(np.max(np.abs(actual.astype(int)-expected.astype(int))))
                if error > 2:
                    raise AssertionError(f'Fused sharpener pixel error {error}>2')
                worst = max(worst, error)
                np.testing.assert_array_equal(actual[~valid], frame[~valid])
                if amount == 0:
                    np.testing.assert_array_equal(actual, frame)
                elif backend.resident_buffer(actual) is None:
                    raise AssertionError('Sharpened image lost device residency')
            if any(name in backend.runtime.kernel_ms for name in forbidden):
                raise AssertionError('Sharpen-only path executed a contrast or HSV kernel')
    frame = np.full((37, 61, 3), (50, 60, 230), np.uint8)
    valid = np.ones(frame.shape[:2], bool)
    frame[:, :7] = 0
    valid[:, :7] = False
    np.testing.assert_array_equal(backend.sharpen(frame, 2, valid), frame)
    np.testing.assert_array_equal(backend.sharpen(frame, 2, np.zeros_like(valid)), frame)
    return dict(max_pixel_error=worst, constant_border_halo=False,
                enhancement_mode='sharpen-only', enhancement_kernel='sharpen_only')


def check_trim_and_sides(backend, endpoint_tolerance=.2, support_tolerance=.01):
    mask = np.zeros((180, 260), np.uint8)
    rods = [((5, 22), (253, 22)), ((27, 58), (27, 173)), ((83, 87), (243, 106))]
    lines = []
    for a, b in rods:
        cv2.line(mask, a, b, 255, 5)
        seed = D.line_from_segment(mask, np.float32([*a, *b]), .48)
        if seed is None:
            raise AssertionError('Invalid trim fixture')
        lines.append(seed)
    mask[15:30, 102:112] = 0  # Short gap that must be joined.
    mask[130:143, 20:35] = 0
    score = (mask > 0).astype(np.float32)
    expected = M.trim_lines(lines, mask, score, (0, 0, 260, 180))
    actual = backend.trim_lines(lines, mask, score, (0, 0, 260, 180))
    if len(actual) != len(expected):
        raise AssertionError('GPU trim run count differs')
    for gpu, cpu in zip(actual, expected):
        for key in ('lo', 'hi', 'width', 'strong_lo', 'strong_hi'):
            if abs(gpu[key]-cpu[key]) > endpoint_tolerance:
                raise AssertionError(f'Trim {key} differs by more than {endpoint_tolerance}px')
        if abs(gpu['support']-cpu['support']) > support_tolerance:
            raise AssertionError('Trim support differs')
    cases, segments = [], []
    for line in actual:
        for shift in (-50, -20, 0, 10):
            cases.append(line)
            segments.append(endpoints(dict(line, lo=line['lo']+shift, hi=line['hi'])))
    values = backend.side_support(mask, cases, segments)
    for line, segment, value in zip(cases, segments, values):
        cpu = M.side_evidence(mask, line, segment)
        if (value < 0) != (cpu is None) or (cpu is not None and abs(cpu-value) > .01):
            raise AssertionError('GPU side acceptance/support differs from CPU')
    return dict(trimmed_lines=len(actual), side_checks=len(values))


def check_pyramid(backend):
    # Compare approximation drift separately from exact-kernel correctness.
    previous = backend.blur_mode
    backend.blur_mode = 'pyramid'
    try:
        rng = np.random.default_rng(91)
        frames = [rng.integers(0, 256, (72, 128, 3), dtype=np.uint8)]
        scene = np.full((360, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(scene, (140, 80), (480, 280), (50, 60, 230), 8)
        cv2.line(scene, (140, 80), (480, 80), (130, 110, 120), 8)
        frames.append(scene)
        worst_mask, worst_mean = 0., 0.
        for frame in frames:
            mask, score = M.combined_evidence(frame, frame)
            actual_mask, actual_score = backend.combined_evidence(frame, frame)
            fraction = float(np.mean(mask != actual_mask))
            error = float(np.mean(np.abs(score-actual_score)))
            if fraction > .01 or error > .008:
                raise AssertionError(f'Pyramid fixture drift exceeds limits: mask={fraction}, mean score={error}')
            worst_mask, worst_mean = max(worst_mask, fraction), max(worst_mean, error)
        geometry = check_geometry(backend)
        return dict(approximation=True, mask_mismatch_fraction=worst_mask, mean_score_error=worst_mean, **geometry)
    finally:
        backend.blur_mode = previous


def check_fast(backend):
    previous = backend.quality
    backend.quality = 'fast'
    try:
        return dict(approximation=True, hough=check_hough(backend), geometry=check_geometry(backend),
                    trim=check_trim_and_sides(backend, endpoint_tolerance=2.5, support_tolerance=.04))
    finally:
        backend.quality = previous


class SearchReferenceBackend:
    """Use the previous CPU search stages with the same image/Hough/fit GPU."""
    def __init__(self, backend):
        self.backend = backend

    def __getattr__(self, name):
        if name in ('merge_split_lines', 'complete_models_gpu', 'search_batch', 'partial_measurements'):
            raise AttributeError(name)
        return getattr(self.backend, name)


def benchmark_search(backend, frames=12, warmup=3, report=None):
    """Board-only throughput comparison of old and batched search stages.

    Both placements use the same GPU colors/Hough/fitting, quality and input.
    This synthetic measurement does not establish field accuracy or stream FPS.
    """
    if frames < 1 or warmup < 0:
        raise ValueError('Search benchmark needs positive frames and nonnegative warmup')
    from .cv_profile import CvFrameProfile
    source = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    cv2.rectangle(source, (145, 75), (500, 285), (50, 60, 230), 8)
    cv2.line(source, (145, 75), (500, 75), (105, 105, 140), 8)
    rng = np.random.default_rng(719)
    for _ in range(32):
        x, y = map(int, rng.integers([25, 20], [605, 325]))
        cv2.line(source, (x, y), (min(635, x+int(rng.integers(25, 70))), y), (50, 60, 230), 3)
    enhanced = enhance_cv_contrast(source, 1.2, None, 0, 0, .6, 1)
    valid = np.ones(source.shape[:2], bool)
    summaries = []
    selected = {}
    for placement, candidate_backend in (('cpu-search-reference', SearchReferenceBackend(backend)),
                                          ('gpu-batched-search', backend)):
        elapsed, cpu = [], []
        for index in range(warmup+frames):
            backend.reset_stats()
            profile = CvFrameProfile(True)
            started, cpu_started = time.perf_counter(), time.thread_time()
            candidates = D.detect(enhanced, reference_frame=source, valid_mask=valid,
                                  profile=profile, backend=candidate_backend)[0]
            wall_ms = (time.perf_counter()-started)*1000
            cpu_ms = (time.thread_time()-cpu_started)*1000
            details = profile.finish()
            nearest = D.select_nearest(candidates)
            selected[placement] = None if nearest is None else D.original_geometry(source, nearest)
            if index < warmup:
                continue
            elapsed.append(wall_ms)
            cpu.append(cpu_ms)
            if report:
                report(dict(event='search_benchmark_frame', placement=placement,
                    frame=index-warmup+1, wall_ms=round(wall_ms, 3), thread_cpu_ms=round(cpu_ms, 3),
                    candidates=len(candidates), selected=selected[placement],
                    cv_profile=details, gpu=backend.diagnostics()))
        summary = dict(event='search_benchmark_summary', placement=placement,
            scene='synthetic-weak-gate-clutter', frames=frames, warmup_frames=warmup,
            mean_ms=round(float(np.mean(elapsed)), 3), p95_ms=round(float(np.percentile(elapsed, 95)), 3),
            mean_thread_cpu_ms=round(float(np.mean(cpu)), 3), selected=selected[placement])
        summaries.append(summary)
        if report:
            report(summary)
    return summaries


def check_batched_search(backend):
    """Compare migrated search to the CPU logic with identical fitted seeds."""

    from .cv_profile import CvFrameProfile
    worst = 0.
    checked = 0
    for quality in ('precise', 'fast'):
        previous = backend.quality
        backend.quality = quality
        try:
            base = np.full((180, 260, 3), (120, 90, 40), np.uint8)
            cv2.rectangle(base, (25, 30), (225, 150), (50, 60, 230), 7)
            cv2.line(base, (25, 30), (225, 30), (105, 105, 140), 7)
            for gap in (False, True):
                frame = base.copy()
                if gap:
                    frame[26:36, 95:113] = (120, 90, 40)
                enhanced = cv2.convertScaleAbs(frame, alpha=1.1)
                mask, _ = M.combined_evidence(enhanced, frame)
                seed_cache = {}
                expected = D.get_lines(enhanced, mask, frame, backend=SearchReferenceBackend(backend), seed_cache=seed_cache)
                profile = CvFrameProfile(True)
                actual = D.get_lines(enhanced, mask, frame, profile, backend=backend, seed_cache=seed_cache)
                for first, second in zip(expected, actual):
                    if len(first) != len(second):
                        raise AssertionError(f'GPU split acceptance changed: {len(first)} != {len(second)}')
                    for cpu, gpu in zip(first, second):
                        error = float(np.max(np.abs(endpoints(cpu)-endpoints(gpu))))
                        worst = max(worst, error)
                        np.testing.assert_allclose(endpoints(gpu), endpoints(cpu), atol=.05, rtol=0)
                        np.testing.assert_allclose([gpu['width'], gpu['support'], gpu['contrast']],
                                                  [cpu['width'], cpu['support'], cpu['contrast']], atol=.002, rtol=0)
                checked += 1
        finally:
            backend.quality = previous
    return dict(scenes=checked, max_endpoint_error=worst)


def check_parallel_sides(backend):
    """Cooperative support must match the existing serial production kernel."""
    segments = np.float32([((10, 20), (165, 20)), ((30, 5), (30, 110)),
                          ((165, 20), (10, 20)), ((8, 45), (169, 77)),
                          ((-12, 50), (110, 50)), ((20, 20), (28, 20))])
    lines = []
    for a, b in segments:
        d = (b-a).astype(float)
        d /= np.linalg.norm(d)
        limits = np.sort(np.asarray([a, b])@d)
        lines.append(dict(d=d, lo=float(limits[0]), hi=float(limits[1]), width=5))
    params = np.float32([[*segment.reshape(-1), *line['d'], line['width'],
        line['lo'], line['hi'], line['lo'], line['hi'], 0]
        for line, segment in zip(lines, segments)])
    for gap in (False, True):
        mask = np.full((120, 180), 255, np.uint8)
        if gap:
            mask[:, 70:90] = 0
            mask[45:65, :] = 0
        expected = backend.side_support(mask, lines, segments)
        rt = backend.runtime
        seeds, source = rt.upload('validation_side_params', params), rt.upload('validation_side_mask', mask)
        out = rt.buffer('validation_parallel_sides', len(lines)*4)
        rt.run('search_side_support', len(lines)*64,
               [source, seeds, out, 180, 120, len(lines), LocalMemory(64*8*4)], local=64)
        actual = rt.read(out, (len(lines),), np.float32)
        np.testing.assert_array_equal(actual, expected)
    return dict(sides=len(lines)*2, serial_parallel_identical=True)


def check_resident_chain(backend):
    """Verify actual device ROI/canvas bytes and absence of mask reuploads."""
    source = np.random.default_rng(19).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
    with backend.frame_batch():
        backend.device_input('validation_source', source)
        before = backend.runtime.upload_bytes
        canvas = backend.prepare_canvas(source)[0]
        buf = backend.resident_buffer(canvas)
        np.testing.assert_array_equal(backend.runtime.read(buf, canvas.shape, canvas.dtype), canvas)
        if backend.runtime.upload_bytes != before:
            raise AssertionError('Resident canvas uploaded pixels instead of reusing the source')
        mask, score = M.combined_evidence(canvas, canvas, backend=backend, bounds=(120, 55, 520, 310))
        before = backend.runtime.upload_bytes
        copied = backend.copy_host(mask)
        cropped = backend.restrict_mask(copied, bounds=(130, 65, 500, 295))
        for array in (mask, score, cropped):
            buf = backend.search_input('validation', array, array.dtype)
            np.testing.assert_array_equal(backend.runtime.read(buf, array.shape, array.dtype), array)
        if backend.runtime.upload_bytes != before:
            raise AssertionError('Resident color/ROI data was uploaded again')
    if backend.resident_buffer(source) is not None:
        raise AssertionError('A previous frame retained its device identity cache')
    return dict(canvas_exact=True, roi_mask_score_exact=True, repeated_upload_bytes=0)


def check_gpu_pipeline(backend):
    """Validate fused colour blur and tracking joins on the selected device."""
    rng = np.random.default_rng(418)
    saved = backend.blur_mode
    worst = 0.
    try:
        for mode in ('exact', 'pyramid'):
            backend.blur_mode = mode
            for shape in ((3, 7), (37, 61)):
                source = rng.uniform(0, 255, shape).astype(np.float32)
                expected = None
                with backend.frame_batch():
                    rt = backend.runtime
                    src = backend.device_input('pipeline_chroma', source)
                    local = backend.device_output('pipeline_local', source.nbytes)
                    for sigma in (3, 9, 18):
                        blur = backend._blur(src, shape[1], shape[0], sigma, 'pipeline_reference')
                        baseline = rt.read(blur, shape, np.float32)
                        delta = source-baseline
                        expected = delta if expected is None else np.maximum(expected, delta)
                        backend._local_blur(src, local, shape[1], shape[0], sigma, sigma == 3, 'pipeline_fused')
                        actual = rt.read(local, shape, np.float32)
                        np.testing.assert_allclose(actual, expected, atol=.002, rtol=0)
                        worst = max(worst, float(np.max(np.abs(actual-expected))))
        mask = np.zeros((120, 190), np.uint8)
        cv2.rectangle(mask, (20, 20), (165, 100), 255, 7)
        lines = [D.line_from_segment(mask, np.asarray(seed, float), .48) for seed in
                 ((20, 20, 165, 20), (165, 20, 165, 100), (165, 100, 20, 100), (20, 100, 20, 20))]
        segments = [endpoints(line) for line in lines]
        with backend.frame_batch():
            occupancy, sides = backend.track_support(mask, lines, segments, True)
        np.testing.assert_allclose(occupancy, 1., atol=.005, rtol=0)
        np.testing.assert_allclose(sides, 1., atol=.005, rtol=0)
    finally:
        backend.blur_mode = saved
    return dict(max_fused_evidence_error=worst, tracking_rods=4, blur_modes=['exact', 'pyramid'])


CHECKS = dict(blur=check_blur, color=check_color, enhancement=check_enhancement,
              sharpen_only=check_sharpen_only,
              remap=check_remap, fitting=check_fitting, hough=check_hough,
              cooperative_selection=check_cooperative_selection, geometry=check_geometry,
              preprocess=check_preprocess, trim_and_sides=check_trim_and_sides,
              batched_search=check_batched_search, parallel_sides=check_parallel_sides,
              residency=check_resident_chain, gpu_pipeline=check_gpu_pipeline)


def validate_backend(backend, report=None):
    results = []
    requested = backend.blur_mode
    requested_quality = backend.quality
    try:
        backend.blur_mode = 'exact'
        backend.quality = 'precise'
        checks = dict(CHECKS)
        if requested_quality == 'fast':
            checks['fast_hough_trim_and_geometry'] = check_fast
        if requested == 'pyramid':
            checks['pyramid_drift_and_geometry'] = check_pyramid
        for name, check in checks.items():
            backend.blur_mode = ('pyramid' if name == 'pyramid_drift_and_geometry' or
                (name == 'fast_hough_trim_and_geometry' and requested == 'pyramid') else 'exact')
            backend.quality = 'fast' if name == 'fast_hough_trim_and_geometry' else 'precise'
            backend.reset_stats()
            started = time.perf_counter()
            metrics = check(backend)
            backend.runtime.finish()
            item = dict(check=name, passed=True, elapsed_ms=round((time.perf_counter()-started)*1000, 3),
                        metrics=metrics, gpu=backend.diagnostics())
            results.append(item)
            if report is not None:
                report(item)
    finally:
        backend.blur_mode = requested
        backend.quality = requested_quality
    return results


def benchmark_cv(backend, frames=12, warmup=6, budget_ms=100., report=None):
    """Board benchmark of full tracker calls, including colors and CPU geometry.

    Synthetic scene has weak rods, white elbows and clutter. Preprocessing/YOLO
    are outside this CV budget. Real video run_summary remains the acceptance
    measurement; a synthetic benchmark cannot establish field performance.
    """
    base = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    cv2.rectangle(base, (220, 100), (410, 280), (50, 60, 230), 8)
    cv2.line(base, (220, 100), (410, 100), (105, 105, 140), 8)
    for point in ((220, 100), (410, 100), (410, 280), (220, 280)):
        cv2.circle(base, point, 6, (220, 220, 220), -1)
    rng = np.random.default_rng(172)
    for _ in range(20):
        x, y = map(int, rng.integers([40, 35], [595, 310]))
        cv2.line(base, (x, y), (min(630, x+int(rng.integers(25, 70))), y), (50, 60, 230), 3)
    source = cv2.resize(base, (1280, 720))
    enhanced = enhance_cv_contrast(source, 1.2, None, 0, 0, .6, 1)
    valid = np.ones(source.shape[:2], bool)
    results = []
    previous_mode = backend.hough_backend
    try:
        for placement in ('opencl', 'cpu'):
            backend.hough_backend = placement
            tracker = D.RedGateTracker(fps=30, detect_every=3, profile=True, backend=backend)
            times = {}
            detections = 0
            for index in range(warmup+frames):
                backend.reset_stats()
                started = time.perf_counter()
                selected, _ = tracker.update(enhanced, [386, 146, 876, 624],
                    reference_frame=source, valid_mask=valid)
                elapsed = (time.perf_counter()-started)*1000
                if index < warmup:
                    continue
                profile = tracker.last_status['cv_profile']
                times.setdefault(profile['mode'], []).append(elapsed)
                detections += selected is not None
                if report:
                    report(dict(event='benchmark_frame', hough_backend=placement,
                                cv_quality=backend.quality,
                                frame=index-warmup+1, opencv_ms=round(elapsed, 3),
                                cv_profile=profile, gpu=backend.diagnostics()))
            modes = {}
            for mode, values in times.items():
                modes[mode] = dict(n=len(values), mean_ms=round(float(np.mean(values)), 3),
                    p95_ms=round(float(np.percentile(values, 95)), 3), max_ms=round(max(values), 3),
                    within_budget_pct=round(100*sum(t <= budget_ms for t in values)/len(values), 2),
                    p95_within_budget=bool(np.percentile(values, 95) <= budget_ms))
            item = dict(event='benchmark_summary', scene='synthetic-weak-gate-clutter',
                        hough_backend=placement, cv_quality=backend.quality, budget_ms=budget_ms, frames=frames,
                        selected_frames=detections, warmup_frames=warmup, cv_modes=modes)
            results.append(item)
            if report:
                report(item)
    finally:
        backend.hough_backend = previous_mode
    return results
