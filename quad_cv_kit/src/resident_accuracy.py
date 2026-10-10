"""Exact integer-stage checks, runnable on the board before resident video.

These validation-only intermediate readbacks never run in the video pipeline.
Original kernels serve as device oracles; OpenCV independently checks CCL.
"""
import itertools

import cv2
import numpy as np

from .gpu_pipeline import ResidentGatePipeline
from .opencl_runtime import LocalMemory


def legacy_morph(backend, source, w, h, steps, prefix):
    """Former sequence of full-image morph3 passes, for validation only."""
    current = source
    for i, dilate in enumerate(steps):
        out = backend.runtime.buffer(prefix+'_pass_'+str(i), w*h)
        backend.runtime.run('morph3', w*h, [current, out, w, h, int(dilate)])
        current = out
    return current


def check_morphology(backend, exhaustive=False):
    rt = backend.runtime
    rng = np.random.default_rng(6001)
    sequences = [(True, False), (True, False, False, True)]
    if exhaustive:
        sequences = list(itertools.product((False, True), repeat=2))+list(itertools.product((False, True), repeat=4))
    shapes = [(1, 1), (1, 19), (21, 1), (7, 11), (33, 49), (17, 257), (9, 641)]
    cases = 0
    for h, w in shapes:
        for binary in (False, True):
            image = rng.integers(0, 256, (h, w), np.uint8)
            if binary:
                image = np.uint8(image > 150)*255
            source = rt.upload('accuracy_morph_source', image)
            for steps in sequences:
                fused = backend._morph(source, w, h, steps, 'accuracy_morph_fused', binary=binary)
                old = legacy_morph(backend, source, w, h, steps, 'accuracy_morph_old')
                np.testing.assert_array_equal(rt.read(fused, image.shape, np.uint8),
                                              rt.read(old, image.shape, np.uint8),
                                              err_msg=f'morphology {image.shape} {steps}')
                cases += 1
    return dict(name='resident_morphology_exact', passed=True, cases=cases,
                comparison='byte-identical to sequential morph3 including intermediate borders')


def component_cases():
    rng = np.random.default_rng(6002)
    masks = [np.zeros((1, 1), np.uint8), np.full((33, 65), 255, np.uint8),
             np.uint8(rng.random((49, 81)) > .68)*255]
    # Both diagonals across vertical, horizontal and four-tile boundaries;
    # long one-pixel chains cross several tiles and linear statistics blocks.
    mask = np.zeros((67, 83), np.uint8)
    for k in range(65):
        mask[k, k] = 255
        mask[k, 82-k] = 255
    mask[15, :] = mask[:, 48] = 255
    mask[40:65:2, 1:41:2] = 255  # isolated roots, hash collisions, threshold failures
    masks.append(mask)
    masks.extend([np.full((1, 47), 255, np.uint8), np.full((47, 1), 255, np.uint8)])
    # Exact per-row maximum capacity; diagonals must connect checkerboard runs.
    yy, xx = np.indices((19, 65))
    masks.append(np.uint8((xx+yy)%2 == 0)*255)
    masks.append(np.uint8((xx%2 == 0) & (yy%2 == 0))*255)
    # Many upper-row runs meet one lower run, including long row/chunk crossings.
    bridge = np.zeros((7, 641), np.uint8)
    bridge[1, ::2] = bridge[5, ::2] = 255
    bridge[2:5, :] = 255
    masks.append(bridge)
    return masks


def check_components(backend):
    rt = backend.runtime
    pipeline = ResidentGatePipeline(backend)
    control = rt.upload('accuracy_cc_control', np.array([0, 1], np.float32))
    cases = 0
    for mask in component_cases():
        h, w = mask.shape
        pipeline.width, pipeline.height = w, h
        source = rt.upload('accuracy_cc_source', mask)
        count, labels, measures, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        roots = np.full(mask.shape, -1, np.int32)
        expected_stats = {}
        for label in range(1, count):
            pixels = np.flatnonzero(labels.reshape(-1) == label)
            root = int(pixels[0])
            roots[labels == label] = root
            x, y, cw, ch, area = measures[label]
            expected_stats[root] = [area, x, y, x+cw-1, y+ch-1]
        for both in (False, True):
            result = pipeline._cc(source, 'accuracy', 3, 3, control, both)
            actual_roots = rt.read(pipeline.buffer('accuracy_parents', mask.size*4), mask.shape, np.int32)
            actual_stats = rt.read(pipeline.buffer('accuracy_stats', mask.size*20), (mask.size, 5), np.int32)
            np.testing.assert_array_equal(actual_roots, roots, err_msg=f'CCL labels {mask.shape}')
            for root, expected in expected_stats.items():
                np.testing.assert_array_equal(actual_stats[root], expected, err_msg=f'CCL stats {mask.shape} root {root}')
            sizes = np.minimum(measures[:, 2], measures[:, 3]) if both else np.maximum(measures[:, 2], measures[:, 3])
            keep = (measures[:, 4] >= 3) & (sizes >= 3)
            keep[0] = False
            np.testing.assert_array_equal(rt.read(result, mask.shape, np.uint8), np.uint8(keep[labels])*255)
            cases += 1
    return dict(name='resident_components_exact', passed=True, cases=cases, connectivity=8,
                comparison='minimum-index labels, area, bounding boxes and filtered pixels match OpenCV')


def check_angle_peaks(backend, full_size=True):
    rt = backend.runtime
    pipeline = ResidentGatePipeline(backend)
    rng = np.random.default_rng(6003)
    nrhos = 1471 if full_size else 73
    votes = rng.integers(0, 32, (360, nrhos), np.int32)
    # Tied plateaus, endpoint maxima and dense equal-score maxima.
    votes[0:3, :] = 20
    votes[359, 0] = votes[359, -1] = 50
    votes[180:184, :] = 0
    votes[180, ::3] = 45
    source = rt.upload('accuracy_peak_votes', votes)
    old, new = rt.buffer('accuracy_old_peaks', 2880*16), rt.buffer('accuracy_new_peaks', 2880*16)
    cases = 0
    for search, threshold in ((True, 18), (True, 100), (False, 18)):
        control = rt.upload('accuracy_peak_control', np.array([0, search], np.float32))
        rt.run('rg_peak_top', 360*64, [source, old, control, backend.fast_angles,
               nrhos, threshold, LocalMemory(64*4), LocalMemory(64*4)], local=64)
        pipeline._peak_top(source, new, control, nrhos, threshold)
        np.testing.assert_array_equal(rt.read(new, (2880, 4), np.int32),
                                      rt.read(old, (2880, 4), np.int32),
                                      err_msg=f'angle Top8 search={search} threshold={threshold}')
        cases += 1
    return dict(name='resident_angle_top8_exact', passed=True, cases=cases, nrhos=nrhos,
                comparison='all peak records identical to former eight-scan kernel')


def check_resident_primitives(backend):
    return [check_morphology(backend), check_components(backend), check_angle_peaks(backend), check_peak_order(backend)]


def check_peak_order(backend):
    """Independent integer lexicographic oracle for ALL peaks before NMS."""
    rt = backend.runtime
    pipeline = ResidentGatePipeline(backend)
    rng = np.random.default_rng(7001)
    rows = np.zeros((2880, 4), np.int32)
    rows[:, 0] = np.arange(2880)//8
    rows[:, 1] = rng.integers(0, 1471, 2880)
    rows[:, 2] = rng.integers(0, 32, 2880)
    rows[:, 3] = rng.integers(0, 128, 2880)  # key ties require original-slot ordering
    cases = 0
    for search, fixture in ((True, rows), (True, rows[np.arange(2880)[::-1]].copy()),
                            (True, np.zeros_like(rows)), (False, rows)):
        # Angle buckets remain fixed: suppression metadata assumes that layout.
        fixture = fixture.copy();fixture[:, 0] = np.arange(2880)//8
        source = rt.upload('accuracy_sort_peaks', fixture)
        control = rt.upload('accuracy_sort_control', np.array([0, search], np.float32))
        selected = rt.buffer('accuracy_sort_selected', pipeline.peak_limit*16)
        pipeline._select_peaks(source, selected, control, 735)
        actual = rt.read(pipeline.buffer('peak_order', 2880*4), (2880,), np.int32)
        valid = np.flatnonzero(fixture[:, 2] > 0) if search else np.empty(0, np.int32)
        ordered = valid[np.lexsort((valid, fixture[valid, 3], -fixture[valid, 2]))]
        expected = np.full(2880, -1, np.int32);expected[:len(ordered)] = ordered
        np.testing.assert_array_equal(actual, expected, err_msg=f'complete peak ordering search={search}')
        cases += 1
    return dict(name='resident_peak_order_exact', passed=True, cases=cases, slots=2880,
                comparison='all valid slots match integer vote/key/original-slot ordering before NMS')
