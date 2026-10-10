"""Board validation/measurement for the resident core, with explicit device provenance."""
import time

import cv2
import numpy as np

from .gpu_pipeline import ResidentGatePipeline


def fixture(dx=0, dy=0, missing=False):
    frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    points = np.array([[170, 70], [470, 70], [470, 290], [170, 290]])+[dx, dy]
    for k in range(3 if missing else 4):
        cv2.line(frame, tuple(points[k]), tuple(points[(k+1)%4]), (50, 60, 230), 8)
    return frame


def check_resident(backend):
    from .resident_accuracy import check_resident_primitives
    primitive_checks = check_resident_primitives(backend)
    pipeline = ResidentGatePipeline(backend)
    boxes = [dict(class_id=0, score=.9, bbox=[155, 55, 485, 305])]
    geometries = []
    for index in range(2):
        frame = fixture(index*3, index*2)
        backend.reset_stats()
        with backend.frame_batch():
            selected, _ = pipeline.update(frame, frame, boxes, {0})
        if selected is None or not selected['complete']:
            raise AssertionError('Resident core lost complete fixture')
        geometries.append(np.asarray(selected['corners']))
        if backend.runtime.download_bytes != 800:
            raise AssertionError('Resident core read back an intermediate image/line/model array')
    if not selected['tracked']:
        raise AssertionError('Resident LK/RANSAC did not track the second frame')
    np.testing.assert_allclose(geometries[1]-geometries[0], np.tile([3, 2], (4, 1)), atol=.8)
    return dict(name='resident_core', passed=True, final_output_bytes=800,
                intermediate_readbacks=0, translated_corners=True,
                primitive_checks=primitive_checks,
                algorithm='color-hough / sampled-fit / pyramidal-LK / similarity-RANSAC')


def benchmark_resident(backend, frames=30, report=None):
    if frames < 1:
        raise ValueError('Resident benchmark frames must be positive')
    pipeline = ResidentGatePipeline(backend, detect_every=3)
    boxes = [dict(class_id=0, score=.9, bbox=[155, 55, 485, 305])]
    samples = []
    for index in range(frames+6):
        frame = fixture(index%3*2, index%2)
        backend.reset_stats()
        started = time.perf_counter()
        with backend.frame_batch():
            selected, _ = pipeline.update(frame, frame, boxes, {0})
        profile = pipeline.last_status['cv_profile']
        item = dict(event='resident_benchmark_frame', frame=index, warmup=index<6,
                    device=dict(backend.runtime.info), mode=profile['mode'],
                    total_ms=round((time.perf_counter()-started)*1000, 3),
                    stage_gpu_ms=profile['stages_ms'], timing_kind=profile['timing_kind'],
                    final_join_ms=profile['final_join_ms'], selected=selected is not None,
                    upload_bytes=backend.runtime.upload_bytes, download_bytes=backend.runtime.download_bytes)
        if report is not None:
            report(item)
        if index >= 6:
            samples.append(item)
    def distribution(values):
        return dict(mean_ms=round(float(np.mean(values)), 3),
                    p95_ms=round(float(np.percentile(values, 95)), 3), max_ms=round(max(values), 3))
    modes = {}
    for mode in sorted({s['mode'] for s in samples}):
        group = [s for s in samples if s['mode']==mode]
        names = sorted({k for s in group for k in s['stage_gpu_ms']})
        modes[mode] = dict(n=len(group), total=distribution([s['total_ms'] for s in group]),
                          stages={name: distribution([s['stage_gpu_ms'].get(name, 0.) for s in group]) for name in names},
                          within_30ms=sum(s['total_ms']<=30 for s in group),
                          valid_geometry=sum(s['selected'] for s in group))
    return dict(event='resident_benchmark_summary', device=dict(backend.runtime.info),
                scope='CV core including input upload, submission, final join and event collection; excludes YOLO/preprocessing/encoder',
                timing_kind=samples[0]['timing_kind'], warmup_frames=6, measured_frames=frames,
                final_output_bytes=800, modes=modes, samples=samples)
