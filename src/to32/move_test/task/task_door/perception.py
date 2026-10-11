"""前视进程里的纯视觉处理；无需 PyTorch，复用板端已有 HBM 检测器。

process(raw_bgr, detector, now) -> (corrected_bgr, detections, observation)
YOLO 与 CV 的几何全部位于同一张校正图，姿态仅用于 UI 显示。
"""
import sys
import json
import time
from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np

from .config import CONFIG

_ROOT = str(Path(__file__).resolve().parents[5])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quad_cv_kit.src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from quad_cv_kit.src.detect_red_gate import RedGateTracker, original_geometry
from quad_cv_kit.src.gate_guidance import build_gate_guidance
from quad_cv_kit.src.yolo_red_gate import boundary_sides, search_region, box_area, box_iou
from quad_cv_kit.src.opencl_backend import create_backend
from quad_cv_kit.src.gpu_pipeline import ResidentGatePipeline


class DoorFrameProcessor:
    def __init__(self, cfg=CONFIG):
        self.cfg = cfg.validate()
        self.corrector = None
        self.serial = 0
        self.cv_backend = None
        self.cv_backend_info = None
        self.closed = False
        self.cv = RedGateTracker(detect_every=cfg.cv_every_frames, hold_seconds=0, profile=True)
        self.reset()

    def reset(self):
        self.target = None
        self.last_seen = None
        self.cv.reset()

    def _prepare(self):
        cfg = self.cfg
        if self.cv_backend_info is None:
            cv2.setNumThreads(cfg.opencv_threads)
            self.cv_backend, info = create_backend(cfg.cv_backend, cfg.gpu_device,
                                                   'opencl', cfg.cv_blur, cfg.cv_quality)
            self.cv_backend_info = dict(info, cv_execution='resident' if self.cv_backend else 'hybrid')
            print('[PassGate] CV backend: '+json.dumps(self.cv_backend_info, ensure_ascii=False), flush=True)
            if self.cv_backend is not None:
                self.cv = ResidentGatePipeline(self.cv_backend, detect_every=cfg.cv_every_frames,
                                               hold_seconds=0, padding=cfg.roi_padding)
        if self.corrector is None:
            camera, _ = adapt_camera_params(load_camera_params(cfg.camera_params_path),
                                            cfg.image_width, cfg.image_height, cfg.camera_fit)
            self.corrector = create_corrector(camera)
            self.valid_mask = self.corrector.valid_mask(cfg.correction_plane_distance_m)
        self.valid_mask.setflags(write=False)
        if self.cv_backend is not None and self.cv.camera is None:
            self.cv.set_camera(self.corrector.output_matrix)

    def close(self):
        if not self.closed:
            self.closed = True
            if self.cv_backend is not None:
                self.cv_backend.close()

    def _select(self, dets, now):
        cfg = self.cfg
        boxes = []
        for d in dets:
            if str(d.get('label', '')).lower() not in ('door', 'gate', 'doorway'):
                continue
            box = np.asarray(d.get('bbox', []), float)
            if (box.shape != (4,) or not np.isfinite(box).all()
                    or float(d.get('score', 0)) < cfg.min_score):
                continue
            box = np.clip(box, [0, 0, 0, 0],
                          [cfg.image_width, cfg.image_height]*2).tolist()
            if box_area(box) > 0:
                boxes.append(dict(d, bbox=box))
        if self.target is not None:
            old_area = box_area(self.target['bbox'])
            matches = [d for d in boxes if box_iou(d['bbox'], self.target['bbox']) >= cfg.target_iou_min
                       and min(box_area(d['bbox']), old_area)/max(box_area(d['bbox']), old_area)
                       >= cfg.target_area_ratio_min]
            if matches:
                chosen = max(matches, key=lambda d: box_iou(d['bbox'], self.target['bbox']))
            elif now-self.last_seen < cfg.target_lost_s:
                return None  # 不将后方嵌套门冒充当前门
            else:
                self.reset()
                chosen = max(boxes, key=lambda d: (box_area(d['bbox']), d['score'])) if boxes else None
        else:
            chosen = max(boxes, key=lambda d: (box_area(d['bbox']), d['score'])) if boxes else None
        if chosen is not None:
            if self.target is None:
                self.serial += 1
            self.target, self.last_seen = chosen, now
        return chosen

    def process(self, frame, detector, now):
        if self.closed:
            raise RuntimeError('DoorFrameProcessor 已关闭')
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError('输入必须为 uint8 BGR 图像')
        cfg = self.cfg
        h, w = frame.shape[:2]
        if (w, h) != (cfg.image_width, cfg.image_height):
            raise ValueError('门视觉要求输入 %d×%d，实际 %d×%d' %
                             (cfg.image_width, cfg.image_height, w, h))
        started = time.perf_counter()
        self._prepare()
        prepared = time.perf_counter()
        with (self.cv_backend.frame_batch() if self.cv_backend else nullcontext()):
            fixed, detections, obs = self._process(frame, detector, now)
        obs['timing_ms']['backend_prepare'] = (prepared-started)*1000
        obs['timing_ms']['processor_total'] = (time.perf_counter()-started)*1000
        return fixed, detections, obs

    def _process(self, frame, detector, now):
        cfg = self.cfg
        h, w = frame.shape[:2]
        if self.cv_backend is not None:
            self.cv_backend.reset_stats()
        started = time.perf_counter()
        if self.cv_backend is not None:
            reference, fixed, _, _ = self.cv_backend.preprocess(
                frame, self.valid_mask, maps=self.corrector.maps(cfg.correction_plane_distance_m),
                device_reference=True)
        else:
            fixed = self.corrector.undistort(frame, cfg.correction_plane_distance_m)
            reference = fixed
        corrected = time.perf_counter()
        # 校正后必须重新转模型输入，不能复用原始相机 NV12。
        detections = detector.detect(fixed, nv12=None)
        detected = time.perf_counter()
        chosen = self._select(detections, now)
        selected = time.perf_counter()
        timings = dict(correction=(corrected-started)*1000, yolo=(detected-corrected)*1000,
                       target_selection=(selected-detected)*1000, opencv=0.0,
                       geometry=0.0, guidance=0.0)
        obs = dict(valid=True, has_target=chosen is not None,
                   coordinate_space='corrected', img_w=w, img_h=h,
                   target_id=self.serial if chosen else None, geometry=None, guidance=None,
                   cv_backend=dict(self.cv_backend_info), enhancement_mode='none',
                   timing_ms=timings,
                   yolo_timing_ms=dict(getattr(detector, 'last_timing_ms', {}) or {}),
                   cv_profile=None, cv_mode='skipped-no-target')
        if chosen is None:
            reset_started = time.perf_counter()
            self.cv.reset()
            timings['cv_reset'] = (time.perf_counter()-reset_started)*1000
            if self.cv_backend is not None:
                obs['gpu'] = self.cv_backend.diagnostics()
            return fixed, detections, obs
        box = chosen['bbox']
        sides = boundary_sides(box, (w, h), self.valid_mask, cfg.boundary_margin_px)
        x1, y1, x2, y2 = box
        matrix = self.corrector.output_matrix
        obs.update(bbox=box, score=chosen['score'], width_px=x2-x1, height_px=y2-y1,
                   area_ratio=(x2-x1)*(y2-y1)/(cfg.image_width*cfg.image_height),
                   center_px=[(x1+x2)/2, (y1+y2)/2],
                   aim_px=matrix[:2, 2].tolist(), focal_px=[matrix[0, 0], matrix[1, 1]],
                   clipped=bool(sides), boundary_sides=sides)
        cv_started = time.perf_counter()
        if self.cv_backend is not None:
            # Preserve formal target IDs/IoU rules; the selected YOLO record is
            # external input to the resident detector, never a host image ROI.
            candidate, _ = self.cv.update(reference, reference,
                                          [dict(chosen, class_id=0)], {0}, self.valid_mask)
        else:
            region = search_region(box, (w, h), bool(sides), cfg.roi_padding)
            candidate, _ = self.cv.update(fixed, region, box, reference_frame=fixed,
                                          valid_mask=self.valid_mask)
        obs['cv_profile'] = self.cv.last_status.get('cv_profile')
        obs['cv_mode'] = (obs['cv_profile'] or {}).get('mode',
                            self.cv.last_status.get('mode', 'unknown'))
        cv_finished = time.perf_counter()
        timings['opencv'] = (cv_finished-cv_started)*1000
        geometry = original_geometry(fixed, candidate)
        geometry_finished = time.perf_counter()
        timings['geometry'] = (geometry_finished-cv_finished)*1000
        obs['geometry'] = geometry
        if self.cv_backend is not None:
            obs['guidance'] = self.cv.last_guidance
        else:
            status = dict(self.cv.last_status, target_bbox=box, yolo_age_frames=0,
                          target_clipped=bool(sides), target_boundary_sides=sides)
            obs['guidance'] = build_gate_guidance(status, geometry, matrix, (w, h),
                                                 self.valid_mask, (cfg.gate_width_m, cfg.gate_height_m))
        timings['guidance'] = (time.perf_counter()-geometry_finished)*1000
        if self.cv_backend is not None:
            obs['gpu'] = self.cv_backend.diagnostics()
        return fixed, detections, obs
