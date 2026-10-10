"""干净校正图保留颜色证据；增强图同时进入 YOLO 和 OpenCV。"""
import sys
import time
import json
import threading
from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np

from .config import CONFIG

_ROOT = str(Path(__file__).resolve().parents[3])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quad_cv_kit.src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from quad_cv_kit.src.detect_red_gate import original_geometry, draw_gate_geometry
from quad_cv_kit.src.gate_guidance import sharpen_frame, build_gate_guidance, draw_gate_guidance
from quad_cv_kit.src.yolo_quad import draw_detections
from quad_cv_kit.src.yolo_red_gate import YoloRedGateTracker
from quad_cv_kit.src.opencl_backend import create_backend


class DoorDetectorAdapter:
    """适配板端 YoloDetector.detect(BGR, nv12=None)，仅保留整门类别。"""
    target_ids = {0}

    def __init__(self, detector, min_score):
        self.detector = detector
        self.min_score = min_score
        self.last_detect_ms = 0.0

    def detect_boxes(self, frame):
        result = []
        # 图像已校正、增强，必须重新转模型输入，不能复用采集时的 NV12。
        started = time.perf_counter()
        detections = self.detector.detect(frame, nv12=None)
        self.last_detect_ms = (time.perf_counter()-started)*1000
        for det in detections:
            if (str(det.get('label', '')).lower() in ('door', 'gate', 'doorway')
                    and float(det.get('score', 0)) >= self.min_score):
                result.append(dict(det, class_id=0))
        return result


class DoorSimFrameProcessor:
    def __init__(self, detector, cfg=CONFIG, fps=30):
        self.cfg = cfg.validate()
        self.fps = max(1., float(fps))
        self.detector = DoorDetectorAdapter(detector, cfg.min_score)
        self.size = None
        self.corrector = None
        self.tracker = None
        self.cv_backend = None
        self.cv_backend_info = None
        self.closed = False
        self.lock = threading.Lock()

    def _prepare(self, width, height):
        cfg = self.cfg
        if self.cv_backend_info is None:
            cv2.setNumThreads(cfg.opencv_threads)
            self.cv_backend, self.cv_backend_info = create_backend(
                cfg.cv_backend, cfg.gpu_device, cfg.cv_hough, cfg.cv_blur, cfg.cv_quality)
            print('[DoorSim] CV backend: '+json.dumps(self.cv_backend_info, ensure_ascii=False), flush=True)
        if cfg.correction_enabled:
            camera, _ = adapt_camera_params(load_camera_params(cfg.camera_params_path),
                                            width, height, cfg.camera_fit)
            self.corrector = create_corrector(camera)
            self.valid_mask = self.corrector.valid_mask(cfg.correction_plane_distance_m)
        else:
            self.corrector = None
            self.valid_mask = np.ones((height, width), bool)
        self.tracker = YoloRedGateTracker(self.detector, fps=self.fps,
                                          detect_every=cfg.cv_every_frames,
                                          hold_seconds=cfg.hold_seconds,
                                          valid_mask=self.valid_mask,
                                          roi_padding=cfg.roi_padding, profile_cv=cfg.cv_profile,
                                          cv_backend=self.cv_backend,
                                          adaptive_search=cfg.cv_search == 'adaptive')
        self.size = (width, height)

    def process(self, frame):
        with self.lock:
            return self._process(frame)

    def _process(self, frame):
        if self.closed:
            raise RuntimeError('DoorSim processor 已关闭')
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError('输入必须为 uint8 BGR 图像')
        height, width = frame.shape[:2]
        if self.size != (width, height):
            self._prepare(width, height)
        with (self.cv_backend.frame_batch() if self.cv_backend is not None else nullcontext()):
            return self._process_prepared(frame)

    def _process_prepared(self, frame):
        height, width = frame.shape[:2]
        cfg = self.cfg
        if self.cv_backend is not None:
            self.cv_backend.reset_stats()
        started = time.perf_counter()
        maps = self.corrector.maps(cfg.correction_plane_distance_m) if self.corrector else None
        if self.cv_backend is not None:
            preprocess_started = time.perf_counter()
            fixed, enhanced, correction_ms, _ = self.cv_backend.preprocess(
                frame, self.valid_mask, cfg.sharpen_amount, maps)
            corrected_at = preprocess_started+correction_ms/1000
        else:
            fixed = (self.cv_backend.remap(frame, *maps) if self.cv_backend is not None and maps is not None else
                     self.corrector.undistort(frame, cfg.correction_plane_distance_m) if self.corrector else frame.copy())
            corrected_at = time.perf_counter()
            enhanced = sharpen_frame(fixed, cfg.sharpen_amount, self.valid_mask)
        enhanced_at = time.perf_counter()
        candidate, _ = self.tracker.update(fixed, yolo_frame=enhanced, cv_frame=enhanced)
        tracked_at = time.perf_counter()
        status = self.tracker.last_status
        geometry = original_geometry(fixed, candidate)
        detections = [dict(det, selected=det['bbox'] == status['target_bbox'])
                      for det in status['yolo_detections']]
        display = draw_detections(enhanced, detections)
        display = draw_gate_geometry(display, geometry, self.tracker.index-1,
                                      self.fps, 'DoorSim')
        guidance = None
        if self.corrector is not None:
            guidance = build_gate_guidance(status, geometry, self.corrector.output_matrix,
                                           self.size, self.valid_mask)
            display = draw_gate_guidance(display, guidance, self.corrector.output_matrix)
        observation = dict(valid=True, has_target=status['yolo_count'] > 0,
                           coordinate_space='corrected' if self.corrector else 'raw',
                           img_w=width, img_h=height, geometry=geometry, yolo=status,
                           guidance=guidance,
                           input='enhanced', enhancement_mode='sharpen-only',
                           sharpen_amount=cfg.sharpen_amount,
                           cv_backend=dict(self.cv_backend_info),
                           timing_ms=dict(correction=round((corrected_at-started)*1000, 2),
                                          enhancement=round((enhanced_at-corrected_at)*1000, 2),
                                          yolo=round(self.detector.last_detect_ms, 2),
                                          cv=round((tracked_at-enhanced_at)*1000-self.detector.last_detect_ms, 2),
                                          overlay=round((time.perf_counter()-tracked_at)*1000, 2)))
        if self.cv_backend is not None:
            observation['gpu'] = self.cv_backend.diagnostics()
        return display, detections, observation

    def close(self):
        """Wait for line workers and release OpenCL when front exits."""
        with self.lock:
            if not self.closed:
                self.closed = True
                if self.cv_backend is not None:
                    self.cv_backend.close()
