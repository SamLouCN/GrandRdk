"""干净校正图保留颜色证据；增强图同时进入 YOLO 和 OpenCV。"""
import sys
from pathlib import Path

import numpy as np

from .config import CONFIG

_ROOT = str(Path(__file__).resolve().parents[3])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quad_cv_kit.src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from quad_cv_kit.src.detect_red_gate import original_geometry, draw_gate_geometry
from quad_cv_kit.src.gate_guidance import enhance_cv_contrast, build_gate_guidance, draw_gate_guidance
from quad_cv_kit.src.yolo_quad import draw_detections
from quad_cv_kit.src.yolo_red_gate import YoloRedGateTracker


class DoorDetectorAdapter:
    """适配板端 YoloDetector.detect(BGR, nv12=None)，仅保留整门类别。"""
    target_ids = {0}

    def __init__(self, detector, min_score):
        self.detector = detector
        self.min_score = min_score

    def detect_boxes(self, frame):
        result = []
        # 图像已校正、增强，必须重新转模型输入，不能复用采集时的 NV12。
        for det in self.detector.detect(frame, nv12=None):
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

    def _prepare(self, width, height):
        self.size = (width, height)
        cfg = self.cfg
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
                                          roi_padding=cfg.roi_padding)

    def process(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError('输入必须为 uint8 BGR 图像')
        height, width = frame.shape[:2]
        if self.size != (width, height):
            self._prepare(width, height)
        cfg = self.cfg
        fixed = (self.corrector.undistort(frame, cfg.correction_plane_distance_m)
                 if self.corrector else frame.copy())
        enhanced = enhance_cv_contrast(fixed, cfg.contrast_gain, self.valid_mask,
                                       cfg.clahe_clip, cfg.clahe_blend,
                                       cfg.sharpen_amount, cfg.saturation_gain)
        candidate, _ = self.tracker.update(fixed, yolo_frame=enhanced, cv_frame=enhanced)
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
                           input='enhanced', contrast_gain=cfg.contrast_gain,
                           sharpen_amount=cfg.sharpen_amount)
        return display, detections, observation
