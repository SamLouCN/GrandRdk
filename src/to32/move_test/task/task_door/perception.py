"""前视进程里的纯视觉处理；无需 PyTorch，复用板端已有 HBM 检测器。

process(raw_bgr, detector, now, reset_token) -> (corrected_bgr, detections, observation)
YOLO 与 CV 的几何全部位于同一张校正图，交点外推沿用算法库的质量限制。
"""
import math
import sys
from pathlib import Path

import numpy as np

from .config import CONFIG

_ROOT = str(Path(__file__).resolve().parents[5])
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quad_cv_kit.src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from quad_cv_kit.src.detect_red_gate import RedGateTracker, original_geometry
from quad_cv_kit.src.gate_guidance import enhance_cv_contrast, complete_gate_edges, estimate_alignment
from quad_cv_kit.src.yolo_red_gate import boundary_sides, search_region, box_area, box_iou


class DoorFrameProcessor:
    def __init__(self, cfg=CONFIG):
        self.cfg = cfg.validate()
        self.corrector = None
        self.reset_token = None
        self.serial = 0
        self.reset()

    def reset(self):
        self.target = None
        self.last_seen = None
        self.cv = RedGateTracker(detect_every=self.cfg.cv_every_frames, hold_seconds=0)

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

    def process(self, frame, detector, now, reset_token=None):
        cfg = self.cfg
        h, w = frame.shape[:2]
        if (w, h) != (cfg.image_width, cfg.image_height):
            raise ValueError('穿门要求输入 %d×%d，实际 %d×%d' %
                             (cfg.image_width, cfg.image_height, w, h))
        if reset_token != self.reset_token:
            self.reset()
            self.reset_token = reset_token
        if self.corrector is None:
            camera, _ = adapt_camera_params(load_camera_params(cfg.camera_params_path), w, h, cfg.camera_fit)
            self.corrector = create_corrector(camera)
            self.valid_mask = self.corrector.valid_mask(cfg.correction_plane_distance_m)
        fixed = self.corrector.undistort(frame, cfg.correction_plane_distance_m)
        # 校正后必须重新转模型输入，不能复用原始相机 NV12。
        detections = detector.detect(fixed, nv12=None)
        chosen = self._select(detections, now)
        obs = dict(valid=True, has_target=chosen is not None, reset_token=reset_token,
                   coordinate_space='corrected', img_w=w, img_h=h,
                   target_id=self.serial if chosen else None, pose=None, corners=None,
                   geometry=None, inferred_segments=[], pose_reason='no-target')
        if chosen is None:
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
        enhanced = enhance_cv_contrast(fixed, cfg.contrast_gain, self.valid_mask,
                                        clahe_clip=0, clahe_blend=0,
                                        sharpen_amount=cfg.sharpen_amount, saturation_gain=1)
        region = search_region(box, (w, h), bool(sides), cfg.roi_padding)
        candidate, _ = self.cv.update(enhanced, region, box, reference_frame=fixed,
                                      valid_mask=self.valid_mask)
        geometry = original_geometry(fixed, candidate)
        obs['geometry'] = geometry
        # 跟踪旧四点不计入有效姿态；补端交点可用，但必须来自四条当前实测边。
        if geometry is not None and geometry['observation'] == 'detected':
            completed, reason = complete_gate_edges(geometry, box, (w, h), self.valid_mask)
            obs['pose_reason'] = reason
            if completed is not None:
                obs.update(corners=completed['corners'], inferred_segments=completed['segments'])
                pose, reason = estimate_alignment(completed['corners'], matrix,
                                                  (cfg.gate_width_m, cfg.gate_height_m))
                obs['pose_reason'] = reason
                if pose is not None:
                    rotation = np.asarray(cfg.camera_to_robot_rotation, float)
                    if (rotation.shape != (3, 3) or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5)
                            or not np.isclose(np.linalg.det(rotation), 1)):
                        raise ValueError('camera_to_robot_rotation 必须是旋转矩阵')
                    center = rotation@np.asarray(pose['gate_center_model_units']) + cfg.camera_position_robot_m
                    normal = rotation@np.asarray(pose['gate_normal_camera'])
                    movement = center-float(center@normal)*normal
                    tilt = math.degrees(math.atan2(abs(normal[1]), math.hypot(normal[0], normal[2])))
                    gate_x = rotation@np.asarray(pose['rotation_matrix'])[:, 0]
                    roll = abs(math.degrees(math.atan2(gate_x[1], gate_x[0])))
                    pose.update(metric_distance_available=True, gate_dimensions_measured_m=[cfg.gate_width_m, cfg.gate_height_m],
                                center_robot_m=center.tolist(), normal_robot=normal.tolist(),
                                alignment_robot_m=movement.tolist(),
                                center_offset_robot_px=[matrix[0, 0]*center[0]/center[2],
                                                        matrix[1, 1]*center[1]/center[2]],
                                yaw_error_deg=math.degrees(math.atan2(normal[0], normal[2])),
                                unactuated_tilt_deg=max(tilt, roll),
                                controllable=normal[2] > 0 and max(tilt, roll) <= cfg.unactuated_tilt_max_deg)
                    obs['pose'] = pose
        return fixed, detections, obs
