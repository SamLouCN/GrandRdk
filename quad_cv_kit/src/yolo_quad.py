"""YOLO boxes -> OpenCV quadrilateral and image-plane measurements."""
import os

# Configure the inference process before importing Ultralytics / PyTorch.
os.environ['PYTORCH_TUNABLEOP_ENABLED'] = '1'
os.environ['PYTORCH_TUNABLEOP_TUNING_DURATION'] = 'short'
os.environ['MIOPEN_FIND_MODE'] = 'FAST'
os.environ['PYTORCH_MIOPEN_SUGGEST_NHWC'] = '0'         # Never be deleted
os.environ['TORCH_BLAS_PREFER_HIPBLASLT'] = '0'         # Not related
os.environ['MIOPEN_DEBUG_CONV_DIRECT'] = '0'            # Not related
# os.environ['MIOPEN_DEBUG_CONV_WINOGRAD'] = '0'        # On with large improvement
os.environ['MIOPEN_DEBUG_CONV_IMPLICIT_GEMM'] = '0'     # Never be deleted

from pathlib import Path
import math

import cv2
import numpy as np

from . import quad_cv_det as Q
from .quad_geom import edge_metrics

DEFAULT_WEIGHTS = Path(__file__).resolve().parents[1] / 'model' / 'best.pt'
DEFAULT_CLASSES = ('door', 'gate', 'ring')
CORNER_CLASSES = ('upper-left', 'upper-right', 'lower-right', 'lower-left')


def _label(name):
    return str(name).strip().lower().replace('_', '-')


def detect_in_box(frame, bbox, opts=None, ball_boxes=()):
    """Measure the CV polygon even if its structure check fails; retain its lvl."""
    quad = Q.detect(frame, bbox, opts=opts, ball_boxes=ball_boxes)
    quad['edges'] = edge_metrics(quad['corners'])
    quad['geometry_valid'] = quad['lvl'] == 4 and bool(quad['corners'])
    return quad


class GateQuadProcessor:
    """Immediately run OpenCV on the largest whole-door box in each frame."""

    def __init__(self, classes=DEFAULT_CLASSES, opts=None):
        self.classes = {_label(c) for c in classes}
        self.opts = opts
        self.last_status = {}

    def process(self, frame, detections):
        # Each call consumes current-frame detections, never cached polygons.
        for det in detections:
            for key in ('selected', 'lock_state', 'quad'):
                det.pop(key, None)
        candidates = [d for d in detections if _label(d['label']) in self.classes
                      or str(d.get('class_id', '')) in self.classes]
        candidates = [d for d in candidates if len(d['bbox']) == 4
                      and all(math.isfinite(v) for v in d['bbox'])
                      and d['bbox'][2] > d['bbox'][0] and d['bbox'][3] > d['bbox'][1]]
        self.last_status = dict(mode='whole-door', selection='largest-area',
                                state='searching', why='no-door', cv_enabled=False)
        if not candidates:
            return detections
        def area(det):
            x1, y1, x2, y2 = det['bbox']
            return (x2 - x1) * (y2 - y1)
        target = max(candidates, key=lambda d: (area(d), d['score']))
        self.last_status.update(state='processing', why='largest-door', cv_enabled=True,
                                bbox=list(target['bbox']), area_px2=area(target))
        target['selected'] = True
        balls = [d['bbox'] for d in detections
                 if _label(d['label']) in ('red-ball', 'ball')]
        target['quad'] = detect_in_box(frame, target['bbox'], self.opts, balls)
        return detections


class YoloQuadDetector(GateQuadProcessor):
    """Load the whole-door checkpoint once; do not combine corner boxes."""

    def __init__(self, weights=DEFAULT_WEIGHTS, classes=DEFAULT_CLASSES,
                 conf=0.25, iou=0.7, imgsz=640, device=None, opts=None, model=None):
        if model is None:
            weights = Path(weights).resolve()
            if not weights.is_file():
                raise FileNotFoundError(f'Model not found: {weights}')
            from ultralytics import YOLO
            model = YOLO(str(weights))
        self.model = model
        names = model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        self.names = {int(i): str(n) for i, n in items}
        requested = {_label(c) for c in classes}
        self.target_ids = {i for i, n in self.names.items()
                           if _label(n) in requested or str(i) in requested}
        if not self.target_ids:
            raise ValueError(f'No target class {classes} in model classes {self.names}. '
                             'A whole-door checkpoint is required; set --classes '
                             'to its gate class name or ID.')
        if any(_label(self.names[i]) in CORNER_CLASSES for i in self.target_ids):
            raise ValueError('Corner checkpoints cannot provide whole-door boxes. '
                             'Use a checkpoint trained for the complete door.')
        super().__init__([str(i) for i in self.target_ids], opts)
        self.predict_opts = dict(conf=conf, iou=iou, imgsz=imgsz, verbose=False)
        if device is not None:
            self.predict_opts['device'] = device

    def detect_boxes(self, frame):
        """Current whole-door boxes without running a CV fitting stage."""
        result = self.model.predict(source=frame, **self.predict_opts)[0]
        boxes = result.boxes
        detections = []
        if boxes is not None and len(boxes):
            bboxes = boxes.xyxy.detach().cpu().tolist()
            scores = boxes.conf.detach().cpu().tolist()
            class_ids = boxes.cls.detach().cpu().tolist()
            detections = [dict(class_id=int(i), label=self.names[int(i)],
                               score=float(s), bbox=list(map(float, b)), bbox_source='yolo')
                          for b, s, i in zip(bboxes, scores, class_ids)]
        return detections

    def detect(self, frame):
        return self.process(frame, self.detect_boxes(frame))

def find_bbox_red(frame, min_area=600):
    """Optional CV-only locator: the largest red connected component."""
    mask, _ = Q.red_mask(frame)
    if mask is None or not np.any(mask):
        return None
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None
    x, y, w, h, area = stats[1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))]
    return [int(x), int(y), int(x + w), int(y + h)] if area >= min_area else None


def _text(image, message, x, y, color=(255, 255, 255), scale=0.45, min_y=0):
    (w, h), _ = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = max(0, min(int(x), image.shape[1] - w - 1))
    y = max(h + 2, min_y, min(int(y), image.shape[0] - 3))
    cv2.putText(image, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def project_detection_geometry(detections, point_mapper=None):
    """Display geometry from one corrected-frame inference, optionally on raw pixels.

    Sample each edge before the nonlinear mapping. Invalid mapping regions are
    split into separate paths instead of drawing a spurious connecting line.
    """
    def mapped(points):
        points = np.asarray(points, dtype=float)
        return points if point_mapper is None else point_mapper(points)

    def point(p):
        q = mapped([p])[0]
        return q.tolist() if np.isfinite(q).all() else None

    def paths(vertices):
        vertices = np.asarray(vertices, dtype=float)
        result = []
        for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
            samples = a + np.linspace(0, 1, 33 if point_mapper else 2)[:, None] * (b-a)
            run = []
            for p in mapped(samples):
                if np.isfinite(p).all():
                    run.append(p.tolist())
                else:
                    if len(run) > 1:
                        result.append(run)
                    run = []
            if len(run) > 1:
                result.append(run)
        return result

    geometries = []
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        bbox_paths = paths([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
        geometry = dict(bbox_paths=bbox_paths, quad_paths=[], corners=[], edge_labels=[],
                        label_anchor=point([x1, y1 - 7]))
        quad = det.get('quad')
        if quad and quad['corners']:
            points = np.asarray(quad['corners'], dtype=float)
            geometry['quad_paths'] = paths(points)
            geometry['corners'] = [point(p) for p in points]
            center = points.mean(axis=0)
            for i, (name, metric) in enumerate(quad['edges'].items()):
                mid = (points[i] + points[(i + 1) % 4]) / 2
                direction = mid - center
                direction /= max(1.0, float(np.linalg.norm(direction)))
                geometry['edge_labels'].append(dict(name=name, metric=metric,
                                                    anchor=point(mid + 18 * direction)))
        geometries.append(geometry)
    return geometries


def draw_detections(frame, detections, geometries=None, header=None):
    """Yellow boxes; green fitted polygons; red corners; side lengths/angles."""
    vis = frame.copy()
    geometries = project_detection_geometry(detections) if geometries is None else geometries
    if len(geometries) != len(detections):
        raise ValueError('Display geometry must match detections')
    def draw_paths(paths, color):
        for path in paths:
            pts = np.rint(np.clip(path, -1000000, 1000000)).astype(np.int32)
            cv2.polylines(vis, [pts], False, color, 2, cv2.LINE_AA)

    for det, geometry in zip(detections, geometries):
        draw_paths(geometry['bbox_paths'], (0, 255, 255))
        quad = det.get('quad')
        message = f"{det['label']} {det['score']:.2f}"
        if quad:
            message += f" lvl={quad['lvl']}"
            if quad['corners'] and not quad['geometry_valid']:
                message += ' fit only'
        if det.get('selected'):
            message += ' [selected]'
        if det.get('observed') is False:
            message += ' [YOLO gap: tracked]'
        if quad and quad.get('observation') == 'tracked':
            message += ' [CV tracked]'
        anchor = geometry['label_anchor']
        if anchor is None:
            anchor = geometry['bbox_paths'][0][0] if geometry['bbox_paths'] else [8, 22]
        _text(vis, message, *anchor, (0, 255, 255), min_y=40 if header else 0)
        if not quad or not quad['corners']:
            continue
        draw_paths(geometry['quad_paths'], (0, 255, 0))
        for p in geometry['corners']:
            if p is not None:
                cv2.circle(vis, tuple(np.rint(np.clip(p, -1000000, 1000000)).astype(int)),
                           4, (0, 0, 255), -1)
        for edge in geometry['edge_labels']:
            name, metric, anchor = edge['name'], edge['metric'], edge['anchor']
            if anchor is None:
                continue
            text = f"{name}: {metric['length_px']:.1f}px {metric['angle_deg']:+.1f}deg"
            _text(vis, text, anchor[0], anchor[1], (0, 255, 0), scale=0.4,
                  min_y=40 if header else 0)
    if header:
        cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, 25), (0, 0, 0), -1)
        _text(vis, header, 8, 18)
    return vis

