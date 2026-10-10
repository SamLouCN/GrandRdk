"""YOLO target selection with ROI-restricted red-pipe CV and short gap tracking."""
import numpy as np

from .detect_red_gate import RedGateTracker
from .gate_guidance import enhance_cv_contrast


def box_area(box):
    return max(0, box[2]-box[0]) * max(0, box[3]-box[1])


def box_iou(a, b):
    intersection = box_area([max(a[0], b[0]), max(a[1], b[1]),
                             min(a[2], b[2]), min(a[3], b[3])])
    union = box_area(a)+box_area(b)-intersection
    return intersection/union if union > 0 else 0.0


def same_target(a, b):
    if box_iou(a['bbox'], b['bbox']) < .15:
        return False
    if a['clipped'] and b['clipped']:
        return True
    areas = [box_area(a['bbox']), box_area(b['bbox'])]
    # A nested background door should not inherit a foreground door's tracker.
    return min(areas)/max(areas) >= .4


def boundary_sides(box, size, valid_mask=None, margin=4):
    """Missing sides inferred only from contact with the valid image boundary."""
    width, height = size
    x1, y1, x2, y2 = box
    sides = []
    for side, touches in [('left', x1 <= margin), ('right', x2 >= width-margin),
                          ('up', y1 <= margin), ('down', y2 >= height-margin)]:
        if touches:
            sides.append(side)
    if valid_mask is None:
        return sides
    # Sample each side: corrected frames can have black margins inside the canvas.
    corners = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
    for side, a, b in zip(('up', 'right', 'down', 'left'), corners,
                          np.roll(corners, -1, axis=0)):
        points = np.rint(np.linspace(a, b, 17)).astype(int)
        xs = np.clip(points[:, 0], 0, width-1)
        ys = np.clip(points[:, 1], 0, height-1)
        # Ignore isolated black corner samples when deriving a travel direction.
        if np.mean(~valid_mask[ys, xs]) >= .2 and side not in sides:
            sides.append(side)
    return sides


def touches_boundary(box, size, valid_mask=None, margin=4):
    return bool(boundary_sides(box, size, valid_mask, margin))


def search_region(box, size, clipped, padding=.08):
    """Small ordinary ROI; wider ROI when YOLO touches an input boundary."""
    width, height = size
    bw, bh = box[2]-box[0], box[3]-box[1]
    px, py = max(4, bw*padding), max(4, bh*padding)
    if clipped:
        px, py = max(px, width*.25), max(py, height*.25)
    return [max(0, box[0]-px), max(0, box[1]-py),
            min(width, box[2]+px), min(height, box[3]+py)]


class YoloRedGateTracker:
    """Largest YOLO door; retain a matched clipped foreground target.

    CV still ranks supported pipes by width inside the chosen search region,
    including overflow beyond a clipped box. YOLO gaps cannot renew the hold
    timer through repeated CV detections.
    """

    def __init__(self, detector, fps=30, detect_every=3, hold_seconds=.2,
                 valid_mask=None, roi_padding=.08, cv_contrast=1.2,
                 cv_clahe_clip=2.0, cv_clahe_blend=.6,
                 cv_sharpen=.6, cv_saturation=1.25, profile_cv=False, cv_backend=None, adaptive_search=True,
                 cv_execution='hybrid'):
        self.detector = detector
        self.cv_execution = cv_execution
        if cv_execution not in ('hybrid', 'resident'):
            raise ValueError('CV execution must be hybrid or resident')
        if cv_execution == 'resident' and cv_backend is None:
            raise ValueError('Resident CV requires an explicit OpenCL backend')
        self.cv = RedGateTracker(fps, detect_every, hold_seconds, profile=profile_cv, backend=cv_backend,
                                 adaptive_search=adaptive_search)
        if cv_execution == 'resident':
            from .gpu_pipeline import ResidentGatePipeline
            self.cv = ResidentGatePipeline(cv_backend, fps, detect_every, hold_seconds, roi_padding)
        self.cv_backend = cv_backend
        self.valid_mask = valid_mask
        self.roi_padding = roi_padding
        self.cv_contrast = cv_contrast
        self.cv_clahe_clip = cv_clahe_clip
        self.cv_clahe_blend = cv_clahe_blend
        self.cv_sharpen = cv_sharpen
        self.cv_saturation = cv_saturation
        self.last_cv_frame = None
        self.hold_frames = int(round(fps*hold_seconds))
        self.target = None
        self.last_yolo = None
        self.index = 0
        self.last_status = {}

    def _boxes(self, frame):
        size = (frame.shape[1], frame.shape[0])
        boxes = []
        for detection in self.detector.detect_boxes(frame):
            if detection.get('class_id') not in self.detector.target_ids:
                continue
            box = np.asarray(detection['bbox'], float)
            if box.shape != (4,) or not np.isfinite(box).all():
                continue
            box = np.clip(box, [0, 0, 0, 0], [*size, *size]).tolist()
            if box_area(box) <= 0:
                continue
            sides = boundary_sides(box, size, self.valid_mask)
            boxes.append(dict(detection, bbox=box, boundary_sides=sides, clipped=bool(sides)))
        return sorted(boxes, key=lambda d: (box_area(d['bbox']), d['score']), reverse=True)

    def update(self, frame, *, yolo_frame=None, cv_frame=None):
        """Keep clean color evidence while optionally supplying enhanced inputs."""
        if self.cv_execution == 'resident':
            enhanced = frame if cv_frame is None else cv_frame
            inputs = self.detector.detect_boxes(frame if yolo_frame is None else yolo_frame)
            with self.cv_backend.frame_batch():
                selected, lines = self.cv.update(frame, enhanced, inputs, self.detector.target_ids, self.valid_mask)
            self.last_status = self.cv.last_status
            self.last_cv_frame = enhanced
            self.index += 1
            return selected, lines
        boxes = self._boxes(frame if yolo_frame is None else yolo_frame)
        proposal = boxes[0] if boxes else None
        reason = 'largest-yolo-area' if proposal else 'yolo-gap'
        old = self.target
        # Associate by IoU rather than containment: a far nested door is not the old door.
        matches = [box for box in boxes if same_target(box, old)] if old else []
        # Prefer a boundary-clipped continuation over a complete nested gate.
        match = max(matches, key=lambda d: (old['clipped'] and d['clipped'],
                                            box_iou(d['bbox'], old['bbox']))) if matches else None
        matched = match is not None
        within_hold = self.last_yolo is not None and self.index-self.last_yolo <= self.hold_frames
        if old is not None and old['clipped'] and within_hold:
            if matched and match['clipped']:
                proposal = match
                reason = 'clipped-target-continuity'
            elif self.cv.previous is not None and not matched:
                # Do not jump to the background immediately when near-door YOLO disappears.
                proposal = None
                reason = 'clipped-target-gap'
        switched = bool(proposal is not None and old is not None
                        and not same_target(proposal, old))
        if switched:
            self.cv.reset()
        if proposal is not None:
            self.target = proposal
            self.last_yolo = self.index
        elif not within_hold:
            self.target = None
            self.cv.reset()
        region = None
        allow_detect = proposal is not None
        if self.target is not None:
            region = search_region(self.target['bbox'], (frame.shape[1], frame.shape[0]),
                                   self.target['clipped'], self.roi_padding)
        if cv_frame is None:
            cv_frame = enhance_cv_contrast(frame, self.cv_contrast, self.valid_mask,
                                           self.cv_clahe_clip, self.cv_clahe_blend,
                                           self.cv_sharpen, self.cv_saturation, backend=self.cv_backend)
        self.last_cv_frame = cv_frame
        selected, lines = self.cv.update(cv_frame, region,
                                         self.target['bbox'] if self.target else None,
                                         allow_detect=allow_detect,
                                         prefer_previous_width=reason=='clipped-target-continuity',
                                         reference_frame=frame, valid_mask=self.valid_mask)
        self.last_status = dict(self.cv.last_status, yolo_count=len(boxes),
                                yolo_detections=boxes,
                                target_bbox=self.target['bbox'] if self.target else None,
                                target_score=self.target['score'] if self.target else None,
                                target_clipped=self.target['clipped'] if self.target else False,
                                target_boundary_sides=self.target['boundary_sides'] if self.target else [],
                                search_bbox=region, selection_reason=reason,
                                target_switched=switched,
                                yolo_age_frames=self.index-self.last_yolo
                                if self.target and self.last_yolo is not None else None,
                                cv_enabled=allow_detect)
        self.index += 1
        return selected, lines
