"""Video display stabilization; inference observations remain untouched."""
import copy
import math

import cv2
import numpy as np

from . import quad_cv_det as Q
from .quad_geom import edge_metrics
from .pole_lines import verify_quad


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2-x1)*max(0, y2-y1)
    areas = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])
    return intersection / max(areas-intersection, 1)


def _warp(points, matrix):
    return np.asarray(points) @ matrix[:, :2].T + matrix[:, 2]


class TemporalOverlay:
    """Immediately show new targets; bridge short gaps using measured optical flow.

    Only the currently largest selected target is stabilized. A different largest
    target resets state immediately. Age counts frames since a real observation,
    not frames since a successful prediction, so tracking cannot extend forever.
    """
    def __init__(self, fps=30, hold_seconds=.2, alpha=.65, opts=None):
        if not math.isfinite(fps) or fps <= 0 or not 0 <= hold_seconds <= 1 or not 0 < alpha <= 1:
            raise ValueError('Invalid overlay frame rate, hold duration or smoothing weight')
        self.max_gap = int(round(fps * hold_seconds))
        self.alpha = alpha
        self.opts = opts or {}
        self.target = None
        self.prev_gray = None
        self.features = None
        self.bbox_age = self.quad_age = 0
        self.status = {}

    def _motion(self, gray, scale):
        if (self.prev_gray is None or self.prev_gray.shape != gray.shape
                or self.features is None or len(self.features) < 6):
            return None
        params = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .03))
        forward, ok1, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.features, None, **params)
        if forward is None:
            return None
        backward, ok2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, forward, None, **params)
        if backward is None:
            return None
        good = ((ok1.ravel() > 0) & (ok2.ravel() > 0)
                & (np.linalg.norm(self.features-backward, axis=2).ravel() < 1.5*scale))
        if good.sum() < 6:
            return None
        before, after = self.features[good].reshape(-1, 2), forward[good].reshape(-1, 2)
        matrix, inliers = cv2.estimateAffinePartial2D(before, after, method=cv2.RANSAC,
                                                     ransacReprojThreshold=2*scale)
        if matrix is None or inliers.sum() < 6 or inliers.mean() < .6:
            return None
        magnification = float(np.linalg.norm(matrix[:, 0]))
        if not .85 <= magnification <= 1.18:
            return None
        bbox = self.target['bbox']
        center = np.array([(bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2])
        if np.linalg.norm(_warp([center], matrix)[0]-center) > .15 * np.hypot(bbox[2]-bbox[0], bbox[3]-bbox[1]):
            return None
        return matrix

    def _supported(self, frame, corners, bbox, scale):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        pad = .1 * max(x2-x1, y2-y1)
        rx1, ry1 = max(0, int(x1-pad)), max(0, int(y1-pad))
        rx2, ry2 = min(w, int(x2+pad)+1), min(h, int(y2+pad)+1)
        if rx2 <= rx1 or ry2 <= ry1:
            return False
        mask, _ = Q.red_mask(frame[ry1:ry2, rx1:rx2], self.opts)
        # A whole quadrilateral needs current-frame evidence, even during a gap.
        valid, _ = verify_quad(mask, np.asarray(corners)-[rx1, ry1],
                               np.asarray(bbox)-[rx1, ry1, rx1, ry1], 5*scale)
        return valid and Q.struct_check(np.asarray(corners).tolist(), self.opts, clip_v=True)[0]

    def _remember(self, frame, gray, display, scale):
        self.target = copy.deepcopy(display)
        self.prev_gray = gray
        mask = np.zeros(gray.shape, np.uint8)
        corners = display.get('quad', {}).get('corners')
        if corners:
            cv2.polylines(mask, [np.rint(corners).astype(np.int32)], True, 255, max(5, int(14*scale)))
        else:
            x1, y1, x2, y2 = np.rint(display['bbox']).astype(int)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, max(5, int(14*scale)))
        self.features = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=.01,
                                                minDistance=max(3, int(4*scale)), mask=mask)

    def update(self, frame, detections):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scale = max(frame.shape[1]/640, frame.shape[0]/480)
        observed = next((d for d in detections if d.get('selected')), None)
        matrix = self._motion(gray, scale) if self.target is not None else None
        predicted = copy.deepcopy(self.target)
        if predicted is not None and matrix is not None:
            x1, y1, x2, y2 = predicted['bbox']
            vertices = _warp([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], matrix)
            predicted['bbox'] = np.r_[vertices.min(axis=0), vertices.max(axis=0)].tolist()
            if predicted.get('quad', {}).get('corners'):
                predicted['quad']['corners'] = _warp(predicted['quad']['corners'], matrix).tolist()
        same = (observed is not None and predicted is not None
                and observed.get('class_id') == predicted.get('class_id')
                and observed.get('label') == predicted.get('label')
                and box_iou(observed['bbox'], predicted['bbox']) >= .50)
        if observed is None:
            self.bbox_age += 1
            if predicted is None or matrix is None or self.bbox_age > self.max_gap:
                self.target = None
                self.features = None
                self.prev_gray = gray
                self.status = dict(state='lost', bbox_age_frames=self.bbox_age)
                return []
            display = copy.deepcopy(predicted)
            display['bbox_source'] = 'optical-flow'
            display['observed'] = False
        else:
            display = copy.deepcopy(observed)
            display['observed'] = True
            self.bbox_age = 0
            if same:
                display['bbox'] = (self.alpha*np.asarray(observed['bbox'])
                                   + (1-self.alpha)*np.asarray(predicted['bbox'])).tolist()
            else:
                predicted = None
                self.quad_age = 0

        fresh_quad = observed.get('quad') if observed is not None else None
        fresh_valid = fresh_quad and fresh_quad.get('geometry_valid') and fresh_quad.get('corners')
        previous_corners = (predicted or {}).get('quad', {}).get('corners')
        if fresh_valid:
            self.quad_age = 0
            display['quad'] = copy.deepcopy(fresh_quad)
            display['quad']['observation'] = 'detected'
            if previous_corners and same:
                q = self.alpha*np.asarray(fresh_quad['corners']) + (1-self.alpha)*np.asarray(previous_corners)
                if self._supported(frame, q, observed['bbox'], scale):
                    display['quad']['corners'] = q.tolist()
                    display['quad']['observation'] = 'smoothed'
        else:
            self.quad_age += 1
            display.pop('quad', None)
            if (previous_corners and matrix is not None and self.quad_age <= self.max_gap
                    and self._supported(frame, previous_corners, display['bbox'], scale)):
                quad = copy.deepcopy(predicted['quad'])
                quad.update(corners=previous_corners, lvl=3, geometry_valid=False,
                            observation='tracked', why='current-frame optical flow + red support')
                # Tracking measurements are display estimates, not current CV observations.
                quad['fields'] = {}
                display['quad'] = quad
        if display.get('quad', {}).get('corners'):
            quad = display['quad']
            quad['edges'] = edge_metrics(quad['corners'])
            quad['fields'] = dict(edges=quad['edges'], display_estimate=True)
        display['tracking'] = dict(bbox_age_frames=self.bbox_age, quad_age_frames=self.quad_age,
                                    flow_valid=matrix is not None)
        self.status = dict(state='observed' if observed is not None else 'tracked',
                           **display['tracking'])
        self._remember(frame, gray, display, scale)
        return [display]
