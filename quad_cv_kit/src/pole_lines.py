"""Robust pole centerlines from separate candidates, with full-edge verification."""
import cv2
import numpy as np


def _tls(points):
    points = np.asarray(points, np.float32)
    vx, vy, x, y = cv2.fitLine(points, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    return np.array([x, y]), np.array([vx, vy])


def _centers(mask, origin, direction, box, key, radius):
    vertical = key in ('left', 'right')
    axis = 1 if vertical else 0
    across = 1 - axis
    if abs(direction[axis]) < 0.7:
        return np.empty((0, 2)), 0.0
    low, high = (box[1], box[3]) if vertical else (box[0], box[2])
    # Avoid elbows, where two poles meet. Verify the whole edge separately later.
    span = high - low
    lo = max(low + 0.12 * span, 1)
    hi = min(high - 0.12 * span, mask.shape[axis] - 2)
    if hi <= lo:
        return np.empty((0, 2)), 0.0
    steps = np.linspace(lo, hi, 48)
    offsets = np.arange(-radius, radius + 1)
    expected_points = origin + (steps-origin[axis])[:, None] / direction[axis] * direction
    points = np.repeat(expected_points[:, None, :], len(offsets), axis=1)
    points[..., across] += offsets
    profiles = cv2.remap(mask, points[..., 0].astype(np.float32),
                         points[..., 1].astype(np.float32), cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT) > 0
    observed = []
    for expected, values in zip(expected_points, profiles):
        changes = np.diff(np.r_[False, values, False].astype(np.int8))
        runs = list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))
        choices = []
        for a, b in runs:
            if a == 0 or b == len(offsets) or b - a > 1.5 * radius:
                continue  # Unbounded blobs do not identify a pole center.
            center = 0.5 * (offsets[a] + offsets[b - 1])
            choices.append((abs(center), center))
        if choices:
            _, center = min(choices)
            expected[across] += center
            observed.append(expected)
    return np.asarray(observed), len(observed) / len(steps)


def fit_pole(mask, box, key, band, pixel_scale=1.0):
    """Hough seeds select one physical pole, then fit equally weighted scan centers.

    Scans prevent bright/thick sections from dominating. Huber fitting and residual
    rejection prevent particles and a crossing background pole from pulling a line.
    """
    h, w = mask.shape
    x1, y1, x2, y2 = [int(round(v)) for v in band]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    sub = mask[y1:y2, x1:x2]
    span = (box[3] - box[1]) if key in ('left', 'right') else (box[2] - box[0])
    seeds = cv2.HoughLinesP(sub, 1, np.pi / 180, max(8, int(span * .08)),
                           minLineLength=max(8, int(span * .10)),
                           maxLineGap=max(3, int(span * .06)))
    if seeds is None:
        return None
    # Deterministic seed ordering, prefer long continuous observations.
    seeds = sorted(seeds.reshape(-1, 4), key=lambda s: -(float(s[2])-s[0])**2-(float(s[3])-s[1])**2)
    radius = max(int(8 * pixel_scale), int(min(box[2]-box[0], box[3]-box[1]) * .10))
    axis = 1 if key in ('left', 'right') else 0
    across = 1 - axis
    boundary = {'left': box[0], 'right': box[2], 'top': box[1], 'bottom': box[3]}[key]
    midpoint = (box[1]+box[3])/2 if axis == 1 else (box[0]+box[2])/2
    candidates = []
    signatures = set()
    for seed in seeds[:80]:
        a = np.array(seed[:2], float) + [x1, y1]
        b = np.array(seed[2:], float) + [x1, y1]
        direction = (b-a) / max(np.linalg.norm(b-a), 1e-9)
        if abs(direction[axis]) < .85:
            continue
        at_mid = a[across] + (midpoint-a[axis])*direction[across]/direction[axis]
        signature = (round(at_mid / max(2, pixel_scale * 2)), round(direction[across]/direction[axis] * 20))
        if signature in signatures:
            continue
        signatures.add(signature)
        centers, coverage = _centers(mask, a, direction, box, key, radius)
        if coverage < .52 or len(centers) < 12:
            continue
        origin, direction = _tls(centers)
        residual = np.abs((centers-origin) @ [-direction[1], direction[0]])
        tol = max(1.5 * pixel_scale, .009 * span)
        centers = centers[residual <= tol]
        if len(centers) < 24:
            continue
        origin, direction = _tls(centers)
        centers, coverage = _centers(mask, origin, direction, box, key, radius)
        if coverage < .60:
            continue
        residual = np.abs((centers-origin) @ [-direction[1], direction[0]])
        inliers = residual <= tol
        if np.count_nonzero(inliers) < 27:
            continue
        origin, direction = _tls(centers[inliers])
        at_mid = origin[across] + (midpoint-origin[axis])*direction[across]/direction[axis]
        distance = abs(at_mid-boundary) / max(1, (box[2]-box[0]) if axis == 1 else (box[3]-box[1]))
        # Small YOLO bbox jitter should not change a strongly supported outer pole.
        score = np.count_nonzero(inliers)/48 - .8*distance - float(np.median(residual[inliers])) / span
        candidates.append((score, origin, direction))
    if not candidates:
        return None
    _, origin, direction = max(candidates, key=lambda item: item[0])
    return ((origin-4000*direction).tolist(), (origin+4000*direction).tolist())


def verify_quad(mask, corners, box, tolerance):
    """Reject extrapolated intersections and unsupported full visible segments."""
    q = np.asarray(corners, float)
    bw, bh = box[2]-box[0], box[3]-box[1]
    margin_x, margin_y = max(tolerance*2, .08*bw), max(tolerance*2, .08*bh)
    bounded = bool(np.all(q[:, 0] >= box[0]-margin_x)
                   and np.all(q[:, 0] <= box[2]+margin_x)
                   and np.all(q[:, 1] >= box[1]-margin_y)
                   and np.all(q[:, 1] <= box[3]+margin_y))
    distance = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 3)
    support, residual = {}, {}
    h, w = mask.shape
    for name, a, b in zip(('top', 'right', 'bottom', 'left'), q, np.roll(q, -1, axis=0)):
        pts = a + np.linspace(.06, .94, 100)[:, None] * (b-a)
        inside = (pts[:, 0] >= 0) & (pts[:, 0] <= w-1) & (pts[:, 1] >= 0) & (pts[:, 1] <= h-1)
        pts = pts[inside]
        if len(pts) < 15:
            support[name], residual[name] = 0.0, None
            continue
        distances = cv2.remap(distance, pts[:, 0].astype(np.float32)[None, :],
                              pts[:, 1].astype(np.float32)[None, :], cv2.INTER_LINEAR)[0]
        support[name] = float(np.mean(distances <= tolerance))
        residual[name] = float(np.percentile(distances, 90))
    valid = bounded and min(support.values()) >= .70
    return valid, dict(corners_within_bbox=bounded, full_edge_support=support,
                      edge_distance_p90_px=residual)
