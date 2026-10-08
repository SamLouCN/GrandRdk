"""CV contrast and UI-only gate completion / camera-frame alignment estimates."""
import math

import cv2
import numpy as np

from .detect_red_gate import project_gate_geometry


AXES = 'camera: X right, Y down, Z forward; right-hand rotations Rz*Ry*Rx'


def enhance_cv_contrast(frame, gain=1.12, valid_mask=None):
    """Mild HSV-value contrast; preserve hue/saturation, input pixels and black margins."""
    if not math.isfinite(gain) or not 1 <= gain <= 1.5:
        raise ValueError('CV contrast gain must be finite and in [1, 1.5]')
    if gain == 1:
        return frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    active = hsv[:, :, 2] > 0
    if valid_mask is not None:
        active &= valid_mask
    if not active.any():
        return frame.copy()
    value = hsv[:, :, 2].astype(float)
    pivot = float(np.median(value[active]))
    hsv[:, :, 2] = np.rint(np.clip((value-pivot)*gain+pivot, 0, 255)).astype(np.uint8)
    enhanced = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    enhanced[~active] = frame[~active]
    return enhanced


def _line(segment):
    a, b = np.asarray(segment, float)
    direction = b-a
    length = np.linalg.norm(direction)
    if length < 20 or not np.isfinite(segment).all():
        return None
    direction /= length
    normal = np.array([-direction[1], direction[0]])
    return dict(segment=np.array([a, b]), normal=normal, offset=float(a@normal),
                vertical=abs(direction[1]) > abs(direction[0]), direction=direction)


def _intersection(a, b):
    matrix = np.stack([a['normal'], b['normal']])
    if abs(np.linalg.det(matrix)) < .3:
        return None
    return np.linalg.solve(matrix, [a['offset'], b['offset']])


def complete_gate_edges(geometry, box, size, valid_mask=None):
    """Intersect four measured lines; return only missing end intervals as inferred edges."""
    if geometry is None:
        return None, 'no-cv-edges'
    segments = geometry.get('observed_segments', geometry['segments'])
    if len(segments) != 4:
        return None, 'need-four-cv-edges'
    lines = [_line(segment) for segment in segments]
    if any(line is None for line in lines):
        return None, 'invalid-line'
    vertical = sorted([line for line in lines if line['vertical']],
                      key=lambda line: line['segment'][:, 0].mean())
    horizontal = sorted([line for line in lines if not line['vertical']],
                        key=lambda line: line['segment'][:, 1].mean())
    if len(vertical) != 2 or len(horizontal) != 2:
        return None, 'need-two-horizontal-two-vertical'
    if any(abs(group[0]['direction']@group[1]['direction']) < .8
           for group in (vertical, horizontal)):
        return None, 'inconsistent-opposite-lines'
    corners = [_intersection(vertical[0], horizontal[0]),
               _intersection(vertical[1], horizontal[0]),
               _intersection(vertical[1], horizontal[1]),
               _intersection(vertical[0], horizontal[1])]
    if any(corner is None for corner in corners):
        return None, 'parallel-lines'
    corners = np.asarray(corners)
    contour = corners.astype(np.float32)
    width, height = size
    if (not np.isfinite(corners).all() or not cv2.isContourConvex(contour)
            or cv2.contourArea(contour) < 400
            or np.any(corners < 0) or np.any(corners > [width-1, height-1])):
        return None, 'invalid-intersections'
    box = np.asarray(box, float).reshape(2, 2)
    padding = np.maximum(8, (box[1]-box[0])*.15)
    if np.any(corners < box[0]-padding) or np.any(corners > box[1]+padding):
        return None, 'corners-outside-target'
    inferred = []
    ordered_lines = [horizontal[0], vertical[1], horizontal[1], vertical[0]]
    for index, line in enumerate(ordered_lines):
        a, b = corners[index], corners[(index+1) % 4]
        length = np.linalg.norm(b-a)
        direction = (b-a)/length
        positions = np.sort((line['segment']-a)@direction)
        low, high = np.clip(positions, 0, length)
        if high-low < length*.45 or max(low, length-high) > max(20, length*.35):
            return None, 'excessive-extrapolation'
        points = np.linspace(a, b, 33)
        if valid_mask is not None:
            indices = np.rint(points).astype(int)
            if not valid_mask[indices[:, 1], indices[:, 0]].all():
                return None, 'completion-crosses-invalid-image'
        if low > 1.5:
            inferred.append([a.tolist(), (a+direction*low).tolist()])
        if length-high > 1.5:
            inferred.append([(a+direction*high).tolist(), b.tolist()])
    return dict(corners=corners.tolist(), segments=inferred, observation='inferred'), 'four-line-intersections'


def _euler_xyz(rotation):
    """Degrees with R = Rz(z) @ Ry(y) @ Rx(x), camera local right-hand axes."""
    y = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    x = math.atan2(rotation[2, 1], rotation[2, 2])
    z = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.degrees([x, y, z])


def estimate_alignment(corners, camera_matrix, gate_size=(.7, .5)):
    """Planar PnP; translate onto the gate-normal axis, then rotate camera by R.

    Pixel equivalents are K focal lengths times camera translation / center depth.
    Z is a virtual pixel scale, never an observed image-space depth coordinate.
    """
    width, height = gate_size
    matrix = np.asarray(camera_matrix, float)
    object_points = np.array([[-width/2, -height/2, 0], [width/2, -height/2, 0],
                              [width/2, height/2, 0], [-width/2, height/2, 0]], float)
    image_points = np.asarray(corners, float)
    if (image_points.shape != (4, 2) or not np.isfinite(image_points).all()
            or matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or min(width, height, matrix[0, 0], matrix[1, 1]) <= 0):
        return None, 'invalid-pose-input'
    try:
        result = cv2.solvePnPGeneric(object_points, image_points, matrix, np.zeros(5),
                                     flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None, 'pnp-failed'
    candidates = []

    def add_candidate(rvec, tvec):
        rotation = cv2.Rodrigues(rvec)[0]
        translation = tvec.reshape(3)
        normal = rotation[:, 2]
        if (not np.isfinite(rotation).all() or not np.isfinite(translation).all()
                or np.min((object_points@rotation.T+translation)[:, 2]) <= 0
                or normal[2] <= .1 or normal@translation <= 0):
            return
        projected = cv2.projectPoints(object_points, rvec, tvec, matrix, np.zeros(5))[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-image_points)**2, axis=1))))
        candidates.append((error, rotation, translation))
    for rvec, tvec in zip(result[1], result[2]):
        add_candidate(rvec, tvec)
    # IPPE can return NaNs for an exactly front-facing symmetric rectangle.
    if not candidates:
        try:
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, matrix, np.zeros(5),
                                        flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                add_candidate(rvec, tvec)
        except cv2.error:
            pass
    if not candidates:
        return None, 'no-front-facing-pose'
    candidates.sort(key=lambda item: item[0])
    error, rotation, translation = candidates[0]
    threshold = max(2, np.linalg.norm(image_points[1]-image_points[0])*.015)
    if error > threshold:
        return None, 'poor-pose-fit'
    if len(candidates) > 1:
        second_error, second_rotation, _ = candidates[1]
        difference = math.degrees(math.acos(float(np.clip(rotation[:, 2]@second_rotation[:, 2], -1, 1))))
        if difference > 10 and second_error-error < .5:
            return None, 'ambiguous-planar-pose'
    normal = rotation[:, 2]
    normal_distance = float(translation@normal)
    # Move to the closest point on the line through the center along its normal.
    movement = translation-normal_distance*normal
    focal = np.array([matrix[0, 0], matrix[1, 1], (matrix[0, 0]+matrix[1, 1])/2])
    pixel_equivalent = focal*movement/translation[2]
    center = matrix@translation
    center = center[:2]/center[2]
    return dict(rotation_xyz_deg=_euler_xyz(rotation).tolist(),
                rotation_matrix=rotation.tolist(), gate_center_camera_m=translation.tolist(),
                gate_normal_camera=normal.tolist(), alignment_translation_camera_m=movement.tolist(),
                translation_pixel_equivalent_xyz=pixel_equivalent.tolist(),
                translation_z_unit='virtual pixel equivalent: mean(fx,fy)*dz/center_depth',
                gate_center_px=center.tolist(), center_offset_xy_px=(center-matrix[:2, 2]).tolist(),
                normal_distance_m=normal_distance, reprojection_rms_px=error,
                gate_size_m=list(gate_size), axes=AXES, estimated=True), 'pose-estimated'


def build_gate_guidance(status, geometry, camera_matrix, size, valid_mask=None, gate_size=(.7, .5)):
    result = dict(mode='no-target', reason='no-current-yolo-target', ui_only=True,
                  coordinate_space='corrected', missing_sides=[], direction_xy=None,
                  completion=None, alignment=None, axes=AXES)
    if status.get('target_bbox') is None:
        return result
    if status.get('yolo_age_frames') != 0:
        result.update(mode='yolo-gap', reason='wait-for-current-yolo')
        return result
    if status.get('target_clipped'):
        sides = status.get('target_boundary_sides', [])
        vector = np.array([int('right' in sides)-int('left' in sides),
                           int('down' in sides)-int('up' in sides)], float)
        direction = (vector/np.linalg.norm(vector)).tolist() if np.linalg.norm(vector) else None
        result.update(mode='clipped', reason='target-touches-valid-image-boundary',
                      missing_sides=sides, direction_xy=direction)
        return result
    completion, reason = complete_gate_edges(geometry, status['target_bbox'], size, valid_mask)
    if completion is None:
        result.update(mode='waiting-geometry', reason=reason)
        return result
    alignment, reason = estimate_alignment(completion['corners'], camera_matrix, gate_size)
    result.update(mode='align' if alignment else 'pose-unavailable', completion=completion,
                  alignment=alignment, reason=reason,
                  geometry_observation=geometry['observation'])
    return result


def draw_gate_guidance(frame, guidance, camera_matrix, point_mapper=None):
    """Light green extrapolations, directional arrow, signed values; never issue controls."""
    vis = frame.copy()
    completion = guidance['completion']
    if completion and completion['segments']:
        overlay = dict(segments=completion['segments'], corners=[], complete=False, observation='inferred')
        projected = project_gate_geometry(overlay, point_mapper or (lambda points: points),
                                          (frame.shape[1], frame.shape[0]))
        for path in projected['edge_paths']:
            cv2.polylines(vis, [np.rint(path).astype(np.int32)], False, (150, 255, 150), 3, cv2.LINE_AA)

    def arrow(direction, color):
        if direction is None or np.linalg.norm(direction) < 1e-6:
            return
        start = np.asarray(camera_matrix, float)[:2, 2]
        delta = np.asarray(direction, float)
        delta = delta/np.linalg.norm(delta)*min(100, frame.shape[0]*.15)
        points = np.stack([start, start+delta])
        if point_mapper is not None:
            points = np.asarray(point_mapper(points))
        if np.isfinite(points).all():
            cv2.arrowedLine(vis, tuple(np.rint(points[0]).astype(int)),
                            tuple(np.rint(points[1]).astype(int)), color, 4, cv2.LINE_AA, tipLength=.25)

    mode = guidance['mode']
    text_lines = ['UI ONLY | X right, Y down, Z forward | light green = inferred extension']
    if mode == 'clipped':
        arrow(guidance['direction_xy'], (0, 180, 255))
        text_lines += ['CLIPPED: ' + ('MOVE '+' + '.join(side.upper() for side in guidance['missing_sides'])
                                     if guidance['direction_xy'] else 'DIRECTION UNCERTAIN'),
                       'Wait for a complete target before pose alignment']
    elif guidance['alignment']:
        pose = guidance['alignment']
        arrow(pose['translation_pixel_equivalent_xyz'][:2], (255, 255, 0))
        text_lines += [f'NORMAL ALIGN | ESTIMATE | {guidance["geometry_observation"]}',
                       'Rotate deg: X {:+.1f}   Y {:+.1f}   Z {:+.1f}'.format(*pose['rotation_xyz_deg']),
                       'Shift px-equiv: X {:+.1f}   Y {:+.1f}   Z {:+.1f} (virtual)'.format(
                           *pose['translation_pixel_equivalent_xyz'])]
    else:
        text_lines += [mode.upper()+': '+guidance['reason'], 'Rotation / shift: unavailable']
    band_height = 26*len(text_lines)+10
    top = max(0, frame.shape[0]-band_height)
    region = vis[top:].copy()
    vis[top:] = cv2.addWeighted(region, .25, np.zeros_like(region), .75, 0)
    scale = min(.55, frame.shape[1]/2000)
    for index, message in enumerate(text_lines):
        cv2.putText(vis, message, (12, top+24+26*index), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (230, 230, 230), 1, cv2.LINE_AA)
    return vis
