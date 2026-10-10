"""CV contrast and UI-only gate completion / camera-frame alignment estimates."""
import math

import cv2
import numpy as np

from .detect_red_gate import project_gate_geometry


AXES = 'camera: X right, Y down, Z forward; right-hand rotations Rz*Ry*Rx'


def sharpen_frame(frame, amount=.6, valid_mask=None, backend=None):
    """Brightness-only unsharp mask; no contrast, HSV or saturation change.

    Blur max(B,G,R) with sigma=1.2 and a three-level noise threshold. Normalize
    the Gaussian over valid nonblack pixels so correction borders cannot create
    halos. Scale the original BGR channels together, retaining their ratios.
    """
    if not math.isfinite(amount) or not 0 <= amount <= 2:
        raise ValueError('Sharpen amount must be finite and in [0, 2]')
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('Sharpen input must be uint8 BGR')
    if valid_mask is not None and valid_mask.shape != frame.shape[:2]:
        raise ValueError('Sharpen valid mask must match the frame')
    if backend is not None:
        return backend.sharpen(frame, amount, valid_mask)
    if amount == 0:
        return frame.copy()
    value = frame.max(axis=2).astype(np.float32)
    active = value > 0
    if valid_mask is not None:
        active &= np.asarray(valid_mask, bool)
    weights = active.astype(np.float32)
    numerator = cv2.GaussianBlur(value*weights, (11, 11), 1.2)
    denominator = cv2.GaussianBlur(weights, (11, 11), 1.2)
    blur = np.divide(numerator, denominator, out=value.copy(), where=denominator > 0)
    detail = value-blur
    detail[np.abs(detail) < 3] = 0
    target = np.rint(np.clip(value+amount*detail, 0, 255))
    scale = np.divide(target, value, out=np.ones_like(value), where=active)
    result = np.rint(np.clip(frame.astype(np.float32)*scale[:, :, None], 0, 255)).astype(np.uint8)
    result[~active] = frame[~active]
    return result


def enhance_cv_contrast(frame, gain=1.2, valid_mask=None, clahe_clip=2.0, clahe_blend=.6,
                        sharpen_amount=.6, saturation_gain=1.25, backend=None):
    """CLAHE, global contrast, value-channel unsharp mask and HSV saturation.

    CLAHE limits local noise amplification. Invalid correction borders are filled
    with the valid median for histogram calculation, then restored unchanged.
    Sharpen only luminance (sigma=1.2px, 3-level noise threshold), preserving hue.
    Set sharpen_amount=0 and saturation_gain=1 for the previous contrast stages.
    """
    if not math.isfinite(gain) or not 1 <= gain <= 1.5:
        raise ValueError('CV contrast gain must be finite and in [1, 1.5]')
    if (not math.isfinite(clahe_clip) or not 0 <= clahe_clip <= 8
            or not math.isfinite(clahe_blend) or not 0 <= clahe_blend <= 1):
        raise ValueError('CLAHE clip must be in [0, 8], blend in [0, 1]')
    if (not math.isfinite(sharpen_amount) or not 0 <= sharpen_amount <= 2
            or not math.isfinite(saturation_gain) or not 1 <= saturation_gain <= 2):
        raise ValueError('Sharpen amount must be in [0, 2], saturation gain in [1, 2]')
    if backend is not None:
        return backend.enhance(frame, gain, valid_mask, clahe_clip, clahe_blend, sharpen_amount, saturation_gain)
    if (gain == 1 and (clahe_clip == 0 or clahe_blend == 0)
            and sharpen_amount == 0 and saturation_gain == 1):
        return frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    active = hsv[:, :, 2] > 0
    if valid_mask is not None:
        active &= np.asarray(valid_mask, bool)
    if not active.any():
        return frame.copy()
    value = hsv[:, :, 2].astype(float)
    pivot = float(np.median(value[active]))
    if clahe_clip > 0 and clahe_blend > 0:
        histogram_input = hsv[:, :, 2].copy()
        histogram_input[~active] = round(pivot)
        local = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8)).apply(histogram_input)
        value = (1-clahe_blend)*value + clahe_blend*local
        pivot = float(np.median(value[active]))
    value = np.clip((value-pivot)*gain+pivot, 0, 255)
    if sharpen_amount > 0:
        # Replace invalid borders during blur to avoid bright halos against black.
        blur_input = value.astype(np.float32)
        blur_input[~active] = float(np.median(value[active]))
        detail = value-cv2.GaussianBlur(blur_input, (0, 0), 1.2)
        detail[np.abs(detail) < 3] = 0
        value = np.clip(value+sharpen_amount*detail, 0, 255)
    hsv[:, :, 2] = np.rint(value).astype(np.uint8)
    hsv[:, :, 1] = np.rint(np.clip(hsv[:, :, 1].astype(float)*saturation_gain, 0, 255)).astype(np.uint8)
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


def estimate_alignment(corners, camera_matrix, gate_size=(.7, .5), pixel_scale=1.0):
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
            or min(width, height, matrix[0, 0], matrix[1, 1]) <= 0
            or not math.isfinite(pixel_scale) or pixel_scale <= 0):
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
                rotation_matrix=rotation.tolist(), gate_center_model_units=translation.tolist(),
                gate_normal_camera=normal.tolist(), alignment_translation_model_units=movement.tolist(),
                translation_pixel_equivalent_xyz=pixel_equivalent.tolist(),
                translation_scaled_xyz=(pixel_equivalent*pixel_scale).tolist(),
                translation_scale=pixel_scale, translation_unit='scaled pixel equivalent',
                translation_formula='scale * (fx*dx, fy*dy, mean(fx,fy)*dz) / center_depth',
                translation_z_unit='virtual pixel equivalent: mean(fx,fy)*dz/center_depth',
                gate_center_px=center.tolist(), center_offset_xy_px=(center-matrix[:2, 2]).tolist(),
                normal_distance_model_units=normal_distance, reprojection_rms_px=error,
                gate_size_reference=list(gate_size), axes=AXES, estimated=True,
                metric_distance_available=False), 'pose-estimated'


def build_gate_guidance(status, geometry, camera_matrix, size, valid_mask=None, gate_size=(.7, .5),
                        pixel_scale=1.0):
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
    alignment, reason = estimate_alignment(completion['corners'], camera_matrix, gate_size, pixel_scale)
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
                       'Shift px-equiv x{:g}: X {:+.1f}   Y {:+.1f}   Z {:+.1f} (Z virtual)'.format(
                           pose['translation_scale'], *pose['translation_scaled_xyz'])]
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
