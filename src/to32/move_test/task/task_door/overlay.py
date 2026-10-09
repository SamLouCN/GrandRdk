"""穿门回传标注：仅绘制校正帧副本，观测和控制数据保持原值。"""
import cv2
import numpy as np

from .config import CONFIG


def draw_door_overlay(frame, detections, observation, cfg=CONFIG):
    """黄色 YOLO 框、绿色实测边、浅绿延长线、红色交点和紧凑姿态数据。"""
    vis = frame.copy()
    height, width = vis.shape[:2]
    scale = cfg.overlay_font_scale
    font = cv2.FONT_HERSHEY_SIMPLEX

    def points(values):
        return np.rint(np.clip(np.asarray(values, float), [0, 0],
                               [width-1, height-1])).astype(np.int32)

    def text(message, x, y, color=(235, 235, 235)):
        (tw, th), _ = cv2.getTextSize(message, font, scale, 1)
        x = max(0, min(int(x), width-tw-1))
        y = max(th+3, min(int(y), height-4))
        cv2.putText(vis, message, (x, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, message, (x, y), font, scale, color, 1, cv2.LINE_AA)

    selected = observation.get('bbox')
    for det in detections:
        box = points(np.asarray(det['bbox']).reshape(2, 2))
        target = selected is not None and np.array_equal(box, points(np.asarray(selected).reshape(2, 2)))
        cv2.rectangle(vis, tuple(box[0]), tuple(box[1]), (0, 255, 255), 2 if target else 1)
        label = '{} {:.2f}{}'.format(det['label'], det['score'], ' [target]' if target else '')
        text(label, box[0, 0], box[0, 1]-6, (0, 255, 255))

    geometry = observation.get('geometry') or {}
    for segment in geometry.get('observed_segments', geometry.get('segments', [])):
        cv2.polylines(vis, [points(segment)], False, (0, 255, 0), 2, cv2.LINE_AA)
    for segment in observation.get('inferred_segments', []):
        cv2.polylines(vis, [points(segment)], False, (150, 255, 150), 1, cv2.LINE_AA)
    corners = observation.get('corners') or geometry.get('corners') or []
    for corner in corners:
        cv2.circle(vis, tuple(points(corner)), 3, (0, 0, 255), -1, cv2.LINE_AA)

    if not observation.get('valid'):
        lines = ['PassGate | vision unavailable', observation.get('reason', 'not ready')]
    elif not observation.get('has_target'):
        lines = ['PassGate | no current target']
    else:
        state = geometry.get('observation', 'missing')
        lines = ['PassGate | area {:.1%} | CV {}'.format(observation.get('area_ratio', 0), state)]
        pose = observation.get('pose')
        if pose is None:
            lines.append('Pose unavailable: ' + observation.get('pose_reason', 'need four edges'))
        else:
            lines[0] += ' | ' + ('OK' if pose['controllable'] else 'TILT LIMIT')
            lines += [
                'Yaw err {:+.2f} deg | Tilt {:.1f} deg | RMS {:.2f} px'.format(
                    pose['yaw_error_deg'], pose['unactuated_tilt_deg'], pose['reprojection_rms_px']),
                'Center m: X {:+.3f}  Y {:+.3f}  Z {:.3f}'.format(*pose['center_robot_m']),
                'Align m:  X {:+.3f}  Y {:+.3f}  Z {:+.3f}'.format(*pose['alignment_robot_m']),
            ]

    # 只按文字大小铺设左下角背景，不覆盖整条画面；默认面板约占 2%～4%。
    pad, margin = 6, 8
    text_height = max(cv2.getTextSize(line, font, scale, 1)[0][1] for line in lines)
    row_height = max(cfg.overlay_line_height_px, text_height+7)
    panel_width = min(width-margin, max(cv2.getTextSize(line, font, scale, 1)[0][0]
                                        for line in lines)+2*pad+12)
    panel_height = min(height-margin, row_height*len(lines)+2*pad)
    left, top = margin, height-margin-panel_height
    panel = vis[top:top+panel_height, left:left+panel_width]
    panel[:] = cv2.addWeighted(panel, 1-cfg.overlay_background_alpha,
                               np.zeros_like(panel), cfg.overlay_background_alpha, 0)
    for index, line in enumerate(lines):
        text(line, left+pad, top+pad+text_height+index*row_height)
    return vis
