"""YOLO-selected red gate CV on corrected frames; paired before/after output.

Run from GrandRdk:
    python quad_cv_kit/demo/demo_video_cv_improved.py
    python quad_cv_kit/demo/demo_video_cv_improved.py E:/TEST/DOOR_TEST_1.mp4

YOLO selects the largest door, with continuity for clipped foreground gates.
CV uses visible pipe width within the ROI; comparable tube diameters are assumed.
Green lines follow supported gate edges, including partial gates.
Yellow boxes show all current YOLO doors; [selected] marks the CV target.
Both videos share detections from the corrected frame; raw edges are mapped curves.
--export-cv-input also shows the actual sharpened, saturation-enhanced CV input.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.detect_red_gate import (RedGateTracker, draw_gate_geometry, original_geometry,
                                 project_gate_geometry)
from src.camera_correction import (DEFAULT_PARAMS_PATH, adapt_camera_params,
                                   create_corrector, load_camera_params)
from src.yolo_quad import (DEFAULT_CLASSES, DEFAULT_WEIGHTS, YoloQuadDetector,
                           draw_detections, project_detection_geometry)
from src.yolo_red_gate import YoloRedGateTracker
from src.gate_guidance import (enhance_cv_contrast, build_gate_guidance, draw_gate_guidance)
from src.video_input import FFmpegVideoInput

VIDEO_SUFFIXES = {'.mp4', '.avi', '.mov', '.mkv', '.m4v', '.wmv', '.mts'}


class VideoOutput:
    """H.264 with FFmpeg when available; otherwise OpenCV MPEG-4."""

    def __init__(self, path, fps, size):
        self.encoder = None
        self.writer = None
        self.size = size
        ffmpeg = shutil.which('ffmpeg')
        if ffmpeg:
            self.encoder = subprocess.Popen([
                ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
                '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{size[0]}x{size[1]}',
                '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                '-preset', 'fast', '-threads', '2', '-crf', '20',
                '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)
        else:
            self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'),
                                          fps, (size[0] + size[0] % 2,
                                                size[1] + size[1] % 2))
            if not self.writer.isOpened():
                raise RuntimeError(f'Cannot create video: {path}')

    def write(self, frame):
        if (frame.shape[1], frame.shape[0]) != self.size:
            raise ValueError('Source dimensions changed within a video')
        if self.encoder is not None:
            self.encoder.stdin.write(np.ascontiguousarray(frame).tobytes())
        else:
            padded = cv2.copyMakeBorder(frame, 0, frame.shape[0] % 2,
                                        0, frame.shape[1] % 2, cv2.BORDER_CONSTANT)
            self.writer.write(padded)

    def close(self):
        if self.encoder is not None:
            encoder, self.encoder = self.encoder, None
            try:
                encoder.stdin.close()
            finally:
                code = encoder.wait()
            if code != 0:
                raise RuntimeError(f'FFmpeg encoding failed (exit {code})')
        if self.writer is not None:
            self.writer.release()
            self.writer = None


def save_image(path, frame):
    ok, encoded = cv2.imencode('.jpg', frame)
    if not ok:
        raise RuntimeError(f'Cannot encode image: {path}')
    encoded.tofile(str(path))


def process_video(source, args, output_dir, detector=None):
    camera = load_camera_params(args.camera_params)
    reader = getattr(args, 'reader', 'opencv')
    cap = FFmpegVideoInput(source) if reader == 'ffmpeg' else cv2.VideoCapture(str(source))
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'Cannot open video: {source}')
    writers = []
    count = found = detected = tracked = complete = 0
    yolo_frames = cv_frames = switches = overflow_frames = 0
    fresh_complete = four_edges = fresh_pose = 0
    guidance_counts = {}
    guidance_samples = set()
    started = time.perf_counter()
    samples = []
    sample_indices = set()
    output_dir.mkdir(parents=True, exist_ok=True)
    corrector = None
    try:
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0:
            fps = 30.0
        expected = max(0, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        if args.max_frames:
            expected = min(expected, args.max_frames) if expected else args.max_frames
        if expected:
            sample_indices = set(np.linspace(0, expected - 1, min(16, expected)).astype(int))
        tracker = None
        selection = 'apparent-pipe-width' if detector is None else 'yolo-area-clipped-continuity'
        output_before = output_dir / 'nearest_gate_before.mp4'
        output_after = output_dir / 'nearest_gate_after.mp4'
        output_cv_input = output_dir / 'nearest_gate_cv_input.mp4'
        print(f'{source.name}: {expected or "?"} frames, {fps:g} fps -> {output_dir}', flush=True)
        with (output_dir / 'rows.jsonl').open('w', encoding='utf-8') as records:
            while args.max_frames is None or count < args.max_frames:
                ok, frame = cap.read()
                if not ok:
                    break
                if corrector is None:
                    size = (frame.shape[1], frame.shape[0])
                    adapted, adaptation = adapt_camera_params(camera, *size, args.camera_fit)
                    corrector = create_corrector(adapted)
                    valid_mask = corrector.valid_mask(args.plane_distance)
                    tracker = (RedGateTracker(fps, args.detect_every, args.hold_seconds)
                               if detector is None else YoloRedGateTracker(
                                   detector, fps, args.detect_every, args.hold_seconds,
                                   valid_mask, args.roi_padding, args.cv_contrast,
                                   args.cv_clahe_clip, args.cv_clahe_blend,
                                   args.cv_sharpen, args.cv_saturation))
                    metadata = dict(camera=camera, adapted_camera=adapted,
                                    camera_params=str(args.camera_params.resolve()),
                                    camera_adaptation=adaptation, input_size=list(size),
                                    output_size=list(size), fps=fps,
                                    input_reader=reader,
                                    output_matrix=corrector.output_matrix.tolist(),
                                    plane_distance_m=args.plane_distance,
                                    shared_detections=True, inference_coordinates='corrected',
                                    before_coordinates='original', after_coordinates='corrected',
                                    valid_pixel_fraction=float(valid_mask.mean()),
                                    selection=selection, yolo_enabled=detector is not None,
                                    yolo_boxes_visible=detector is not None,
                                    cv_contrast_gain=args.cv_contrast,
                                    cv_clahe_clip=args.cv_clahe_clip, cv_clahe_blend=args.cv_clahe_blend,
                                    cv_sharpen_amount=args.cv_sharpen, cv_saturation_gain=args.cv_saturation,
                                    cv_preprocessing_order=['CLAHE', 'contrast', 'value-unsharp', 'saturation'],
                                    cv_geometry='four-line-color-validated + endpoint-chain partial fallback',
                                    cv_reference='clean corrected frame for color and white elbows',
                                    pixel_shift_scale=args.pixel_shift_scale,
                                    gate_size_reference=[args.gate_width_m, args.gate_height_m],
                                    guidance_ui_only=True)
                    (output_dir / 'camera_used.json').write_text(
                        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
                    for output in (output_before, output_after):
                        writers.append(VideoOutput(output, fps, size))
                    if args.export_cv_input:
                        writers.append(VideoOutput(output_cv_input, fps, size))
                    print(f'  Correction: {size[0]}x{size[1]}, {args.camera_fit}, '
                          f'f_out={corrector.f_out:.2f}px', flush=True)
                if (frame.shape[1], frame.shape[0]) != size:
                    raise ValueError('Source dimensions changed within a video')
                fixed = corrector.undistort(frame, args.plane_distance)
                if detector is not None:
                    selected, _ = tracker.update(fixed)
                    cv_frame = tracker.last_cv_frame
                else:
                    cv_frame = enhance_cv_contrast(fixed, args.cv_contrast, valid_mask,
                        args.cv_clahe_clip, args.cv_clahe_blend, args.cv_sharpen, args.cv_saturation)
                    selected, _ = tracker.update(cv_frame, reference_frame=fixed, valid_mask=valid_mask)
                status = tracker.last_status
                yolo_frames += status.get('yolo_count', 0) > 0
                cv_frames += status.get('detection_ran', False)
                switches += status.get('target_switched', False)
                overflow_frames += status.get('target_clipped', False)
                geometry = original_geometry(fixed, selected)
                display_geometry = (dict(geometry, segments=geometry['observed_segments'], corners=[])
                                    if geometry is not None else None)
                raw_geometry = project_gate_geometry(
                    display_geometry, lambda points: corrector.corrected_to_raw(points, args.plane_distance),
                    size)
                guidance = build_gate_guidance(
                    status, geometry, corrector.output_matrix, size, valid_mask,
                    (args.gate_width_m, args.gate_height_m), args.pixel_shift_scale)
                fresh_pose += bool(guidance['alignment'] and geometry['observation'] == 'detected')
                guidance_counts[guidance['mode']] = guidance_counts.get(guidance['mode'], 0)+1
                yolo_boxes = []
                raw_yolo_geometry = []
                if detector is not None:
                    yolo_boxes = [dict(box, selected=(status['yolo_age_frames'] == 0
                                                     and box['bbox'] == status['target_bbox']))
                                  for box in status['yolo_detections']]

                    def raw_box_mapper(points):
                        mapped = corrector.corrected_to_raw(points, args.plane_distance)
                        valid = (np.isfinite(mapped).all(axis=1)
                                 & (mapped[:, 0] >= 0) & (mapped[:, 0] <= size[0]-1)
                                 & (mapped[:, 1] >= 0) & (mapped[:, 1] <= size[1]-1))
                        mapped[~valid] = np.nan
                        return mapped

                    raw_yolo_geometry = project_detection_geometry(yolo_boxes, raw_box_mapper)
                if geometry:
                    found += 1
                    detected += geometry['observation'] == 'detected'
                    tracked += geometry['observation'] == 'tracked'
                    complete += geometry['complete']
                    fresh_complete += bool(geometry['complete'] and geometry['observation'] == 'detected')
                    four_edges += len(geometry['observed_segments']) == 4
                row = dict(frame=count, time_s=count/fps, source=str(source.resolve()),
                           img_w=frame.shape[1], img_h=frame.shape[0],
                           coordinate_space='corrected', selection=selection,
                           nearest_gate=geometry, raw_geometry=raw_geometry,
                           display_gate_geometry=display_geometry, guidance=guidance,
                           cv_contrast_gain=args.cv_contrast,
                           cv_clahe_clip=args.cv_clahe_clip, cv_clahe_blend=args.cv_clahe_blend,
                           cv_sharpen_amount=args.cv_sharpen, cv_saturation_gain=args.cv_saturation,
                           candidate_count=tracker.last_status['candidate_count'],
                           age_frames=tracker.last_status['age_frames'])
                if detector is not None:
                    row['yolo'] = {key: status[key] for key in (
                        'yolo_detections', 'target_bbox', 'target_score', 'target_clipped',
                        'target_boundary_sides',
                        'search_bbox', 'selection_reason', 'target_switched',
                        'yolo_age_frames', 'cv_enabled', 'detection_ran')}
                    row['yolo']['display_detections'] = yolo_boxes
                    row['yolo']['raw_box_geometry'] = raw_yolo_geometry
                records.write(json.dumps(row, ensure_ascii=False) + '\n')
                before = draw_detections(frame, yolo_boxes, raw_yolo_geometry)
                after = draw_detections(fixed, yolo_boxes)
                before = draw_gate_geometry(before, raw_geometry, count, fps, 'BEFORE')
                after = draw_gate_geometry(after, display_geometry, count, fps, 'AFTER')
                before = draw_gate_guidance(before, guidance, corrector.output_matrix,
                                           lambda points: corrector.corrected_to_raw(points, args.plane_distance))
                after = draw_gate_guidance(after, guidance, corrector.output_matrix)
                writers[0].write(before)
                writers[1].write(after)
                enhanced_view = None
                if args.export_cv_input:
                    enhanced_view = draw_detections(cv_frame, yolo_boxes)
                    enhanced_view = draw_gate_geometry(enhanced_view, display_geometry, count, fps, 'CV INPUT')
                    enhanced_view = draw_gate_guidance(enhanced_view, guidance, corrector.output_matrix)
                    writers[2].write(enhanced_view)
                sample_modes = {guidance['mode']}
                if guidance['completion'] and guidance['completion']['segments']:
                    sample_modes.add('completion')
                for sample_mode in sorted(sample_modes & {'clipped', 'align', 'completion'}):
                    if sample_mode not in guidance_samples:
                        save_image(output_dir / f'guidance_{sample_mode}_before.jpg', before)
                        save_image(output_dir / f'guidance_{sample_mode}_after.jpg', after)
                        guidance_samples.add(sample_mode)
                if count in sample_indices:
                    save_image(output_dir / f'frame_{count:06d}_before.jpg', before)
                    save_image(output_dir / f'frame_{count:06d}_after.jpg', after)
                    if enhanced_view is not None:
                        save_image(output_dir / f'frame_{count:06d}_cv_input.jpg', enhanced_view)
                    samples.append(np.hstack([cv2.resize(before, (480, 270)),
                                              cv2.resize(after, (480, 270))]))
                count += 1
                if count % 150 == 0:
                    print(f'  {count}/{expected or "?"}, gate visible {found}/{count}', flush=True)
                if args.show:
                    cv2.imshow('Nearest red gate: before / after', np.hstack([before, after]))
                    if cv2.waitKey(1) & 0xff in (27, ord('q')):
                        break
        if count == 0:
            raise RuntimeError(f'No readable video frames: {source}')
    finally:
        cap.release()
        close_errors = []
        for writer in writers:
            try:
                writer.close()
            except (OSError, RuntimeError) as error:
                close_errors.append(error)
        if args.show:
            cv2.destroyAllWindows()
        if close_errors:
            raise close_errors[0]
    if samples:
        while len(samples) % 2:
            samples.append(np.zeros_like(samples[0]))
        contact = np.vstack([np.hstack(samples[i:i+2]) for i in range(0, len(samples), 2)])
        save_image(output_dir / 'contact.jpg', contact)
    elapsed = time.perf_counter() - started
    summary = dict(source=str(source.resolve()),
                   videos=dict(before=str(output_before.resolve()), after=str(output_after.resolve())),
                   frames=count, fps=fps, size=list(size), gate_frames=found,
                   expected_frames=expected, input_reader=reader,
                   input_frame_count_matches=count == expected if expected else None,
                   detected_frames=detected, tracked_frames=tracked, complete_frames=complete,
                   fresh_complete_frames=fresh_complete, four_observed_edge_frames=four_edges,
                   fresh_pose_frames=fresh_pose,
                   elapsed_s=round(elapsed, 3), processing_fps=round(count/elapsed, 2),
                   selection=selection,
                   assumption='Gates use comparable physical pipe diameters',
                   detect_every=args.detect_every, hold_seconds=args.hold_seconds,
                   coordinate_space='corrected', shared_detections=True,
                   camera_params=str(args.camera_params.resolve()), camera_fit=args.camera_fit,
                   yolo_enabled=detector is not None, yolo_frames=yolo_frames,
                   yolo_boxes_visible=detector is not None,
                   cv_detection_frames=cv_frames, target_switches=switches,
                   overflow_allowed_frames=overflow_frames,
                   weights=str(args.weights.resolve()) if detector is not None else None,
                   device=args.device, conf=args.conf, iou=args.iou, imgsz=args.imgsz,
                   roi_padding=args.roi_padding, cv_contrast_gain=args.cv_contrast,
                   cv_clahe_clip=args.cv_clahe_clip, cv_clahe_blend=args.cv_clahe_blend,
                   cv_sharpen_amount=args.cv_sharpen, cv_saturation_gain=args.cv_saturation,
                   cv_geometry='four-line-color-validated + endpoint-chain partial fallback',
                   cv_reference='clean corrected frame for color and white elbows',
                   pixel_shift_scale=args.pixel_shift_scale, metric_distance_available=False,
                   gate_size_reference=[args.gate_width_m, args.gate_height_m],
                   guidance_ui_only=True, guidance_frames=guidance_counts)
    if args.export_cv_input:
        summary['videos']['cv_input'] = str(output_cv_input.resolve())
    (output_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                                            encoding='utf-8')
    print(f'  Done: {count} frames, {found} with gate, {count/elapsed:.1f} processing fps', flush=True)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', type=Path, default=Path('E:/TEST'),
                        help='Input video or directory; default E:/TEST')
    parser.add_argument('--out', type=Path, default=ROOT / 'runs' / 'cv_improved')
    parser.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument('--classes', nargs='+', default=list(DEFAULT_CLASSES))
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--iou', type=float, default=.7)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='cpu', help='YOLO inference device: cpu or GPU index')
    parser.add_argument('--reader', choices=['opencv', 'ffmpeg'], default='opencv',
                        help='Use FFmpeg if OpenCV stops reading an otherwise decodable video early')
    parser.add_argument('--roi-padding', type=float, default=.08,
                        help='Ordinary box padding ratio; clipped targets allow wider overflow')
    parser.add_argument('--cv-only', action='store_true', help='Run the previous CV-only detector')
    parser.add_argument('--cv-contrast', type=float, default=1.2,
                        help='CV-only HSV value contrast gain (1..1.5)')
    parser.add_argument('--cv-clahe-clip', type=float, default=2.0,
                        help='Local contrast clip limit (0 disables CLAHE, range 0..8)')
    parser.add_argument('--cv-clahe-blend', type=float, default=.6,
                        help='Blend of locally enhanced brightness (0..1)')
    parser.add_argument('--cv-sharpen', type=float, default=.6,
                        help='Value-channel unsharp amount (0..2, 0 disables; sigma 1.2px)')
    parser.add_argument('--cv-saturation', type=float, default=1.25,
                        help='HSV saturation multiplier (1..2, 1 disables)')
    parser.add_argument('--export-cv-input', action='store_true',
                        help='Also export the actual enhanced CV input with shared YOLO/CV overlays')
    parser.add_argument('--pixel-shift-scale', type=float, default=1.0,
                        help='Common positive scale applied to X/Y/Z pixel equivalents; no metric distance')
    parser.add_argument('--gate-width', '--gate-width-m', dest='gate_width_m', type=float, default=.7,
                        help='Reference gate width; only width/height ratio affects angles and pixel equivalents')
    parser.add_argument('--gate-height', '--gate-height-m', dest='gate_height_m', type=float, default=.5,
                        help='Reference gate height in the same arbitrary units as width')
    parser.add_argument('--max-frames', '--limit', type=int, default=None)
    parser.add_argument('--detect-every', type=int, default=3,
                        help='Fresh detection interval; track between detections')
    parser.add_argument('--hold-seconds', type=float, default=.2,
                        help='Maximum time since real detection for image-supported tracking')
    parser.add_argument('--camera-params', type=Path, default=DEFAULT_PARAMS_PATH,
                        help='Air intrinsics / flat-port correction JSON')
    parser.add_argument('--camera-fit', choices=['center-crop', 'resize', 'strict'],
                        default='center-crop', help='Adapt reference intrinsics to video dimensions')
    parser.add_argument('--plane-distance', type=float, default=None,
                        help='Known perpendicular scene plane distance (metres); optional')
    parser.add_argument('--show', action='store_true', help='Preview; q/Esc stops current video')
    args = parser.parse_args(argv)
    if args.max_frames is not None and args.max_frames < 1:
        raise ValueError('--max-frames must be positive')
    if (not np.isfinite(args.cv_contrast) or not 1 <= args.cv_contrast <= 1.5
            or not np.isfinite([args.gate_width_m, args.gate_height_m]).all()
            or min(args.gate_width_m, args.gate_height_m) <= 0):
        raise ValueError('Invalid CV contrast gain or reference gate dimensions')
    if (not np.isfinite([args.cv_clahe_clip, args.cv_clahe_blend, args.pixel_shift_scale]).all()
            or not 0 <= args.cv_clahe_clip <= 8 or not 0 <= args.cv_clahe_blend <= 1
            or args.pixel_shift_scale <= 0):
        raise ValueError('Invalid CLAHE parameters or pixel shift scale')
    if (not np.isfinite([args.cv_sharpen, args.cv_saturation]).all()
            or not 0 <= args.cv_sharpen <= 2 or not 1 <= args.cv_saturation <= 2):
        raise ValueError('Invalid CV sharpening or saturation gain')
    if (not 0 <= args.conf <= 1 or not 0 <= args.iou <= 1 or args.imgsz < 1
            or not 0 <= args.roi_padding <= .5):
        raise ValueError('Invalid YOLO thresholds, image size, or ROI padding')
    if args.detect_every < 1 or not np.isfinite(args.hold_seconds) or not 0 <= args.hold_seconds <= 1:
        raise ValueError('Invalid detection interval or hold time (0..1 seconds)')
    if args.plane_distance is not None and (not np.isfinite(args.plane_distance)
                                             or args.plane_distance <= 0):
        raise ValueError('--plane-distance must be positive and finite')
    if args.source.is_file():
        videos = [args.source]
    elif args.source.is_dir():
        videos = sorted(p for p in args.source.iterdir()
                        if p.is_file() and p.suffix.lower() in VIDEO_SUFFIXES)
    else:
        raise FileNotFoundError(f'Input does not exist: {args.source}')
    if not videos:
        raise ValueError(f'No videos found: {args.source}')
    detector = None if args.cv_only else YoloQuadDetector(
        args.weights, args.classes, args.conf, args.iou, args.imgsz, args.device)
    if detector is not None:
        print(f'YOLO: {detector.names}, device={args.device}', flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    names = set()
    for video in videos:
        name = video.stem if video.stem not in names else video.name.replace('.', '_')
        names.add(name)
        # Never allow a caller's output setting to replace the input video.
        destination = args.out / name
        if video.resolve() in [(destination / name).resolve()
                               for name in ('nearest_gate_before.mp4', 'nearest_gate_after.mp4',
                                            'nearest_gate_cv_input.mp4')]:
            raise ValueError('Output video would overwrite the source')
        process_video(video, args, destination, detector)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError) as error:
        print(f'Error: {error}', file=sys.stderr)
        sys.exit(2)
