"""Shared image/video CLI for the YOLO -> OpenCV pipeline."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np

from .yolo_quad import (DEFAULT_CLASSES, DEFAULT_WEIGHTS, YoloQuadDetector,
                        GateQuadProcessor, draw_detections, find_bbox_red,
                        project_detection_geometry)
from .camera_correction import (DEFAULT_PARAMS_PATH, load_camera_params, create_corrector,
                                adapt_camera_params)
from .temporal_overlay import TemporalOverlay

ROOT = Path(__file__).resolve().parents[1]
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'}


def read_image(path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def write_image(path, image):
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise RuntimeError(f'Could not encode image: {path}')
    encoded.tofile(str(path))


def parser(kind):
    ap = argparse.ArgumentParser(description=f'{kind}: YOLO boxes -> OpenCV gate polygon')
    ap.add_argument('source', type=Path, help='Image/directory' if kind == 'images' else 'Video file')
    ap.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    ap.add_argument('--out', type=Path, default=ROOT / 'runs' / kind)
    ap.add_argument('--classes', nargs='+', default=list(DEFAULT_CLASSES), help='Gate class names or IDs')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.7)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--device', default=None, help='cpu or GPU index, e.g. 0')
    ap.add_argument('--opts', default='', help='JSON object overriding CV defaults')
    ap.add_argument('--bbox-source', choices=['yolo', 'red', 'json'] if kind == 'images'
                    else ['yolo', 'red'], default='yolo')
    ap.add_argument('--min-area', type=int, default=600, help='Minimum area for --bbox-source red')
    ap.add_argument('--max-frames', type=int, default=None, help='Process only the first N images/frames')
    if kind == 'images':
        ap.add_argument('--sample', type=int, default=None, help='Save at most N images per lvl; default all')
    else:
        ap.add_argument('--show', action='store_true', help='Preview; press q to stop')
        ap.add_argument('--camera-params', type=Path, default=DEFAULT_PARAMS_PATH,
                        help='Camera JSON; defaults to the project root parameter file')
        ap.add_argument('--plane-distance', type=float, default=None,
                        help='Known perpendicular scene plane distance in metres; default ignores offset')
        ap.add_argument('--rotate-180', action='store_true', help='Rotate unaligned input before correction')
        ap.add_argument('--camera-fit', choices=['center-crop', 'resize', 'strict'],
                        default='center-crop',
                        help='How recording dimensions relate to calibration: centered crop (default), '
                             'full-view resize, or require the same aspect ratio')
        ap.add_argument('--no-stabilize', action='store_true', help='Disable temporal display stabilization')
        ap.add_argument('--hold-seconds', type=float, default=.2,
                        help='Maximum optical-flow gap duration; default 0.2 seconds, range 0..1')
        ap.add_argument('--smooth-alpha', type=float, default=.65,
                        help='Current-frame weight for display smoothing; range (0, 1]')
        ap.add_argument('--show-all-boxes', action='store_true',
                        help='Also draw other current YOLO boxes; default displays the largest target')
    return ap


def _setup(args):
    if not args.source.exists():
        raise FileNotFoundError(f'Source not found: {args.source}')
    if args.max_frames is not None and args.max_frames < 1:
        raise ValueError('--max-frames must be positive')
    if getattr(args, 'sample', None) is not None and args.sample < 0:
        raise ValueError('--sample must be nonnegative')
    if not 0 <= args.conf <= 1 or not 0 <= args.iou <= 1 or args.imgsz < 1:
        raise ValueError('Invalid confidence, IoU, or image size')
    opts = json.loads(args.opts) if args.opts else None
    if opts is not None and not isinstance(opts, dict):
        raise ValueError('--opts must be a JSON object')
    args.out.mkdir(parents=True, exist_ok=True)
    config_dir = ROOT / 'runs' / '.ultralytics'
    config_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(config_dir))
    if args.bbox_source == 'yolo':
        detector = YoloQuadDetector(args.weights, args.classes, args.conf, args.iou,
                                    args.imgsz, args.device, opts)
        print(f'Model classes: {detector.names}; selection=largest-area; '
              f'CV target IDs: {sorted(detector.target_ids)}')
    else:
        detector = GateQuadProcessor(args.classes, opts)
    return detector, opts


def _detect(frame, source, args, detector, opts):
    if args.bbox_source == 'yolo':
        return detector.detect(frame)
    if args.bbox_source == 'red':
        bbox = find_bbox_red(frame, args.min_area)
        dets = [dict(label='door', score=1.0, bbox=bbox)] if bbox else []
    else:
        json_path = source.with_suffix('.json')
        if not json_path.is_file():
            raise FileNotFoundError(f'Missing YOLO JSON: {json_path}')
        dets = json.loads(json_path.read_text(encoding='utf-8')).get('dets', [])
    detections = []
    for d in dets:
        det = dict(label=d['label'], score=float(d.get('score', 1.0)), bbox=d['bbox'])
        if 'class_id' in d:
            det['class_id'] = int(d['class_id'])
        detections.append(det)
    return detector.process(frame, detections)


def _record(handle, frame_id, source, detections, **extra):
    handle.write(json.dumps(dict(frame=frame_id, source=str(source),
                                 detections=detections, **extra), ensure_ascii=False) + '\n')


def _count(stats, detections):
    quads = [d['quad'] for d in detections if 'quad' in d]
    if not quads:
        stats['no-target'] += 1
    for quad in quads:
        stats[f"lvl{quad['lvl']}"] += 1


def run_images(argv=None):
    args = parser('images').parse_args(argv)
    detector, opts = _setup(args)
    paths = ([args.source] if args.source.is_file() else
             sorted(p for p in args.source.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES))
    if not paths:
        raise ValueError(f'No images in {args.source}')
    if args.max_frames is not None:
        paths = paths[:args.max_frames]
    stats, saved = Counter(), Counter()
    with (args.out / 'rows.jsonl').open('w', encoding='utf-8') as rows:
        for index, path in enumerate(paths, 1):
            image = read_image(path)
            if image is None:
                raise RuntimeError(f'Could not decode image: {path}')
            try:
                detections = _detect(image, path, args, detector, opts)
            except FileNotFoundError as error:
                if args.bbox_source != 'json':
                    raise
                _record(rows, index, path, [], error=str(error))
                stats['missing-json'] += 1
                continue
            _record(rows, index, path, detections,
                    gate_status=detector.last_status if detector else None)
            _count(stats, detections)
            lvl = max((d['quad']['lvl'] for d in detections if 'quad' in d), default=-1)
            if args.sample is None or saved[lvl] < args.sample:
                write_image(args.out / f'{path.stem}_{path.suffix[1:]}.jpg',
                            draw_detections(image, detections))
                saved[lvl] += 1
            if index % 100 == 0:
                print(f'Processed {index}/{len(paths)} images')
    print(f'Images={len(paths)}; detections={dict(stats)}; output={args.out.resolve()}')
    return 0


def run_video(argv=None):
    args = parser('video').parse_args(argv)
    camera = load_camera_params(args.camera_params)
    if not 0 <= args.hold_seconds <= 1 or not 0 < args.smooth_alpha <= 1:
        raise ValueError('Invalid --hold-seconds or --smooth-alpha')
    cap = cv2.VideoCapture(str(args.source))
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'Could not open video: {args.source}')
    writers = []
    stats, count = Counter(), 0
    outputs = [args.out / 'before_correction.mp4', args.out / 'after_correction.mp4']
    if args.source.resolve() in [p.resolve() for p in outputs]:
        cap.release()
        raise ValueError('Output video must differ from source video')
    try:
        ok, first_frame = cap.read()
        if not ok:
            raise RuntimeError('Video contains no decodable frames')
        input_height, input_width = first_frame.shape[:2]
        adapted_camera, adaptation = adapt_camera_params(camera, input_width, input_height,
                                                         args.camera_fit)
        corrector = create_corrector(adapted_camera)
        corrector.maps(args.plane_distance)
        print(f'Camera: reference={camera["width"]}x{camera["height"]}; '
              f'input={input_width}x{input_height}; processing={corrector.w}x{corrector.h}; '
              f'fit={args.camera_fit}')
        if adaptation['aspect_changed']:
            print(f'Camera mode assumption: {adaptation["assumption"]}; '
                  'use recording-mode intrinsics for calibrated accuracy.')
        detector, opts = _setup(args)
        detector.opts = dict(opts or {}, fx=corrector.f_out,
                             pixel_scale=max(input_width / 640, input_height / 480))
        fps = cap.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            fps = 30.0
        overlay = None if args.no_stabilize else TemporalOverlay(
            fps, args.hold_seconds, args.smooth_alpha, detector.opts)
        metadata = dict(camera=camera, camera_params=str(args.camera_params.resolve()),
                        adapted_camera=adapted_camera, camera_adaptation=adaptation,
                        input_size=[input_width, input_height],
                        output_matrix=corrector.output_matrix.tolist(), fps=fps,
                        plane_distance_m=args.plane_distance, rotate_180=args.rotate_180,
                        inference_coordinates='corrected', measurement_coordinates='corrected',
                        before_coordinates='prepared-raw', shared_detections=True,
                        raw_size=[corrector.w, corrector.h], output_size=[corrector.w, corrector.h],
                        video_size=[corrector.w + corrector.w % 2,
                                    corrector.h + corrector.h % 2],
                        stabilization=dict(enabled=overlay is not None, hold_seconds=args.hold_seconds,
                                           alpha=args.smooth_alpha, show_all_boxes=args.show_all_boxes,
                                           predictions_labeled=True),
                        valid_pixel_fraction=float(corrector.valid_mask(args.plane_distance).mean()))
        (args.out / 'camera_used.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2),
                                                 encoding='utf-8')
        mapper = lambda points: corrector.corrected_to_raw(points, args.plane_distance)
        with (args.out / 'rows.jsonl').open('w', encoding='utf-8') as rows:
            while args.max_frames is None or count < args.max_frames:
                if count == 0:
                    ok, frame = True, first_frame
                else:
                    ok, frame = cap.read()
                if not ok:
                    break
                if frame.shape[:2] != (input_height, input_width):
                    raise ValueError('视频中途分辨率发生变化，无法复用同一套内参和视频编码器')
                count += 1
                raw = cv2.rotate(frame, cv2.ROTATE_180) if args.rotate_180 else frame
                fixed = corrector.undistort(raw, args.plane_distance)
                if not writers:
                    metadata['input_size'] = [frame.shape[1], frame.shape[0]]
                    (args.out / 'camera_used.json').write_text(
                        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
                    for output in outputs:
                        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'),
                                                 fps, (corrector.w + corrector.w % 2,
                                                       corrector.h + corrector.h % 2))
                        writers.append(writer)
                        if not writer.isOpened():
                            raise RuntimeError(f'Could not create output video: {output}')
                # YOLO and OpenCV both consume exactly this corrected clean frame.
                detections = _detect(fixed, args.source, args, detector, detector.opts)
                raw_geometry = project_detection_geometry(detections, mapper)
                display_detections = (overlay.update(fixed, detections) if overlay else
                                      [d for d in detections if d.get('selected')])
                if args.show_all_boxes:
                    display_detections = display_detections + [d for d in detections if not d.get('selected')]
                display_raw_geometry = project_detection_geometry(display_detections, mapper)
                corners = next((d['quad']['corners'] for d in detections if 'quad' in d), None)
                diagnostic = corrector.assess_distortion(fixed, corners, args.plane_distance)
                _record(rows, count, args.source, detections, time_s=(count - 1) / fps,
                        gate_status=detector.last_status, coordinate_space='corrected',
                        raw_geometry=raw_geometry, distortion_diagnostic=diagnostic,
                        display_detections=display_detections, display_raw_geometry=display_raw_geometry,
                        temporal_status=overlay.status if overlay else dict(state='disabled'))
                _count(stats, detections)
                before = draw_detections(raw, display_detections, display_raw_geometry,
                                         header='BEFORE | measurements: corrected px')
                after = draw_detections(fixed, display_detections,
                                        header='AFTER | measurements: corrected px')
                # Preserve all source pixels; pad only odd dimensions for MPEG-4.
                if corrector.w % 2 or corrector.h % 2:
                    before = cv2.copyMakeBorder(before, 0, corrector.h % 2, 0,
                                                corrector.w % 2, cv2.BORDER_CONSTANT)
                    after = cv2.copyMakeBorder(after, 0, corrector.h % 2, 0,
                                               corrector.w % 2, cv2.BORDER_CONSTANT)
                writers[0].write(before)
                writers[1].write(after)
                if count % 100 == 0:
                    print(f'Processed {count} frames')
                if args.show:
                    cv2.imshow('Before / After | YOLO + OpenCV', np.hstack([before, after]))
                    if cv2.waitKey(1) & 0xff == ord('q'):
                        break
    finally:
        cap.release()
        for writer in writers:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()
    if count == 0:
        raise RuntimeError('Video contains no decodable frames')
    print(f'Frames={count}; detections={dict(stats)}')
    for output in outputs:
        print(f'Video: {output.resolve()}')
    return 0
