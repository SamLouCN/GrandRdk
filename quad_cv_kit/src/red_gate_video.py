"""Offline optimized red-gate pipeline: S100 HBM or Ultralytics, MP4 and timings."""
import argparse
from contextlib import contextmanager, nullcontext
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from .camera_correction import (DEFAULT_PARAMS_PATH, adapt_camera_params,
                                create_corrector, load_camera_params)
from .detect_red_gate import original_geometry, draw_gate_geometry
from .gate_guidance import build_gate_guidance, draw_gate_guidance
from .video_input import FFmpegVideoInput
from .video_output import VideoOutput
from .yolo_quad import DEFAULT_CLASSES, DEFAULT_WEIGHTS, YoloQuadDetector, draw_detections
from .yolo_red_gate import YoloRedGateTracker
from .opencl_backend import create_backend

ROOT = Path(__file__).resolve().parents[1]
STAGES = ('preprocess', 'yolo', 'opencv', 'postprocess')


class YoloStageTimer:
    """Collect the existing board detector's preprocessing/inference/NMS timers."""
    def __init__(self):
        self.ms = {}

    @contextmanager
    def measure(self, stage):
        started = time.perf_counter()
        try:
            yield
        finally:
            self.ms[stage] = self.ms.get(stage, 0.)+(time.perf_counter()-started)*1000


def _board_detector_factory(cfg, timer):
    # Reuse the deployed HBM decoder (including direct-LTRB support) read-only.
    # Do not import front.py: this demo opens only a file and never shared frames.
    source = ROOT.parent/'src'/'function.py'
    if not source.is_file():
        raise FileNotFoundError(f'S100 HBM adapter requires the GrandRdk library: {source}')
    name = '_quad_cv_kit_board_function'
    module = sys.modules.get(name)
    if module is None:
        spec = importlib.util.spec_from_file_location(name, source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        try:
            spec.loader.exec_module(module)
        except BaseException:
            sys.modules.pop(name, None)
            raise
    return module.YoloDetector(cfg, timer=timer)


class HbmBoxDetector:
    def __init__(self, args):
        self.names = dict(enumerate(args.class_names))
        requested = {str(name).lower().replace('_', '-') for name in args.classes}
        self.target_ids = {i for i, name in self.names.items()
                           if name.lower().replace('_', '-') in requested or str(i) in requested}
        if not self.target_ids:
            raise ValueError(f'No target {args.classes} in --class-names {args.class_names}')
        if any(self.names[i].lower().replace('_', '-') in
               ('upper-left', 'upper-right', 'lower-right', 'lower-left') for i in self.target_ids):
            raise ValueError('A whole-door HBM checkpoint is required, not corner classes')
        if len(set(args.class_names)) != len(args.class_names):
            raise ValueError('--class-names must contain unique labels in model class order')
        self.ids = {name: i for i, name in self.names.items()}
        self.timer = YoloStageTimer()
        cfg = dict(backend='hbm', model_path=str(args.weights.resolve()),
                   class_names=args.class_names, target_class_ids=sorted(self.target_ids),
                   target_class_names=[], score_thres=args.conf, nms_thres=args.iou,
                   strides=[8, 16, 32], bpu_cores=args.bpu_cores, priority=0,
                   preprocess_mode='bgr')
        self.detector = _board_detector_factory(cfg, self.timer)

    def detect_boxes(self, frame):
        self.timer.ms.clear()
        detections = self.detector.detect(frame, nv12=None)
        return [dict(det, class_id=self.ids[det['label']], bbox_source='yolo') for det in detections
                if det['label'] in self.ids and self.ids[det['label']] in self.target_ids]


class TimedBoxDetector:
    """Time real YOLO calls inside the shared tracker, without a second inference."""
    def __init__(self, detector):
        self.detector = detector
        self.target_ids = detector.target_ids
        self.last_ms = 0.

    def detect_boxes(self, frame):
        started = time.perf_counter()
        try:
            return self.detector.detect_boxes(frame)
        finally:
            self.last_ms = (time.perf_counter()-started)*1000


def create_detector(args):
    if args.backend == 'hbm':
        return HbmBoxDetector(args)
    config_dir = args.out/'.ultralytics'
    config_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(config_dir.resolve()))
    return YoloQuadDetector(args.weights, args.classes, args.conf, args.iou,
                            args.imgsz, args.device)


def parser():
    ap = argparse.ArgumentParser(description='Optimized YOLO/red-pipe CV video demo; offline S100 preview')
    ap.add_argument('source', type=Path)
    ap.add_argument('--backend', choices=('auto', 'hbm', 'ultralytics'), default='auto',
                    help='auto selects HBM for .hbm weights, otherwise Ultralytics')
    ap.add_argument('--weights', '--model', type=Path, help='Whole-door .hbm or .pt checkpoint')
    ap.add_argument('--class-names', nargs='+', default=['door'],
                    help='HBM class names in model output order; default single-class door')
    ap.add_argument('--classes', nargs='+', default=list(DEFAULT_CLASSES))
    ap.add_argument('--bpu-cores', type=int, nargs='+', default=[0])
    ap.add_argument('--conf', type=float, default=.25)
    ap.add_argument('--iou', type=float, default=.7)
    ap.add_argument('--imgsz', type=int, default=640, help='Ultralytics only; HBM uses compiled input size')
    ap.add_argument('--device', default='cpu', help='Ultralytics device only; HBM uses --bpu-cores')
    ap.add_argument('--out', type=Path, default=ROOT/'runs', help='Directory for after.mp4 and rows.jsonl')
    ap.add_argument('--output', type=Path, help='Override output video path, otherwise --out/after.mp4')
    ap.add_argument('--perf-log', type=Path, default=ROOT/'logs'/'perf.log',
                    help='Append per-frame JSONL timings and run summaries')
    ap.add_argument('--reader', choices=('opencv', 'ffmpeg'), default='opencv')
    ap.add_argument('--max-frames', type=int)
    ap.add_argument('--cv-every', '--detect-every', type=int, default=3)
    ap.add_argument('--hold-seconds', type=float, default=.2)
    ap.add_argument('--roi-padding', type=float, default=.08)
    ap.add_argument('--opencv-threads', type=int, default=3)
    ap.add_argument('--cv-backend', choices=('cpu', 'opencl', 'auto'), default='cpu',
                    help='opencl requires a GPU and runs explicit Hough/fitting/image kernels; auto allows startup fallback')
    ap.add_argument('--gpu-device', help='OpenCL device name/vendor substring, e.g. Mali')
    ap.add_argument('--cv-hough', choices=('opencl', 'cpu'), default='opencl',
                    help='Hough stage placement inside the GPU CV backend; fitting remains on GPU')
    ap.add_argument('--cv-blur', choices=('pyramid', 'exact'), default='pyramid',
                    help='Large-scale blur: area pyramid approximation or exact Gaussian')
    ap.add_argument('--cv-quality', choices=('fast', 'precise'), default='fast',
                    help='GPU CV: bounded half-degree Hough and sparse sampling, or original precise algorithms')
    ap.add_argument('--cv-search', choices=('adaptive', 'full'), default='adaptive',
                    help='Certified predicted side crops with same-frame fallback, or full ROI at every search')
    ap.add_argument('--cv-execution', choices=('resident', 'hybrid'),
                    help='OpenCL defaults to resident GPU CV; hybrid retains the LSD reference workflow')
    ap.add_argument('--cv-budget-ms', type=float, default=30., help='Report CV frames meeting this budget, without dropping work')
    ap.add_argument('--camera-params', type=Path, default=DEFAULT_PARAMS_PATH)
    ap.add_argument('--camera-fit', choices=('center-crop', 'resize', 'strict'), default='center-crop')
    ap.add_argument('--plane-distance', type=float)
    ap.add_argument('--no-correction', action='store_true')
    ap.add_argument('--rotate-180', action='store_true')
    ap.add_argument('--show', action='store_true')
    return ap


def _json_numpy(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f'Object of type {type(value).__name__} is not JSON serializable')


def _log(handle, **record):
    handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False, default=_json_numpy)+'\n')


def run_video(argv=None):
    args = parser().parse_args(argv)
    if not args.source.is_file():
        raise FileNotFoundError(f'Source not found: {args.source}')
    if args.backend == 'auto':
        args.backend = 'hbm' if args.weights and args.weights.suffix.lower() == '.hbm' else 'ultralytics'
    if args.weights is None:
        args.weights = (ROOT.parent/'models'/'door_4_nashe_1280x1280_nv12.hbm'
                        if args.backend == 'hbm' else DEFAULT_WEIGHTS)
    if not args.weights.is_file():
        raise FileNotFoundError(f'Model not found: {args.weights}')
    if args.backend == 'hbm' and args.weights.suffix.lower() != '.hbm':
        raise ValueError('--backend hbm requires a .hbm checkpoint')
    if args.backend == 'ultralytics' and args.weights.suffix.lower() == '.hbm':
        raise ValueError('Use --backend hbm for .hbm checkpoints')
    if (args.max_frames is not None and args.max_frames < 1) or args.cv_every < 1 or args.opencv_threads < 1:
        raise ValueError('--max-frames, --cv-every and --opencv-threads must be positive')
    if (not 0 < args.conf < 1 or not 0 <= args.iou <= 1 or args.imgsz < 1 or
            not 0 <= args.hold_seconds <= 1 or not 0 <= args.roi_padding <= 1 or
            any(core < 0 for core in args.bpu_cores)):
        raise ValueError('Invalid detector or tracking parameters')
    if args.plane_distance is not None and (not np.isfinite(args.plane_distance) or args.plane_distance <= 0):
        raise ValueError('--plane-distance must be positive and finite')
    if not np.isfinite(args.cv_budget_ms) or args.cv_budget_ms <= 0:
        raise ValueError('--cv-budget-ms must be positive and finite')
    output = args.output or args.out/'after.mp4'
    paths = [output, args.out/'rows.jsonl', args.out/'camera_used.json', args.perf_log]
    resolved = [path.resolve() for path in paths]
    protected = [args.source.resolve(), args.weights.resolve()]
    if not args.no_correction:
        protected.append(args.camera_params.resolve())
    if any(path in resolved for path in protected) or len(set(resolved)) != len(resolved):
        raise ValueError('Input/model and output/log paths must differ')
    args.out.mkdir(parents=True, exist_ok=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    args.perf_log.parent.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(args.opencv_threads)
    cap = writer = gpu = None
    count = 0
    timings = {stage: [] for stage in STAGES}
    cv_modes = {}
    run_started = time.perf_counter()
    with args.perf_log.open('a', encoding='utf-8', buffering=1) as perf:
        _log(perf, event='run_start', source=str(args.source.resolve()), output=str(output.resolve()),
             pipeline='red-gate', backend=args.backend, weights=str(args.weights.resolve()),
             stages=list(STAGES), cv_every=args.cv_every, opencv_version=cv2.__version__,
             opencv_threads=cv2.getNumThreads(),
             cv_backend_requested=args.cv_backend,
             timing_scope=dict(preprocess='video read/decode, rotation and correction; no enhancement',
                               yolo='model input preparation, inference, decode/NMS and box conversion',
                               opencv='YOLO target association, ROI red-pipe search or tracking',
                               postprocess='geometry, guidance, drawing, result serialization and video write',
                               excluded='initialization, performance log writes, optional preview and encoder finalization'))
        try:
            init_started = time.perf_counter()
            gpu, cv_backend_info = create_backend(args.cv_backend, args.gpu_device, args.cv_hough, args.cv_blur, args.cv_quality)
            execution = args.cv_execution or ('resident' if gpu is not None else 'hybrid')
            if execution == 'resident' and gpu is None:
                raise ValueError('--cv-execution resident requires an OpenCL GPU backend')
            cv_backend_info['cv_execution'] = execution
            print(f'CV backend: {cv_backend_info}', flush=True)
            detector = create_detector(args)
            timed_detector = TimedBoxDetector(detector)
            camera = None if args.no_correction else load_camera_params(args.camera_params)
            cap = FFmpegVideoInput(args.source) if args.reader == 'ffmpeg' else cv2.VideoCapture(str(args.source))
            if not cap.isOpened():
                raise RuntimeError(f'Could not open video: {args.source}')
            fps = float(cap.get(cv2.CAP_PROP_FPS))
            if not np.isfinite(fps) or fps <= 0:
                fps = 30.
            first_started = time.perf_counter()
            ok, first_frame = cap.read()
            first_decode_ms = (time.perf_counter()-first_started)*1000
            if not ok:
                raise RuntimeError('Video contains no decodable frames')
            size = (first_frame.shape[1], first_frame.shape[0])
            corrector = None
            adaptation = None
            if camera is not None:
                adapted, adaptation = adapt_camera_params(camera, *size, args.camera_fit)
                corrector = create_corrector(adapted)
                corrector.maps(args.plane_distance)
                valid = corrector.valid_mask(args.plane_distance)
            else:
                valid = np.ones(first_frame.shape[:2], bool)
            valid.setflags(write=False)
            tracker = YoloRedGateTracker(timed_detector, fps=fps, detect_every=args.cv_every,
                hold_seconds=args.hold_seconds, valid_mask=valid, roi_padding=args.roi_padding,
                cv_contrast=1, cv_clahe_clip=0, cv_clahe_blend=0,
                cv_sharpen=0, cv_saturation=1, profile_cv=True, cv_backend=gpu,
                cv_execution=execution,
                adaptive_search=args.cv_search == 'adaptive')
            if execution == 'resident' and corrector is not None:
                tracker.cv.set_camera(corrector.output_matrix)
            warmup_started = time.perf_counter()
            if execution == 'hybrid':
                # Lookup construction is startup work, not a frame measurement.
                cv2.cvtColor(np.zeros((16, 16, 3), np.uint8), cv2.COLOR_BGR2LAB)
            if gpu is not None:
                with gpu.frame_batch():
                    maps = None if corrector is None else corrector.maps(args.plane_distance)
                    warm_fixed, warm_enhanced, _, _ = gpu.preprocess(
                        first_frame, valid, maps=maps,
                        device_reference=execution == 'resident', rotate_180=args.rotate_180)
                    if execution == 'resident':
                        # Exercise all kernels without calling external YOLO.
                        warm_boxes = [dict(class_id=0, score=1., bbox=[0, 0, *size])]
                        tracker.cv.update(warm_fixed, warm_enhanced, warm_boxes, {0}, valid)
                        tracker.cv.reset()
                        tracker.cv.index = 0
                    else:
                        from .detect_red_gate import prepare_detection_frame
                        warm_reference = prepare_detection_frame(warm_fixed)[0]
                        warm_extra = prepare_detection_frame(warm_enhanced)[0]
                        gpu.combined_evidence(warm_extra, warm_reference)
                        gpu.red_mask(warm_reference)
            _log(perf, event='warmup', warmup_ms=round((time.perf_counter()-warmup_started)*1000, 3),
                 scope='GPU preprocessing and CV kernels; no YOLO; tracker reset before video',
                 gpu=None if gpu is None else gpu.diagnostics())
            writer = VideoOutput(output, fps, size)
            metadata = dict(source=str(args.source.resolve()), output=str(output.resolve()),
                            pipeline='red-gate', backend=args.backend, weights=str(args.weights.resolve()),
                            class_names=detector.names, target_ids=sorted(detector.target_ids),
                            fps=fps, input_size=list(size), output_size=[size[0]+size[0] % 2, size[1]+size[1] % 2],
                            reader=args.reader, camera_adaptation=adaptation,
                            camera_params=None if camera is None else str(args.camera_params.resolve()),
                            correction=corrector is not None, rotate_180=args.rotate_180,
                            inference_coordinates='corrected' if corrector else 'raw',
                            cv_reference='clean frame', yolo_cv_input='clean frame',
                            enhancement_mode='none',
                            cv_every=args.cv_every, cv_search=args.cv_search, cv_execution=execution,
                            opencv_threads=cv2.getNumThreads(), cv_backend=cv_backend_info)
            if corrector is not None:
                metadata['output_matrix'] = corrector.output_matrix.tolist()
            (args.out/'camera_used.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n',
                                                    encoding='utf-8')
            _log(perf, event='initialized', init_ms=round((time.perf_counter()-init_started)*1000, 3),
                 names=detector.names, target_ids=sorted(detector.target_ids), input_size=list(size), fps=fps,
                 cv_backend=cv_backend_info)
            print(f'Backend={args.backend}; source={args.source}; output={output}; perf={args.perf_log}', flush=True)
            with (args.out/'rows.jsonl').open('w', encoding='utf-8') as rows:
                while args.max_frames is None or count < args.max_frames:
                    if gpu is not None:
                        gpu.reset_stats()
                    started = time.perf_counter()
                    if count == 0:
                        ok, frame = True, first_frame
                    else:
                        ok, frame = cap.read()
                    decoded_at = time.perf_counter()
                    if not ok:
                        break
                    if (frame.shape[1], frame.shape[0]) != size:
                        raise ValueError('Source dimensions changed within a video')
                    with (gpu.frame_batch() if gpu is not None else nullcontext()):
                        if gpu is None:
                            raw = cv2.rotate(frame, cv2.ROTATE_180) if args.rotate_180 else frame
                            fixed = raw if corrector is None else corrector.undistort(raw, args.plane_distance)
                            corrected_at = time.perf_counter()
                            enhanced = fixed
                        else:
                            maps = None if corrector is None else corrector.maps(args.plane_distance)
                            fixed, enhanced, correction_ms, _ = gpu.preprocess(
                                frame, valid, maps=maps,
                                device_reference=execution == 'resident', rotate_180=args.rotate_180)
                            corrected_at = decoded_at+correction_ms/1000
                        preprocessed_at = time.perf_counter()
                        selected, _ = tracker.update(fixed, yolo_frame=enhanced, cv_frame=enhanced)
                        detected_at = time.perf_counter()
                        status = tracker.last_status
                        profile = status['cv_profile']
                        geometry = original_geometry(fixed, selected)
                        boxes = [dict(box, selected=(status['yolo_age_frames'] == 0 and
                                                     box['bbox'] == status['target_bbox']))
                                 for box in status['yolo_detections']]
                        after = draw_detections(enhanced, boxes)
                        after = draw_gate_geometry(after, geometry, count, fps, 'AFTER')
                        guidance = None
                        if corrector is not None:
                            guidance = (tracker.cv.last_guidance if execution == 'resident' else
                                        build_gate_guidance(status, geometry, corrector.output_matrix, size, valid))
                            after = draw_gate_guidance(after, guidance, corrector.output_matrix)
                        _log(rows, frame=count+1, time_s=count/fps, detections=boxes,
                             geometry=geometry, guidance=guidance, status=status)
                        drawn_at = time.perf_counter()
                        writer.write(after)
                        written_at = time.perf_counter()
                        decode_ms = first_decode_ms if count == 0 else (decoded_at-started)*1000
                        stage_ms = dict(preprocess=(preprocessed_at-decoded_at)*1000+decode_ms,
                                        yolo=timed_detector.last_ms,
                                        opencv=max(0., (detected_at-preprocessed_at)*1000-timed_detector.last_ms),
                                        postprocess=(written_at-detected_at)*1000)
                        count += 1
                        for stage, value in stage_ms.items():
                            timings[stage].append(value)
                        cv_modes.setdefault(profile['mode'], []).append(stage_ms['opencv'])
                        details = dict(decode=decode_ms, correction=(corrected_at-decoded_at)*1000,
                                       enhancement=0.,
                                       drawing_and_results=(drawn_at-detected_at)*1000,
                                       video_write=(written_at-drawn_at)*1000)
                        if isinstance(detector, HbmBoxDetector):
                            details.update({'yolo_input': detector.timer.ms.get('YOLO预处理', 0.),
                                            'bpu_inference': detector.timer.ms.get('推理', 0.),
                                            'yolo_decode_nms': detector.timer.ms.get('后处理', 0.)})
                        _log(perf, event='frame', frame=count, time_s=round((count-1)/fps, 6),
                             timing_ms={name: round(value, 3) for name, value in stage_ms.items()},
                             total_ms=round(sum(stage_ms.values()), 3),
                             details_ms={name: round(value, 3) for name, value in details.items()},
                             yolo_count=status['yolo_count'], cv_mode=profile['mode'], cv_profile=profile,
                             cv_backend=cv_backend_info, gpu=None if gpu is None else gpu.diagnostics())
                        if count % 100 == 0:
                            print(f'Processed {count} frames; preprocess={stage_ms["preprocess"]:.1f}ms '
                                  f'YOLO={stage_ms["yolo"]:.1f}ms OpenCV={stage_ms["opencv"]:.1f}ms '
                                  f'postprocess={stage_ms["postprocess"]:.1f}ms', flush=True)
                        if args.show:
                            cv2.imshow('YOLO + optimized red-gate CV', after)
                            if cv2.waitKey(1) & 0xff == ord('q'):
                                break
            closing_started = time.perf_counter()
            writer.close()
            writer = None
            _log(perf, event='encoder_finalized', finalize_ms=round((time.perf_counter()-closing_started)*1000, 3))
            def summary(values):
                return dict(n=len(values), mean_ms=round(float(np.mean(values)), 3),
                            p95_ms=round(float(np.percentile(values, 95)), 3), max_ms=round(max(values), 3))
            _log(perf, event='run_summary', frames=count, elapsed_s=round(time.perf_counter()-run_started, 3),
                 stages={stage: summary(values) for stage, values in timings.items()},
                 cv_modes={mode: dict(**summary(values), budget_ms=args.cv_budget_ms,
                                      within_budget=sum(value <= args.cv_budget_ms for value in values),
                                      within_budget_pct=round(100*sum(value <= args.cv_budget_ms for value in values)/len(values), 2))
                           for mode, values in cv_modes.items()}, cv_backend=cv_backend_info)
            for mode, values in cv_modes.items():
                within = sum(value <= args.cv_budget_ms for value in values)
                print(f'CV {mode}: mean={np.mean(values):.1f}ms P95={np.percentile(values, 95):.1f}ms '
                      f'max={max(values):.1f}ms <= {args.cv_budget_ms:g}ms: {within}/{len(values)}', flush=True)
        except BaseException as exc:
            _log(perf, event='error', completed_frames=count, error=repr(exc))
            raise
        finally:
            if cap is not None:
                cap.release()
            try:
                if writer is not None:
                    writer.close()
            finally:
                if gpu is not None:
                    gpu.close()
                if args.show:
                    cv2.destroyAllWindows()
    print(f'Done: {count} frames -> {output.resolve()}\nPerformance log: {args.perf_log.resolve()}', flush=True)
    return 0
