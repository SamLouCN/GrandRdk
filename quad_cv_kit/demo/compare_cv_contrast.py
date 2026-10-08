"""Same clean frames and saved YOLO ROIs: compare CV contrast, without tracking.

These counts measure detection availability, not accuracy without ground truth.
Run before replacing the previous demo's rows.jsonl.
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from src.detect_red_gate import detect, select_nearest, original_geometry, draw_gate_geometry
from src.gate_guidance import enhance_cv_contrast, build_gate_guidance
from src.yolo_red_gate import YoloRedGateTracker

VARIANTS = {'baseline': (1.12, 0, 0, 0, 1), 'global_stronger': (1.35, 0, 0, 0, 1),
            'local': (1.2, 2, .6, 0, 1), 'local_stronger': (1.35, 3, .75, 0, 1)}
ENHANCEMENT_VARIANTS = {'baseline': (1.2, 2, .6, 0, 1), 'sharpen': (1.2, 2, .6, .6, 1),
                       'saturation': (1.2, 2, .6, 0, 1.25), 'both': (1.2, 2, .6, .6, 1.25)}


def compare_sequence(source, rows, camera, variants):
    """Replay actual source frames and identical saved YOLO detections for all variants."""
    class SavedDetector:
        target_ids = {0}
        boxes = []

        def detect_boxes(self, frame):
            return self.boxes

    detector = SavedDetector()
    cap = cv2.VideoCapture(str(source))
    trackers = None
    keys = ('gate', 'four_edges', 'complete', 'fresh_complete', 'pose', 'fresh_pose', 'cv_runs')
    totals = {name: dict.fromkeys(keys, 0) for name in variants}
    try:
        for index, row in enumerate(rows):
            ok, raw = cap.read()
            if not ok:
                raise RuntimeError(f'Cannot read {source} at frame {index}')
            if trackers is None:
                size = (raw.shape[1], raw.shape[0])
                adapted, _ = adapt_camera_params(camera, *size, 'center-crop')
                corrector = create_corrector(adapted)
                valid = corrector.valid_mask()
                fps = float(cap.get(cv2.CAP_PROP_FPS))
                trackers = {name: YoloRedGateTracker(detector, fps, valid_mask=valid,
                             cv_contrast=gain, cv_clahe_clip=clip, cv_clahe_blend=blend,
                             cv_sharpen=sharpen, cv_saturation=saturation)
                            for name, (gain, clip, blend, sharpen, saturation) in variants.items()}
            frame = corrector.undistort(raw)
            detector.boxes = row['yolo']['yolo_detections']
            for name, tracker in trackers.items():
                selected, _ = tracker.update(frame)
                geometry = original_geometry(frame, selected)
                guidance = build_gate_guidance(tracker.last_status, geometry,
                                                corrector.output_matrix, size, valid)
                totals[name]['cv_runs'] += tracker.last_status['detection_ran']
                if geometry:
                    fresh = geometry['observation'] == 'detected'
                    totals[name]['gate'] += 1
                    totals[name]['four_edges'] += len(geometry['observed_segments']) == 4
                    totals[name]['complete'] += geometry['complete']
                    totals[name]['fresh_complete'] += bool(fresh and geometry['complete'])
                    totals[name]['pose'] += guidance['alignment'] is not None
                    totals[name]['fresh_pose'] += bool(fresh and guidance['alignment'] is not None)
            if index % 300 == 299:
                print(f'{source.stem}: replay {index+1}/{len(rows)}', flush=True)
    finally:
        cap.release()
    return dict(frames=len(rows), totals=totals)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('E:/TEST'))
    parser.add_argument('--records', type=Path, default=ROOT/'runs/cv_improved')
    parser.add_argument('--out', type=Path, default=ROOT/'runs/contrast_comparison')
    parser.add_argument('--stride', type=int, default=15)
    parser.add_argument('--sequence', action='store_true', help='All frames, including the real tracking pipeline')
    parser.add_argument('--video-stem', help='Compare only this source name, e.g. DOOR_TEST_2')
    parser.add_argument('--enhancement', action='store_true', help='Compare sharpening/saturation against contrast only')
    args = parser.parse_args()
    variants = ENHANCEMENT_VARIANTS if args.enhancement else VARIANTS
    if args.stride < 1:
        parser.error('stride must be positive')
    cv2.setNumThreads(4)
    args.out.mkdir(parents=True, exist_ok=True)
    report = dict(method='same corrected source frames and same saved YOLO ROIs; independent CV, no tracking',
                  limitations='No annotated ground truth; counts are availability, not recognition accuracy',
                  stride=args.stride, variants=variants,
                  variant_parameters=['contrast', 'clahe_clip', 'clahe_blend', 'sharpen', 'saturation'], videos={})
    camera = load_camera_params(ROOT/'camera_correction_params.json')
    for source in sorted(args.source.glob('DOOR_TEST_*.mp4')):
        if args.video_stem and source.stem != args.video_stem:
            continue
        old = args.records/source.stem
        rows = [json.loads(line) for line in (old/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
        if args.sequence:
            report['method'] = 'all clean corrected source frames, same cached per-frame YOLO detections; real CV tracking'
            result = compare_sequence(source, rows, camera, variants)
            result['previous_summary'] = json.loads((old/'summary.json').read_text(encoding='utf-8'))
            report['videos'][source.stem] = result
            print(source.stem, result['totals'], flush=True)
            (args.out/'sequence_comparison.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            continue
        totals = {name: dict(gate=0, four_edges=0, complete=0, pose=0) for name in variants}
        details = []
        cap = cv2.VideoCapture(str(source))
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open {source}')
        corrector = None
        examples = 0
        try:
            for index, row in enumerate(rows):
                ok, raw = cap.read()
                if not ok:
                    raise RuntimeError(f'Cannot read {source} at frame {index}')
                if index % args.stride or row['yolo']['yolo_age_frames'] != 0:
                    continue
                if corrector is None:
                    size = (raw.shape[1], raw.shape[0])
                    adapted, _ = adapt_camera_params(camera, *size, 'center-crop')
                    corrector = create_corrector(adapted)
                    valid = corrector.valid_mask()
                frame = corrector.undistort(raw)
                status = row['yolo']
                metrics = {}
                visuals = []
                for name, (gain, clip, blend, sharpen, saturation) in variants.items():
                    enhanced = enhance_cv_contrast(frame, gain, valid, clip, blend, sharpen, saturation)
                    candidates, _, _ = detect(enhanced, status['search_bbox'], status['target_bbox'], frame, valid)
                    geometry = original_geometry(frame, select_nearest(candidates))
                    guidance = build_gate_guidance(status, geometry, corrector.output_matrix, size, valid)
                    metrics[name] = dict(gate=int(geometry is not None),
                                         four_edges=int(geometry is not None and len(geometry['observed_segments']) == 4),
                                         complete=int(geometry is not None and geometry['complete']),
                                         pose=int(guidance['alignment'] is not None))
                    for key, value in metrics[name].items():
                        totals[name][key] += value
                    image = draw_gate_geometry(frame, geometry, index, row.get('fps', 30), name)
                    cv2.rectangle(image, tuple(np.rint(status['target_bbox'][:2]).astype(int)),
                                  tuple(np.rint(status['target_bbox'][2:]).astype(int)), (0, 255, 255), 2)
                    visuals.append(cv2.resize(image, (640, 360)))
                details.append(dict(frame=index, results=metrics))
                # Keep improvements AND regressions for visual review.
                if examples < 12 and any(metrics[name] != metrics['baseline'] for name in variants):
                    contact = np.vstack([np.hstack(visuals[:2]), np.hstack(visuals[2:])])
                    cv2.imencode('.jpg', contact)[1].tofile(str(args.out/f'{source.stem}_{index:06d}.jpg'))
                    examples += 1
        finally:
            cap.release()
        report['videos'][source.stem] = dict(samples=len(details), totals=totals,
                                              previous_summary=json.loads((old/'summary.json').read_text(encoding='utf-8')),
                                              frames=details)
        print(source.stem, len(details), totals, flush=True)
        (args.out/'comparison.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    if not report['videos']:
        raise RuntimeError(f'No DOOR_TEST source videos in {args.source}')


if __name__ == '__main__':
    main()
