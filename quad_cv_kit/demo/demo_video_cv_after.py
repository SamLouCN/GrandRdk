"""Render one corrected MP4 with YOLO boxes, CV pipe edges and UI guidance."""
import argparse
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.yolo_quad import DEFAULT_WEIGHTS, YoloQuadDetector, draw_detections
from src.camera_correction import load_camera_params, adapt_camera_params, create_corrector
from src.detect_red_gate import original_geometry, draw_gate_geometry
from src.gate_guidance import build_gate_guidance, draw_gate_guidance
from src.yolo_red_gate import YoloRedGateTracker
from src.video_input import FFmpegVideoInput
from demo_video_cv_improved import VideoOutput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        raise ValueError('Output must differ from input')
    detector = YoloQuadDetector(weights=args.weights, device=args.device)
    print(f'YOLO: {detector.names}, device={args.device}', flush=True)
    cap = FFmpegVideoInput(args.source)
    writer = None
    count = 0
    try:
        fps = float(cap.get(cv2.CAP_PROP_FPS)) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        corrector = tracker = None
        args.output.parent.mkdir(parents=True, exist_ok=True)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if corrector is None:
                size = (frame.shape[1], frame.shape[0])
                camera, _ = adapt_camera_params(load_camera_params(), *size, 'center-crop')
                corrector = create_corrector(camera)
                valid_mask = corrector.valid_mask()
                tracker = YoloRedGateTracker(
                    detector, fps, valid_mask=valid_mask, cv_contrast=1.2,
                    cv_clahe_clip=0, cv_clahe_blend=0, cv_sharpen=.6, cv_saturation=1)
                writer = VideoOutput(args.output, fps, size)
                print(f'{args.source.name}: {total} frames -> {args.output}', flush=True)
            fixed = corrector.undistort(frame)
            selected, _ = tracker.update(fixed)
            status = tracker.last_status
            geometry = original_geometry(fixed, selected)
            display = (dict(geometry, segments=geometry['observed_segments'], corners=[])
                       if geometry is not None else None)
            guidance = build_gate_guidance(status, geometry, corrector.output_matrix, size, valid_mask)
            boxes = [dict(box, selected=(status['yolo_age_frames'] == 0
                                        and box['bbox'] == status['target_bbox']))
                     for box in status['yolo_detections']]
            after = draw_detections(fixed, boxes)
            after = draw_gate_geometry(after, display, count, fps, 'AFTER')
            after = draw_gate_guidance(after, guidance, corrector.output_matrix)
            writer.write(after)
            count += 1
            if count % 150 == 0:
                print(f'  {count}/{total}', flush=True)
        if count == 0:
            raise RuntimeError('No readable frames')
    finally:
        cap.release()
        if writer is not None:
            writer.close()
    print(f'Done: {count} frames -> {args.output}', flush=True)


if __name__ == '__main__':
    main()
