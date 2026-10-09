"""Profile real CV on an image/video or a reproducible synthetic clutter scene."""
import argparse
import cProfile
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from quad_cv_kit.src.detect_red_gate import RedGateTracker
from quad_cv_kit.src.gate_guidance import enhance_cv_contrast
from quad_cv_kit.src.cv_profile import CvProfileReporter


def synthetic_scene(clutter=False):
    frame = np.full((360, 640, 3), (120, 90, 40), np.uint8)
    cv2.rectangle(frame, (220, 130), (390, 275), (50, 60, 230), 5)
    if clutter:
        rng = np.random.default_rng(17)
        for _ in range(50):
            x, y = rng.integers([50, 30], [590, 330])
            length = int(rng.integers(35, 130))
            end = ((min(639, int(x)+length), int(y)) if rng.random() < .5
                   else (int(x), min(359, int(y)+length)))
            cv2.line(frame, (int(x), int(y)), end, (50, 60, 230), int(rng.integers(2, 7)))
    return frame


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--source', type=Path, help='图像或录像路径；无需 YOLO/BPU')
    source.add_argument('--synthetic', choices=('clean', 'clutter'))
    parser.add_argument('--roi', nargs=4, type=float, metavar=('X1', 'Y1', 'X2', 'Y2'),
                        help='原图坐标搜索区域；默认全图')
    parser.add_argument('--frames', type=int, default=12)
    parser.add_argument('--cv-every', type=int, default=3)
    parser.add_argument('--opencv-threads', type=int, help='固定 OpenCV 线程数，便于优化前后对比')
    parser.add_argument('--out', type=Path, help='保存各帧全部计时、计数与汇总 JSON')
    parser.add_argument('--python-profile', type=Path, help='额外保存 cProfile 数据，计时会有额外开销')
    args = parser.parse_args(argv)
    if args.frames < 1 or args.cv_every < 1:
        parser.error('--frames 和 --cv-every 必须为正整数')
    if args.opencv_threads is not None:
        if args.opencv_threads < 1:
            parser.error('--opencv-threads 必须为正整数')
        cv2.setNumThreads(args.opencv_threads)
    cap = None
    if args.synthetic:
        image = synthetic_scene(args.synthetic == 'clutter')
    else:
        image = cv2.imread(str(args.source))
        if image is None:
            cap = cv2.VideoCapture(str(args.source))
            if not cap.isOpened():
                cap.release()
                parser.error('无法打开输入图像或录像')
    # Warm up LAB conversion separately: first-call setup is not steady-state CV work.
    cv2.cvtColor(np.zeros((360, 640, 3), np.uint8), cv2.COLOR_BGR2LAB)
    tracker = RedGateTracker(detect_every=args.cv_every, profile=True)
    reporter = CvProfileReporter()
    results = []
    python_profile = cProfile.Profile() if args.python_profile else None
    if python_profile:
        python_profile.enable()
    try:
        for index in range(args.frames):
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    break
            else:
                frame = image
            valid = np.ones(frame.shape[:2], bool)
            enhanced = enhance_cv_contrast(frame, 1.2, valid, 0, 0, .6, 1)
            tracker.update(enhanced, search_bbox=args.roi, reference_frame=frame, valid_mask=valid)
            profile = tracker.last_status['cv_profile']
            results.append(dict(profile, frame=index))
            print(json.dumps(CvProfileReporter.details(results[-1]), ensure_ascii=False))
            for report in reporter.add(profile, index):
                print(json.dumps(report, ensure_ascii=False))
    finally:
        if cap is not None:
            cap.release()
        if python_profile:
            python_profile.disable()
            args.python_profile.parent.mkdir(parents=True, exist_ok=True)
            python_profile.dump_stats(str(args.python_profile))
    modes = {}
    for mode in sorted({result['mode'] for result in results}):
        values = [r['total_ms'] for r in results if r['mode'] == mode]
        modes[mode] = dict(n=len(values), mean_ms=round(float(np.mean(values)), 2),
                          p95_ms=round(float(np.percentile(values, 95)), 2), max_ms=round(max(values), 2))
    summary = dict(source=str(args.source) if args.source else args.synthetic,
                   input_space='uncorrected', roi=args.roi, opencv_version=cv2.__version__,
                   opencv_threads=cv2.getNumThreads(), modes=modes)
    print(json.dumps(summary, ensure_ascii=False))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(dict(summary=summary, frames=results), ensure_ascii=False, indent=2)+'\n')
    return 0 if results else 1


if __name__ == '__main__':
    raise SystemExit(main())
