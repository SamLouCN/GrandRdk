"""Randomly compare old and robust CV on identical frames and saved YOLO boxes."""
import argparse
import copy
import json
from pathlib import Path
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.yolo_quad import GateQuadProcessor, draw_detections
from src.camera_correction import create_corrector
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('rows', type=Path, help='Saved corrected-coordinate rows.jsonl')
    parser.add_argument('--out', type=Path, default=Path('runs/cv_comparison'))
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--seed', type=int, default=1006)
    args = parser.parse_args()
    if args.samples <= 0:
        raise ValueError('--samples must be positive')
    records = [json.loads(line) for line in args.rows.open(encoding='utf-8')]
    metadata = json.loads(args.rows.with_name('camera_used.json').read_text(encoding='utf-8'))
    eligible = [r for r in records if any(d.get('selected') for d in r['detections'])]
    # Sample frames with a YOLO target; CV success is not a condition.
    samples = sorted(random.Random(args.seed).sample(eligible, min(args.samples, len(eligible))),
                     key=lambda r: r['frame'])
    if not samples:
        raise ValueError('No detected doors to inspect')
    args.out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(samples[0]['source'])
    corrector = create_corrector(metadata['camera'])
    tiles, summary = [], []
    try:
        for rec in samples:
            if rec.get('coordinate_space') != 'corrected':
                raise ValueError('Expected corrected-coordinate detections')
            cap.set(cv2.CAP_PROP_POS_FRAMES, rec['frame'] - 1)
            ok, raw = cap.read()
            if not ok:
                raise RuntimeError(f'Cannot read frame {rec["frame"]}')
            corrector = corrector.for_frame(raw)
            if metadata.get('rotate_180'):
                raw = cv2.rotate(raw, cv2.ROTATE_180)
            fixed = corrector.undistort(raw, metadata.get('plane_distance_m'))
            sx, sy = fixed.shape[1]/metadata['output_size'][0], fixed.shape[0]/metadata['output_size'][1]
            detections = copy.deepcopy(rec['detections'])
            for d in detections:
                d['bbox'] = (np.asarray(d['bbox'])*[sx, sy, sx, sy]).tolist()
            views, results = [], {}
            for name, robust in [('old', False), ('robust', True)]:
                processor = GateQuadProcessor(opts={'robust_lines': robust, 'fx': corrector.f_out})
                result = processor.process(fixed, copy.deepcopy(detections))
                quad = next(d['quad'] for d in result if d.get('selected'))
                results[name] = dict(lvl=quad['lvl'], corners=quad['corners'],
                                     evidence=quad['diag'].get('full_edge_support'),
                                     why=quad['why'])
                vis = draw_detections(fixed, result, header=f'{name.upper()} | frame {rec["frame"]}')
                cv2.imwrite(str(args.out / f'{rec["frame"]}_{name}.jpg'), vis)
                views.append(cv2.resize(vis, (640, int(fixed.shape[0]*640/fixed.shape[1]))))
            tiles.append(np.hstack(views))
            summary.append(dict(frame=rec['frame'], **results))
        cv2.imwrite(str(args.out / 'comparison.jpg'), np.vstack(tiles))
        (args.out / 'summary.json').write_text(json.dumps(dict(seed=args.seed, frames=summary),
                                                       indent=2, ensure_ascii=False), encoding='utf-8')
    finally:
        cap.release()
    print(f'Compared {len(samples)} randomly selected target frames; output={args.out.resolve()}')


if __name__ == '__main__':
    main()
