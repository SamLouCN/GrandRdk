"""Inspect a saved YOLO ROI on a clean source frame without rerunning YOLO."""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import detect_red_gate as D, gate_models as M
from src.camera_correction import create_corrector
from src.gate_guidance import enhance_cv_contrast


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path)
    parser.add_argument('frame', type=int)
    parser.add_argument('--out', type=Path, default=ROOT/'runs/gate_diagnostic')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    row = json.loads(args.records.joinpath('rows.jsonl').read_text(encoding='utf-8').splitlines()[args.frame])
    metadata = json.loads(args.records.joinpath('camera_used.json').read_text(encoding='utf-8'))
    cap = cv2.VideoCapture(row['source'])
    cap.set(cv2.CAP_PROP_POS_FRAMES, args.frame)
    ok, raw = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError('Cannot read requested frame')
    corrector = create_corrector(metadata['adapted_camera'])
    fixed = corrector.undistort(raw, metadata['plane_distance_m'])
    valid = corrector.valid_mask(metadata['plane_distance_m'])
    enhanced = enhance_cv_contrast(fixed, valid_mask=valid)
    stages = []
    trim = M.trim_lines
    small = D.prepare_detection_frame(enhanced)[0]

    def redness(line):
        points = np.rint(np.linspace(*D.endpoints(line), 60)).astype(int)
        pixels = small[np.clip(points[:, 1], 0, 359), np.clip(points[:, 0], 0, 639)].astype(float)
        return float(np.median(np.log((pixels[:, 2]+10)/(pixels[:, 1]+10))))

    def record_trim(*args, **kwargs):
        result = trim(*args, **kwargs)
        stages.append([dict(width=line['width'], support=line['support'],
                            endpoints=D.endpoints(line).tolist(), vertical=bool(line['vertical']),
                            redness=redness(line)) for line in result])
        return result

    with patch.object(M, 'trim_lines', side_effect=record_trim):
        candidates, lines, mask = D.detect(enhanced, row['yolo']['search_bbox'],
                                           row['yolo']['target_bbox'], fixed, valid)
    (args.out/'line_stages.json').write_text(json.dumps(stages, indent=2), encoding='utf-8')
    geometries = [D.original_geometry(fixed, candidate) for candidate in candidates]
    selected = D.original_geometry(fixed, D.select_nearest(candidates))
    (args.out/'selected.json').write_text(json.dumps(selected, indent=2), encoding='utf-8')
    display = dict(selected, segments=selected['observed_segments'], corners=[]) if selected else None
    cv2.imencode('.jpg', D.draw_gate_geometry(fixed, display, args.frame, 30, 'SELECTED'))[1].tofile(str(args.out/'selected.jpg'))
    (args.out/'candidates.json').write_text(json.dumps(geometries, indent=2), encoding='utf-8')
    reference = D.prepare_detection_frame(fixed)[0]
    for name, value in [('weak_mask', mask), ('strict_mask', D.red_mask(reference))]:
        cv2.imencode('.png', value)[1].tofile(str(args.out/f'{name}.png'))
    cv2.imencode('.jpg', fixed)[1].tofile(str(args.out/'clean.jpg'))
    for index, candidate in enumerate(candidates):
        geometry = geometries[index]
        annotated = D.draw_gate_geometry(fixed, dict(geometry, segments=geometry['observed_segments']),
                                         args.frame, 30, f'{index}: {geometry["model"]}')
        cv2.imencode('.jpg', annotated)[1].tofile(str(args.out/f'candidate_{index}.jpg'))
    print(json.dumps([dict(model=item['model'], width=item['pipe_width_px'], bbox=item['bbox'],
                          complete=item['complete'], sides=len(item['segments'])) for item in geometries], indent=2))


if __name__ == '__main__':
    main()
