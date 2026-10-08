"""Validate full paired videos, records and randomly sampled exported frames."""
import argparse
import json
from pathlib import Path
import random
import subprocess

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT/'runs/cv_improved')
    parser.add_argument('--out', type=Path, default=ROOT/'runs/contrast_comparison')
    parser.add_argument('--baseline', type=Path, help='Optional previous output: compare counts and per-frame YOLO detections')
    parser.add_argument('--verify-source', action='store_true',
                        help='Decode source videos with ffprobe and require every source frame in the outputs')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(20261008)
    validation = {}
    panels = []
    recovered_panels = []
    directories = sorted(path.parent for path in args.input.glob('*/summary.json'))
    if not directories:
        parser.error(f'No completed video outputs found in {args.input}')
    for directory in directories:
        name = directory.name
        summary = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
        rows = [json.loads(line) for line in (directory/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
        assert len(rows) == summary['frames']
        poses = []
        for index, row in enumerate(rows):
            assert index == row['frame']
            pose = row['guidance']['alignment']
            if pose:
                assert len(row['nearest_gate']['observed_segments']) == 4
                assert row['yolo']['yolo_age_frames'] == 0
                assert not row['yolo']['target_clipped']
                assert not pose['metric_distance_available']
                assert np.isfinite(pose['rotation_xyz_deg']).all()
                np.testing.assert_allclose(pose['translation_scaled_xyz'],
                    np.array(pose['translation_pixel_equivalent_xyz'])*summary['pixel_shift_scale'])
                poses.append(index)
        probes = {}
        for view in summary['videos']:
            video = directory/f'nearest_gate_{view}.mp4'
            data = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames',
                '-select_streams', 'v:0', '-show_entries', 'stream=width,height,avg_frame_rate,nb_read_frames',
                '-of', 'json', str(video)], text=True))['streams'][0]
            assert int(data['nb_read_frames']) == len(rows)
            assert [data['width'], data['height']] == summary['size']
            numerator, denominator = map(float, data['avg_frame_rate'].split('/'))
            assert abs(numerator/denominator-summary['fps']) < 1e-4
            probes[view] = data
        assert probes['before'] == probes['after']
        # Random pose, partial/clipped and arbitrary frames from exported videos.
        partials = [i for i, row in enumerate(rows) if row['nearest_gate']
                    and row['guidance']['alignment'] is None]
        indices = [rng.choice(poses) if poses else rng.randrange(len(rows)),
                   rng.choice(partials) if partials else rng.randrange(len(rows)), rng.randrange(len(rows))]
        for index in indices:
            pair = []
            for view in summary['videos']:
                cap = cv2.VideoCapture(str(directory/f'nearest_gate_{view}.mp4'))
                try:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                    ok, frame = cap.read()
                    assert ok
                finally:
                    cap.release()
                cv2.imencode('.jpg', frame)[1].tofile(str(args.out/f'check_{name}_{index:06d}_{view}.jpg'))
                pair.append(cv2.resize(frame, (640, 360)))
            panels.append(np.hstack(pair))
        validation[name] = dict(frames=len(rows), pose_records=len(poses),
                                 random_frames=indices, videos=probes)
        if args.verify_source:
            source_probe = json.loads(subprocess.check_output([
                'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                '-show_entries', 'stream=width,height,avg_frame_rate,nb_frames,nb_read_frames',
                '-of', 'json', summary['source']], text=True))['streams'][0]
            assert int(source_probe['nb_read_frames']) == len(rows), f'{name}: source frames were skipped'
            assert [source_probe['width'], source_probe['height']] == summary['size']
            source_num, source_den = map(float, source_probe['avg_frame_rate'].split('/'))
            assert abs(source_num/source_den-summary['fps']) < 1e-4
            validation[name]['source'] = source_probe
        model_counts = {}
        for row in rows:
            gate = row['nearest_gate']
            if gate:
                model = gate.get('model', 'endpoint-chain')
                model_counts[model] = model_counts.get(model, 0)+1
                if model == 'four-line-color-validated':
                    assert len(gate['side_support']) == len(gate['segments'])
                    assert min(gate['side_support']) >= .62
        validation[name]['model_frames'] = model_counts
        if args.baseline:
            old_directory = args.baseline/name
            previous = json.loads((old_directory/'summary.json').read_text(encoding='utf-8'))
            old_rows = [json.loads(line) for line in (old_directory/'rows.jsonl').read_text(encoding='utf-8').splitlines()]
            assert len(old_rows) == len(rows)
            mismatch = [index for index, (old, new) in enumerate(zip(old_rows, rows))
                        if old['yolo']['yolo_detections'] != new['yolo']['yolo_detections']]
            metric_names = ('gate_frames', 'complete_frames', 'fresh_complete_frames',
                            'four_observed_edge_frames', 'fresh_pose_frames')
            validation[name]['comparison'] = dict(
                baseline={key: previous[key] for key in metric_names},
                enhanced={key: summary[key] for key in metric_names},
                baseline_pose_frames=previous['guidance_frames'].get('align', 0),
                enhanced_pose_frames=summary['guidance_frames'].get('align', 0),
                yolo_detection_mismatch_frames=mismatch)
            def fresh_complete(row):
                gate = row['nearest_gate']
                return bool(gate and gate['complete'] and gate['observation'] == 'detected')

            gained = [index for index, (old, new) in enumerate(zip(old_rows, rows))
                      if fresh_complete(new) and not fresh_complete(old)]
            lost = [index for index, (old, new) in enumerate(zip(old_rows, rows))
                    if fresh_complete(old) and not fresh_complete(new)]
            selected_gains = rng.sample(gained, min(3, len(gained)))
            validation[name]['comparison'].update(
                newly_complete_fresh_frames=gained, lost_complete_fresh_frames=lost,
                randomly_checked_newly_complete_frames=selected_gains)
            for index in selected_gains:
                triplet = []
                for path, label in ((old_directory/'nearest_gate_after.mp4', 'BASELINE'),
                                    (directory/'nearest_gate_after.mp4', 'MIGRATED'),
                                    (directory/'nearest_gate_cv_input.mp4', 'CV INPUT')):
                    if not path.exists():
                        continue
                    cap = cv2.VideoCapture(str(path))
                    try:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                        ok, frame = cap.read()
                        assert ok
                    finally:
                        cap.release()
                    frame = cv2.resize(frame, (640, 360))
                    cv2.rectangle(frame, (0, 0), (640, 25), (0, 0, 0), -1)
                    cv2.putText(frame, f'{name} / {index} / {label}', (10, 19),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                    triplet.append(frame)
                panel = np.hstack(triplet)
                cv2.imencode('.jpg', panel)[1].tofile(str(args.out/f'recovered_{name}_{index:06d}.jpg'))
                recovered_panels.append(panel)
    cv2.imencode('.jpg', np.vstack(panels))[1].tofile(str(args.out/'random_frames.jpg'))
    if recovered_panels:
        cv2.imencode('.jpg', np.vstack(recovered_panels))[1].tofile(str(args.out/'newly_complete_frames.jpg'))
    (args.out/'validation.json').write_text(json.dumps(validation, indent=2), encoding='utf-8')
    print(json.dumps(validation, indent=2))


if __name__ == '__main__':
    main()
