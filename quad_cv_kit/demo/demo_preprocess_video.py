"""Export a clean high-contrast CV input, with no detection or box overlays."""
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
from src.camera_correction import adapt_camera_params, create_corrector, load_camera_params
from src.cv_preprocessing import high_contrast_frame
from src.gate_guidance import enhance_cv_contrast
from src.video_input import FFmpegVideoInput

PROFILES = {
    'moderate': dict(clahe_clip=2., clahe_blend=.55, contrast=1.3, saturation=1.3, sharpen=.2),
    'high': dict(clahe_clip=2.5, clahe_blend=.65, contrast=1.6, saturation=1.4, sharpen=.2),
    'strong': dict(clahe_clip=4., clahe_blend=.85, contrast=2., saturation=1.5, sharpen=.25),
}
SAMPLES = [0, 598, 1796, 2300, 3030, 4500, 5987, 7783, 8382]


def save_image(path, frame):
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError(f'Cannot save {path}')
    encoded.tofile(str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--out', type=Path, default=ROOT/'runs/door4_high_contrast')
    parser.add_argument('--profile', choices=PROFILES, default='high')
    parser.add_argument('--preview-only', action='store_true')
    parser.add_argument('--no-correction', action='store_true')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(4)
    reader = FFmpegVideoInput(args.source)
    width, height, fps, expected = reader.width, reader.height, reader.fps, reader.frame_count
    camera, adaptation = adapt_camera_params(load_camera_params(), width, height, 'center-crop')
    corrector = create_corrector(camera)
    valid = np.ones((height, width), bool) if args.no_correction else corrector.valid_mask()

    def correct(frame):
        return frame if args.no_correction else corrector.undistort(frame)

    if args.preview_only:
        reader.release()
        panels, metrics = [], []
        for index in SAMPLES:
            if expected and index >= expected:
                continue
            decoded = subprocess.check_output([
                shutil.which('ffmpeg'), '-v', 'error', '-nostdin', '-ss', str(index/fps),
                '-i', str(args.source), '-map', '0:v:0', '-frames:v', '1', '-threads', '2',
                '-pix_fmt', 'bgr24', '-f', 'rawvideo', 'pipe:1'])
            if len(decoded) != width*height*3:
                raise RuntimeError(f'Cannot extract sample {index}')
            clean = correct(np.frombuffer(decoded, np.uint8).reshape(height, width, 3))
            variants = {'corrected': clean, 'previous': enhance_cv_contrast(clean, valid_mask=valid)}
            variants.update({name: high_contrast_frame(clean, valid, **params)
                             for name, params in PROFILES.items()})
            row = []
            for name, frame in variants.items():
                save_image(args.out/f'frame_{index:06d}_{name}.jpg', frame)
                value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2][valid]
                metrics.append(dict(frame=index, variant=name, value_percentiles=np.percentile(value, [1, 10, 50, 90, 99]).tolist(),
                                    black_fraction=float((value <= 3).mean()), white_fraction=float((value >= 252).mean())))
                thumbnail = cv2.resize(frame, (512, 288))
                cv2.rectangle(thumbnail, (0, 0), (512, 25), (0, 0, 0), -1)
                cv2.putText(thumbnail, f'{index} / {index/fps:.1f}s / {name}', (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                row.append(thumbnail)
            panels.append(np.hstack(row))
        save_image(args.out/'comparison.jpg', np.vstack(panels))
        (args.out/'preview_metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
        print(f'Preview: {len(panels)} frames, corrected / previous / moderate / high / strong', flush=True)
        return

    # This entry point imports neither YOLO nor PyTorch. Encode the clean input only.
    destination = args.out/f'{args.source.stem}_high_contrast.mp4'
    if destination.resolve() == args.source.resolve():
        reader.release()
        raise ValueError('Output must differ from input')
    encoder = subprocess.Popen([
        shutil.which('ffmpeg'), '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}', '-r', str(fps), '-i', 'pipe:0', '-an',
        '-c:v', 'libx264', '-preset', 'fast', '-threads', '2', '-crf', '16',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(destination)], stdin=subprocess.PIPE,
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    count, started = 0, time.perf_counter()
    try:
        while True:
            ok, raw = reader.read()
            if not ok:
                break
            enhanced = high_contrast_frame(correct(raw), valid, **PROFILES[args.profile])
            encoder.stdin.write(np.ascontiguousarray(enhanced).tobytes())
            if count in SAMPLES:
                save_image(args.out/f'output_{count:06d}.jpg', enhanced)
            count += 1
            if count % 300 == 0:
                print(f'{count}/{expected} frames', flush=True)
    finally:
        reader.release()
        encoder.stdin.close()
        code = encoder.wait()
    if code:
        raise RuntimeError(f'Encoding failed: {code}')
    if expected and count != expected:
        raise RuntimeError(f'Incomplete video: {count}/{expected}')
    report = dict(source=str(args.source.resolve()), video=str(destination.resolve()), frames=count,
                  expected_frames=expected, size=[width, height], fps=fps,
                  correction=not args.no_correction, camera=camera, camera_adaptation=adaptation,
                  profile=args.profile, parameters=PROFILES[args.profile],
                  stages=['HSV value bilateral denoise', 'CLAHE', '1/99 percentile stretch',
                          'soft contrast curve', 'value unsharp', 'saturation'],
                  detection_ran=False, overlays=False, input_reader='ffmpeg', encoding_crf=16,
                  elapsed_s=round(time.perf_counter()-started, 3))
    (args.out/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
