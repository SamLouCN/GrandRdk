#!/usr/bin/env python3
"""门视觉测试启动入口；采集、JPEG/JSON/统计共享写入统一交给 front.py。"""
import argparse
from dataclasses import replace
import os
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[3]
for _path in (_ROOT, _ROOT/'src', _ROOT/'config'):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from to32.task_door_sim.config import CONFIG


def main(argv=None):
    parser = argparse.ArgumentParser(description='DoorSim：经 front.py 采集、增强、YOLO/CV 并写 /cam1 共享帧')
    parser.add_argument('--source', help='相机设备/索引或视频路径；默认用 front.py 前视相机配置')
    parser.add_argument('--backend', choices=('hbm', 'ultralytics'), default='hbm')
    parser.add_argument('--model', help='整门模型路径；默认使用 config/stage_model.py 的 gate 配置')
    parser.add_argument('--shm-dir', help='共享帧目录，前视与 web_server 必须一致')
    parser.add_argument('--no-correction', action='store_true')
    parser.add_argument('--contrast', type=float, default=CONFIG.contrast_gain)
    parser.add_argument('--sharpen', type=float, default=CONFIG.sharpen_amount)
    parser.add_argument('--cv-every', type=int, default=CONFIG.cv_every_frames,
                        help='CV 完整搜索间隔；其余帧按当前图像跟踪，默认 3')
    parser.add_argument('--jpeg-quality', type=int, default=CONFIG.jpeg_quality,
                        help='共享 JPEG 质量，默认 85')
    # front.py 的 --frames/--device/--index/--timing/--log 等原样传给它。
    args, front_args = parser.parse_known_args(argv)
    cfg = replace(CONFIG, correction_enabled=not args.no_correction,
                  contrast_gain=args.contrast, sharpen_amount=args.sharpen,
                  cv_every_frames=args.cv_every, jpeg_quality=args.jpeg_quality).validate()
    if args.shm_dir:
        shm_dir = Path(args.shm_dir).resolve()
        shm_dir.mkdir(parents=True, exist_ok=True)
        os.environ['GRDK_SHM_DIR'] = str(shm_dir)

    import front
    from stage_model import model_config
    from to32.task_door_sim.perception import DoorSimFrameProcessor

    model_cfg = model_config('PassGate', front.YOLO_CFG)
    model_cfg['backend'] = args.backend
    if args.backend == 'ultralytics':
        model_cfg['model_path'] = str(_ROOT/'quad_cv_kit/model/best.pt')
    if args.model:
        model_cfg['model_path'] = str(Path(args.model).resolve())
    detector = front.YoloDetector(model_cfg)
    processor = DoorSimFrameProcessor(detector, cfg, fps=front.FPS)
    video_source = None
    if args.source is not None:
        if args.source.isdecimal():
            front_args.extend(['--index', args.source])
            # index 显式指定时不沿用配置中的 device。
            front_args.extend(['--device', '/dev/video' + args.source])
        elif args.source.startswith('/dev/'):
            front_args.extend(['--device', args.source])
        else:
            video_source = args.source
    print('[DoorSim] 启动 front.py：前视采集 -> 增强/YOLO/CV -> 前视共享帧；model=%s'
          % model_cfg['model_path'], flush=True)
    return front.main(['--no-show', *front_args], door_sim_processor=processor,
                      door_sim_model_path=model_cfg['model_path'], video_source=video_source)


if __name__ == '__main__':
    raise SystemExit(main())
