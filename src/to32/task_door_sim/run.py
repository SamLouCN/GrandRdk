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
    parser = argparse.ArgumentParser(description='DoorSim：经 front.py 采集 640×480、校正、YOLO/GPU CV 并写 :5000/cam1 共享帧')
    parser.add_argument('--source', help='相机设备/索引或视频路径；默认用 front.py 前视相机配置')
    parser.add_argument('--backend', choices=('hbm', 'ultralytics'), default='hbm')
    parser.add_argument('--model', help='整门模型路径；默认使用 config/stage_model.py 的 gate 配置')
    parser.add_argument('--shm-dir', help='共享帧目录，前视与 web_server 必须一致')
    parser.add_argument('--no-correction', action='store_true')
    parser.add_argument('--cv-every', type=int, default=CONFIG.cv_every_frames,
                        help='CV 完整搜索间隔；其余帧按当前图像跟踪，默认 3')
    parser.add_argument('--jpeg-quality', type=int, default=CONFIG.jpeg_quality,
                        help='共享 JPEG 质量，默认 85')
    parser.add_argument('--no-cv-profile', action='store_true', help='关闭 CV 内部性能诊断')
    parser.add_argument('--cv-backend', choices=('auto', 'opencl', 'cpu'), default=CONFIG.cv_backend)
    parser.add_argument('--gpu-device', default=CONFIG.gpu_device)
    parser.add_argument('--cv-hough', choices=('opencl', 'cpu'), default=CONFIG.cv_hough)
    parser.add_argument('--cv-blur', choices=('pyramid', 'exact'), default=CONFIG.cv_blur)
    parser.add_argument('--cv-quality', choices=('fast', 'precise'), default=CONFIG.cv_quality)
    parser.add_argument('--cv-execution', choices=('resident', 'hybrid'), default=CONFIG.cv_execution,
                        help='GPU 默认 v7 常驻路径；CPU 后端使用 hybrid 对照')
    parser.add_argument('--cv-search', choices=('adaptive', 'full'), default=CONFIG.cv_search)
    parser.add_argument('--opencv-threads', type=int, default=CONFIG.opencv_threads)
    # front.py 的 --frames/--device/--index/--timing/--log 等原样传给它。
    args, front_args = parser.parse_known_args(argv)
    cfg = replace(CONFIG, correction_enabled=not args.no_correction,
                  cv_every_frames=args.cv_every, jpeg_quality=args.jpeg_quality,
                  cv_profile=not args.no_cv_profile, cv_backend=args.cv_backend,
                  gpu_device=args.gpu_device, cv_hough=args.cv_hough, cv_blur=args.cv_blur,
                  cv_quality=args.cv_quality, cv_execution=args.cv_execution, cv_search=args.cv_search,
                  opencv_threads=args.opencv_threads).validate()
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
    print('[DoorSim] 启动 front.py：%d×%d 前视采集 -> 校正/YOLO/GPU CV -> :5000/cam1；model=%s'
          % (front.W, front.H, model_cfg['model_path']), flush=True)
    try:
        return front.main(['--no-show', *front_args], door_sim_processor=processor,
                          door_sim_model_path=model_cfg['model_path'], video_source=video_source)
    finally:
        processor.close()


if __name__ == '__main__':
    raise SystemExit(main())
