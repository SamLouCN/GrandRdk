"""Require a real GPU; compile and validate all migrated stages before video use."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.opencl_backend import create_backend
from src.opencl_validation import validate_backend, benchmark_cv, benchmark_search
from src.resident_validation import check_resident, benchmark_resident


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu-device', help='Device/vendor substring, e.g. Mali')
    parser.add_argument('--report', type=Path, help='Optional JSON report file')
    parser.add_argument('--benchmark-frames', type=int, default=0,
                        help='After validation, time full CV with GPU/CPU Hough; recommend 12 or more')
    parser.add_argument('--benchmark-search-frames', type=int, default=0,
                        help='Compare previous CPU search stages with batched GPU search on identical input')
    parser.add_argument('--resident-only', action='store_true', help='Validate the resident GPU core only')
    parser.add_argument('--benchmark-resident-frames', type=int, default=0,
                        help='Measure resident search/track and per-stage GPU event durations after six warmup frames')
    parser.add_argument('--opencv-threads', type=int, default=3)
    parser.add_argument('--cv-blur', choices=('pyramid', 'exact'), default='pyramid')
    parser.add_argument('--cv-quality', choices=('fast', 'precise'), default='fast')
    args = parser.parse_args(argv)
    if min(args.benchmark_frames, args.benchmark_search_frames, args.benchmark_resident_frames) < 0 or args.opencv_threads < 1:
        parser.error('benchmark-frames must be nonnegative and opencv-threads positive')
    import cv2
    cv2.setNumThreads(args.opencv_threads)
    backend = None
    result = dict(passed=False, checks=[])
    try:
        backend, info = create_backend('opencl', args.gpu_device, blur_mode=args.cv_blur, quality=args.cv_quality)
        result['device'] = info
        print(json.dumps(dict(event='device', **info), ensure_ascii=False), flush=True)

        def report(item):
            result['checks'].append(item)
            print(json.dumps(item, ensure_ascii=False), flush=True)

        if args.resident_only:
            report(check_resident(backend))
        else:
            validate_backend(backend, report)
            report(check_resident(backend))
        if args.benchmark_resident_frames:
            result['resident_benchmark'] = benchmark_resident(backend, args.benchmark_resident_frames,
                report=lambda item: print(json.dumps(item, ensure_ascii=False), flush=True))
        if args.benchmark_frames:
            result['benchmark'] = benchmark_cv(backend, frames=args.benchmark_frames,
                report=lambda item: print(json.dumps(item, ensure_ascii=False), flush=True))
        if args.benchmark_search_frames:
            result['search_benchmark'] = benchmark_search(backend, frames=args.benchmark_search_frames,
                report=lambda item: print(json.dumps(item, ensure_ascii=False), flush=True))
        result['passed'] = True
        print(json.dumps(dict(event='validation_complete', passed=True,
                             checks=len(result['checks'])), ensure_ascii=False), flush=True)
    except Exception as exc:
        result['error'] = repr(exc)
        print(json.dumps(dict(event='validation_failed', error=repr(exc)), ensure_ascii=False),
              file=sys.stderr, flush=True)
    finally:
        if backend is not None:
            backend.close()
        if args.report is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
