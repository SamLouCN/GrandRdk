"""Video entry: quad_cv_kit/demo/demo_video.py (run from the project root).

--pipeline red-gate selects the S100 HBM/OpenCL path with frame-resident device
inputs. See quad_cv_kit/GPU_SEARCH.md for the complete board command and audit.
The default legacy pipeline retains its existing behavior.
"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.media_demo import run_video


def main(argv=None):
    selector = argparse.ArgumentParser(add_help=False)
    selector.add_argument('--pipeline', choices=('legacy', 'red-gate'), default='legacy')
    args, remaining = selector.parse_known_args(argv)
    if args.pipeline == 'red-gate':
        from src.red_gate_video import run_video as run_red_gate_video
        return run_red_gate_video(remaining)
    if '--help' in remaining or '-h' in remaining:
        print('Pipeline: --pipeline legacy (default) | --pipeline red-gate (optimized CV, S100 HBM)')
    return run_video(remaining)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError) as error:
        print(f'Error: {error}', file=sys.stderr)
        sys.exit(2)
