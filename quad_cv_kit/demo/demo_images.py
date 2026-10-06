"""Images: YOLO detection followed by OpenCV polygon fitting."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.media_demo import run_images

if __name__ == '__main__':
    try:
        sys.exit(run_images())
    except (ValueError, OSError, RuntimeError) as error:
        print(f'Error: {error}', file=sys.stderr)
        sys.exit(2)
