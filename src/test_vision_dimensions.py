"""Front VGA geometry must not rescale bottom observations or override actual sizes."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/to32/move_test'), str(ROOT/'src'), str(ROOT/'config')]
from obs import VisionIF


class VisionDimensionsTests(unittest.TestCase):
    def poll(self, cam, bbox, **dimensions):
        with tempfile.TemporaryDirectory() as directory:
            vision = VisionIF(shm_dir=directory)
            path = Path(directory)/vision.file[cam]
            path.write_text(json.dumps(dict(frame=1, dets=[dict(label='door', score=.9, bbox=bbox)],
                                            **dimensions)))
            return vision.poll(cam, 'gate', time.time())

    def test_front_center_and_normalization(self):
        obs = self.poll('front', [320, 240, 480, 360])
        self.assertEqual((obs['img_w'], obs['img_h']), (640, 480))
        self.assertEqual((obs['dx'], obs['dy']), (80, 60))
        self.assertEqual((obs['ex'], obs['ey']), (.25, .25))
        self.assertFalse(obs['clip'])
        clipped = self.poll('front', [400, 300, 640, 480])
        self.assertTrue(clipped['clip_r'] and clipped['clip_b'])

    def test_bottom_retains_hd_center_normalization_and_boundaries(self):
        obs = self.poll('bottom', [640, 360, 960, 540])
        self.assertEqual((obs['img_w'], obs['img_h']), (1280, 720))
        self.assertEqual((obs['dx'], obs['dy']), (160, 90))
        self.assertEqual((obs['ex'], obs['ey']), (.25, .25))
        self.assertFalse(obs['clip'])

    def test_actual_payload_size_overrides_defaults_and_invalid_size_is_rejected(self):
        obs = self.poll('front', [100, 50, 700, 550], img_w=800, img_h=600)
        self.assertEqual((obs['dx'], obs['dy']), (0, 0))
        self.assertFalse(obs['clip'])
        for value in (0, -1, float('nan'), 'invalid'):
            with self.subTest(value=value):
                self.assertIsNone(self.poll('front', [1, 1, 20, 20], img_w=value))


if __name__ == '__main__':
    unittest.main()
