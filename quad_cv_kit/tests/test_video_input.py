"""Verify FFmpeg input preserves actual frames, dimensions and frame rate."""
from pathlib import Path
import shutil
import tempfile
import unittest

import cv2
import numpy as np

from src.video_input import FFmpegVideoInput


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'FFmpeg is unavailable')
class VideoInputTests(unittest.TestCase):
    def test_decodes_each_frame_and_handles_eof_and_early_release(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'测试.avi'
            writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'MJPG'), 12., (160, 120))
            self.assertTrue(writer.isOpened())
            expected = []
            for value in (30, 100, 180):
                frame = np.full((120, 160, 3), value, np.uint8)
                cv2.rectangle(frame, (40, 30), (120, 90), (40, 60, 210), -1)
                writer.write(frame)
                expected.append(frame)
            writer.release()
            reader = FFmpegVideoInput(source)
            try:
                self.assertEqual(reader.get(cv2.CAP_PROP_FPS), 12.)
                self.assertEqual(reader.get(cv2.CAP_PROP_FRAME_COUNT), 3)
                for frame in expected:
                    ok, decoded = reader.read()
                    self.assertTrue(ok)
                    self.assertEqual(decoded.shape, frame.shape)
                    self.assertLess(np.abs(decoded.astype(float)-frame).mean(), 3)
                self.assertEqual(reader.read(), (False, None))
                self.assertEqual(reader.read(), (False, None))
            finally:
                reader.release()
            reader.release()
            reader = FFmpegVideoInput(source)
            reader.release()
            self.assertEqual(reader.read(), (False, None))


if __name__ == '__main__':
    unittest.main()
