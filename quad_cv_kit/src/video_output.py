"""Single MP4 output with checked FFmpeg completion or OpenCV fallback."""
import shutil
import subprocess

import cv2
import numpy as np


class VideoOutput:
    def __init__(self, path, fps, size):
        self.encoder = None
        self.writer = None
        self.size = size
        ffmpeg = shutil.which('ffmpeg')
        if ffmpeg:
            # Embedded FFmpeg builds can omit libx264. Avoid choosing an
            # encoder that fails only after the first processed frame.
            try:
                encoders = subprocess.run([ffmpeg, '-hide_banner', '-encoders'],
                    capture_output=True, text=True, timeout=10, check=True).stdout
            except (OSError, subprocess.SubprocessError):
                ffmpeg = None
            else:
                if not any(len(parts := line.split()) > 1 and parts[1] == 'libx264'
                           for line in encoders.splitlines()):
                    ffmpeg = None
        if ffmpeg:
            self.encoder = subprocess.Popen([
                ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
                '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{size[0]}x{size[1]}',
                '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                '-preset', 'fast', '-threads', '2', '-crf', '20',
                '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)
        else:
            self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'),
                fps, (size[0]+size[0] % 2, size[1]+size[1] % 2))
            if not self.writer.isOpened():
                self.writer.release()
                raise RuntimeError(f'Cannot create video: {path}')

    def write(self, frame):
        if (frame.shape[1], frame.shape[0]) != self.size:
            raise ValueError('Source dimensions changed within a video')
        if self.encoder is not None:
            try:
                self.encoder.stdin.write(np.ascontiguousarray(frame).tobytes())
            except BrokenPipeError as exc:
                raise RuntimeError('FFmpeg stopped while encoding output video') from exc
        else:
            padded = cv2.copyMakeBorder(frame, 0, frame.shape[0] % 2,
                                        0, frame.shape[1] % 2, cv2.BORDER_CONSTANT)
            self.writer.write(padded)

    def close(self):
        if self.encoder is not None:
            encoder, self.encoder = self.encoder, None
            try:
                encoder.stdin.close()
            finally:
                code = encoder.wait()
            if code != 0:
                raise RuntimeError(f'FFmpeg encoding failed (exit {code})')
        if self.writer is not None:
            self.writer.release()
            self.writer = None
