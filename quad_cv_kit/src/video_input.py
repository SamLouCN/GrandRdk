"""FFmpeg frame reader for files which OpenCV's bundled decoder stops early on."""
from fractions import Fraction
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import cv2
import numpy as np


class FFmpegVideoInput:
    def __init__(self, source):
        ffmpeg, ffprobe = shutil.which('ffmpeg'), shutil.which('ffprobe')
        if not ffmpeg or not ffprobe:
            raise RuntimeError('FFmpeg input requires ffmpeg and ffprobe on PATH')
        source = str(Path(source))
        data = json.loads(subprocess.check_output([
            ffprobe, '-v', 'error', '-select_streams', 'v:0', '-show_entries',
            'stream=width,height,avg_frame_rate,r_frame_rate,nb_frames', '-of', 'json', source]))
        if not data.get('streams'):
            raise RuntimeError(f'No video stream: {source}')
        stream = data['streams'][0]
        self.width, self.height = int(stream['width']), int(stream['height'])
        self.fps = 0.
        for key in ('avg_frame_rate', 'r_frame_rate'):
            try:
                self.fps = float(Fraction(stream[key]))
            except (KeyError, ValueError, ZeroDivisionError):
                continue
            if self.fps > 0:
                break
        try:
            self.frame_count = int(stream.get('nb_frames', 0))
        except (ValueError, TypeError):
            self.frame_count = 0
        self.frame_bytes = self.width*self.height*3
        self.ended = False
        self.log = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen([
                ffmpeg, '-v', 'error', '-nostdin', '-threads', '2', '-noautorotate',
                '-i', source, '-map', '0:v:0', '-an', '-sn', '-dn',
                '-fps_mode', 'passthrough', '-threads', '2', '-pix_fmt', 'bgr24',
                '-f', 'rawvideo', 'pipe:1'], stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=self.log,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        except BaseException:
            self.log.close()
            raise

    def isOpened(self):
        return self.process is not None and not self.ended

    def get(self, prop):
        return {cv2.CAP_PROP_FPS: self.fps, cv2.CAP_PROP_FRAME_COUNT: self.frame_count,
                cv2.CAP_PROP_FRAME_WIDTH: self.width, cv2.CAP_PROP_FRAME_HEIGHT: self.height}.get(prop, 0)

    def read(self):
        if not self.isOpened():
            return False, None
        chunks, remaining = [], self.frame_bytes
        while remaining:
            chunk = self.process.stdout.read(remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        if remaining:
            self.ended = True
            code = self.process.wait(timeout=10)
            if chunks or code:
                self.log.seek(0, 2)
                self.log.seek(max(0, self.log.tell()-4096))
                detail = self.log.read().decode('utf-8', errors='replace')
                raise RuntimeError(f'FFmpeg frame decoding failed (exit {code}): {detail}')
            return False, None
        frame = np.frombuffer(b''.join(chunks), np.uint8).reshape(self.height, self.width, 3).copy()
        return True, frame

    def release(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=10)
        finally:
            self.process.stdout.close()
            self.log.close()
            self.process = None
            self.ended = True
