"""前视接线：共享跟踪器串行处理，发布同帧检测和姿态，不依赖 JPEG 序号。"""
import json
import os
import threading
import time

from .perception import DoorFrameProcessor
from .config import CONFIG


class DoorFrontPipeline:
    def __init__(self, shm_dir):
        self.reset_path = os.path.join(shm_dir, 'momo_door_reset.json')
        self.lock = threading.Lock()
        self.processor = None
        self.last_frame = -1

    def process(self, frame_id, frame, detector, captured_at, frame_writer, det_writer, quality):
        import cv2
        with self.lock:
            if frame_id <= self.last_frame:
                return None
            self.last_frame = frame_id
            if time.time()-captured_at > CONFIG.vision_stale_s:
                return None  # 排空积压帧，避免花时间推理已无法控制的旧图
            if self.processor is None:
                self.processor = DoorFrameProcessor()
            try:
                with open(self.reset_path, encoding='utf-8') as f:
                    token = json.load(f).get('token')
            except (OSError, ValueError, TypeError):
                token = None
            fixed, dets, observation = self.processor.process(frame, detector, time.time(), token)
            if not detector.is_current() or detector.stage != 'PassGate':
                return None
            if not detector.ready:
                observation = dict(valid=False, has_target=False, reason='model-not-ready', reset_token=token)
            if frame_writer is not None:
                ok, encoded = cv2.imencode('.jpg', fixed, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if ok:
                    frame_writer.write(encoded.tobytes())
            if det_writer is not None:
                det_writer.write(dict(frame=frame_id, ts=time.time(), capture_ts=captured_at,
                                      dets=dets, stage='PassGate', status='done' if detector.ready else 'model_error',
                                      model_path=detector.cfg['model_path'], door=observation))
            return fixed, dets
