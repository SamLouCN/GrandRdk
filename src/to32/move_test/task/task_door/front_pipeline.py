"""前视接线：共享跟踪器串行处理，发布同帧视觉结果和 JPEG。"""
import threading
import time

from .perception import DoorFrameProcessor
from .config import CONFIG
from .overlay import draw_door_overlay
from stage_model import StageDetector


class DoorDetector(StageDetector):
    """纯视觉模式固定使用门模型，不读取或写入运动任务阶段。"""

    def _stage(self):
        return 'PassGate'


class DoorFrontPipeline:
    def __init__(self):
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
                return None  # 排空积压帧，避免回传延迟不断增长
            if self.processor is None:
                self.processor = DoorFrameProcessor()
            fixed, dets, observation = self.processor.process(frame, detector, time.time())
            if not detector.is_current() or detector.stage != 'PassGate':
                return None
            if not detector.ready:
                observation = dict(valid=False, has_target=False, reason='model-not-ready')
            if frame_writer is not None:
                display = draw_door_overlay(fixed, dets, observation)
                ok, encoded = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if ok:
                    frame_writer.width, frame_writer.height = display.shape[1], display.shape[0]
                    frame_writer.write(encoded.tobytes())
            if det_writer is not None:
                det_writer.write(dict(frame=frame_id, ts=time.time(), capture_ts=captured_at,
                                      img_w=fixed.shape[1], img_h=fixed.shape[0],
                                      dets=dets, stage='PassGate', status='done' if detector.ready else 'model_error',
                                      model_path=detector.cfg['model_path'], door=observation))
            return fixed, dets

    def close(self):
        with self.lock:
            if self.processor is not None:
                self.processor.close()
