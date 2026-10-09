"""参考 task_door：串行处理一帧，发布带标注 JPEG 与同帧 JSON。"""
import threading
import time

import cv2

from .config import CONFIG
from .perception import DoorSimFrameProcessor


class DoorSimFrontPipeline:
    def __init__(self, detector, cfg=CONFIG, fps=30):
        self.cfg = cfg.validate()
        self.processor = DoorSimFrameProcessor(detector, cfg, fps)
        self.lock = threading.Lock()
        self.last_frame = -1

    def process(self, frame_id, frame, captured_at, frame_writer, det_writer):
        with self.lock:
            if frame_id <= self.last_frame:
                return None
            self.last_frame = frame_id
            if self.cfg.vision_stale_s and time.time()-captured_at > self.cfg.vision_stale_s:
                return None
            display, detections, observation = self.processor.process(frame)
            ok, encoded = cv2.imencode('.jpg', display,
                                        [cv2.IMWRITE_JPEG_QUALITY, int(self.cfg.jpeg_quality)])
            if not ok:
                raise RuntimeError('DoorSim JPEG 编码失败')
            jpeg = encoded.tobytes()
            if frame_writer is not None:
                if len(jpeg) > frame_writer.max_jpeg:
                    raise ValueError('DoorSim JPEG 超过共享帧容量')
                frame_writer.width, frame_writer.height = display.shape[1], display.shape[0]
                frame_writer.write(jpeg)
            if det_writer is not None:
                # 使用独立字段/阶段，避免穿门控制端把视觉实验当作正式门观测。
                det_writer.write(dict(frame=frame_id, ts=time.time(), capture_ts=captured_at,
                                      stage='DoorSim', status='done', dets=detections,
                                      door_sim=observation))
            return display, detections, observation
