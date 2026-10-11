"""前视接线：共享跟踪器串行处理，发布同帧视觉结果和 JPEG。"""
import threading
import time

from .perception import DoorFrameProcessor
from .config import CONFIG
from .overlay import draw_door_overlay
from stage_model import StageDetector
from vision_timing import log_vision_timing


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
            started = time.perf_counter()
            if self.processor is None:
                self.processor = DoorFrameProcessor()
            fixed, dets, observation = self.processor.process(frame, detector, time.time())
            timings = dict(observation.get('timing_ms', {}))
            log_details = dict(yolo_timing_ms=observation.get('yolo_timing_ms'),
                               cv_profile=observation.get('cv_profile'), gpu=observation.get('gpu'),
                               cv_mode=observation.get('cv_mode'), yolo_count=len(dets))
            if not detector.is_current() or detector.stage != 'PassGate':
                log_vision_timing(frame_id, 'PassGate', 'discarded-stage-changed',
                                  timing_ms=timings, **log_details)
                return None
            if not detector.ready:
                observation = dict(valid=False, has_target=False, reason='model-not-ready')
            timings.update(overlay=0.0, jpeg_encode=0.0, frame_write=0.0, results_write=0.0)
            if frame_writer is not None:
                overlay_started = time.perf_counter()
                display = draw_door_overlay(fixed, dets, observation)
                encoding_started = time.perf_counter()
                timings['overlay'] = (encoding_started-overlay_started)*1000
                ok, encoded = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, quality])
                write_started = time.perf_counter()
                timings['jpeg_encode'] = (write_started-encoding_started)*1000
                if ok:
                    frame_writer.width, frame_writer.height = display.shape[1], display.shape[0]
                    frame_writer.write(encoded.tobytes())
                    timings['frame_write'] = (time.perf_counter()-write_started)*1000
            if det_writer is not None:
                results_started = time.perf_counter()
                det_writer.write(dict(frame=frame_id, ts=time.time(), capture_ts=captured_at,
                                      img_w=fixed.shape[1], img_h=fixed.shape[0],
                                      dets=dets, stage='PassGate', status='done' if detector.ready else 'model_error',
                                      model_path=detector.cfg['model_path'], door=observation))
                timings['results_write'] = (time.perf_counter()-results_started)*1000
            timings['pipeline_total'] = (time.perf_counter()-started)*1000
            log_vision_timing(frame_id, 'PassGate', 'done' if detector.ready else 'model_error',
                              timing_ms=timings, capture_to_publish_ms=(time.time()-captured_at)*1000,
                              **log_details)
            return fixed, dets

    def close(self):
        with self.lock:
            if self.processor is not None:
                self.processor.close()
