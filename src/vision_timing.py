"""逐处理帧输出 JSON 耗时；run.sh 将 stdout 重定向到 logs/front.log。"""
import json
import sys
import threading

_LOG_LOCK = threading.Lock()


def log_vision_timing(frame_id, stage, status, *, timing_ms=None, yolo_timing_ms=None,
                      cv_profile=None, gpu=None, **details):
    # 不调用 GPU finish/readback；保留设备事件时间与主机墙钟时间的原始分组。
    record = dict(event='frame', frame=int(frame_id), stage=stage, status=status,
                  timing_ms=timing_ms or {}, yolo_timing_ms=yolo_timing_ms or {},
                  cv_profile=cv_profile, gpu=gpu, **details)
    line = '[front][%s][Timing] ' % stage + json.dumps(record, ensure_ascii=False) + '\n'
    with _LOG_LOCK:
        sys.stdout.write(line)
        sys.stdout.flush()
