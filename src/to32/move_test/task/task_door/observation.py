"""任务的门观测接口：明确区分新无门帧、重复帧与数据故障。"""
import json
import os
import uuid

from .config import CONFIG


class DoorVisionIF:
    def __init__(self, shm_dir, cfg=CONFIG):
        self.cfg = cfg
        self.path = os.path.join(shm_dir, 'momo_det_front.json')
        self.reset_path = os.path.join(shm_dir, 'momo_door_reset.json')
        self.last_frame = None
        self.token = None

    def reset_target(self):
        self.token = uuid.uuid4().hex
        tmp = self.reset_path + '.' + self.token + '.tmp'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump({'token': self.token}, f)
            os.replace(tmp, self.reset_path)
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)
        self.last_frame = None

    def poll(self, now):
        try:
            with open(self.path, encoding='utf-8') as f:
                record = json.load(f)
            timestamp = float(record['capture_ts'])
            obs = record['door']
            frame = record['frame']
            if (record.get('stage') != 'PassGate' or record.get('status') != 'done'
                    or not isinstance(obs, dict) or not obs.get('valid')
                    or obs.get('reset_token') != self.token
                    or not 0 <= now-timestamp <= self.cfg.vision_stale_s):
                return {'valid': False, 'fresh': False, 'reason': 'stale-or-not-ready'}
            if (obs.get('img_w'), obs.get('img_h')) != (self.cfg.image_width, self.cfg.image_height):
                return {'valid': False, 'fresh': False, 'reason': 'wrong-image-size'}
            fresh = frame != self.last_frame
            self.last_frame = frame
            return dict(obs, frame=frame, capture_ts=timestamp, fresh=fresh, valid=True)
        except (OSError, ValueError, TypeError, KeyError):
            return {'valid': False, 'fresh': False, 'reason': 'unavailable'}
