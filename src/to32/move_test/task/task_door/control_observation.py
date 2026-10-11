"""从一条前视观测读取 cx/cy 和 CV Z 角；不运行模型、不写视觉阶段。"""
import json
import math
from pathlib import Path

from .config import CONFIG


class GateControlObservation:
    def __init__(self, shm_dir, cfg=CONFIG):
        self.path = Path(shm_dir)/'momo_det_front.json'
        self.cfg = cfg
        self.last_frame = None

    def reset(self):
        self.last_frame = None

    def read(self, now):
        try:
            record = json.loads(self.path.read_text(encoding='utf-8'))
            obs = record['door']
            age = float(now)-float(record['capture_ts'])
            frame = record['frame']
            if (record.get('stage') != 'PassGate' or record.get('status') != 'done'
                    or not obs.get('valid') or not obs.get('has_target')
                    or obs.get('coordinate_space') != 'corrected'
                    or not math.isfinite(age) or not 0 <= age <= self.cfg.vision_stale_s
                    or (obs.get('img_w'), obs.get('img_h')) !=
                       (self.cfg.image_width, self.cfg.image_height)
                    or not isinstance(frame, int) or isinstance(frame, bool)):
                return None
            cx = float(obs['center_px'][0])
            cy = float(obs['center_px'][1])
            if (not math.isfinite(cx) or not 0 <= cx <= self.cfg.image_width
                    or not math.isfinite(cy) or not 0 <= cy <= self.cfg.image_height):
                return None
            z_deg = None
            # Z is optional: cx control does not require a complete CV pose.
            try:
                angles = obs['guidance']['alignment']['rotation_xyz_deg']
                value = float(angles[2])
                if len(angles) == 3 and math.isfinite(value):
                    z_deg = value
            except (KeyError, IndexError, TypeError, ValueError, OverflowError):
                pass
            fresh = frame != self.last_frame
            self.last_frame = frame
            return dict(frame=frame, target_id=obs.get('target_id'), capture_ts=record['capture_ts'],
                        cx=cx, cy=cy, z_deg=z_deg, fresh=fresh)
        except (OSError, KeyError, IndexError, AttributeError, TypeError, ValueError, OverflowError):
            return None
