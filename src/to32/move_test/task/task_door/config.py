"""门视觉参数：相机校正、YOLO/CV 和回传显示，不包含运动控制。"""
from dataclasses import dataclass
from pathlib import Path
import math


@dataclass(frozen=True)
class DoorConfig:
    image_width: int = 640
    image_height: int = 480
    gate_width_m: float = .70  # 姿态显示的门框参考尺寸
    gate_height_m: float = .50
    camera_params_path: str = str(Path(__file__).resolve().parents[5] /
                                  'quad_cv_kit/camera_correction_params.json')
    camera_fit: str = 'center-crop'
    correction_plane_distance_m: object = None
    cv_backend: str = 'auto'
    gpu_device: str = 'Mali'
    cv_blur: str = 'pyramid'
    cv_quality: str = 'fast'
    opencv_threads: int = 3
    cv_every_frames: int = 1
    roi_padding: float = .08
    min_score: float = .5
    boundary_margin_px: float = 4.
    target_iou_min: float = .15
    target_area_ratio_min: float = .4
    target_lost_s: float = 1.5
    vision_stale_s: float = .5
    overlay_font_scale: float = .42
    overlay_line_height_px: int = 18
    overlay_background_alpha: float = .55

    def validate(self):
        for name in ('image_width', 'image_height', 'opencv_threads', 'cv_every_frames',
                     'overlay_line_height_px'):
            value = getattr(self, name)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(name + ' 必须为正整数')
        for name in ('gate_width_m', 'gate_height_m', 'target_lost_s',
                     'vision_stale_s', 'overlay_font_scale'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' 必须为正有限值')
        for name in ('min_score', 'target_iou_min', 'target_area_ratio_min',
                     'overlay_background_alpha'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(name + ' 必须在 [0,1]')
        for name in ('roi_padding', 'boundary_margin_px'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(name + ' 必须为非负有限值')
        for name, choices in {'cv_backend': ('auto', 'opencl', 'cpu'),
                              'cv_blur': ('pyramid', 'exact'),
                              'cv_quality': ('fast', 'precise')}.items():
            if getattr(self, name) not in choices:
                raise ValueError(name + ' 配置无效')
        return self


CONFIG = DoorConfig()
