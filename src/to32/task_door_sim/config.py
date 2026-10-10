"""独立视觉参数，不读取穿门运动参数或任务表。"""
from dataclasses import dataclass
from pathlib import Path
import math


@dataclass(frozen=True)
class DoorSimConfig:
    camera_params_path: str = str(Path(__file__).resolve().parents[3] /
                                  'quad_cv_kit/camera_correction_params.json')
    camera_fit: str = 'center-crop'
    correction_enabled: bool = True
    correction_plane_distance_m: object = None
    cv_every_frames: int = 3  # 间隔帧仍按当前图像跟踪/验证；跟踪失败立即重新搜索。
    hold_seconds: float = .2
    roi_padding: float = .08
    min_score: float = .5
    vision_stale_s: float = .5
    jpeg_quality: int = 85
    cv_profile: bool = True  # 每帧分段计时；窗口统计及慢帧日志用来定位 CV 瓶颈。
    cv_backend: str = 'auto'  # Mali OpenCL 优先；不可用时明确记录 CPU 回退。
    gpu_device: str = 'Mali'
    cv_hough: str = 'opencl'
    cv_blur: str = 'pyramid'
    cv_quality: str = 'fast'
    cv_execution: str = 'resident'  # Mali 使用 quad_cv_kit v7；CPU 对照使用 hybrid。
    cv_search: str = 'adaptive'
    opencv_threads: int = 3

    def validate(self):
        limits = {'min_score': (0, 1),
                  'jpeg_quality': (1, 100)}
        for name, (low, high) in limits.items():
            value = getattr(self, name)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError('%s 必须在 [%s, %s]' % (name, low, high))
        if not isinstance(self.cv_every_frames, int) or self.cv_every_frames < 1:
            raise ValueError('cv_every_frames 必须为正整数')
        for name, choices in {'cv_backend': ('cpu', 'auto', 'opencl'),
                              'cv_hough': ('cpu', 'opencl'), 'cv_blur': ('exact', 'pyramid'),
                              'cv_quality': ('precise', 'fast'), 'cv_search': ('full', 'adaptive'),
                              'cv_execution': ('resident', 'hybrid')}.items():
            if getattr(self, name) not in choices:
                raise ValueError(name + ' 配置无效')
        if not isinstance(self.opencv_threads, int) or self.opencv_threads < 1:
            raise ValueError('opencv_threads 必须为正整数')
        for name in ('hold_seconds', 'roi_padding', 'vision_stale_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + ' 必须为非负有限值')
        return self


CONFIG = DoorSimConfig()
