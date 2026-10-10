"""穿门唯一参数入口；单位写在字段名中，TODO 项必须实车测量。

70×50 cm 必须与 CV 四条管中心线交点对应的矩形尺寸一致。
速度是指定推力下的实际平移速度，不是推力值本身。
"""
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DoorConfig:
    gates_to_pass: int = 4  # 完成数量；单门调试设为1，不使用v2预设路线
    image_width: int = 640   # ★ 必须与前摄采集/标定一致（quad_cv_kit 前摄标定 = 640×480；
    image_height: int = 480  #   2026-10-10 前摄采集由 1280×720 改为 640×480）。不符时
                             #   perception 判观测无效、observation 直接丢弃该帧。
    gate_width_m: float = .70
    gate_height_m: float = .50
    area_near: float = .40
    area_tolerance: float = .03  # 停靠屏占比37%～43%，仅此处开始四点控制
    center_tolerance_px: float = 8.
    stable_frames: int = 5
    filter_alpha: float = .5
    pause_error_px: float = 40.

    # YOLO 和 v7 resident CV 共用干净校正图，不做锐化/对比度增强。
    camera_params_path: str = str(Path(__file__).resolve().parents[5] /
                                  'quad_cv_kit/camera_correction_params.json')
    camera_fit: str = 'center-crop'  # TODO: 核对前视采集裁剪模式及水下内参
    correction_plane_distance_m: object = None  # 未知距离时沿用库的远场近似
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
    observe_frames: int = 5

    # 回传数据面板：小字号、紧凑行距，仅覆盖左下角文字区域。
    overlay_font_scale: float = .42
    overlay_line_height_px: int = 18
    overlay_background_alpha: float = .55

    # TODO: 相机 -> 机器人，机器人坐标采用 X右/Y下/Z前。
    camera_to_robot_rotation: tuple = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    camera_position_robot_m: tuple = (0., 0., 0.)
    yaw_sign: float = 1.  # 任务系右转为正；固件镜像仍只在输出壳施加
    depth_sign: float = 1.  # 图像向下 -> 增加水面以下深度
    sway_sign: float = 1.  # 任务系向右为正
    initial_depth_cm: float = 26.  # TODO: 入段无深度遥测时的保持目标
    min_depth_cm: float = 25.  # TODO: 可运行深度范围（需结合机体及池深）
    max_depth_cm: float = 86.
    yaw_step_max_deg: float = 3.  # TODO: 控制步长、增益及容差
    yaw_gain: float = .5
    yaw_tolerance_deg: float = 3.
    normal_yaw_tolerance_deg: float = .3  # CV法线容差需兼容8px中心要求
    yaw_command_tolerance_deg: float = .3
    depth_step_max_cm: float = 2.
    depth_tolerance_cm: float = 1.  # 视觉2cm小步调深的到位容差
    depth_hold_tolerance_cm: float = 8.  # 已确认定深的保持带；小步到位仍用1cm
    depth_hold_samples: int = 5  # 连续不同卡尔曼样本到位，重复读取不计数
    depth_velocity_max_mps: float = .02
    translation_tolerance_m: float = .01
    unactuated_tilt_max_deg: float = 12.
    vertical_gain: float = .5
    yolo_range_max_m: float = 3.
    sway_step_max_m: float = .03
    motion_timeout_s: float = 6.
    settle_s: float = .3
    align_timeout_s: float = 30.
    approach_timeout_s: float = 60.
    target_episode_timeout_s: float = 180.

    # TODO: 分别测量各档推力的速度；暂为估算，距离执行采用定时开环。
    approach_surge: float = .35
    blind_surge: float = .5
    sway_thrust: float = .35
    blind_speed_mps: float = .25
    sway_speed_mps: float = .15
    blind_distance_m: float = .80  # 固定距离模式的距离
    blind_use_pose_distance: bool = True
    blind_clearance_m: float = .80  # 超过门平面的余量，需按机体实测
    blind_max_distance_m: float = 4.  # 距离异常时保持，不静默截短盲冲
    blind_extra_s: float = 0.  # TODO: 从静止加速、惯性等实测补偿
    probe_step_m: float = .03
    probe_max_travel_m: float = .18
    probe_max_steps: int = 6
    probe_max_s: float = 15.
    width_trend_min_fraction: float = .03
    search_angle_deg: float = 45.
    search_step_deg: float = 2.  # 下发目标每次最多推进2°，不等待中间小角到位
    search_step_interval_s: float = .5  # 小步间最短间隔，限制目标推进速度
    search_yaw_tolerance_deg: float = 3.  # 扫视到位口径沿用 door_v2/turn_step
    search_hold_frames: int = 10  # 到最终观察角后连续到位；不用于2°中间目标
    search_observe_s: float = .6
    search_timeout_s: float = 90.  # 慢速扫视左45°、右45°、回基准需要更长时间

    def validate(self):
        import math
        if not isinstance(self.gates_to_pass, int) or self.gates_to_pass < 1:
            raise ValueError('gates_to_pass 必须为正整数')
        if not isinstance(self.blind_use_pose_distance, bool):
            raise ValueError('blind_use_pose_distance 必须为布尔值')
        if not 0 < self.area_tolerance < min(self.area_near, 1-self.area_near):
            raise ValueError('屏占比目标和容差无效')
        positive = ('image_width', 'image_height', 'gate_width_m', 'gate_height_m',
                    'center_tolerance_px', 'stable_frames', 'observe_frames',
                    'blind_speed_mps', 'sway_speed_mps',
                    'blind_distance_m', 'blind_clearance_m', 'blind_max_distance_m', 'probe_step_m',
                    'probe_max_steps', 'vision_stale_s', 'motion_timeout_s',
                    'align_timeout_s', 'search_timeout_s', 'cv_every_frames',
                    'normal_yaw_tolerance_deg', 'approach_timeout_s', 'target_episode_timeout_s',
                    'depth_tolerance_cm', 'depth_hold_tolerance_cm', 'depth_hold_samples', 'depth_velocity_max_mps',
                    'search_yaw_tolerance_deg', 'search_hold_frames',
                    'search_angle_deg', 'search_step_deg', 'search_step_interval_s')
        if any(not math.isfinite(float(getattr(self, k))) or getattr(self, k) <= 0
               for k in positive):
            raise ValueError('尺寸、计数、速度、距离和超时必须为正有限值')
        if not 0 < self.filter_alpha <= 1 or not 0 < self.min_depth_cm < self.max_depth_cm:
            raise ValueError('滤波系数或深度限值无效')
        if not isinstance(self.depth_hold_samples, int) or self.depth_hold_samples < 1:
            raise ValueError('depth_hold_samples 必须为正整数')
        if self.depth_hold_tolerance_cm < self.depth_tolerance_cm:
            raise ValueError('定深保持带不能小于到位容差')
        if (not isinstance(self.search_hold_frames, int) or self.search_hold_frames < 1
                or self.search_yaw_tolerance_deg >= self.search_angle_deg):
            raise ValueError('扫视到位计数或航向容差无效')
        if (not math.isfinite(self.blind_extra_s) or self.blind_extra_s < 0
                or self.blind_distance_m > self.blind_max_distance_m):
            raise ValueError('盲冲补偿时间或固定距离无效')
        for name, choices in {'cv_backend': ('auto', 'opencl', 'cpu'),
                              'cv_blur': ('pyramid', 'exact'),
                              'cv_quality': ('fast', 'precise')}.items():
            if getattr(self, name) not in choices:
                raise ValueError(name + ' 配置无效')
        if not isinstance(self.opencv_threads, int) or self.opencv_threads <= 0:
            raise ValueError('opencv_threads 必须为正整数')
        if (not math.isfinite(self.overlay_font_scale) or self.overlay_font_scale <= 0
                or not math.isfinite(self.overlay_line_height_px) or self.overlay_line_height_px <= 0
                or not 0 <= self.overlay_background_alpha <= 1):
            raise ValueError('回传字号、行距或背景透明度无效')
        for k in ('approach_surge', 'blind_surge', 'sway_thrust'):
            if not 0 < getattr(self, k) <= 1:
                raise ValueError(k + ' 必须在 (0,1]')
        return self


CONFIG = DoorConfig()
