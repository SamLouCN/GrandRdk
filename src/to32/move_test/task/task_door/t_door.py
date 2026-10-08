"""PassGate 状态机：只生成任务指令，不推理、不直接写串口。

ACQUIRE -> YOLO_ALIGN -> APPROACH_50 -> CV_ALIGN -> APPROACH_80
        -> CV_FINAL -> BLIND -> ACQUIRE（下一门）
无门：SEARCH_TURN/SEARCH_OBSERVE -> RETURN_HEADING -> EXIT -> DONE。
四点缺失：PROBE_MOVE/PROBE_OBSERVE，有限小步探索；不可用时 HOLD_FAULT。
"""
import math
import os
import statistics
import sys
from pathlib import Path

_MV = str(Path(__file__).resolve().parents[2])
if _MV not in sys.path:
    sys.path.insert(0, _MV)
from stage_base import Stage
from .config import CONFIG
from .observation import DoorVisionIF


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, actual):
    return (target-actual+180) % 360-180


class DoorTask(Stage):
    NAME = 'PassGate'  # 已对应 stage_model 中 gate 模型

    def __init__(self, ctx, cfg=CONFIG, vision=None):
        super().__init__(ctx)
        self.cfg = cfg.validate()
        shm_dir = getattr(ctx.cfg, 'AUV_SHM_DIR', os.environ.get('GRDK_SHM_DIR', '/dev/shm'))
        self.vision = vision or DoorVisionIF(shm_dir, cfg)

    def enter(self, now):
        self.phase = 'ACQUIRE'
        self.phase_start = now
        self.yaw = None
        self.depth = self.cfg.initial_depth_cm
        self.entry_yaw = None
        self.target_id = None
        self.motion = None
        self.settle_until = now
        self.counts = {}
        self.filtered_center = None
        self.filtered_frame = None
        self.episode_started = now
        self.empty_since = None
        self.empty_frames = 0
        self.gates_passed = 0
        self.drive_started = None
        self.vision.reset_target()
        self.ctx.say('穿门启动：70×50cm，面积阈值50%/80%，中心容差8px；距离采用标定速度定时执行')

    def _go(self, phase, now):
        self.phase, self.phase_start = phase, now
        self.drive_started = None
        self.counts = {}
        self.ctx.say('PassGate -> ' + phase)

    def _command(self, note='', surge=0., sway=0.):
        from task import t_function as TF
        return TF._cmd(self.NAME, self.phase + ': ' + note, self.yaw, self.depth, surge, sway)

    def _fault(self, now, reason):
        self.fault_reason = reason
        self.motion = None
        self._go('HOLD_FAULT', now)
        return self._command(reason)

    def _count(self, key, condition, obs):
        if obs.get('fresh'):
            self.counts[key] = self.counts.get(key, 0)+1 if condition else 0
        return self.counts.get(key, 0) >= self.cfg.stable_frames

    def _start_motion(self, kind, now, value):
        self.motion = dict(kind=kind, start=now)
        if kind == 'yaw':
            self.yaw = (self.actual_yaw+value+180) % 360-180
        elif kind == 'depth':
            self.depth = clamp(self.actual_depth+value, self.cfg.min_depth_cm, self.cfg.max_depth_cm)
        else:
            self.motion.update(duration=abs(value)/self.cfg.sway_speed_mps,
                               sway=math.copysign(self.cfg.sway_thrust, value)*self.cfg.sway_sign)
        return self._motion_tick(now)

    def _motion_tick(self, now):
        m, cfg = self.motion, self.cfg
        elapsed = now-m['start']
        if elapsed > cfg.motion_timeout_s:
            return self._fault(now, '动作到位超时，请检查遥测／标定')
        if m['kind'] == 'yaw':
            done = abs(angle_error(self.yaw, self.actual_yaw)) <= cfg.yaw_command_tolerance_deg
        elif m['kind'] == 'depth':
            done = abs(self.depth-self.actual_depth) <= cfg.depth_tolerance_cm
        else:
            done = elapsed >= m['duration']
        if done:
            self.motion = None
            self.settle_until = now+cfg.settle_s
            return self._command('动作完成，等待稳定后的新观测')
        return self._command('分步调整 ' + m['kind'], sway=m.get('sway', 0.))

    def _center_error(self, obs):
        center = obs['center_px']
        if obs['fresh'] and obs.get('frame') != self.filtered_frame:
            if self.filtered_center is None:
                self.filtered_center = list(center)
            else:
                a = self.cfg.filter_alpha
                self.filtered_center = [a*v+(1-a)*p for v, p in zip(center, self.filtered_center)]
            self.filtered_frame = obs.get('frame')
        center = self.filtered_center or center
        return center[0]-obs['aim_px'][0], center[1]-obs['aim_px'][1]

    def _yolo_adjust(self, obs, now):
        cfg = self.cfg
        dx, dy = self._center_error(obs)
        if obs.get('clipped'):
            sides = obs['boundary_sides']
            horizontal = int('right' in sides)-int('left' in sides)
            vertical = int('down' in sides)-int('up' in sides)
            if not horizontal and not vertical:
                return self._command('多侧裁切，无法由框中心判断方向')
            dx, dy = horizontal*cfg.pause_error_px, vertical*cfg.pause_error_px
        if not obs['fresh']:
            return self._command('等待新帧纠偏')
        if abs(dx) > cfg.center_tolerance_px:
            delta = math.degrees(math.atan2(dx, obs['focal_px'][0]))*cfg.yaw_gain*cfg.yaw_sign
            return self._start_motion('yaw', now, clamp(delta, -cfg.yaw_step_max_deg, cfg.yaw_step_max_deg))
        if abs(dy) > cfg.center_tolerance_px:
            z = min(cfg.yolo_range_max_m, obs['focal_px'][1]*cfg.gate_height_m/max(obs['height_px'], 1))
            delta = dy/obs['focal_px'][1]*z*100*cfg.vertical_gain*cfg.depth_sign
            delta = clamp(delta, -cfg.depth_step_max_cm, cfg.depth_step_max_cm)
            if clamp(self.actual_depth+delta, cfg.min_depth_cm, cfg.max_depth_cm) == self.actual_depth:
                return self._fault(now, '目标超出深度可调整范围')
            return self._start_motion('depth', now, delta)
        return None

    def _pose_good(self, obs):
        return (obs.get('pose') is not None and obs['pose'].get('controllable')
                and len(obs.get('corners') or []) == 4
                and (obs.get('geometry') or {}).get('observation') == 'detected')

    def _pose_aligned(self, obs):
        if not self._pose_good(obs):
            return False
        p, cfg = obs['pose'], self.cfg
        x, y, _ = p['alignment_robot_m']
        return (abs(p['yaw_error_deg']) <= cfg.normal_yaw_tolerance_deg
                and max(abs(x), abs(y)) <= cfg.translation_tolerance_m
                and max(abs(v) for v in p['center_offset_robot_px']) <= cfg.center_tolerance_px)

    def _cv_adjust(self, obs, now):
        p, cfg = obs['pose'], self.cfg
        if not obs['fresh']:
            return self._command('等待四点新观测')
        yaw = p['yaw_error_deg']
        if abs(yaw) > cfg.normal_yaw_tolerance_deg:
            return self._start_motion('yaw', now, clamp(yaw*cfg.yaw_gain*cfg.yaw_sign,
                                                       -cfg.yaw_step_max_deg, cfg.yaw_step_max_deg))
        x, y, _ = p['alignment_robot_m']
        # 航向接近法线后，使用门中心相对机器人位置修正，避免几何容差允许偏航残差时卡住。
        cx, cy, _ = p['center_robot_m']
        image_dx, image_dy = p['center_offset_robot_px']
        if abs(y) > cfg.translation_tolerance_m or abs(image_dy) > cfg.center_tolerance_px:
            delta = (y if abs(y) > cfg.translation_tolerance_m else cy)*100*cfg.depth_sign
            delta = clamp(delta, -cfg.depth_step_max_cm, cfg.depth_step_max_cm)
            target = clamp(self.actual_depth+delta, cfg.min_depth_cm, cfg.max_depth_cm)
            if abs(target-self.actual_depth) < 1e-6:
                return self._fault(now, 'CV 对准需要的深度超出限值')
            return self._start_motion('depth', now, delta)
        if abs(x) > cfg.translation_tolerance_m or abs(image_dx) > cfg.center_tolerance_px:
            step = x if abs(x) > cfg.translation_tolerance_m else cx
            return self._start_motion('sway', now, clamp(step, -cfg.sway_step_max_m, cfg.sway_step_max_m))
        return self._command('等待四点稳定')

    def _start_probe(self, obs, now, return_phase):
        self.probe_return = return_phase
        self.probe_started = now
        self.probe_steps = 0
        self.probe_travel = 0.
        self.probe_direction = -1.
        self.probe_reference_width = obs['width_px']
        self.probe_reference_quality = len((obs.get('geometry') or {}).get('observed_segments', []))
        self.probe_samples = []
        self._go('PROBE_MOVE', now)
        return self._command('四点不可用，开始有限左移试探')

    def _start_search(self, now):
        self.search_base = self.actual_yaw
        self.search_started = now
        self.search_index = 0
        self.search_angles = [-self.cfg.search_angle_deg, self.cfg.search_angle_deg]
        self.vision.reset_target()
        self.target_id = None
        self.filtered_center = None
        self._go('SEARCH_TURN', now)
        return self._command('无门，左右各45°扫视')

    def step(self, now, dt):
        cfg = self.cfg
        if self.phase == 'DONE':
            return None
        if self.phase == 'HOLD_FAULT':
            return self._command(self.fault_reason)
        # 盲冲及赛段尾部直行不消费视觉，不修改已锁定目标；急停仍由外层模式管理。
        if self.phase in ('BLIND', 'EXIT'):
            if self.drive_started is None:
                self.drive_started = now  # 从首个实际前进指令开始计时，排除阶段切换停推拍
            duration = (cfg.blind_distance_m/cfg.blind_speed_mps+cfg.blind_extra_s
                        if self.phase == 'BLIND' else cfg.exit_distance_m/cfg.exit_speed_mps)
            if now-self.drive_started < duration:
                return self._command('定时开环直行', surge=cfg.blind_surge if self.phase == 'BLIND' else cfg.exit_surge)
            if self.phase == 'EXIT':
                self._go('DONE', now)
                return self._command('门赛段结束，交接下一阶段')
            self.gates_passed += 1
            self.vision.reset_target()
            self.target_id, self.filtered_center = None, None
            self.empty_since, self.empty_frames = None, 0
            self.motion = None
            self._go('ACQUIRE', now)
            return self._command('盲冲完成，重新捕获下一门')

        tel = self.ctx.tel or {}
        try:
            self.actual_yaw = float(tel['actual_yaw'])
            self.actual_depth = float(tel['actual_depth_cm'])
            if not all(math.isfinite(v) for v in (self.actual_yaw, self.actual_depth)):
                raise ValueError('非有限遥测')
        except (KeyError, TypeError, ValueError):
            from task import t_function as TF
            return self._command('等待航向／深度遥测') if self.yaw is not None else TF.wait_cmd(self.NAME)
        if self.yaw is None:
            self.yaw, self.entry_yaw = self.actual_yaw, self.actual_yaw
            self.depth = clamp(self.actual_depth, cfg.min_depth_cm, cfg.max_depth_cm)
        obs = self.vision.poll(now)
        if self.motion is not None:
            if self.motion['kind'] == 'sway' and (not obs.get('valid') or not obs.get('has_target')):
                self.motion = None
                self.counts = {}
                self.settle_until = now+cfg.settle_s
                return self._command('横移中观测失效，停止横移')
            return self._motion_tick(now)
        if now < self.settle_until:
            return self._command('等待运动稳定')
        if not obs.get('valid'):
            self.counts = {}
            self.empty_since, self.empty_frames = None, 0
            return self._command('视觉未就绪／过期，保持零水平推力')
        if obs.get('capture_ts', now) < self.settle_until:
            return self._command('等待运动后采集的新帧')

        # 搜索转向：只有在目标角到位后的有效新帧，才开始计无门观察。
        if self.phase.startswith('SEARCH') or self.phase == 'RETURN_HEADING':
            if now-self.search_started > cfg.search_timeout_s:
                return self._fault(now, '扫视未完成有效观察／航向到位超时')
            if obs.get('has_target') and obs.get('fresh'):
                self.target_id, self.filtered_center = obs['target_id'], None
                self.filtered_frame, self.episode_started = None, now
                self._go('YOLO_ALIGN', now)
                return self._command('扫视发现门')
            if self.phase in ('SEARCH_TURN', 'RETURN_HEADING'):
                target = (self.search_base+self.search_angles[self.search_index]
                          if self.phase == 'SEARCH_TURN' else self.search_base)
                self.yaw = (target+180) % 360-180
                if abs(angle_error(self.yaw, self.actual_yaw)) <= cfg.yaw_tolerance_deg:
                    if self.phase == 'RETURN_HEADING':
                        self._go('EXIT', now)
                    else:
                        self.search_valid_frames = 0
                        self._go('SEARCH_OBSERVE', now)
                        self.settle_until = now+cfg.settle_s
                return self._command('转向搜索基准角／观察角')
            if obs['fresh']:
                self.search_valid_frames += 1
            if (now-self.phase_start >= cfg.search_observe_s
                    and self.search_valid_frames >= cfg.observe_frames):
                self.search_index += 1
                self._go('SEARCH_TURN' if self.search_index < len(self.search_angles) else 'RETURN_HEADING', now)
            return self._command('观察有效无门帧')

        if not obs.get('has_target'):
            self.counts = {}
            if obs['fresh']:
                if self.empty_since is None:
                    self.empty_since = now
                self.empty_frames += 1
            if (self.empty_since is not None and now-self.empty_since >= cfg.target_lost_s
                    and self.empty_frames >= cfg.observe_frames):
                return self._start_search(now)
            return self._command('目标暂失，等待有效无门帧确认')
        self.empty_since, self.empty_frames = None, 0
        if obs['target_id'] != self.target_id:
            self.target_id, self.filtered_center = obs['target_id'], None
            self.filtered_frame, self.episode_started = None, now
            self._go('YOLO_ALIGN', now)
        if self.phase == 'ACQUIRE':
            self._go('YOLO_ALIGN', now)
        if now-self.episode_started > cfg.target_episode_timeout_s:
            return self._fault(now, '当前门未完成对准／穿越，达到总时限')

        if self.phase in ('YOLO_ALIGN', 'APPROACH_50'):
            dx, dy = self._center_error(obs)
            centered = not obs.get('clipped') and max(abs(dx), abs(dy)) <= cfg.center_tolerance_px
            stable = self._count('center', centered, obs)
            if self.phase == 'YOLO_ALIGN':
                if stable:
                    self._go('APPROACH_50', now)
                else:
                    action = self._yolo_adjust(obs, now)
                    return action or self._command('中心8px范围内，等待稳定')
            if obs['area_ratio'] >= cfg.area_near:
                if self._count('area', True, obs):
                    self._go('CV_ALIGN', now)
                return self._command('已到50%附近，停止接近并验证')
            self._count('area', False, obs)
            if obs.get('clipped') or max(abs(dx), abs(dy)) > cfg.pause_error_px:
                self._go('YOLO_ALIGN', now)
                return self._command('偏差过大，暂停接近')
            action = self._yolo_adjust(obs, now)
            return action or self._command('接近50%，持续纠偏', surge=cfg.approach_surge)

        if self.phase in ('CV_ALIGN', 'CV_FINAL'):
            if now-self.phase_start > cfg.align_timeout_s:
                return self._fault(now, '四点回正超时，未满足盲冲条件')
            if not self._pose_good(obs):
                self.counts['pose'] = 0
                if self.phase == 'CV_ALIGN':
                    return self._start_probe(obs, now, 'CV_ALIGN')
                return self._command('最终四点无效／姿态不可控，禁止进入盲冲')
            if self._count('pose', self._pose_aligned(obs), obs):
                self._go('APPROACH_80' if self.phase == 'CV_ALIGN' else 'BLIND', now)
                return self._command('四点回正完成')
            return self._cv_adjust(obs, now)

        if self.phase == 'APPROACH_80':
            if now-self.phase_start > cfg.approach_timeout_s:
                return self._fault(now, '80%面积或完整四点未同时满足，请核对视场／门尺寸／阈值')
            if not self._pose_good(obs):
                self.counts['area'] = 0
                return self._command('接近中四点失效，停止前进；完整几何恢复后继续')
            if not self._pose_aligned(obs):
                self.counts['area'] = 0
                return self._cv_adjust(obs, now)
            if obs['area_ratio'] >= cfg.area_commit:
                if self._count('area', True, obs):
                    self._go('CV_FINAL', now)
                return self._command('80%面积阈值，最终CV校准')
            self._count('area', False, obs)
            return self._command('接近80%，保持CV回正', surge=cfg.approach_surge)

        if self.phase.startswith('PROBE'):
            if self._count('probe_pose', self._pose_good(obs), obs):
                self._go(self.probe_return, now)
                return self._command('试探获取稳定四点')
            if now-self.probe_started > cfg.probe_max_s:
                return self._fault(now, '有限横移仍未获得四点，请检查视角／检测')
            if self.phase == 'PROBE_MOVE':
                if (self.probe_steps >= cfg.probe_max_steps
                        or self.probe_travel+cfg.probe_step_m > cfg.probe_max_travel_m):
                    return self._fault(now, '有限横移仍未获得四点，请检查视角／检测')
                self.probe_steps += 1
                self.probe_travel += cfg.probe_step_m
                self.probe_samples = []
                self._go('PROBE_OBSERVE', now)
                return self._start_motion('sway', now, self.probe_direction*cfg.probe_step_m)
            if obs['fresh'] and not obs.get('clipped'):
                quality = len((obs.get('geometry') or {}).get('observed_segments', []))
                self.probe_samples.append((obs['width_px'], quality))
            if len(self.probe_samples) >= cfg.observe_frames:
                width = statistics.median(v[0] for v in self.probe_samples)
                quality = statistics.median(v[1] for v in self.probe_samples)
                # 四边支持程度优先，框宽只在质量相同、变化超出噪声门槛时辅助。
                worse = (quality < self.probe_reference_quality or
                         (quality == self.probe_reference_quality and
                          width < self.probe_reference_width*(1-cfg.width_trend_min_fraction)))
                if worse:
                    self.probe_direction *= -1
                self.probe_reference_width, self.probe_reference_quality = width, quality
                self._go('PROBE_MOVE', now)
            return self._command('横移后观察四边质量与框宽趋势')
        return self._fault(now, '未知状态 ' + self.phase)


DOOR_TABLE = [DoorTask]
