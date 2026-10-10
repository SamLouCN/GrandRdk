"""PassGate 状态机：只生成任务指令，不推理、不直接写串口。

ACQUIRE -> SEARCH_TURN/SEARCH_OBSERVE -> YOLO_ALIGN -> APPROACH_40
        -> CV_ALIGN -> BLIND -> ACQUIRE（下一门）-> DONE。
无门：左右各45°扫视并回基准，继续扫描；不横移或自动直行。
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
    from task import t_function as TF
    return TF.yaw_err_deg(actual, target)


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
        self.fused_target_cm = None
        self.fused_depth_cm = None
        self.depth_sample_ts = None
        self.depth_last_counted_ts = None
        self.depth_ok_count = 0
        self.depth_ready = False
        self.depth_confirmed = False
        self.depth_valid = False
        self.blind_travel_m = self.cfg.blind_distance_m
        self.vision.reset_target()
        self.ctx.say('穿门启动：当前航向±45°扫视；YOLO中心8px；40%四点回正后盲冲')

    def _go(self, phase, now):
        self.phase, self.phase_start = phase, now
        self.drive_started = None
        self.counts = {}
        self.ctx.say('PassGate -> ' + phase)

    def _command(self, note='', surge=0., sway=0., require_depth=True):
        from task import t_function as TF
        self.depth = self._depth_target(self.depth)
        if require_depth and not self.depth_confirmed:
            surge, sway = 0., 0.
            note += '；等待深度计定深稳定'
        return TF._cmd(self.NAME, self.phase + ': ' + note, self.yaw, self.depth, surge, sway)

    def _read_depth(self, now):
        """深度判据：**固件深度计遥测 actual_depth_cm**（cm）—— 与下发 depth_cm 同帧，可直接相减。

        [2026-10-11] 深度卡尔曼已从板端移除：本函数原读融合 D（obs.DepthIF），现改读固件
        深度计遥测；随之取消"融合目标相对映射"（fused_target_cm 现在就是固件帧目标）、
        去掉 v_z 垂速门限（固件遥测不提供该量）。无遥测 → depth_confirmed 保持 False，
        调用方 _command(require_depth=True) 会清零推力等待（安全语义不变）。
        """
        from task import t_function as TF
        self.depth_valid = False
        a_d = TF._tel_f(self.ctx, 'actual_depth_cm')
        try:
            depth = float(a_d)
        except (TypeError, ValueError):
            depth = None
        if depth is None or depth != depth or depth < 0:   # 缺失 / NaN / 负值
            self.depth_ok_count, self.depth_ready = 0, False
            self.depth_confirmed = False
            return
        self.depth_valid = True
        self.fused_depth_cm, self.depth_sample_ts = depth, now
        in_band = (self.fused_target_cm is not None
                   and abs(depth - self.fused_target_cm) <= self.cfg.depth_tolerance_cm)
        if not in_band:
            self.depth_ok_count = 0
        else:
            self.depth_ok_count += 1
        self.depth_last_counted_ts = now
        self.depth_ready = self.depth_ok_count >= self.cfg.depth_hold_samples
        if self.depth_ready:
            self.depth_confirmed = True
        elif (self.fused_target_cm is not None
              and abs(depth - self.fused_target_cm) > self.cfg.depth_hold_tolerance_cm):
            self.depth_confirmed = False

    def _lock_depth_target(self, target):
        # [2026-10-11] 下发与判据现在都是**固件深度计帧** → 目标即自身，
        # 不再需要原来"融合 D 与固件绝对深度不相减"的相对映射（fused_target_cm 保留
        # 字段名以兼容旧引用，值就是固件帧目标）。
        self.depth = self._depth_target(target)
        self.fused_target_cm = self.depth
        self.depth_ok_count, self.depth_ready = 0, False
        self.depth_confirmed = False
        self.depth_last_counted_ts = None
        self.ctx.say('PassGate 定深：固件深度目标 %.1fcm（深度计帧直比）' % self.depth)

    def _depth_target(self, value):
        from task import t_function as TF
        # Already in firmware depth centimetres: do not convert it again as
        # bottom clearance. Honor both task limits and the shared pool limits.
        low = max(self.cfg.min_depth_cm, TF.clamp_depth_cm(float('-inf')))
        high = min(self.cfg.max_depth_cm, TF.clamp_depth_cm(float('inf')))
        if low > high:
            raise ValueError('穿门深度范围与 t_function 水面／池底限值不相交')
        return clamp(value, low, high)

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
            self._lock_depth_target(self.actual_depth+value)
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
            done = self.depth_ready
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
            if self._depth_target(self.actual_depth+delta) == self.actual_depth:
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
            target = self._depth_target(self.actual_depth+delta)
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

    def _start_search(self, now, base=None, reset=True):
        self.search_base = self.actual_yaw if base is None else base
        self.yaw = self.actual_yaw
        self.search_started = now
        self.search_index = 0
        self.search_angles = [-self.cfg.search_angle_deg, self.cfg.search_angle_deg]
        self.search_step_target = None
        self.search_turn_goal = None
        self.search_turn_state = {}
        self.search_ramp_finished_at = None
        self.search_next_step_at = now
        if reset:
            self.vision.reset_target()
        self.target_id = None
        self.filtered_center = self.filtered_frame = None
        self._go('SEARCH_TURN', now)
        left = (self.search_base-self.cfg.search_angle_deg+180) % 360-180
        right = (self.search_base+self.cfg.search_angle_deg+180) % 360-180
        self.ctx.say('PassGate 扫视基准 %.1f°；左目标 %.1f°；右目标 %.1f°；保持深度 %.1fcm'
                     % (self.search_base, left, right, self.depth))
        return self._command('以相对小步左右扫视', require_depth=False)

    def _search_turn(self, target, now, dt=0.):
        """复用v2的锁目标/转向原语；缓慢推进下发角，仅对最终观察角判到位。"""
        from task import t_function as TF
        cfg = self.cfg
        if self.search_turn_goal != target:
            self.search_turn_goal = target
            self.search_turn_state = {}
            self.search_step_target = self.actual_yaw
            self.search_next_step_at = now
            self.search_ramp_finished_at = None
            relative = angle_error(target, self.actual_yaw)
            TF.lock_turn_target(self.ctx, self.search_turn_state, now, relative, stage=self.NAME)
            # turn_step 的深度参数是距底高度；反算后沿用当前固件深度，避免误用默认26cm。
            body = float(getattr(TF.TC, 'AUV_BODY_HEIGHT_CM', TF.BODY_HEIGHT_CM))
            self.search_turn_state['last_height_cm'] = float(TF.TC.AUV_POOL_DEPTH_CM)-body-self.depth
        locked = self.search_turn_state.get('turn_tgt')
        if locked is None:
            return TF.wait_cmd(self.NAME, '扫视等待航向遥测，不能锁存相对目标')
        error = angle_error(locked, self.search_step_target)
        if abs(error) > 1e-6 and now >= self.search_next_step_at:
            delta = clamp(error, -cfg.search_step_deg, cfg.search_step_deg)
            self.search_step_target = (self.search_step_target+delta+180) % 360-180
            self.search_next_step_at = now+cfg.search_step_interval_s
        self.yaw = self.search_step_target
        ramp_done = abs(angle_error(locked, self.yaw)) <= 1e-6
        if not ramp_done:
            self.search_turn_state['ok_cnt'] = 0
        elif self.search_ramp_finished_at is None:
            self.search_ramp_finished_at = now
        # v2同样只在输出壳镜像一次；任务内保持actual_yaw同系绝对角。
        cmd = TF.turn_step(self.ctx, self.search_turn_state, now, dt, locked,
                           tol_deg=cfg.search_yaw_tolerance_deg,
                           hold_n=cfg.search_hold_frames, stage=self.NAME)
        if ramp_done and cmd is None:
            self.yaw = self.actual_yaw
            if self.phase == 'RETURN_HEADING':
                # 无门不等于结束赛段；返回同一基准后重新扫视，不输出平移推力。
                return self._start_search(now, self.search_base)
            else:
                self.search_valid_frames = 0
                self._go('SEARCH_OBSERVE', now)
                self.settle_until = now+self.cfg.settle_s
            return self._command('搜索观察角到位', require_depth=False)
        if (self.search_ramp_finished_at is not None
                and now-self.search_ramp_finished_at > cfg.motion_timeout_s):
            return self._fault(now, '扫视最终航向未到位：目标 %.1f°／实测 %.1f°' % (target, self.actual_yaw))
        note = ('扫视终点 %.1f°／本拍任务系目标 %.1f°／实测 %.1f°'
                % (target, self.yaw, self.actual_yaw))
        if cmd is None:
            return self._command(note, require_depth=False)
        cmd.update(yaw=self.yaw, depth=self.depth, note=self.phase+': '+note)
        return cmd

    def _search_tick(self, obs, now, dt):
        """纯转向不依赖融合定深或视觉初始化；有效新帧用于发现门和到位观察。"""
        cfg = self.cfg
        if now-self.search_started > cfg.search_timeout_s:
            return self._fault(now, '扫视未完成有效观察／航向到位超时')
        usable = (obs.get('valid') and now >= self.settle_until
                  and obs.get('capture_ts', now) >= self.settle_until)
        if usable and obs.get('has_target') and obs.get('fresh'):
            self.yaw = self.actual_yaw
            self.search_step_target = None
            self.target_id, self.filtered_center = obs['target_id'], None
            self.filtered_frame, self.episode_started = None, now
            self._go('YOLO_ALIGN', now)
            return self._command('扫视发现门，锁定当前航向；对中／接近仍需卡尔曼定深')
        if self.phase in ('SEARCH_TURN', 'RETURN_HEADING'):
            target = (self.search_base+self.search_angles[self.search_index]
                      if self.phase == 'SEARCH_TURN' else self.search_base)
            return self._search_turn((target+180) % 360-180, now, dt)
        if not usable:
            return self._command('观察角到位，等待稳定后的有效视觉帧', require_depth=False)
        if obs.get('fresh'):
            self.search_valid_frames += 1
        if (now-self.phase_start >= cfg.search_observe_s
                and self.search_valid_frames >= cfg.observe_frames):
            self.search_index += 1
            self._go('SEARCH_TURN' if self.search_index < len(self.search_angles) else 'RETURN_HEADING', now)
        return self._command('观察有效无门帧', require_depth=False)

    def _commit_blind(self, obs, now):
        distance = self.cfg.blind_distance_m
        if self.cfg.blind_use_pose_distance:
            p = obs['pose']
            try:
                remaining = float(p['center_robot_m'][2])
                if not p.get('metric_distance_available') or not math.isfinite(remaining) or remaining <= 0:
                    raise ValueError('无可靠的门距离')
                distance = remaining+self.cfg.blind_clearance_m
            except (KeyError, TypeError, ValueError):
                return self._fault(now, '四点度量距离不可用，不能计算盲冲距离')
        if not 0 < distance <= self.cfg.blind_max_distance_m:
            return self._fault(now, '盲冲距离超出配置范围，请核对内参／门尺寸')
        self.blind_travel_m = distance
        self.yaw = self.actual_yaw
        self._go('BLIND', now)
        return self._command('四点回正完成，锁定航向／深度，盲冲距离 %.2fm' % distance)

    def step(self, now, dt):
        cfg = self.cfg
        if self.phase == 'DONE':
            return None
        if self.phase == 'HOLD_FAULT':
            return self._command(self.fault_reason)
        # 已承诺的有界穿越不再消费视觉／门底梁污染的融合净空；外层急停仍生效。
        if self.phase == 'BLIND':
            if self.drive_started is None:
                self.drive_started = now
            duration = self.blind_travel_m/cfg.blind_speed_mps+cfg.blind_extra_s
            if now-self.drive_started < duration:
                return self._command('锁定航向／深度的定时盲冲',
                                     surge=cfg.blind_surge, require_depth=False)
            self.gates_passed += 1
            if self.gates_passed >= cfg.gates_to_pass:
                self._go('DONE', now)
                return self._command('全部门已完成，交接下一任务', require_depth=False)
            self.vision.reset_target()
            self.target_id = self.filtered_center = self.filtered_frame = None
            self.empty_since, self.empty_frames = None, 0
            self.motion = None
            self.entry_yaw = None
            self.episode_started = now
            self._go('ACQUIRE', now)
            return self._command('过门完成，重新锁存当前航向寻找下一门')

        self._read_depth(now)

        tel = self.ctx.tel or {}
        try:
            self.actual_yaw = float(tel['actual_yaw'])
            self.actual_depth = float(tel['actual_depth_cm'])
            if not all(math.isfinite(v) for v in (self.actual_yaw, self.actual_depth)):
                raise ValueError('非有限遥测')
        except (KeyError, TypeError, ValueError):
            from task import t_function as TF
            return self._command('等待航向／深度遥测') if self.yaw is not None else TF.wait_cmd(self.NAME)
        if self.entry_yaw is None:
            self.entry_yaw = self.actual_yaw
        if self.yaw is None:
            self.yaw = self.actual_yaw
            self.depth = self._depth_target(self.actual_depth)
        if self.depth_valid and self.fused_target_cm is None:
            self._lock_depth_target(self.depth)
        # 启动纯转向必须在视觉/融合到位门槛之前，避免一直停在 ACQUIRE。
        # 仍需有效固件航向/深度遥测；搜索始终不输出前进或横移推力。
        if self.phase == 'ACQUIRE':
            self._start_search(now, self.entry_yaw, reset=False)
        if self.phase.startswith('SEARCH') or self.phase == 'RETURN_HEADING':
            return self._search_tick(self.vision.poll(now), now, dt)

        if not self.depth_valid:
            self.counts = {}
            if self.motion is not None and now-self.motion['start'] > cfg.motion_timeout_s:
                return self._fault(now, '动作期间深度计读数持续不可用，停止调整')
            if self.motion is not None and self.motion['kind'] == 'sway':
                self.motion = None  # 停推时间不能算成已完成的横移距离
                self.counts = {}
                self.settle_until = now+cfg.settle_s
            return self._command('深度计读数不可用，保持目标并停止水平运动')
        obs = self.vision.poll(now)
        if self.motion is not None:
            if self.motion['kind'] == 'sway' and (not self.depth_confirmed or not obs.get('valid')
                                                or not obs.get('has_target')):
                self.motion = None
                self.counts = {}
                self.settle_until = now+cfg.settle_s
                return self._command('横移中观测失效／定深离带，停止横移')
            return self._motion_tick(now)
        if not self.depth_confirmed:
            self.counts = {}
            return self._command('深度计读数未连续到位')
        if now < self.settle_until:
            return self._command('等待运动稳定')
        if not obs.get('valid'):
            self.counts = {}
            self.empty_since, self.empty_frames = None, 0
            return self._command('视觉未就绪／过期，保持零水平推力')
        if obs.get('capture_ts', now) < self.settle_until:
            return self._command('等待运动后采集的新帧')

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
        if now-self.episode_started > cfg.target_episode_timeout_s:
            return self._fault(now, '当前门未完成对准／穿越，达到总时限')

        if self.phase in ('YOLO_ALIGN', 'APPROACH_40'):
            dx, dy = self._center_error(obs)
            raw_dx = obs['center_px'][0]-obs['aim_px'][0]
            raw_dy = obs['center_px'][1]-obs['aim_px'][1]
            centered = (not obs.get('clipped') and
                        max(abs(dx), abs(dy), abs(raw_dx), abs(raw_dy)) <= cfg.center_tolerance_px)
            stable = self._count('center', centered, obs)
            if self.phase == 'YOLO_ALIGN':
                if obs.get('clipped') and obs['area_ratio'] > cfg.area_near+cfg.area_tolerance:
                    self._go('APPROACH_40', now)
                    return self._command('初次捕获框过大／裁切，先后退恢复完整框',
                                         surge=-cfg.approach_surge)
                if not stable:
                    action = self._yolo_adjust(obs, now)
                    return action or self._command('中心8px范围内，等待稳定')
                self._go('APPROACH_40', now)
            if now-self.phase_start > cfg.approach_timeout_s:
                return self._fault(now, '40%接近阶段超时，请核对目标／推力')
            area = obs['area_ratio']
            low, high = cfg.area_near-cfg.area_tolerance, cfg.area_near+cfg.area_tolerance
            in_band = low <= area <= high and centered
            if self._count('area', in_band, obs):
                self._go('CV_ALIGN', now)
                return self._command('约40%面积且对中，开始四点反算回正')
            if in_band:
                return self._command('约40%面积，停止前后运动并等待稳定新帧')
            # 框过大或多侧贴边时先退回可观察距离，不能继续靠裁切框前进。
            if area > high:
                return self._command('框过大，后退至40%附近', surge=-cfg.approach_surge)
            if not centered:
                self.counts['area'] = 0
                action = self._yolo_adjust(obs, now)
                return action or self._command('暂停前后运动，重新对中')
            return self._command('向门接近至40%附近', surge=cfg.approach_surge)

        if self.phase == 'CV_ALIGN':
            if now-self.phase_start > cfg.align_timeout_s:
                return self._fault(now, '40%阶段四点回正超时')
            if (obs.get('clipped') or not cfg.area_near-cfg.area_tolerance <= obs['area_ratio'] <= cfg.area_near+cfg.area_tolerance):
                self._go('APPROACH_40', now)
                return self._command('回正改变面积／框裁切，先重新调整40%距离')
            if not self._pose_good(obs):
                self.counts['pose'] = 0
                return self._start_probe(obs, now, 'CV_ALIGN')
            if self._count('pose', self._pose_aligned(obs), obs):
                return self._commit_blind(obs, now)
            return self._cv_adjust(obs, now)

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
