"""无硬件测试：新帧计数、姿态分步控制、失效停推、盲冲及赛段交接。"""
import json
import sys
import shutil
import time
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[5]
for path in (ROOT, ROOT/'config', ROOT/'src/to32/move_test'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from task.task_door.config import CONFIG
from task.task_door.t_door import DoorTask
from task.task_door.observation import DoorVisionIF


class FakeVision:
    def __init__(self):
        self.obs = {}
        self.resets = 0
        self.calls = 0

    def reset_target(self):
        self.resets += 1

    def poll(self, now):
        self.calls += 1
        return dict(self.obs, capture_ts=now)


class FakeDepth:
    def __init__(self):
        # 有意与固件40cm差18cm，验证不同参考点不能直接混比。
        self.value = dict(ok=True, stale=False, D=.58, v_z=0.)
        self.stamp = None

    def read(self, now):
        return dict(self.value, sample_ts=now if self.stamp is None else self.stamp)


def target(frame=1, area=.2, center=(320, 240), pose=True):
    return dict(valid=True, fresh=True, has_target=True, frame=frame, target_id=1,
                center_px=list(center), aim_px=[320, 240], focal_px=[900, 900],
                width_px=350, height_px=250, area_ratio=area, clipped=False, boundary_sides=[],
                corners=[[145, 115], [495, 115], [495, 365], [145, 365]],
                geometry=dict(observation='detected', observed_segments=[1, 2, 3, 4]),
                pose=(dict(controllable=True, yaw_error_deg=0., alignment_robot_m=[0, 0, 0],
                           center_robot_m=[0, 0, 1.], center_offset_robot_px=[0, 0]) if pose else None))


class StateTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CONFIG, stable_frames=2, observe_frames=2, settle_s=.01,
                           filter_alpha=1., search_observe_s=.1, depth_hold_samples=2)
        self.ctx = SimpleNamespace(cfg=SimpleNamespace(), tel={'actual_yaw': 10., 'actual_depth_cm': 40.},
                                   say=lambda text: None, depth=FakeDepth())
        self.vision = FakeVision()
        self.task = DoorTask(self.ctx, self.cfg, self.vision)
        self.task.enter(0.)
        self.now = 0.
        self.frame = 0
        for _ in range(3):
            self.tick({'valid': False})  # 锁定初始融合目标，连续新样本确认定深

    def tick(self, obs=None, seconds=.1):
        self.now += seconds
        self.frame += 1
        if obs is not None:
            self.vision.obs = dict(obs, frame=self.frame)
        return self.task.step(self.now, seconds)

    def phase(self, name):
        self.task.yaw, self.task.depth = 10., 40.
        self.task.target_id = 1
        self.task._go(name, self.now)

    def test_yolo_align_requires_new_frames_and_only_yaw_depth(self):
        cmd = self.tick(target(center=(380, 240)))
        self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))
        self.assertGreater(cmd['yaw'], 10)
        self.ctx.tel['actual_yaw'] = cmd['yaw']
        self.tick(target())
        self.tick(target())
        self.assertEqual(self.task.phase, 'YOLO_ALIGN')
        cmd = self.tick(target())
        self.assertEqual(self.task.phase, 'APPROACH_50')
        self.assertGreater(cmd['surge'], 0)

    def test_duplicate_frames_do_not_complete_alignment(self):
        self.tick(target())
        self.vision.obs = dict(target(), fresh=False)
        for _ in range(8):
            self.tick()
        self.assertEqual(self.task.phase, 'YOLO_ALIGN')

    def test_stale_input_sends_explicit_zero_thrust_not_a_pause(self):
        self.phase('APPROACH_50')
        self.assertGreater(self.tick(target())['surge'], 0)
        cmd = self.tick({'valid': False, 'fresh': False})
        self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))
        self.assertNotIn('paused', cmd)
        self.assertNotIn('SEARCH', self.task.phase)

    def test_thresholds_and_blind_lock_ignore_all_visual_input(self):
        self.phase('APPROACH_50')
        self.tick(target(area=.5))
        self.tick(target(area=.5))
        self.assertEqual(self.task.phase, 'CV_ALIGN')
        self.tick(target(area=.5))
        self.tick(target(area=.5))
        self.assertEqual(self.task.phase, 'APPROACH_80')
        self.tick(target(area=.8, pose=False))
        self.tick(target(area=.8, pose=False))
        self.assertEqual(self.task.phase, 'BLIND')
        calls = self.vision.calls
        self.ctx.tel = {}
        self.vision.obs = {'valid': False}
        cmd = self.tick(seconds=.5)
        self.assertEqual(self.vision.calls, calls)
        self.assertEqual((cmd['yaw'], cmd['depth'], cmd['sway']), (10., 40., 0))
        self.assertEqual(cmd['surge'], self.cfg.blind_surge)
        self.tick(seconds=3.3)
        self.assertEqual(self.task.phase, 'ACQUIRE')
        self.assertEqual(self.task.gates_passed, 1)
        self.assertEqual(self.vision.resets, 2)

    def test_cv_adjustment_is_yaw_then_depth_then_sway(self):
        self.phase('CV_ALIGN')
        obs = target()
        obs['pose'].update(yaw_error_deg=15, alignment_robot_m=[.1, .1, 0])
        cmd = self.tick(obs)
        self.assertEqual(self.task.motion['kind'], 'yaw')
        self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))
        self.task.motion = None
        self.task.settle_until = 0
        obs['pose']['yaw_error_deg'] = 0
        cmd = self.tick(obs)
        self.assertEqual(self.task.motion['kind'], 'depth')
        self.assertGreater(cmd['depth'], 40)
        self.ctx.depth.value['D'] = self.task.fused_target_cm/100.
        self.ctx.tel['actual_depth_cm'] = self.task.depth
        for _ in range(self.cfg.depth_hold_samples):
            self.tick(obs)
        self.assertIsNone(self.task.motion)
        obs['pose']['alignment_robot_m'][1] = 0
        cmd = self.tick(obs)
        self.assertEqual(self.task.motion['kind'], 'sway')
        self.assertGreater(cmd['sway'], 0)

    def test_missing_pose_below_commit_stops_approach_and_has_timeout(self):
        self.phase('APPROACH_80')
        for _ in range(5):
            self.assertEqual(self.tick(target(area=.79, pose=False))['surge'], 0)
        self.assertEqual(self.task.phase, 'APPROACH_80')
        self.phase('APPROACH_80')
        self.tick(target(pose=False), seconds=self.cfg.approach_timeout_s+1)
        self.assertEqual(self.task.phase, 'HOLD_FAULT')

    def test_commit_accepts_clipped_current_target_without_second_pose_check(self):
        self.phase('APPROACH_80')
        obs = target(area=.82, pose=False)
        obs.update(clipped=True, boundary_sides=['left', 'right'], corners=None, geometry=None)
        self.tick(obs)
        self.assertEqual(self.task.phase, 'APPROACH_80')
        self.tick(obs)
        self.assertEqual(self.task.phase, 'BLIND')

    def test_commit_requires_consecutive_fresh_valid_frames_of_same_target(self):
        self.phase('APPROACH_80')
        self.tick(target(area=.82, pose=False))
        self.vision.obs = dict(target(area=.82, pose=False), fresh=False)
        for _ in range(3):
            self.tick()
        self.assertEqual(self.task.phase, 'APPROACH_80')
        self.tick(target(area=.79))
        self.tick(target(area=.82, pose=False))
        self.tick({'valid': False, 'fresh': False})
        self.tick(target(area=.82, pose=False))
        self.assertEqual(self.task.phase, 'APPROACH_80')
        self.tick(dict(target(area=.82, pose=False), target_id=2))
        self.assertEqual(self.task.phase, 'YOLO_ALIGN')

    def test_shared_depth_limits_and_task_signs_follow_t_function(self):
        from task import t_function as TF
        from task.task_door.t_door import angle_error
        with patch.object(TF.TC, 'AUV_POOL_DEPTH_CM', 90), \
             patch.object(TF.TC, 'AUV_BODY_HEIGHT_CM', 20, create=True), \
             patch.object(TF.TC, 'AUV_SURF_SAFE_CM', 30, create=True):
            self.phase('BLIND')
            self.task.depth = 86
            self.ctx.tel['actual_yaw'] = -99
            cmd = self.tick()
            self.assertEqual((cmd['depth'], cmd['yaw'], cmd['surge'], cmd['sway']),
                             (70, 10, self.cfg.blind_surge, 0))
            self.task.depth = 1
            self.assertEqual(self.task._command()['depth'], 30)
        self.assertEqual(angle_error(-179, 179), TF.yaw_err_deg(179, -179))
        self.assertEqual(angle_error(-179, 179), 2)

    def test_probe_reverses_when_width_narrows_and_is_bounded(self):
        self.phase('CV_ALIGN')
        obs = target(pose=False)
        self.tick(obs)
        cmd = self.tick(obs)
        self.assertLess(cmd['sway'], 0)
        self.tick(obs, seconds=.3)
        obs['width_px'] = 300
        self.tick(obs)
        self.tick(obs)
        cmd = self.tick(obs)
        self.assertGreater(cmd['sway'], 0)
        self.task.motion = None
        self.task.probe_steps = self.cfg.probe_max_steps
        self.task._go('PROBE_MOVE', self.now)
        self.tick(obs)
        self.assertEqual(self.task.phase, 'HOLD_FAULT')

    def test_search_left_right_return_and_bounded_exit(self):
        empty = {'valid': True, 'fresh': True, 'has_target': False}
        self.tick(empty)
        self.tick(empty, seconds=1.6)
        self.assertEqual(self.task.phase, 'SEARCH_TURN')
        cmd = self.tick(empty)
        self.assertEqual(cmd['yaw'], 8.)
        observation_angles = []
        for _ in range(140):
            previous_phase = self.task.phase
            previous_yaw = self.ctx.tel['actual_yaw']
            self.ctx.tel['actual_yaw'] = cmd['yaw']
            self.assertLessEqual(abs(cmd['yaw']-previous_yaw), self.cfg.search_step_deg+1e-6)
            cmd = self.tick(empty, seconds=.5)
            if previous_phase != 'SEARCH_OBSERVE' and self.task.phase == 'SEARCH_OBSERVE':
                observation_angles.append(self.ctx.tel['actual_yaw'])
            if self.task.phase == 'EXIT':
                break
        self.assertEqual(observation_angles, [-35., 55.])
        self.assertEqual(self.ctx.tel['actual_yaw'], 10.)
        self.assertEqual(self.task.phase, 'EXIT')
        self.assertGreater(self.tick(empty)['surge'], 0)
        self.tick(empty, seconds=3.)
        self.assertEqual(self.task.phase, 'DONE')
        self.assertIsNone(self.tick(empty))

    def test_depth_motion_uses_fused_delta_and_distinct_stable_samples(self):
        self.tick(target(center=(320, 290)))
        self.assertEqual(self.task.motion['kind'], 'depth')
        self.assertEqual(self.task.depth, 42.)
        self.assertAlmostEqual(self.task.fused_target_cm, 60.)
        # 固件已到位，但融合深度仍未到位，不能通过。
        self.ctx.tel['actual_depth_cm'] = 42.
        self.tick(target())
        self.assertIsNotNone(self.task.motion)
        self.ctx.depth.value.update(D=.60)
        self.ctx.depth.stamp = self.now+.1
        self.tick(target())
        for _ in range(3):
            self.tick(target())
        self.assertIsNotNone(self.task.motion)  # 重复同一个样本不累计
        self.ctx.depth.stamp = None
        self.ctx.depth.value.update(v_z=.1)
        self.tick(target())
        self.assertEqual(self.task.depth_ok_count, 0)
        self.ctx.depth.value.update(v_z=0.)
        self.tick(target())
        self.assertIsNotNone(self.task.motion)
        self.tick(target())
        self.assertIsNone(self.task.motion)

    def test_invalid_fusion_blocks_approach_and_depth_completion(self):
        self.phase('APPROACH_50')
        self.assertGreater(self.tick(target())['surge'], 0)
        for bad in ({'ok': False}, {'stale': True}, {'D': float('nan')}, {'v_z': float('inf')}):
            with self.subTest(bad=bad):
                original = dict(self.ctx.depth.value)
                self.ctx.depth.value.update(bad)
                cmd = self.tick(target())
                self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))
                self.assertFalse(self.task.depth_ready)
                self.ctx.depth.value = original
        self.tick(target())
        self.assertGreater(self.tick(target())['surge'], 0)

    def test_blind_keeps_visual_lock_but_faults_on_depth_loss(self):
        self.phase('BLIND')
        calls = self.vision.calls
        self.ctx.depth.value['ok'] = False
        cmd = self.tick()
        self.assertEqual(self.vision.calls, calls)
        self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))
        self.assertEqual(self.task.phase, 'HOLD_FAULT')

    def test_interrupted_sway_does_not_count_stopped_time_as_distance(self):
        for bad in ({'ok': False}, {'D': .70}):
            with self.subTest(bad=bad):
                self.setUp()
                self.phase('CV_ALIGN')
                obs = target()
                obs['pose']['alignment_robot_m'][0] = .1
                self.assertGreater(self.tick(obs)['sway'], 0)
                self.ctx.depth.value.update(bad)
                cmd = self.tick(obs)
                self.assertEqual(cmd['sway'], 0)
                self.assertIsNone(self.task.motion)

    def test_search_step_is_latched_and_interval_limits_progress(self):
        self.task.actual_yaw = 179.
        self.task._start_search(self.now)
        self.ctx.tel['actual_yaw'] = 179.
        cmd = self.tick({'valid': True, 'fresh': True, 'has_target': False})
        self.assertEqual(cmd['yaw'], 177.)
        self.tick()
        self.assertEqual(self.task.yaw, 177.)  # 未到位不逐拍累加
        self.ctx.tel['actual_yaw'] = 177.
        self.tick()
        self.assertEqual(self.task.yaw, 177.)  # 到位但未满最短间隔
        self.tick(seconds=.5)
        self.assertEqual(self.task.yaw, 175.)
        cmd = self.tick(target())
        self.assertEqual((self.task.phase, cmd['yaw']), ('YOLO_ALIGN', 177.))

    def test_search_shortest_relative_step_wraps_at_180(self):
        self.task.actual_yaw = 179.
        self.task._start_search(self.now)
        self.task.search_angles = [45.]
        self.ctx.tel['actual_yaw'] = 179.
        cmd = self.tick({'valid': True, 'fresh': True, 'has_target': False})
        self.assertEqual(cmd['yaw'], -179.)

    def test_clamped_depth_delta_has_reachable_fused_target(self):
        self.task.actual_depth = self.cfg.max_depth_cm
        self.task.fused_depth_cm = self.cfg.max_depth_cm+18.
        self.task._lock_depth_target(self.cfg.max_depth_cm+2.)
        self.assertEqual(self.task.depth, self.cfg.max_depth_cm)
        self.assertAlmostEqual(self.task.fused_target_cm, self.cfg.max_depth_cm+18.)

    def test_search_stalled_step_has_timeout(self):
        self.task.actual_yaw = 10.
        self.task._start_search(self.now)
        self.tick({'valid': True, 'fresh': True, 'has_target': False})
        cmd = self.tick(seconds=self.cfg.motion_timeout_s+1)
        self.assertEqual(self.task.phase, 'HOLD_FAULT')
        self.assertEqual((cmd['surge'], cmd['sway']), (0, 0))


class ObservationTests(unittest.TestCase):
    def test_depth_interface_reports_sample_identity_and_rejects_bad_quality(self):
        import obs
        import os
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'momo_depth.json'
            reader = obs.DepthIF(directory)
            record = dict(valid=True, D=.58, v_z=0., sigma={'D': .01})
            path.write_text(json.dumps(record))
            os.utime(path, (100., 100.))
            first = reader.read(100.1)
            self.assertTrue(first['ok'])
            self.assertEqual(first['sample_ts'], reader.read(100.2)['sample_ts'])
            self.assertFalse(reader.read(102.)['ok'])
            record['sigma']['D'] = 1.
            path.write_text(json.dumps(record))
            os.utime(path, (100., 100.))
            self.assertFalse(reader.read(100.1)['ok'])

    def test_reset_ack_stale_and_duplicate_are_distinct_from_no_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            vision = DoorVisionIF(directory)
            vision.reset_target()
            record = dict(frame=1, stage='PassGate', status='done', capture_ts=100.,
                          door=dict(valid=True, has_target=False, reset_token=vision.token,
                                    img_w=CONFIG.image_width, img_h=CONFIG.image_height))
            path = Path(directory)/'momo_det_front.json'
            path.write_text(json.dumps(record))
            obs = vision.poll(100.1)
            self.assertTrue(obs['valid'] and obs['fresh'])
            self.assertFalse(obs['has_target'])
            self.assertFalse(vision.poll(100.2)['fresh'])
            self.assertFalse(vision.poll(101.)['valid'])
            vision.reset_target()
            self.assertFalse(vision.poll(100.2)['valid'])

    def test_front_pipeline_publishes_atomic_frame_contract_and_drops_backlog(self):
        import numpy as np
        from task.task_door.front_pipeline import DoorFrontPipeline
        with tempfile.TemporaryDirectory() as directory:
            vision = DoorVisionIF(directory)
            vision.reset_target()
            pipeline = DoorFrontPipeline(directory)
            image = np.full((CONFIG.image_height, CONFIG.image_width, 3), 80, np.uint8)
            def process(frame, detector, now, token):
                return frame, [], dict(valid=True, has_target=False, reset_token=token,
                                       img_w=CONFIG.image_width, img_h=CONFIG.image_height)
            pipeline.processor = SimpleNamespace(process=process)
            detector = SimpleNamespace(stage='PassGate', ready=True, is_current=lambda: True,
                                       cfg={'model_path': 'door.hbm'})
            class Writer:
                def write(self, record):
                    (Path(directory)/'momo_det_front.json').write_text(json.dumps(record))
            with patch('task.task_door.front_pipeline.time.time', return_value=100.):
                self.assertIsNotNone(pipeline.process(10, image, detector, 99.9, None, Writer(), 90))
                self.assertTrue(vision.poll(100.)['valid'])
                self.assertIsNone(pipeline.process(9, image, detector, 99.9, None, Writer(), 90))
                self.assertIsNone(pipeline.process(11, image, detector, 98., None, Writer(), 90))


class PerceptionTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('clang') and shutil.which('clang++'), 'OpenCL C correctness harness requires clang')
    def test_resident_gpu_publishes_formal_control_pose_without_image_reuploads(self):
        import cv2
        import numpy as np
        sys.path[:0] = [str(ROOT/'src'), str(ROOT/'quad_cv_kit'), str(ROOT/'quad_cv_kit/tests')]
        from opencl_host import HostRuntime
        from quad_cv_kit.src.opencl_backend import OpenCLBackend
        from quad_cv_kit.src.opencl_runtime import LocalMemory
        from task.task_door.perception import DoorFrameProcessor
        from task.task_door.front_pipeline import DoorFrontPipeline
        from shm_writer import ShmFrameWriter, ShmJsonWriter
        from shm_reader import ShmFrameReader, ShmJsonReader
        matrix = np.array([[500., 0, 320], [0, 510., 240], [0, 0, 1]])
        frame = np.full((480, 640, 3), (120, 90, 40), np.uint8)
        cv2.rectangle(frame, (145, 113), (495, 368), (50, 60, 230), 6)
        yy, xx = np.indices((480, 640), dtype=np.float32)
        with patch('quad_cv_kit.src.opencl_backend.OpenCLRuntime', HostRuntime):
            gpu = OpenCLBackend(quality='fast', blur_mode='pyramid')
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='opencl'))
        processor.corrector = SimpleNamespace(output_matrix=matrix, maps=lambda distance: (xx, yy))
        processor.valid_mask = np.ones((480, 640), bool)
        seen = []
        def detect(fixed, nv12=None):
            self.assertIsNone(nv12)
            seen.append(fixed.copy())
            return [dict(label='door', score=.9, bbox=[135, 100, 505, 380])]
        detector = SimpleNamespace(stage='PassGate', ready=True, is_current=lambda: True,
                                   cfg={'model_path': 'door_6_nashe_640x640_nv12.hbm'}, detect=detect)
        try:
            with tempfile.TemporaryDirectory() as directory, \
                 patch('opencl_host.LocalMemory', LocalMemory), \
                 patch('task.task_door.front_pipeline.time.time', return_value=100.), \
                 patch('task.task_door.perception.create_backend', return_value=(gpu, gpu.runtime.info)), \
                 patch('task.task_door.perception.RedGateTracker.update', side_effect=AssertionError('CPU image CV forbidden')):
                vision = DoorVisionIF(directory)
                vision.reset_target()
                pipeline = DoorFrontPipeline(directory)
                pipeline.processor = processor
                fw = ShmFrameWriter(str(Path(directory)/'momo_frame_front.bin'), 1280, 720)
                dw = ShmJsonWriter(str(Path(directory)/'momo_det_front.json'))
                reader = ShmFrameReader(fw.path)
                try:
                    now = time.time()
                    self.assertIsNotNone(pipeline.process(1, frame, detector, now, fw, dw, 85))
                    first = ShmJsonReader(dw.path).read()['door']
                    self.assertTrue(first['has_target'])
                    self.assertEqual(first['cv_backend']['cv_execution'], 'resident')
                    self.assertIsNotNone(first['pose'], first['pose_reason'])
                    self.assertEqual(first['pose_execution'], 'cpu-four-point-IPPE')
                    self.assertTrue(first['pose']['metric_distance_available'])
                    self.assertTrue(first['pose']['controllable'])
                    np.testing.assert_allclose(first['pose']['center_robot_m'], [0, 0, 1], atol=.02)
                    self.assertEqual(first['cv_profile']['intermediate_readbacks'], 0)
                    self.assertEqual(first['gpu']['pipeline_version'], 7)
                    self.assertEqual(first['gpu']['download_bytes'], 640*480*3+800)
                    np.testing.assert_array_equal(seen[0], frame)
                    self.assertEqual((fw.width, fw.height), (640, 480))
                    _, jpeg = reader.read_latest()
                    self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape, frame.shape)
                    self.assertTrue(vision.poll(time.time())['valid'])
                    self.assertIsNotNone(pipeline.process(2, frame, detector, time.time(), fw, dw, 85))
                    steady = ShmJsonReader(dw.path).read()['door']
                    self.assertEqual(steady['gpu']['upload_bytes'], 640*480*3+24)
                    self.assertEqual(steady['gpu']['download_bytes'], 640*480*3+800)
                    self.assertEqual(vision.poll(time.time())['target_id'], 1)
                    vision.reset_target()
                    self.assertIsNotNone(pipeline.process(3, frame, detector, time.time(), fw, dw, 85))
                    self.assertEqual(vision.poll(time.time())['target_id'], 2)
                    with patch.object(gpu, 'close', wraps=gpu.close) as close:
                        pipeline.close(); pipeline.close()
                        self.assertEqual(close.call_count, 1)
                finally:
                    fw.close(); dw.close(); reader.close()
        finally:
            processor.close()

    def test_missing_or_tracked_geometry_cannot_produce_control_pose(self):
        import numpy as np
        from task.task_door.perception import DoorFrameProcessor
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='cpu'))
        processor.corrector = SimpleNamespace(output_matrix=np.eye(3), undistort=lambda frame, distance: frame)
        processor.valid_mask = np.ones((480, 640), bool)
        detector = SimpleNamespace(detect=lambda frame, nv12=None: [dict(label='door', score=.9, bbox=[100, 100, 500, 400])])
        with patch.object(processor.cv, 'update', return_value=({}, [])), \
             patch('task.task_door.perception.original_geometry', return_value=dict(observation='tracked')), \
             patch('task.task_door.perception.estimate_alignment', side_effect=AssertionError('old geometry cannot control')):
            _, _, obs = processor.process(np.zeros((480, 640, 3), np.uint8), detector, 100.)
            self.assertIsNone(obs['pose'])

    def test_completed_model_edges_do_not_bypass_measured_extrapolation_limits(self):
        import numpy as np
        from task.task_door.perception import DoorFrameProcessor
        processor = DoorFrameProcessor(replace(CONFIG, cv_backend='cpu'))
        processor.corrector = SimpleNamespace(output_matrix=np.eye(3), undistort=lambda frame, distance: frame)
        processor.valid_mask = np.ones((480, 640), bool)
        corners = np.array([[100, 100], [500, 100], [500, 400], [100, 400]])
        segments = [[a, b] for a, b in zip(corners, np.roll(corners, -1, axis=0))]
        geometry = dict(observation='detected', segments=segments, observed_segments=[
            [a+(b-a)*.4, a+(b-a)*.6] for a, b in segments])
        detector = SimpleNamespace(detect=lambda frame, nv12=None: [dict(label='door', score=.9, bbox=[100, 100, 500, 400])])
        with patch.object(processor.cv, 'update', return_value=({}, [])), \
             patch('task.task_door.perception.original_geometry', return_value=geometry), \
             patch('task.task_door.perception.estimate_alignment', side_effect=AssertionError('short measured rods cannot control')):
            _, _, obs = processor.process(np.zeros((480, 640, 3), np.uint8), detector, 100.)
            self.assertIsNone(obs['pose'])
            self.assertEqual(obs['pose_reason'], 'excessive-extrapolation')

    def test_four_extended_lines_produce_metric_pose_and_area(self):
        import cv2
        import numpy as np
        from task.task_door.perception import DoorFrameProcessor
        cfg = replace(CONFIG, image_width=1280, image_height=720,
                      cv_backend='cpu',
                      camera_params_path=str(ROOT/'quad_cv_kit/camera_correction_params.json'))
        p = DoorFrameProcessor(cfg)
        # 纯投影矩形：四条边只保留中间76%，四交点由允许的端部延长得到。
        matrix = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1]])
        objects = np.array([[-.35, -.25, 0], [.35, -.25, 0], [.35, .25, 0], [-.35, .25, 0]])
        corners = cv2.projectPoints(objects, np.zeros(3), np.array([0., 0., 1.]), matrix, np.zeros(5))[0].reshape(4, 2)
        geometry = dict(observation='detected', segments=[
            [a+(b-a)*.12, a+(b-a)*.88] for a, b in zip(corners, np.roll(corners, -1, axis=0))])
        p.corrector = SimpleNamespace(output_matrix=matrix, undistort=lambda image, distance: image)
        p.valid_mask = np.ones((720, 1280), bool)
        image = np.full((720, 1280, 3), 80, np.uint8)
        detector = SimpleNamespace(detect=lambda fixed, nv12=None: [dict(label='door', score=.9, bbox=[325, 135, 955, 585])])
        with patch.object(p.cv, 'update', return_value=({}, [])), patch(
                'task.task_door.perception.original_geometry', return_value=geometry):
            _, _, obs = p.process(image, detector, 100.)
        self.assertAlmostEqual(obs['area_ratio'], 630*450/(1280*720))
        self.assertEqual(len(obs['corners']), 4)
        self.assertEqual(len(obs['inferred_segments']), 8)
        self.assertTrue(obs['pose']['metric_distance_available'])
        np.testing.assert_allclose(obs['pose']['center_robot_m'], [0, 0, 1], atol=1e-6)

    def test_tracker_keeps_target_over_confident_background_gate(self):
        from task.task_door.perception import DoorFrameProcessor
        p = DoorFrameProcessor()
        near = dict(label='door', score=.7, bbox=[50, 60, 600, 440])
        far = dict(label='door', score=.99, bbox=[250, 180, 350, 280])
        self.assertEqual(p._select([near, far], 0)['bbox'], near['bbox'])
        self.assertIsNone(p._select([far], .1))
        self.assertEqual(p._select([far], 2)['bbox'], far['bbox'])


class SourceTests(unittest.TestCase):
    def test_depth_launcher_uses_actual_deployment_directory(self):
        import task_config
        from kalman_launcher import _depth_spec
        spec = _depth_spec(task_config)
        self.assertTrue(spec['enabled'])
        self.assertEqual(Path(spec['root']), ROOT/'src/kalman/depth_kalman')
        self.assertTrue((Path(spec['root'])/spec['script']).is_file())

    def test_changed_sources_compile_and_task_is_registered(self):
        import task_config
        self.assertIn(DoorTask, task_config.DOOR_TABLE)
        self.assertIn(DoorTask, task_config.STAGE_TABLE)
        paths = list(Path(__file__).parent.glob('*.py')) + [ROOT/'src/front.py', ROOT/'config/stage_model.py',
                                                         ROOT/'src/to32/move_test/mission.py',
                                                         ROOT/'src/to32/move_test/stage_base.py',
                                                         ROOT/'src/to32/move_test/obs.py',
                                                         ROOT/'src/to32/move_test/task_config.py',
                                                         ROOT/'src/to32/move_test/test_mode/test_config.py']
        for path in paths:
            compile(path.read_text(encoding='utf-8-sig'), str(path), 'exec')

    def test_mission_first_import_keeps_all_existing_task_tables(self):
        import subprocess
        result = subprocess.run([sys.executable, '-c',
            'import sys; sys.path[:0]=sys.argv[1:]; import mission, task_config; '
            'assert all([task_config.TASK1_TABLE, task_config.TASK2_TABLE, '
            'task_config.HIT_BALL_TABLE, task_config.DOOR_TABLE]); '
            'assert issubclass(task_config.DOOR_TABLE[0], mission.Stage)',
            str(ROOT/'src/to32/move_test'), str(ROOT/'config')],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
