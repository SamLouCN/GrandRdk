"""无硬件测试：新帧计数、姿态分步控制、失效停推、盲冲及赛段交接。"""
import json
import sys
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


def target(frame=1, area=.2, center=(640, 360), pose=True):
    return dict(valid=True, fresh=True, has_target=True, frame=frame, target_id=1,
                center_px=list(center), aim_px=[640, 360], focal_px=[900, 900],
                width_px=700, height_px=500, area_ratio=area, clipped=False, boundary_sides=[],
                corners=[[290, 110], [990, 110], [990, 610], [290, 610]],
                geometry=dict(observation='detected', observed_segments=[1, 2, 3, 4]),
                pose=(dict(controllable=True, yaw_error_deg=0., alignment_robot_m=[0, 0, 0],
                           center_robot_m=[0, 0, 1.], center_offset_robot_px=[0, 0]) if pose else None))


class StateTests(unittest.TestCase):
    def setUp(self):
        self.cfg = replace(CONFIG, stable_frames=2, observe_frames=2, settle_s=.01,
                           filter_alpha=1., search_observe_s=.1)
        self.ctx = SimpleNamespace(cfg=SimpleNamespace(), tel={'actual_yaw': 10., 'actual_depth_cm': 40.},
                                   say=lambda text: None)
        self.vision = FakeVision()
        self.task = DoorTask(self.ctx, self.cfg, self.vision)
        self.task.enter(0.)
        self.now = 0.
        self.frame = 0

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
        cmd = self.tick(target(center=(700, 360)))
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
        self.tick(target(area=.8))
        self.tick(target(area=.8))
        self.assertEqual(self.task.phase, 'CV_FINAL')
        self.tick(target(area=.8))
        self.tick(target(area=.8))
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
        self.task.motion = None
        obs['pose']['alignment_robot_m'][1] = 0
        cmd = self.tick(obs)
        self.assertEqual(self.task.motion['kind'], 'sway')
        self.assertGreater(cmd['sway'], 0)

    def test_missing_final_pose_never_blind_and_approach_has_timeout(self):
        self.phase('CV_FINAL')
        for _ in range(5):
            self.assertEqual(self.tick(target(area=.9, pose=False))['surge'], 0)
        self.assertEqual(self.task.phase, 'CV_FINAL')
        self.phase('APPROACH_80')
        self.tick(target(pose=False), seconds=self.cfg.approach_timeout_s+1)
        self.assertEqual(self.task.phase, 'HOLD_FAULT')

    def test_probe_reverses_when_width_narrows_and_is_bounded(self):
        self.phase('CV_ALIGN')
        obs = target(pose=False)
        self.tick(obs)
        cmd = self.tick(obs)
        self.assertLess(cmd['sway'], 0)
        self.tick(obs, seconds=.3)
        obs['width_px'] = 600
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
        self.assertEqual(cmd['yaw'], -35.)
        self.ctx.tel['actual_yaw'] = -35.
        self.tick(empty)
        self.tick(empty)
        self.tick(empty)
        cmd = self.tick(empty)
        self.assertEqual(cmd['yaw'], 55.)
        self.ctx.tel['actual_yaw'] = 55.
        self.tick(empty)
        self.tick(empty)
        self.tick(empty)
        cmd = self.tick(empty)
        self.assertEqual(cmd['yaw'], 10.)
        self.ctx.tel['actual_yaw'] = 10.
        self.tick(empty)
        self.assertEqual(self.task.phase, 'EXIT')
        self.assertGreater(self.tick(empty)['surge'], 0)
        self.tick(empty, seconds=3.)
        self.assertEqual(self.task.phase, 'DONE')
        self.assertIsNone(self.tick(empty))


class ObservationTests(unittest.TestCase):
    def test_reset_ack_stale_and_duplicate_are_distinct_from_no_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            vision = DoorVisionIF(directory)
            vision.reset_target()
            record = dict(frame=1, stage='PassGate', status='done', capture_ts=100.,
                          door=dict(valid=True, has_target=False, reset_token=vision.token,
                                    img_w=1280, img_h=720))
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
            image = np.full((720, 1280, 3), 80, np.uint8)
            def process(frame, detector, now, token):
                return frame, [], dict(valid=True, has_target=False, reset_token=token,
                                       img_w=1280, img_h=720)
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
    def test_four_extended_lines_produce_metric_pose_and_area(self):
        import cv2
        import numpy as np
        from task.task_door.perception import DoorFrameProcessor
        cfg = replace(CONFIG, camera_params_path=str(ROOT/'quad_cv_kit/camera_correction_params.json'))
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
        near = dict(label='door', score=.7, bbox=[100, 100, 1000, 650])
        far = dict(label='door', score=.99, bbox=[400, 300, 600, 400])
        self.assertEqual(p._select([near, far], 0)['bbox'], near['bbox'])
        self.assertIsNone(p._select([far], .1))
        self.assertEqual(p._select([far], 2)['bbox'], far['bbox'])


class SourceTests(unittest.TestCase):
    def test_changed_sources_compile_and_task_is_registered(self):
        import task_config
        self.assertIn(DoorTask, task_config.DOOR_TABLE)
        self.assertIn(DoorTask, task_config.STAGE_TABLE)
        paths = list(Path(__file__).parent.glob('*.py')) + [ROOT/'src/front.py', ROOT/'config/stage_model.py',
                                                         ROOT/'src/to32/move_test/mission.py',
                                                         ROOT/'src/to32/move_test/stage_base.py',
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
