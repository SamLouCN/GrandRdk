"""下载版 cx/cy 控制的离线接线测试；不打开相机、BPU 或串口。"""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[5]
sys.path[:0] = [str(ROOT/'config'), str(ROOT/'src/to32/move_test'), str(ROOT/'src/to32')]

from task.task_door.gate_yaw_cx_ctrl import GateYawController, GateYawCxTask, calc_target_yaw, DOOR_TABLE
from task.task_door.control_observation import GateControlObservation
from task.task_door.gate_depth_ctrl import GateDepthController, calc_target_depth


def record(frame=1, cx=300., cy=240., z=12., captured_at=100.):
    return dict(frame=frame, capture_ts=captured_at, stage='PassGate', status='done',
                door=dict(valid=True, has_target=True, coordinate_space='corrected',
                          img_w=640, img_h=480, target_id=3, center_px=[cx, cy],
                          guidance=dict(alignment=None if z is None else
                                        dict(rotation_xyz_deg=[1., 2., z]))))


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'momo_det_front.json'
        self.reader = GateControlObservation(self.directory.name)
        self.logs = []
        self.ctx = SimpleNamespace(cfg=SimpleNamespace(AUV_SHM_DIR=self.directory.name),
                                   vision=None, tel=dict(actual_yaw=10., actual_depth_cm=40.),
                                   say=self.logs.append)
        self.task = GateYawCxTask(self.ctx, self.reader)
        self.task.enter(100.)

    def publish(self, **kwargs):
        self.path.write_text(json.dumps(record(**kwargs)))

    def test_original_formula_gain_sign_and_angle_wrap(self):
        controller = GateYawController()
        for yaw, cx, expected in ((10, 300, 30), (10, 340, -10), (10, 320, 10),
                                  (179, 300, -161), (-179, 340, 161), (180, 320, 180),
                                  (-180, 320, 180), (0, 0, -40), (0, 640, 40)):
            with self.subTest(yaw=yaw, cx=cx):
                self.assertEqual(controller.step({'actual_yaw': yaw}, cx), expected)
        self.assertEqual(controller.K_PSI_DEG, 1.)
        self.assertEqual(calc_target_yaw(10, 300, .25), 15.)
        self.assertIsNone(controller.step({}, 300))
        self.assertIsNone(controller.step({'actual_yaw': 10}, None))

    def test_same_frame_cx_cy_and_z_update_both_targets_without_translation(self):
        self.publish()
        command = self.task.step(100.1, .05)
        self.assertEqual(command['yaw'], 30.)
        self.assertEqual((command['depth'], command['surge'], command['sway'], command['stop']),
                         (40., 0., 0., 0))
        self.assertEqual(self.task.last_observation['z_deg'], 12.)
        self.assertIn('CV_Z=+12.000deg', command['note'])
        self.assertEqual(self.task.last_observation['target_id'], 3)
        self.ctx.tel = dict(actual_yaw=15., actual_depth_cm=48.)
        self.publish(frame=2, cy=220., z=-70.)
        command = self.task.step(100.2, .05)
        self.assertEqual(command['yaw'], 35.)
        self.assertEqual(command['depth'], 68.)
        self.assertEqual(self.task.last_observation['cy'], 220.)
        self.assertIn('target_depth=68.00cm', command['note'])
        self.assertEqual(self.task.last_observation['z_deg'], -70.)

    def test_missing_cv_pose_never_blocks_original_cx_control(self):
        self.publish(z=None)
        command = self.task.step(100.1, .05)
        self.assertEqual(command['yaw'], 30.)
        self.assertIsNone(self.task.last_observation['z_deg'])
        self.assertIn('CV_Z=unavailable', command['note'])

    def test_duplicate_or_old_frame_never_recomputes_a_command(self):
        self.publish()
        self.assertEqual(self.task.step(100.1, .05)['yaw'], 30.)
        self.ctx.tel['actual_yaw'] = 20.
        self.assertTrue(self.task.step(100.2, .05)['paused'])
        self.assertTrue(self.task.step(100.6, .05)['paused'])
        self.assertIsNone(self.task.last_observation)

    def test_unavailable_or_invalid_observation_never_completes_the_task(self):
        self.assertTrue(self.task.step(100.1, .05)['paused'])
        self.path.write_text('{')
        self.assertTrue(self.task.step(100.1, .05)['paused'])
        mutations = ((lambda r: r.update(stage='Task1')), (lambda r: r.update(status='model_error')),
                     (lambda r: r.update(capture_ts=101.)),
                     (lambda r: r['door'].update(has_target=False)),
                     (lambda r: r['door'].update(img_w=1280)),
                     (lambda r: r['door'].update(center_px=[float('nan'), 240.])))
        for change in mutations:
            value = record()
            change(value)
            self.path.write_text(json.dumps(value))
            self.assertTrue(self.task.step(100.1, .05)['paused'])

    def test_waits_for_both_telemetry_fields_and_uses_current_actual_depth(self):
        self.ctx.tel = {}
        self.task.enter(100.)
        self.publish()
        self.assertTrue(self.task.step(100.1, .05)['paused'])
        self.ctx.tel = dict(actual_depth_cm=42.)
        self.assertTrue(self.task.step(100.2, .05)['paused'])
        self.ctx.tel.update(actual_yaw=10., actual_depth_cm=48.)
        self.publish(frame=2)
        command = self.task.step(100.3, .05)
        self.assertEqual((command['yaw'], command['depth']), (30., 48.))
        self.ctx.tel['actual_yaw'] = float('nan')
        self.publish(frame=3)
        self.assertTrue(self.task.step(100.4, .05)['paused'])

    def test_mission_testmode_and_real_binary_framing_apply_mirror_once(self):
        from mission import Mission
        from test_mode.test_runner import TestMode
        import link_stm32 as protocol
        for mirror in (False, True):
            with self.subTest(mirror=mirror):
                sent = []
                mode_ctx = SimpleNamespace(log=lambda _: None,
                    cfg=SimpleNamespace(STM32_POLL_HZ=0), estop_latch=False, task_pids=None,
                    link_stm32=SimpleNamespace(send=lambda data, note: sent.append(data)))
                mode = TestMode(mode_ctx)
                mode.yaw_mirror = mirror
                mode.mission = Mission(SimpleNamespace(STAGE_TABLE=DOOR_TABLE,
                                                       AUV_SHM_DIR=self.directory.name),
                                       vision=SimpleNamespace(shm_dir=self.directory.name))
                raw_tel = bytearray(44)
                struct.pack_into('<h', raw_tel, 8, 1000)
                struct.pack_into('<H', raw_tel, 42, 4000)
                mode.on_downlink(protocol.parse_telemetry(raw_tel))
                self.publish(cy=220.)
                mode.tick(100.1, .05)
                self.assertEqual(json.loads((Path(self.directory.name)/'momo_stage.json').read_text())['stage'],
                                 'PassGate')
                expected = protocol.frame_motion(yaw_deg=-30. if mirror else 30., depth_cm=60.,
                                                surge=0., sway=0., stick_stop=False)
                self.assertEqual(sent, [expected])
                mode.tick(100.2, .05)  # same frame: no second control update
                self.assertEqual(sent, [expected])
                self.assertFalse(mode.mission.done)
                mode.ctx.estop_latch = True
                self.publish(frame=2)
                mode.tick(100.3, .05)
                self.assertEqual(sent, [expected])
                mode.ctx.estop_latch = False
                mode.on_exit(0)
                self.assertEqual(sent[-1], protocol.frame_motion(stick_stop=True))

    def test_original_depth_formula_gain_and_missing_inputs(self):
        controller = GateDepthController()
        for depth, cy, expected in ((40, 240, 40), (40, 220, 60), (40, 260, 20),
                                   (40, 0, 280), (40, 480, -200)):
            with self.subTest(depth=depth, cy=cy):
                self.assertEqual(controller.step({'actual_depth_cm': depth}, cy), expected)
        self.assertEqual(controller.K_DEPTH, 1.)
        self.assertEqual(calc_target_depth(40, 220, .25), 45.)
        self.assertIsNone(controller.step({}, 220))
        self.assertIsNone(controller.step({'actual_depth_cm': 40}, None))
        for depth in (None, float('nan'), float('inf')):
            self.assertIsNone(controller.step({'actual_depth_cm': depth}, 220))

    def test_each_new_frame_recalculates_both_axes_from_actual_telemetry(self):
        self.publish(cx=340., cy=220.)
        command = self.task.step(100.1, .05)
        self.assertEqual((command['yaw'], command['depth']), (-10., 60.))
        self.ctx.tel.update(actual_yaw=-8., actual_depth_cm=55.)
        self.publish(frame=2, cx=320., cy=260.)
        command = self.task.step(100.2, .05)
        self.assertEqual((command['yaw'], command['depth']), (-8., 35.))
        self.assertEqual(DOOR_TABLE, [GateYawCxTask])

    def test_invalid_cy_or_depth_suppresses_combined_command(self):
        for index, cy in enumerate((None, float('nan'), float('inf'), -1., 481.), 1):
            self.publish(frame=index, cy=cy)
            self.assertTrue(self.task.step(100.1, .05)['paused'])
        for index, depth in enumerate((None, float('nan'), float('inf')), 10):
            self.ctx.tel['actual_depth_cm'] = depth
            self.publish(frame=index, cy=220.)
            self.assertTrue(self.task.step(100.1, .05)['paused'])

    def test_depth_formula_is_not_clipped_by_task_wrapper(self):
        self.publish(cy=480.)
        command = self.task.step(100.1, .05)
        self.assertEqual(command['depth'], -200.)
        self.publish(frame=2, cy=0.)
        command = self.task.step(100.2, .05)
        self.assertEqual(command['depth'], 280.)
        # Only the existing frame_motion layer clamps to protocol depth limits.
        import link_stm32 as protocol
        frame = protocol.frame_motion(yaw_deg=command['yaw'], depth_cm=command['depth'], depth_max_cm=200.)
        _, payload = protocol.FrameParser().feed(frame)[0]
        self.assertEqual(struct.unpack_from('<H', payload, 6)[0], 20000)

    def test_registration_is_test_only_and_import_order_has_no_cycle(self):
        for first in ('mission', 'task_config'):
            code = ('import sys; sys.path[:0]=sys.argv[1:]; import ' + first + '; '
                    'import task_config; from mission import Stage; '
                    'from test_mode import test_config; '
                    'from task.task_door.gate_yaw_cx_ctrl import DOOR_TABLE; '
                    'assert task_config.DOOR_TABLE == DOOR_TABLE == test_config.DOOR_TABLE; '
                    'assert issubclass(DOOR_TABLE[0], Stage); '
                    'assert DOOR_TABLE[0] not in task_config.STAGE_TABLE')
            result = subprocess.run([sys.executable, '-c', code, str(ROOT/'config'),
                                     str(ROOT/'src/to32/move_test')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
