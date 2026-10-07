# -*- coding: utf-8 -*-
"""test_runner.py —— 测试模式（TEST）实现：开关开启时接管 AUV 模式位，跑可配置测试表

生效链路（2026-10-06）：
    to32/mode_dispatcher._register_modes()
      → 读 test_mode/test_config.py 的 TEST_MODE_ENABLED
      → True : 本文件的 TestMode 注册到 MODE_AUV（替代内联 AuvModeStub）
      → False/文件缺失/加载失败 : 完全回退标准 AUV 壳（现有行为零变化）

测试表来源（2026-10-06 改版）：test_config.TEST_TABLE —— Stage 类的**列表表达式**，
    单阶段/多阶段/任意混拼都在 test_config 一行写定（写法同 task_config.STAGE_TABLE），
    本文件只负责驱动，不做任何档位解析。内置观测回显项 EchoObs 也定义在 test_config。

与标准 AUV 模式壳（move_test/mode_auv.py AuvMode）的差异（其余全部一致）：
    - 任务表取 test_config.TEST_TABLE（测试内容与正式任务 task_config.STAGE_TABLE 互不影响）
    - on_exit 主动发一帧停推 0x09（stick_stop=1，与 Mission 收尾 _StopCmd 同语义，
      急停锁存期间不发）—— 测试中途切回 ROV 不留残余推力
    - 可选 TEST_LOOP：测试表跑完自动从头重跑（间隔 0.5s），便于重复观测
    - 0x09 日志前缀 TEST: 与 AUV: 区分

随时切回 ROV：上位机 $CMD.mode=0 → dispatcher 常规切换路径（TestMode.on_exit 停推
→ RovMode.on_enter 接管），本文件无需任何特殊处理；再切 AUV（mode=1）会重新
on_enter 从头进测试模式。模式记忆文件照常生效（记录的是 mode id，不受接管影响）。

运行前提（继承各测试项自身口径）：测试项的完成判据/兜底由各自 Stage 决定
（t_function 2026-10-06 口径）；判据失效的死等行为属测试项自身设计，本框架不干预。
"""
import os
import sys
import types

# 自带 sys.path 注入（幂等；同 mode_auv / t_task* 约定）：to32 根（mode_base/link_stm32）、
# move_test（task_config/obs/mission）、task（t_function、各测试脚本）、本目录（test_config）
_HERE = os.path.dirname(os.path.abspath(__file__))          # move_test/test_mode
_MV = os.path.dirname(_HERE)                                 # move_test
_TASK = os.path.join(_MV, 'task')                            # move_test/task
_TO32 = os.path.dirname(_MV)                                 # src/to32
for _p in (_TO32, _MV, _TASK, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import link_stm32 as S        # 0x04/0x09 组帧
from mode_base import ModeBase
import task_config as TC      # YAW_MIRROR / AUV_LOG_EVERY_S（测试项参数都在那）
import obs                    # VisionIF / DepthIF（Mission 观测接口）
from mission import Mission, apply_yaw_mirror
import test_config as TCFG    # 本目录：TEST_MODE_ENABLED / TEST_TABLE / TEST_LOOP


class TestMode(ModeBase):
    """测试模式：占用 AUV 模式位；任务表 = test_config.TEST_TABLE，可随时被切回 ROV。"""

    id = -1                      # 注册时由 dispatcher 覆盖为 to32_config.MODE_AUV
    name = "TEST(AUV)"
    desc = "测试模式（test_mode/test_config.py 开关接管 AUV 位）"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.last_tel = None            # 最近一帧下位机遥测（on_downlink 缓存）
        self.mission = None             # 测试状态机：on_enter 重建，on_exit 丢弃
        self._ignore_cnt = 0            # 被忽略的手动杆位帧计数（只提示前 3 帧）
        self._loop_wait_ts = None       # TEST_LOOP 重跑前的等待起点
        self.yaw_mirror = bool(getattr(TC, 'YAW_MIRROR', True))
        self.log_every = float(getattr(TC, 'AUV_LOG_EVERY_S', 1.0))
        self._last_log_ts = 0.0
        self._last_note_stage = ''

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        super().on_enter(prev_id)
        self._ignore_cnt = 0
        self._loop_wait_ts = None
        # 模式帧不被急停闩锁抑制：与固件配合，AUV 语义就位优先（协议 §12.3，同 AuvMode）
        self.send_downlink(S.frame_mode(S.MODE_AUV), "0x04 AUV(测试模式)")
        # 测试表直读 test_config.TEST_TABLE（Stage 类列表，写法同 STAGE_TABLE）；
        # Mission 只从 cfg 读 STAGE_TABLE，测试项内部参数都直接 getattr(TC,...)，
        # 故传一个只带 STAGE_TABLE 的轻量代理，正式任务表不受影响
        table = list(getattr(TCFG, 'TEST_TABLE', []) or [])
        cfg = types.SimpleNamespace(STAGE_TABLE=table)
        self.mission = Mission(cfg, vision=obs.VisionIF(log=self.log),
                               depth=obs.DepthIF(log=self.log), log=self.log, task_pids=self.ctx.task_pids)
        self.log("[TEST] 测试模式接管 AUV 位：test_config.TEST_TABLE 共 %d 项；"
                 "上位机切 ROV(mode=0) 即退出停推" % len(table))
        if not table:
            self.log("[TEST] 警告：TEST_TABLE 为空 —— 进入即 DONE，不下发运动指令"
                     "（去 test_mode/test_config.py 写 TEST_TABLE）")

    def on_exit(self, next_id):
        # 中途被切走（如切回 ROV）：发一帧停推，不留残余推力。
        # stop=1 时固件按停推处理（与 Mission 收尾 _StopCmd 同语义）；
        # 急停锁存期间 0x09 一个字节都不能发（安全红线，同 AuvMode）。
        if not self.ctx.estop_latch:
            try:
                self.send_downlink(
                    S.frame_motion(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0,
                                   depth_cm=0.0, surge=0.0, sway=0.0, stick_stop=1),
                    "0x09 测试模式退出停推")
            except Exception as e:
                self.log("[TEST] 退出停推帧下发异常: %r" % e)
        self.mission = None              # 丢弃状态机：计时器/锁存不残留到下次进入
        super().on_exit(next_id)

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):
        """测试模式忽略手动杆位（只记录前 3 帧，同 AuvMode）"""
        self._ignore_cnt += 1
        if self._ignore_cnt <= 3:
            self.log("[TEST] 忽略手动杆位 surge=%.2f sway=%.2f heave=%.2f yaw=%.2f"
                     % (cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))

    # ---------------- 周期 ----------------
    def tick(self, now, dt):
        self.pump_telemetry(now)         # 测试期间由本模式掌握遥测回传节拍
        if self.ctx.estop_latch:         # 安全红线：急停闩锁期间一个字节都不能发
            return
        if self.mission is None:
            return
        cmd = self.mission.step(now, dt, self.last_tel)
        if not cmd:                      # None = 测试表已跑完（收尾 stop 帧已发）
            self._maybe_loop(now)        # 按 TEST_LOOP 决定是否重跑
            return
        if cmd.get('paused'):
            return
        yaw = apply_yaw_mirror(cmd["yaw"], self.yaw_mirror)
        self.send_downlink(
            S.frame_motion(pitch_deg=0.0, yaw_deg=yaw, roll_deg=0.0,
                           depth_cm=cmd["depth"],    # 目标深度(cm，固件内闭环)
                           surge=cmd["surge"],
                           sway=cmd["sway"],
                           stick_stop=cmd["stop"]),  # 仅收尾停推时置 1
            self._note(now, cmd))

    def _maybe_loop(self, now):
        """测试表跑完后的处理：TEST_LOOP=True → 等 0.5s 从头重跑；否则保持 DONE 静默。"""
        if not bool(getattr(TCFG, 'TEST_LOOP', False)) or self.mission is None:
            return
        if not getattr(self.mission, 'done', False):
            return
        if self._loop_wait_ts is None:   # 先让收尾 stop 帧发出去，再等间隔
            self._loop_wait_ts = now
            return
        if now - self._loop_wait_ts < 0.5:
            return
        self._loop_wait_ts = None
        self.log("[TEST] TEST_LOOP=True —— 重跑测试表")
        self.on_enter(self.id)           # 重建状态机从头跑（重发 0x04 无害，同 id 幂等）

    def _note(self, now, cmd):
        """0x09 日志标注：阶段切换立即打，同阶段内按 AUV_LOG_EVERY_S 节流（同 AuvMode）。"""
        if cmd["stage"] != self._last_note_stage:
            self._last_note_stage = cmd["stage"]
            self._last_log_ts = now
            return "0x09 TEST:%s %s" % (cmd["stage"], cmd["note"])
        if (now - self._last_log_ts) >= self.log_every:
            self._last_log_ts = now
            return "0x09 TEST:%s %s" % (cmd["stage"], cmd["note"])
        return ""

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):
        """缓存最近一帧遥测供状态机判据；解析统一交给 tel_builder（同 AuvMode）"""
        self.last_tel = tel

    def build_telemetry(self):
        """返回 None = 回落 tel_builder 统一映射（同 AuvMode，$TEL 不断流）"""
        return None
