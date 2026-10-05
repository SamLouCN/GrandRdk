# -*- coding: utf-8 -*-
"""离线台架 —— 把 runner + 假件装起来，按固定节拍跑完整个任务（或单段）

用法:
    sim = Sim(cfg)                       # 建仿真
    sim.run(max_s=600)                   # 跑完（或超时）
    print(sim.stages())                  # 走过的阶段序列
    print(sim.report())                  # 人读的汇总

★ 这个台架测的是**流程与判据**，不是水动力：
  深度是"以有限速率逼近目标"的一阶模型，视觉是按 test_config.VISION_SCRIPT 给的剧本。
  它能抓出跳转顺序错、判据永远不满足、超时不生效、测试模式过滤错这类结构性 bug，
  抓不出"推力和实际速度对不上"—— 后者必须下水标定。

★ 仿真参数（dt / 池深 / 垂向速率 / 深度源是否可用 / 舵机是否就绪 / 视觉剧本）
  **全部读 test_config.py 的 SIM 与 VISION_SCRIPT**，与板端共用同一个配置文件。
  显式传参（如 Sim(dt=0.02)）优先级高于 test_config。
"""
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))                 # .../move_test/auv_task/tests
_PKG_PARENT = os.path.dirname(os.path.dirname(_HERE))              # .../move_test
for _p in (_PKG_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fakes import (default_cfg, sim_defaults, FakeVision, FakeDepth, FakeViskf,  # noqa: E402
                   FakeServo, make_obs, script_obs, vk_ok, LogSink, RecNotifier)
from auv_task import TaskRunner, TestCfg  # noqa: E402
from auv_task.notify import NoMsg  # noqa: E402


class Sim(object):
    """一次离线仿真"""

    def __init__(self, cfg=None, dt=None, pool_m=None, viskf=None, servo=None,
                 test=None, echo=None, script=None, notify=None):
        s = sim_defaults()                               # 读 test_config.SIM
        # ★ notify：状态提示通道（$MSG）。
        #   离线台架**绝不真发 UDP**（Windows 上往没监听的端口发会抛 ConnectionResetError），
        #   所以不显式传时一律用 NoMsg 静默通道；要断言提示内容就传 RecNotifier。
        self.notify = notify if notify is not None else NoMsg()
        self.cfg = cfg or default_cfg()
        self.dt = float(s['dt'] if dt is None else dt)
        self.echo = bool(s['echo_log'] if echo is None else echo)
        self.logs = LogSink(echo=self.echo)
        self.depth = FakeDepth(pool_m=(s['pool_m'] if pool_m is None else pool_m),
                               rate_mps=s['depth_rate_mps'], d0=s['start_depth_m'])
        self.depth.ok = bool(s['depth_ok'])              # False = 测"深度源掉线"的降级路径
        self.servo = servo if servo is not None else FakeServo(ready=bool(s['servo_ready']))
        if viskf is not None:
            self.viskf = viskf
        elif s['viskf_ok']:                              # 模拟"图像卡尔曼可用"
            self.viskf = FakeViskf(lambda now: vk_ok(e_x=0.01, s_n=0.5))
        else:                                            # 默认恒不可用 → 走原始像素伺服
            self.viskf = FakeViskf(None)
        self.vision = FakeVision(lambda cam, lab, now: self._vis(cam, lab, now))
        self.script = script or {}                       # 阶段名 -> obs 生成器（覆盖剧本，最高优先）
        self.runner = TaskRunner(self.cfg, vision=self.vision, depth=self.depth,
                                 viskf=self.viskf, log=self.logs, servo=self.servo,
                                 test=test, notify=self.notify)
        self.t = time.time()
        self.trace = []                                  # (t, stage, cmd)
        self.stage_seq = []                              # 阶段变化序列
        self.mode_requests = []                          # 切模式请求

    # ---------------------------------------------------------------- 视觉剧本
    def _vis(self, cam, label, now):
        """按当前阶段给观测

        优先级：外部 script > test_config.VISION_SCRIPT（script_obs）> None
        """
        st = self.runner.stage
        el = self.runner.ctx.elapsed(now) if self.runner.ctx else 0.0
        if st in self.script:                            # 外部指定的剧本优先
            g = self.script[st]
            return g(now, el) if callable(g) else g
        return script_obs(st, el, label=label)

    # ---------------------------------------------------------------- 节拍
    def step(self, tel=None):
        """跑一拍，返回 cmd（None = 不下发）"""
        cmd = self.runner.step(self.t, self.dt, tel)
        if cmd is not None:
            self.depth.set_target(float(cmd['depth']) / 100.0)     # 把下发的目标深度喂回仿真
            if not self.stage_seq or self.stage_seq[-1] != cmd['stage']:
                self.stage_seq.append(cmd['stage'])
            self.trace.append((self.t, cmd['stage'], cmd))
        mr = self.runner.pop_mode_request()
        if mr is not None:
            self.mode_requests.append((self.t, mr))
        self.t += self.dt
        return cmd

    def run(self, max_s=None, tel=None, stop_stage=None):
        """跑到结束（cmd=None）或超时；返回实际耗时 s"""
        s = sim_defaults()
        max_s = float(s['max_s'] if max_s is None else max_s)
        t_end = self.t + max_s
        while self.t < t_end:
            cmd = self.step(tel)
            if cmd is None:
                break
            if stop_stage is not None and cmd['stage'] == stop_stage:
                break
        return (self.t - (self.trace[0][0] if self.trace else self.t))

    # ---------------------------------------------------------------- 报告
    def stages(self):
        """走过的阶段序列（去重相邻）"""
        out = []
        for _, s, _ in self.trace:
            if not out or out[-1] != s:
                out.append(s)
        return out

    def stage_durs(self):
        """阶段 -> 停留秒数"""
        d = {}
        for t, s, _ in self.trace:
            d[s] = d.get(s, 0.0) + self.dt
        return d

    def messages(self):
        """收集到的状态提示（$MSG）列表：[(level, code, text, stage), ...]

        ★ 只有显式传了记录型 notify（如 RecNotifier）才有内容；
          不传时台架用 NoMsg 静默通道，这里是空的（离线台架绝不真发 UDP）。
        """
        n = self.notify
        return list(getattr(n, 'items', []) or [])

    def report(self, max_rows=60, show_msg=True):
        """人读的汇总报告"""
        lines = []
        if show_msg and self.messages():
            lines.append('== 状态提示（$MSG，上位机终端会看到这些）==')
            for lv, code, text, st in self.messages():
                lines.append('  %-5s %-12s %-14s %s' % (lv, code, st, text))
            lines.append('')
        lines.append('== 阶段序列 ==')
        lines.append(' → '.join(self.stages()))
        lines.append('')
        lines.append('== 各阶段停留 ==')
        for s, v in self.stage_durs().items():
            lines.append('  %-16s %6.2f s' % (s, v))
        lines.append('')
        lines.append('== 模式切换请求 ==')
        lines.append('  %s' % (self.mode_requests or '无'))
        lines.append('')
        lines.append('== 关键日志 ==')
        for l in self.logs.lines[:max_rows]:
            lines.append('  ' + l)
        return '\n'.join(lines)


# ================================================================ 两种起跑方式
def from_test_config(cfg=None, dt=None, echo=None, **kw):
    """★ 完全按 test_config.py 跑（TEST_ENABLED / TEST_STAGES / STAGE_PARAMS）

    这是最常用的入口：改完 test_config 直接跑，看选段与参数覆盖对不对。
    """
    cfg = cfg or default_cfg()
    sim = Sim(cfg=cfg, dt=dt, echo=echo, **kw)
    sim.run()
    return sim


def single_stage(stage, cfg=None, dt=None, max_s=None, echo=None, notify=None):
    """★ 单段调试：只跑一个阶段（其余被测试模式过滤掉）

    这就是"我可以独立测试每一阶段任务的实现效果"的落地方式：
    开测试模式 + 把选段写成那一段，runner 会自动从它开始，
    跑完它就直接进 SURFACE（FORCED，删不掉）→ DONE。
    """
    cfg = cfg or default_cfg()
    cfg.AUV_TEST_MODE = True
    cfg.AUV_TEST_STAGES = [str(stage).upper()]
    t = TestCfg(cfg, log=(lambda m: None))
    sim = Sim(cfg=cfg, dt=dt, echo=echo, test=t, notify=notify)
    sim.run(max_s=max_s)
    return sim


def full_run(cfg=None, dt=None, max_s=None, echo=None, notify=None, **kw):
    """全流程干跑（验收路径：AUV_TEST_MODE=False）"""
    cfg = cfg or default_cfg()
    cfg.AUV_TEST_MODE = False
    sim = Sim(cfg=cfg, dt=dt, echo=echo, notify=notify, **kw)
    sim.run(max_s=max_s)
    return sim
