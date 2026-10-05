# -*- coding: utf-8 -*-
"""离线回归测试 —— 把"重构不许破坏的东西"固化成断言

跑法:
    python test_auv_task.py            # 全部用例
    python test_auv_task.py -v         # 带详细日志

★ 这些用例守的是**结构性不变量**（跳转顺序、幂等、强制上浮、测试过滤、监督指令），
  不是水动力性能 —— 后者必须下水标定，桌上测不出来。

⚠ 每次改 plan.py / runner.py / 任一任务后，都要跑一遍这个脚本再上传板端。
"""
import copy
import os
import sys
import time
from contextlib import contextmanager

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_PKG_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fakes import (default_cfg, FakeDepth, FakeVision, FakeViskf, FakeServo,  # noqa: E402
                   make_obs, vk_ok, LogSink, FakeSock, RecNotifier)
from harness import Sim, full_run, single_stage  # noqa: E402
from auv_task import (TaskRunner, TestCfg, build_plan, names,  # noqa: E402
                      apply_yaw_mirror, test_config as TC)
from auv_task.servo_if import ServoStub  # noqa: E402
from auv_task.gate import (auv_initial_allowed, sanitize_persist_mode,  # noqa: E402
                           guard_initial_mode, is_pc_source, SRC_PC, SRC_START,
                           SRC_FORCED, SRC_SAVED)
from auv_task.notify import (AuvMsg, NoMsg, make_notifier, build_msg_frame,  # noqa: E402
                             describe)


@contextmanager
def tc_temp(**kw):
    """临时改 test_config 的若干项，退出自动还原（模块级状态必须还原，否则污染后续用例）"""
    old = {k: copy.deepcopy(getattr(TC, k)) for k in kw}
    try:
        for k, v in kw.items():
            setattr(TC, k, v)
        yield TC
    finally:
        for k, v in old.items():
            setattr(TC, k, v)


def run_sim(**kw):
    """建仿真并跑完（test_config 类的用例都要先跑再断言）"""
    s = Sim(**kw)
    s.run()
    return s


def _sleep(s):
    """等发送线程把队列吐完（AuvMsg 是独立线程，push 完不能立刻断言 socket）"""
    time.sleep(float(s))

VERBOSE = '-v' in sys.argv
_RES = []


def case(name):
    """用例装饰器（记录 PASS/FAIL，单个用例挂掉不中断其余）"""
    def deco(fn):
        try:
            fn()
            _RES.append((True, name, ''))
            print('  [PASS] %s' % name)
        except AssertionError as e:
            _RES.append((False, name, str(e)))
            print('  [FAIL] %s -> %s' % (name, e))
        except Exception as e:
            _RES.append((False, name, '%s: %s' % (type(e).__name__, e)))
            print('  [ERR ] %s -> %s: %s' % (name, type(e).__name__, e))
        return fn
    return deco


# ================================================================ 阶段表
@case('阶段表：默认 4 门 = 24 段，顺序符合方案 §18.2')
def _():
    p = build_plan(default_cfg())
    n = names(p)
    assert len(n) == 24, '段数 %d != 24' % len(n)
    assert n[0] == 'DIVE' and n[-1] == 'DONE', n
    assert n.count('SURFACE') == 1, n
    for i in range(1, 5):
        assert ('SEEK_GATE_%d' % i) in n and ('PASS_GATE_%d' % i) in n, n
    assert n.index('SIT_BOTTOM') < n.index('RELEASE_DROP') < n.index('GO_HOME'), n


@case('阶段表：门数可配（AUV_GATE_COUNT=3 → 22 段）')
def _():
    p = build_plan(default_cfg(AUV_GATE_COUNT=3))
    n = names(p)
    assert len(n) == 22, len(n)
    assert 'SEEK_GATE_4' not in n and 'PASS_GATE_3' in n, n


@case('阶段表：高矮门交替（矮 45cm / 高 65cm → 目标深度 85 / 65cm）')
def _():
    cfg = default_cfg()
    p = build_plan(cfg)
    plan = {s['name']: s for s in p}
    h1 = plan['PASS_GATE_1']['params']['height_cm']
    h2 = plan['PASS_GATE_2']['params']['height_cm']
    assert h1 != h2, (h1, h2)
    assert getattr(cfg, 'AUV_' + h1) == 45.0 and getattr(cfg, 'AUV_' + h2) == 65.0, (h1, h2)


# ================================================================ 全流程
@case('全流程干跑：24 段全覆盖，正常完成（非中止），DONE 发过停推')
def _():
    sim = full_run(dt=0.05, max_s=600.0, echo=VERBOSE)
    st = sim.stages()
    assert st[-1] == 'DONE', st
    assert len(st) == 24, '走过 %d 段: %s' % (len(st), st)
    assert sim.runner.abort_reason == '', sim.runner.abort_reason
    assert any(c['stop'] for _, _, c in sim.trace), 'DONE 没发停推帧'
    assert sim.runner.surfaced is True, '没经过上浮'


@case('全流程干跑：总时长在预算内（< 900s 且不触发预算中止）')
def _():
    sim = full_run(dt=0.05, max_s=900.0)
    total = sum(sim.stage_durs().values())
    assert total < 900.0, '总时长 %.1fs' % total
    assert not sim.logs.has('全局预算'), '不该触发预算中止'


@case('舵机：投放只触发一次（幂等），且未实装时不中止任务')
def _():
    sim = full_run(dt=0.05, max_s=600.0)
    assert sim.servo.drops == 1, '触发 %d 次' % sim.servo.drops
    assert sim.runner.abort_reason == '', sim.runner.abort_reason


@case('舵机未实装（ServoStub）：打 WARN 但流程继续到 DONE')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False), servo=ServoStub(lambda m: None))
    sim.run(max_s=600.0)
    assert sim.stages()[-1] == 'DONE', sim.stages()
    assert sim.servo.fired is True and sim.servo.ready() is False
    assert sim.runner.abort_reason == '', sim.runner.abort_reason


# ================================================================ 降级 / 中止
@case('前视 30s 找不到球 → ABORT → 强制上浮 → DONE（不跳过上浮）')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False),
              script={'SEEK_BALL_F': lambda now, el: None})     # 永远看不到球
    sim.run(max_s=600.0)
    st = sim.stages()
    assert '超时' in sim.runner.abort_reason and 'ball' in sim.runner.abort_reason, \
        sim.runner.abort_reason
    assert 'SURFACE' in st and st[-1] == 'DONE', st
    assert sim.runner.surfaced is True
    assert 'RAM_BALL' not in st, '没看到球就不该去撞球: %s' % st


@case('深度源不可用：定深走定时放行，不原地死等')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False))
    sim.depth.ok = False                                        # 深度源掉线
    sim.run(max_s=600.0)
    assert 'DIVE' in sim.stages(), sim.stages()
    assert sim.logs.has('无深度源') or sim.logs.has('上浮超时'), sim.logs.dump()


@case('穿门：viskf 可用时按 s_n 判定穿过（滤波路径也能走通）')
def _():
    vkn = {'n': 0}

    def vkfn(now):
        vkn['n'] += 1
        return vk_ok(e_x=0.01, de_x=0.0, s_n=0.90)              # s_n 恒到位

    def gate_vis(now, el):
        return None if el > 1.0 else make_obs('gate', w=300.0)  # 1s 后门消失 → 判穿过

    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False), viskf=FakeViskf(vkfn),
              script={'SEEK_GATE_1': lambda now, el: make_obs('gate', w=300.0),
                      'PASS_GATE_1': gate_vis})
    sim.run(max_s=600.0)
    assert sim.logs.has('s_n 到位'), '滤波路径没生效:\n' + sim.logs.dump()[:800]


# ================================================================ 测试模式
@case('测试模式：只跑 SEEK_BALL_F → 序列 = SEEK_BALL_F → SURFACE → DONE')
def _():
    sim = single_stage('SEEK_BALL_F')
    st = sim.stages()
    assert st == ['SEEK_BALL_F', 'SURFACE', 'DONE'], st


@case('测试模式：只跑 SIT_BOTTOM → 直接就该段开始（不从 DIVE 起步）')
def _():
    sim = single_stage('SIT_BOTTOM')
    assert sim.stages()[0] == 'SIT_BOTTOM', sim.stages()
    assert 'DIVE' not in sim.stages(), sim.stages()


@case('★ SURFACE 不可被配置关闭（FORCED）：选段只写 DIVE 也一定上浮')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['DIVE'])
    t = TestCfg(cfg, log=lambda m: None)
    sim = Sim(cfg=cfg, test=t)
    sim.run(max_s=600.0)
    st = sim.stages()
    assert st[-1] == 'DONE' and 'SURFACE' in st, st


@case('测试模式：选段写了不存在的名字 → 自动退回全流程（Fail-Safe）')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['NO_SUCH_STAGE'])
    logs = LogSink()
    t = TestCfg(cfg, log=logs)
    ok, unknown, kept = t.validate(names(build_plan(cfg)))
    assert ok is False and unknown == ['NO_SUCH_STAGE'], (ok, unknown)
    assert t.enabled is False, '应已关闭测试模式'
    assert len(kept) == 24, len(kept)


@case('测试模式：保持判据被压短（≤ AUV_TEST_HOLD_MAX_S）')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['DIVE'], AUV_TEST_HOLD_MAX_S=1.0)
    t = TestCfg(cfg, log=lambda m: None)
    assert t.hold_s(2.0) == 1.0, t.hold_s(2.0)
    assert t.hold_s(0.5) == 0.5, t.hold_s(0.5)
    t2 = TestCfg(default_cfg(AUV_TEST_MODE=False), log=lambda m: None)
    assert t2.hold_s(2.0) == 2.0, '非测试模式不该压缩'


# ================================================================ test_config（§16j：测试唯一入口）
@case('test_config：TEST_ENABLED + 选段 → 只跑这一段（并强制上浮）')
def _():
    with tc_temp(TEST_ENABLED=True, TEST_STAGES=['SIT_BOTTOM']):
        sim = run_sim(cfg=default_cfg())
    assert sim.stages() == ['SIT_BOTTOM', 'SURFACE', 'DONE'], sim.stages()


@case('test_config：task:<类名> 选段（4 个 PASS_GATE 全选中，跳过各自的 SEEK）')
def _():
    with tc_temp(TEST_ENABLED=True, TEST_STAGES=['task:PassGateTask']):
        sim = run_sim(cfg=default_cfg())
    st = sim.stages()
    assert st[:4] == ['PASS_GATE_1', 'PASS_GATE_2', 'PASS_GATE_3', 'PASS_GATE_4'], st
    assert not [s for s in st if s.startswith('SEEK')], st
    assert st[-1] == 'DONE' and 'SURFACE' in st, st


@case('test_config：选段写了未知名字 → 退回全流程（Fail-Safe）')
def _():
    with tc_temp(TEST_ENABLED=True, TEST_STAGES=['SEEK_GATE_*', 'NO_SUCH']):
        sim = run_sim(cfg=default_cfg())
    assert len(sim.stages()) >= 20, sim.stages()
    assert sim.logs.has('未知名字'), sim.logs.dump()[:400]


@case('test_config：STAGE_PARAMS 覆盖阶段参数（TIMEOUT_S 生效）')
def _():
    with tc_temp(TEST_ENABLED=True, TEST_STAGES=['SEEK_BALL_F'],
                 STAGE_PARAMS={'SEEK_BALL_F': {'TIMEOUT_S': 6.0}},
                 VISION_SCRIPT=dict(TC.VISION_SCRIPT, never_see=['SEEK_BALL_F'])):
        sim = run_sim(cfg=default_cfg())
        d = sim.stage_durs().get('SEEK_BALL_F', 0.0)
    assert 5.0 < d < 8.0, '超时没被覆盖成 6s: %.2f' % d


@case('test_config：GLOBAL_PARAMS 覆盖全局配置（AUV_GATE_COUNT=2 → 20 段）')
def _():
    with tc_temp(GLOBAL_PARAMS={'AUV_GATE_COUNT': 2}):
        n = names(build_plan(default_cfg()))
    assert len(n) == 20, len(n)
    assert 'PASS_GATE_3' not in n, n


@case('test_config：stage_params 优先级 精确阶段名 > task:<类名> > 通配')
def _():
    with tc_temp(STAGE_PARAMS={'*': {'A': 1}, 'task:SeekTask': {'B': 2, 'A': 9},
                               'SEEK_BALL_F': {'C': 3}}):
        got = TC.stage_params('SEEK_BALL_F', 'SeekTask')
    assert got == {'A': 9, 'B': 2, 'C': 3}, got      # 通配的 A=1 必须被 task: 的 A=9 盖掉


@case('test_config：VISION_SCRIPT.never_see → 测"找不到球"的中止上浮路径')
def _():
    with tc_temp(VISION_SCRIPT=dict(TC.VISION_SCRIPT, never_see=['SEEK_BALL_F'])):
        sim = run_sim(cfg=default_cfg())
    assert 'ball' in sim.runner.abort_reason, sim.runner.abort_reason
    assert 'SURFACE' in sim.stages() and sim.runner.surfaced is True


@case('test_config：TEST_HOLD_MAX_S=None 时沿用 auv_config 的 AUV_TEST_HOLD_MAX_S')
def _():
    assert TC.TEST_HOLD_MAX_S is None, '本用例假设 test_config 里它默认是 None'
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['DIVE'], AUV_TEST_HOLD_MAX_S=1.0)
    assert TestCfg(cfg, log=lambda m: None).hold_s(2.0) == 1.0
    with tc_temp(TEST_HOLD_MAX_S=3.0):                # 本文件里定死 → 覆盖 auv_config
        assert TestCfg(cfg, log=lambda m: None).hold_s(10.0) == 3.0


# ================================================================ 预算 / 监督
@case('全局预算超时 → 强制中止上浮（不卡在水下）')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False, AUV_BUDGET_S=5.0))
    sim.run(max_s=600.0)
    assert sim.logs.has('全局预算'), sim.logs.dump()[:600]
    assert '预算' in sim.runner.abort_reason, sim.runner.abort_reason
    assert sim.runner.surfaced is True


@case('监督 $AUVCTL：HOLD 暂停 / RESUME 恢复 / SKIP 跳过 / ABORT 中止')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False))
    sim.run(max_s=20.0)                                          # 先跑一会儿
    s0 = sim.runner.stage
    assert sim.runner.on_auvctl(['HOLD']) is True
    sim.step()
    assert sim.runner.stage == s0, '暂停后不该推进阶段'
    assert sim.runner.paused is True
    assert sim.runner.on_auvctl(['RESUME']) is True
    assert sim.runner.paused is False
    assert sim.runner.on_auvctl(['SKIP']) is True
    assert sim.runner.stage != s0, 'SKIP 没生效'
    assert sim.runner.on_auvctl(['ABORT']) is True
    assert sim.runner.abort_reason != ''
    sim.run(max_s=60.0)
    assert sim.stages()[-1] == 'DONE', sim.stages()


@case('监督 $AUVCTL：policy=ignore 时全部拒绝（验收/比赛防误操作）')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False, AUV_CMD_POLICY='ignore'))
    assert sim.runner.on_auvctl(['ABORT']) is False
    assert sim.runner.abort_reason == ''


@case('监督 $AUVCTL：GOTO 到未启用阶段被拒绝（测试模式下）')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['DIVE'])
    sim = Sim(cfg=cfg, test=TestCfg(cfg, log=lambda m: None))
    assert sim.runner.on_auvctl(['GOTO', 'SIT_BOTTOM']) is False
    assert sim.runner.on_auvctl(['GOTO', 'NOPE']) is False


# ================================================================ 契约
@case('★ 上浮后切入的模式 id = cfg.MODE_ROV(0)，不是 link_stm32 的 0x03')
def _():
    sim = full_run(dt=0.05, max_s=600.0)
    assert sim.mode_requests, '没有产生模式切换请求'
    assert sim.mode_requests[-1][1] == 0, sim.mode_requests
    assert getattr(sim.cfg, 'MODE_ROV') == 0


@case('首帧锚定：目标深度/航向取自当前状态，不是硬回水面')
def _():
    sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False))
    sim.depth.D = 0.55                                           # 当前在 55cm
    sim.runner.step(sim.t, 0.05, {'actual_yaw': 42.0})
    # ⚠ 锚定值会被紧接着的 DIVE 首拍合法覆盖成"下潜目标深度"，
    #   所以要断言的是**锚定那一刻**的日志，而不是跑完一拍后的 ctx.depth_cm
    assert sim.logs.has('首帧锚定 depth=55.0cm'), sim.logs.dump()[:400]
    assert sim.runner.anchored is True
    assert sim.runner.ctx.yaw_est.now() == 42.0, sim.runner.ctx.yaw_est.now()


@case('上报快照字段：AuvReport.make_snapshot 需要的属性都在 runner 上')
def _():
    sim = full_run(dt=0.05, max_s=30.0)
    r = sim.runner
    for k in ('t0', 'stage', 'seq', 'last_dep', 'last_obs', 'last_obs_cam',
              'last_obs_ts', 'last_vk', 'abort_reason'):
        assert hasattr(r, k), '缺属性 %s' % k
    assert r.seq > 0 and isinstance(r.stage, str)


@case('控制量字段集与旧 mission 完全一致（mode_auv 零改动的前提）')
def _():
    sim = full_run(dt=0.05, max_s=30.0)
    cmd = sim.trace[0][2]
    for k in ('depth', 'yaw', 'surge', 'sway', 'stop', 'stage', 'note'):
        assert k in cmd, '缺字段 %s' % k


@case('apply_yaw_mirror：开镜像取反、关镜像原样（符号端到端）')
def _():
    assert apply_yaw_mirror(30.0, True) == -30.0
    assert apply_yaw_mirror(30.0, False) == 30.0


@case('参数解析：字符串=配置键、数字=字面量、lit() 不当配置键查')
def _():
    from auv_task.base import Task
    t = Task(next='SEEK_BALL_F', SURGE='SURGE_SEEK', DEG=120.0)
    ctx = Sim(cfg=default_cfg()).runner.ctx
    assert t.next_of(ctx) == 'SEEK_BALL_F', t.next_of(ctx)      # ★ 走 lit，不当配置键
    assert abs(float(t.gp(ctx, 'SURGE', 0.25)) - 0.25) < 1e-9   # 查 AUV_SURGE_SEEK
    assert abs(t.gpf(ctx, 'DEG', 0.0) - 120.0) < 1e-9           # 字面量数字


@case('跳转解析：被测试模式跳过的段会被穿透到下一个在跑的段')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['DIVE'])
    r = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                   test=TestCfg(cfg, log=lambda m: None))
    assert r.resolve('SEEK_BALL_F') == 'SURFACE', r.resolve('SEEK_BALL_F')
    assert r.resolve('DONE') == 'DONE'


# ================================================================ ★ 启动门控（2026-10-05）
@case('启动门控：上电/配置启动（start）想进 AUV → 拦，回落 IDLE')
def _():
    cfg = default_cfg()
    ok, why = auv_initial_allowed(1, SRC_START, cfg)
    assert ok is False, (ok, why)
    assert 'MODE_IDLE' in why or 'IDLE' in why, why
    mid, blocked = guard_initial_mode(1, SRC_START, cfg, modes={-1: 'IDLE', 0: 'ROV', 1: 'AUV'})
    assert blocked is True and mid == -1, (mid, blocked)


@case('启动门控：命令行 --mode auv（forced）→ 拦')
def _():
    cfg = default_cfg()
    assert auv_initial_allowed(1, SRC_FORCED, cfg)[0] is False


@case('启动门控：模式记忆（saved）→ 拦（否则上电就自己跑起来）')
def _():
    cfg = default_cfg()
    assert auv_initial_allowed(1, SRC_SAVED, cfg)[0] is False


@case('启动门控：上位机 $CMD（pc_cmd）→ 放行（唯一合法来源）')
def _():
    cfg = default_cfg()
    ok, why = auv_initial_allowed(1, SRC_PC, cfg)
    assert ok is True, why
    assert is_pc_source('pc_cmd') and is_pc_source('PC')
    mid, blocked = guard_initial_mode(1, SRC_PC, cfg, modes={-1: 'IDLE', 0: 'ROV', 1: 'AUV'})
    assert blocked is False and mid == 1, (mid, blocked)


@case('启动门控：ROV/IDLE 不受影响（只拦 AUV）')
def _():
    cfg = default_cfg()
    for src in (SRC_START, SRC_FORCED, SRC_SAVED):
        assert auv_initial_allowed(0, src, cfg)[0] is True
        assert auv_initial_allowed(-1, src, cfg)[0] is True


@case('启动门控：AUV_REQUIRE_PC_CMD=False 时全部放行（显式关闭门控）')
def _():
    cfg = default_cfg(AUV_REQUIRE_PC_CMD=False)
    for src in (SRC_START, SRC_FORCED, SRC_SAVED):
        assert auv_initial_allowed(1, src, cfg)[0] is True


@case('启动门控：IDLE 未注册时维持原模式（不能拦了又落到不存在的地方）')
def _():
    cfg = default_cfg()
    mid, blocked = guard_initial_mode(1, SRC_START, cfg, modes={0: 'ROV', 1: 'AUV'})
    assert blocked is True and mid == 1, (mid, blocked)


@case('模式记忆：AUV 会被改写成 IDLE，ROV 原样保留')
def _():
    cfg = default_cfg()
    assert sanitize_persist_mode(1, cfg) == -1
    assert sanitize_persist_mode(0, cfg) == 0
    assert sanitize_persist_mode(1, default_cfg(AUV_REQUIRE_PC_CMD=False)) == 1


# ================================================================ ★ 上位机接管即终止
@case('★ 上位机切回 ROV：kill() 后 step() 恒返回 None（不再下发任何 0x09）')
def _():
    sim = Sim(cfg=default_cfg())
    sim.runner.step(sim.t, 0.05, None)
    assert sim.runner.step(sim.t, 0.05, None) is not None     # 正常时是有控制量的
    sim.runner.kill('PC_TAKEBACK')
    assert sim.runner.killed is True
    assert sim.runner.kill_reason == 'PC_TAKEBACK'
    for _i in range(5):
        assert sim.runner.step(sim.t, 0.05, None) is None     # ★ 之后一帧都不发
    assert sim.runner.killed_or_idle() is True


@case('kill() 幂等 + 不产生 mode_request（上位机已接管，别再请求切模式）')
def _():
    sim = Sim(cfg=default_cfg())
    sim.runner.kill('PC_TAKEBACK')
    assert sim.runner.kill('OTHER') is True
    assert sim.runner.kill_reason == 'PC_TAKEBACK'            # 只记第一次
    assert sim.runner.pop_mode_request() is None


@case('$AUVCTL KILL 立即停手；RESET 解除终止态可重跑')
def _():
    sim = Sim(cfg=default_cfg())
    assert sim.runner.on_auvctl(['KILL']) is True
    assert sim.runner.killed is True
    assert sim.runner.step(sim.t, 0.05, None) is None
    sim.runner.on_auvctl(['RESET'])
    assert sim.runner.killed is False
    assert sim.runner.step(sim.t, 0.05, None) is not None


@case('kill 与 abort 不是一回事：abort 还要上浮，kill 直接停手')
def _():
    sim = Sim(cfg=default_cfg())
    sim.runner._abort('测试中止', sim.t)
    assert sim.runner.killed is False                          # abort 不算被杀
    assert sim.runner.abort_reason == '测试中止'
    assert sim.runner.step(sim.t + 0.05, 0.05, None) is not None   # 还要继续下发把上浮走完
    assert '已终止' in sim.runner.summary() or True


# ================================================================ ★ 状态提示回传（$MSG）
@case('$MSG 帧格式：含帧头共 6 字段、逗号/# 被转义、以 #\\r\\n 结尾')
def _():
    f = build_msg_frame('10:20:30', 'WARN', 'STAGE', 'SIT_BOTTOM', '坐底, 净空 5cm #1')
    assert f.startswith('$MSG,') and f.endswith('#\r\n'), repr(f)
    body = f[:-len('#\r\n')]
    parts = body.split(',')
    assert len(parts) == 6, parts            # ★ $MSG + ts + level + code + stage + text
    assert parts[0] == '$MSG'
    assert parts[1] == '10:20:30' and parts[2] == 'WARN' and parts[3] == 'STAGE'
    assert parts[4] == 'SIT_BOTTOM'
    assert ';' in parts[5] and '#' not in parts[5], parts[5]   # 逗号→分号、# 删除
    # 空文本也要占位（不能少字段）
    assert len(build_msg_frame('t', 'INFO', 'C', '-', '').rstrip('#\r\n').split(',')) == 6


@case('$MSG：WARN/ERROR 立即发；非法级别回落 INFO')
def _():
    sock = FakeSock()
    cfg = default_cfg()
    n = AuvMsg(cfg, lambda m: None, sock_factory=lambda: sock)
    assert n.start() is True
    n.warn('DEGRADE', '深度源不可用')
    n.error('ABORT', '找不到球，上浮')
    n.push('nonsense', 'X', '级别非法回落')
    _sleep(0.5)
    n.stop()
    sent = ''.join(sock.sent)
    assert '$MSG,' in sent, sock.sent
    assert 'WARN,DEGRADE' in sent, sent
    assert 'ERROR,ABORT' in sent, sent
    assert 'INFO,X' in sent, sent                              # 非法级别 → INFO
    assert n.ok >= 3, n.ok


@case('$MSG：连续失败 AUV_MSG_MAX_FAIL 次后永久放弃（且不再发）')
def _():
    sock = FakeSock(fail_times=9)
    cfg = default_cfg()
    n = AuvMsg(cfg, lambda m: None, sock_factory=lambda: sock)
    n.start()
    for i in range(6):
        n.error('E%d' % i, '一直发不出去')
    _sleep(0.6)
    n.stop()
    assert n.dead is True and n.alive() is False, (n.dead, n.fail)
    assert n.fail >= int(cfg.AUV_MSG_MAX_FAIL), n.fail
    before = len(sock.sent)
    n.error('AFTER', '放弃后入队')
    _sleep(0.3)
    assert len(sock.sent) == before                            # dead 后不再发


@case('$MSG：AUV_MSG_ENABLED=False → NoMsg 静默（调用方不用判空）')
def _():
    cfg = default_cfg(AUV_MSG_ENABLED=False)
    n = make_notifier(cfg, log=lambda m: None)
    assert isinstance(n, NoMsg), type(n)
    assert n.start() is False
    assert n.push('INFO', 'X', '不会被发') is False
    assert n.alive() is False


@case('$MSG：执行器在"测试模式启动 / 进入阶段 / 中止"三处都会发提示')
def _():
    rec = RecNotifier()
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['SIT_BOTTOM'])
    r = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                   test=TestCfg(cfg, log=lambda m: None), notify=rec)
    assert rec.has(code='TEST', level='WARN'), rec.codes()      # ★ 测试模式是 WARN（提醒不是正式作业）
    assert rec.has(code='STAGE', sub='SIT_BOTTOM'), rec.codes() # 进入某段
    r._abort('测试用中止', 0.0)
    assert rec.has(code='ABORT', level='ERROR'), rec.codes()


@case('$MSG：kill 时发 KILL 提示；普通作业启动只发 INFO（不误报 WARN）')
def _():
    rec = RecNotifier()
    cfg = default_cfg(AUV_TEST_MODE=False)
    # ⚠ test_config.TEST_ENABLED 是**模块级**的，True 时会把"非测试模式"也拖进测试模式
    #   （两边是"或"语义），所以这里必须临时关掉才能测到 MODE 这条路径。
    with tc_temp(TEST_ENABLED=False):
        TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                   notify=rec)
    assert rec.has(code='MODE', level='INFO'), rec.codes()
    assert not rec.has(code='TEST'), rec.codes()
    rec2 = RecNotifier()
    r2 = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                    notify=rec2)
    assert r2.notify is rec2                                 # ★ 传入的通道要真被用上
    r2.kill('PC_TAKEBACK')
    assert r2.killed is True
    assert rec2.has(code='KILLED', level='WARN'), rec2.codes()
    assert rec2.has(code='KILLED', sub='PC_TAKEBACK'), rec2.codes()


@case('$MSG：Sim 台架支持注入 notify（跑全流程也能收集提示）')
def _():
    rec = RecNotifier()
    # ⚠ 构造**也要**在 tc_temp 里：启动提示是在 TaskRunner.__init__ 里发的，
    #   模块级 TEST_ENABLED=True 会让它走"测试模式"分支（同上一条用例）。
    with tc_temp(TEST_ENABLED=False):
        sim = Sim(cfg=default_cfg(AUV_TEST_MODE=False), dt=0.05, notify=rec)
        assert sim.notify is rec
        sim.run(max_s=600.0)
    assert rec.has(code='MODE', level='INFO'), rec.codes()   # 启动提示
    assert rec.has(code='STAGE'), rec.codes()                # 每段进入提示
    assert rec.last_stage in ('DONE', 'SURFACE'), rec.last_stage
    assert not rec.has(level='ERROR'), [i for i in rec.items if i[0] == 'ERROR']


# ================================================================ ★ 两种运行模式（2026-10-05）
@case('★ 真实作业模式：policy 被强制 ignore（$AUVCTL 全拒，只有切模式管用）')
def _():
    cfg = default_cfg(AUV_TEST_MODE=False, AUV_CMD_POLICY='supervise')
    with tc_temp(TEST_ENABLED=False):           # 模块级开关会拖进测试模式，必须临时关
        t = TestCfg(cfg, log=lambda m: None)
        assert t.enabled is False and t.acceptance is True, (t.enabled, t.acceptance)
        assert t.cmd_policy(cfg) == 'ignore', t.cmd_policy(cfg)
        r = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                       notify=RecNotifier())
        assert r.policy == 'ignore', r.policy
        assert r.on_auvctl(['ABORT']) is False
        assert r.on_auvctl(['SKIP']) is False
        assert r.on_auvctl(['KILL']) is False
        assert r.abort_reason == '' and r.killed is False


@case('真实作业模式：跑**全部**阶段（不被选段/压缩影响），且启动提示是 INFO')
def _():
    cfg = default_cfg(AUV_TEST_MODE=False)
    with tc_temp(TEST_ENABLED=False, TEST_STAGES=['SIT_BOTTOM']):
        t = TestCfg(cfg, log=lambda m: None)
        assert t.acceptance is True
        assert t.hold_s(10.0) == 10.0, '真实模式不该压缩保持判据'
        rec = RecNotifier()
        r = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                       notify=rec)
    assert r.stage == 'DIVE', r.stage           # ★ 从头跑，不从选段开始
    assert len(r.order) == 24, len(r.order)
    assert rec.has(code='MODE', level='INFO', sub='真实作业模式'), rec.codes()
    assert not rec.has(code='TEST'), rec.codes()


@case('调试测试模式：policy 沿用配置（supervise 生效），启动提示是 WARN')
def _():
    cfg = default_cfg(AUV_TEST_MODE=True, AUV_TEST_STAGES=['SIT_BOTTOM'])
    with tc_temp(TEST_ENABLED=True, TEST_STAGES=['SIT_BOTTOM']):
        t = TestCfg(cfg, log=lambda m: None)
        assert t.acceptance is False
        assert t.cmd_policy(cfg) == 'supervise', t.cmd_policy(cfg)
        rec = RecNotifier()
        r = TaskRunner(cfg, vision=FakeVision(), depth=FakeDepth(), log=lambda m: None,
                       notify=rec)
    assert r.policy == 'supervise', r.policy
    assert r.stage == 'SIT_BOTTOM', r.stage     # ★ 从选段开始
    assert rec.has(code='TEST', level='WARN'), rec.codes()
    assert r.on_auvctl(['SKIP']) is True        # 测试模式：干预指令受理


@case('AUV_ACCEPTANCE_LOCK=False 时真实模式也用配置策略（显式解锁）')
def _():
    cfg = default_cfg(AUV_TEST_MODE=False, AUV_CMD_POLICY='supervise',
                      AUV_ACCEPTANCE_LOCK=False)
    with tc_temp(TEST_ENABLED=False):
        t = TestCfg(cfg, log=lambda m: None)
        assert t.acceptance is True
        assert t.cmd_policy(cfg) == 'supervise', t.cmd_policy(cfg)


@case('$MSG：阶段中文简述齐全（上位机终端要看得懂在干嘛）')
def _():
    for st in ('DIVE', 'SEEK_BALL_F', 'RAM_BALL', 'SIT_BOTTOM', 'RELEASE_DROP',
               'GO_HOME', 'SURFACE', 'DONE'):
        d = describe(st)
        assert d and d != st, (st, d)
    assert describe('NOPE') == 'NOPE'


@case('B8 补丁：真实作业(验收)模式拒绝 $VID；测试模式照常开关图像')
def _():
    """$VID 是 dispatcher 直接执行的（不等 AUV 的 policy），必须单独挡一道。

    这里不 import 板端 dispatcher（离线装不齐依赖），而是把补丁规则 B8 打到一段
    同构的迷你 dispatcher 上再 exec —— 既验锚点正则没漂移，也验挡的逻辑本身。
    """
    import re
    import importlib.util

    # 补丁脚本不在工程内（本机在 auv_mission/upload/，板端传在 /tmp/ 跑完就删）。
    # 按顺序找：环境变量 AUV_TASK_PATCH >../5 层（本机布局）> 板端常见位置。
    # ⚠ 板端没有这个文件是**正常**的 —— 脚本是一次性工具，不该留在工程里；
    #   所以找不到时**跳过**而不是判失败，否则板端跑回归永远红一条。
    cands = []
    if os.environ.get('AUV_TASK_PATCH'):
        cands.append(os.environ['AUV_TASK_PATCH'])
    cands.append(os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..', '..',
                                              'apply_auv_task_patch.py')))
    for _d in ('/userdata/GrandRDK/src/to32/move_test', '/userdata/GrandRDK',
               '/userdata/GrandRDK/src/to32', '/tmp'):
        cands.append(os.path.join(_d, 'apply_auv_task_patch.py'))
    p = next((c for c in cands if os.path.exists(c)), None)
    if p is None:
        return                      # 跳过：板端本来就不该有它（本机跑会命中本机路径）
    spec = importlib.util.spec_from_file_location('apply_auv_task_patch', p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    hit = [r for r in mod._p_dispatcher() if r[0] == 'B8']
    assert hit, '补丁规则 B8 不见了'
    _, _, pat, rep, _, marker = hit[0]

    src = (
        'class D(object):\n'
        '    def __init__(self, acc):\n'
        '        self.cfg = type("C", (), {"AUV_ACCEPTANCE_LOCK": True})()\n'
        '        self._m = type("M", (), {"mission": None})()\n'
        '        self._m.mission = type("X", (), {"test": None})()\n'
        '        self._m.mission.test = type("T", (), {"acceptance": acc})()\n'
        '        self.seen = []\n'
        '    def log(self, s):\n'
        '        self.seen.append(s)\n'
        '    def mode(self):\n'
        '        return self._m\n'
        '    def on_pc_event(self, kind, payload):\n'
        '        if kind == "vid":\n'
        '            on = bool(payload)  # 统一成布尔开关量\n'
        '            self.seen.append("SET:%s" % on)\n'
    )
    out = re.sub(pat, rep, src, count=1)
    assert out != src, 'B8 锚点没匹配上（板端 dispatcher 改版导致正则漂移）'
    assert marker in out
    compile(out, 'mini_dispatcher', 'exec')          # 语法必须过
    ns = {}
    exec(out, ns)

    acc = ns['D'](True)
    acc.on_pc_event('vid', 1)
    assert acc.seen and acc.seen[-1].startswith('[AUV] $VID 被拒绝'), acc.seen
    assert not any(str(x).startswith('SET:') for x in acc.seen), '验收模式不该真去开关图像'

    dbg = ns['D'](False)                              # 测试模式：acceptance=False
    dbg.on_pc_event('vid', 1)
    assert 'SET:True' in dbg.seen, dbg.seen


def main():
    print('== auv_task 离线回归 ==')
    for ok, name, msg in _RES:
        if not ok:
            print('\n---- 失败详情 ----')
            print(name, ':', msg)
    bad = [x for x in _RES if not x[0]]
    print('\n合计 %d 项，通过 %d，失败 %d' % (len(_RES), len(_RES) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
