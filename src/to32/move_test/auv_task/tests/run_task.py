# -*- coding: utf-8 -*-
"""离线单跑 CLI —— 在 Windows 上跑单个/全部任务，不用连板子

用法（在 tests/ 目录下）:
    python run_task.py --cfg                # ★ 打印当前 test_config 的配置摘要
    python run_task.py --list               # 列出全部阶段（带任务类名 + 参数覆盖）
    python run_task.py --full               # 全流程干跑（验收路径）
    python run_task.py --stage SEEK_BALL_F  # 单段调试（自动开测试模式）
    python run_task.py --stage SIT_BOTTOM -v  # 单段 + 打印每条日志
    python run_task.py --full --dt 0.05 --max-s 600
    python run_task.py --stage SIT_BOTTOM --msg   # ★ 顺带打印这一路会发哪些 $MSG 提示

★ 不带 --full / --stage 时，**完全按 test_config.py 跑**
  （TEST_ENABLED / TEST_STAGES / STAGE_PARAMS / SIM / VISION_SCRIPT 全部生效），
  所以"改完配置直接跑一遍看看"就是这个命令。

★ 这就是"我可以独立测试每一阶段任务的实现效果"的落地方式：
  开测试模式 + 把选段写成那一段，执行器自动从它开始，
  跑完它直接进 SURFACE（FORCED，删不掉）→ DONE。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_PKG_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fakes import default_cfg, sim_defaults, RecNotifier  # noqa: E402
from harness import Sim, full_run, single_stage, from_test_config  # noqa: E402
from auv_task import build_plan, names, pairs, test_config as TC  # noqa: E402


def _arg(name, default=None):
    """取 --name value 形式的参数"""
    return default if name not in sys.argv else sys.argv[sys.argv.index(name) + 1]


def _show_cfg():
    """打印 test_config 摘要（一眼确认"现在到底在测什么"）"""
    print('== test_config 摘要 ==')
    print('  测试模式     : %s' % ('开' if TC.TEST_ENABLED else '关（全流程作业）'))
    print('  选段         : %s' % (', '.join(TC.TEST_STAGES) if TC.TEST_STAGES else '（空 = 全部）'))
    print('  保持判据上限 : %s s%s' % (TC.TEST_HOLD_MAX_S or '默认',
                                      '' if TC.TEST_HOLD_MAX_S else '（沿用 auv_config 的 AUV_TEST_HOLD_MAX_S）'))
    print('  单段循环     : %s' % TC.TEST_LOOP)
    print('  阶段参数覆盖 : %d 项' % len(TC.STAGE_PARAMS or {}))
    for k, v in (TC.STAGE_PARAMS or {}).items():
        print('      %-18s %s' % (k, v))
    print('  全局参数覆盖 : %d 项' % len(TC.GLOBAL_PARAMS or {}))
    for k, v in (TC.GLOBAL_PARAMS or {}).items():
        print('      %-24s = %s' % (k, v))
    sim = sim_defaults()
    print('  离线仿真     : dt=%s max_s=%s pool=%sm depth_ok=%s viskf_ok=%s servo_ready=%s'
          % (sim['dt'], sim['max_s'], sim['pool_m'],
             sim['depth_ok'], sim['viskf_ok'], sim['servo_ready']))
    vs = TC.VISION_SCRIPT or {}
    print('  视觉剧本     : enabled=%s never_see=%s' % (vs.get('enabled'), vs.get('never_see')))
    n = TC.NOTIFY or {}
    print('  状态提示回传 : enabled=%s min_level=%s hz=%s 心跳=%ss 去重=%ss'
          % (n.get('enabled'), n.get('min_level'), n.get('hz'),
             n.get('heartbeat_s'), n.get('dedup_s')))
    print('  %s' % TC.summary())
    return 0


def _show_list():
    """列出全部阶段（阶段名 + 任务类名 + 生效的参数覆盖）"""
    cfg = default_cfg()
    print('  %-2s %-16s %-16s %s' % ('#', '阶段', '任务类', 'test_config 覆盖'))
    for i, (n, cls) in enumerate(pairs(build_plan(cfg))):
        ov = TC.stage_params(n, cls)
        print('  %-2d %-16s %-16s %s' % (i, n, cls, ov if ov else ''))
    return 0


def main():
    dt = float(_arg('--dt', str(sim_defaults()['dt'])))
    max_s = float(_arg('--max-s', str(sim_defaults()['max_s'])))
    verbose = '-v' in sys.argv or '--verbose' in sys.argv
    # ★ --msg：把这一路会发的状态提示（$MSG）收集并打印出来
    #   （离线台架绝不真发 UDP；要验证"上位机终端看到什么"，看这里的输出）
    rec = RecNotifier() if '--msg' in sys.argv else None

    if '--cfg' in sys.argv:
        return _show_cfg()
    if '--list' in sys.argv:
        return _show_list()

    if '--stage' in sys.argv:
        stage = _arg('--stage', 'DIVE').upper()
        print('== 单段调试: %s ==' % stage)
        sim = single_stage(stage, dt=dt, echo=verbose, notify=rec)
        sim.run(max_s=max_s)
        print(sim.report(max_rows=200 if verbose else 40))
        return 0

    if '--full' in sys.argv:
        print('== 全流程干跑 ==')
        sim = full_run(dt=dt, max_s=max_s, echo=verbose, notify=rec)
        print(sim.report(max_rows=200 if verbose else 40))
        return 0

    print('== 按 test_config.py 跑（%s）==' % TC.summary())
    sim = from_test_config(dt=dt, echo=verbose, notify=rec)
    print(sim.report(max_rows=200 if verbose else 40))
    return 0


if __name__ == '__main__':
    sys.exit(main())
