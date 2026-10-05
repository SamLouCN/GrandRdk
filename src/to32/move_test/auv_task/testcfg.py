# -*- coding: utf-8 -*-
"""测试模式配置 —— 只跑某几段、把等待时间压短，但**上浮永不关闭**

★ 配置来源有两处（方案 §16j：统一到一个 test_config.py）：
  1. `test_config.py`（**推荐**，改它不用碰 auv_config.py，离线台架也读它）
  2. `auv_config.py` 的 `AUV_TEST_MODE` / `AUV_TEST_STAGES`（旧口径，保留兼容）

合并规则（避免"改一处不生效"）：
  * `enabled`：**或**语义 —— 两边任一为 True 就进测试模式
    （防止一边开了另一边关掉、你以为关了其实没关）
  * `patterns` / `hold_max_s` / `loop`：test_config 有值就用它，否则回落 AUV_TEST_*
  * 阶段参数覆盖（`STAGE_PARAMS`）在 `plan.build_plan()` 里套用
  * 全局参数覆盖（`GLOBAL_PARAMS`）在 `TaskRunner.__init__` 里套用

★ 三条硬规则（改这个文件前先看）：
  1. `SURFACE` / `DONE` 在 FORCED 里，**任何配置都删不掉** —— 上浮是必须的，
     不允许配置文件关闭它（用户 2026-10-03 拍板）。
  2. 测试模式下所有"判据需保持 N 秒"会被 `hold_max_s` 钳短，
     否则单段调试要干等 2~3 秒 × 几十拍，水池里根本没法看现象。
  3. 选段里写了不存在的阶段名 → **启动即打 ERROR 并退回全流程**（Fail-Safe），
     宁可跑全流程，也不能悄悄跳过一段让你以为"这段没问题"。
"""
import fnmatch  # 标准库: 选段通配匹配

try:                                    # test_config 是可选的：没有就纯用 auv_config
    from . import test_config as TC
except Exception:                       # pragma: no cover - 缺文件/语法错都要能降级
    TC = None


# 强制保留的阶段（配置删不掉）
FORCED = ('SURFACE', 'DONE')


def _up(x):
    return str(x).upper()


def _patterns(raw):
    """把 AUV_TEST_STAGES 的多种写法统一成大写模式列表"""
    if raw is None:
        return []
    if isinstance(raw, (list, tuple, set)):
        return [_up(x).strip() for x in raw if str(x).strip()]
    return [p.strip() for p in str(raw).split(',') if p.strip()]


class TestCfg(object):
    """测试模式配置：选段 + 时间压缩 + 校验 + 阶段参数覆盖"""

    def __init__(self, cfg=None, log=None):
        cfg = cfg if cfg is not None else object()   # 允许无配置构造（离线单测）
        self.log = log or (lambda m: None)

        # ---- ① 总开关：两边任一为 True 即测试模式
        from_cfg = bool(getattr(cfg, 'AUV_TEST_MODE', False))
        from_tc = bool(getattr(TC, 'TEST_ENABLED', False)) if TC else False
        self.enabled = bool(from_cfg or from_tc)

        # ---- ② 选段：test_config 优先，回落 AUV_TEST_STAGES
        tc_stages = list(getattr(TC, 'TEST_STAGES', None) or []) if TC else []
        self.patterns = tc_stages or _patterns(getattr(cfg, 'AUV_TEST_STAGES', None))

        # ---- ③ 保持判据时间上限：钳到 [1.0, 30.0] s
        #      （钳下界是防止手滑配成 0 → 判据"瞬间满足"，测出来的时序全是假的）
        hold = getattr(TC, 'TEST_HOLD_MAX_S', None) if TC else None
        if hold is None:
            hold = getattr(cfg, 'AUV_TEST_HOLD_MAX_S', 5.0)
        try:
            self.hold_max_s = float(hold)
        except (TypeError, ValueError):
            self.hold_max_s = 5.0
        self.hold_max_s = max(1.0, min(30.0, self.hold_max_s))

        # ---- ④ 单段循环 / 上浮后切 ROV
        self.loop = bool((getattr(TC, 'TEST_LOOP', False) if TC else False)
                         or getattr(cfg, 'AUV_TEST_LOOP', False))
        self.surface_then_rov = bool(getattr(cfg, 'AUV_SURFACE_THEN_ROV', True))

        # ---- ⑤ [2026-10-05] ★ 两种运行模式：真实作业(验收) vs 调试测试
        #   非测试 = 真实作业：跑**全部**阶段，**不接受**上位机除"切模式"之外的任何指令。
        #   测试   = 按 test_config 跑部分阶段，回传图像/数据/终端状态，接受上位机切回 ROV。
        self.acceptance = not self.enabled
        self.force_video = bool(getattr(cfg, 'AUV_TEST_FORCE_VIDEO', True))
        self.acceptance_lock = bool(getattr(cfg, 'AUV_ACCEPTANCE_LOCK', True))

    # ---------------------------------------------------------------- 运行模式
    def cmd_policy(self, cfg=None):
        """★ 监督指令策略：真实作业模式**强制** ignore，测试模式沿用配置

        为什么硬锁：验收/比赛时 `$AUVCTL`（HOLD/SKIP/ABORT/GOTO）是"岸上人手痒"的主要来源，
        一发就把正在跑的流程打断。真实模式只允许"上位机切模式"这一条路（那是 dispatcher
        层面的切模式，不归本策略管），其余干预全部拒绝。

        想临时解锁：`AUV_ACCEPTANCE_LOCK = False`（不建议在比赛机上做）。
        """
        if self.acceptance and self.acceptance_lock:
            return 'ignore'
        cfg = cfg if cfg is not None else object()
        return str(getattr(cfg, 'AUV_CMD_POLICY', 'supervise') or 'supervise').lower()

    def mode_label(self):
        """一行人读的运行模式（终端/日志都打这个，防止"以为在测其实在验收"）"""
        if self.acceptance:
            return '真实作业模式（验收）：跑全部阶段，不接受上位机干预（仅"切模式"生效）'
        return '调试测试模式：%s' % self.summary()

    # ---------------------------------------------------------------- 选段
    def keep(self, name, task=None):
        """该阶段是否保留在测试流程里（task = 任务类名，用于 `task:<类名>` 写法）"""
        if not self.enabled:
            return True
        n = _up(name)
        if n in FORCED:                                  # 强制保留：上浮删不掉
            return True
        if not self.patterns:                            # 没写次级开关 = 全部跑
            return True
        tn = ('TASK:' + _up(task)) if task else ''
        for p in self.patterns:
            pu = _up(p)
            if fnmatch.fnmatch(n, pu) or (tn and fnmatch.fnmatch(tn, pu)):
                return True
        return False

    def validate(self, all_names):
        """校验选段：写了不存在的名字 → 报错并**关闭**测试模式（Fail-Safe 到全流程）

        all_names 可以是 [阶段名, ...]，也可以是 [(阶段名, 任务类名), ...]（推荐后者，
        这样 `task:PassGateTask` 这种写法才校验得了）。

        返回 (ok, unknown, kept)：
          ok      = 是否可进入测试模式
          unknown = 匹配不到任何阶段的花样列表
          kept    = 实际保留的阶段名列表
        """
        pairs = []
        for item in (all_names or []):
            if isinstance(item, (list, tuple)):
                pairs.append((item[0], (item[1] if len(item) > 1 else None)))
            else:
                pairs.append((item, None))
        names = [_up(p[0]) for p in pairs]

        if not self.enabled or not self.patterns:
            return True, [], [p[0] for p in pairs if self.keep(p[0], p[1])]

        pool = set(names)
        for n, t in pairs:
            if t:
                pool.add('TASK:' + _up(t))
        unknown = [p for p in self.patterns if not fnmatch.filter(sorted(pool), _up(p))]
        if unknown:
            self.log('[AUV][TEST] ✗ 选段里有未知名字 %s —— 测试模式已关闭，'
                     '本次按全流程作业跑（可跑阶段: %s）' % (unknown, ','.join(names)))
            self.enabled = False
            return False, unknown, [p[0] for p in pairs]
        return True, [], [p[0] for p in pairs if self.keep(p[0], p[1])]

    # ---------------------------------------------------------------- 参数覆盖
    def params(self, name, task=None):
        """取某阶段的参数覆盖（转发 test_config.stage_params）"""
        if TC is None:
            return {}
        try:
            return dict(TC.stage_params(name, task) or {})
        except Exception:
            return {}

    # ---------------------------------------------------------------- 时间压缩
    def hold_s(self, want_s):
        """把"判据需保持 N 秒"压到测试上限（非测试模式原样返回）"""
        w = float(want_s)
        if not self.enabled:
            return w
        return max(0.0, min(w, self.hold_max_s))

    def summary(self):
        """一行摘要（进 AUV 时打出来，方便一眼确认"现在是不是在测某一段"）"""
        if not self.enabled:
            return '全流程作业'
        if not self.patterns:
            return '测试模式(全部阶段, 保持判据≤%.1fs)' % self.hold_max_s
        return '测试模式(仅 %s, 保持判据≤%.1fs)' % (','.join(self.patterns), self.hold_max_s)


def apply_global(cfg, log=None):
    """★ 把 test_config.GLOBAL_PARAMS 覆盖到配置对象上（板端 = to32_config 模块对象）

    返回被覆盖的键名列表。放在这里是为了让 runner 与离线台架共用同一条路径。
    """
    if TC is None:
        return []
    try:
        hit = TC.apply_global(cfg)
    except Exception:
        return []
    if hit and log:
        log('[AUV] test_config 全局覆盖 %d 项: %s' % (len(hit), ','.join(hit)))
    return hit
