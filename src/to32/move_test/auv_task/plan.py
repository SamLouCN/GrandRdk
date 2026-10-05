# -*- coding: utf-8 -*-
"""阶段表 plan —— 24 个阶段的权威顺序（方案 §18.2）

★ 这个文件的地位：**改流程只改这里**。
  加/删/调序/改参数都在 build_plan() 里一行搞定，执行器与任务类都不用动。

★ 参数写法速记：
  * 大写键（'SURGE', 'TIMEOUT_S', 'DUR_S', 'DEG' ...）：值写**配置键名**（不含 AUV_ 前缀）
    → 运行时查 `AUV_<键名>`，查不到用任务里的默认；
  * 小写键（'cam', 'next', 'fail', 'scan', 'label_key' ...）：**字面量**，原样使用；
  * 也可以直接写数字（如 `'DEG': 120.0`）→ 该阶段专属量，不进全局配置。

★ 门数量可配（现场 3~4 个门，方案 §16f）：AUV_GATE_COUNT；
  高矮交替顺序可配：AUV_GATE_SEQ = 'low,high,low,high'。
  找不到门 → `fail` 指向**下一个门**（跳过这一个，别整局报废）；
  撞球/捡球找不到 → `fail='ABORT'`（核心得分项没了，上浮止损）。

★ 阶段参数还能被 `test_config.STAGE_PARAMS` 覆盖（方案 §16j）：
  键写阶段名 / `task:<类名>` / 通配，优先级 精确名 > task: > 通配 > 这里的默认值。
  覆盖在 `_sp()` 里统一套用，所以**加阶段不用自己管这件事**。
"""
from .tasks import (DepthTask, SurfaceTask, SeekTask, RamBallTask, PassGateTask,
                    CenterBallTask, SitBottomTask, HoldTask, TimedTask, TurnTask,
                    DeadReckonTask, GoHomeTask, ServoDropTask, DoneTask)

try:                                    # test_config 可选：没有就纯用这里的默认值
    from . import test_config as TC
except Exception:                       # pragma: no cover
    TC = None


def _overrides(name, cls):
    """取 test_config.STAGE_PARAMS 对本阶段的覆盖（没有就返回空）"""
    if TC is None:
        return {}
    try:
        return dict(TC.stage_params(name, getattr(cls, '__name__', None)) or {})
    except Exception:                   # 配置写错不能让流程起不来
        return {}


def _sp(name, cls, **params):
    """阶段表条目工厂：name=阶段名, cls=任务类, params=参数（会自动套 test_config 覆盖）"""
    ov = _overrides(name, cls)
    if ov:
        merged = dict(params)
        merged.update(ov)               # 覆盖优先级最高
        params = merged
    return {'name': name, 'cls': cls, 'params': params}


def _gate_seq(cfg, n):
    """解析高矮门顺序 → ['low','high',...]（不足则交替补齐）"""
    raw = getattr(cfg, 'AUV_GATE_SEQ', None)
    seq = []
    if isinstance(raw, (list, tuple)):
        seq = [str(x).strip().lower() for x in raw]
    elif isinstance(raw, str) and raw.strip():
        seq = [x.strip().lower() for x in raw.split(',') if x.strip()]
    out = []
    for i in range(max(0, n)):
        kind = seq[i] if i < len(seq) else ('low' if i % 2 == 0 else 'high')
        out.append('low' if kind.startswith('lo') else 'high')
    return out


def build_plan(cfg=None, log=None):
    """按配置生成阶段表（列表，元素为 {name, cls, params}）

    cfg 为 None 时用全部默认值（便于离线单测不依赖板端配置）。
    """
    cfg = cfg if cfg is not None else object()
    log = log or (lambda m: None)

    # ---- 门数量（现场 3~4 个，规则给的是"至少 3 个"）
    try:
        n = int(getattr(cfg, 'AUV_GATE_COUNT', 4) or 0)
    except (TypeError, ValueError):
        n = 4
    n = max(0, min(6, n))
    seq = _gate_seq(cfg, n)
    first_gate = 'SEEK_GATE_1' if n > 0 else 'SEEK_BALL_B'

    plan = []
    # ---------------------------------------------------------------- 下潜（10 分）
    plan.append(_sp('DIVE', DepthTask,
                    height_cm='BALL_HEIGHT_CM',      # 直接下到撞球高度（离底 60cm）
                    depth_m_key='DIVE_DEPTH_M',      # 没配离底高度时的退路（读 AUV_DIVE_DEPTH_M，单位 m）
                    SURGE='SURGE_SEEK',
                    next='SEEK_BALL_F', fail='SEEK_BALL_F'))   # 定深超时也继续找球

    # ---------------------------------------------------------------- 撞球（30 分）
    plan.append(_sp('SEEK_BALL_F', SeekTask,
                    cam='front', label_key='LABEL_BALL', aim_key='AIM_RAM', scan=True,
                    next='RAM_BALL', fail='ABORT'))
    plan.append(_sp('RAM_BALL', RamBallTask,
                    cam='front', label_key='LABEL_BALL', aim_key='AIM_RAM',
                    next='TURN_120', fail='TURN_120'))          # 没撞上也继续（过门还有 30×N 分）
    plan.append(_sp('TURN_120', TurnTask,
                    DEG='TURN1_DEG',
                    next=first_gate, fail=first_gate))

    # ---------------------------------------------------------------- 过门（每个 30 分）
    for i in range(1, n + 1):
        hkey = 'GATE_LOW_HEIGHT_CM' if seq[i - 1] == 'low' else 'GATE_HIGH_HEIGHT_CM'
        nxt = ('SEEK_GATE_%d' % (i + 1)) if i < n else 'SEEK_BALL_B'
        plan.append(_sp('SEEK_GATE_%d' % i, SeekTask,
                        cam='front', label_key='LABEL_GATE', aim_key='AIM_GATE', scan=True,
                        next='PASS_GATE_%d' % i, fail=nxt))     # 找不到 → 跳过这一个门
        plan.append(_sp('PASS_GATE_%d' % i, PassGateTask,
                        cam='front', label_key='LABEL_GATE', aim_key='AIM_GATE',
                        height_cm=hkey,
                        next=nxt, fail=nxt))                    # 穿不过 → 也跳过，别耗

    # ---------------------------------------------------------------- 捡球（100 分，收集框坐底兜）
    plan.append(_sp('SEEK_BALL_B', SeekTask,
                    cam='bottom', label_key='LABEL_PICK', aim_key='AIM_PICK', scan=False,
                    next='CENTER_BALL', fail='ABORT'))
    plan.append(_sp('CENTER_BALL', CenterBallTask,
                    cam='bottom', label_key='LABEL_PICK', aim_key='AIM_PICK',
                    next='SIT_BOTTOM', fail='SIT_BOTTOM'))      # 对不准也坐底（尽力而为）
    plan.append(_sp('SIT_BOTTOM', SitBottomTask,
                    next='BOTTOM_HOLD', fail='BOTTOM_HOLD'))
    plan.append(_sp('BOTTOM_HOLD', HoldTask,
                    DUR_S='BOTTOM_HOLD_S',
                    next='RELEASE_ASCEND', fail='RELEASE_ASCEND'))

    # ---------------------------------------------------------------- 投放（100 分，舵机）
    plan.append(_sp('RELEASE_ASCEND', DepthTask,
                    height_cm='RELEASE_HEIGHT_CM', depth_m_key='ASCEND_DEPTH_M',
                    next='RELEASE_TURN', fail='RELEASE_TURN'))
    plan.append(_sp('RELEASE_TURN', TurnTask,
                    DEG='RELEASE_TURN_DEG',
                    next='RELEASE_MOVE', fail='RELEASE_MOVE'))
    plan.append(_sp('RELEASE_MOVE', DeadReckonTask,
                    DIST_M='RELEASE_DIST_M',
                    next='RELEASE_HOVER', fail='RELEASE_HOVER'))
    plan.append(_sp('RELEASE_HOVER', HoldTask,
                    DUR_S='RELEASE_HOVER_S',
                    next='RELEASE_DROP', fail='RELEASE_DROP'))
    plan.append(_sp('RELEASE_DROP', ServoDropTask,
                    next='GO_HOME', fail='GO_HOME'))

    # ---------------------------------------------------------------- 回出发区（30 分）+ 上浮
    plan.append(_sp('GO_HOME', GoHomeTask,
                    turn_deg='HOME_TURN_DEG',
                    next='SURFACE', fail='SURFACE'))
    plan.append(_sp('SURFACE', SurfaceTask, next='DONE', fail='DONE'))
    plan.append(_sp('DONE', DoneTask, next='DONE', fail='DONE'))
    return plan


def names(plan):
    """取阶段名列表（按序）"""
    return [s['name'] for s in plan]


def pairs(plan):
    """取 [(阶段名, 任务类名), ...]（给 TestCfg.validate 校验 `task:<类名>` 写法用）"""
    return [(s['name'], getattr(s['cls'], '__name__', '')) for s in plan]


def index_of(plan, name):
    """阶段名 → 序号（找不到返回 -1）"""
    for i, s in enumerate(plan):
        if s['name'] == name:
            return i
    return -1
