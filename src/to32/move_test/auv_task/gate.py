# -*- coding: utf-8 -*-
"""启动门控 —— ★ AUV（**无论正式作业还是测试**）只能由上位机 $CMD 切入

需求（2026-10-05 用户拍板）：
  1. 无论是任务执行还是测试，都必须由**上位机发送指令切换到 AUV 模式**后才开启；
     上电默认、命令行 `--mode auv`、模式记忆文件，三种"非上位机"来源一律不许直接进 AUV。
  2. 测试/作业过程中上位机切回 ROV → 立即终止任务、交还遥控
     （这一半由 `runner.kill()` + `mode_auv.on_exit` 负责，不在这里）。

★ 为什么单独一个文件：
  这条规则要在**两个地方**生效（dispatcher 决定进哪个模式 / 离线回归要能测它），
  抽成**纯函数**才能在 Windows 上脱离板端跑测试。

四种模式来源（与 mode_dispatcher 的 self.mode_source 对齐）：
  start    上电/配置启动（config.START_MODE）
  forced   命令行 --mode（MODE_FORCE_START）
  saved    模式记忆文件
  pc_cmd   上位机 $CMD 的 mode 字段  ← **唯一允许进 AUV 的来源**

★ 失效方向必须是"安全侧"：
  dispatcher import 不到本模块 → **不进 AUV**（停在 IDLE 等上位机），
  绝不能反过来默认放行 —— 宁可不动，也不能没人看着就自己跑起来。
"""
from __future__ import annotations

# ---------------------------------------------------------------- 来源常量
SRC_PC = 'pc_cmd'        # 上位机 $CMD 触发（唯一允许进 AUV）
SRC_START = 'start'      # 配置启动
SRC_FORCED = 'forced'    # 命令行 --mode
SRC_SAVED = 'saved'      # 模式记忆文件
SRC_UNKNOWN = 'unknown'  # 兜底（按不安全处理）

# 允许进 AUV 的来源集合（大小写不敏感，兼容 dispatcher 里的其它写法）
_PC_ALIASES = ('pc', 'pc_cmd', 'cmd', 'command', '上位机')


def _cfg_int(cfg, key, default):
    """读配置并转 int（键缺失/非数字都回落缺省）"""
    try:
        return int(getattr(cfg, key, default))
    except (TypeError, ValueError):
        return int(default)


def _cfg_bool(cfg, key, default):
    """读配置并转 bool"""
    return bool(getattr(cfg, key, default))


def is_pc_source(source):
    """来源是否来自上位机指令"""
    s = str(source or '').strip().lower()
    return s in _PC_ALIASES


def auv_initial_allowed(mode_id, source, cfg):
    """启动期是否允许进入 mode_id（**只有 AUV 会被拦**，其它模式一律放行）

    返回 (allowed: bool, why: str)
      allowed=True  → 正常进入
      allowed=False → 调用方应把模式回落到 MODE_IDLE（why 是可直接打进日志的原因）

    ⚠ 只用于**启动/初始化**（dispatcher 的 switch_mode(initial=True)）。
      运行期由上位机切模式走的是 initial=False，不受本门控影响 ——
      否则上位机就再也切不进 AUV 了。
    """
    auv_id = _cfg_int(cfg, 'MODE_AUV', 1)
    try:
        mid = int(mode_id)
    except (TypeError, ValueError):
        return True, '模式 id 非法（交给调用方兜底）'
    if mid != auv_id:                                   # 不是 AUV → 不拦
        return True, ''
    if not _cfg_bool(cfg, 'AUV_REQUIRE_PC_CMD', True):  # 显式关掉门控
        return True, 'AUV_REQUIRE_PC_CMD=False（门控已关闭）'
    if is_pc_source(source):
        return True, '上位机 $CMD 触发'
    idle_id = _cfg_int(cfg, 'MODE_IDLE', -1)
    return False, ('AUV 只能由上位机指令切入（来源=%s）→ 回落 IDLE(%d)'
                   % (source or SRC_UNKNOWN, idle_id))


def sanitize_persist_mode(mode_id, cfg):
    """写模式记忆文件前，把 AUV 改写成 IDLE

    ★ 为什么要这一步：模式记忆会在下次上电时**自动沿用**，
      如果上次退出时是 AUV，下一次开机就会自己跑起来 —— 正是要禁止的行为。
      这里在"写入"这一侧就消掉它，比启动时再拦更早、更彻底（双保险）。
    """
    auv_id = _cfg_int(cfg, 'MODE_AUV', 1)
    try:
        mid = int(mode_id)
    except (TypeError, ValueError):
        return mode_id
    if mid != auv_id:
        return mid
    if not _cfg_bool(cfg, 'AUV_REQUIRE_PC_CMD', True):
        return mid
    return _cfg_int(cfg, 'MODE_IDLE', -1)


def guard_initial_mode(mode_id, source, cfg, modes=None, log=None):
    """一键式门控：不允许就返回回落后的模式 id（dispatcher 补丁直接调这个）

    modes: {模式id: 模式实例或名字}（用来确认 IDLE 已注册；不传就用 cfg.MODE_IDLE）
    log:   日志回调（不给就不打）
    返回 (最终模式 id, 是否被拦)
    """
    ok, why = auv_initial_allowed(mode_id, source, cfg)
    if ok:
        return mode_id, False
    idle_id = _cfg_int(cfg, 'MODE_IDLE', -1)
    if modes is not None and idle_id not in modes:
        if log is not None:
            log('[MODE] 启动门控触发，但 IDLE(%d) 未注册 → 维持原模式（%s）' % (idle_id, why))
        return mode_id, True
    if log is not None:
        name = '-'
        try:
            m = (modes or {}).get(mode_id)
            name = getattr(m, 'name', str(mode_id))
        except Exception:
            name = str(mode_id)
        log('[MODE] 启动门控：%s(%s) 来源=%s → 回落 IDLE(%d)：%s'
            % (name, mode_id, source, idle_id, why))
    return idle_id, True


def summary(cfg):
    """一行状态摘要（启动日志里打出来，一眼确认门控开没开）"""
    if not _cfg_bool(cfg, 'AUV_REQUIRE_PC_CMD', True):
        return 'AUV 启动门控：关（允许上电/记忆/命令行直接进 AUV）'
    return ('AUV 启动门控：开（只能由上位机 $CMD.mode=AUV(%d) 切入，否则停在 IDLE(%d)）'
            % (_cfg_int(cfg, 'MODE_AUV', 1), _cfg_int(cfg, 'MODE_IDLE', -1)))
