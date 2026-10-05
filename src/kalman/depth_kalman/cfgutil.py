# -*- coding: utf-8 -*-
"""配置装载：把 ``depth_config`` 的模块级常量摊平成可覆盖的命名空间对象。

* 所有代码只接收这个 cfg 对象（便于自检里做参数扫描）。
* ``ACCEL_UNIT='unknown'`` 时**强制关闭**加速度 —— 单位没标定就积分等于自杀。
"""
import types

import depth_config as dc

_UNITS_MS2 = ('m/s^2', 'm/s2', 'ms2', 'si')
_UNITS_G = ('g', 'grav', 'gravity')


def load(**overrides):
    d = {}
    for k in dir(dc):
        if k.isupper():
            d[k] = getattr(dc, k)
    for k, v in overrides.items():
        if v is None:
            continue
        d[str(k).upper()] = v
    cfg = types.SimpleNamespace(**d)

    # 派生规则：单位未标定 → 不允许启用加速度，也不允许有 b_a 状态
    unit = str(cfg.ACCEL_UNIT).strip().lower()
    cfg.ACCEL_UNIT_KNOWN = unit in _UNITS_MS2 or unit in _UNITS_G
    if not cfg.ACCEL_UNIT_KNOWN:
        cfg.ACCEL_ENABLED = False
        cfg.ENABLE_BA = False
    if not cfg.ACCEL_ENABLED:
        cfg.ENABLE_BA = False

    # 通道白名单：去空、去重、统一大写
    seen = []
    for c in (cfg.ALT_CHANNELS or []):
        c = str(c).strip().upper()
        if len(c) == 1 and c.isalpha() and c not in seen:
            seen.append(c)
    cfg.ALT_CHANNELS = seen

    # 锚路
    anchor = str(getattr(cfg, 'ALT_ANCHOR', '') or '').strip().upper()
    if anchor and anchor not in seen:
        anchor = ''
    cfg.ALT_ANCHOR = anchor or (seen[0] if seen else '')

    return cfg


def explain(cfg):
    """一行摘要，启动时打印，避免"以为开了其实没开"。"""
    st = ['D', 'v_z']
    if cfg.ENABLE_BD and cfg.ALT_CHANNELS and str(cfg.H_MODE) == 'known':
        st.append('b_d')
    if cfg.ENABLE_BA and cfg.ACCEL_ENABLED:
        st.append('b_a')
    st.append('H')
    return ('状态 [%s] | 高度计 %s（锚路 %s） | 加速度 %s(单位=%s) | H=%.3f m(%s)'
            % (','.join(st),
               ','.join(cfg.ALT_CHANNELS) or '无',
               cfg.ALT_ANCHOR or '-',
               '开' if cfg.ACCEL_ENABLED else '关',
               cfg.ACCEL_UNIT,
               float(cfg.H_M), cfg.H_MODE))
