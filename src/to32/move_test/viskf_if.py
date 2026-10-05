# -*- coding: utf-8 -*-
"""图像卡尔曼接口 —— 读 /dev/shm/momo_viskf.json（camera_kalman / viskf 写的穿门滤波量）

[AUV-MISSION 2026-10-01 新增] 过门阶段的视觉来源。
与 `vision_if.py` 的区别：**那边是原始检测框（像素、抖、会被裁），这边是滤波后的物理量**
（横向偏差 `e_x` 已按门宽归一化、带速度 `de_x`、带标准差 `sig_x`、带可信度 `trust`）。
过门闭环应当**优先用这边**，原始框只在它不可信时兜底。

输入（viskf 原子写，20Hz）:
    {"track":1,"trust":1,"age":0.03,"gate_visible":1,
     "e_x":-0.052,"de_x":0.011,"sig_x":0.14,
     "el":0.021,"del":0.002,"sig_el":0.004,
     "s_n":0.62,"ds_n":0.05,"sig_s":0.006,
     "flags":{"clip_lr":0,"clip_tb":0,"coasting":0,"lost":0,"att_degraded":1}, ...}

输出（read 的返回值，永远返回 dict，绝不抛）:
    {'ok','trust','age','gate_visible','e_x','de_x','sig_x',
     'el','d_el','sig_el','s_n','ds_n','sig_s','coasting','lost','att_degraded','clip','stale'}
    ok=False 时调用方**不得**做视觉闭环，只能退回原始像素伺服或定时放行。

★ 判"能不能用"看 `trust`，**不要只看 `gate_visible`**（实测踩出来的坑）
-----------------------------------------------------------------------
门离场后 `gate_visible` 会**立刻**变 False，但 coasting 窗口（0.5~2s）内 `track` 仍是 1、
滤波器**仍在往外推**。老实现在门离场 6.68s 时 `e_x` 被自由外推到 +2.38（正常上限 +0.60），
`sig_x` 到 2.40（正常 ≤0.50）。拿外推值去控舵 = 满舵乱转。
所以这里的 `ok` 默认按 `trust` 门禁（`AUV_VISKF_TRUST_GATE=True`）：
`trust` = `track` 且 `age ≤ 0.2s` 且 `sig_x ≤ 1.0`（滤波器侧算的，本模块只在字段缺失时自己兜底算）。
"""
import json
import os


class ViskfIF(object):
    """图像卡尔曼接口：read(now) -> dict"""

    def __init__(self, cfg, shm_dir=None, log=None):
        """初始化：记住 momo_viskf.json 的路径

        shm_dir 可注入（台架用它指向临时目录）；配置项一律 getattr 兜底，
        缺了就按"不可用"处理 —— 过门会自动退回原始像素伺服，不会崩。
        """
        self.cfg = cfg                                       # 配置对象（to32_config）
        self.log = log                                       # 日志函数（可为 None）
        self.shm_dir = shm_dir or str(getattr(cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.fname = str(getattr(cfg, 'AUV_VISKF_FILE', 'momo_viskf.json'))
        self.stale_s = float(getattr(cfg, 'AUV_VISKF_STALE_S', 0.5) or 0.0)
        self.trust_gate = bool(getattr(cfg, 'AUV_VISKF_TRUST_GATE', True))
        self.trust_age_s = float(getattr(cfg, 'AUV_VISKF_TRUST_AGE_S', 0.25) or 0.0)
        self.max_sig_x = float(getattr(cfg, 'AUV_VISKF_MAX_SIG_X', 1.0) or 0.0)
        self._warned = set()                                 # 同类警告只打一次

    def _warn_once(self, key, msg):
        """同类警告只打一次 —— 缺文件是常态级故障，20Hz 下必须节流"""
        if key in self._warned:
            return
        self._warned.add(key)
        if self.log:
            self.log('[viskf_if] ' + msg)

    def _empty(self, age=None):
        """统一的"没有滤波量"返回体：字段齐全且 ok=False，调用方不必判 KeyError"""
        return {'ok': False, 'trust': False, 'age': age, 'gate_visible': False,
                'e_x': None, 'de_x': None, 'sig_x': None,
                'el': None, 'd_el': None, 'sig_el': None,
                's_n': None, 'ds_n': None, 'sig_s': None,
                'coasting': False, 'lost': True, 'att_degraded': True,
                'clip': False, 'stale': True}

    @staticmethod
    def _flt(v):
        """宽容转 float：None / 非数字 / NaN 一律返回 None（NaN 进了伺服会污染一整条链）"""
        try:
            f = float(v)
        except Exception:
            return None
        if f != f:                                           # NaN
            return None
        return f

    def read(self, now):
        """读一帧滤波输出。任何异常都退化成 ok=False，绝不让调用方崩"""
        path = os.path.join(self.shm_dir, self.fname)
        if not os.path.isfile(path):
            self._warn_once('nofile', '%s 不存在 —— 图像卡尔曼没起？'
                            '（过门将退回原始像素伺服）' % path)
            return self._empty()
        try:
            st = os.stat(path)
            with open(path, 'r') as f:
                d = json.load(f)
        except Exception:
            return self._empty()                             # 读到半个 JSON：本次当没数据
        if not isinstance(d, dict):
            return self._empty()

        age = now - st.st_mtime
        if age > self.stale_s:
            self._warn_once('stale', '%s 超期 age=%.2fs > %.2fs' % (self.fname, age, self.stale_s))
            return self._empty(age)

        flags = d.get('flags') or {}
        e_x = self._flt(d.get('e_x'))
        sig_x = self._flt(d.get('sig_x'))
        track = bool(d.get('track'))
        age_out = self._flt(d.get('age'))

        # ---- 可信度：优先用滤波器自己给的 trust；字段缺失（老版本）才自己兜底算
        tr = d.get('trust', None)
        if tr is None:
            tr = bool(track
                      and (age_out is None or age_out <= self.trust_age_s)
                      and (sig_x is None or sig_x <= self.max_sig_x))
        else:
            tr = bool(tr)
        ok = bool(track and (tr if self.trust_gate else True))

        return {
            'ok': ok, 'trust': tr, 'age': age_out,
            'gate_visible': bool(d.get('gate_visible')),
            'e_x': e_x, 'de_x': self._flt(d.get('de_x')), 'sig_x': sig_x,
            'el': self._flt(d.get('el')), 'd_el': self._flt(d.get('del')),
            'sig_el': self._flt(d.get('sig_el')),
            's_n': self._flt(d.get('s_n')), 'ds_n': self._flt(d.get('ds_n')),
            'sig_s': self._flt(d.get('sig_s')),
            'coasting': bool(flags.get('coasting')), 'lost': bool(flags.get('lost')),
            'att_degraded': bool(flags.get('att_degraded')),
            'clip': bool(flags.get('clip') or flags.get('clip_lr') or flags.get('clip_tb')),
            'stale': False,
        }
