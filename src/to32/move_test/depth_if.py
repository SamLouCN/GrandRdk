# -*- coding: utf-8 -*-
"""深度接口 —— 读 /dev/shm/momo_depth.json（depth_kalman 写的融合深度）

[AUV-MISSION 2026-09-26 新增] 任务脚本的深度来源。
**主深度源是 depth_kalman 的融合输出，不是遥测 raw** —— 遥测链路当前不通（0 帧），
而 depth_kalman 在无遥测时可以靠「高度计 + H_M(known)」单独出绝对深度
（D = H_eff − clearance），这正是本模块存在的理由。

输入（depth_kalman 原子写，20Hz）:
    {"ts":..,"valid":true,"D":1.0234,"v_z":0.012,"H":1.30,
     "sigma":{"D":0.008,...},"clearance":{"B":0.276,"C":0.281},
     "sources":{"telem_stale":false,...},"flags":{"degraded":false,...}}

输出（read 的返回值，永远返回 dict，绝不抛）:
    {'ok','D','v_z','clearance','sigma_D','age_s','stale','degraded','H'}
    ok=False 时调用方不得做深度闭环，只能走「定时 + 上浮」的安全路径。

★ clearance = 离底净空（m），多探头取最小值 —— 坐底判定用它，是绝对量，
  不受池深、固件钳位、深度计漂移影响。
"""
import json
import os


class DepthIF(object):
    """深度接口：read(now) -> dict"""

    def __init__(self, cfg, shm_dir=None, log=None):
        """初始化：记住 momo_depth.json 的路径与读目录

        shm_dir 可注入（台架用它指向临时目录）；配置项一律 getattr 兜底。
        """
        self.cfg = cfg                                       # 配置对象（to32_config）
        self.log = log                                       # 日志函数（可为 None）
        self.shm_dir = shm_dir or str(getattr(cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.fname = str(getattr(cfg, 'AUV_DEPTH_FILE', 'momo_depth.json'))
        self._warned = set()                                 # 同类警告只打一次

    def _warn_once(self, key, msg):
        """同类警告只打一次 —— 深度源缺失是常态级故障，20Hz 下必须节流"""
        if key in self._warned:
            return
        self._warned.add(key)
        if self.log:
            self.log('[depth_if] ' + msg)

    def _empty(self, age=None):
        """统一的"没有深度"返回体：字段齐全且 ok=False，调用方不必判 KeyError"""
        return {'ok': False, 'D': None, 'v_z': None, 'clearance': None,
                'sigma_D': None, 'age_s': age, 'stale': True, 'degraded': True,
                'H': None}

    def read(self, now):
        """读一帧融合深度。任何异常都退化成 ok=False，绝不让调用方崩"""
        path = os.path.join(self.shm_dir, self.fname)
        if not os.path.isfile(path):
            self._warn_once('nofile', '%s 不存在 —— depth_kalman 没起？'
                            '（无深度源时任务脚本会走定时降级路径）' % path)
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
        stale_s = float(getattr(self.cfg, 'AUV_DEPTH_STALE_S', 1.0))
        if age > stale_s:
            self._warn_once('stale', '%s 超期 age=%.2fs > %.2fs' % (self.fname, age, stale_s))
            return self._empty(age)

        # ---- 净空：多探头取最小值（最保守）
        cl = None
        raw_cl = d.get('clearance') or {}
        vals = []
        for v in raw_cl.values():
            try:
                fv = float(v)
            except Exception:
                continue
            if fv == fv and fv > -1e-9:                      # 排除 NaN / 负值
                vals.append(fv)
        if vals:
            cl = min(vals)

        sig = (d.get('sigma') or {}).get('D', None)
        try:
            sig = float(sig) if sig is not None else None
        except Exception:
            sig = None
        sig_max = float(getattr(self.cfg, 'AUV_SIGMA_D_MAX', 0.05))
        flags = d.get('flags') or {}
        try:
            d_m = float(d.get('D'))
        except Exception:
            d_m = None
        try:
            v_z = float(d.get('v_z'))
        except Exception:
            v_z = None

        ok = bool(d.get('valid')) and d_m is not None \
            and (sig is None or sig < sig_max)
        return {
            'ok': bool(ok), 'D': d_m, 'v_z': v_z, 'clearance': cl,
            'sigma_D': sig, 'age_s': age, 'stale': False,
            'degraded': bool(flags.get('degraded', cl is None)),
            'H': d.get('H'),
        }
