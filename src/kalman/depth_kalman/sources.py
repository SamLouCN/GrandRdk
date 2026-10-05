# -*- coding: utf-8 -*-
"""数据源：读 ``momo_telemetry.json`` 与 ``momo_alt.json``。

设计要点
--------
* **新鲜度用文件 mtime 判**，不用 JSON 里的 ``ts`` —— 写端挂了，读端不能一直吃旧帧。
* 解析**宽容**：字段缺失/类型不对/文件被写坏（json 解析失败）都退化成"本次无观测"，绝不抛异常。
* 每个样本带 ``token``（``(mtime_ns, size)``），上层据此做**事件驱动**更新，避免同一帧被重复吃。
* 只读，不写、不建、不删任何文件。
"""
import json
import os

from samples import AltSample, Telemetry, deg_to_rad

_OK = ('ok',)


class ShmReader(object):
    def __init__(self, cfg, log=None, root=None):
        self.cfg = cfg
        self.log = log
        self.root = root or getattr(cfg, 'SHM_DIR', '/dev/shm')
        self.path_telem = os.path.join(self.root, getattr(cfg, 'SHM_TELEM', 'momo_telemetry.json'))
        self.path_alt = os.path.join(self.root, getattr(cfg, 'SHM_ALT', 'momo_alt.json'))
        self.warned = {}
        self._cache = {}

    # ------------------------------------------------------------ 基础
    def _warn_once(self, key, msg):
        if self.warned.get(key):
            return
        self.warned[key] = True
        if self.log is not None:
            self.log.warn(msg)

    def _read_json(self, path):
        """返回 (obj, mtime_ns, size, age_s)。失败返回 (None, None, 0, inf)。

        按 ``(mtime_ns, size)`` 缓存解析结果 —— 文件没变就不重复 json.loads（50 Hz 循环友好）。
        """
        try:
            st = os.stat(path)
        except OSError:
            self._cache.pop(path, None)
            return None, None, 0, float('inf')
        age = max(0.0, _now() - st.st_mtime)
        c = self._cache.get(path)
        if c is not None and c[0] == st.st_mtime_ns and c[1] == st.st_size:
            return c[2], st.st_mtime_ns, st.st_size, age
        try:
            with open(path, 'rb') as f:
                raw = f.read()
            obj = json.loads(raw.decode('utf-8', 'replace'))
        except (OSError, ValueError):
            # 写端正在写（非原子写）或内容损坏 —— 本次当无数据
            return None, None, st.st_size, age
        if not isinstance(obj, dict):
            return None, None, st.st_size, age
        self._cache[path] = (st.st_mtime_ns, st.st_size, obj)
        return obj, st.st_mtime_ns, st.st_size, age

    # ------------------------------------------------------------ 遥测
    def read_telemetry(self):
        cfg = self.cfg
        obj, mtime_ns, size, age = self._read_json(self.path_telem)
        if obj is None:
            self._warn_once('tel_missing',
                            '遥测落点无数据：%s（等待 To32 侧写入）' % self.path_telem)
            return Telemetry(stale=True, age=age)

        token = (mtime_ns, size)
        tel = Telemetry(token=token, age=age,
                        stale=age > float(cfg.TEL_STALE_S))
        ts = obj.get('ts')
        tel.ts = float(ts) if isinstance(ts, (int, float)) else None

        raw = _first(obj, ('depth_raw', 'depthRaw', 'depth_raw_cm100', 'actual_depth_raw'))
        if raw is None:
            dm = _first(obj, ('depth_m', 'depth', 'actual_depth_m'))
            if isinstance(dm, (int, float)):
                tel.depth_m = float(dm) - float(getattr(cfg, 'DEPTH_ZERO_OFFSET', 0.0))
                tel.depth_raw = None
        if raw is not None:
            tel.depth_raw = raw
            tel.depth_m = _model_depth_to_m(cfg, raw)

        ax = _first(obj, ('acc_x', 'ax', 'accX'))
        ay = _first(obj, ('acc_y', 'ay', 'accY'))
        az = _first(obj, ('acc_z', 'az', 'accZ'))
        acc = _first(obj, ('acc',))
        if isinstance(acc, dict):
            ax = ax if ax is not None else _first(acc, ('x', 'ax'))
            ay = ay if ay is not None else _first(acc, ('y', 'ay'))
            az = az if az is not None else _first(acc, ('z', 'az'))
        tel.acc = _accel_to_ms2(cfg, ax, ay, az)

        pitch = _first(obj, ('pitch', 'actual_pitch', 'pitch_raw'))
        roll = _first(obj, ('roll', 'actual_roll', 'roll_raw'))
        yaw = _first(obj, ('yaw', 'actual_yaw', 'yaw_raw'))
        att = _first(obj, ('att', 'attitude'))
        if isinstance(att, dict):
            pitch = pitch if pitch is not None else _first(att, ('pitch', 'p'))
            roll = roll if roll is not None else _first(att, ('roll', 'r'))
            yaw = yaw if yaw is not None else _first(att, ('yaw', 'y'))
        if pitch is None and roll is None:
            tel.has_att = False
        else:
            tel.has_att = True
            tel.pitch_rad = _att_to_rad(cfg, pitch)
            tel.roll_rad = _att_to_rad(cfg, roll)
            tel.yaw_rad = _att_to_rad(cfg, yaw)

        tel.acc_fresh = (tel.acc is not None) and (age <= float(cfg.ACCEL_STALE_S))
        return tel

    # ------------------------------------------------------------ 高度计
    def read_alt(self):
        cfg = self.cfg
        obj, mtime_ns, size, age = self._read_json(self.path_alt)
        out = {}
        if obj is None:
            self._warn_once('alt_missing',
                            '高度计落点无数据：%s（等待 read_altimeter 侧写入）' % self.path_alt)
            return out

        body = obj.get('ch') if isinstance(obj.get('ch'), dict) else obj
        stale = age > float(cfg.ALT_STALE_S)
        for ch, v in body.items():
            if not isinstance(ch, str) or len(ch) != 1 or not ch.isalpha():
                continue
            ch = ch.upper()
            mm = None
            status = ''
            if isinstance(v, dict):
                mm = _first(v, ('mm', 'clearance_mm', 'value', 'dist_mm'))
                status = str(v.get('status', '') or '')
            elif isinstance(v, (int, float)):
                mm = v
                status = 'OK'
            ok = (str(status).strip().lower() in _OK) and isinstance(mm, (int, float))
            clr = (float(mm) / 1000.0) if isinstance(mm, (int, float)) else None
            out[ch] = AltSample(ch=ch, token=(mtime_ns, size, ch), ts=obj.get('ts'),
                                age=age, clearance_m=clr, status=status, ok=ok, stale=stale)
        return out


# ---------------------------------------------------------------- 工具
def _now():
    import time
    return time.time()


def _first(obj, keys):
    for k in keys:
        if k in obj:
            v = obj[k]
            if v is not None:
                return v
    return None


def _model_depth_to_m(cfg, raw):
    """与 ``Model.depth_raw_to_m`` 同规则（这里独立实现，避免 sources 依赖 model）。"""
    try:
        r = float(raw)
    except (TypeError, ValueError):
        return None
    if r != r:
        return None
    if r < cfg.DEPTH_RAW_MIN_VALID or r > cfg.DEPTH_RAW_MAX_VALID:
        return None
    return r * cfg.DEPTH_SCALE - float(getattr(cfg, 'DEPTH_ZERO_OFFSET', 0.0))


def _accel_to_ms2(cfg, ax, ay, az):
    unit = str(getattr(cfg, 'ACCEL_UNIT', 'unknown')).strip().lower()
    if unit in ('g', 'grav', 'gravity'):
        gscale = 9.80665
    elif unit in ('m/s^2', 'm/s2', 'ms2', 'si'):
        gscale = 1.0
    else:
        return None          # 单位未标定 → 直接不用加速度
    if ax is None or ay is None or az is None:
        return None
    try:
        vals = [float(ax), float(ay), float(az)]
    except (TypeError, ValueError):
        return None
    s = float(getattr(cfg, 'ACCEL_RAW_SCALE', 0.01)) * gscale
    sx, sy, sz = getattr(cfg, 'ACCEL_SIGN', (1.0, 1.0, 1.0))
    return (vals[0] * s * sx, vals[1] * s * sy, vals[2] * s * sz)


def _att_to_rad(cfg, v):
    if v is None:
        return 0.0
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if getattr(cfg, 'ATT_RAW_IS_DEGX100', True):
        f = f / 100.0
    return deg_to_rad(f)
