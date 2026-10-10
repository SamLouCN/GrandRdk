# -*- coding: utf-8 -*-
"""观测接口 —— 任务状态机的两个数据源，全新精简重写（数据格式与 v2.2 写端完全兼容）

VisionIF : 读 /dev/shm/momo_det_*.json → 最优目标 + 距瞄准点的像素/归一化偏差
DepthIF  : 读 /dev/shm/momo_depth.json  → 融合深度 D / 垂速 v_z / 离底净空 clearance

必守的坑（均来自 v2.2 已验证经验，重写时不许丢）：
  1. 帧号去重：同一帧不重复出控制量（写端 ~10Hz，任务 20Hz，不去重会重复计数）
  2. mtime 新鲜度：写端挂了不能拿旧帧做控制
  3. 同标签取 score 最高
  4. clip 分边判定：目标贴边时 w/h 不可信（判距/穿门判定必须避开）
  5. 读 JSON 一律 try/except 兜空 —— 写端正覆盖写时会读到半个 JSON
  6. 永远不抛异常：poll/read 任何失败都退化成 None / ok=False
"""
import json
import math
import os

import task_config as TC

# 标签别名归一化：模型类别名 → canonical 名。新模型定名后只改这张表。
CANON = {
    'door': 'gate', 'gate': 'gate', 'doorway': 'gate',
    'red-ball': 'ball', 'red_ball': 'ball', 'ball': 'ball',
    'sphere': 'ball', 'redball': 'ball', 'target-ball': 'ball',
    'yellow-ball': 'ball_y', 'yellow_ball': 'ball_y', 'yellowball': 'ball_y',
}


def canon_label(name):
    """模型输出的类别名 → canonical 名（未登记的按小写下划线原名返回）"""
    s = str(name).strip().lower().replace(' ', '_').replace('-', '_')
    return CANON.get(s, s)


def _read_json(path):
    """读可能被并发覆盖写的 JSON；返回 (obj, mtime)，失败返回 (None, None)"""
    try:
        st = os.stat(path)
        with open(path, 'r') as f:
            obj = json.load(f)
    except Exception:
        return None, None
    return (obj if isinstance(obj, dict) else None), st.st_mtime


class _WarnOnce(object):
    """同类警告只打一次：20Hz 循环里'文件不存在'会每拍触发，不节流会刷爆日志"""

    def __init__(self, log=None):
        self.log = log
        self._seen = set()

    def __call__(self, tag, msg):
        if tag in self._seen:
            return
        self._seen.add(tag)
        if self.log:
            self.log('[obs] %s: %s' % (tag, msg))


class VisionIF(object):
    """视觉接口：poll(cam, want, now, aim=None) -> dict | None

    cam  : 'front' | 'bottom'
    want : canonical 名（'ball' / 'gate' / 'ball_y'）
    aim  : 瞄准点 (px,py)；None = 画面中心
    返回 dict 含: score/cx/cy/dx/dy/ex/ey/w/h/clip*/age_s/frame
    dx/dy = 目标中心距瞄准点的带符号像素偏差（ex/ey 为归一化形式）。
    ★ 没有距离字段 —— 模型只给像素，要距离另接单目测距。
    """

    def __init__(self, shm_dir=None, log=None):
        self.shm_dir = shm_dir or TC.AUV_SHM_DIR
        self.file = {'front': TC.AUV_DET_FRONT, 'bottom': TC.AUV_DET_BOTTOM}
        self.w = float(TC.AUV_IMG_W)
        self.h = float(TC.AUV_IMG_H)
        self.sizes = {'front': (self.w, self.h),
                      'bottom': (float(getattr(TC, 'AUV_BOTTOM_IMG_W', self.w)),
                                 float(getattr(TC, 'AUV_BOTTOM_IMG_H', self.h)))}
        self.margin = float(TC.AUV_CLIP_MARGIN_PX)
        self._last_frame = {}
        self._warn = _WarnOnce(log)

    def poll(self, cam, want, now, aim=None):
        fname = self.file.get(cam)
        if fname is None:
            self._warn('cam', '未知相机名 %r（只认 front/bottom）' % cam)
            return None
        path = os.path.join(self.shm_dir, fname)
        if not os.path.isfile(path):
            self._warn('nofile_' + cam, '%s 不存在（写端没起？）' % path)
            return None
        obj, mtime = _read_json(path)
        if obj is None:
            return None                                   # 半个 JSON：当没数据
        age = (now - mtime) if mtime else 9.9
        if age > float(TC.AUV_DET_STALE_S):
            self._warn('stale_' + cam, '%s 超期 %.2fs，按无目标' % (fname, age))
            return None

        frame = obj.get('frame')
        key = (cam, str(want))
        if frame is not None and self._last_frame.get(key) == frame:
            return None                                   # 同一帧不重复出控制量
        self._last_frame[key] = frame

        best = None
        for d in (obj.get('dets') or []):
            if not isinstance(d, dict) or canon_label(d.get('label', '')) != want:
                continue
            try:
                sc = float(d.get('score', 0.0))
            except Exception:
                continue
            if sc >= float(TC.AUV_MIN_SCORE) and (best is None or sc > best[0]):
                best = (sc, d)
        if best is None:
            return None
        score, d = best

        bbox = d.get('bbox') or []
        try:
            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        except Exception:
            x1 = y1 = x2 = y2 = 0.0
        w, h = abs(x2 - x1), abs(y2 - y1)
        ctr = d.get('center') or []
        try:
            cx, cy = float(ctr[0]), float(ctr[1])
        except Exception:
            cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)

<<<<<<< HEAD
        img_w, img_h = self.sizes[cam]
        # New writers publish actual dimensions; legacy files use per-camera
        # defaults so lowering front resolution never rescales bottom output.
        for key, fallback in (('img_w', img_w), ('img_h', img_h)):
            try:
                value = float(obj.get(key, fallback))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value) or value <= 0:
                return None
            if key == 'img_w':
                img_w = value
            else:
                img_h = value
=======
        # [2026-10-10] 坐标系自检：检测框超出配置的画面宽 ⇒ AUV_IMG_W 与检测坐标系不符
        #   （典型：相机采集 1280，而模型/写端给的是 640 空间）。只提示一次，不阻断。
        #   该错配会让"中心"算成 640 而实际中心 320 → ex 恒单侧 → 对中永远对不正。
        if self.w > 0 and (cx > self.w or x2 > self.w + self.margin):
            self._warn('coord_%s' % cam,
                       '检测框 x=%.0f/右边界=%.0f 超出配置画面宽 %.0f —— AUV_IMG_W 与'
                       '检测坐标系不符？（改 task_config.AUV_IMG_W）' % (cx, x2, self.w))

>>>>>>> 9c99bbf (door适应性修改)
        if aim is None:
            aim = (0.5 * img_w, 0.5 * img_h)
        dx, dy = cx - float(aim[0]), cy - float(aim[1])
        ex = dx / (0.5 * img_w)
        ey = dy / (0.5 * img_h)

        m = self.margin
        clip_l, clip_t = x1 <= m, y1 <= m
        clip_r, clip_b = x2 >= img_w - m, y2 >= img_h - m
        return {
            'canon': want, 'label': d.get('label'), 'score': score,
            'img_w': img_w, 'img_h': img_h,
            'cx': cx, 'cy': cy, 'dx': dx, 'dy': dy, 'ex': ex, 'ey': ey,
            'w': w, 'h': h, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            'clip_l': clip_l, 'clip_r': clip_r, 'clip_t': clip_t, 'clip_b': clip_b,
            'clip': clip_l or clip_r or clip_t or clip_b,
            'age_s': age, 'frame': frame,
        }

    def fresh(self, cam, now):
        """检测链路就绪判据：写端文件存在且未超期（不要求本帧有目标）。

        用途：过门 step0 等"视觉就绪 + 模型切换完成"。front.py 在模型
        切换/加载期间直接 continue（不更新 JSON）→ mtime 停更 → fresh=False；
        恢复写帧 → fresh=True。连续 N 拍 fresh 可确认链路与模型已就绪。
        """
        fname = self.file.get(cam)
        if fname is None:
            return False
        path = os.path.join(self.shm_dir, fname)
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            return False
        return (now - mtime) <= float(TC.AUV_DET_STALE_S)


class DepthIF(object):
    """深度接口：read(now) -> dict（永远返回 dict，绝不抛）

    返回 {'ok','D','v_z','clearance','sigma_D','age_s','sample_ts','stale','degraded','H'}
    ok=False 时不得做深度闭环，只能走「定时 + 上浮」的安全路径。
    ★ clearance = 离底净空(m) 多探头取最小 —— 坐底判定的绝对量，
      不受池深、固件钳位、深度计漂移影响。
    """

    def __init__(self, shm_dir=None, log=None):
        self.shm_dir = shm_dir or TC.AUV_SHM_DIR
        self.fname = TC.AUV_DEPTH_FILE
        self._warn = _WarnOnce(log)

    def _empty(self, age=None):
        return {'ok': False, 'D': None, 'v_z': None, 'clearance': None,
                'sigma_D': None, 'age_s': age, 'stale': True, 'degraded': True,
                'H': None, 'sample_ts': None}

    def read(self, now):
        path = os.path.join(self.shm_dir, self.fname)
        if not os.path.isfile(path):
            self._warn('nofile', '%s 不存在 —— depth_kalman 没起？（任务将走定时降级）' % path)
            return self._empty()
        d, mtime = _read_json(path)
        if d is None:
            return self._empty()
        age = now - mtime if mtime else 9.9
        if age > float(TC.AUV_DEPTH_STALE_S):
            self._warn('stale', '%s 超期 %.2fs' % (self.fname, age))
            return self._empty(age)

        vals = []
        for v in (d.get('clearance') or {}).values():
            try:
                fv = float(v)
            except Exception:
                continue
            if fv == fv and fv > -1e-9:                   # 排除 NaN / 负值
                vals.append(fv)
        cl = min(vals) if vals else None

        sig = (d.get('sigma') or {}).get('D')
        try:
            sig = float(sig) if sig is not None else None
        except Exception:
            sig = None
        try:
            d_m = float(d.get('D'))
        except Exception:
            d_m = None
        try:
            v_z = float(d.get('v_z'))
        except Exception:
            v_z = None
        ok = bool(d.get('valid')) and d_m is not None \
            and (sig is None or sig < float(TC.AUV_SIGMA_D_MAX))
        return {
            'ok': ok, 'D': d_m, 'v_z': v_z, 'clearance': cl,
            'sigma_D': sig, 'age_s': age, 'stale': False,
            'degraded': bool((d.get('flags') or {}).get('degraded', cl is None)),
            'H': d.get('H'), 'sample_ts': mtime,
        }


class AltIF(object):
    """高度计接口：read(now) -> dict —— 读 momo_alt.json（read_altimeter.py 5Hz 落盘）

    返回 {'ok','age_s','ch':{ch:{'mm','status'}}}：
      ok=False = 文件缺失/解析失败/超期（本拍不做任何高度计判据）；
      mm = 该通道原始读数(mm，探头面到波束内最近物体的净空)；None = 无有效读数。
    ★ 与 DepthIF 的 clearance 不同：这里给**原始通道值**，过门判据要用原始突变
      （融合 clearance 会把门底梁的瞬态回波当野值平滑/拒掉，看不到突变）。
    """

    def __init__(self, shm_dir=None, log=None):
        self.shm_dir = shm_dir or TC.AUV_SHM_DIR
        self.fname = getattr(TC, 'AUV_ALT_FILE', 'momo_alt.json')
        self.stale_s = float(getattr(TC, 'AUV_ALT_STALE_S', 1.5))
        self._warn = _WarnOnce(log)

    def read(self, now):
        path = os.path.join(self.shm_dir, self.fname)
        if not os.path.isfile(path):
            self._warn('alt_nofile', '%s 不存在 —— read_altimeter 没起？（突变判据将不可用）' % path)
            return {'ok': False, 'age_s': None, 'ch': {}}
        d, mtime = _read_json(path)
        if d is None:
            return {'ok': False, 'age_s': None, 'ch': {}}
        age = (now - mtime) if mtime else 9.9
        if age > self.stale_s:
            self._warn('alt_stale', '%s 超期 %.2fs' % (self.fname, age))
            return {'ok': False, 'age_s': age, 'ch': {}}
        ch = {}
        for name, v in (d.get('ch') or {}).items():
            if not isinstance(v, dict):
                continue
            mm = v.get('mm')
            try:
                mm = float(mm) if mm is not None else None
            except (TypeError, ValueError):
                mm = None
            ch[str(name).upper()] = {'mm': mm, 'status': str(v.get('status', ''))}
        return {'ok': True, 'age_s': age, 'ch': ch}
