#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_vision_quad.py — 过门 CV 观测适配层（quad_cv过门落地方案_2026-10-08.md §四-3）

职责:
  door bbox（obs.VisionIF 的 det 观测）+ 干净帧（front.py 双写的
  momo_frame_front_cv.bin, MFS1 格式）→ quad_cv_det.detect 框内找 4 根红杆
  → 输出 psi / 交点中心 / 出画比 观测, 供 GateMission 的 S2 FINE 正对环使用。

设计红线（照抄 quad_cv_kit/src/cv_quad_if.py 分支②的教训）:
  * 全兜底: 读帧 / detect / 几何任何异常 → 本拍按无 CV 观测, 绝不影响 e_raw 老链路;
  * 节流: 同一帧 seq 直接复用; GATE_CV_PERIOD_S 窗口内复用上次结果（CPU 保护）;
  * 干净帧超期（写端挂了/没起）→ 无 CV 观测, 不拿旧帧喂控制;
  * e 源升级: lvl>=2 用四角对角线交点（消 psi/2·w 的 bbox 中心系统偏差）,
    lvl<2 退 bbox 中心; 归一化口径与 e_raw 完全一致（半宽归一, 已乘 E_SIGN）;
  * psi 只信 lvl=4（psi_src='edge' 主路）; psi_width 实验开关默认关, 不碰。

输出 obs 键（在基础 det 观测之上增补）:
  cv_lvl(0..4)  psi  psi_deg  q_cx q_cy  par_h  out_frac out_n  cv_diag  why
  e_raw/w_ratio/score/frame 恒有（e_raw 已按上面口径选好源）
"""
import os
import time

try:
    if __package__:
        from . import quad_cv_det as QD, frame_io, quad_geom as G
    else:
        import quad_cv_det as QD
        import frame_io
        import quad_geom as G
except Exception:                                    # pragma: no cover
    QD = frame_io = G = None

import json

CV_TAG = 'cv'


class _WarnOnce(object):
    """同类警告只打一次（20Hz 循环里刷屏是踩过的坑）"""

    def __init__(self, log=None):
        self.log = log
        self._seen = set()

    def __call__(self, tag, msg):
        if tag in self._seen:
            return
        self._seen.add(tag)
        if self.log:
            self.log('[gate_cv] %s: %s' % (tag, msg))


class GateCvVision(object):
    """door bbox + 干净帧 → 增强观测。基础键与 _VisionAdapter 老输出同构。"""

    def __init__(self, img_w=1280, img_h=720, e_sign=+1.0, fx=0.0,
                 frame_path='/dev/shm/momo_frame_front_cv.bin',
                 det_path='/dev/shm/momo_det_front.json',
                 period_s=0.12, stale_s=0.30, cv_opts=None, log=None):
        if QD is None or frame_io is None:
            raise RuntimeError('quad_cv_det / frame_io / quad_geom 不可用（文件没部署或 import 失败）')
        self.w = float(img_w)
        self.h = float(img_h)
        self.e_sign = float(e_sign)
        self.fx = float(fx or 0.0)
        self.frame_path = str(frame_path)
        self.det_path = str(det_path)
        self.period_s = float(period_s)
        self.stale_s = float(stale_s)
        self.log = log
        self._warn = _WarnOnce(log)

        # detect 阈值: DEFAULTS 打底, GATE_CV_OPTS 覆盖（调参唯一入口）
        self.cv_opts = dict(QD.DEFAULTS)
        if isinstance(cv_opts, dict) and cv_opts:
            self.cv_opts.update(dict(cv_opts))
        self.cv_opts.setdefault('fx', self.fx)

        # 运行时状态 / 诊断
        self._seq = None            # 上次 detect 用的帧 seq
        self._res = None            # 上次 detect 结果（节流复用）
        self._t = 0.0               # 上次 detect 的墙钟
        self._n_lvl = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
        self._n_frame_miss = 0

    # ------------------------------------------------------------ 主接口
    def observe(self, det_obs, now):
        """det_obs: obs.VisionIF 的 door 观测（须含 x1..y2/w/score/frame）。
        返回基础键 + CV 增补键的 dict; 任何异常退纯 bbox 口径, 绝不抛。"""
        try:
            x1, y1, x2, y2 = (float(det_obs[k]) for k in ('x1', 'y1', 'x2', 'y2'))
        except Exception:
            return self._base(det_obs, None)
        w_img, h_img = self.w, self.h
        cx_bbox = 0.5 * (x1 + x2)
        e_bbox = ((cx_bbox - 0.5 * w_img) / (0.5 * w_img)) * self.e_sign
        base = self._base(det_obs, e_bbox)

        r = None
        try:
            r = self._detect_cached((x1, y1, x2, y2), now)
        except Exception as e:                       # 任何异常都不许影响老链路
            self._warn('cv_err', 'CV 四边形解析异常（按无四边形处理）: %s' % e)
        if r is None:
            base['cv_lvl'] = 0
            return base

        lvl = int(r.get('lvl') or 0)
        self._n_lvl[lvl] = self._n_lvl.get(lvl, 0) + 1
        f = r.get('fields') or {}
        d = r.get('diag') or {}
        corners = r.get('corners')

        base['cv_lvl'] = lvl
        base['why'] = r.get('why')
        if lvl <= 0:
            return base

        # ---- psi（lvl=4 主路才有; psi_width 实验路默认关）----
        psi = f.get('psi')
        base['psi'] = float(psi) if psi is not None else None
        base['psi_deg'] = (float(f['psi_deg']) if f.get('psi_deg') is not None else None)

        # ---- 交点中心（lvl>=2 才优于 bbox 中心）----
        qcx, qcy = f.get('cx'), f.get('cy')
        if lvl >= 2 and qcx is not None and qcy is not None:
            base['q_cx'], base['q_cy'] = float(qcx), float(qcy)
            base['e_raw'] = ((float(qcx) - 0.5 * w_img) / (0.5 * w_img)) * self.e_sign
            base['geom_src'] = CV_TAG
        else:
            base['geom_src'] = 'bbox'

        # ---- 出画比 / 矩形确认位 ----
        if corners and G is not None:
            base['out_n'], base['out_tot'] = G.out_of_frame(
                [tuple(p) for p in corners], w_img, h_img, margin=0.0)
            base['out_frac'] = (float(base['out_n']) / float(base['out_tot'])
                                if base['out_tot'] else 0.0)
        if corners and G is not None:
            try:
                base['par_h'] = G.par_deg([tuple(p) for p in corners])[0]
            except Exception:
                base['par_h'] = None

        # ---- 现场排障小字典 ----
        base['cv_diag'] = {'src': r.get('src'), 'lines': d.get('lines'),
                           'red_frac': d.get('red_frac'), 'why': r.get('why'),
                           'fill': d.get('band_fill'), 'sup': d.get('line_support'),
                           'clip_t': d.get('clip_t'), 'clip_b': d.get('clip_b')}
        return base

    # ------------------------------------------------------------ 内部
    def _base(self, det_obs, e_raw):
        """无 CV 增补时的基础观测（与老 _VisionAdapter 输出同键）。"""
        return {'e_raw': e_raw, 'w_ratio': det_obs.get('w_ratio'),
                'score': det_obs.get('score', 0), 'frame': det_obs.get('frame'),
                'cv_lvl': 0, 'psi': None, 'psi_deg': None,
                'q_cx': None, 'q_cy': None, 'par_h': None,
                'out_n': 0, 'out_tot': 0, 'out_frac': None,
                'par_h': None, 'geom_src': 'bbox', 'cv_diag': None, 'why': None}

    def _detect_cached(self, bbox, now):
        """读最新干净帧 → detect; 同帧 / 节流窗口内复用上次结果。"""
        meta = frame_io.read_latest(self.frame_path)
        if meta is None or not meta.get('jpeg'):
            self._n_frame_miss += 1
            self._warn('cv_frame', '%s 读不到帧（front.py 没起?）' % self.frame_path)
            return None
        seq = meta.get('seq')
        if self._res is not None and seq == self._seq:
            return self._res                         # 同一帧: 直接复用
        if (self._res is not None and self.period_s > 0.0
                and (now - self._t) < self.period_s):
            return self._res                         # 节流窗口内: 复用（CPU 保护）
        ts = meta.get('ts_us')
        if ts and (now - ts / 1e6) > self.stale_s:
            self._warn('cv_stale', '干净帧超期 %.2fs, 按无 CV 观测' % (now - ts / 1e6))
            return None
        img = frame_io.decode(meta['jpeg'])
        if img is None:
            self._n_frame_miss += 1
            return None
        r = QD.detect(img, bbox, opts=self.cv_opts,
                      ball_boxes=self._ball_boxes(), img_wh=(int(self.w), int(self.h)))
        self._seq, self._res, self._t = seq, r, now
        return r

    def _ball_boxes(self):
        """本帧 red-ball det（保险用: 真门内不会有球, 挖洞只防遮挡误检）; 读不到就空。"""
        out = []
        try:
            with open(self.det_path, 'rb') as fp:
                obj = json.loads(fp.read().decode('utf-8', 'replace'))
        except Exception:
            return out
        for d in (obj.get('dets') or []):
            try:
                lab = str(d.get('label', '')).strip().lower().replace('-', '_')
                if lab in ('red_ball', 'ball'):
                    b = d.get('bbox')
                    if b and len(b) == 4:
                        out.append([float(v) for v in b])
            except Exception:
                pass
        return out

    def stats(self):
        """lvl 直方图 + 帧缺失计数, 供日志。"""
        lv = self._n_lvl
        return ('[gate_cv] lvl4/3/2/1/0=%d/%d/%d/%d/%d frame_miss=%d'
                % (lv.get(4, 0), lv.get(3, 0), lv.get(2, 0),
                   lv.get(1, 0), lv.get(0, 0), self._n_frame_miss))
