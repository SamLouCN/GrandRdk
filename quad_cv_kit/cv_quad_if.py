# -*- coding: utf-8 -*-
"""[red_pole_cv 2026-10-06] 分支② 视觉接口 —— ψ 的供应商从「连接件 det」换成「框内 OpenCV 找红杆」

════════════════════════════════════════════════════════════════════════
它解决什么（用户 2026-10-06 口径）
────────────────────────────────────────────────────────────────────────
  「原来的 AUV 过门任务没有有效的视线自动调整方向」—— 根因是：现役模型只输出
  `door/red-ball/yellow-ball` 三个**整框**类别，没有「连接件」det ⇒ 分支①的
  `QuadVisionIF._corner_dets()` 永远收不到候选 ⇒ `_legacy_quad` 也凑不出四角
  ⇒ `psi` 恒为 None ⇒ `gate_quad.psi_err()` 取不到值 ⇒ **正对伺服从未工作过**。

  本类把 ψ 的来源换成 `quad_cv_det.detect()`：在 YOLO `door` bbox 内用 OpenCV
  找 4 根红色 PVC 杆 → 四边形 → ψ（竖边像长比）。**不依赖任何新模型输出**，
  现役 180fps 的 front.py 检测链原样可用。

接线（为什么 mode_auv.py 只改一行 import）
────────────────────────────────────────────────────────────────────────
  mode_auv.py:  from cv_quad_if import make_vision_if   # 原来从 quad_vision 导入
  选择优先级：  AUV_QUAD_CV_ENABLE=True  → 本类（CV 找红杆）
                否则                     → quad_vision.make_vision_if（分支①连接件 → 整门 bbox）
  回退：本类构造/detect 出任何异常都退回下一级；**绝不让 AUV 起不来**。

obs 契约（与分支① `_attach_quad` 逐键对齐，下游 gate_quad.py **零改动**）
────────────────────────────────────────────────────────────────────────
  quad_ok / quad_lvl / kpt_ok   ← detect 的 lvl（4=ψ 可用，语义与 gate_quad 对齐）
  quad                          ← 4 角（全画幅坐标；lvl<4 时缺省）
  psi / psi_src / psi_deg       ← 竖边像长比；fx=AUV_QUAD_FX_PX 时给角度
  q_cx/q_cy/q_dx/q_dy/q_ex      ← 对角线交点中心（bbox 中心的偏航偏差已消掉）
  cx/cy/dx/dy/ex/ey + geom_src  ← lvl≥2 时改用 CV 口径（分支①同规矩）
  clip_l/r/t/b / kpt_clip       ← bbox 贴边保留基类值；quad 角点存在时用角点 refine
  out_n/out_tot/out_frac        ← 穿门判据（角点出画数）
  cv_diag                       ← 现场排障小字典（lines/red_frac/why）

帧来源与对齐
────────────────────────────────────────────────────────────────────────
  front.py 每帧写 `/dev/shm/momo_frame_front.bin`（MFS1 头 + JPEG，见 frame_io.py）。
  2026-10-06 起 front.py 写**干净帧**（黄框只留给显示）——分支②的 OpenCV 要吃
  原始像素，2px 黄框正压红杆（3cm 杆 2m 处仅 ~8px）。
  bbox 与帧天然对齐：front.py 同一轮循环里先写帧再写 det json，错位 ≤1 帧（5.5ms）。
  节流：`AUV_CV_PERIOD_S`（默认 0.10s）内复用上次 detect 结果 —— JPEG 解码+detect
  在 ARM 上约 30~60ms，不给 20Hz tick 添 CPU 压力。
"""
from __future__ import annotations

import math
import time

# quad_vision 是分支①现役模块（板端已部署且 mode_auv 直接 import 它）
# ⇒ 它挂了 AUV 本来就起不来，这里不需要比它更强的保护。
from quad_vision import QuadVisionIF, VisionIF  # noqa: F401  (VisionIF 供 make_vision_if 回退链)

# quad_cv_det / frame_io 是本分支**新增**文件 —— 缺文件/坏版本都不许拖死 AUV
try:
    import quad_cv_det as QD
    import frame_io
except Exception:                                      # pragma: no cover
    QD = None
    frame_io = None

try:
    import quad_geom as G
except Exception:                                      # pragma: no cover
    G = None

CV_TAG = 'cv'                                          # obs['quad_src'] / geom_src 标记


class CvQuadVisionIF(QuadVisionIF):
    """对外契约与 VisionIF/QuadVisionIF 完全一致；四边形改由框内 OpenCV 解出"""

    def __init__(self, cfg, shm_dir=None, log=None):
        QuadVisionIF.__init__(self, cfg, shm_dir=shm_dir, log=log)
        if QD is None:
            raise RuntimeError('quad_cv_det / frame_io 不可用（文件没部署或 import 失败）')

        # ---- 帧源与节流
        self.frame_path = str(getattr(cfg, 'AUV_CV_FRAME_PATH',
                                      '/dev/shm/momo_frame_front.bin'))
        self.period_s = float(getattr(cfg, 'AUV_CV_PERIOD_S', 0.10) or 0.0)

        # ---- detect 阈值：DEFAULTS 打底，AUV_CV_OPTS 覆盖（调参唯一入口）
        self.cv_opts = dict(QD.DEFAULTS)
        extra = getattr(cfg, 'AUV_CV_OPTS', None)
        if isinstance(extra, dict) and extra:
            self.cv_opts.update(dict(extra))
        self.cv_opts.setdefault('fx', self.fx)         # 与分支①同一把 fx（AUV_QUAD_FX_PX）

        # ---- 运行时状态 / 诊断计数
        self._cv_seq = None                            # 上次 detect 用的帧 seq
        self._cv_res = None                            # 上次 detect 结果（节流复用）
        self._cv_t = 0.0                               # 上次 detect 的墙钟
        self._n_cv = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}    # lvl 直方图
        self._n_frame_miss = 0                         # 帧读失败次数（front 没起/坏帧）

    # ------------------------------------------------------------ 主接口
    def poll(self, cam, want, now, aim=None):
        """与 QuadVisionIF.poll 同构，但**没有 door det 就直接无观测**：
        CV 路没有 bbox 就没有 ROI，无从下手（连接件路的 R2 自建 obs 在这里不适用）。"""
        obs = VisionIF.poll(self, cam, want, now, aim=aim)      # 老链路：det json → bbox obs
        if not self.enable:
            if obs is not None:
                self._clear_new(obs)
            return obs
        if obs is None:
            return None                                          # 本帧没有 door det ⇒ 无观测
        self._n_poll += 1
        try:
            self._attach_quad(obs, aim, now)
        except Exception as e:                                   # 任何异常都不许影响老链路
            self._warn_once('cv_err', 'CV 四边形解析异常（按无四边形处理）：%s' % e)
            self._clear_new(obs)
        return obs

    # ------------------------------------------------------------ detect（带缓存）
    def _detect_cached(self, bb, W, H):
        """读最新干净帧 → quad_cv_det.detect；同帧/节流窗口内复用上次结果"""
        meta = frame_io.read_latest(self.frame_path)
        if meta is None or not meta.get('jpeg'):
            self._n_frame_miss += 1
            self._warn_once('cv_frame', '%s 读不到帧（front.py 没起？）' % self.frame_path)
            return None
        seq = meta.get('seq')
        now = time.time()
        if self._cv_res is not None and seq == self._cv_seq:
            return self._cv_res                                  # 同一帧：直接复用
        if (self._cv_res is not None and self.period_s > 0.0
                and (now - self._cv_t) < self.period_s):
            return self._cv_res                                  # 节流窗口内：复用（CPU 保护）
        img = frame_io.decode(meta['jpeg'])
        if img is None:
            self._n_frame_miss += 1
            return None
        r = QD.detect(img, bb, opts=self.cv_opts,
                      ball_boxes=self._ball_boxes(), img_wh=(W, H))
        self._cv_seq, self._cv_res, self._cv_t = seq, r, now
        return r

    def _ball_boxes(self):
        """本帧 red-ball det（保险用：真门内不会有球，挖洞只防遮挡误检）"""
        out = []
        for d in ((self._raw or {}).get('dets') or []):
            try:
                if str(d.get('label')).strip().lower() in ('red-ball', 'red_ball', 'ball'):
                    b = d.get('bbox')
                    if b and len(b) == 4:
                        out.append([float(v) for v in b])
            except Exception:
                pass
        return out

    # ------------------------------------------------------------ 核心：把 detect 结果挂到 obs
    def _attach_quad(self, obs, aim, now):
        self._clear_new(obs)

        # door bbox：基类 obs 已带（x1..y2 全画幅坐标）
        try:
            bb = (float(obs['x1']), float(obs['y1']),
                  float(obs['x2']), float(obs['y2']))
        except Exception:
            obs['quad_src'] = 'none'
            return

        W, H = int(self.w), int(self.h)
        r = self._detect_cached(bb, W, H)
        if r is None:
            obs['quad_src'] = 'none'
            return

        lvl = int(r.get('lvl') or 0)
        self._n_cv[lvl] = self._n_cv.get(lvl, 0) + 1
        obs['quad_src'] = CV_TAG if lvl > 0 else 'none'
        if lvl <= 0:
            return                                               # 无观测：保留基类 bbox 字段

        f = r.get('fields') or {}
        d = r.get('diag') or {}
        corners = r.get('corners')

        # ---- lvl / ψ（gate_quad 的三要素：quad_ok / psi_deg|psi / q_ex|ex）
        obs['quad_lvl'] = lvl
        obs['kpt_ok'] = lvl                                      # 兼容 R1
        obs['quad_ok'] = bool(lvl >= 4)
        if corners:
            obs['quad'] = [(float(p[0]), float(p[1])) for p in corners]
        psi = f.get('psi')
        if psi is not None:
            obs['psi'] = float(psi)
            obs['psi_src'] = str(r.get('psi_src') or f.get('psi_src') or 'edge')
            pd = f.get('psi_deg')
            obs['psi_deg'] = float(pd) if pd is not None else None
        else:
            obs['psi'] = None
            obs['psi_src'] = None
            obs['psi_deg'] = None

        # ---- 中心 / 对中（对角线交点；lvl<4 时是 bbox/可见线中心）
        px = float(aim[0]) if aim else 0.5 * self.w
        py = float(aim[1]) if aim else 0.5 * self.h
        cx, cy = f.get('cx'), f.get('cy')
        span_w = float(f.get('w') or 0.0)
        span_h = float(f.get('h') or 0.0)
        if cx is not None and cy is not None:
            obs['q_cx'], obs['q_cy'] = float(cx), float(cy)
            obs['q_dx'], obs['q_dy'] = float(cx) - px, float(cy) - py
            if span_w > 1.0:
                obs['q_ex'] = obs['q_dx'] / span_w
                obs['q_ey'] = (obs['q_dy'] / span_h) if span_h > 1.0 else None
            obs['q_w'], obs['q_h'] = span_w, span_h
            obs['area'] = f.get('area')

        # ---- 出画（穿门判据；有完整四角才算）
        if corners and G is not None:
            obs['out_n'], obs['out_tot'] = G.out_of_frame(
                obs['quad'], self.w, self.h, margin=self.margin)
            obs['out_frac'] = (float(obs['out_n']) / float(obs['out_tot'])
                               ) if obs['out_tot'] else 0.0

        # ---- 贴边：bbox 口径基类已给；纵向用 detect 的 clip_t/b 补充
        obs['clip_t'] = bool(obs.get('clip_t')) or bool(d.get('clip_t'))
        obs['clip_b'] = bool(obs.get('clip_b')) or bool(d.get('clip_b'))
        obs['clip'] = bool(obs.get('clip_l') or obs.get('clip_r')
                           or obs['clip_t'] or obs['clip_b'])
        obs['kpt_clip'] = bool(obs['clip_t'] or obs['clip_b'])

        # ---- 老几何字段改用 CV 口径（lvl≥2 才改；lvl=1 保留 bbox/基类，走贴边定向转）
        obs['cx_bbox'], obs['cy_bbox'] = obs.get('cx'), obs.get('cy')
        obs['ex_bbox'] = obs.get('ex')
        if lvl >= 2 and cx is not None and cy is not None:
            obs['cx'], obs['cy'] = obs['q_cx'], obs['q_cy']
            obs['dx'], obs['dy'] = obs['q_dx'], obs['q_dy']
            obs['ex'] = obs['dx'] / (0.5 * self.w) if self.w > 0 else 0.0
            obs['ey'] = obs['dy'] / (0.5 * self.h) if self.h > 0 else 0.0
            obs['geom_src'] = CV_TAG
        else:
            obs['geom_src'] = 'bbox'

        # ---- 现场排障小字典
        obs['cv_diag'] = {'det_src': r.get('src'), 'lines': d.get('lines'),
                          'red_frac': d.get('red_frac'), 'why': r.get('why'),
                          'fill': d.get('band_fill'), 'sup': d.get('line_support')}

    # ------------------------------------------------------------ 诊断
    def stats(self):
        """父类 stats 若在（新版 quad_vision）则拼接；旧版（v2.4 板端）没有 ⇒ 只打 CV 段"""
        try:
            base = QuadVisionIF.stats(self)
        except AttributeError:
            base = '[quad] (board quad_vision v2.4 无 stats)'
        lv = self._n_cv
        return ('%s [cv] lvl4/3/2/1/0=%d/%d/%d/%d/%d frame_miss=%d last_psi=%s'
                % (base, lv.get(4, 0), lv.get(3, 0), lv.get(2, 0),
                   lv.get(1, 0), lv.get(0, 0), self._n_frame_miss,
                   ((self._cv_res or {}).get('fields') or {}).get('psi')))


# ================================================================= 工厂
def make_vision_if(cfg, log=None, shm_dir=None):
    """★ mode_auv.py 只认它。优先级：CV 找红杆 → 分支①连接件/四边形 → 整门 bbox。

    回退条件（逐级）：
      * `AUV_QUAD_CV_ENABLE` 为 False ⇒ 直接走分支①
      * quad_cv_det/frame_io 缺文件或构造异常 ⇒ 退分支①（打一条日志）
    """
    try:
        if QD is not None and bool(getattr(cfg, 'AUV_QUAD_CV_ENABLE', False)):
            return CvQuadVisionIF(cfg, shm_dir=shm_dir, log=log)
    except Exception as e:
        if log:
            log('[red_pole_cv] CV 视觉不可用，退回四边形/整门 bbox：%s' % e)
    from quad_vision import make_vision_if as _quad_make
    return _quad_make(cfg, log=log, shm_dir=shm_dir)
