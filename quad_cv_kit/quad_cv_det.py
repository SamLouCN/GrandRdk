# -*- coding: utf-8 -*-
"""red_pole_cv 核心 —— 在 YOLO `door` 的 bbox 内用 OpenCV 找 4 根红杆 → 四边形 + lvl

════════════════════════════════════════════════════════════════════════
设计约束（与分支① `gate_kpt` 同构，照抄它的教训）
────────────────────────────────────────────────────────────────────────
  * **纯函数**：不读配置、不写文件、不做任何 I/O。所有阈值走 `opts` 字典
    ⇒ 板端数据还没到也能先写完 + 离线合成图单测（这就是本文件先于 Stage 0 落地的原因）。
  * 只依赖 `numpy` + `cv2`；几何量（psi / 门中心 / 面积）**复用 `quad_geom`**
    （分支①的产物，v2.4 已在板端 `src/to32/` 下）；**拿不到就退回本文件的兜底实现**
    —— 少一个文件整条链断，是踩过的教训。
  * **宁可不出（lvl 0/1）也不出错 ψ**：错的 ψ 会把机头往错方向拧，比"不转弯"更危险。

════════════════════════════════════════════════════════════════════════
分工（详见 `../可行性分析_2026-10-06.md` §3.5）
────────────────────────────────────────────────────────────────────────
  YOLO `door` det  → **门在哪**：给出 ROI（全画幅里唯一能分开 门/红球/引导线/收集框 的东西）
  本模块（框内）  → **门朝哪**：解出 4 根杆的中心线 → 4 个交点 → psi + 门中心
  ⚠ bbox 轴对齐 ⇒ 宽高比对 ψ 是**偶函数** ⇒ 单靠 bbox 判不出 ψ，故必须在框内再做这一步。

════════════════════════════════════════════════════════════════════════
lvl 口径（分支② 口径：以**能力**为准，不是数角点）
────────────────────────────────────────────────────────────────────────
  4 = **ψ 可用（正对项可用）**。两种来路：
        · `psi_src='edge'`  四角齐全 + 结构校验通过 ⇒ 竖边像长比（**主路**）
        · `psi_src='width'` **纵向残缺**时 ⇒ 左右立柱**像宽比**
          ⚠ **实验开关，`psi_width_on` 默认关**（真实帧实测符号一致率仅 ~0.50）
  3 = 有门中心、ψ 不可靠 ⇒ 只给对中项（**纵向残缺且横杆整个出画时落在这里**）
  2 = 红度够但线凑不齐 ⇒ 只有横向位置
  1 = 门被画面**横向**切掉 ⇒ 只能"朝可见部分定向转"把门拉回画面（**别拿半个门算 ψ**）
  0 = 本帧无观测（下游退回今天的整门 bbox 行为）

★★ 为什么 4 ≠ "四角齐全"（2026-10-06 用户口径）
  本分支**只用偏航对准**（定深过门），上下横杆出画**不影响 ψ** ⇒ 不该一刀切降级
  （真实 344 帧里 `clip_v` 占 30%）。但**也不能直接拿被截断的四角算 ψ** ——
  被切那侧的横杆不在画面里，该带只剩两根立柱截面 ⇒ 拟合出**假线**；它让"竖边像长比"
  两端被**同一条画面边**截断 ⇒ ψ 被系统压小（解析估算差 ~10×）。
  ⇒ 正解是 `line_support()`（见下）：横杆还在就继续用（方向对、量级衰减），
    横杆整个出画就丢线降级。**绝不输出方向可能错的 ψ。**
"""
from __future__ import annotations

import math

try:
    import numpy as np
except Exception:                                    # pragma: no cover
    np = None

try:
    import cv2
except Exception:                                    # pragma: no cover
    cv2 = None

try:
    import quad_geom as _G                            # 分支①产物，优先复用
except Exception:
    _G = None

_EPS = 1e-9

# ---------------------------------------------------------------- 默认参数
DEFAULTS = dict(
    # --- ROI ---
    roi_pad=0.15,          # door bbox 外扩比例（框紧贴红管，不外扩会把杆切掉）
    clip_margin_px=4.0,    # 贴边判据（与现役 AUV_CLIP_MARGIN_PX 同口径）
    # --- 红度 ---
    red_metric='r-g',      # ★ R−G（2026-10-06 在 344 张真实水下帧上实测最优）
                           #   'r-g' | 'r-avg' | 'r-maxgb'。见下方说明。
    red_otsu=False,        # ★ 实测 Otsu 是负作用（会把暗的那根柱整条切掉）
    red_rel=True,          # ★★ 基线相对红度（2026-10-06 板端 live 帧发现，见下）
                           #   True ⇒ 先把 (R−G) 减去"ROI 内水的基线"再阈值化；
                           #   False ⇒ 回到绝对口径（`R−G > red_abs_min`）
    red_rel_pct=25.0,      # 基线 = ROI 内 (R−G) 的这个百分位（水占多数 ⇒ 取低分位）
    red_rel_min=22.0,      # 相对基线抬升多少算"红"（在减去基线后的域里）
    red_abs_min=0.0,       # 绝对口径下限（**仅 red_rel=False 时生效**）：`R−G > 0`
    red_min_frac=0.002,    # ROI 内红像素占比下限，低于 ⇒ 本帧视为"没看到红"
    red_norm=False,        # ★ 实测归一化也是负作用（暗柱被压到阈值以下）
    # --- 形态学 ---
    morph_px=3,            # 开运算核（★ 别调大：细柱 6~10px，5 核会把它抹掉）
    min_blob_px=24,        # 连通域最小面积
    ball_fill=0.72,        # 圆胖度：填充率 > 它 且 长短轴比≈1 ⇒ 判为撞球，剔除
    ball_aspect=(0.75, 1.33),
    # --- 四边带 / 拟合 ---
    band_frac=0.30,        # 带厚 = 门宽(高) 的 30%
    band_mid=(0.30, 0.70),  # 每条边只用中间 40%，避开四角互相串味
    min_line_px=3,         # 拟合一条中心线的最小点数
    line_tol_px=5.0,       # 判"这条线被 mask 支撑"的垂距容差
    min_support=0.50,      # ★ 线段被支撑的最低"沿线覆盖度"，低于 ⇒ 丢这条线（见 line_support）
    min_band_fill=0.05,    # ★★ 边带内 mask **面积占比**下限（见 band_fill）
                           #   ★ 真杆 p05=0.13~0.31，画面边界杂波 p50=0.019 ⇒ 0.05 是干净的分界
    min_band_fill_edge=0.008,  # ★ 同上，但给 `src='edge'`（Canny 边缘只有 1~2px 宽，填充率天然低）
    edge_trim_px=2,        # ★★ 测带内密度前，先剔掉紧贴 mask 边界的这几行/列
                           #   （画幅最外几行有固定编码偏色，会整行全红，把密度门绕过）
    min_band_run=0.06,     # ★ 带内「最长连续实心段」占长边比例的下限（`band_fill` 的备用通道，
                           #   OR 关系）——专捞"被切走后只剩贴边一截"的真横杆
    psi_width_on=False,    # ★ 立柱像宽比解 ψ：**实验开关，默认关**
                           #   实测（344 张真实帧）符号一致率仅 ~0.50 ⇒ 不达可用标准，
                           #   见下方"为什么默认关"。
    psi_width_gain=1.0,    # 像宽比口径的增益（将来标定后用）
    # --- 结构校验 ---
    aspect=1.40,           # 门宽高比 70:50
    aspect_tol=0.35,       # 透视+偏航会让它变小（cosψ），留足
    min_area_px=300.0,
    # --- 输出 ---
    fx=0.0,                # 像素焦距（算 psi_deg 用；0/取不到 ⇒ psi_deg=None）
    src='auto',            # 'red' | 'edge' | 'auto'（红度失败自动切边缘）
    min_lvl=4,             # 只看不吃，给调用方参考
    psi_max=0.55,          # |ψ| 合理性上限（超过 ⇒ 认为是误检，不采纳）
    pole_w_min=1.5,        # 立柱像宽下限（px）；低于 ⇒ 太细，像宽比不可信
)

# ★★ 为什么红度用 `R−G` 而不是 `R−max(G,B)`（2026-10-06 真实帧实测，见下）
#   在 344 张 door1 真实水下帧上：
#     R−max(G,B) : lvl=4 只有 ~34%（换成 Otsu/归一化后更差，~8~12%）
#     R−G        : lvl=4 达 **~76%**
#   原因：水下场景里**蓝通道 B 在亮水区也很高**（画面整体偏青蓝），
#   用 max(G,B) 等于拿"水最亮的那一维"当参照 ⇒ 把同一扇门上较暗的那根立柱压到阈值以下
#   （实测同一门两柱的红度可差 4 倍，00237 帧右柱甚至为负）。
#   `R−G` 只看"比绿更红多少" ⇒ 亮水区（G≈B）与红管（R≫G）被干净分开。
#   ⚠ 代价：**偏黄的白色物体**（R≈G，略 R>G）可能混进来 ⇒ 靠闭合环结构校验兜底。
#
# ★★ 为什么在 `R−G` 之上还要"减去基线"（2026-10-06 板端 live 帧实测）
#   绝对口径 `R−G > 0` 隐含假设"画面里 R 能超过 G"。板端实测**该假设不成立**：
#     door1 真值帧 ：全画幅 (R−G) 中位 −18 ~ −39，红管处最大 +68 ~ +87（R 能超过 G）
#     板端 live 帧 ：全画幅 (R−G) 中位 **−60**，门横杆处最大 **0**（R 全程 < G！）
#   ⇒ 绝对 0 阈值在 live 帧上**一个像素都取不到**（red_frac=0 ⇒ 直接 lvl=0）。
#   但两种帧的共同点是：**门都比周围的水更"红"**（比水的 (R−G) 高出约 +60）。
#   ⇒ 改用**相对基线**：(R−G) − percentile(R−G, red_rel_pct)，再按 red_rel_min 阈值化。
#   基线随水色整体漂移不影响 ⇒ 对白平衡 / 曝光 / 水质都鲁棒。
#   ⚠ 门框 bbox 里水占多数（环内+环外）⇒ 取低分位(25)当"水"是安全的；
#
# ★★ 为什么「立柱像宽比」解 ψ 默认关（2026-10-06 在 168 张真实帧上实测）
#   理论上它很漂亮：立柱像宽 w ≈ f·d/z ⇒ w_l/w_r ≈ z_r/z_l，与"竖边像长比"同源。
#   但实测（只用无纵向残缺、且四角结构校验通过的帧，与 edge 口径逐帧对照）：
#     整数行计数   ：相关系数 +0.69，符号一致率 **0.50**，像宽中位 14/20 px
#     亚像素等效宽 ：相关系数 +0.73，符号一致率 **0.57**
#     二阶矩宽     ：相关系数 +0.65，符号一致率 **0.59**
#   ⇒ 立柱只有 14~20px 宽，±1px 量化就相当于 ψ 的 ±10%，而真实 ψ 量级只有 ~0.1
#     ⇒ **信噪比不足，符号会掷硬币**。按"宁可不出也不出错 ψ"的铁律 ⇒ 默认关。
#   要打开它，得先把像宽估计做到亚像素级 + 现场标定 `psi_width_gain`；否则别喂给控制器。



def _o(opts, key):
    if opts and key in opts:
        return opts[key]
    return DEFAULTS[key]


def _finite(p):
    return p is not None and p[0] == p[0] and p[1] == p[1]


# ================================================================ 颜色
def red_map(roi_bgr, normalize=False, metric='r-g', rel=False, rel_pct=25.0):
    """相对红度图 uint8(0..255)

    `metric`：
      * `'r-g'`    = **R − G**（★ 默认；真实水下帧实测最优，见 DEFAULTS 下方说明）
      * `'r-avg'`  = R − (G+B)/2
      * `'r-maxgb'`= R − max(G,B)（最严；水下亮水区 B 高 ⇒ 会把暗柱切掉）

    `rel=True` 时**先减去 ROI 内 (R−G) 的 `rel_pct` 分位**（≈ 水的基线）再裁到 0..255。
    ⚠ 裁负数之前**必须先减基线**：否则 live 帧（R 全程 < G）整幅被裁成 0，信息全丢。

    物理依据：水对红光吸收最强 ⇒ **任何"还活着"的红物体都比周围的水更红**；
    整体偏色 / 变暗**不影响**它与水的差值 —— 这就是"相对"的用处，比 HSV 的 H 区间稳得多。
    """
    if np is None or roi_bgr is None or roi_bgr.size == 0:
        return None
    x = roi_bgr.astype(np.float32)
    b, g, r = x[..., 0], x[..., 1], x[..., 2]
    if metric == 'r-g':
        v = r - g
    elif metric == 'r-avg':
        v = r - 0.5 * (g + b)
    else:                                             # 'r-maxgb'
        v = r - np.maximum(g, b)
    if rel:                                           # ★ 基线相对化：先减水基线，再裁负
        v = v - float(np.percentile(v, rel_pct))
    if normalize:
        s = r + g + b
        v = np.where(s > _EPS, v * 3.0 / np.maximum(s, _EPS), 0.0) * 255.0 / 3.0
        # ↑ v/s*3 把量纲拉回 ~0..255（纯红 v=s/3 ⇒ 255）
    v = np.clip(v, 0.0, 255.0)
    return v.astype(np.uint8)


def red_mask(roi_bgr, opts=None):
    """红度 → 二值 mask（+ diag）

    ★ 默认 `red_rel=True`：阈值是"相对 ROI 内水基线的抬升量"（`red_rel_min`），
      不再假设 R 能超过 G —— 见 `red_map` / DEFAULTS 上方"为什么还要减去基线"。
    """
    rel = bool(_o(opts, 'red_rel'))
    gray = red_map(roi_bgr, normalize=bool(_o(opts, 'red_norm')),
                   metric=str(_o(opts, 'red_metric')),
                   rel=rel, rel_pct=float(_o(opts, 'red_rel_pct')))
    if gray is None:
        return None, {}
    if cv2 is None:
        return None, {}
    if rel:
        thr = float(_o(opts, 'red_rel_min'))
    else:
        thr = float(_o(opts, 'red_abs_min'))
    if _o(opts, 'red_otsu'):
        t, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        thr = max(thr, float(t))
    _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)
    k = int(_o(opts, 'morph_px'))
    if k >= 3:
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
    frac = float(np.count_nonzero(mask)) / float(mask.size)
    return mask, {'red_thr': thr, 'red_frac': frac}


def edge_mask(roi_bgr, opts=None):
    """红度失败时的兜底：灰度梯度/Canny（**不需要红色**）

    门框与蓝水之间明暗/色彩都有强边缘 ⇒ 边缘仍在。
    """
    if cv2 is None or roi_bgr is None or roi_bgr.size == 0:
        return None
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    e = cv2.Canny(gray, 40, 120)
    k = int(_o(opts, 'morph_px'))
    if k >= 3:
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        e = cv2.morphologyEx(e, cv2.MORPH_CLOSE, ker)
    return e


# ================================================================ mask 清理
def clean_mask(mask, opts=None):
    """去小块 + 去圆胖块（撞球）→ (mask, diag)

    ⚠ 闭合红管环本身是"细环"：填充率 ~0.2 ⇒ **不会被当球剔掉**；
      实心红球填充率 ~0.79 ⇒ 被剔。这个判据同时挡住"引导线断成小段"的碎块（面积门槛）。
    """
    if cv2 is None or mask is None:
        return mask, {}
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    kept, dropped = 0, 0
    min_a = float(_o(opts, 'min_blob_px'))
    fill_th = float(_o(opts, 'ball_fill'))
    a_lo, a_hi = _o(opts, 'ball_aspect')
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a < min_a:
            continue
        fill = float(a) / float(max(1, w * h))
        asp = float(w) / float(max(1, h))
        if fill > fill_th and (a_lo <= asp <= a_hi) and (a_lo <= 1.0 / asp <= a_hi):
            dropped += 1
            continue                                    # 圆胖 ⇒ 撞球
        out[lab == i] = 255
        kept += 1
    return out, {'blobs_kept': kept, 'blobs_dropped': dropped}


def punch_balls(mask, roi, ball_boxes, opts=None):
    """用 `momo_det_front.json` 里的 red-ball 框把红球挖掉（**保险**，不是主手段）

    ★ 用户 2026-10-06 澄清：**球不会在门后面** ⇒ 正常帧里 door 框内本来就没有红球，
      这一步只是防「ROI 外扩 10%」把门旁边的球带进来。
    """
    if cv2 is None or mask is None or not ball_boxes:
        return mask
    x1, y1, _, _ = roi
    for (bx1, by1, bx2, by2) in ball_boxes or ():
        ax1 = int(max(0, min(bx1, bx2) - x1))
        ay1 = int(max(0, min(by1, by2) - y1))
        ax2 = int(max(0, max(bx1, bx2) - x1))
        ay2 = int(max(0, max(by1, by2) - y1))
        cv2.rectangle(mask, (ax1, ay1), (ax2, ay2), 0, -1)
    return mask


# ================================================================ 直线拟合
def _fit_line(pts):
    """pts: Nx2 float32 → (p0, p1) 两个远端点；点太少/退化返回 None"""
    if cv2 is None or pts is None or len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts.reshape(-1, 1, 2), cv2.DIST_L2, 0, 0.01, 0.01)
    vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)
    n = math.hypot(vx, vy)
    if n < _EPS:
        return None
    vx, vy = vx / n, vy / n
    L = 4000.0
    return ((x0 - L * vx, y0 - L * vy), (x0 + L * vx, y0 + L * vy))


def _band_pts(mask, x1, y1, x2, y2):
    """取 mask 的矩形子域内的点（全画幅坐标）→ Nx2"""
    if np is None:
        return None
    h, w = mask.shape[:2]
    x1 = int(max(0, min(x1, x2)))
    x2 = int(min(w, max(x1, x2)))
    y1 = int(max(0, min(y1, y2)))
    y2 = int(min(h, max(y1, y2)))
    if x2 <= x1 or y2 <= y1:
        return None
    sub = mask[y1:y2, x1:x2]
    ys, xs = np.nonzero(sub)
    if len(xs) == 0:
        return None
    return np.stack([xs + x1, ys + y1], axis=1).astype(np.float32)


def band_rect(bbox, key, opts=None):
    """某条边的"带"矩形（全画幅/ROI 局部坐标一致即可）"""
    x1, y1, x2, y2 = [float(v) for v in bbox]
    W = max(1.0, x2 - x1)
    H = max(1.0, y2 - y1)
    bw = max(3.0, float(_o(opts, 'band_frac')) * W)
    bh = max(3.0, float(_o(opts, 'band_frac')) * H)
    m0, m1 = _o(opts, 'band_mid')
    ylo, yhi = y1 + m0 * H, y1 + m1 * H
    xlo, xhi = x1 + m0 * W, x1 + m1 * W
    return {'left': (x1, ylo, x1 + bw, yhi),
            'right': (x2 - bw, ylo, x2, yhi),
            'top': (xlo, y1, xhi, y1 + bh),
            'bottom': (xlo, y2 - bh, xhi, y2)}[key]


def fit_four_lines(mask, bbox, opts=None):
    """在 bbox **四条边带**里各拟合一条中心线 → {'left','right','top','bottom'}

    只用每条边的**中间 40%**（`band_mid`）：四角处竖杆与横杆交织，全用会互相串味。
    ⚠ 拟合出来**不一定是真线**（见 `line_support`）——调用方必须过一遍支撑度校验。
    """
    if mask is None:
        return {}
    minp = int(_o(opts, 'min_line_px'))
    out = {}
    for k in ('left', 'right', 'top', 'bottom'):
        pts = _band_pts(mask, *band_rect(bbox, k, opts))
        if pts is None or len(pts) < minp:
            continue
        ln = _fit_line(pts)
        if ln:
            out[k] = ln
    return out


def line_support(mask, bbox, key, line, opts=None):
    """★ 该边带上"这条线被 mask 支撑的比例" ∈ [0,1]

    做法：取带内所有 mask 点，算到线的垂距；保留 ±`line_tol_px` 的内点，
    再沿**线的方向**分 20 箱，看有多少箱里有内点。

    为什么要它（这是本分支最实用的一个补丁）：
      * **纵向残缺**时，被切掉那侧的带里只剩两根立柱的截面 ⇒ 拟合出的"横杆线"
        穿过两个小团 ⇒ 沿线覆盖度只有 2~4/20 ≈ 0.1~0.2 ⇒ 一眼识破。
        而这条假线会让 ψ 被**同一条画面边截断**而有偏（实测量级差 ~10×）。
      * 反过来，真横杆扫过整条带 ⇒ 覆盖度 ≈ 1.0。
      * 因"红度漏检"而断裂的真线也能被认出来（断口不占满箱数但覆盖仍高）。
    """
    if mask is None or line is None or np is None:
        return 0.0
    (ax, ay), (bx, by) = line
    dx, dy = float(bx - ax), float(by - ay)
    n = math.hypot(dx, dy)
    if n < _EPS:
        return 0.0
    ux, uy = dx / n, dy / n
    pts = _band_pts(mask, *band_rect(bbox, key, opts))
    if pts is None or len(pts) < 3:
        return 0.0
    rel = pts.astype(np.float32) - np.array([ax, ay], dtype=np.float32)
    t = rel[:, 0] * ux + rel[:, 1] * uy
    d = np.abs(rel[:, 0] * (-uy) + rel[:, 1] * ux)
    inl = d <= float(_o(opts, 'line_tol_px'))
    if int(inl.sum()) < 3:
        return 0.0
    tmin, tmax = float(t.min()), float(t.max())
    if tmax - tmin < 1.0:
        return 0.0
    nb = 20
    idx = ((t[inl] - tmin) / (tmax - tmin) * nb).astype(int)
    idx = np.clip(idx, 0, nb - 1)
    return float(len(set(idx.tolist()))) / float(nb)


def band_fill(mask, bbox, key, opts=None):
    """★ 该边带内mask 的**面积占比** ∈ [0,1]（不看线，只看带里到底有多少红）

    ★★ 这是 `line_support` 之外的**第二把刀**，专门砍两类假线：
      ①「两根立柱截面连成的假横线」—— 密度极低
      ②「紧贴画幅**边界几行**的边行」—— JPEG 编码/传感器最后一行的固定偏色，
         会让整行全红（实测 001720 帧：图像最后 3 行196/196 全红），
         把 bottom 带"填"满 ⇒ 密度判据被绕过。**这类行必须先剔掉。**

    板端 1492 真帧实测（`src='red'`）：
        真横/竖杆 fill：p05 = 0.31 / 0.13
        底边杂波fill：p50 = 0.019、p75 = 0.06
    """
    if mask is None or np is None:
        return 0.0
    x1, y1, x2, y2 = [int(round(v)) for v in band_rect(bbox, key, opts)]
    h, w = mask.shape[:2]
    xa, xb = max(0, min(x1, x2)), min(w, max(x1, x2))
    ya, yb = max(0, min(y1, y2)), min(h, max(y1, y2))
    if xb <= xa or yb <= ya:
        return 0.0
    sub = mask[ya:yb, xa:xb]
    if sub.size == 0:
        return 0.0
    #★ 剔掉紧贴 mask 边界的 `edge_trim` 行/列（画幅最外几行的编码偏色，不是门）
    et = int(_o(opts, 'edge_trim_px'))
    if et > 0 and sub.shape[0] > 2 * et + 2:
        sub = sub[et:-et, :]
    if sub.size == 0:
        return 0.0
    return float(np.count_nonzero(sub)) / float(sub.size)


def band_run(mask, bbox, key, opts=None):
    """★ 该边带内，沿**长边方向**的**最长连续实心段**占长边的比例 ∈ [0,1]

    这是 `band_fill` 的**备用通道**（两者取或）：
      *门被切走后只剩**贴边的一小截**真横杆时，带内`fill` 只有 0.003~0.01（过不了密度门），
        但那截是**连续**的 ⇒ 本指标 ≈ 0.08~0.4，能把它捞回来（合成图 t14：只露 13px/164）。
      * 纯杂波/边行是**断续散点** ⇒ 最长连续段很短，本指标也上不去。
    ⚠ 同样先剔掉紧贴画幅边界的 `edge_trim_px` 行/列（那些是编码偏色，不是门）。
    """
    if mask is None or np is None:
        return 0.0
    x1, y1, x2, y2 = [int(round(v)) for v in band_rect(bbox, key, opts)]
    h, w = mask.shape[:2]
    xa, xb = max(0, min(x1, x2)), min(w, max(x1, x2))
    ya, yb = max(0, min(y1, y2)), min(h, max(y1, y2))
    if xb <= xa or yb <= ya:
        return 0.0
    sub = mask[ya:yb, xa:xb]
    if sub.size == 0:
        return 0.0
    et = int(_o(opts, 'edge_trim_px'))
    horiz = (xb - xa) >= (yb - ya)
    if et > 0:
        if horiz and sub.shape[1] > 2 * et + 2:
            sub = sub[:, et:-et]
        elif (not horiz) and sub.shape[0] > 2 * et + 2:
            sub = sub[et:-et, :]
    if sub.size == 0:
        return 0.0
    prof = np.count_nonzero(sub, axis=0) if horiz else np.count_nonzero(sub, axis=1)
    n = int(prof.size)
    if n == 0:
        return 0.0
    b = (prof > 0).astype(np.int8)
    best = 0
    cur = 0
    for v in b.tolist():
        cur = cur + 1 if v else 0
        if cur > best:
            best = cur
    return float(best) / float(n)


def filter_lines_by_support(mask, bbox, lines, opts=None, src='red'):
    """丢掉**支撑度 < `min_support`** 或**带内密度 < 阈值**的线
        → (lines, 每条的支撑度, 每条的带内密度)

    ★ 两道判据**缺一不可**（2026-10-06 板端真帧教训）：
      - 只看支撑度 ⇒ 贴着画面边界的**杂波**能伪装成"线"（001720 帧，支撑度 1.0）
      - 只看密度   ⇒ 因红度断裂而"真线但稀疏"的情况会被误杀
    ★ 密度阈值**按 mask 来源分档**：`src='edge'` 的 mask 是 Canny 边缘，只有 1~2px 宽，
      填充率天然比"红色填充带"低一个量级 ⇒ 用 `min_band_fill_edge`（更松）。
    """
    keep, sup, fil, run = {}, {}, {}, {}
    thr = float(_o(opts, 'min_support'))
    fthr = float(_o(opts, 'min_band_fill_edge' if src == 'edge' else 'min_band_fill'))
    rthr = float(_o(opts, 'min_band_run'))
    for k, ln in (lines or {}).items():
        s = line_support(mask, bbox, k, ln, opts)
        f = band_fill(mask, bbox, k, opts)
        ru = band_run(mask, bbox, k, opts)
        sup[k] = round(s, 3)
        fil[k] = round(f, 4)
        run[k] = round(ru, 4)
        # ★★ 密度 **或** 连续段达标就留：
        #   「整条被切走后只剩贴边残段」的真横杆（合成图 t14：只露 13px）密度只有 0.003，
        #   但那13px 是**连续实心**的 ⇒ 用 `band_run` 这条备用通道把它捞回来；
        #   而纯杂波是断续散点，两条都过不了。
        if s >= thr and (f >= fthr or ru >= rthr):
            keep[k] = ln
    return keep, sup, fil, run


# ================================================================ 纵向残缺时的 ψ
def band_pole_width(mask, x1, y1, x2, y2):
    """带内「立柱」的横向像宽（px，取整行前景点数的**中位数**）

    ⚠ 立柱在带内基本是竖条 ⇒ 每一行的前景点数 ≈ 该行的横向厚度。
      用中位数而不是均值：抗单行噪点 / 反光 / 局部断裂。
      **只看横向** ⇒ 上下被画面切掉多少都无所谓（这就是它的价值）。
    """
    if np is None or mask is None:
        return None
    h, w = mask.shape[:2]
    xa = int(max(0, min(x1, x2)))
    xb = int(min(w, max(x1, x2)))
    ya = int(max(0, min(y1, y2)))
    yb = int(min(h, max(y1, y2)))
    if xb <= xa or yb <= ya:
        return None
    sub = mask[ya:yb, xa:xb]
    rows = np.count_nonzero(sub, axis=1)
    rows = rows[rows > 0]
    if rows.size == 0:
        return None
    return float(np.median(rows))


def psi_from_bands(mask, roi_box, opts=None):
    """★ 纵向残缺时的 ψ：**左右立柱像宽比**

    物理：立柱像宽 w ≈ f·d/z（d = 管径）⇒ `w_l / w_r ≈ z_r / z_l`，
    与「竖边像长比 L_l/L_r ≈ z_r/z_l」**同源** ⇒ 得到的是**同一量纲、同一符号**的 ψ，
    下游 `psi_proxy` 的伺服增益/死区**直接沿用**，不需要另开一套参数。

    ⚠⚠ **但真实帧上它的信噪比不足**（2026-10-06，168 张对照）：
      整数行计数 / 亚像素等效宽 / 二阶矩宽 三种估计器的**符号一致率分别只有 0.50 / 0.57 / 0.59**，
      因为立柱只有 14~20px 宽，±1px 量化就相当于 ψ 的 ±10%，而真实 ψ 量级只有 ~0.1。
      ⇒ 只在 `psi_width_on=True`（**默认关**）时才走这条路，且输出带 `psi_low_conf=True`。
      要用它，必须先把像宽估计做到亚像素级并现场标定 `psi_width_gain`。

    返回 psi（有符号，口径与 `quad_geom.psi_proxy` 一致）或 None。
    """
    if mask is None:
        return None
    x1, y1, x2, y2 = roi_box
    W = max(1.0, float(x2 - x1))
    H = max(1.0, float(y2 - y1))
    bw = max(3.0, float(_o(opts, 'band_frac')) * W)
    m0, m1 = _o(opts, 'band_mid')
    ylo, yhi = y1 + m0 * H, y1 + m1 * H
    wl = band_pole_width(mask, x1, ylo, x1 + bw, yhi)
    wr = band_pole_width(mask, x2 - bw, ylo, x2, yhi)
    wmin = float(_o(opts, 'pole_w_min'))
    if not wl or not wr or wl < wmin or wr < wmin:
        return None
    rho = float(wl) / float(wr)
    return (rho - 1.0) / (rho + 1.0)


def _round_quad(quad):
    return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in quad]


def corners_from_lines(lines):
    """4 条中心线两两求交 → **[TL, TR, BR, BL]（已规范化顺序）**

    顺序口径与 `quad_geom.normalize_quad` 一致（TL = 图像左上，x 右 y 下）
    ⇒ 下游 `order_ok` / `psi_proxy` 直接可用。
    """
    if _G is not None:
        inter = _G.intersect_lines
    else:
        inter = _intersect_lines_local
    need = ('left', 'right', 'top', 'bottom')
    if any(k not in lines for k in need):
        return None
    def X(a, b):
        return inter(lines[a][0], lines[a][1], lines[b][0], lines[b][1])
    tl = X('left', 'top')
    tr = X('top', 'right')
    br = X('right', 'bottom')
    bl = X('bottom', 'left')
    if not all(_finite(p) for p in (tl, tr, br, bl)):
        return None
    return normalize_quad([tl, tr, br, bl])


def normalize_quad(pts):
    """排成 TL,TR,BR,BL（优先 `quad_geom.normalize_quad`，拿不到用本地等价实现）"""
    q = [(float(p[0]), float(p[1])) for p in (pts or [])]
    if len(q) != 4:
        return q
    if _G is not None:
        n = _G.normalize_quad(q)
        if n:
            return [(float(p[0]), float(p[1])) for p in n]
    cx = sum(p[0] for p in q) / 4.0
    cy = sum(p[1] for p in q) / 4.0
    s = sorted(q, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    k = min(range(4), key=lambda i: (s[i][0] + s[i][1]))
    return s[k:] + s[:k]


def _intersect_lines_local(p1, p2, p3, p4):
    (x1, y1), (x2, y2) = p1, p2
    (x3, y3), (x4, y4) = p3, p4
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < _EPS:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    return ((a * (x3 - x4) - (x1 - x2) * b) / d,
            (a * (y3 - y4) - (y1 - y2) * b) / d)


# ================================================================ 几何量
def _dist(a, b):
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def _center_local(quad):
    tl, tr, br, bl = quad
    c = _intersect_lines_local(tl, br, tr, bl)
    if c is None or not _finite(c):
        return (sum(p[0] for p in quad) / 4.0, sum(p[1] for p in quad) / 4.0)
    return c


def _psi_local(quad):
    tl, tr, br, bl = quad
    left = _dist(tl, bl)
    right = _dist(tr, br)
    if left < _EPS or right < _EPS:
        return None
    rho = left / right
    return (rho - 1.0) / (rho + 1.0)


def _area_local(quad):
    s = 0.0
    n = len(quad)
    for i in range(n):
        x1, y1 = quad[i]
        x2, y2 = quad[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def _seg_len_in_box(line, box):
    """线段落在 box 内的**可见长度**（box=(x1,y1,x2,y2)，Liang-Barsky 裁剪）

    线段 P(t) = P0 + t·D，t ∈ [0,1]。每条 box 边给一个约束 `p·t ≤ q`：
      * `p > 0` ⇒ t ≤ q/p  ⇒ **离开**边界，对 t 的**上界**取 min
      * `p < 0` ⇒ t ≥ q/p  ⇒ **进入**边界，对 t 的**下界**取 max
    ⚠ 两个方向都要和 [0,1] 求交；`p≈0`（平行）时若 `q<0`（整段在外侧）直接不可见。
    """
    (px0, py0), (px1, py1) = line
    bx1, by1, bx2, by2 = [float(v) for v in box]
    bx1, bx2 = min(bx1, bx2), max(bx1, bx2)
    by1, by2 = min(by1, by2), max(by1, by2)
    dx, dy = float(px1 - px0), float(py1 - py0)
    seg = math.hypot(dx, dy)
    if seg < _EPS:
        return 0.0
    t0, t1 = 0.0, 1.0
    for (p, q) in ((-dx, px0 - bx1), (dx, bx2 - px0),
                   (-dy, py0 - by1), (dy, by2 - py0)):
        if abs(p) < _EPS:
            if q < 0.0:
                return 0.0                    # 平行且整段在外侧
            continue
        t = q / float(p)
        if p > 0.0:
            if t < t1:
                t1 = t                        # 离开边界
        else:
            if t > t0:
                t0 = t                        # 进入边界
    if t1 <= t0:
        return 0.0
    return seg * (t1 - t0)


def psi_from_poles_in_box(lines, box, opts=None):
    """★ 同 `psi_from_poles`，但用 ROI 框裁剪可见段 → (psi, (l_left, l_right))"""
    if 'left' not in lines or 'right' not in lines:
        return (None, None)
    ll = _seg_len_in_box(lines['left'], box)
    rr = _seg_len_in_box(lines['right'], box)
    if ll < _EPS or rr < _EPS:
        return (None, None)
    rho = ll / rr
    return ((rho - 1.0) / (rho + 1.0), (ll, rr))


def quad_fields(corners, fx=0.0, bbox_cx=None):
    """4 角 → 下游字段（psi / 中心 / 尺寸 / 面积 / 竖杆像长）"""
    G = _G
    center = G.quad_center(corners) if G else _center_local(corners)
    psi = G.psi_proxy(corners) if G else _psi_local(corners)
    if G:
        w, h = G.quad_size(corners)
        area = G.quad_area(corners)
        top, right, bottom, left = G.edge_lens(corners)
    else:
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        area = _area_local(corners)
        left, right, top, bottom = (_dist(corners[0], corners[3]), _dist(corners[1], corners[2]),
                                    _dist(corners[0], corners[1]), _dist(corners[2], corners[3]))
    d = {'cx': center[0], 'cy': center[1], 'w': w, 'h': h, 'area': area,
         'psi': psi, 'lens': [left, right, top, bottom]}
    if psi is not None and fx and w > _EPS:
        d['psi_deg'] = math.degrees(math.atan(2.0 * float(fx) * psi / w))
    else:
        d['psi_deg'] = None
    if bbox_cx is not None:
        d['psi_alt'] = (G.psi_from_centers(corners, bbox_cx) if G else None)
    return d


# ================================================================ 结构校验
def struct_check(corners, opts=None, clip_v=False):
    """→ (ok, why)：宽高比 ≈1.4、凸、四点顺序对、面积够

    ★ `clip_v=True`（上/下横杆出画）⇒ **跳过宽高比校验**：门在像面被竖直截断，
      观测到的 w/h 必然偏大（高少了一截），拿 1.40 去卡它等于把"残缺"误判成"误检"。
      这正是用户口径「上下残缺无所谓，定深过门只需偏航对准」的落点。
      其余校验（凸 / 退化 / 面积）照旧 —— 它们与截断无关，仍在挡误检。
    """
    if not corners or len(corners) != 4:
        return (False, '角点不全')
    bad = []
    if _G is not None:
        cv_, why = _G.quad_valid(corners, min_area_px=float(_o(opts, 'min_area_px')))
        if not cv_:
            bad.append('退化(%s)' % why)
        if not _G.is_convex(corners):
            bad.append('非凸')
    w_img = 0.5 * (_dist(corners[0], corners[1]) + _dist(corners[3], corners[2]))
    h_img = 0.5 * (_dist(corners[0], corners[3]) + _dist(corners[1], corners[2]))
    if h_img < _EPS:
        bad.append('高为0')
    elif not clip_v:                                  # ★ 纵向残缺时不卡宽高比（见上）
        asp = w_img / h_img
        tgt = float(_o(opts, 'aspect'))
        tol = float(_o(opts, 'aspect_tol'))
        if abs(asp - tgt) > tol:
            bad.append('宽高比%.2f(应≈%.2f±%.2f)' % (asp, tgt, tol))
    return ((not bad), '；'.join(bad))


def clip_state(bbox, img_w, img_h, margin=0.0):
    """门框贴画面边的状态 → (clip_h, clip_v, clip_t, clip_b)

    ⚠ **横向贴边 = 门被切了一半** ⇒ 左右杆像长比不可信 ⇒ 不能算 ψ（§3.5 第 2 条）。
      补救办法：按 lvl=1「朝可见部分转」，把门拉回画面，转全了再升级 ψ 伺服。
    ⚠ **纵向贴边**（上/下横杆出画）⇒ 定深过门只需要偏航 ⇒ **不降级**，
      但被切那侧的"横杆线"是假线，必须丢（见 `detect` ③）。
    """
    x1, y1, x2, y2 = bbox
    m = float(margin)
    clip_l = (x1 <= m)
    clip_r = (x2 >= img_w - m)
    clip_t = (y1 <= m)
    clip_b = (y2 >= img_h - m)
    return (bool(clip_l or clip_r), bool(clip_t or clip_b), bool(clip_t), bool(clip_b))


# ================================================================ 主入口
def detect(frame_bgr, bbox, opts=None, ball_boxes=(), img_wh=None):
    """门 bbox 内找 4 根红杆 → 四边形

    参数
      frame_bgr : HxWx3 uint8（BGR，全画幅，**干净帧**，见可行性分析 §4.1）
      bbox      : (x1, y1, x2, y2) —— YOLO 的 door det 框（全画幅坐标）
      opts      : 阈值字典（缺项走 DEFAULTS）
      ball_boxes: [(x1,y1,x2,y2), ...] 红球框（保险用，正常帧内没有）
      img_wh    : (W, H) 全画幅尺寸；不给就从 frame 推

    返回 dict：
      ok, lvl, src, corners(4×[x,y] | None), fields(...), diag(...)
    """
    out = {'ok': False, 'lvl': 0, 'src': None, 'corners': None,
           'fields': {}, 'diag': {}, 'why': ''}
    if np is None or frame_bgr is None or bbox is None:
        out['why'] = 'no-input'
        return out
    H, W = (frame_bgr.shape[0], frame_bgr.shape[1])
    if img_wh:
        W, H = int(img_wh[0]), int(img_wh[1])

    # ---- ROI（外扩，防框紧贴红管把杆切掉）----
    x1, y1, x2, y2 = [float(v) for v in bbox]
    pad = float(_o(opts, 'roi_pad'))
    dx = pad * max(1.0, x2 - x1)
    dy = pad * max(1.0, y2 - y1)
    rx1 = int(max(0, math.floor(x1 - dx)))
    ry1 = int(max(0, math.floor(y1 - dy)))
    rx2 = int(min(W, math.ceil(x2 + dx)))
    ry2 = int(min(H, math.ceil(y2 + dy)))
    if rx2 - rx1 < 8 or ry2 - ry1 < 8:
        out['why'] = 'roi-too-small'
        return out
    roi_bgr = frame_bgr[ry1:ry2, rx1:rx2]
    out['diag']['roi'] = [rx1, ry1, rx2, ry2]

    c_h, c_v, c_t, c_b = clip_state((x1, y1, x2, y2), W, H, _o(opts, 'clip_margin_px'))
    out['diag']['clip_h'] = c_h
    out['diag']['clip_v'] = c_v
    out['diag']['clip_t'] = c_t
    out['diag']['clip_b'] = c_b

    # ---- 红度（失败 ⇒ 按 src 决定切边缘 / 直接退出）----
    src_req = _o(opts, 'src')
    mask, mdiag = red_mask(roi_bgr, opts)
    out['diag'].update(mdiag)
    used = 'red'
    if mask is None or mdiag.get('red_frac', 0.0) < float(_o(opts, 'red_min_frac')):
        if src_req in ('edge', 'auto'):
            em = edge_mask(roi_bgr, opts)
            if em is not None and np.count_nonzero(em) > 0:
                mask, used = em, 'edge'
        if used == 'red' and (mask is None or mdiag.get('red_frac', 0.0) < float(_o(opts, 'red_min_frac'))):
            out['why'] = 'red-too-little'
            # 红度不够但门框中心仍有 ⇒ 只有横向位置（横向还残缺就退到 lvl=1）
            out['lvl'] = 1 if c_h else 2
            out['src'] = 'red'
            f = {'cx': 0.5 * (x1 + x2), 'cy': 0.5 * (y1 + y2),
                 'w': abs(x2 - x1), 'h': abs(y2 - y1), 'psi': None}
            out['fields'] = f
            out['ok'] = True
            return out
    out['src'] = used
    if used == 'red':
        mask = punch_balls(mask, (rx1, ry1, rx2, ry2), ball_boxes, opts)
    mask, cdiag = clean_mask(mask, opts)
    out['diag'].update(cdiag)

    # ---- 四边带拟合（★ mask 是 ROI 局部坐标，故带也用局部坐标）----
    roi_box = (0, 0, rx2 - rx1, ry2 - ry1)
    raw_lines = fit_four_lines(mask, roi_box, opts)
    # ★★ 证据驱动：**不管 clip 与否**，一条线要同时过两道门才留
    #   ① `line_support` 沿线覆盖度 —— 砍"两根立柱截面连成的假横线"
    #   ② `band_fill`    带内密度   —— 砍"贴着画面边界的断续杂波"（★ 2026-10-06 板端真帧发现：
    #      门底边出画时 bottom 带被夹到图像下沿，里面只剩几段小白点，
    #      而这些杂波恰好散在"贴画面底边的那条线"上 ⇒ 支撑度也能算出 1.0，单靠①会被骗）
    lines, sup, fil, run = filter_lines_by_support(mask, roi_box, raw_lines, opts, src=used)
    out['diag']['lines_raw'] = sorted(raw_lines.keys())
    out['diag']['lines'] = sorted(lines.keys())
    out['diag']['line_support'] = sup
    out['diag']['band_fill'] = fil
    out['diag']['band_run'] = run
    corners = corners_from_lines(lines)
    if corners is not None:                              # 局部 → 全画幅
        corners = [(float(p[0]) + rx1, float(p[1]) + ry1) for p in corners]

    struct_ok, struct_why = False, ''
    if corners is not None:
        struct_ok, struct_why = struct_check(corners, opts, clip_v=bool(c_v))
        out['diag']['struct'] = struct_why

    bbox_cx = 0.5 * (x1 + x2)
    bbox_cy = 0.5 * (y1 + y2)
    fx = float(_o(opts, 'fx') or 0.0)
    center_fields = {'cx': bbox_cx, 'cy': bbox_cy, 'w': abs(x2 - x1),
                     'h': abs(y2 - y1), 'psi': None}

    # ---- ① 横向残缺（**最高优先级**）：门被切了一半 ⇒ 只允许"朝可见部分定向转" ----
    #     ★ 绝不拿"半个门"的左右杆像长比算 ψ（§3.5 第 2 条）
    if c_h:
        out['lvl'] = 1
        out['ok'] = True
        out['why'] = 'clipped-h ⇒ lvl=1（定向转把门拉回画面）'
        if corners is not None:
            out['corners'] = _round_quad(corners)
        out['fields'] = dict(center_fields)
        return out

    # ---- ② 四角齐全 + 结构校验通过 ⇒ 完整能力（ψ = 竖边像长比）----
    if corners is not None and struct_ok:
        out['lvl'] = 4
        out['psi_src'] = 'edge'
        out['corners'] = _round_quad(corners)
        f = quad_fields(corners, fx=fx, bbox_cx=bbox_cx)
        f['psi_src'] = 'edge'
        out['fields'] = f
        out['ok'] = True
        if c_v:                                       # ★ 纵向残缺 ⇒ ψ 标低置信，但仍出
            out['psi_low_conf'] = True
            out['why'] = 'quad ok（纵向残缺：不卡宽高比）；ψ 由立柱端点给出'
        else:
            out['why'] = 'quad ok（竖边像长比）'
        return out

    # ---- ③ 纵向残缺（上下横杆出画）：角点凑不齐，且**此时图像几何无法稳健恢复 ψ**。
    #      ★ 这里踩过两条死路（2026-10-06 合成图 + 板端真帧都证伪，**别再走回去**）：
    #        ①「两根立柱**可见段像长比**」：同一条画面边界同时截两根柱 ⇒ **恒等长**
    #           （实测 yaw=±20°/±15°/±6°、切 30%/40%，pole_lens 全部 [257,257]/[215,215]）
    #           ⇒ ψ 恒为 0，**完全无信息**。
    #        ②「上/下横杆**图像倾角**」：门出画时横杆也被切掉一截 ⇒ **符号会翻转**
    #           （实测 yaw=+20°、切 30% ⇒ tilt 由 +4.36° 变成 −0.89°）。
    #      ⇒ 结论：只靠 1~2 条边，**没有任何无偏的 ψ 口径**。宁可不给，也不给反号。
    #        `psi_from_poles_in_box` 保留供诊断（看两根柱到底等不等长），不参与判定。
    pw, lens = (None, None)
    if 'left' in lines and 'right' in lines:
        pw, lens = psi_from_poles_in_box(lines, roi_box, opts)
    if lens is not None:
        out['diag']['pole_lens'] = [round(lens[0], 2), round(lens[1], 2)]
        out['diag']['pole_ratio'] = round(lens[0] / lens[1], 4) if lens[1] > _EPS else None

    # ---- ④ 纵向残缺的**实验**备选：「左右立柱**像宽比**」解 ψ（像宽 ∝ 1/z）。
    #         ⚠ 但真实帧实测**符号一致率只有 ~0.50**（像宽仅 14~20px，±1px 就翻转）
    #           ⇒ `psi_width_on` **默认关**；打开也只当实验，必须自己确认符号对了。 ----
    if c_v and bool(_o(opts, 'psi_width_on')):
        pw = psi_from_bands(mask, roi_box, opts)
        gain = float(_o(opts, 'psi_width_gain') or 1.0)
        if pw is not None:
            pw = pw * gain
        if pw is not None and abs(pw) <= float(_o(opts, 'psi_max')):
            out['lvl'] = 4
            out['psi_src'] = 'width'
            out['corners'] = None                        # 角点不可信 ⇒ 不给
            f = dict(center_fields)
            f.update({'area': None, 'psi': pw, 'psi_src': 'width',
                      'psi_low_conf': True})
            if fx and abs(x2 - x1) > _EPS:
                f['psi_deg'] = math.degrees(math.atan(2.0 * fx * pw / abs(x2 - x1)))
            out['fields'] = f
            out['ok'] = True
            out['why'] = 'clipped-v ⇒ 立柱像宽比解 ψ（实验开关，低置信）'
            out['diag']['psi_width'] = round(float(pw), 4)
            return out

    # ---- ⑤ 有角但结构不过 ⇒ 只给对中（lvl=3）----
    if corners is not None:
        out['lvl'] = 3
        out['ok'] = True
        out['why'] = struct_why or '结构不过'
        out['corners'] = _round_quad(corners)
        out['fields'] = quad_fields(corners, fx=fx, bbox_cx=bbox_cx)
        return out

    # ---- ⑥ 线凑不齐 ----
    #   ★ `lines` 里剩下的都是**过了支撑度 + 带内密度两道门**的真边 ⇒ 只要还剩≥1 条，
    #     就是"看到了门的一部分"，至少能给出横向位置（lvl=2）。
    #     **别再报 lvl=0（无观测）**——2026-10-06 板端live 帧就是这种情形：
    #     门横杆支撑度 1.0 明明白白，却因为立柱太淡只剩 1 条线而被判成"没看到"。
    if len(lines) >= 3:
        out['lvl'] = 3
        out['ok'] = True
        out['why'] = 'only %d lines' % len(lines)
        out['fields'] = dict(center_fields)
        return out
    if len(lines) >= 1:
        out['lvl'] = 2
        out['ok'] = True
        out['why'] = 'only %d line(s) ⇒ 只给横向' % len(lines)
        out['fields'] = dict(center_fields)
        return out
    out['lvl'] = 0
    out['ok'] = False
    out['why'] = 'no-lines'
    out['fields'] = dict(center_fields)
    return out


# ---------------------------------------------------------------- 自检
if __name__ == '__main__':                           # pragma: no cover
    import sys
    print('quad_cv_det: cv2=%s numpy=%s quad_geom=%s'
          % (getattr(cv2, '__version__', None), getattr(np, '__version__', None),
             getattr(_G, '__name__', None)))
    sys.exit(0 if (cv2 is not None and np is not None) else 1)
