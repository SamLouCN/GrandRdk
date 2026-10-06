# -*- coding: utf-8 -*-
"""穿门四边形几何 —— 纯函数集（无 I/O / 无配置 / 不 import 任何板端模块）

[gate_kpt 2026-10-05 新建 | 2026-10-06 R2 重写]

★★ 全模块唯一口径（改之前读三遍）★★
  1. 顶点顺序恒为 **TL, TR, BR, BL**（图像坐标：x 向右、y 向下）。
  2. 门中心 = **两条对角线的交点**，不是四角坐标平均 ——
     射影变换保持「相交」不保持「平均」，偏航时两者会分开。
  3. psi = (rho-1)/(rho+1)，rho = 左竖边像长 / 右竖边像长。
     **>0 表示机头偏右**（右偏 ⇒ 门左柱更近 ⇒ 左竖边在像面上更长）⇒ 应**左转**修正。
     闭式：psi = (W/(2z))·sin(psi_r)。★ 有符号 —— 恰是整门 bbox 给不出的那半个信息。
  4. ⚠ tilt（上横边倾角）符号依赖「门在光轴的上下方」⇒ **只做诊断，绝不当控制量**。
  5. psi / par_v / orth_err 都「正对时为 0」，但量纲完全不同 ⇒ **判据只用 psi**。

★★ R2 新增：模型检的是「门框转角处的连接件」（实体小框，多条 det）★★
  由此带来两个新问题，本模块新增两组函数解决：

  A) **哪一个候选属于哪一扇门**（画面里可能有多扇门，前后错开、带旋转）
     → 不能按图像距离聚类（前后门在画面上会重叠、必然串门），
       改用 `group_by_size()`：**连接件是同一规格零件 ⇒ 像面尺寸 ∝ 1/z**
       ⇒ 同一扇门的 4 件尺寸相近、不同门差得开。
       相邻两门怎么定"最前面那扇"见 `pick_group()`（用同一把尺子排序）。

  B) **这一帧能算多少**（模型可能只检到 1~3 个连接件）
     → `classify()` 四象限归类 + `quad_lvl()` 可用等级，**信息用尽、绝不丢帧**：
         lvl=4 四角齐全 ⇒ 对角交点中心 + psi 正对 + 面积
         lvl=3 缺一角   ⇒ 补第 4 角（平行四边形）后取交点，**没有 psi**
         lvl=2 两个角   ⇒ 两点中点，只有横向
         lvl=1 一个角   ⇒ 只能"朝它转"
         lvl=0          ⇒ 本帧无观测

纯函数的意义：这一段可以在 Windows 上直接跑断言（见 ../tests/test_quad.py），
不需要板端、不需要相机、不需要模型。
"""
import math  # 三角函数与平方根

ORDER = ('tl', 'tr', 'br', 'bl')   # 顶点顺序：左上 / 右上 / 右下 / 左下
_EPS = 1e-9                        # 浮点判零阈值


# ================================================================ 点 / 解析
def as_point(p):
    """宽容解析一个点 → (x, y)；解析不出来返回 None

    模型导出格式不统一，这里一并认：`[x,y]` / `[x,y,conf]` / `(x,y)` /
    `{'x':..,'y':..}` / `{'u':..,'v':..}` / `{'cx':..,'cy':..}`。
    """
    if p is None:
        return None
    if isinstance(p, dict):
        for a, b in (('x', 'y'), ('u', 'v'), ('cx', 'cy'), ('px', 'py')):
            if a in p and b in p:
                try:
                    return (float(p[a]), float(p[b]))
                except (TypeError, ValueError):
                    return None
        return None
    try:
        return (float(p[0]), float(p[1]))
    except (TypeError, ValueError, IndexError, KeyError):
        return None


def kpt_conf(p):
    """取一个角点的置信度；没有该字段返回 None（= 不参与阈值过滤）"""
    if isinstance(p, dict):
        for k in ('conf', 'score', 's', 'vis', 'visibility'):
            if k in p:
                try:
                    return float(p[k])
                except (TypeError, ValueError):
                    return None
        return None
    try:
        if len(p) >= 3:
            return float(p[2])
    except (TypeError, ValueError, IndexError):
        return None
    return None


def parse_quad(raw, min_conf=0.0):
    """把模型输出的角点原样解析成 4 个点（**不排序**）

    返回 (pts, n_ok)：
      * pts 长度恒为 4，元素是 (x,y) 或 None（= 该角点缺失/置信度不够）
      * n_ok = 有效角点数（0~4）

    认得的形状：
      `[[x,y],[x,y],[x,y],[x,y]]` / `[[x,y,c] x4]` / `[{'x':..,'y':..,'conf':..} x4]`
      `{'tl':..,'tr':..,'br':..,'bl':..}`
      `[x1,y1,x2,y2,x3,y3,x4,y4]`（展平的 8 个数）
    """
    items = None
    if isinstance(raw, dict):
        if all(k in raw for k in ORDER):
            items = [raw[k] for k in ORDER]
        else:
            return ([None] * 4, 0)
    elif isinstance(raw, (list, tuple)):
        vals = list(raw)
        if len(vals) == 8 and all(isinstance(v, (int, float)) for v in vals):
            items = [[vals[2 * i], vals[2 * i + 1]] for i in range(4)]   # 展平 8 数
        elif len(vals) >= 4:
            items = vals[:4]
        else:
            return ([None] * 4, 0)
    else:
        return ([None] * 4, 0)

    pts, n = [], 0
    for it in items:
        p = as_point(it)
        if p is not None and min_conf > 0.0:
            c = kpt_conf(it)
            if c is not None and c < float(min_conf):
                p = None                                  # 置信度不够 → 当缺失
        pts.append(p)
        if p is not None:
            n += 1
    return (pts, n)


# ================================================================ ★ 连接件 → 候选点
def _median(vals):
    """中位数（空列表返回 None）

    ★ 归类分左右/上下**必须用中位数而不是均值**：均值会被离群候选（错检、远处门的
      残留件）整个拽偏，中位数不会。
    """
    v = sorted(float(x) for x in (vals or []))
    n = len(v)
    if n == 0:
        return None
    m = n // 2
    return v[m] if (n % 2) else 0.5 * (v[m - 1] + v[m])


def corner_points(dets, ref='center', min_size_px=0.0, min_conf=0.0):
    """把「连接件 det 列表」转成候选点列表（每个框 → 一个角点候选）

    参数：
      dets        已按类别筛过的 det 字典列表（每个要有 bbox；label 筛选由调用方做）
      ref         'center' = 取框中心当角点（默认）
                  'inner'  = 取框**靠门内**的那个角（连接件贴在门框转角上，
                             取朝向门心的一角更贴近理论角点）
      min_size_px 框的等效边长下限（太小的框尺寸估计不可信，别拿它去排序距离）
      min_conf    单件置信度下限

    返回 `[{'pt':(x,y), 'size':float, 'score':float, 'box':(x1,y1,x2,y2)}, ...]`

    ★ size = sqrt(框宽 × 框高)：**同规格连接件 ⇒ ∝ 1/z**，是"哪扇门更近"的唯一
      可靠标尺（门框宽度含 cosψ、门框高度含门高 H，两者都不干净，见多门选择）。
    """
    boxes = []
    for d in (dets or []):
        if not isinstance(d, dict):
            continue
        try:
            sc = float(d.get('score', 0.0))
        except (TypeError, ValueError):
            sc = 0.0
        if min_conf > 0.0 and sc < float(min_conf):
            continue
        bb = d.get('bbox') or []
        if len(bb) < 4:
            continue
        try:
            x1, y1, x2, y2 = [float(v) for v in bb[:4]]
        except (TypeError, ValueError):
            continue
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        bw, bh = x2 - x1, y2 - y1
        if bw <= _EPS or bh <= _EPS:
            continue
        size = math.sqrt(bw * bh)
        if size < float(min_size_px):
            continue
        boxes.append((x1, y1, x2, y2, size, sc))

    if not boxes:
        return []

    out = []
    if str(ref).lower() == 'inner' and len(boxes) >= 2:
        # 粗门心 = 全部框中心的均值；每个框取**离它最近的那个角**（= 靠门内一角）
        gx = sum(0.5 * (b[0] + b[2]) for b in boxes) / len(boxes)
        gy = sum(0.5 * (b[1] + b[3]) for b in boxes) / len(boxes)
        for (x1, y1, x2, y2, size, sc) in boxes:
            four = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))
            pt = min(four, key=lambda p: math.hypot(p[0] - gx, p[1] - gy))
            out.append({'pt': pt, 'size': size, 'score': sc, 'box': (x1, y1, x2, y2)})
        return out

    for (x1, y1, x2, y2, size, sc) in boxes:
        out.append({'pt': (0.5 * (x1 + x2), 0.5 * (y1 + y2)),
                    'size': size, 'score': sc, 'box': (x1, y1, x2, y2)})
    return out


# ================================================================ ★ 分门（按尺寸分层）
def group_by_size(cands, tol=0.25):
    """候选点 → 按「连接件像面尺寸」分层的若干组（每组 ≈ 一扇门）

    ★ 为什么不能按**图像距离**聚类：前后两扇门在画面上会重叠，近门的连接件与
      远门的连接件在像面上可能挨得很近 ⇒ 按距离聚类必然串门。
      而连接件是同一规格零件、尺寸 ∝ 1/z ⇒ **同门尺寸相近、异门差得开**。

    算法：从最大尺寸的候选出发作种子，凡 size ≥ 种子·(1−tol) 的归入同组，
    剩下的再递归一轮。返回**按尺寸降序**的组列表（第一组 = 最近的门）。
    """
    rest = sorted([c for c in (cands or []) if c and c.get('size')],
                  key=lambda c: -float(c['size']))
    t = float(tol)
    if t < 0.05:
        t = 0.05
    if t > 0.9:
        t = 0.9
    groups = []
    while rest:
        seed = float(rest[0]['size'])
        lo = seed * (1.0 - t)
        g = [c for c in rest if float(c['size']) >= lo]
        rest = [c for c in rest if float(c['size']) < lo]
        groups.append(g)
    return groups


def group_size(g):
    """一组的代表尺寸 = 组内 size 的中位数（抗单件误检）"""
    return _median([c.get('size') for c in (g or [])]) or 0.0


def group_center(g):
    """一组的中心（组内点均值）；空组返回 None"""
    pts = [c.get('pt') for c in (g or []) if c and c.get('pt')]
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def pick_group(groups, prev_center=None, continuity_tol_px=0.0):
    """从若干组里选**最前面（最近）**的那一扇 → (group, 说明)

    排序量 = 组代表尺寸（∝ 1/z，同规格零件 ⇒ 不受门高、不受门旋转影响）。
    `continuity_tol_px > 0` 且给了 prev_center 时：**只要上一帧那扇门还在**
    （组中心离 prev_center ≤ 该门限），就继续选它 —— 这是"锁定"的一半，
    另一半是调用方的滞回（见 quad_vision 的选门逻辑）。

    ⚠ 单帧的尺寸比较必然抖；真正的稳定来自"锁定 + 滞回 + 单调逼近"。
    """
    gs = [g for g in (groups or []) if g]
    if not gs:
        return (None, '没有候选组')
    if len(gs) == 1:
        return (gs[0], '唯一一扇门')
    if prev_center is not None and float(continuity_tol_px) > 0.0:
        for g in gs:
            c = group_center(g)
            if c is None:
                continue
            d = math.hypot(c[0] - prev_center[0], c[1] - prev_center[1])
            if d <= float(continuity_tol_px):
                return (g, '延续上一帧那扇门（中心距 %.0fpx）' % d)
    best, bs = gs[0], group_size(gs[0])
    for g in gs[1:]:
        s = group_size(g)
        if s > bs:
            best, bs = g, s
    return (best, '尺寸最大 ⇒ 最近（%.1fpx）' % bs)


# ================================================================ ★ 四象限归类
def classify(cands):
    """候选点 → 四个槽位（tl/tr/br/bl）→ (slots, meta)

    用**中位数**分左右、上下（抗离群）。同一槽落了多个候选 = **槽冲突**：
    留 score 最高的，其余丢弃 —— ⚠ 丢的是**候选点**，**不是整帧**。

    meta = {'n', 'x_med', 'y_med', 'conflict', 'single'}
      single：只有一个候选时，那个候选（其余槽位全 None）

    点数 < 2 时不做任何筛选（唯一的信息不能丢）。
    """
    pts = [c for c in (cands or []) if c and c.get('pt')]
    n = len(pts)
    meta = {'n': n, 'x_med': None, 'y_med': None, 'conflict': 0, 'single': None}
    if n == 0:
        return ({}, meta)
    if n == 1:
        meta['single'] = pts[0]
        return ({'tl': None, 'tr': None, 'br': None, 'bl': None}, meta)

    mx = _median([c['pt'][0] for c in pts])
    my = _median([c['pt'][1] for c in pts])
    meta['x_med'], meta['y_med'] = mx, my
    buckets = {'tl': [], 'tr': [], 'br': [], 'bl': []}
    for c in pts:
        x, y = float(c['pt'][0]), float(c['pt'][1])
        k = ('t' if y <= my else 'b') + ('l' if x <= mx else 'r')
        buckets[k].append(c)
    out = {}
    for k in ORDER:
        lst = buckets.get(k) or []
        if len(lst) > 1:                     # 槽冲突：只留最高分
            meta['conflict'] += len(lst) - 1
            lst = sorted(lst, key=lambda c: -float(c.get('score') or 0.0))
        out[k] = lst[0] if lst else None
    return (out, meta)


def slots_to_quad(slots):
    """槽位字典 → 4 元组 [tl, tr, br, bl]（缺失为 None）"""
    s = slots or {}
    return [((s.get(k) or {}).get('pt') if s.get(k) else None) for k in ORDER]


def count_pts(quad):
    """4 元组里非 None 的个数"""
    return len([p for p in (quad or []) if p is not None])


# ================================================================ ★ 可用等级
def quad_lvl(quad):
    """★ 可用等级：这一帧能算多少（0~4）

    4 四角齐全 ⇒ 对角交点中心 + **psi 正对** + 面积
    3 缺一角   ⇒ 补第 4 角取交点，只有中心/横向，**没有 psi**
    2 两个角   ⇒ 两点中点，只有横向
    1 一个角   ⇒ 只能"朝它转"
    0          ⇒ 本帧无观测

    ★ **psi 必须 4 个角**：psi = 左竖边像长/右竖边像长，两条竖边各自需要
      上下两个端点；缺任何一角就有一条竖边是残的。
    """
    if not quad:
        return 0
    n = count_pts(quad)
    return n if n <= 4 else 4


def lvl_kind(quad):
    """等级的细分类型（给日志用）：'quad'/'tri'/'diag'/'row'/'col'/'one'/'none'"""
    n = count_pts(quad)
    if n == 0:
        return 'none'
    if n == 1:
        return 'one'
    if n == 4:
        return 'quad'
    if n == 3:
        return 'tri'
    tl, tr, br, bl = quad
    if tl is not None and br is not None:
        return 'diag'
    if tr is not None and bl is not None:
        return 'diag'
    if tl is not None and tr is not None:
        return 'row'
    if bl is not None and br is not None:
        return 'row'
    return 'col'


# ================================================================ 排序 / 校验
def normalize_quad(pts):
    """把 4 个点排成 TL, TR, BR, BL

    做法：先按「绕质心的方位角」升序排（图像 y 向下时正好得到
    TL→TR→BR→BL 的顺时针序），再把「最左上」（x+y 最小）那个旋到首位。
    任何一个点缺失都返回 None。
    """
    q = [as_point(p) for p in (pts or [])]
    if len(q) != 4 or any(p is None for p in q):
        return None
    cx = sum(p[0] for p in q) / 4.0
    cy = sum(p[1] for p in q) / 4.0
    s = sorted(q, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    k = min(range(4), key=lambda i: (s[i][0] + s[i][1]))
    return s[k:] + s[:k]


def order_ok(quad):
    """★ 用户点名的两条顺序判据（+ 对称补全）→ (ok, why)

    图像坐标 x 右、y 下，顺序 TL, TR, BR, BL：

      ① TL.x < TR.x   上边向右      ← 用户点名
      ② TL.y < BL.y   左边向下      ← 用户点名
      ③ BL.x < BR.x   下边向右
      ④ TR.y < BR.y   右边向下
      ⑤ TL.x < BR.x   左上必在右下左侧
      ⑥ TL.y < BR.y   左上必在右下上侧

    ①+② 单独成立时，仍可能被"四点交叉摆放"骗过（对角互换仍满足 ①②）；
    六条一起才等价于「上/下边都指向 +x、左/右边都指向 +y 且不自交」。
    四角不全时返回 (True, '角点不全，跳过顺序校验')。

    ⚠ R2 口径：**这一条不过时不是"丢帧"，而是降级**（丢掉最可疑的角 → 退到三角）。
      调用方（quad_vision）负责降级，本函数只报结论。
    """
    q = list(quad or [])
    if count_pts(q) != 4:
        return (True, '角点不全，跳过顺序校验')
    tl, tr, br, bl = q
    bad = []
    if not (tl[0] < tr[0]):
        bad.append('上边未向右(TL.x>=TR.x)')
    if not (tl[1] < bl[1]):
        bad.append('左边未向下(TL.y>=BL.y)')
    if not (bl[0] < br[0]):
        bad.append('下边未向右(BL.x>=BR.x)')
    if not (tr[1] < br[1]):
        bad.append('右边未向下(TR.y>=BR.y)')
    if not (tl[0] < br[0]):
        bad.append('TL 不在 BR 左侧')
    if not (tl[1] < br[1]):
        bad.append('TL 不在 BR 上侧')
    return ((not bad), '；'.join(bad))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def is_convex(quad):
    """四点是否构成严格凸四边形

    ★ 平面矩形经射影后一定还是凸的（射影保持凸性），所以「不凸」= 有角点检测错了，
      这时宁可不认这个四边形（退回整门 bbox），也别拿它去做控制。
    """
    if not quad or len(quad) != 4:
        return False
    try:
        s = [_cross(quad[i], quad[(i + 1) % 4], quad[(i + 2) % 4]) for i in range(4)]
    except (TypeError, IndexError):
        return False
    return all(v > _EPS for v in s) or all(v < -_EPS for v in s)


def quad_valid(quad, min_edge_px=3.0, min_area_px=100.0):
    """四边形可用性检查 → (ok, 原因文字)"""
    if not quad or len(quad) != 4 or any(p is None for p in quad):
        return (False, '角点不足 4 个')
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        if math.hypot(b[0] - a[0], b[1] - a[1]) < float(min_edge_px):
            return (False, '第 %d 条边过短(角点重合)' % (i + 1))
    if not is_convex(quad):
        return (False, '非凸(有角点错检)')
    if quad_area(quad) < float(min_area_px):
        return (False, '面积过小')
    return (True, '')


def drop_worst(quad, why=''):
    """★ 四角不全自洽时：丢掉**最可疑的那一个角** → 降到三角（不丢帧）

    可疑度排序（谁最可能是错检）：
      1. 与"补出来的位置"（平行四边形预测）差得最远的那个 —— 其余三点自洽、
         唯独它不合群 ⇒ 它就是错的那个
      2. 退化（与其他点共线/重合）的

    返回新的 4 元组（缺 1 个）→ (quad, 被丢掉的下标)；无法判定时返回 (原样, None)
    """
    q = list(quad or [])
    if count_pts(q) != 4:
        return (q, None)
    worst, wi = -1.0, None
    for i in range(4):
        try:
            pred = fill_missing([None if j == i else q[j] for j in range(4)])
        except Exception:
            continue
        if count_pts(pred) != 4:
            continue
        d = math.hypot(pred[i][0] - q[i][0], pred[i][1] - q[i][1])
        if d > worst:
            worst, wi = d, i
    if wi is None:
        return (q, None)
    out = list(q)
    out[wi] = None
    return (out, wi)


def fill_missing(quad):
    """用「对角和相等」（平行四边形假设）补出缺失的那一个角

    TL + BR = TR + BL（矢量式，对任意平行四边形精确；对射影矩形是近似）。
    缺 2 个及以上时原样返回（不做无依据的猜测）。

    ⚠ 这是**仿射近似**：偏航大时补出来的角会偏，所以补位后只用于**估中心**，
      绝不用它去算 psi（psi 要求真实四角 ⇒ 只有 lvl=4 才算）。
    """
    q = list(quad or [])
    if len(q) != 4:
        return q
    miss = [i for i, p in enumerate(q) if p is None]
    if len(miss) != 1:
        return q
    i = miss[0]
    tl, tr, br, bl = q
    try:
        if i == 0:
            q[0] = (tr[0] + bl[0] - br[0], tr[1] + bl[1] - br[1])
        elif i == 1:
            q[1] = (tl[0] + br[0] - bl[0], tl[1] + br[1] - bl[1])
        elif i == 2:
            q[2] = (tr[0] + bl[0] - tl[0], tr[1] + bl[1] - tl[1])
        else:
            q[3] = (tl[0] + br[0] - tr[0], tl[1] + br[1] - tr[1])
    except (TypeError, IndexError):
        return list(quad or [])
    return q


# ================================================================ 基础量
def edge_lens(quad):
    """四条边长度 → (top, right, bottom, left)（按 TL,TR,BR,BL 顺序）"""
    tl, tr, br, bl = quad
    return (math.hypot(tr[0] - tl[0], tr[1] - tl[1]),
            math.hypot(br[0] - tr[0], br[1] - tr[1]),
            math.hypot(bl[0] - br[0], bl[1] - br[1]),
            math.hypot(tl[0] - bl[0], tl[1] - bl[1]))


def poly_area(pts):
    """任意点数的鞋带面积（点不足 3 个返回 None）"""
    p = [x for x in (pts or []) if x is not None]
    if len(p) < 3:
        return None
    s = 0.0
    for i in range(len(p)):
        x1, y1 = p[i]
        x2, y2 = p[(i + 1) % len(p)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def quad_area(quad):
    """鞋带公式面积（px²）；面积 ∝ 1/z² 且**与偏航无关**，比 bbox 的 w·h 干净"""
    if not quad or len(quad) != 4:
        return 0.0
    s = 0.0
    for i in range(4):
        x1, y1 = quad[i]
        x2, y2 = quad[(i + 1) % 4]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def quad_aabb(quad):
    """四点的轴对齐外框 → (x1, y1, x2, y2)"""
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    return (min(xs), min(ys), max(xs), max(ys))


def quad_size(quad):
    """投影宽高 → (w, h)

    ★ 取四点 AABB 而不是边长的平均：
      矩形经射影后，左右两条竖边的 **u 坐标各自相同**（竖边在门平面内无 x 分量），
      所以 AABB 宽**恰好**等于「左右竖边之间的投影宽度」 = f·W·cosψ/z，
      AABB 高同理。这样算出来的 w/h 与 bbox 的 w/h 口径一致，下游阈值可平移。
    """
    x1, y1, x2, y2 = quad_aabb(quad)
    return (x2 - x1, y2 - y1)


def out_of_frame(quad, img_w, img_h, margin=0.0):
    """★ 出画的角点数 → (n_out, n_pts)

    穿越判据之一：门框贴脸时四角会依次跑出画面。
    ⚠ 必须在**还能看见时**记录峰值（全出画时模型本身就检不到了 ⇒ 那一帧必然是 L0）。
    """
    p = [x for x in (quad or []) if x is not None]
    if not p:
        return (0, 0)
    m = float(margin)
    n = 0
    for (x, y) in p:
        if x < -m or x > float(img_w) + m or y < -m or y > float(img_h) + m:
            n += 1
    return (n, len(p))


def out_frac(quad, img_w, img_h, margin=0.0):
    """出画比例（0~1；无点时返回 0）"""
    n, t = out_of_frame(quad, img_w, img_h, margin=margin)
    return (float(n) / float(t)) if t else 0.0


# ================================================================ 门中心
def intersect_lines(p1, p2, p3, p4):
    """直线 p1p2 与 p3p4 的交点；近平行返回 None（不抛）"""
    (x1, y1), (x2, y2) = p1, p2
    (x3, y3), (x4, y4) = p3, p4
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < _EPS:
        return None
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    return ((a * (x3 - x4) - (x1 - x2) * b) / d,
            (a * (y3 - y4) - (y1 - y2) * b) / d)


def quad_center(quad):
    """门中心 = **两条对角线的交点**（射影精确）；退化时退回坐标平均

    ⚠ 不是坐标平均！pole 平均只在仿射（正对）时才对，偏航时门框中心会被
      「更近的那条竖边」往那头拽走 psi/2·w —— 这正是现役整门 bbox 的系统性偏差来源。
    """
    if not quad or len(quad) != 4:
        return None
    tl, tr, br, bl = quad
    c = intersect_lines(tl, br, tr, bl)
    if c is None or c[0] != c[0] or c[1] != c[1]:        # None / NaN
        return (sum(p[0] for p in quad) / 4.0, sum(p[1] for p in quad) / 4.0)
    return c


def _finite(p):
    return p is not None and p[0] == p[0] and p[1] == p[1]


def partial_center(quad):
    """★ 按**当前可用角点数**估门中心（信息用尽，不丢帧）→ (cx, cy) | None

      lvl=4 四角齐全 ⇒ 对角交点（射影精确）
      lvl=3 缺一角   ⇒ 平行四边形补第 4 角后再取交点
      lvl=2 两个角   ⇒ 两点中点（偏航时略有偏差，但对中够用）
      lvl=1 一个角   ⇒ 就返回它（调用方按 lvl 自己降级使用）
      lvl=0          ⇒ None
    """
    q = list(quad or [])[:4] if quad else []
    p = [x for x in q if x is not None]
    n = len(p)
    if n == 0:
        return None
    if n == 1:
        return (float(p[0][0]), float(p[0][1]))
    if n >= 4:
        c = quad_center(q)
        if _finite(c):
            return (float(c[0]), float(c[1]))
    if n == 3:
        filled = fill_missing(q)
        if count_pts(filled) == 4:
            c = quad_center(filled)
            if _finite(c):
                return (float(c[0]), float(c[1]))
    return (sum(float(x[0]) for x in p) / n, sum(float(x[1]) for x in p) / n)


def partial_area(quad):
    """按可用点算的面积（< 3 点返回 None）"""
    return poly_area([x for x in (quad or []) if x is not None])


def quad_span(quad):
    """可用角点的外接框宽高 → (w, h)；无点返回 (0, 0)

    对 lvl=4 与 quad_size 同值；对 2~3 点给出"已知点覆盖的宽度"，
    用作 q_ex 的归一化分母（与距离无关，与 viskf 的 e_x 同口径）。
    """
    p = [x for x in (quad or []) if x is not None]
    if not p:
        return (0.0, 0.0)
    xs = [float(q[0]) for q in p]
    ys = [float(q[1]) for q in p]
    return (max(xs) - min(xs), max(ys) - min(ys))


# ================================================================ 正对量
def psi_proxy(quad):
    """★ 正对误差代理：psi = (rho-1)/(rho+1)，rho = 左竖边像长 / 右竖边像长

    闭式：psi = (W/(2z))·sin(psi_r)  —— **>0 表示机头偏右，应左转修正**。
    免标定（不需要焦距/门宽真值）、有符号、零位正确、对距离单调。

    ⚠ **必须有真实四角**（lvl=4）：缺任何一角都会让某条竖边变残 ⇒ 返回 None。
    """
    if not quad or len(quad) != 4 or any(p is None for p in quad):
        return None
    top, right, bottom, left = edge_lens(quad)
    if left < _EPS or right < _EPS:
        return None
    rho = left / right
    return (rho - 1.0) / (rho + 1.0)


def psi_from_centers(quad, bbox_cx):
    """用「四边形中心 与 整门 bbox 中心」的差反算 psi（**与 psi_proxy 恒等**，交叉验证用）

    恒等式（可自行验算）：
        (u_bbox_center - u_true) / w_proj  ==  -psi/2
      ⇒ psi == 2·(u_true - u_bbox_center) / w_proj
    两个来源互相独立（一个用竖边像长比、一个用横向位置差），
    现场拿它对不上就说明角点检错了。
    """
    c = quad_center(quad)
    if c is None:
        return None
    w, _ = quad_size(quad)
    if w < _EPS:
        return None
    return 2.0 * (c[0] - float(bbox_cx)) / w


def _line_angle_deg(a, b):
    """线段方向角，归一到 (-90, 90]（方向无所谓，只关心是否平行）"""
    d = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180.0
    return d - 180.0 if d > 90.0 else d


def tilt_deg(quad):
    """上横边倾角（deg）

    ⚠ **只做诊断**：其符号取决于门在光轴上方还是下方，过门时门从画面上方滑到下方，
      符号会在半途自己翻掉。控制量请用 psi。
    """
    if not quad or len(quad) != 4:
        return None
    tl, tr = quad[0], quad[1]
    if tl is None or tr is None:
        return None
    return _line_angle_deg(tl, tr)


def par_deg(quad):
    """不平行度 → (par_h, par_v)，单位 deg

    par_h = 左竖边 vs 右竖边（不为 0 ⇒ **俯仰**：左右两柱在像面上不等高）
    par_v = 上横边 vs 下横边（不为 0 ⇒ **偏航**：与 psi 同源，量纲不同）
    正对时两者恒为 0；⚠ 绝对值还随门在画面中的位置变化，**不作为判据**。
    """
    if not quad or len(quad) != 4 or any(p is None for p in quad):
        return (None, None)
    tl, tr, br, bl = quad

    def _fold(x, y):
        d = abs(x - y) % 180.0
        return min(d, 180.0 - d)

    return (_fold(_line_angle_deg(tl, bl), _line_angle_deg(tr, br)),
            _fold(_line_angle_deg(tl, tr), _line_angle_deg(bl, br)))


def orth_err_deg(quad):
    """四角偏离 90° 的最大量（deg）

    ★ 用户口径：「如果上下、左右分别平行且角度接近 90 度，就近似正对」。
      但要注意「角度接近 90°」**抓不到滚转** —— 一个整体倾斜的矩形照样四角 90°
      （直角旋转后还是直角）。滚转要靠 IMU / 上边是否水平来判。
      正对时该量恒为 0（矩形射影后还是矩形）。
    """
    if not quad or len(quad) != 4 or any(p is None for p in quad):
        return None
    worst = 0.0
    for i in range(4):
        o = quad[(i + 3) % 4]
        a = quad[i]
        b = quad[(i + 1) % 4]
        v1 = (o[0] - a[0], o[1] - a[1])
        v2 = (b[0] - a[0], b[1] - a[1])
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 < _EPS or n2 < _EPS:
            return None
        cosv = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        worst = max(worst, abs(math.degrees(math.acos(cosv)) - 90.0))
    return worst


def psi_deg(quad, fx_px):
    """把 psi 换算成**角度估计**（deg）；fx 未知（<=0）返回 None

    tan(psi_r) = 2·f·psi / w_proj   （f、w_proj 同为像素 ⇒ 与距离无关）
    推导：psi = (W/(2z))·sinψ，w_proj = f·W·cosψ/z ⇒ 2f·psi/w_proj = tanψ。
    ★ 有了它，对准死区就能用「真实角度」定，不再随距离变化（推荐）；
      没有焦距时用无量纲的 psi，缺点是**越近判据越严**。
    """
    p = psi_proxy(quad)
    if p is None or fx_px is None or float(fx_px) <= 0.0:
        return None
    w, _ = quad_size(quad)
    if w < _EPS:
        return None
    return math.degrees(math.atan(2.0 * float(fx_px) * p / w))


# ================================================================ ★ 多门：距离代理
def size_psi(size_l, size_r):
    """★ 用左右连接件框的**像面尺寸比**估 psi（lvl < 4 时的替代源，实验项）

    连接件是同一规格零件 ⇒ 像面尺寸 ∝ 1/z ⇒ 与竖边像长比同源（都是"哪边更近"）。
    符号约定与 psi_proxy 完全一致：>0 = 左边更近 = 机头偏右 ⇒ 应左转。

    参数是两件的 size（不是点）。返回 (psi, ok)；任一边缺失返回 (None, False)。
    """
    try:
        sl, sr = float(size_l), float(size_r)
    except (TypeError, ValueError):
        return (None, False)
    if sl <= _EPS or sr <= _EPS:
        return (None, False)
    rho = sl / sr
    return ((rho - 1.0) / (rho + 1.0), True)


def size_psi_from_quad(slots):
    """从槽位里的左右连接件尺寸比估 psi（lvl=3 时的替代正对量）

    左边取 tl/bl 里存在的那个，右边取 tr/br 里存在的那个。
    返回 (psi, why)；取不到返回 (None, why)。
    """
    s = slots or {}

    def _sz(k):
        c = s.get(k)
        if not c:
            return None
        try:
            return float(c.get('size'))
        except (TypeError, ValueError):
            return None
    sl = _sz('tl')
    if sl is None:
        sl = _sz('bl')
    sr = _sz('tr')
    if sr is None:
        sr = _sz('br')
    if sl is None or sr is None:
        return (None, '左右连接件尺寸不全')
    v, ok = size_psi(sl, sr)
    return (v if ok else None, '框尺寸比 %.1f/%.1f' % (sl, sr))


# ================================================================ 双立柱 → 四边形
def quad_from_post_boxes(box_l, box_r, use_center_x=True):
    """左右两根立柱的检测框 → 四边形（TL, TR, BR, BL）

    这是「方案 B（两根柱子各一个框）」的桥接函数：完全**不需要关键点模型**，
    纯 bbox 标注就能拿到 psi —— 因为 psi 只用到「两条竖边的像长比」。

    use_center_x=True 取柱框的**中心线**当边（门宽 = 两柱中心距）：
      ★ 与「四角点」的外沿口径差一个柱宽，但 psi 只比较**两条竖边长度**，
        两柱口径一致 ⇒ psi 不受影响。
    ⚠ 返回 None = 两框在 x 上重叠/顺序反了（不是一对左右柱）。

    调用方必须自己保证：两柱**都没贴画面上下边**，否则像长被裁 ⇒ psi 失真。
    """
    try:
        lx1, ly1, lx2, ly2 = [float(v) for v in box_l[:4]]
        rx1, ry1, rx2, ry2 = [float(v) for v in box_r[:4]]
    except (TypeError, ValueError, IndexError):
        return None
    lx = 0.5 * (lx1 + lx2) if use_center_x else min(lx1, lx2)
    rx = 0.5 * (rx1 + rx2) if use_center_x else max(rx1, rx2)
    if rx <= lx:
        return None
    # ★ 上下端**自己归一化**：千万不要假设调用方给的 y1 < y2。
    #   模型 bbox 顺序、标注口径都可能不同；本模块的离线用例就踩过一次 ——
    #   传进来的是 [TL.x, TL.y, BL.x, BL.y]，图像 y 向下时 TL.y > BL.y，
    #   若直接当 (top, bottom) 用，整个四边形会上下翻转 ⇒ 顺序校验随即判"不自洽"
    #   ⇒ 降级到三角 ⇒ **psi 直接消失**（现象是"明明给了框却算不出正对量"）。
    ltop, lbot = min(ly1, ly2), max(ly1, ly2)
    rtop, rbot = min(ry1, ry2), max(ry1, ry2)
    return [(lx, ltop), (rx, rtop), (rx, rbot), (lx, lbot)]
