# -*- coding: utf-8 -*-
"""red_pole_cv —— 离线自测（合成图，零板端依赖）

跑法（本机工作区 venv **没有 cv2**，用 Anaconda 的）：
    D:\\Anaconda\\python.exe test_quad_cv.py            # 全部断言
    D:\\Anaconda\\python.exe test_quad_cv.py --probe    # 打印几何/阈值对照表，不断言

★ 本文件的价值：**几何与 lvl 逻辑不依赖板端数据** ⇒ 在 Stage 0 抓帧之前就能把算法钉死。
  只有"阈值/红度可分离度"必须等真实帧（见 `../开发计划.md` §5 Stage 0）。
  ★★ 最重要的一条断言：**"宁可降级也不出错 ψ"** ——
     遮挡 / 贴边 / 红度不足 时，lvl 必须**诚实地**降下来，且**绝不给一个错的 psi**。
"""
import math
import os
import sys

try:
    import cv2
    import numpy as np
except Exception as e:                                # pragma: no cover
    print('需要 cv2 + numpy（用 D:\\Anaconda\\python.exe 跑）：%s' % e)
    sys.exit(2)

_HERE = os.path.dirname(os.path.abspath(__file__))        # .../branches/red_pole_cv/tests
_BR = os.path.dirname(_HERE)                              # .../branches/red_pole_cv


def _find(rel, start, up=6):
    d = start
    for _ in range(up):
        p = os.path.join(d, *rel)
        if os.path.isdir(p):
            return p
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return None


# 本目录（kit 布局）+ 兼容原 branches/red_pole_cv/tests 布局
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_BR, 'board'))
# 分支①的 quad_geom（psi/中心/面积都复用它）
_GK = _find(('branches', 'gate_kpt', 'board'), os.path.dirname(_BR)) or \
      _find(('gate_kpt', 'board'), _BR)
if _GK and _GK not in sys.path:
    sys.path.insert(0, _GK)

import quad_cv_det as D                                   # noqa: E402
import frame_io as FIO                                    # noqa: E402
try:
    import quad_geom as G                                 # noqa: E402
except Exception:                                         # pragma: no cover
    G = None

# ---------------------------------------------------------------- 迷你用例框架
_CASES = []
_PASSED = 0
_FAILED = []


def case(title):
    def deco(fn):
        _CASES.append((title, fn))
        return fn
    return deco


def approx(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol


# ---------------------------------------------------------------- 合成图
def _gate_pts3(yaw_right_deg=0.0, z=2.0, gx=0.0, gy=0.0, W=1.0, H=1.0):
    """门的 4 个角在**相机系**下的 3D 坐标（顺序 TL,TR,BR,BL）"""
    psi = -math.radians(float(yaw_right_deg))
    ug = (math.cos(psi), 0.0, -math.sin(psi))
    Gp = (float(gx), float(gy), float(z))
    return [(Gp[0] + sx * (W / 2.0) * ug[0], Gp[1] + sy * (H / 2.0),
             Gp[2] + sx * (W / 2.0) * ug[2])
            for sx, sy in ((-1, +1), (+1, +1), (+1, -1), (-1, -1))]


def project_gate(yaw_right_deg=0.0, z=2.0, gx=0.0, gy=0.0, W=1.0, H=1.0,
                 f=554.0, img_w=640.0, img_h=480.0):
    """针孔投影（口径与 `branches/gate_kpt/tests/test_quad.py` 逐字相同）

    yaw_right_deg > 0 = 机头右偏（门绕竖轴转 -psi_r）⇒ 左柱更近、左竖边更长 ⇒ psi > 0。
    """
    return [(img_w / 2.0 + f * P[0] / P[2], img_h / 2.0 + f * P[1] / P[2])
            for P in _gate_pts3(yaw_right_deg, z, gx, gy, W, H)]


def bbox_of(corners):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    return (min(xs), min(ys), max(xs), max(ys))


def psi_of(corners):
    """真值 psi（优先用 quad_geom，保证与本分支口径一致）"""
    if G is not None:
        return G.psi_proxy(corners)
    tl, tr, br, bl = corners
    l = math.hypot(bl[0] - tl[0], bl[1] - tl[1])
    r = math.hypot(br[0] - tr[0], br[1] - tr[1])
    return (l / r - 1.0) / (l / r + 1.0)


def make_door_img(yaw=0.0, z=2.0, gx=0.0, gy=0.0, W=0.70, H=0.50, d_pipe=0.030,
                  f=554.0, img_w=640, img_h=480, bg=(112, 98, 68), noise=6.0,
                  jpeg_q=95, red=(30, 30, 200), pole_color=None,
                  balls=(), wipe=(), seed=7):
    """画一块红管门环 → (img, corners_true, bbox)

    * `pole_color != None` ⇒ 用指定颜色画杆（测"红度失败"时给灰色）
    * `balls=[(cx,cy,r), ...]` ⇒ 额外画红色实心球（干扰）
    * `wipe=[(x1,y1,x2,y2), ...]` ⇒ 把这几个矩形抹成背景（测遮挡）
    """
    rng = np.random.RandomState(seed)
    pts3 = _gate_pts3(yaw, z, gx, gy, W, H)
    corners = [(img_w / 2.0 + f * P[0] / P[2], img_h / 2.0 + f * P[1] / P[2]) for P in pts3]
    img = np.zeros((img_h, img_w, 3), np.uint8)
    img[:, :] = bg
    if noise:
        n = rng.normal(0.0, noise, (img_h, img_w, 1)).repeat(3, axis=2)
        img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)

    col = pole_color if pole_color is not None else red
    q = [(int(round(p[0])), int(round(p[1]))) for p in corners]
    # ★ 每根管按**它自己的深度**定像宽（f·d/z）——
    #   这样"左右立柱像宽比"才真的含 ψ；四根都画同样粗就没法验像宽比了。
    for i in range(4):
        j = (i + 1) % 4
        zm = max(0.05, 0.5 * (pts3[i][2] + pts3[j][2]))
        th_i = max(2, int(round(f * d_pipe / zm)))
        cv2.line(img, q[i], q[j], col, th_i, cv2.LINE_AA)

    for (cx, cy, r) in balls:
        cv2.circle(img, (int(round(cx)), int(round(cy))), int(round(r)), red, -1, cv2.LINE_AA)
    for (x1, y1, x2, y2) in wipe:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), bg, -1)

    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_q)])
    if ok:
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    # ★ 真值也按 TL,TR,BR,BL 规范化（与 quad_geom / detect 同口径）
    return img, D.normalize_quad(corners), bbox_of(corners)


def max_corner_err(a, b):
    """同一多边形的最小角点误差（允许循环旋转/反向 —— 只看"是不是同一个四边形"）

    `project_gate` 的 TL 取在图像**下方**（写它时按 y 向上假设），与 `quad_geom`
    的 TL（图像左上）差一次上下翻转。翻转是同一个多边形，对 psi / 中心**无影响**，
    故这里按多边形等价比较，而不是按点编号硬比。
    """
    bb = list(b)
    best = float('inf')
    for seq in (bb, bb[::-1]):
        for k in range(4):
            r = seq[k:] + seq[:k]
            best = min(best, max(math.hypot(a[i][0] - r[i][0], a[i][1] - r[i][1]) for i in range(4)))
    return best


# ---------------------------------------------------------------- 用例
@case('正对（yaw=0, z=2）：lvl=4、psi≈0、角点准')
def _t1():
    img, ct, bb = make_door_img(yaw=0.0, z=2.0)
    r = D.detect(img, bb)
    assert r['lvl'] == 4, 'lvl=%d why=%s diag=%s' % (r['lvl'], r['why'], r['diag'])
    assert approx(r['fields']['psi'], 0.0, 0.010), 'psi=%.4f' % r['fields']['psi']
    e = max_corner_err([tuple(p) for p in r['corners']], ct)
    assert e < 4.0, '角点误差 %.2f px' % e
    return 'psi=%.4f corner_err=%.2fpx' % (r['fields']['psi'], e)


@case('机头右偏 yaw=+15：psi>0 且 = 真值')
def _t2():
    img, ct, bb = make_door_img(yaw=+15.0, z=2.0)
    r = D.detect(img, bb)
    pt = psi_of(ct)
    assert r['lvl'] == 4, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    assert r['fields']['psi'] > 0.0, 'psi 符号错：%.4f' % r['fields']['psi']
    assert approx(r['fields']['psi'], pt, 0.012), 'psi=%.4f 真值=%.4f' % (r['fields']['psi'], pt)
    return 'psi=%.4f (真 %.4f)' % (r['fields']['psi'], pt)


@case('机头左偏 yaw=-15：psi<0 且 = 真值')
def _t3():
    img, ct, bb = make_door_img(yaw=-15.0, z=2.0)
    r = D.detect(img, bb)
    pt = psi_of(ct)
    assert r['lvl'] == 4, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    assert r['fields']['psi'] < 0.0, 'psi 符号错：%.4f' % r['fields']['psi']
    assert approx(r['fields']['psi'], pt, 0.012), 'psi=%.4f 真值=%.4f' % (r['fields']['psi'], pt)
    return 'psi=%.4f (真 %.4f)' % (r['fields']['psi'], pt)


@case('远端 z=3：杆只剩 ~6px ⇒ lvl 必须 ≥3（诚实降级）')
def _t4():
    img, ct, bb = make_door_img(yaw=+10.0, z=3.0)
    r = D.detect(img, bb)
    assert r['lvl'] >= 3, 'lvl=%d why=%s diag=%s' % (r['lvl'], r['why'], r['diag'])
    return 'lvl=%d psi=%s' % (r['lvl'], r['fields'].get('psi'))


@case('框内有一个红球（不在门后）：不干扰 ψ')
def _t5():
    img0, ct, bb = make_door_img(yaw=+12.0, z=2.0)
    # 把球放在 ROI 内、四边带之外的"门洞"中央
    cx = sum(p[0] for p in ct) / 4.0
    cy = sum(p[1] for p in ct) / 4.0
    img, _, _ = make_door_img(yaw=+12.0, z=2.0, balls=[(cx, cy, 16)])
    pt = psi_of(ct)
    r = D.detect(img, bb)
    assert r['lvl'] == 4, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    assert approx(r['fields']['psi'], pt, 0.015), 'psi=%.4f 真值=%.4f' % (r['fields']['psi'], pt)
    return 'psi=%.4f (真 %.4f)' % (r['fields']['psi'], pt)


@case('红球压在左边带上：宁可降级，绝不给错 ψ')
def _t6():
    img, ct, bb = make_door_img(yaw=0.0, z=2.0)
    tl, tr, br, bl = ct
    mid = (0.5 * (tl[0] + bl[0]), 0.5 * (tl[1] + bl[1]))
    img2, _, _ = make_door_img(yaw=0.0, z=2.0, balls=[(mid[0] - 2, mid[1], 14)])
    pt = psi_of(ct)
    r = D.detect(img2, bb)
    if r['lvl'] == 4:
        assert approx(r['fields']['psi'], pt, 0.030), '给了错 psi：%.4f (真 %.4f)' % (r['fields']['psi'], pt)
    else:
        assert r['fields'].get('psi') is None or r['lvl'] < 4
    return 'lvl=%d psi=%s' % (r['lvl'], r['fields'].get('psi'))


@case('缺一根竖杆（遮挡）⇒ lvl 必须 ≤3')
def _t7():
    img, ct, bb = make_door_img(yaw=0.0, z=2.0)
    tl, tr, br, bl = ct
    x1 = int(min(tl[0], bl[0]) - 12)
    x2 = int(max(tl[0], bl[0]) + 12)
    img2, _, _ = make_door_img(yaw=0.0, z=2.0, wipe=[(x1, min(tl[1], bl[1]) - 16, x2, max(tl[1], bl[1]) + 16)])
    r = D.detect(img2, bb)
    assert r['lvl'] <= 3, '遮挡了还给 lvl=%d' % r['lvl']
    return 'lvl=%d lines=%s' % (r['lvl'], r['diag'].get('lines'))


@case('门被画面左边切掉 ⇒ lvl=1（定向转补全视野），不给 ψ')
def _t8():
    img, ct, bb = make_door_img(yaw=0.0, z=1.2, gx=-0.42)     # 把门推到画面左侧外
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r['lvl'] == 1, 'lvl=%d why=%s diag=%s bb=%s' % (r['lvl'], r['why'], r['diag'], bb)
    return 'lvl=%d clip_h=%s' % (r['lvl'], r['diag'].get('clip_h'))


@case('杆是灰的（红度失败, src=red）⇒ lvl=2（只有横向），不出 ψ')
def _t9():
    img, ct, bb = make_door_img(yaw=+10.0, z=2.0, pole_color=(120, 120, 120))
    r = D.detect(img, bb, opts={'src': 'red', 'red_min_frac': 0.5})
    assert r['lvl'] == 2, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    assert r['fields'].get('psi') is None
    return 'lvl=%d red_frac=%.4f' % (r['lvl'], r['diag'].get('red_frac', -1))


@case('★ 弱红杆：R−max(G,B)<0 但 R>G ⇒ 必须仍被检出（真实帧 00237 的失效模式）')
def _t17():
    # 真实水下：同一扇门两根柱的红度可差 4 倍，其中一根 R−max(G,B) 甚至为负
    weak = (150, 120, 135)          # BGR：R=135 G=120 B=150 ⇒ R−G=+15, R−max(G,B)=−15
    img, ct, bb = make_door_img(yaw=+15.0, z=2.0, pole_color=weak)
    pt = psi_of(ct)
    r = D.detect(img, bb)
    assert r['lvl'] == 4, 'lvl=%d why=%s red_frac=%s' % (
        r['lvl'], r['why'], r['diag'].get('red_frac'))
    assert approx(r['fields']['psi'], pt, 0.02), 'psi=%.4f 真=%.4f' % (r['fields']['psi'], pt)
    # 对照：老口径（R−max(G,B) **且不减去基线**）在这张图上抠不出东西
    r_old = D.detect(img, bb, opts={'red_metric': 'r-maxgb', 'red_rel': False, 'src': 'red'})
    assert r_old['lvl'] < 4, '老口径居然也过了 ⇒ 这条断言失去意义'
    return '新口径 lvl=4 psi=%.4f / 老口径 lvl=%d' % (r['fields']['psi'], r_old['lvl'])


@case('杆是灰的但 src=auto ⇒ 边缘兜底仍能出几何')
def _t10():
    img, ct, bb = make_door_img(yaw=+10.0, z=2.0, pole_color=(120, 120, 120))
    r = D.detect(img, bb, opts={'src': 'auto', 'red_min_frac': 0.5})
    assert r['lvl'] >= 1, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    return 'lvl=%d src=%s' % (r['lvl'], r['src'])


@case('没有 bbox ⇒ lvl=0')
def _t11():
    img, ct, bb = make_door_img()
    r = D.detect(img, None)
    assert r['lvl'] == 0
    return 'lvl=0'


@case('frame_io：MFS1 头读回一致')
def _t12():
    import tempfile
    img, ct, bb = make_door_img(yaw=0.0, z=2.0)
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    jb = buf.tobytes()
    p = os.path.join(tempfile.gettempdir(), 'quad_cv_t12.bin')
    FIO.write_frame(p, 12345, jb, wh=640 * 480, ts_us=1758000000123456)
    rec = FIO.read_latest(p)
    assert rec and rec['seq'] == 12345, rec
    assert rec['jpeg_len'] == len(jb) and rec['jpeg'][:2] == b'\xff\xd8'
    back = FIO.decode(rec['jpeg'], scale=0.5)
    assert back is not None and back.shape[1] in (320,), back.shape
    os.remove(p)
    return 'seq=%d jpeg=%dB half=%s' % (rec['seq'], rec['jpeg_len'], back.shape[:2])


@case('半分辨率解码：psi 不退化（尺度无关）')
def _t13():
    img, ct, bb = make_door_img(yaw=+15.0, z=2.0)
    pts = psi_of(ct)
    bb_half = tuple(v * 0.5 for v in bb)
    half = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
    r = D.detect(half, bb_half, img_wh=(half.shape[1], half.shape[0]))
    assert r['lvl'] >= 3, 'lvl=%d' % r['lvl']
    if r['fields'].get('psi') is not None:
        assert approx(r['fields']['psi'], pts, 0.02), 'psi=%.4f 真=%.4f' % (r['fields']['psi'], pts)
    return 'lvl=%d psi=%s (真 %.4f)' % (r['lvl'], r['fields'].get('psi'), pts)


# ------------------------------------------------ 纵向残缺（2026-10-06 用户口径）
@case('★ 下边切掉门、横杆仍大半可见 ⇒ **不降级**，ψ 方向正确（量级略衰减）')
def _t14():
    img, ct, bb = make_door_img(yaw=+15.0, z=1.2, gy=+0.30)
    ymax = max(p[1] for p in ct)
    assert ymax > img.shape[0] - 4, '前提不成立：门没被下边切到 ymax=%.1f' % ymax
    pt = psi_of(ct)
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r['diag']['clip_v'] is True, r['diag']
    assert r['lvl'] == 4, 'lvl=%d why=%s sup=%s' % (
        r['lvl'], r['why'], r['diag'].get('line_support'))
    psi = r['fields'].get('psi')
    assert psi is not None and psi > 0.0, 'psi 符号错：%s' % psi
    # ★ 下边被切 ⇒ 左右两侧长度被同一条底边截断 ⇒ ψ **量级被压向 0**（本帧 ~40%）。
    #   保守但方向对（宁可少转、绝不反着转）；不做固定增益补偿（截断比例在变）。
    assert 0.30 * pt <= psi <= pt + 0.02, 'psi=%.4f 真=%.4f' % (psi, pt)
    return 'lvl=%d src=%s psi=%.4f(真 %.4f)' % (r['lvl'], r.get('psi_src'), psi, pt)


@case('★ 上边切掉门、横杆仍大半可见 ⇒ **不降级**，ψ 方向正确')
def _t15():
    img, ct, bb = make_door_img(yaw=-15.0, z=1.2, gy=-0.30)
    ymin = min(p[1] for p in ct)
    assert ymin < 4, '前提不成立：门没被上边切到 ymin=%.1f' % ymin
    pt = psi_of(ct)
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r['lvl'] == 4, 'lvl=%d why=%s sup=%s' % (
        r['lvl'], r['why'], r['diag'].get('line_support'))
    psi = r['fields'].get('psi')
    assert psi is not None and psi < 0.0, 'psi 符号错：%s' % psi
    assert -(abs(pt) + 0.02) <= psi <= -0.4 * abs(pt), 'psi=%.4f 真=%.4f' % (psi, pt)
    return 'lvl=%d src=%s psi=%.4f(真 %.4f)' % (r['lvl'], r.get('psi_src'), psi, pt)


@case('★★ 横杆整个出画 ⇒ 那条假线必须被丢，**宁可降级也不给错 ψ**')
def _t18():
    img, ct, bb = make_door_img(yaw=+15.0, z=1.2, gy=+0.50)
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert 'bottom' not in r['diag'].get('lines', []), \
        '假横杆线没被丢掉：%s sup=%s' % (r['diag'].get('lines'), r['diag'].get('line_support'))
    assert r['lvl'] == 3, 'lvl=%d why=%s' % (r['lvl'], r['why'])
    assert r['fields'].get('psi') is None, '给了 ψ：%s' % r['fields'].get('psi')
    return 'lvl=3 lines=%s sup=%s' % (r['diag'].get('lines'), r['diag'].get('line_support'))


@case('像宽比口径（实验开关 psi_width_on）⇒ 符号正确，默认关闭')
def _t19():
    img, ct, bb = make_door_img(yaw=+15.0, z=1.2, gy=+0.50)
    pt = psi_of(ct)
    r_off = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r_off.get('psi_src') != 'width', '默认不该走像宽比'
    r_on = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]),
                    opts={'psi_width_on': True})
    assert r_on.get('psi_src') == 'width', 'psi_src=%s' % r_on.get('psi_src')
    psi = r_on['fields'].get('psi')
    assert psi is not None and psi > 0.0, 'psi 符号错：%s' % psi
    return '关=%s 开=%s psi=%.4f(真 %.4f)' % (r_off.get('psi_src'), r_on.get('psi_src'),
                                              psi, pt)


@case('★ 横向残缺优先级 > 纵向：左右被切时仍必须 lvl=1，绝不给 ψ')
def _t16():
    img, ct, bb = make_door_img(yaw=0.0, z=1.2, gx=-0.42, gy=+0.30)
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r['lvl'] == 1, 'lvl=%d why=%s clip=%s' % (r['lvl'], r['why'], r['diag'])
    assert r['fields'].get('psi') is None
    return 'lvl=1 clip_h=%s clip_v=%s' % (r['diag'].get('clip_h'), r['diag'].get('clip_v'))


@case('★★ 板端 live 口径：全画幅 R−G 恒负 ⇒ 绝对阈值取不到红，相对基线必须救回来')
def _t20():
    # 2026-10-06 板端 live 实测：全画幅 (R−G) 中位 −60、门横杆处**最大也才 0**
    #   ⇒ 绝对口径 `R−G > 0` **一个像素都取不到**（red_frac=0 ⇒ lvl=0）。
    bg = (185, 195, 140)          # BGR：R=140 G=195 ⇒ R−G=−55（水）
    pole = (190, 170, 155)        # BGR：R=155 G=170 ⇒ R−G=−15（门）**仍 <0**，但比水高 +40
    img, ct, bb = make_door_img(yaw=0.0, z=2.0, bg=bg, pole_color=pole)
    pt = psi_of(ct)
    # 绝对口径必须失败（src 锁 red，禁掉边缘兜底，否则会从边缘救回来）
    r_abs = D.detect(img, bb, opts={'red_rel': False, 'src': 'red'},
                     img_wh=(img.shape[1], img.shape[0]))
    assert r_abs['lvl'] < 4, '绝对口径居然也过了 lvl=%d' % r_abs['lvl']
    # 相对基线口径（当前默认）必须拿回完整能力
    r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
    assert r['lvl'] == 4, 'lvl=%d why=%s red_frac=%s' % (
        r['lvl'], r['why'], r['diag'].get('red_frac'))
    assert approx(r['fields']['psi'], pt, 0.02), 'psi=%.4f 真=%.4f' % (r['fields']['psi'], pt)
    return '绝对 lvl=%d / 相对 lvl=4 psi=%.4f(真 %.4f)' % (r_abs['lvl'], r['fields']['psi'], pt)


# ------------------------------------------- 板端真帧的三条假线判据（2026-10-06）
@case('★★ 画幅最外几行整行全红（编码边行）⇒ 必须被edge_trim 剔掉，不许当门')
def _t21():
    # 复现板端 001720 帧：门贴画面下沿（bottom 带被夹到图像底部），
    # 且图像**最后 3 行整行全红**（JPEG 边行偏色，实测 196/196）。
    # 不剔掉 ⇒ bottom 带被"填满" ⇒ 假底边通过所有门。
    img, ct, bb = make_door_img(yaw=+12.0, z=1.4, gy=+0.42)
    h, w = img.shape[:2]
    img = img.copy()
    img[h-3:, :] = (40, 190, 200)            # BGR：R=200 G=190 ⇒ R−G=+10 整行判红
    r_on = D.detect(img, bb, img_wh=(w, h))
    r_off = D.detect(img, bb, img_wh=(w, h), opts={'edge_trim_px': 0})
    fil_on = (r_on['diag'].get('band_fill') or {}).get('bottom')
    fil_off = (r_off['diag'].get('band_fill') or {}).get('bottom')
    assert fil_off is not None, '没测到 bottom（diag=%s）' % (r_off['diag'].get('band_fill'),)
    assert fil_on < fil_off, 'edge_trim 失效：剔后 %.4f ≥ 未剔 %.4f' % (fil_on, fil_off)
    return 'bottom fill 剔后=%.4f < 未剔=%.4f' % (fil_on, fil_off)


@case('★★ 断续杂波（在门外的带里）⇒ 密度门必须砍掉它')
def _t22():
    # 门贴画面下沿（bottom 完全出画）⇒ bottom 带落在**画面最外几行**，
    # 在那里撒一批断续白点（模拟真机底边杂波）⇒ 靠 edge_trim 剔掉后必须是空带。
    img, ct, bb = make_door_img(yaw=0.0, z=1.4, gy=+0.55)
    h, w = img.shape[:2]
    assert max(p[1] for p in ct) > h - 4, '前提不成立：门下沿没出画'
    img = img.copy()
    rng = np.random.RandomState(7)
    for _ in range(60):                # 断续小点，绝不成条
        cx = int(rng.randint(0, w))
        cy = int(rng.randint(int(h*0.88), h))
        cv2.circle(img, (cx, cy), 1, (30, 200, 215), -1)
    r = D.detect(img, bb, img_wh=(w, h))
    assert r['diag'].get('clip_v') is True, '前提不成立'
    assert 'bottom' not in (r['diag'].get('lines') or []), (
        '断续杂波被当成了门底边：%s' % (r['diag'].get('lines'),))
    return 'lvl=%d lines=%s fill=%s' % (r['lvl'], r['diag'].get('lines'),
                                       (r['diag'].get('band_fill') or {}).get('bottom'))


@case('★★ 只剩两根立柱时**不许出 ψ**（立柱可见段恒等长 ⇒ ψ≡0；横杆倾角会反号）')
def _t23():
    # 门被下边切掉 40%+ ⇒ bottom 完全出画。
    # 已证伪两条口径：①立柱可见段比恒 1.0 ⇒ psi=0；②上横杆倾角符号翻转。
    #⇒ 只能诚实退 lvl≤3，**绝不给反号的 ψ**。
    worst = None
    for yaw in (+20.0, -20.0, +6.0, -6.0):
        img, ct, bb = make_door_img(yaw=yaw, z=1.2, gy=+0.45)
        r = D.detect(img, bb, img_wh=(img.shape[1], img.shape[0]))
        assert r['lvl'] <= 3, 'yaw=%+.0f 竟然给了 lvl=%d（psi=%s）' % (
            yaw, r['lvl'], (r.get('fields') or {}).get('psi'))
        assert (r.get('fields') or {}).get('psi') is None, (
            'yaw=%+.0f 给出了 psi=%s' % (yaw, r['fields'].get('psi')))
        worst = 'yaw=%+.0f→lvl%d' % (yaw, r['lvl'])
    return worst


# ---------------------------------------------------------------- runner
def _probe():
    print('%-28s %-4s %-8s %-8s %s' % ('场景', 'lvl', 'psi', 'psi真', '角点误差'))
    for yaw in (-30, -15, 0, 15, 30):
        for z in (1.0, 1.5, 2.0, 3.0):
            img, ct, bb = make_door_img(yaw=yaw, z=z)
            r = D.detect(img, bb)
            e = max_corner_err([tuple(p) for p in r['corners']], ct) if r['corners'] else float('nan')
            print('yaw=%+3d z=%.1f           %-4d %-8.4f %-8.4f %.2f'
                  % (yaw, z, r['lvl'], r['fields'].get('psi') or float('nan'), psi_of(ct), e))
    print()
    print('cv2=%s numpy=%s quad_geom=%s' % (cv2.__version__, np.__version__,
                                             getattr(G, '__file__', None)))


def main():
    global _PASSED
    if '--probe' in sys.argv:
        _probe()
        return 0
    for title, fn in _CASES:
        try:
            note = fn()
            _PASSED += 1
            print('  [PASS] %-42s %s' % (title, note or ''))
        except AssertionError as e:
            _FAILED.append((title, str(e)))
            print('  [FAIL] %-42s %s' % (title, e))
        except Exception as e:                        # noqa: BLE001
            _FAILED.append((title, '%s: %s' % (type(e).__name__, e)))
            print('  [ERR ] %-42s %s: %s' % (title, type(e).__name__, e))
    print()
    print('通过 %d/%d' % (_PASSED, len(_CASES)))
    if _FAILED:
        print('失败：')
        for t, e in _FAILED:
            print('  - %s : %s' % (t, e))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
