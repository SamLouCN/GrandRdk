# -*- coding: utf-8 -*-
"""demo_video.py —— 对一段视频跑「框内 OpenCV 找红杆」，输出标注视频 + 统计

没有 YOLO 时，bbox 由「全画幅相对红度 → 最大连通域」回归（模拟 YOLO 的角色）。
有 YOLO 检测结果的场合请用 demo_images.py（逐帧 json 提供 bbox）。

用法:
    D:/Anaconda/python.exe demo_video.py 视频.mp4 [--out 输出目录] [--show]
依赖: numpy + opencv（任意 4.x；4.11 与 4.13 数值一致已验证）
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import quad_cv_det as Q  # noqa: E402


def find_bbox_red(frame, min_area=600):
    """bbox 供应商：全画幅相对红度 -> 最大连通域。返回 (x1,y1,x2,y2) 或 None。"""
    m, _ = Q.red_mask(frame)
    if m is None or np.count_nonzero(m) == 0:
        return None
    m2 = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    ncc, lab, st, _ = cv2.connectedComponentsWithStats(m2, 8)
    if ncc <= 1:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if int(st[i, cv2.CC_STAT_AREA]) < min_area:
        return None
    x, y = int(st[i, cv2.CC_STAT_LEFT]), int(st[i, cv2.CC_STAT_TOP])
    w, h = int(st[i, cv2.CC_STAT_WIDTH]), int(st[i, cv2.CC_STAT_HEIGHT])
    return (x, y, x + w, y + h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('video')
    ap.add_argument('--out', default=None, help='输出目录（默认 视频同目录/sim_out）')
    ap.add_argument('--min-area', type=int, default=600)
    ap.add_argument('--opts', default='', help='JSON，覆盖 quad_cv_det.DEFAULTS，如 \'{"min_band_fill":0.04}\'')
    a = ap.parse_args()

    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.video)), 'sim_out')
    os.makedirs(out, exist_ok=True)
    opts = json.loads(a.opts) if a.opts else None

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        print('打不开视频: %s' % a.video)
        return 2
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vw = cv2.VideoWriter(os.path.join(out, 'out_sim.mp4'),
                         cv2.VideoWriter_fourcc(*'mp4v'), fps, (W, H))

    lvl_cnt = {}
    psis = []
    fid = 0
    n_det = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        fid += 1
        bbox = find_bbox_red(frame, a.min_area)
        vis = frame.copy()
        if bbox is None:
            lvl_cnt[None] = lvl_cnt.get(None, 0) + 1
        else:
            n_det += 1
            r = Q.detect(frame, bbox, opts=opts)
            lvl = int(r.get('lvl', 0))
            psi = (r.get('fields') or {}).get('psi')
            lvl_cnt[lvl] = lvl_cnt.get(lvl, 0) + 1
            if psi is not None:
                psis.append(float(psi))
            cv2.rectangle(vis, bbox[:2], bbox[2:], (0, 255, 255), 1)
            if r.get('corners'):
                pts = np.array(r['corners'], dtype=np.int32)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
                for p in pts:
                    cv2.circle(vis, (int(p[0]), int(p[1])), 4, (0, 0, 255), -1)
            txt = 'lvl=%d psi=%s' % (lvl, ('%.3f' % psi) if psi is not None else 'None')
            cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        vw.write(vis)
    cap.release()
    vw.release()

    n_lvl = sum(v for k, v in lvl_cnt.items() if k is not None)
    print('frames=%d  det=%d' % (fid, n_det))
    for lv in (4, 3, 2, 1, 0):
        c = lvl_cnt.get(lv, 0)
        print('  lvl%d: %5.1f%%' % (lv, 100.0 * c / max(1, n_lvl)))
    print('  no-det: %d frames' % lvl_cnt.get(None, 0))
    if psis:
        q = np.percentile(np.abs(np.array(psis)), [5, 50, 95])
        print('|psi| p5/p50/p95: %.3f / %.3f / %.3f' % tuple(q))
    print('标注视频: %s' % os.path.join(out, 'out_sim.mp4'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
