# -*- coding: utf-8 -*-
"""demo_images.py —— 对一组「图 + YOLO 检测 json」跑框内红杆识别，输出统计 + 标注图

图片目录要求: xxx.jpg 与 xxx.json 成对；json 里要有 dets 数组，
每个 det 至少含 label（'door'/'gate'）、bbox=[x1,y1,x2,y2]（板端 momo_det_front.json 就是这个格式）。

用法:
    D:/Anaconda/python.exe demo_images.py 图片目录 [--out 输出目录] [--sample 每档标注图张数]
依赖: numpy + opencv
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import quad_cv_det as Q  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('imgdir')
    ap.add_argument('--out', default=None)
    ap.add_argument('--sample', type=int, default=6, help='每个 lvl 档最多存几张标注图')
    ap.add_argument('--opts', default='', help='JSON，覆盖 quad_cv_det.DEFAULTS')
    a = ap.parse_args()

    out = a.out or os.path.join(a.imgdir, 'sim_out')
    os.makedirs(out, exist_ok=True)
    opts = json.loads(a.opts) if a.opts else None

    lvl_cnt = {}
    why_cnt = {}
    psis = []
    rows = []
    kept = {}

    for jp in sorted(glob.glob(os.path.join(a.imgdir, '*.jpg')) + glob.glob(os.path.join(a.imgdir, '*.png'))):
        jf = os.path.splitext(jp)[0] + '.json'
        if not os.path.isfile(jf):
            continue
        with open(jf, 'r', encoding='utf-8') as f:
            dets = json.load(f).get('dets', [])
        door = None
        for d in dets:
            if d.get('label') in ('door', 'gate'):
                if door is None or d.get('score', 0) > door.get('score', 0):
                    door = d
        fid = os.path.splitext(os.path.basename(jp))[0]
        row = {'frame': fid}
        if door is None:
            row.update(lvl=None, why='no-door-det')
            rows.append(row)
            continue
        img = cv2.imread(jp)
        if img is None:
            continue
        bbox = tuple(int(v) for v in door['bbox'])
        r = Q.detect(img, bbox, opts=opts)
        lvl = int(r.get('lvl', 0))
        psi = (r.get('fields') or {}).get('psi')
        row.update(lvl=lvl, psi=(float(psi) if psi is not None else None),
                   why=r.get('why', ''), lines=r.get('diag', {}).get('lines'))
        rows.append(row)
        lvl_cnt[lvl] = lvl_cnt.get(lvl, 0) + 1
        if r.get('why'):
            why_cnt[r['why']] = why_cnt.get(r['why'], 0) + 1
        if psi is not None:
            psis.append(float(psi))
        tag = 'lvl%d' % lvl
        if kept.get(tag, 0) < a.sample:
            vis = img.copy()
            cv2.rectangle(vis, bbox[:2], bbox[2:], (0, 255, 255), 1)
            if r.get('corners'):
                pts = np.array(r['corners'], dtype=np.int32)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
                for p in pts:
                    cv2.circle(vis, (int(p[0]), int(p[1])), 4, (0, 0, 255), -1)
            txt = 'lvl=%d psi=%s' % (lvl, ('%.3f' % psi) if psi is not None else 'None')
            cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(vis, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imwrite(os.path.join(out, '%s_%s.jpg' % (tag, fid)), vis)
            kept[tag] = kept.get(tag, 0) + 1

    n_lvl = sum(lvl_cnt.values())
    print('frames=%d (door det %d)' % (len(rows), n_lvl))
    for lv in (4, 3, 2, 1, 0):
        c = lvl_cnt.get(lv, 0)
        print('  lvl%d: %5.1f%%' % (lv, 100.0 * c / max(1, n_lvl)))
    if psis:
        q = np.percentile(np.abs(np.array(psis)), [5, 50, 95])
        print('|psi| p5/p50/p95: %.3f / %.3f / %.3f' % tuple(q))
    print('why TOP:')
    for w, c in sorted(why_cnt.items(), key=lambda kv: -kv[1])[:6]:
        print('  %5d  %s' % (c, w[:60]))
    with open(os.path.join(out, 'rows.jsonl'), 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + '\n')
    print('输出: %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
