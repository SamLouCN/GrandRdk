#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay_gate_a.py — 方案A: 真机视频检测回放, 开环验证过门对准算法

数据源: D:/RC/Test_any/out/dets.jsonl（9 月评测时 pc_detect.py 对真机门视频
20260828_194232.MOV 逐帧推理的产物, 1280x720@60fps, 2025 帧）

流程: dets → GateVision.observe → EFilter → GateMission(ALIGN 常驻) → GatePid
      → dpsi/sway 曲线。机体不动, 闭环不成立, 只验: 符号/死区/限幅/滤波对接/状态迁移。

用法:
    python replay_gate_a.py                 # 默认 none + complementary 两遍
    python replay_gate_a.py --mode none     # 只跑单模式

输出: replay_out/gate_replay.png + gate_replay_<mode>.csv + 终端摘要
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gate_config as GC
from gate_vision import GateVision
from gate_filter import EFilter
from gate_pid import GatePid
from gate_mission import GateMission

DEFAULT_DETS = r'D:\RC\Test_any\out\dets.jsonl'


def cfg_dict():
    return {k: getattr(GC, k) for k in dir(GC) if k.startswith('GATE_')}


def run_pass(mode, rows_src, args):
    """单遍回放, 返回逐帧记录列表。"""
    cfg = cfg_dict()
    cfg['GATE_EF_TYPE'] = mode
    vision = GateVision(cfg['GATE_DET_PATH'], cfg['GATE_IMG_W'], cfg['GATE_IMG_H'],
                        cfg['GATE_STALE_S'], cfg['GATE_LABEL'], cfg['GATE_E_SIGN'])
    filt = EFilter(mode, cfg['GATE_VISKF_PATH'], cfg['GATE_STALE_S'],
                   cfg['GATE_EF_ALPHA'])
    pid = GatePid(cfg['GATE_KP_YAW'], cfg['GATE_KD_YAW'], cfg['GATE_KP_SWAY'],
                  cfg['GATE_SWAY_DEAD'], cfg['GATE_KI'], cfg['GATE_I_MAX'],
                  cfg['GATE_PSI_MAX'])
    recs = []

    def send(psi, surge, sway):
        recs[-1]['psi_target'] = psi
        recs[-1]['surge'] = surge
        recs[-1]['sway'] = sway

    def tel():
        return {'yaw': None, 'omega': 0.0}     # 开环: 无机体反馈

    mission = GateMission(cfg, vision, filt, pid, tel, send,
                          log=lambda m: print('  ' + m), allow_blind=False)

    for r in rows_src:
        t = r['t']
        rec = {'t': t, 'frame': r['frame'], 'state': mission.state,
               'e_raw': None, 'e_f': None, 'psi_target': None,
               'surge': 0.0, 'sway': 0.0, 'w_ratio': None, 'score': None}
        recs.append(rec)
        obs = vision.observe(r['dets'], r.get('img_w'), r.get('img_h'),
                             r.get('frame'))
        if obs is not None:
            rec['e_raw'] = obs['e_raw']
            rec['w_ratio'] = obs['w_ratio']
            rec['score'] = obs['score']
        mission.tick(now=t, obs=obs)
        rec['state'] = mission.state
        rec['e_f'] = getattr(mission, '_last_e', None)
    return recs


def summarize(name, recs):
    n = len(recs)
    door = [r for r in recs if r['e_raw'] is not None]
    ef = [r['e_f'] for r in recs if r['e_f'] is not None]
    dpsi = [r['psi_target'] for r in recs if r['psi_target'] is not None]
    sway = [abs(r['sway']) for r in recs if r['sway'] is not None]
    dead = sum(1 for v in ef if abs(v) < GC.GATE_E_DEAD)
    print('\n===== %s =====' % name)
    print('总帧 %d, 见门 %d (%.0f%%)' % (n, len(door), 100.0 * len(door) / n))
    if door:
        es = [r['e_raw'] for r in door]
        wr = [r['w_ratio'] for r in door]
        print('e_raw 范围 [%.3f, %.3f], 门宽占比 [%.2f, %.2f]'
              % (min(es), max(es), min(wr), max(wr)))
    if ef:
        print('e_f: |e|<死区 拍数 %d/%d (%.0f%%)' % (dead, len(ef), 100.0 * dead / len(ef)))
    if dpsi:
        print('psi_target(相对修正): 幅值 max %.2f° mean %.2f°'
              % (max(abs(v) for v in dpsi),
                 sum(abs(v) for v in dpsi) / len(dpsi)))
    if sway:
        print('|sway| max %.3f' % max(sway))
    from collections import Counter
    print('状态分布:', dict(Counter(r['state'] for r in recs)))


def plot_all(recs_none, recs_comp, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    t0 = [r['t'] for r in recs_none]
    e_raw = [r['e_raw'] if r['e_raw'] is not None else float('nan') for r in recs_none]
    e_none = [r['e_f'] if r['e_f'] is not None else float('nan') for r in recs_none]
    e_comp = [r['e_f'] if r['e_f'] is not None else float('nan') for r in recs_comp]

    ax = axes[0]
    ax.plot(t0, e_raw, color='#bbbbbb', lw=0.7, label='e_raw (检测原始)')
    ax.plot(t0, e_none, color='#185FA5', lw=1.2, label='e_f (直通→PID 实际输入)')
    ax.plot(t0, e_comp, color='#D85A30', lw=1.0, alpha=0.8, label='e_f (complementary 参考)')
    ax.axhline(GC.GATE_E_DEAD, color='#639922', ls='--', lw=0.8)
    ax.axhline(-GC.GATE_E_DEAD, color='#639922', ls='--', lw=0.8, label='对准死区 ±%.2f' % GC.GATE_E_DEAD)
    ax.set_ylabel('e')
    ax.set_title('真机视频开环回放: 过门对准算法输出 (机体不动, 开环)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot([r['t'] for r in recs_none],
            [r['psi_target'] if r['psi_target'] is not None else float('nan') for r in recs_none],
            color='#185FA5', lw=1.2, label='Δψ 位置式PD输出(相对修正,°)')
    ax.axhline(0, color='#888888', lw=0.6)
    ax.set_ylabel('dpsi (deg)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot([r['t'] for r in recs_none],
            [r['sway'] if r['sway'] is not None else float('nan') for r in recs_none],
            color='#D85A30', lw=1.2, label='sway')
    ax.axhline(0, color='#888888', lw=0.6)
    ax.set_ylabel('sway (-1~1)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[3]
    state_color = {'WAIT': '#B4B2A9', 'ALIGN': '#97C459', 'BLIND': '#EF9F27', 'DONE': '#888888'}
    for i in range(len(recs_none) - 1):
        st = recs_none[i]['state']
        ax.fill_between([recs_none[i]['t'], recs_none[i + 1]['t']], 0, 1,
                        color=state_color.get(st, '#ffffff'), linewidth=0)
    ax.set_yticks([])
    ax.set_xlabel('t (s)')
    ax.set_title('状态迁移 (灰=等门 绿=对准)', fontsize=10)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in state_color.values()]
    ax.legend(handles, state_color.keys(), loc='upper right', fontsize=8, ncol=4)

    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print('[OK] 图: %s' % out_png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dets', default=DEFAULT_DETS)
    ap.add_argument('--mode', default=None, choices=['none', 'complementary'],
                    help='只跑单模式; 缺省两遍都跑')
    ap.add_argument('--out', default=os.path.join(HERE, 'replay_out'))
    args = ap.parse_args()

    rows = []
    with open(args.dets, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    print('[i] dets 帧数: %d' % len(rows))

    os.makedirs(args.out, exist_ok=True)
    modes = [args.mode] if args.mode else ['none', 'complementary']
    results = {}
    for m in modes:
        recs = run_pass(m, rows, args)
        results[m] = recs
        summarize('mode=%s' % m, recs)
        csv_path = os.path.join(args.out, 'gate_replay_%s.csv' % m)
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'frame', 'state', 'e_raw',
                                              'e_f', 'psi_target', 'surge',
                                              'sway', 'w_ratio', 'score'])
            w.writeheader()
            w.writerows(recs)
        print('[OK] csv: %s' % csv_path)

    if len(results) == 2:
        png = os.path.join(args.out, 'gate_replay.png')
        plot_all(results['none'], results['complementary'], png)
    elif args.mode == 'none':
        png = os.path.join(args.out, 'gate_replay.png')
        plot_all(results['none'], results['none'], png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
