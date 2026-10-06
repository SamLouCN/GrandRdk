#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""depth_kalman 入口：50 Hz 循环，事件驱动观测更新，原子写 momo_depth.json。

独立运行，**不接入 GrandRDK/run.sh、不接入 To32**。

本模块属于生产代码：只 import 宿主的 ``config/``（由 PYTHONPATH 给出，见 run.sh）
与同目录的 ``*.py``，**不引用 tests/ 里的任何东西**（合成真值、mock 喂数都在那边）。

⚠ 2026-10-01 目录归位：源码从 ``<包根>/src/`` **平铺到包根**（与 run.sh 同级），
配置从 ``<包根>/config/`` 收进宿主的 ``config/``。所以 ``ROOT`` 只取一层 dirname，
（老代码取两层，平铺后会指到 ``src/kalman/`` 去，logs 落错地方）。

用法示例::

    ./run.sh                          # 前台跑，读 /dev/shm
    ./run.sh --dry-run                # 只算不落盘
    ./run.sh --shm-dir /tmp/dk_mock   # 换数据目录（离线联调）
    ./run.sh --selftest               # 合成真值自检，不碰硬件（脚本在 tests/）
    ./run.sh --channels B --h-mode rw --no-ba
"""
import argparse
import os
import signal
import sys
import time

import cfgutil
from fusion import DepthFusion
from sinks import Log, atomic_write_json
from sources import ShmReader

ROOT = os.path.dirname(os.path.abspath(__file__))
_STOP = {'flag': False}


def _on_signal(signum, frame):
    _STOP['flag'] = True


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog='depth_kalman',
        description='深度卡尔曼滤波（深度计 + 两路朝下高度计 + 三轴加速度）')
    p.add_argument('--shm-dir', default=None, help='共享内存目录，默认取配置 SHM_DIR')
    p.add_argument('--channels', default=None,
                   help="高度计通道白名单，逗号分隔，如 'B,C'；空串 = 不用高度计")
    p.add_argument('--anchor', default=None, help='锚路通道，默认取白名单第一个')
    p.add_argument('--h-mode', default=None, choices=['known', 'rw'], help='池底深度模式')
    p.add_argument('--h-m', type=float, default=None, help='池底深度实测值（m）')
    p.add_argument('--depth-zero', type=float, default=None, help='深度计水面零点标定值（m）')
    p.add_argument('--accel-unit', default=None, choices=['unknown', 'm/s^2', 'g'],
                   help='加速度单位（标定后填）；unknown 时加速度自动关闭')
    p.add_argument('--accel-on', action='store_true',
                   help='强制启用加速度；单位仍未知则报错退出（不许猜）')
    p.add_argument('--accel-off', action='store_true', help='关闭加速度（强制）')
    p.add_argument('--no-ba', action='store_true', help='关闭加速度零偏状态 b_a')
    p.add_argument('--loop-hz', type=float, default=None, help='滤波循环频率')
    p.add_argument('--dry-run', action='store_true', help='不写 momo_depth.json')
    p.add_argument('--once', action='store_true', help='只跑一拍后退出（冒烟测试）')
    p.add_argument('--duration', type=float, default=None, help='运行秒数后退出')
    p.add_argument('--print-every', type=float, default=1.0, help='状态行打印间隔（秒），0=不打印')
    p.add_argument('--log-file', default=None, help='日志文件，默认 logs/depth.log')
    p.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARN', 'ERROR'])
    p.add_argument('--quiet', action='store_true', help='只写日志文件，不输出到终端')
    return p.parse_args(argv)


def build_cfg(args):
    ov = {'SHM_DIR': args.shm_dir, 'H_MODE': args.h_mode, 'H_M': args.h_m,
          'DEPTH_ZERO_OFFSET': args.depth_zero, 'ACCEL_UNIT': args.accel_unit,
          'LOOP_HZ': args.loop_hz, 'ALT_ANCHOR': args.anchor}
    if args.channels is not None:
        ov['ALT_CHANNELS'] = [c for c in args.channels.replace(' ', '').split(',') if c]
    if args.accel_off:
        ov['ACCEL_ENABLED'] = False
    if args.no_ba:
        ov['ENABLE_BA'] = False
    return cfgutil.load(**ov)


def check_units(cfg, args):
    """单位未知时的策略：**默认自动关掉加速度**；只有显式 --accel-on 才报错退出。"""
    if cfg.ACCEL_UNIT_KNOWN:
        return 0
    if args.accel_on:
        sys.stderr.write(
            '错误：ACCEL_UNIT=%r 未标定，拒绝启用加速度。\n'
            '      先静置读遥测 acc_z 的 raw 值：≈981 → m/s^2；≈100 → g。\n'
            '      定好后写进 config/depth_config.py 的 ACCEL_UNIT，或用 --accel-unit 指定。\n'
            % cfg.ACCEL_UNIT)
        return 3
    return 0


def fmt_status(snap):
    c = snap['counters']
    clr = ' '.join('%s=%.2f' % (k, v) for k, v in sorted(snap['clearance'].items())) or '-'
    alt_age = ' '.join('%s%s' % (k, ('%.2f' % v) if v is not None else 'NA')
                       for k, v in sorted(snap['sources']['alt_age'].items())) or '-'
    tel_age = snap['sources']['telem_age']
    bd = snap.get('b_d')
    ba = snap.get('b_a')
    return ('D=%.3f(σ%.3f) v=%+.3f b_d=%s b_a=%s H=%.3f | 净空 %s | 观测 d%d/a%d'
            ' | 剔除 d%d/a%d 饱和%d 重置%d | 源 tel%s alt[%s] | 降级=%s'
            % (snap['D'], snap['sigma'].get('D', 0.0), snap['v_z'],
               ('%+.4f' % bd) if bd is not None else '-',
               ('%+.4f' % ba) if ba is not None else '-',
               snap['H'], clr,
               c['depth'], c['alt'], c['rej_depth'], c['rej_alt'], c['sat'], c['reset'],
               ('%.2f' % tel_age) if tel_age is not None else 'NA', alt_age,
               '是' if snap['flags']['degraded'] else '否'))


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    cfg = build_cfg(args)
    rc = check_units(cfg, args)
    if rc:
        return rc

    log_path = args.log_file
    if log_path is None and not args.quiet:
        log_path = os.path.join(ROOT, 'logs', 'depth.log')
    log = Log(path=log_path, level=args.log_level,
              also_stdout=True, max_bytes=cfg.LOG_MAX_BYTES, keep=cfg.LOG_ROTATE_KEEP)

    log.info('=' * 68)
    log.info('depth_kalman 启动 pid=%d | 根目录 %s' % (os.getpid(), ROOT))
    log.info('配置：' + cfgutil.explain(cfg))
    log.info('数据目录 %s（只读 %s / %s，只写 %s）'
             % (cfg.SHM_DIR, cfg.SHM_TELEM, cfg.SHM_ALT, cfg.SHM_DEPTH))
    if not cfg.ACCEL_UNIT_KNOWN:
        log.warn('加速度单位未标定（ACCEL_UNIT=%s）→ **已自动关闭加速度**，退化为'
                 '「深度计 + 高度计」两源融合。标定后用 --accel-unit 打开。' % cfg.ACCEL_UNIT)
    if not cfg.ALT_CHANNELS:
        log.warn('ALT_CHANNELS 为空 → 纯「深度计 +（无）加速度」模式，H 不可观（自动冻结）。')

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    reader = ShmReader(cfg, log=log)
    fusion = DepthFusion(cfg, log=log)
    out_path = os.path.join(cfg.SHM_DIR, cfg.SHM_DEPTH)

    period = 1.0 / max(1e-3, float(cfg.LOOP_HZ))
    out_period = 0.0 if args.dry_run else 1.0 / max(1e-3, float(cfg.OUT_HZ))
    t_stop = (time.time() + args.duration) if args.duration else None

    t_next = time.time()
    t_last_log = 0.0
    t_last_out = 0.0
    n_out = 0
    t_boot = time.time()

    while not _STOP['flag']:
        try:
            now = time.time()
            tel = reader.read_telemetry()
            alt = reader.read_alt()
            snap = fusion.step(now, tel, alt)

            if out_period and (now - t_last_out) >= out_period:
                if atomic_write_json(out_path, snap):
                    t_last_out = now
                    n_out += 1
                else:
                    log.error('写 %s 失败' % out_path)

            if args.print_every and (now - t_last_log) >= args.print_every:
                t_last_log = now
                log.info(fmt_status(snap))

            if args.once:
                break
            if t_stop is not None and now >= t_stop:
                break

            t_next += period
            sleep = t_next - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.time()      # 落后了就重新对齐，不追帧
        except KeyboardInterrupt:
            break

    log.info('退出：运行 %.1f s，落盘 %d 次，观测 深度%d/高度计%d，剔除 %d/%d，'
             '饱和 %d，重置 %d'
             % (time.time() - t_boot, n_out,
                fusion.cnt['depth'], fusion.cnt['alt'],
                fusion.cnt['rej_depth'], fusion.cnt['rej_alt'],
                fusion.cnt['sat'], fusion.cnt['reset']))
    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
