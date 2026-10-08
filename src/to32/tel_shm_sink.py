# -*- coding: utf-8 -*-
"""tel_shm_sink.py —— 下位机遥测落盘（写 /dev/shm/momo_telemetry.json）

[2026-10-08 接回] depth_kalman / viskf 等观测工程都以 momo_telemetry.json 为
遥测输入源（sources.py 明确"等待 To32 侧写入"），但 v2.5 重写时落盘端丢了。
本模块在 mode_dispatcher.on_stm32_telemetry 每帧回调里写一次，所有模式通用。

口径（★ 与 depth_kalman 的模型约定对齐，写 raw 协议值，不是换算后的物理量）：
    depth_raw = actual_depth_cm × 100      # cm×100，depth_config.DEPTH_SCALE=1e-4 → m
    pitch/roll/yaw = actual_* × 100        # deg×100，ATT_RAW_IS_DEGX100=True
    acc_x/y/z = tel 值 × 100               # 还原 int16，ACCEL_RAW_SCALE=0.01 由读端再乘
文件原子写（tmp + os.replace），20Hz 节流；任何异常吞掉绝不影响主链路。
"""
import json
import os
import time


class TelemetryShmSink(object):

    def __init__(self, path='/dev/shm/momo_telemetry.json', min_interval=0.05, log=None):
        self.path = str(path)
        self.min_interval = float(min_interval)
        self.log = log
        self._last_ts = 0.0
        self._warned = False

    def _warn(self, msg):
        if not self._warned and self.log:
            self._warned = True
            try:
                self.log('[tel_shm_sink] ' + msg)
            except Exception:
                pass

    def write(self, tel):
        """tel = link_stm32.parse_telemetry 的输出 dict。任何异常都不外抛。"""
        try:
            if not isinstance(tel, dict) or not tel:
                return
            now = time.time()
            if now - self._last_ts < self.min_interval:
                return
            self._last_ts = now
            obj = {'ts': now}
            adc = tel.get('actual_depth_cm')
            if isinstance(adc, (int, float)):
                obj['depth_raw'] = int(round(float(adc) * 100.0))
            for k_out, k_in in (('pitch', 'actual_pitch'), ('roll', 'actual_roll'),
                                ('yaw', 'actual_yaw')):
                v = tel.get(k_in)
                if isinstance(v, (int, float)):
                    obj[k_out] = int(round(float(v) * 100.0))
            for k in ('acc_x', 'acc_y', 'acc_z'):
                v = tel.get(k)
                if isinstance(v, (int, float)):
                    obj[k] = int(round(float(v) * 100.0))
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(obj, f)
            os.replace(tmp, self.path)
        except Exception as exc:
            self._warn('写遥测落盘失败（已忽略）: %r' % (exc,))


def make_sink(log=None):
    """dispatcher 用：默认路径构造，失败返回哑 sink（绝不拖垮主链路）。"""
    try:
        return TelemetryShmSink(log=log)
    except Exception as exc:
        if log:
            try:
                log('[tel_shm_sink] 初始化失败，遥测不落盘: %r' % (exc,))
            except Exception:
                pass

        class _Null(object):
            def write(self, tel):
                pass
        return _Null()
