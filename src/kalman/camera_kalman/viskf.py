#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""viskf.py — 视觉穿门卡尔曼滤波（image-based servo KF）

方案: 《视觉穿门卡尔曼滤波方案.md》v2（2026-09-23）。
依赖: 纯 Python 标准库（无 numpy / cv2），Python 3.8+。

部署（同一份代码，两种放法都支持，靠配置文件自动识别）:
    A 独立子工程（推荐）: /userdata/kalman/camera_kalman/src/viskf.py
      → 配置读 <工程根>/config/viskf_config.py，与 GrandRDK 完全解耦
    B 并入 GrandRDK: /userdata/GrandRDK/src/viskf.py
      → 配置读 <工程根>/config/quick_config.py 的 VISKF_* 段（控制侧同工程）
两者都是独立小进程，不进 run.sh，手动启停。

定位: **本文件 = 纯生产代码**（滤波 + 数据源 + 输出 + 主循环），可直接 import 并入更大的工程。
      自检 / 离线联调 / 合成几何在 tests/test_viskf.py，不在本文件 —— 别把测试捎进集成。

输入（只读共享内存，绝不碰相机 / 串口）:
    /dev/shm/momo_det_front.json  front.py 已在写:
        {"frame": int, "ts": float,
         "dets": [{"label","score","bbox":[x1,y1,x2,y2],"center":[cx,cy]}]}
    /dev/shm/momo_telemetry.json  中位机 shm_sink 写（本方案一并交付）:
        {"ts": float, "actual_pitch": deg, "actual_roll": deg, ...}
        （字段名 = link_stm32.parse_telemetry 的输出键）

输出:
    /dev/shm/momo_viskf.json  原子写（tmp + os.replace）:
        {"ts","frame","track","trust","gate_visible","age","flags{...}","outliers","reinits",
         "att{pitch,roll}","e_x","de_x","sig_x","el","del","sig_el","s_n","ds_n","sig_s",
         "obs{cx,cy,w,h,x1,y1,x2,y2,score,clip,clip_l,clip_r,clip_t,clip_b}"}
    logs/viskf.log            运行日志

    trust = 这个数能不能信（track 且 age≤trust_age_s 且 sig_x≤max_sig_x）——控制侧应看它，
            不要只看 gate_visible：门离场 >2s 时 gate_visible 已经 False，但 e_x 仍在被
            自由外推（实测能冲到 +2.38，正常跟踪时上限只有 +0.60）。
    age   = 距最近一次有效观测的秒数（无观测时持续增长）

模型: **三个独立的 2 维匀速（CV）线性卡尔曼**，非线性全部前置在 measure() 里做完
      （不是 EKF：滤波器内部 F/H 都是常数矩阵，不需要雅可比，也不会线性化发散；
       代价是线性化误差被折进观测噪声 R）
    x 通道 [e_x, de_x]    e_x=(dx_lvl-dx0)/w_bbox   滚转像面反旋转(免 f), 无量纲距离不变 → 偏航
    y 通道 [el,  del ]    el=atan2(-dy_lvl,f)+pitch  水平系仰角(rad), 俯仰补偿(需 f) → 到位确认
    尺寸通道 [s_n, ds_n]  s_n=sqrt(w*h)/W_img        √S 归一化(免标定) → 前向

鲁棒性: score 门限 + 新息 nσ 门限(带附加底噪) + 连续 N 帧拒绝→重置(重捕) + 0.5s 滑行
        + 2s 丢失 + 重捕首帧跳门限 + trust 可信位。
出框(裁切)分边处理: **口径恒定，只用 R 表达质量**。左右裁 → e_x 的 σ 放大 clip_r_mult 倍；
        上下裁 → el/s_n 的 σ 放大；两者互不牵连。**不再**切换 e_x 归一化口径、
        也**不再**用高宽比反推被裁的尺寸 —— 两个方案都在真机数据上实测过，都是负收益
        （口径切换使观测跳变中位 0.209 vs 正常 0.004；反推法的误差比它要修的偏差还大）。
        详见 measure() 的 docstring 与 D:/RC/Test_any/README_测试报告.md §5.1/§5.4。
姿态降级链: 遥测新鲜(<stale_s)全补偿 / stale~degrade_s 用保持值 / 超时当水平(att_degraded)。
符号约定(PITCH_SIGN/ROLL_SIGN)是 E8 类坑: 上真机前必须 P3 手持实验核对(见方案 §7)。

用法（生产）:
    python3 src/viskf.py --status            # 打印生效配置与数据源新鲜度后退出
    nohup python3 src/viskf.py [选项] &       # 真机(读 /dev/shm)

用法（自检 / 离线联调，在 tests/ 里）:
    python3 tests/test_viskf.py --selftest   # 无硬件自检(合成几何+噪声), 改完参数的第一道闸
    python3 tests/test_viskf.py --mock       # 离线端到端(合成数据走一遍文件读写链路)

嵌入更大的工程:
    import viskf
    cfg  = viskf.load_cfg()                      # 读配置文件（缺省静默回落到内置值）
    core = viskf.VisKF(cfg)                      # 纯计算，无 I/O
    out  = core.step(now, obs, att)              # 单步推进，返回 dict
    #  或者直接跑数据循环：  viskf.run(cfg)

配置优先级: 内置 DEFAULTS < 配置文件的 VISKF_<键名> < 命令行参数。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

__all__ = ['DEFAULTS', 'load_cfg', 'load_config_source', 'read_json_shm', 'CVFilter',
           'DetSource', 'AttitudeSource', 'VisKF', 'JsonSink', 'LogSink',
           'run', 'cmd_status', 'main']

# ============================================================
# 配置
# ============================================================
# 内置缺省（quick_config.VISKF_<NAME> 可覆盖；🔴 = 上真机前必须实测/标定）
DEFAULTS = {
    # ---- 数据源 / 输出 ----
    'enabled':        True,
    'shm_dir':        '/dev/shm',
    'det_path':       '/dev/shm/momo_det_front.json',
    'tel_path':       '/dev/shm/momo_telemetry.json',
    'out_path':       '/dev/shm/momo_viskf.json',
    'log_dir':        '',                # 空 = <工程根>/logs（src/viskf.py 的上一级）
    'det_stale_s':    0.5,               # det 文件 mtime 超时（写端挂了就当没数据）
    'tel_stale_s':    1.0,               # 姿态全补偿窗口
    'tel_degrade_s':  3.0,               # 超过则当水平(不补偿)
    # ---- 相机几何 ----
    'w_img':          640,
    'h_img':          480,
    'x0_px':          320.0,             # 主点（🔴 有偏差就标）
    'y0_px':          240.0,
    'dx0_px':         0.0,               # 期望门中心相对主点的横向偏移（默认 0=对准光轴）
    'focal_px':       554.0,             # 🔴 与 RANGE_FOCAL_PX 同源占位值，一次标定两边用
    # ---- 检测筛选 ----
    'target_label':   'door',
    'min_score':      0.5,               # 🔴 按实测检测质量调
    'clip_margin_px': 2.0,               # bbox 距画面边缘小于此值判"该边贴边"
    'clip_r_mult':    4.0,               # 贴边帧观测方差放大倍数（分边放大，见 step）
    # ---- 姿态补偿 ----
    'att_enable':     True,
    'pitch_sign':     1.0,               # 🔴 P3 手持实验定（翻号看方案 §7）
    'roll_sign':      1.0,               # 🔴
    'att_sigma':      0.01,              # 姿态σ(rad)，进 el 观测方差
    # ---- 观测噪声 σ ----
    # 2026-09-23 真机视频实测回填（排除出框帧；见 D:/RC/Test_any/README_测试报告.md §3.2）：
    #   e_x 0.0177（原占位 0.02 基本对）/ el 0.0036（占位 0.005 保守 1.4x）/ s_n 0.0043（保守 2.3x）
    # 仍建议真机静止对门录 30s 复核一遍再闭环。
    'sig_ex':         0.0177,            # e_x（占门宽比例）
    'sig_el':         0.0036,            # el (rad)
    'sig_s':          0.0043,            # s_n
    # ---- 过程噪声密度（连续白噪声加速度模型，每通道）----
    'q_ex':           0.05,
    'q_el':           0.02,
    'q_s':            0.10,              # 接近段 ṡ_n 大，尺寸通道 Q 偏大
    # ---- 门限 / 丢失 ----
    'gate_nsigma':    3.0,               # 新息门限（|y|/√S 超过则拒）
    'gate_reset_n':   5,                 # 连续拒绝 N 帧 → 该通道重置(重捕)
    'coast_s':        0.5,               # 无观测超时 → coasting（纯预测滑行）
    'lost_s':         2.0,               # 无观测超时 → lost（重捕后整体重置）
    # 外推防护（2026-09-23 实测：门离开视野 >2s 时 e_x 会被自由外推到 +2.38、
    # sig_x 到 2.40，而正常跟踪时上限只有 0.50 / +0.60。控制侧不能只看 gate_visible）
    'reacquire_skip_gate': True,         # coast 后的重捕首帧不做新息门限（此刻状态比观测更不可信）
    'trust_age_s':    0.2,               # 距最近有效观测超过此值 → trust=False
    'max_sig_x':      1.0,               # sig_x 超此值 → trust=False（track 时 ≤0.50，外推可到 2.31）
    # 新息门限的附加底噪（模型误差：目标机动 / 线性化误差 / 姿态补偿残差）。
    # 这三个量都会进新息，但**不在 R 里** —— 实测跟踪残差是观测 σ 的 1.5~3.5 倍。
    # 取 σ 同量级 = 门限放宽 √2 倍，够吸收模型误差又不放野点进来。
    'gate_floor_x':   0.018,             # ≈ sig_ex（0.0177）
    'gate_floor_el':  0.0036,            # ≈ sig_el
    'gate_floor_s':   0.0043,            # ≈ sig_s
    # ---- 节奏 ----
    'loop_hz':        50.0,
    'out_hz':         20.0,
    'log_every_s':    1.0,
    'print_hz':       2.0,
    'dt_min':         1e-4,
    'dt_max':         0.5,
}


# 配置文件搜索顺序（命中即用；键名前缀固定 VISKF_）:
#   1. load_cfg(config=...) 显式给的路径 / 命令行 --config
#   2. 环境变量 VISKF_CONFIG
#   3. <工程根>/config/viskf_config.py         独立部署时（自己带 config/）
#   4. <宿主根>/config/viskf_config.py         ★ 并入 GrandRDK 后的实际位置
#   5. 同上但叫 quick_config.py               （取其 VISKF_* 段）
# [2026-10-01] 本工程并入 GrandRDK 后按它的分类归档：
#   viskf.py 平铺在 src/kalman/camera_kalman/，配置收进 <宿主>/config/。
# 所以这里必须**逐级向上**找 config/，而不是只认 <工程根>/config ——
# 向上找（最多 4 级）而不是写死 ../../../，是为了整包拷出去独立跑时仍然成立。
# 都找不到就用内置 DEFAULTS（不报错，照跑；--status 会显示"未找到配置文件"）。
CONFIG_NAMES = ('viskf_config.py', 'quick_config.py')
CONFIG_PREFIX = 'VISKF_'
CONFIG_UPLEVELS = 4          # 从工程根往上找几级 config/


def _exec_config_file(path):
    """执行一个配置 .py 并把其中的大写常量取出来。

    **刻意不用 import**：配置模块是纯常量表，直接 exec 能精确指定文件，
    不会因为 sys.path 里恰好还有别的 quick_config.py（GrandRDK 的、以及
    PYTHONPATH 带进来的）而静默取错 —— 那种"改了参数却没生效"最难查。
    注意 quick_config.py 并非纯常量（它 `from camera_ports import device_for`），
    故执行期间把它所在目录临时挂进 sys.path，之后再摘掉。
    """
    d = os.path.dirname(os.path.abspath(path))
    ns = {'__file__': path, '__name__': os.path.splitext(os.path.basename(path))[0]}
    with open(path, 'r', encoding='utf-8') as f:
        code = f.read()
    added = d not in sys.path
    if added:
        sys.path.insert(0, d)
    try:
        exec(compile(code, path, 'exec'), ns)
    finally:
        if added:
            try:
                sys.path.remove(d)
            except ValueError:
                pass
    return {k: v for k, v in ns.items() if k.isupper() and not k.startswith('_')}


def project_root():
    """本工程根 = viskf.py 所在目录

    ⚠ 2026-10-01 起 viskf.py **平铺**在工程根下（原来是 `<工程根>/src/viskf.py`），
    所以这里取一层 dirname 就够；写两层会指到 `src/kalman/`，配置和 logs 全错位。
    """
    return os.path.dirname(os.path.abspath(__file__))


def config_dir_candidates():
    """配置目录候选：工程自己的 config/ 优先，然后逐级向上找宿主的 config/"""
    out = []
    up = project_root()
    for _ in range(CONFIG_UPLEVELS + 1):
        out.append(os.path.join(up, 'config'))
        up = os.path.dirname(up)
    return out


def load_config_source(explicit=None):
    """按搜索顺序返回 (实际用到的配置文件路径 or None, 其常量字典)。"""
    cands = [explicit, os.environ.get('VISKF_CONFIG')]
    cands += [os.path.join(d, n) for d in config_dir_candidates() for n in CONFIG_NAMES]
    for p in cands:
        if p and os.path.isfile(p):
            try:
                return p, _exec_config_file(p)
            except Exception as e:
                # 不静默：配置文件存在却读不了（语法错、缺依赖）必须让人看见
                sys.stderr.write('[viskf] 警告: 配置文件 %s 加载失败（%s: %s），'
                                 '改用下一个候选\n' % (p, type(e).__name__, e))
                continue
    return None, {}


def load_cfg(cli=None, config=None):
    """DEFAULTS < 配置文件的 VISKF_<键名> < cli 覆盖。无配置文件则静默用内置缺省。"""
    cfg = dict(DEFAULTS)
    path, consts = load_config_source(config)
    for k in cfg:
        v = consts.get(CONFIG_PREFIX + k.upper())
        if v is not None:
            cfg[k] = v
    if not cfg.get('log_dir'):
        cfg['log_dir'] = os.path.join(project_root(), 'logs')
    for k, v in (cli or {}).items():
        if v is not None:
            cfg[k] = v
    cfg['config_path'] = path or ''       # 供 --status 显示来源；空 = 用了内置缺省
    return cfg


# ============================================================
# 共享内存 JSON 读取（兼容 front 的定长空格填充 mmap 格式）
# ============================================================
def read_json_shm(path):
    """读共享内存里的 JSON。兼容 front 写端"定长 + 空格/\\0 填充"的做法。

    任何异常都返回 None 而不是抛：**读端永远不许因为上游脏数据把自己搞崩**，
    脏帧按"这一拍没数据"处理，交给上层的超时/滑行逻辑去降级。
    """
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, 'rb') as f:
            raw = f.read()
    except OSError:
        return None
    raw = raw.strip(b' \x00\r\n\t')
    if not raw:
        return None
    try:
        obj = json.loads(raw.decode('utf-8', 'replace'))
    except (ValueError, UnicodeDecodeError):
        return None
    return obj if isinstance(obj, dict) else None


def _mtime(path):
    """取 mtime，文件不在/不可读时返回 0.0（0 会被上层当成"无穷老"，即不可用）"""
    try:
        return float(os.path.getmtime(path))
    except OSError:
        return 0.0


# ============================================================
# 2 维匀速卡尔曼（纯 Python 实现）
# ============================================================
class CVFilter(object):
    """x=[z, zdot]; 观测 z（标量）。Q 用连续白噪声加速度模型（密度 q）。"""

    __slots__ = ('x', 'p', 'q')

    def __init__(self, q, z0=0.0, sig0=1.0):
        """q = 连续白噪声加速度密度（越大越信观测、跟得越快）；sig0 = 初值不确定度"""
        self.x = [float(z0), 0.0]
        self.p = [[sig0 * sig0, 0.0], [0.0, 1.0]]
        self.q = float(q)

    def predict(self, dt):
        """时间更新：x ← F x，P ← F P Fᵀ + Q（Q 为连续白噪声加速度的离散化形式）"""
        x0, x1 = self.x
        p00, p01 = self.p[0]
        p10, p11 = self.p[1]
        self.x = [x0 + dt * x1, x1]
        # F P F^T
        a = p00 + dt * (p01 + p10) + dt * dt * p11
        b = p01 + dt * p11
        c = p10 + dt * p11
        # Q = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]
        qa = self.q * dt * dt * dt / 3.0
        qb = self.q * dt * dt / 2.0
        qc = self.q * dt
        self.p = [[a + qa, b + qb], [c + qb, p11 + qc]]

    def update(self, z, r_var, n_sigma=None, gate_floor=0.0):
        """带新息门限的更新。返回 (accepted, y, S)。门限判定在改状态之前。

        gate_floor 是新息门限的**附加底噪**（模型误差，不是观测噪声）：
        门限判据用 ``|y| > n_sigma · √(S + gate_floor²)``。
        为什么需要它：实测跟踪残差是观测 σ 的 1.5~3.5 倍 —— 多出来的部分来自
        目标机动、线性化误差、姿态补偿残差，它们都会进新息，但**不在 R 里**。
        只按 √S 定门限会让好观测被周期性误拒（实测 σ 回填后拒收从 20 涨到 35 拍）。
        同一条经验在 depth_kalman 里也踩过（门限用 √(EWMA(ν²)+S+FLOOR²)）。
        """
        p00 = self.p[0][0]
        s = p00 + r_var
        y = z - self.x[0]
        if s <= 1e-12:
            return False, y, s
        if n_sigma is not None:
            thr = n_sigma * math.sqrt(s + gate_floor * gate_floor)
            if abs(y) > thr:
                return False, y, s
        p01 = self.p[0][1]
        p10 = self.p[1][0]
        p11 = self.p[1][1]
        k0 = p00 / s
        k1 = p10 / s
        x0, x1 = self.x
        self.x = [x0 + k0 * y, x1 + k1 * y]
        # Joseph 形式: A = I - K H = [[1-k0, 0], [-k1, 1]]; P' = A P A^T + K r K^T
        t00 = (1.0 - k0) * p00
        t01 = (1.0 - k0) * p01
        t10 = -k1 * p00 + p10
        t11 = -k1 * p01 + p11
        self.p = [[t00 * (1.0 - k0) + k0 * k0 * r_var,
                   -t00 * k1 + t01 + k0 * k1 * r_var],
                  [t10 * (1.0 - k0) + k1 * k0 * r_var,
                   -t10 * k1 + t11 + k1 * k1 * r_var]]
        return True, y, s

    def reset(self, z, sig0=0.5):
        """重捕/初始化：位置直接用观测，速度归零（不猜速度，避免重捕瞬间被外推带飞）"""
        self.x = [float(z), 0.0]
        self.p = [[sig0 * sig0, 0.0], [0.0, 1.0]]

    def sigma(self):
        """位置分量的 1σ —— 外部判"能不能信"就是看它（trust 的三条件之一）"""
        return math.sqrt(max(0.0, self.p[0][0]))


# ============================================================
# 数据源
# ============================================================
class DetSource(object):
    """momo_det_front.json → 最优 door 观测。frame 去重 + mtime 新鲜度。"""

    def __init__(self, cfg):
        """cfg 必须有 det_path / det_stale_s；last_frame 用于帧号去重"""
        self.cfg = cfg
        self.path = cfg['det_path']
        self.last_frame = None
        self.last_ok = 0.0

    def poll(self, now):
        """返回 (obs, has_new_frame)。obs=None 且 has_new=True 表示"新帧但没门"。"""
        obj = read_json_shm(self.path)
        if not obj:
            return None, False
        mt = _mtime(self.path)
        if mt <= 0 or (now - mt) > self.cfg['det_stale_s']:
            return None, False                     # 写端挂了/太旧，宁可当没数据
        frame = obj.get('frame')
        if frame is not None and frame == self.last_frame:
            return None, False                     # 同帧不重复更新
        self.last_frame = frame
        self.last_ok = mt
        best, best_score = None, -1.0
        for d in (obj.get('dets') or []):
            if not isinstance(d, dict):
                continue
            if d.get('label') != self.cfg['target_label']:
                continue
            sc = float(d.get('score', 0.0))
            if sc < self.cfg['min_score']:
                continue
            if sc > best_score:
                best, best_score = d, sc
        if best is None:
            return None, True
        bb = best.get('bbox') or [0, 0, 0, 0]
        try:
            x1, y1, x2, y2 = (float(bb[0]), float(bb[1]), float(bb[2]), float(bb[3]))
        except (TypeError, ValueError, IndexError):
            return None, True
        c = best.get('center')
        if c and len(c) >= 2:
            try:
                cx, cy = float(c[0]), float(c[1])
            except (TypeError, ValueError):
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        else:
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        m = self.cfg['clip_margin_px']
        w_img, h_img = self.cfg['w_img'], self.cfg['h_img']
        # 四边**分开**判：裁在哪条边决定哪个量不可信（方案 §6.4 / 测试报告 §5.1）
        #   左右裁 → bbox 宽度 w 不可信 → e_x 的归一化口径必须降级
        #   上下裁 → bbox 高度 h 与中心 cy 不可信 → s_n / el 需按门宽反推（见 measure）
        # 实测：669 个出框帧里 606 帧只是**下沿裁切**，此时 w 依然可靠 ——
        # 老实现"一裁就切 e_x 口径"是过度的，会在切换处造出 0.057 的观测跳变。
        clip_l = x1 <= m
        clip_r = x2 >= w_img - 1 - m
        clip_t = y1 <= m
        clip_b = y2 >= h_img - 1 - m
        obs = {'frame': frame if frame is not None else -1,
               'ts': float(obj.get('ts', now)),
               'cx': cx, 'cy': cy,
               'w': max(1.0, x2 - x1), 'h': max(1.0, y2 - y1),
               'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
               'score': best_score,
               'clip': clip_l or clip_r or clip_t or clip_b,   # 旧消费方语义：任一边裁
               'clip_l': clip_l, 'clip_r': clip_r,
               'clip_t': clip_t, 'clip_b': clip_b}
        return obs, True


class AttitudeSource(object):
    """momo_telemetry.json → (pitch, roll) rad + 新鲜度模式。

    返回 {'mode': 'ok'|'hold'|'degraded'|'none', 'pitch': rad, 'roll': rad}
    'none' = 文件从未出现（中位机 shm_sink 未部署/遥测 0 帧）→ 当水平跑。
    """
    KEYS = ('pitch_key', 'roll_key')
    DEFAULT_KEYS = {'pitch_key': 'actual_pitch', 'roll_key': 'actual_roll'}

    def __init__(self, cfg):
        """cfg 需含 tel_path / tel_stale_s / tel_degrade_s / pitch_sign / roll_sign"""
        self.cfg = cfg
        self.path = cfg['tel_path']
        self.pitch = 0.0
        self.roll = 0.0
        self.seen = False

    def poll(self, now):
        """读一次遥测。⚠ 遥测 0 帧（当前真机状态）时恒返回 mode='none' → 全程当水平跑，
        于是 `el` 失去姿态补偿 ⇒ **不可信**，下游只用 e_x / s_n，不用 el。
        """
        cfg = self.cfg
        obj = read_json_shm(self.path)
        mode = 'none'
        if obj is not None:
            mt = _mtime(self.path)
            age = (now - mt) if mt > 0 else 1e9
            pk = cfg.get('pitch_key', self.DEFAULT_KEYS['pitch_key'])
            rk = cfg.get('roll_key', self.DEFAULT_KEYS['roll_key'])
            try:
                p_deg = float(obj.get(pk, 0.0))
                r_deg = float(obj.get(rk, 0.0))
            except (TypeError, ValueError):
                p_deg = r_deg = 0.0
            k = math.pi / 180.0
            self.pitch = cfg['pitch_sign'] * p_deg * k
            self.roll = cfg['roll_sign'] * r_deg * k
            self.seen = True
            if age <= cfg['tel_stale_s']:
                mode = 'ok'
            elif age <= cfg['tel_degrade_s']:
                mode = 'hold'
            else:
                mode = 'degraded'
        elif self.seen:
            mode = 'degraded'                     # 文件消失也按降级
        return {'mode': mode, 'pitch': self.pitch, 'roll': self.roll}


# ============================================================
# 核心滤波器
# ============================================================
class VisKF(object):
    """三通道 CV KF + 姿态补偿 + 门限/重捕/滑行。step() 无副作用外的状态。"""

    def __init__(self, cfg):
        """三路各自独立的 CV 滤波器（**互不耦合**：没有任何交叉项，不是 EKF）"""
        self.cfg = cfg
        self.f_x = CVFilter(cfg['q_ex'])
        self.f_el = CVFilter(cfg['q_el'])
        self.f_s = CVFilter(cfg['q_s'])
        self._last_pred_t = None
        self.last_obs_t = None
        self._initialized = False
        self._rej = {'x': 0, 'el': 0, 's': 0}
        self.outliers = 0
        self.reinits = 0
        self.obs_debug = None
        self.n_clip_lr = 0               # 左右贴边计数
        self.n_clip_tb = 0               # 上下贴边计数
        self.n_reacquire = 0             # 重捕首帧（跳门限）次数

    # ---- 贴边边拆解（兼容只给一个 clip 布尔的老调用方）----
    @staticmethod
    def _clips(obs):
        """返回 (clip_l, clip_r, clip_t, clip_b)。老调用方只给综合 clip 时保守全真。"""
        if 'clip_l' in obs:
            return (bool(obs.get('clip_l')), bool(obs.get('clip_r')),
                    bool(obs.get('clip_t')), bool(obs.get('clip_b')))
        c = bool(obs.get('clip'))
        return c, c, c, c

    # ---- 观测构造（姿态补偿在此发生）----
    def measure(self, obs, att):
        """det 原始框 + 姿态 → (e_x, el, s_n, clip, att_used)。

        e_x : 滚转像面反旋转后 (dx-dx0)/w —— **恒定口径，永不切换**
        el  : 水平系仰角 = atan2(-dy_lvl, f) + pitch（rad，正=门在相机水平轴上方）
        s_n : sqrt(w*h)/w_img

        为什么不再"贴边就换 e_x 口径"（2026-09-24 真机数据实测后的决定）
        ------------------------------------------------------------
        老实现在 bbox 贴边时把 e_x 改成按**画面宽**归一化，与非贴边帧的
        "按门框宽归一化"相差 w_img/w_bbox ≈ 1.5~2.1 倍。实测后果：
          · 切换处观测跳变中位 **0.209**，而正常帧间变化只有 0.004（差 50 倍）
          · 三个通道的 3σ 新息门限被周期性击穿
          · 滤波输出的抖动被放大到观测的 3.9 倍（σ_filt 0.589 vs σ_obs 0.152）
        而实测贴边帧的**框宽其实基本可靠**：
          · 703 个下沿贴边帧的 w 与干净帧同级（h 被裁 ~10%，但 e_x 不用 h）
          · 71 个左右贴边帧的 w 中位 669 vs 干净帧 724，只小 7.6%
        所以正确做法不是"修尺寸"、更不是"换单位"，而是**保持口径恒定**，
        用放大的 R（见 step）来表达"这一帧质量差"——这是软信息，不是硬切换。
        试过的反方案：用门高宽比 h/w 反推被裁的边。实测干净帧自身的 h/w 波动
        (p10 0.704 ~ p90 0.804) 与要修的偏差(10%)同量级，净收益为负，已放弃。
        """
        cfg = self.cfg
        cx = float(obs['cx'])
        cy = float(obs['cy'])
        w = max(1.0, float(obs['w']))
        h = max(1.0, float(obs['h']))
        clip_l, clip_r, clip_t, clip_b = self._clips(obs)

        dx = cx - cfg['x0_px']
        dy = cy - cfg['y0_px']
        pitch = 0.0
        roll = 0.0
        att_used = False
        if att is not None and cfg['att_enable'] and att.get('mode') in ('ok', 'hold'):
            att_used = True
            pitch = float(att.get('pitch', 0.0))
            roll = float(att.get('roll', 0.0))
            c, s = math.cos(roll), math.sin(roll)
            dxl = dx * c - dy * s
            dyl = dx * s + dy * c
        else:
            dxl, dyl = dx, dy
        e_x = (dxl - cfg['dx0_px']) / w
        el = math.atan2(-dyl, float(cfg['focal_px'])) + pitch
        s_n = math.sqrt(w * h) / float(cfg['w_img'])
        return e_x, el, s_n, (clip_l or clip_r or clip_t or clip_b), att_used

    # ---- 单拍 ----
    def step(self, now, obs, att):
        """跑一拍：预测 →（有观测则）量测更新 → 组装输出字典。

        obs=None 表示这一拍没看到门：只做预测，靠 `coasting` / `lost` 让下游知道
        输出是**自由外推**的结果（外推越久 σ 越大，`trust` 会自己掉下去）。
        """
        cfg = self.cfg
        # 预测（dt 变步长）
        if self._last_pred_t is None:
            self._last_pred_t = now
        dt = now - self._last_pred_t
        if dt >= cfg['dt_min']:
            d = min(dt, cfg['dt_max'])
            self.f_x.predict(d)
            self.f_el.predict(d)
            self.f_s.predict(d)
            self._last_pred_t = now

        # 丢失判定（基于上一观测时刻）
        lost = (self.last_obs_t is None) or (now - self.last_obs_t) > cfg['lost_s']
        coasting = (self.last_obs_t is not None
                    and cfg['coast_s'] < (now - self.last_obs_t) <= cfg['lost_s'])

        clip = False
        att_used = False
        if obs is not None:
            e_x, el, s_n, clip, att_used = self.measure(obs, att)
            cl, cr, ct, cb = self._clips(obs)
            clip_lr = cl or cr
            clip_tb = ct or cb
            if lost or not self._initialized:
                # 首次启动初始化 / 丢失后重捕：整体重置到当前观测（换门场景的关键路径）
                # 首次不计入 reinits —— reinits 语义 = "重捕次数"，供控制器判断是否需要重新对准
                self.f_x.reset(e_x)
                self.f_el.reset(el)
                self.f_s.reset(s_n)
                self._rej = {'x': 0, 'el': 0, 's': 0}
                if self._initialized:
                    self.reinits += 1
                self._initialized = True
            else:
                # 观测方差只放大**真正受影响**的通道（老实现是"任一边贴边就三通道全放大"）：
                #   左右贴边 → 框宽略偏（实测小 7.6%）→ e_x 不确定
                #   上下贴边 → 框高被裁 ~10%、中心偏移 → s_n 与 el 不确定
                # e_x 不吃上下贴边（它只用 w），el 不吃左右贴边（无 roll 时只依赖 cy）。
                mult = cfg['clip_r_mult']
                mx = mult if clip_lr else 1.0
                ms = mult if clip_tb else 1.0
                me = mult if clip_tb else 1.0
                r_x = (cfg['sig_ex'] * mx) ** 2
                r_el = (cfg['sig_el'] * me) ** 2 + cfg['att_sigma'] ** 2
                r_s = (cfg['sig_s'] * ms) ** 2
                # coast（0.5~2s 无观测）之后的第一帧：此刻"状态"比"观测"更不可信
                # （它已经外推了整整一个 coast 周期），拿新息门限把观测拒掉是本末倒置。
                # 只跳过门限、仍然正常更新；真正的换门由 lost 路径整体重置负责。
                reacq = bool(cfg['reacquire_skip_gate']) and coasting
                if reacq:
                    self.n_reacquire += 1
                gate = None if reacq else cfg['gate_nsigma']
                for name, filt, z, r, gfl in (('x', self.f_x, e_x, r_x, cfg['gate_floor_x']),
                                             ('el', self.f_el, el, r_el, cfg['gate_floor_el']),
                                             ('s', self.f_s, s_n, r_s, cfg['gate_floor_s'])):
                    ok, _y, _s = filt.update(z, r, gate, gfl)
                    if ok:
                        self._rej[name] = 0
                    else:
                        self._rej[name] += 1
                        self.outliers += 1
                        if self._rej[name] >= cfg['gate_reset_n']:
                            filt.reset(z)
                            self._rej[name] = 0
                            self.reinits += 1
            if clip_lr:
                self.n_clip_lr += 1
            if clip_tb:
                self.n_clip_tb += 1
            self.last_obs_t = now
            self.obs_debug = {'cx': obs['cx'], 'cy': obs['cy'], 'w': obs['w'],
                              'h': obs['h'], 'score': obs['score'], 'clip': clip,
                              'clip_l': cl, 'clip_r': cr, 'clip_t': ct, 'clip_b': cb,
                              'y1': obs.get('y1'), 'y2': obs.get('y2')}
            lost = False
        elif self.obs_debug is not None:
            self.obs_debug = None

        am = att.get('mode', 'none') if att else 'none'
        age = None if self.last_obs_t is None else max(0.0, now - self.last_obs_t)
        cl, cr, ct, cb = self._clips(obs) if obs is not None else (False,) * 4
        out = {
            'ts': now,
            'frame': (obs['frame'] if obs is not None else None),
            'track': not lost and self.last_obs_t is not None,
            'gate_visible': obs is not None,
            'age': age,
            'flags': {
                'clip': clip,
                'clip_lr': bool(cl or cr),
                'clip_tb': bool(ct or cb),
                'coasting': coasting,
                'lost': lost,
                'att_ok': am == 'ok',
                'att_hold': am == 'hold',
                'att_degraded': am in ('degraded', 'none'),
            },
            'outliers': self.outliers,
            'reinits': self.reinits,
            'n_clip_lr': self.n_clip_lr,
            'n_clip_tb': self.n_clip_tb,
            'n_reacquire': self.n_reacquire,
            'att': {'pitch': float(att.get('pitch', 0.0)) if att else 0.0,
                    'roll': float(att.get('roll', 0.0)) if att else 0.0},
            'e_x': self.f_x.x[0], 'de_x': self.f_x.x[1], 'sig_x': self.f_x.sigma(),
            'el': self.f_el.x[0], 'del': self.f_el.x[1], 'sig_el': self.f_el.sigma(),
            's_n': self.f_s.x[0], 'ds_n': self.f_s.x[1], 'sig_s': self.f_s.sigma(),
        }
        # trust = "这个数现在能不能信"。控制侧应该看它，而不是只看 gate_visible：
        # 门离场后 gate_visible 立刻为 False，但 e_x 仍会被自由外推（实测能到 +2.38，
        # 正常跟踪时上限只有 +0.60）。age 是时间判据，sig_x 是不确定性判据，两者都要过。
        out['trust'] = bool(out['track'] and age is not None
                            and age <= cfg['trust_age_s']
                            and out['sig_x'] <= cfg['max_sig_x'])
        if self.obs_debug is not None:
            out['obs'] = self.obs_debug
        out['att_used'] = att_used
        return out


# ============================================================
# 输出
# ============================================================
class JsonSink(object):
    """原子写 JSON：tmp + os.replace，读端永远不会读到半个文件。"""

    def __init__(self, path, hz=20.0, log=None):
        """path = 输出 JSON；hz = 写出频率（0/None = 每拍都写）"""
        self.path = path
        self.period = 1.0 / hz if hz and hz > 0 else 0.0
        self.last = 0.0
        self.tmp = path + '.tmp'
        self.n_wrote = 0
        self.log = log or (lambda m: None)
        try:
            d = os.path.dirname(path)
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
        except OSError as e:
            self.log('[sink] 目录不可用: %s' % e)

    def write(self, obj, now):
        """按 hz 节流后原子写（tmp → fsync → os.replace）。写失败只记日志，不抛。"""
        if self.period and (now - self.last) < self.period:
            return
        try:
            with open(self.tmp, 'w', encoding='utf-8') as f:
                json.dump(obj, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(self.tmp, self.path)
            self.last = now
            self.n_wrote += 1
        except OSError as e:
            self.log('[sink] 写 %s 失败: %s' % (self.path, e))


class LogSink(object):
    """终端 + 文件日志。"""

    def __init__(self, log_dir, enabled=True, print_hz=2.0, every_s=1.0, quiet=False):
        """终端打印与落文件分开节流：终端给人看（低频），文件给事后分析（每 1s）"""
        self.enabled = enabled
        self.quiet = quiet
        self.print_period = 1.0 / print_hz if print_hz > 0 else 0.0
        self.every = every_s
        self.last_print = 0.0
        self.last_log = 0.0
        self.fh = None
        if enabled and log_dir:
            try:
                os.makedirs(log_dir, exist_ok=True)
                self.fh = open(os.path.join(log_dir, 'viskf.log'), 'a', encoding='utf-8')
            except OSError:
                self.fh = None

    def emit(self, out, now):
        """一行状态：三通道值 + 姿态模式 + 拒收/重捕计数 + CLIP/COAST/LOST 标记"""
        fl = out['flags']
        line = ('e_x=%+.3f(%.3f/s) el=%+.3fdeg(%.3f/s) s=%.3f(%.3f/s) '
                'att=%s out=%d reinit=%d%s%s%s'
                % (out['e_x'], out['de_x'], math.degrees(out['el']), out['del'],
                   out['s_n'], out['ds_n'],
                   'ok' if fl['att_ok'] else ('hold' if fl['att_hold'] else 'degraded'),
                   out['outliers'], out['reinits'],
                   ' CLIP' if fl['clip'] else '', ' COAST' if fl['coasting'] else '',
                   ' LOST' if fl['lost'] else ''))
        if (not self.quiet) and self.print_period and (now - self.last_print) >= self.print_period:
            print(line, flush=True)
            self.last_print = now
        if self.fh and self.every and (now - self.last_log) >= self.every:
            self.fh.write('%s %s\n' % (time.strftime('%H:%M:%S'), line))
            self.fh.flush()
            self.last_log = now

    def close(self):
        """关日志文件句柄（退出路径必调，否则最后几行可能还在缓冲区里）"""
        if self.fh:
            try:
                self.fh.close()
            except OSError:
                pass


# ============================================================
# 主循环
# ============================================================
def run(cfg, quiet=False):
    """生产主循环：读 det + 读遥测 → 滤波 → 写 momo_viskf.json → 打日志，直到 Ctrl-C。

    ⚠ 固定 sleep 节流（不是精确定时）：50Hz 下每拍 20ms，抖动几 ms 无所谓，
    因为滤波器用的是**实测 dt** 做预测，不假设定步长。
    """
    det_src = DetSource(cfg)
    att_src = AttitudeSource(cfg)
    core = VisKF(cfg)
    sink = JsonSink(cfg['out_path'], cfg['out_hz'])
    logs = LogSink(cfg['log_dir'], print_hz=cfg['print_hz'], every_s=cfg['log_every_s'],
                   quiet=quiet)
    period = 1.0 / max(1.0, cfg['loop_hz'])
    print('[viskf] 启动 | det=%s tel=%s out=%s | loop %.0fHz' %
          (cfg['det_path'], cfg['tel_path'], cfg['out_path'], cfg['loop_hz']), flush=True)
    try:
        while True:
            now = time.time()
            att = att_src.poll(now)
            obs, _has_new = det_src.poll(now)
            out = core.step(now, obs, att)
            sink.write(out, now)
            logs.emit(out, now)
            time.sleep(period)
    except KeyboardInterrupt:
        print('[viskf] 退出', flush=True)
    finally:
        logs.close()


def cmd_status(cfg):
    """`--status`：打印配置来源与两个数据源的存活情况（排障第一步就看它）"""
    src = cfg.get('config_path')
    print('[viskf] 配置来源: %s' % (src if src else '（未找到配置文件，全用内置缺省）'))
    print('[viskf] 生效值（内置缺省 < 配置文件 VISKF_* < 命令行）:')
    for k in sorted(cfg):
        print('  %-16s = %r' % (k, cfg[k]))
    for name, path in (('det', cfg['det_path']), ('tel', cfg['tel_path'])):
        if os.path.exists(path):
            print('[viskf] %s 源: %s 存在, mtime 年龄 %.2fs' % (name, path, time.time() - _mtime(path)))
        else:
            print('[viskf] %s 源: %s 不存在（%s）' %
                  (name, path, '中位机 shm_sink 未部署或遥测 0 帧' if name == 'tel' else 'front.py 未在跑'))


# ============================================================
# 入口（生产）
# 自检 / 离线联调**不在本文件** —— 见 tests/test_viskf.py。
# 主程序保持纯生产代码，便于整体并入更大的工程（无测试依赖、无 random）。
# ============================================================
def main(argv=None):
    """入口。优先级：内置缺省 < `config/viskf_config.py` < 环境变量 VISKF_CONFIG < 命令行。

    `--selftest` / `--mock` 已被移出主程序（保留参数只为了给出明确指引），
    见 `tests/test_viskf.py`。
    """
    ap = argparse.ArgumentParser(description='viskf — 视觉穿门卡尔曼滤波（生产进程）')
    ap.add_argument('--status', action='store_true', help='打印生效配置与数据源状态后退出')
    ap.add_argument('--config', default=None,
                    help='配置文件路径（默认 <工程根>/config/viskf_config.py，'
                         '回落到 quick_config.py；也可用环境变量 VISKF_CONFIG）')
    ap.add_argument('--shm-dir', default=None, help='共享内存目录（默认 /dev/shm）')
    ap.add_argument('--det-path', default=None, help='det JSON 路径覆盖')
    ap.add_argument('--tel-path', default=None, help='遥测 JSON 路径覆盖')
    ap.add_argument('--out-path', default=None, help='输出 JSON 路径覆盖')
    ap.add_argument('--loop-hz', type=float, default=None)
    ap.add_argument('--quiet', action='store_true', help='不打印到终端（仅日志文件）')
    # 测试入口已移出：仍接受这两个参数，但给出明确指引（而不是 "unrecognized arguments"）
    ap.add_argument('--selftest', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--mock', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.selftest or args.mock:
        flag = '--selftest' if args.selftest else '--mock'
        print('[viskf] %s 不属于主程序：自检与离线联调已移到 tests/test_viskf.py\n'
              '        用法:  python3 tests/test_viskf.py %s' % (flag, flag), flush=True)
        return 2

    cli = {'loop_hz': args.loop_hz}
    cfg = load_cfg(cli, config=args.config)
    if args.shm_dir:
        s = args.shm_dir.rstrip('/')
        cfg['shm_dir'] = s
        cfg['det_path'] = args.det_path or os.path.join(s, 'momo_det_front.json')
        cfg['tel_path'] = args.tel_path or os.path.join(s, 'momo_telemetry.json')
        cfg['out_path'] = args.out_path or os.path.join(s, 'momo_viskf.json')
    else:
        if args.det_path:
            cfg['det_path'] = args.det_path
        if args.tel_path:
            cfg['tel_path'] = args.tel_path
        if args.out_path:
            cfg['out_path'] = args.out_path

    if args.status:
        cmd_status(cfg)
        return 0
    if not cfg['enabled']:
        print('[viskf] VISKF_ENABLED=False，退出')
        return 0
    run(cfg, quiet=args.quiet)
    return 0


if __name__ == '__main__':
    sys.exit(main())
