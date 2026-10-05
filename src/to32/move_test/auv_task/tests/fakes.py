# -*- coding: utf-8 -*-
"""离线测试假件 —— 让任务代码脱离板端（无串口/无共享内存）在 Windows 上跑起来

★ 为什么必须有这一层：任务代码只通过 ctx 看世界（vision/depth/viskf 都是接口），
  所以把这三个接口换成假实现，同一份任务代码就能在电脑上跑回归。
  水池时间很贵 —— 能在桌上验的（跳转顺序、超时、判据、测试模式过滤）就别下水验。
"""
import fnmatch
import os
import sys
import time

# 让 `python run_task.py` 直接跑时也能 import auv_task（把 move_test 目录塞进 sys.path）
_HERE = os.path.dirname(os.path.abspath(__file__))                 # .../move_test/auv_task/tests
_PKG_PARENT = os.path.dirname(os.path.dirname(_HERE))              # .../move_test
for _p in (_PKG_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _tc():
    """取 test_config 模块（取不到返回 None —— 台架要能在没有它时也跑）"""
    try:
        from auv_task import test_config as TC
        return TC
    except Exception:
        return None


def sim_defaults(**over):
    """读 test_config.SIM 的仿真参数（over 覆盖之）

    只有**本机离线**用；板端不看 SIM。这样"改仿真节奏"也在同一个 test_config 里。
    """
    d = {'dt': 0.05, 'max_s': 600.0, 'pool_m': 1.30, 'depth_rate_mps': 0.15,
         'start_depth_m': 0.0, 'depth_ok': True, 'viskf_ok': False,
         'servo_ready': True, 'echo_log': False}
    TC = _tc()
    if TC is not None:
        try:
            for k, v in (getattr(TC, 'SIM', None) or {}).items():
                d[k] = v
        except Exception:
            pass
    d.update(over or {})
    return d


# ---------------------------------------------------------------- 配置假件
def default_cfg(**over):
    """造一个最小配置对象（只含任务要读的键；缺的键由各任务的 default 兜住）

    传 over 可以覆盖任意键，便于单测特定分支。

    ★ 建好后会自动套用 test_config.GLOBAL_PARAMS（与板端 TaskRunner 同一条路径），
      所以离线跑出来的现象与"按 test_config 配好的板端"是一致的。
      ⚠ over 的优先级最高（单测里的显式覆盖不该被配置吃掉），所以放在最后。
    """
    from types import SimpleNamespace
    c = SimpleNamespace(
        # ---- 水池 / 高度（赛事规则口径：离底高度 cm）
        AUV_POOL_DEPTH_CM=130.0,
        AUV_MIN_DEPTH_CM=10.0,
        AUV_DEPTH_BOTTOM_MARGIN_CM=10.0,
        AUV_BALL_HEIGHT_CM=60.0,          # 撞球：距底 60cm
        AUV_GATE_LOW_HEIGHT_CM=45.0,      # 矮门：中心距底 45cm
        AUV_GATE_HIGH_HEIGHT_CM=65.0,     # 高门：中心距底 65cm
        AUV_RACK_HEIGHT_CM=30.0,          # 待捡球（占位，现场量）
        AUV_BASKET_HEIGHT_CM=45.0,        # 收集框（占位）
        AUV_HOME_DEPTH_CM=25.0,
        AUV_RELEASE_HEIGHT_CM=110.0,      # 投放前悬停：离底 110cm（≈水深 20cm）
        # ---- 门
        AUV_GATE_COUNT=4,
        AUV_GATE_SEQ='low,high,low,high',
        # ---- 视觉 / 目标
        AUV_LABEL_BALL='ball',
        AUV_LABEL_GATE='gate',
        AUV_LABEL_PICK='ball',
        AUV_IMG_W=640.0,
        AUV_IMG_H=480.0,
        AUV_PIX_TOL_PX=25.0,
        AUV_CENTER_HOLD_S=1.5,
        # ---- 伺服
        AUV_VERT_SERVO=True,
        AUV_YAW_SERVO=True,
        AUV_TURN_RIGHT_SIGN=1.0,
        AUV_YAW_SIGN=1.0,
        AUV_DEPTH_SIGN=1.0,
        AUV_KP_DEPTH_CMS=40.0,
        AUV_DEPTH_RATE_MAX_CMS=30.0,
        AUV_VERT_DEADBAND_PX=15.0,
        AUV_KP_YAW_DPS=25.0,
        AUV_YAW_RATE_MAX_DPS=20.0,
        AUV_YAW_DEADBAND_PX=15.0,
        AUV_KP_SWAY=0.60,
        AUV_SWAY_MAX=0.50,
        AUV_SWAY_SIGN=1.0,
        AUV_SURGE_SIGN=1.0,
        AUV_KP_SURGE_PICK=0.40,
        AUV_SURGE_MAX_PICK=0.30,
        AUV_MIN_THRUST=0.10,
        AUV_SURGE_MAX=1.0,
        AUV_SURGE_SEEK=0.25,
        AUV_SURGE_RAM=0.40,
        AUV_SURGE_GATE=0.30,
        # ---- viskf
        AUV_VISKF_USE=True,
        AUV_GATE_YAW_DEADBAND=0.03,
        AUV_GATE_YAW_KP_DPS=40.0,
        AUV_GATE_YAW_KD_DPS=8.0,
        AUV_GATE_YAW_RATE_MAX_DPS=20.0,
        AUV_GATE_SURGE_K=1.2,
        AUV_GATE_SURGE_MIN=0.25,
        AUV_GATE_SURGE_MAX=0.85,
        AUV_GATE_S_STAR=0.85,
        AUV_GATE_CROSS_TOL=0.05,
        AUV_GATE_SWAY_WITH_VISKF=False,
        AUV_GATE_PASS_W_RATIO=0.80,
        AUV_GATE_LOST_S=0.5,
        # ---- 判据阈值
        AUV_DEPTH_TOL_CM=8.0,
        AUV_DEPTH_HOLD_S=2.0,
        AUV_SEEK_TIMEOUT_S=30.0,
        AUV_SEEK_SETTLE_S=1.5,
        AUV_RAM_TIMEOUT_S=20.0,
        AUV_RAM_ARM_S=0.8,
        AUV_RAM_W_PX=220.0,
        AUV_RAM_W_NEAR_PX=150.0,
        AUV_RAM_LOST_S=0.8,
        AUV_GATE_TIMEOUT_S=30.0,
        AUV_CENTER_TIMEOUT_S=15.0,
        AUV_SIT_RATE_CMS=15.0,
        AUV_SIT_CLEAR_CM=6.0,
        AUV_SIT_CLEAR_HOLD_S=1.0,
        AUV_SIT_STALL_CM=2.0,
        AUV_SIT_STALL_S=2.0,
        AUV_SIT_VZ=0.02,
        AUV_SIT_TIMEOUT_S=25.0,
        AUV_BOTTOM_HOLD_S=3.0,
        AUV_ASCEND_DEPTH_M=1.0,
        AUV_ASCEND_TIMEOUT_S=20.0,
        AUV_SURFACE_DEPTH_M=0.0,
        AUV_SURFACE_DONE_CM=15.0,
        AUV_SURFACE_TIMEOUT_S=30.0,
        AUV_WALL_TIMEOUT_S=20.0,
        AUV_USE_IMU=False,
        # ---- 转向
        AUV_TURN1_DEG=120.0,
        AUV_TURN2_DEG=60.0,
        AUV_YAW_RATE_DPS=30.0,
        AUV_YAW_TOL_DEG=3.0,
        AUV_TURN_SETTLE_S=1.0,
        AUV_TURN_TIMEOUT_S=10.0,
        # ---- 投放
        AUV_RELEASE_TURN_DEG=90.0,
        AUV_RELEASE_DIST_M=2.0,
        AUV_RELEASE_HOVER_S=3.0,
        AUV_SERVO_IMPL='stub',
        # ---- 航位推算
        AUV_SPEED_MPS=0.25,
        AUV_SPEED_K_SURGE=1.0,
        AUV_G=9.81,
        AUV_DR_STILL_G=0.05,
        AUV_DR_STUCK_S=2.0,
        AUV_HOME_TURN_DEG=90.0,
        # ---- 预算 / 模式
        AUV_BUDGET_S=900.0,
        AUV_SURFACE_THEN_ROV=True,
        AUV_CMD_POLICY='supervise',
        # ---- [2026-10-05] 两种运行模式（真实作业 / 调试测试）
        AUV_ACCEPTANCE_LOCK=True,              # 真实作业模式强制 ignore（除切模式外拒绝一切）
        AUV_TEST_FORCE_VIDEO=True,             # 测试模式自动开图像回传
        MODE_ROV=0,                        # ★ dispatcher 模式 id（有线 ROV），不是 0x04 帧码
        MODE_AUV=1,                        # dispatcher 模式 id（AUV 自主）
        MODE_IDLE=-1,                      # [2026-10-04] 待命态 id（上电默认）
        # ---- [2026-10-05] 启动门控 + 状态提示回传
        AUV_REQUIRE_PC_CMD=True,           # ★ AUV 只能由上位机 $CMD 切入
        AUV_MSG_ENABLED=True,
        AUV_MSG_IP='127.0.0.1',
        AUV_MSG_PORT=8085,
        AUV_MSG_HZ=50.0,                   # 离线测试：高节拍，别被节流挡住
        AUV_MSG_MIN_LEVEL='INFO',
        AUV_MSG_DEDUP_S=0.0,               # 离线测试：关掉去重，每条都发
        AUV_MSG_MAX_FAIL=3,
        AUV_MSG_HEARTBEAT_S=0.0,           # 离线测试：关心跳，免得干扰断言
    )
    # ---- test_config.GLOBAL_PARAMS（先套配置，再用 over 覆盖）
    TC = _tc()
    if TC is not None:
        try:
            for k, v in (getattr(TC, 'GLOBAL_PARAMS', None) or {}).items():
                setattr(c, str(k), v)
        except Exception:
            pass
    for k, v in (over or {}).items():
        setattr(c, k, v)
    return c


# ---------------------------------------------------------------- 视觉假件
def make_obs(label='ball', cx=320.0, cy=240.0, w=100.0, h=100.0, score=0.9,
             aim=None, img_w=640.0, img_h=480.0, frame=1):
    """按 vision_if.poll 的**完整字段契约**造一帧观测（少一个键就可能 KeyError）

    ex/ey 是归一化偏差（横向 /w_bbox，纵向 /h_bbox），伺服律直接用它们，
    所以必须按同样口径算，否则测出来的伺服方向是假的。
    """
    x0, y0 = (img_w / 2.0), (img_h / 2.0)
    dx0, dy0 = (0.0, 0.0)
    if aim:
        try:
            dx0, dy0 = float(aim[0]), float(aim[1])
        except (TypeError, ValueError, IndexError):
            dx0, dy0 = 0.0, 0.0
    dx = cx - x0 - dx0
    dy = cy - y0 - dy0
    bw = max(1.0, float(w))
    bh = max(1.0, float(h))
    return {
        'label': label, 'canon': label, 'score': float(score),
        'cx': float(cx), 'cy': float(cy), 'dx': dx, 'dy': dy,
        'ex': dx / bw, 'ey': dy / bh,
        'w': bw, 'h': bh,
        'x1': cx - bw / 2.0, 'y1': cy - bh / 2.0, 'x2': cx + bw / 2.0, 'y2': cy + bh / 2.0,
        'clip_l': False, 'clip_r': False, 'clip_t': False, 'clip_b': False, 'clip': False,
        'age_s': 0.0, 'frame': int(frame),
    }


def script_obs(stage, elapsed_s, label=None):
    """★ 按 test_config.VISION_SCRIPT 造一帧观测（None = 这一帧看不见）

    剧本规则（全部在 test_config.py 里配，改剧本不用动代码）：
      1. VISION_SCRIPT['enabled']=False 或阶段在 never_see 里 → 恒 None
      2. 命中 stages 表（键支持 * ? 通配）→ 用该条目的 label/cx/cy/w/grow/delay/disappear
      3. 没命中 → 用通用规则：default_delay_s / start_w_px / grow_w_px_per_s / disappear_w_px

    `disappear` ≤ 0 表示"永不消失"。
    """
    TC = _tc()
    sc = (getattr(TC, 'VISION_SCRIPT', None) or {}) if TC is not None else {}
    if not sc or not sc.get('enabled', True):
        return None
    st = str(stage).upper()

    def _hit(pool):
        return any(fnmatch.fnmatch(st, str(p).upper()) for p in pool)

    if _hit(sc.get('never_see') or []):
        return None

    spec = None
    for pat, val in (sc.get('stages') or {}).items():
        if fnmatch.fnmatch(st, str(pat).upper()):
            spec = dict(val)
            break
    if spec is None:                                    # 通用规则兜底
        spec = {'w': sc.get('start_w_px', 80.0), 'grow': sc.get('grow_w_px_per_s', 0.0),
                'disappear': sc.get('disappear_w_px', 0.0)}
    spec.setdefault('label', label or 'ball')
    spec.setdefault('cx', 320.0)
    spec.setdefault('cy', 240.0)
    spec.setdefault('w', sc.get('start_w_px', 80.0))
    spec.setdefault('grow', 0.0)
    spec.setdefault('delay', 0.0 if _hit(sc.get('see_immediately') or [])
                    else float(sc.get('default_delay_s', 0.0)))
    spec.setdefault('disappear', 0.0)                   # ≤0 = 永不消失

    el = float(elapsed_s)
    if el < float(spec['delay']):
        return None
    w = float(spec['w']) + float(spec['grow']) * (el - float(spec['delay']))
    dis = float(spec.get('disappear') or 0.0)
    if dis > 0 and w > dis:
        return None                                     # 门贴近到出画 → 触发"已穿过"判定
    return make_obs(str(spec['label']), cx=float(spec['cx']), cy=float(spec['cy']), w=w)


class FakeVision(object):
    """视觉假件：观测由注入的 getter(cam, label, now) 决定；返回 None = 本帧没检出"""

    def __init__(self, getter=None):
        self.getter = getter or (lambda cam, label, now: None)
        self.calls = 0                                   # 调用计数（断言"到底有没有去看"）

    def poll(self, cam, label, now, aim=None):
        self.calls += 1
        return self.getter(cam, label, now)


# ---------------------------------------------------------------- 深度假件
class FakeDepth(object):
    """深度假件：目标深度以有限速率逼近（模拟固件定深闭环 + 水阻）

    ★ 必须模拟"逼近需要时间"，否则所有"定深到位"判据第一拍就满足，
      测出来的流程时序全是假的（这个坑会让水池现场判据全对不上）。
    """

    def __init__(self, pool_m=1.3, rate_mps=0.15, d0=0.0):
        self.pool = float(pool_m)                        # 池深 m
        self.rate = float(rate_mps)                      # 垂向最大速率 m/s
        self.D = float(d0)                               # 当前深度 m
        self.target = None                               # 目标深度 m（外部每拍写入）
        self._last = None                                # 上次 read 的时刻
        self.ok = True                                   # 可以置 False 测降级路径

    def set_target(self, d_m):
        """外部（harness）把本拍下发的目标深度喂回来"""
        self.target = float(d_m)

    def read(self, now):
        if self._last is None:
            self._last = float(now)
        dt = max(0.0, float(now) - self._last)
        self._last = float(now)
        v = 0.0
        if self.ok and self.target is not None:
            err = self.target - self.D
            step = self.rate * dt
            if abs(err) <= step:
                self.D = self.target
            else:
                self.D += (step if err > 0 else -step)
                v = (step if err > 0 else -step) / dt if dt > 0 else 0.0
        cl = max(0.0, self.pool - self.D)
        return {'ok': bool(self.ok), 'D': self.D, 'v_z': v, 'clearance': cl,
                'sigma_D': 0.05, 'age_s': 0.0, 'stale': False, 'degraded': False,
                'H': self.pool}


# ---------------------------------------------------------------- 图像卡尔曼假件
class FakeViskf(object):
    """viskf 假件：默认恒不可用（测原始像素伺服降级路径）；注入 fn 可测滤波路径"""

    def __init__(self, fn=None):
        self.fn = fn or (lambda now: {'ok': False, 'trust': False})

    def read(self, now):
        r = self.fn(now)
        if r is None:
            return {'ok': False, 'trust': False}
        return r


def vk_ok(e_x=0.0, de_x=0.0, s_n=0.5, sig_x=0.02, el=0.0, d_el=0.0, coasting=False):
    """造一帧"可用"的 viskf 读数（字段与 viskf_if._empty 一致）"""
    return {'ok': True, 'trust': True, 'age': 0.0, 'gate_visible': True,
            'e_x': e_x, 'de_x': de_x, 'sig_x': sig_x,
            'el': el, 'd_el': d_el, 'sig_el': 0.02,
            's_n': s_n, 'ds_n': 0.0, 'sig_s': 0.01,
            'coasting': coasting, 'lost': False, 'att_degraded': False,
            'clip': False, 'stale': False}


# ---------------------------------------------------------------- 舵机假件
class FakeServo(object):
    """舵机假件：记录被触发了几次（用来验证"幂等：只触发一次"）"""

    def __init__(self, ready=True, ok=True):
        self.ready_flag = bool(ready)
        self.ok = bool(ok)
        self.drops = 0                                   # drop() 被调用次数

    def ready(self):
        return self.ready_flag

    def drop(self):
        self.drops += 1
        return self.ok

    def done(self):
        return self.drops > 0 and self.ok


# ---------------------------------------------------------------- 状态提示假件（$MSG）
class FakeSock(object):
    """假 UDP socket：把发出去的帧收进 list，绝不真发包"""

    def __init__(self, fail_times=0):
        self.sent = []            # 已发出的文本帧
        self.fail_times = int(fail_times)   # 前 N 次 sendto 抛异常（测"连续失败放弃"）
        self.closed = False

    def setblocking(self, flag):
        pass

    def sendto(self, data, addr):
        if self.fail_times > 0:
            self.fail_times -= 1
            raise OSError('fake send failure')
        text = data.decode('utf-8') if isinstance(data, bytes) else str(data)
        self.sent.append(text)
        return len(text)

    def close(self):
        self.closed = True


class RecNotifier(object):
    """记录型提示通道（与 AuvMsg 同接口，但不启线程、不发网络）

    用来断言"执行器在某个节点到底有没有发提示、发的什么级别"。
    """

    def __init__(self):
        self.items = []           # [(level, code, text, stage), ...]
        self.last_stage = '-'
        self.started = 0
        self.stopped = 0

    def start(self):
        self.started += 1
        return True

    def stop(self):
        self.stopped += 1
        return None

    def push(self, level, code, text, stage=None):
        if stage:
            self.last_stage = str(stage)
        self.items.append((str(level), str(code), str(text), str(stage or '-')))
        return True

    def info(self, code, text, stage=None):
        return self.push('INFO', code, text, stage)

    def warn(self, code, text, stage=None):
        return self.push('WARN', code, text, stage)

    def error(self, code, text, stage=None):
        return self.push('ERROR', code, text, stage)

    def codes(self):
        return [c for _, c, _, _ in self.items]

    def has(self, code=None, level=None, sub=None):
        for lv, c, t, _ in self.items:
            if code is not None and c != code:
                continue
            if level is not None and lv != level:
                continue
            if sub is not None and sub not in t:
                continue
            return True
        return False

    def alive(self):
        return True

    def summary(self):
        return 'rec(%d)' % len(self.items)


# ---------------------------------------------------------------- 日志收集
class LogSink(object):
    """把日志收进 list，便于断言"某条 WARN 到底打没打"""

    def __init__(self, echo=False):
        self.lines = []
        self.echo = echo

    def __call__(self, msg):
        self.lines.append(str(msg))
        if self.echo:
            print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)

    def has(self, sub):
        """是否包含某子串"""
        return any(sub in l for l in self.lines)

    def dump(self):
        """全部日志（一行一条）"""
        return '\n'.join(self.lines)
