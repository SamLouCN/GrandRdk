# -*- coding: utf-8 -*-
"""任务上下文 TaskCtx —— 任务与执行器之间**唯一**的窄接口

★ 为什么要这层：任务代码必须能在 Windows 上脱离板端跑（离线回归），
  所以任务**不许**直接摸 vision_if / depth_if / viskf_if / 配置模块，
  一切通过 ctx 走。换一套 Fake 接口注入进来，同一份任务代码就能离线跑。

职责边界（别越界）：
  * ctx **提供**观测（see/vk/dep/tel）、计时（elapsed/holding）、参数（g/gf）、出口（out/goto/abort/finish）
  * ctx **不决定**流程 —— 跳转只是往 ctx.pending 里放一个信号，由 runner 统一裁决
    （测试模式过滤、SURFACE 强制保留、DONE 收尾都在 runner 里做，任务不知道这些）
"""
import time  # 时间库: 初始时刻兜底

from .servo import wrap180, sign  # 伺服律里的角度工具（纯函数，可离线跑）
from .servo import tel_depth_cm, tel_yaw, tel_acc  # 遥测宽容取值
from .notify import NoMsg         # 状态提示通道（默认静默；runner 会换成真 $MSG 通道）


class YawEst(object):
    """航向估计器 —— 无遥测时开环积分，有遥测时自动升级为真闭环

    开环模式：按 AUV_YAW_RATE_DPS 的标称角速度朝目标角逼近（不会越过目标）。
    ⚠ 这是**估计**不是测量：AUV_YAW_RATE_DPS 未经水池标定时，转 120° 可能实际转 100~140°。
    """

    def __init__(self, cfg):
        self.cfg = cfg            # 配置对象
        self.yaw = 0.0            # 当前航向估计（deg）
        self.src = 'none'         # 来源: none / openloop / tel

    def update(self, dt, tel_yaw=None, target=None, cfg_gf=None):
        """按 dt 推进航向估计

        tel_yaw 非空 → 直接用遥测真值覆盖（优先级最高，自动升级为真闭环）；
        否则若有 target，按标称角速度朝它逼近；都没有 → 保持不动。
        """
        gf = cfg_gf or (lambda n, d: getattr(self.cfg, n, d))
        if tel_yaw is not None:                          # 有遥测 → 真闭环
            self.yaw = wrap180(float(tel_yaw))
            self.src = 'tel'
            return
        if target is None:                               # 没目标也没遥测 → 不动
            self.src = 'none'
            return
        rate = float(gf('AUV_YAW_RATE_DPS', 30.0))       # 标称角速度 °/s（🔴 待标定）
        if rate <= 0:
            self.src = 'none'
            return
        d = wrap180(float(target) - self.yaw)
        step = rate * max(0.0, float(dt))
        if abs(d) <= step:
            self.yaw = wrap180(float(target))
        else:
            self.yaw = wrap180(self.yaw + sign(d) * step)
        self.src = 'openloop'

    def now(self):
        """返回当前航向估计值（deg，已归一化到 (-180,180]）"""
        return self.yaw


class TaskCtx(object):
    """任务上下文：观测 + 计时 + 参数 + 出口"""

    def __init__(self, cfg, log, vision, depth, viskf, servo=None, notify=None):
        self.cfg = cfg                                   # 配置对象（to32_config，含 auv_config 全量并入）
        self.log = log or (lambda m: None)               # 日志回调
        self.vision = vision                             # 视觉接口（VisionIF 或 Fake）
        self.depth = depth                               # 深度接口（DepthIF 或 Fake）
        self.viskf = viskf                               # 图像卡尔曼接口（ViskfIF 或 Fake）
        self.servo = servo                               # 投放舵机接口（ServoIF 或 Stub）
        # ★ 上位机终端状态提示（$MSG）。默认 NoMsg（静默）—— 离线仿真不会真的往外发 UDP。
        self.notify = notify if notify is not None else NoMsg()

        # ---- 持久目标量（下发给固件，跨阶段保持）
        self.depth_cm = 0.0                              # 目标深度 cm
        self.yaw_deg = 0.0                               # 目标航向 deg（绝对角）
        self.yaw_est = YawEst(cfg)                       # 航向估计器

        # ---- 本拍输入（runner 每拍刷新）
        self.now = time.time()                           # 当前时刻
        self.dt = 0.0                                    # 距上拍秒数
        self.tel = None                                  # 最近一帧下位机遥测
        self.dep = {'ok': False, 'D': None, 'v_z': 0.0, 'clearance': None}  # 深度读数
        self.stage = 'IDLE'                              # 当前阶段名（runner 写，任务只读）

        # ---- 阶段计时（runner 在切段时复位）
        self.t0 = time.time()                            # 进入本阶段的时刻
        self.hold = {}                                   # 判据保持计时器 key -> 起始时刻
        self.scan_base_cm = None                         # 搜索期扫深基准（进入搜索时的深度）

        # ---- 上报缓存（AuvReport.make_snapshot 从这些名字取，★ 名字不许改）
        self.seq = 0                                     # 已跑拍数
        self.last_dep = None                             # 最近一次深度读数
        self.last_obs = None                             # 最近一次视觉观测
        self.last_obs_cam = '-'                          # 该观测来自 front / bottom
        self.last_obs_ts = 0.0                           # 该观测的时刻
        self.last_vk = None                              # 最近一次图像卡尔曼读数
        self.abort_reason = ''                           # 非空 = 走的是中止上浮路径

        # ---- 跳转信号（任务写、runner 读并清空）
        self.pending = None                              # ('goto', 阶段名) / ('abort', 原因) / ('finish', None)

        # ---- 测试模式（runner 注入；None = 全流程作业）
        self.test = None                                 # TestCfg 实例

    # ================================================================ 参数
    def g(self, name, default=None):
        """读配置；键缺失一律回落缺省（板端用户可能没同步新配置段）"""
        return getattr(self.cfg, name, default)

    def gf(self, name, default=0.0):
        """读配置并强制转 float（配置里可能被写成字符串/int，转失败用缺省）"""
        try:
            return float(self.g(name, default))
        except (TypeError, ValueError):
            return float(default)

    # ================================================================ 观测
    def see(self, cam, label, now, aim=None):
        """★ 视觉取流统一入口：顺手把最近一次观测缓存下来供状态上报用

        观测为 None（本帧没检出）时**保留上一次** —— 上报里带 vis_age，
        "多久没看到目标"本身就是极重要的诊断量（搜索超时前一定先看到它变大）。
        """
        obs = self.vision.poll(cam, label, now, aim=aim) if self.vision is not None else None
        if obs is not None:
            self.last_obs = obs
            self.last_obs_cam = cam
            self.last_obs_ts = float(now)
        return obs

    def vk(self, now):
        """★ 图像卡尔曼取流统一入口：顺手缓存

        返回 None = 这一帧不能用（没起 / 超期 / trust=False）。
        调用方必须判 None 并**退回原始像素伺服** —— 这是设计内的降级路径，不是故障。
        """
        if self.viskf is None or not self.g('AUV_VISKF_USE', True):
            self.last_vk = None
            return None
        r = self.viskf.read(now)
        self.last_vk = r
        return r if (isinstance(r, dict) and r.get('ok')) else None

    def depth_ok(self):
        """深度源是否可用（ok 且 D 有值）"""
        d = self.dep or {}
        return bool(d.get('ok')) and d.get('D') is not None

    def depth_cm_now(self):
        """当前深度 cm（不可用返回 None）"""
        if not self.depth_ok():
            return None
        return float(self.dep['D']) * 100.0

    # ---- 遥测（宽容：字段取不到就是没有，绝不抛）
    def tel_depth_cm(self):
        return tel_depth_cm(self.tel)

    def tel_yaw(self):
        return tel_yaw(self.tel)

    def tel_acc(self):
        return tel_acc(self.tel)

    # ================================================================ 计时
    def elapsed(self, now=None):
        """进入本阶段后的秒数"""
        return float(now if now is not None else self.now) - float(self.t0)

    def holding(self, key, cond, now, need_s):
        """条件连续保持 need_s 秒才返回 True；条件中断即清零"""
        if cond:
            if self.hold.get(key) is None:
                self.hold[key] = now
        else:
            self.hold[key] = None
        t = self.hold.get(key)
        return bool(t is not None and (now - t) >= float(need_s))

    def clear_hold(self, key):
        """手动清掉一个判据计时器（目标丢失时必须清，否则回来就"瞬间满足"）"""
        self.hold[key] = None

    def hold_s(self, want_s):
        """★ "判据需保持 N 秒"的唯一取时入口（测试模式下会被压短）

        所有 holding(...) 的 need_s 都必须过这里，否则测试模式下要干等 2~3 秒/判据，
        水池里单段调试根本没法看现象。
        """
        if self.test is not None:
            return self.test.hold_s(want_s)
        return float(want_s)

    # ================================================================ 出口
    def out(self, surge=0.0, sway=0.0, depth=None, yaw=None, stop=False, note=''):
        """组装控制量。depth/yaw 缺省沿用当前保持的目标值（它们是持久目标量）

        ★ 字段集与旧 mission._out **完全一致**：mode_auv 直接按这些键组装 0x09。
        """
        return {
            'depth': (self.depth_cm if depth is None else float(depth)),
            'yaw': (self.yaw_deg if yaw is None else float(yaw)),
            'surge': float(surge), 'sway': float(sway),
            'stop': bool(stop), 'stage': self.stage, 'note': note,
        }

    def goto(self, stage_name):
        """请求跳到指定阶段（真正生效在 runner，便于做测试模式过滤）"""
        self.pending = ('goto', stage_name)

    def abort(self, reason):
        """请求中止：打标记后强制走 SURFACE 上浮

        ★ 上浮是硬性动作，不允许配置为"直接结束"（方案 §16c 用户拍板）。
        """
        self.abort_reason = str(reason)
        self.pending = ('abort', reason)

    def finish(self):
        """请求直接结束（不经上浮）—— 只允许 DONE 阶段用"""
        self.pending = ('finish', None)

    def thrust(self, x, limit):
        """推力限幅 + 死区补偿（转发到 servo.py 的纯函数）"""
        from .servo import thrust as _thrust
        return _thrust(self, x, limit)
