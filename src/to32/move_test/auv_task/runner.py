# -*- coding: utf-8 -*-
"""任务执行器 TaskRunner —— 替代旧 mission.Mission 的 805 行单体状态机

★ 对外契约（**与 mode_auv.py 零改动对接**，这是重构能落地的硬前提）：
    runner = TaskRunner(cfg, vision=..., depth=..., viskf=..., log=...)
    cmd = runner.step(now, dt, tel)     # 返回 None = 本拍不下发
    cmd 的键：depth / yaw / surge / sway / stop / stage / note   ← 与旧版完全一致
    make_snapshot(now, cmd, runner, tel) 需要的属性也都在 runner 上代理好了：
    t0 / stage / seq / last_dep / last_obs / last_obs_cam / last_obs_ts / last_vk / abort_reason

★ 执行器负责的四件事（任务类不许自己做）：
  1. 阶段顺序与跳转裁决（含测试模式过滤：被跳过的段会被"穿透"解析到下一个在跑的段）
  2. 首帧锚定（不锚定一进 AUV 就会"回水面 + 转向"）
  3. 全局预算（赛事 15 分钟硬上限）与 SURFACE 强制保留
  4. 监督指令 $AUVCTL（暂停/跳过/跳转/中止/重跑）—— 水池测试的保命通道

★★ MODE_ROV 同名巨坑（改这段代码前必读）：
  `config/to32_config.py` 的 MODE_ROV = 0（**dispatcher 模式 id = 有线 ROV**，能用）
  `src/to32/link_stm32.py` 的 MODE_ROV = 0x03（**0x04 帧码 = 无线 ROV**，S100 禁发）
  上浮后"切回 ROV"必须用 **cfg.MODE_ROV（=0）**。写成 link_stm32 的那个，
  0x04 一帧都发不出去（frame_mode 直接返回 None），机器人会留在 AUV 不动。
"""
import time  # 时间库: 首帧锚定与预算计时

from .ctx import TaskCtx                       # 任务上下文
from .testcfg import TestCfg, apply_global     # 测试模式配置 + 全局参数覆盖
from .plan import build_plan, names, pairs, index_of  # 阶段表
from .servo_if import make_servo         # 舵机工厂
from .servo import wrap180               # 角度归一
from .base import Task                   # 任务基类（isinstance 校验用）
from .notify import (make_notifier, apply_notify, describe,
                     CODE_STAGE, CODE_ABORT, CODE_BUDGET, CODE_TEST,
                     CODE_KILLED, CODE_MODE)  # ★ 上位机终端状态提示（$MSG 帧）


# 上浮/结束阶段名（硬编码，与 plan.py 保持一致）
SURFACE = 'SURFACE'
DONE = 'DONE'


class TaskRunner(object):
    """任务执行器：按 plan 顺序驱动一个个独立任务"""

    # ---------------------------------------------------------------- 构造
    def __init__(self, cfg, vision=None, depth=None, viskf=None, log=None,
                 servo=None, plan=None, test=None, start_stage=None, notify=None):
        self.cfg = cfg                                       # 配置对象（to32_config）
        self.log = log or (lambda m: None)                   # 日志回调

        # ---- ★ 上位机终端状态提示通道（$MSG 帧；失败/关闭时不发声，任务不受影响）
        self.notify = notify if notify is not None else make_notifier(cfg, log=self.log)
        apply_notify(self.notify, log=self.log)   # test_config.NOTIFY 覆盖（测试期想静默就在这儿改）

        # ---- ★ test_config.GLOBAL_PARAMS 覆盖（必须在建阶段表**之前**，
        #      否则 AUV_GATE_COUNT / AUV_GATE_SEQ 这类"影响流程结构"的覆盖不生效）
        apply_global(cfg, log=self.log)

        self.plan = plan or build_plan(cfg, log=self.log)    # 阶段表（内部会套 STAGE_PARAMS）
        self.order = names(self.plan)                        # 阶段名（按序）
        self.test = test if test is not None else TestCfg(cfg, log=self.log)

        # ---- 测试模式选段校验（写了不存在的名字 → 自动退回全流程）
        #      传 (阶段名, 任务类名) 而不是纯名字，`task:PassGateTask` 这种写法才校验得了
        _, _, kept = self.test.validate(pairs(self.plan))
        self.active = set(kept)                              # 本轮实际会跑的阶段

        # ---- 建任务实例（每个阶段一个实例；同一类可被多个阶段共用）
        self.tasks = {}
        for s in self.plan:
            self.tasks[s['name']] = s['cls'](**s['params'])

        # ---- 上下文（三个观测接口没传就惰性自建；导入失败 = 该路降级，绝不让构造崩掉）
        self.ctx = TaskCtx(cfg, self.log,
                           vision if vision is not None else self._auto_if('vision_if', 'VisionIF', cfg),
                           depth if depth is not None else self._auto_if('depth_if', 'DepthIF', cfg),
                           viskf if viskf is not None else self._auto_if('viskf_if', 'ViskfIF', cfg),
                           servo=(servo if servo is not None else make_servo(cfg, log=self.log)))
        self.ctx.test = self.test
        self.ctx.notify = self.notify                        # ★ 任务内部可直接发提示（base.bail 用）

        # ---- 运行时状态
        self.killed = False                                  # ★ 是否已被终止（上位机切走 / KILL）
        self.kill_reason = ''                                # 终止原因（供日志与提示用）
        self.anchored = False                                # 首帧是否已锚定
        self.total_t0 = None                                 # 全局起始时刻（预算计时）
        self.budget_s = self._f('AUV_BUDGET_S', 900.0)       # 赛事硬上限 15 分钟
        self._budget_fired = False                           # 预算只触发一次
        self.surfaced = False                                # 是否已经上浮过（保命校验用）
        self.mode_request = None                             # 请求主链切换到的模式 id（★ cfg.MODE_ROV）
        self.paused = False                                  # 监督暂停（$AUVCTL HOLD）
        self.stage = start_stage or self._first_active()     # 当前阶段名
        self.ctx.stage = self.stage
        # ★ 监督指令策略：真实作业(验收)模式**强制** ignore —— 除"上位机切模式"外一律不接受
        self.policy = self.test.cmd_policy(cfg)

        self.log('[AUV] 任务执行器就绪：%d 个阶段，运行模式 = %s'
                 % (len(self.order), self.test.summary()))
        if self.test.enabled:
            self.log('[AUV] 测试模式实际保留：%s' % ','.join([n for n in self.order
                                                              if n in self.active]))
        # ---- 打印 test_config.STAGE_PARAMS 实际生效的覆盖（改了配置一眼看得到）
        ov = [(s['name'], self.test.params(s['name'], getattr(s['cls'], '__name__', '')))
              for s in self.plan]
        ov = [(n, p) for n, p in ov if p]
        if ov:
            self.log('[AUV] test_config 阶段参数覆盖：%s'
                     % '; '.join('%s←%s' % (n, p) for n, p in ov))

        # ---- 给上位机终端的第一条提示：现在到底是"真实作业"还是"调试测试"
        #   ★ 这两条**必须一眼能分出来**：下水前看一眼终端就知道有没有配错模式。
        if self.test.enabled:
            self.log('[AUV] 运行模式 = 调试测试模式（%s）；上位机可随时切回 ROV 接管'
                     % self.test.summary())
            self.notify.warn(CODE_TEST, '调试测试模式：%s；起始 %s（可随时切回 ROV）'
                             % (self.test.summary(), self.stage), stage=self.stage)
        else:
            self.log('[AUV] 运行模式 = 真实作业模式（验收）：跑全部 %d 段，'
                     '不接受上位机干预（仅"切模式"生效）' % len(self.order))
            self.notify.info(CODE_MODE, '真实作业模式（验收）：跑全部阶段，起始 %s，'
                                        '不接受上位机干预（仅"切模式"生效）' % self.stage,
                             stage=self.stage)
        self.log('[AUV] 监督指令策略 = %s（$AUVCTL %s）'
                 % (self.policy, '全部拒绝' if self.policy == 'ignore' else '受理'))
        # ---- 起始阶段也播报一次：上位机终端一进 AUV 就能看到"现在从第几段开始"
        self.notify.info(CODE_STAGE, '起始阶段 %s（%s）' % (self.stage, describe(self.stage)),
                         stage=self.stage)

    def _f(self, key, default):
        """读配置并转 float（键缺失/转失败都用缺省）"""
        try:
            return float(getattr(self.cfg, key, default))
        except (TypeError, ValueError):
            return float(default)

    def _auto_if(self, module, cls, cfg):
        """惰性自建观测接口（vision_if / depth_if / viskf_if）

        ★ 为什么用惰性 import 而不是模块顶部 import：
          auv_task 必须能在**没有板端模块**的 Windows 上跑离线回归，
          顶部 import 会直接 ImportError；放在这里导入失败只是"这一路降级"，不影响其它。
        """
        try:
            mod = __import__(module)
            return getattr(mod, cls)(cfg, log=self.log)
        except Exception:                                # 没这个模块 / 构造失败 → 该路不可用
            self.log('[AUV] 观测接口 %s.%s 未就绪 —— 对应判据将走降级路径' % (module, cls))
            return None

    def _first_active(self):
        """第一个会跑的阶段（测试模式下可能是被选中的那段）"""
        for n in self.order:
            if n in self.active:
                return n
        return DONE

    # ================================================================ 主入口
    def step(self, now, dt, tel=None):
        """跑一拍。返回控制量 dict，或 None（本拍不下发 / 已结束）

        与旧 mission.step(now, dt, tel) **签名与语义完全一致**。
        """
        ctx = self.ctx
        # ---- ★ 已被终止（上位机切回 ROV / $AUVCTL KILL）：本拍起不再下发任何运动帧
        #      返回 None = mode_auv 不组 0x09，控制权立刻回到上位机手里
        if self.killed:
            return None
        # ---- 读深度（唯一深度入口；读不到就用"不可用"的空读数，绝不抛）
        dep = {'ok': False, 'D': None, 'v_z': 0.0, 'clearance': None}
        if ctx.depth is not None:
            try:
                dep = ctx.depth.read(now)
            except Exception as e:                            # 深度源异常绝不能拖垮控制
                self.log('[AUV] 深度读取异常: %s' % e)
        ctx.now, ctx.dt, ctx.tel, ctx.dep = now, dt, tel, dep
        ctx.last_dep = dep                                    # ★ 供 AuvReport 快照
        ctx.seq += 1

        if self.total_t0 is None:
            self.total_t0 = now

        # ---- 首帧锚定：不锚定的话一进 AUV 就会"回水面 + 转向"
        if not self.anchored:
            d = dep.get('D') if dep.get('ok') else None
            if d is None:
                dcm = ctx.tel_depth_cm()
                d = (dcm / 100.0) if dcm is not None else 0.0
            ctx.depth_cm = float(d) * 100.0
            ctx.yaw_deg = ctx.yaw_est.now()
            self.anchored = True
            self.log('[AUV] 首帧锚定 depth=%.1fcm yaw=%.1f°' % (ctx.depth_cm, ctx.yaw_deg))

        # ---- 航向估计（有遥测自动升级为真闭环）
        ctx.yaw_est.update(dt, tel_yaw=ctx.tel_yaw(), target=ctx.yaw_deg, cfg_gf=ctx.gf)

        # ---- 全局预算（赛事 15 分钟硬上限）
        self._check_budget(now)

        # ---- 监督暂停：只发零推力悬停，不推进阶段（水池里用来"定住看现象"）
        if self.paused:
            return ctx.out(surge=0.0, sway=0.0,
                           note='[监督] 已暂停（$AUVCTL RESUME 恢复）')

        # ---- 分发当前阶段
        task = self.tasks.get(self.stage)
        if task is None:
            self.log('[AUV] ✗ 未知阶段 %s，停机' % self.stage)
            return None
        if not task.entered:
            ctx.t0 = now                                      # ★ 先复位计时，再 enter
            ctx.hold = {}
            ctx.stage = self.stage
            task.entered = True
            task.enter(ctx, now)
            # ★ 上位机终端提示：进入某一段（中文简述，人在岸上能看懂在干嘛）
            self.notify.info(CODE_STAGE, '进入 %s（%s）' % (self.stage, describe(self.stage)),
                             stage=self.stage)

        out = task.tick(ctx, now, dt)

        # ---- 跳转裁决（任务只放信号，真正跳在这里做，便于做过滤/强制保留）
        self._drain(now)

        # SURFACE 之后的收尾（DONE 首拍停推由 DoneTask 自己发）
        return out

    # ================================================================ 跳转
    def _drain(self, now):
        """处理任务放出的 pending 信号"""
        p = self.ctx.pending
        self.ctx.pending = None
        if not p:
            return
        kind, val = p
        if kind == 'goto':
            self._goto(val, now)
        elif kind == 'abort':
            self._abort(val, now)
        elif kind == 'finish':
            self._goto(DONE, now)

    def resolve(self, name):
        """解析目标阶段：被测试模式跳过的段会被**穿透**到下一个在跑的段"""
        if name in (DONE, 'ABORT'):
            return DONE
        i = index_of(self.plan, name)
        if i < 0:
            self.log('[AUV] ✗ 跳转到不存在的阶段 %s → 按 DONE 处理' % name)
            return DONE
        while i < len(self.order):
            if self.order[i] in self.active:
                return self.order[i]
            i += 1
        return DONE

    def _goto(self, name, now):
        """切段：退出旧任务 → 换 stage（新任务在下一拍 enter）"""
        tgt = self.resolve(name)
        old = self.stage
        if old == tgt:
            return
        t = self.tasks.get(old)
        if t is not None:
            try:
                t.exit(self.ctx, now)
            except Exception as e:                            # 收尾异常不许中断流程
                self.log('[AUV] 阶段 %s 退出异常: %s' % (old, e))
            t.entered = False                                 # 复位，允许再次进入
        if old == SURFACE:                                    # ★ 离开上浮阶段 = 确实浮过了
            self.surfaced = True
            if self.test.surface_then_rov:
                self.mode_request = int(getattr(self.cfg, 'MODE_ROV', 0))   # ★★ 必须是 cfg.MODE_ROV(=0)
                self.log('[AUV] 上浮完成 → 请求切回有线 ROV（模式 id=%d）' % self.mode_request)
        self.stage = tgt
        self.ctx.stage = tgt
        self.ctx.t0 = now
        self.ctx.hold = {}
        self.log('[AUV] %s → %s' % (old, tgt))

    def _abort(self, reason, now):
        """中止：打标记 → **强制走 SURFACE**（上浮不允许被配置关闭）"""
        self.ctx.abort_reason = str(reason)
        self.log('[AUV] ✗ ABORT: %s → 上浮' % reason)
        self.notify.error(CODE_ABORT, '中止：%s → 强制上浮' % reason, stage=self.stage)
        self._goto(SURFACE, now)

    # ================================================================ 终止（上位机接管）
    def kill(self, reason='PC_TAKEBACK'):
        """★ 立即终止任务执行 —— 之后 step() 恒返回 None，控制权交还上位机

        与 abort() 的区别（别混用）：
          abort() = **任务自己**判断"不行了" → 还要继续下发 0x09 把上浮走完；
          kill()  = **人**决定"立刻停手"   → 一帧都不再发，等上位机接管。

        触发场景：
          1. 上位机把模式切回 ROV（mode_auv.on_exit 调，★ 需求：直接杀死测试）；
          2. 上位机 $AUVCTL KILL（水池保命）；
          3. 中位机退出/停止。

        ★ 幂等：重复调用无害（只记第一次的原因）。
        ⚠ 这里**不**请求切模式：上位机已经自己切了，再请求反而会打架。
        """
        if self.killed:
            return True
        self.killed = True
        self.kill_reason = str(reason or 'PC_TAKEBACK')
        self.ctx.pending = None                              # 丢弃未裁决的跳转信号
        self.mode_request = None                             # 别再请求切 ROV（上位机已接管）
        self.notify.warn(CODE_KILLED, '自主任务已终止：%s（停止下发，交还遥控）'
                         % self.kill_reason, stage=self.stage)
        self.log('[AUV] ✗ 任务执行已终止（%s）→ 停止下发 0x09，交还遥控' % self.kill_reason)
        return True

    def killed_or_idle(self):
        """是否处于"已终止/不再动作"状态（外部查询用）"""
        return bool(self.killed)

    def _check_budget(self, now):
        """全局预算：超时 → 强制中止上浮（赛事 15 分钟未完成即结束）"""
        if self._budget_fired or self.budget_s <= 0 or self.total_t0 is None:
            return
        if (float(now) - float(self.total_t0)) < self.budget_s:
            return
        self._budget_fired = True
        if self.stage in (SURFACE, DONE):
            return
        self.log('[AUV] ⚠ 全局预算 %.0fs 用尽 → 强制上浮收尾' % self.budget_s)
        self.notify.error(CODE_BUDGET, '全局预算 %.0fs 用尽 → 强制上浮收尾' % self.budget_s,
                          stage=self.stage)
        self._abort('总预算 %.0fs 用尽' % self.budget_s, now)

    # ================================================================ 监督接口
    def on_auvctl(self, args, now=None):
        """★ $AUVCTL 监督指令入口（方案 §17.3）

        args: 指令令牌列表，如 ['HOLD'] / ['GOTO','SEEK_GATE_2'] / ['SKIP'] / ['ABORT'] / ['RESET']
        返回 True = 已受理。

        AUV_CMD_POLICY:
          ignore      → 全部拒绝（验收/比赛时用，防止误操作打断作业）
          supervise   → 受理流程监督类指令（默认；**不下发任何手动杆位**）
          passthrough → 额外允许手动杆位下发（极端调试，慎用）
        """
        now = now if now is not None else time.time()
        if not args:
            return False
        verb = str(args[0]).upper()
        if self.policy == 'ignore':
            self.log('[AUV] $AUVCTL %s 被拒绝（AUV_CMD_POLICY=ignore）' % verb)
            return False

        if verb == 'HOLD':
            self.paused = True
            self.log('[AUV] [监督] 暂停 —— 保持当前深度/航向，零推力悬停')
            return True
        if verb == 'RESUME':
            self.paused = False
            self.log('[AUV] [监督] 恢复')
            return True
        if verb == 'SKIP':
            t = self.tasks.get(self.stage)
            tgt = t.next_of(self.ctx) if t is not None else DONE
            self._goto(tgt, now)
            return True
        if verb == 'GOTO' and len(args) >= 2:
            tgt = str(args[1]).upper()
            if index_of(self.plan, tgt) < 0:
                self.log('[AUV] [监督] 未知阶段 %s' % tgt)
                return False
            if self.test.enabled and tgt not in self.active:
                self.log('[AUV] [监督] %s 不在测试选段内，已忽略' % tgt)
                return False
            self._goto(tgt, now)
            return True
        if verb == 'ABORT':
            self._abort('上位机 $AUVCTL ABORT', now)
            return True
        if verb == 'KILL':
            # ★ 与 ABORT 的区别：不走上浮，立刻停手（水池里要"马上不动"就用这个）
            self.kill('上位机 $AUVCTL KILL')
            return True
        if verb == 'RESET':
            self.reset(now)
            return True
        self.log('[AUV] [监督] 未知指令 %s' % verb)
        return False

    def pop_mode_request(self):
        """取走并清空模式切换请求（mode_auv 每拍调一次）

        ★ 返回的是 **dispatcher 模式 id**（cfg.MODE_ROV = 0，有线 ROV），
          不是 link_stm32 的 0x04 帧码 —— 这两个 MODE_ROV 不是同一个东西。
        """
        r = self.mode_request
        self.mode_request = None
        return r

    def reset(self, now=None):
        """整体重跑：清状态、复位所有任务、回到起始阶段"""
        now = now if now is not None else time.time()
        for t in self.tasks.values():
            t.entered = False
        self.ctx.pending = None
        self.ctx.hold = {}
        self.ctx.abort_reason = ''
        self.ctx.t0 = now
        self.ctx.scan_base_cm = None
        self.anchored = False
        self.total_t0 = None
        self._budget_fired = False
        self.surfaced = False
        self.mode_request = None
        self.paused = False
        self.killed = False                 # RESET 是显式重跑指令 → 解除终止态
        self.kill_reason = ''
        self.stage = self._first_active()
        self.ctx.stage = self.stage
        self.log('[AUV] 已重置 → 从 %s 重跑' % self.stage)

    # ================================================================ 对外属性代理
    # ★ AuvReport.make_snapshot(now, cmd, mission, tel) 用 getattr 取这些名字，
    #   名字与旧 Mission 完全一致 —— auv_report.py 一行都不用改。
    @property
    def t0(self):
        """当前阶段的进入时刻"""
        return self.ctx.t0

    @property
    def seq(self):
        """已跑拍数"""
        return self.ctx.seq

    @property
    def last_dep(self):
        return self.ctx.last_dep

    @property
    def last_obs(self):
        return self.ctx.last_obs

    @property
    def last_obs_cam(self):
        return self.ctx.last_obs_cam

    @property
    def last_obs_ts(self):
        return self.ctx.last_obs_ts

    @property
    def last_vk(self):
        return self.ctx.last_vk

    @property
    def abort_reason(self):
        return self.ctx.abort_reason

    @abort_reason.setter
    def abort_reason(self, v):
        self.ctx.abort_reason = v

    def summary(self):
        """一行状态摘要（日志/排障用）"""
        if self.killed:
            return '已终止(%s) stage=%s seq=%d' % (self.kill_reason, self.stage, self.ctx.seq)
        return 'stage=%s t=%.1fs seq=%d%s' % (
            self.stage, (time.time() - self.ctx.t0), self.ctx.seq,
            (' 中止:' + self.ctx.abort_reason) if self.ctx.abort_reason else '')
