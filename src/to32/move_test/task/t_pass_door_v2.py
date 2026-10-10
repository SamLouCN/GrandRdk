# -*- coding: utf-8 -*-
"""t_pass_door_v2.py —— 穿门任务 v2（2026-10-11 四门过门：门循环 + 高度计突变判完成）

流程（内部状态机）：
    门循环（AUV_PASS_DOOR_V2_GATES，每门独立 st dict，互不残留）：
        每门 sub=0 转向定深：相对当前航向转 turn（右正左负，lock_turn_target 每次
            重新锁存，门间误差不累积）+ 下发定深目标；转到位后定深（dive_step 判融合
            clearance ≈ height 到位）。
        每门 sub=1 横移对中：起步锁存航向；无有效目标帧分三级——从未见过门/超过
            LOST_S 视为真丢门 → 固定方向横移找门（永不超时，60s→HOLD_FAULT 已移除）；
            见过门且短暂丢帧（≤ LOST_S）→ 保持上一帧运动状态（sway=last_sway，
            ok_cnt 不清零），用下一有效帧修复；见门帧走比例伺服（sway_align_step），
            门中心 x 距画面中心 ≤ PX_TOL 且稳定 HOLD_N 帧 → 对准。
        每门 sub=2 前冲判完成：锁存航向纯前进（恒 surge，sway=0）；高度计 B、C
            （obs.AltIF 读 momo_alt.json 原始 mm）均出现一次突变（读数 < 本门起步锁定
            的基线 − ALT_DROP_MM，连续 ALT_HIT_N 拍）后的 DONE_DELAY_S（1s）→ 过门；
            兜底：运行 RUSH_DUR_S（15s）未检出突变 → 自动过门。
    门4 完成（含其 15s 兜底）→ 全任务完成，阶段结束。

★ 兜底矩阵（2026-10-11 用户确认）：
    每门 Step1（转向定深）/每门 Step2（对中）无兜底 —— 判据失效即持续执行；
    每门 Step3（前冲）有 15s 兜底；整体无总超时，门4 Step3 的 15s 兜底即全任务兜底。

任务侧不直接碰共享内存：视觉走 ctx.vision（obs.VisionIF，poll），
高度计走自建 AltIF（obs.AltIF，读原始 B/C），定深走 ctx.depth（obs.DepthIF）。
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                           # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC                               # 任务参数（AUV_PASS_DOOR_V2_* 全部从这里 getattr 读取）
from mission import Stage                              # 阶段基类（enter/step 契约）
import t_function                                      # 运动原语库：turn_step/dive_step/sway_align_step/_cmd/_depth_out/_tel_f
from obs import AltIF                                  # 高度计原始通道接口（B/C 突变判据专用）


def _det_img_w(ctx):
    """检测画面宽(px)：统一口径的唯一取值口（2026-10-10）。

    优先用观测接口自己持有的画面宽（obs.VisionIF.w ← task_config.AUV_IMG_W），
    取不到才回落 task_config；绝不在这里写死数字 —— 混用"相机采集分辨率"与
    "检测框坐标系"正是过门横移只朝一个方向推的根因。
    """
    w = getattr(getattr(ctx, 'vision', None), 'w', None)
    if w is None:
        w = getattr(TC, 'AUV_IMG_W', 640.0)
    return float(w)


class PassDoorV2(Stage):
    """穿门 v2 四门：门循环（每门 转向定深 → 对中 → 前冲+B/C突变判完成）。"""

    NAME = getattr(TC, 'AUV_PASS_DOOR_V2_STAGE', 'PassDoorV2')  # 阶段名：日志 + 0x09 stage 字段 + stage_model 模型映射键

    def __init__(self, ctx, alt=None):
        super().__init__(ctx)
        self.alt = alt or AltIF(shm_dir=getattr(ctx.cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.gates = []                       # 门参数表（enter 时从 TC 读取）
        self.n_gates = 0
        self.gate = 0                         # 当前门（0-based）
        self.sub = 0                          # 门内子步骤：0 转向定深 / 1 对中 / 2 前冲
        self.sts = []                         # 每门独立 st dict（下标 = gate）

    # ---------------- 辅助 ----------------
    def log(self, msg):
        """阶段日志（走 ctx.say → mission 日志函数）。"""
        self.ctx.say(msg)

    def _say(self, st, msg):
        """节流日志（同 st 内按 AUV_LOG_EVERY_S 限频）。"""
        t_function._say_throttled(self.ctx, st, self._now, msg)

    def _gname(self):
        """当前门显示名（1-based）。"""
        return '门%d' % (self.gate + 1)

    def _poll_gate(self, gst, now):
        """取本拍的门观测（**同一拍内只真正 poll 一次**，结果缓存复用）。

        为什么必须缓存：`obs.VisionIF.poll` 带**同一帧去重**（同一 frame 号第二次调用返回
        None）。如果 sub=0 的"是否已识别到门"判断先 poll 一次，紧接着 sub=1 的对中再 poll
        就会拿到 None，被误判成"没看到门"→ 走找门分支。缓存把同一拍的两个消费者对齐。
        """
        if gst.get('_obs_t') == now:
            return gst.get('_obs')
        cam = str(getattr(TC, 'AUV_PASS_DOOR_V2_CAM', 'front'))
        want = str(getattr(TC, 'AUV_PASS_DOOR_V2_WANT', 'gate'))
        o = self.ctx.vision.poll(cam, want, now)
        gst['_obs_t'] = now
        gst['_obs'] = o
        return o

    # ---------------- Stage 契约（enter / step） ----------------
    def enter(self, now):
        self.gates = list(getattr(TC, 'AUV_PASS_DOOR_V2_GATES', []))
        if not self.gates:
            raise ValueError('AUV_PASS_DOOR_V2_GATES 为空：四门参数未配置')
        self.n_gates = len(self.gates)
        self.gate = 0
        self.sub = 0
        self.sts = [{} for _ in range(self.n_gates)]
        self._now = now
        self._dt = 0.0
        self.log('穿门 v2 启动：%d 门循环（%s）'
                 % (self.n_gates, ' → '.join('%s%+.0f°/%dcm' % ('门%d ' % (i + 1), g['turn'], g['height'])
                                              for i, g in enumerate(self.gates))))
        self._enter_selfcheck(now)

    def _enter_selfcheck(self, now):
        """进入时的前置自检（[2026-10-10 增]）：把"为什么不动的先决条件"提前说清楚。

        本阶段的每门 sub=0 是**相对转向**（目标角 = 当前 yaw + 门转角），所以**没有 yaw
        遥测就没有目标角** —— 按设计本拍不下发 0x09（宁停勿猜）→ 现场表现为"机器一动不动，
        又没有明显报错"。同理 sub=1 对中依赖前视检测帧、sub=0 的定深依赖融合深度。
        这里一次性播报三条链路是否可用，缺哪条直接点名缺哪条。
        """
        y = t_function._tel_f(self.ctx, 'actual_yaw')
        d = {}
        try:
            d = self.ctx.depth.read(now) or {}
        except Exception:
            d = {}
        depth_ok = bool(d.get('ok'))
        depth_v = d.get('clearance')
        self.log('%s 前置自检：yaw遥测=%s 融合深度=%s(clearance=%s)'
                 % (self.NAME,
                    ('%.1f°' % y) if y is not None else '不可用',
                    'ok' if depth_ok else ('超期/缺失' if d else '不可用'),
                    ('%.3fm' % depth_v) if isinstance(depth_v, (int, float)) else 'N/A'))
        if y is None:
            self.log('%s [WARN] 无 yaw 遥测 → sub=0 无法锁存转向目标角：本阶段将一直等待、'
                     '不下发任何 0x09（机器不会动）。先查下位机串口链路与 $TEL 是否正常。' % self.NAME)
        if not depth_ok:
            self.log('%s [WARN] 融合深度不可用 → sub=0 的定深判据永不满足，转到角度后会停在原地'
                     '（需 depth_kalman 在跑）。' % self.NAME)

    def step(self, now, dt):
        self._now = now                       # 刷新时间戳缓存
        self._dt = dt                         # 刷新拍间隔缓存
        while self.gate < self.n_gates:       # 门循环（有界：每门 ≤3 子步骤，最多 n_gates 门）
            gst = self.sts[self.gate]
            cfg = self.gates[self.gate]
            if self.sub == 0:
                cmd = self._turn_dive(gst, cfg)
            elif self.sub == 1:
                cmd = self._align(gst, cfg)
            else:
                cmd = self._pass(gst, cfg)
            if cmd is not None:
                return cmd
            if self.sub < 2:                  # 本子步骤完成 → 下一子步骤（同拍继续）
                self.sub += 1
                continue
            self.log('%s 过门完成（B/C 突变判完成）' % self._gname())
            self.gate += 1                    # 下一门
            self.sub = 0
            if self.gate >= self.n_gates:
                self.log('全部 %d 门完成 —— 穿门 v2 任务结束' % self.n_gates)
                return None
            continue                          # 下一门同拍起步
        return None

    #=====门循环 sub=0：转向定深 ==============
    def _turn_dive(self, gst, cfg):
        """本门转向：相对当前航向转 cfg['turn']（右正左负；lock_turn_target 每次重新
        锁存，门间误差不累积）+ 下发定深目标 cfg['height']；转到位后定深到位
        （dive_step 判融合 clearance）。★ 无 yaw 遥测 / 融合深度源不可用 → 持续执行不完成。
        """
        now, dt = self._now, self._dt
        height = float(cfg['height'])
        turn_deg = float(cfg['turn'])
        tst = gst.setdefault('turn', {})                  # 转向子步骤独立 st（与 dive 分离，避免 t0/ok_cnt 残留）
        if not gst.get('turned'):
            tgt = t_function.lock_turn_target(self.ctx, tst, now, turn_deg, stage=self.NAME)
            if tgt is None:
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测：本拍不下发 0x09（等遥测，不以 0° 兜底）' % self._gname())
            cmd = t_function.turn_step(self.ctx, tst, now, dt, target_yaw_deg=tgt,
                                       target_height_cm=height, stage=self.NAME)
            if cmd is not None:
                return cmd
            gst['turned'] = True
            self.log('%s 转向完成：%+.0f° → %.1f°，开始定深 %.0fcm(离底)' % (self._gname(), turn_deg, tgt, height))
        # ---- sub=0 后半：定深到本门高度 ----
        d = t_function.dive_step(self.ctx, gst.setdefault('dive', {}), now, dt,
                                 target_height_cm=height, stage=self.NAME)
        if d is None:                                     # 定深到位 → 正常进 sub=1
            return None
        # [2026-10-10] ★「看到门就允许对中」：定深判据依赖融合深度（depth_kalman / 池深标定），
        #   它一旦不可用就**永不完成**（设计上无兜底）—— 此时哪怕门就在正前方，也永远轮不到
        #   "横移对中"，现场表现就是"YOLO 已经识别到门了，却不进对准逻辑"。
        #   故：转向已到位 + 已识别到门 + ALIGN_EARLY → 立刻转 sub=1；定深不中断，
        #   对中/前冲每拍仍会下发定深目标（_align/_pass 里的 _depth_out）。
        if bool(getattr(TC, 'AUV_PASS_DOOR_V2_ALIGN_EARLY', True)):
            o = self._poll_gate(gst, now)                  # 与 _align 共用同一拍观测（见 _poll_gate）
            if o is not None:
                gst['_align_early'] = True
                self.log('%s 已识别到门(cx=%.0f) → 提前进入横移对中（定深目标继续下发，不等定深到位）'
                         % (self._gname(), float(o['cx'])))
                return None                                # 完成 sub=0 → 同拍进 sub=1
            self._say(gst, '%s 转向已到位：尚未识别到门 → 继续定深（或 ALIGN_EARLY 关时等定深到位）'
                           % self._gname())
        return d

    #=====门循环 sub=1：横移对中 ==============
    def _align(self, gst, cfg):
        """横移对中（[2026-10-10 重写] 视觉伺服唯一驱动）：sway 由「YOLO 框中心 x − 画面中心 x」
        的比例伺服给出，方向由误差符号唯一决定（框偏右 → 右移，任务系右为正），带换向滞回，
        因此表现为**朝一个方向持续横移到对中**，而不是在中心附近来回抖。
        无有效帧：按**最后已知方向**继续找门（不是固定方向盲扫），永不超时。

        改前的问题（根因）：`center = 0.5·AUV_IMG_W`，而 AUV_IMG_W 写的是相机采集宽 1280，
        检测框实际在 640 空间（中心 320）→ `ex = cx − 640` 恒为负 ⇒ 只会朝一个方向横移、
        永远对不正；归一化分母也错一倍（增益等效减半）。现口径统一到 task_config.AUV_IMG_W=640。
        """
        now = self._now
        height = float(cfg['height'])
        kp = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_KP', 1.0))
        sway_max = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_THRUST', 0.3))
        sway_min = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_MIN_THRUST', 0.15))
        px_tol = float(getattr(TC, 'AUV_PASS_DOOR_V2_PX_TOL', 20.0))
        hyst_px = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_HYST_PX', 24.0))
        hold_n = int(getattr(TC, 'AUV_PASS_DOOR_V2_HOLD_N', 10))
        img_w = float(_det_img_w(self.ctx) or getattr(TC, 'AUV_IMG_W', 640.0))
        center = 0.5 * img_w
        # 状态键兜底（正常路径在下面的锁存块里初始化；这里保证任何入口进来都不 KeyError）
        gst.setdefault('ok_cnt', 0)
        gst.setdefault('last_seen', None)
        gst.setdefault('last_sway', 0.0)
        gst.setdefault('dir', 1.0 if float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_DIR', 1.0)) >= 0 else -1.0)

        # 起步：锁存本门对中航向（全程不动，只横移）+ 初始化方向（仅"从未见门"时用）
        if gst.get('yaw_ref') is None:
            y = t_function._tel_f(self.ctx, 'actual_yaw')
            if y is None:
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测：本拍不下发 0x09（等遥测，不以 0° 兜底）' % self._gname())
            gst['yaw_ref'] = float(y)
            gst['last_seen'] = None            # 最后见有效门帧时刻（None=从未见过门）
            gst['ok_cnt'] = 0
            gst['dir'] = 1.0 if float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_DIR', 1.0)) >= 0 else -1.0
            gst['last_sway'] = 0.0
            self.log('%s 锁存航向 %.1f°，横移对中：画面中心 x=%.0f，容差 %.0fpx/保持 %d 帧，'
                     '滞回 %.0fpx（口径 AUV_IMG_W）'
                     % (self._gname(), gst['yaw_ref'], center, px_tol, hold_n, hyst_px))

        lost_s = float(getattr(TC, 'AUV_PASS_DOOR_V2_LOST_S', 0.5))
        o = self._poll_gate(gst, now)                      # 同一拍复用 sub=0 的观测
        if o is None:                                     # 无新有效目标帧
            if gst.get('last_seen') is None:               # 从未见过门：按初始方向找门
                note = '%s 未见门：按 %s 方向横移找门 (sway=%.2f)' % (
                    self._gname(), '右' if gst['dir'] > 0 else '左', gst['dir'] * sway_max)
                self._say(gst, note)
                return t_function._cmd(self.NAME, note, gst['yaw_ref'],
                                       t_function._depth_out(gst, height), sway=gst['dir'] * sway_max)
            lost = now - gst['last_seen']
            if lost <= lost_s:
                # 短暂丢帧（**含 VisionIF 同一帧去重造成的隔拍 None**）：保持上一拍输出，
                #   ★ ok_cnt 绝不清零 —— 判到位要的是"连续 N 个新检测帧"，清零会让
                #   对中永远攒不满 hold_n（2026-10-10 修：原来清零写在这条分支之前）。
                keep = float(gst.get('last_sway') or 0.0)
                note = '%s 丢帧保持 %.2fs sway=%+.2f' % (self._gname(), lost, keep)
                self._say(gst, note)
                return t_function._cmd(self.NAME, note, gst['yaw_ref'],
                                       t_function._depth_out(gst, height), sway=keep)
            gst['ok_cnt'] = 0                              # 真丢门（>LOST_S）才清零计数器
            note = '%s 丢门 %.1fs：按最后方向 %s 继续找门 (sway=%.2f)' % (
                self._gname(), lost, '右' if gst['dir'] > 0 else '左', gst['dir'] * sway_max)
            self._say(gst, note)
            return t_function._cmd(self.NAME, note, gst['yaw_ref'],
                                   t_function._depth_out(gst, height), sway=gst['dir'] * sway_max)

        gst['last_seen'] = now                             # 见门：刷新丢门计时
        # 口径校正：obs 按相机分别给出本帧的画面宽（见 obs.VisionIF：优先读写端发布的
        #   img_w/img_h，回落该相机配置）→ 有就用它，前/下摄不同分辨率也不会算错中心。
        try:
            _w = float(o.get('img_w'))
            if _w > 0:
                img_w, center = _w, 0.5 * _w
        except (TypeError, ValueError):
            pass
        cx = float(o['cx'])
        ex = cx - center                                   # 正 = 门在画面右侧
        # ---- 方向锁 + 换向滞回：只有误差翻到另一侧且越过 (容差+滞回) 才允许换向（防抖）----
        want_dir = 1.0 if ex > 0 else -1.0
        gst.setdefault('dir', 1.0 if float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_DIR', 1.0)) >= 0 else -1.0)
        if want_dir != gst['dir'] and abs(ex) > (px_tol + hyst_px):
            gst['dir'] = want_dir
            self._say(gst, '%s 换向→%s（ex=%+.0fpx 越过容差+滞回 %.0fpx）'
                           % (self._gname(), '右' if want_dir > 0 else '左', ex, px_tol + hyst_px))
        # ---- 比例伺服输出（幅度保底，保证推得动）；进入容差带则停横移，避免持续推力漂移 ----
        if abs(ex) <= px_tol:
            sway = 0.0
            gst['ok_cnt'] += 1
        else:
            sway = (kp * ex / center) if center > 0 else 0.0
            sway = max(-sway_max, min(sway_max, sway))
            if abs(sway) < sway_min:
                sway = gst['dir'] * sway_min
            gst['ok_cnt'] = 0
        gst['last_sway'] = sway
        gst.setdefault('_log_ts', 0.0)

        if gst['ok_cnt'] >= hold_n:
            self.log('%s 对中完成：框中心 x=%.0f，距画面中心 %+.0fpx（≤%.0fpx 保持 %d 帧）'
                     % (self._gname(), cx, ex, px_tol, hold_n))
            return None

        move = '带内停横移' if sway == 0.0 else ('右移' if sway > 0 else '左移')
        note = '%s 对中 cx=%.0f 中心=%.0f ex=%+.0fpx %s sway=%+.2f ok=%d/%d' % (
            self._gname(), cx, center, ex, move, sway, gst['ok_cnt'], hold_n)
        self._say(gst, note)
        return t_function._cmd(self.NAME, note, gst['yaw_ref'],
                               t_function._depth_out(gst, height), sway=sway)

    #=====门循环 sub=2：前冲 + 高度计突变判完成 ==============
    def _pass(self, gst, cfg):
        """本门固定航向（锁存起步航向）纯前进（恒 surge，sway=0），监视高度计 B、C。

        完成判据：B、C 均出现一次突变（读数 < 本门起步锁定的基线 − ALT_DROP_MM，
        连续 ALT_HIT_N 拍）后延迟 DONE_DELAY_S（默认 1s）→ 过门完成。
        兜底：运行 RUSH_DUR_S（默认 15s）未检出突变 → 自动过门。
        基线 = 本门进入后前 ALT_BASE_N 拍有效均值（@5Hz≈2s，锁存后不再滑动，不跨门复用）。
        """
        now = self._now
        surge = float(getattr(TC, 'AUV_PASS_DOOR_V2_SURGE', 0.5))
        rush_s = float(getattr(TC, 'AUV_PASS_DOOR_V2_RUSH_DUR_S', 15.0))
        delay_s = float(getattr(TC, 'AUV_PASS_DOOR_V2_DONE_DELAY_S', 1.0))

        if gst.get('t0') is None:                         # 起步：锁存航向 + 前冲计时 + 本门高度计基线容器
            y = t_function._tel_f(self.ctx, 'actual_yaw')
            if y is None:
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测：本拍不下发 0x09（等遥测，不以 0° 兜底）' % self._gname())
            gst['yaw_ref'] = float(y)
            gst['t0'] = now
            gst['alt'] = dict(B=dict(buf=[], base=None, hit=0, done_at=None),
                              C=dict(buf=[], base=None, hit=0, done_at=None))
            self.log('%s 锁死航向 %.1f°，前冲过门（B/C 突变 +%.0fs 判完成；%.0fs 兜底）'
                     % (self._gname(), gst['yaw_ref'], delay_s, rush_s))

        elapsed = now - gst['t0']
        if elapsed >= rush_s:                             # 兜底：到时未检出突变 → 自动过门
            self.log('%s 兜底完成：%.1fs 内未检出 B/C 突变（%s %s）'
                     % (self._gname(), elapsed, self._alt_state(gst, 'B'), self._alt_state(gst, 'C')))
            return None

        done_ts, msg = self._alt_check(gst, self.alt.read(now), now)
        if done_ts is not None and (now - done_ts) >= delay_s:   # 两者均突变后延迟 → 过门
            self.log('%s 完成：%s 后 %.1fs，判已过门' % (self._gname(), msg, now - done_ts))
            return None

        note = '%s 前冲 t=%.1fs' % (self._gname(), elapsed)
        if done_ts is not None:
            note += '（B/C均突变，等 %.1fs）' % (delay_s - (now - done_ts))
        return t_function._cmd(self.NAME, note, gst['yaw_ref'],
                               t_function._depth_out(gst, float(cfg['height'])), surge=surge)

    def _alt_state(self, gst, ch):
        """突变判据状态摘要（日志用）：基线/命中/完成时刻。"""
        s = gst['alt'][ch]
        base = ('%.0fmm' % s['base']) if s['base'] is not None else '未锁'
        done = ('@%.1f' % s['done_at']) if s['done_at'] is not None else '未触发'
        return '%s base=%s hit=%d%s' % (ch, base, s['hit'], done)

    def _alt_check(self, gst, d, now):
        """高度计 B/C 突变检测（本门独立基线）。返回 (done_ts|None, 说明|None)。

        每通道独立：攒 ALT_BASE_N 拍有效读数锁基线（均值，锁后不再滑动）；
        之后读数 < 基线 − ALT_DROP_MM 连续 ALT_HIT_N 拍 → 该通道"突变发生"
        （记录首次完成时刻）。B、C 均发生 → 返回 max(done_at)。
        数据不可用（ok=False / 通道无有效读数）→ 该拍不判（不累计也不清零）。
        """
        drop = float(getattr(TC, 'AUV_PASS_DOOR_V2_ALT_DROP_MM', 80.0))
        base_n = int(getattr(TC, 'AUV_PASS_DOOR_V2_ALT_BASE_N', 10))
        hit_n = int(getattr(TC, 'AUV_PASS_DOOR_V2_ALT_HIT_N', 3))
        if not d.get('ok'):
            return None, None
        for ch in ('B', 'C'):
            v = (d.get('ch') or {}).get(ch)
            mm = v.get('mm') if isinstance(v, dict) else None
            status = v.get('status') if isinstance(v, dict) else ''
            if mm is None or status != 'OK':
                continue                                  # 无效拍：不累计不清零
            s = gst['alt'][ch]
            if s['base'] is None:
                s['buf'].append(mm)
                if len(s['buf']) >= base_n:
                    s['base'] = sum(s['buf'][-base_n:]) / base_n
                    s['buf'] = []
            elif mm < s['base'] - drop:
                s['hit'] += 1
                if s['hit'] >= hit_n and s['done_at'] is None:
                    s['done_at'] = now
            else:
                s['hit'] = 0
        if gst['alt']['B']['done_at'] is not None and gst['alt']['C']['done_at'] is not None:
            return (max(gst['alt']['B']['done_at'], gst['alt']['C']['done_at']),
                    'B 突变@%.2f C 突变@%.2f' % (gst['alt']['B']['done_at'], gst['alt']['C']['done_at']))
        return None, None


# 阶段注册表：task_config.PASS_DOOR_V2_TABLE / test_mode TEST_TABLE 引用
# （一项 = 整个穿门 v2 四门任务；未挂正式 STAGE_TABLE，测试时在 test_config.py 用
#   TEST_TABLE = PASS_DOOR_V2_TABLE 即可）
PASS_DOOR_V2_TABLE = [PassDoorV2]
