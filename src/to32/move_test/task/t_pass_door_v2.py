# -*- coding: utf-8 -*-
"""t_pass_door_v2.py —— 穿门任务 v2（2026-10-11 四门过门：门循环 + 高度计突变判完成）

流程（内部状态机）：
    Step0. 视觉就绪（仅一次）：前视相机由 front.py 常驻采集；进入本阶段 Mission
           即发布 momo_stage.json（Stage.NAME → stage_model.STAGE_MODELS 热切换 YOLO
           模型）。本步等「前视检测文件新鲜连续 3 拍」→ 确认链路与模型切换完成。
    门循环（AUV_PASS_DOOR_V2_GATES，每门独立 st dict，互不残留）：
        每门 sub=0 转向定深：相对当前航向转 turn（右正左负，lock_turn_target 每次
            重新锁存，门间误差不累积）+ 下发定深目标；转到位后定深（dive_step 判融合
            clearance ≈ height 到位）。
        每门 sub=1 横移对中：起步锁存航向；无门帧按固定方向横移找门（永不超时，
            60s→HOLD_FAULT 已按 2026-10-11 要求移除）；见门帧走比例伺服
            （sway_align_step），门中心 x 距画面中心 ≤ PX_TOL 且稳定 HOLD_N 帧 → 对准。
        每门 sub=2 前冲判完成：锁存航向纯前进（恒 surge，sway=0）；高度计 B、C
            （obs.AltIF 读 momo_alt.json 原始 mm）均出现一次突变（读数 < 本门起步锁定
            的基线 − ALT_DROP_MM，连续 ALT_HIT_N 拍）后的 DONE_DELAY_S（1s）→ 过门；
            兜底：运行 RUSH_DUR_S（15s）未检出突变 → 自动过门。
    门4 完成（含其 15s 兜底）→ 全任务完成，阶段结束。

★ 兜底矩阵（2026-10-11 用户确认）：
    Step0/每门 Step1（转向定深）/每门 Step2（对中）无兜底 —— 判据失效即持续执行；
    每门 Step3（前冲）有 15s 兜底；整体无总超时，门4 Step3 的 15s 兜底即全任务兜底。

任务侧不直接碰共享内存：视觉走 ctx.vision（obs.VisionIF，fresh/poll），
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


class PassDoorV2(Stage):
    """穿门 v2 四门：Step0 视觉就绪（一次）→ 门循环（每门 转向定深 → 对中 → 前冲+B/C突变判完成）。"""

    NAME = getattr(TC, 'AUV_PASS_DOOR_V2_STAGE', 'PassDoorV2')  # 阶段名：日志 + 0x09 stage 字段 + stage_model 模型映射键

    def __init__(self, ctx, alt=None):
        super().__init__(ctx)
        self.alt = alt or AltIF(shm_dir=getattr(ctx.cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.gates = []                       # 门参数表（enter 时从 TC 读取）
        self.n_gates = 0
        self.idx = 0                          # 0 = Step0 视觉就绪；1 = 门循环
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

    def _hold_cmd(self, note):
        """零推力保持帧：锁当前航向 + 保持门1 定深目标（Step0 等视觉用）。"""
        height = float(self.gates[0]['height'])
        return t_function._cmd(self.NAME, note,
                               t_function._yaw_hold(self.sts[0], self.ctx),
                               t_function._depth_out(self.sts[0], height),
                               surge=0.0, sway=0.0)

    # ---------------- Stage 契约（enter / step） ----------------
    def enter(self, now):
        self.gates = list(getattr(TC, 'AUV_PASS_DOOR_V2_GATES', []))
        if not self.gates:
            raise ValueError('AUV_PASS_DOOR_V2_GATES 为空：四门参数未配置')
        self.n_gates = len(self.gates)
        self.idx = 0
        self.gate = 0
        self.sub = 0
        self.sts = [{} for _ in range(self.n_gates)]
        self._now = now
        self._dt = 0.0
        self.log('穿门 v2 启动：%d 门循环（%s）'
                 % (self.n_gates, ' → '.join('%s%+.0f°/%dcm' % ('门%d ' % (i + 1), g['turn'], g['height'])
                                              for i, g in enumerate(self.gates))))

    def step(self, now, dt):
        self._now = now                       # 刷新时间戳缓存
        self._dt = dt                         # 刷新拍间隔缓存
        if self.idx == 0:                     # Step0（仅一次）：等视觉链路 + 模型切换就绪
            cmd = self.step0_wait_vision()
            if cmd is not None:
                return cmd
            self.idx = 1                      # 视觉就绪 → 进入门循环
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

    #=====Step0. 视觉就绪 + 模型切换（仅一次） ==============
    def step0_wait_vision(self):
        """确认前视检测链路就绪（front.py 常驻采集；模型由 stage_model 按 NAME 热切换）。

        完成判据：momo_det_front.json 文件新鲜（mtime ≤ AUV_DET_STALE_S）连续 3 拍。
        模型切换/加载期间 front 不更新 JSON → fresh=False；恢复写帧 → fresh=True，
        因此本判据同时覆盖"模型切换完成"。无 fresh → 持续保持等待（无兜底，符合口径）。
        """
        st = self.sts[0]
        now = self._now
        cam = str(getattr(TC, 'AUV_PASS_DOOR_V2_CAM', 'front'))
        if self.ctx.vision.fresh(cam, now):
            st['ok'] = st.get('ok', 0) + 1
            if st['ok'] >= 3:
                self.log('Step0 完成：前视检测链路就绪（阶段 %s 已发布，front 按 stage_model 热切换模型）' % self.NAME)
                return None
            return self._hold_cmd('等视觉链路稳定 ok=%d/3' % st['ok'])
        st['ok'] = 0
        self._say(st, '前视检测未就绪（模型切换/相机启动中，或 front 未起）')
        return self._hold_cmd('前视检测未就绪')

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
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测，等待(不以0兜底)' % self._gname())
            cmd = t_function.turn_step(self.ctx, tst, now, dt, target_yaw_deg=tgt,
                                       target_height_cm=height, stage=self.NAME)
            if cmd is not None:
                return cmd
            gst['turned'] = True
            self.log('%s 转向完成：%+.0f° → %.1f°，开始定深 %.0fcm(离底)' % (self._gname(), turn_deg, tgt, height))
        return t_function.dive_step(self.ctx, gst.setdefault('dive', {}), now, dt,
                                    target_height_cm=height, stage=self.NAME)

    #=====门循环 sub=1：横移对中 ==============
    def _align(self, gst, cfg):
        """横移对中（方向可配置，默认向右）：无门帧按固定方向横移找门（永不超时，
        60s→HOLD_FAULT 已移除）；见门帧走比例伺服（sway_align_step），门中心 x 距
        画面中心 ≤ PX_TOL 且稳定 HOLD_N 个新检测帧 → 对准完成。
        """
        now = self._now
        height = float(cfg['height'])
        cam = str(getattr(TC, 'AUV_PASS_DOOR_V2_CAM', 'front'))
        want = str(getattr(TC, 'AUV_PASS_DOOR_V2_WANT', 'gate'))
        img_w = float(getattr(TC, 'AUV_IMG_W', 1280.0))
        center = 0.5 * img_w
        dirn = 1.0 if float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_DIR', 1.0)) >= 0 else -1.0
        sway_thr = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_THRUST', 0.3))

        # 起步：锁存本门对中航向（全程不动，只横移）
        if gst.get('yaw_ref') is None:
            y = t_function._tel_f(self.ctx, 'actual_yaw')
            if y is None:
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测，等待(不以0兜底)' % self._gname())
            gst['yaw_ref'] = float(y)
            gst['last_seen'] = None
            gst['ok_cnt'] = 0
            self.log('%s 锁存航向 %.1f°，横移对中（%s，%.0fpx/%.0f帧）'
                     % (self._gname(), gst['yaw_ref'], '右' if dirn > 0 else '左',
                        float(getattr(TC, 'AUV_PASS_DOOR_V2_PX_TOL', 20.0)),
                        float(getattr(TC, 'AUV_PASS_DOOR_V2_HOLD_N', 10))))

        obs = self.ctx.vision.poll(cam, want, now)
        if obs is None:                                   # 无门帧：固定方向横移找门（对准计数清零，永不超时）
            if gst.get('last_seen') is None:
                gst['last_seen'] = now
            gst['ok_cnt'] = 0
            lost = now - gst['last_seen']
            self._say(gst, '%s 横移找门（无门帧 %.1fs）' % (self._gname(), lost))
            return t_function._cmd(self.NAME, '%s 横移找门(无门) %.1fs' % (self._gname(), lost),
                                   gst['yaw_ref'], t_function._depth_out(gst, height),
                                   sway=dirn * sway_thr)
        gst['last_seen'] = now                             # 见门：刷新丢门计时
        ex_px = obs['cx'] - center                        # 门中心 x 距画面中心像素误差（右正）
        return t_function.sway_align_step(
            self.ctx, gst, now, self._dt, ex_px, height, gst['yaw_ref'],
            px_tol=float(getattr(TC, 'AUV_PASS_DOOR_V2_PX_TOL', 20.0)),
            hold_n=int(getattr(TC, 'AUV_PASS_DOOR_V2_HOLD_N', 10)),
            stage=self.NAME)

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
                return t_function.wait_cmd(self.NAME, '%s 无 yaw 遥测，等待(不以0兜底)' % self._gname())
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
