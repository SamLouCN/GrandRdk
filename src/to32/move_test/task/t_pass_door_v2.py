# -*- coding: utf-8 -*-
"""t_pass_door_v2.py —— 穿门任务 v2（2026-10-10 新过门流程：对中 → 前冲 + 高度计突变判完成）

流程（内部子状态机，idx 0..4 对应用户 Step0..4）：
    Step0. 视觉就绪：前视相机由 front.py 常驻采集；进入本阶段 Mission 即发布
           momo_stage.json（Stage.NAME → stage_model.STAGE_MODELS 热切换 YOLO 模型）。
           本步等「前视检测文件新鲜连续 3 拍」→ 确认链路与模型切换完成。
    Step1. 旋转固定角度（AUV_PASS_DOOR_V2_TURN_DEG，默认 60°，右为正，相对当前航向，
           可快速配置）并定深至离底 AUV_PASS_DOOR_V2_HEIGHT_CM（默认 45cm）。
    Step2. 横移对中（★ 2026-10-10 重写：**直接假设 Step1 转完角度后门已在视野里**）：
           只做一件事 —— 把「识别框中心 x」拉到「画面中心 x」（640×480 → 320；240 是
           纵向中心，本步只用横向）。用「框中心 x − 320」的差值下发左右横移：
           框偏右 → 右移，框偏左 → 左移；差值进入 ±20px 带内 → 停横移。
           丢帧 → 按**上一帧的运动方向**继续（不反向、不清零连续计数）。
           完成：|差值| ≤ 20px（AUV_PASS_DOOR_V2_PX_TOL）连续 10 个有效帧
           （AUV_PASS_DOOR_V2_HOLD_N）→ 本步完成。
    Step3. 固定航向纯前进：高度计 B、C（obs.AltIF 读 momo_alt.json 原始 mm）均出现
           一次突变（读数 < 进入本步锁定的基线 − AUV_PASS_DOOR_V2_ALT_DROP_MM，
           连续 AUV_PASS_DOOR_V2_ALT_HIT_N 拍）后的 AUV_PASS_DOOR_V2_DONE_DELAY_S
           （默认 1s）→ 完成；兜底：运行 AUV_PASS_DOOR_V2_RUSH_DUR_S（默认 15s）
           未检出突变 → 自动完成。
    Step4. 暂空占位：直接完成，交接下一阶段。

★ 无兜底口径：Step1 转向/定深判据失效（无 yaw 遥测 / 融合深度源不可用）即持续执行不完成；
  Step2 **丢帧也按上一帧方向继续、不设超时**（原"对中超时 → HOLD_FAULT"已按 2026-10-10
  新口径移除；_set_fault/_fault_cmd 保留备用）；唯一的兜底是 Step3 前冲的 RUSH_DUR_S（15s）。

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
    """穿门 v2：Step0 视觉就绪 → Step1 转固定角度+定深 → Step2 横移对中 → Step3 前冲+高度计突变判完成 → Step4 占位。"""

    NAME = getattr(TC, 'AUV_PASS_DOOR_V2_STAGE', 'PassDoorV2')  # 阶段名：日志 + 0x09 stage 字段 + stage_model 模型映射键

    def __init__(self, ctx, alt=None):
        super().__init__(ctx)
        self.alt = alt or AltIF(shm_dir=getattr(ctx.cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.fault_reason = None

    # ---------------- 辅助 ----------------
    def log(self, msg):
        """阶段日志（走 ctx.say → mission 日志函数）。"""
        self.ctx.say(msg)

    def _say(self, st, msg):
        """节流日志（同阶段内按 AUV_LOG_EVERY_S 限频）。"""
        t_function._say_throttled(self.ctx, st, self._now, msg)

    def _height(self):
        return float(getattr(TC, 'AUV_PASS_DOOR_V2_HEIGHT_CM', 45.0))

    def _hold_cmd(self, note):
        """零推力保持帧：锁当前航向 + 保持定深目标（Step0 等视觉/故障保持用）。"""
        return t_function._cmd(self.NAME, note,
                               t_function._yaw_hold(self.sts[0], self.ctx),
                               t_function._depth_out(self.sts[0], self._height()),
                               surge=0.0, sway=0.0)

    def _set_fault(self, reason):
        """进入 HOLD_FAULT：零推力保持、永不完成（需重新进入任务重置）。"""
        self.fault_reason = reason
        self.log('FAULT: ' + reason)

    def _fault_cmd(self):
        st = self.sts[2]                                  # 用 Step2 的锁存航向（若已锁存）
        yaw = st.get('yaw_ref')
        if yaw is None:
            yaw = t_function._yaw_hold(st, self.ctx) or 0.0
        return t_function._cmd(self.NAME, 'HOLD_FAULT: ' + self.fault_reason,
                               yaw, t_function._depth_out(st, self._height()),
                               surge=0.0, sway=0.0)

    # ---------------- Stage 契约（enter / step） ----------------
    def enter(self, now):
        self.idx = 0                                      # 当前内部子步骤（Step0..4）
        self.sts = [{} for _ in range(5)]                 # 每个子步骤独立 st dict
        self.fault_reason = None                          # 复位故障状态
        self._now = now
        self._dt = 0.0
        self.log('穿门 v2 启动：视觉就绪 → 转%.0f°定深%.0fcm → 横移对中(20px/10帧) → 前冲+B/C突变判完成(1s, 15s兜底)'
                 % (float(getattr(TC, 'AUV_PASS_DOOR_V2_TURN_DEG', 60.0)), self._height()))

    def step(self, now, dt):
        self._now = now                                   # 刷新时间戳缓存
        self._dt = dt                                     # 刷新拍间隔缓存
        if self.fault_reason is not None:                 # HOLD_FAULT：零推力保持，永不完成
            return self._fault_cmd()
        while self.idx < 5:                               # 还有子步骤没跑完就继续（循环次数有界：≤子步骤数+1）
            if self.idx == 0:
                cmd = self.step0_wait_vision()
            elif self.idx == 1:
                cmd = self.step1_turn_dive()
            elif self.idx == 2:
                cmd = self.step2_align()
            elif self.idx == 3:
                cmd = self.step3_forward_pass()
            else:
                cmd = self.step4_placeholder()
            if cmd is None:                               # 子步骤返回 None = 本子步骤完成
                self.idx += 1                             # 切下一个子步骤
                continue                                  # 同拍继续跑下一步（允许一拍内连续完成）
            return cmd                                    # 子步骤未完成：下发本拍控制帧
        return None                                       # 全部子步骤完成 = 穿门 v2 阶段完成

    #=====Step0. 视觉就绪 + 模型切换 ==============
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

    #=====Step1. 旋转固定角度 + 定深 ==============
    def step1_turn_dive(self):
        """先旋转固定角度（相对当前 yaw，右为正；turn_step 下发目标+定深），
        到位后定深至离底 AUV_PASS_DOOR_V2_HEIGHT_CM（dive_step 判融合 clearance 到位）。
        """
        st = self.sts[1]
        now, dt = self._now, self._dt
        height = self._height()
        turn_deg = float(getattr(TC, 'AUV_PASS_DOOR_V2_TURN_DEG', 60.0))
        tst = st.setdefault('turn', {})                   # 转向子步骤独立 st（与 dive 分离，避免 t0/ok_cnt 残留）
        if not st.get('turned'):
            tgt = t_function.lock_turn_target(self.ctx, tst, now, turn_deg, stage=self.NAME)
            if tgt is None:
                return t_function.wait_cmd(self.NAME, 'Step1 无 yaw 遥测，等待(不以0兜底)')
            cmd = t_function.turn_step(self.ctx, tst, now, dt, target_yaw_deg=tgt,
                                       target_height_cm=height, stage=self.NAME)
            if cmd is not None:
                return cmd
            st['turned'] = True
            self.log('Step1 转向完成：+%.0f° → %.1f°，开始定深 %.0fcm(离底)' % (turn_deg, tgt, height))
        return t_function.dive_step(self.ctx, st.setdefault('dive', {}), now, dt,
                                    target_height_cm=height, stage=self.NAME)

    #=====Step2. 横移对中（框中心 x → 画面中心 x）==============
    def step2_align(self):
        """横移对中：**直接假设旋转之后能识别到门**，只做一件事 ——
        把「识别框中心 x」拉到「画面中心 x」（前摄 640×480 → 320；240 是纵向中心，本步只用横向）。

        控制律（2026-10-10 用户口径，简单直白）：
            有帧：sway ∝ (框中心 x − 画面中心 x)，方向 = 差值符号
                  （框偏右 → 右移 sway>0；框偏左 → 左移 sway<0）；幅度限幅（SWAY_THRUST）+ 保底（MIN_THRUST）。
            丢帧：按**上一帧的运动方向**继续横移 —— 沿用上一拍的 sway（方向与幅度都不反）；
                  若上一拍在容差带内（sway=0）则保持不动，避免在中心附近来回蹭。
        完成：|框中心 x − 画面中心 x| ≤ 20px（AUV_PASS_DOOR_V2_PX_TOL）
              **连续 10 个有效检测帧**（AUV_PASS_DOOR_V2_HOLD_N）→ 本步完成。

        ★ 连续计数只在"收到有效帧"时更新：检测端 ~10Hz、任务 20Hz，VisionIF 对同一帧去重
          （隔拍 poll 返回 None）→ 那些拍走丢帧分支**不清零**，否则永远攒不满 10 帧。
        ★ 不设超时（原 ALIGN_TIMEOUT_S → HOLD_FAULT 已按新口径移除）：丢帧就一直按上一帧方向走。
        """
        st = self.sts[2]
        now = self._now
        height = self._height()
        cam = str(getattr(TC, 'AUV_PASS_DOOR_V2_CAM', 'front'))
        want = str(getattr(TC, 'AUV_PASS_DOOR_V2_WANT', 'gate'))
        px_tol = float(getattr(TC, 'AUV_PASS_DOOR_V2_PX_TOL', 20.0))
        hold_n = int(getattr(TC, 'AUV_PASS_DOOR_V2_HOLD_N', 10))
        sway_kp = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_KP', 1.0))
        sway_max = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_THRUST', 0.3))
        sway_min = float(getattr(TC, 'AUV_PASS_DOOR_V2_SWAY_MIN_THRUST', 0.15))
        img_w = float(getattr(TC, 'AUV_IMG_W', 640.0))     # 前摄 640 → 画面中心 x = 320
        center = 0.5 * img_w

        # 起步：锁存对中航向（全程不动，只横移）+ 初始化"上一帧运动"
        if st.get('yaw_ref') is None:
            y = t_function._tel_f(self.ctx, 'actual_yaw')
            if y is None:
                return t_function.wait_cmd(self.NAME, 'Step2 无 yaw 遥测，等待(不以0兜底)')
            st['yaw_ref'] = float(y)
            st['ok_cnt'] = 0
            st['sway'] = 0.0                               # 上一拍横向输出（丢帧沿用：方向+幅度）
            self.log('Step2 锁死航向 %.1f°，横移对中：画面中心 x=%.0f，判完成 |差值|≤%.0fpx 连续 %d 帧'
                     % (st['yaw_ref'], center, px_tol, hold_n))

        o = self.ctx.vision.poll(cam, want, now)
        if o is None:                                      # —— 丢帧：按上一帧运动方向继续 ——
            sway = float(st['sway'])                       # 沿用上一拍输出（方向不变、不清零计数）
            move = '不动' if sway == 0.0 else ('右移' if sway > 0 else '左移')
            note = 'Step2 丢帧：按上一帧方向%s 继续 sway=%+.2f ok=%d/%d' % (move, sway, st['ok_cnt'], hold_n)
            self._say(st, note)
            return t_function._cmd(self.NAME, note, st['yaw_ref'],
                                   t_function._depth_out(st, height), sway=sway)

        cx = float(o['cx'])                                # —— 有帧：框中心 vs 画面中心 ——
        try:                                               # 口径：优先用检测帧自带的画面宽（按相机不同）
            _w = float(o.get('img_w'))
            if _w > 0:
                img_w, center = _w, 0.5 * _w
        except (TypeError, ValueError):
            pass
        ex = cx - center                                   # 正 = 框在画面右侧
        if abs(ex) <= px_tol:                              # 带内：停横移 + 连续帧计数
            sway = 0.0
            st['ok_cnt'] += 1
        else:                                              # 带外：差值比例输出（限幅 + 保底）
            sway = (sway_kp * ex / center) if center > 0 else 0.0
            sway = max(-sway_max, min(sway_max, sway))
            if abs(sway) < sway_min:
                sway = (1.0 if ex > 0 else -1.0) * sway_min
            st['ok_cnt'] = 0
        st['sway'] = sway
        st.setdefault('_log_ts', 0.0)

        if st['ok_cnt'] >= hold_n:                         # 连续 hold_n 帧在 ±px_tol 内 → 完成
            self.log('Step2 对中完成：框中心 x=%.0f，与画面中心相差 %+.0fpx（≤%.0fpx 连续 %d 帧）'
                     % (cx, ex, px_tol, hold_n))
            return None

        move = '带内停横移' if sway == 0.0 else ('右移' if sway > 0 else '左移')
        note = 'Step2 对中 cx=%.0f 中心=%.0f 差=%+.0fpx %s sway=%+.2f ok=%d/%d' % (
            cx, center, ex, move, sway, st['ok_cnt'], hold_n)
        self._say(st, note)
        return t_function._cmd(self.NAME, note, st['yaw_ref'],
                               t_function._depth_out(st, height), sway=sway)

    #=====Step3. 固定航向纯前进 + 高度计突变判完成 ==============
    def step3_forward_pass(self):
        """固定航向（锁存起步航向）纯前进（恒 surge，sway=0），监视高度计 B、C。

        完成判据：B、C 均出现一次突变（读数 < 基线 − ALT_DROP_MM，连续 ALT_HIT_N 拍）
        后延迟 DONE_DELAY_S（默认 1s）→ 完成。兜底：运行 RUSH_DUR_S（默认 15s）
        未检出突变 → 自动完成。
        基线 = 进入本步后前 ALT_BASE_N 拍有效均值（@5Hz≈2s，锁存后不再滑动）。
        """
        st = self.sts[3]
        now = self._now
        surge = float(getattr(TC, 'AUV_PASS_DOOR_V2_SURGE', 0.5))
        rush_s = float(getattr(TC, 'AUV_PASS_DOOR_V2_RUSH_DUR_S', 15.0))
        delay_s = float(getattr(TC, 'AUV_PASS_DOOR_V2_DONE_DELAY_S', 1.0))

        if st.get('t0') is None:                          # 起步：锁存航向 + 前冲计时 + 高度计基线容器
            y = t_function._tel_f(self.ctx, 'actual_yaw')
            if y is None:
                return t_function.wait_cmd(self.NAME, 'Step3 无 yaw 遥测，等待(不以0兜底)')
            st['yaw_ref'] = float(y)
            st['t0'] = now
            st['alt'] = dict(B=dict(buf=[], base=None, hit=0, done_at=None),
                             C=dict(buf=[], base=None, hit=0, done_at=None))
            self.log('Step3 锁死航向 %.1f°，前冲过门（B/C 突变判完成 +%.0fs；%.0fs 兜底）'
                     % (st['yaw_ref'], delay_s, rush_s))

        elapsed = now - st['t0']
        if elapsed >= rush_s:                             # 兜底：到时未检出突变 → 自动完成
            self.log('Step3 兜底完成：%.1fs 内未检出 B/C 突变（B=%s C=%s）'
                     % (elapsed, self._alt_state(st, 'B'), self._alt_state(st, 'C')))
            return None

        done_ts, msg = self._alt_check(st, self.alt.read(now), now)
        if done_ts is not None and (now - done_ts) >= delay_s:   # 两者均突变后延迟 → 完成
            self.log('Step3 完成：%s 后 %.1fs，判已过门' % (msg, now - done_ts))
            return None

        note = '前冲过门 t=%.1fs' % elapsed
        if done_ts is not None:
            note += '（B/C均突变，等 %.1fs）' % (delay_s - (now - done_ts))
        return t_function._cmd(self.NAME, note, st['yaw_ref'],
                               t_function._depth_out(st, self._height()), surge=surge)

    def _alt_state(self, st, ch):
        """突变判据状态摘要（日志用）：基线/命中/完成时刻。"""
        s = st['alt'][ch]
        base = ('%.0fmm' % s['base']) if s['base'] is not None else '未锁'
        done = ('@%.1f' % s['done_at']) if s['done_at'] is not None else '未触发'
        return '%s base=%s hit=%d%s' % (ch, base, s['hit'], done)

    def _alt_check(self, st, d, now):
        """高度计 B/C 突变检测。返回 (done_ts|None, 说明|None)。

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
            s = st['alt'][ch]
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
        if st['alt']['B']['done_at'] is not None and st['alt']['C']['done_at'] is not None:
            return (max(st['alt']['B']['done_at'], st['alt']['C']['done_at']),
                    'B 突变@%.2f C 突变@%.2f' % (st['alt']['B']['done_at'], st['alt']['C']['done_at']))
        return None, None

    #=====Step4. 暂空占位 ==============
    def step4_placeholder(self):
        """占位：下一动作（如交接/搜索）待实现，直接完成并交接下一阶段。"""
        self.log('Step4（占位）完成 —— 穿门 v2 全流程结束，交接下一阶段')
        return None


# 阶段注册表：task_config.PASS_DOOR_V2_TABLE / test_mode TEST_TABLE 引用
# （一项 = 整个穿门 v2 任务；未挂正式 STAGE_TABLE，测试时在 test_config.py 用
#   TEST_TABLE = PASS_DOOR_V2_TABLE 即可）
PASS_DOOR_V2_TABLE = [PassDoorV2]
