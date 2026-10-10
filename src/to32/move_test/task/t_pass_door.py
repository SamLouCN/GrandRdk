# -*- coding: utf-8 -*-
"""t_pass_door.py —— 穿门任务（新实现，Step1 先行落地）

Step1（2026-10-10 用户口径，唯一公式）：
        目标航向角 = 当前实测航向角 + Kp × z角

    z角（★ 当前为**临时固定值 +50°**，先跑通转向链；置
        AUV_PASS_DOOR_Z_FIXED_DEG=None 才回落到读前视 CV）：
        固定模式 —— 直接取常数，不依赖视觉；
        CV 模式 —— door.guidance.alignment.rotation_xyz_deg[Z_ANGLE_INDEX]
                   （task_door 已纯视觉化，不再有 pose['yaw_error_deg']；
                    相机 X右/Y下/Z前 ⇒ 索引 1(Y) 是竖直轴=偏航，2(Z) 是光轴=像面滚转）
    Kp  = AUV_PASS_DOOR_KP_Z_YAW（默认 1.0）
    算出的目标航向**锁存一次**，之后每拍交给 turn_step 下发，直到转到目标为止。

本步只动 yaw：**转向前锁存目标航向，再交给原语 `t_function.turn_step` 把该绝对角
下发到板端**（0x09 yaw = 目标绝对角，固件内闭环），surge = sway = 0；
depth 显式换算为「离底高度」传入，保持当前实测深度（本步不调深）。
下发侧（mode_auv.tick）会做一次 Yaw 镜像；任务内一律用遥测原始数值系，
**绝不在这里再镜像**（见 t_function 文件头口径 4）。

完成判据：`|actual_yaw − 目标航向| ≤ YAW_TOL_DEG` 连续 `TURN_HOLD_N` 拍（turn_step 内置，
只用无偏的航向遥测判，不看 z 抖不抖）。Kp=1 时等价于 |z| ≤ YAW_TOL_DEG。

异常口径（无兜底）：
    CV 模式下无姿态/过期 → 锁不到目标，本拍不下发（wait_cmd），不完成
    （固定 z 角模式下不会有这种情况）；
    无 actual_yaw 或 actual_depth_cm 遥测 → wait_cmd（本拍不下发），不以 0 兜底。

参数（可写在 task_config.py，缺项全部走本文件默认值，不改配置也能跑）：
    AUV_PASS_DOOR_STAGE      阶段名，默认 'PassGate'
                             （★ 与 stage_model 模型映射、前视发布口径一致；
                               改名要同步 stage_model，否则前视不发布本阶段观测）
    AUV_PASS_DOOR_Z_FIXED_DEG  ★ 固定 z 角，默认 50.0；置 None 改为读前视 CV
    AUV_PASS_DOOR_Z_ANGLE_INDEX CV 模式取 rotation_xyz_deg 的下标，默认 1(Y=竖直轴)
    AUV_PASS_DOOR_KP_Z_YAW   唯一增益 Kp，默认 1.0
    AUV_PASS_DOOR_YAW_TOL_DEG 转向到位容差，默认 1.5
    AUV_PASS_DOOR_TURN_HOLD_N 到位保持拍数，默认 5
    AUV_PASS_DOOR_YAW_SIGN   转向符号 ±1，默认 1.0（现场定号：反了就把这里翻号）
    AUV_PASS_DOOR_DET_FILE   前视观测文件，默认 momo_det_front.json
    AUV_PASS_DOOR_STALE_S    观测超期秒，默认 0.5
"""
import json
import os
import sys

# 把 task/ 与 move_test/ 放进 sys.path —— 内部文件用平级 import（同 t_pass_door_v2 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))         # move_test/task
_PARENT = os.path.dirname(_HERE)                            # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC              # 任务参数（AUV_PASS_DOOR_* 可选，缺项走默认）
from mission import Stage             # 阶段基类（enter/step 契约）
import t_function as TF               # 原语库：_cmd / _tel_f / wait_cmd / clamp_depth_cm / _say_throttled


def _wrap180(deg):
    """角度归一到 [-180, 180)（与 t_function.yaw_err_deg 同系，任务系不做镜像）"""
    return (float(deg) + 180.0) % 360.0 - 180.0


def _num(key, default):
    """读数值型任务参数：task_config 有就用，没有就用默认"""
    try:
        return float(getattr(TC, key, default))
    except (TypeError, ValueError):
        return float(default)


class PassDoorStep1(Stage):
    """Step1：拿 z 角（当前为固定 +50°）把机头转正 —— 目标航向 = 实测航向 + Kp × z角。"""

    NAME = str(getattr(TC, 'AUV_PASS_DOOR_STAGE', 'PassGate'))

    def __init__(self, ctx):
        super().__init__(ctx)
        self.kp = _num('AUV_PASS_DOOR_KP_Z_YAW', 1.0)          # 唯一增益 Kp
        self.yaw_tol = _num('AUV_PASS_DOOR_YAW_TOL_DEG', 1.5)  # 转向到位容差（等价 |z| ≤ tol/Kp）
        try:
            self.turn_hold_n = int(getattr(TC, 'AUV_PASS_DOOR_TURN_HOLD_N', 5))
        except (TypeError, ValueError):
            self.turn_hold_n = 5
        self.yaw_sign = _num('AUV_PASS_DOOR_YAW_SIGN', 1.0)    # 现场定号
        self.stale_s = _num('AUV_PASS_DOOR_STALE_S', 0.5)      # 观测超期秒
        # ★ 临时：固定 z 角先跑通转向（默认 +50°）。置 None 才回落到读前视 CV。
        self.z_fixed = getattr(TC, 'AUV_PASS_DOOR_Z_FIXED_DEG', 50.0)
        try:
            self.z_fixed = None if self.z_fixed is None else float(self.z_fixed)
        except (TypeError, ValueError):
            self.z_fixed = 50.0
        try:
            self.z_axis = int(getattr(TC, 'AUV_PASS_DOOR_Z_ANGLE_INDEX', 1))
        except (TypeError, ValueError):
            self.z_axis = 1
        shm = str(getattr(ctx.cfg, 'AUV_SHM_DIR', None)
                  or getattr(TC, 'AUV_SHM_DIR', '/dev/shm'))
        self.det_path = os.path.join(
            shm, str(getattr(TC, 'AUV_PASS_DOOR_DET_FILE', 'momo_det_front.json')))

    # ---------------- 观测：只取 z 角 ----------------
    def _z_deg(self, now):
        """z 角 → (z_deg, frame)；取不到 → (None, None)

        ① 固定值模式（AUV_PASS_DOOR_Z_FIXED_DEG，默认 +50°）：不读 CV，直接给常数
           —— 先把"算目标航向 → turn_step 下发 → 转到目标"这条链跑通上水验证。
        ② 固定值置 None 时读前视 CV：door.guidance.alignment.rotation_xyz_deg[Z_ANGLE_INDEX]。
           ★ 新链路（task_door 已纯视觉化）不再输出 pose['yaw_error_deg']，只剩 PnP 的
             相机系三轴旋转；哪一轴等于"偏航"必须现场确认：
             相机 X右/Y下/Z前 ⇒ **Y(索引1) 才是绕竖直轴的偏航**，Z(索引2) 是光轴=像面滚转。
             默认取索引 1；现场把船故意转 10° 看哪个分量跟着变即可定死。
        """
        if self.z_fixed is not None:
            return self.z_fixed, None                              # 固定值：不依赖视觉
        try:
            with open(self.det_path, encoding='utf-8') as f:
                rec = json.load(f)
            ts = float(rec['capture_ts'])
            if not 0.0 <= now - ts <= self.stale_s:
                return None, None                                  # 过期：写端没在更新
            door = rec.get('door') or {}
            align = ((door.get('guidance') or {}).get('alignment')) or {}
            rot = align.get('rotation_xyz_deg')
            if rot is None or len(rot) <= self.z_axis:
                return None, None                                  # 姿态不可用/未解出
            z = float(rot[self.z_axis])
            if z != z:                                             # NaN
                return None, None
            return z, rec.get('frame')
        except (OSError, ValueError, TypeError, KeyError, IndexError):
            return None, None                                      # 读到半个 JSON / 文件不存在

    # ---------------- Stage 契约 ----------------
    def enter(self, now):
        self.st = {}                                               # 本步内部状态（锁存目标/计数器/帧号）
        src = ('固定 %.1f°' % self.z_fixed) if self.z_fixed is not None else \
              ('前视 CV %s[轴%d]' % (os.path.basename(self.det_path), self.z_axis))
        self.ctx.say('穿门 Step1 启动：目标航向 = 实测航向 + %.2f × z角（z 来源：%s）'
                     ' → turn_step 下发到板端；到位 |err| ≤ %.1f° 连续 %d 拍'
                     % (self.kp, src, self.yaw_tol, self.turn_hold_n))

    def _height_cm(self, depth_cm):
        """固件深度(cm，水面下) → turn_step 要的**离底高度**(cm)。

        换算口径（Task.md §4.1）：depth_cm = 池深 − 目标高度 − 机体高度
        ⇒ 离底高度 h = 池深 − 机体高度 − depth_cm。
        ★ 必须显式传，否则 turn_step 会沿用 last_height_cm／默认 60cm → 顺手把船变深。
        """
        pool = float(getattr(TC, 'AUV_POOL_DEPTH_CM', 106.0))
        body = float(getattr(TC, 'AUV_BODY_HEIGHT_CM', 20.0))
        return pool - body - float(depth_cm)

    def step(self, now, dt):
        st = self.st

        # ---- 遥测：航向是唯一必需量（判到位要用）；深度用于"保持不动" ----
        yaw_now = TF._tel_f(self.ctx, 'actual_yaw')
        if yaw_now is None:
            return TF.wait_cmd(self.NAME, 'Step1 无 yaw 遥测，本拍不下发（不以 0 兜底）')
        depth_now = TF._tel_f(self.ctx, 'actual_depth_cm')
        if depth_now is None:
            return TF.wait_cmd(self.NAME, 'Step1 无深度遥测，本拍不下发')

        # ---- 目标航向只算一次并锁存：目标 = 切入点实测航向 + Kp × z角 ----
        #   锁存后 turn_step 用固定目标判到位，避免"目标随 z 抖动→到位计数来回清零"。
        #   需要转完复核（水流/过冲）时把 st.pop('yaw_tgt', None) 即可再算一轮。
        if st.get('yaw_tgt') is None:
            z, _ = self._z_deg(now)
            if z is None:
                TF._say_throttled(self.ctx, st, now,
                                  'Step1 CV 模式无可用 z 角（无姿态/不可控/过期），等观测后锁存目标')
                return TF.wait_cmd(self.NAME, 'Step1 无 z 角，本拍不下发')
            st['yaw_tgt'] = _wrap180(yaw_now + self.kp * z * self.yaw_sign)
            st['z0'] = z
            self.ctx.say('Step1 目标航向锁存：%.1f°（实测 %.1f° + %.2f×%+.2f°，z 来源 %s），交 turn_step 下发'
                         % (st['yaw_tgt'], yaw_now, self.kp, z,
                            '固定值' if self.z_fixed is not None else 'CV'))

        # ---- 转向：把锁存的目标 yaw 交给 turn_step 下发到板端（0x09 yaw=目标绝对角） ----
        turn_st = st.setdefault('turn', {})
        turn_st['last_height_cm'] = self._height_cm(depth_now)     # 本步只转向，深度保持当前实测
        cmd = TF.turn_step(self.ctx, turn_st, now, dt, st['yaw_tgt'],
                           tol_deg=self.yaw_tol, hold_n=self.turn_hold_n, stage=self.NAME)
        if cmd is None:                                            # 转到目标 → 本步完成
            self.ctx.say('Step1 完成：已转到目标航向 %.1f°（到位 %.1f°×%d 拍）'
                         % (st['yaw_tgt'], self.yaw_tol, self.turn_hold_n))
            return None
        return cmd


# 任务表：test_config.TEST_TABLE = PASS_DOOR_TABLE 即可单独调试本步
PASS_DOOR_TABLE = [PassDoorStep1]
