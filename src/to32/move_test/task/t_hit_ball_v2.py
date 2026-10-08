# -*- coding: utf-8 -*-
"""t_hit_ball_v2.py —— 撞球任务 v2（用户新流程，2026-10-08）：Step1 摇摆找球 → Step2 对准 → Step3 冲撞

流程（v2 为"扫视找球 → 对准 → 冲撞"三段式）：
    Step1. Sway to find the ball（已落地）
        左右摇摆：相对当前 yaw 沿轨迹 ±15°（AUV_HIT_V2_YAW_TRAJ）来回扫视，
        并非一次性下发 15° 目标角，而是分成若干小步、逐步下发目标角度；
        前视见球立即退出摇摆 → Step2
    Step2. Turn towards the target ball（已落地）
        悬停并调整 yaw：识别框 cx 经 KF1D（vservo）滤波后与画面中心比较，
        像素误差 × (FOV/画面宽) 换算 yaw 增量递推目标角；误差 ≤20px 连续保持
        20 帧 → 锁定航向 → Step3
    Step3. Hit the target ball（未落地，用户后续给冲撞参数）

Step1/2 无兜底口径（2026-10-06 用户口径）：无 yaw 遥测 → wait_cmd 等待，
    绝不以 0 兜底；Step2 无球时保持航向悬停等待（判据失效不完成）。
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                          # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC
from mission import Stage
import t_function
from task.task_hit_ball import vservo        # KF1D / wrap_deg（公共零件层）

class HitBallAll(Stage):

    NAME = "HitBall_v2"

    # ---------------- 辅助（用户框架沿用） ----------------
    def log(self, msg):
        """阶段日志（走 ctx.say → mission 日志函数）。"""
        self.ctx.say(msg)

    def get_yaw_naw(self):
        """当前任务系 yaw（遥测原始数值，不镜像）；无遥测返回 None。"""
        return t_function._tel_f(self.ctx, 'actual_yaw')

    # ---------------- Stage 契约（enter / step） ----------------
    def enter(self, now):
        self.idx = 0                    # 内部子步骤序号（Step1/Step2）
        self.sts = [{}, {}]             # 每个子步骤独立 st dict（加 Step3 时补一个）
        self._now = now
        self._dt = 0.0

    def step(self, now, dt):
        self._now = now
        self._dt = dt
        while self.idx < 2:
            if self.idx == 0:
                cmd = self.step1_find_ball()
            else:
                cmd = self.step2_turn_to_ball()
            if cmd is None:
                self.idx += 1           # 本子步骤完成 → 切下一个
                continue
            return cmd
        return None                     # 全部子步骤完成 = 撞球阶段完成

    #======Step1. Sway to find the ball==============
    '''
    与上一级任务（task1）衔接，完成前进后，开始撞球任务
    第一步：
    左右摇摆，相对yaw的角度，幅度为±15°，且并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度，

    退出机制：
    一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
    '''
    def step1_find_ball(self):
        self.log("Step1. Sway to find the ball")        
        # 1. 左右摇摆，幅度为±15°，并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度
        # 2. 一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
        yaw_naw = self.get_yaw_naw()
        yaw_target_add = list(getattr(
            TC, 'AUV_HIT_V2_YAW_TRAJ',
            [5, 10, 15, 10, 5, -5, -10, -15, -10, -5]))   # 相对 yaw 的角度（task_config 可调）
        st = self.sts[0]
        now, dt = self._now, self._dt
        cam = str(getattr(TC, 'AUV_HIT_V2_CAM', 'front'))
        want = str(getattr(TC, 'AUV_HIT_V2_WANT', 'ball'))

        # 1. 每拍先查球：见球立即退出摇摆（进入 Step2）
        if self.ctx.vision.poll(cam, want, now):
            self.log('Step1 完成：前视(%s)见 %s，退出摇摆' % (cam, want))
            return None

        # 2. 锁存摇摆基准：当前 yaw 为中心，轨迹 = 相对 yaw 的角度序列（循环）
        if st.get('yaw0') is None:
            if yaw_naw is None:
                return t_function.wait_cmd(self.NAME, 'Step1 无 yaw 遥测，等待(不以0兜底)')
            st['yaw0'] = yaw_naw
            st['yaw_target_add'] = yaw_target_add
            st['i'] = 0                 # 轨迹下标（到位后步进）
            st['ts'] = {}               # 单小步转向状态（每步重置，独立计数）
            self.log('Step1 锁存 yaw0=%.1f°，轨迹=%s' % (yaw_naw, yaw_target_add))

        # 3. 分小步摇摆：逐步下发目标角（yaw0 + 轨迹点），到位后走轨迹下一个点，循环
        while True:
            if self.ctx.vision.poll(cam, want, now):   # 步进间隙也查球（立即退出）
                self.log('Step1 完成：前视见 %s' % want)
                return None
            target = st['yaw0'] + st['yaw_target_add'][st['i']]
            cmd = t_function.turn_step(
                self.ctx, st['ts'], now, dt, target,
                target_height_cm=float(getattr(TC, 'AUV_HIT_V2_HEIGHT_CM', 60.0)),
                tol_deg=float(getattr(TC, 'AUV_HIT_V2_TURN_TOL_DEG', 3.0)),
                hold_n=int(getattr(TC, 'AUV_HIT_V2_TURN_HOLD_N', 10)),
                stage=self.NAME)
            if cmd is not None:
                return cmd
            # 本小步到位 → 轨迹下一个点；走完一圈自动回起点循环
            st['i'] = (st['i'] + 1) % len(st['yaw_target_add'])
            st['ts'] = {}               # 新目标 → 新到位计数
            self.log('Step1 步进 → 相对 %.1f°（目标 %.1f°）'
                     % (st['yaw_target_add'][st['i']],
                        st['yaw0'] + st['yaw_target_add'][st['i']]))

    #=====Step2. Turn towards to the target ball==============
    '''
    第二步：
    悬停并调整yaw，使得yolo识别框（经过卡尔曼滤波处理后）中心x坐标与图像中心尽可能的贴合，允许误差在20像素以内，
    调整过程，先假设FOV为120°，图像宽度为720像素，则每个像素对应的角度为120/720=0.1667°，因此每个像素对应的yaw调整量为0.1667°，

    退出机制：
    一旦yolo识别框中心x坐标与图像中心的误差在20像素以内，并保持20帧，锁定航向，进入下一阶段（Step3. hit the target ball）
    '''
    def step2_turn_to_ball(self):
        self.log("Step2. Turn towards to the target ball")
        # 1. 悬停并调整yaw，使得yolo识别框（经过卡尔曼滤波处理后）中心x坐标与图像中心尽可能的贴合，允许误差在20像素以内
        # 2. 调整过程，先假设FOV为120°，图像宽度为720像素，则每个像素对应的角度为120/720=0.1667°，因此每个像素对应的yaw调整量为0.1667°
        # 3. 一旦yolo识别框中心x坐标与图像中心的误差在20像素以内，并保持20帧，锁定航向，进入下一阶段（Step3. hit the target ball）
        FOV = 120.0 # 假设FOV为120°
        pixel_per_degree = 720 / FOV # 每个像素对应的角度

        st = self.sts[1]
        now, dt = self._now, self._dt
        cam = str(getattr(TC, 'AUV_HIT_V2_CAM', 'front'))
        want = str(getattr(TC, 'AUV_HIT_V2_WANT', 'ball'))
        fov = float(getattr(TC, 'AUV_HIT_V2_FOV_DEG', FOV))
        img_w = float(getattr(TC, 'AUV_IMG_W', 1280.0))          # 与 poll 的 dx 同口径
        deg_per_px = fov / img_w                                   # °/像素（FOV 覆盖全宽）
        px_tol = float(getattr(TC, 'AUV_HIT_V2_ALIGN_PX_TOL', 20.0))
        hold_n = int(getattr(TC, 'AUV_HIT_V2_ALIGN_HOLD_N', 20))
        sign = float(getattr(TC, 'AUV_HIT_V2_YAW_SIGN', 1.0))
        center = 0.5 * img_w
        height = float(getattr(TC, 'AUV_HIT_V2_HEIGHT_CM', 60.0))

        # 1. 起步：KF 实例 + 起始航向（从当前实际 yaw 起步，对准递推的基准）
        if st.get('kf') is None:
            st['kf'] = vservo.KF1D(
                float(getattr(TC, 'AUV_HIT_V2_KF_R_PX2', 225.0)),
                float(getattr(TC, 'AUV_HIT_V2_KF_Q_ACC', 800.0)),
                gate_nsigma=float(getattr(TC, 'AUV_HIT_V2_KF_GATE_NSIGMA', 3.0)),
                reset_n=int(getattr(TC, 'AUV_HIT_V2_KF_RESET_N', 5)),
                trust_age_s=float(getattr(TC, 'AUV_HIT_V2_KF_TRUST_AGE_S', 0.2)))
            y = self.get_yaw_naw()
            if y is None:
                return t_function.wait_cmd(self.NAME, 'Step2 无 yaw 遥测，等待(不以0兜底)')
            st['yaw_ref'] = float(y)
            st['ok_cnt'] = 0
            st['last_ex'] = 0.0
            self.log('Step2 起步：yaw0=%.1f°，deg/px=%.4f' % (y, deg_per_px))

        # 2. 每拍：吃球帧 → KF 滤波 cx → trust 门禁 → 像素误差 → yaw 增量递推
        obs = self.ctx.vision.poll(cam, want, now)
        if obs is None:
            st['kf'].predict(now)                # 丢帧拍：KF 状态滚到 now
            self.log('Step2 无球：保持航向 %.1f° 悬停等待（无兜底，不完成）' % st['yaw_ref'])
            return t_function._cmd(self.NAME, '对准中(无球)',
                                   st['yaw_ref'], t_function._depth_out(st, height))
        in_band = False
        if st['kf'].update(obs['cx'], now, clip=bool(obs.get('clip'))):
            if st['kf'].trust_ok(now, float(getattr(TC, 'AUV_HIT_V2_KF_SIGMA_MAX', 60.0))):
                ex_px = st['kf'].x[0] - center   # 滤波后球心距画面中心（像素，右正）
                st['last_ex'] = ex_px
                st['yaw_ref'] = vservo.wrap_deg(st['yaw_ref'] + sign * ex_px * deg_per_px)
                in_band = abs(ex_px) <= px_tol
        # 3. 完成判据：误差 ≤20px 连续保持 hold_n 帧 → 锁定航向 → Step3
        st['ok_cnt'] = st['ok_cnt'] + 1 if in_band else 0
        if st['ok_cnt'] >= hold_n:
            self.log('Step2 完成：误差 %.0fpx 保持 %d 帧，锁定航向 %.1f°'
                     % (st['last_ex'], hold_n, st['yaw_ref']))
            return None
        return t_function._cmd(self.NAME,
                               '对准中 ex=%.0fpx ok=%d/%d' % (st['last_ex'], st['ok_cnt'], hold_n),
                               st['yaw_ref'], t_function._depth_out(st, height))

    #=====Step3. Hit the target ball==============
    '''
    第三步：
    冲撞球，直接开环，以最高速度前进，
    '''
    def step3_hit_ball(self):
        # 未落地：冲撞参数（前进时长/触壁判据/推力档位）用户确认后补全
        raise NotImplementedError('Step3 未落地：待用户给冲撞参数')


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个撞球任务；空表 = 开机即 DONE）
HIT_BALL_TABLE = [HitBallAll]
