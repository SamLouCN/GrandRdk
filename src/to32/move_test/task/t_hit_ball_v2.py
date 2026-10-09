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
    Step3. Hit the target ball（已落地 2026-10-09）：开环最高速前冲（surge 满档 +
        锁死 Step2 对准航向 + 定深），撞到球（ACCx 突降，临时阈值 0.15 待标定）
        即完成；RUSH_DUR_S(10s) 内没撞到 → 切 Exit：停推 + 自动上浮至水面安全区

无兜底口径（2026-10-06 用户口径）：无 yaw 遥测 → wait_cmd 等待，
    绝不以 0 兜底。2026-10-09 用户新增：
    - Step2 对准中丢球超时(AUV_HIT_V2_LOST_S) → 切 Exit(停推+上浮)，不再死等；
    - Exit 上浮到位 → 返回 STOP 终止整链（后续任务不进行；撞到球正常完成仍返回 None 继续）。
"""
import os
import sys

# 把 task/ 与 move_test/ 都放进 sys.path —— 内部文件用平级 import（同 mode_auv 约定）
_HERE = os.path.dirname(os.path.abspath(__file__))        # move_test/task
_PARENT = os.path.dirname(_HERE)                          # move_test
for _p in (_HERE, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_config as TC                               # 任务参数（AUV_HIT_V2_* 全部从这里 getattr 读取）
from mission import Stage, STOP                        # 阶段基类 + 终止整链哨兵（Exit 上浮到位后返回）
import t_function                                      # 运动原语库：turn_step/wait_cmd/_cmd/_depth_out/_tel_f
from task.task_hit_ball import vservo                  # 视觉伺服公共层：KF1D（滤 cx）/ wrap_deg（角度回绕）

class HitBallAll(Stage):                               # 撞球 v2 阶段：内部子状态机（Step1→Step2→Step3）

    NAME = "HitBall_v2"                                # 阶段名：日志展示 + 0x09 的 stage 字段用

    # ---------------- 辅助（用户框架沿用） ----------------
    def log(self, msg):                                # 阶段日志入口（节流由各调用方控制）
        """阶段日志（走 ctx.say → mission 日志函数）。"""
        self.ctx.say(msg)                              # 转发给上下文日志函数（板端落 journal）

    def get_yaw_now(self):                             # 读当前航向（Step1 基准 / Step2 起始航向共用）
        """当前任务系 yaw（遥测 actual_yaw 原始数值，不镜像；下发侧由 mode_auv 镜像一次）。

        用途：Step1 摇摆基准（yaw0 = 进入时的实际航向）、Step2 对准递推的起始航向。
        无遥测 / 字段缺失返回 None —— 调用方必须以 wait_cmd 等待，不以 0 兜底。
        """
        return t_function._tel_f(self.ctx, 'actual_yaw')   # 遥测安全取数：字段缺失/非法返回 None

    # ---------------- Stage 契约（enter / step） ----------------
    def enter(self, now):                              # 阶段进入时 Mission 调用一次（初始化）
        self.idx = 0                                   # 内部子步骤序号：0=Step1 摇摆 / 1=Step2 对准
        self.sts = [{}, {}, {}]                         # 每个子步骤独立 st dict（Step1/2/3）
        self._now = now                                # 缓存当前时间戳（供无参子步骤方法读取）
        self._dt = 0.0                                 # 缓存拍间隔（首拍 0，step() 里每拍刷新）

    def step(self, now, dt):                           # Stage 契约：每拍 Mission 调用，返回 cmd 或 None
        self._now = now                                # 刷新时间戳缓存
        self._dt = dt                                  # 刷新拍间隔缓存
        while self.idx < 3:                            # 还有子步骤没跑完就继续（循环次数有界：≤子步骤数+1）
            if self.idx == 0:                          # 当前子步骤是 Step1
                cmd = self.step1_find_ball()           # 跑摇摆找球
            elif self.idx == 1:                        # 当前子步骤是 Step2
                cmd = self.step2_turn_to_ball()        # 跑悬停对准
            else:                                      # 当前子步骤是 Step3
                cmd = self.step3_hit_ball()            # 跑开环冲撞
            if cmd is None:                            # 子步骤返回 None = 本子步骤完成
                self.idx += 1                          # 切到下一个子步骤
                continue                               # 同拍继续跑下一步（允许一拍内连续完成）
            return cmd                                 # 子步骤未完成：把本拍控制帧下发（mode_auv 组 0x09）
        return None                                    # 全部子步骤完成 = 撞球阶段完成（Mission 切下一阶段）

    #======Step1. Sway to find the ball==============
    '''
    与上一级任务（task1）衔接，完成前进后，开始撞球任务
    第一步：
    左右摇摆，相对yaw的角度，幅度为±15°，且并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度，

    退出机制：
    一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
    '''
    def step1_find_ball(self):                         # Step1：左右摇摆扫视找球（见球即退出）
        self.log("Step1. Sway to find the ball")       
        # 1. 左右摇摆，幅度为±15°，并非一次性下发15°的目标角度，而是分成若干小步，来回摇摆，逐步下发目标角度
        # 2. 一旦front camera检测到球，立即退出摇摆，进入下一阶段（Step2. turn towards to the target ball）
        yaw_now = self.get_yaw_now()                   # 读当前实际航向（摇摆中心的基准，锁存前只用一次）
        yaw_target_add = list(getattr(                # 摇摆轨迹：相对 yaw 的增量角度序列（从 task_config 读）
            TC, 'AUV_HIT_V2_YAW_TRAJ',                # 参数键名（改 task_config 即可调轨迹，不用动代码）
            [5, 10, 15, 10, 5, -5, -10, -15, -10, -5]))   # 兜底默认：右摆到+15°回中偏右，再左摆到-15°，循环
        st = self.sts[0]                                   # Step1 独立状态 dict（yaw0/yaw_target_add/i/ts）
        now, dt = self._now, self._dt                      # 取本拍时间戳与拍间隔（turn_step 判到位/计数用）
        cam = str(getattr(TC, 'AUV_HIT_V2_CAM', 'front'))  # 检测相机：前视（front camera）
        want = str(getattr(TC, 'AUV_HIT_V2_WANT', 'ball')) # 检测目标：ball（CANON red-ball→ball 已归一）

        # 1. 每拍先查球：见球立即退出摇摆（进入 Step2）
        if self.ctx.vision.poll(cam, want, now):       # 前视检测帧里有没有球（stale/无文件自动 None）
            self.log('Step1 完成：前视(%s)见 %s，退出摇摆' % (cam, want))  # 打完成日志（含相机/目标名）
            return None                               # 完成 Step1 → Mission 同拍切到 Step2

        # 2. 锁存摇摆基准：当前 yaw 为中心，轨迹 = 相对 yaw 的角度序列（循环）
        if st.get('yaw0') is None:                    # 只在首拍锁存一次（后续摇摆全程以它为中心）
            if yaw_now is None:                       # 无 yaw 遥测（深度/遥测链路没起）
                return t_function.wait_cmd(self.NAME, 'Step1 无 yaw 遥测，等待(不以0兜底)')  # 本拍不下发，等遥测到位
            st['yaw0'] = yaw_now                      # 摇摆中心 = 进入时的实际航向（任务系）
            st['yaw_target_add'] = yaw_target_add     # 存轨迹序列（本子步骤内不再重读）
            st['i'] = 0                               # 轨迹下标：从 +5° 那一点开始走
            st['ts'] = {}                             # 单小步转向状态（turn_step 的计时/到位计数，每步重置）
            self.log('Step1 锁存 yaw0=%.1f°，轨迹=%s' % (yaw_now, yaw_target_add))  # 锁存日志（方便核对轨迹）

        # 3. 分小步摇摆：逐步下发目标角（yaw0 + 轨迹点），到位后走轨迹下一个点，循环
        while True:                                   # 摇摆循环：见球退出 or 到位步进，永不"跑完"（无兜底）
            if self.ctx.vision.poll(cam, want, now):  # 步进间隙也查球（到位瞬间球可能刚好进视场）
                self.log('Step1 完成：前视见 %s' % want)  # 见球日志
                return None                           # 完成 Step1（立即退出摇摆）
            target = st['yaw0'] + st['yaw_target_add'][st['i']]  # 本小步目标角 = 摇摆中心 + 当前轨迹增量
            cmd = t_function.turn_step(               # 调转向原语：下发目标角，判 yaw 到位
                self.ctx, st['ts'], now, dt, target,  # 参数：上下文 / 本小步状态 / 时间 / 目标角（任务系）
                target_height_cm=float(getattr(TC, 'AUV_HIT_V2_HEIGHT_CM', 60.0)),  # 摇摆期间定深(距池底 cm)
                tol_deg=float(getattr(TC, 'AUV_HIT_V2_TURN_TOL_DEG', 3.0)),   # 到位容差(°)：|actual_yaw−target|<tol 算到位
                hold_n=int(getattr(TC, 'AUV_HIT_V2_TURN_HOLD_N', 10)),        # 到位保持拍数(20Hz≈0.5s)：带内连续 N 拍才步进
                stage=self.NAME)                      # 阶段名（turn_step 日志与 0x09 stage 字段）
            if cmd is not None:                       # 本小步还没到位（返回了控制帧）
                return cmd                            # 下发这一拍（机身正朝 target 转）
            # 本小步到位 → 轨迹下一个点；走完一圈自动回起点循环
            st['i'] = (st['i'] + 1) % len(st['yaw_target_add'])  # 下标+1；走完一圈取模回到 +5°（来回循环）
            st['ts'] = {}                             # 重置小步状态：新目标角要重新计数到位（t0/ok_cnt 归零）
            self.log('Step1 步进 → 相对 %.1f°（目标 %.1f°）'  # 步进日志：显示相对增量和绝对目标
                     % (st['yaw_target_add'][st['i']],       # 新的轨迹增量（相对 yaw 的角度）
                        st['yaw0'] + st['yaw_target_add'][st['i']]))  # 新的绝对目标角

    #=====Step2. Turn towards to the target ball==============
    '''
    第二步：
    悬停并调整yaw，使得yolo识别框（经过卡尔曼滤波处理后）中心x坐标与图像中心尽可能的贴合，允许误差在20像素以内，
    调整过程，先假设FOV为120°，图像宽度为720像素，则每个像素对应的角度为120/720=0.1667°，因此每个像素对应的yaw调整量为0.1667°，

    退出机制：
    一旦yolo识别框中心x坐标与图像中心的误差在20像素以内，并保持20帧，锁定航向，进入下一阶段（Step3. hit the target ball）
    '''
    def step2_turn_to_ball(self):                     # Step2：悬停 + 递推调整 yaw，把球心对准画面中心
        self.log("Step2. Turn towards to the target ball")  # 进入 Step2 打日志
        # 1. 悬停并调整yaw，使得yolo识别框（经过卡尔曼滤波处理后）中心x坐标与图像中心尽可能的贴合，允许误差在20像素以内
        # 2. 调整过程，先假设FOV为120°，图像宽度为720像素，则每个像素对应的角度为120/720=0.1667°，因此每个像素对应的yaw调整量为0.1667°
        # 3. 一旦yolo识别框中心x坐标与图像中心的误差在20像素以内，并保持20帧，锁定航向，进入下一阶段（Step3. hit the target ball）
        
        FOV = 120.0 # 假设FOV为120°（用户初值，实际取 AUV_HIT_V2_FOV_DEG）
        pixel_per_degree = 720 / FOV # 每个像素对应的角度（用户口径：px/°；正式换算见下方 deg_per_px）

        st = self.sts[1]                              # Step2 独立状态：kf / yaw_ref / ok_cnt / last_ex
        now, dt = self._now, self._dt                 # 本拍时间戳与拍间隔（KF 递推/计数用）
        cam = str(getattr(TC, 'AUV_HIT_V2_CAM', 'front'))  # 检测相机：前视（与 Step1 同一路）
        want = str(getattr(TC, 'AUV_HIT_V2_WANT', 'red-ball')) # 检测目标：ball（CANON 归一）
        fov = float(getattr(TC, 'AUV_HIT_V2_FOV_DEG', FOV))  # 前视水平视场角(°)：像素→角度换算的分母来源
        img_w = float(getattr(TC, 'AUV_IMG_W', 1280.0))    # 画面宽(px)：与 obs.poll 的 cx/dx 同口径（中心=img_w/2）
        deg_per_px = fov / img_w                       # °/像素（假设 FOV 覆盖整个画面宽）：误差像素→yaw 增量
        px_tol = float(getattr(TC, 'AUV_HIT_V2_ALIGN_PX_TOL', 20.0))  # 对准容差(px)：滤波后球心距画面中心 ≤ 此值
        hold_n = int(getattr(TC, 'AUV_HIT_V2_ALIGN_HOLD_N', 20))      # 容差带内连续保持帧数 → 锁定航向完成
        sign = float(getattr(TC, 'AUV_HIT_V2_YAW_SIGN', 1.0))  # ★符号键：dx>0（球在画面右半）→ 右转（yaw 增）；上车反了改 -1
        center = 0.5 * img_w                           # 画面中心 x 坐标（像素）
        height = float(getattr(TC, 'AUV_HIT_V2_HEIGHT_CM', 60.0))  # 对准期间定深（距池底 cm）

        # 1. 起步：KF 实例 + 起始航向（从当前实际 yaw 起步，作为对准递推的基准）
        if st.get('kf') is None:                      # 首拍才建 KF（子步骤只初始化一次）
            st['kf'] = vservo.KF1D(                   # 1D 常速(CV)卡尔曼：滤球心 cx 的抖动
                float(getattr(TC, 'AUV_HIT_V2_KF_R_PX2', 225.0)),   # 量测方差(px²)：R=(15px)²，TODO 实测回填
                float(getattr(TC, 'AUV_HIT_V2_KF_Q_ACC', 800.0)),   # 过程噪声加速度谱密度（CV 模型用）
                gate_nsigma=float(getattr(TC, 'AUV_HIT_V2_KF_GATE_NSIGMA', 3.0)),  # 新息门限 σ：超限帧拒收
                reset_n=int(getattr(TC, 'AUV_HIT_V2_KF_RESET_N', 5)),  # 连续拒收 N 帧 → 重置重捕（防跟丢）
                trust_age_s=float(getattr(TC, 'AUV_HIT_V2_KF_TRUST_AGE_S', 0.2)))  # 喂舵新鲜度门禁(≈2 写帧周期)
            y = self.get_yaw_now()                    # 读当前实际航向（对准递推的起点）
            if y is None:                             # 无 yaw 遥测
                return t_function.wait_cmd(self.NAME, 'Step2 无 yaw 遥测，等待(不以0兜底)')  # 本拍不下发，等遥测
            st['yaw_ref'] = float(y)                  # 起始目标航向 = 当前实际航向（后续每拍递推）
            st['ok_cnt'] = 0                          # 容差带内连续计数（从 0 开始累计）
            st['last_ex'] = 0.0                       # 最近一次像素误差（日志与完成判据用）
            st['last_seen'] = now                     # 最近一次见球时刻（丢球超时计时起点）
            self.log('Step2 起步：yaw0=%.1f°，deg/px=%.4f' % (y, deg_per_px))  # 起步日志（含换算系数）

        # 已切 Exit（丢球超时触发过）：继续上浮，不再回到对准（球回来也不回头）
        if self.sts[2].get('mode') == 'exit':        # Exit 机制激活（在 Step3 的 st 上）
            cmd = t_function.exit_step(               # 调上浮退出原语（停推 + 上浮水面安全区）
                self.ctx, self.sts[2], now, dt,      # 参数：上下文 / 状态 / 时间 / 拍间隔
                stage=self.NAME)                      # 阶段名（日志展示）
            if cmd is None:                           # Exit 完成：已上浮到水面安全区
                self.log('Step2 Exit 完成：已上浮至水面 → 终止整链')  # 完成日志
                return STOP                           # 终止整链（后续任务不进行）
            return cmd                                # 未完成 → 继续上浮

        # 2. 每拍：吃球帧 → KF 滤波 cx → trust 门禁 → 像素误差 → yaw 增量递推
        obs = self.ctx.vision.poll(cam, want, now)    # 查前视检测：有球返回帧 dict，无球返回 None
        if obs is None:                               # 本拍无球帧
            st['kf'].predict(now)                     # KF 状态滚到 now（纯预测滑行，保持滤波连续性）
            lost = now - st['last_seen']              # 距最近一次见球时长(s)
            lost_s = float(getattr(TC, 'AUV_HIT_V2_LOST_S', 3.0))  # 丢球超时阈值(可改)：超时无球 → 上浮退出
            if lost > lost_s:                         # 丢球超时 → 切 Exit（停推+上浮，终止整链）
                self.log('Step2 丢球超时 %.1fs → 切 Exit（停推+上浮水面）' % lost)  # 丢球超时日志
                self.sts[2]['mode'] = 'exit'          # 激活 Exit 子状态（下拍起走上浮路径）
                cmd = t_function.exit_step(           # 同拍切入：立即下发上浮帧
                    self.ctx, self.sts[2], now, dt,  # 参数：上下文 / 状态 / 时间 / 拍间隔
                    stage=self.NAME)                  # 阶段名（日志展示）
                if cmd is None:                       # Exit 首拍即完成（已在安全区）
                    self.log('Step2 Exit 完成：已上浮至水面 → 终止整链')  # 完成日志
                    return STOP                       # 终止整链（后续任务不进行）
                return cmd                            # 未完成 → 继续上浮
            self.log('Step2 无球：保持航向 %.1f° 悬停等待(丢球 %.1fs/%s)'  # 无球日志（含超时倒计时）
                     % (st['yaw_ref'], lost, lost_s))
            return t_function._cmd(self.NAME, '对准中(无球)',  # 组悬停控制帧：锁定 yaw_ref
                                   st['yaw_ref'], t_function._depth_out(st, height))  # 定深沿用/目标高度
        st['last_seen'] = now                         # 本拍见球 → 刷新丢球计时起点
        in_band = False                               # 本拍"在容差带内"标记（默认 False，防误计数）
        if st['kf'].update(obs['cx'], now, clip=bool(obs.get('clip'))):  # KF 吃一帧球心 cx（贴边框→方差×4）
            if st['kf'].trust_ok(now, float(getattr(TC, 'AUV_HIT_V2_KF_SIGMA_MAX', 60.0))):  # trust 门禁：新鲜 & σ 达标才喂舵
                ex_px = st['kf'].x[0] - center        # 滤波后球心距画面中心的像素误差（右正）
                st['last_ex'] = ex_px                 # 存误差（日志/完成判据引用）
                st['yaw_ref'] = vservo.wrap_deg(st['yaw_ref'] + sign * ex_px * deg_per_px)  # 递推目标航向：yaw+符号×误差×度/像素
                in_band = abs(ex_px) <= px_tol        # 误差 ≤20px → 本拍算带内
        # 3. 完成判据：误差 ≤20px 连续保持 hold_n 帧 → 锁定航向 → Step3
        st['ok_cnt'] = st['ok_cnt'] + 1 if in_band else 0  # 带内 +1，带外清零（连续计数）
        if st['ok_cnt'] >= hold_n:                    # 连续 20 帧都在容差带内
            self.log('Step2 完成：误差 %.0fpx 保持 %d 帧，锁定航向 %.1f°'  # 完成日志（误差/帧数/锁定航向）
                     % (st['last_ex'], hold_n, st['yaw_ref']))
            return None                               # 完成 Step2 → Mission 切 Step3（航向已锁定）
        return t_function._cmd(self.NAME,             # 未完成：下发本拍悬停对准帧
                               '对准中 ex=%.0fpx ok=%d/%d' % (st['last_ex'], st['ok_cnt'], hold_n),  # note：当前误差/计数
                               st['yaw_ref'], t_function._depth_out(st, height))  # 目标航向 + 定深（surge/sway=0）

    #=====Step3. Hit the target ball==============
    '''
    第三步：
    冲撞球，直接开环，以最高速度前进

    退出机制：
    机器撞到球，理论上会有一个冲击，使得ACCx出现一个突变，具体的阈值需要根据实际情况来确定。根据这个待定的阈值可以认为机器已经撞到球。
    '''
    def step3_hit_ball(self):                         # Step3：开环最高速冲撞球（ACCx 突降判撞球；超时切 Exit 上浮）
        self.log("Step3. Hit the target ball")        # 进入 Step3 打日志
        # 1. 冲撞球，直接开环，以最高速度前进（surge 满档）
        # 2. 退出机制：撞到球瞬间 ACCx 负向突变(突降) → 判已撞到球（阈值临时值待标定）；
        #    若 RUSH_DUR_S(默认10s) 内一直没撞到 → 切 Exit：停止运动 + 自动上浮至水面
        st = self.sts[2]                              # Step3 独立状态（mode/yaw_ref/t0/accx_buf/accx_base/accx_hit）
        now, dt = self._now, self._dt                 # 本拍时间戳与拍间隔

        # 0. 已进入 Exit 子状态（超时兜底触发过）：调上浮原语，到位即终止整链
        if st.get('mode') == 'exit':                  # Exit 机制激活
            cmd = t_function.exit_step(               # 调上浮退出原语（停推 + 上浮水面安全区）
                self.ctx, st, now, dt,                # 参数：上下文 / 状态 / 时间 / 拍间隔
                stage=self.NAME)                      # 阶段名（日志展示）
            if cmd is None:                           # Exit 完成：已上浮到水面安全区
                self.log('Step3 Exit 完成：已上浮至水面 → 终止整链')  # 完成日志
                return STOP                           # 终止整链（后续任务不进行）
            return cmd                                # 未完成 → 继续上浮

        # 1. 起步：锁死冲撞航向（= Step2 对准结果）与冲撞起始时刻
        if st.get('yaw_ref') is None:                 # 首拍锁存航向
            y = self.get_yaw_now()                    # 读当前实际航向（Step2 完成时已对准球）
            if y is None:                             # 无 yaw 遥测
                return t_function.wait_cmd(self.NAME, 'Step3 无 yaw 遥测，等待(不以0兜底)')  # 本拍不下发，等遥测
            st['yaw_ref'] = float(y)                  # 锁死冲撞航向（之后每拍固定下发，不跟随）
            st['t0'] = now                            # 冲撞起始时刻（超时兜底计时起点）
            self.log('Step3 锁死航向 %.1f°，开始全速冲撞' % st['yaw_ref'])  # 起步日志

        # 2. 撞球判据：ACCx 突降（基线=前 N 拍均值锁定；当前 < 基线−阈值 连续 M 拍 → 撞到球）
        ax = t_function._tel_f(self.ctx, 'acc_x')     # 读 x 轴加速度遥测（撞球瞬间的负向冲击峰）
        hit = False                                   # 本拍"已撞到球"标记（默认否）
        if ax is not None:                            # 有 acc_x 遥测才判（无遥测 → 纯靠超时兜底）
            buf = st.setdefault('accx_buf', [])       # 基线缓冲（最近 N 拍）
            buf.append(ax)                            # 收进缓冲
            if len(buf) > 30:                         # 缓冲只留最近 30 拍
                buf.pop(0)                            # 弹出最旧一帧
            if st.get('accx_base') is None:           # 基线未锁存
                bn = int(getattr(TC, 'AUV_TOUCH_ACC_BASE_N', 10))  # 基线窗口拍数（复用触壁键）
                if len(buf) >= bn:                    # 攒够基线窗口
                    st['accx_base'] = sum(buf[-bn:]) / bn  # 基线 = 窗口均值（锁定不再滑动）
            drop = float(getattr(TC, 'AUV_HIT_V2_RUSH_ACCX_DROP', 1.0))  # 突降阈值(临时值,单位 m/s²)：acc_x<基线−此值 算命中
            if st.get('accx_base') is not None:       # 基线就绪才判
                if ax < (st['accx_base'] - drop):     # acc_x 相对基线突降（负向冲击峰）
                    st['accx_hit'] = st.get('accx_hit', 0) + 1   # 命中计数 +1
                else:                                 # 未突降
                    st['accx_hit'] = 0                # 命中计数清零
                hn = int(getattr(TC, 'AUV_TOUCH_ACC_HIT_N', 3))   # 连续命中拍数（复用触壁键）
                hit = st['accx_hit'] >= hn            # 连续 M 拍命中 → 判已撞到球
        if hit:                                       # 撞到球（ACCx 突降确认）
            self.log('Step3 完成：ACCx 突降(%.2f vs 基线 %.2f m/s²)判已撞到球' % (ax, st['accx_base']))  # 完成日志
            return None                               # 完成 Step3 = 撞球阶段完成（Mission 切下一阶段）

        # 3. 超时兜底：RUSH_DUR_S(10s) 内没撞到 → 切 Exit（停止运动 + 自动上浮至水面）
        elapsed = now - st['t0']                      # 冲撞已跑时长
        if elapsed >= float(getattr(TC, 'AUV_HIT_V2_RUSH_DUR_S', 10.0)):  # 超时(时长可改)
            self.log('Step3 超时 %.1fs 未撞到 → 进入 Exit（停推+上浮水面）' % elapsed)  # 兜底日志
            st['mode'] = 'exit'                       # 激活 Exit 子状态（下拍起走 exit_step）
            cmd = t_function.exit_step(               # 同拍切入：立即下发上浮帧
                self.ctx, st, now, dt,                # 参数：上下文 / 状态 / 时间 / 拍间隔
                stage=self.NAME)                      # 阶段名（日志展示）
            if cmd is None:                           # Exit 首拍即完成（已在安全区）
                self.log('Step3 Exit 完成：已上浮至水面 → 终止整链')  # 完成日志
                return STOP                           # 终止整链（后续任务不进行）
            return cmd                                # 未完成 → 继续上浮

        # 4. 未撞到且未超时：下发本拍冲撞帧（全速前冲 + 定深 + 锁死航向）
        return t_function._cmd(self.NAME, '冲撞中 t=%.1fs' % elapsed,  # 组冲撞控制帧
                               st['yaw_ref'], t_function._depth_out(st, float(getattr(TC, 'AUV_HIT_V2_HEIGHT_CM', 60.0))),  # 锁死航向 + 定深(距池底 cm)
                               surge=float(getattr(TC, 'AUV_HIT_V2_RUSH_SURGE', 1.0)))  # 满档前冲（sway 默认 0）


# 阶段注册表：task_config.STAGE_TABLE 切换引用（一项 = 整个撞球任务；空表 = 开机即 DONE）
HIT_BALL_TABLE = [HitBallAll]                          # 本文件导出的阶段注册表（STAGE_TABLE/TEST_TABLE 引用）
