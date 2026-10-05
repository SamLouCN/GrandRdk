# -*- coding: utf-8 -*-
"""AUV 自主模式 —— 中位机模式之一

进入时把下位机切到 **0x05 AUV** 模式（《多源控制协议设计.md》§5.2），
忽略上位机手动杆位（只记录不动作），自主逻辑全部经 tick() 扩展。
[2026-10-03 协议换代] 原先发的是 0x02 预编程——按双源协议 0x02 属 S100 禁发码，
且 AUV 已有专用码 0x05。此处改发 0x05；模式帧本身不受急停闩锁抑制（见 on_enter）。

[AUV-MISSION 2026-09-26 改动] tick() 接上任务状态机（此前是空 pass）:
  - 新建 mission.Mission：撞球 → 过门 → 捡球 → 触壁 → 上浮 全流程
  - tick() 每拍: pump_telemetry → 查急停闩锁 → mission.step() → 组装 0x09 下发
  - 首次进入时重建 Mission（保证每次切进 AUV 都从头跑）

预留空间（后续各子系统的落点）:
  - on_cmd:      可选择性响应目标深度/姿态等扩展字段（协议暂未定义）
"""
import time  # 导入时间库: 预留给任务计时与超时判定(当前骨架尚未实质使用)

import to32_config as C  # 导入中位机配置: 提供 MODE_AUV 等模式常量
import link_stm32 as S  # 导入下位机链路: 提供 frame_mode 组帧与下行发送
from mode_base import ModeBase  # 导入模式基类: 统一 on_enter/tick/on_cmd 等生命周期接口
from auv_task.notify import make_notifier  # [auv_task] 上位机终端状态提示($MSG)

# [AUV-MISSION 2026-09-26 新增] 任务状态机与两个数据源接口
from auv_task import TaskRunner, apply_yaw_mirror  # [auv_task] 任务化执行器（替代 mission.Mission）
from vision_if import VisionIF  # 导入视觉接口: 读 momo_det_*.json
from depth_if import DepthIF  # 导入深度接口: 读 momo_depth.json(depth_kalman)
from viskf_if import ViskfIF  # [auv_task] 图像卡尔曼接口（过门用）

# [2026-09-30 状态回传] 独立线程把 AUV 运行状态 UDP 推给上位机（失败 N 次后放弃，不影响本模式）
from auv_report import AuvReport, make_snapshot

# [2026-09-25 两路卡尔曼托管] 进 AUV 自动拉起 depth_kalman / camera_kalman（viskf），
# 退出时停掉自己起的那些。这样"上位机切 AUV"这一个动作就把两个观测源都准备好了，
# 不需要额外的启停脚本。两路各自独立：一路起不来只影响它自己那一路。
from kalman_launcher import DepthKalmanLauncher, ViskfLauncher


class AuvMode(ModeBase):  # AUV 自主模式: 忽略上位机手动杆位, 自主逻辑走 tick()
    """AUV 自主模式：on_enter 建状态机，tick 每拍驱动它并把控制量组装成 0x09 下发

    安全红线：0x09 帧没有 STANDBY 守卫，急停闩锁期间一个字节都不能发，
    否则会静默退出急停 —— 所以 tick() 里 estop_latch 检查排在所有下发之前。
    """
    id = C.MODE_AUV  # 模式号: 与上位机 $CMD 的 mode 字段对应, 主循环据此切换
    name = "AUV"  # 模式名: 日志与状态上报显示用
    desc = "自主模式：撞球→过门→捡球→触壁→上浮"  # 模式描述文案

    def __init__(self, ctx):  # 构造: ctx 是全局上下文(急停闩锁/日志/下行发送都在上面)
        """只做状态位初始化；Mission 推迟到 on_enter 再建（保证每次切入都从头跑）"""
        super().__init__(ctx)  # 先跑基类初始化: 绑定 ctx 与 log/send_downlink 等工具方法
        self.state = "idle"          # 预留: idle/init/run/hold/abort
        self.last_tel = None  # 缓存最近一帧下位机遥测, 供后续深度/航向闭环取用
        self._ignore_cnt = 0  # 被忽略的手动杆位帧计数: 只用于前几帧打日志
        self.dropped = {"pid": 0}    # $PID 丢弃计数（2026-09-15 决策 D：暂不打通）
        # [AUV-MISSION 2026-09-26 新增]
        self.yaw_mirror = bool(getattr(C, "YAW_MIRROR", True))   # 抵消固件对 Yaw 的取负归一化
        self.log_every = float(getattr(C, "AUV_LOG_EVERY_S", 1.0))  # 0x09 日志节流秒数
        self.mission = None          # 任务状态机实例: on_enter 时重建
        self.notify = make_notifier(C, log=self.log)  # [auv_task] $MSG 状态提示
        self._last_log_ts = 0.0      # 上次打印 0x09 日志的时刻(节流用)
        self._last_note_stage = ''   # 上次打印时的阶段名(阶段切换时立即打印)
        self.report = None           # [2026-09-30] 状态上报器实例: on_enter 建、on_exit 停
        # [2026-09-25] 卡尔曼进程托管: on_enter 拉起、on_exit 停（起不来不影响任务）
        # 两路并列：深度（momo_depth.json）与图像（momo_viskf.json），互不影响
        self.kalman = DepthKalmanLauncher(C, log=self.log)
        self.viskf = ViskfLauncher(C, log=self.log)

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):  # 进入本模式时调用: prev_id 为上一个模式号
        """切进 AUV：下发 0x04 模式帧(0x05 AUV) → 重建任务状态机 → 停在 idle 等 tick"""
        super().on_enter(prev_id)  # 基类进入处理(清标志/记日志)
        self._ignore_cnt = 0  # 重置忽略计数, 让每次进入都重新提示前 3 帧
        # [2026-10-03] 模式帧不再被急停闩锁抑制：与固件配合，AUV 就位优先（§12.3）
        # AUV = 专用码 0x05；帧语义见《多源控制协议设计.md》§5.2
        self.send_downlink(S.frame_mode(S.MODE_AUV), "0x04 AUV")  # 下发 0x04 模式帧, 值取 0x05: 让下位机切到 AUV 语义
        self.state = "idle"  # 进入后停在 idle, 等 tick() 里的任务状态机启动
        # [AUV-MISSION 2026-09-26 新增] 每次进入都重建状态机，保证从头跑
        self.mission = TaskRunner(C, vision=VisionIF(C, log=self.log),
                                  depth=DepthIF(C, log=self.log),
                                  viskf=ViskfIF(C, log=self.log),
                                  log=self.log, notify=self.notify)
        self.log("[AUV] 任务状态机已就绪（阶段: %s）" % self.mission.stage)
        # [2026-09-25 卡尔曼托管] 进 AUV 就把两个观测源准备好。幂等：已在跑就复用，不会起第二个。
        # 起不来各自只影响自己那一路，任务都会走对应的降级路径，不会卡死。
        if self.kalman.ensure_started():
            self.log("[AUV] 深度卡尔曼已就绪（深度源 /dev/shm/momo_depth.json）")
        else:
            self.log("[AUV] 深度卡尔曼未就绪 —— 将走无深度源的定时降级路径"
                     "（判据只剩时间，定深不可用）")
        if self.viskf.ensure_started():
            self.log("[AUV] 图像卡尔曼已就绪（过门源 /dev/shm/momo_viskf.json）")
        else:
            self.log("[AUV] 图像卡尔曼未就绪 —— 过门将退回原始像素伺服"
                     "（滤波量不可用，穿门判定仍走门宽阈值）")
        # [2026-09-30 状态回传] 起上报线程。起不来也只是"看不到数据"，绝不影响任务
        self.report = AuvReport(C, self.log)
        self.report.start()
        try:  # [auv_task] 状态提示（$MSG）：上位机终端能看到当前在测什么
            self.notify.start()
            self.notify.info('MODE', 'AUV 已启动（上位机切换触发）')
            # [auv_task] ★ 测试模式：自动打开图像回传 + 数据回传（真实作业模式不干预）
            try:
                _t = getattr(getattr(self, 'mission', None), 'test', None)
                if _t is not None and _t.enabled and bool(getattr(C, 'AUV_TEST_FORCE_VIDEO', True)):
                    _d = getattr(self, 'dispatcher', None) or getattr(self, 'ctx', None)
                    _v = getattr(_d, 'video', None)
                    if _v is not None and hasattr(_v, 'set_enabled'):
                        _v.set_enabled(True)
                        self.log('[AUV] 测试模式: 图像回传已打开 -> http://<板端IP>:5000/cam1 /cam2')
                    self.log('[AUV] 测试模式: 数据回传 $TEL/$AUV 与终端提示 $MSG 均在运行')
            except Exception as e:
                self.log('[AUV] 测试模式打开回传异常(不影响任务): %s' % e)
        except Exception as e:
            self.log('[AUV] 提示通道启动异常: %s' % e)
        # 图像回传用的是与 ROV 完全同一条通道（web_server 的 /cam1 /cam2），
        # 它由 web_server 进程常驻提供、与运行模式无关，这里只把地址打出来方便你直接开看
        self.log("[AUV] 图像回传(与 ROV 同一条): http://<板端IP>:5000/cam1 前视 | /cam2 下视（带检测框）")

    def on_exit(self, next_id):  # 退出本模式: next_id 为即将进入的模式号
        """退出 AUV：丢弃状态机实例，防止残留阶段/计时器被下次进入继承"""
        self.state = "idle"  # 先把状态归零, 防止残留 run 状态被下次进入继承
        # [auv_task] ★ 上位机切走（如切回 ROV）→ 立即终止任务执行，交还遥控
        #   需求：测试/作业过程中，上位机一切回 ROV 就杀死自主任务，改收上位机杆位。
        if self.mission is not None:
            try:
                if hasattr(self.mission, 'kill'):
                    self.mission.kill('PC_TAKEBACK')
                if self.notify is not None:
                    self.notify.warn('PC_TAKEBACK', '上位机切换模式 → 自主任务已终止，交还遥控')
            except Exception as e:
                self.log('[AUV] 终止任务异常: %s' % e)
            # 补一帧"零推力 + 停推"让下位机立刻不动（急停闩锁期间绝不发 0x09）
            if bool(getattr(C, 'AUV_STOP_FRAME_ON_EXIT', True)) and not self.ctx.estop_latch:
                try:
                    c = getattr(self.mission, 'ctx', None)
                    self.send_downlink(S.frame_motion(
                        pitch_deg=0.0,
                        yaw_deg=float(getattr(c, 'yaw_deg', 0.0) or 0.0),
                        roll_deg=0.0,
                        depth_cm=float(getattr(c, 'depth_cm', 0.0) or 0.0),
                        surge=0.0, sway=0.0, stick_stop=True),
                        '0x09 停推（上位机接管）')
                except Exception as e:
                    self.log('[AUV] 停推帧发送异常: %s' % e)
        if self.report is not None:  # [2026-09-30] 停上报线程并关 socket
            self.report.stop()  # join 带超时, 不会卡住模式切换
            self.report = None  # 清句柄
        # [2026-09-25] 停掉"进 AUV 时我们自己起的那些"卡尔曼（深度 + 图像）；
        # 如果是你手动 ./run.sh --daemon 起的，本模块不会抢着停它。
        if self.kalman is not None:
            self.kalman.stop()
        if self.viskf is not None:
            self.viskf.stop()
        self.mission = None  # [AUV-MISSION 2026-09-26] 丢弃状态机, 下次进入重建
        if self.notify is not None:  # [auv_task]
            self.notify.stop()
        super().on_exit(next_id)  # 再跑基类退出处理

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):  # 收到上位机 $CMD: AUV 下只记录不下发
        """自主模式忽略手动杆位; mode 字段由主循环负责切换, 这里只记录"""
        self._ignore_cnt += 1  # 累计被忽略的杆位帧数
        if self._ignore_cnt <= 3 and self.notify is not None:  # [auv_task]
            self.notify.warn('PC_IGNORED', 'AUV 模式下不执行上位机杆位；要接管请切回 ROV 模式')
        if self._ignore_cnt <= 3:  # 只打前 3 帧日志, 避免 20Hz 疯狂刷屏
            self.log("[AUV] 忽略手动杆位 surge=%.2f sway=%.2f heave=%.2f yaw=%.2f"  # 日志首行: 说明忽略原因
                     % (cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"]))  # 日志参数: 打印四个被忽略的杆位值

    def on_pid(self, pid):  # 收到 $PID 调参帧: AUV 下同样不下发, 只计数
        """$PID 在 AUV 下同样不下发（2026-09-15 决策 D：暂不打通），只计数 + 首帧提示。

        打通时与 ROV 一致：ch 0~3 → link_stm32.frame_set_pid(ch, p, i, d)。
        """
        self.dropped["pid"] += 1  # 丢弃计数 +1, 便于排查"上位机发了却没生效"
        if self.dropped["pid"] == 1:  # 只在首帧提示一次, 避免重复刷日志
            self.log("[AUV] $PID ch=%d 未下发：暂不打通（固件 0x01 仅 ch0~3）" % pid["ch"])  # 提示该通道未打通

    # ---------------- 周期：自主逻辑入口 ----------------
    def tick(self, now, dt):  # 周期任务入口: now=当前时刻, dt=距上次调用的秒数
        """[AUV-MISSION 2026-09-26 改动] 驱动任务状态机，把控制量组装成 0x09 下发"""
        self.pump_telemetry(now)  # 先按节拍请求遥测: AUV 期间由本模式掌握回传节拍
        if self.ctx.estop_latch:  # 安全红线: 急停已闩锁时一个字节都不下发
            return  # 0x09 没有 STANDBY 守卫, 急停后发 0x09 会静默退出急停
        if self.mission is None:  # 状态机未就绪(理论上不会): 直接返回, 绝不碰下发
            return

        cmd = self.mission.step(now, dt, self.last_tel)  # 跑一拍状态机, 取控制量
        if not cmd:  # None = 本拍不下发(任务已结束)
            return
        self.state = "run" if cmd["stage"] != "DONE" else "idle"  # 状态回写: 供状态上报显示
        yaw = apply_yaw_mirror(cmd["yaw"], self.yaw_mirror)  # 固件对 Yaw 取负, 此处镜像取反抵消
        self.send_downlink(  # 组装 0x09 摇杆综合运动帧并下发
            S.frame_motion(
                pitch_deg=0.0,  # Pitch 恒 0: 任务脚本不控俯仰
                yaw_deg=yaw,  # 目标航向(绝对角, 非角速度)
                roll_deg=0.0,  # Roll 恒 0
                depth_cm=cmd["depth"],  # 目标深度(cm, 固件内闭环; heave 无推力槽位)
                surge=cmd["surge"],  # 前后推力 [-1,1]
                sway=cmd["sway"],  # 横向推力 [-1,1]
                stick_stop=cmd["stop"]),  # FLAG bit0: 仅任务结束时置 1 停推
            self._note(now, cmd))  # 日志标注: 阶段 + 说明(带节流)
        # [2026-09-30 状态回传] 把本拍快照交给上报线程。O(1) 非阻塞，队满丢旧保新，
        # 上报线程挂了也不影响上面已经发出的 0x09 —— 观测通道永远排在控制之后
        if self.report is not None:
            self.report.push(make_snapshot(now, cmd, self.mission, self.last_tel))
        # [auv_task] 上浮完成后请求切回有线 ROV（放在 report 判断之外，回传关了也要生效）
        mr = self.mission.pop_mode_request()
        if mr is not None:
            d = getattr(self, "dispatcher", None)
            if d is not None and hasattr(d, "switch_mode"):
                d.switch_mode(mr)  # ★ 传的是 dispatcher 模式 id(cfg.MODE_ROV=0)
            else:
                self.log("[AUV] 收到切模式请求 %s，但 dispatcher 未回挂" % mr)

    def _note(self, now, cmd):  # 生成 0x09 的日志标注(阶段切换立即打, 否则按节流)
        """日志节流: 阶段切换或距上次打印超过 AUV_LOG_EVERY_S 才带说明文字"""
        if cmd["stage"] != self._last_note_stage:  # 阶段刚变: 立刻打一次
            self._last_note_stage = cmd["stage"]  # 记住新阶段
            self._last_log_ts = now  # 重置节流基准
            return "0x09 AUV:%s %s" % (cmd["stage"], cmd["note"])  # 带说明的标注
        if (now - self._last_log_ts) >= self.log_every:  # 同阶段内按秒节流
            self._last_log_ts = now  # 更新节流基准
            return "0x09 AUV:%s %s" % (cmd["stage"], cmd["note"])  # 带说明的标注
        return ""  # 空标注: send_downlink 会用模式名兜底, 不刷屏

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):  # 收到下位机 V2 遥测帧
        """缓存最近一帧遥测；只有缓存不解析，字段映射统一交给 tel_builder"""
        self.last_tel = tel  # 供 tick() 传给 mission.step()：深度/航向/IMU 判据都从这取

    def build_telemetry(self):  # 构造本模式自有遥测; 返回 None 表示交给统一映射
        """返回 None = 本模式不发自有 $TEL，由 mode_dispatcher 回落到 tel_builder 统一映射。

        （2026-09-15 校正注释：此前写"暂不发"易被读成"AUV 期间 $TEL 断流"——实际不会。）
        """
        return None  # 回落 tel_builder: AUV 期间 $TEL 不断流, 仍按 41 字段正常发
