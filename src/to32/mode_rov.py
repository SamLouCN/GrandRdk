# -*- coding: utf-8 -*-
"""有线 ROV 遥控模式 —— 中位机模式之一
[2026-10-03 协议换代] 本模式对下位机而言是「有线 ROV」(ROV_TETHERED, 0x06) ——
即 S100 把操作手的有线遥控中继给下位机。原先发的是 0x03，按双源协议 0x03 已重定义为
「无线 ROV」且属 S100 禁发码（《多源控制协议设计.md》§5.3）。中位机内部模式 id 仍是
config.MODE_ROV(=0)，只有下发给固件的模式码变成了 0x06。

运行逻辑（首版，映射语义参照 /userdata/To32_1/README.md §4 已验证结论）:
  - surge / sway → V2 `0x09` 推力槽 [8]/[9]（[-1,1] ×127，固件内再 ×2 → ±254）
  - heave        → 积分成目标深度（默认 20 cm/s；heave>0=上浮=深度减小）
  - yaw          → 积分成目标航向（默认 60 °/s；固件对 Yaw 取负归一化 → 此处取反抵消）
  - pitch/roll   → 目标恒 0（上位机 $CMD 不下发姿态）
  - led/grab/store → V2 无对应字段，丢弃并计数
  - **急停锁存期间抑制一切 0x09**（0x09 无 STANDBY 守卫，可绕过急停，安全红线）

预留空间:
  - 单推进器分配 / 速度环（tick 钩子）
  - 目标姿态/深度闭环（on_downlink 钩子已有, 可接 PID）
  - $PID 透传 ch0~3（2026-09-15 决策 D：暂不打通，保持丢弃+计数）

参数来源（2026-09-15 起）: 速率/符号/量程全部读 config.py（单一来源）
$TEL 上行（2026-09-15 起）: 统一走 tel_builder（本文件不再自建一份 41 字段映射）
"""
import time  # time：积分 dt 与时间戳

import to32_config as C  # 配置常量（速率/符号/量程的单一来源）
import link_stm32 as S  # 下位机链路与 V2 组帧函数
import tel_builder as TB  # $TEL 41 字段统一映射
from mode_base import ModeBase  # 模式基类


def _cfgv(cfg, name, default):  # 配置读取函数：运行时 → 模块常量 → 缺省
    """取配置：运行时 cfg（main.build_cfg / CLI 覆盖）→ config.py 模块常量 → 内置缺省。

    只读模块常量会让 `--xx` 之类的运行时覆盖失效；只读运行时 cfg 又会在
    cfg 未携带该键时无声回退到缺省。故按此顺序取。
    """
    v = getattr(cfg, name, None) if cfg is not None else None  # 优先取运行时 cfg（支持 CLI 覆盖）
    if v is None:  # 运行时 cfg 没有该键时才有回退必要
        v = getattr(C, name, default)  # 回退到 config.py 的模块常量
    return v  # 返回最终生效值


class RovMode(ModeBase):  # ROV 遥控模式类
    id = C.MODE_ROV  # 模式 ID 取自配置
    name = "ROV"  # 模式名
    desc = "有线遥控模式：手柄杆位 → V2 0x09（对固件是有线 ROV / 0x06）"  # 模式描述

    def __init__(self, ctx):  # 构造函数
        super().__init__(ctx)  # 先执行基类初始化（ctx / log 等）
        self.wait_dt_max = 0.25          # 积分 dt 上限（帧间隔异常时防跳变）
        # ↓ 全部来自 config.py（单一来源；此前硬编码在 __init__，改 config 不生效 = U7）
        self.yaw_rate = float(_cfgv(self.cfg, "YAW_RATE_DPS", 60.0))      # 满偏航向角速率 °/s
        self.depth_rate = float(_cfgv(self.cfg, "DEPTH_RATE_CMS", 20.0))  # 满偏深度速率 cm/s
        self.heave_sign = int(_cfgv(self.cfg, "HEAVE_SIGN", 1))           # +1: heave>0=上浮
        self.yaw_mirror = bool(_cfgv(self.cfg, "YAW_MIRROR", True))       # 抵消固件 Yaw 取负
        self.depth_max = float(_cfgv(self.cfg, "DEPTH_MAX_CM", 200.0))    # 目标深度钳位 cm
        self.surge_full_scale = float(_cfgv(self.cfg, "SURGE_FULL_SCALE", 127.0))  # 推力满量程：surge/sway 的量化基数 127
        self.pid_passthru = bool(_cfgv(self.cfg, "PID_PASSTHRU", False))  # 决策 D：暂不打通
        self.pid_max_ch = int(_cfgv(self.cfg, "PID_MAX_CH", 3))           # 固件 0x01 的通道上限
        self.target_yaw_deg = 0.0  # 目标航向（度），由 yaw 输入积分得到
        self.target_depth_cm = 0.0  # 目标深度（cm），由 heave 输入积分得到
        self.last_tel = None  # 最近一帧遥测，供 build_telemetry 用
        self.dropped = {"led": 0, "grab": 0, "store": 0, "pid": 0}  # V2 装不下而被丢弃的字段计数
        self._drop_state = {"led": False, "grab": False, "store": False}  # 上一帧各开关状态，用于只统计上升沿
        self._pid_warned = False  # $PID 首帧提示只打一次的标志
        self._last_t = None  # 上次 $CMD 时间戳，用于算积分 dt
        # 空闲静默（2026-09-15）：0x09 只在载荷变化时下发
        self.motion_silent = bool(_cfgv(self.cfg, "MOTION_SILENT_WHEN_IDLE", True))  # 空闲静默总开关
        # 2026-09-21 修订: 仅当四轴(surge/sway/heave/yaw)输入全为 0 时判为回中; 其它情况每帧下发
        # 默认 True=仅回中静默(修复"持续推杆被吞"问题); False=旧字节比对(已知 bug, 不推荐)
        self.motion_silent_axes_only = bool(_cfgv(self.cfg, "MOTION_SILENT_WHEN_IDLE_AXES_ONLY", True))  # 静默判定方式：四轴回中(True) / 旧字节比对(False)
        self._last_motion_frame = None  # 上一帧 0x09 字节，仅旧比对模式使用
        self._was_latched = False  # 上一帧是否处于急停锁存
        self._was_centered = None  # None=首帧未发, True/False=上一帧回中状态(用于沿触发)
        self.idle_skipped = 0  # 被静默跳过的帧计数

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):  # 进入本模式时调用
        super().on_enter(prev_id)  # 先执行基类的进入逻辑
        self._last_t = time.time()  # 重置积分时间基准
        self._last_motion_frame = None     # 重进本模式后第一帧必发（重新建立基准）
        self._was_centered = None          # 2026-09-21: 重进 ROV 首帧必发(沿触发)
        # 进入遥控模式先让固件处于 ROV 模式（不发 0x09 也能先就位）
        self.send_downlink(S.frame_mode(S.MODE_ROV_TETHERED), "0x04 有线ROV")  # [2026-10-03] 改发 0x06（原 0x03 已重定义为无线ROV/S100禁发）；模式帧不受急停闩锁抑制
        # 上位机把中位机切到 ROV 后：按协议立刻请求一次回传（0x0C，1 请求回 1 帧），
        # 之后由本模式 tick() 按 STM32_POLL_HZ 持续请求（见下方 tick）
        self._last_poll_ts = time.time()  # 记录本次轮询的时间戳
        # [2026-10-04 恢复] /userdata/To32 恢复进入 ROV 时主动请求一次 0x0C 遥测回传
        #   （GrandRDK/src/to32 那份仍保持暂停）
        self.link_stm32.send_telemetry_request("0x0C 遥测请求(ROV 进入)")  # 主动请求一次 0x0C 遥测回传

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):  # 每帧 $CMD → 0x09 摇杆综合运动帧
        """每帧 $CMD → 0x09 摇杆综合运动（**只在载荷变化时下发**）

        2026-09-15 需求：摇杆在中间（未动）时不要给下位机发任何信号。
        做法：把组好的帧与上一帧**逐字节比对**，相同就不发 → F 口静默。
          - 回中瞬间载荷变化（surge/sway 归零）仍会发一帧把推力归零，之后静默；
          - 固件 Motion_Flag/Motion_Time 只写不读，无"靠周期帧维持"的看门狗 → 静默安全；
          - 开关：config.MOTION_SILENT_WHEN_IDLE（默认 True）。
        """
        if self.ctx.estop_latch:  # 急停锁存期间一律抑制 0x09（安全红线）
            self._was_latched = True  # 记下锁存态，解除后需补发一帧
            self.log("[ROV] 急停锁存中, 抑制 0x09（恢复必须解除锁存）")  # 打日志说明被抑制的原因
            return  # 直接返回，不再组帧下发
        if self._was_latched:  # 上一帧仍在锁存、本帧已解除
            # 急停刚被解除：清基准并补发一帧，让固件从 STANDBY 回到 ROV
            self._was_latched = False  # 清除锁存标记
            self._last_motion_frame = None  # 清掉旧基准字节以强制补发
            self._was_centered = None  # 2026-09-21: 急停刚解除, 下一帧作为"新基准"必发
            self.log("[ROV] 急停已解除 -> 恢复下发 0x09")  # 日志：恢复下发 0x09
        now = time.time()  # 取当前时间
        if self._last_t is None:  # 首次进入没有时间基准
            self._last_t = now  # 以当前时间作为基准
        dt = min(self.wait_dt_max, max(0.001, now - self._last_t))  # dt 上下限保护：上限 0.25s、下限 1ms
        self._last_t = now  # 更新时间基准供下一帧使用

        # yaw 积分 → 目标航向（固件镜像取反）
        self.target_yaw_deg = self.target_yaw_deg + cmd["yaw"] * self.yaw_rate * dt  # yaw 输入 × 角速率 × dt 积分成目标航向
        # heave 积分 → 目标深度（钳位 0~config.DEPTH_MAX_CM）
        self.target_depth_cm = max(0.0, min(self.depth_max,  # 目标深度 = 当前 − heave×速率×dt（heave>0 上浮）
                                self.target_depth_cm - cmd["heave"] * self.heave_sign * self.depth_rate * dt))  # 钳位到 0~depth_max，避免负深度/超上限
        yaw_out = (-self.target_yaw_deg) if self.yaw_mirror else self.target_yaw_deg  # 固件对 Yaw 取负，此处镜像取反抵消

        frame = S.frame_motion(pitch_deg=0.0, yaw_deg=yaw_out, roll_deg=0.0,  # 组 0x09 运动帧：目标 pitch/roll 恒 0
                               depth_cm=self.target_depth_cm,  # 下发积分得到的目标深度（cm）
                               surge=cmd["surge"], sway=cmd["sway"],  # surge/sway 直接取杆位 [-1,1]
                               full_scale=self.surge_full_scale,  # 推力量化基数（默认 127）
                               depth_max_cm=self.depth_max)  # 深度钳位上限随配置一并下发
        # 2026-09-21 修订: 静默判定改为"四轴回中"语义(输入侧)
        #   旧逻辑(字节比对)有 bug: 用户持续前推时 surge/sway 数值恒定, 字节比对相等
        #   ⇒ 0x09 静默 ⇒ 固件收不到持续推力指令, S100→32 下行消失。日志实证。
        # 新逻辑(motion_silent_axes_only=True, 默认):
        #   仅当 surge/sway/heave/yaw 输入侧全 0(真正的"摇杆回中")才静默;
        #   任何一轴非零(含恒定推力)→ 每帧下发, 不再吞指令。
        # 沿触发必发(覆盖自测 [12] 旧用例语义):
        #   - 首帧(_was_centered is None): 必发;
        #   - 非中位→中位跳变沿: 必发一帧把 surge/sway 归零(固件才知道回中)。
        # 兼容: motion_silent=False → 全不静默, 每帧发(自测 cv_j 路径)。
        if self.motion_silent:  # 静默总开关打开才做静默判定
            if self.motion_silent_axes_only:  # 新语义分支：按四轴输入是否回中判定
                is_centered = (cmd["surge"] == 0 and cmd["sway"] == 0  # surge/sway 为 0 是回中的必要条件
                               and cmd["heave"] == 0 and cmd["yaw"] == 0)  # heave/yaw 同样必须为 0 才算回中
                if is_centered and self._was_centered is True:  # 持续回中（上一帧也是回中）才静默
                    self.idle_skipped += 1                 # 持续回中: 静默
                else:  # 否则（非回中或刚回中的沿）走必发分支
                    self.send_downlink(frame, "0x09 ROV")  # 沿触发或非回中: 必发
                self._was_centered = is_centered  # 记录本帧回中状态，供下帧做沿判定
            else:  # 旧语义分支：整帧字节比对
                # 旧字节比对(留作回退, 不推荐)
                if bytes(frame) == self._last_motion_frame:  # 与上一帧字节完全相同
                    self.idle_skipped += 1                 # 静默: 与上一帧相同, 不发
                else:  # 不同则下发这一帧
                    self.send_downlink(frame, "0x09 ROV")
                    self._last_motion_frame = bytes(frame)  # 记住本帧字节，作为下次比对的基准
        else:  # 总开关关闭分支
            self.send_downlink(frame, "0x09 ROV")         # 总开关关: 每帧发

        # V2 装不下的字段：丢弃并计数（预留：后续可扩展 0x0D/0x0E/0x0F）
        # 只统计"上升沿"(0→非0)：这些字段挂在 20Hz 循环帧上，逐帧累加会刷成无意义计数
        for key, on in (("led", bool(cmd["led1"] or cmd["led2"])),  # 遍历 led/grab/store 三个 V2 装不下的字段
                        ("grab", bool(cmd["grab"])),  # led 取 led1/led2 任一为真的或值
                        ("store", bool(cmd["store"]))):  # grab 与 store 直接取布尔
            if on and not self._drop_state[key]:  # 只在上升沿(0→非0)计数
                self.dropped[key] += 1  # 丢弃计数 +1
            self._drop_state[key] = on  # 保存本帧状态供下帧判沿

    def on_pid(self, pid):  # $PID 处理入口
        """$PID：2026-09-15 决策 D —— 暂不打通，保持丢弃 + 计数 + 首帧提示。

        原因：上位机 UI 是 12 通道（文案"每台推进器一路 PID 速度环"），
        而固件 0x01 只认 ch 0~3 = Pitch/Yaw/Roll/Depth，语义不对齐。
        打通时：ch 0~3 → S.frame_set_pid(ch, p, i, d)（组帧函数已就绪）。
        """
        self.dropped["pid"] += 1  # $PID 一律丢弃，只累加计数
        if not self._pid_warned:  # 仅首次提示，避免每帧刷屏
            self._pid_warned = True  # 置位提示标志
            self.log("[ROV] $PID ch=%d 未下发：暂不打通（固件 0x01 仅 ch0~%d"  # 打印未下发的原因
                     "=Pitch/Yaw/Roll/Depth，与上位机 12 通道语义不一致）" % (pid["ch"], self.pid_max_ch))  # 说明固件 0x01 只认 ch0~3（Pitch/Yaw/Roll/Depth）

    # ---------------- 周期 ----------------
    def tick(self, now, dt):  # 周期钩子（按 TICK_HZ 调用）
        """ROV 期间按 STM32_POLL_HZ 持续请求下位机回传（0x0C）；控制律暂为预留"""
        self.pump_telemetry(now)  # 按 STM32_POLL_HZ 请求 0x0C 遥测回传

    # ---------------- 遥测 ----------------
    def on_downlink(self, tel):  # 收到下位机遥测时的回调
        self.last_tel = tel  # 缓存最新遥测供 $TEL 组包

    def build_telemetry(self):  # 构造上行给上位机的 $TEL
        """$TEL 41 字段：统一走 tel_builder 的**唯一一份**映射（2026-09-15 起不再自建）

        此前本文件与 tel_builder.tel_values() 各存一份映射，改一处会漂移（README U8）。
        """
        if not self.last_tel:  # 还没有遥测则不组包
            return None  # 无遥测返回 None，编排层会跳过上行
        return TB.build_tel(self.last_tel)  # 交给 tel_builder 生成 41 字段 $TEL