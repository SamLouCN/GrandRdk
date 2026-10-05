# -*- coding: utf-8 -*-
"""IDLE 待命模式 —— 上电后**不进入 ROV/AUV**，等上位机下发 `$CMD.mode` 再进入。

背景（2026-10-04 用户需求）:
    run.sh 拉起整栈后，中位机不应自动进入有线 ROV 或 AUV 模式；
    必须等上位机按下「有线遥控 / 自主航行」按钮、下发对应 `$CMD.mode` 后才进入。

本模式的行为:
    - on_enter : **不下发 0x04 模式帧**（固件保持原状，中位机不擅自改它）；
                 也不主动请求 0x0C 遥测（避免待命期就打扰下位机）。
    - on_cmd   : **不下发任何 0x09 运动帧**（待命期四轴视为 0，安全）；
                 仅记录首帧杆位，供日志观察。
                 ⚠ 编排层 `_on_cmd` 会在调用本方法**之前**处理 mode 字段 ——
                 一旦收到带 mode 的 $CMD，会先把中位机切到目标模式（本模式 on_exit），
                 所以这里通常拿不到"带 mode 的 $CMD"。
    - tick     : 空跑（不请求 0x0C、不作动）。
    - build_telemetry : 发一份 **IDLE 专用占位 $TEL**（41 字段全 0），
                 让上位机知道"板端在线但未进入任何模式"，避免上位机因零帧而误判离线。

设计取舍:
    - 待命期保留 $TEL 上行（全部 0 占位）—— 上位机靠 `_last_tel_time` 判在线，断流会被当成断连。
    - 待命期**不请求 0x0C**：下位机不被打扰；$TEL 也不是实测值。
      一旦上位机切到 ROV/AUV，对应模式 on_enter 会自行请求 0x0C 恢复实测回传。
"""
import time  # 时间戳（记录占位 $TEL 最近发送时刻，便于观察）

import protocol as P  # 组 $TEL 文本帧（41 字段，缺字段补 0）
import to32_config as C  # 配置常量（MODE_IDLE 等）
from mode_base import ModeBase  # 模式基类


class IdleMode(ModeBase):
    """待命模式：不进入 ROV/AUV，不发模式帧与运动帧，只回占位 $TEL。"""

    id = C.MODE_IDLE  # 模式 ID（-1，负值，不与 ROV=0 / AUV=1 冲突）
    name = "IDLE"  # 模式名（日志与 $TEL 上报用）
    desc = "待命：等上位机下发 $CMD.mode 再进入 ROV/AUV"  # 模式描述

    def __init__(self, ctx):  # 构造函数
        super().__init__(ctx)  # 基类初始化（ctx / log / cfg / link_stm32）
        self._idle_tel_ts = 0.0  # 最近一次发占位 $TEL 的时刻
        self._idle_tel_count = 0  # 已发占位 $TEL 帧数
        self._first_cmd_logged = False  # 是否已打印"收到首帧 $CMD"提示
        self.dropped = {"led": 0, "grab": 0, "store": 0, "pid": 0}  # 与 ROV 一致的丢弃计数表

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):  # 进入待命
        super().on_enter(prev_id)  # 基类日志（"→ 进入 IDLE"）
        # ⚠ 关键：**不下发 0x04 模式帧**，也不请求 0x0C —— 待命期不打扰下位机。
        self.log("[IDLE] 待命：不进入 ROV/AUV，不发 0x04/0x09/0x0C，等上位机下发 $CMD.mode")  # 明确待命语义

    def on_exit(self, next_id):  # 离开待命（被切到 ROV/AUV）
        super().on_exit(next_id)  # 基类日志（"← 退出 IDLE"）
        try:  # 目标模式名仅用于日志
            nm = self.ctx.modes[int(next_id)].name  # 取目标模式名
        except Exception:  # 转换失败不影响退出
            nm = next_id  # 退回原值
        self.log("[IDLE] 收到模式指令 -> 退出待命，交给目标模式（%s）接管" % nm)  # 说明交接

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):  # 待命期收到 $CMD
        """待命期收到 $CMD：**不发任何 0x09 运动帧**（四轴视为 0，安全）。

        正常路径下，带 mode 的 $CMD 会在编排层 `_on_cmd` 里先把本模式切走，
        所以本方法多半只在"已进入其它模式前的一瞬"被调用；即便被调用也绝不作动。
        """
        if not self._first_cmd_logged:  # 仅首次提示，避免 20Hz 刷屏
            self._first_cmd_logged = True  # 置位
            self.log("[IDLE] 收到上位机 $CMD（杆位 surge=%.2f sway=%.2f heave=%.2f yaw=%.2f）"  # 打印首帧杆位
                     "，待命期不下发 0x09" % (cmd.get("surge", 0.0), cmd.get("sway", 0.0),  # surge/sway
                                             cmd.get("heave", 0.0), cmd.get("yaw", 0.0)))  # heave/yaw

    def on_pid(self, pid):  # 待命期收到 $PID
        """待命期不下发 $PID（与 ROV 的决策 D 一致：暂不打通）。"""  # 说明
        self.dropped["pid"] = self.dropped.get("pid", 0) + 1  # 计数

    def on_vid(self, on):  # 待命期收到 $VID
        """图像开关是全局的，与模式无关 —— 编排层已处理，这里什么都不做。"""  # 说明
        pass  # 无操作

    def on_downlink(self, tel):  # 待命期收到下位机遥测（正常不会发生：未请求 0x0C）
        pass  # 忽略：待命期不做闭环

    # ---------------- 周期 ----------------
    def tick(self, now, dt):  # 待命期周期钩子
        """待命期**不请求 0x0C**、不作动、不集成 —— 保持下位机与水面安静。"""  # 说明
        pass  # 空跑

    # ---------------- 遥测 ----------------
    def build_telemetry(self):  # 组装待命期占位 $TEL
        """待命期 $TEL：41 字段全 0，让上位机知道板端在线（靠 `_last_tel_time` 判连）。

        ⚠ 这不是实测值，是占位。一旦进入 ROV/AUV，对应模式会用自己的 build_telemetry。
        """
        self._idle_tel_ts = time.time()  # 记录发送时刻
        self._idle_tel_count += 1  # 计数
        if self._idle_tel_count == 1:  # 仅首次打日志
            self.log("[IDLE] 开始上行占位 $TEL（41 字段全 0，非实测）")  # 说明
        return P.build_tel([0.0] * P.TEL_FIELD_COUNT)  # 41 个 0.00 的 $TEL 文本帧
