# -*- coding: utf-8 -*-
"""中位机编排核心 —— 模式注册/切换 + 安全 + 周期调度 + 遥测上行。

三机分层（命名见《上位机通讯协议.md》）:
    上位机(PC UI)  ──UDP 8080/8081──▶  link_pc.py      只收发文本帧(自带 PING->PONG / $TEL 出口)
    模式逻辑       ──mode_rov / mode_auv──▶  各模式的运行逻辑写在这里
    下位机(STM32)  ──V2 串口帧──────▶  link_stm32.py   只收发 V2 帧
    本文件         把三者接起来: 事件派发 / $CMD.mode 切模式 / 安全三件套 / tick / $TEL

本文件不含任何"模式自己的逻辑"; 模式的运行逻辑请写在 mode_rov.py / mode_auv.py。
[2026-10-04 变更] AUV 的自主运动逻辑已整体移到 ./move_test/（主工程不再调用）；
本文件注册的是内联的 AuvModeStub（只切模式、不发 0x09）。
新增一个模式三步:
    1) 新建 mode_xxx.py 继承 mode_base.ModeBase, 实现 on_enter/on_exit/on_cmd/tick/on_downlink
    2) 在 config.py 登记 MODE_XXX（$CMD 的 mode 字段取值）
    3) 在本文件 _register_modes() 里加入
"""
from __future__ import annotations  # 启用延迟求值注解，兼容新式类型写法

import json  # 读写模式记忆文件（JSON）
import os  # 目录创建与原子替换 os.replace
import queue  # 上位机事件队列及 Empty 异常
import threading  # 后台线程与停止事件
import time  # 时间戳、节拍与睡眠

import link_stm32 as S  # 下位机 V2 帧收发，S 作帧构造命名空间
import tel_builder as TB  # $TEL 统一映射组帧（回落用）
from mode_base import ModeBase  # 模式基类（AUV 占位壳继承它）
from mode_idle import IdleMode  # [2026-10-04] IDLE 待命模式（上电默认，等上位机 $CMD.mode）
from mode_rov import RovMode  # ROV 遥控模式实现

# [2026-10-04 AUV 运动逻辑摘除] 原 `from mode_auv import AuvMode` 已移除：
#   AUV 的一整套自主运动逻辑（mission.py 状态机 + vision_if/depth_if/viskf_if 观测接口
#   + auv_report.py 状态回传 + kalman_launcher.py 卡尔曼托管 + mode_auv.py 模式壳）
#   已整体搬到 ./move_test/ 目录，主工程**不再调用**。
#   本文件改为注册下方内联的 AuvModeStub —— 壳还在（切 AUV 仍下发 0x04=0x05 让下位机进
#   AUV 语义），但**没有任何自主运动（不发 0x09）**。
#   要把自主逻辑接回来：见 src/to32/move_test/README_move_test.md。


class AuvModeStub(ModeBase):  # [2026-10-04] AUV 占位模式：只切模式，不跑自主运动
    """AUV 模式的空壳。

    原 AuvMode 在 tick() 里驱动 17 阶段任务状态机并把控制量组装成 0x09 下发；
    这套逻辑已移到 ./move_test/。本壳保留"切模式"这一必要动作：
      - on_enter 下发 0x04 = 0x05，让下位机进入 AUV 语义（与固件约定）；
      - tick 不做任何事 —— **不发 0x09**，即 AUV 期间无自主运动；
      - 其余回调照基类默认（忽略手动杆位等语义由 ModeBase 提供）。
    这样即便上位机切到 mode=1，机器人也不会自己动；运动必须由别的途径下发。
    """

    id = -1  # 占位；下面用 config.MODE_AUV 覆盖，避免在类体里读配置
    name = "AUV"
    desc = "自主模式（已摘除运动逻辑，仅切模式）"

    def on_enter(self, prev_id):  # 进入 AUV：只切下位机模式，不起任何自主逻辑
        super().on_enter(prev_id)  # 基类进入处理（置 active / 记日志）
        # 下发 0x04 模式帧 = 0x05 AUV，让下位机切到 AUV 语义（原 mode_auv 的做法，保留）
        self.send_downlink(S.frame_mode(S.MODE_AUV), "0x04 AUV")
        self.log("[AUV] 运动逻辑已摘除（见 move_test/）—— 仅切换下位机模式，不发 0x09")

    def tick(self, now, dt):  # 周期入口：空实现，绝不发 0x09
        # 仍然按节拍请求遥测（0x0C），让上位机 $TEL 不断流；这只涉及回传，与运动无关。
        self.pump_telemetry(now, note="0x0C 遥测请求(AUV)")

    def build_telemetry(self):  # 返回 None = 回落 tel_builder 统一映射，$TEL 不断流
        return None


class Dispatcher(object):  # 编排核心；同时充当各模式回调的 ctx
    """编排核心；同时充当各模式的 ctx（模式只使用 ctx.log / ctx.cfg / ctx.link_stm32 / ctx.estop_latch）。"""

    def __init__(self, cfg, pc_link, link_stm32, log, rx_queue=None, video=None):  # cfg=配置 log=日志回调 video 可选
        self.cfg = cfg  # 保存配置对象，供模式读取各类常量
        self.pc_link = pc_link  # 上位机 UDP 链路
        self.link_stm32 = link_stm32  # 下位机串口链路
        self.video = video          # 图像回传服务（可选；None = 本进程不做图像）
        self.rx_queue = rx_queue if rx_queue is not None else queue.Queue()  # 外部没传队列就自建一个
        self._log_fn = log  # 日志输出函数（通常由 main 注入）

        self.modes = {}  # 模式表：模式 id -> 模式实例
        self._register_modes()  # 注册全部可用模式
        # 日志状态必须先就绪：_resolve_initial_mode() 会写日志（记忆命中/记忆文件损坏）
        self._log_last = ("", 0.0)  # 上一条日志内容与时间，用于去重
        self._log_dup = 0  # 被抑制的重复日志条数
        # 启动模式：命令行 --mode > 模式记忆文件 > config.START_MODE / DEFAULT_MODE
        self.mode_state_path = str(getattr(cfg, "MODE_STATE_PATH", "") or "")  # 模式记忆文件路径，空串=不落盘
        self.mode_persist = bool(getattr(cfg, "MODE_PERSIST", False))  # 是否开启模式记忆
        self.mode_id = self._resolve_initial_mode()  # 解析出启动模式 id

        # 运行状态
        self.last_cmd_ts = 0.0  # 最后一次收到上位机 $CMD 的时刻（deadman 依据）
        self.last_tel = None  # 最近一帧下位机遥测
        self.anchored = False  # 是否已用首帧遥测锚定目标量
        self.estop_latch = False  # 急停锁存标志
        self.estop_reason = ""  # 触发急停的原因描述

        self.stats = {  # 运行统计计数器
            "cmd": 0, "pid": 0, "vid": 0, "unknown": 0, "bad_mode": 0,  # 上位机各类帧与非法模式计数
            "tel": 0, "tel_tx": 0, "switch": 0,  # 遥测接收/上行与模式切换次数
            "tx_total": 0, "tx_mode": 0, "tx_motion": 0, "tx_poll": 0, "tx_other": 0,  # 下行帧按功能码分类计数
        }

        self._stop = threading.Event()  # 后台线程的停止信号
        self._threads = []  # 已启动的后台线程列表
        self._log_last = ("", 0.0)  # 再初始化一次，保证日志去重可用
        self._log_dup = 0  # 重复计数归零
        self._wrap_link()  # 包装下行 send 以统计帧数

    # ------------------------------------------------------------------ #
    # 日志 / 统计
    # ------------------------------------------------------------------ #
    def log(self, msg, min_gap=1.0):  # 对外日志入口：min_gap 秒内同文只计次数
        """写日志；同一行 min_gap 秒内重复只计数，避免 ROV 抑制 0x09 时 20Hz 刷屏。"""  # 去重策略说明
        now = time.time()  # 当前时间戳
        last_msg, last_t = self._log_last  # 取出上一条日志的内容与时间
        if msg == last_msg and (now - last_t) < min_gap:  # 内容相同且间隔过短则抑制输出
            self._log_dup += 1  # 累计被抑制的条数
            self._log_last = (msg, now)  # 刷新时间戳，形成滑动窗口
            return  # 抑制期内不打印
        self._flush_dup()  # 换话题前先补打"重复 N 次"
        self._log_fn(msg)  # 真正输出这一行
        self._log_last = (msg, now)  # 记录本次日志内容与时间

    def _flush_dup(self):  # 输出被抑制的重复条数并清零
        if self._log_dup:  # 只有积压过才需要补打印
            n, self._log_dup = self._log_dup, 0  # 取出并立即清零，避免重入重复打印
            self._log_fn("      ... 上一行重复 %d 次" % n)  # 汇总一行，既不刷屏又留痕

    def _wrap_link(self):  # 给下行 send 包一层计数
        """统计下行帧数：在实例上包一层 send()，不改 link_stm32.py 本体。"""  # 采用实例级包装而非改源文件
        if getattr(self.link_stm32, "_rdk_counted", False):  # 已包装过则跳过，防止重复叠加
            return  # 幂等返回
        self._orig_send = self.link_stm32.send  # 先保存原始发送函数

        def _send(frame, note=""):  # 替代原 send 的包装函数
            self._count_tx(frame)  # 先统计功能码再转发
            return self._orig_send(frame, note)  # 调用原始发送

        self.link_stm32.send = _send  # 用包装函数接管实例属性
        self.link_stm32._rdk_counted = True  # 打标记，避免二次包装

    def _unwrap_link(self):  # 退出前还原被包装的 send
        self._flush_dup()  # 收尾时补齐被抑制的重复日志
        if getattr(self.link_stm32, "_rdk_counted", False):  # 只有确实包装过才需要拆
            for attr in ("send", "_rdk_counted"):  # 删掉包装方法与标记
                try:  # 属性可能已被外部删掉
                    delattr(self.link_stm32, attr)  # 移除实例属性，重新暴露类方法
                except Exception:  # 删除失败无所谓
                    pass  # 保持静默

    def _count_tx(self, frame):  # 按下行帧功能码累加统计
        if not frame:  # 空帧不计入统计
            return  # 直接返回
        self.stats["tx_total"] += 1  # 下行帧总数
        func = frame[2] if len(frame) > 2 else -1  # 第 3 字节是功能码，帧太短按未知处理
        if func == S.FUNC_MODE:  # 0x04 模式帧
            self.stats["tx_mode"] += 1  # 模式帧计数
        elif func == S.FUNC_MOTION:  # 0x09 运动帧
            self.stats["tx_motion"] += 1  # 运动帧计数
        elif func == S.FUNC_TELEMETRY:  # 0x0C 轮询遥测帧
            self.stats["tx_poll"] += 1  # 轮询帧计数
        else:  # 其他功能码
            self.stats["tx_other"] += 1  # 其他帧计数

    # ------------------------------------------------------------------ #
    # 模式
    # ------------------------------------------------------------------ #
    def _register_modes(self):  # 注册所有可用模式
        # [2026-10-04] 先注册 IDLE 待命模式：上电默认进入，等上位机 $CMD.mode 才切走。
        auv = AuvModeStub(self)  # [2026-10-04 AUV 运动逻辑摘除] 用内联占位壳替代原 AuvMode
        auv.id = getattr(self.cfg, "MODE_AUV", -1)  # 模式号从配置取（类体里读配置不便，故在此覆盖）
        self.modes[auv.id] = auv  # 以模式 id 为键入表
        for m in (IdleMode(self), RovMode(self)):  # 构造时把 self 作为 ctx 传入模式
            self.modes[m.id] = m  # 以模式 id 为键入表

    # ------------------------------------------------------------------ #
    # 启动模式：可配置 + 模式记忆（下次启动沿用上位机最后切换的模式）
    # ------------------------------------------------------------------ #
    def _config_start_mode(self):  # 读取 config 中配置的启动模式
        """config 里的启动模式（命令行 --mode 会以 MODE_FORCE_START 形式覆盖到这里）"""  # 说明与命令行参数的关系
        v = getattr(self.cfg, "START_MODE", None)  # 优先取 START_MODE
        if v is None:  # 未配置则逐级回退
            v = getattr(self.cfg, "DEFAULT_MODE", getattr(self.cfg, "MODE_ROV", 0))  # 再回退 DEFAULT_MODE，最后用 ROV 常量
        try:  # 转换可能失败（非数字字符串/None）
            return int(v)  # 转成整型模式 id
        except (TypeError, ValueError):  # 非法取值
            self.log("[MODE] START_MODE=%r 非法 -> 回退 ROV" % (v,))  # 打日志说明回退原因
            return int(getattr(self.cfg, "MODE_ROV", 0))  # 兜底返回 ROV 模式

    def load_saved_mode(self):  # 读取上次记忆的模式
        """读模式记忆文件；未开启/文件不存在/损坏/模式未注册 -> None"""  # 返回值约定
        if not (self.mode_persist and self.mode_state_path):  # 未开启记忆或路径为空
            return None  # 当作无记忆
        try:  # 文件可能不存在或内容损坏
            with open(self.mode_state_path, "r", encoding="utf-8") as f:  # 以 UTF-8 打开记忆文件
                obj = json.load(f)  # 解析 JSON
            mid = int(obj.get("mode"))  # 取出模式 id（缺失时会抛异常走兜底）
        except FileNotFoundError:  # 首次运行没有记忆文件
            return None  # 按无记忆处理
        except Exception as e:  # 其他读取/解析错误
            self.log("[MODE] 记忆文件读取失败(%s): %s -> 忽略" % (self.mode_state_path, e))  # 告警但不阻断启动
            return None  # 忽略损坏的记忆
        if mid not in self.modes:  # 记忆里的模式可能已下线
            self.log("[MODE] 记忆文件里的模式 %r 未注册 -> 忽略" % (mid,))  # 提示该模式未注册
            return None  # 忽略未知模式
        return mid  # 返回可用的记忆模式

    def save_mode(self, mode_id):  # 把当前模式写入记忆文件
        """把模式写入记忆文件（原子写：临时文件 + os.replace）"""  # 说明写入方式的可靠性
        if not (self.mode_persist and self.mode_state_path):  # 未开启记忆直接跳过
            return False  # 未写入
        try:  # 磁盘写入可能失败
            d = os.path.dirname(os.path.abspath(self.mode_state_path))  # 记忆文件所在目录（绝对路径）
            if d and not os.path.isdir(d):  # 目录不存在则创建
                os.makedirs(d, exist_ok=True)  # 递归创建，已存在不报错
            tmp = self.mode_state_path + ".tmp"  # 先写同目录临时文件
            with open(tmp, "w", encoding="utf-8") as f:  # 打开临时文件准备写入
                json.dump({"mode": int(mode_id),  # 写入模式 id
                           "name": self.modes[int(mode_id)].name,  # 写入模式名，便于人工查看
                           "ts": time.strftime("%Y-%m-%d %H:%M:%S"),  # 写入写入时刻
                           "source": "pc"}, f, ensure_ascii=False)  # 标记来源为上位机切换，中文不转义
                f.write("\n")  # 结尾补换行，方便 cat 查看
                f.flush()  # 刷到操作系统缓冲
                os.fsync(f.fileno())  # 强制落盘，防断电丢内容
            os.replace(tmp, self.mode_state_path)  # 原子替换，避免读者读到半截文件
            return True  # 写入成功
        except Exception as e:  # 任何写入异常都只告警
            self.log("[MODE] 记忆文件写入失败(%s): %s" % (self.mode_state_path, e))  # 记录失败原因
            return False  # 写入失败

    def _resolve_initial_mode(self):  # 启动模式优先级解析
        """启动模式解析：--mode(MODE_FORCE_START) > 记忆文件 > config.START_MODE

        [2026-10-04] config.START_MODE 默认已改为 MODE_IDLE —— 上电停在待命态，
        等上位机下发带 mode 的 $CMD 才进入 ROV/AUV。
        """
        if bool(getattr(self.cfg, "MODE_FORCE_START", False)):  # 命令行 --mode 强制指定
            self.mode_source = "forced"  # 标记来源，便于上报与排查
            return self._config_start_mode()  # 直接用强制模式
        saved = self.load_saved_mode()  # 尝试读模式记忆文件
        if saved is not None:  # 有可用记忆
            self.mode_source = "saved"  # 标记为记忆启动
            self.log("[MODE] 沿用记忆模式启动: %s（来自 %s）"  # 日志前半段：说明沿用记忆
                     % (self.modes[saved].name, self.mode_state_path))  # 模式名与记忆文件路径
            return saved  # 沿用记忆模式
        self.mode_source = "start"  # 标记为配置启动
        return self._config_start_mode()  # 用 config 的启动模式

    def mode(self):  # 返回当前模式实例
        return self.modes[self.mode_id]  # 按当前模式 id 取实例

    def switch_mode(self, new_id, initial=False):  # 切模式；initial=True 表示启动首次进入
        """切换模式：调用旧模式 on_exit、新模式 on_enter（新模式在 on_enter 里下发 0x04 模式码）。"""  # 切换时序说明
        try:  # 模式 id 可能是字符串或非法值
            new_id = int(new_id)  # 统一转成整型
        except (TypeError, ValueError):  # 无法解析成模式 id
            self.stats["bad_mode"] += 1  # 记一次非法模式
            return False  # 切换失败
        if new_id not in self.modes:  # 模式未注册
            self.stats["bad_mode"] += 1  # 记一次未知模式
            self.log("[MODE] 未知模式 %r（已知 %s）-> 忽略" % (new_id, sorted(self.modes)))  # 打印已知模式列表
            return False  # 忽略未知模式
        if not initial and new_id == self.mode_id:  # 非初始化且目标就是当前模式
            return False  # 未发生切换
        old_id = self.mode_id  # 记住旧模式 id，供 on_exit/on_enter 传参
        if not initial:  # 初始化时没有旧模式需要退出
            self.modes[old_id].on_exit(new_id)  # 旧模式收尾（停输出/复位）
        self.mode_id = new_id  # 先更新 id，模式内部读到的即为新模式
        self.stats["switch"] += 1  # 切换次数累加
        # [2026-10-04] IDLE 待命期抑制链路层兜底 0x0C 补发（下位机保持安静）。
        idle_id = int(getattr(self.cfg, "MODE_IDLE", -1))  # 待命态 id
        self.link_stm32.poll_suspended = (new_id == idle_id)  # 进 IDLE 置位，出 IDLE 清位
        self.modes[new_id].on_enter(old_id)  # 新模式进入并下发 0x04 模式码
        self.log("[MODE] 当前模式 -> %s" % self.modes[new_id].name)  # 打印当前模式名
        if not initial:  # 仅上位机触发的切换才落记忆
            # 上位机切换的模式写入记忆文件：下次启动沿用
            # （可用 --mode 强制覆盖 / --no-mode-persist 关闭记忆）
            if self.save_mode(new_id):  # 写入模式记忆文件
                self.log("[MODE] 已记忆模式 %s -> %s"  # 记忆成功提示的前半段
                         % (self.modes[new_id].name, self.mode_state_path))  # 模式名与文件路径
        return True  # 切换成功

    # ------------------------------------------------------------------ #
    # 上位机事件（link_pc 线程填队列 -> bridge 线程取 -> 这里）
    # ------------------------------------------------------------------ #
    def on_pc_event(self, kind, payload):  # 上位机事件统一入口
        if kind == "cmd":  # $CMD 指令帧
            self._on_cmd(payload)  # 处理指令与模式切换
        elif kind == "pid":  # $PID 调参帧
            self.stats["pid"] += 1  # PID 帧计数
            self.mode().on_pid(payload)  # 交给当前模式处理
        elif kind == "vid":  # $VID 图像开关帧
            self.stats["vid"] += 1  # 图像帧计数
            on = bool(payload)  # 统一成布尔开关量
            # 图像回传是"全局开关"，与 ROV/AUV 模式无关：先切服务，再通知模式
            if self.video is not None:  # 有图像服务才操作
                try:  # 服务开关可能抛异常
                    self.video.set_enabled(on)  # 开/停图像回传
                except Exception as e:  # 开关异常不应影响主链路
                    self.log("[ERR] 图像回传开关异常: %s" % e)  # 打印异常详情
            self.mode().on_vid(on)  # 无论服务是否存在都通知模式
        elif kind == "raw":  # 未解析的裸文本帧
            self._on_raw(payload)  # 兜底解析 $ESTOP 之类
        else:  # 未知事件类型
            self.stats["unknown"] += 1  # 未识别计数

    def _on_cmd(self, cmd):  # 处理上位机 $CMD
        self.stats["cmd"] += 1  # 指令帧计数
        self.last_cmd_ts = time.time()  # 刷新 deadman 心跳时间
        if self.estop_latch and not bool(getattr(self.cfg, "ESTOP_LATCH", True)):  # 未开真锁存时，新指令自动解除
            self.estop_latch = False  # 清除锁存
            self.log("[ESTOP] 收到新 $CMD -> 解除急停锁存")  # 打印解除提示
        # [2026-10-04] 待命态（IDLE）特殊处理：
        #   只有**显式带 mode 字段**的 $CMD 才把中位机从 IDLE 切走；
        #   否则（老格式 $CMD 只有 6 字段）保持待命，避免"缺字段默认成 ROV=0"误激活。
        id_modes = getattr(self.cfg, "MODE_IDLE", -1)  # 待命态 id
        if self.mode_id == id_modes:  # 当前处于待命态
            if cmd.get("mode_explicit"):  # 帧里显式带了 mode 字段
                want = int(cmd.get("mode", 0))  # 目标模式 id
                if want in self.modes and want != id_modes:  # 是已注册的实体模式
                    self.log("[MODE] 待命态收到 $CMD.mode=%d -> 激活 %s"  # 打印激活
                             % (want, self.modes[want].name))  # 目标模式名
                    self.switch_mode(want)  # 从 IDLE 切到目标模式
                    self.mode().on_cmd(cmd)  # 把本帧交给新模式（首帧立即生效）
                    return  # 已处理，避免重复 on_cmd
            self.mode().on_cmd(cmd)  # 待命态：不切模式，仅记录（IdleMode.on_cmd 不下发 0x09）
            return  # 待命期到此为止，不做后续模式切换
        want = int(cmd.get("mode", getattr(self.cfg, "MODE_ROV", 0)))  # 目标模式 id，缺省按 ROV
        if want != self.mode_id:  # 目标与当前不一致
            self.switch_mode(want)  # 执行模式切换
        self.mode().on_cmd(cmd)  # 交给当前模式执行具体指令

    def _on_raw(self, text):  # 兜底处理未识别的文本帧
        """未识别的文本帧：只认扩展的 $ESTOP# / $ESTOP,0#（v3.2 UI 不发这两个帧，留作人工兜底）。"""  # 支持的兜底帧
        s = (text or "").strip()  # 去除首尾空白，兼容 None
        if s == "$ESTOP#":  # 显式急停帧
            self.trigger_estop("上位机 $ESTOP（显式急停帧，来自 %s）" % self.pc_link.pc_ip())  # 触发急停并记录来源 IP
        elif s.startswith("$ESTOP,") and s.endswith("#"):  # $ESTOP,<值># 形式
            try:  # 载荷可能不是数字
                if int(float(s[7:-1])) == 0:  # 取逗号后到 # 前的数值，0 表示解除
                    self.release_estop("上位机 $ESTOP,0")  # 解除急停
            except ValueError:  # 数值解析失败
                pass  # 忽略非法帧
        else:  # 其他文本帧
            self.stats["unknown"] += 1  # 记为未识别

    # ------------------------------------------------------------------ #
    # 下位机遥测（link_stm32 读线程 -> 这里）
    # ------------------------------------------------------------------ #
    def on_stm32_telemetry(self, tel):  # 下位机遥测回调
        if not tel:  # 空遥测直接忽略
            return  # 不做后续处理
        self.stats["tel"] += 1  # 遥测帧计数
        self.last_tel = tel  # 保存最新遥测，供 $TEL 与状态上报
        if not self.anchored:  # 首次收到遥测需要锚定
            self._anchor(tel)  # 用实测值初始化模式内部目标量
        try:  # 模式处理异常不能拖垮串口读线程
            self.mode().on_downlink(tel)  # 交给模式做闭环计算
        except Exception as e:  # 捕获模式异常
            self.log("[ERR] %s.on_downlink 异常: %s" % (self.mode().name, e))  # 打印异常

    def _anchor(self, tel):  # 冷启动锚定目标量
        """冷启动锚定：模式内部的目标量取首帧实测值，避免一上电/切模式就"回水面、转向"。"""  # 锚定的目的
        m = self.mode()  # 当前模式实例
        done = []  # 记录本模式实际锚定了哪些目标量
        for attr, key in (("target_depth_cm", "actual_depth_cm"),  # 深度目标 <- 实测深度
                          ("target_yaw_deg", "actual_yaw")):  # 航向目标 <- 实测航向
            if hasattr(m, attr):  # 模式有该目标量才锚定
                setattr(m, attr, float(tel.get(key, 0.0)))  # 用首帧实测值覆盖目标
                done.append(attr)  # 记入锚定清单
        self.anchored = True  # 只锚定一次
        self.log("[ANCHOR] 首帧遥测 -> %s 锚定到实测值%s" % (  # 打印锚定结果的前半段
            m.name, ("（%s）" % ",".join(done)) if done else "（本模式无内部目标量）"))  # 模式名与锚定字段清单

    # ------------------------------------------------------------------ #
    # 安全
    # ------------------------------------------------------------------ #
    def trigger_estop(self, reason):  # 触发急停并锁存
        """急停：下发 0x04 0x01 并锁存。锁存后模式侧会抑制 0x09（0x09 无 STANDBY 守卫）。"""  # 锁存的必要性
        self.link_stm32.send(S.frame_mode(S.MODE_ESTOP), "0x04 急停")  # 立即下发 0x04 + 0x01
        self.estop_latch = True  # 置锁存，抑制后续 0x09 输出
        self.estop_reason = reason  # 记录原因，便于上报排查
        self.log("[ESTOP] %s -> 已下发 0x04 0x01，锁存并抑制 0x09" % reason)  # 打印急停信息

    def release_estop(self, reason="手动解除"):  # 解除急停锁存
        if not self.estop_latch:  # 本来就没锁存
            return  # 无需处理
        self.estop_latch = False  # 清锁存，恢复模式输出
        self.estop_reason = ""  # 清空原因
        self.last_cmd_ts = time.time()          # deadman 重新起算
        # [2026-10-03] 急停解除时显式下发 START(0x00)：固件要求 STANDBY→运行需 0x00，
        # 否则下位机停在 STANDBY，后续 0x09 虽能拉起但在新固件下有 STANDBY 门控。
        self.link_stm32.send(S.frame_mode(S.MODE_START), "0x04 START(急停解除)")  # 恢复 Last_Con_Mode
        self.log("[ESTOP] %s -> 解除锁存，已下发 START(0x00) 恢复 Last_Con_Mode" % reason)  # 打印解除信息

    def _check_deadman(self, now):  # 下行静默看门狗检查
        """下行静默看门狗：上位机(或链路)静默超过 ESTOP_TIMEOUT_S 自动急停。0=关闭。"""  # 超时阈值含义
        timeout = float(getattr(self.cfg, "ESTOP_TIMEOUT_S", 0.0) or 0.0)  # 超时秒数，0 表示关闭
        if timeout <= 0 or self.estop_latch or self.last_cmd_ts <= 0:  # 未开启/已急停/从未收到指令
            return  # 不做检查
        silent = now - self.last_cmd_ts  # 已静默时长
        if silent > timeout:  # 超过阈值
            self.trigger_estop("下行静默 %.1fs" % silent)  # 自动触发急停

    # ------------------------------------------------------------------ #
    # 周期
    # ------------------------------------------------------------------ #
    def _send_tel(self):  # 组装并上行一帧 $TEL
        """$TEL 上行：优先用当前模式自己的组帧，模式没提供则回落到统一映射（无遥测则不发）。"""  # 组帧优先级
        mode = self.mode()  # 当前模式
        txt = None  # 待上行的遥测文本
        try:  # 模式组帧可能抛异常
            txt = mode.build_telemetry()  # 优先用模式自己的组帧
        except Exception as e:  # 组帧失败
            self.log("[ERR] %s.build_telemetry 异常: %s" % (mode.name, e))  # 打印异常
        if not txt:  # 模式没提供或未实现
            txt = TB.build_tel(self.last_tel)  # 回落到统一映射组帧
        txt = TB.append_fusion_fields(txt)  # v3.6: 帧尾追加融合深度/净空（无效/缺失原样返回）
        if txt and self.pc_link.send_telem(txt):  # 有内容且发送成功
            self.stats["tel_tx"] += 1  # 上行帧计数

    def _bridge_loop(self):  # 上位机事件分发线程主循环
        while not self._stop.is_set():  # 未收到停止信号就一直循环
            try:  # 队列取数可能超时
                item = self.rx_queue.get(timeout=0.05)  # 50ms 超时，保证能及时响应停止
            except queue.Empty:  # 本轮无数据
                continue  # 继续下一轮
            except Exception:  # 队列异常（如已关闭）
                break  # 退出线程
            try:  # 派发异常不能杀掉线程
                kind, payload = item  # 拆出事件类型与负载
                self.on_pc_event(kind, payload)  # 分发到具体处理逻辑
            except Exception as e:  # 捕获处理异常
                self.log("[ERR] 上位机事件处理异常: %s" % e)  # 打印异常

    def _tick_loop(self):  # 周期线程：tick / 遥测 / 上报
        tick_hz = max(1.0, float(getattr(self.cfg, "TICK_HZ", getattr(self.cfg, "CMD_HZ", 20.0))))  # 模式 tick 频率，兜底 20Hz
        tel_hz = max(0.0, float(getattr(self.cfg, "TEL_HZ", 10.0)))  # $TEL 上行频率，0 表示关闭
        report_s = float(getattr(self.cfg, "REPORT_S", 5.0) or 0.0)  # 状态上报周期，0 表示关闭
        step = 1.0 / tick_hz  # 每轮休眠时长
        acc_tel = acc_rep = 0.0  # 遥测与上报的时间累加器
        last = time.time()  # 上一轮时间戳
        while not self._stop.is_set():  # 周期主循环
            now = time.time()  # 本轮时间戳
            dt = min(0.25, max(0.0, now - last))  # 时间间隔并限幅，防长卡顿后步长过大
            last = now  # 滚动时间戳
            self._check_deadman(now)  # 每轮检查下行是否静默超时
            acc_tel += dt  # 累加遥测计时
            acc_rep += dt  # 累加上报计时
            # 急停锁存期间不跑模式 tick：模式输出已被抑制，只保留看门狗与遥测上行
            if not self.estop_latch:  # 未急停时才跑模式逻辑
                try:  # tick 异常不能杀线程
                    self.mode().tick(now, dt)  # 驱动模式周期计算与下发
                except Exception as e:  # 捕获 tick 异常
                    self.log("[ERR] %s.tick 异常: %s" % (self.mode().name, e))  # 打印异常
            if tel_hz > 0 and acc_tel >= 1.0 / tel_hz:  # 到点上行一次 $TEL
                acc_tel = 0.0  # 重置累加器（不减周期，长周期会略漂移）
                self._send_tel()  # 发送遥测上行
            if report_s > 0 and acc_rep >= report_s:  # 到点打印运行状态
                acc_rep = 0.0  # 重置上报计时
                self.report()  # 输出运行报告
            time.sleep(step)  # 按固定节拍休眠
        self._flush_dup()  # 退出前补打被抑制的重复日志

    # ------------------------------------------------------------------ #
    # 上报
    # ------------------------------------------------------------------ #
    def report(self):  # 打印中位机整体运行状态
        s = self.stats  # 统计字典简写
        m = self.mode()  # 当前模式
        self.log("---- 中位机 ----")  # 报告标题行
        self.log("  模式 %s | 锚定 %s | 急停 %s%s" % (  # 模式 / 锚定 / 急停三态
            m.name, "已锚定" if self.anchored else "未锚定",  # 模式名与锚定标记
            "锁存" if self.estop_latch else "正常",  # 急停状态文字
            (" (%s)" % self.estop_reason) if self.estop_latch else ""))  # 锁存时附上原因
        self.log("  上位机 %s | $CMD %d / $PID %d / $VID %d / 未识别 %d | 模式切换 %d" % (  # 上位机各类帧统计
            self.pc_link.pc_ip(), s["cmd"], s["pid"], s["vid"], s["unknown"], s["switch"]))  # 上位机 IP 与计数
        self.log("  下位机 TX %d 帧 [0x04 %d / 0x09 %d / 0x0C %d / 其他 %d] | 遥测 %d 帧 | $TEL 上行 %d 帧" % (  # 下行与遥测统计
            s["tx_total"], s["tx_mode"], s["tx_motion"], s["tx_poll"], s["tx_other"],  # 下行帧各功能码计数
            s["tel"], s["tel_tx"]))  # 遥测接收与上行计数
        drops = getattr(m, "dropped", None)  # 模式上报的"无落点字段"统计
        if isinstance(drops, dict):  # 只有模式提供字典才打印
            self.log("  未下发字段（上位机已发 / V2 无落点）: led=%d grab=%d store=%d pid=%d" % (  # 打印丢弃计数
                drops.get("led", 0), drops.get("grab", 0), drops.get("store", 0),  # led/grab/store 计数
                drops.get("pid", 0)))  # pid 计数
        skipped = getattr(m, "idle_skipped", 0)  # 空闲静默跳过的帧数
        if skipped:  # 有跳过才打印
            self.log("  空闲静默：本周期未下发 0x09 %d 帧（杆位回中且载荷未变）" % skipped)  # 说明静默帧数
        if self.video is not None:  # 有图像服务
            try:  # summary 可能抛异常
                self.log("  图像回传 %s" % self.video.summary())  # 打印图像服务概况
            except Exception:  # 图像异常不影响报告
                pass  # 静默忽略
        t = self.last_tel  # 最近一帧遥测
        if not t:  # 还没有收到过遥测
            self.log("  (尚无下位机遥测：检查串口设备与下位机是否在线)")  # 提示排查串口与下位机
        else:  # 已有遥测
            self.log("  遥测: 姿态 roll=%.1f pitch=%.1f yaw=%.1f | 深度 %.1fcm(目标 %.1fcm)" % (  # 打印姿态与深度
                t.get("actual_roll", 0.0), t.get("actual_pitch", 0.0), t.get("actual_yaw", 0.0),  # 实测三轴姿态角
                t.get("actual_depth_cm", 0.0), t.get("target_depth_cm", 0.0)))  # 实测深度与目标深度

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def start(self):  # 启动链路与后台线程
        self.link_stm32.open()  # 打开下位机串口
        # [2026-10-04] 在链路轮询线程起来之前先设好"是否抑制兜底 0x0C"，
        #   否则 IDLE 启动的最初一瞬可能漏发一帧 0x0C。
        idle_id = int(getattr(self.cfg, "MODE_IDLE", -1))  # 待命态 id
        self.link_stm32.poll_suspended = (self.mode_id == idle_id)  # 待命态则抑制兜底轮询
        self.link_stm32.start()  # 启动串口读线程
        self.pc_link.start()  # 启动上位机 UDP 接收线程
        # 初始模式进入（initial=True 不会写记忆文件）。
        # [2026-10-04] 默认 mode_id = IDLE：IdleMode.on_enter 不发 0x04/0x0C，
        # 所以下位机在上电后保持原状，等上位机下发 $CMD.mode 才被切到 ROV/AUV。
        self.switch_mode(self.mode_id, initial=True)     # 让下位机与中位机初始模式一致
        self._stop.clear()  # 清除停止标志，允许再次 start
        for target in (self._bridge_loop, self._tick_loop):  # 依次启动两个后台线程
            th = threading.Thread(target=target, daemon=True)  # 守护线程，随主进程退出
            th.start()  # 启动线程
            self._threads.append(th)  # 记录以便 stop 时 join
        # 注意: 不在这里调 pc_link.is_alive() —— link_pc.PcLink 的 self._stop 覆盖了
        # Thread._stop，调用会在后期抛 TypeError。端口占用改由 main.preflight_ports() 启动前探测。
        time.sleep(0.1)  # 给后台线程一点启动时间

    def stop(self):  # 停止全部链路与线程
        self._stop.set()  # 通知后台循环退出
        for obj in (self.pc_link, self.link_stm32):  # 依次停上位机与下位机链路
            try:  # stop 可能抛异常
                obj.stop()  # 关闭链路
            except Exception:  # 忽略关闭异常
                pass  # 保持静默
        for th in list(self._threads) + [self.pc_link]:  # 等待线程与上位机链路结束
            try:  # join 可能超时或抛错
                th.join(timeout=1.0)  # 最多等 1s，避免退出卡死
            except Exception:  # 忽略 join 异常
                pass  # 保持静默
        self._unwrap_link()  # 还原被包装的 send