# -*- coding: utf-8 -*-
"""中位机编排核心 —— 模式注册/切换 + 安全 + 周期调度 + 遥测上行。

三机分层（命名见《上位机通讯协议.md》）:
    上位机(PC UI)  ──UDP 8080/8081──▶  link_pc.py      只收发文本帧(自带 PING->PONG / $TEL 出口)
    模式逻辑       ──mode_rov / mode_auv──▶  各模式的运行逻辑写在这里
    下位机(STM32)  ──V2 串口帧──────▶  link_stm32.py   只收发 V2 帧
    本文件         把三者接起来: 事件派发 / $CMD.mode 切模式 / 安全三件套 / tick / $TEL

本文件不含任何"模式自己的逻辑"; 模式的运行逻辑请写在 mode_rov.py / mode_auv.py。
新增一个模式三步:
    1) 新建 mode_xxx.py 继承 mode_base.ModeBase, 实现 on_enter/on_exit/on_cmd/tick/on_downlink
    2) 在 config.py 登记 MODE_XXX（$CMD 的 mode 字段取值）
    3) 在本文件 _register_modes() 里加入
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time

import link_stm32 as S
import tel_builder as TB
from mode_auv import AuvMode
from mode_rov import RovMode


class Dispatcher(object):
    """编排核心；同时充当各模式的 ctx（模式只使用 ctx.log / ctx.cfg / ctx.link_stm32 / ctx.estop_latch）。"""

    def __init__(self, cfg, pc_link, link_stm32, log, rx_queue=None, video=None):
        self.cfg = cfg
        self.pc_link = pc_link
        self.link_stm32 = link_stm32
        self.video = video          # 图像回传服务（可选；None = 本进程不做图像）
        self.rx_queue = rx_queue if rx_queue is not None else queue.Queue()
        self._log_fn = log

        self.modes = {}
        self._register_modes()
        # 日志状态必须先就绪：_resolve_initial_mode() 会写日志（记忆命中/记忆文件损坏）
        self._log_last = ("", 0.0)
        self._log_dup = 0
        # 启动模式：命令行 --mode > 模式记忆文件 > config.START_MODE / DEFAULT_MODE
        self.mode_state_path = str(getattr(cfg, "MODE_STATE_PATH", "") or "")
        self.mode_persist = bool(getattr(cfg, "MODE_PERSIST", False))
        self.mode_id = self._resolve_initial_mode()

        # 运行状态
        self.last_cmd_ts = 0.0
        self.last_tel = None
        self.anchored = False
        self.estop_latch = False
        self.estop_reason = ""

        self.stats = {
            "cmd": 0, "pid": 0, "vid": 0, "unknown": 0, "bad_mode": 0,
            "tel": 0, "tel_tx": 0, "switch": 0,
            "tx_total": 0, "tx_mode": 0, "tx_motion": 0, "tx_poll": 0, "tx_other": 0,
        }

        self._stop = threading.Event()
        self._threads = []
        self._log_last = ("", 0.0)
        self._log_dup = 0
        self._wrap_link()

    # ------------------------------------------------------------------ #
    # 日志 / 统计
    # ------------------------------------------------------------------ #
    def log(self, msg, min_gap=1.0):
        """写日志；同一行 min_gap 秒内重复只计数，避免 ROV 抑制 0x09 时 20Hz 刷屏。"""
        now = time.time()
        last_msg, last_t = self._log_last
        if msg == last_msg and (now - last_t) < min_gap:
            self._log_dup += 1
            self._log_last = (msg, now)
            return
        self._flush_dup()
        self._log_fn(msg)
        self._log_last = (msg, now)

    def _flush_dup(self):
        if self._log_dup:
            n, self._log_dup = self._log_dup, 0
            self._log_fn("      ... 上一行重复 %d 次" % n)

    def _wrap_link(self):
        """统计下行帧数：在实例上包一层 send()，不改 link_stm32.py 本体。"""
        if getattr(self.link_stm32, "_rdk_counted", False):
            return
        self._orig_send = self.link_stm32.send

        def _send(frame, note=""):
            self._count_tx(frame)
            return self._orig_send(frame, note)

        self.link_stm32.send = _send
        self.link_stm32._rdk_counted = True

    def _unwrap_link(self):
        self._flush_dup()
        if getattr(self.link_stm32, "_rdk_counted", False):
            for attr in ("send", "_rdk_counted"):
                try:
                    delattr(self.link_stm32, attr)
                except Exception:
                    pass

    def _count_tx(self, frame):
        if not frame:
            return
        self.stats["tx_total"] += 1
        func = frame[2] if len(frame) > 2 else -1
        if func == S.FUNC_MODE:
            self.stats["tx_mode"] += 1
        elif func == S.FUNC_MOTION:
            self.stats["tx_motion"] += 1
        elif func == S.FUNC_TELEMETRY:
            self.stats["tx_poll"] += 1
        else:
            self.stats["tx_other"] += 1

    # ------------------------------------------------------------------ #
    # 模式
    # ------------------------------------------------------------------ #
    def _register_modes(self):
        for m in (RovMode(self), AuvMode(self)):
            self.modes[m.id] = m

    # ------------------------------------------------------------------ #
    # 启动模式：可配置 + 模式记忆（下次启动沿用上位机最后切换的模式）
    # ------------------------------------------------------------------ #
    def _config_start_mode(self):
        """config 里的启动模式（命令行 --mode 会以 MODE_FORCE_START 形式覆盖到这里）"""
        v = getattr(self.cfg, "START_MODE", None)
        if v is None:
            v = getattr(self.cfg, "DEFAULT_MODE", getattr(self.cfg, "MODE_ROV", 0))
        try:
            return int(v)
        except (TypeError, ValueError):
            self.log("[MODE] START_MODE=%r 非法 -> 回退 ROV" % (v,))
            return int(getattr(self.cfg, "MODE_ROV", 0))

    def load_saved_mode(self):
        """读模式记忆文件；未开启/文件不存在/损坏/模式未注册 -> None"""
        if not (self.mode_persist and self.mode_state_path):
            return None
        try:
            with open(self.mode_state_path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            mid = int(obj.get("mode"))
        except FileNotFoundError:
            return None
        except Exception as e:
            self.log("[MODE] 记忆文件读取失败(%s): %s -> 忽略" % (self.mode_state_path, e))
            return None
        if mid not in self.modes:
            self.log("[MODE] 记忆文件里的模式 %r 未注册 -> 忽略" % (mid,))
            return None
        return mid

    def save_mode(self, mode_id):
        """把模式写入记忆文件（原子写：临时文件 + os.replace）"""
        if not (self.mode_persist and self.mode_state_path):
            return False
        try:
            d = os.path.dirname(os.path.abspath(self.mode_state_path))
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            tmp = self.mode_state_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"mode": int(mode_id),
                           "name": self.modes[int(mode_id)].name,
                           "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "source": "pc"}, f, ensure_ascii=False)
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.mode_state_path)
            return True
        except Exception as e:
            self.log("[MODE] 记忆文件写入失败(%s): %s" % (self.mode_state_path, e))
            return False

    def _resolve_initial_mode(self):
        """启动模式解析：--mode(MODE_FORCE_START) > 记忆文件 > config.START_MODE"""
        if bool(getattr(self.cfg, "MODE_FORCE_START", False)):
            self.mode_source = "forced"
            return self._config_start_mode()
        saved = self.load_saved_mode()
        if saved is not None:
            self.mode_source = "saved"
            self.log("[MODE] 沿用记忆模式启动: %s（来自 %s）"
                     % (self.modes[saved].name, self.mode_state_path))
            return saved
        self.mode_source = "start"
        return self._config_start_mode()

    def mode(self):
        return self.modes[self.mode_id]

    def switch_mode(self, new_id, initial=False):
        """切换模式：调用旧模式 on_exit、新模式 on_enter（新模式在 on_enter 里下发 0x04 模式码）。"""
        try:
            new_id = int(new_id)
        except (TypeError, ValueError):
            self.stats["bad_mode"] += 1
            return False
        if new_id not in self.modes:
            self.stats["bad_mode"] += 1
            self.log("[MODE] 未知模式 %r（已知 %s）-> 忽略" % (new_id, sorted(self.modes)))
            return False
        if not initial and new_id == self.mode_id:
            return False
        old_id = self.mode_id
        if not initial:
            self.modes[old_id].on_exit(new_id)
        self.mode_id = new_id
        self.stats["switch"] += 1
        self.modes[new_id].on_enter(old_id)
        self.log("[MODE] 当前模式 -> %s" % self.modes[new_id].name)
        if not initial:
            # 上位机切换的模式写入记忆文件：下次启动沿用
            # （可用 --mode 强制覆盖 / --no-mode-persist 关闭记忆）
            if self.save_mode(new_id):
                self.log("[MODE] 已记忆模式 %s -> %s"
                         % (self.modes[new_id].name, self.mode_state_path))
        return True

    # ------------------------------------------------------------------ #
    # 上位机事件（link_pc 线程填队列 -> bridge 线程取 -> 这里）
    # ------------------------------------------------------------------ #
    def on_pc_event(self, kind, payload):
        if kind == "cmd":
            self._on_cmd(payload)
        elif kind == "pid":
            self.stats["pid"] += 1
            self.mode().on_pid(payload)
        elif kind == "vid":
            self.stats["vid"] += 1
            on = bool(payload)
            # 图像回传是"全局开关"，与 ROV/AUV 模式无关：先切服务，再通知模式
            if self.video is not None:
                try:
                    self.video.set_enabled(on)
                except Exception as e:
                    self.log("[ERR] 图像回传开关异常: %s" % e)
            self.mode().on_vid(on)
        elif kind == "raw":
            self._on_raw(payload)
        else:
            self.stats["unknown"] += 1

    def _on_cmd(self, cmd):
        self.stats["cmd"] += 1
        self.last_cmd_ts = time.time()
        if self.estop_latch and not bool(getattr(self.cfg, "ESTOP_LATCH", True)):
            self.estop_latch = False
            self.log("[ESTOP] 收到新 $CMD -> 解除急停锁存")
        want = int(cmd.get("mode", getattr(self.cfg, "MODE_ROV", 0)))
        if want != self.mode_id:
            self.switch_mode(want)
        self.mode().on_cmd(cmd)

    def _on_raw(self, text):
        """未识别的文本帧：只认扩展的 $ESTOP# / $ESTOP,0#（v3.2 UI 不发这两个帧，留作人工兜底）。"""
        s = (text or "").strip()
        if s == "$ESTOP#":
            self.trigger_estop("上位机 $ESTOP")
        elif s.startswith("$ESTOP,") and s.endswith("#"):
            try:
                if int(float(s[7:-1])) == 0:
                    self.release_estop("上位机 $ESTOP,0")
            except ValueError:
                pass
        else:
            self.stats["unknown"] += 1

    # ------------------------------------------------------------------ #
    # 下位机遥测（link_stm32 读线程 -> 这里）
    # ------------------------------------------------------------------ #
    def on_stm32_telemetry(self, tel):
        if not tel:
            return
        self.stats["tel"] += 1
        self.last_tel = tel
        if not self.anchored:
            self._anchor(tel)
        try:
            self.mode().on_downlink(tel)
        except Exception as e:
            self.log("[ERR] %s.on_downlink 异常: %s" % (self.mode().name, e))

    def _anchor(self, tel):
        """冷启动锚定：模式内部的目标量取首帧实测值，避免一上电/切模式就"回水面、转向"。"""
        m = self.mode()
        done = []
        for attr, key in (("target_depth_cm", "actual_depth_cm"),
                          ("target_yaw_deg", "actual_yaw")):
            if hasattr(m, attr):
                setattr(m, attr, float(tel.get(key, 0.0)))
                done.append(attr)
        self.anchored = True
        self.log("[ANCHOR] 首帧遥测 -> %s 锚定到实测值%s" % (
            m.name, ("（%s）" % ",".join(done)) if done else "（本模式无内部目标量）"))

    # ------------------------------------------------------------------ #
    # 安全
    # ------------------------------------------------------------------ #
    def trigger_estop(self, reason):
        """急停：下发 0x04 0x01 并锁存。锁存后模式侧会抑制 0x09（0x09 无 STANDBY 守卫）。"""
        self.link_stm32.send(S.frame_mode(S.MODE_ESTOP), "0x04 急停")
        self.estop_latch = True
        self.estop_reason = reason
        self.log("[ESTOP] %s -> 已下发 0x04 0x01，锁存并抑制 0x09" % reason)

    def release_estop(self, reason="手动解除"):
        if not self.estop_latch:
            return
        self.estop_latch = False
        self.estop_reason = ""
        self.last_cmd_ts = time.time()          # deadman 重新起算
        self.log("[ESTOP] %s -> 解除锁存" % reason)

    def _check_deadman(self, now):
        """下行静默看门狗：上位机(或链路)静默超过 ESTOP_TIMEOUT_S 自动急停。0=关闭。"""
        timeout = float(getattr(self.cfg, "ESTOP_TIMEOUT_S", 0.0) or 0.0)
        if timeout <= 0 or self.estop_latch or self.last_cmd_ts <= 0:
            return
        silent = now - self.last_cmd_ts
        if silent > timeout:
            self.trigger_estop("下行静默 %.1fs" % silent)

    # ------------------------------------------------------------------ #
    # 周期
    # ------------------------------------------------------------------ #
    def _send_tel(self):
        """$TEL 上行：优先用当前模式自己的组帧，模式没提供则回落到统一映射（无遥测则不发）。"""
        mode = self.mode()
        txt = None
        try:
            txt = mode.build_telemetry()
        except Exception as e:
            self.log("[ERR] %s.build_telemetry 异常: %s" % (mode.name, e))
        if not txt:
            txt = TB.build_tel(self.last_tel)
        if txt and self.pc_link.send_telem(txt):
            self.stats["tel_tx"] += 1

    def _bridge_loop(self):
        while not self._stop.is_set():
            try:
                item = self.rx_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            except Exception:
                break
            try:
                kind, payload = item
                self.on_pc_event(kind, payload)
            except Exception as e:
                self.log("[ERR] 上位机事件处理异常: %s" % e)

    def _tick_loop(self):
        tick_hz = max(1.0, float(getattr(self.cfg, "TICK_HZ", getattr(self.cfg, "CMD_HZ", 20.0))))
        tel_hz = max(0.0, float(getattr(self.cfg, "TEL_HZ", 10.0)))
        report_s = float(getattr(self.cfg, "REPORT_S", 5.0) or 0.0)
        step = 1.0 / tick_hz
        acc_tel = acc_rep = 0.0
        last = time.time()
        while not self._stop.is_set():
            now = time.time()
            dt = min(0.25, max(0.0, now - last))
            last = now
            self._check_deadman(now)
            acc_tel += dt
            acc_rep += dt
            # 急停锁存期间不跑模式 tick：模式输出已被抑制，只保留看门狗与遥测上行
            if not self.estop_latch:
                try:
                    self.mode().tick(now, dt)
                except Exception as e:
                    self.log("[ERR] %s.tick 异常: %s" % (self.mode().name, e))
            if tel_hz > 0 and acc_tel >= 1.0 / tel_hz:
                acc_tel = 0.0
                self._send_tel()
            if report_s > 0 and acc_rep >= report_s:
                acc_rep = 0.0
                self.report()
            time.sleep(step)
        self._flush_dup()

    # ------------------------------------------------------------------ #
    # 上报
    # ------------------------------------------------------------------ #
    def report(self):
        s = self.stats
        m = self.mode()
        self.log("---- 中位机 ----")
        self.log("  模式 %s | 锚定 %s | 急停 %s%s" % (
            m.name, "已锚定" if self.anchored else "未锚定",
            "锁存" if self.estop_latch else "正常",
            (" (%s)" % self.estop_reason) if self.estop_latch else ""))
        self.log("  上位机 $CMD %d / $PID %d / $VID %d / 未识别 %d | 模式切换 %d" % (
            s["cmd"], s["pid"], s["vid"], s["unknown"], s["switch"]))
        self.log("  下位机 TX %d 帧 [0x04 %d / 0x09 %d / 0x0C %d / 其他 %d] | 遥测 %d 帧 | $TEL 上行 %d 帧" % (
            s["tx_total"], s["tx_mode"], s["tx_motion"], s["tx_poll"], s["tx_other"],
            s["tel"], s["tel_tx"]))
        if self.video is not None:
            try:
                self.log("  图像回传 %s" % self.video.summary())
            except Exception:
                pass
        t = self.last_tel
        if not t:
            self.log("  (尚无下位机遥测：检查串口设备与下位机是否在线)")
        else:
            self.log("  遥测: 姿态 roll=%.1f pitch=%.1f yaw=%.1f | 深度 %.1fcm(目标 %.1fcm)" % (
                t.get("actual_roll", 0.0), t.get("actual_pitch", 0.0), t.get("actual_yaw", 0.0),
                t.get("actual_depth_cm", 0.0), t.get("target_depth_cm", 0.0)))

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def start(self):
        self.link_stm32.open()
        self.link_stm32.start()
        self.pc_link.start()
        self.switch_mode(self.mode_id, initial=True)     # 让下位机与中位机初始模式一致
        self._stop.clear()
        for target in (self._bridge_loop, self._tick_loop):
            th = threading.Thread(target=target, daemon=True)
            th.start()
            self._threads.append(th)
        # 注意: 不在这里调 pc_link.is_alive() —— link_pc.PcLink 的 self._stop 覆盖了
        # Thread._stop，调用会在后期抛 TypeError。端口占用改由 main.preflight_ports() 启动前探测。
        time.sleep(0.1)

    def stop(self):
        self._stop.set()
        for obj in (self.pc_link, self.link_stm32):
            try:
                obj.stop()
            except Exception:
                pass
        for th in list(self._threads) + [self.pc_link]:
            try:
                th.join(timeout=1.0)
            except Exception:
                pass
        self._unwrap_link()