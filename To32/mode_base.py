# -*- coding: utf-8 -*-
"""模式基类 —— 中位机每个模式实现自己的运行逻辑

主循环（main.py）统一调用下列回调:
  on_enter(prev_id)    切换到本模式时调用一次
  on_exit(next_id)     离开本模式时调用一次
  on_cmd(cmd)          收到上位机 $CMD（20Hz）
  on_pid(pid)          收到 $PID
  on_vid(on)           收到 $VID
  on_downlink(tel)     收到下位机 V2 遥测（预留）
  tick(now, dt)        周期调用（TICK_HZ）
  build_telemetry()    返回要回给上位机的 $TEL 文本；None = 本周期不发

新增模式只需继承 ModeBase、填 id/name/desc，再在 main.py 里 register 一次。
"""


class ModeBase:
    id = -1
    name = "BASE"
    desc = ""

    def __init__(self, ctx):
        self.ctx = ctx
        self.log = ctx.log
        self.cfg = ctx.cfg
        self.link_stm32 = ctx.link_stm32
        self.active = False

    # ---------------- 生命周期 ----------------
    def on_enter(self, prev_id):
        self.active = True
        self.log("[MODE] → 进入 %s（%s）" % (self.name, self.desc))

    def on_exit(self, next_id):
        self.active = False
        self.log("[MODE] ← 退出 %s" % self.name)

    # ---------------- 事件 ----------------
    def on_cmd(self, cmd):
        """每帧 $CMD。默认什么都不做（自主模式可不理手动指令）"""

    def on_pid(self, pid):
        """$PID 调到本模式时的处理（预留）"""

    def on_vid(self, on):
        """视频开关（本程序暂不推流，留给 relay/vp 侧）"""

    def on_downlink(self, tel):
        """下位机遥测（预留：可用于闭环/状态显示）"""

    # ---------------- 周期 ----------------
    def tick(self, now, dt):
        """周期逻辑入口（预留）"""

    # ---------------- 输出 ----------------
    def build_telemetry(self):
        """返回 $TEL 文本；None 表示本周期不发"""
        return None

    # ---------------- 给子类的工具 ----------------
    def send_downlink(self, frame, note=""):
        """把一帧 V2 交给下位机链路（骨架阶段 sim 仅打印）"""
        if frame:
            self.link_stm32.send(frame, note or self.name)

    def pump_telemetry(self, now, note=""):
        """按 `config.STM32_POLL_HZ` 的节拍向下位机发 0x0C 遥测请求（即"请求回传"）。

        各模式在自己的 tick() 里调用：本模式活跃期间由本模式掌握回传节拍。
        链路层只在"最近没有模式请求过"时才兜底补发，故本方法不会与链路层重复请求
        （协议为 1 请求回 1 帧，重复会翻倍遥测率）。
        返回本次是否真的发出了请求。
        """
        hz = float(getattr(self.cfg, "STM32_POLL_HZ", 0.0) or 0.0)
        if hz <= 0:
            return False
        if now - getattr(self, "_last_poll_ts", 0.0) < 1.0 / hz:
            return False
        self._last_poll_ts = now
        self.link_stm32.send_telemetry_request(note or ("0x0C 遥测请求(%s)" % self.name))
        return True