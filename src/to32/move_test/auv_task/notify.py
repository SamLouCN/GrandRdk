# -*- coding: utf-8 -*-
"""状态提示回传 —— 把"正在跑什么 / 警告 / 报错"推给上位机终端显示

★ 需求（2026-10-05 用户拍板）：
  水池测试时人在岸上，板子上只有日志。**上位机终端要能直接看到**：
    当前正在测哪个任务阶段、有没有降级、为什么超时、为什么中止。
  所以本模块开一条**纯文本**通道，专门送"给人看的一句话"。

★ 为什么另开一条而不是塞进 $TEL / $AUV：
  $TEL 是 41 字段定长、上位机硬编码解析，加字段会破坏老上位机；
  $AUV 是 16 字段数值快照，装不下中文句子。
  这里发 **$MSG 帧**（帧头不同，上位机按帧头分发即可，老解析完全不受影响）。

帧格式（6 字段，与 $TEL/$AUV 同风格：逗号分隔 + `#` 结尾 + CRLF）：
    $MSG,<ts>,<level>,<code>,<stage>,<text>#\r\n
      ts    发送时刻 HH:MM:SS（终端直接可读）
      level INFO / WARN / ERROR（上位机据此染色：白 / 黄 / 红）
      code  机器码（STAGE/ABORT/TIMEOUT/DEGRADE/TEST/KILLED/BUDGET/PC_TAKEBACK ...）
      stage 当前阶段名（没有就 '-'）
      text  中文提示（内部逗号会转成 `;`，`#` 删除，绝不破坏帧结构）

★ 与 AuvReport 同样的四层隔离（**回传绝不能影响主功能**）：
  1. 独立线程 + 独立 socket，push() 只做 O(1) 入队，队满丢旧保新；
  2. socket 非阻塞，sendto 绝不卡住；
  3. 所有异常消化在线程里，不冒泡到任务；
  4. 连续失败 AUV_MSG_MAX_FAIL 次 → 永久放弃（无缆验收时零开销、零影响）。

★ 发送策略（防止 20Hz 刷屏）：
  ERROR / WARN → **立即发**（出事要第一时间看到）
  INFO         → 按 AUV_MSG_HZ 节流（默认 2Hz）
  相同 (level,code,text) 在 AUV_MSG_DEDUP_S 秒内不重复发
"""
import queue          # 线程安全队列：控制线程 -> 发送线程
import socket         # UDP
import threading      # 发送线程
import time           # 时间戳与节流


# ---------------------------------------------------------------- 级别
INFO = 'INFO'
WARN = 'WARN'
ERROR = 'ERROR'
LEVELS = (INFO, WARN, ERROR)
_LEVEL_RANK = {INFO: 0, WARN: 1, ERROR: 2}

# 机器码（上位机可按 code 做分类统计/过滤）
CODE_STAGE = 'STAGE'              # 阶段切换
CODE_ABORT = 'ABORT'              # 中止上浮
CODE_TIMEOUT = 'TIMEOUT'          # 阶段超时
CODE_SKIP = 'SKIP'                # 这一段没成，跳过继续（如没找到门 → 找下一个门）
CODE_DEGRADE = 'DEGRADE'          # 降级（无深度源 / 舵机未实装 / viskf 不可用 ...）
CODE_BUDGET = 'BUDGET'            # 全局预算用尽
CODE_TEST = 'TEST'                # 测试模式相关
CODE_KILLED = 'KILLED'            # 被上位机切走 / 被监督指令杀掉
CODE_PC = 'PC_TAKEBACK'           # 上位机接管（切回 ROV 等）
CODE_MODE = 'MODE'                # 模式进出
CODE_SERVO = 'SERVO'              # 舵机
CODE_VISION = 'VISION'            # 视觉（YOLO 有没有框）
CODE_HEARTBEAT = 'HB'             # 心跳（周期播报当前阶段）


def _clean(text, limit=160):
    """把提示文本洗成"能安全塞进逗号分隔帧"的形式"""
    s = '' if text is None else str(text)
    s = s.replace('\r', ' ').replace('\n', ' ')   # 换行会破坏行协议
    s = s.replace(',', ';').replace('#', '')      # 逗号/# 会破坏帧结构
    s = ' '.join(s.split())                       # 合并连续空白
    if len(s) > limit:                            # 超长截断，UDP 也不该发小说
        s = s[:limit - 3] + '...'
    return s


def build_msg_frame(ts, level, code, stage, text):
    """组帧（纯函数，离线可测）：返回 '$MSG,ts,level,code,stage,text#\\r\\n'

    ⚠ 字段数恒为 6：text 为空也必须占位，否则上位机按索引取会错位。
    """
    lv = str(level or INFO).upper()
    if lv not in LEVELS:
        lv = INFO
    parts = (str(ts or '-'), lv, str(code or '-'), str(stage or '-'), _clean(text))
    return '$MSG,' + ','.join(parts) + '#\r\n'


class NoMsg(object):
    """空实现（回传关闭 / 构造失败时用）—— 调用方不用到处判空"""

    enabled = False
    dead = True
    last_stage = '-'      # 与 AuvMsg 同名属性，外部可无差别读写

    def start(self):
        return False

    def stop(self):
        return None

    def push(self, level, code, text, stage=None):
        return False

    def info(self, code, text, stage=None):
        return False

    def warn(self, code, text, stage=None):
        return False

    def error(self, code, text, stage=None):
        return False

    def alive(self):
        return False

    def summary(self):
        return '关'


class AuvMsg(object):
    """状态提示上报器：独立线程按策略把提示 UDP 推给上位机"""

    def __init__(self, cfg, log, sock_factory=None):
        self.cfg = cfg
        self.log = log or (lambda m: None)
        # 默认复用 $AUV 的通道（同 IP/端口），帧头 $MSG 与 $AUV 互不干扰
        self.ip = str(getattr(cfg, 'AUV_MSG_IP', None)
                      or getattr(cfg, 'AUV_REPORT_IP', '192.168.127.100'))
        self.port = int(getattr(cfg, 'AUV_MSG_PORT', 0)
                        or getattr(cfg, 'AUV_REPORT_PORT', 8085))
        self.hz = float(getattr(cfg, 'AUV_MSG_HZ', 2.0) or 0.0)
        self.max_fail = int(getattr(cfg, 'AUV_MSG_MAX_FAIL', 3))
        self.dedup_s = float(getattr(cfg, 'AUV_MSG_DEDUP_S', 2.0))
        self.min_level = str(getattr(cfg, 'AUV_MSG_MIN_LEVEL', INFO) or INFO).upper()
        if self.min_level not in LEVELS:
            self.min_level = INFO
        self.enabled = bool(getattr(cfg, 'AUV_MSG_ENABLED', True))
        self.heartbeat_s = float(getattr(cfg, 'AUV_MSG_HEARTBEAT_S', 0.0) or 0.0)  # 0=关
        self._sock_factory = sock_factory

        self.q = queue.Queue(maxsize=64)
        self.sock = None
        self.thread = None
        self._stop = threading.Event()
        self.ok = 0
        self.fail = 0
        self.dropped = 0
        self.dead = False
        self._run_fail = 0
        self._last_sent = {}      # (level,code,text) -> 上次发送时刻（去重）
        self._next_info_ts = 0.0  # INFO 节流：下一次允许发送的时刻
        self._last_hb_ts = 0.0    # 心跳时间戳
        self.last_stage = '-'     # 最近一次播报的阶段（心跳用）

    # ---------------------------------------------------------------- 生命周期
    def start(self):
        """起线程。失败只降级（不上报），绝不影响主功能"""
        if not self.enabled:
            self.log('[AUV-MSG] 状态提示回传已关闭（AUV_MSG_ENABLED=False）')
            return False
        try:
            self.sock = (self._sock_factory() if self._sock_factory is not None
                         else socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.sock.setblocking(False)          # ★ 非阻塞：sendto 绝不卡住
        except OSError as e:
            self.dead = True
            self.log('[AUV-MSG] socket 创建失败 -> 放弃提示回传（%s）' % e)
            return False
        self._stop.clear()
        self.dead = False
        self._run_fail = 0
        self.thread = threading.Thread(target=self._loop, name='AuvMsg', daemon=True)
        self.thread.start()
        self.log('[AUV-MSG] 状态提示回传已启动 -> %s:%d @ %.1fHz（$MSG 帧）'
                 % (self.ip, self.port, self.hz))
        return True

    def stop(self):
        """停线程、关 socket（join 带超时，绝不卡住模式切换）"""
        self._stop.set()
        t = self.thread
        if t is not None:
            t.join(timeout=1.0)
        self.thread = None
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.log('[AUV-MSG] 已停止（成功 %d / 失败 %d / 丢弃 %d）'
                 % (self.ok, self.fail, self.dropped))

    # ---------------------------------------------------------------- 入口
    def push(self, level, code, text, stage=None):
        """入队一条提示（O(1)、非阻塞、永不抛异常）

        level: INFO/WARN/ERROR；code: 机器码；text: 中文提示；stage: 阶段名
        """
        if self.dead or not self.enabled or self.thread is None:
            return False
        lv = str(level or INFO).upper()
        if lv not in LEVELS:
            lv = INFO
        if _LEVEL_RANK[lv] < _LEVEL_RANK[self.min_level]:   # 级别不够，直接丢
            return False
        if stage:
            self.last_stage = str(stage)
        item = (time.time(), lv, str(code or '-'), str(stage or '-'), _clean(text))
        try:
            self.q.put_nowait(item)
            return True
        except queue.Full:                                   # 队满：丢旧保新
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put_nowait(item)
            except queue.Full:
                self.dropped += 1
                return False
            self.dropped += 1
            return True

    def info(self, code, text, stage=None):
        return self.push(INFO, code, text, stage)

    def warn(self, code, text, stage=None):
        return self.push(WARN, code, text, stage)

    def error(self, code, text, stage=None):
        return self.push(ERROR, code, text, stage)

    # ---------------------------------------------------------------- 线程
    def _loop(self):
        """发送线程：取出 → 判重/节流 → 组帧 → 非阻塞发送

        ⚠ 退出条件是"收到停止信号 **且** 队列已空"，不是"收到停止信号就退出"：
          mode_auv.on_exit 的顺序是 kill() → notify.stop()，中间只隔几毫秒，
          如果一收到 stop 就退出，最后那几条（KILLED / PC_TAKEBACK）根本来不及发，
          上位机终端就看不到"为什么停了" —— 恰恰是最该看到的那一条。
        """
        while True:
            try:
                item = self.q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():          # 队列空了才允许退出
                    return
                self._heartbeat()
                continue
            except Exception:
                if self._stop.is_set():
                    return
                continue
            ts, lv, code, stage, text = item
            now = time.time()
            key = (lv, code, text)
            last = self._last_sent.get(key, 0.0)
            if now - last < self.dedup_s:               # 去重：完全相同的提示别刷屏
                continue
            if lv == INFO and now < self._next_info_ts:  # INFO 按节拍节流
                continue
            self._send(ts, lv, code, stage, text)
            self._last_sent[key] = now
            if lv == INFO:
                self._next_info_ts = now + (1.0 / self.hz if self.hz > 0 else 0.5)

    def _heartbeat(self):
        """心跳：长时间没有新提示时，周期播报一次"还在跑 + 当前阶段" """
        if self.heartbeat_s <= 0 or not self.last_stage or self.last_stage == '-':
            return
        now = time.time()
        if now - self._last_hb_ts < self.heartbeat_s:
            return
        self._last_hb_ts = now
        self._send(now, INFO, CODE_HEARTBEAT, self.last_stage,
                   '运行中，当前阶段 %s' % self.last_stage)

    def _send(self, ts, lv, code, stage, text):
        """真正发一帧（异常全消化在这里）"""
        frame = build_msg_frame(time.strftime('%H:%M:%S', time.localtime(ts)),
                                lv, code, stage, text)
        try:
            self.sock.sendto(frame.encode('utf-8'), (self.ip, self.port))
            self.ok += 1
            self._run_fail = 0
        except Exception as e:
            self.fail += 1
            self._run_fail += 1
            if self._run_fail == 1:
                self.log('[AUV-MSG] 发送失败: %s' % e)
            if self._run_fail >= self.max_fail:      # ★ 连续失败到上限 → 永久放弃
                self.dead = True
                self.log('[AUV-MSG] 连续 %d 次发送失败 -> 放弃提示回传（任务不受影响）'
                         % self._run_fail)
                return

    # ---------------------------------------------------------------- 状态
    def alive(self):
        return bool(self.enabled and not self.dead and self.thread is not None)

    def summary(self):
        if not self.enabled:
            return '关'
        if self.dead:
            return '已放弃(失败%d次)' % self.fail
        return '%s:%d 成功%d/失败%d' % (self.ip, self.port, self.ok, self.fail)


def apply_notify(n, log=None):
    """把 test_config.NOTIFY 的覆盖套到提示通道上（板端与离线**共用同一份配置**）

    返回被覆盖的键名列表（启动日志里打一行，一眼确认生效没有）。
    """
    try:
        from . import test_config as TC          # 延迟 import：test_config 不依赖本模块，无循环
        ov = dict(getattr(TC, 'NOTIFY', {}) or {})
    except Exception:
        ov = {}
    hit = []
    for k, v in ov.items():
        try:
            setattr(n, k, v)
            hit.append(str(k))
        except Exception:
            pass
    if hit and log is not None:
        log('[AUV-MSG] test_config.NOTIFY 覆盖 %d 项: %s' % (len(hit), ','.join(hit)))
    return hit


def make_notifier(cfg, log=None, sock_factory=None, enabled=None):
    """工厂：按配置返回 AuvMsg 或 NoMsg（调用方不用判空）

    enabled: 显式覆盖开关（None = 看配置 AUV_MSG_ENABLED）；
             测试代码传 False 可完全静默。
    """
    log = log or (lambda m: None)
    if enabled is False:
        return NoMsg()
    try:
        n = AuvMsg(cfg, log, sock_factory=sock_factory)
    except Exception as e:                       # 构造都失败 → 静默，绝不影响任务
        log('[AUV-MSG] 构造失败 -> 关闭提示回传（%s）' % e)
        return NoMsg()
    if enabled is True:
        n.enabled = True
    if not n.enabled:
        return NoMsg()
    return n


# ---------------------------------------------------------------- 阶段中文简述
# 上位机终端显示用：只写"这段在干嘛"，不写参数（参数看 $AUV 帧）
STAGE_DESC = {
    'DIVE': '下潜定深（撞球高度）',
    'SEEK_BALL_F': '前视找球',
    'RAM_BALL': '对准直冲撞球',
    'TURN_120': '转向找门',
    'SEEK_GATE_1': '前视找门 1',
    'PASS_GATE_1': '穿门 1',
    'SEEK_GATE_2': '前视找门 2',
    'PASS_GATE_2': '穿门 2',
    'SEEK_GATE_3': '前视找门 3',
    'PASS_GATE_3': '穿门 3',
    'SEEK_GATE_4': '前视找门 4',
    'PASS_GATE_4': '穿门 4',
    'SEEK_BALL_B': '下视找待捡球',
    'CENTER_BALL': '把球对到收集框正上方',
    'SIT_BOTTOM': '坐底（收集框兜球）',
    'BOTTOM_HOLD': '坐底保持等球入框',
    'RELEASE_ASCEND': '上升到投放高度',
    'RELEASE_TURN': '转向投放方向',
    'RELEASE_MOVE': '推进到投放点',
    'RELEASE_HOVER': '悬停吃掉余速',
    'RELEASE_DROP': '舵机投放',
    'GO_HOME': '回出发区顶池壁',
    'SURFACE': '上浮',
    'DONE': '结束停推',
}


def describe(stage):
    """阶段名 → 中文简述（未知阶段返回原名）"""
    return STAGE_DESC.get(str(stage or '').upper(), str(stage or '-'))
