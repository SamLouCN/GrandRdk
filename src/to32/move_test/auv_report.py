# -*- coding: utf-8 -*-
"""AUV 状态上报器 —— 把自主运行状态通过网线（UDP）推给上位机

★ 第一原则：**回传绝不能影响主功能**（你要实地测自动运行效果，观测通道不能拖累控制）。
  为此做了四层隔离：
    1. 独立线程 + 独立 socket；tick() 里只调一次 push()，它只做 O(1) 的队列操作，
       **队满就丢旧保新，绝不等待、绝不阻塞状态机**；
    2. socket 设成非阻塞（setblocking(False)），sendto 不会卡住线程；
    3. 所有网络操作包 try/except，任何异常都消化在线程里，绝不冒泡到 mission；
    4. 连续失败 AUV_REPORT_MAX_FAIL 次 → **永久放弃**（自杀式降级），
       不再重试、不再刷日志，状态机照跑，只是上位机看不到。

与 ROV 模式的对应关系（"一致"到底指什么）：
  - **图像**：ROV 用 web_server 的 http :5000 /cam1 /cam2（front/bottom 写共享内存、
    带 YOLO 检测框的 MJPEG）。这条路由 web_server 进程常驻提供，**与运行模式无关**，
    所以 AUV 下照样有画面 —— 本模块不重复开相机（相机被 front/bottom 独占，
    重开设备节点会打架）。上报帧里带 vis_* 字段，用来确认"画面这条链是不是活的"。
  - **数据**：ROV 用 $TEL 41 字段（:8081）。AUV 模式下 mode_auv 返回 None，
    mode_dispatcher 回落到 tel_builder，$TEL **同样不断流**。
    但 41 字段是上位机硬编码的，装不下 AUV 的阶段/目标/观测，
    所以本模块**额外**发一条 $AUV 帧（独立端口，不干扰 $TEL 解析）。

$AUW 帧格式（逗号分隔定长 16 字段，与 $TEL 风格一致，便于报文窗口直接人读）：
    $AUV,stage,t,depth_cmd,yaw_cmd,surge,sway,depth_now,depth_ok,
         vis_cam,vis_label,vis_dx,vis_dy,vis_w,vis_age,abort,seq#
"""
import queue  # 标准库：线程安全队列，tick 与上报线程之间只靠它传递快照
import socket  # 标准库：UDP 发送
import threading  # 标准库：上报线程
import time  # 标准库：节拍与时间戳


# 帧字段顺序（= 逗号分隔的索引顺序），改这里必须同步改 README 的字段表
FIELDS = (
    'stage',      # 0  当前阶段名（DIVE/SEEK_BALL_F/RAM_BALL/...）
    't',          # 1  进入本阶段后的秒数
    'depth_cmd',  # 2  本拍下发的**目标**深度 cm
    'yaw_cmd',    # 3  本拍下发的**目标**航向 deg（已含镜像前的值）
    'surge',      # 4  前向推力 [-1,1]
    'sway',       # 5  横向推力 [-1,1]
    'depth_now',  # 6  当前深度 cm（深度卡尔曼/遥测；-1 = 无数据源）
    'depth_ok',   # 7  深度可信标志 0/1
    'vis_cam',    # 8  最近一次观测来自 front / bottom / -
    'vis_label',  # 9  最近一次观测的类别（归一化后）
    'vis_dx',     # 10 目标中心相对瞄准点的横向像素偏差（右为正）
    'vis_dy',     # 11 纵向像素偏差（下为正）
    'vis_w',      # 12 目标框宽度 px（贴脸判据用）
    'vis_age',    # 13 距最近一次观测的秒数（越大说明越久没看到目标）
    'abort',      # 14 中止原因（空 = 正常）
    'seq',        # 15 帧序号，丢包/卡顿一眼看出
)


def _fmt(v, nd=2):
    """把数值格式化成定长小数字符串；非数值统一写成 '-'，保证字段数不塌"""
    if v is None:  # 缺失值
        return '-'
    if isinstance(v, bool):  # bool 是 int 的子类，必须先判，否则会输出 1/0 之外的东西
        return '1' if v else '0'
    try:  # 可能是字符串数字
        f = float(v)
    except (TypeError, ValueError):  # 转不了就当字符串处理
        s = str(v).replace(',', ';').replace('#', '')  # 逗号会破坏帧结构，# 是帧尾，都要清掉
        return s
    if f != f or f in (float('inf'), float('-inf')):  # NaN / inf 不能写进帧
        return '-'
    return ('%.*f' % (nd, f))


class AuvReport(object):
    """AUV 状态上报器：独立线程按固定节拍把最新快照 UDP 推给上位机"""

    def __init__(self, cfg, log, sock_factory=None):
        """cfg=配置对象；log=日志函数；sock_factory=测试注入点（台架用假 socket）"""
        self.cfg = cfg  # 配置对象（AUV_REPORT_* 全部从这取）
        self.log = log  # 日志回调（与模式共用同一个）
        self.ip = str(getattr(cfg, 'AUV_REPORT_IP', '192.168.127.100'))  # 上位机 IP（网线对端）
        self.port = int(getattr(cfg, 'AUV_REPORT_PORT', 8085))  # 上报端口（避开已占用的 8080/8081/8082/8084）
        self.hz = float(getattr(cfg, 'AUV_REPORT_HZ', 5.0) or 0.0)  # 上报节拍（远低于 20Hz 主循环）
        self.max_fail = int(getattr(cfg, 'AUV_REPORT_MAX_FAIL', 3))  # 连续失败几次后放弃
        self.enabled = bool(getattr(cfg, 'AUV_REPORT_ENABLED', True))  # 总开关
        self._sock_factory = sock_factory  # None = 用真实 socket
        self.q = queue.Queue(maxsize=1)  # 快照只留最新一份（丢旧保新）
        self.sock = None  # UDP socket 句柄
        self.thread = None  # 上报线程句柄
        self._stop = threading.Event()  # 停止信号
        self.ok = 0  # 累计成功帧数
        self.fail = 0  # 累计失败次数
        self.dropped = 0  # 因队满被丢弃的快照数（正常现象，20Hz 塞 5Hz 取必然丢）
        self.dead = False  # True = 已放弃上报，之后不再尝试
        self._run_fail = 0  # 当前连续失败计数（成功一次即清零）

    # ---------------------------------------------------------------- 生命周期
    def start(self):
        """起线程。任何失败都只降级不打崩：起不来 = 不上报，任务照跑"""
        if not self.enabled:  # 总开关关着
            self.log('[AUV-REP] 状态回传已关闭（AUV_REPORT_ENABLED=False），任务不受影响')
            return False
        try:  # 建 socket 可能失败（权限/资源）
            if self._sock_factory is not None:  # 测试注入
                self.sock = self._sock_factory()
            else:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4 + UDP
            self.sock.setblocking(False)  # ★ 非阻塞：sendto 绝不卡住本线程
        except OSError as e:  # 建不起来就放弃，绝不影响主功能
            self.dead = True
            self.log('[AUV-REP] socket 创建失败 -> 放弃回传（%s）' % e)
            return False
        self._stop.clear()  # 允许重启（切出再切进 AUV）
        self.thread = threading.Thread(target=self._loop, name='AuvReport', daemon=True)  # 守护线程
        self.thread.start()  # 起线程
        self.log('[AUV-REP] 状态回传已启动 -> %s:%d @ %.1fHz（$AUV 帧）'
                 % (self.ip, self.port, self.hz))
        return True

    def stop(self):
        """停线程、关 socket。join 带超时，绝不无限等待"""
        self._stop.set()  # 通知线程退出
        t = self.thread  # 取句柄
        if t is not None:  # 线程起过才 join
            t.join(timeout=1.0)  # 最多等 1s，卡住也放主线程走
        self.thread = None  # 清句柄
        if self.sock is not None:  # 关 socket
            try:
                self.sock.close()  # 关闭，唤醒可能阻塞的调用
            except OSError:
                pass  # 关闭失败无所谓
            self.sock = None  # 清句柄
        self.log('[AUV-REP] 已停止（成功 %d 帧 / 失败 %d 次 / 丢弃 %d）'
                 % (self.ok, self.fail, self.dropped))

    # ---------------------------------------------------------------- 数据入口
    def push(self, snap):
        """★ tick() 唯一调用点：O(1)、非阻塞、永不抛异常

        队里只保留**最新**一份快照：先把旧的取走（若有），再放新的。
        20Hz 主循环往 5Hz 上报线程塞，绝大部分快照会被丢掉 —— 这是设计使然，
        不是 bug：上报要的是"最新状态"，不是"每一拍都不漏"。
        """
        if self.dead or not self.enabled or self.thread is None:  # 已放弃/未启动
            return False
        try:
            self.q.get_nowait()  # 取走旧快照（没有就抛 Empty，正常）
        except queue.Empty:
            pass  # 队列本来就是空的，直接放新的
        try:
            self.q.put_nowait(snap)  # 放最新快照
            return True
        except queue.Full:  # 理论上不会（刚清过），兜底防死锁
            self.dropped += 1
            return False

    # ---------------------------------------------------------------- 组帧
    @staticmethod
    def build_frame(snap):
        """快照 dict -> '$AUV,...#' 文本帧（纯函数，便于离线验证格式）"""
        parts = []  # 各字段字符串
        for k in FIELDS:  # 严格按索引顺序拼，字段数恒定 = 16
            parts.append(_fmt(snap.get(k) if isinstance(snap, dict) else None))
        body = ','.join(parts)  # 逗号分隔
        return '$AUV,' + body + '#\r\n'  # 与上位机其它帧一样：# 结尾 + CRLF

    # ---------------------------------------------------------------- 线程
    def _loop(self):
        """上报线程主体：取最新快照 → 按节拍节流 → 组帧 → 非阻塞发送 → 失败计数"""
        period = (1.0 / self.hz) if self.hz > 0 else 0.2  # 节拍间隔；hz 非法时按 0.2s
        next_ts = 0.0  # 下一次允许发送的时刻
        while not self._stop.is_set():  # 收到停止信号才退出
            snap = None  # 本轮要发的快照
            try:
                snap = self.q.get(timeout=0.2)  # 等 0.2s，留出检查停止位的机会
            except queue.Empty:  # 没有新快照
                continue  # 不发重复数据，继续等
            except Exception:  # 队列异常（理论上不会）—— 绝不让线程死在主循环前面
                continue
            now = time.time()  # 取当前时刻
            if now < next_ts:  # ★ 未到节拍：这一份直接丢掉（反正下一拍还会有新的）
                continue  # 节流的关键：20Hz 塞进来也只按 AUV_REPORT_HZ 往外发
            next_ts = now + period  # 排下一次发送时刻
            text = self.build_frame(snap)  # 组帧（纯函数，不会抛）
            try:
                self.sock.sendto(text.encode('utf-8'), (self.ip, self.port))  # 非阻塞发送
                self.ok += 1  # 成功计数
                self._run_fail = 0  # 连续失败清零
            except Exception as e:  # 网络不可达 / 缓冲区满 / socket 已关
                self.fail += 1  # 累计失败
                self._run_fail += 1  # 连续失败
                if self._run_fail == 1:  # 只在第一次失败时打日志，避免刷屏
                    self.log('[AUV-REP] 发送失败: %s' % e)
                if self._run_fail >= self.max_fail:  # ★ 达到上限 → 永久放弃
                    self.dead = True  # 标记放弃（push 会直接返回）
                    self.log('[AUV-REP] 连续 %d 次发送失败 -> 放弃回传，'
                             '任务继续正常运行（要恢复请重新切入 AUV 模式）' % self._run_fail)
                    return  # 线程退出，不再重试

    # ---------------------------------------------------------------- 状态
    def summary(self):
        """一行状态摘要，供日志/排障用"""
        if not self.enabled:  # 开关关着
            return '关'
        if self.dead:  # 已放弃
            return '已放弃(失败%d次)' % self.fail
        return '%s:%d 成功%d/失败%d' % (self.ip, self.port, self.ok, self.fail)

    def alive(self):
        """是否还在正常回传（供外部判断是否值得继续 push）"""
        return bool(self.enabled and not self.dead and self.thread is not None)


def make_snapshot(now, cmd, mission, tel=None):
    """把状态机当前状态打包成上报快照（纯函数，便于台架直接断言字段）

    now=当前时刻；cmd=mission.step() 的返回值；mission=Mission 实例；tel=最近一帧遥测。
    """
    snap = {
        'stage': (cmd or {}).get('stage', '-'),  # 阶段名
        't': round(float(now) - float(getattr(mission, 't0', 0.0)), 2),  # 本阶段已运行秒数
        'depth_cmd': (cmd or {}).get('depth'),  # 下发给固件的目标深度
        'yaw_cmd': (cmd or {}).get('yaw'),  # 下发给固件的目标航向
        'surge': (cmd or {}).get('surge'),  # 前向推力
        'sway': (cmd or {}).get('sway'),  # 横向推力
        'seq': int(getattr(mission, 'seq', 0) or 0),  # 状态机帧序号
    }
    dep = getattr(mission, 'last_dep', None)  # 最近一次深度读取结果
    if isinstance(dep, dict):  # 有深度源
        snap['depth_now'] = dep.get('depth_cm')  # 当前深度
        snap['depth_ok'] = 1 if dep.get('ok') else 0  # 是否可信
    else:  # 无深度源
        snap['depth_now'] = -1  # 用 -1 表示"没有"，别写 0 会被当成水面
        snap['depth_ok'] = 0
    obs = getattr(mission, 'last_obs', None)  # 最近一次视觉观测
    if isinstance(obs, dict):  # 看到过目标
        snap['vis_cam'] = getattr(mission, 'last_obs_cam', '-')  # front / bottom
        snap['vis_label'] = obs.get('canon') or obs.get('label')  # 归一化类别名
        snap['vis_dx'] = obs.get('dx')  # 横向偏差（右为正）
        snap['vis_dy'] = obs.get('dy')  # 纵向偏差（下为正）
        snap['vis_w'] = obs.get('w')  # 目标框宽度
        snap['vis_age'] = round(max(0.0, float(now) - float(getattr(mission, 'last_obs_ts', 0.0))), 2)  # 距今多久
    else:  # 从未看到目标
        snap['vis_cam'] = '-'
        snap['vis_label'] = '-'
        snap['vis_age'] = -1  # -1 = 从没看到过
    snap['abort'] = getattr(mission, 'abort_reason', '') or ''  # 中止原因（空 = 正常）
    return snap


if __name__ == '__main__':
    # 独立调试：python3 auv_report.py [目标IP] [端口]  —— 循环发 3 帧例子帧后退出
    import sys  # 命令行参数
    import types as _types  # 造一个最小配置对象

    _ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.127.100'  # 目标 IP
    _port = int(sys.argv[2]) if len(sys.argv) > 2 else 8085  # 目标端口
    _cfg = _types.SimpleNamespace(AUV_REPORT_IP=_ip, AUV_REPORT_PORT=_port,  # 拼配置
                                  AUV_REPORT_HZ=2.0, AUV_REPORT_MAX_FAIL=3,
                                  AUV_REPORT_ENABLED=True)

    def _p(msg):
        """带时间戳的日志回调（独立调试用）"""
        print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)

    r = AuvReport(_cfg, _p)  # 建上报器
    r.start()  # 起线程
    for i in range(3):  # 发 3 帧例子
        r.push({'stage': 'RAM_BALL', 't': i * 0.5, 'depth_cmd': 72.9, 'yaw_cmd': 4.7,
                'surge': 0.5, 'sway': 0.0, 'depth_now': 71.2, 'depth_ok': 1,
                'vis_cam': 'front', 'vis_label': 'ball', 'vis_dx': -12.0,
                'vis_dy': 8.0, 'vis_w': 150.0, 'vis_age': 0.05, 'abort': '', 'seq': i})
        time.sleep(0.5)  # 等半秒
    time.sleep(0.5)  # 等最后一帧发出
    r.stop()  # 收尾
