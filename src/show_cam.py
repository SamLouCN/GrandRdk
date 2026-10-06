#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
show_cam.py — 第三路相机(CAM3) 按需采集 + MJPEG 推流

    位置: /userdata/GrandRDK/src/show_cam.py
          2026-09-18 由 /userdata/show_cam.py 迁入, 由 run.sh/stop.sh/status.sh/logs.sh 统一托管,
          日志 logs/show_cam.log
    运行:  cd /userdata/GrandRDK && ./run.sh      (一键启动, 本文件随之启动)
           或单独: python3 src/show_cam.py
    停止:  ./stop.sh  (或 pkill -f src/show_cam.py)

    按需启用 (2026-09-19 改造):
      * HTTP 服务(0.0.0.0:8084)常驻, 但进程本身很轻: 不占相机、不编码、几乎不吃 CPU;
      * 只有上位机真的"打开 CAM3 取流"(即连上 /stream 或 /cam3)时, 才打开 cam3 节点开始采集;
      * 最后一个取流客户端断开后, 空闲 IDLE_RELEASE_SEC 秒自动释放相机(关闭 V4L2 句柄),
        相机回到空闲, 可被其它程序使用, 也省电/降温;
      * /snapshot 会临时唤醒相机取一帧; /status 只读状态, 不唤醒相机。
      * 上位机侧无需任何改动: ROV控制站 pc_main2.py 的 "CAM3 取流" 按钮
        用 cv2.VideoCapture("http://<板卡IP>:8084/stream") 拉流, 连上即触发开机, 断开即触发释放。

    上位机取流（网线）:
        http://192.168.127.10:8084/stream     ← 主地址（上位机 v3.3 CAM3 取流默认用这个）
        http://192.168.127.10:8084/cam3       ← 同内容别名
        http://192.168.127.10:8084/snapshot   ← 单张 JPEG 快照（也会临时唤醒相机）
        http://192.168.127.10:8084/status     ← JSON 状态（相机是否开启/客户端数/帧率, 不唤醒相机）
        http://192.168.127.10:8084/           ← 浏览器直接看的小页面

    相机分配（2026-09-23 实测 usb 拓扑, 按物理口固定, 勿混用）:
        USB 口 3-2 (USB Camera 0bda:5883)                            -> front.py  前视 / cam1
        USB 口 1-2 (USB Camera 0bda:5883)                            -> bottom.py 下视 / cam2
        USB 口 1-1 (ENDOSCOPE HDCAM 090c:f37d 内窥镜)                  -> 本文件(第三路 / cam3)
    设备节点号由 config/camera_ports.py 按 USB 物理口实时解析, 节点号不作为配置真值;
    因此上电/重插导致 videoN 编号漂移时, 本文件也不需要改。

    要改行为就改下面这几个常量（没有命令行参数）。
"""
import json  # /status 接口的 JSON 序列化
import os  # 路径拼接, 定位 config 目录
import signal  # 注册 SIGINT/SIGTERM 做优雅退出
import sys  # 注入模块搜索路径
import threading  # 采集线程与条件变量
import time  # 时间戳、退避计时、帧率节流
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # 标准库 HTTP 多线程服务

import cv2  # OpenCV: V4L2 采集 + JPEG 编码

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))  # 把上级目录的 config/ 加进搜索路径, 才能 import camera_ports
try:
    from camera_ports import device_for  # 按 USB 物理口解析出设备节点
    DEVICE = os.environ.get('GRDK_CAM3_DEV') or device_for('cam3')     # 第三路 = USB 物理口 1-1；GRDK_CAM3_DEV=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
except Exception as _exc:           # 解析模块异常时不拖垮 CAM3 进程
    print('[show_cam] 警告: camera_ports 解析失败 %r, 回退 cam3 节点占位（永远打开失败以便上层报警）' % (_exc,), flush=True)
    DEVICE = '/dev/_no_node_fallback_show_cam'  # 占位路径: 永远打开失败
WIDTH, HEIGHT, FPS = 640, 480, 30   # 采集参数
STREAM_PORT = 8084                  # 推流端口
STREAM_FPS = 20                     # 有客户端时的推流帧率上限
IDLE_FPS = 1.0                      # 无客户端但相机尚未释放时的保活帧率
JPEG_QUALITY = 100                  # JPEG 质量 1-100
IDLE_RELEASE_SEC = 5.0              # 最后一个取流客户端断开后, 空闲多久释放相机
FIRST_FRAME_TIMEOUT = 10.0          # 客户端等待首帧(含开相机+预热)的最长时间
OPEN_RETRY_SEC = 2.0                # 相机打不开时的重试间隔(避免死循环高频占用设备)
FRAME_FRESH_SEC = 2.0               # 超过该年龄的缓存帧视为失效, 不发给新客户端
WARMUP_FRAMES = 3                   # 开机后丢弃的前几帧(USB 相机首帧常偏暗/花屏)

BOUNDARY = b"frame"  # MJPEG multipart 分隔符


def log(msg):
    print("[show_cam] %s" % msg, flush=True)  # flush=True 保证日志实时落盘, 便于 logs.sh 观察


class Camera:
    """按需持有 cam3 节点: 有取流需求才打开, 空闲自动释放。

    - clients          : 正在取流的 HTTP 客户端数(MJPEG)
    - snapshot_waiters : 正在等快照的请求数
    - demand_ts        : 最近一次"有需求"的时刻; 最后一次需求后 IDLE_RELEASE_SEC 秒释放
    """

    def __init__(self, device=DEVICE):
        self.device = device  # V4L2 设备节点路径
        self.cv = threading.Condition()  # 保护下面所有共享状态并用于等帧通知
        self.jpeg = None            # 最新一帧 JPEG(按需推送, 带序号)
        self.jpeg_ts = 0.0          # 该帧产生时刻
        self.seq = 0  # 帧序号, 递增用于客户端判重
        self.frames = 0             # 累计编码帧数
        self.clients = 0  # 正在 MJPEG 取流的客户端数
        self.snapshot_waiters = 0  # 正在等 /snapshot 的请求数
        self.demand_ts = 0.0  # 最近一次"有取流需求"的时刻
        self.cam_open = False  # 相机是否已打开
        self.opened_at = 0.0  # 本次打开的时刻, 算已开时长
        self.sessions = 0           # 相机累计被打开次数(诊断用)
        self.last_error = ""  # 最近一次错误信息, 供 /status 与 503 上报
        self.pub_times = []         # 最近发布时刻, 用于算实时帧率
        self.cap = None  # cv2.VideoCapture 句柄, 未开时为 None
        self._retry_after = 0.0  # 该时刻之前不再尝试打开(失败退避)
        self._stop = False  # 采集线程停止标志
        self._thread = None  # 采集线程对象

    # ---------------- 需求登记 ----------------
    def touch_demand(self):
        with self.cv:  # 加锁保护共享状态
            self.demand_ts = time.time()  # 刷新需求时刻, 推迟自动释放

    def _has_demand(self):
        """调用前需持有 self.cv。"""
        return (self.clients > 0 or self.snapshot_waiters > 0  # 有人取流或等快照
                or (time.time() - self.demand_ts) < IDLE_RELEASE_SEC)  # 或还在空闲宽限期内

    # ---------------- 帧访问 ----------------
    def wait_frame(self, last_seq, timeout):
        """等一帧比 last_seq 新且未过期的 JPEG; 超时返回 (None, last_seq)。

        等待期间持续刷新需求时间戳 -> 相机不会在有人等帧时被释放。
        """
        deadline = time.time() + timeout  # 绝对超时时刻
        with self.cv:
            while True:
                if (self.jpeg is not None and self.seq != last_seq  # 有新帧且序号变了
                        and (time.time() - self.jpeg_ts) <= FRAME_FRESH_SEC):  # 且帧未过期
                    return self.jpeg, self.seq
                left = deadline - time.time()  # 剩余等待时间
                if left <= 0:
                    return None, last_seq  # 超时: 返回空, 让调用方决定是否报 503
                self.demand_ts = time.time()  # 阻塞前再续一次需求, 防止相机被释放
                self.cv.wait(min(left, 0.5))  # 最多等 0.5s 再检查, 避免错过唤醒

    def is_open(self):
        with self.cv:
            return self.cam_open  # 相机当前是否打开

    def snapshot(self):
        """取一帧(必要时唤醒相机); 返回 JPEG bytes 或 None。"""
        with self.cv:
            self.snapshot_waiters += 1  # 登记需求, 触发 _ensure_open
        try:
            jpeg, _ = self.wait_frame(0, FIRST_FRAME_TIMEOUT)  # seq=0 表示取任意新的一帧
            return jpeg
        finally:
            with self.cv:
                self.snapshot_waiters -= 1  # 无论成功失败都要撤销计数, 否则相机会常开

    def status(self):
        with self.cv:
            now = time.time()  # 统一取一次时间, 避免统计口径不一致
            recent = [t for t in self.pub_times if now - t <= 2.0]  # 截取最近 2 秒的发布记录
            return {
                "device": self.device,  # 实际使用的设备节点
                "cam_open": self.cam_open,  # 相机是否已打开
                "stream_clients": self.clients,  # 在线取流客户端数
                "snapshot_waiters": self.snapshot_waiters,  # 等快照的请求数
                "frames_total": self.frames,  # 累计编码帧数
                "fps": round(len(recent) / 2.0, 1),  # 近 2 秒平均帧率
                "open_sessions": self.sessions,  # 相机累计开机次数
                "opened_for_sec": round(now - self.opened_at, 1) if self.cam_open else 0.0,  # 本次已开时长, 未开为 0
                "idle_release_sec": IDLE_RELEASE_SEC,  # 空闲释放阈值, 供上位机显示
                "last_error": self.last_error,  # 最近错误(空串表示正常)
            }

    # ---------------- 采集线程 ----------------
    def start(self):
        self._thread = threading.Thread(target=self._run, name="capture", daemon=True)  # 守护线程, 主进程退出即回收
        self._thread.start()

    def stop(self):
        self._stop = True  # 让采集循环自然退出
        with self.cv:
            self.cv.notify_all()  # 唤醒可能在 cv.wait 上阻塞的采集线程
        if self._thread is not None:
            self._thread.join(timeout=3.0)  # 最多等 3 秒, 超时不强杀
        self._release("进程退出")  # 无论如何都要关掉相机句柄

    def _ensure_open(self):
        """确保相机已打开; 返回 VideoCapture 或 None(打不开/在退避期)。"""
        with self.cv:
            if self.cam_open and self.cap is not None:
                return self.cap  # 已打开, 直接复用
            if time.time() < self._retry_after:
                return None  # 还在失败退避期, 不反复尝试开设备
        cap = cv2.VideoCapture(self.device)  # 在锁外打开, 避免长时间持锁阻塞 HTTP 线程
        if not cap.isOpened():
            cap.release()  # 打开失败也要释放句柄, 防止 fd 泄漏
            with self.cv:
                self.last_error = "打不开摄像头 %s" % self.device  # 记录原因供状态上报
                self._retry_after = time.time() + OPEN_RETRY_SEC  # 设定下次可重试的时刻
            log("警告: 打不开 %s, %.0fs 后重试" % (self.device, OPEN_RETRY_SEC))
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)  # 请求采集宽度
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)  # 请求采集高度
        cap.set(cv2.CAP_PROP_FPS, FPS)  # 请求采集帧率(驱动不一定支持)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 回读实际生效宽度
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 回读实际生效高度
        with self.cv:
            self.cap = cap  # 登记句柄
            self.cam_open = True  # 标记已开
            self.opened_at = time.time()  # 记录开机时刻
            self.sessions += 1  # 开机次数累加(诊断用)
            self.last_error = ""  # 打开成功即清空历史错误
            self._retry_after = 0.0  # 清除退避计时
            self.jpeg = None        # 作废旧帧, 避免新客户端看到关机前的历史画面
            self.jpeg_ts = 0.0  # 帧时间戳归零, 旧缓存立刻判定过期
        log("信息: 已打开 %s: %dx%d (第 %d 次开机)" % (self.device, w, h, self.sessions))
        return cap

    def _release(self, reason):
        with self.cv:
            cap, was_open = self.cap, self.cam_open  # 取出句柄后在锁外释放
            self.cap = None  # 先置空句柄
            self.cam_open = False  # 标记已关
            self.jpeg = None  # 丢弃缓存帧, 下次重开不会串画面
            self.jpeg_ts = 0.0  # 时间戳归零
        if cap is None and not was_open:
            return  # 本来就关着, 无需重复打日志
        if cap is not None:
            cap.release()  # 真正关闭 V4L2 设备, 让相机可被其它程序占用
        log("信息: 释放 %s (%s)" % (self.device, reason))

    def _run(self):
        last_pub = 0.0  # 上一帧发布时刻, 用于限速
        warm = 0  # 已丢弃的预热帧计数
        while not self._stop:
            with self.cv:
                need = self._has_demand()  # 当前是否有保留相机的理由
                has_client = self.clients > 0  # 是否有真实在线客户端(决定推流帧率)
            if not need:
                self._release("空闲 %.0fs 无取流客户端" % IDLE_RELEASE_SEC)  # 无需求则释放相机
                with self.cv:
                    self.cv.wait(0.5)  # 休眠等待新需求唤醒, 不空转 CPU
                continue

            cap = self._ensure_open()
            if cap is None:
                with self.cv:
                    self.cv.wait(0.5)  # 打不开就退避, 避免高频重试占设备
                continue

            ok, frame = cap.read()  # 抓一帧
            if not ok:
                self._release("读帧失败(相机掉线或被占用)")  # 读失败说明链路断了, 关掉重来
                time.sleep(0.3)  # 稍等再试, 防止掉线时刷爆日志
                continue
            if warm < WARMUP_FRAMES:     # 开机预热: 丢掉最初几帧
                warm += 1
                continue

            now = time.time()
            if now - last_pub < 1.0 / (STREAM_FPS if has_client else IDLE_FPS):  # 按是否有客户端限速: 20fps 或 1fps 保活
                continue
            last_pub = now
            ok_enc, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])  # 编码为 JPEG
            if not ok_enc:
                continue  # 编码失败直接丢弃这一帧
            with self.cv:
                self.jpeg = buf.tobytes()  # 更新最新 JPEG
                self.jpeg_ts = now  # 记录产生时刻, 用于新鲜度判断
                self.seq += 1  # 序号自增, 通知等待中的客户端
                self.frames += 1  # 累计帧数
                self.pub_times.append(now)  # 追加发布时刻用于算帧率
                if len(self.pub_times) > 120:
                    del self.pub_times[:60]  # 超出上限就砍掉老的一半, 防止列表无限增长
                self.cv.notify_all()  # 唤醒所有等帧的 HTTP 线程


class Handler(BaseHTTPRequestHandler):
    cam = None          # 由 main() 注入

    # ---------------- 工具 ----------------
    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')  # ensure_ascii=False 让中文正常显示
        self.send_response(code)  # 发送状态码
        self.send_header('Content-Type', 'application/json; charset=utf-8')  # 声明 JSON 编码
        self.send_header('Content-Length', str(len(body)))  # 定长, 便于客户端收完整
        self.send_header('Connection', 'close')  # 短连接, 响应后立即断开
        self.end_headers()  # 结束头部, 开始正文
        try:
            self.wfile.write(body)  # 写响应体
        except OSError:
            pass  # 客户端已断开, 写失败忽略即可

    def _send_bytes(self, body, ctype):
        self.send_response(200)  # 固定 200
        self.send_header('Content-Type', ctype)  # 按调用方给的 MIME 声明
        self.send_header('Content-Length', str(len(body)))  # 字节长度
        self.send_header('Connection', 'close')  # 短连接
        self.end_headers()  # 头部结束
        try:
            self.wfile.write(body)  # 写二进制正文
        except OSError:
            pass  # 断连忽略

    # ---------------- 路由 ----------------
    def do_GET(self):
        path = self.path.split('?')[0].rstrip('/') or '/'  # 去掉查询串与尾部斜杠
        if path in ('/stream', '/cam3'):
            self._serve_mjpeg()  # MJPEG 流, 触发相机开机
        elif path == '/snapshot':
            self._serve_snapshot()  # 单帧快照, 临时唤醒相机
        elif path == '/status':
            self._send_json(self.cam.status())  # 只查状态, 不唤醒相机
        elif path == '/':
            html = ("<!doctype html><meta charset='utf-8'><title>show_cam</title>"  # 简易查看页: 头部
                    "<body style='margin:0;background:#111;text-align:center'>"  # 黑底居中
                    "<img src='/stream' style='max-width:100%'>").encode('utf-8')  # 唯一 img 标签拉 MJPEG
            self._send_bytes(html, 'text/html; charset=utf-8')  # 返回页面
        else:
            self.send_error(404)  # 未知路径一律 404

    # ---------------- MJPEG 取流(按需开机) ----------------
    def _serve_mjpeg(self):
        cam = self.cam
        peer = self.client_address[0]  # 客户端 IP, 只用于日志
        with cam.cv:
            cam.clients += 1  # 客户端计数 +1, 表示有取流需求
            cam.demand_ts = time.time()  # 刷新需求时刻
            n = cam.clients  # 取 increments 后的在线数(仅日志用)
        log("信息: 取流接入 %s (在线客户端 %d, 相机%s)"  # 记录谁接入了
            % (peer, n, "已开" if cam.is_open() else "待开"))
        try:
            jpeg, seq = cam.wait_frame(0, FIRST_FRAME_TIMEOUT)   # 需要时会自动开相机
            if jpeg is None:
                self.send_error(503, "camera not ready: %s" % (cam.status()['last_error'] or "no frame"))  # 首帧超时: 带上错误原因便于排查
                return
            self.send_response(200)  # 有帧了再回 200, 避免客户端收到空流
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=' + BOUNDARY.decode())  # MJPEG 标准类型
            self.send_header('Cache-Control', 'no-cache')  # 禁止代理/浏览器缓存
            self.send_header('Connection', 'close')  # 断开即视为停止取流
            self.end_headers()  # 头部结束, 后续持续写帧

            last = seq  # 已发出的最新序号
            while True:
                jpeg, seq = cam.wait_frame(last, 1.0)  # 等新帧, 1s 超时用于检查相机状态
                if jpeg is None:
                    if not cam.is_open():      # 相机掉线/打不开 -> 结束本次连接, 让上位机重连
                        log("警告: 相机不可用, 结束本次取流连接 (%s)" % peer)
                        return
                    continue  # 相机还在但帧慢, 继续等
                last = seq  # 更新已发序号
                try:
                    self.wfile.write(b'--' + BOUNDARY +
                                     b'\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')  # 拼一个 MJPEG 分块
                    self.wfile.flush()  # 立即刷出, 保证实时性
                except OSError:
                    return  # 客户端断开, 退出推流循环
        finally:
            with cam.cv:
                cam.clients -= 1  # 必须在 finally 里减计数, 否则相机永不释放
                n = cam.clients  # 剩余在线数(日志用)
            log("信息: 取流断开 %s (剩余客户端 %d, 相机将在无客户端 %.0fs 后释放)"
                % (peer, n, IDLE_RELEASE_SEC))

    # ---------------- 快照(临时唤醒) ----------------
    def _serve_snapshot(self):
        jpeg = self.cam.snapshot()  # 取一帧(内部登记 snapshot_waiters)
        if jpeg is None:
            self.send_error(503, "camera not ready: %s"
                            % (self.cam.status()['last_error'] or "no frame"))  # 取不到就报 503 并附原因
            return
        self._send_bytes(jpeg, 'image/jpeg')  # 成功则直接返回 JPEG

    def log_message(self, *args):
        pass                                        # 不刷访问日志


def main():
    cam = Camera()
    Handler.cam = cam  # 把相机实例注入 HTTP Handler 类
    cam.start()  # 启动后台采集线程

    httpd = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), Handler)  # 监听所有网卡
    httpd.daemon_threads = True  # 请求线程设为守护, 退出时不阻塞

    stop_evt = threading.Event()  # 退出信号标志

    def _on_signal(signum, _frame):
        log("信息: 收到信号 %s, 退出中..." % signum)
        stop_evt.set()  # 置位事件唤醒主循环

    for _sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(_sig, _on_signal)  # 注册信号回调
        except (ValueError, OSError):
            pass  # 非主线程等场景注册失败, 忽略即可

    threading.Thread(target=httpd.serve_forever, name="http", daemon=True).start()  # HTTP 服务放后台线程
    log("信息: HTTP 已监听 0.0.0.0:%d (按需模式: 上位机取流才开相机, 空闲 %.0fs 自动释放)"
        % (STREAM_PORT, IDLE_RELEASE_SEC))
    log("信息: http://<板卡IP>:%d/stream  /snapshot  /status  (Ctrl-C 停止)" % STREAM_PORT)

    try:
        while not stop_evt.is_set():
            stop_evt.wait(1.0)  # 可被事件或信号即时唤醒, 同时不忙等
    except KeyboardInterrupt:
        pass  # Ctrl-C 走正常清理流程
    finally:
        cam.stop()  # 停采集并释放相机
        try:
            httpd.shutdown()  # 关闭 HTTP 服务
        except Exception:
            pass  # 关闭失败不影响退出
    return 0  # 0 = 正常退出码


if __name__ == "__main__":
    raise SystemExit(main())  # 用 main 返回值作为进程退出码
