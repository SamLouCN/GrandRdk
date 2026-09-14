#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# optical_flow_demo.py - RDK S100 实时光流演示(调度框架)
# ============================================================
# 设计:
#   1. 本文件只负责摄像头采集、可视化与统计, 不再含具体光流算法
#   2. 光流算法全部封装在子目录 algos/ 的独立纯函数文件(algo_*.py)中:
#        algos/algo_dense.py   <- Farneback 稠密光流(mode = dense)
#        algos/algo_dis.py     <- DIS 稠密光流(推荐, mode = dis)
#        algos/algo_sparse.py  <- PyrLK 稀疏跟踪(mode = sparse)
#        algos/algo_common.py  <- 共享: HSV 渲染与运动统计(纯函数)
#      每个算法文件暴露统一签名纯函数:
#        process(prev_gray, gray, frame, cfg, state=None)
#            -> (viz, mean_motion, moving_ratio, state)
#      稠密算法无跨帧状态; 稀疏跟踪的角点经 state 显式传入传出
#   3. 通过 config.ini 的 [algorithm].mode 字段选择算法(dense/dis/sparse)
#   4. 两种实时可视化(可分别开关): 板端桌面窗口(HDMI/VNC) + Web 推流
# 运行方式: python3 optical_flow_demo.py(无命令行参数)
# 想换算法: 改 config.ini 的 [algorithm].mode 即可(dense/dis/sparse)
# 想加算法: 复制 algo_*.py 改名并实现同签名 process(), 再把 mode 指向它
# ============================================================

import configparser        # 标准库: 解析 config.ini
import importlib           # 标准库: 按模式名动态导入算法模块
import os                  # 标准库: 探测桌面显示环境(X11)
import socket              # 标准库: 探测本机 IP(打印 Web 地址)
import sys                 # 标准库: 错误退出码
import threading           # 标准库: 后台运行 Web 服务
import time                # 标准库: 计时(帧率)与重试等待
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # 标准库: MJPEG 推流

import cv2                 # OpenCV: 摄像头采集 / 图像编解码

CONFIG_PATH = "config.ini"           # 配置文件路径(与脚本同目录)
ALLOWED_MODES = ("dense", "dis", "sparse", "tvl1")  # 支持的算法模式


def load_config(path):
    """读取 ini 配置文件, 返回配置对象。"""
    cfg = configparser.ConfigParser()          # 创建 ini 解析器实例
    cfg.read(path, encoding="utf-8")           # 以 utf-8 读取(支持中文注释)
    return cfg                                 # 返回配置对象


def load_process(mode):
    """按模式名动态导入同名算法模块并返回其纯函数 process。"""
    module = importlib.import_module(f"algos.algo_{mode}")  # 载入 algos/algo_<mode>.py
    return module.process                             # 返回统一签名的纯函数


def get_local_ip():
    """探测板卡局域网 IP(无外网时退回 127.0.0.1)。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 建 UDP 套接字
    try:                                       # 探测过程可能无外网
        s.connect(("8.8.8.8", 80))             # 仅让内核选路, 不会真正发包
        ip = s.getsockname()[0]                # 取内核为本机分配的出口地址
    except OSError:                            # 无外网路由?
        ip = "127.0.0.1"                       # 退回回环地址兜底
    finally:                                   # 无论成功失败
        s.close()                              # 都要关闭套接字
    return ip                                  # 返回探测到的 IP


def display_available():
    """探测当前会话是否具备桌面显示条件(避免 Qt 后端在无显示时直接崩溃进程)。"""
    display = os.environ.get("DISPLAY", "")        # 取 DISPLAY 环境变量
    if not display:                                # 根本没设置 DISPLAY?
        return False                               # 判定为无显示可用
    num = display.split(":")[-1].split(".")[0]     # 解析显示编号(:0 -> 0)
    sock = f"/tmp/.X11-unix/X{num}"                # X server 的 socket 路径
    if not os.path.exists(sock):                   # X socket 不存在?
        return False                               # 判定为无显示可用
    auth = os.environ.get("XAUTHORITY", "")        # 取 X 授权文件路径
    if not auth:                                   # 未显式指定授权文件?
        auth = os.path.expanduser("~/.Xauthority")  # 退回默认授权文件
    return os.path.exists(auth)                    # 授权文件存在才认为可用


def open_camera(cfg):
    """打开真实摄像头; 失败重试一次, 仍失败则报错退出(绝不使用合成图像)。"""
    dev = cfg.getint("camera", "device_index")     # 摄像头编号(0=/dev/video0)
    w = cfg.getint("camera", "width")              # 期望采集宽度
    h = cfg.getint("camera", "height")             # 期望采集高度
    fps = cfg.getint("camera", "fps")              # 摄像头硬件采集帧率
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)      # 用 V4L2 后端打开摄像头
    if not cap.isOpened():                         # 第一次打开失败?
        print("[INFO] 摄像头打开失败, 2 秒后自动重试一次...")  # 友好提示
        time.sleep(2)                              # 等待设备/端口释放
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)  # 重试一次
    if not cap.isOpened():                         # 重试之后仍打不开?
        print(f"[ERROR] 无法打开摄像头 /dev/video{dev}, 请检查 USB 连接后重试")  # 明确报错
        sys.exit(1)                                # 直接退出, 不回退到任何模拟数据
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 请求 MJPG 压缩格式
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)           # 设置采集宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)          # 设置采集高度
    cap.set(cv2.CAP_PROP_FPS, fps)                 # 设置采集帧率
    return cap                                     # 返回可用的摄像头对象


class MjpegStreamer:
    """Web MJPEG 推流器: 保存最新 JPEG 帧并响应浏览器的连续拉流。"""

    def __init__(self):
        """初始化帧缓存与后台线程句柄。"""
        self._frame = None                         # 最新一帧 JPEG 字节
        self._lock = threading.Lock()              # 保护帧缓存
        self._httpd = None                         # HTTP 服务句柄
        self._thread = None                        # HTTP 服务后台线程

    def set_frame(self, jpg):
        """主线程调用: 把新编码好的 JPEG 帧写入缓存(只保留最新帧)。"""
        with self._lock:                           # 加锁写
            self._frame = jpg                      # 覆盖为最新帧

    def get_frame(self):
        """HTTP 线程调用: 读取当前最新帧(无帧返回 None)。"""
        with self._lock:                           # 加锁读
            return self._frame                     # 返回最新帧

    def start(self, port):
        """在后台启动 HTTP 服务,监听所有网卡的 port 端口。"""
        httpd = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(self))  # 建服务
        self._httpd = httpd                        # 保存服务句柄
        self._thread = threading.Thread(target=httpd.serve_forever, daemon=True)  # 后台线程
        self._thread.start()                       # 启动服务线程

    def stop(self):
        """停止 HTTP 服务(程序退出前调用, 释放端口)。"""
        if self._httpd is not None:                # 服务已启动?
            self._httpd.shutdown()                 # 停止受理
            self._httpd.server_close()             # 关闭监听 socket


def _make_handler(streamer):
    """根据推流器生成 HTTP 请求处理类。"""

    class Handler(BaseHTTPRequestHandler):
        """MJPEG 请求处理: / 返回页面, /video.mjpg 返回连续图像流。"""

        def do_GET(self):
            """按路径分发浏览器请求。"""
            if self.path in ("/", "/index.html"):  # 请求根页面?
                self._serve_page()                 # 返回简易观看页
            elif self.path == "/video.mjpg":       # 请求视频流?
                self._serve_stream()               # 返回 MJPEG 流
            else:                                  # 其它路径?
                self.send_response(404)            # 返回 404
                self.end_headers()                 # 结束响应头

        def _serve_page(self):
            """返回内嵌 <img> 的简易 HTML 页面(浏览器打开即出画面)。"""
            page = ('<!DOCTYPE html><html><head><meta charset="utf-8">'
                    '<title>RDK S100 光流实时画面</title></head>'
                    '<body style="background:#111;text-align:center">'
                    '<h3 style="color:#eee">光流可视化</h3>'
                    '<img src="/video.mjpg" style="max-width:95%"></body></html>')
            data = page.encode("utf-8")            # 页面文本转字节流
            self.send_response(200)                # 状态码 200
            self.send_header("Content-Type", "text/html; charset=utf-8")  # 类型
            self.send_header("Content-Length", str(len(data)))            # 长度
            self.end_headers()                     # 发送响应头
            self.wfile.write(data)                 # 发送页面正文

        def _serve_stream(self):
            """以 multipart/x-mixed-replace 持续推 JPEG 帧。"""
            self.send_response(200)                # 状态码 200
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")  # 流类型
            self.end_headers()                     # 发送响应头
            try:                                   # 浏览器随时可能断开
                while True:                        # 持续推帧直到浏览器关闭
                    jpg = streamer.get_frame()     # 取最新帧
                    if jpg is not None:            # 已有帧?
                        self.wfile.write(b"--frame\r\n")           # 写帧边界
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")  # 帧类型
                        self.wfile.write(b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")  # 帧长度
                        self.wfile.write(jpg + b"\r\n")           # 写 JPEG 数据
                    time.sleep(0.03)               # 约 30fps 轮询节奏
            except (BrokenPipeError, ConnectionResetError):  # 浏览器断开?
                pass                               # 静默结束该连接

        def log_message(self, fmt, *args):
            """静默覆盖默认访问日志, 避免每个请求都刷屏。"""

    return Handler                                # 返回处理类


def main():
    """主流程: 读配置 -> 加载算法函数 -> 开摄像头 -> 逐帧调度 -> 可视化/保存 -> 打印统计。"""
    cfg = load_config(CONFIG_PATH)                          # 加载全部配置
    mode = cfg.get("algorithm", "mode").strip().lower()     # 取算法模式
    if mode not in ALLOWED_MODES:                          # 模式名非法?
        print(f"[ERROR] config.ini 中 mode 只能填 {ALLOWED_MODES}, 当前是: {mode}")  # 报错提示
        return 3                                            # 退出
    max_frames = cfg.getint("camera", "max_frames")         # 最大帧数(0=无限)
    save_path = cfg.get("output", "save_image").strip()      # 渲染图保存路径
    log_interval = cfg.getint("output", "log_interval")     # 状态打印间隔
    enable_window = cfg.getboolean("visualization", "enable_window")  # 桌面窗口开关
    enable_web = cfg.getboolean("visualization", "enable_web")        # Web 推流开关
    web_port = cfg.getint("visualization", "web_port")                # Web 端口
    mjpg_quality = cfg.getint("visualization", "mjpg_quality")        # 推流画质

    process_algo = load_process(mode)                       # 按 mode 加载对应算法纯函数
    cap = open_camera(cfg)                                  # 打开真实摄像头(失败即退出)
    print(f"[INFO] 摄像头已打开: /dev/video{cfg.getint('camera', 'device_index')}")  # 确认
    print(f"[INFO] 当前算法模式: {mode} (algos.algo_{mode}.process)")  # 提示算法来源

    streamer = None                                         # Web 推流器(默认不启动)
    if enable_web:                                          # 配置允许 Web 推流?
        streamer = MjpegStreamer()                          # 创建推流器实例
        try:                                                # 端口可能被占
            streamer.start(web_port)                        # 后台启动 HTTP 服务
        except OSError:                                     # 端口绑定失败?
            print(f"[ERROR] Web 端口 {web_port} 被占用, 请修改 config.ini 的 web_port")  # 提示
            return 4                                        # 退出
        print(f"[INFO] Web 可视化地址: http://{get_local_ip()}:{web_port}/")  # 打印访问地址
    window_ok = enable_window and display_available()        # 窗口模式仅在显示可用时开启
    if enable_window and not window_ok:                      # 用户开了窗口但显示不可用?
        print("[WARN] 当前会话无可用桌面(DISPLAY), 已自动关闭窗口模式, 请用 Web 可视化")  # 提示降级

    t0 = time.time()                                        # 起始时间(算帧率)
    frame_count = 0                                         # 已处理帧数
    mean_motion = 0.0                                       # 平均运动量(用于打印)
    moving_ratio = 0.0                                      # 运动像素占比
    last_viz = None                                         # 最近一帧渲染图(保存用)
    algo_state = None                                       # 算法跨帧状态(稀疏存角点, 稠密为 None)
    prev_gray = None                                        # 上一帧灰度图
    prev_frame = None                                       # 上一帧彩色图

    try:                                                    # 便于捕获 Ctrl+C
        while True:                                         # 主采集循环
            ok, frame = cap.read()                          # 读取一帧真实图像
            if not ok:                                      # 采集中途失败?
                print("[ERROR] 读取摄像头帧失败, 程序退出(未使用任何模拟数据)")  # 报错
                return 2                                    # 非零退出
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 当前帧转灰度
            if prev_gray is None:                           # 第一帧还没有"上一帧"?
                prev_gray = gray                            # 缓存为上一帧灰度
                prev_frame = frame.copy()                   # 缓存上一帧彩色
                continue                                    # 跳过计算, 读下一帧

            last_viz, mean_motion, moving_ratio, algo_state = process_algo(prev_gray, gray, frame, cfg, algo_state)  # 调度纯函数算一帧

            frame_count += 1                                # 成功处理一帧, 计数+1
            fps = frame_count / (time.time() - t0)          # 计算当前平均帧率
            if frame_count % log_interval == 0:             # 到达打印间隔?
                print(f"[{mode}] frame={frame_count} fps={fps:5.1f} "  # 打印帧号与帧率
                      f"mean_motion={mean_motion:5.2f} moving_ratio={moving_ratio:.3f}")  # 打印运动统计

            if enable_web and streamer is not None:         # Web 推流开启?
                ok_enc, buf = cv2.imencode(".jpg", last_viz, [cv2.IMWRITE_JPEG_QUALITY, mjpg_quality])  # 渲染图 JPEG 编码
                if ok_enc:                                  # 编码成功?
                    streamer.set_frame(buf.tobytes())       # 交给推流器广播给浏览器
            if window_ok:                                   # 窗口模式可用?
                try:                                        # 无桌面环境会抛 cv2.error
                    cv2.imshow("RDK S100 光流", last_viz)   # 在桌面窗口实时显示渲染图
                except cv2.error:                           # 没有 DISPLAY 等显示环境?
                    window_ok = False                       # 永久关闭窗口模式
                    print("[WARN] 桌面显示不可用(无 DISPLAY?), 已自动关闭窗口, 请用 Web 查看")  # 提示
                if cv2.waitKey(1) & 0xFF == ord("q"):       # 用户按了 q 键?
                    print("[INFO] 收到 q 键, 结束")          # 提示退出原因
                    break                                   # 结束主循环

            prev_gray = gray                                # 当前帧灰度变为下一轮"上一帧"
            prev_frame = frame.copy()                       # 同步更新彩色缓存
            if max_frames > 0 and frame_count >= max_frames:  # 达到设定的最大帧数?
                break                                       # 正常结束循环
    except KeyboardInterrupt:                               # 用户按了 Ctrl+C?
        print("[INFO] 收到中断信号, 结束采集")               # 友好提示

    elapsed = time.time() - t0                              # 计算总耗时
    fps = frame_count / elapsed if elapsed > 0 else 0.0     # 计算平均帧率
    print("=" * 60)                                         # 输出分隔线
    print(f"summary: mode={mode} frames={frame_count} "     # 汇总: 模式与帧数
          f"avg_fps={fps:.1f} mean_motion={mean_motion:.2f} "  # 汇总: 平均帧率与运动量
          f"moving_ratio={moving_ratio:.3f}")               # 汇总: 运动占比
    if save_path and last_viz is not None:                  # 配置要求保存且有渲染图?
        cv2.imwrite(save_path, last_viz)                    # 保存最后一帧渲染图
        print(f"[save] {save_path}")                        # 提示保存位置
    if window_ok:                                           # 窗口模式曾成功打开?
        cv2.destroyAllWindows()                             # 关闭所有 OpenCV 窗口
    if streamer is not None:                                # Web 推流已启动?
        streamer.stop()                                     # 停止 HTTP 服务释放端口
    return 0                                                # 正常结束


if __name__ == "__main__":                                  # 作为主程序运行时?
    sys.exit(main())                                        # 进入主流程并以返回值退出