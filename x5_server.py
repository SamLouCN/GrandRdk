"""
============================================================
  【运行在: RDK X5 (Ubuntu)】  ROV 中间层服务
============================================================
功能: 1) 接收PC的UDP指令, 串口透传给STM32
      2) 读取STM32串口遥测, UDP回传给PC
      3) 双摄像头MJPEG推流(Flask)
运行: python x5_server.py
依赖: pip install flask pyserial opencv-python

架构位置:
  [PC pc_main.py] ---UDP 8080---> [本程序 X5] ---串口---> [STM32]
  [PC pc_main.py] <--HTTP 5000--- [本程序 X5 Flask推流]
  [PC pc_main.py] <--UDP 8081---- [本程序 X5 遥测回传]
"""
import socket, time, threading, cv2, sys

try:
    import serial
except ImportError:
    print("[WARN] pyserial未安装, 串口功能不可用, pip install pyserial")
    serial = None

from flask import Flask, Response

# 导入共享协议
try:
    from protocol import *
    from protocol import parse_vid
except ImportError:
    CMD_PORT = 8080; TELEM_PORT = 8081; VIDEO_PORT = 5000
    SERIAL_PORT = "/dev/ttyACM0"; BAUD_RATE = 115200
    CAM1_ID = 0; CAM2_ID = 2
    CAM3_ID = -1   # v3.3: 第三路画面(预留); -1=未接入, 接好摄像头后改成实际 /dev/videoX 序号
    CAM_W = 640; CAM_H = 480; CAM_FPS = 15; JPEG_QUALITY = 60
    def parse_vid(frame):
        frame = frame.strip()
        if not (frame.startswith("$VID,") and frame.endswith("#")):
            return None
        try:
            return int(frame[5:-1]) == 1
        except ValueError:
            return None

pc_addr = None  # 自动记录PC地址, 用于遥测回传

# v3.2: 视频传输开关(由PC端 $VID 帧控制), False时停止采集推流省CPU/带宽
video_enabled = True

# ====================== 指令透传线程 ======================
def cmd_relay():
    """接收PC的 $CMD/$PID 帧, 原样透传给STM32串口; $VID 帧板端拦截(控制推流)"""
    global pc_addr, video_enabled
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CMD_PORT))
    print(f"[CMD] 监听 UDP :{CMD_PORT}")

    ser = None
    if serial:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"[UART] 已连接 {SERIAL_PORT} @ {BAUD_RATE}")
        except Exception as e:
            print(f"[UART] 打开失败: {e} (空跑模式)")

    while True:
        try:
            raw, addr = sock.recvfrom(512)
            pc_addr = addr
            text = raw.decode("utf-8", errors="ignore").strip()
            # v3.2: $VID 帧板端处理, 不透传STM32
            vid = parse_vid(text)
            if vid is not None:
                video_enabled = vid
                print(f"[VID] 视频传输 -> {'开' if vid else '关'}")
                continue
            if ser and ser.is_open:
                ser.write(raw)
        except Exception as e:
            print(f"[CMD] err: {e}"); time.sleep(0.1)

# ====================== 遥测回传线程 ======================
def telem_relay():
    """读取STM32串口的 $TEL 帧, UDP回传给PC"""
    global pc_addr
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ser = None
    if serial:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except:
            pass

    if not ser:
        print("[TEL] 串口不可用, 启动模拟遥测(调试用)")
        import random
        while True:
            if pc_addr:
                fake = f"$TEL,{random.uniform(-5,5):.2f},{random.uniform(-5,5):.2f},{random.uniform(0,360):.2f},0.00,0.00,0.00,{random.uniform(0,10):.2f}#\r\n"
                sock.sendto(fake.encode(), (pc_addr[0], TELEM_PORT))
            time.sleep(0.1)
        return

    print("[TEL] 监听STM32遥测...")
    buf = ""
    while True:
        try:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("$TEL,") and line.endswith("#") and pc_addr:
                        sock.sendto((line+"\r\n").encode(), (pc_addr[0], TELEM_PORT))
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[TEL] err: {e}"); time.sleep(0.5)

# ====================== PING 应答(供 PC 端 RTT 延迟测量) ======================
def ping_responder():
    """监听 TELEM_PORT, 收到 'PING' 则回 'PONG'"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", TELEM_PORT))
    print(f"[PING] 监听 TELEM_PORT :{TELEM_PORT}")
    while True:
        try:
            data, addr = sock.recvfrom(64)
            if data == b"PING":
                sock.sendto(b"PONG", addr)
        except Exception as e:
            print(f"[PING] err: {e}"); time.sleep(0.1)


# ====================== Flask 双摄推流 ======================
app = Flask(__name__)

def gen_mjpeg(cam_id):
    global video_enabled
    cap = None
    errs = 0
    while True:
        # v3.2: 视频被$VID关闭时暂停采集(省CPU/带宽), 保持连接等待重开
        if not video_enabled:
            time.sleep(0.5)
            continue
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            cap = cv2.VideoCapture(cam_id)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
        ok, frame = cap.read()
        if not ok:
            errs += 1
            if errs > 5:
                cap.release(); cap = None
                time.sleep(1)
                errs = 0
            continue
        errs = 0
        # v3.2: 强制输出640x480 (部分摄像头驱动忽略cap.set, 需软件resize兜底)
        if frame.shape[1] != CAM_W or frame.shape[0] != CAM_H:
            frame = cv2.resize(frame, (CAM_W, CAM_H))
        _, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")

@app.route("/cam1")
def cam1(): return Response(gen_mjpeg(CAM1_ID), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/cam2")
def cam2(): return Response(gen_mjpeg(CAM2_ID), mimetype="multipart/x-mixed-replace; boundary=frame")

# v3.3: 第三路画面的备选取流口(走的仍是 :5000 mjpeg_bridge)。
#       上位机 CAM3 默认从 :8084/stream 取流(见 protocol.py 的 CAM3_PORT / CAM3_PATH),
#       只有把 CAM3_PORT 改成 5000、CAM3_PATH 改成 "/cam3" 时才会用到下面这个路由。
#       未接入摄像头(CAM3_ID<0)时返回 503, 上位机会持续重试直到摄像头接上。
@app.route("/cam3")
def cam3():
    if CAM3_ID is None or CAM3_ID < 0:
        return "CAM3 未接入 (CAM3_ID<0)", 503
    return Response(gen_mjpeg(CAM3_ID), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index(): return ("<h3>ROV Video: <a href='/cam1'>CAM1</a> | "
                     "<a href='/cam2'>CAM2</a> | <a href='/cam3'>CAM3</a>(预留)</h3>")

# ====================== 入口 ======================
if __name__ == "__main__":
    print("="*50)
    print("  RDK X5 ROV Server v2.0")
    print("="*50)
    threading.Thread(target=cmd_relay, daemon=True).start()
    threading.Thread(target=telem_relay, daemon=True).start()
    threading.Thread(target=ping_responder, daemon=True).start()
    print(f"[VIDEO] Flask MJPEG 推流启动 :{VIDEO_PORT}")
    app.run(host="0.0.0.0", port=VIDEO_PORT, threaded=True)
