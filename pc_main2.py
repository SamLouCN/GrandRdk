"""
============================================================
  【运行在: PC (Windows)】  ROV 遥控上位机 v3.8
============================================================
功能:
  1) 工作模式切换: 有线遥控 / 自主航行 (LB 键或界面按钮), 交还自主 (显式退出遥控)
  2) 目标/实际 姿态角、角速度、加速度、深度、高度、线速度显示
  3) 电池电量/电压/电流/SOC/温度/舱内温度显示
  4) 捡球(A)/抛球(B) 功能
  5) 视频录制(X) 双摄像头同时录制 / 数据存储(Y, 遥测写入 CSV)
  6) 网络传输状态(速度/延迟)显示 + 连接状态指示
  7) 12个推进器油门(转速)显示
  8) LED亮度 十字键上下(LED1) 左右(LED2)
  9) 实时曲线图面板 (可勾选任意通道曲线)
  10) 目标深度输入与本地设定 (下发接口预留，尚未接入 S100/STM32)
  11) 遥测参数记录: 三轴角度、角速度、加速度、高度、线速度、12推进器
  12) [v3.2] 终端页面: 监控运行情况(事件流/链路状态/原始帧)
  13) [v3.2] 有线(网线)/无线(LoRa) 链路模式切换:
      有线: 指令+遥测+视频 全走网线 (UDP/HTTP)
      无线: 仅遥控指令走 USB串口->地面LoRa->机载LoRa->STM32 (9600, 5Hz)
  14) [v3.2] 视频传输开关: 本地断流 + $VID 帧通知板端停/启推流
  15) [v3.2] 连接状态指示: 遥测超时判断(有线) / 串口状态(无线)
  16) [v3.3] 新增“可视窗口”页面: CAM1/CAM2 画面从主界面迁入, 并预留
      第三个视频窗口 CAM3 (默认待机, 可手动开启取流 /cam3)
  17) [v3.3.2] “全部”页移除实时曲线: 与“函数图”页重复, 曲线统一在
      “函数图”页查看; “全部”页只留 姿态/电池/高度计/快捷键
  18) [v3.3.3] 卡尔曼融合深度: “全部”页“深度/高度”组新增 Kalman深度 一行,
      由 DepthKalmanFilter 融合 depth(压力) + vz(垂向速度) + alt(高度计, 可选);
      同时进函数图曲线与 CSV 记录。参数见文件头 KF_* 常量
    19) [v3.4] 多模式 UI 对齐《多源控制协议设计.md》§12.4:
      按钮分组(工作模式/遥控交还/安全/作业) + 文案重命名
      (遥控模式→有线遥控、AUV模式→自主航行、解除急停→温启动)
      + 新增「交还自主」+ 模式标签新增「待机」态(_frozen 自记)。
      注意: $CMD.mode 在 S100 侧只认 0/1, 故不设"测试模式"按钮
  20) [v3.5] 画面罗盘: CAM1 画面右上角叠加圆形航向指示器(HeadingCompass)。
      零度朝上、顺时针为正; 可切「绝对/相对」——单击盘面把当前航向定义为 0°
      并转相对模式, 指针随板端 yaw 变化显示相对偏转; 双击/右键切回绝对(跟随真值)。
      角度值来自 $TEL 的 yaw 字段(度)。纯自绘(QPainter), 无外部资源, 见 HeadingCompass。
  21) [v3.5] 录像画质: 原用 cv2.VideoWriter(XVID) 写 AVI, 但 OpenCV 的 FFMPEG 后端
      不支持 XVID, **静默降级为低码率 FMP4**(实测 PSNR 仅 40.5dB); 且 VideoWriter
      无法设置质量。改为 MjpegRecorder: 自己 cv2.imencode(质量可调) + 手写 MJPEG-AVI
      容器(build_v34/mjpeg_avi_writer.py)。质量 90 时 PSNR 45.2dB(+4.7dB), 帧内编码
      无 P 帧拖影。帧率由写死的 15 改为与显示一致的 REC_FPS(30)。见 REC_JPEG_QUALITY。
  22) [v3.6] 板端融合值上屏: 「全部」页"深度/高度"组新增 板端融合深度 / 离底净空 两行,
      数据来自 $TEL 帧尾追加字段(index 41=融合深度 m, 42=离底净空 m), 由板端
      depth_kalman 计算并回传; 板端没跑融合/数据超期时显示 "--"(与"Kalman深度"
      区分: 那是 PC 端本地 DepthKalmanFilter 用原始 depth/vz/alt 算的, 两套滤波器)
"""
#  Qt 平台插件路径修复 
import os as _os, sys
_meipass: str = getattr(sys, '_MEIPASS', '')
if getattr(sys, 'frozen', False) and _meipass:
    _os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _os.path.join(_meipass, 'PyQt5', 'Qt5', 'plugins', 'platforms')
    _os.environ['PATH'] = _os.path.join(_meipass, 'PyQt5', 'Qt5', 'bin') + _os.pathsep + _os.environ.get('PATH', '')
else:
    try:
        import PyQt5 as _pyqt5_pkg
        _qt5_root = _os.path.join(_os.path.dirname(_pyqt5_pkg.__file__), 'Qt5')
        _os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _os.path.join(_qt5_root, 'plugins', 'platforms')
        _os.environ['PATH'] = _os.path.join(_qt5_root, 'bin') + _os.pathsep + _os.environ.get('PATH', '')
    except Exception:
        pass
# 
import time, socket, csv, threading, pathlib, math, struct
from datetime import datetime
from collections import deque
from PyQt5 import QtCore
from PyQt5.QtCore import (QTimer, pyqtSignal, QThread, Qt, QRect, QSize,
                          QPoint, QPointF, QRectF)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, QGroupBox, QBoxLayout,
    QStatusBar, QLineEdit, QPushButton, QProgressBar, QFrame,
    QTextEdit, QSplitter, QCheckBox, QScrollArea, QTabWidget, QDoubleSpinBox, QComboBox,
    QSizePolicy
)
from PyQt5.QtGui import (QImage, QPixmap, QFont, QPainter, QPen, QBrush, QColor,
                         QPainterPath, QPolygonF, QFontMetrics)
import pygame, cv2, numpy as np
from navigation import PositionTracker, PositionMap
from pid_panel import ControlPidPanel
from task_pid_wire import build_task_pid, parse_task_pid_ack
import uuid

# v3.2: pyserial 用于无线模式(地面LoRa USB串口), 缺失时无线功能不可用
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

# v3.2.2: 视频显示帧率上限(30~60)。板端采集/推流可达约 200fps，
# 上位机只按该帧率渲染；超出的帧读到即丢(不进缩放/绘制/Qt 信号)，
# 避免 Qt 事件队列被 200Hz 淹没。改这一行即可调档。
# v3.4.1: 默认由 60 降到 30 —— 渲染压力减半, 给"清晰度"让路:
#   缩放/上屏是主线程最重的活, 30fps 下 CPU 占用与掉帧明显更少,
#   配合「1:1 原始尺寸」显示观感更稳更清。要回到旧手感改回 60 即可。
VID_DISPLAY_FPS = 30
VID_DISPLAY_FPS_MIN = 15
VID_DISPLAY_FPS_MAX = 60

# v3.4.1: 自适应缩放模式下的插值方式。
#   True  = 邻域/锐利放大(Qt.FastTransformation) —— 保留像素边界, 检测框/边缘更清楚(当前默认)
#   False = 平滑插值(Qt.SmoothTransformation)  —— 观感柔和, 但低分辨率放大更糊
VID_SCALE_SHARP = True

# ---- v3.5: 录像参数 (改这两个数即可调录像画质/体积) ----
# 录像帧率: 与被显示/推流的帧率保持一致(原来是写死的 15, 播放比实际快一倍)。
REC_FPS = 30
# 录像 JPEG 质量 (1~100)。质量越高越清晰、文件越大。参考(640×480 单帧/30fps):
#   75 → ~10 KB/帧(≈2.6Mbps) | 90 → ~24 KB/帧(≈5.9Mbps, 默认) | 95 → ~37 KB/帧(≈9.2Mbps)
# 录像用 MJPEG 帧内编码: 每帧独立(无 P 帧拖影), 水下快速运动/旋转时观感好于逐帧间压缩。
REC_JPEG_QUALITY = 90
# 单段录像文件上限 (GB): 手写 AVI 为单 RIFF 块, 上限约 1 GB(OpenDML 未实现)。
#   达到上限自动切新文件(rec_camN_<时间戳>_p2.avi …)。90 质量 30fps 约 2.6 GB/h。
REC_MAX_FILE_GB = 1.0

# ---- v3.3.3: 卡尔曼融合深度 (调这 6 个数即可改融合特性) ----
# 过程噪声 Q: 越大越"信观测"(跟得紧但抖); 越小越"信模型"(平滑但滞后)
KF_Q_DEPTH = 0.02      # 深度过程噪声
KF_Q_VEL   = 0.50      # 垂向速度过程噪声
# 观测噪声 R: 越小越信任该传感器
KF_R_DEPTH = 0.05      # 压力深度 (主传感器)
KF_R_VEL   = 0.30      # 遥测垂向速度 vz
KF_R_ALT   = 0.02      # 高度计换算深度 (超声精度高, 噪声给最小)
# 高度计观测是否启用: 需先确认 $TEL 的 alt 与 depth 同单位, 否则改 KF_ALT_SCALE
KF_USE_ALT   = False
KF_ALT_SCALE = 1.0     # alt -> m 的换算系数 (若 alt 单位为 mm 改 0.001)
KF_RESET_GAP = 2.0     # 遥测中断超过该秒数 -> 重置滤波器(避免用陈旧状态预测)

# ==================== 协议导入 ====================
try:
    from protocol import *
    from protocol import (build_cmd, build_emergency_stop, parse_telemetry, build_pid,
                          ReferenceTelemetryBuffer,
                          build_vid, build_estop, build_estop_release,
                          CAM3_PORT, CAM3_PATH,
                          MSG_PORT, parse_msg, parse_auv)   # v3.5: AUV 状态提示 $MSG
except ImportError:
    CMD_PORT = 8080; TELEM_PORT = 8081; VIDEO_PORT = 5000
    CAM3_PORT = 8084; CAM3_PATH = "/stream"
    SEND_HZ = 20; DEADZONE = 0.08; LED_STEP = 5; MAX_LED = 100
    THRUSTER_COUNT = 12
    LORA_BAUD = 9600; LORA_SEND_HZ = 5
    def build_cmd(surge, sway, heave, yaw, led1, led2, mode=0, grab=0, store=0):
        return f"$CMD,{surge:.2f},{sway:.2f},{heave:.2f},{yaw:.2f},{led1},{led2},{mode},{grab},{store}#\r\n"
    def build_emergency_stop():
        return "$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#\r\n"
    def build_vid(on):
        return f"$VID,{1 if on else 0}#\r\n"
    def parse_telemetry(frame):
        frame = frame.strip()
        if not (frame.startswith("$TEL,") and frame.endswith("#")): return None
        try:
            p = frame[5:-1].split(",")
            return {"roll":float(p[0]),"pitch":float(p[1]),"yaw":float(p[2]),"gx":float(p[3]),"gy":float(p[4]),"gz":float(p[5]),"depth":float(p[6]),"vx":float(p[7]) if len(p)>7 else 0,"vy":float(p[8]) if len(p)>8 else 0,"vz":float(p[9]) if len(p)>9 else 0}
        except: return None
    ALT_PORT = 8082
    MSG_PORT = 8085          # v3.5: AUV 状态提示 $MSG(与 $AUV 共用端口, 靠帧头区分)
    def build_alt(ch, mm=None, status="OK"):
        v = "" if mm is None else str(int(mm))
        return f"$ALT,{ch},{v},{status}#\r\n"
    def parse_alt(frame):
        frame = frame.strip()
        if not (frame.startswith("$ALT,") and frame.endswith("#")): return None
        try:
            parts = frame[5:-1].split(",")
            if len(parts) < 3: return None
            raw = parts[1].strip()
            return {"ch": parts[0].strip().upper(), "mm": int(raw) if raw else None,
                    "status": parts[2].strip().upper()}
        except Exception: return None
    def parse_auv(frame):
        frame = frame.strip()
        if not (frame.startswith("$AUV,") and frame.endswith("#")): return None
        try:
            return {"fields": frame[5:-1].split(",")}
        except Exception: return None
    def parse_msg(frame):
        """$MSG,<ts>,<level>,<code>,<stage>,<text># -> dict / None"""
        frame = frame.strip()
        if not (frame.startswith("$MSG,") and frame.endswith("#")): return None
        try:
            p = frame[5:-1].split(",")
            if len(p) < 5: return None      # 去帧头后剩 ts/level/code/stage/text 共 5 段
            return {"ts": p[0].strip(), "level": p[1].strip().upper(),
                    "code": p[2].strip().upper(), "stage": p[3].strip().upper(),
                    "text": ",".join(p[4:]).strip()}
        except Exception: return None

# README §六.1: 兜底网段与 X5_IP 保持一致(S100 eth1 网线直连), 不再回落旧网段
# v3.3.2: “全部”页文字统一字号 —— 用 pt 而非 px, 随系统 DPI 缩放,
# 与全局字体 (Microsoft YaHei 10pt) 及姿态/电池等区(继承默认字体)保持一致
FS_HEAD = "10pt"    # 小标题 / 通道名
FS_TEXT = "10pt"    # 普通文本(状态)
FS_VAL = "11pt"     # 数值(略大, 便于读数)
STY_HEAD = f"font-weight:bold; color:#555; font-size:{FS_HEAD};"
STY_TEXT = f"font-size:{FS_TEXT};"
STY_VAL = f"font-weight:bold; font-size:{FS_VAL};"

target_ip = X5_IP if 'X5_IP' in dir() else "192.168.127.10"

# ==================== 当前手柄轴映射 ====================
# 基于当前控制器实测:
#   axis0 = 左摇杆X
#   axis1 = 左摇杆Y
#   axis2 = 右摇杆X
#   axis3 = 右摇杆Y
#   axis4 = LT
#   axis5 = RT
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 2
AXIS_RY = 3
AXIS_LT = 4
AXIS_RT = 5
YAW_AXIS_SIGN = -1.0  # v3.6: 修正当前实机右摇杆左右绕轴方向颠倒

def dz(v):
    """摇杆死区过滤"""
    return 0.0 if abs(v) < DEADZONE else round(v, 2)


def joystick_yaw(raw_x):
    """当前手柄映射修正；显示与实际 $CMD 都使用该结果。"""
    return round(YAW_AXIS_SIGN * dz(raw_x), 3) or 0.0

# ==================== 手柄线程 ====================
class JoystickThread(QThread):
    """
    手柄按键映射 (Xbox布局):
    左摇杆Y(axis1)   Surge前后     左摇杆X(axis0)   Sway左右
    右摇杆X(axis2)   Yaw偏航       右摇杆Y(axis3)   Heave浮沉
    RT(axis5)        待定(预留)     LT(axis4)        待定(预留)
      十字键 上/下      LED1 +/-     十字键 左/右      LED2 +/-
      A(btn0)  捡球   B(btn1)  抛球
      X(btn2)  录制   Y(btn3)  存储
      LB(btn4) 切模式  RB(btn5) (预留)
    """
    data_sig     = pyqtSignal(dict)
    status_sig   = pyqtSignal(str)
    btn_sig      = pyqtSignal(str)
    net_tx_sig   = pyqtSignal(int, int)
    net_stat_sig = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._running = True
        self._led1 = 0
        self._led2 = 0
        self._mode = 0        # 0=ROV, 1=AUV
        self._grab = 0        # 0=无, 1=捡球, 2=抛球
        self._store = 0       # 0=无, 1=录制中, 2=存储中
        self._recording = False
        self._storing = False
        self._prev_btns = [False] * 6
        self._prev_hat = (0, 0)
        self._lt_bias = None  # 延迟到 warmup 后采集
        self._rt_bias = None
        self._tx_cnt = 0
        self._tx_bytes = 0
        self._t_stat_last = 0.0
        self._t_joy_scan = 0.0    # 手柄未插入时的重扫节流(0.5s)
        # v3.2: 链路模式 0=有线(UDP) 1=无线(LoRa串口)
        self._link_mode = 0
        self._lora_port = None    # 地面LoRa的COM口名, 如 "COM3"
        self._lora_ser = None     # 已打开的串口对象
        self._lora_retry_after = 0.0  # 串口打开失败后的重试节流时间戳

    def _warmup_and_calibrate(self, joy):
        """【修复核心】连接手柄后先 pump 多轮让轴值稳定，再采集偏置。
        Xbox XInput 的 LT/RT (axis2/5) 静止时 SDL 报 -1.0 或 0.0，
        但 joy.init() 后首帧常为默认 0.0，必须等真实值到达才能正确归一化。"""
        for _ in range(30):          # pump ~30帧，约150ms足够稳定
            pygame.event.pump()
            time.sleep(0.005)
        self._lt_bias = joy.get_axis(AXIS_LT)
        self._rt_bias = joy.get_axis(AXIS_RT)
        self.status_sig.emit(
            f"手柄校准完成: LT_bias={self._lt_bias:.2f}, RT_bias={self._rt_bias:.2f}")

    @staticmethod
    def _trigger_normalize(raw, bias):
        """将扳机原始值归一化到 [0.0, 1.0]。
        扳机从 bias(静止) 向任意方向偏移均视为 '按下'，
        兼容 SDL(-1~+1), XInput(0~+1), 以及部分手柄(0~-1)。"""
        max_dev = max(abs(1.0 - bias), abs(-1.0 - bias))
        if max_dev < 0.01:
            return 0.0
        v = abs(raw - bias) / max_dev
        return max(0.0, min(1.0, v))     # 始终 clamp 到 [0,1]

    # ==================== v3.2: LoRa 串口管理 ====================
    def _ensure_lora(self):
        """确保地面LoRa串口已打开(懒打开, 失败后5秒内不重试避免刷屏)"""
        if self._lora_ser is not None and self._lora_ser.is_open:
            return self._lora_ser
        if serial is None:
            if time.perf_counter() > self._lora_retry_after:
                self.status_sig.emit("[ERR] 未安装pyserial, 无线模式不可用 (pip install pyserial)")
                self._lora_retry_after = time.perf_counter() + 5.0
            return None
        if not self._lora_port:
            if time.perf_counter() > self._lora_retry_after:
                self.status_sig.emit("[WARN] 未选择LoRa串口, 请在顶部COM下拉框选择后刷新")
                self._lora_retry_after = time.perf_counter() + 5.0
            return None
        try:
            self._lora_ser = serial.Serial(self._lora_port, LORA_BAUD, timeout=0.1)
            self.status_sig.emit(f"[LORA] 串口已打开: {self._lora_port} @ {LORA_BAUD}")
            return self._lora_ser
        except Exception as e:
            self.status_sig.emit(f"[ERR] LoRa串口打开失败: {e}")
            self._lora_ser = None
            self._lora_retry_after = time.perf_counter() + 5.0
            return None

    def _close_lora(self):
        """关闭地面LoRa串口"""
        if self._lora_ser is not None:
            try:
                self._lora_ser.close()
            except Exception:
                pass
            self._lora_ser = None
            self.status_sig.emit("[LORA] 串口已关闭")

    def set_link_mode(self, mode):
        """切换链路模式(主线程调用): 0=有线(网线) 1=无线(LoRa)。
        切换前经当前活动链路发送急停, 保证电机停转"""
        if mode == self._link_mode:
            return
        # 先急停(走当前还在用的链路)
        try:
            if self._link_mode == 1:
                if self._lora_ser and self._lora_ser.is_open:
                    self._lora_ser.write(build_emergency_stop().encode())
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(build_emergency_stop().encode(), (target_ip, CMD_PORT))
                s.close()
        except Exception:
            pass
        if mode == 0:
            self._close_lora()
        self._link_mode = mode
        self.status_sig.emit(f"[NET] 链路模式 -> {'无线(LoRa 9600)' if mode else '有线(网线 UDP/HTTP)'}")

    def _lora_emergency_stop(self):
        """无线模式急停(经LoRa串口)"""
        ser = self._lora_ser
        if ser and ser.is_open:
            try:
                ser.write(build_emergency_stop().encode())
            except Exception:
                pass

    def _send_raw_wired(self, text):
        """v3.3: 有线链路直发一帧文本（$ESTOP 系列只走网线；无线 LoRa 直连 STM32，不认该帧）"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(text.encode(), (target_ip, CMD_PORT))
            s.close()
            return True
        except Exception:
            return False

    def send_estop(self):
        """v3.3(决策 B): 显式急停 $ESTOP# —— 板端进 STANDBY 并锁存(抑制 0x09)。无线模式跳过。"""
        if self._link_mode == 1:
            self.status_sig.emit("[!] 无线模式: $ESTOP# 只走网线, 已跳过(仅零杆位 $CMD 生效)")
            return False
        return self._send_raw_wired(build_estop())

    def send_estop_release(self):
        """v3.3(决策 B): 解除急停 $ESTOP,0# —— 现场自行解锁板端的唯一手段。无线模式跳过。"""
        if self._link_mode == 1:
            self.status_sig.emit("[!] 无线模式: $ESTOP,0# 只走网线, 已跳过")
            return False
        return self._send_raw_wired(build_estop_release())

    def run(self):
        pygame.init(); pygame.joystick.init()
        joy = None
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _t_next = time.perf_counter()
        self._t_stat_last = time.perf_counter()

        while self._running:
            # v3.2: 限速周期按链路模式动态取(无线降频, 避免超出LoRa空速)
            dt = 1.0 / (LORA_SEND_HZ if self._link_mode == 1 else SEND_HZ)
            pygame.event.pump()
            if pygame.joystick.get_count() == 0:
                _just_unplugged = joy is not None
                if _just_unplugged:
                    self.status_sig.emit("[!] 手柄断开, 急停已发送")
                    try:
                        if self._link_mode == 1:
                            self._lora_emergency_stop()
                        else:
                            sock.sendto(build_emergency_stop().encode(), (target_ip, CMD_PORT))
                            # v3.3(决策 B): 再补一帧显式急停, 让板端真正进 STANDBY 并锁存
                            self.send_estop()
                    except: pass
                    joy = None
                # README §3.2: 手柄未插入时 $CMD 仍按当前链路频率以零杆位持续下发,
                # LED/模式/抓球/存储 等非杆位字段照常携带(链路与状态不断, 仅杆位无效)
                try:
                    idle = build_cmd(0.0, 0.0, 0.0, 0.0,
                                     self._led1, self._led2,
                                     self._mode, self._grab, self._store)
                    if self._link_mode == 1:
                        ser = self._ensure_lora()
                        if ser:
                            ser.write(idle.encode())
                            self._tx_cnt += 1
                            self._tx_bytes += len(idle)
                    else:
                        sock.sendto(idle.encode(), (target_ip, CMD_PORT))
                        self._tx_cnt += 1
                        self._tx_bytes += len(idle)
                except Exception as e:
                    self.status_sig.emit(f"发送失败: {e}")
                self.data_sig.emit({
                    "surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0,
                    "ry": 0.0, "lt": 0.0, "rt": 0.0,
                    "led1": self._led1, "led2": self._led2,
                    "mode": self._mode, "grab": self._grab, "store": self._store
                })
                # 手柄热插拔重扫按 0.5s 节流(避免高频 quit/init 开销)
                _now = time.perf_counter()
                if _now >= self._t_joy_scan:
                    self._t_joy_scan = _now + 0.5
                    if not _just_unplugged:
                        self.status_sig.emit("等待手柄插入...")
                    pygame.joystick.quit(); pygame.joystick.init()
                _t_next += dt
                _sleep = _t_next - time.perf_counter()
                if _sleep > 0:
                    time.sleep(_sleep)
                else:
                    _t_next = time.perf_counter()
                continue

            if joy is None:
                joy = pygame.joystick.Joystick(0); joy.init()
                self.status_sig.emit(f"手柄连接: {joy.get_name()}, 正在校准...")
                self._warmup_and_calibrate(joy)
                self.status_sig.emit(f"手柄就绪: {joy.get_name()}")
                _t_next = time.perf_counter()

            # --- 读取手柄轴 ---
            sx = dz(joy.get_axis(AXIS_LX))   # 左摇杆X  Sway
            sy = dz(joy.get_axis(AXIS_LY))   # 左摇杆Y  Surge (前为负)
            rx = dz(joy.get_axis(AXIS_RX))   # 右摇杆X  Yaw
            ry = dz(joy.get_axis(AXIS_RY))   # 右摇杆Y  Heave (前为负)

            # --- RT/LT 扳机: 归一化逻辑保留, 用途待分配(RB 同) ---
            lt = self._trigger_normalize(joy.get_axis(AXIS_LT), self._lt_bias)
            rt = self._trigger_normalize(joy.get_axis(AXIS_RT), self._rt_bias)
            surge = round(-sy, 3) or 0.0  # 避免 -0.0
            sway  = round(sx, 3) or 0.0
            heave = round(-ry, 3) or 0.0  # 右摇杆前推为正: 上浮 / 下潜 (README §3.2)
            yaw   = joystick_yaw(rx)

            # --- 按钮 (单次触发 + 连续) ---
            for ev in pygame.event.get(pygame.JOYBUTTONDOWN):
                b = ev.button
                if b == 4:  # LB  模式切换
                    self._mode = 1 - self._mode
                    self.status_sig.emit(f"模式 -> {'自主航行(AUV)' if self._mode else '有线遥控(ROV)'}")
                elif b == 0:  # A  抓球
                    self.btn_sig.emit("grab")
                elif b == 1:  # B  抛球
                    self.btn_sig.emit("throw")
                elif b == 2:  # X  录像
                    self.btn_sig.emit("record")
                elif b == 3:  # Y  数据存储
                    self.btn_sig.emit("store")

            # --- D-pad  两路 LED ---
            hat = joy.get_hat(0) if joy.get_numhats() > 0 else (0, 0)
            hx, hy = hat
            self._led1 = max(0, min(100, self._led1 + hy * 5))   # 协议 LED 范围 0..100
            self._led2 = max(0, min(100, self._led2 + hx * 5))

            # --- v3.3: AUV 模式 —— 上位机不参与运动控制 ---
            # AUV(self._mode == 1): 仅下发模式信号, 4 个运动通道强制 0.00。
            # v3.4 实证补充: S100 侧 mode_auv.on_cmd() 本就会忽略杆位(只记录不动作),
            #   且切模式只看 $CMD.mode 字段值(不看谁发了运动帧), 故此归零并非安全必需;
            #   保留它是为了①UDP 报文干净(抓包一眼可见"上位机没在指挥")
            #   ②将来若 S100 允许 AUV 期间人工干预, 零杆位不会突然变成控制量。
            if self._mode == 1:
                surge = sway = heave = yaw = 0.0

            # --- 发送指令帧 (v3.2: 有线走UDP, 无线走LoRa串口) ---
            frame = build_cmd(surge, sway, heave, yaw,
                              self._led1, self._led2,
                              self._mode, self._grab, self._store)
            try:
                if self._link_mode == 1:
                    # 无线: 地面LoRa USB串口透传
                    ser = self._ensure_lora()
                    if ser:
                        ser.write(frame.encode())
                        self._tx_cnt += 1
                        self._tx_bytes += len(frame)
                else:
                    # 有线: 网线UDP
                    sock.sendto(frame.encode(), (target_ip, CMD_PORT))
                    self._tx_cnt += 1
                    self._tx_bytes += len(frame)
            except Exception as e:
                self.status_sig.emit(f"发送失败: {e}")
                if self._link_mode == 1:
                    self._close_lora()  # 串口异常时关闭, 下轮重开

            # --- 发射摇杆数据供 UI 显示 ---
            self.data_sig.emit({
                "surge": surge, "sway": sway, "heave": heave, "yaw": yaw,
                "ry": ry, "lt": lt, "rt": rt,
                "led1": self._led1, "led2": self._led2,
                "mode": self._mode, "grab": self._grab, "store": self._store
            })

            # --- 每秒统计网络发送 ---
            _now = time.perf_counter()
            if _now - self._t_stat_last >= 1.0:
                self.net_tx_sig.emit(self._tx_cnt, self._tx_bytes)
                self._tx_cnt = 0; self._tx_bytes = 0
                self._t_stat_last = _now

            # --- 限速 ---
            _t_next += dt
            _sleep = _t_next - time.perf_counter()
            if _sleep > 0:
                time.sleep(_sleep)
            else:
                _t_next = time.perf_counter()

        sock.close(); pygame.quit()

    def stop(self):
        self._running = False


# 
#  2. TelemetryThread  接收遥测数据
# 
class TelemetryThread(QThread):
    tel_sig   = pyqtSignal(dict)
    raw_sig   = pyqtSignal(str)        # v3.2: 原始帧文本(终端监视用)
    net_rx_sig = pyqtSignal(int, int)   # pkt_cnt, byte_cnt

    def __init__(self):
        super().__init__()
        self._running = True
        self._rx_cnt = 0; self._rx_bytes = 0
        self._t_stat = time.perf_counter()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", TELEM_PORT))
        sock.settimeout(0.5)
        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                text = data.decode("utf-8", errors="ignore")
                self.raw_sig.emit(text)   # v3.2: 原始帧直接上报(终端按需显示)
                parsed = parse_telemetry(text)
                if parsed:
                    self.tel_sig.emit(parsed)
                    self._rx_cnt += 1
                    self._rx_bytes += len(data)
            except socket.timeout:
                pass
            except Exception:
                pass
            now = time.perf_counter()
            if now - self._t_stat >= 1.0:
                self.net_rx_sig.emit(self._rx_cnt, self._rx_bytes)
                self._rx_cnt = 0; self._rx_bytes = 0
                self._t_stat = now
        sock.close()

    def stop(self):
        self._running = False


# 
#  2.4 MjpegAviWriter  手写 MJPEG-AVI 容器 (v3.5, 从 build_v34/mjpeg_avi_writer.py 内联)
# 
class MjpegAviWriter:
    """手写 MJPEG-AVI。append(jpeg_bytes) 逐帧写，close() 回填尺寸与索引。"""

    def __init__(self, path, width, height, fps=30.0):
        self.path = str(path)
        self.w = int(width)
        self.h = int(height)
        self.fps = float(fps) if fps and fps > 0 else 30.0
        self._frames = []          # (offset_in_file, jpeg_size)
        self._max_jpeg = 0
        self._closed = False
        self._f = open(self.path, "wb")
        # 先写头 + movi 头（占位），帧数据紧随其后
        hdrl = self._build_hdrl()
        self._f.write(b"RIFF")
        self._f.write(struct.pack("<I", 0))        # RIFF size（回填）
        self._f.write(b"AVI ")
        self._f.write(hdrl)
        self._movi_list_pos = self._f.tell()       # 'LIST' 的位置
        self._f.write(b"LIST")
        self._f.write(struct.pack("<I", 0))        # movi LIST size（回填）
        self._f.write(b"movi")
        self._movi_start = self._f.tell()          # 帧数据起点（含 4 字节 'movi' 之后）

    # ---------- 头部构造 ----------
    def _build_hdrl(self):
        rate = 1000
        scale = max(1, int(round(self.fps * 1000)))
        # avih：dwMicroSecPerFrame, dwMaxBytesPerSec, pad, flags, totalFrames, initial, streams,
        #       suggestedBuf, width, height, reserved[4]
        avih = struct.pack(
            "<IIIIIIIIIIIIII",
            int(round(1_000_000.0 / self.fps)),  # dwMicroSecPerFrame
            0,                                   # dwMaxBytesPerSec
            0,                                   # dwPaddingGranularity
            0x00000110,                          # AVIF_HASINDEX(0x10) | AVIF_TRUSTCKTYPE(0x100)
            0,                                   # dwTotalFrames（回填）
            0,                                   # dwInitialFrames
            1,                                   # dwStreams
            0,                                   # dwSuggestedBufferSize
            self.w, self.h, 0, 0, 0, 0)
        avih_chunk = b"avih" + struct.pack("<I", len(avih)) + avih

        # strh：fccType, fccHandler, dwFlags, wPriority, wLanguage, dwInitialFrames,
        #       dwScale, dwRate, dwStart, dwLength, dwSuggestedBufferSize, dwQuality,
        #       dwSampleSize, rcFrame(left,top,right,bottom)
        strh = (b"vids" + b"MJPG" +
                struct.pack("<I", 0) +                 # dwFlags
                struct.pack("<H", 0) +                 # wPriority
                struct.pack("<H", 0) +                 # wLanguage
                struct.pack("<I", 0) +                 # dwInitialFrames
                struct.pack("<I", rate) +              # dwScale
                struct.pack("<I", scale) +             # dwRate
                struct.pack("<I", 0) +                 # dwStart
                struct.pack("<I", 0) +                 # dwLength（回填 = 帧数）
                struct.pack("<I", 0) +                 # dwSuggestedBufferSize
                struct.pack("<I", 0xFFFFFFFF) +        # dwQuality = -1（默认，用编码器自带）
                struct.pack("<I", 0) +                 # dwSampleSize
                struct.pack("<hhhh", 0, 0, self.w, self.h))
        strh_chunk = b"strh" + struct.pack("<I", len(strh)) + strh

        # strf：BITMAPINFOHEADER（biCompression 用 MJPG，OpenCV/VLC 都认；用 'MJPG' 更稳）
        strf = struct.pack("<IiiHHIIiiII",
                           40, self.w, self.h, 1, 24,
                           0x47504A4D,                 # biCompression = 'MJPG' (little-endian)
                           0, 0, 0, 0, 0)
        strf_chunk = b"strf" + struct.pack("<I", len(strf)) + strf

        strl_body = strh_chunk + strf_chunk
        strl = b"LIST" + struct.pack("<I", 4 + len(strl_body)) + b"strl" + strl_body
        hdrl_body = avih_chunk + strl
        return b"LIST" + struct.pack("<I", 4 + len(hdrl_body)) + b"hdrl" + hdrl_body

    # ---------- 写帧 ----------
    def append(self, jpeg_bytes):
        """追加一帧（已编码的 JPEG 字节）。"""
        if self._closed:
            raise RuntimeError("writer 已关闭")
        size = len(jpeg_bytes)
        off = self._f.tell()
        # '00dc' 帧块；奇数长度补 1 字节 padding
        self._f.write(b"00dc")
        self._f.write(struct.pack("<I", size))
        self._f.write(jpeg_bytes)
        if size & 1:
            self._f.write(b"\x00")
        self._frames.append((off, size))
        if size > self._max_jpeg:
            self._max_jpeg = size

    # ---------- 收尾 ----------
    def close(self):
        if self._closed:
            return self._file_size_guard()
        self._closed = True
        f = self._f
        frame_count = len(self._frames)

        # 1) 索引 idx1：每条 16 字节 (ckid, flags, offset, size)
        idx_entries = b"".join(
            b"00dc" + struct.pack("<III", 0x10, off, size)
            for (off, size) in self._frames)
        f.write(b"idx1")
        f.write(struct.pack("<I", len(idx_entries)))
        f.write(idx_entries)

        file_end = f.tell()
        movi_list_size = (file_end - 8 - len(idx_entries) - 8) - self._movi_list_pos + 8
        # 精确算: movi LIST 的 size 字段 = 其内容字节数 = ('movi' 4B) + 所有帧块
        frames_bytes = sum(8 + size + (size & 1) for (_, size) in self._frames)
        movi_list_size = 4 + frames_bytes

        f.seek(0, 2)
        total = f.tell()
        # 2) 回填 movi LIST size
        f.seek(self._movi_list_pos + 4)
        f.write(struct.pack("<I", movi_list_size))

        # 3) 回填 avih.dwTotalFrames（avih 在 RIFF(12) + 'LIST'+size+'hdrl'(12) + 'avih'+size(8) + 8）
        avih_data_off = 12 + 12 + 8
        f.seek(avih_data_off + 4 * 4)          # 跳到 dwTotalFrames 字段
        f.write(struct.pack("<I", frame_count))

        # 4) 回填 strh.dwLength / dwSuggestedBufferSize
        #    strh 数据起点 = RIFF(12) + hdrl LIST(12) + avih chunk(8+56) + strl LIST(12) + 'strh'+size(8)
        strh_data_off = 12 + 12 + (8 + 56) + 12 + 8
        _strh_prefix = 4 + 4 + 4 + 2 + 2 + 4 + 4 + 4 + 4   # 到 dwLength 之前的字段总长
        f.seek(strh_data_off + _strh_prefix)
        f.write(struct.pack("<I", frame_count))            # dwLength
        # dwSuggestedBufferSize = 最大帧长
        f.write(struct.pack("<I", self._max_jpeg))

        # 5) 回填 RIFF size
        f.seek(4)
        f.write(struct.pack("<I", total - 8))
        f.flush()
        f.close()
        return self._file_size_guard(total)

    @staticmethod
    def _file_size_guard(total=None):
        # 单 RIFF 块上限 ~1GB（未实现 OpenDML）；超限由 MjpegRecorder 负责分段
        return {"bytes": total, "over_1gb": bool(total and total > int(1.0 * (1024 ** 3)))}

    @property
    def frame_count(self):
        return len(self._frames)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


# 
#  2.5 MjpegRecorder  录像写入器 (v3.5)
# 
class MjpegRecorder:
    """录像写入器：**自己控制 JPEG 质量** + 手写 MJPEG-AVI 容器。

    为什么不用 cv2.VideoWriter：
      · OpenCV 的 VideoWriter 请求 `XVID` 时, 在 FFMPEG 后端下**静默降级为 FMP4**
        (mpeg4 默认参数, 实测 PSNR 仅 ~40.5dB), 且 VideoWriter **没有任何接口能设码率/质量**。
      · 请求 `MJPG` 虽可用(帧内 JPEG), 但质量写死约 q75(实测 ~10KB/帧), 同样调不了。
    因此改为：`cv2.imencode(".jpg", frame, [IMWRITE_JPEG_QUALITY, q])` 自己编码, 再用
    `MjpegAviWriter`(见 build_v34/mjpeg_avi_writer.py) 按 AVI 容器封装 —— 质量完全可控。

    接口与 cv2.VideoWriter 对齐(write/release/isOpened), 便于直接替换。
    达到单文件上限(REC_MAX_FILE_GB)自动切新文件, 通过 fname 属性暴露当前文件名。
    """

    def __init__(self, dir_path, stem, width, height, fps=REC_FPS,
                 quality=REC_JPEG_QUALITY, max_bytes=None):
        self._dir = pathlib.Path(dir_path)
        self._stem = stem                  # 例: "rec_cam1_20261004_180000"
        self.w = int(width)
        self.h = int(height)
        self.fps = int(fps) if fps else REC_FPS
        self.quality = int(quality)
        self._max_bytes = int(max_bytes if max_bytes is not None
                              else REC_MAX_FILE_GB * (1024 ** 3))
        self._part = 1
        self.fname = None                  # 当前段文件名(供日志/UI)
        self.bytes_written = 0             # 本次录像累计字节
        self._w = None
        self._ok = False
        self._open_part()

    # ---- 内部 ----
    def _open_part(self):
        name = f"{self._stem}.avi" if self._part == 1 else f"{self._stem}_p{self._part}.avi"
        self.fname = self._dir / name
        self._w = MjpegAviWriter(self.fname, self.w, self.h, self.fps)
        self._ok = True

    def _rotate(self):
        """切下一段(单文件达上限)。返回是否成功。"""
        try:
            self._w.close()
        except Exception:
            pass
        self._part += 1
        self._open_part()

    # ---- 对外(cv2.VideoWriter 对齐) ----
    def isOpened(self):
        return bool(self._ok)

    def write(self, frame):
        """写入一帧(BGR ndarray)。尺寸不符则拒绝(避免容器头与实际帧不一致)。"""
        if self._w is None:
            return
        try:
            fh, fw = frame.shape[:2]
            if fw != self.w or fh != self.h:
                return
            ok, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                return
            data = buf.tobytes()
            # 达上限先切段(留出本次写入余量), 保证单文件不超过上限
            if self.bytes_written and \
                    self.bytes_written + len(data) > self._max_bytes:
                self._rotate()
                ok, buf = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                if not ok:
                    return
                data = buf.tobytes()
            self._w.append(data)
            self.bytes_written += len(data)
        except Exception:
            # 录像失败不能拖垮 UI/收流线程
            pass

    def release(self):
        if self._w is not None:
            try:
                self._w.close()
            except Exception:
                pass
            self._w = None
        self._ok = False


# 
#  3. VideoThread  MJPEG 视频流接收
# 
class VideoThread(QThread):
    frame_sig = pyqtSignal(np.ndarray)
    fps_sig   = pyqtSignal(str, int)   # v3.2: (cam名, fps) 供终端监控

    def __init__(self, url, cam_name="cam", display_fps=None):
        super().__init__()
        self._url = url
        self._cam_name = cam_name
        self._running = True
        # v3.2.2: 显示帧率上限(30~60)。板端可能以 ~200fps 推 MJPEG，
        # 本线程照收(必须读走 socket，否则 TCP 反压)，但只按 display_fps 出图。
        df = VID_DISPLAY_FPS if display_fps is None else display_fps
        self.display_fps = max(VID_DISPLAY_FPS_MIN, min(VID_DISPLAY_FPS_MAX, int(df)))
        self._min_dt = 1.0 / float(self.display_fps)
        self.recv_fps = 0          # v3.2.2: 收到的推流帧率(诊断用)

    def run(self):
        while self._running:
            cap = cv2.VideoCapture(self._url)
            if not cap.isOpened():
                time.sleep(1); continue
            # v3.2.2: 双计数 —— recv=收到(推流)帧, show=实际渲染帧
            recv_cnt = 0; show_cnt = 0
            last_show = 0.0
            fps_t0 = time.perf_counter()
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    break
                recv_cnt += 1
                now = time.perf_counter()
                # 限帧: 距上次出图不足 1/display_fps 的帧直接丢弃(只保留最新帧)
                if now - last_show >= self._min_dt:
                    last_show = now
                    self.frame_sig.emit(frame)
                    show_cnt += 1
                if now - fps_t0 >= 2.0:
                    dt = now - fps_t0
                    self.recv_fps = int(recv_cnt / dt)
                    self.fps_sig.emit(self._cam_name, int(show_cnt / dt))
                    recv_cnt = 0; show_cnt = 0; fps_t0 = now
            cap.release()
            time.sleep(0.5)

    def stop(self):
        self._running = False


# 
#  3.5 AltThread 高度计(CH348) UDP 接收  v3.2.1
# 
class AltThread(QThread):
    """接收板端 read_altimeter.py 推送的 $ALT 帧 (UDP ALT_PORT)"""
    alt_sig = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", ALT_PORT))
        except Exception:
            sock.close()
            return
        sock.settimeout(0.5)
        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                d = parse_alt(data.decode("utf-8", errors="ignore"))
                if d:
                    self.alt_sig.emit(d)
            except socket.timeout:
                pass
            except Exception:
                pass
        sock.close()

    def stop(self):
        self._running = False


# 
#  3.5b MsgThread  AUV 状态提示(UDP MSG_PORT)  v3.5
# 
class MsgThread(QThread):
    """接收板端 auv_task/notify.py 推送的 $MSG 帧 (UDP MSG_PORT=8085)

    ★ 存在的意义: 水池测试时人在岸上, 板子上只有日志。这个线程把"正在跑哪个阶段、
      有没有降级、为什么超时/中止"直接顶到上位机终端页, 一抬眼就能看见。

    ⚠ 与 $AUV(数值快照) 共用 8085, 靠帧头区分: parse_msg 认 $MSG, parse_auv 认 $AUV,
      两者都返回 None 的帧(其它帧)直接忽略 —— 老协议完全不受影响。
    ⚠ 无缆验收时板端不发(上位机地址学不到), 收不到属正常, 不是故障。
    """
    msg_sig = pyqtSignal(dict)      # 一条 $MSG 提示
    raw_sig = pyqtSignal(str)       # 非 $MSG/$AUV 的其它文本(调试用, 目前不下发)

    def __init__(self):
        super().__init__()
        self._running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", MSG_PORT))
        except Exception:
            sock.close()
            return
        sock.settimeout(0.5)
        while self._running:
            try:
                data, _ = sock.recvfrom(4096)
                frame = data.decode("utf-8", errors="ignore")
                d = parse_msg(frame)
                if d:
                    self.msg_sig.emit(d)
                    continue
                if parse_auv(frame):
                    continue                      # $AUV 数值快照: 认得但暂不显示
                self.raw_sig.emit(frame.strip())  # 未知文本帧: 交给终端(可关)
            except socket.timeout:
                pass
            except Exception:
                pass
        sock.close()

    def stop(self):
        self._running = False


# 
#  3.6 DepthKalmanFilter  深度卡尔曼融合  v3.3.3
# 
class DepthKalmanFilter:
    """深度卡尔曼融合: 状态 [深度 depth, 垂向速度 vz]

    预测(匀速模型):  depth += vz*dt;  vz 随机游走
    观测(逐个标量更新):
      ① 压力深度 depth           —— 绝对值可靠但噪声大/有滞后
      ② 遥测垂向速度 vz          —— 提供变化率, 让估计跟得上运动
      ③ 高度计换算深度 H - alt   —— 可选(见 KF_USE_ALT), 超声精度高

    输出融合深度 = 状态 depth, 比原始 depth 更平滑、滞后更小。
    纯 Python 实现(2x2 矩阵手工展开), 不依赖 numpy。
    """
    def __init__(self):
        self._x = 0.0            # 状态: 深度
        self._v = 0.0            # 状态: 垂向速度
        self._p = [1.0, 0.0, 0.0, 1.0]   # 协方差 2x2 展开 [p00,p01,p10,p11]
        self._t = None           # 上次更新时间(perf_counter)
        self._ready = False
        self._water = None       # 水深估计 H = depth + alt (用于 alt -> depth)
        self._wbuf = []

    # ---- 对外 ----
    def reset(self):
        self._x = 0.0; self._v = 0.0
        self._p = [1.0, 0.0, 0.0, 1.0]
        self._t = None; self._ready = False
        self._water = None; self._wbuf = []

    @property
    def ready(self):
        return self._ready

    @property
    def depth(self):
        return self._x

    @property
    def vel(self):
        return self._v

    def update(self, depth=None, vz=None, alt=None, now=None):
        """喂一帧遥测; 返回融合深度(float) 或 None(还没收到过 depth)"""
        if depth is None:
            return self._x if self._ready else None
        now = time.perf_counter() if now is None else now
        # 首帧 / 断流恢复: 直接用观测值初始化, 不做预测
        if self._t is None or (now - self._t) > KF_RESET_GAP:
            self._x = float(depth)
            self._v = float(vz) if vz is not None else 0.0
            self._p = [1.0, 0.0, 0.0, 1.0]
            self._t = now
            self._ready = True
            self._track_water(depth, alt)
            return self._x
        dt = min(max(now - self._t, 0.001), 0.5)   # 夹到 [1ms, 0.5s]
        self._t = now
        self._predict(dt)
        self._obs(depth, 1.0, 0.0, KF_R_DEPTH)
        if vz is not None:
            self._obs(vz, 0.0, 1.0, KF_R_VEL)
        if KF_USE_ALT and alt is not None:
            self._track_water(depth, alt)
            if self._water is not None:
                self._obs(self._water - alt * KF_ALT_SCALE, 1.0, 0.0, KF_R_ALT)
        return self._x

    # ---- 内部 ----
    def _predict(self, dt):
        """x = F x;  P = F P F^T + Q   (F = [[1,dt],[0,1]])"""
        self._x += self._v * dt
        p00, p01, p10, p11 = self._p
        q00 = KF_Q_DEPTH * dt
        q11 = KF_Q_VEL * dt
        self._p = [p00 + dt * (p10 + p01) + dt * dt * p11 + q00,
                   p01 + dt * p11,
                   p10 + dt * p11,
                   p11 + q11]

    def _obs(self, z, h0, h1, r):
        """标量观测更新: y = z - Hx;  K = P H^T / S;  x += K y;  P = (I-KH) P"""
        p00, p01, p10, p11 = self._p
        s = h0 * (p00 * h0 + p01 * h1) + h1 * (p10 * h0 + p11 * h1) + r
        if s <= 1e-12:
            return
        k0 = (p00 * h0 + p01 * h1) / s
        k1 = (p10 * h0 + p11 * h1) / s
        y = z - (h0 * self._x + h1 * self._v)
        self._x += k0 * y
        self._v += k1 * y
        p00, p01, p10, p11 = self._p
        a00, a01 = 1.0 - k0 * h0, -k0 * h1
        a10, a11 = -k1 * h0, 1.0 - k1 * h1
        self._p = [a00 * p00 + a01 * p10, a00 * p01 + a01 * p11,
                   a10 * p00 + a11 * p10, a10 * p01 + a11 * p11]

    def _track_water(self, depth, alt):
        """水深 H = depth + alt: 前若干帧取中位数定初值, 之后慢速跟踪"""
        if alt is None:
            return
        h = float(depth) + float(alt) * KF_ALT_SCALE
        if self._water is None:
            self._wbuf.append(h)
            if len(self._wbuf) >= 20:
                self._wbuf.sort()
                self._water = self._wbuf[len(self._wbuf) // 2]
                self._wbuf = []
        else:
            self._water += 0.01 * (h - self._water)   # 慢速跟踪, 抗突变


# 
#  4. ThrusterBar  单个推进器油门条
# 
class ThrusterBar(QWidget):
    """垂直进度条 + 百分比标签，显示单个推进器油门"""
    def __init__(self, label="T1", parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        self._bar = QProgressBar()
        self._bar.setOrientation(Qt.Vertical)
        self._bar.setRange(-100, 100)
        self._bar.setValue(0)
        self._bar.setTextVisible(False)
        self._bar.setFixedWidth(22)
        self._lbl = QLabel("0%")
        self._lbl.setAlignment(Qt.AlignCenter)
        self._lbl.setStyleSheet("font-size:12px;")
        self._title = QLabel(label)
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setStyleSheet("font-size:12px; font-weight:bold;")
        lay.addWidget(self._title)
        lay.addWidget(self._bar, 1)
        lay.addWidget(self._lbl)

    def set_value(self, v):
        """v: -100~100 整数"""
        v = max(-100, min(100, int(v)))
        self._bar.setValue(v)
        self._lbl.setText(f"{v}%")
        if v > 10:
            self._bar.setStyleSheet("QProgressBar::chunk{background:#4caf50;}")
        elif v < -10:
            self._bar.setStyleSheet("QProgressBar::chunk{background:#f44336;}")
        else:
            self._bar.setStyleSheet("QProgressBar::chunk{background:#9e9e9e;}")


# ==================== 左摇杆 2D 可视化 ====================
class StickWidget(QWidget):
    """正方形 2D 十字线 + 圆点，表示左摇杆 XY"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self._x = 0.0   # [-1, 1]
        self._y = 0.0   # [-1, 1]

    def set_pos(self, x, y):
        self._x = max(-1.0, min(1.0, x))
        self._y = max(-1.0, min(1.0, y))
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        s = min(w, h) - 4
        ox, oy = (w - s) // 2, (h - s) // 2

        # 背景圆
        p.setPen(QPen(QColor("#999"), 1))
        p.setBrush(QBrush(QColor("#2a2a2a")))
        p.drawEllipse(ox, oy, s, s)

        # 十字线
        cx, cy = ox + s // 2, oy + s // 2
        p.setPen(QPen(QColor("#555"), 1, Qt.DashLine))
        p.drawLine(cx, oy, cx, oy + s)
        p.drawLine(ox, cy, ox + s, cy)

        # 摇杆点
        px = cx + int(self._x * s * 0.45)
        py = cy + int(self._y * s * 0.45)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#00e676")))
        p.drawEllipse(px - 7, py - 7, 14, 14)

        # 标注
        p.setPen(QColor("#aaa"))
        f = QFont("Consolas", 10)
        p.setFont(f)
        p.drawText(ox + 2, oy + 12, "Sway→")
        p.drawText(ox + 2, oy + s - 4, "Surge↑")
        p.end()


# ==================== 右摇杆横条 (Yaw / Heave / 扳机共用) ====================
class YawBar(QWidget):
    """水平横条: 中间为零, 左右表示 [-1, 1]。label 决定条内文字(同一控件被多条复用)"""
    def __init__(self, label="Yaw", parent=None):
        super().__init__(parent)
        self._label = label
        self.setMinimumSize(120, 28)
        self.setMaximumHeight(32)
        self._val = 0.0

    def set_value(self, v):
        self._val = max(-1.0, min(1.0, v))
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        margin = 4
        bar_h = h - margin * 2
        bar_w = w - margin * 2
        cx = w // 2

        # 背景槽
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#2a2a2a")))
        p.drawRoundedRect(margin, margin, bar_w, bar_h, 4, 4)

        # 填充
        fill_w = int(abs(self._val) * bar_w * 0.5)
        if self._val > 0.01:
            p.setBrush(QBrush(QColor("#29b6f6")))
            p.drawRoundedRect(cx, margin, fill_w, bar_h, 3, 3)
        elif self._val < -0.01:
            p.setBrush(QBrush(QColor("#ffa726")))
            p.drawRoundedRect(cx - fill_w, margin, fill_w, bar_h, 3, 3)

        # 中线
        p.setPen(QPen(QColor("#fff"), 2))
        p.drawLine(cx, margin, cx, h - margin)

        # 数值
        p.setPen(QColor("#eee"))
        p.setFont(QFont("Consolas", 11, QFont.Bold))
        txt = f"{self._label} {self._val:+.2f}"
        p.drawText(margin + 4, h - margin - 2, txt)
        p.end()


# ==================== 实时曲线绘制控件(自绘, 无第三方依赖) ====================
class CurvePlot(QWidget):
    """QPainter 自绘实时曲线。每通道一个环形缓冲(deque), 最新样本靠右。"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 120)
        self._maxlen = 300
        self._series = {}          # key -> {"name","color","show","data"}

    def add_channel(self, key, name, color, show=True, style=Qt.SolidLine):
        self._series[key] = {"name": name, "color": color, "show": show,
                             "style": style, "data": deque(maxlen=self._maxlen)}
        self.update()

    def set_channel_show(self, key, show):
        if key in self._series:
            self._series[key]["show"] = show
        self.update()

    def feed(self, ch):
        """ch: dict {channel_key:value}; 每个已注册通道取一个样本, 缺失补None"""
        for key, s in self._series.items():
            s["data"].append(ch.get(key))
        self.update()

    def clear(self):
        for s in self._series.values():
            s["data"].clear()
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        if w < 60 or h < 60:
            p.end(); return
        p.fillRect(self.rect(), QColor("#101820"))
        # ============ 自适应布局: 先算 y 范围/标签/图例, 再定刻度区与绘图区 ============
        # ---- 收集可见通道数据 -> y 范围自适应 ----
        yvals = []
        for s in self._series.values():
            if s["show"]:
                for v in s["data"]:
                    if isinstance(v, (int, float)) and math.isfinite(v):
                        yvals.append(v)
        if yvals:
            ymin, ymax = min(yvals), max(yvals)
            if ymax - ymin < 1e-6:
                ymin -= 1; ymax += 1
            _span = ymax - ymin
            _pad = _span * 0.15; ymin -= _pad; ymax += _pad
        else:
            ymin, ymax = -1.0, 1.0
        _span = ymax - ymin
        # ---- 刻度标签格式自适应: 大跨度/大量级去小数位, 保证纵轴数字完整 ----
        if _span >= 1000 or max(abs(ymin), abs(ymax)) >= 1000:
            _fmt = "{:.0f}"
        elif _span >= 10:
            _fmt = "{:.1f}"
        else:
            _fmt = "{:.2f}"
        _ticks = [ymin + _i * _span / 5 for _i in range(6)]
        _labels = [_fmt.format(_v) for _v in _ticks]
        # ---- 字体与图例预估(粗体) ----
        p.setFont(QFont("Consolas", 11, QFont.Bold))
        _lfm = p.fontMetrics()
        _names = [s for s in self._series.values() if s["show"]]
        _rows = 1; _x = 8
        for s in _names:
            _wd = _lfm.width(s["name"]) + 24
            if _x + _wd > self.width() - 12 and _x > 8:
                _rows += 1; _x = 8
            _x += _wd + 14
        # ---- 左侧刻度区宽度自适应(保证最长标签完整显示) ----
        p.setFont(QFont("Consolas", 11))
        _fm = p.fontMetrics()
        _lw = max([_fm.width(_t) for _t in _labels] or [0])
        m = max(56, _lw + 14)
        # ---- 顶部图例带高度自适应(条目多自动换行, 不截断表头) ----
        _rowh = max(16, _lfm.height() + 6)
        top = max(40, 8 + _rows * _rowh + 6)
        top = min(top, self.height() - 60)
        bottom = self.height() - 14
        plot_w = self.width() - m - 10
        plot_h = bottom - top
        if plot_w < 40 or plot_h < 40:
            p.end(); return
        def ypix(v):
            return int(bottom - (v - ymin) / _span * plot_h)
        # ---- 网格 + y 刻度(自适应 m / 格式) ----
        for _i in range(6):
            _yy = ypix(_ticks[_i])
            p.setPen(QPen(QColor("#263238"), 1))
            p.drawLine(m, _yy, self.width() - 8, _yy)
            p.setPen(QColor("#80cbc4"))
            _lh = max(16, _fm.height() + 6)
            # TextDontClip: 禁止矩形裁剪, 数字上下永不切半
            p.drawText(QRect(0, _yy - _lh // 2, m - 6, _lh),
                       Qt.AlignRight | Qt.AlignVCenter | Qt.TextDontClip, _labels[_i])
        # ---- 绘图区边框 ----
        p.setPen(QPen(QColor("#37474f"), 1))
        p.drawLine(m, top, self.width() - 8, top)
        p.drawLine(m, bottom, self.width() - 8, bottom)
        # ---- 曲线 ----
        n = self._maxlen
        for s in self._series.values():
            if not s["show"]:
                continue
            data = list(s["data"])
            path = QPainterPath()
            connected = False
            for _i, v in enumerate(data):
                if isinstance(v, (int, float)) and math.isfinite(v):
                    _x2 = m + (n - len(data) + _i) / max(1, n - 1) * plot_w
                    if connected:
                        path.lineTo(_x2, ypix(v))
                    else:
                        path.moveTo(_x2, ypix(v))
                        p.setPen(QPen(QColor(s["color"]), 2))
                        p.drawPoint(QPointF(_x2, ypix(v)))
                    connected = True
                else:
                    connected = False
            p.setPen(QPen(QColor(s["color"]), 2, s["style"]))
            p.drawPath(path)
        # ---- 图例(矩形绘制+垂直居中+TextDontClip, 上下永不切半) ----
        p.setFont(QFont("Consolas", 11, QFont.Bold))
        _px = 8; _row = 0
        for s in _names:
            _wd = _lfm.width(s["name"]) + 24
            if _px + _wd > self.width() - 12 and _px > 8:
                _px = 8; _row += 1
            if _row >= _rows:
                break
            _band = 8 + _row * _rowh
            p.setPen(QPen(QColor(s["color"]), 2, s["style"]))
            p.drawLine(_px, _band + _rowh // 2, _px + 18, _band + _rowh // 2)
            p.setPen(QColor(s["color"]))
            p.drawText(QRect(_px + 24, _band, _wd - 18, _rowh),
                       Qt.AlignLeft | Qt.AlignVCenter | Qt.TextDontClip, s["name"])
            _px += _wd + 14
        p.end()


# ==================== 实时曲线面板(曲线 + 通道选择) ====================
class PlotPanel(QWidget):
    """实时曲线面板: 上为 CurvePlot, 下为通道选择复选框"""
    PALETTE = ["#f44336","#ff9800","#ffeb3b","#8bc34a","#4caf50","#00bcd4",
               "#2196f3","#3f51b5","#9c27b0","#e91e63","#ff5722","#795548",
               "#009688","#607d8b","#cddc39","#ffc107","#c0ca33","#00e5ff",
               "#536dfe","#ff80ab","#b9f6ca","#b0bec5","#76ff03","#64ffda",
               "#e040fb","#ffab40"]

    def __init__(self, parent=None, include_thr: bool = True):
        super().__init__(parent)
        self._plot = CurvePlot()
        self._channels = ([
            ("roll","Roll"),("pitch","Pitch"),("yaw","Yaw"),
            ("gx","GyroX"),("gy","GyroY"),("gz","GyroZ"),
            ("ax","AccX"),("ay","AccY"),("az","AccZ"),
            ("depth","Depth"),("alt","Alt"),("kdepth","KF深度"),
            ("vx","Vx"),("vy","Vy"),("vz","Vz"),
        ] + ([(f"thr{i}", f"Thr{i}") for i in range(THRUSTER_COUNT)]
              if include_thr else []))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2,2,2,2)
        lay.addWidget(self._plot, 1)

        sel = QWidget(); sel_lay = QGridLayout(sel)
        sel_lay.setContentsMargins(0,0,0,0); sel_lay.setSpacing(2)
        self._checks = {}
        cols = 5
        for idx, (key, name) in enumerate(self._channels):
            color = self.PALETTE[idx % len(self.PALETTE)]
            cb = QCheckBox(name)
            cb.setStyleSheet(f"font-size:12px; color:{color};")
            # 默认显示: 三轴角度 + 深度/高度
            default_show = key in ("roll","pitch","yaw","depth","alt")
            cb.setChecked(default_show)
            cb.stateChanged.connect(lambda _st, k=key: self._on_check(k))
            sel_lay.addWidget(cb, idx // cols, idx % cols)
            self._plot.add_channel(key, name, color, show=default_show)
            self._checks[key] = cb
        lay.addWidget(sel)

    def _on_check(self, key):
        self._plot.set_channel_show(key, self._checks[key].isChecked())

    def feed(self, ch):
        self._plot.feed(ch)

    def clear(self):
        self._plot.clear()


# ==================== 推进器独立曲线网格 (v3.2.2) ====================
class ThrusterPlotGrid(QWidget):
    """12 路推进器按 2 路一组 -> 6 个 CurvePlot 排成 2 行 x 3 列。
    通道显示名 1-based: Thr1..Thr12 (数据键仍为 thr0..thr11)。
    配色: 奇数路统一浅蓝、偶数路统一橙, 便于跨图对比对称路。"""
    PAIRS = [(0, 1), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11)]
    COLOR_ODD = "#29b6f6"   # Thr1, Thr3, ... Thr11
    COLOR_EVEN = "#ffa726"  # Thr2, Thr4, ... Thr12

    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        self._plots = []
        for _gi, (_a, _b) in enumerate(self.PAIRS):
            plot = CurvePlot()
            plot.setFixedHeight(400)
            plot.setMinimumWidth(220)
            plot.add_channel(f"thr{_a}", f"Thr{_a + 1}", self.COLOR_ODD)
            plot.add_channel(f"thr{_b}", f"Thr{_b + 1}", self.COLOR_EVEN)
            grid.addWidget(plot, _gi // 3, _gi % 3)
            self._plots.append(plot)
        for _c in range(3):
            grid.setColumnStretch(_c, 1)
        for _r in range(2):
            grid.setRowStretch(_r, 1)

    def feed(self, ch):
        for plot in self._plots:
            plot.feed(ch)

    def clear(self):
        for plot in self._plots:
            plot.clear()


# ==================== 函数图: 配置窗口 + 数据块独立折线图 (v3.2.2) ====================
class DataPlotBoard(QWidget):
    """函数图页主体: 顶部数据块多选 + 下方每自由度独立小图。
    实际值蓝色实线、目标值橙色虚线；本地深度设定紫色点线。
    不勾选的块整行隐藏但继续收数(环形缓冲)。推进器块复用 ThrusterPlotGrid。"""
    _ORDER = [("roll","Roll"),("pitch","Pitch"),("yaw","Yaw"),
              ("gx","GyroX"),("gy","GyroY"),("gz","GyroZ"),
              ("ax","AccX"),("ay","AccY"),("az","AccZ"),
              ("depth","Depth"),("alt","Alt"),("kdepth","KF深度"),
              ("vx","Vx"),("vy","Vy"),("vz","Vz")]
    # (key, 标签, 默认勾选, 通道key列表 | None=复用推进器网格)
    _SPEC = [
        ("att",  "姿态角",   True,  ["roll","pitch","yaw"]),
        ("gyro", "角速度",   False, ["gx","gy","gz"]),
        ("acc",  "加速度",   False, ["ax","ay","az"]),
        ("vel",  "速度",     False, ["vx","vy","vz"]),
        ("dep",  "深度/高度", True,  ["depth","alt","kdepth"]),
        ("thr",  "推进器",   True,  None),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self); lay.setContentsMargins(2, 2, 2, 2); lay.setSpacing(8)
        # 顶部配置窗口
        cfg = QGroupBox("曲线显示配置 (勾选要查看的数据块, 未勾选的自动隐藏)")
        cl = QHBoxLayout(cfg); cl.setContentsMargins(8, 4, 8, 4); cl.setSpacing(14)
        cl.addStretch(1)
        self._checks = {}; self._rows = {}; self._feed_objs = []
        for key, label, default, _chks in self._SPEC:
            cb = QCheckBox(label)
            cb.setStyleSheet("font-size:13px; font-weight:bold;")
            cb.setChecked(default)
            cb.stateChanged.connect(lambda _s, k=key: self._rows[k].setVisible(self._checks[k].isChecked()))
            cl.addWidget(cb)
            self._checks[key] = cb
        cl.addStretch(1)
        lay.addWidget(cfg)
        # 下方数据块: 每块一行独立图
        name_map = dict(self._ORDER)
        for key, label, default, chkeys in self._SPEC:
            grp = QGroupBox(label)
            gl = QHBoxLayout(grp); gl.setContentsMargins(4, 6, 4, 6); gl.setSpacing(6)
            if chkeys is None:
                board = ThrusterPlotGrid()
                gl.addWidget(board, 1)
                self._feed_objs.append(board)
            else:
                for ck in chkeys:
                    plot = CurvePlot()
                    plot.setFixedHeight(400); plot.setMinimumWidth(200)
                    color = "#81c784" if ck == "kdepth" else "#4fc3f7"
                    units = {"roll":"°", "pitch":"°", "yaw":"°",
                             "gx":"°/s", "gy":"°/s", "gz":"°/s",
                             "ax":"m/s²", "ay":"m/s²", "az":"m/s²",
                             "vx":"m/s", "vy":"m/s", "vz":"m/s",
                             "depth":"m", "alt":"m", "kdepth":"m"}
                    name = f"{name_map[ck]} ({units[ck]})"
                    plot.add_channel(ck, name + " 实际", color)
                    if ck != "kdepth":
                        plot.add_channel("t_" + ck, "目标(回传)", "#ffb74d", style=Qt.DashLine)
                    if ck == "depth":
                        plot.add_channel("local_t_depth", "本地设定", "#ce93d8", style=Qt.DotLine)
                    gl.addWidget(plot, 1)
                    self._feed_objs.append(plot)
            grp.setVisible(default)
            lay.addWidget(grp)
            self._rows[key] = grp
        lay.addStretch(1)

    def feed(self, ch):
        for obj in self._feed_objs:
            obj.feed(ch)

    def clear(self):
        for obj in self._feed_objs:
            if hasattr(obj, "clear"):
                obj.clear()


#
#  4.4 HeadingCompass  圆形航向罗盘 (v3.5)
#
class HeadingCompass(QWidget):
    """圆形航向指示器(罗盘)。

    设计要点(按需求):
      · **零度朝上**(正上方 = 0°/北), 顺时针为正 —— 与航向角(yaw)定义一致。
      · **绝对/相对两种模式**:
          - ABS(绝对): 指针直接指向板端上报的 yaw 绝对值。
          - REL(相对): 指针指向 (yaw - 基准角), 即相对"按压时刻航向"的偏转量。
        两种模式都由**本控件单击**切换:
          · 在 ABS 下单击 → 记下当前 yaw 作基准, 转 REL, 此刻指针归零(朝上)。
          · 在 REL 下单击 → 把基准角累加回当前 yaw(即"重置基准"), 指针仍朝上;
            想要回到绝对值显示, 用**双击**或**右键**。
      · 视觉: 圆形半透明底盘(低不透明度) + 外圈刻度; 指针橙红色偏亮(#ff6a3d 系),
        "稍高"= 指针主体比圆心略长、箭头略微超出外圈, 视觉上更醒目。
      · 尺寸: 由外部 setFixedSize 给定; 默认 168(约下方日志宽度的 1.5 倍观感)。

    角度约定: 入参 yaw 单位**度**(可为任意实数, 内部对 360 取模)。
    """

    _BG      = QColor(10, 14, 20, 110)     # 半透明深色底盘(低不透明度)
    _RING    = QColor(170, 185, 200, 130)  # 外圈
    _TICK    = QColor(200, 210, 220, 120)  # 常规刻度
    _TICK_MAJ= QColor(230, 238, 245, 190)  # 主刻度(每 90°)
    _TXT     = QColor(214, 224, 235, 205)  # 文字
    _TXT_DIM = QColor(170, 182, 195, 150)
    _NDL     = QColor(255, 106, 61)        # 指针主体: 橙红偏亮
    _NDL_HI  = QColor(255, 158, 110)       # 指针高光/箭头亮部
    _NDL_TAIL= QColor(255, 120, 80, 170)   # 指针尾(对向短线)
    _CENTER  = QColor(255, 240, 232, 230)  # 圆心帽

    def __init__(self, title="航向", parent=None):
        super().__init__(parent)
        self._mode_abs = True          # True=绝对, False=相对
        self._yaw = 0.0                # 板端最新 yaw(度)
        self._base = 0.0               # 相对基准角(度), REL 模式下显示 yaw-_base
        self._title = title
        self._has_data = False
        self.setFixedSize(168, 168)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip(
            "航向罗盘(零度朝上, 顺时针为正)\n"
            "· 单击: 绝对模式下把当前航向记为 0°(转相对); 相对模式下重置基准\n"
            "· 双击 / 右键: 切回绝对模式(跟随板端 yaw 真值)")

    # ---------- 数据接口 ----------
    def set_heading(self, yaw_deg):
        """喂入板端 yaw(度)。仅在绝对模式下指针跟随真值。"""
        try:
            v = float(yaw_deg)
        except (TypeError, ValueError):
            return
        self._has_data = True
        if abs(v - self._yaw) < 1e-6:
            return
        self._yaw = v
        self.update()

    def reset_relative(self):
        """把"此刻航向"定义为 0° —— 基准角 = 当前 yaw, 指针归零朝上。"""
        self._base = self._yaw
        self._mode_abs = False
        self.update()

    def set_absolute(self):
        """切回绝对模式: 指针直接指 yaw 真值。"""
        self._mode_abs = True
        self.update()

    def mode_is_abs(self):
        return self._mode_abs

    # ---------- 交互 ----------
    def mousePressEvent(self, ev):
        if ev.button() == Qt.RightButton:
            self.set_absolute()
            return
        if ev.button() == Qt.LeftButton:
            # 绝对 → 记基准转相对; 相对 → 重置基准(仍朝上)
            self._base = self._yaw
            self._mode_abs = False
            self.update()

    def mouseDoubleClickEvent(self, ev):
        # 双击回绝对值
        self.set_absolute()

    # ---------- 绘制 ----------
    def _disp_angle(self):
        """要显示的角度(度): ABS=yaw, REL=yaw-base。"""
        return (self._yaw if self._mode_abs else self._yaw - self._base)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy = w / 2.0, h / 2.0
        r = side / 2.0 - 6.0                       # 外圈半径
        if r <= 6:
            p.end(); return

        # --- 半透明圆底盘 ---
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._BG))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # --- 外圈 ---
        pen = QPen(self._RING); pen.setWidthF(1.6)
        p.setPen(pen); p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # --- 刻度(每 15°, 每 90° 加长加亮) ---
        for deg in range(0, 360, 15):
            major = (deg % 90 == 0)
            tlen = 9.0 if major else 5.0
            pen = QPen(self._TICK_MAJ if major else self._TICK)
            pen.setWidthF(1.8 if major else 1.0)
            p.setPen(pen)
            # deg=0 在正上方, 顺时针为正
            a = math.radians(deg - 90.0)
            ca, sa = math.cos(a), math.sin(a)
            x1, y1 = cx + ca * (r - 1.0), cy + sa * (r - 1.0)
            x2, y2 = cx + ca * (r - tlen), cy + sa * (r - tlen)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # --- 方位字 N/E/S/W (N 朝上) ---
        p.setFont(self._mk_font(side * 0.082, bold=True))
        p.setPen(QPen(self._TXT))
        tr = r - 19.0
        for lab, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            a = math.radians(deg - 90.0)
            tx = cx + math.cos(a) * tr
            ty = cy + math.sin(a) * tr
            p.drawText(QRectF(tx - 10, ty - 9, 20, 18),
                       Qt.AlignCenter, lab)

        # --- 指针(橙红偏亮): 主体从圆心指向显示角, "稍高"= 比圆心略长并略微超出外圈 ---
        ang = self._disp_angle()
        a = math.radians(ang - 90.0)
        ca, sa = math.cos(a), math.sin(a)
        L = r * 0.98                                # 主体长(近外圈, 略"高")
        tipx, tipy = cx + ca * L, cy + sa * L
        # 指针尾部对向短线(让重心偏前, 更像罗盘针)
        tl = r * 0.30
        p.setPen(QPen(self._NDL_TAIL, max(2.2, side * 0.024),
                      Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx - ca * tl, cy - sa * tl), QPointF(cx, cy))
        # 指针主体
        pen = QPen(self._NDL, max(2.8, side * 0.032),
                   Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(QPointF(cx, cy), QPointF(tipx, tipy))
        # 箭头(三角) —— 尖端略微超出外圈, 更醒目
        hw = max(5.0, side * 0.052)                # 箭头半宽
        tipx2, tipy2 = cx + ca * (L + hw * 0.5), cy + sa * (L + hw * 0.5)
        bx, by = cx + ca * (L - hw * 1.4), cy + sa * (L - hw * 1.4)
        nx, ny = -sa, ca                            # 法向
        poly = QPolygonF([
            QPointF(tipx2, tipy2),
            QPointF(bx + nx * hw, by + ny * hw),
            QPointF(bx - nx * hw, by - ny * hw),
        ])
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._NDL_HI))           # 箭头用亮部, 更醒目
        p.drawPolygon(poly)

        # --- 中心读数(角度值 + 模式标记) ---
        #   放在圆盘下缘内侧, 加一层半透明小底衬, 避免与任意方向的指针糊在一起。
        shown = ang % 360.0
        txt_ang = "%d°" % int(round(shown))
        tag = "ABS 绝对" if self._mode_abs else "REL 相对"
        if not self._has_data:
            tag = "无数据"
        # 底衬: 圆盘下半部一块, 两行(角度值 + 模式标记), 按实测行高排布
        fm_val = QFontMetrics(self._mk_font(side * 0.135, bold=True))
        fm_tag = QFontMetrics(self._mk_font(side * 0.070, bold=True))
        line1 = fm_val.height()
        line2 = fm_tag.height()
        pad_v = side * 0.018
        chip_h = line1 + line2 + pad_v * 2.0
        chip_w = max(fm_val.horizontalAdvance("888") + pad_v * 3.0, r * 0.86)
        chip_x = cx - chip_w / 2.0
        # 居中对齐到圆盘下半部: 上沿从圆心略下开始, 底边不越圆盘可见区
        chip_y = cy + r * 0.20
        max_bottom = cy + r * 0.92
        if chip_y + chip_h > max_bottom:
            chip_y = max_bottom - chip_h
        if chip_y < cy + r * 0.08:                # 太高就往下压, 别压住圆心帽
            chip_y = cy + r * 0.08
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(0, 0, 0, 118)))
        p.drawRoundedRect(QRectF(chip_x, chip_y, chip_w, chip_h),
                          side * 0.026, side * 0.026)
        # 第一行: 角度值(亮白)
        p.setFont(self._mk_font(side * 0.135, bold=True))
        p.setPen(QPen(QColor(255, 250, 246, 244)))
        p.drawText(QRectF(chip_x, chip_y + pad_v, chip_w, line1),
                   Qt.AlignCenter, txt_ang)
        # 第二行: 模式标记(REL 用暖橙, ABS 用冷蓝, 一眼区分)
        p.setFont(self._mk_font(side * 0.070, bold=True))
        p.setPen(QPen(QColor(255, 175, 125, 240) if not self._mode_abs
                      else QColor(150, 205, 240, 230)))
        p.drawText(QRectF(chip_x, chip_y + pad_v + line1, chip_w, line2),
                   Qt.AlignCenter, tag)

        # --- 圆心帽(最后画, 压在指针与底衬之上) ---
        p.setPen(Qt.NoPen); p.setBrush(QBrush(self._CENTER))
        cap_r = max(2.6, side * 0.028)
        p.drawEllipse(QPointF(cx, cy), cap_r, cap_r)
        p.end()

    @staticmethod
    def _mk_font(px, bold=False):
        """构造绘制字体。用**像素尺寸**而非点数 —— 罗盘是固定像素尺寸,
        点数会随 DPI 放大导致排布失控。显式给字体族 + 兜底, 避免静默不出字。"""
        f = QFont("Consolas")
        f.setStyleHint(QFont.TypeWriter)
        f.setPixelSize(max(7, int(round(px))))
        f.setBold(bold)
        return f


#
#  4.5 VideoTile  单个视频画面卡片 (v3.3.1 可视窗口)
#
class VideoTile(QFrame):
    """单个视频画面卡片:
    - 画面 4:3 等比缩放居中, 绝不变形; 未填满处与卡片同为纯黑, 看不出黑边
    - 左上角: 画面名称; 右下角: 实际分辨率 + 帧率(显示/收到)
    - 边框颜色表示状态: 绿=有画面 灰=等待信号 暗灰=已关闭/未开启"""
    _BORDER = {"live": "#3fb950", "wait": "#8b949e", "off": "#30363d"}

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName("videoTile")
        self._state = "off"
        self._res = ""
        self._aw, self._ah = 4, 3        # 画面宽高比(按实际收到的帧更新)
        self._fw, self._fh = 0, 0        # v3.4.1: 最近一帧的真实像素宽高(1:1 显示用)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        grid = QGridLayout(self)
        grid.setContentsMargins(3, 3, 3, 3)
        grid.setSpacing(0)

        self.video = QLabel(title)
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(240, 180)      # 4:3 下限, 之上自由放大
        self.video.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video.setStyleSheet(
            "background:#000; border:none; color:#8b949e; font-size:15px;")
        grid.addWidget(self.video, 0, 0)

        self.badge_title = QLabel(title)
        self.badge_title.setStyleSheet(
            "background:rgba(0,0,0,0.55); color:#e6edf3; font-size:12px; "
            "font-weight:bold; padding:2px 8px; border-radius:3px;")
        grid.addWidget(self.badge_title, 0, 0, Qt.AlignTop | Qt.AlignLeft)

        self.badge_stat = QLabel("")
        self.badge_stat.setStyleSheet(
            "background:rgba(0,0,0,0.5); color:#9aa4b2; font-size:11px; "
            "padding:2px 8px; border-radius:3px;")
        grid.addWidget(self.badge_stat, 0, 0, Qt.AlignBottom | Qt.AlignRight)
        for b in (self.badge_title, self.badge_stat):
            b.setAttribute(Qt.WA_TransparentForMouseEvents)
            b.raise_()          # 保证角标画在画面之上

        # v3.5: 航向罗盘浮层(默认不创建, 由 MainWindow 只给 CAM1 挂上)
        self.compass = None
        self.set_state("off")

    def add_compass(self, size=168):
        """v3.5: 在画面右上角挂一个航向罗盘浮层, 返回该控件(供喂 yaw)。
        用 QGridLayout 同格叠加 + AlignTop|AlignRight 贴角, 与角标同一套定位方式。"""
        if self.compass is None:
            self.compass = HeadingCompass("航向", self)
            self.compass.setFixedSize(size, size)
            # 罗盘需要接收鼠标(单击/双击切绝对/相对), **不能**设 WA_TransparentForMouseEvents
            self.layout().addWidget(self.compass, 0, 0,
                                    Qt.AlignTop | Qt.AlignRight)
        self.compass.show()
        self.compass.raise_()
        return self.compass

    def compass_widget(self):
        return self.compass

    def set_state(self, state):
        self._state = state if state in self._BORDER else "off"
        c = self._BORDER[self._state]
        self.setStyleSheet(f"QFrame#videoTile{{background:#000; border:2px solid {c};"
                           f" border-radius:6px;}}")

    def set_res(self, w, h):
        """记录实际分辨率(角标显示 + 宽高比); 传 0 表示画面中断, 清空"""
        if not w or not h:
            if self._res:
                self._res = ""
                self._fw, self._fh = 0, 0
                self.updateGeometry()
            return
        self._fw, self._fh = int(w), int(h)   # v3.4.1: 记真实像素, 供 1:1 显示抬最小尺寸
        txt = f"{w}×{h}"
        if txt != self._res:
            self._res = txt
            if w > 0 and h > 0:
                self._aw, self._ah = w, h
                self.updateGeometry()   # 按新宽高比重排

    def set_stat(self, text):
        cur = self.badge_stat.text()
        new = f"{self._res} · {text}" if self._res else text
        if new != cur:
            self.badge_stat.setText(new)

    # 高度 = 宽度 / 宽高比 (+ 边框与内边距); 由 VideoWall 在布局完成后用于收紧高度
    def heightForWidth(self, w):
        pad = 10   # 边框2*2 + 布局边距3*2
        inner = max(0, w - pad)
        return max(self.video.minimumHeight() + pad,
                   int(round(inner * self._ah / float(self._aw))) + pad)


class VideoWall(QWidget):
    """视频墙容器: 每轮布局/缩放结束后, 把卡片高度收到 4:3,
    使卡片边框紧贴画面(竖长排布时不会拖出大片空白)"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background:#0d1117; border-radius:6px;")
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(6, 6, 6, 6)
        self.grid.setSpacing(6)
        self._fitting = False

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        QTimer.singleShot(0, self.fit_tiles)   # 等布局算完再收高度

    def fit_tiles(self):
        if self._fitting:
            return
        self._fitting = True
        try:
            # v3.4.1: 1:1 原始尺寸模式下不收高度 —— 卡片必须能容纳帧的真实像素,
            #   否则 640×480 会被裁掉下半截。此时把每张卡的 maximumHeight 放开。
            raw = bool(getattr(self.window(), "_rawsize_1to1", False))
            for i in range(self.grid.count()):
                w = self.grid.itemAt(i).widget()
                if isinstance(w, VideoTile) and w.isVisible():
                    if raw:
                        if w.maximumHeight() != 16777215:  # QWIDGETSIZE_MAX
                            w.setMaximumHeight(16777215)
                        continue
                    want = w.heightForWidth(w.width())
                    if abs(want - w.maximumHeight()) > 1:
                        w.setMaximumHeight(want)   # 变化会触发下一轮, 数值收敛即停
        finally:
            self._fitting = False


#
#  5. MainWindow  主窗口
#
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV控制站 v3.8")
        self.resize(1360, 900)
        self._grab = False
        self._store = False
        self._recording = False
        self._rec_writer1 = None      # v3.2: 双摄各自独立录像writer
        self._rec_writer2 = None
        self._rec_writer3 = None      # v3.3: 第三路画面(预留)录像writer
        # v3.3: 第三路画面(CAM3)取流开关, 默认关(待机占位), 由“可视窗口”页面按钮开启
        self._cam3_on = False
        # v3.4.1: 画面是否按 1:1 原始尺寸显示(不做放大/平滑缩放), 由"显示方式"按钮切换
        self._rawsize_1to1 = False
        self._vid3_thread = None
        self._last_frame = None
        self._mode = 0   # 0=有线遥控(ROV), 1=自主航行(AUV) —— 对应 $CMD.mode 第 7 字段
        # v3.4: 急停待机标志。S100 没有模式回传通道, UI 只能自己记"已急停"以显示「待机」;
        #       点「温启动」($ESTOP,0#) 时清除。仅影响 UI 显示与按钮高亮, 不影响协议。
        self._frozen = False
        self._latency_ms = -1
        # v3.2: 链路模式 0=有线(网线) 1=无线(LoRa)
        self._link_mode = 0
        # v3.2: 视频传输开关状态(默认开)
        self._video_on = True
        # v3.2: 连接状态判断依据
        self._last_tel_time = None    # 最后一次收到遥测的时间
        self._last_ping_ok = None     # 最后一次PING成功的时间
        self._tel_hz = 0.0            # 遥测帧率
        self._tel_cnt_window = 0
        # v3.3.1: 三路(含预留CAM3)显示帧率 / 推流帧率 / 最近一帧时间
        self._vid_fps = {"cam1": 0, "cam2": 0, "cam3": 0}
        self._vid_recv_fps = {"cam1": 0, "cam2": 0, "cam3": 0}  # v3.2.2: 推流实际帧率
        self._vid_last_frame = {"cam1": 0.0, "cam2": 0.0, "cam3": 0.0}
        # 数据记录(CSV)
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._csv_fields = self._make_csv_fields()
        # v3.3.3: 深度卡尔曼融合器 (融合 depth + vz + 可选 alt), 见 DepthKalmanFilter
        self._kf_depth = DepthKalmanFilter()
        self._kf_value = None
        self._local_target_depth = None
        self._reported_target_depth = None
        self._depth_sender = None  # 接入真实通讯后由 set_depth_sender 注入
        self._position = PositionTracker()
        self._pid_ser = None
        self._pid_rx_buffer = ReferenceTelemetryBuffer()
        self._pid_reply_count = 0
        self._pid_pending = {}
        self._build_ui()
        self._start_threads()

    def _make_csv_fields(self):
        """数据记录 CSV 的列名: 姿态/角速度/加速度/深度/高度/线速度 + 12推进器"""
        f = ["roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az",
             "depth", "alt", "kdepth", "vx", "vy", "vz"]
        f += [f"thr{i}" for i in range(THRUSTER_COUNT)]
        f += ["t_" + k for k in ("roll", "pitch", "yaw", "gx", "gy", "gz",
                                  "ax", "ay", "az", "depth", "alt", "vx", "vy", "vz")]
        f += ["local_t_depth", "position_x", "position_y", "position_heading"]
        return f

    #  UI 构建 
    def _build_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ====== 顶部状态栏 ======
        top_bar = QHBoxLayout()
        top_bar.setSpacing(6)
        self.rov_btn = QPushButton("ROV 遥控")
        self.rov_btn.setCheckable(True)
        self.rov_btn.setToolTip("切换到 ROV 遥控，$CMD.mode=0。所有页面均可操作。")
        self.rov_btn.clicked.connect(lambda: self._set_mode(0))
        self.auv_btn = QPushButton("AUV 自主")
        self.auv_btn.setCheckable(True)
        self.auv_btn.setToolTip("切换到 AUV 自主，$CMD.mode=1。所有页面均可操作。")
        self.auv_btn.clicked.connect(lambda: self._set_mode(1))
        self.mode_lbl = QLabel("模式: 有线遥控")
        self.mode_lbl.setStyleSheet("font-weight:bold; font-size:15px; padding:4px 12px; background:#c8e6c9; border-radius:4px;")
        # v3.2: 连接状态指示 (红=未连接 绿=已连接)
        self.conn_lbl = QLabel("连接: 检测中...")
        self.conn_lbl.setStyleSheet("font-weight:bold; font-size:13px; padding:4px 10px; background:#ffcdd2; color:#b71c1c; border-radius:4px;")
        # v3.2: 链路模式切换 (有线/无线)
        self.link_btn = QPushButton("链路: 有线 [网线]")
        self.link_btn.setCheckable(False)
        self.link_btn.setStyleSheet("padding:5px 12px; font-size:13px; font-weight:bold; background:#c8e6c9; border-radius:4px;")
        self.link_btn.clicked.connect(self._toggle_link_mode)
        # v3.2: LoRa 串口选择 (无线模式用)
        self.com_refresh_btn = QPushButton("刷新串口")
        self.com_refresh_btn.setStyleSheet("padding:5px 8px; font-size:12px;")
        self.com_refresh_btn.clicked.connect(self._refresh_com_ports)
        from PyQt5.QtWidgets import QComboBox
        self.com_combo = QComboBox()
        self.com_combo.setMinimumWidth(110)
        self.com_combo.setToolTip("地面LoRa模块的USB串口(无线模式用)")
        self.com_combo.currentTextChanged.connect(self._on_com_selected)
        # v3.2: 视频传输开关
        self.video_btn = QPushButton("视频: 开")
        self.video_btn.setCheckable(False)
        self.video_btn.setStyleSheet("padding:5px 12px; font-size:13px; background:#bbdefb; border-radius:4px;")
        self.video_btn.clicked.connect(self._toggle_video_stream)
        self.status_lbl = QLabel("等待手柄...")
        self.status_lbl.setStyleSheet("color:#555; font-size:13px;")
        self.net_lbl = QLabel("网络: --")
        self.net_lbl.setStyleSheet("font-size:13px;")
        self.latency_lbl = QLabel("延迟: --")
        self.latency_lbl.setStyleSheet("font-size:13px;")
        self.rec_btn = QPushButton("录像 [X]")
        self.rec_btn.setCheckable(True)
        self.rec_btn.setStyleSheet("padding:5px 12px; font-size:13px;")
        self.rec_btn.clicked.connect(self._toggle_record)
        self.store_btn = QPushButton("存储 [Y]")
        self.store_btn.setCheckable(True)
        self.store_btn.setStyleSheet("padding:5px 12px; font-size:13px;")
        self.store_btn.clicked.connect(self._toggle_store)
        top_bar.addWidget(self.rov_btn)
        top_bar.addWidget(self.auv_btn)
        top_bar.addWidget(self.mode_lbl)
        top_bar.addWidget(self.conn_lbl)
        top_bar.addWidget(self.link_btn)
        top_bar.addStretch(1)
        top_bar.addWidget(self.rec_btn)
        top_bar.addWidget(self.store_btn)
        root.addLayout(top_bar)
        self._paint_mode()
        # 链路细节独立一行，避免新增模式按钮挤掉状态栏。
        detail_bar = QHBoxLayout()
        detail_bar.setSpacing(6)
        detail_bar.addWidget(self.com_refresh_btn)
        detail_bar.addWidget(self.com_combo)
        detail_bar.addWidget(self.video_btn)
        detail_bar.addWidget(self.status_lbl, 1)
        detail_bar.addWidget(self.net_lbl)
        detail_bar.addWidget(self.latency_lbl)
        root.addLayout(detail_bar)

        depth_bar = QHBoxLayout()
        depth_bar.addWidget(QLabel("目标深度"))
        self.depth_input = QDoubleSpinBox()
        self.depth_input.setRange(0.0, 300.0)
        self.depth_input.setDecimals(2)
        self.depth_input.setSingleStep(0.01)
        self.depth_input.setSuffix(" m")
        self.depth_input.setMinimumWidth(140)
        self.depth_input.setKeyboardTracking(False)
        self.depth_input.setToolTip("填写目标深度，点击设定；不会随遥测回传覆盖输入值。")
        depth_bar.addWidget(self.depth_input)
        self.depth_set_btn = QPushButton("设定目标")
        self.depth_set_btn.clicked.connect(self._set_target_depth)
        depth_bar.addWidget(self.depth_set_btn)
        self.depth_send_btn = QPushButton("发送目标（待接入）")
        self.depth_send_btn.setEnabled(False)
        self.depth_send_btn.setToolTip("当前 S100/STM32 目标下发接口尚未接入；本地设定不发送到机器。")
        self.depth_send_btn.clicked.connect(self._send_target_depth)
        depth_bar.addWidget(self.depth_send_btn)
        self.depth_status_lbl = QLabel("本地目标: -- · 尚未下发")
        self.depth_status_lbl.setStyleSheet("color:#8e5a18;")
        depth_bar.addWidget(self.depth_status_lbl)
        depth_bar.addStretch(1)
        self.depth_feedback_lbl = QLabel("回传目标: -- · 实际深度: --")
        depth_bar.addWidget(self.depth_feedback_lbl)
        root.addLayout(depth_bar)
        self._refresh_com_ports()  # 启动时枚举一次串口

        # ====== 选项卡(一键切换, 共6页) ======
        self._main_tabs = QTabWidget()
        self._main_tabs.setStyleSheet(
            "QTabBar::tab{height:34px; font-weight:bold; font-size:14px; padding:0 18px;}")

        # ---------------------------------------------------------------
        # 页面1 主界面: 视频 + 实时操控 + 重要按钮/LED调参 (密集)
        # ---------------------------------------------------------------
        drive = QWidget(); dlay = QVBoxLayout(drive); dlay.setContentsMargins(4,4,4,4)
        # v3.3: 视频画面(CAM1/CAM2/CAM3)已迁入“可视窗口”页面, 主界面专注操控与调参
        #       三个画面在下方 _build_view_page() 中创建

        # 操控/调参区
        bot = QHBoxLayout()
        left = QVBoxLayout()
        stick_grp = QGroupBox("左摇杆 (Surge/Sway)")
        stick_lay = QVBoxLayout(stick_grp)
        self.stick_widget = StickWidget()
        stick_lay.addWidget(self.stick_widget)
        stick_val_lay = QHBoxLayout()
        self.surge_val_lbl = QLabel("Surge: 0.00")
        self.sway_val_lbl = QLabel("Sway: 0.00")
        self.surge_val_lbl.setAlignment(Qt.AlignCenter)
        self.sway_val_lbl.setAlignment(Qt.AlignCenter)
        self.surge_val_lbl.setStyleSheet("font-size:13px; font-weight:bold;")
        self.sway_val_lbl.setStyleSheet("font-size:13px; font-weight:bold;")
        stick_val_lay.addWidget(self.surge_val_lbl)
        stick_val_lay.addWidget(self.sway_val_lbl)
        stick_lay.addLayout(stick_val_lay)
        left.addWidget(stick_grp, 1)

        ry_grp = QGroupBox("右摇杆 (Yaw / Heave)")
        ry_lay = QVBoxLayout(ry_grp)
        ry_lay.addWidget(QLabel("Yaw (RStick X，右推为正)"))
        self.yaw_bar = YawBar("Yaw"); ry_lay.addWidget(self.yaw_bar)
        ry_lay.addWidget(QLabel("Heave (RStick Y，前推上浮)"))
        self.heave_bar = YawBar("Heave"); ry_lay.addWidget(self.heave_bar)
        ry_lay.addWidget(QLabel("LT / RT 扳机 (预留)"))
        self.rsticky_bar = YawBar("Trig"); ry_lay.addWidget(self.rsticky_bar)
        ry_lay.addStretch()
        left.addWidget(ry_grp, 1)
        bot.addLayout(left, 1)

        thr_grp = QGroupBox("推进器转速 (油门%)")
        thr_lay = QHBoxLayout(thr_grp)
        thr_lay.setSpacing(1); thr_lay.setContentsMargins(2, 2, 2, 2)
        self.thruster_bars = []
        for i in range(THRUSTER_COUNT):
            tb = ThrusterBar(f"T{i+1}")
            thr_lay.addWidget(tb)
            self.thruster_bars.append(tb)
        bot.addWidget(thr_grp, 2)

        right = QVBoxLayout()
        ctrl_grp = QGroupBox("指令值")
        ctrl_lay = QGridLayout(ctrl_grp); ctrl_lay.setSpacing(3)
        self.ctrl_labels = {}
        items = [("surge","Surge"),("sway","Sway"),("heave","Heave"),("yaw","Yaw"),
                 ("led1","LED1"),("led2","LED2"),("grab","Grab"),("store","Store")]
        for i, (key, text) in enumerate(items):
            lbl = QLabel(text); lbl.setStyleSheet("font-size:12px;")
            ctrl_lay.addWidget(lbl, i, 0)
            v = QLabel("0"); v.setAlignment(Qt.AlignCenter)
            v.setStyleSheet("font-weight:bold; font-size:12px;")
            ctrl_lay.addWidget(v, i, 1)
            self.ctrl_labels[key] = v
        right.addWidget(ctrl_grp)

        # v3.4: 按功能分组重排(工作模式 / 遥控交还 / 安全 / 作业)。
        #   模式语义对齐《多源控制协议设计.md》§12.4:
        #     「有线遥控」= $CMD.mode=0 → S100 转 V2 0x04 0x06(ROV_TETHERED)
        #     「自主航行」= $CMD.mode=1 → S100 转 V2 0x04 0x05(AUV)
        #     「交还自主」= 同 mode=1, 但语义是"退出遥控", 独立按钮不易误操作
        #     「温启动」  = $ESTOP,0# → S100 转 V2 0x04 0x00(START), 从 STANDBY 恢复到 Last_Con_Mode
        #   注意: $CMD.mode 在 S100 侧只认 0/1, 故不设"测试模式"按钮(见改造方案 §2.1)。
        act_grp = QGroupBox("快捷操作")
        act_lay = QGridLayout(act_grp); act_lay.setSpacing(4)
        btn_style = "padding:8px 10px; font-size:13px; border-radius:4px;"
        _sec = "font-size:11px; color:#777; padding:0 2px;"   # 分组小标题
        _row = 0
        # 工作模式按钮常驻顶部状态栏，所有页面均可切换。
        # --- 遥控交还(显式退出遥控; §12.4 推荐的"甲"方案) ---
        self.handback_btn = QPushButton("交还自主")
        self.handback_btn.setToolTip("退出遥控、把控制权交回 S100 自主($CMD.mode=1)")
        self.handback_btn.setStyleSheet(btn_style + "background:#e3f2fd;")
        self.handback_btn.clicked.connect(lambda: self._set_mode(1))
        act_lay.addWidget(self.handback_btn, _row, 0, 1, 2)
        _row += 1
        # --- 安全(v3.3 决策 B: 急停/温启动, 常驻可见) ---
        act_lay.addWidget(QLabel("安全"), _row, 0, 1, 2); _row += 1
        self.estop_btn = QPushButton("急停")
        self.estop_btn.setStyleSheet(btn_style + "background:#d32f2f; color:white; font-weight:bold;")
        self.estop_btn.setToolTip("零杆位 $CMD + $ESTOP# → S100 转 V2 0x04 0x01 进 STANDBY 并锁存")
        self.estop_btn.clicked.connect(self._click_estop)
        act_lay.addWidget(self.estop_btn, _row, 0)
        # 「温启动」= 原「解除急停」。$ESTOP,0# → S100 转 V2 0x04 0x00(START),
        # 从 STANDBY 恢复到 Last_Con_Mode(不是复位重启, 故用 §12.4 的"温启动"措辞)
        self.estop_release_btn = QPushButton("温启动")
        self.estop_release_btn.setStyleSheet(btn_style + "background:#9e9e9e; color:white;")
        self.estop_release_btn.setToolTip("$ESTOP,0# → S100 转 V2 0x04 0x00(START)\n从 STANDBY 恢复到急停前的模式(温启动)")
        self.estop_release_btn.clicked.connect(self._click_estop_release)
        act_lay.addWidget(self.estop_release_btn, _row, 1)
        _row += 1
        # --- 作业 ---
        act_lay.addWidget(QLabel("作业"), _row, 0, 1, 2); _row += 1
        self.grab_btn = QPushButton("抓球 [A]")
        self.grab_btn.setStyleSheet(btn_style + "background:#4caf50; color:white;")
        self.grab_btn.clicked.connect(lambda: self._click_grab(1))
        act_lay.addWidget(self.grab_btn, _row, 0)
        self.throw_btn = QPushButton("抛球 [B]")
        self.throw_btn.setStyleSheet(btn_style + "background:#f44336; color:white;")
        self.throw_btn.clicked.connect(lambda: self._click_grab(2))
        act_lay.addWidget(self.throw_btn, _row, 1)
        right.addWidget(act_grp)

        led_grp = QGroupBox("LED 亮度 (调参)")
        led_lay = QGridLayout(led_grp); led_lay.setSpacing(4)
        led_lay.addWidget(QLabel("LED1"), 0, 0)
        self.led1_slider = QSlider(Qt.Horizontal)
        self.led1_slider.setRange(0, 100); self.led1_slider.setValue(0)
        self.led1_slider.setStyleSheet("QSlider::groove:horizontal{height:8px;background:#ddd;border-radius:4px;}"
            "QSlider::handle:horizontal{width:16px;margin:-4px 0;background:#ff9800;border-radius:8px;}"
            "QSlider::sub-page:horizontal{background:#ff9800;border-radius:4px;}")
        self.led1_val = QLabel("0"); self.led1_val.setFixedWidth(40)
        self.led1_val.setAlignment(Qt.AlignCenter); self.led1_val.setStyleSheet("font-weight:bold; font-size:13px;")
        self.led1_slider.valueChanged.connect(self._on_led1_slider)
        led_lay.addWidget(self.led1_slider, 0, 1); led_lay.addWidget(self.led1_val, 0, 2)
        led_lay.addWidget(QLabel("LED2"), 1, 0)
        self.led2_slider = QSlider(Qt.Horizontal)
        self.led2_slider.setRange(0, 100); self.led2_slider.setValue(0)
        self.led2_slider.setStyleSheet("QSlider::groove:horizontal{height:8px;background:#ddd;border-radius:4px;}"
            "QSlider::handle:horizontal{width:16px;margin:-4px 0;background:#2196f3;border-radius:8px;}"
            "QSlider::sub-page:horizontal{background:#2196f3;border-radius:4px;}")
        self.led2_val = QLabel("0"); self.led2_val.setFixedWidth(40)
        self.led2_val.setAlignment(Qt.AlignCenter); self.led2_val.setStyleSheet("font-weight:bold; font-size:13px;")
        self.led2_slider.valueChanged.connect(self._on_led2_slider)
        led_lay.addWidget(self.led2_slider, 1, 1); led_lay.addWidget(self.led2_val, 1, 2)
        right.addWidget(led_grp)
        right.addStretch()
        bot.addLayout(right, 1)
        dlay.addLayout(bot, 1)
        self._main_tabs.addTab(drive, "主界面")

        # ---------------------------------------------------------------
        # 页面2 可视窗口 (v3.3): CAM1/CAM2 迁入本页 + 预留第三个画面 CAM3
        # ---------------------------------------------------------------
        self._main_tabs.addTab(self._build_view_page(), "可视窗口")

        # ---------------------------------------------------------------
        # 页面3 终端 v3.2: 运行情况监控 (事件流 + 链路状态 + 原始帧)
        # ---------------------------------------------------------------
        term_page = QWidget(); tlayout = QVBoxLayout(term_page); tlayout.setContentsMargins(4,4,4,4)
        # 顶部工具行
        tbar = QHBoxLayout()
        self.term_pause_chk = QCheckBox("暂停自动滚动")
        self.term_pause_chk.stateChanged.connect(lambda: None)
        self.term_rawtel_chk = QCheckBox("显示原始遥测帧 $TEL")
        self.term_rawcmd_chk = QCheckBox("显示发送指令帧 $CMD")
        self.term_summary_chk = QCheckBox("周期链路摘要(5s)")
        self.term_summary_chk.setChecked(True)
        self.term_clear_btn = QPushButton("清空终端")
        self.term_clear_btn.setStyleSheet("padding:4px 12px;")
        self.term_clear_btn.clicked.connect(lambda: self.term_text.clear())
        tbar.addWidget(self.term_pause_chk)
        tbar.addWidget(self.term_rawtel_chk)
        tbar.addWidget(self.term_rawcmd_chk)
        tbar.addWidget(self.term_summary_chk)
        tbar.addStretch(1)
        tbar.addWidget(self.term_clear_btn)
        tlayout.addLayout(tbar)
        # 终端正文
        self.term_text = QTextEdit()
        self.term_text.setReadOnly(True)
        self.term_text.setStyleSheet(
            "background:#0d1117; color:#c9d1d9; font-family:Consolas,'Courier New',monospace; "
            "font-size:13px; border:1px solid #30363d; border-radius:3px; padding:4px;")
        tlayout.addWidget(self.term_text, 1)
        # 底部链路摘要行(常显)
        self.term_stat_lbl = QLabel("链路: --")
        self.term_stat_lbl.setStyleSheet(
            "font-family:Consolas,monospace; font-size:12px; color:#58a6ff; padding:2px 6px;")
        tlayout.addWidget(self.term_stat_lbl)
        self._main_tabs.addTab(term_page, "终端")

        # ---------------------------------------------------------------
        # 页面4 函数图: 顶部配置窗口(数据块多选) + 下方每自由度独立折线图
        # ---------------------------------------------------------------
        curve_page = QWidget(); clayout = QVBoxLayout(curve_page); clayout.setContentsMargins(4,4,4,4)
        cscroll = QScrollArea(); cscroll.setWidgetResizable(True)
        cscroll.setStyleSheet("QScrollArea{border:none;}")
        self._plot_board = DataPlotBoard()
        cscroll.setWidget(self._plot_board)
        clayout.addWidget(cscroll, 1)
        self._main_tabs.addTab(curve_page, "函数图")

        # ---------------------------------------------------------------
        # 页面5 全部: 参数+电池+高度计+快捷键
        # ---------------------------------------------------------------
        all_page = QWidget(); allay = QVBoxLayout(all_page); allay.setContentsMargins(4,4,4,4)
        ascr = QScrollArea(); ascr.setWidgetResizable(True)
        inner = QWidget(); ilay = QVBoxLayout(inner)
        ilay.setContentsMargins(6,6,6,6); ilay.setSpacing(8)

        # 第一行: 姿态 + 电池
        h1 = QHBoxLayout()
        att_grp = QGroupBox("姿态 / 运动参数")
        att_lay = QGridLayout(att_grp)
        att_lay.addWidget(QLabel(""), 0, 0)
        att_lay.addWidget(QLabel("目标"), 0, 1)
        att_lay.addWidget(QLabel("实际"), 0, 2)
        self.att_labels = {}
        _att_rows = [
            ("section", "三轴角度"),
            ("roll","Roll"), ("pitch","Pitch"), ("yaw","Yaw"),
            ("section", "角速度 (Gyro)"),
            ("gx","GyroX"), ("gy","GyroY"), ("gz","GyroZ"),
            ("section", "加速度 (Acc)"),
            ("ax","AccX"), ("ay","AccY"), ("az","AccZ"),
            ("section", "深度 / 高度"),
            ("depth","Depth"), ("alt","Alt"),
            # v3.3.3: 卡尔曼融合深度 (由 DepthKalmanFilter 计算, 非遥测原始值)
            ("kdepth","Kalman深度"),
            # v3.6: 板端融合深度/离底净空 ($TEL 帧尾追加字段, 板端 depth_kalman 计算)
            ("fdepth","板端融合深度"),
            ("clr","离底净空"),
            ("section", "线速度 (Vel)"),
            ("vx","VelX"), ("vy","VelY"), ("vz","VelZ"),
        ]
        _r = 1
        for kind, name in _att_rows:
            if kind == "section":
                lbl = QLabel(f"─ {name} ─")
                lbl.setStyleSheet(STY_HEAD)   # v3.3.2: 与高度计小标题同字号
                att_lay.addWidget(lbl, _r, 0, 1, 3); _r += 1
            else:
                att_lay.addWidget(QLabel(name), _r, 0)
                tgt = QLabel("--"); act = QLabel("--")
                tgt.setAlignment(Qt.AlignCenter); act.setAlignment(Qt.AlignCenter)
                att_lay.addWidget(tgt, _r, 1)
                att_lay.addWidget(act, _r, 2)
                self.att_labels[name] = (tgt, act); _r += 1
        h1.addWidget(att_grp, 1)

        bat_grp = QGroupBox("电池 / 环境")
        bat_lay = QGridLayout(bat_grp)
        self.bat_labels = {}
        for i, (k, txt) in enumerate([("volt","电池电压 V"),("curr","电池电流 A"),
                                      ("pct","电量 %"),("temp_water","电池温度 C"),
                                      ("temp_internal","舱内温度 C")]):
            bat_lay.addWidget(QLabel(txt), i, 0)
            v = QLabel("--"); v.setAlignment(Qt.AlignCenter)
            bat_lay.addWidget(v, i, 1)
            self.bat_labels[k] = v
        h1.addWidget(bat_grp, 1)
        ilay.addLayout(h1)

        # v3.3.2: 曲线已移除 —— 与「函数图」页完全重复, 统一在函数图页查看
        _plot_tip = QLabel("实时曲线已合并到「函数图」页 (曲线不再在本页重复显示)")
        _plot_tip.setStyleSheet("font-size:12px; color:#888; padding:4px 6px;")
        ilay.addWidget(_plot_tip)

        # 高度计 CH348 (v3.2.1): 板端 read_altimeter.py UDP -> 8082, 显示在参数汇总页
        # v3.2.2: 高度计区域 3行x6列 —— 第0列标签(通道/状态/数值), A~E 每通道一列
        alt_grp = QGroupBox("高度计 CH348 (DYP-L08 超声波)")
        alt_lay = QGridLayout(alt_grp); alt_lay.setSpacing(4)
        # v3.3.2: 字号改 pt(随 DPI 缩放), 与姿态/电池等区保持一致
        self._alt_labels = {}
        for _r, _txt in enumerate(["通道", "状态", "数值 (mm)"]):
            _tl = QLabel(_txt); _tl.setStyleSheet(STY_HEAD)
            alt_lay.addWidget(_tl, _r, 0)
        for _c, _ch in enumerate(list("ABCDE"), start=1):
            _hl = QLabel(_ch + " 口"); _hl.setAlignment(Qt.AlignCenter)
            _hl.setStyleSheet(STY_HEAD)
            alt_lay.addWidget(_hl, 0, _c)
            _st = QLabel("--"); _st.setAlignment(Qt.AlignCenter)
            _st.setStyleSheet(STY_TEXT + "color:#999;")
            _vv = QLabel("--"); _vv.setAlignment(Qt.AlignCenter)
            _vv.setStyleSheet(STY_VAL)
            alt_lay.addWidget(_st, 1, _c)
            alt_lay.addWidget(_vv, 2, _c)
            self._alt_labels[_ch] = (_st, _vv)
        ilay.addWidget(alt_grp)

        # 快捷键映射
        key_grp = QGroupBox("手柄快捷键映射")
        key_lay = QVBoxLayout(key_grp)
        keymap_text = ("左摇杆 X/Y .. Sway/Surge (水平移动)\n"
                       "右摇杆 X .... Yaw  (水平旋转，已反向修正)\n"
                       "右摇杆 Y .... Heave (上浮/下潜)\n"
                       "RT / LT ..... (预留)\n"
                       "A .......... 抓球\n"
                       "B .......... 抛球\n"
                       "X .......... 录像开关\n"
                       "Y .......... 数据存储开关\n"
                       "LB ......... 有线遥控/自主航行 切换\n"
                       "DPad ↑↓ .... LED1 亮度\n"
                       "DPad ←→ .... LED2 亮度")
        kml = QLabel(keymap_text)
        kml.setStyleSheet("font-size:13px; padding:6px; background:#f5f5f5; border-radius:3px;")
        kml.setWordWrap(True)
        key_lay.addWidget(kml)
        ilay.addWidget(key_grp)

        ascr.setWidget(inner)
        allay.addWidget(ascr, 1)
        self._main_tabs.addTab(all_page, "全部")

        # v3.7: 七项控制 PID，采用参考二进制帧，与原推进器 PID 无关。
        pid_page = QWidget()
        pid_layout = QVBoxLayout(pid_page)
        pid_scroll = QScrollArea()
        pid_scroll.setWidgetResizable(True)
        self._pid_panel = ControlPidPanel()
        self._pid_panel.send_requested.connect(self._send_control_pid)
        self._pid_panel.status_sig.connect(self._log)
        self._pid_panel.refresh_ports_requested.connect(self._refresh_pid_ports)
        self._pid_panel.serial_toggle_requested.connect(self._toggle_pid_serial)
        pid_scroll.setWidget(self._pid_panel)
        pid_layout.addWidget(pid_scroll)
        self._main_tabs.addTab(pid_page, "PID 调参")
        self._refresh_pid_ports()
        self._pid_rx_timer = QTimer(self)
        self._pid_rx_timer.timeout.connect(self._poll_pid_serial)
        self._pid_rx_timer.timeout.connect(self._poll_pid_timeouts)
        self._pid_rx_timer.start(50)

        root.addWidget(self._main_tabs, 1)

        # ====== 底部日志面板 ======
        log_grp = QGroupBox("操作日志")
        log_lay = QVBoxLayout(log_grp)
        log_lay.setContentsMargins(4, 4, 4, 4)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(140)
        self.log_text.setStyleSheet(
            "background:#1e1e1e; color:#00ff00; font-family:Consolas,monospace; "
            "font-size:13px; border:1px solid #333; border-radius:3px; padding:2px;")
        log_lay.addWidget(self.log_text)
        root.addWidget(log_grp)

    # ---------- v3.3 / v3.3.1: “可视窗口”页面 ----------
    # 画面布局预设: (画面key, row, col, rowSpan, colSpan) + 行列拉伸系数
    #   设计原则: CAM1/CAM2 谁重要谁当主画面(可只留一个); CAM3 始终与 CAM2 同步
    #   ——CAM2 上版则 CAM3 上版(紧邻其旁), CAM2 收起则 CAM3 一并收起。
    _VIEW_PRESETS = (
        {"name": "CAM1 主 + CAM2/CAM3",
         "items": (("cam1", 0, 0, 2, 1), ("cam2", 0, 1, 1, 1), ("cam3", 1, 1, 1, 1)),
         "col": {0: 2, 1: 1}, "row": {0: 1, 1: 1}},
        {"name": "CAM2 主 + CAM1/CAM3",
         "items": (("cam2", 0, 0, 2, 1), ("cam1", 0, 1, 1, 1), ("cam3", 1, 1, 1, 1)),
         "col": {0: 2, 1: 1}, "row": {0: 1, 1: 1}},
        {"name": "三画面等分 (1×3)",
         "items": (("cam1", 0, 0, 1, 1), ("cam2", 0, 1, 1, 1), ("cam3", 0, 2, 1, 1)),
         "col": {0: 1, 1: 1, 2: 1}, "row": {0: 1}},
        {"name": "仅 CAM1 (单画面)",
         "items": (("cam1", 0, 0, 1, 1),),
         "col": {0: 1}, "row": {0: 1}},
        {"name": "CAM2 主 + CAM3 (不带 CAM1)",
         "items": (("cam2", 0, 0, 1, 1), ("cam3", 0, 1, 1, 1)),
         "col": {0: 2, 1: 1}, "row": {0: 1}},
    )

    def _build_view_page(self):
        """v3.3.1: 视频墙 —— CAM1/CAM2 主画面可切换, CAM3 与 CAM2 同步显示
        画面严格 4:3 等比缩放居中(640×480 不变形), 未填满处与卡片同色纯黑"""
        page = QWidget()
        vlay = QVBoxLayout(page)
        vlay.setContentsMargins(6, 6, 6, 6)
        vlay.setSpacing(6)

        # --- 顶部工具行 ---
        bar = QHBoxLayout()
        bar.setSpacing(8)
        tip = QLabel("画面布局")
        tip.setStyleSheet("font-size:13px; font-weight:bold;")
        bar.addWidget(tip)
        self.view_layout_combo = QComboBox()
        self.view_layout_combo.addItems([p["name"] for p in self._VIEW_PRESETS])
        self.view_layout_combo.setMinimumWidth(200)
        self.view_layout_combo.setToolTip(
            "CAM1/CAM2 谁重要就把谁设为主画面(也可只留一路);\n"
            "CAM3 始终跟随 CAM2: CAM2 上版则 CAM3 上版, CAM2 收起则 CAM3 一并收起。")
        self.view_layout_combo.currentIndexChanged.connect(self._apply_view_layout)
        bar.addWidget(self.view_layout_combo)

        self.cam3_btn = QPushButton("CAM3 取流: 关")
        self.cam3_btn.setStyleSheet(
            "padding:5px 12px; font-size:13px; background:#e0e0e0; border-radius:4px;")
        self.cam3_btn.setToolTip(
            "开启后从板端第三路画面地址拉流(板端 show_cam.py 推流 / mjpeg_bridge)\n"
            f"取流地址: http://{target_ip}:{CAM3_PORT}{CAM3_PATH}")
        self.cam3_btn.clicked.connect(self._toggle_cam3)
        bar.addWidget(self.cam3_btn)

        # v3.4.1: 「1:1 原始尺寸」显示开关 —— 按帧的真实像素显示, 不做放大/平滑缩放;
        #   默认关(沿用"填满卡片"的自适应缩放)。开: 能看清 640×480 的每一个像素细节,
        #   代价是画面不满卡片(四周留黑), 小分辨率下显得小。用于排查/看细节。
        self.rawsize_btn = QPushButton("显示: 自适应缩放")
        self.rawsize_btn.setCheckable(True)
        self.rawsize_btn.setChecked(False)
        self.rawsize_btn.setStyleSheet(
            "padding:5px 12px; font-size:13px; background:#e0e0e0; border-radius:4px;")
        self.rawsize_btn.setToolTip(
            "切换画面显示方式:\n"
            "· 自适应缩放(默认): 4:3 等比放大填满卡片, 平滑插值, 观感柔和但低分辨率会糊\n"
            "· 1:1 原始尺寸: 按帧真实像素显示, 不放大不插值, 最能看清细节/检测框\n"
            "(板端 640×480 时, 1:1 会比卡片小, 四周留黑属正常)")
        self.rawsize_btn.toggled.connect(self._on_rawsize_toggled)
        bar.addWidget(self.rawsize_btn)
        bar.addStretch(1)
        hint = QLabel("画面 640×480 · 4:3 等比居中不变形")
        hint.setStyleSheet("font-size:12px; color:#6e7681;")
        bar.addWidget(hint)
        vlay.addLayout(bar)

        position_bar = QHBoxLayout()
        position_bar.addWidget(QLabel("位置视图"))
        self.position_mode_combo = QComboBox()
        self.position_mode_combo.addItems(["模式1：当前方向朝前", "模式2：固定开机方向"])
        self.position_mode_combo.currentIndexChanged.connect(lambda index: self.position_map.set_view_mode(index))
        position_bar.addWidget(self.position_mode_combo)
        reset_position_btn = QPushButton("重设位置零点")
        reset_position_btn.setToolTip("清空历史轨迹，以下一帧有效航向定义新原点/方向。")
        reset_position_btn.clicked.connect(self._reset_position)
        position_bar.addWidget(reset_position_btn)
        export_position_btn = QPushButton("导出轨迹")
        export_position_btn.clicked.connect(self._export_position)
        position_bar.addWidget(export_position_btn)
        position_bar.addStretch(1)
        position_bar.addWidget(QLabel("速度积分估算 · 当前位置居中 · 在位置图上滚轮缩放"))
        vlay.addLayout(position_bar)

        # --- 视频墙 ---
        self.cam_tiles = {
            "cam1": VideoTile("CAM1 前置"),
            "cam2": VideoTile("CAM2 底部"),
            "cam3": VideoTile("CAM3 备用"),
        }
        # 旧的 camX_lbl 仍指向卡片内的画面标签, 取流/录像/开关代码无需改动
        self.cam1_lbl = self.cam_tiles["cam1"].video
        self.cam2_lbl = self.cam_tiles["cam2"].video
        self.cam3_lbl = self.cam_tiles["cam3"].video
        self.cam1_lbl.setText("CAM1 前置 - 等待连接...")
        self.cam2_lbl.setText("CAM2 底部 - 等待连接...")
        self.cam3_lbl.setText("CAM3 备用 - 未开启取流")

        # v3.5: 航向罗盘 —— 只挂在 CAM1(主画面)右上角。尺寸 168 ≈ 下方日志宽度的 1.5 倍观感。
        self.compass = self.cam_tiles["cam1"].add_compass(168)

        wall = VideoWall()
        self.view_wall = wall                       # v3.4.1: 供"1:1/自适应"切换时重排
        self.view_grid = wall.grid
        self.view_grid.setAlignment(Qt.AlignCenter)   # 卡片在格内居中
        # 在重排时挂到左上角视频卡片，与实际画面左上角对齐。
        self.position_map = PositionMap(self._position, wall)
        vlay.addWidget(wall, 1)
        self._apply_view_layout(0)
        QTimer.singleShot(0, wall.fit_tiles)
        return page

    def _apply_view_layout(self, idx=0):
        """v3.3.1: 按预设重排视频墙(未入选的画面直接收起, 不再空占位)"""
        if not hasattr(self, "view_grid"):
            return
        grid = self.view_grid
        if not 0 <= idx < len(self._VIEW_PRESETS):
            idx = 0
        preset = self._VIEW_PRESETS[idx]
        for tile in self.cam_tiles.values():
            grid.removeWidget(tile)
            tile.hide()
        for key, r, c, rs, cs in preset["items"]:
            tile = self.cam_tiles[key]
            tile.setMaximumHeight(16777215)   # 先解除上一轮的高度收紧
            grid.addWidget(tile, r, c, rs, cs)
            tile.show()
        for i in range(4):
            grid.setColumnStretch(i, preset["col"].get(i, 0))
            grid.setRowStretch(i, preset["row"].get(i, 0))
        if hasattr(self, "position_map"):
            # 共用一份轨迹，切换 CAM1/CAM2 或单画面时移动浮层。
            old_parent = self.position_map.parentWidget()
            if old_parent.layout() is not None:
                old_parent.layout().removeWidget(self.position_map)
            anchor_key = min(preset["items"], key=lambda item: (item[1], item[2]))[0]
            anchor = self.cam_tiles[anchor_key]
            self.position_map.setParent(anchor)
            anchor.layout().addWidget(self.position_map, 0, 0, Qt.AlignTop | Qt.AlignLeft)
            self.position_map.show()
            self.position_map.raise_()
            for tile in self.cam_tiles.values():
                # 透明位置图下不叠放摄像头名称，避免两行文字重合。
                tile.layout().removeWidget(tile.badge_title)
                title_alignment = Qt.AlignBottom if tile is anchor else Qt.AlignTop
                tile.layout().addWidget(tile.badge_title, 0, 0, title_alignment | Qt.AlignLeft)
        self._update_video_tiles()      # 立刻刷新角标/描边状态
        wall = grid.parentWidget()
        if isinstance(wall, VideoWall):
            QTimer.singleShot(0, wall.fit_tiles)

    def _start_threads(self):
        self._joy_thread = JoystickThread()
        self._joy_thread.data_sig.connect(self._on_joy)
        self._joy_thread.status_sig.connect(self._on_status)
        self._joy_thread.btn_sig.connect(self._on_btn)
        self._joy_thread.net_tx_sig.connect(self._on_net_tx)
        self._joy_thread.start()

        self._tel_thread = TelemetryThread()
        self._tel_thread.tel_sig.connect(self._on_tel)
        self._tel_thread.raw_sig.connect(self._on_raw_tel)
        self._tel_thread.net_rx_sig.connect(self._on_net_rx)
        self._tel_thread.start()

        # v3.2.1: 高度计 CH348 接收线程 (UDP 8082)
        self._alt_thread = AltThread()
        self._alt_thread.alt_sig.connect(self._on_alt)
        self._alt_thread.start()

        # v3.5: AUV 状态提示接收线程 (UDP 8085) —— 板端在跑哪一段/报什么错, 直接顶到终端页
        self._msg_thread = MsgThread()
        self._msg_thread.msg_sig.connect(self._on_msg)
        self._msg_thread.start()
        self._auv_stage = "-"      # AUV 当前阶段(供日志与后续 UI 复用)

        # v3.2: 双路视频线程(带fps统计), 默认开启
        self._start_video_threads()

        # 延迟测量定时器 (仅有线模式有效)
        self._lat_timer = QTimer(self)
        self._lat_timer.timeout.connect(self._measure_latency)
        self._lat_timer.start(3000)

        # v3.2: 连接状态检查定时器(1s): 遥测超时 -> 断开
        self._conn_timer = QTimer(self)
        self._conn_timer.timeout.connect(self._update_conn_status)
        self._conn_timer.start(1000)

        # v3.2: 终端链路摘要定时器(5s)
        self._term_timer = QTimer(self)
        self._term_timer.timeout.connect(self._term_summary)
        self._term_timer.start(5000)

        # v3.3.1: 可视窗口画面卡片状态/角标刷新(1s)
        self._camfps_timer = QTimer(self)
        self._camfps_timer.timeout.connect(self._update_video_tiles)
        self._camfps_timer.start(1000)

    # v3.3: 视频线程启动/停止 + 第三路画面 CAM3
    def _start_video_threads(self):
        base = f"http://{target_ip}:{VIDEO_PORT}"
        self._vid1_thread = VideoThread(f"{base}/cam1", "cam1")
        self._vid1_thread.frame_sig.connect(lambda f: self._show_frame(self.cam1_lbl, f, cam_id=1))
        self._vid1_thread.fps_sig.connect(lambda name, fps: self._on_vid_fps(name, fps))
        self._vid1_thread.start()
        self._vid2_thread = VideoThread(f"{base}/cam2", "cam2")
        self._vid2_thread.frame_sig.connect(lambda f: self._show_frame(self.cam2_lbl, f, cam_id=2))
        self._vid2_thread.fps_sig.connect(lambda name, fps: self._on_vid_fps(name, fps))
        self._vid2_thread.start()
        # 第三路画面: 仅在用户开启时取流
        if self._cam3_on:
            self._start_cam3()

    def _stop_video_threads(self):
        for t in (getattr(self, "_vid1_thread", None), getattr(self, "_vid2_thread", None),
                  getattr(self, "_vid3_thread", None)):
            if t is not None:
                t.stop()
        # 不 wait() 阻塞UI, 线程最长0.5~1s后自行退出
        self._vid1_thread = None
        self._vid2_thread = None
        self._vid3_thread = None
        self.cam1_lbl.setText("CAM1 前置 - 视频已关闭")
        self.cam1_lbl.setPixmap(QPixmap())
        self.cam2_lbl.setText("CAM2 底部 - 视频已关闭")
        self.cam2_lbl.setPixmap(QPixmap())
        # 第三路: 保留用户开启意图(_cam3_on), 关闭期间显示待机提示
        self.cam3_lbl.setText("CAM3 备用 - 未开启取流" if not self._cam3_on
                              else "CAM3 备用 - 视频已关闭")
        self.cam3_lbl.setPixmap(QPixmap())
        self._vid_fps = {"cam1": 0, "cam2": 0, "cam3": 0}
        self._vid_recv_fps = {"cam1": 0, "cam2": 0, "cam3": 0}  # v3.2.2: 推流实际帧率
        self._vid_last_frame = {"cam1": 0.0, "cam2": 0.0, "cam3": 0.0}
        for _t in getattr(self, "cam_tiles", {}).values():
            _t.set_res(0, 0)         # v3.3.1: 断流后清掉旧的分辨率角标
        self._update_video_tiles()   # v3.3.1: 立刻把卡片置为“已关闭”状态

    # ---- 第三路画面 CAM3 ----
    def _toggle_cam3(self):
        """v3.3: 第三路画面取流开关(可视窗口页面按钮)"""
        if self._link_mode == 1:
            self._log("[WARN] 无线模式下无视频, 请先切回有线链路再开启 CAM3")
            return
        if not self._video_on:
            self._log("[WARN] 视频传输已关闭, 请先打开顶部“视频”开关再开启 CAM3")
            return
        self._cam3_on = not self._cam3_on
        if self._cam3_on:
            self._start_cam3()
        else:
            self._stop_cam3()
        self.cam3_btn.setText(f"CAM3 取流: {'开' if self._cam3_on else '关'}")
        self.cam3_btn.setStyleSheet(
            f"padding:5px 12px; font-size:13px; border-radius:4px; "
            f"background:{'#bbdefb' if self._cam3_on else '#e0e0e0'};")

    def _start_cam3(self):
        if getattr(self, "_vid3_thread", None) is not None:
            return
        url = f"http://{target_ip}:{CAM3_PORT}{CAM3_PATH}"
        self._vid3_thread = VideoThread(url, "cam3")
        self._vid3_thread.frame_sig.connect(lambda f: self._show_frame(self.cam3_lbl, f, cam_id=3))
        self._vid3_thread.fps_sig.connect(lambda name, fps: self._on_vid_fps(name, fps))
        self._vid3_thread.start()
        self.cam3_lbl.setText(f"CAM3 备用 - 等待连接... ({CAM3_PORT}{CAM3_PATH})")
        self._update_video_tiles()
        self._log(f"[VID] CAM3 取流已开启: {url}")

    def _stop_cam3(self):
        t = getattr(self, "_vid3_thread", None)
        if t is not None:
            t.stop()
        self._vid3_thread = None
        self.cam3_lbl.setText("CAM3 备用 - 未开启取流")
        self.cam3_lbl.setPixmap(QPixmap())
        self._vid_fps["cam3"] = 0
        self._vid_recv_fps["cam3"] = 0
        self._vid_last_frame["cam3"] = 0.0
        tile = getattr(self, "cam_tiles", {}).get("cam3")
        if tile is not None:
            tile.set_res(0, 0)
        self._update_video_tiles()
        self._log("[VID] CAM3 取流已关闭")

    def _update_video_tiles(self):
        """v3.3.1: 刷新三张画面卡片的描边状态与右下角角标(1s 周期 + 布局切换时)"""
        if not hasattr(self, "cam_tiles"):
            return
        now = time.time()
        for key, tile in self.cam_tiles.items():
            th = getattr(self, f"_vid{key[-1]}_thread", None)
            if th is None:
                tile.set_state("off")
                if key == "cam3" and not self._cam3_on:
                    tile.set_stat("未开启取流")
                elif self._link_mode == 1:
                    tile.set_stat("无线模式无视频")
                elif not self._video_on:
                    tile.set_stat("视频传输已关闭")
                else:
                    tile.set_stat("未连接")
                continue
            # 1.5s 内来过帧 = 有画面; 否则视为等待信号(线程在重试连接)
            if now - self._vid_last_frame.get(key, 0.0) < 1.5:
                tile.set_state("live")
                tile.set_stat(f"{self._vid_fps.get(key, 0)}fps"
                              f"(收{self._vid_recv_fps.get(key, 0)})")
            else:
                tile.set_state("wait")
                tile.set_stat("等待信号…")

    def _on_vid_fps(self, name, fps):
        self._vid_fps[name] = fps
        th = getattr(self, f"_vid{name[-1]}_thread", None)
        if th is not None:
            self._vid_recv_fps[name] = int(getattr(th, "recv_fps", 0))

    #  槽函数 
    # v3.2: 终端输出级别 -> 颜色
    _TERM_COLORS = {
        "INFO": "#c9d1d9",   # 常规灰白
        "NET":  "#58a6ff",   # 网络/链路 蓝
        "TEL":  "#3fb950",   # 遥测 绿
        "VID":  "#d2a8ff",   # 视频 紫
        "LORA": "#39c5cf",   # LoRa 青
        "WARN": "#d29922",   # 警告 黄
        "ERR":  "#f85149",   # 错误 红
    }

    def _term(self, msg, level="INFO"):
        """v3.2: 追加一条到终端页(按级别着色), 并按需自动滚动"""
        if not hasattr(self, "term_text"):
            return
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = self._TERM_COLORS.get(level, "#c9d1d9")
        # HTML 转义, 防止原始帧里的字符破坏格式
        msg_esc = (str(msg).replace("&", "&amp;").replace("<", "&lt;")
                   .replace(">", "&gt;"))
        self.term_text.append(
            f'<span style="color:#8b949e">[{ts}]</span> '
            f'<span style="color:{color};font-weight:bold">[{level:>4}]</span> '
            f'<span style="color:{color}">{msg_esc}</span>')
        if not self.term_pause_chk.isChecked():
            sb = self.term_text.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _log(self, msg):
        """追加一条带时间戳的日志到日志面板 + 终端页"""
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        # 自动滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())
        # v3.2: 同步进终端页, 按前缀分类着色
        if isinstance(msg, str):
            if msg.startswith("[ERR]") or msg.startswith("[!]"):
                self._term(msg, "ERR")
            elif msg.startswith("[WARN]"):
                self._term(msg, "WARN")
            elif msg.startswith("[LORA]"):
                self._term(msg, "LORA")
            elif msg.startswith("[NET]"):
                self._term(msg, "NET")
            elif msg.startswith("[VID]"):
                self._term(msg, "VID")
            else:
                self._term(msg, "INFO")

    def _term_summary(self):
        """v3.2: 每5秒向终端输出一次链路运行摘要"""
        if not self.term_summary_chk.isChecked():
            # 摘要关闭时只刷新底部状态行
            self._update_term_stat_lbl()
            return
        self._update_term_stat_lbl()
        if self._link_mode == 0:
            lat = f"{self._latency_ms:.0f}ms" if self._latency_ms >= 0 else "超时"
            _md = "待机" if getattr(self, "_frozen", False) else ("自主航行" if self._mode else "有线遥控")
            self._term(
                f"链路=有线 模式={_md} "
                f"遥测={self._tel_hz:.1f}Hz RTT={lat} "
                f"视频cam1={self._vid_fps['cam1']}fps(收{self._vid_recv_fps['cam1']}) "
                f"cam2={self._vid_fps['cam2']}fps(收{self._vid_recv_fps['cam2']}) "
                f"cam3={'关' if not self._cam3_on else str(self._vid_fps['cam3']) + 'fps(收' + str(self._vid_recv_fps['cam3']) + ')'} "
                f"视频传输={'开' if self._video_on else '关'}",
                "NET")
        else:
            com = self._joy_thread._lora_port or "未选"
            ser_ok = bool(self._joy_thread._lora_ser and self._joy_thread._lora_ser.is_open)
            self._term(
                f"链路=无线(LoRa) 串口={com}({'打开' if ser_ok else '未打开'}) @9600 5Hz "
                f"遥测/视频=不适用(纯遥控下行)",
                "LORA")

    def _update_term_stat_lbl(self):
        """v3.2: 终端底部常显链路摘要行"""
        tx_p = getattr(self, "_tx_pps", 0)
        rx_p = getattr(self, "_rx_pps", 0)
        if self._link_mode == 0:
            lat = f"{self._latency_ms:.0f}ms" if self._latency_ms >= 0 else "--"
            self.term_stat_lbl.setText(
                f"有线 | TX {tx_p}pps | RX {rx_p}pps | 遥测 {self._tel_hz:.1f}Hz | "
                f"RTT {lat} | CAM1 {self._vid_fps['cam1']}fps(收{self._vid_recv_fps['cam1']}) | "
                f"CAM2 {self._vid_fps['cam2']}fps(收{self._vid_recv_fps['cam2']})")
        else:
            ser_ok = bool(self._joy_thread._lora_ser and self._joy_thread._lora_ser.is_open)
            self.term_stat_lbl.setText(
                f"无线LoRa @9600 5Hz | 串口{'已打开' if ser_ok else '未打开'} | TX {tx_p}pps | "
                f"遥测/视频 不适用")

    def _on_status(self, msg):
        self.status_lbl.setText(msg)
        self._log(msg)

    # ==================== v3.2: 连接状态 / 串口 / 链路切换 / 视频开关 ====================
    def _update_conn_status(self):
        """每1s刷新连接状态指示:
        有线: 遥测1.5s内活跃=已连接(板+STM32全通); 仅PING通=板在线; 否则断开
        无线: LoRa串口打开=链路就绪(纯下行, 对端状态无法确认)"""
        now = time.perf_counter()
        if self._link_mode == 0:
            tel_fresh = (self._last_tel_time is not None
                         and now - self._last_tel_time < 1.5)
            ping_fresh = (self._last_ping_ok is not None
                          and now - self._last_ping_ok < 5.0)
            if tel_fresh:
                txt, bg, fg = "连接: 已连接 (板+STM32)", "#c8e6c9", "#1b5e20"
            elif ping_fresh:
                txt, bg, fg = "连接: 板在线 (无遥测)", "#fff9c4", "#827717"
            else:
                txt, bg, fg = "连接: 未连接", "#ffcdd2", "#b71c1c"
        else:
            ser_ok = bool(self._joy_thread._lora_ser and self._joy_thread._lora_ser.is_open)
            if ser_ok:
                com = self._joy_thread._lora_port
                txt, bg, fg = f"LoRa: {com} 已连接", "#b2ebf2", "#006064"
            else:
                txt, bg, fg = "LoRa: 串口未打开", "#ffcdd2", "#b71c1c"
        self.conn_lbl.setText(txt)
        self.conn_lbl.setStyleSheet(
            f"font-weight:bold; font-size:13px; padding:4px 10px; "
            f"background:{bg}; color:{fg}; border-radius:4px;")
        # 无线模式时顶部网络/延迟标签置灰显示
        if self._link_mode == 1:
            self.net_lbl.setText("网络: 无线模式(不适用)")
            self.latency_lbl.setText("RTT: 无线模式(不适用)")

    def _refresh_com_ports(self):
        """v3.2: 枚举本机可用串口填充下拉框"""
        self.com_combo.blockSignals(True)
        self.com_combo.clear()
        if list_ports is None:
            self.com_combo.addItem("(未安装pyserial)")
            self.com_combo.blockSignals(False)
            return
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            self.com_combo.addItem("(无可用串口)")
        else:
            self.com_combo.addItem("(选择COM口)")
            for p in sorted(ports):
                self.com_combo.addItem(p)
        self.com_combo.blockSignals(False)

    def _on_com_selected(self, text):
        """v3.2: 选择地面LoRa串口"""
        if not text or text.startswith("("):
            return
        self._joy_thread._lora_port = text
        # 若串口已开, 先关掉让发送循环用新口重开
        if self._joy_thread._lora_ser:
            self._joy_thread._close_lora()
        self._log(f"[LORA] 已选择地面LoRa串口: {text} (波特率9600)")

    def _toggle_link_mode(self):
        """v3.2: 有线/无线链路切换按钮"""
        new_mode = 1 - self._link_mode
        self._joy_thread.set_link_mode(new_mode)
        self._link_mode = new_mode
        if new_mode == 1:
            # 切到无线: 强制停视频(本地断流+通知板端), 但保留 _video_on 用户意图, 切回有线时恢复
            self.link_btn.setText("链路: 无线 [LoRa]")
            self.link_btn.setStyleSheet("padding:5px 12px; font-size:13px; font-weight:bold; background:#b2ebf2; border-radius:4px;")
            self._stop_video_threads()
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(build_vid(False).encode(), (target_ip, CMD_PORT))
                s.close()
            except Exception:
                pass
            self.video_btn.setText("视频: 关(无线)")
            self.video_btn.setStyleSheet("padding:5px 12px; font-size:13px; background:#e0e0e0; border-radius:4px;")
            self._log("[NET] 已切换到无线模式: 遥控指令走LoRa(9600, 5Hz), 遥测/视频不可用")
        else:
            # 切到有线: 恢复视频(若开关意图是开)
            self.link_btn.setText("链路: 有线 [网线]")
            self.link_btn.setStyleSheet("padding:5px 12px; font-size:13px; font-weight:bold; background:#c8e6c9; border-radius:4px;")
            if self._video_on:
                self._set_video_stream(True, notify_board=True)
            else:
                self.video_btn.setText("视频: 关")
            self._log("[NET] 已切换到有线模式: 指令/遥测/视频全走网线")

    def _toggle_video_stream(self):
        """v3.2: 视频传输开关(顶部按钮)"""
        if self._link_mode == 1:
            self._log("[WARN] 无线模式下无视频, 请先切回有线链路再操作")
            return
        self._set_video_stream(not self._video_on, notify_board=True)

    def _set_video_stream(self, on, notify_board=True):
        """v3.2: 视频传输开关核心实现
        on=True: 重启本地取流线程; on=False: 停止取流
        notify_board: 同时发 $VID 帧通知板端停/启推流(省板端CPU/带宽)"""
        self._video_on = on
        if on:
            self._start_video_threads()
            self.cam1_lbl.setText("CAM1 前置 - 等待连接...")
            self.cam2_lbl.setText("CAM2 底部 - 等待连接...")
            # v3.3: 第三路保持用户意图; 未开启时显示待机提示
            if not self._cam3_on:
                self.cam3_lbl.setText("CAM3 备用 - 未开启取流")
            self._update_video_tiles()   # v3.3.1: 立即刷新卡片状态角标
        else:
            self._stop_video_threads()
        if notify_board:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(build_vid(on).encode(), (target_ip, CMD_PORT))
                s.close()
                self._log(f"[VID] 已通知板端{'开始' if on else '停止'}推流 ($VID,{1 if on else 0}#)")
            except Exception as e:
                self._log(f"[WARN] $VID 帧发送失败(板端可能离线): {e}")
        self.video_btn.setText(f"视频: {'开' if on else '关'}")
        self.video_btn.setStyleSheet(
            f"padding:5px 12px; font-size:13px; border-radius:4px; "
            f"background:{'#bbdefb' if on else '#e0e0e0'};")

    def _on_btn(self, name):
        btn_names = {"grab": "执行抓球动作 (A)", "throw": "执行抛球动作 (B)",
                     "record": "录像开关 (X)", "store": "数据存储开关 (Y)"}
        self._log(btn_names.get(name, name))
        if name == "grab":
            self._grab = True
            self._joy_thread._grab = 1
            QTimer.singleShot(500, self._reset_grab)
        elif name == "throw":
            self._grab = True
            self._joy_thread._grab = 2
            QTimer.singleShot(500, self._reset_grab)
        elif name == "record":
            self.rec_btn.click()
        elif name == "store":
            self.store_btn.click()

    def _reset_grab(self):
        self._grab = False
        self._joy_thread._grab = 0

    def _get_record_dir(self):
        base_dir = pathlib.Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) \
            else pathlib.Path(__file__).resolve().parent
        rec_dir = base_dir / "recordings"
        rec_dir.mkdir(parents=True, exist_ok=True)
        return rec_dir

    # --- LED 滑块鼠标拖动回调 ---
    def _on_led1_slider(self, val):
        self._joy_thread._led1 = val
        self.led1_val.setText(str(val))
        self.ctrl_labels["led1"].setText(str(val))

    def _on_led2_slider(self, val):
        self._joy_thread._led2 = val
        self.led2_val.setText(str(val))
        self.ctrl_labels["led2"].setText(str(val))

    # --- 操作按钮回调 ---
    def _set_mode(self, mode):
        """显式设置 S100 模式: 0=有线遥控(ROV), 1=自主航行(AUV)。
        与手柄 LB 切换共用 _joy_thread._mode, 立刻重绘按钮高亮并补发一帧模式帧。

        v3.4: 点模式按钮视为"主动接管", 顺带清除急停待机标志。
        v3.8(方案A): 急停锁存中按模式按钮 = 先解除板端锁存再切模式。
        S100 的 estop_latch 不会因 $CMD 自动清除(0x09 一直抑制), 仅清 UI 标志会
        造成"PC 显示已切回、板端仍锁存"的不一致; 故先补发 $ESTOP,0# 让 S100 下发
        START 恢复 Last_Con_Mode, 再切目标模式(无线模式 send_estop_release 内部跳过)。
        """
        mode = 1 if mode else 0
        jt = getattr(self, "_joy_thread", None)
        if jt is None:
            return
        if getattr(self, "_frozen", False):
            # 方案A: 急停锁存中按模式按钮 = 主动接管, 先解除板端锁存
            try:
                if jt.send_estop_release():
                    self._log("[ESTOP] 急停锁存中按模式按钮 -> 已发 $ESTOP,0# 解除锁存")
            except Exception as e:
                self._log(f"[WARN] 解除急停锁存发送失败: {e}")
            self._frozen = False
        jt._mode = mode
        self._mode = mode
        self._paint_mode()
        self._log(f"模式切换 -> {'自主航行(AUV)' if mode else '有线遥控(ROV)'}")
        # 立即补发一帧含新模式的 $CMD(零杆位=最安全), 无需等手柄循环
        self._send_mode_frame_now()

    def _click_estop(self):
        """v3.3(决策 B): 急停 = 零杆位 $CMD(先停推) + 显式 $ESTOP#(板端进 STANDBY 并锁存)。"""
        jt = getattr(self, "_joy_thread", None)
        try:
            if jt is not None:
                if self._link_mode == 1:
                    jt._lora_emergency_stop()
                else:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.sendto(build_emergency_stop().encode(), (target_ip, CMD_PORT))
                    s.close()
        except Exception as e:
            self._log(f"[WARN] 急停(零杆位)发送失败: {e}")
        ok = False
        try:
            ok = bool(jt is not None and jt.send_estop())
        except Exception as e:
            self._log(f"[WARN] $ESTOP# 发送失败: {e}")
        self._log("[ESTOP] 已发急停: 零杆位 $CMD + $ESTOP#"
                  if ok else "[ESTOP] 已发急停: 零杆位 $CMD（$ESTOP# 未发出: 无线模式或失败）")
        # v3.4: 置"待机"标志 —— S100 收 $ESTOP# 后进 STANDBY, UI 自行记录以显示「模式: 待机」
        if ok:
            self._frozen = True
            self._paint_mode()

    def _click_estop_release(self):
        """v3.4: 「温启动」—— 发 $ESTOP,0#，S100 转 V2 0x04 0x00(START)，
        从 STANDBY 恢复到急停前的模式(Last_Con_Mode)。原按钮名「解除急停」。"""
        jt = getattr(self, "_joy_thread", None)
        ok = False
        try:
            ok = bool(jt is not None and jt.send_estop_release())
        except Exception as e:
            self._log(f"[WARN] $ESTOP,0# 发送失败: {e}")
        self._log("[ESTOP] 已发温启动 $ESTOP,0#（STANDBY → 恢复到急停前模式）" if ok
                  else "[ESTOP] 温启动未发出: 无线模式或发送失败")
        # v3.4: 清除"待机"标志, 恢复显示急停前的模式
        if ok:
            self._frozen = False
            self._paint_mode()

    def _send_mode_frame_now(self):
        """立即向 S100 发送一帧携带新模式的 $CMD(零杆位, 有线UDP/无线LoRa)。"""
        try:
            jt = self._joy_thread
            if jt is None:
                return
            frame = build_cmd(0.0, 0.0, 0.0, 0.0,
                              jt._led1, jt._led2,
                              jt._mode, jt._grab, jt._store)
            if jt._link_mode == 1:
                ser = jt._ensure_lora()
                if ser:
                    ser.write(frame.encode())
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(frame.encode(), (target_ip, CMD_PORT))
                s.close()
        except Exception as e:
            self._log(f"[WARN] 模式帧发送失败: {e}")

    def _paint_mode(self):
        """模式显示的唯一刷新入口: 左上角模式标签 + 两个模式按钮的互斥高亮。
        按钮点击(_set_mode)、手柄 LB(_on_joy)、启动初始化(_build_ui) 共用,
        保证三处状态永远一致。

        v3.4: 新增「待机」态 —— 点「急停」后 S100 进 STANDBY, 但 S100 不主动上报
        模式(没有模式回传通道), 故由 UI 自己用 _frozen 标志记录; 「温启动」清除。
        """
        jt = getattr(self, "_joy_thread", None)
        m = 1 if (jt is not None and jt._mode) else getattr(self, "_mode", 0)
        frozen = bool(getattr(self, "_frozen", False))
        if frozen:
            # 待机: 急停锁存中, 模式未知(等温启动恢复)
            self.mode_lbl.setText("模式: 待机")
            self.mode_lbl.setStyleSheet(
                "font-weight:bold; font-size:14px; padding:4px 12px; "
                "background:#ffcdd2; color:#b71c1c; border-radius:4px;")
        else:
            self.mode_lbl.setText("模式: 有线遥控" if m == 0 else "模式: 自主航行")
            self.mode_lbl.setStyleSheet(
                "font-weight:bold; font-size:14px; padding:4px 12px; "
                "background:%s; border-radius:4px;" % ("#c8e6c9" if m == 0 else "#bbdefb"))
        self.rov_btn.setChecked(m == 0 and not frozen)
        self.auv_btn.setChecked(m == 1 and not frozen)
        self.rov_btn.setStyleSheet(
            "padding:8px 10px; font-size:13px; border-radius:4px; "
            + ("background:#c8e6c9; font-weight:bold;" if (m == 0 and not frozen) else "background:#e0e0e0;"))
        self.auv_btn.setStyleSheet(
            "padding:8px 10px; font-size:13px; border-radius:4px; "
            + ("background:#bbdefb; font-weight:bold;" if (m == 1 and not frozen) else "background:#e0e0e0;"))

    def _click_grab(self, action):
        """action: 1=抓球, 2=抛球"""
        self._joy_thread._grab = action
        self._log("执行抓球动作 (A)" if action == 1 else "执行抛球动作 (B)")
        QTimer.singleShot(500, self._reset_grab)

    def _on_joy(self, d):
        """手柄数据 -> 更新摇杆可视化 + 控制值"""
        surge, sway = d["surge"], d["sway"]
        heave, yaw = d["heave"], d["yaw"]
        ry = d.get("ry", 0.0)

        # 左摇杆 2D 图
        self.stick_widget.set_pos(sway, -surge)  # x=sway, y轴翻转(推杆向前=负)
        self.surge_val_lbl.setText(f"Surge: {surge:+.2f}")
        self.sway_val_lbl.setText(f"Sway: {sway:+.2f}")

        # 右摇杆 / 扳机 横条
        self.yaw_bar.set_value(yaw)
        self.heave_bar.set_value(heave)
        # v3.3: 右摇杆 Y 已改作 Heave, 第三根横条改显示 RT/LT 扳机差值(当前预留)
        self.rsticky_bar.set_value(d.get("rt", 0.0) - d.get("lt", 0.0))

        # 指令值面板
        self.ctrl_labels["surge"].setText(f'{surge:.2f}')
        self.ctrl_labels["sway"].setText(f'{sway:.2f}')
        self.ctrl_labels["heave"].setText(f'{heave:.2f}')
        self.ctrl_labels["yaw"].setText(f'{yaw:.2f}')
        self.ctrl_labels["led1"].setText(str(d["led1"]))
        self.ctrl_labels["led2"].setText(str(d["led2"]))
        self.ctrl_labels["grab"].setText(str(d["grab"]))
        self.ctrl_labels["store"].setText(str(d["store"]))

        # LED 滑块同步 (手柄 D-pad 调节时同步到滑块)
        self.led1_slider.blockSignals(True)
        self.led1_slider.setValue(d["led1"])
        self.led1_slider.blockSignals(False)
        self.led1_val.setText(str(d["led1"]))
        self.led2_slider.blockSignals(True)
        self.led2_slider.setValue(d["led2"])
        self.led2_slider.blockSignals(False)
        self.led2_val.setText(str(d["led2"]))
        # 模式文字
        m = d.get("mode", 0)
        if m != self._mode:
            self._mode = m
            self._paint_mode()

    def _on_raw_tel(self, text):
        """v3.2: 遥测原始帧 -> 终端(勾选'显示原始遥测帧'时) + 遥测帧率统计"""
        ack = parse_task_pid_ack(text)
        if ack is not None:
            self._on_task_pid_ack(ack)
            return
        # 帧率统计(所有帧都计)
        self._tel_cnt_window += 1
        now = time.perf_counter()
        if not hasattr(self, "_tel_t0") or self._tel_t0 is None:
            self._tel_t0 = now
        elif now - self._tel_t0 >= 1.0:
            self._tel_hz = self._tel_cnt_window / (now - self._tel_t0)
            self._tel_cnt_window = 0
            self._tel_t0 = now
        # 终端原始帧显示
        if (hasattr(self, "term_rawtel_chk") and self.term_rawtel_chk.isChecked()):
            self._term(text.strip(), "TEL")

    def _on_tel(self, t):
        """遥测数据 -> 更新姿态/电池/推进器, 并刷新曲线与数据记录"""
        # v3.2: 记录最后收到遥测时间(连接状态判断依据)
        self._last_tel_time = time.perf_counter()
        self._position.update(t)
        self.position_map.update()
        if "t_depth" in t:
            self._reported_target_depth = t["t_depth"]
        target = self._reported_target_depth
        actual = t.get("depth")
        self.depth_feedback_lbl.setText(
            "回传目标: " + (f"{target:.2f} m" if target is not None else "--")
            + " · 实际深度: " + (f"{actual:.2f} m" if actual is not None else "--"))
        # 实际值: 角度/角速度/加速度/深度/高度/线速度
        act_map = {"roll":"Roll","pitch":"Pitch","yaw":"Yaw",
                   "gx":"GyroX","gy":"GyroY","gz":"GyroZ",
                   "ax":"AccX","ay":"AccY","az":"AccZ",
                   "depth":"Depth","alt":"Alt",
                   "fused_depth":"板端融合深度","clearance":"离底净空",
                   "vx":"VelX","vy":"VelY","vz":"VelZ"}
        for tk, n in act_map.items():
            if tk in t and n in self.att_labels:
                self.att_labels[n][1].setText(f'{t[tk]:.2f}')
        # 目标值
        tgt_map = {"t_roll":"Roll","t_pitch":"Pitch","t_yaw":"Yaw",
                   "t_gx":"GyroX","t_gy":"GyroY","t_gz":"GyroZ",
                   "t_ax":"AccX","t_ay":"AccY","t_az":"AccZ",
                   "t_depth":"Depth","t_alt":"Alt",
                   "t_vx":"VelX","t_vy":"VelY","t_vz":"VelZ"}
        for tk, n in tgt_map.items():
            if tk in t and n in self.att_labels:
                self.att_labels[n][0].setText(f'{t[tk]:.2f}')
        # 电池 / 温度
        bat_map = {"batt_v":"volt","batt_a":"curr","batt_soc":"pct",
                   "batt_temp":"temp_water","cabin_temp":"temp_internal"}
        for tk, uk in bat_map.items():
            if tk in t and uk in self.bat_labels:
                self.bat_labels[uk].setText(f'{t[tk]:.1f}')
        # 推进器 (12路, 协议值为 [-1,1], 显示为百分比)
        thr_list = t.get("thrusters", [])
        for i in range(THRUSTER_COUNT):
            if i < len(thr_list) and i < len(self.thruster_bars):
                self.thruster_bars[i].set_value(thr_list[i] * 100)
        # v3.3.3: 卡尔曼融合深度 —— 由 DepthKalmanFilter 计算, 不是遥测原始值
        kd = self._kf_depth.update(t.get("depth"), t.get("vz"), t.get("alt"))
        self._kf_value = kd
        if kd is not None and "Kalman深度" in self.att_labels:
            self.att_labels["Kalman深度"][1].setText(f'{kd:.2f}')
        # v3.5: 航向罗盘 —— 喂入 $TEL 的 yaw(度)。绝对模式跟随真值, 相对模式显示差值。
        if "yaw" in t and getattr(self, "compass", None) is not None:
            self.compass.set_heading(t["yaw"])
        # 刷新实时曲线(仅函数图页) + 数据记录
        ch = self._channel_values(t)
        if kd is not None:
            ch["kdepth"] = kd        # 融合深度一并进曲线与 CSV
        self._plot_board.feed(ch)
        self._write_csv_row(ch)

    def _on_alt(self, d):
        """v3.2.1: 高度计(CH348)帧 -> '全部'页高度计区 (状态着色, 数值 mm)"""
        row = self._alt_labels.get(d.get("ch"))
        if row is None:
            return
        st_lbl, v_lbl = row
        st = d.get("status", "ERR")
        # v3.3.2: 着色时沿用统一字号(STY_TEXT), 不再用固定 px
        if st == "OK" and d.get("mm") is not None:
            st_lbl.setText("正常"); st_lbl.setStyleSheet(STY_TEXT + "color:#2e7d32;")
            v_lbl.setText(f'{d["mm"]} mm')
        elif st == "EMPTY":
            st_lbl.setText("未接设备"); st_lbl.setStyleSheet(STY_TEXT + "color:#999;")
            v_lbl.setText("--")
        else:
            st_lbl.setText("异常"); st_lbl.setStyleSheet(STY_TEXT + "color:#c62828;")
            v_lbl.setText("--")

    # --- v3.5: AUV 状态提示 ($MSG) ---
    # 级别 -> 终端页配色(_TERM_COLORS 里已有的键直接复用)
    _MSG_LEVEL = {"INFO": "INFO", "WARN": "WARN", "ERROR": "ERR"}

    def _on_msg(self, d):
        """v3.5: 板端 AUV 状态提示 -> 终端页(按级别染色) + 日志面板

        显示成: [AUV][STAGE] SEEK_BALL_F: 前视找球
        """
        lv = self._MSG_LEVEL.get(d.get("level", "INFO"), "INFO")
        code = d.get("code", "-")
        stage = d.get("stage", "-") or "-"
        text = d.get("text", "")
        if stage and stage != "-":
            self._auv_stage = stage
        line = f"[AUV][{code}] {stage}: {text}" if text else f"[AUV][{code}] {stage}"
        self._term(line, lv)
        # 日志面板同步一份(便于事后回看); ERROR/WARN 加前缀, 与既有 _log 口径一致
        if lv == "ERR":
            self._log(f"[ERR] {line}")
        elif lv == "WARN":
            self._log(f"[WARN] {line}")
        else:
            self._log(line)

    def _channel_values(self, t):
        """把遥测字典转成一条扁平通道数据 (供曲线/CSV 用)"""
        ch = {}
        for k in ["roll","pitch","yaw","gx","gy","gz","ax","ay","az",
                  "depth","alt","vx","vy","vz",
                  "t_roll","t_pitch","t_yaw","t_gx","t_gy","t_gz",
                  "t_ax","t_ay","t_az","t_depth","t_alt","t_vx","t_vy","t_vz"]:
            if k in t:
                ch[k] = t[k]
        if self._local_target_depth is not None:
            ch["local_t_depth"] = self._local_target_depth
        if self._position.base_heading is not None:
            ch.update(position_x=self._position.x, position_y=self._position.y,
                      position_heading=self._position.heading)
        thr = t.get("thrusters", [])
        for i in range(THRUSTER_COUNT):
            ch[f"thr{i}"] = thr[i] if i < len(thr) else 0.0
        return ch

    def _write_csv_row(self, ch):
        """若正在存储, 追加一行遥测数据到 CSV"""
        if not (self._csv_writer and self._csv_file):
            return
        try:
            row = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")]
            for fld in self._csv_fields:
                row.append(ch.get(fld, ""))
            self._csv_writer.writerow(row)
        except Exception as e:
            self._log(f"CSV 写入失败: {e}")

    def _set_target_depth(self):
        """仅设定 PC 本地目标；与板端回传目标分别显示/记录。"""
        self.depth_input.interpretText()
        self._local_target_depth = self.depth_input.value()
        self.depth_status_lbl.setText(f"本地目标: {self._local_target_depth:.2f} m · 尚未下发")
        self._log(f"[DEPTH] 本地目标设为 {self._local_target_depth:.2f} m；尚未下发到 S100/STM32")

    def _refresh_pid_ports(self):
        combo = self._pid_panel.port_combo
        selected = combo.currentData()
        combo.clear()
        if list_ports is not None:
            try:
                for port in list_ports.comports():
                    combo.addItem(f"{port.device} · {port.description}", port.device)
            except Exception as exc:
                self._pid_panel.link_status.setText("串口枚举失败：" + str(exc))
        if combo.count() == 0:
            combo.addItem("无可用串口", None)
        if selected:
            index = combo.findData(selected)
            if index >= 0:
                combo.setCurrentIndex(index)

    def _toggle_pid_serial(self):
        if self._pid_ser is not None:
            self._close_pid_serial()
            return
        panel = self._pid_panel
        port = panel.port_combo.currentData()
        if serial is None or not port:
            panel.link_status.setText("未连接：请安装 pyserial 并选择可用串口")
            return
        joy = getattr(self, "_joy_thread", None)
        if self._link_mode == 1 and joy is not None and port == getattr(joy, "_lora_port", None):
            panel.link_status.setText("未连接：该串口已用于 LoRa 控制链路")
            return
        try:
            self._pid_ser = serial.Serial(port, int(panel.baud_combo.currentText()),
                                          timeout=0, write_timeout=0.5)
            self._pid_rx_buffer = ReferenceTelemetryBuffer()
            self._pid_reply_count = 0
            panel.port_combo.setEnabled(False)
            panel.baud_combo.setEnabled(False)
            panel.serial_btn.setText("断开 PID 串口")
            panel.link_status.setText(f"PID 串口已连接：{port}，等待参考协议遥测应答")
            self._log(f"[PID] 串口已连接：{port} @ {panel.baud_combo.currentText()}")
        except Exception as exc:
            self._pid_ser = None
            panel.link_status.setText("PID 串口连接失败：" + str(exc))
            self._log(f"[PID] 串口连接失败: {exc}")

    def _close_pid_serial(self):
        connection, self._pid_ser = self._pid_ser, None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if hasattr(self, "_pid_panel"):
            self._pid_panel.port_combo.setEnabled(True)
            self._pid_panel.baud_combo.setEnabled(True)
            self._pid_panel.serial_btn.setText("连接 PID 串口")
            self._pid_panel.link_status.setText("PID 串口已断开")
            self._pid_panel.mapping_confirm.setChecked(False)

    def _poll_pid_serial(self):
        connection = self._pid_ser
        if connection is None:
            return
        try:
            waiting = min(connection.in_waiting, 4096)
            if waiting <= 0:
                return
            frames = self._pid_rx_buffer.feed(connection.read(waiting))
            if frames:
                self._pid_reply_count += len(frames)
                self._pid_panel.link_status.setText(
                    f"收到参考遥测应答 {self._pid_reply_count} 帧 · 应答不含 PID 参数回读")
        except Exception as exc:
            self._close_pid_serial()
            self._pid_panel.link_status.setText("PID 串口读入失败：" + str(exc))
            self._log(f"[PID] 串口读入失败: {exc}")

    def _send_control_pid(self, key, index, p, i, d):
        """仅由逐项确认触发；原样发送参考二进制帧，不发送 $PID 文本。"""
        panel = self._pid_panel
        row = panel._rows.get(key)
        if row is None or not row["send"].isEnabled():
            panel.mark_result(key, False, "未解锁下发")
            return
        if key == 'gate':
            self._send_task_pid(key, index, p, i, d)
            return
        expected = {'pitch': 0, 'yaw': 1, 'roll': 2, 'depth': 3}.get(key)
        if expected is None or index != expected:
            panel.mark_result(key, False, '执行端或编号不匹配，未发送')
            return
        transport = panel.transport_combo.currentIndex()
        try:
            frame = build_pid(index, p, i, d)
            if transport == 1:
                if self._pid_ser is None or not self._pid_ser.is_open:
                    panel.mark_result(key, False, "请先连接 PID 串口")
                    return
                written = self._pid_ser.write(frame)
                if written != len(frame):
                    raise IOError("PID 帧写入不完整")
            else:
                if self._link_mode != 0:
                    panel.mark_result(key, False, "UDP 下发需切回有线链路")
                    return
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
                    written = connection.sendto(frame, (target_ip, CMD_PORT))
                if written != len(frame):
                    raise IOError("PID 数据报提交不完整")
            panel.mark_result(key, True)
            self._log(f"[PID] 已提交 {row['title']} 编号={index} P={p:.2f} I={i:.2f} D={d:.2f} "
                      f"[{frame.hex(' ').upper()}]；未回读确认参数")
        except Exception as exc:
            panel.mark_result(key, False, "发送失败：" + str(exc))
            self._log(f"[PID] {row['title']} 发送失败: {exc}")

    def _send_task_pid(self, key, index, p, i, d):
        panel = self._pid_panel
        if key != 'gate' or index != -1 or panel.transport_combo.currentIndex() != 0 or self._link_mode != 0:
            panel.mark_result(key, False, '任务参数只支持有线S100 UDP')
            return
        request = uuid.uuid4().hex
        try:
            frame = build_task_pid(request, key, p, i, d)
            self._pid_pending[request] = (key, (p, i, d), time.monotonic() + 2.0)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
                if connection.sendto(frame, (target_ip, CMD_PORT)) != len(frame):
                    raise IOError('任务PID数据报提交不完整')
            panel.mark_task_pending(key)
            self._log('[TASKPID] 过门参数已提交，等待S100确认')
        except Exception as e:
            self._pid_pending.pop(request, None)
            panel.mark_task_result(key, '发送失败：' + str(e))

    def _on_task_pid_ack(self, ack):
        pending = self._pid_pending.get(ack['request'])
        if pending is None:
            return  # 不接受过期/重复/无对应请求的确认。
        key, gains, deadline = pending
        if ack['loop'] != key:
            return
        if time.monotonic() >= deadline:
            self._poll_pid_timeouts()
            return
        if ack['ok'] and tuple(ack['gains']) != tuple(gains):
            return
        self._pid_pending.pop(ack['request'])
        if ack['ok']:
            current = tuple(spin.value() for spin in self._pid_panel._rows[key]['gains'])
            message = 'S100已确认参数更新 · 未启动任务'
            if current != tuple(gains):
                message = 'S100已确认上一组 · 当前修改未下发'
            self._pid_panel.mark_task_result(key, message, True)
        else:
            self._pid_panel.mark_task_result(key, 'S100拒绝：' + ack['error'])

    def _poll_pid_timeouts(self):
        now = time.monotonic()
        for request, (key, _gains, deadline) in list(self._pid_pending.items()):
            if now >= deadline:
                self._pid_pending.pop(request)
                self._pid_panel.mark_task_result(key, '确认超时 · 参数是否更新未知')

    def set_depth_sender(self, sender):
        """预留接入点：sender(depth_m)->bool，True 仅表示已提交，不代表下位机确认。

        未提供真实 sender 时发送按钮禁用。协议/链路尚未约定，不自动生成新报文。
        """
        if sender is not None and not callable(sender):
            raise TypeError("depth sender 必须为可调用对象或 None")
        self._depth_sender = sender
        self.depth_send_btn.setEnabled(sender is not None)
        self.depth_send_btn.setText("发送目标" if sender is not None else "发送目标（待接入）")
        self.depth_send_btn.setToolTip("提交目标深度，执行结果以板端回传为准。" if sender is not None
                                      else "当前 S100/STM32 目标下发接口尚未接入；本地设定不发送到机器。")

    def _send_target_depth(self):
        if self._depth_sender is None:
            self._log("[DEPTH] 未发送：S100/STM32 目标下发接口尚未接入")
            return
        self._set_target_depth()
        try:
            sent = bool(self._depth_sender(self._local_target_depth))
        except Exception as e:
            self.depth_status_lbl.setText("发送失败 · 本地目标保留")
            self._log(f"[DEPTH] 目标深度发送失败: {e}")
            return
        if sent:
            self.depth_status_lbl.setText(f"本地目标: {self._local_target_depth:.2f} m · 已提交，待回传核对")
            self._log(f"[DEPTH] 已提交目标 {self._local_target_depth:.2f} m；请核对板端回传目标")
        else:
            self.depth_status_lbl.setText("未发送成功 · 本地目标保留")
            self._log("[DEPTH] 目标深度未发送成功")

    def _reset_position(self):
        self._position.reset()
        self.position_map.update()
        self._log("[POS] 位置和轨迹已清零；下一帧有效航向定义新零点/初始方向")

    def _export_position(self):
        from PyQt5.QtWidgets import QFileDialog
        if self._position.base_heading is None:
            self._log("[POS] 尚无位置记录，请先接收有效遥测")
            return
        default = str(self._get_record_dir() / f"position_{datetime.now():%Y%m%d_%H%M%S}.csv")
        path, _ = QFileDialog.getSaveFileName(self, "导出估算轨迹", default, "CSV (*.csv)")
        if not path:
            return
        try:
            points = list(self._position.history)
            points.append((time.monotonic(), self._position.x, self._position.y, self._position.heading))
            start_time = next(point[0] for point in points if point is not None)
            with open(path, "w", encoding="utf-8-sig", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["elapsed_s", "x_m", "y_m", "yaw_deg", "initial_yaw_deg", "segment"])
                segment = 0
                for point in points:
                    if point is None:
                        segment += 1
                        continue
                    stamp, x, y, heading = point
                    writer.writerow([f"{stamp - start_time:.3f}", f"{x:.4f}", f"{y:.4f}",
                                     f"{heading:.2f}", f"{self._position.base_heading:.2f}", segment])
            self._log(f"[POS] 估算轨迹已导出: {path}")
        except Exception as e:
            self._log(f"[POS] 轨迹导出失败: {e}")

    def _on_rawsize_toggled(self, checked):
        """v3.4.1: 切换"1:1 原始尺寸 / 自适应缩放"显示方式。"""
        self._rawsize_1to1 = bool(checked)
        if checked:
            self.rawsize_btn.setText("显示: 1:1 原始尺寸")
            self.rawsize_btn.setStyleSheet(
                "padding:5px 12px; font-size:13px; background:#2ea043; "
                "color:#fff; border-radius:4px;")   # 开=绿色高亮
            self._log("[VID] 画面显示切换为 1:1 原始尺寸(不缩放, 看得更清)")
        else:
            self.rawsize_btn.setText("显示: 自适应缩放")
            self.rawsize_btn.setStyleSheet(
                "padding:5px 12px; font-size:13px; background:#e0e0e0; border-radius:4px;")
            self._log("[VID] 画面显示切换为 自适应缩放(4:3 填满卡片)")
        # v3.4.1: 1:1 模式下把标签最小尺寸抬到当前帧的真实像素, 保证卡片容得下整帧
        #   (否则卡片比 640×480 窄时会裁掉一部分); 自适应模式恢复原下限。
        for key in ("cam1", "cam2", "cam3"):
            tile = getattr(self, "cam_tiles", {}).get(key)
            if tile is None:
                continue
            if checked:
                fw, fh = tile._fw, tile._fh    # 最近一帧的真实像素
                if fw > 0 and fh > 0:
                    tile.video.setMinimumSize(fw, fh)
            else:
                tile.video.setMinimumSize(240, 180)
        # v3.4.1: 通知视频墙重排(1:1 放开卡片高度 / 自适应收高度)
        wall = getattr(self, "view_wall", None)
        if wall is not None:
            QTimer.singleShot(0, wall.fit_tiles)

    def _show_frame(self, lbl, frame, cam_id=0):
        """视频帧 -> 显示到指定 QLabel; cam_id=1/2/3 时按需写入对应录像文件
        v3.2: 双摄各自独立录像; writer按实际帧尺寸动态创建(避免尺寸不匹配花屏)
        v3.3: 新增第三路 cam3; 页面不可见时跳过渲染(省CPU), 录像照常"""
        if cam_id in (1, 2, 3):
            # v3.3.1: 记录最近一帧时间与实际分辨率(角标显示用)
            key = f"cam{cam_id}"
            self._vid_last_frame[key] = time.time()
            tile = getattr(self, "cam_tiles", {}).get(key)
            if tile is not None:
                tile.set_res(frame.shape[1], frame.shape[0])
            if self._recording:
                writer = {1: self._rec_writer1, 2: self._rec_writer2, 3: self._rec_writer3}[cam_id]
                if writer is None:
                    # 首帧: 按实际帧尺寸创建 writer
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    rec_dir = self._get_record_dir()
                    stem = f"rec_cam{cam_id}_{ts}"
                    h, w = frame.shape[:2]
                    # v3.5: 改用 MjpegRecorder(质量可控 + 手写 MJPEG-AVI)。
                    #   原 cv2.VideoWriter(*"XVID") 在 OpenCV/FFMPEG 后端下静默降级为
                    #   低码率 FMP4(实测 40.5dB), 且无法设质量 —— 这是"录像画质差"的根因。
                    writer = MjpegRecorder(rec_dir, stem, w, h,
                                           fps=REC_FPS, quality=REC_JPEG_QUALITY)
                    if cam_id == 1:
                        self._rec_writer1 = writer
                        self._rec_fname1 = writer.fname
                    elif cam_id == 2:
                        self._rec_writer2 = writer
                        self._rec_fname2 = writer.fname
                    else:
                        self._rec_writer3 = writer
                        self._rec_fname3 = writer.fname
                    self._log(f"[VID] CAM{cam_id} 开始写入录像: {writer.fname.name} "
                              f"({w}x{h} @ {REC_FPS}fps, JPEG q{REC_JPEG_QUALITY})")
                writer.write(frame)
                # v3.5: 达到单文件上限会自动切段, 同步最新段名(收尾日志/UI 用)
                if getattr(writer, "fname", None) is not None:
                    if cam_id == 1 and self._rec_fname1 != writer.fname:
                        self._rec_fname1 = writer.fname
                        self._log(f"[VID] CAM1 录像分段: {writer.fname.name}")
                    elif cam_id == 2 and self._rec_fname2 != writer.fname:
                        self._rec_fname2 = writer.fname
                        self._log(f"[VID] CAM2 录像分段: {writer.fname.name}")
                    elif cam_id == 3 and self._rec_fname3 != writer.fname:
                        self._rec_fname3 = writer.fname
                        self._log(f"[VID] CAM3 录像分段: {writer.fname.name}")
        # v3.3: 画面所在页面不可见(如切到其他选项卡)时跳过渲染, 仅保留录像
        if not lbl.isVisible():
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        # v3.4.1: QImage 不持有缓冲区 —— .copy() 拷一份, 保证 rgb 被回收后像素仍有效
        # (原代码直接包 rgb.data, 生命周期边界模糊, 偶发花屏/撕裂)。
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        src = QPixmap.fromImage(img)
        if self._rawsize_1to1:
            # v3.4.1: 1:1 原始尺寸 —— 不缩放, 按帧真实像素上屏, 看清每个像素与检测框。
            # 分辨率变了就同步抬高标签最小尺寸, 避免卡片装不下造成裁剪。
            if lbl.minimumWidth() != w or lbl.minimumHeight() != h:
                lbl.setMinimumSize(w, h)
            lbl.setPixmap(src)
        else:
            # 自适应缩放: 4:3 等比放大填满卡片。
            # v3.4.1: 插值方式由顶部 VID_SCALE_SHARP 决定(默认锐利放大, 低分辨率更清楚)
            mode = Qt.FastTransformation if VID_SCALE_SHARP else Qt.SmoothTransformation
            scaled = src.scaled(lbl.size(), Qt.KeepAspectRatio, mode)
            lbl.setPixmap(scaled)

    def _on_net_tx(self, cnt, byt):
        self._tx_pps = cnt; self._tx_bps = byt
        self._update_net_lbl()

    def _on_net_rx(self, cnt, byt):
        self._rx_pps = cnt; self._rx_bps = byt
        self._update_net_lbl()

    def _update_net_lbl(self):
        tx_p = getattr(self, "_tx_pps", 0)
        tx_b = getattr(self, "_tx_bps", 0)
        rx_p = getattr(self, "_rx_pps", 0)
        rx_b = getattr(self, "_rx_bps", 0)
        lat = self._latency_ms
        lat_str = f"{lat:.0f}ms" if lat >= 0 else "--"
        self.net_lbl.setText(
            f"TX:{tx_p}pps/{tx_b}B  RX:{rx_p}pps/{rx_b}B")
        self.latency_lbl.setText(f"RTT: {lat_str}")

    def _measure_latency(self):
        """UDP ping -> 测量 RTT (仅有线模式; 无线纯下行不测)"""
        if self._link_mode == 1:
            return
        def _ping():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(1.0)
                t0 = time.perf_counter()
                s.sendto(b"PING", (target_ip, TELEM_PORT))
                s.recvfrom(64)
                self._latency_ms = (time.perf_counter() - t0) * 1000
                self._last_ping_ok = time.perf_counter()   # v3.2: 记录PING成功时间
            except:
                self._latency_ms = -1
            finally:
                try: s.close()
                except: pass
        threading.Thread(target=_ping, daemon=True).start()

    #  录像 / 存储 
    def _toggle_record(self):
        if not self._recording:
            # v3.2: 双摄writer延迟到首帧创建(见 _show_frame), 需要视频流开启
            if not self._video_on or self._link_mode == 1:
                self._log("[WARN] 视频传输已关闭, 无法录像 (请先打开视频)")
                return
            self._recording = True
            self.rec_btn.setText("停止录像 [X]")
            self.rec_btn.setStyleSheet("padding:4px 10px; background:#ef5350; color:white;")
            # v3.3: 第三路开启时也一并录制
            msg = "双摄将各自保存 rec_cam1_*.avi / rec_cam2_*.avi"
            if self._cam3_on:
                msg += " / rec_cam3_*.avi"
            self._log(f"[VID] 开始录像: {msg}")
        else:
            self._recording = False
            # v3.2: 释放各路writer (v3.3: 含第三路; v3.5: fname 取 release 前的最新段名)
            for which, writer in (
                (1, self._rec_writer1),
                (2, self._rec_writer2),
                (3, self._rec_writer3),
            ):
                if writer is not None:
                    cur = getattr(writer, "fname", None)
                    mb = getattr(writer, "bytes_written", 0) / 1048576.0
                    writer.release()
                    if cur:
                        self._log(f"[VID] CAM{which} 录像已保存: {cur.name}"
                                  + (f" ({mb:.1f}MB)" if mb else ""))
            self._rec_writer1 = None
            self._rec_writer2 = None
            self._rec_writer3 = None
            self._rec_fname1 = None
            self._rec_fname2 = None
            self._rec_fname3 = None
            self.rec_btn.setText("录像 [X]")
            self.rec_btn.setStyleSheet("padding:4px 10px;")
            self._log("录像已停止")

    def _toggle_store(self):
        self._store = not self._store
        self._joy_thread._store = 1 if self._store else 0
        if self._store:
            self._start_csv()
            self.store_btn.setText("停止存储 [Y]")
            self.store_btn.setStyleSheet("padding:4px 10px; background:#42a5f5; color:white;")
            self._log("数据存储: 开始")
        else:
            self._stop_csv()
            self.store_btn.setText("存储 [Y]")
            self.store_btn.setStyleSheet("padding:4px 10px;")
            self._log("数据存储: 停止")

    def _start_csv(self):
        """开始把遥测数据写入 CSV 文件"""
        try:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            rec_dir = self._get_record_dir()
            self._csv_path = rec_dir / f"telemetry_{ts}.csv"
            self._csv_file = open(self._csv_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(["timestamp"] + self._csv_fields)
            self._log(f"数据记录开始: {self._csv_path.name}")
        except Exception as e:
            self._log(f"数据记录启动失败: {e}")
            self._store = False
            self._csv_file = None; self._csv_writer = None

    def _stop_csv(self):
        """结束并刷新 CSV 文件"""
        try:
            if self._csv_file:
                self._csv_file.flush()
                self._csv_file.close()
            self._csv_file = None; self._csv_writer = None
            if self._csv_path:
                self._log(f"数据记录已保存: {self._csv_path.name}")
                self._csv_path = None
        except Exception as e:
            self._log(f"数据记录关闭失败: {e}")

    #  关闭 
    def closeEvent(self, ev):
        self._pid_rx_timer.stop()
        self._close_pid_serial()
        # v3.2: 退出前经当前链路发急停
        try:
            self._joy_thread.set_link_mode(self._link_mode)  # no-op(模式相同), 仅确保状态
        except Exception:
            pass
        try:
            if self._link_mode == 1:
                self._joy_thread._lora_emergency_stop()
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(build_emergency_stop().encode(), (target_ip, CMD_PORT))
                s.close()
        except Exception:
            pass
        self._joy_thread.stop(); self._joy_thread.wait(2000)
        self._tel_thread.stop(); self._tel_thread.wait(2000)
        self._alt_thread.stop(); self._alt_thread.wait(2000)
        if getattr(self, "_msg_thread", None) is not None:
            self._msg_thread.stop(); self._msg_thread.wait(2000)
        # v3.2: 视频线程可能已被视频开关置为None
        for t in (getattr(self, "_vid1_thread", None), getattr(self, "_vid2_thread", None),
                  getattr(self, "_vid3_thread", None)):
            if t is not None:
                t.stop(); t.wait(2000)
        # v3.2: 释放各路录像writer (v3.3: 含第三路)
        for writer in (self._rec_writer1, self._rec_writer2, self._rec_writer3):
            if writer:
                writer.release()
        self._rec_writer1 = None; self._rec_writer2 = None; self._rec_writer3 = None
        self._stop_csv()
        ev.accept()


# 
#  Entry Point
# 
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    # 统一调大默认字号(微软雅黑, 若系统无此字体 Qt 自动回退)
    try:
        app.setFont(QFont("Microsoft YaHei", 10))
    except Exception:
        pass
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
