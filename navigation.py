"""开机坐标系下的速度积分轨迹；无有效速度时不估计位移。"""
import math
import time

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget


class PositionTracker:
    """vx 为机体前向、vy 为机体右向，单位 m/s，yaw 顺时针为正。

    x 为开机方向的右侧，y 为开机方向的前方。首帧有效 yaw 建立原点；
    中断超过两秒不跨越缺失时间积分。history 中 None 表示轨迹断点。
    """
    MAX_GAP = 2.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = self.y = 0.0
        self.base_heading = None
        self.heading = 0.0
        self._previous = None
        self.history = []
        self.status = "等待航向/速度"
        self.last_feed = None

    @staticmethod
    def _number(value):
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except (ValueError, TypeError):
            return None

    def update(self, telemetry, now=None):
        now = time.monotonic() if now is None else now
        yaw = self._number(telemetry.get("yaw"))
        vx = self._number(telemetry.get("vx"))
        vy = self._number(telemetry.get("vy"))
        self.last_feed = now
        if yaw is None:
            self._previous = None
            self.status = "缺少有效航向"
            self._break_path()
            return
        self.heading = yaw % 360.0
        if self.base_heading is None:
            self.base_heading = self.heading
            self.history.append((now, 0.0, 0.0, self.heading))
        if vx is None or vy is None or not telemetry.get("position_velocity_valid", True):
            self._previous = None
            self.status = "缺少有效速度"
            self._break_path()
            return
        angle = math.radians(self.heading - self.base_heading)
        # 机体速度旋转到开机坐标系；yaw=90°时前进对应 x 增大。
        wx = vx * math.sin(angle) + vy * math.cos(angle)
        wy = vx * math.cos(angle) - vy * math.sin(angle)
        if self._previous is not None:
            last_time, px, py = self._previous
            dt = now - last_time
            if 0 < dt <= self.MAX_GAP:
                self.x += (px + wx) * 0.5 * dt
                self.y += (py + wy) * 0.5 * dt
            else:
                self._break_path()
        self._previous = (now, wx, wy)
        self.status = "速度积分估算"
        point = (now, self.x, self.y, self.heading)
        last = self.history[-1] if self.history else None
        if (last is None or math.hypot(self.x - last[1], self.y - last[2]) >= 0.02
                or abs((self.heading - last[3] + 180) % 360 - 180) >= 3):
            self.history.append(point)

    def _break_path(self):
        if self.history and self.history[-1] is not None:
            self.history.append(None)

    def screen_offset(self, x, y, heading_up=True):
        """当前位置始终居中；返回以右/上为正的显示偏移。"""
        dx, dy = x - self.x, y - self.y
        angle = math.radians(self.heading - (self.base_heading or 0)) if heading_up else 0
        return (dx * math.cos(angle) - dy * math.sin(angle),
                dx * math.sin(angle) + dy * math.cos(angle))


class PositionMap(QWidget):
    """与航向罗盘等大的定位浮层，两个视角共用同一段历史轨迹。"""
    def __init__(self, tracker, parent=None):
        super().__init__(parent)
        self.tracker = tracker
        self.heading_up = True
        self.range_m = 5.0
        self.setFixedSize(168, 168)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAutoFillBackground(False)
        self.setStyleSheet("background:transparent; border:none; border-radius:0px;")
        self.setToolTip("开机原点为零，当前位置居中。路径由机体 vx/vy (m/s) 与 yaw 积分估算。\n"
                        "蓝线=历史路径，绿色圆点=开机位置，橙色箭头=当前位置/方向。\n"
                        "滚轮缩放；无速度/断流时停止积分。")
        self.refresh_timer = self.startTimer(500)

    def timerEvent(self, event):
        self.update()

    def set_view_mode(self, index):
        self.heading_up = index == 0
        self.update()

    def wheelEvent(self, event):
        factor = 0.8 if event.angleDelta().y() > 0 else 1.25
        self.range_m = min(10000.0, max(0.25, self.range_m * factor))
        self.update()
        event.accept()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        # 直角、低遮挡背景；轨迹和方向略透明，但比背景更清晰。
        painter.fillRect(self.rect(), QColor(13, 24, 37, 51))  # 背景透明度 80%
        painter.setPen(QPen(QColor(66, 96, 120, 90), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(QRectF(0.5, 0.5, 167, 167))
        painter.setFont(QFont("Microsoft YaHei", 8))
        painter.setPen(QColor(197, 215, 231, 220))
        title = "位置 · 方向朝前" if self.heading_up else "位置 · 初始方向"
        painter.drawText(QRectF(5, 5, 158, 19), Qt.AlignCenter, title)
        rect = QRectF(10, 28, 148, 98)
        center = rect.center()
        scale = rect.height() / (2 * self.range_m)

        def screen(x, y):
            dx, dy = self.tracker.screen_offset(x, y, self.heading_up)
            return QPointF(center.x() + dx * scale, center.y() - dy * scale)

        painter.save()
        painter.setClipRect(rect)
        painter.setPen(QPen(QColor(83, 124, 153, 65), 1, Qt.DotLine))
        for offset in (-36, 0, 36):
            painter.drawLine(QPointF(center.x() + offset, rect.top()),
                             QPointF(center.x() + offset, rect.bottom()))
        painter.drawLine(QPointF(rect.left(), center.y()), QPointF(rect.right(), center.y()))
        path = QPainterPath()
        active = False
        for point in self.tracker.history:
            if point is None:
                active = False
                continue
            pos = screen(point[1], point[2])
            if active:
                path.lineTo(pos)
            else:
                path.moveTo(pos)
                active = True
        if active:
            path.lineTo(center)
        painter.setPen(QPen(QColor(80, 191, 255, 178), 2))  # 轨迹不透明度 70%
        painter.drawPath(path)
        if self.tracker.base_heading is not None:
            origin = screen(0, 0)
            painter.setPen(QColor(136, 223, 174, 200))
            painter.setBrush(QColor(136, 223, 174, 200))
            painter.drawEllipse(origin, 3, 3)
            painter.drawText(origin + QPointF(5, -4), "0")
            painter.translate(center)
            rotation = 0 if self.heading_up else self.tracker.heading - self.tracker.base_heading
            painter.rotate(rotation)
            painter.setPen(QPen(QColor(255, 224, 163, 204), 1))
            painter.setBrush(QColor(255, 183, 77, 204))  # 方向不透明度 80%
            painter.drawPolygon(QPolygonF([QPointF(0, -10), QPointF(7, 7),
                                            QPointF(0, 4), QPointF(-7, 7)]))
        painter.restore()
        painter.setFont(QFont("Consolas", 8))
        painter.setPen(QColor(224, 234, 245, 220))
        coords = (f"X {self.tracker.x:+.2f}  Y {self.tracker.y:+.2f} m"
                  if self.tracker.base_heading is not None else "X --  Y -- m")
        painter.drawText(QRectF(2, 128, 164, 17), Qt.AlignCenter, coords)
        status = self.tracker.status
        if self.tracker.last_feed is not None and time.monotonic() - self.tracker.last_feed > 2:
            status = "遥测超时 · 位置保持"
        painter.setFont(QFont("Microsoft YaHei", 7))
        painter.setPen(QColor(139, 169, 192, 210))
        painter.drawText(QRectF(2, 146, 164, 17), Qt.AlignCenter, f"±{self.range_m:g}m · {status}")
        painter.end()
