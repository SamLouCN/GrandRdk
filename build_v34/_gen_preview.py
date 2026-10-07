# -*- coding: utf-8 -*-
"""生成交付预览图: 罗盘特写 + 整窗位置。"""
import os, sys
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
os.environ.pop("QT_QPA_PLATFORM", None)
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage, QPainter, QColor, QFont
from PyQt5.QtCore import Qt
app = QApplication.instance() or QApplication(sys.argv)
import pc_main2 as M

OUT = r"C:\Users\lenovo\WorkBuddy\RC\output\compass_v35"

# --- 1) 罗盘多角度特写 ---
S = 168
COLS, ROWS = 4, 2
PAD, LH = 8, 16
tw, th = S + PAD, S + LH + PAD
shot = QImage(COLS * tw + PAD, ROWS * th + PAD, QImage.Format_ARGB32)
shot.fill(QColor(58, 78, 92))
cases = [
    (0.0, True, "0 ABS"), (45.0, True, "45 ABS"),
    (120.0, True, "120 ABS"), (215.0, True, "215 ABS"),
    (0.0, False, "REL 0 (just clicked)"), (12.0, False, "REL +12"),
    (-25.0, False, "REL -25"), (180.0, False, "REL 180"),
]
p = QPainter(shot); p.setFont(QFont("Consolas", 8))
for i, (ang, isabs, label) in enumerate(cases):
    c = M.HeadingCompass("hangxiang"); c.setFixedSize(S, S)
    if isabs:
        c.set_absolute(); c.set_heading(ang)
    else:
        c.set_heading(100.0); c.reset_relative(); c.set_heading(100.0 + ang)
    sub = QImage(S, S, QImage.Format_ARGB32); sub.fill(0)
    c.render(sub)
    r, cc = divmod(i, COLS)
    x, y = PAD + cc * tw, PAD + r * th
    p.drawImage(x, y, sub)
    p.setPen(QColor(235, 240, 245))
    p.drawText(x + 4, y + S + 12, label)
p.end()
os.makedirs(OUT, exist_ok=True)
shot.save(os.path.join(OUT, "compass_angles.png"))

# --- 2) 整窗位置 ---
win = M.MainWindow(); win.resize(1360, 900); win.show(); app.processEvents()
for i in range(win._main_tabs.count()):
    if win._main_tabs.tabText(i) == "可视窗口":
        win._main_tabs.setCurrentIndex(i); break
win.compass.set_absolute(); win.compass.set_heading(47.0)
for _ in range(6):
    app.processEvents()
wimg = QImage(win.size(), QImage.Format_ARGB32); wimg.fill(QColor(13, 17, 23))
win.render(wimg)
wimg.save(os.path.join(OUT, "window_compass.png"))
print("saved to", OUT)
print("compass geom:", win.compass.geometry().getRect())
