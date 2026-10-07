# -*- coding: utf-8 -*-
"""集成验证: 罗盘随 _on_tel 的 yaw 更新 (模拟遥测帧)。"""
import os, sys
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtWidgets import QApplication
app = QApplication.instance() or QApplication(sys.argv)
import pc_main2 as M

P = F = 0
def chk(c, d, x=""):
    global P, F
    if c: P += 1; print("  [PASS]", d, x)
    else: F += 1; print("  [FAIL]", d, x)

win = M.MainWindow()
c = win.compass
chk(c is not None, "主窗口有 compass")
chk(not c._has_data, "初始无数据")

# 模拟遥测帧(只需 yaw, 其余字段走默认)
for yaw in (0.0, 30.0, 90.0, -45.0, 359.5):
    win._on_tel({"yaw": yaw})
    chk(c._has_data and abs(c._yaw - yaw) < 1e-6,
        "_on_tel 把 yaw=%.1f 喂进罗盘" % yaw, "实际=%s" % c._yaw)

# 相对模式
win._on_tel({"yaw": 200.0})
c.reset_relative()
chk(abs(c._disp_angle()) < 1e-9, "相对归零")
win._on_tel({"yaw": 230.0})
chk(abs((c._disp_angle()%360)-30.0) < 1e-6, "相对显示 +30", "实际=%s" % c._disp_angle())

# 缺 yaw 的帧不炸
win._on_tel({"roll": 1.0})
chk(True, "缺 yaw 的帧不报错")

print("\n结果: %d/%d" % (P, P+F))
sys.exit(0 if F == 0 else 1)
