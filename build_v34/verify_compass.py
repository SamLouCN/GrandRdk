# -*- coding: utf-8 -*-
"""v3.5 航向罗盘(HeadingCompass) 自检 —— 纯本机, offscreen, 不碰网络。

验证:
  [1] 控件创建 + 尺寸 + 零度朝上时指针朝上(角度换算)
  [2] 绝对模式: set_heading() 后指针角度 = yaw
  [3] 相对模式: reset_relative() 后指针归零; 再喂新 yaw 显示差值
  [4] 交互: 左键(相对)归零 / 左键(绝对)转相对 / 双击回绝对 / 右键回绝对
  [5] 真实绘制不报错(渲染到 QImage, 逐像素有内容)
  [6] 挂到 VideoTile 右上角后存在且可见
运行: D:\\Anaconda\\python.exe build_v34\\verify_compass.py
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_PASS = 0
_FAIL = 0


def chk(cond, desc, detail=""):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print("  [PASS] %s %s" % (desc, ("| " + detail) if detail else ""))
    else:
        _FAIL += 1
        print("  [FAIL] %s %s" % (desc, ("| " + detail) if detail else ""))


def main():
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import Qt, QPoint, QEvent
    from PyQt5.QtGui import QImage, QMouseEvent
    app = QApplication.instance() or QApplication(sys.argv)

    import pc_main2 as M

    print("[1] 控件创建与基本属性")
    c = M.HeadingCompass("航向")
    chk(c.width() == 168 and c.height() == 168, "默认 168×168", "实际=%dx%d" % (c.width(), c.height()))
    chk(c.mode_is_abs() is True, "默认绝对模式")
    chk(c._disp_angle() == 0.0, "无数据时显示角 = 0", "实际=%s" % c._disp_angle())

    print("[2] 绝对模式: 指针 = yaw")
    for y in (0, 45, 90, 180, 270, 359, -30, 400):
        c.set_absolute(); c.set_heading(y)
        want = y % 360
        got = c._disp_angle() % 360
        chk(abs(got - want) < 1e-6, "set_heading(%s) 显示 %s" % (y, want), "实际=%s" % got)

    print("[3] 相对模式: 基准归零 + 差值")
    c.set_heading(123.0)
    c.reset_relative()
    chk(c.mode_is_abs() is False, "reset_relative 后为相对模式")
    chk(abs(c._disp_angle()) < 1e-9, "指针归零(朝上)", "实际=%s" % c._disp_angle())
    c.set_heading(133.0)          # +10°
    chk(abs((c._disp_angle() % 360) - 10.0) < 1e-6, "yaw 变 +10 显示 +10",
        "实际=%s" % c._disp_angle())
    c.set_heading(113.0)          # -10°
    chk(abs((c._disp_angle() % 360) - 350.0) < 1e-6, "yaw 变 -10 显示 -10(即 350)",
        "实际=%s" % c._disp_angle())

    print("[4] 鼠标交互")
    def click(btn=Qt.LeftButton, dbl=False):
        pos = QPoint(c.width() // 2, c.height() // 2)
        ev = QMouseEvent(QEvent.MouseButtonDblClick if dbl else QEvent.MouseButtonPress,
                         pos, btn, btn, Qt.NoModifier)
        (c.mouseDoubleClickEvent if dbl else c.mousePressEvent)(ev)

    c.set_absolute(); c.set_heading(200.0)
    click(Qt.LeftButton)
    chk(c.mode_is_abs() is False, "左键: 绝对 → 相对")
    chk(abs(c._disp_angle()) < 1e-9, "左键后指针归零")
    c.set_heading(215.0)
    click(Qt.LeftButton)
    chk(c.mode_is_abs() is False and abs(c._disp_angle()) < 1e-9,
        "相对下再左键: 重置基准(仍朝上)")
    click(Qt.LeftButton, dbl=True)
    chk(c.mode_is_abs() is True, "双击: 回绝对")
    c.reset_relative()
    click(Qt.RightButton)
    chk(c.mode_is_abs() is True, "右键: 回绝对")

    print("[5] 真实绘制(渲染到 QImage, 检查非空)")
    c.set_absolute(); c.set_heading(60.0)
    c.resize(168, 168)
    img = QImage(168, 168, QImage.Format_ARGB32)
    img.fill(0)
    c.render(img)
    # 统计不透明像素
    nonempty = 0
    for yy in range(0, 168, 4):
        for xx in range(0, 168, 4):
            if img.pixelColor(xx, yy).alpha() > 0:
                nonempty += 1
    chk(nonempty > 100, "绘制出可见内容", "采样非空像素=%d" % nonempty)

    print("[6] 挂到 VideoTile 右上角")
    tile = M.VideoTile("CAM1 前置")
    comp = tile.add_compass(168)
    chk(comp is not None and isinstance(comp, M.HeadingCompass), "add_compass 返回罗盘")
    chk(tile.compass_widget() is comp, "compass_widget() 可取回")
    tile.show()
    chk(comp.isVisible(), "罗盘随 tile 显示")

    print("[7] 罗盘可接收鼠标(未设 WA_TransparentForMouseEvents)")
    chk(not comp.testAttribute(Qt.WA_TransparentForMouseEvents),
        "罗盘不被鼠标穿透(可点击)")
    chk(tile.badge_title.testAttribute(Qt.WA_TransparentForMouseEvents),
        "角标仍鼠标穿透(不受影响)")

    print("")
    print("=" * 50)
    print("结果: %d/%d 通过" % (_PASS, _PASS + _FAIL))
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
