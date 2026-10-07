# -*- coding: utf-8 -*-
"""v3.4 多模式 UI 改造 —— 验证脚本（本机跑，不需连板）

验证三件事：
  [1] 按钮文案与分组：有线遥控 / 自主航行 / 交还自主 / 急停 / 温启动 / 抓球 / 抛球
  [2] 模式显示：默认「有线遥控」；切 AUV 显示「自主航行」；急停显示「待机」
  [3] 协议帧：点按钮发出的 $CMD 里 mode 字段只可能是 0 或 1（S100 侧只认这两个值）

运行：
    cd D:\\RC\\ROV控制站_v3.3
    D:\\Anaconda\\python.exe build_v34\\verify_v34_ui.py

说明：本脚本用 QApplication + 假手柄线程，纯本机跑，不碰网络（UDP 用 mock 拦截）。
"""
import os
import sys

# 让 import pc_main2 能找到本目录（脚本在 build_v34/ 下，父目录才是工程根）
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # 无显示器也能跑

_PASS = 0
_FAIL = 0


def chk(cond, desc, detail=""):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print("  [PASS] %s %s" % (desc, (" | " + detail) if detail else ""))
    else:
        _FAIL += 1
        print("  [FAIL] %s %s" % (desc, (" | " + detail) if detail else ""))


def main():
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)

    import protocol as P
    import pc_main2 as M

    # ---------------- [1] 按钮文案与分组 ----------------
    print("[1] 主界面「快捷操作」区按钮文案")
    win = M.MainWindow()
    # 按钮必须存在
    for attr, want in (("rov_btn", "有线遥控"), ("auv_btn", "自主航行"),
                       ("handback_btn", "交还自主"), ("estop_btn", "急停"),
                       ("estop_release_btn", "温启动"),
                       ("grab_btn", "抓球 [A]"), ("throw_btn", "抛球 [B]")):
        btn = getattr(win, attr, None)
        chk(btn is not None, "%s 存在" % attr)
        if btn is not None:
            chk(btn.text() == want, "%s 文案 == %r" % (attr, want), "实际=%r" % btn.text())

    # 分组小标题（QLabel 文本里应有 工作模式 / 安全 / 作业）
    grp = None
    for w in win.findChildren(type(win.mode_lbl)):
        if w.text() in ("工作模式", "安全", "作业", "遥控交还"):
            grp = True
    chk(bool(grp), "出现分组小标题(工作模式/安全/作业)")

    # ---------------- [2] 模式显示 ----------------
    print("[2] 模式标签显示")
    win._mode = 0
    win._frozen = False
    win._paint_mode()
    chk(win.mode_lbl.text() == "模式: 有线遥控", "默认显示「有线遥控」", "实际=%r" % win.mode_lbl.text())

    win._mode = 1
    win._paint_mode()
    chk(win.mode_lbl.text() == "模式: 自主航行", "切 AUV 显示「自主航行」", "实际=%r" % win.mode_lbl.text())

    # 急停 -> 待机
    win._frozen = True
    win._paint_mode()
    chk(win.mode_lbl.text() == "模式: 待机", "急停后显示「待机」", "实际=%r" % win.mode_lbl.text())
    chk(not win.rov_btn.isChecked() and not win.auv_btn.isChecked(),
        "待机时两个模式按钮都不高亮")

    # 温启动 -> 恢复
    win._frozen = False
    win._mode = 0
    win._paint_mode()
    chk(win.mode_lbl.text() == "模式: 有线遥控" and win.rov_btn.isChecked(),
        "温启动后恢复「有线遥控」且按钮高亮")

    # ---------------- [3] 协议帧只可能 mode=0/1 ----------------
    print("[3] $CMD 帧的 mode 字段取值范围")
    # 拦截 UDP 发送，抓帧
    import socket as _socket
    _sent = []
    _orig_socket = _socket.socket

    class _FakeSock(object):
        def __init__(self, *a, **k):
            pass
        def sendto(self, data, addr):
            _sent.append(data.decode("utf-8", "replace"))
        def close(self):
            pass

    _socket.socket = _FakeSock
    try:
        # 模拟已有手柄线程（JoystickThread 起线程会碰 pygame，这里只造一个轻量替身）
        # 注意：send_estop/_send_raw_wired 在生产代码里也是经 socket 发（_send_raw_wired），
        # 但 FakeJT 会直接覆盖它们，所以这里让替身**真正记录帧**，才能验证 $ESTOP 系列。
        class _FakeJT(object):
            _mode = 0
            _led1 = 0
            _led2 = 0
            _grab = 0
            _store = 0
            _link_mode = 0
            def send_estop(self):
                _sent.append("$ESTOP#\r\n"); return True
            def send_estop_release(self):
                _sent.append("$ESTOP,0#\r\n"); return True
            def _ensure_lora(self): return None
            def _lora_emergency_stop(self): pass

        win._joy_thread = _FakeJT()
        win._link_mode = 0

        for mode, label in ((0, "有线遥控"), (1, "自主航行")):
            _sent[:] = []
            win._set_mode(mode)
            chk(len(_sent) >= 1, "点「%s」发出 %d 帧" % (label, len(_sent)))
            ok = True
            for fr in _sent:
                if fr.startswith("$CMD,"):
                    body = fr.strip()[5:-1].split(",")
                    if len(body) >= 7 and body[6] not in ("0", "1"):
                        ok = False
            chk(ok, "「%s」发出的 $CMD mode 字段只可能是 0/1" % label,
                "帧=%r" % (_sent[:1],))
            # 校验 mode 值与预期一致
            if _sent:
                body = _sent[0].strip()[5:-1].split(",")
                chk(body[6] == str(mode), "「%s」的 $CMD mode == %d" % (label, mode),
                    "实际=%s" % body[6])

        # 急停帧
        _sent[:] = []
        win._frozen = False
        win._click_estop()
        joined = "".join(_sent)
        chk("$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#" in joined,
            "急停先发零杆位 $CMD")
        chk("$ESTOP#" in joined, "急停补发 $ESTOP#")
        chk(win._frozen is True, "急停后 _frozen 置位(显示待机)")

        # 温启动帧
        _sent[:] = []
        win._click_estop_release()
        joined = "".join(_sent)
        chk("$ESTOP,0#" in joined, "温启动发 $ESTOP,0#")
        chk(win._frozen is False, "温启动后 _frozen 清除")
    finally:
        _socket.socket = _orig_socket

    print("")
    print("=" * 50)
    print("结果: %d/%d 通过" % (_PASS, _PASS + _FAIL))
    if _FAIL:
        print("失败 %d 项" % _FAIL)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
