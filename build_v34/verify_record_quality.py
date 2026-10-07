# -*- coding: utf-8 -*-
"""v3.5 录像(画质提升) 自检 —— 纯本机, 用真实录像帧走完整写入链。

验证:
  [1] MjpegAviWriter: 容器头正确(OpenCV 可打开/fps/尺寸/fourcc=MJPG/帧数精确)
  [2] MjpegAviWriter: idx1 随机寻址有效
  [3] MjpegRecorder: write/release 接口, 落盘可回放, PSNR 明显优于旧 FMP4
  [4] MjpegRecorder: 单文件上限自动分段
  [5] 端到端: 模拟 _show_frame 逐帧喂 MainWindow, 走完整录像链
运行: D:\\Anaconda\\python.exe build_v34\\verify_record_quality.py
"""
import os
import sys
import glob
import shutil
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2
import numpy as np

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


def psnr(a, b):
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    return 99.0 if mse == 0 else 10 * np.log10(255.0 ** 2 / mse)


def load_frames(src, n):
    cap = cv2.VideoCapture(src)
    out = []
    for _ in range(n):
        ok, f = cap.read()
        if not ok:
            break
        out.append(f)
    cap.release()
    return out


def main():
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    import pc_main2 as M

    src = os.path.join(_ROOT, "recordings", "rec_cam1_20261004_180727.avi")
    if not os.path.exists(src):
        print("找不到测试源录像:", src)
        return 1
    frames = load_frames(src, 90)
    print("源帧: %d 帧 %s" % (len(frames), frames[0].shape))
    tmp = tempfile.mkdtemp(prefix="recq_")

    # ---------------- [1] MjpegAviWriter 容器 ----------------
    print("[1] MjpegAviWriter 容器正确性")
    p1 = os.path.join(tmp, "w1.avi")
    w = M.MjpegAviWriter(p1, 640, 480, fps=30.0)
    for f in frames:
        ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 90])
        w.append(buf.tobytes())
    w.close()
    cap = cv2.VideoCapture(p1)
    chk(cap.isOpened(), "OpenCV 能打开")
    chk(abs(cap.get(cv2.CAP_PROP_FPS) - 30.0) < 0.5, "fps=30",
        "实际=%.2f" % cap.get(cv2.CAP_PROP_FPS))
    chk(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) == 640 and
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) == 480, "尺寸 640x480")
    fi = int(cap.get(cv2.CAP_PROP_FOURCC))
    cc = "".join(chr((fi >> (8 * k)) & 0xFF) for k in range(4))
    chk(cc == "MJPG", "fourcc=MJPG", "实际=%s" % cc)
    chk(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == len(frames), "帧数精确",
        "实际=%d 期望=%d" % (cap.get(cv2.CAP_PROP_FRAME_COUNT), len(frames)))
    cap.release()

    # ---------------- [2] 索引寻址 ----------------
    print("[2] idx1 索引随机寻址")
    cap = cv2.VideoCapture(p1)
    ok_all = True
    for tgt in (0, 1, 44, 45, 89):
        cap.set(cv2.CAP_PROP_POS_FRAMES, tgt)
        ok, d = cap.read()
        g = psnr(frames[tgt], d) if ok else -1
        if not (ok and g > 35):
            ok_all = False
    cap.release()
    chk(ok_all, "各点 seek 后解码 PSNR>35dB")

    # ---------------- [3] MjpegRecorder + 画质对比 ----------------
    print("[3] MjpegRecorder 落盘与画质")
    rec = M.MjpegRecorder(tmp, "rec_cam1_test", 640, 480,
                          fps=M.REC_FPS, quality=M.REC_JPEG_QUALITY)
    chk(rec.isOpened(), "isOpened()")
    for f in frames:
        rec.write(f)
    p_new = str(rec.fname)
    nb = rec.bytes_written
    rec.release()
    chk(os.path.getsize(p_new) > 0, "落盘非空", "%.0fKB" % (os.path.getsize(p_new) / 1024))

    # 对比: 旧方式(XVID -> 实际 FMP4)
    p_old = os.path.join(tmp, "old_xvid.avi")
    wr = cv2.VideoWriter(p_old, cv2.VideoWriter_fourcc(*"XVID"), 30.0, (640, 480))
    ok_open = wr.isOpened()
    if ok_open:
        for f in frames:
            wr.write(f)
    wr.release()

    def avg_psnr(path):
        c = cv2.VideoCapture(path)
        vals = []
        i = 0
        while True:
            ok, d = c.read()
            if not ok:
                break
            if i < len(frames):
                vals.append(psnr(frames[i], d))
            i += 1
        c.release()
        return float(np.mean(vals)) if vals else -1.0

    q_new = avg_psnr(p_new)
    q_old = avg_psnr(p_old) if ok_open else -1
    print("     新(MjpegRecorder q%d) PSNR=%.2fdB  |  旧(XVID→FMP4) PSNR=%.2fdB"
          % (M.REC_JPEG_QUALITY, q_new, q_old))
    chk(q_new > q_old + 2.0, "新方案 PSNR 明显更高(>2dB)", "+%.2fdB" % (q_new - q_old))
    chk(q_new > 43.0, "新方案 PSNR>43dB")

    # ---------------- [4] 自动分段 ----------------
    print("[4] 单文件上限自动分段")
    rec2 = M.MjpegRecorder(tmp, "rec_cam1_seg", 640, 480,
                           fps=30, quality=90, max_bytes=120 * 1024)
    for i in range(300):
        rec2.write(frames[i % len(frames)])
    seg_files = sorted(glob.glob(os.path.join(tmp, "rec_cam1_seg*.avi")))
    rec2.release()
    seg_files = sorted(glob.glob(os.path.join(tmp, "rec_cam1_seg*.avi")))
    chk(len(seg_files) >= 2, "产生多段文件", "段数=%d" % len(seg_files))
    # 每段都能打开
    all_open = all(cv2.VideoCapture(p).isOpened() for p in seg_files)
    chk(all_open, "各段均可打开")
    over = [p for p in seg_files if os.path.getsize(p) > 120 * 1024 * 1.05]
    chk(not over, "各段不超上限", "超出=%d" % len(over))

    # ---------------- [5] 端到端(走 MainWindow) ----------------
    print("[5] 端到端: MainWindow._show_frame 录像链")
    win = M.MainWindow()
    recdir = os.path.join(tmp, "winrec")
    os.makedirs(recdir, exist_ok=True)
    win._get_record_dir = lambda: __import__("pathlib").Path(recdir)
    win._recording = True
    win.cam_tiles["cam1"].video.hide()          # 不渲染也要录(测"不可见也录")
    for f in frames[:40]:
        win._show_frame(win.cam1_lbl, f, cam_id=1)
    wobj = win._rec_writer1
    chk(wobj is not None, "writer 已创建")
    chk(isinstance(wobj, M.MjpegRecorder), "类型是 MjpegRecorder")
    f1 = str(wobj.fname)
    win._toggle_record()                        # 停录(会 release)
    chk(win._rec_writer1 is None, "停录后 writer 已释放")
    cap = cv2.VideoCapture(f1)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    chk(n == 40, "录像帧数=40", "实际=%d" % n)
    chk(abs(fps - 30.0) < 0.5, "录像 fps=REC_FPS(30)", "实际=%.2f" % fps)

    shutil.rmtree(tmp, ignore_errors=True)
    print("")
    print("=" * 52)
    print("结果: %d/%d 通过" % (_PASS, _PASS + _FAIL))
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
