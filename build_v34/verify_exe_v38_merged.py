# -*- coding: utf-8 -*-
"""从打包好的 EXE 里核验 pc_main2/protocol/task_pid_wire 是否为「v3.8 合并 v3.6」版。

- pc_main2（入口脚本）以字节码直接存在 CArchive 顶层。
- protocol / task_pid_wire（普通导入模块）在 PYZ 里，需先取 PYZ 条目再按
  ZlibArchiveReader 读取。
- co_consts 里的元组常量（如 ("fdepth","板端融合深度")）也递归收集，否则搜不到。

用法：python -X utf8 -B verify_exe_v38_merged.py [EXE路径]
"""
import marshal
import os
import sys
import tempfile

sys.path.insert(0, r"C:\Users\lenovo\rovpack\venv\Lib\site-packages")
from PyInstaller.archive.readers import CArchiveReader, ZlibArchiveReader

EXE = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\RC\ROV控制站_v3.8\ROV_ControlStation_v3.8.exe"

failures = []


def collect_strings(co, out, depth=0):
    if depth > 10:
        return
    if isinstance(co, str):
        out.append(co)
        return
    if isinstance(co, (tuple, frozenset)):
        for item in co:
            collect_strings(item, out, depth + 1)
        return
    if not hasattr(co, "co_consts"):
        return
    for c in co.co_consts:
        collect_strings(c, out, depth + 1)
    for n in list(co.co_names) + list(co.co_varnames) + list(co.co_freevars):
        if isinstance(n, str):
            out.append(n)


def check_code(code, label, groups):
    strings = []
    collect_strings(code, strings)
    joined = "\n".join(strings)
    print("=== %s：字符串 %d 条 ===" % (label, len(strings)))
    for title, items in groups:
        print("-- %s --" % title)
        for s in items:
            ok = s in joined
            if not ok:
                failures.append("%s/%s: %s" % (label, title, s))
            print("  %-24s : %s" % (s, "存在" if ok else "**缺失**"))


r = CArchiveReader(EXE)

# --- 入口脚本 pc_main2（CArchive 顶层） ---
data = r.extract("pc_main2")
check_code(
    marshal.loads(data), "上位机主程序 pc_main2", [
        ("v3.6 板端融合值上屏（合并回归）", ["fdepth", "clr", "fused_depth",
                                             "板端融合深度", "离底净空"]),
        ("v3.8 PID 调参页", ["PID 调参", "ControlPidPanel", "pid_panel",
                             "_send_control_pid", "_send_task_pid",
                             "ReferenceTelemetryBuffer"]),
        ("v3.8 S100 过门任务 PID", ["build_task_pid", "parse_task_pid_ack",
                                    "task_pid_wire"]),
        ("v3.8 急停后按模式按钮(方案A)", ["急停锁存中按模式按钮",
                                        "已发 $ESTOP,0# 解除锁存", "send_estop_release"]),
        ("既有功能（应保留）", ["HeadingCompass", "MjpegRecorder", "MsgThread",
                             "set_depth_sender", "Kalman深度"]),
        ("版本号与窗口标题", ["ROV控制站 v3.8"]),
    ])

# --- 普通导入模块（PYZ 内） ---
pyz_blob = r.extract("PYZ.pyz")
fd, tmp_path = tempfile.mkstemp(suffix=".pyz")
with os.fdopen(fd, "wb") as f:
    f.write(pyz_blob)
try:
    z = ZlibArchiveReader(tmp_path)

    check_code(
        z.extract("protocol"), "协议层 protocol", [
            ("v3.6 板端融合解析（合并回归）", ["fused_depth", "clearance"]),
            ("v3.7 参考二进制 PID", ["PID_REFERENCE_MAX_INDEX", "build_pid",
                                    "parse_pid", "ReferenceTelemetryBuffer"]),
            ("协议版本号", ["protocol.py v3.7"]),
        ])

    check_code(
        z.extract("task_pid_wire"), "过门任务 PID task_pid_wire", [
            ("帧字面量", ["$TASKPID,", "$TASKPID_ACK,"]),
            ("函数与标识符", ["build_task_pid", "build_task_pid_ack",
                            "parse_task_pid", "parse_task_pid_ack"]),
        ])
finally:
    os.unlink(tmp_path)

print()
if failures:
    print("核验失败项: %d 条" % len(failures))
    for f in failures:
        print("  -", f)
    sys.exit(1)
print("全部核验通过 ✅")
