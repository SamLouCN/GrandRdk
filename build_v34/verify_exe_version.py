# -*- coding: utf-8 -*-
"""从打包好的 EXE 里核验 pc_main2 是不是 v3.5 版。

入口脚本 pc_main2 以**字节码**形式直接存在 CArchive 里（不在 PYZ 内）。
Python 3.13 的 marshal 可直接 loads，无需 xdis。

用法：python verify_exe_version.py [EXE路径]
"""
import marshal
import sys

sys.path.insert(0, r"C:\Users\lenovo\rovpack\venv\Lib\site-packages")
from PyInstaller.archive.readers import CArchiveReader

EXE = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\RC\ROV控制站_v3.3\ROV_ControlStation_v3.5.exe"
r = CArchiveReader(EXE)

data = r.extract("pc_main2")
print("提取 pc_main2 字节码: %d 字节" % len(data))
code = marshal.loads(data)


def collect_strings(co, out, depth=0):
    """递归收集 code object 里的字符串常量 + 标识符（含嵌套函数/类）。

    [2026-10-05] 增加 co_names / co_varnames 收集：类名、方法名、属性名
    只出现在 co_names，不在 co_consts。不补这一块就搜不到 MsgThread/_on_msg。
    """
    if depth > 8:
        return
    for c in co.co_consts:
        if isinstance(c, str):
            out.append(c)
        elif hasattr(c, "co_consts"):
            collect_strings(c, out, depth + 1)
    for n in list(co.co_names) + list(co.co_varnames) + list(co.co_freevars):
        if isinstance(n, str):
            out.append(n)


strings = []
collect_strings(code, strings)
# 脚本自己的源码文本也在 co_consts 里（如 docstring），一并搜
joined = "\n".join(strings)
print("收集到字符串常量 %d 条" % len(strings))

print("\n=== v3.4 新文案核验 ===")
for s in ("有线遥控", "自主航行", "交还自主", "温启动", "模式: 待机", "工作模式", "遥控交还"):
    print("  %-16s : %s" % (s, "存在" if s in joined else "**缺失**"))

print("\n=== v3.4.1 画质改动核验 ===")
for s in ("显示: 1:1 原始尺寸", "显示: 自适应缩放", "自适应缩放"):
    print("  %-16s : %s" % (s, "存在" if s in joined else "**缺失**"))

print("\n=== v3.5 航向罗盘核验 ===")
for s in ("HeadingCompass", "航向罗盘", "航向", "ABS 绝对", "REL 相对",
          "零度朝上", "ROV控制站 v3.5"):
    print("  %-16s : %s" % (s, "存在" if s in joined else "**缺失**"))

print("\n=== v3.5 录像画质核验 ===")
# 注：'idx1'/'00dc' 是 bytes 常量（b'idx1'），不在 str 常量池里，此处只查标识符与常量名
for s in ("MjpegAviWriter", "MjpegRecorder", "REC_JPEG_QUALITY", "REC_FPS",
          "REC_MAX_FILE_GB", "image/jpeg"):
    print("  %-16s : %s" % (s, "存在" if s in joined else "**缺失**"))

print("\n=== v3.5 AUV 状态提示 $MSG 回传核验 ===")
# 板端 auv_task/notify.py 发 $MSG 帧，上位机 MsgThread 收、_on_msg 打终端
for s in ("MsgThread", "msg_sig", "raw_sig", "parse_msg", "parse_auv", "MSG_PORT",
          "_on_msg", "_msg_thread", "_MSG_LEVEL", "_auv_stage"):
    print("  %-16s : %s" % (s, "存在" if s in joined else "**缺失**"))
# 'MSG' 常以 b"$MSG" 形式出现，bytes 不在 str 池，故另查字节码里的裸标识
print("  %-16s : %s" % ("$MSG(字符串)", "存在" if "$MSG" in joined else "（见 bytes 常量）"))

print("\n=== 旧文案（应已消失）===")
for s in ("遥控模式", "AUV模式", "解除急停", "ROV控制站 v3.3", "ROV控制站 v3.4.1"):
    print("  %-16s : %s" % (s, "仍存在!" if s in joined else "已移除"))

print("\n=== 版本号 ===")
for s in strings:
    if "ROV 遥控上位机 v" in s:
        print("  ", s.strip().splitlines()[0] if s.strip() else s)
        break
