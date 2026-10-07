# -*- coding: utf-8 -*-
"""
静态核验 ROV_ControlStation_v3.5.exe 里打包的 pc_main2 是不是"含 $MSG 的新版"。

原理：PyInstaller onefile 的 EXE = [bootloader][CArchive][cookie]。
      CArchive 的 TOC 条目 = ulen(4) pos(4) csize(4) usize(4) cmprs(1) typecode(1) name\\0 pad
      入口脚本 pc_main2 的 .pyc 是 zlib 压缩的，解压后 marshal 字节码里的
      字符串常量（函数名/属性名）是明文 —— 直接数关键词即可。旧 EXE 必然搜不到。

用法：python verify_exe_v35_msg.py [新exe] [旧exe对照]
"""
import struct
import zlib
import sys

MAGIC = b'MEI\014\013\012\013\016'

NEW = sys.argv[1] if len(sys.argv) > 1 else r'D:/rov_pkg_v35/dist/ROV_ControlStation_v3.5.exe'
OLD = sys.argv[2] if len(sys.argv) > 2 else r'D:/RC/ROV控制站_v3.3/ROV_ControlStation_v3.5.exe'

# 新版 pc_main2.py 才有的标识（旧版必无）
WANT = [b'MSG_PORT', b'parse_msg', b'parse_auv', b'MsgThread', b'_on_msg', b'_msg_thread']


def parse(path):
    data = open(path, 'rb').read()
    pos = data.rfind(MAGIC)
    if pos < 0:
        raise SystemExit('%s: 找不到 PyInstaller MAGIC' % path)
    magic, pkg_len, toc_off, toc_len, pyvers, pylib = struct.unpack(
        '!8sIIii64s', data[pos:pos + 88])
    start = pos + 88 - pkg_len
    end = start + toc_off + toc_len
    p = start + toc_off
    entries = {}
    while p < end:
        ulen = struct.unpack('!i', data[p:p + 4])[0]
        e_pos, e_csize, e_usize = struct.unpack('!III', data[p + 4:p + 16])
        e_cmprs = data[p + 16]
        e_type = chr(data[p + 17])
        name = data[p + 18:p + ulen].split(b'\x00')[0].decode('utf-8', 'replace')
        entries[name] = (e_pos, e_csize, e_usize, e_cmprs, e_type)
        p += ulen
    return data, start, entries, pyvers


def entry_raw(data, start, ent):
    pos, csize, usize, cmprs, typ = ent
    raw = data[start + pos:start + pos + csize]
    if cmprs:
        raw = zlib.decompress(raw)
    assert len(raw) == usize, 'usize 不符 %d != %d' % (len(raw), usize)
    return raw


def main():
    rc = 0
    for tag, path in (('新', NEW), ('旧', OLD)):
        print('=' * 64)
        print('%s版：%s' % (tag, path))
        try:
            data, start, entries, pyvers = parse(path)
        except SystemExit as e:
            print('  !!', e)
            continue
        print('  %d 字节 / 顶层条目 %d / pyvers %d / start_offset %d' %
              (len(data), len(entries), pyvers, start))
        e = entries.get('pc_main2')
        if not e:
            print('  !! 顶层没有 pc_main2 条目')
            continue
        raw = entry_raw(data, start, e)
        print('  pc_main2: pos=%d csize=%d usize=%d cmprs=%s type=%s -> 解压 %d 字节'
              % (e[0], e[1], e[2], e[3], e[4], len(raw)))
        hit = {k.decode(): raw.count(k) for k in WANT}
        for k, v in hit.items():
            print('    %-12s %s' % (k, ('OK x%d' % v) if v else '—'))
        ok = all(hit.values())
        print('  => %s' % ('✔ 含 $MSG 新代码' if ok else '✘ 不含 $MSG（旧版）'))
        if tag == '新' and not ok:
            rc = 1
    print('=' * 64)
    print('核验结果：%s' % ('PASS' if rc == 0 else 'FAIL'))
    return rc


if __name__ == '__main__':
    sys.exit(main())
