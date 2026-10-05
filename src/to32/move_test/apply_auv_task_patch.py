# -*- coding: utf-8 -*-
"""把 move_test/ 从"旧的 mission.py 单体状态机 + AuvModeStub 占位壳"
   切成"新的 auv_task 任务化执行器"—— 自动备份 + 逐项报告，任一必改项失败就**不落盘**

板端跑法（cd 到中位机目录）:
    cd /userdata/GrandRDK/src/to32/move_test
    cp -a ../../../config/auv_config.py /tmp/auv_config.py.bak      # 配置另备一份
    python3 apply_auv_task_patch.py --dry-run      # 先看每一条能不能匹配上
    python3 apply_auv_task_patch.py                # 确认无误再真改

改的是三个文件:
  1) move_test/mode_auv.py        （3 处：import / 构造 / tick 末尾处理切模式请求）
  2) move_test/mode_dispatcher.py （3 处：import AuvMode / 用真 AuvMode / 回挂 dispatcher）
  3) ../../run.sh                 （1 处：PYTHONPATH 补 move_test 段）—— 用 --run-sh 指定路径，默认不动

★ 为什么必须回挂 dispatcher：
  mode_base.py 只有 ctx，**没有 dispatcher 引用**，所以 AUV 上浮后想切回 ROV，
  模式内部拿不到切换入口 —— 必须在这里补 `m.dispatcher = self`。

★★ MODE_ROV 同名巨坑（再强调一次）：
  要切的是 **to32_config.MODE_ROV = 0（dispatcher 模式 id = 有线 ROV）**；
  link_stm32.MODE_ROV = 0x03 是"0x04 帧码 = 无线 ROV"，S100 禁发（frame_mode 返回 None）。
  写错那个，0x04 一帧都发不出去，机器人会留在 AUV 不动，而且只在 stderr 多一行。
"""
import os
import re
import sys
import time

_NEW_MISSION = ('self.mission = TaskRunner(C, vision=VisionIF(C, log=self.log),\n'
                '                                  depth=DepthIF(C, log=self.log),\n'
                '                                  viskf=ViskfIF(C, log=self.log), log=self.log)')


def _a2(src):
    """A2：把 `self.mission = Mission(...)` 整句换成 TaskRunner(...)

    ⚠ 不能写 `Mission\\([^;]*?log=self\\.log\\)` 这种正则：非贪婪会在
      `VisionIF(C, log=self.log)` 里就提前收尾，把后半句留在原地 → 语法直接崩。
      这里改用**括号配平扫描**定位整句，对换行的多行调用同样稳。
    """
    m = re.search(r'self\.mission\s*=\s*Mission', src)
    if not m:
        return src, False
    i = src.find('(', m.end())
    if i < 0:
        return src, False
    depth, j = 0, i
    while j < len(src):
        c = src[j]
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                break
        j += 1
    if depth != 0:
        return src, False
    return src[:m.start()] + _NEW_MISSION + src[j + 1:], True


def _a3(m):
    """A3 的替换函数：把切模式处理**回退一级缩进**，追加到 tick() 函数体里

    ⚠ 不能沿用 push 那一行的缩进：它嵌在 `if self.report is not None:` 里，
      照抄缩进会导致"回传关掉时（AUV_REPORT_ENABLED=False）切模式请求永远取不到"，
      机器人上浮后卡在 AUV 不回 ROV。
    """
    ind = m.group(1)[:-4] if len(m.group(1)) >= 4 else ''
    i2 = ind + '    '
    i3 = ind + '        '
    return '\n'.join([
        m.group(0),
        ind + '# [auv_task] 上浮完成后请求切回有线 ROV（放在 report 判断之外，回传关了也要生效）',
        ind + 'mr = self.mission.pop_mode_request()',
        ind + 'if mr is not None:',
        i2 + 'd = getattr(self, "dispatcher", None)',
        i2 + 'if d is not None and hasattr(d, "switch_mode"):',
        i3 + 'd.switch_mode(mr)  # ★ 传的是 dispatcher 模式 id(cfg.MODE_ROV=0)',
        i2 + 'else:',
        i3 + 'self.log("[AUV] 收到切模式请求 %s，但 dispatcher 未回挂" % mr)',
    ])


# ---------------------------------------------------------------- 补丁定义
# 每条: (编号, 说明, 正则, 替换为, 是否必改)
def _p_mode_auv():
    return [
        ('A1', 'import 换成任务化执行器',
         r'(?m)^from mission import Mission, apply_yaw_mirror.*$',
         'from auv_task import TaskRunner, apply_yaw_mirror  # [auv_task] 任务化执行器（替代 mission.Mission）',
         True, 'from auv_task import TaskRunner'),
        ('A2', 'Mission(...) 换成 TaskRunner(...)（补 viskf）', _a2, None, True,
         'TaskRunner(C, vision='),
        ('A3', 'tick 末尾处理"上浮后切回 ROV"请求',
         r'(?m)^(\s*)self\.report\.push\(make_snapshot\(now, cmd, self\.mission, self\.last_tel\)\)',
         lambda m: _a3(m),
         True, 'pop_mode_request()'),
        ('A4', '补 viskf 接口 import（A2 用到）',
         r'(?m)^from depth_if import DepthIF.*$',
         lambda m: (m.group(0) + '\n'
                    + 'from viskf_if import ViskfIF  # [auv_task] 图像卡尔曼接口（过门用）'),
         False, 'from viskf_if import ViskfIF'),   # 已经导入过就跳过
    ]


def _p_dispatcher():
    return [
        ('B1', 'import 真实 AuvMode（替换占位壳）',
         r'(?m)^from mode_rov import RovMode.*$',
         lambda m: (m.group(0) + '\n'
                    + 'from mode_auv import AuvMode  # [auv_task] 真实 AUV 模式（任务化执行器）'),
         # ⚠ 标记必须带上行尾注释：原文件注释里就有 "原 `from mode_auv import AuvMode` 已移除"，
         #   只匹配裸 import 会被那句注释骗过 → import 永远加不上 → 运行时 NameError。
         True, 'from mode_auv import AuvMode  # [auv_task]'),
        ('B2', '注册真 AuvMode 而不是 AuvModeStub',
         r'auv\s*=\s*AuvModeStub\(self\)',
         'auv = AuvMode(self)  # [auv_task] 接回真实 AUV 自主运动',
         True, 'auv = AuvMode(self)'),
        ('B3', '回挂 dispatcher（AUV 上浮后要能自己切回 ROV）',
         r'(?m)^(\s*)self\.modes\[auv\.id\]\s*=\s*auv.*$',
         lambda m: (m.group(0) + '\n'
                    + m.group(1) + 'auv.dispatcher = self  # [auv_task] 回挂：模式内部才能请求切模式'),
         True, 'auv.dispatcher = self'),
        ('B4', '其余模式也回挂 dispatcher',
         r'(?m)^(\s*)self\.modes\[m\.id\]\s*=\s*m.*$',
         lambda m: (m.group(0) + '\n'
                    + m.group(1) + 'm.dispatcher = self  # [auv_task] 回挂：模式内部才能请求切模式'),
         False, 'm.dispatcher = self'),
    ]


def _p_run_sh():
    return [
        ('C1', 'PYTHONPATH 补 move_test 段（auv_task 要能被 import 到）',
         r'(?m)^(PYTHONPATH=.*?)(\s*\\\s*)?$',
         lambda m: (m.group(1)
                    + ('' if 'move_test' in m.group(1) else ':$SRC_DIR/to32/move_test')
                    + (m.group(2) or '')),
         False, None),
    ]


# ---------------------------------------------------------------- 执行
def patch_file(path, rules, dry=False):
    """对单个文件应用补丁；返回 (是否可写, 明细列表)"""
    try:
        src = open(path, 'r', encoding='utf-8').read()
    except IOError as e:
        return False, [('READ', 'FAIL', '读不到 %s: %s' % (path, e))]
    out, rows = src, []
    for rid, desc, pat, rep, required, marker in rules:
        # ★ 幂等保护：标记已存在 = 这条已经改过了，直接跳过（防止二次运行重复插入）
        if marker and marker in out:
            rows.append((rid, 'SKIP', '%s（已改过）' % desc))
            continue
        if callable(pat):                      # 自定义规则：整串进整串出（括号配平等复杂替换）
            out2, ok = pat(out)
            if not ok:
                rows.append((rid, ('FAIL' if required else 'SKIP'), '%s（没匹配到）' % desc))
                continue
            out = out2
            rows.append((rid, 'OK', '%s（已改）' % desc))
            continue
        n = len(re.findall(pat, out))
        if n == 0:
            rows.append((rid, ('FAIL' if required else 'SKIP'), '%s（没匹配到）' % desc))
            continue
        out = re.sub(pat, rep, out, count=1)
        rows.append((rid, 'OK', '%s（匹配 %d 处，已改 1 处）' % (desc, n)))
    ok = all(r[1] != 'FAIL' for r in rows)
    if ok and out == src:                      # 全部 SKIP = 早就改过了，别再备份一份
        rows.append(('--', 'SKIP', '内容与目标一致，无需改动'))
        return ok, rows
    if ok and not dry and out != src:
        bak = '%s.bak_auvtask_%s' % (path, time.strftime('%Y%m%d_%H%M%S'))
        open(bak, 'w', encoding='utf-8').write(src)
        open(path, 'w', encoding='utf-8').write(out)
        rows.append(('--', 'OK', '已备份 %s' % bak))
    elif ok and dry:
        rows.append(('--', 'OK', '[dry-run] 不落盘'))
    return ok, rows


def main():
    argv = sys.argv[1:]
    dry = '--dry-run' in argv
    here = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
    root = os.getcwd()
    if '--root' in argv:
        root = argv[argv.index('--root') + 1]
    run_sh = argv[argv.index('--run-sh') + 1] if '--run-sh' in argv else None

    jobs = [
        (os.path.join(root, 'mode_auv.py'), _p_mode_auv(), 'move_test/mode_auv.py'),
        (os.path.join(root, 'mode_dispatcher.py'), _p_dispatcher(), 'move_test/mode_dispatcher.py'),
    ]
    if run_sh:
        jobs.append((run_sh, _p_run_sh(), run_sh))

    all_ok = True
    for path, rules, label in jobs:
        print('== %s ==' % label)
        if not os.path.exists(path):
            print('  [SKIP] 文件不存在: %s' % path)
            continue
        ok, rows = patch_file(path, rules, dry=dry)
        for rid, st, msg in rows:
            print('  [%s] %s %s' % (st, rid, msg))
        all_ok = all_ok and ok
    print('')
    if all_ok:
        print('全部必改项匹配成功%s。' % ('（dry-run，未落盘）' if dry else '，已写入'))
        print('下一步: 配置里确认 AUV_TEST_MODE=False（验收）或 True + AUV_TEST_STAGES=[...]（测试）')
        return 0
    print('有必改项没匹配上 —— **没有改动任何文件**。请人工核对上文 FAIL 项后改脚本正则。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
