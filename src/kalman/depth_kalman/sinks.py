# -*- coding: utf-8 -*-
"""输出侧：原子写 ``momo_depth.json`` + 带轮转的日志。

* **原子写**：先写同目录临时文件 → ``fsync`` → ``os.replace``。读端永远看到完整 JSON。
* 日志：时间戳 + 级别，超过 ``LOG_MAX_BYTES`` 自动轮转，最多留 ``LOG_ROTATE_KEEP`` 份。
"""
import json
import os
import sys
import tempfile
import time


class Log(object):
    LEVELS = ('DEBUG', 'INFO', 'WARN', 'ERROR')

    def __init__(self, path=None, level='INFO', also_stdout=True, max_bytes=0, keep=2):
        self.path = path
        self.level = level.upper() if level else 'INFO'
        self.also_stdout = also_stdout
        self.max_bytes = int(max_bytes or 0)
        self.keep = int(keep or 0)
        self._fh = None
        if self.path:
            d = os.path.dirname(self.path)
            if d:
                try:
                    os.makedirs(d, exist_ok=True)
                except OSError:
                    pass
            try:
                self._fh = open(self.path, 'a', encoding='utf-8', buffering=1)
            except OSError:
                self._fh = None

    # ------------------------------------------------------------
    def _emit(self, lvl, msg):
        if self.LEVELS.index(lvl) < self.LEVELS.index(self.level):
            return
        line = '%s [%s] %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), lvl, msg)
        if self.also_stdout:
            sys.stdout.write(line + '\n')
            sys.stdout.flush()
        if self._fh is not None:
            try:
                self._fh.write(line + '\n')
            except OSError:
                pass
            self._maybe_rotate()

    def _maybe_rotate(self):
        if not self.max_bytes or not self.path:
            return
        try:
            if self._fh is not None:
                self._fh.flush()
            if os.path.getsize(self.path) < self.max_bytes:
                return
            if self._fh is not None:
                self._fh.close()
                self._fh = None
            for i in range(self.keep, 0, -1):
                src = '%s.%d' % (self.path, i)
                dst = '%s.%d' % (self.path, i + 1)
                if os.path.exists(src):
                    os.replace(src, dst)
            os.replace(self.path, self.path + '.1')
            self._fh = open(self.path, 'a', encoding='utf-8', buffering=1)
        except OSError:
            pass

    def debug(self, m):
        self._emit('DEBUG', m)

    def info(self, m):
        self._emit('INFO', m)

    def warn(self, m):
        self._emit('WARN', m)

    def error(self, m):
        self._emit('ERROR', m)

    def close(self):
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None


def atomic_write_json(path, obj, indent=None):
    """原子写 JSON。返回 True/False。"""
    d = os.path.dirname(path) or '.'
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        pass
    tmp = None
    try:
        fd, tmp = tempfile.mkstemp(prefix='.dk_', suffix='.tmp', dir=d)
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(obj, f, ensure_ascii=False, indent=indent, default=_json_default)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        return True
    except (OSError, ValueError, TypeError):
        if tmp:
            try:
                os.unlink(tmp)
            except OSError:
                pass
        return False


def _json_default(o):
    try:
        return float(o)
    except (TypeError, ValueError):
        return str(o)
