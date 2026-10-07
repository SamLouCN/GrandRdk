"""PC和S100共享的任务PID协议；不会发送给STM32。"""
from decimal import Decimal, InvalidOperation
import re

REQUEST = re.compile(r"[A-Za-z0-9_-]{1,32}\Z")
LOOP = re.compile(r"[a-z][a-z_]{0,31}\Z")


def _gains(values):
    checked = []
    for value in values:
        try:
            v = Decimal(str(value))
        except (InvalidOperation, ValueError):
            raise ValueError("PID参数不是有效数值") from None
        if not v.is_finite() or not Decimal('-327.68') <= v <= Decimal('327.67'):
            raise ValueError("PID参数超出范围")
        if v * 100 != (v * 100).to_integral_value():
            raise ValueError("任务PID最多两位小数")
        checked.append(float(v))
    return tuple(checked)


def _identity(request, loop):
    if not isinstance(request, str) or not REQUEST.fullmatch(request):
        raise ValueError("请求标识不合法")
    if not isinstance(loop, str) or not LOOP.fullmatch(loop):
        raise ValueError("任务标识不合法")


def build_task_pid(request, loop, p, i, d):
    _identity(request, loop)
    gains = _gains((p, i, d))
    return ("$TASKPID,%s,%s,%.2f,%.2f,%.2f#\r\n" % (request, loop, *gains)).encode('ascii')


def parse_task_pid(frame):
    try:
        s = frame.decode('ascii') if isinstance(frame, bytes) else frame
        s = s.strip()
        if not s.startswith('$TASKPID,') or not s.endswith('#'):
            return None
        fields = s[len('$TASKPID,'):-1].split(',')
        if len(fields) != 5:
            return None
        request, loop = fields[:2]
        _identity(request, loop)
        p, i, d = _gains(fields[2:])
        return dict(request=request, loop=loop, p=p, i=i, d=d)
    except (UnicodeError, AttributeError, TypeError, ValueError):
        return None


def build_task_pid_ack(request, loop, gains=None, error=None):
    _identity(request, loop)
    if error:
        if not re.fullmatch(r'[A-Z_]{1,32}', error):
            raise ValueError("错误码不合法")
        return '$TASKPID_ACK,%s,%s,ERR,%s#\r\n' % (request, loop, error)
    values = _gains(gains)
    return '$TASKPID_ACK,%s,%s,OK,%.2f,%.2f,%.2f#\r\n' % (request, loop, *values)


def parse_task_pid_ack(frame):
    try:
        s = frame.decode('ascii') if isinstance(frame, bytes) else frame
        s = s.strip()
        if not s.startswith('$TASKPID_ACK,') or not s.endswith('#'):
            return None
        f = s[len('$TASKPID_ACK,'):-1].split(',')
        if len(f) not in (4, 6):
            return None
        request, loop, status = f[:3]
        _identity(request, loop)
        if status == 'OK' and len(f) == 6:
            return dict(request=request, loop=loop, ok=True, gains=_gains(f[3:]))
        if status == 'ERR' and len(f) == 4 and re.fullmatch(r'[A-Z_]{1,32}', f[3]):
            return dict(request=request, loop=loop, ok=False, error=f[3])
    except (UnicodeError, AttributeError, TypeError, ValueError):
        pass
    return None
