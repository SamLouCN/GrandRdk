#!/bin/bash
# 停止 camera_kalman（viskf）
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
P="$ROOT/logs/camera_kalman.pid"
if [ ! -f "$P" ]; then
  echo "没有 pid 文件（$P），本工程大概没在后台跑。"
  echo "若确在跑，用： ps -ef | grep 'viskf[.]py'"
  exit 0
fi
PID="$(cat "$P" 2>/dev/null || true)"
if [ -n "${PID:-}" ] && kill -0 "$PID" 2>/dev/null; then
  echo "停止 viskf pid=$PID"
  kill "$PID" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 "$PID" 2>/dev/null || break
    sleep 0.3
  done
  if kill -0 "$PID" 2>/dev/null; then
    echo "  优雅退出超时，SIGKILL"
    kill -9 "$PID" 2>/dev/null
  fi
else
  echo "pid=$PID 已不在（进程可能自己退了），清掉 pid 文件"
fi
rm -f "$P"
echo "已停止。"
