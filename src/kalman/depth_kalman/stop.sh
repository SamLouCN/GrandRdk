#!/bin/bash
# 停止 depth_kalman（以及 --mock 起的喂数器）
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
for f in logs/depth_kalman.pid logs/mock_feed.pid; do
  P="$ROOT/$f"
  [ -f "$P" ] || continue
  PID="$(cat "$P" 2>/dev/null || true)"
  if [ -n "${PID:-}" ] && kill -0 "$PID" 2>/dev/null; then
    echo "停止 $(basename "$f" .pid) pid=$PID"
    kill "$PID" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$PID" 2>/dev/null || break
      sleep 0.3
    done
    if kill -0 "$PID" 2>/dev/null; then
      echo "  优雅退出超时，SIGKILL"
      kill -9 "$PID" 2>/dev/null
    fi
  fi
  rm -f "$P"
done
echo "已停止。"
