#!/bin/bash
# camera_kalman（viskf）独立启停脚本
#
#   ./run.sh                    前台跑，读 /dev/shm（路径见 config/viskf_config.py）
#   ./run.sh --daemon           后台跑（nohup），日志进 logs/viskf.log
#   ./run.sh --status           打印**实际命中的配置文件**与数据源新鲜度后退出
#   ./run.sh --selftest         11 项无硬件自检（改完参数的第一道闸）
#   ./run.sh --mock             离线端到端：合成数据走一遍真实文件读写链路
#   ./run.sh --config FILE ...  换配置文件（也可用环境变量 VISKF_CONFIG）
#   ./run.sh --shm-dir DIR ...  换共享内存目录（联调用）
#   ./stop.sh                   停止
#
# [2026-10-01 新布局] 按 GrandRDK 的分类归档：
#   配置 → <宿主>/config/viskf_config.py   （与 auv_config.py 等统一）
#   源码 → 本目录平铺的 viskf.py           （不再套一层 src/）
# 生产代码 = <宿主>/config/viskf_config.py + 本目录 viskf.py + 本脚本；
# --selftest / --mock 用到的东西都在 tests/，整目录删掉不影响运行。
#
# 2026-10-01 首次上板到 /userdata/GrandRDK/src/kalman/camera_kalman/，
#   并按 GrandRDK 分类拆开（配置进 config/、源码平铺在本目录）。
# 硬约束（与 depth_kalman 同款）：
#   * 本工程**不开任何串口**、**不接 GrandRDK/run.sh**、**只读 /dev/shm**
#     （读 momo_det_front.json + 可选 momo_telemetry.json，只写 momo_viskf.json）
#   * 生命周期由 GrandRDK/src/to32/kalman_launcher.py 的 ViskfLauncher 托管：
#     上位机切到 AUV 模式时 on_enter 自动拉起，退出 AUV 时 on_exit 停掉自己起的那个。
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
PY="${PYTHON:-python3}"
# ★ 配置已收进宿主的 config/，源码平铺在本目录 —— PYTHONPATH 必须这两段都给
REPO="$(cd "$ROOT/../../.." && pwd)"
export PYTHONPATH="$REPO/config:$ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$ROOT/logs"
PIDFILE="$ROOT/logs/camera_kalman.pid"
LOG="$ROOT/logs/viskf.log"

# --selftest / --mock 需要 tests/（测试代码，不在生产代码里）。缺了就给人话，别甩 Python 栈。
need_tests() {
  if [ ! -f "$ROOT/tests/$1" ]; then
    echo "[run.sh] 缺少 tests/$1 —— 本目录大概只拷了生产代码。" >&2
    echo "         自检与 mock 联调需要 tests/ 目录；生产运行不需要。" >&2
    exit 1
  fi
}

case "${1:-}" in
  --selftest|-t)
    need_tests test_viskf.py
    shift
    exec "$PY" "$ROOT/tests/test_viskf.py" --selftest "$@"
    ;;
  --mock)
    need_tests test_viskf.py
    shift
    exec "$PY" "$ROOT/tests/test_viskf.py" --mock "$@"
    ;;
  --status)
    shift
    exec "$PY" "$ROOT/viskf.py" --status "$@"
    ;;
  --daemon|--background)
    shift
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "已在运行 pid=$(cat "$PIDFILE")；先 ./stop.sh" >&2
      exit 1
    fi
    nohup "$PY" "$ROOT/viskf.py" --quiet "$@" >> "$LOG" 2>&1 &
    echo $! > "$PIDFILE"
    sleep 1
    if kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "已启动 pid=$(cat "$PIDFILE")，日志 $LOG"
    else
      echo "启动失败，看 $LOG" >&2
      rm -f "$PIDFILE"
      exit 1
    fi
    ;;
  -h|--help)
    sed -n '2,20p' "$0"
    ;;
  *)
    exec "$PY" "$ROOT/viskf.py" "$@"
    ;;
esac
