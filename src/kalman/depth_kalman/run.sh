#!/bin/bash
# depth_kalman 独立启停脚本
#
#   ./run.sh                    前台跑，读 /dev/shm（配置里的 SHM_DIR）
#   ./run.sh --daemon           后台跑（nohup），日志进 logs/depth.log
#   ./run.sh --selftest         合成真值自检，不碰硬件
#   ./run.sh --mock             离线端到端联调：起 mock 喂数 + 前台跑滤波
#   ./run.sh --mock-daemon      同上，但两个都进后台
#   ./run.sh --shm-dir DIR ...  换数据目录
#   ./stop.sh                   停止
#
# [2026-10-01 新布局] 按 GrandRDK 的分类归档：
#   配置 → <宿主>/config/depth_config.py      （与 auv_config.py 等统一）
#   源码 → 本目录平铺（main.py / fusion.py / model.py / ...），不再套一层 src/
# 所以生产代码 = <宿主>/config/depth_config.py + 本目录的 *.py + 本脚本；
# --selftest / --mock 用到的东西都在 tests/，整目录删掉不影响运行。
# [2026-10-01 R8] tests/ 已迁到 hwless_tests/legacy_depth_kalman/（板端"测试代码只在
#   hwless_tests/ 一处"的约定）；本脚本用 TESTS 变量指过去，加 --selftest/--mock 时
#   若那里也不存在，会给出人话提示。生产运行（默认/--daemon）完全不碰 tests。
#
# 2026-09-25 迁入 GrandRDK（原 /userdata/kalman/depth_kalman 保留为备份）。
# 2026-10-01 归位到 src/kalman/depth_kalman/ 并按 GrandRDK 分类拆开：
#   config/depth_config.py → /userdata/GrandRDK/config/depth_config.py
#   老包 src/*.py         → /userdata/GrandRDK/src/kalman/depth_kalman/*.py（平铺）
#   （老的 GrandRDK/depth_kalman/ 与 /userdata/kalman/depth_kalman/ 都保留为备份）
# 硬约束（迁入后仍然成立）：
#   * 本工程**不开任何串口**（数据源只有共享内存：只读 momo_telemetry/momo_alt，只写 momo_depth）
#   * **不被 GrandRDK/run.sh 自动拉起** —— 那套 wait/kill 逻辑会放大故障。
#     生命周期由 GrandRDK/src/to32/kalman_launcher.py 的 DepthKalmanLauncher 托管：
#     上位机切到 AUV 模式（$CMD mode=1）时 on_enter 自动拉起，退出 AUV 时 on_exit 停掉自己起的那个。
#     要单独调深度就用本脚本 ./run.sh --daemon（手动起的不会被 AUV 抢停，这是设计内行为）。
#   * **不改 To32 / GrandRDK 的既有代码**
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
# tests/ 迁到 hwless_tests 后的落点（R8）；若本机仍是老布局则回退到本目录 tests/
REPO_FOR_TESTS="$(cd "$ROOT/../../.." && pwd)"
TESTS="$REPO_FOR_TESTS/hwless_tests/legacy_depth_kalman"
[ -d "$TESTS" ] || TESTS="$ROOT/tests"
PY="${PYTHON:-python3}"
# ★ 配置已收进宿主的 config/，源码平铺在本目录 —— PYTHONPATH 必须这两段都给
REPO="$(cd "$ROOT/../../.." && pwd)"
export PYTHONPATH="$REPO/config:$ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$ROOT/logs"
PIDFILE="$ROOT/logs/depth_kalman.pid"
MOCKPID="$ROOT/logs/mock_feed.pid"

# --selftest / --mock 需要 tests/（测试代码，不在生产代码里）。缺了就给人话，别甩 Python 报错。
need_tests() {
  if [ ! -f "$TESTS/$1" ]; then
    echo "[run.sh] 找不到 $TESTS/$1" >&2
    echo "         自检/mock 的测试代码在 hwless_tests/legacy_depth_kalman/（R8 起）；" >&2
    echo "         生产运行（默认或 --daemon）不需要它。" >&2
    exit 1
  fi
}

case "${1:-}" in
  --selftest|-t)
    need_tests test_depth_kalman.py
    shift
    exec "$PY" "$TESTS/test_depth_kalman.py" "$@"
    ;;
  --mock|--mock-daemon)
    need_tests mock_feed.py
    MODE="$1"; shift
    D="${MOCK_DIR:-/tmp/dk_mock}"
    # mock 真值的池底深度 3.0 m、锚路(B)安装 dz=0.10 m → H_M 必须给 2.90，
    # 否则高度计新息里有 2 m 常量偏移、会被门限全拒（这是特意保留的真实行为，不是 bug）。
    HM="${MOCK_HM:-2.9}"
    ARGS=("$@")
    # 若用户没指定 --shm-dir / --h-m，就补上
    case " ${ARGS[*]-} " in *" --shm-dir "*) ;; *) ARGS+=(--shm-dir "$D");; esac
    case " ${ARGS[*]-} " in *" --h-m "*) ;; *) ARGS+=(--h-m "$HM");; esac
    if [ "$MODE" = "--mock" ]; then
      echo "[run.sh] 起 mock 喂数 → $D（H_M=$HM，加 --accel-unit m/s^2）"
      "$PY" "$TESTS/mock_feed.py" --dir "$D" --accel-unit m/s^2 &
      MOCK=$!
      trap 'kill $MOCK 2>/dev/null' EXIT INT TERM
      "$PY" "$ROOT/main.py" "${ARGS[@]}"
      exit $?
    else
      nohup "$PY" "$TESTS/mock_feed.py" --dir "$D" --accel-unit m/s^2 \
        > "$ROOT/logs/mock_feed.log" 2>&1 &
      echo $! > "$MOCKPID"
      echo "[run.sh] mock 已后台启动 pid=$(cat $MOCKPID)"
      exec "$0" --daemon "${ARGS[@]}"
    fi
    ;;
  --daemon|--background)
    shift
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "已在运行 pid=$(cat "$PIDFILE")；先 ./stop.sh" >&2
      exit 1
    fi
    nohup "$PY" "$ROOT/main.py" --quiet "$@" >> "$ROOT/logs/depth.log" 2>&1 &
    echo $! > "$PIDFILE"
    sleep 1
    if kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
      echo "已启动 pid=$(cat "$PIDFILE")，日志 $ROOT/logs/depth.log"
    else
      echo "启动失败，看 $ROOT/logs/depth.log" >&2
      rm -f "$PIDFILE"
      exit 1
    fi
    ;;
  -h|--help)
    sed -n '2,17p' "$0"
    ;;
  *)
    exec "$PY" "$ROOT/main.py" "$@"
    ;;
esac
