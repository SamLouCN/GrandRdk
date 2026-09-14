#!/bin/bash
# ============================================================
# run.sh — front/bottom 双进程 + 推流桥 + 遥测滚动上传 + 光流测速（同一生命周期）
#   - front  : 绑定 CPU 0-2 (3 核)；bottom: 绑定 CPU 3-5 (3 核)
#   - 每个进程内部开启多个线程（front.py/bottom.py 内 N_WORKERS=3）
#   - mjpeg_bridge.py : STREAM(TCP JPEG) -> HTTP MJPEG /cam1|/cam2 (:5000)
#   - telem_sender.py : 虚拟参数滚动上传 (UDP 8081 -> $TELEM_DST, 10Hz)
#     —— 参数与图像同生命周期：run.sh 停止则上位机收不到任何参数
#   - flow_speed.py(momo_pwmnet) : 读 bottom(下视) 共享帧做光流测速, Web 推流 :$FLOW_PORT
#     —— 与图像同生命周期：run.sh 停止则光流测速停止（日志 /tmp/flow_speed.log）
# 用法:
#   ./run.sh                             # 真实相机，无限运行，Ctrl-C 结束
#   TELEM_DST=192.168.127.100 ./run.sh   # 覆盖遥测目标 IP（默认同上）
#   FLOW_PORT=8080 FLOW_FPS=20 ./run.sh   # 覆盖光流测速 Web 端口/抽帧率（默认 8080/20）
#   ./run.sh --virtual                   # 两路都用虚拟相机（无实体相机联调）
#   ./run.sh --device /dev/video2        # 给两个进程都指定真实设备
#   ./run.sh --frames 60                 # 每进程跑 60 帧后退出（验收用）
#   ./run.sh --no-show                   # 关闭 imshow 显示窗口
# ============================================================
cd "$(dirname "$0")" || exit 1

export OMP_NUM_THREADS=3
export OPENBLAS_NUM_THREADS=3
export MKL_NUM_THREADS=3
export NUMEXPR_NUM_THREADS=3

# 两个相机独立：front 用 FRONT_DEVICE（默认 config FRONT_CAMERA.device），
# bottom 用 BOTTOM_DEVICE（默认 config BOTTOM_CAMERA.device）。互不共享、互不影响。
# 某个相机掉线时，对应进程自行退出，另一个继续跑。
FRONT_DEV="${FRONT_DEVICE:-}"
BOTTOM_DEV="${BOTTOM_DEVICE:-}"
FRONT_ARGS=()
BOTTOM_ARGS=()
VIRTUAL_MODE=0
i=0
args=( "$@" )
while [ $i -lt $# ]; do
    a="${args[$i]}"
    case "$a" in
        --virtual|--no-show|--no-stream) FRONT_ARGS+=("$a"); BOTTOM_ARGS+=("$a"); [ "$a" = "--virtual" ] && VIRTUAL_MODE=1; i=$((i+1));;
        --frames)            FRONT_ARGS+=("$a" "${args[$((i+1))]}"); BOTTOM_ARGS+=("$a" "${args[$((i+1))]}"); i=$((i+2));;
        --frames=*)          FRONT_ARGS+=("$a"); BOTTOM_ARGS+=("$a"); i=$((i+1));;
        --workers|--index)   FRONT_ARGS+=("$a" "${args[$((i+1))]}"); BOTTOM_ARGS+=("$a" "${args[$((i+1))]}"); i=$((i+2));;
        *)                   FRONT_ARGS+=("$a"); BOTTOM_ARGS+=("$a"); i=$((i+1));;
    esac
done
[ -n "$FRONT_DEV" ] && FRONT_ARGS+=(--device "$FRONT_DEV")
[ -n "$BOTTOM_DEV" ] && BOTTOM_ARGS+=(--device "$BOTTOM_DEV")

echo "run.sh: front -> CPU 0-2, bottom -> CPU 3-5"
taskset -c 0-2 python3 front.py "${FRONT_ARGS[@]}" &
PID_FRONT=$!
taskset -c 3-5 python3 bottom.py "${BOTTOM_ARGS[@]}" &
PID_BOTTOM=$!
echo "run.sh: front pid=$PID_FRONT  bottom pid=$PID_BOTTOM"

# 光流测速(独立进程, /userdata/momo_pwmnet): 读 bottom(下视) 共享帧 /dev/shm/momo_flow_bottom.bin, 不占相机
#   依赖 main_config.ENABLE_FLOW_SHARE_BOTTOM=True (bottom 写共享帧);
#   --virtual 虚拟相机不建共享帧 -> 跳过; 共享帧不可用时该进程自行退出, 不影响其它进程
if [ "$VIRTUAL_MODE" = "1" ]; then
    PID_FLOW=""
    echo "run.sh: --virtual 模式跳过光流测速 (虚拟模式不建共享帧)"
else
    FLOW_FPS="${FLOW_FPS:-50}"
    FLOW_PORT="${FLOW_PORT:-8080}"
    # GPU 光流测速: 不绑核 + nice 19 (纯低优先级), 不继承顶层 OMP/OPENBLAS 多线程,
    # 避免与 front/bottom 抢 CPU 核; 光流计算走 Mali-G78AE GPU (OpenCL/UMat).
    nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
        python3 /userdata/momo_pwmnet/flow_speed.py \
        --fps "$FLOW_FPS" --port "$FLOW_PORT" > /tmp/flow_speed.log 2>&1 &
    PID_FLOW=$!
    echo "run.sh: flow_speed pid=$PID_FLOW (光流测速 http :$FLOW_PORT, 抽帧 ${FLOW_FPS}fps; 日志 /tmp/flow_speed.log)"
fi

# MJPEG 桥: 把 vp5.0 STREAM(TCP JPEG) 转 HTTP MJPEG /cam1 /cam2 (供上位机 pc_main2 显示)
python3 mjpeg_bridge.py > /tmp/mjpeg_bridge.log 2>&1 &
PID_BRIDGE=$!
echo "run.sh: mjpeg bridge pid=$PID_BRIDGE (http :5000 /cam1 /cam2)"

# 遥测虚拟参数: 与图像同生命周期 —— run.sh 停止则参数停止（上位机收不到）
TELEM_DST="${TELEM_DST:-192.168.127.100}"
python3 telem_sender.py --dst "$TELEM_DST" --port 8081 --hz 10 > /tmp/telem.log 2>&1 &
PID_TELEM=$!
echo "run.sh: telem pid=$PID_TELEM (UDP :8081 -> $TELEM_DST, 10Hz)"

trap 'kill $PID_FRONT $PID_BOTTOM $PID_BRIDGE $PID_TELEM $PID_FLOW 2>/dev/null' INT TERM

# 等两路检测进程结束；front/bottom 都退出后（如 --frames 跑完）收掉桥、遥测与光流测速
wait $PID_FRONT $PID_BOTTOM
kill $PID_BRIDGE $PID_TELEM $PID_FLOW 2>/dev/null
echo "run.sh: 全部进程已结束，遥测、推流与光流测速已停止"
exit 0
