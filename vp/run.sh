#!/bin/bash
# ============================================================
# run.sh — vp6.0 启动脚本（双相机空闲超时单向切换 + YOLO）
#   - taskset 绑 CPU 0-5（整机 6 核都可用，配合 main.py 内
#     N_WORKERS=6 多 worker 线程 -> 多实例并行榨 BPU 算力）
#   - 不做推流（相对 vp5.0 去掉 mjpeg_bridge / telem）
#   - 其余参数全部透传给 main.py，也可直接 python3 main.py ...
# 用法:
#   ./run.sh                            # 真实相机（cam1 空闲超时 -> cam2）
#   ./run.sh --timeout 3                # 覆盖 cam1 空闲超时（联调）
#   ./run.sh --virtual --no-target      # 无实体相机验证切换流程
#   ./run.sh --frames 100               # 限帧验收
# ============================================================
cd "$(dirname "$0")" || exit 1

export OMP_NUM_THREADS=6
export OPENBLAS_NUM_THREADS=6
export MKL_NUM_THREADS=6
export NUMEXPR_NUM_THREADS=6

echo "run.sh: taskset 绑定 CPU 0-5 (6 核) 运行 main.py $*"
exec taskset -c 0-5 python3 main.py "$@"
