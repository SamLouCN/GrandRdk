#!/bin/bash
# logs.sh — 实时查看日志
cd "$(dirname "$0")" || exit 1              # 切到脚本所在目录(GrandRDK 根); cd 失败就退出

case "${1:-all}" in                          # 按第一个参数分派; 没传默认 'all' (tail 所有日志)
    front)   tail -f logs/front.log ;;       # front 子: 实时跟踪前视检测进程日志
    bottom)  tail -f logs/bottom.log ;;      # bottom 子: 实时跟踪下视检测进程日志
    web)     tail -f logs/web_server.log ;;  # web 子: 实时跟踪 FastAPI :5000 的访问/错误日志
    alt)     tail -f logs/altimeter.log ;;   # alt 子: 实时跟踪高度计读数日志
    to32)    tail -f logs/to32_main.log ;;   # to32 子: 实时跟踪中位机主日志
    flow)    echo "[logs.sh] 光流测速已于 2026-10-04 停用; 显示历史日志 logs/flow_speed.log" >&2; tail -f logs/flow_speed.log ;;  # flow 子: 已停用, 仅看历史日志
    showcam) tail -f logs/show_cam.log ;;    # showcam 子: 实时跟踪第三路相机按需推流日志
    all)     tail -f logs/*.log ;;           # all 子: 一次性 tail 所有 .log 文件(看不到新增 log 文件)
    *)       echo "用法: $0 [front|bottom|web|alt|to32|flow|showcam|all]"; exit 1 ;;  # 其它参数: 打印用法并退出非 0
esac                                         # case 块结束