#!/bin/bash
# ============================================================
# stop.sh — 停止 GrandRdkRefresh 所有进程
# ============================================================
set -u                                       # 启用未定义变量检查; 拼错变量名立刻报错退出
cd "$(dirname "$0")" || exit 1               # 切到脚本所在目录(GrandRDK 根); 失败就退出 1

echo "[stop.sh] 停止检测与 Web 进程..."      # 给用户一个明确的进度提示
pkill -f "src/front.py"         2>/dev/null  # 向前视检测进程发 SIGTERM (匹配 cmdline 含 src/front.py)
pkill -f "src/bottom.py"        2>/dev/null  # 向下视检测进程发 SIGTERM
pkill -f "src/web_server.py"    2>/dev/null  # 向 FastAPI :5000 Web 进程发 SIGTERM
pkill -f "src/read_altimeter.py" 2>/dev/null # 向高度计进程发 SIGTERM
pkill -f "src/to32/main.py"          2>/dev/null   # To32 中位机(已注册 SIGTERM 处理器, 优雅退出)
# [2026-10-04 光流停用] 仍保留这行: 万一有旧实例/手动起的 flow_speed 残留, 一并清掉
pkill -f "src/flow_speed.py"     2>/dev/null   # 光流测速(已停用, 仅清理残留)
pkill -f "src/show_cam.py"       2>/dev/null   # 第三路相机推流(CAM3 :8084)
pkill -f "kalman/depth_kalman/main.py" 2>/dev/null # 深度卡尔曼(2026-10-08 起纳入停止范围; main.py 注册了 SIGTERM 处理器, 优雅退出)

sleep 0.5                                    # 给上面的进程留出 SIGTERM 处理时间(默认动作: 退出)

# ---------- 强制收尾 ----------
# 已知问题(2026-09-17 实测): 上位机若正连着 web_server 的 /cam1 /cam2 MJPEG 流,
# uvicorn 的优雅关闭会等 StreamingResponse 生成器结束而长期不退出(端口已释放、
# 进程挂着)。这里在宽限期后对仍存活的进程补一刀 SIGKILL(纯读共享内存, 无状态, 安全)。
sleep 1.5                                    # 宽限期: 让 SIGTERM 优先走优雅路径
for pat in "src/front.py" "src/bottom.py" "src/web_server.py" \
           "src/read_altimeter.py" "src/flow_speed.py" "src/show_cam.py" "src/to32/main.py" \
           "kalman/depth_kalman/main.py"; do  # 遍历所有应被停掉的模式
    if pgrep -f "$pat" >/dev/null 2>&1; then  # 还活着: 需要 SIGKILL 兜底
        echo "[stop.sh] 强制结束残留进程: $pat"  # 提示用户: 这条是被强杀的(便于排查)
        pkill -9 -f "$pat" 2>/dev/null       # 发 SIGKILL: 不给进程清理机会, 必定终止
    fi                                       # if 结束
done                                         # for 循环结束
sleep 0.5                                    # 等 SIGKILL 落实, 避免下一行 rm 撞到仍持有 fd 的进程

echo "[stop.sh] 清理共享内存..."             # 提示开始清理 /dev/shm
rm -f /dev/shm/momo_*.bin /dev/shm/momo_*.json 2>/dev/null  # 删除所有 momo_ 开头的二进制/JSON(帧、检测、遥测、统计)
rm -f "$ROOT/src/kalman/depth_kalman/logs/depth_kalman.pid" 2>/dev/null  # 深度卡尔曼 pidfile 一并清掉, 防陈旧 pid 干扰下次判活

echo "[stop.sh] 完成"                        # 全部完成, 给用户收尾提示