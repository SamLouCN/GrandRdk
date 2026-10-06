#!/bin/bash
# ============================================================
# status.sh — 查看 GrandRdkRefresh 运行状态
# ============================================================
set -u                                                          # 启用未定义变量检查; 拼错变量名立刻报错
cd "$(dirname "$0")" || exit 1                                  # 切到脚本所在目录(GrandRDK 根); 失败就退出 1
ROOT="$(pwd)"                                                   # 当前绝对路径(项目根); 后面多处会引用
CONFIG_DIR="$ROOT/config"                                       # 配置文件目录: 读 main_config.py 用

echo "========== 进程 =========="                                # 标题: 列出每个 Python 进程是否在跑
for name in front.py bottom.py web_server.py read_altimeter.py flow_speed.py show_cam.py; do  # 7 个主进程名(都是 src/<name>.py)
    pids=$(pgrep -f "src/$name" 2>/dev/null | tr '\n' ' ')      # 用 pgrep -f 按 cmdline 匹配; 多 PID 用换行转空格拼接
    if [ -n "$pids" ]; then                                     # 找到了: 进程在跑
        printf "  %-20s [ON ] pid=%s\n" "$name" "$pids"        # 打印 ON, 把 pid 列出(空格分隔多 pid)
    else                                                        # 没找到: 进程不在
        printf "  %-20s [OFF]\n" "$name"                       # 打印 OFF
    fi                                                          # if 结束
done                                                            # for 循环结束

# 中位机: 已迁入 src/to32/, 仍单独判断
MID_PIDS=$(pgrep -f "src/to32/main.py" 2>/dev/null | tr '\n' ' ')  # To32 中位机单独查: 路径多一层 src/to32/
if [ -n "$MID_PIDS" ]; then                                     # 在跑
    printf "  %-20s [ON ] pid=%s\n" "src/to32/main.py" "$MID_PIDS"  # 打印 ON + pid
else                                                            # 不在
    printf "  %-20s [OFF]\n" "src/to32/main.py"                # 打印 OFF
fi                                                              # if 结束

echo ""                                                         # 空行: 分隔两块
echo "========== 共享内存 =========="                            # 标题: 共享内存文件
ls -lh /dev/shm/momo_* 2>/dev/null || echo "  (无)"            # 列出所有 momo_* 共享文件; 没有就提示"无"

echo ""                                                         # 空行
echo "========== 高度计 (CH348 / Modbus-RTU) =========="          # 标题: 高度计(CH348 多串口, Modbus-RTU 协议)
ALT_PORTS=$(ls /dev/ttyCH9344USB* 2>/dev/null | wc -l)           # 数一下 CH348 的 ttyCH9344USB* 节点(本板 8 个)
echo "  串口节点 : $ALT_PORTS 个 (/dev/ttyCH9344USB0-7, 本程序用 A-E)"  # 提示: 本程序只读 A~E
ALT_PID=$(pgrep -f "src/read_altimeter.py" 2>/dev/null | tr '\n' ' ')  # 高度计进程 pid(可能多进程, 转空格)
if [ -n "$ALT_PID" ]; then                                      # 在跑
    echo "  进程     : pid=$ALT_PID"                            # 打印 pid
else                                                            # 不在
    echo "  进程     : 未运行"                                  # 提示未运行
fi                                                              # if 结束
ALT_LOG="$ROOT/logs/altimeter.log"                              # 高度计日志绝对路径
if [ -f "$ALT_LOG" ]; then                                      # 日志存在
    LATEST=$(grep -E '^\[(OK|!!)\]' "$ALT_LOG" 2>/dev/null | tail -n 3)  # 取最近 3 条 [OK]/[!!] 行
    if [ -n "$LATEST" ]; then                                   # 抓到了
        echo "$LATEST" | sed 's/^/  /'                         # 每行前加 2 个空格缩进再打印
    else                                                        # 还没数据
        echo "  (暂无读数)"                                    # 提示暂无读数
    fi                                                          # if 结束
else                                                            # 日志文件没有
    echo "  (无 altimeter.log)"                                # 提示日志不存在
fi                                                              # if 结束

echo ""                                                         # 空行
echo "========== 中位机 (To32: 上位机 ↔ 下位机) =========="        # 标题: To32 中位机
for p in 8080 8081; do                                          # 8080(指令收)、8081(遥测发 + PING)两个端口
    if ss -lun 2>/dev/null | grep -q ":$p "; then               # UDP 端口在监听(用 ss -lun 列 UDP 监听)
        echo "  UDP :$p  [监听中]"                              # 提示监听中
    else                                                        # 没监听
        echo "  UDP :$p  [未监听]"                              # 提示未监听
    fi                                                          # if 结束
done                                                            # for 循环结束
if fuser /dev/ttyCH9344USB5 >/dev/null 2>&1; then               # 用 fuser 看 F 口串口是否被占用(中位机持有)
    echo "  下位机 F 口 /dev/ttyCH9344USB5 : 已被占用 (中位机持有)"  # 在用: 说明中位机在持有
else                                                            # 空闲
    echo "  下位机 F 口 /dev/ttyCH9344USB5 : 空闲"                # 空闲: 说明中位机没起来或没拿到串口
fi                                                              # if 结束
MID_LOG="$ROOT/logs/to32_main.log"                              # 中位机日志绝对路径
if [ -s "$MID_LOG" ]; then                                      # 日志非空(存在且 size>0)
    echo "  最近状态:"                                          # 标题
    grep -E "模式 |上位机 |下位机 TX|遥测:" "$MID_LOG" 2>/dev/null | tail -n 5 | sed 's/^/    /'  # 抓关键词 + 最近 5 行 + 缩进
elif [ -f "$MID_LOG" ]; then                                    # 文件存在但为空
    echo "  (to32_main.log 为空)"                               # 提示日志是空的
else                                                            # 文件都没生成
    echo "  (无 to32_main.log)"                                 # 提示没有日志
fi                                                              # if 结束

echo ""                                                         # 空行
echo "========== 光流测速 [2026-10-04 已停用] =========="      # 标题: 光流已停用
echo "  状态   : 已停用 (不计算 / 不共享); 图像回传不受影响"     # 明确提示停用
if [ -e /dev/shm/momo_flow_bottom.bin ]; then                   # 残留共享帧存在
    echo "  共享帧 : 存在(残留) $(ls -l /dev/shm/momo_flow_bottom.bin | awk '{print $5}') bytes"  # 提示是残留
else                                                            # 无共享帧
    echo "  共享帧 : 不存在 (符合预期)"                         # 符合预期
fi                                                              # if 结束

echo ""                                                         # 空行
echo "========== 第三路相机 (CAM3 :8084) =========="              # 标题: 第三路相机(CAM3)
if ss -ltn 2>/dev/null | grep -q ":8084 "; then                 # TCP 8084 在监听
    echo "  Web    : :8084 [监听中] http://<板卡IP>:8084/stream  (快照 /snapshot, 状态 /status)"  # 打印监听中 + 端点
    # 按需模式: 是否有上位机在取流 / 相机是否已打开
    python3 - <<'PY' 2>/dev/null                                 # 内嵌 Python: 拉 /status JSON 查按需状态
import json, urllib.request
try:
    d = json.load(urllib.request.urlopen('http://127.0.0.1:8084/status', timeout=2))  # 拉一次 /status, 2s 超时
    print("  按需模式: 相机%s | 取流客户端 %d | %.1f fps | 累计开机 %d 次"
          % ("已开" if d.get("cam_open") else "空闲(已释放)", d.get("stream_clients", 0),
             d.get("fps", 0.0), d.get("open_sessions", 0)))  # 拼一行: 相机状态/客户端数/当前 fps/累计开机次数
except Exception:
    print("  按需模式: /status 读取失败")                       # 拉不到: 给出失败提示
PY
else                                                            # 端口没监听
    echo "  Web    : :8084 [未监听]"                             # 提示未监听
fi                                                              # if 结束
if [ -s "$ROOT/logs/show_cam.log" ]; then                       # show_cam.log 非空
    tail -n 2 "$ROOT/logs/show_cam.log" | sed 's/^/  /'         # 打印最近 2 行
else                                                            # 没日志
    echo "  (无 show_cam.log)"                                  # 提示没日志
fi                                                              # if 结束

echo ""                                                         # 空行
echo "========== Web 服务 =========="                            # 标题: Web 服务(直接 curl :5000/api/status)
curl -s -o /dev/null -w "  web_server :5000   -> %{http_code}\n" \
    http://127.0.0.1:5000/api/status 2>/dev/null || echo "  web_server :5000   -> 无响应"  # 静默只取 HTTP 状态码; 失败回落"无响应"

echo ""                                                         # 空行
echo "========== Nginx =========="                               # 标题: Nginx(直接 curl 80)
curl -s -o /dev/null -w "  nginx      :80     -> %{http_code}\n" \
    http://127.0.0.1/ 2>/dev/null || echo "  nginx      :80     -> 无响应"  # 同上: 静默取状态码; 失败回落"无响应"

echo ""                                                         # 空行
echo "========== 日志尾部 =========="                            # 标题: 每个日志文件最后 3 行
for f in "$ROOT"/logs/*.log; do                                 # 遍历 logs/ 下所有 .log
    [ -f "$f" ] || continue                                    # 不是普通文件就跳(包含软链/不存在等)
    echo "--- $(basename "$f") ---"                             # 打印文件名作为分隔
    tail -n 3 "$f" 2>/dev/null || echo "  (空)"                # 打印最近 3 行; 失败回落"(空)"
    echo ""                                                     # 空行隔开
done                                                            # for 循环结束