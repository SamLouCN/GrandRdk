#!/bin/bash
# ============================================================
# run.sh — GrandRdkRefresh 统一启动脚本
#
# 启动:
#   front.py       前视检测 (CPU 0-2)  -> 写共享内存
#   bottom.py      下视检测 (CPU 3-5)  -> 写共享内存
#   read_altimeter.py 高度计 A-E (CH348) -> UDP $ALT 帧推送 PC
#   flow_speed.py  光流测速 (读 bottom 共享帧) -> GPU LK 光流, Web :8000
#   show_cam.py    第三路相机 (按需采集: 上位机取流才开相机) -> MJPEG :8084 (上位机 CAM3)
#   web_server.py  0.0.0.0:5000        -> 读共享内存 (上位机直连 /cam1 /cam2)
#   src/to32/main.py 中位机               -> 上位机 UDP 8080/8081 + 下位机 CH348 F 口
#   src/kalman/depth_kalman 深度卡尔曼    -> 读 momo_telemetry/momo_alt, 写 momo_depth (随 run.sh/stop.sh 起停)
#   Nginx          :80                  -> 静态 + 反代 (网页端, 可 --no-nginx)
#
# 用法:
#   ./run.sh                     # 真实相机 + 全部启动
# #   ./run.sh --frames 100        # 每个检测进程跑 100 帧
#   ./run.sh --no-web            # 只跑检测, 不起 Web/Nginx
#   ./run.sh --setup-only        # 只做初始化 (依赖检查 + Nginx 配置)
#   ./run.sh --no-nginx          # 不 reload Nginx
#   ./run.sh --no-altimeter      # 不启动高度计读取
#   ./run.sh --alt-channels A,B  # 高度计只读指定口 (默认 A,B,C,D,E)
#   ./run.sh --alt-no-udp        # 高度计不推 UDP, 只打终端日志
#   ./run.sh --alt-args "--reg 0x0100 --interval 0.3"   # 高度计附加参数透传
#   ./run.sh --no-to32           # 不启动 To32 中位机
#   ./run.sh --to32-dir /path    # To32 目录 (默认 <项目根>/src/to32)
#   ./run.sh --to32-estop        # 保留 To32 的自动急停+锁存 (默认: 关闭)
#   ./run.sh --to32-args "--mode rov"   # To32 附加参数透传
#   ./run.sh --no-depthkf        # 不启动深度卡尔曼 (默认拉起; stop.sh 会一并停掉)
#   ./run.sh --no-flow           # 不启动光流测速
#   ./run.sh --flow-args "--fps 30 --port 8000"   # 光流测速附加参数透传
#   ./run.sh --no-showcam        # 不启动第三路相机推流(CAM3)
# ============================================================
set -u # 引用未定义变量立即报错，避免变量名拼错静默生效

# ---------- 项目路径 ----------
ROOT="$(cd "$(dirname "$0")" && pwd)" # 用脚本自身位置推出项目根绝对路径
cd "$ROOT" || exit 1 # 切到项目根，失败则直接退出

CONFIG_DIR="$ROOT/config" # 配置模块目录（main_config 等）
SRC_DIR="$ROOT/src" # 源码目录
WEB_DIR="$ROOT/web" # 前端静态文件目录
LOG_DIR="$ROOT/logs" # 各子进程日志落盘目录
mkdir -p "$LOG_DIR" # 确保日志目录存在

# ---------- 环境变量 ----------
export PYTHONPATH="$CONFIG_DIR:$SRC_DIR:$SRC_DIR/utils:$SRC_DIR/to32" # 让子进程能 import 各子目录模块
export OMP_NUM_THREADS=3 # 限制 OpenMP 线程数，避免抢满 CPU
export OPENBLAS_NUM_THREADS=3 # 限制 OpenBLAS 线程数
export MKL_NUM_THREADS=3 # 限制 MKL 线程数
export NUMEXPR_NUM_THREADS=3 # 限制 NumExpr 线程数

# ---------- 默认参数 ----------
FRAMES=0 # 检测帧数上限，0 表示不限
NO_WEB=0 # 1=不启动 Web 与 Nginx
NO_NGINX=0 # 1=不 reload Nginx
SETUP_ONLY=0 # 1=只做初始化就退出
NO_ALTIMETER=0 # 1=不启动高度计
NO_TO32=0 # 1=不启动中位机
NO_DEPTHKF=0 # 1=不启动深度卡尔曼（默认常驻，见 3.65 段）
TO32_ESTOP=0 # 1=保留中位机自动急停与锁存
TO32_DIR="$SRC_DIR/to32" # 中位机目录，可用 --to32-dir 覆盖
NO_FLOW=0 # 1=不启动光流测速
NO_SHOWCAM=0 # 1=不启动第三路相机推流
DETECT_ARGS=() # 透传给 front/bottom 的参数数组
ALT_ARGS=() # 高度计参数数组
ALT_ARGS_STR="" # --alt-args 原始字符串，稍后按空白切分
TO32_ARGS=() # 中位机参数数组
TO32_ARGS_STR="" # --to32-args 原始字符串
FLOW_ARGS=() # 光流参数数组
FLOW_ARGS_STR="" # --flow-args 原始字符串

# ---------- 解析参数 ----------
while [ $# -gt 0 ]; do # 还有参数就继续解析
    case "$1" in # 按参数名分支处理
        --frames)    FRAMES="$2"; DETECT_ARGS+=("$1" "$2"); shift 2 ;;
        --frames=*)  FRAMES="${1#*=}"; DETECT_ARGS+=("$1"); shift ;;
        --no-web)    NO_WEB=1; shift ;;
        --no-nginx)  NO_NGINX=1; shift ;;
        --setup-only) SETUP_ONLY=1; shift ;;
        --no-altimeter)   NO_ALTIMETER=1; shift ;;
        --alt-channels)   ALT_ARGS+=("--channels" "$2"); shift 2 ;;
        --alt-channels=*) ALT_ARGS+=("--channels" "${1#*=}"); shift ;;
        --alt-no-udp)     ALT_ARGS+=("--no-udp"); shift ;;
        --alt-args)       ALT_ARGS_STR="$2"; shift 2 ;;
        --no-to32)        NO_TO32=1; shift ;;
        --no-depthkf)     NO_DEPTHKF=1; shift ;;
        --to32-dir)       TO32_DIR="$2"; shift 2 ;;
        --to32-estop)     TO32_ESTOP=1; shift ;;
        --to32-args)      TO32_ARGS_STR="$2"; shift 2 ;;
        --no-flow)        NO_FLOW=1; shift ;;
        --flow-args)      FLOW_ARGS_STR="$2"; shift 2 ;;
        --no-showcam)     NO_SHOWCAM=1; shift ;;
        -h|--help)
            awk 'NR>1 && /^set -u/{exit} NR>1{print}' "$0" # 把脚本头部注释块当帮助打印出来
            exit 0 # 打印完帮助即退出
            ;;
        *)
            # 其他参数透传给 front/bottom
            DETECT_ARGS+=("$1") # 未识别参数原样转给检测进程
            shift # 消费掉该参数
            ;;
    esac # 参数分支处理结束
done # 参数全部解析完毕

# --alt-args 原始透传: 按空白切分成独立参数
if [ -n "$ALT_ARGS_STR" ]; then # 只有传了 --alt-args 才切分
    read -r -a _ALT_EXTRA <<< "$ALT_ARGS_STR" # 按空白切分成数组
    ALT_ARGS+=(${_ALT_EXTRA[@]+"${_ALT_EXTRA[@]}"}) # 追加进高度计参数，空数组时安全展开
fi # 高度计参数处理结束

# --to32-args 原始透传: 按空白切分成独立参数
if [ -n "$TO32_ARGS_STR" ]; then # 只有传了 --to32-args 才切分
    read -r -a _TO32_EXTRA <<< "$TO32_ARGS_STR" # 按空白切分成数组
    TO32_ARGS+=(${_TO32_EXTRA[@]+"${_TO32_EXTRA[@]}"}) # 追加进中位机参数
fi # 中位机参数处理结束

# --flow-args 原始透传: 按空白切分成独立参数
if [ -n "$FLOW_ARGS_STR" ]; then # 只有传了 --flow-args 才切分
    read -r -a _FLOW_EXTRA <<< "$FLOW_ARGS_STR" # 按空白切分成数组
    FLOW_ARGS+=(${_FLOW_EXTRA[@]+"${_FLOW_EXTRA[@]}"}) # 追加进光流参数
fi # 光流参数处理结束

# ---------- 工具函数 ----------
info() { echo "[run.sh] $*"; } # 普通信息输出
warn() { echo "[run.sh] [!] $*"; } # 警告输出，不中断
err()  { echo "[run.sh] [X] $*" >&2; } # 错误输出到 stderr

# ---------- 读取 main_config 开关 ----------
read_cfg() { # 读取 main_config.py 里的某个开关值（只把"最后一行=目标值"交给调用者）
    # [2026-10-04 修复] 原来直接 print 会让 import main_config 触发的 camera_ports 模块级
    #   print('[camera] ...') 污染 stdout（2 行），导致 `[ "$EN_NEW" = "True" ]` 永远判假、
    #   web_server / flow_speed 被静默跳过。这里把导入期的 stdout 暂时改道到 stderr，
    #   导入完成后再用真正的 stdout 打印目标值；外层再 tail -n 1 兜底。
    python3 -c "
import sys, os
_real = sys.stdout                              # 保留真正的 stdout（fd 1）
sys.stdout = sys.stderr                         # 导入期所有 print 改走 stderr，不污染取值
sys.path.insert(0, '$CONFIG_DIR')               # 挂上 config 目录
import main_config as m                         # 导入配置（此刻的 print 全部走 stderr）
sys.stdout = _real                              # 恢复真正的 stdout
print(getattr(m, '$1', ''))                     # 只输出目标值一行
" 2>/dev/null | tail -n 1 # stderr 丢弃（污染信息不需要），tail -1 双保险
}

# ============================================================
# 0. 依赖检查
# ============================================================
info "项目根: $ROOT" # 打印项目根路径

# 检查关键文件
for f in "$CONFIG_DIR/main_config.py" "$SRC_DIR/front.py" "$SRC_DIR/bottom.py"; do # 逐个检查启动必需文件
    if [ ! -f "$f" ]; then # 文件不存在
        err "缺少文件: $f" # 报错提示
        exit 1 # 缺关键文件直接中止
    fi # 检查结束
done # 关键文件全部检查完
info "关键文件检查通过" # 提示检查通过

# Python 依赖
if [ "$NO_WEB" = "0" ]; then # 只有要起 Web 才需要这些依赖
    if ! python3 -c "import fastapi, uvicorn" 2>/dev/null; then # 探测 fastapi/uvicorn 是否已装
        warn "缺少 fastapi/uvicorn, 尝试安装..." # 提示将自动安装
        pip3 install -q fastapi uvicorn websockets 2>/dev/null || { # 静默安装，失败则走右侧错误处理
            err "安装失败, 请手动: pip3 install fastapi uvicorn websockets" # 给出手动安装命令
            exit 1 # 依赖装不上就中止
        }
    fi # 依赖探测分支结束
    info "Web 依赖检查通过" # 提示依赖就绪
fi # Web 依赖检查段结束

# ============================================================
# 1. 配置 Nginx (首次)
# ============================================================
if [ "$NO_WEB" = "0" ] && [ "$NO_NGINX" = "0" ]; then # 起 Web 且不禁 Nginx 才配置
    if [ ! -L /etc/nginx/sites-enabled/vp ]; then # 软链不存在说明是首次运行
        info "首次运行, 配置 Nginx" # 提示开始配置
        if [ -x "$ROOT/nginx_setup.sh" ]; then # 配置脚本存在且可执行
            "$ROOT/nginx_setup.sh" || warn "nginx_setup.sh 失败, 继续" # 配置失败只告警，不阻断启动
        else # 没有配置脚本
            warn "未找到 nginx_setup.sh, 跳过 Nginx 配置" # 提示跳过
        fi # 配置脚本分支结束
    fi # 首次配置判断结束
fi # Nginx 配置段结束

if [ "$SETUP_ONLY" = "1" ]; then # --setup-only 只需做完前面的检查与配置
    info "--setup-only 完成" # 提示初始化完成
    exit 0 # 到此退出，不启动任何子进程
fi # setup-only 分支结束

# ============================================================
# 2. 清理旧进程
# ============================================================
info "清理旧进程..." # 提示开始清理
pkill -f "src/front.py"        2>/dev/null # 杀掉上一轮前视进程，没匹配到也不报错
pkill -f "src/bottom.py"       2>/dev/null # 杀掉上一轮下视进程
pkill -f "src/web_server.py"   2>/dev/null # 杀掉上一轮 Web 服务
pkill -f "src/read_altimeter.py" 2>/dev/null # 杀掉上一轮高度计进程
pkill -f "src/to32/main.py" 2>/dev/null # 杀掉上一轮中位机进程
pkill -f "src/flow_speed.py" 2>/dev/null # 杀掉上一轮光流进程
pkill -f "src/show_cam.py" 2>/dev/null # 杀掉上一轮第三路相机进程
pkill -f "kalman/depth_kalman/main.py" 2>/dev/null # 杀掉上一轮深度卡尔曼（2026-10-08 起随 run.sh 生命周期）
rm -f "$SRC_DIR/kalman/depth_kalman/logs/depth_kalman.pid" 2>/dev/null # 顺手清掉陈旧 pidfile
sleep 0.5 # 给被杀进程一点退出时间

# 清理旧的共享内存（避免读到脏数据）
rm -f /dev/shm/momo_*.bin /dev/shm/momo_*.json 2>/dev/null # 删除旧共享内存文件，防止读到上一轮脏帧

# ============================================================
# 3. 启动检测进程
# ============================================================
info "启动 front.py (CPU 0-2)" # 提示启动前视
taskset -c 0-2 python3 "$SRC_DIR/front.py" "${DETECT_ARGS[@]}" \
    > "$LOG_DIR/front.log" 2>&1 & # 前视日志重定向并放入后台
PID_FRONT=$! # 记录前视进程 PID

info "启动 bottom.py (CPU 3-5)" # 提示启动下视
taskset -c 3-5 python3 "$SRC_DIR/bottom.py" "${DETECT_ARGS[@]}" \
    > "$LOG_DIR/bottom.log" 2>&1 & # 下视日志重定向并放入后台
PID_BOTTOM=$! # 记录下视进程 PID

sleep 2 # 等检测进程初始化并建好共享内存

# 检测进程是否活着
if ! kill -0 $PID_FRONT 2>/dev/null; then # kill -0 只探活，进程没了说明启动失败
    err "front.py 启动失败, 查看 $LOG_DIR/front.log" # 报错并指路日志
    tail -n 20 "$LOG_DIR/front.log" # 打印日志尾部便于定位
    exit 1 # 前视是主链路，失败直接中止
fi # 前视探活结束
if ! kill -0 $PID_BOTTOM 2>/dev/null; then # 下视探活
    err "bottom.py 启动失败, 查看 $LOG_DIR/bottom.log" # 报错并指路日志
    tail -n 20 "$LOG_DIR/bottom.log" # 打印日志尾部
    exit 1 # 下视同为主链路，失败中止
fi # 下视探活结束
info "检测进程已启动: front=$PID_FRONT bottom=$PID_BOTTOM" # 打印两个检测进程 PID

# ============================================================
# 3.5 启动高度计读取 (read_altimeter.py)
#     - 独立进程 + 独立日志; 某口未接设备只降频探测, 不影响主链路
#     - 启动失败仅告警, 不终止 run.sh (高度计非必需链路)
# ============================================================
PID_ALT="" # 高度计 PID，未启动时为空
if [ "$NO_ALTIMETER" = "1" ]; then # 显式关闭高度计
    info "--no-altimeter, 跳过高度计读取" # 提示跳过
elif [ ! -f "$SRC_DIR/read_altimeter.py" ]; then # 脚本不存在则跳过
    warn "未找到 $SRC_DIR/read_altimeter.py, 跳过高度计读取" # 告警跳过
else # 正常启动高度计
    info "启动 read_altimeter.py (高度计 A-E, Modbus-RTU)" # 提示启动高度计
    nice -n 5 python3 "$SRC_DIR/read_altimeter.py" \
        ${ALT_ARGS[@]+"${ALT_ARGS[@]}"} \
        > "$LOG_DIR/altimeter.log" 2>&1 & # 高度计日志重定向并后台运行
    PID_ALT=$! # 记录高度计进程 PID
    sleep 1 # 探活前稍等，给它起串口的时间
    if ! kill -0 $PID_ALT 2>/dev/null; then # 探活失败
        warn "read_altimeter.py 未存活 (常见原因: 对应串口全未接高度计), 不影响主链路" # 只告警，不终止
        tail -n 20 "$LOG_DIR/altimeter.log" # 打印日志尾部
        PID_ALT="" # 置空，后续清理时跳过
    else # 高度计已存活
        info "高度计读取已启动: pid=$PID_ALT" # 打印高度计 PID
    fi # 高度计探活结束
fi # 高度计启动段结束

# ============================================================
# 3.6 启动 To32 中位机 (上位机 UDP 8080/8081 <-> 下位机 CH348 F 口)
#     - --no-video: 关掉 To32 自带图像回传, 让出摄像头设备与 :5000
#     - --video-port 0: 避免 To32 端口预检误报 "VIDEO_HTTP_PORT(5000) 已被占用"
#     - --estop-timeout 0 --estop-no-latch: 暂时关闭下行静默自动急停与锁存(联调期)
#       要恢复自动急停: ./run.sh --to32-estop
#     - 启动失败仅告警, 不终止 run.sh
# ============================================================
PID_TO32="" # 中位机 PID，未启动时为空
if [ "$NO_TO32" = "1" ]; then # 显式关闭中位机
    info "--no-to32, 跳过 To32 中位机" # 提示跳过
elif [ ! -f "$TO32_DIR/main.py" ]; then # 中位机入口不存在
    warn "未找到 $TO32_DIR/main.py, 跳过 To32 中位机 (可用 --to32-dir 指定)" # 告警并提示可指定目录
else # 正常启动中位机
    # 用绝对脚本路径启动: 让 cmdline 含 src/to32/main.py,
    # status.sh / stop.sh 的进程匹配("src/to32/main.py")才能命中
    TO32_CMD=(python3 "$TO32_DIR/main.py" --no-video --video-port 0) # 关图像回传并把视频端口置 0，避开端口预检
    [ "$TO32_ESTOP" = "1" ] || TO32_CMD+=(--estop-timeout 0 --estop-no-latch) # 默认关掉自动急停与锁存（联调期）
    if [ "$TO32_ESTOP" = "1" ]; then # 保留了自动急停
        ESTOP_DESC="保留(默认 1s + 锁存)" # 提示文案：急停保留
    else # 关闭自动急停
        ESTOP_DESC="关" # 提示文案：急停关闭
    fi # 急停描述分支结束
    info "启动 To32 中位机: $TO32_DIR (图像回传=关, 自动急停=$ESTOP_DESC)" # 打印中位机目录与急停状态
    ( cd "$TO32_DIR" && exec nice -n 5 "${TO32_CMD[@]}" \
        ${TO32_ARGS[@]+"${TO32_ARGS[@]}"} ) \
        > "$LOG_DIR/to32_main.log" 2>&1 & # 子 shell 切目录后 exec 启动，日志重定向并后台
    PID_TO32=$! # 记录中位机进程 PID
    sleep 2 # 给中位机起 UDP 端口的时间
    if ! kill -0 $PID_TO32 2>/dev/null; then # 探活失败
        warn "To32 中位机未存活, 查看 $LOG_DIR/to32_main.log" # 只告警，不终止主流程
        tail -n 20 "$LOG_DIR/to32_main.log" # 打印日志尾部
        PID_TO32="" # 置空，后续清理时跳过
    else # 中位机已存活
        info "To32 中位机已启动: pid=$PID_TO32" # 打印中位机 PID
    fi # 中位机探活结束
fi # 中位机启动段结束

# ============================================================
# 3.65 启动深度卡尔曼 (src/kalman/depth_kalman) —— [2026-10-08 起由本脚本托管]
#     - 生命周期与 front/bottom/to32 同款: 清理段 pkill 上一轮实例, 本段拉起新的,
#       stop.sh / Ctrl-C(cleanup) / 主链路退出都会停它。ROV 模式下 $TEL[41/42]
#       的融合深度/净空恒有值 (tel_builder 持续读 momo_depth.json)。
#     - 幂等双保险: 本段 pidfile+kill -0 判活 (兜住 kill -9 残留等漏网场景,
#       活着就复用不起第二个); kalman_launcher.py 进 AUV 模式时同样判活复用 ——
#       双进程同时写 momo_depth.json 会打架, 任何路径都绝不双开。
#     - 启动失败仅告警, 不终止 run.sh (AUV 任务会走降级路径)。
# ============================================================
DEPTHKF_DIR="$SRC_DIR/kalman/depth_kalman" # 深度卡尔曼工程根（与板端 /userdata/GrandRDK 布局一致）
DEPTHKF_PIDFILE="$DEPTHKF_DIR/logs/depth_kalman.pid" # run.sh --daemon 写的 pid 文件
DEPTHKF_PID="" # 汇总展示用，未启动/复用失败时为空
_depthkf_alive() { # pidfile 判活: 文件在且里面的 pid 活着才返回 0
    [ -f "$DEPTHKF_PIDFILE" ] || return 1 # 没有 pidfile 视为没在跑
    kill -0 "$(cat "$DEPTHKF_PIDFILE" 2>/dev/null)" 2>/dev/null # 信号 0 只探活
}
if [ "$NO_DEPTHKF" = "1" ]; then # 显式关闭
    info "--no-depthkf, 跳过深度卡尔曼" # 提示跳过
elif [ ! -f "$DEPTHKF_DIR/run.sh" ]; then # 工程不存在
    warn "未找到 $DEPTHKF_DIR/run.sh, 跳过深度卡尔曼" # 告警跳过
elif _depthkf_alive; then # 漏网实例还在跑（kill -9 残留等）: 复用, 绝不起第二个
    DEPTHKF_PID="$(cat "$DEPTHKF_PIDFILE" 2>/dev/null)" # 记下 pid 供汇总展示
    info "深度卡尔曼已在运行 (pid=$DEPTHKF_PID), 复用不重启" # 幂等核心提示
else # 正常拉起
    info "启动 depth_kalman (深度卡尔曼, 常驻后台)" # 提示启动
    ( cd "$DEPTHKF_DIR" && exec ./run.sh --daemon ) 2>&1 \
        | sed 's/^/[depthkf] /' # 借它的 --daemon 落 pidfile; 输出加前缀防与总 run.sh 的 [run.sh] 混淆
    sleep 1 # 探活前稍等
    if _depthkf_alive; then # 探活成功
        DEPTHKF_PID="$(cat "$DEPTHKF_PIDFILE" 2>/dev/null)" # 记 pid
        info "深度卡尔曼已启动: pid=$DEPTHKF_PID (日志 $DEPTHKF_DIR/logs/depth.log)" # 打印 pid 与日志位置
    else # 探活失败
        warn "深度卡尔曼未存活, 查看 $DEPTHKF_DIR/logs/depth.log (不影响主链路)" # 只告警不终止
    fi # 深度卡尔曼探活结束
fi # 深度卡尔曼启动段结束

# ============================================================
# 3.7 光流测速 (flow_speed.py) —— [2026-10-04 已停用]
#     - 停用原因: 用户要求先关掉光流相关功能（计算 + 共享），保留图像回传。
#     - 现状: 本段整体短路，不再拉起 flow_speed.py；--no-flow / FLOW_ENABLE 仍兼容
#       （传了也不会起）。src/flow_speed.py 文件保留，将来要去掉短路即可恢复。
#     - ⚠ 图像回传不经过这里: 上位机 /cam2 由 web_server.py 读 SHM_FRAME_BOTTOM(JPEG) 提供。
# ============================================================
PID_FLOW="" # 光流 PID：已停用，恒为空
FLOW_DISABLED=1 # 光流停用硬开关（1=停用）；改回 0 可恢复原启动逻辑
if [ "$FLOW_DISABLED" = "1" ]; then # 光流已停用
    info "光流测速已停用 (2026-10-04), 跳过 flow_speed.py" # 明确提示已停用
elif [ "$NO_FLOW" = "1" ]; then # 显式关闭光流
    info "--no-flow, 跳过光流测速" # 提示跳过
elif [ "$FLOW_ON" != "True" ]; then # 配置未开启
    info "FLOW_ENABLE=$FLOW_ON, 跳过光流测速" # 打印配置值并跳过
else # 正常启动光流
    FLOW_SHM=$(read_cfg SHM_FLOW_BOTTOM) # 光流输入共享帧路径
    FLOW_WPORT=$(read_cfg FLOW_WEB_PORT) # 光流 Web 服务端口
    # 等 bottom 建好共享帧(最多 6s), 否则 FlowShareReader 会因文件不存在直接退出
    waited=0 # 已等待秒数
    while [ ! -e "$FLOW_SHM" ] && [ "$waited" -lt 6 ]; do # 共享帧未出现且未超时就继续等
        sleep 1 # 每次等 1 秒
        waited=$((waited + 1)) # 等待计数加一
    done # 等待循环结束
    if [ ! -e "$FLOW_SHM" ]; then # 超时仍未等到共享帧
        warn "未等到共享帧 $FLOW_SHM (bottom 是否在写?), 跳过光流测速" # 告警并跳过
    else # 共享帧已就绪
        info "启动 flow_speed.py (光流测速, Web :$FLOW_WPORT)" # 提示启动光流
        OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
            nice -n 19 python3 "$SRC_DIR/flow_speed.py" \
            ${FLOW_ARGS[@]+"${FLOW_ARGS[@]}"} \
            > "$LOG_DIR/flow_speed.log" 2>&1 & # 日志重定向并后台运行
        PID_FLOW=$! # 记录光流进程 PID
        sleep 2 # 探活前稍等
        if ! kill -0 $PID_FLOW 2>/dev/null; then # 探活失败
            warn "flow_speed.py 未存活, 查看 $LOG_DIR/flow_speed.log" # 只告警，不终止
            tail -n 20 "$LOG_DIR/flow_speed.log" # 打印日志尾部
            PID_FLOW="" # 置空，后续清理时跳过
        else # 光流已存活
            info "光流测速已启动: pid=$PID_FLOW" # 打印光流 PID
        fi # 光流探活结束
    fi # 共享帧等待结果分支结束
fi # 光流启动段结束 [2026-10-04: 上分支已短路, 仅保留供恢复]

# ============================================================
# 3.8 启动 show_cam.py (第三路相机 -> MJPEG :8084, 上位机 CAM3)
#     - 独立进程自己开相机: 默认 cam3（USB 物理口 1-1 内窥镜）,
#       与 front.py(cam1, 物理口 3-2) / bottom.py(cam2, 物理口 1-2) 互不冲突
#     - 按需模式: HTTP(:8084)常驻, 上位机连上 /stream 才打开相机, 最后一个客户端断开后 5s 自动释放
#     - 启动失败仅告警, 不终止 run.sh (相机被占或未接时它自己会退出)
# ============================================================
PID_SHOWCAM="" # 第三路相机 PID，未启动时为空
if [ "$NO_SHOWCAM" = "1" ]; then # 显式关闭第三路相机
    info "--no-showcam, 跳过第三路相机推流" # 提示跳过
elif [ ! -f "$SRC_DIR/show_cam.py" ]; then # 脚本不存在
    warn "未找到 $SRC_DIR/show_cam.py, 跳过第三路相机推流" # 告警跳过
else # 正常启动第三路相机
    info "启动 show_cam.py (第三路相机 -> MJPEG :8084)" # 提示启动
    nice -n 5 python3 "$SRC_DIR/show_cam.py" \
        > "$LOG_DIR/show_cam.log" 2>&1 & # 日志重定向并后台运行
    PID_SHOWCAM=$! # 记录第三路相机进程 PID
    sleep 2 # 探活前稍等
    if ! kill -0 $PID_SHOWCAM 2>/dev/null; then # 探活失败
        warn "show_cam.py 未存活(相机被占/未接?), 查看 $LOG_DIR/show_cam.log" # 只告警，不终止
        tail -n 20 "$LOG_DIR/show_cam.log" # 打印日志尾部
        PID_SHOWCAM="" # 置空，后续清理时跳过
    else # 第三路相机已存活
        info "第三路相机推流已启动: pid=$PID_SHOWCAM" # 打印 CAM3 PID
    fi # CAM3 探活结束
fi # CAM3 启动段结束

# ============================================================
# 4. 启动 Web 服务
# ============================================================
PID_WEB="" # Web 服务 PID，未启动时为空
PID_LEGACY="" # 旧版 Web 进程 PID（当前未启用，仅占位）

if [ "$NO_WEB" = "0" ]; then # 需要启动 Web
    EN_NEW=$(read_cfg ENABLE_WEB_NEW) # 从配置读取新版 Web 开关

    if [ "$EN_NEW" = "True" ]; then # 配置启用新版 Web
        info "启动 web_server.py (FastAPI :5000)" # 提示启动 Web 服务
        nice -n 10 python3 "$SRC_DIR/web_server.py" \
            > "$LOG_DIR/web_server.log" 2>&1 & # 日志重定向并后台运行
        PID_WEB=$! # 记录 Web 进程 PID
        sleep 1 # 探活前稍等
        if ! kill -0 $PID_WEB 2>/dev/null; then # 探活失败
            err "web_server.py 启动失败, 查看 $LOG_DIR/web_server.log" # 报错并指路日志
            tail -n 20 "$LOG_DIR/web_server.log" # 打印日志尾部
        else # Web 已存活
            info "web_server 已启动: pid=$PID_WEB" # 打印 Web PID
        fi # Web 探活结束
    else # 配置关闭新版 Web
        info "ENABLE_WEB_NEW=False, 跳过 web_server" # 提示跳过
    fi # 新版 Web 开关分支结束

    # ---------- Nginx reload ----------
    if [ "$NO_NGINX" = "0" ]; then # 未禁用 Nginx 才 reload
        info "reload Nginx" # 提示开始 reload
        sudo nginx -t 2>/dev/null && \
            (sudo systemctl reload nginx 2>/dev/null || sudo nginx -s reload 2>/dev/null) \
            || warn "Nginx reload 失败" # 配置检查或 reload 任一失败只告警
    fi # Nginx reload 结束
fi # Web 段结束

# ============================================================
# 5. 打印访问信息
# ============================================================
BOARD_IP=$(hostname -I 2>/dev/null | awk '{print $1}') # 取第一个网卡 IP，用于拼访问地址
[ -z "$BOARD_IP" ] && BOARD_IP="<板卡IP>" # 取不到 IP 时显示占位提示

echo "" # 输出空行做视觉分隔
info "============================================" # 打印分隔线
info " 启动完成" # 提示启动完成
info " 前视 pid : $PID_FRONT" # 打印前视 PID
info " 下视 pid : $PID_BOTTOM" # 打印下视 PID
[ -n "$PID_ALT" ]    && info " Alt  pid : $PID_ALT" # 高度计启动成功才打印该行
[ -n "$PID_TO32" ]   && info " Mid  pid : $PID_TO32" # 中位机启动成功才打印该行
[ -n "$DEPTHKF_PID" ] && info " DKF  pid : $DEPTHKF_PID" # 深度卡尔曼启动成功才打印该行
[ -n "$PID_WEB" ]    && info " Web  pid : $PID_WEB" # Web 启动成功才打印该行
info " 日志目录 : $LOG_DIR" # 打印日志目录
[ -n "$PID_FLOW" ]   && info " 光流画面 : http://$BOARD_IP:$(read_cfg FLOW_WEB_PORT)/" # 打印光流页面地址
[ -n "$PID_SHOWCAM" ] && info " CAM3 画面: http://$BOARD_IP:8084/stream  (上位机第三路)" # 打印 CAM3 流地址
if [ "$NO_WEB" = "0" ]; then # 起了 Web 才打印网页相关地址
    info " MJPEG    : http://$BOARD_IP:5000/cam1  http://$BOARD_IP:5000/cam2  (上位机直连)" # 打印两路 MJPEG 地址
    info " 网页端   : http://$BOARD_IP/" # 打印网页入口
    info " API      : http://$BOARD_IP/api/status" # 打印状态接口
    info " WS       : ws://$BOARD_IP/ws/status" # 打印 WebSocket 地址
fi # 访问信息打印结束
info "============================================" # 打印分隔线
echo "" # 输出空行

# ============================================================
# 6. 等待 / 退出
# ============================================================
cleanup() { # 收到 INT/TERM 时的收尾处理
    echo "" # 输出空行
    info "收到停止信号, 清理进程..." # 提示开始清理
    # depth_kalman 2026-10-08 起纳入停止范围（$DEPTHKF_PID 可能来自复用的漏网实例, 一并停）
    kill $PID_FRONT $PID_BOTTOM $PID_ALT $PID_TO32 $PID_FLOW $PID_SHOWCAM $PID_WEB $DEPTHKF_PID 2>/dev/null # 逐个终止子进程，忽略空 PID 与已退出
    wait 2>/dev/null # 等待所有后台任务收尸
    info "全部已停止" # 提示清理完成
    exit 0 # 正常退出
}
trap cleanup INT TERM # 捕获 Ctrl-C 与 TERM 信号

# 等待任一检测进程退出（有限帧模式会自然退出）
wait $PID_FRONT $PID_BOTTOM 2>/dev/null # 阻塞等待任一检测进程退出，有限帧模式会自然结束
info "检测进程已退出" # 提示主链路已结束
kill $PID_ALT $PID_TO32 $PID_FLOW $PID_SHOWCAM $PID_WEB $DEPTHKF_PID 2>/dev/null # 主链路退出后停掉其余子进程，防止留下孤儿进程（含深度卡尔曼）
wait 2>/dev/null # 等待剩余子进程全部退出
info "run.sh 结束" # 打印结束提示