#!/bin/bash
# ============================================================
# nginx_setup.sh — 自动配置 Nginx 站点 (项目根目录执行)
#
# 作用:
#   1) 检查 Nginx 是否安装
#   2) 把 config/nginx/vp.conf 软链到 /etc/nginx/sites-enabled/vp
#   3) 删除默认站点 (sites-enabled/default)，避免抢 80 端口
#   4) 语法检查 + reload/启动 Nginx
#
# 用法:
#   ./nginx_setup.sh              # 安装并 reload
#   ./nginx_setup.sh --uninstall  # 卸载软链并 reload
#   ./nginx_setup.sh --dry-run    # 只打印将要做什么, 不实际执行
# ============================================================
set -u                                                  # 启用未定义变量检查; 拼错变量名立刻报错

# ---------- 参数 ----------
DRY_RUN=0                                              # 0=实跑, 1=dry-run: 只打不执行
UNINSTALL=0                                            # 0=安装模式, 1=卸载模式(移除软链并 reload)
for a in "$@"; do                                      # 遍历所有命令行参数
    case "$a" in                                       # 按参数匹配分支
        --dry-run)   DRY_RUN=1 ;;                     # --dry-run: 打开 dry-run 开关
        --uninstall) UNINSTALL=1 ;;                   # --uninstall: 走卸载分支
        -h|--help)                                    # -h / --help: 打印脚本头注释
            sed -n '2,20p' "$0"                       # 只打印前 20 行(脚本头部说明, 跳过 shebang 与空行)
            exit 0                                     # 退出 0
            ;;                                          # help 分支结束
        *) echo "[!] 未知参数: $a"; exit 2 ;;         # 其它参数: 报错并退出 2
    esac                                               # case 块结束
done                                                    # for 循环结束

# ---------- 常量 ----------
ROOT="$(cd "$(dirname "$0")" && pwd)"                  # 脚本所在绝对路径(GrandRDK 项目根)
SRC_CONF="$ROOT/config/nginx/vp.conf"                  # 站点源配置: 真正要载入 Nginx 的文件
SITE_NAME="vp"                                         # 站点短名: 软链文件 basename

NGINX_SITES_ENABLED="/etc/nginx/sites-enabled"         # Nginx 加载目录(Debian/Ubuntu 约定)
NGINX_SITES_AVAILABLE="/etc/nginx/sites-available"     # 可用站点目录(本脚本不直接写, 仅参考)
DEST_LINK="$NGINX_SITES_ENABLED/$SITE_NAME"            # 目标软链路径: /etc/nginx/sites-enabled/vp

# ---------- 工具函数 ----------
info() { echo "[*] $*"; }                              # 普通提示前缀: [*]
warn() { echo "[!] $*"; }                              # 警告前缀: [!]
err()  { echo "[X] $*" >&2; }                          # 错误前缀: [X] 写到 stderr

run() {                                                # 执行器: 在 dry-run 模式下只打印, 不真正执行
    if [ "$DRY_RUN" = "1" ]; then                      # dry-run 模式
        echo "    [dry-run] $*"                        # 缩进打印动作, 提示用户"如果实跑会执行"
    else                                                # 正常模式
        eval "$@"                                      # 用 eval 执行命令字符串(支持复合命令)
    fi                                                  # if 结束
}                                                      # run 函数结束

# ---------- 0. 检查 Nginx 是否存在 ----------
if ! command -v nginx >/dev/null 2>&1; then            # nginx 不在 PATH 里
    err "未找到 nginx, 请先安装: sudo apt install -y nginx"  # 报错: 给安装命令
    exit 1                                             # 退出 1
fi                                                      # if 结束
info "Nginx: $(nginx -v 2>&1)"                         # 打印 nginx 版本(把 stderr 也抓过来)

# ---------- 1. 检查站点配置文件 ----------
if [ ! -f "$SRC_CONF" ]; then                         # 源配置不存在
    err "站点配置不存在: $SRC_CONF"                    # 报错: 文件路径
    err "请先在 config/nginx/ 下创建 vp.conf"          # 提示下一步该做什么
    exit 1                                             # 退出 1
fi                                                      # if 结束
info "站点配置: $SRC_CONF"                             # 提示源配置已就绪

# ---------- 2. 检查目录 ----------
if [ ! -d "$NGINX_SITES_ENABLED" ]; then              # Debian/Ubuntu 风格的 sites-enabled 不存在
    # 兼容 CentOS/RHEL: 用 conf.d 代替 sites-enabled
    if [ -d /etc/nginx/conf.d ]; then                  # CentOS/RHEL 的 conf.d 存在
        NGINX_SITES_ENABLED="/etc/nginx/conf.d"        # 切换加载目录到 conf.d
        DEST_LINK="$NGINX_SITES_ENABLED/$SITE_NAME.conf"  # 目标软链改成 vp.conf(Nginx conf.d 直接吃 .conf)
        warn "未找到 sites-enabled, 使用 conf.d: $DEST_LINK"  # 提示用户已切换加载目录
    else                                                # 都没有: 没法装
        err "未找到 $NGINX_SITES_ENABLED 或 /etc/nginx/conf.d"  # 报错
        exit 1                                         # 退出 1
    fi                                                  # 兼容分支结束
fi                                                      # if 结束
info "Nginx 加载目录: $NGINX_SITES_ENABLED"            # 提示加载目录

# ---------- 3. 卸载模式 ----------
if [ "$UNINSTALL" = "1" ]; then                        # 卸载模式
    info "卸载模式: 移除软链 $DEST_LINK"                # 提示要删的软链
    run "sudo rm -f '$DEST_LINK'"                      # 删除软链(force)
    info "重载 Nginx"                                   # 提示重载
    run "sudo nginx -t && sudo systemctl reload nginx || sudo nginx -s reload"  # 先语法检查再 reload, systemd 失败回落 nginx -s reload
    info "完成"                                         # 提示卸载完成
    exit 0                                             # 退出 0
fi                                                      # if 结束

# ---------- 4. 备份并移除默认站点 ----------
DEFAULT_LINK="$NGINX_SITES_ENABLED/default"           # Debian 默认站点的软链路径
if [ -L "$DEFAULT_LINK" ] || [ -f "$DEFAULT_LINK" ]; then  # 默认站点存在(软链或普通文件)
    info "移除默认站点: $DEFAULT_LINK"                  # 提示要删
    run "sudo rm -f '$DEFAULT_LINK'"                   # 强删默认站点(避免抢 80)
fi                                                      # if 结束

# ---------- 5. 创建软链 (幂等) ----------
if [ -L "$DEST_LINK" ]; then                           # 软链已存在
    CUR_TARGET="$(readlink -f "$DEST_LINK" 2>/dev/null || true)"  # 解出软链指向的真实路径; 失败时回落空
    if [ "$CUR_TARGET" = "$SRC_CONF" ]; then           # 指向正确
        info "软链已存在且指向正确: $DEST_LINK"         # 提示无需重建
    else                                                # 指向其它配置
        warn "软链已存在但指向 $CUR_TARGET, 重新创建"  # 提示旧链接不对
        run "sudo rm -f '$DEST_LINK'"                  # 先删旧链接
        run "sudo ln -sf '$SRC_CONF' '$DEST_LINK'"     # 再创建新链接(-sf 强制覆盖已存在)
    fi                                                  # 校验分支结束
else                                                    # 软链不存在
    info "创建软链: $DEST_LINK -> $SRC_CONF"            # 提示新软链路径与指向
    run "sudo ln -sf '$SRC_CONF' '$DEST_LINK'"          # 创建新软链
fi                                                      # if 结束

# ---------- 6. 语法检查 ----------
info "Nginx 语法检查"                                   # 标题
if [ "$DRY_RUN" = "0" ]; then                          # 实跑模式才做语法检查(dry-run 没起服务)
        if ! sudo nginx -t; then                        # nginx -t 失败
            err "Nginx 语法检查失败, 请检查 $SRC_CONF"  # 报错: 检查配置文件
            exit 1                                     # 退出 1
        fi                                              # 语法检查结束
fi                                                      # if 结束

# ---------- 7. 启动 / reload ----------
info "启动 / 重载 Nginx"                                # 标题
if [ "$DRY_RUN" = "0" ]; then                          # 实跑模式才动
        if systemctl is-active --quiet nginx; then      # nginx 已运行: 走 reload
            sudo systemctl reload nginx && info "reload 完成"  # 重载; 成功打印"reload 完成"
        else                                            # 没运行: 走 start + enable
            sudo systemctl start nginx && info "启动完成"  # 启动; 成功打印"启动完成"
            sudo systemctl enable nginx >/dev/null 2>&1 || true  # 设为开机自启(失败也忽略)
        fi                                              # 启/重载分支结束
fi                                                      # if 结束

# ---------- 8. 验证 ----------
info "验证配置已加载"                                   # 标题
if [ "$DRY_RUN" = "0" ]; then                          # 实跑模式
        if sudo nginx -T 2>/dev/null | grep -q "listen 80"; then  # 用 nginx -T 看完整配置, 找 listen 80
            info "listen 80 已生效"                     # 提示端口已在监听
        fi                                              # listen 80 验证结束
        if curl -s -o /dev/null -w "%{http_code}" http://127.0.0.1/ 2>/dev/null | grep -q "200\|404"; then  # curl 拿状态码, 200 或 404 都算"有响应"
            info "HTTP 响应正常"                        # 提示响应正常
        else                                            # 拉不到
            warn "HTTP 无响应, 检查 Nginx 是否启动 / 端口是否被占"  # 给排查提示
        fi                                              # curl 验证结束
fi                                                      # if 结束

echo ""                                                 # 空行: 分隔
info "============================================"      # 收尾横幅开头
info " Nginx 站点配置完成"                              # 标题
info " 站点配置 : $SRC_CONF"                            # 源配置路径
info " 软链     : $DEST_LINK"                           # 软链路径
info " 访问     : http://<板卡IP>/"                     # 访问入口(板卡IP由用户自己替换)
info "============================================"      # 收尾横幅结尾