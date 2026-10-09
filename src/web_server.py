#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web_server.py — 读共享内存, 对外提供 MJPEG / REST / WebSocket

数据源:
    /dev/shm/momo_frame_front.bin   (front 最新 JPEG)
    /dev/shm/momo_frame_bottom.bin  (bottom 最新 JPEG)
    /dev/shm/momo_det_front.json    (front 检测结果)
    /dev/shm/momo_det_bottom.json   (bottom 检测结果)
    /dev/shm/momo_stats_front.json  (front 帧率/耗时)
    /dev/shm/momo_stats_bottom.json (bottom 帧率/耗时)

对外接口:
    GET  /cam1              前视 MJPEG 流
    GET  /cam2              下视 MJPEG 流
    GET  /api/status        两路检测 + 统计 (REST)
    GET  /api/detections    两路检测结果 (REST)
    GET  /healthz           健康检查
    WS   /ws/status         两路检测 + 统计 (WebSocket, 默认 10Hz)

部署:
    - 本服务只监听 127.0.0.1:5000
    - 由 Nginx (:80) 反代 /cam1 /cam2 /api/ /ws/ 到本服务
"""
import asyncio  # WebSocket 推送的异步休眠
import os  # 路径与文件状态查询
import sys  # 路径注入与退出码
import time  # 时间戳与轮询间隔
from contextlib import asynccontextmanager  # 用于 FastAPI 生命周期钩子

# ---- 路径注入: config/ + src/ ----
_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录（src/）
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 工程根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE):  # 依次加入配置目录和源码目录
    if _p not in sys.path:  # 避免重复插入
        sys.path.insert(0, _p)  # 插到最前，优先命中本地模块

try:
    import main_config as MC  # 板端总配置（共享内存路径、端口等）
    from shm_reader import ShmFrameReader, ShmJsonReader  # 共享内存读端
except Exception as exc:
    print(f'[FATAL] import 失败: {exc!r}', file=sys.stderr)  # 打印失败原因到 stderr
    print(f'        PYTHONPATH={os.environ.get("PYTHONPATH", "")}', file=sys.stderr)  # 打印环境 PYTHONPATH 便于排查
    print(f'        sys.path={sys.path}', file=sys.stderr)  # 打印实际搜索路径
    raise  # 配置/读端缺失无法运行，直接抛出

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response  # Web 框架与 WS 类型
    from fastapi.responses import StreamingResponse, JSONResponse  # 流式与 JSON 响应
    import uvicorn  # ASGI 服务器
except ImportError as exc:
    print(f'[FATAL] 缺少 Web 依赖: {exc!r}', file=sys.stderr)  # 打印缺哪个包
    print('        安装: pip3 install fastapi uvicorn websockets', file=sys.stderr)  # 提示安装命令
    raise  # 依赖缺失直接终止


# ============================================================
# 启动检查
# ============================================================
_REQUIRED_MC_ATTRS = (  # main_config 必须具备的字段清单
    'SHM_FRAME_FRONT', 'SHM_FRAME_BOTTOM',  # 两路图像共享内存名
    'SHM_DET_FRONT', 'SHM_DET_BOTTOM',  # 两路检测结果
    'SHM_STATS_FRONT', 'SHM_STATS_BOTTOM',  # 两路统计信息
    'HDR_MAGIC', 'HDR_FMT', 'HDR_SIZE',  # 帧头魔数/格式/长度
    'WEB_HOST', 'WEB_PORT', 'WEB_TELEM_HZ',  # 监听地址/端口/遥测频率
)

def _check_config():
    missing = [a for a in _REQUIRED_MC_ATTRS if not hasattr(MC, a)]  # 找出缺失字段
    if missing:  # 缺字段就启动失败，避免运行中才报 AttributeError
        raise RuntimeError(  # 明确报错
            f'main_config 缺少字段: {missing}\n'  # 列出缺失项
            f'请在 config/main_config.py 中补上'  # 给出修复位置
        )
    print('[*] main_config 字段检查通过', flush=True)  # 检查通过提示
    print(f'[*] SHM_FRAME_FRONT  = {MC.SHM_FRAME_FRONT}',  flush=True)  # 打印前视帧路径
    print(f'[*] SHM_FRAME_BOTTOM = {MC.SHM_FRAME_BOTTOM}', flush=True)  # 打印下视帧路径
    print(f'[*] SHM_DET_FRONT    = {MC.SHM_DET_FRONT}',    flush=True)  # 打印前视检测结果路径
    print(f'[*] SHM_DET_BOTTOM   = {MC.SHM_DET_BOTTOM}',   flush=True)  # 打印下视检测结果路径
    print(f'[*] SHM_STATS_FRONT  = {MC.SHM_STATS_FRONT}',  flush=True)  # 打印前视统计路径
    print(f'[*] SHM_STATS_BOTTOM = {MC.SHM_STATS_BOTTOM}', flush=True)  # 打印下视统计路径


_check_config()  # 导入即校验配置


# ============================================================
# 共享内存读端
# ============================================================
front_frames  = ShmFrameReader(MC.SHM_FRAME_FRONT)  # 前视最新 JPEG 读端
bottom_frames = ShmFrameReader(MC.SHM_FRAME_BOTTOM)  # 下视最新 JPEG 读端
front_det     = ShmJsonReader(MC.SHM_DET_FRONT)  # 前视检测结果读端
bottom_det    = ShmJsonReader(MC.SHM_DET_BOTTOM)  # 下视检测结果读端
front_stats   = ShmJsonReader(MC.SHM_STATS_FRONT)  # 前视帧率/耗时读端
bottom_stats  = ShmJsonReader(MC.SHM_STATS_BOTTOM)  # 下视帧率/耗时读端


# ============================================================
# MJPEG 生成器
# ============================================================
MJPEG_BOUNDARY = 'frame'  # 多段分隔边界名
_POLL_INTERVAL = 0.01          # 无新帧时等待 10ms
_HEARTBEAT_SEC = 2.0           # 超过 2s 没有新帧，发空行保持连接（防 Nginx 断流）

def mjpeg_generator(reader):
    """读共享内存里的最新 JPEG，按 MJPEG 协议输出。

    - seq 不变就不发，避免重复帧占带宽
    - 长时间无新帧时发送空注释行，保持 HTTP 连接不断（Nginx proxy_read_timeout）
    """
    last_seq = -1  # 上次发出的帧序号，-1 表示还没发过
    last_send_ts = time.time()  # 上次发出数据的时间，用于心跳判断
    while True:
        try:
            seq, jpeg = reader.read_latest(after_seq=last_seq)  # 未更新时不复制整张 JPEG。
        except Exception:
            seq, jpeg = -1, None  # 读失败按无帧处理，不中断流

        now = time.time()  # 当前时间

        if jpeg is not None and seq != last_seq:  # 有新帧且序号变化才发
            last_seq = seq  # 记住已发序号，避免重复帧
            last_send_ts = now  # 刷新心跳计时
            yield (b'--' + MJPEG_BOUNDARY.encode() + b'\r\n'  # 段起始 + 边界
                   b'Content-Type: image/jpeg\r\n'  # 段头类型
                   b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'  # 段头长度
                   + jpeg + b'\r\n')  # 帧数据 + 段尾
        elif now - last_send_ts > _HEARTBEAT_SEC:  # 长时间无新帧
            # 心跳：空注释行，不会破坏 MJPEG 帧
            last_send_ts = now  # 重置心跳计时
            yield b'\r\n'  # 发空行保活，防 Nginx 超时断流

        # 无新帧：让出 CPU
        await_sec = _POLL_INTERVAL  # 轮询间隔（10ms）
        time.sleep(await_sec)  # 让出 CPU，避免忙等


# ============================================================
# 生命周期
# ============================================================
@asynccontextmanager  # 把异步生成器变成 lifespan 上下文管理器
async def lifespan(app: FastAPI):
    print(f'[*] web_server 启动: http://{MC.WEB_HOST}:{MC.WEB_PORT}/', flush=True)  # 打印监听地址
    print(f'[*] 端点: /cam1 /cam2 /api/status /api/detections /ws/status', flush=True)  # 打印可用端点
    yield  # 此处挂起，服务运行期间保持
    print('[*] web_server 停止', flush=True)  # 退出时打印


app = FastAPI(title='GrandRdk Web Server', lifespan=lifespan)  # 创建应用并绑定生命周期


# ============================================================
# 路由
# ============================================================
@app.get('/cam1')  # 注册 GET /cam1
def cam1():
    """前视 MJPEG 流。"""
    return StreamingResponse(  # 以流式响应持续输出
        mjpeg_generator(front_frames),  # 用前视共享内存读端生成流
        media_type=f'multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}',  # MJPEG 媒体类型
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',  # 禁缓存，保证实时
            'Pragma': 'no-cache',  # 兼容旧代理
            'Expires': '0',  # 立即过期
            'Connection': 'close',  # 断开即结束流
        },
    )


@app.get('/cam2')  # 注册 GET /cam2
def cam2():
    """下视 MJPEG 流。"""
    return StreamingResponse(  # 同 /cam1，换下视读端
        mjpeg_generator(bottom_frames),  # 用下视共享内存读端生成流
        media_type=f'multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}',  # MJPEG 媒体类型
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',  # 禁缓存
            'Pragma': 'no-cache',  # 兼容旧代理
            'Expires': '0',  # 立即过期
            'Connection': 'close',  # 断开即结束流
        },
    )


@app.get('/api/status')  # 注册 GET /api/status
def status():
    """两路检测 + 统计 (REST)。"""
    return {  # 直接返回字典，由 FastAPI 序列化成 JSON
        'front':  {'det': front_det.read(),  'stats': front_stats.read()},  # 前视检测 + 统计
        'bottom': {'det': bottom_det.read(), 'stats': bottom_stats.read()},  # 下视检测 + 统计
        'ts': time.time(),  # 服务端时间戳，便于上位机判断新鲜度
    }


@app.get('/api/detections')  # 注册 GET /api/detections
def detections():
    """两路检测结果 (REST)。"""
    return {'front': front_det.read(), 'bottom': bottom_det.read()}  # 只返回两路检测结果


@app.get('/healthz')  # 注册健康检查端点
def healthz():
    """健康检查：确认共享内存文件状态。"""
    def _exists(p):  # 内层工具：查单个共享内存文件
        try:
            st = os.stat(p)  # 取文件状态
            return {'exists': True, 'size': st.st_size}  # 存在则回报大小
        except FileNotFoundError:
            return {'exists': False, 'size': 0}  # 文件不存在（写端还没起来）
        except Exception as e:
            return {'exists': False, 'error': repr(e)}  # 其它异常带上原因

    return {  # 健康检查响应体
        'ok': True,  # 进程活着即 ok
        'ts': time.time(),  # 当前时间戳
        'shm': {  # 各共享内存文件的存在情况
            'front_frame':  _exists(MC.SHM_FRAME_FRONT),  # 前视帧文件
            'bottom_frame': _exists(MC.SHM_FRAME_BOTTOM),  # 下视帧文件
            'front_det':    _exists(MC.SHM_DET_FRONT),  # 前视检测结果
            'bottom_det':   _exists(MC.SHM_DET_BOTTOM),  # 下视检测结果
            'front_stats':  _exists(MC.SHM_STATS_FRONT),  # 前视统计
            'bottom_stats': _exists(MC.SHM_STATS_BOTTOM),  # 下视统计
        },
    }


@app.get('/')  # 注册根路径
def root():
    """本服务根路径：直接访问给个提示（正常走 Nginx）。"""
    return JSONResponse({  # 用 JSONResponse 明确返回 JSON
        'msg': 'web_server 运行中，请通过 Nginx 访问 http://<board-ip>/',  # 提示走 80 端口
        'endpoints': ['/cam1', '/cam2', '/api/status', '/api/detections', '/healthz', '/ws/status'],  # 端点清单
    })


@app.websocket('/ws/status')  # 注册 WebSocket 端点
async def ws_status(ws: WebSocket):
    """两路检测 + 统计 (WebSocket)。"""
    await ws.accept()  # 接受连接握手
    interval = 1.0 / max(1, MC.WEB_TELEM_HZ)  # 推送间隔，频率至少为 1
    print(f'[ws] client connected: {ws.client}', flush=True)  # 打印客户端地址
    try:
        while True:  # 持续推送直到断开
            try:
                payload = {  # 与 /api/status 同构的推送体
                    'front':  {'det': front_det.read(),  'stats': front_stats.read()},  # 前视数据
                    'bottom': {'det': bottom_det.read(), 'stats': bottom_stats.read()},  # 下视数据
                    'ts': time.time(),  # 时间戳
                }
                await ws.send_json(payload)  # 发送 JSON 帧
            except WebSocketDisconnect:  # 客户端已断开
                break  # 跳出推送循环
            except Exception as e:
                print(f'[ws] send 异常: {e!r}', flush=True)  # 打印发送异常
                break  # 异常即结束本连接
            await asyncio.sleep(interval)  # 按设定频率休眠，让出事件循环
    except WebSocketDisconnect:
        pass  # 外层断开忽略
    finally:
        print(f'[ws] client disconnected: {ws.client}', flush=True)  # 打印断开日志


# ============================================================
# main
# ============================================================
if __name__ == '__main__':
    # 打印启动信息
    print('=' * 60, flush=True)  # 分隔线
    print('[*] web_server 准备启动', flush=True)  # 启动提示
    print(f'[*] host={MC.WEB_HOST} port={MC.WEB_PORT}', flush=True)  # 打印监听地址端口
    print('=' * 60, flush=True)  # 分隔线

    # 检查端口占用（提前报错，避免和别的东西冲突）
    # 注意: 必须设 SO_REUSEADDR —— 否则上一实例被 kill 后, 上位机 MJPEG 连接残留的
    # TIME-WAIT 会让 bind 报 EADDRINUSE(其实端口上并没有人 LISTEN), 表现为
    # "stop 后马上 run 起不来、日志报 [FATAL] 端口 5000 已被占用"(2026-09-18 实测)。
    # uvicorn 自身也设了 SO_REUSEADDR, 这里与它保持一致。
    import socket  # 仅启动时用于探测端口
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 建 TCP 套接字
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 设地址复用，避开 TIME-WAIT 误报
    try:
        s.bind((MC.WEB_HOST, MC.WEB_PORT))  # 试探性绑定
        s.close()  # 能绑说明空闲，立即关闭
    except OSError as e:
        print(f'[FATAL] 端口 {MC.WEB_PORT} 已被占用: {e!r}', file=sys.stderr, flush=True)  # 报端口占用
        print(f'        排查: sudo lsof -i :{MC.WEB_PORT}', file=sys.stderr, flush=True)  # 给排查命令
        sys.exit(1)  # 非零退出，便于 systemd 感知失败

    try:
        uvicorn.run(  # 启动 ASGI 服务，阻塞在此
            app,  # FastAPI 应用实例
            host=MC.WEB_HOST,  # 监听地址（127.0.0.1）
            port=MC.WEB_PORT,  # 监听端口（5000）
            log_level='warning',  # 只打警告以上，减少刷屏
            access_log=False,  # 关闭访问日志（MJPEG 请求量太大）
        )
    except KeyboardInterrupt:
        print('[*] 收到 Ctrl-C, 退出', flush=True)  # 打印退出提示
