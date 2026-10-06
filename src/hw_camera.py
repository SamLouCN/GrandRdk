#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hw_camera.py — USB MJPG 相机硬件解码封装 (JPU)。

接口故意与 cv2.VideoCapture 对齐, 用于替换 cv2.VideoCapture:
  HwMjpgCamera(device, width, height, fps)
    .read()     -> (ok, bgr_frame)   # JPU 解 NV12 -> 一次 cvtColor 转 BGR
    .get(prop)  -> 支持 WIDTH/HEIGHT/FPS
    .release()
    .isOpened()

底层: ctypes 调 libmjpg_hw.so (V4L2 抓 MJPG 原始帧 + JPU 硬件解码)。
"""
import ctypes  # FFI: 用它直接调 C 导出的 mjcam_* 接口
import os  # 拼 .so 路径 + 判断文件是否存在
import sys  # 把 config/ 与 src/ 加进模块搜索路径

# ---- 路径注入: config/ + src/ ----
_HERE = os.path.dirname(os.path.abspath(__file__))     # src/
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 项目根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE):  # config/ 优先, src/ 其次
    if _p not in sys.path:  # 已存在就不重复插, 免得路径越堆越长
        sys.path.insert(0, _p)  # 插到最前: 保证 import 命中本工程而非同名库

import cv2  # 只用到颜色转换与 CAP_PROP_* 常量, 抓帧交给 .so
import numpy as np  # 把 C 缓冲包成 ndarray 视图

# ---- .so 路径: 优先 libs/, 再退回同目录 / jpu_dec/ ----
_CANDIDATES = (  # 按优先级排列, 命中第一个真实存在的就用
    os.path.join(_ROOT, 'libs', 'libmjpg_hw.so'),   # 新结构: <root>/libs/
    os.path.join(_HERE, 'libmjpg_hw.so'),           # 兼容: src/
    os.path.join(_HERE, 'jpu_dec', 'libmjpg_hw.so'),# 兼容: src/jpu_dec/
)
_SO = next((p for p in _CANDIDATES if os.path.exists(p)), _CANDIDATES[0])  # 全都没编译就退回首选, 让报错信息更明确

_CAP_W = cv2.CAP_PROP_FRAME_WIDTH if hasattr(cv2, 'CAP_PROP_FRAME_WIDTH') else 3  # 宽属性; 老版 OpenCV 回落成数字
_CAP_H = cv2.CAP_PROP_FRAME_HEIGHT if hasattr(cv2, 'CAP_PROP_FRAME_HEIGHT') else 4  # 高属性
_CAP_FPS = cv2.CAP_PROP_FPS if hasattr(cv2, 'CAP_PROP_FPS') else 5  # 帧率属性

_lib = None  # 模块级单例: .so 全进程只 dlopen 一次


def _load():
    """懒加载 libmjpg_hw.so 并声明 C 接口原型（mjcam_open/grab/close），首次调用时执行。"""
    global _lib  # 声明改的是模块级单例
    if _lib is not None:  # 已加载过: 直接复用, 避免重复 dlopen
        return _lib
    if not os.path.exists(_SO):  # .so 没编译出来
        raise RuntimeError(f'libmjpg_hw.so 不存在: {_SO} (需要先编译)')  # 明确报错并提示先编译
    lib = ctypes.CDLL(_SO)  # dlopen 载入 JPU 硬解封装库
    # ---- C 接口 (libmjpg_hw.so) 约定 ----
    #   mjcam_open(dev, w, h, fps) -> void*      打开相机句柄; 失败返回 NULL
    #   mjcam_grab(h, buf, cap, &ow, &oh) -> int 抓一帧并 JPU 硬解为 NV12 写入 buf;
    #                                          返回实际写入字节数 (= w*h*3//2), ow/oh 输出宽高
    #   mjcam_close(h)                           释放相机与解码器资源
    lib.mjcam_open.restype = ctypes.c_void_p  # 返回值按句柄指针处理, NULL 会变成 None
    lib.mjcam_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]  # 设备名 + 宽高帧率
    lib.mjcam_grab.restype = ctypes.c_int  # 返回实际写入的字节数
    lib.mjcam_grab.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                               ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]  # 句柄/缓冲/容量/宽高出参
    lib.mjcam_close.argtypes = [ctypes.c_void_p]  # 只需句柄
    _lib = lib  # 存进单例, 下次直接返回
    return _lib


class HwMjpgCamera:  # JPU 硬解相机; 接口刻意与 cv2.VideoCapture 对齐以便平替
    """JPU 硬件解码的 USB MJPG 相机 (cv2.VideoCapture 兼容接口)."""

    def __init__(self, device, width=640, height=480, fps=200):  # fps 默认 200: 走相机自曝上限, 实际由驱动协商
        """按 device/宽高/帧率打开 JPU 硬解相机，预分配 NV12 帧缓冲。"""
        self._dev = str(device)  # 设备路径, 如 /dev/video0
        self._w = int(width)  # 请求宽
        self._h = int(height)  # 请求高
        self._fps = int(fps)  # 请求帧率
        self._handle = None  # C 句柄; None 表示没打开
        self._nv12 = None  # NV12 帧缓冲 (ctypes 字符串缓冲)
        self._cap = self._w * self._h * 3 // 2  # NV12 字节数: Y 全平面 + UV 半平面
        try:  # 打开失败不外抛: 退化成 isOpened()=False, 让上层能继续
            lib = _load()  # 首次调用时才 dlopen
            h = lib.mjcam_open(self._dev.encode(), self._w, self._h, self._fps)  # 设备名要转 bytes; 失败返回 NULL
            if h:  # 句柄非空才算真正打开
                self._lib = lib  # 留一份库引用, release() 时要用
                self._handle = h
                self._nv12 = ctypes.create_string_buffer(self._cap)  # 预分配, 免得每帧反复申请内存
                self._ow = ctypes.c_int(0)  # 出参: 相机回报的实际宽
                self._oh = ctypes.c_int(0)  # 出参: 相机回报的实际高
        except Exception as exc:  # .so 缺失/设备打不开都吞掉
            print(f'[警告] HwMjpgCamera 初始化失败: {exc!r}')  # 只告警不中断, 上层按未打开处理
            self._handle = None  # 标记不可用

    def isOpened(self):
        """返回相机是否成功打开（handle 非空）。"""
        return self._handle is not None  # 句柄非空即在用

    def _grab(self):
        """抓一帧到内部缓冲; 返回 (ok, nv12_2d_view, ow, oh)."""
        if self._handle is None:  # 没打开: 直接失败
            return False, None, 0, 0
        n = self._lib.mjcam_grab(self._handle, self._nv12, self._cap,
                                 ctypes.byref(self._ow), ctypes.byref(self._oh))  # 抓帧 + JPU 硬解, 返回写入字节数
        if n != self._cap:  # 字节数不等于期望: 这帧不完整或解码失败
            return False, None, 0, 0
        w, h = self._ow.value, self._oh.value  # 用相机回报的真实宽高, 而非请求值
        view = np.frombuffer(self._nv12.raw, np.uint8).reshape(h * 3 // 2, w)  # 零拷贝视图: NV12 摊平成单平面二维
        return True, view, w, h

    def read(self):
        """阻塞抓一帧 -> JPU 解 -> BGR。返回 (ok, bgr_frame) 或 (False, None)."""
        ok, nv, _w, _h = self._grab()  # 宽高用不上: cvtColor 自己从数组形状推
        if not ok:  # 抓帧失败
            return False, None  # 语义与 cv2.VideoCapture.read() 一致
        bgr = cv2.cvtColor(nv, cv2.COLOR_YUV2BGR_NV12)  # 全流程唯一一次颜色转换
        return True, bgr

    def grab_raw_nv12(self):
        """抓一帧并保留 NV12 (不转 BGR), 供模型 NV12 直通。"""
        ok, nv, _w, _h = self._grab()
        if not ok:
            return False, None
        return True, nv.copy()  # 必须拷贝: 底层缓冲下一帧会被覆盖

    def read_both(self):
        """抓一帧同时给出 BGR 与 NV12。"""
        ok, nv, _w, _h = self._grab()
        if not ok:
            return False, None, None
        bgr = cv2.cvtColor(nv, cv2.COLOR_YUV2BGR_NV12)  # 转 BGR 给显示/算法
        return True, bgr, nv.copy()  # NV12 拷贝一份给模型直通, 两者互不干扰

    def get(self, prop_id):
        """读取属性（宽/高/帧率），未打开时返回 0.0。"""
        if self._handle is None:  # 没打开: 一律返回 0.0
            return 0.0
        if prop_id == _CAP_W:
            return float(self._ow.value or self._w)  # 优先实际宽, 为 0 时回落请求宽
        if prop_id == _CAP_H:
            return float(self._oh.value or self._h)  # 同理取实际高
        if prop_id == _CAP_FPS:
            return float(self._fps)  # 帧率只有请求值, 实测值这里不统计
        return 0.0  # 不支持的属性

    def release(self):
        """关闭相机句柄并释放 JPU 解码资源（幂等）。"""
        if self._handle is not None:  # 已打开才关, 重复调用安全
            try:
                self._lib.mjcam_close(self._handle)  # 释放相机与解码器
            except Exception:
                pass  # 关闭失败也无所谓, 进程退出会回收资源
            self._handle = None  # 置空, 后续 isOpened() 返回 False