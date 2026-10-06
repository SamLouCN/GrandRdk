#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# flow_share.py — 相机帧共享桥 (vp5.x producer -> 光流测速进程)
# ============================================================
# 目的: 让 vp5.x 保持独占相机 + YOLO 帧率的前提下, 把解出的帧
#       共享给独立的光流测速进程 (momo_pwmnet/flow_speed.py), 免第三相机。
# 机制:
#   - 单文件共享内存 /dev/shm/momo_flow_<name>.bin
#   - 布局: [header 32B][数据区 w*h*3 字节]
#   - header: magic 'MFS1', ver, seq, width, height, fmt(0=NV12 1=BGR) → 24B
#             紧跟一个独立 8B 小端 ts_us（HDR_TS）
#     ⚠ 本文件历史注释曾写 48B，是错的；以 _HEADER_SIZE 为准（=32）
#   - 并发: fcntl.flock 互斥 (写端 EX, 读端 SH), 无撕裂; 写一次 ~0.1ms,
#           对 200fps 采集的额外开销 <2% 单核
# 写端 (vp5.1 front.py/bottom.py producer):
#   sh = FlowShareWriter('front', 640, 480, fmt=0)   # 进程内建一次
#   sh.write(nv12_or_bgr)                             # 每帧 read 后调用
# 读端 (momo_pwmnet/flow_speed.py):
#   rd = FlowShareReader('front')
#   seq, img = rd.read()          # img: NV12 (h*3//2, w) 或 BGR (h,w,3)
# ============================================================

import fcntl  # 文件锁: 写端 EX / 读端 SH, 保证不会读到写了一半的帧
import mmap  # 把 /dev/shm 文件映射进进程地址空间
import os
import struct  # header 的二进制打包/解包
import time  # 微秒时间戳 + 等写端建文件的重试计时

MAGIC = b"MFS1"  # 头魔数: 读端据此确认这是本工程的帧文件
HDR = struct.Struct("<4sIIIII")        # magic, ver, seq, w, h, fmt
HDR_TS = struct.Struct("<Q")           # ts_us
_HEADER_SIZE = HDR.size + HDR_TS.size  # 32 字节 (HDR 24 + HDR_TS 8)
FMT_NV12 = 0  # 数据区是 NV12: 形状 (h*3//2, w)
FMT_BGR = 1  # 数据区是 BGR: 形状 (h, w, 3)


def _path(name):  # 命名规则集中一处, 写端读端共用
    # GRDK_SHM_DIR = 测试接缝（把共享内存重定向到临时目录）；生产默认 /dev/shm。仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
    _base = os.environ.get('GRDK_SHM_DIR') or '/dev/shm'
    return os.path.join(_base, f"momo_flow_{name}.bin")  # 放 /dev/shm: 内存盘, 不落磁盘所以无 IO 开销


class FlowShareWriter:
    """写端: 每次 write() 覆盖最新一帧 (持 EX 锁, 单缓冲)。"""

    def __init__(self, name, width, height, fmt=FMT_NV12, create=True):  # create=False: 复用已有文件, 不重置 header
        self.name = name
        self.w, self.h = int(width), int(height)  # 尺寸一旦定型就不变
        self.fmt = fmt
        data_bytes = self.w * self.h * 3          # 统一按 BGR 最大容量分配
        self._total = _HEADER_SIZE + data_bytes  # 文件总长: 头 + 数据区
        self._fh = open(_path(name), "w+b")  # 读写打开, 不存在就新建
        if create:
            self._fh.truncate(self._total)        # 首次建文件时定型
            self._fh.flush()
        self._mm = mmap.mmap(self._fh.fileno(), self._total)  # 映射整块: 之后写内存就等于写共享文件
        if create:                                 # 初始化 header
            HDR.pack_into(self._mm, 0, MAGIC, 1, 0, self.w, self.h, self.fmt)  # ver=1, seq 从 0 起
            HDR_TS.pack_into(self._mm, HDR.size, 0)  # 时间戳先置 0
            self._mm.flush()  # 刷一次, 让读端立刻能看到合法 header

    def write(self, frame):
        """frame: uint8 数组, NV12 为 (h*3//2, w), BGR 为 (h, w, 3)。返回 seq。"""
        if frame is None or frame.size == 0:  # 空帧直接跳过, 保留上一帧给读端
            return None
        n = int(frame.size)  # 实际字节数
        if n > self._total - _HEADER_SIZE:  # 超出容量: 宁可丢帧也不截断写入
            return None
        fcntl.flock(self._fh, fcntl.LOCK_EX)  # 加独占锁: 写期间读端挡在门外
        try:
            # 读当前 seq, 递增
            seq = HDR.unpack_from(self._mm, 0)[2] + 1  # 取 header 第 3 个字段 seq 并 +1
            # 数据区 (可能 < 容量, 只拷实际长度)
            self._mm[_HEADER_SIZE:_HEADER_SIZE + n] = frame.tobytes()  # tobytes 兼容非连续数组
            # header 最后更新: seq/时间戳 (数据写完后读者才见到新 seq)
            HDR.pack_into(self._mm, 0, MAGIC, 1, seq, self.w, self.h, self.fmt)  # seq 最后写: 天然的顺序保证
            HDR_TS.pack_into(self._mm, HDR.size, int(time.time() * 1e6))  # 微秒时间戳, 读端可算帧龄/判超时
            self._mm.flush()
            return seq  # 返回序号, 方便写端打日志或统计
        finally:
            fcntl.flock(self._fh, fcntl.LOCK_UN)  # 无论成败都必须解锁, 否则读端永久卡死

    def close(self):
        try:
            self._mm.close()  # 先解除映射
        finally:
            self._fh.close()  # 再关 fd; 文件本身留在 /dev/shm 供读端继续用


class FlowShareReader:
    """读端: read() 返回 (seq, frame); 无新帧时返回 (last_seq, None)。"""

    def __init__(self, name, timeout_s=5.0):  # 最多等写端 5 秒建好文件
        self.name = name
        self.path = _path(name)
        self._fh = None
        self._mm = None  # 同时作为"是否就绪"的标志
        self._total = 0
        self._w = self._h = self._fmt = 0
        deadline = time.time() + timeout_s  # 绝对截止时刻, 避免每次重算
        while self._mm is None and time.time() < deadline:  # 等写端建好文件
            try:
                self._fh = open(self.path, "r+b")  # 打不开说明写端还没创建
                # 读 header 得到尺寸
                self._mm = mmap.mmap(self._fh.fileno(), 0, access=mmap.ACCESS_READ)  # 长度 0 = 映射整个文件; 只读
                magic, ver, seq, w, h, fmt = HDR.unpack_from(self._mm, 0)
                if magic != MAGIC:  # 不是本工程的文件, 或 header 还没写完
                    raise RuntimeError(f"bad magic: {magic}")
                self._total = _HEADER_SIZE + w * h * 3  # 按 header 里的尺寸重算总长
                self._w, self._h, self._fmt = w, h, fmt  # 记住布局, read() 按它切数据
                self._last_seq = -1  # -1: 保证第一帧也被判定为新帧
                return  # 就绪就立即返回, 不用等满超时
            except (FileNotFoundError, RuntimeError):  # 文件未建 / 头还没写全
                time.sleep(0.2)  # 200ms 后重试, 别空转烧 CPU
        raise RuntimeError(f"flow_share '{name}' 不可用: {self.path}")  # 超时: 说明写端没起来

    def _mm_ro(self):  # 暴露只读映射, 需要直接摸数据的场景用
        return self._mm

    def read(self, last_seq=None):
        """返回 (seq, frame): frame 为最新完整帧; 无更新返回 (last, None)。"""
        if last_seq is None:  # 没显式传就沿用内部记录的序号
            last_seq = self._last_seq if hasattr(self, "_last_seq") else -1  # 兼容尚未初始化完成的情况
        fcntl.flock(self._fh, fcntl.LOCK_SH)  # 共享锁: 与写端互斥, 读端之间互不阻塞
        try:
            magic, ver, seq, w, h, fmt = HDR.unpack_from(self._mm, 0)
            if magic != MAGIC or seq <= last_seq:  # 头坏了, 或写端还没推新帧
                return seq if magic == MAGIC else last_seq, None  # 头坏时回退上次 seq, 帧给 None
            if fmt == FMT_NV12:
                n = w * h * 3 // 2  # NV12: Y 全平面 + UV 半平面
                buf = self._mm[_HEADER_SIZE:_HEADER_SIZE + n]
                import numpy as np  # 延迟导入: 只读 header 的场景不必依赖 numpy
                frame = np.frombuffer(buf, dtype=np.uint8).reshape(h * 3 // 2, w)  # 零拷贝视图, 直接喂模型
            else:
                import numpy as np
                buf = self._mm[_HEADER_SIZE:_HEADER_SIZE + w * h * 3]  # BGR: 三通道满格
                frame = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 3)  # 同样零拷贝
            self._last_seq = seq  # 记住序号, 下次据此判断有没有新帧
            return seq, frame
        finally:
            fcntl.flock(self._fh, fcntl.LOCK_UN)  # 解锁: 注意 frame 是映射视图, 解锁后仍指向同一块内存

    def close(self):
        if self._mm is not None:  # 可能初始化失败, 逐个判空
            self._mm.close()  # 解映射
        if self._fh is not None:
            self._fh.close()  # 关 fd
