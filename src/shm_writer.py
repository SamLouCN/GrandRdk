#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shm_writer.py — 写共享内存(帧 + JSON), 检测进程用"""
import json  # 导入 json 标准库: 把 Python 对象序列化成 JSON 文本
import mmap  # 导入 mmap 标准库: 文件内存映射, 写 /dev/shm 零拷贝
import os  # 导入 os 标准库: 路径拼接、文件存在性判断、文件描述符操作
import struct  # 导入 struct 标准库: 把帧头的几个整数打包成定长二进制
import sys  # 导入 sys 标准库: 修改模块搜索路径, 让本文件能 import main_config
import threading
import time  # 导入 time 标准库: 取当前时间戳写进帧头

_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录 (src/)
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 项目根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE):  # 依次处理两个待加入的搜索路径: config/ 与 src/
    if _p not in sys.path:  # 若该路径还不在搜索列表里 (避免重复插入)
        sys.path.insert(0, _p)  # 插到列表最前面: 优先于系统路径, 保证 import main_config 命中本项目

import main_config as MC  # 导入本项目配置: 提供 HDR_FMT/HDR_MAGIC/HDR_SIZE/MAX_JPEG_BYTES/MAX_JSON_BYTES


class ShmFrameWriter:  # 类: 把最新一帧 JPEG 写进共享内存 (定长容量, 覆盖式, 只保留最新)
    """写最新 JPEG 到共享内存。定长容量, 覆盖式。"""

    def __init__(self, path, width=640, height=480, max_jpeg=None):  # 构造: path=共享内存文件路径, width/height=图像尺寸, max_jpeg=JPEG 字节上限
        self.path = path  # 存下路径, close() 与外部排查时用
        self.width = int(width)  # 图像宽 (转 int, 防止传入字符串/None 导致后续 struct.pack 报错)
        self.height = int(height)  # 图像高 (同上转 int)
        self.max_jpeg = int(max_jpeg if max_jpeg is not None else MC.MAX_JPEG_BYTES)  # JPEG 容量上限: 用调用方传入值, 未传则取配置默认
        self.total = MC.HDR_SIZE + self.max_jpeg  # 共享内存总字节 = 定长帧头 + JPEG 容量
        if not os.path.exists(path):  # 首次运行该文件还不存在时:
            with open(path, 'wb') as f:  # 以二进制写模式创建文件
                f.truncate(self.total)  # 把文件撑到 total 字节 (mmap 要求文件不小于映射长度)
        self._fd = os.open(path, os.O_RDWR)  # 以读写方式打开文件, 拿到文件描述符供 mmap 使用
        self._mm = mmap.mmap(self._fd, self.total)  # 把整个文件映射进内存: 之后写内存即写共享内存
        self._seq = 0  # 帧序号计数器, 每写一帧 +1 (消费端靠它判断"是不是新帧")
        self._lock = threading.Lock()

    def write(self, jpeg):  # 写入一帧 JPEG 字节流
        """写入一帧 JPEG。"""
        if not jpeg or len(jpeg) > self.max_jpeg:  # 空数据或超过容量上限时:
            return  # 直接返回不写 (宁可丢这一帧, 也不能越界写坏映射区)
        with self._lock:
            self._seq += 1
            fields = (MC.HDR_MAGIC, self._seq, 0, self.width * self.height,
                      int(time.time() * 1e6))
            # length=0 标记正在覆盖；先写完整 JPEG，再发布有效帧头。
            self._mm[:MC.HDR_SIZE] = struct.pack(MC.HDR_FMT, *fields)
            self._mm[MC.HDR_SIZE:MC.HDR_SIZE + len(jpeg)] = jpeg
            self._mm[:MC.HDR_SIZE] = struct.pack(MC.HDR_FMT, fields[0], fields[1], len(jpeg),
                                                fields[3], fields[4])
            # MAP_SHARED 的修改跨进程直接可见；实时帧不需要逐帧同步落盘。

    def close(self):  # 释放资源 (进程退出时调用, 可重复调用)
        try:  # 兜住"已经关过"导致的异常
            self._mm.close()  # 关闭内存映射
            os.close(self._fd)  # 关闭文件描述符
        except Exception:  # 若已关闭 / 句柄无效
            pass  # 静默忽略: 关闭操作要求幂等


class ShmJsonWriter:  # 类: 把一个 JSON 对象写进共享内存 (定长容量, 覆盖式)
    """写 JSON 到共享内存(定长, 覆盖式)。"""

    def __init__(self, path, max_bytes=None):  # 构造: path=共享内存文件路径, max_bytes=这块区域的字节容量
        self.path = path  # 存下路径备用
        self.max_bytes = int(max_bytes if max_bytes is not None else MC.MAX_JSON_BYTES)  # 容量: 用调用方传入值, 未传取配置默认
        if not os.path.exists(path):  # 首次运行文件不存在时:
            with open(path, 'wb') as f:  # 二进制写模式创建
                f.truncate(self.max_bytes)  # 撑到 max_bytes 字节 (满足 mmap 长度要求)
        self._fd = os.open(path, os.O_RDWR)  # 读写方式打开, 拿文件描述符
        self._mm = mmap.mmap(self._fd, self.max_bytes)  # 映射进内存

    def write(self, obj):  # 写入一个可 JSON 序列化的对象 (dict/list)
        """写入 JSON 对象（覆盖式）。"""
        data = json.dumps(obj, ensure_ascii=False, default=str).encode('utf-8')  # 序列化成 UTF-8 字节: 保留中文不转义(ensure_ascii=False), 无法序列化的值转成字符串(default=str)
        if len(data) > self.max_bytes:  # 若序列化结果超出容量:
            data = data[:self.max_bytes]  # 截断到容量上限 (宁缺毋滥, 保证不越界)
        pad = self.max_bytes - len(data)  # 计算还需补齐多少字节
        self._mm.seek(0)  # 写指针回到映射区开头
        self._mm.write(data)  # 写入 JSON 数据
        if pad > 0:  # 若还有剩余空间:
            self._mm.write(b' ' * pad)  # 用空格填满 (保持定长, 消费端按固定长度读不会读到上一帧残留)
        # MAP_SHARED 已使更新跨进程可见，不在实时路径同步落盘。

    def close(self):  # 释放资源 (幂等)
        try:  # 兜住重复关闭
            self._mm.close()  # 关内存映射
            os.close(self._fd)  # 关文件描述符
        except Exception:  # 已关闭时忽略
            pass  # 保持幂等
