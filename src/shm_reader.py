#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""shm_reader.py — 读共享内存, Web 服务 / legacy 兼容层用"""
import json  # 标准库: 解析共享内存里的 det/stats JSON
import mmap  # 标准库: 把共享内存文件映射成内存视图
import os  # 标准库: 路径拼接、文件存在判断、open/fstat/close
import struct  # 标准库: 按 HDR_FMT 解包帧头（magic/seq/length…）
import sys  # 标准库: 插入模块搜索路径

_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录（src/）
_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 工程根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
for _p in (os.path.join(_ROOT, 'config'), _HERE):  # 要加入搜索路径的两个目录
    if _p not in sys.path:  # 已存在就不重复插，避免路径表膨胀
        sys.path.insert(0, _p)  # 插到最前：优先用工程内的同名模块

import main_config as MC  # 帧头常量来源（HDR_FMT/HDR_SIZE/HDR_MAGIC）；必须在改完 sys.path 之后 import


class ShmFrameReader:
    """读共享内存最新 JPEG 帧。"""  # 类职责：只读 mmap，取最新一帧 JPEG

    def __init__(self, path):  # 只记路径，真正的打开延迟到 _ensure()
        self.path = path  # 共享内存文件路径，如 /dev/shm/momo_frame_front
        self._fd = None  # 文件描述符，延迟打开
        self._mm = None  # mmap 对象；None 表示当前未映射
        self._size = 0  # 映射总字节数（含帧头）

    def _ensure(self):  # 保证映射可用：写端重建文件后能自动重连
        """打开/重开 mmap（写端删除文件后也能重连）。"""
        if self._mm is not None and os.path.exists(self.path):  # 已映射且文件还在
            return  # 直接复用现有映射
        if self._mm is not None:  # 映射在但文件没了：先释放旧的
            try:  # 释放过程可能出错，不能中断读帧
                self._mm.close()  # 关闭内存映射
                os.close(self._fd)  # 关闭文件描述符
            except Exception:  # 任何关闭异常都吞掉
                pass  # 下一步会重新建
            self._fd = self._mm = None  # 清空引用，回到未映射状态
        if not os.path.exists(self.path):  # 写端还没起来：文件不存在
            return  # 保持 None，read_latest 会返回无帧
        try:  # 打开并映射
            self._fd = os.open(self.path, os.O_RDONLY)  # 只读打开共享内存文件
            self._size = os.fstat(self._fd).st_size  # 取当前大小（写端扩容后会变）
            self._mm = mmap.mmap(self._fd, self._size, prot=mmap.PROT_READ)  # 只读映射，杜绝误写写端数据
        except Exception:  # 权限不足 / 长度为 0 等
            self._fd = self._mm = None  # 失败即置空，下轮再试

    def read_latest(self, after_seq=None):  # 取最新一帧；任何失败都返回 (-1, None)
        """返回 (seq, jpeg_bytes)；未更新返回 (seq, None)，无有效帧返回 (-1, None)。"""
        self._ensure()  # 先确保映射就绪
        if self._mm is None:  # 没映射成功（写端未启动）
            return -1, None  # seq=-1 表示当前无帧
        try:  # 可能读到"写一半"的帧，必须整段兜住
            header = self._mm[:MC.HDR_SIZE]
            magic, seq, length, wh, ts_us = struct.unpack(MC.HDR_FMT, header)
            if magic != MC.HDR_MAGIC or length == 0:  # 魔术字不对或长度 0：帧头未写完
                return -1, None  # 判为无帧
            if length > self._size - MC.HDR_SIZE:  # 长度越界：可能是正在写的不一致态
                return -1, None  # 不冒险读，判为无帧
            if seq == after_seq:
                return seq, None  # 100Hz 轮询时，旧帧只读帧头，不反复复制 JPEG。
            jpeg = bytes(self._mm[MC.HDR_SIZE:MC.HDR_SIZE + length])  # 深拷贝 JPEG 字节，避免返回会被写端改动的视图
            if self._mm[:MC.HDR_SIZE] != header:
                return -1, None  # 复制期间写端换帧，下一次取最新完整帧。
            return seq, jpeg  # 返回帧序号与 JPEG 数据
        except Exception:  # 解包失败/切片越界等
            return -1, None  # 统一无帧，读端永不崩

    def close(self):  # 释放映射与文件描述符
        try:  # 关闭可能抛异常，忽略即可
            if self._mm is not None:  # 有映射才关
                self._mm.close()  # 关闭内存映射
            if self._fd is not None:  # 有描述符才关
                os.close(self._fd)  # 关闭文件描述符
        except Exception:  # 忽略关闭异常
            pass  # 释放阶段无需补救


class ShmJsonReader:
    """读共享内存 JSON（覆盖写）。"""  # 类职责：读 det/stats 这类整块覆盖写的 JSON

    def __init__(self, path):  # 只记路径，每次 read() 重新打开文件
        self.path = path  # JSON 共享内存路径，如 /dev/shm/momo_stats_front

    def read(self):  # 读一次 JSON；失败一律返回空字典，绝不抛给上层
        if not os.path.exists(self.path):  # 写端还没建这个文件
            return {}  # 返回空 dict 而不是报错
        try:  # JSON 可能正被覆盖到一半，必须兜住
            with open(self.path, 'r', encoding='utf-8') as f:  # 文本方式整读
                raw = f.read().strip()  # 去掉首尾空白与补位的 \0
            if not raw:  # 空文件
                return {}  # 返回空 dict
            return json.loads(raw)  # 解析成 dict
        except Exception:  # 解析失败 / 权限问题
            return {}  # 空 dict 兜底，保证读端不崩
