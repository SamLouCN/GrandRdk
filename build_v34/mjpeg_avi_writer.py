# -*- coding: utf-8 -*-
"""MjpegAviWriter — 纯 stdlib 手写 MJPEG-AVI 容器（供上位机录像用，质量由调用方 imencode 决定）。

设计：
  · 只写**单个 RIFF 块**（不做 OpenDML 分块）—— 单文件上限约 1 GB（1073741824 B）。
    上位机单段录像按 30fps/q90 约 2.6 GB/h ⇒ 需要限制单文件时长或分段（调用方负责）。
  · `idx1` 索引在 close() 时统一回写，保证 VLC / ffplay / OpenCV 都能正常定位。
  · 帧率用 `dwScale/dwRate`（微秒/帧）。帧率可变时用动态 scale/rate 较复杂，这里固定。

用法：
    w = MjpegAviWriter(path, w=640, h=480, fps=30)
    for frame in frames:                  # frame 是 BGR ndarray
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        w.append(buf.tobytes())
    w.close()
"""
import struct

_GB = 1024 * 1024 * 1024


class MjpegAviWriter:
    """手写 MJPEG-AVI。append(jpeg_bytes) 逐帧写，close() 回填尺寸与索引。"""

    def __init__(self, path, width, height, fps=30.0):
        self.path = str(path)
        self.w = int(width)
        self.h = int(height)
        self.fps = float(fps) if fps and fps > 0 else 30.0
        self._frames = []          # (offset_in_file, jpeg_size)
        self._max_jpeg = 0
        self._closed = False
        self._f = open(self.path, "wb")
        # 先写头 + movi 头（占位），帧数据紧随其后
        hdrl = self._build_hdrl()
        self._f.write(b"RIFF")
        self._f.write(struct.pack("<I", 0))        # RIFF size（回填）
        self._f.write(b"AVI ")
        self._f.write(hdrl)
        self._movi_list_pos = self._f.tell()       # 'LIST' 的位置
        self._f.write(b"LIST")
        self._f.write(struct.pack("<I", 0))        # movi LIST size（回填）
        self._f.write(b"movi")
        self._movi_start = self._f.tell()          # 帧数据起点（含 4 字节 'movi' 之后）

    # ---------- 头部构造 ----------
    def _build_hdrl(self):
        rate = 1000
        scale = max(1, int(round(self.fps * 1000)))
        # avih：dwMicroSecPerFrame, dwMaxBytesPerSec, pad, flags, totalFrames, initial, streams,
        #       suggestedBuf, width, height, reserved[4]
        avih = struct.pack(
            "<IIIIIIIIIIIIII",
            int(round(1_000_000.0 / self.fps)),  # dwMicroSecPerFrame
            0,                                   # dwMaxBytesPerSec
            0,                                   # dwPaddingGranularity
            0x00000110,                          # AVIF_HASINDEX(0x10) | AVIF_TRUSTCKTYPE(0x100)
            0,                                   # dwTotalFrames（回填）
            0,                                   # dwInitialFrames
            1,                                   # dwStreams
            0,                                   # dwSuggestedBufferSize
            self.w, self.h, 0, 0, 0, 0)
        avih_chunk = b"avih" + struct.pack("<I", len(avih)) + avih

        # strh：fccType, fccHandler, dwFlags, wPriority, wLanguage, dwInitialFrames,
        #       dwScale, dwRate, dwStart, dwLength, dwSuggestedBufferSize, dwQuality,
        #       dwSampleSize, rcFrame(left,top,right,bottom)
        strh = (b"vids" + b"MJPG" +
                struct.pack("<I", 0) +                 # dwFlags
                struct.pack("<H", 0) +                 # wPriority
                struct.pack("<H", 0) +                 # wLanguage
                struct.pack("<I", 0) +                 # dwInitialFrames
                struct.pack("<I", rate) +              # dwScale
                struct.pack("<I", scale) +             # dwRate
                struct.pack("<I", 0) +                 # dwStart
                struct.pack("<I", 0) +                 # dwLength（回填 = 帧数）
                struct.pack("<I", 0) +                 # dwSuggestedBufferSize
                struct.pack("<I", 0xFFFFFFFF) +        # dwQuality = -1（默认，用编码器自带）
                struct.pack("<I", 0) +                 # dwSampleSize
                struct.pack("<hhhh", 0, 0, self.w, self.h))
        strh_chunk = b"strh" + struct.pack("<I", len(strh)) + strh

        # strf：BITMAPINFOHEADER（biCompression 用 MJPG，OpenCV/VLC 都认；用 'MJPG' 更稳）
        strf = struct.pack("<IiiHHIIiiII",
                           40, self.w, self.h, 1, 24,
                           0x47504A4D,                 # biCompression = 'MJPG' (little-endian)
                           0, 0, 0, 0, 0)
        strf_chunk = b"strf" + struct.pack("<I", len(strf)) + strf

        strl_body = strh_chunk + strf_chunk
        strl = b"LIST" + struct.pack("<I", 4 + len(strl_body)) + b"strl" + strl_body
        hdrl_body = avih_chunk + strl
        return b"LIST" + struct.pack("<I", 4 + len(hdrl_body)) + b"hdrl" + hdrl_body

    # ---------- 写帧 ----------
    def append(self, jpeg_bytes):
        """追加一帧（已编码的 JPEG 字节）。"""
        if self._closed:
            raise RuntimeError("writer 已关闭")
        size = len(jpeg_bytes)
        off = self._f.tell()
        # '00dc' 帧块；奇数长度补 1 字节 padding
        self._f.write(b"00dc")
        self._f.write(struct.pack("<I", size))
        self._f.write(jpeg_bytes)
        if size & 1:
            self._f.write(b"\x00")
        self._frames.append((off, size))
        if size > self._max_jpeg:
            self._max_jpeg = size

    # ---------- 收尾 ----------
    def close(self):
        if self._closed:
            return self._file_size_guard()
        self._closed = True
        f = self._f
        frame_count = len(self._frames)

        # 1) 索引 idx1：每条 16 字节 (ckid, flags, offset, size)
        idx_entries = b"".join(
            b"00dc" + struct.pack("<III", 0x10, off, size)
            for (off, size) in self._frames)
        f.write(b"idx1")
        f.write(struct.pack("<I", len(idx_entries)))
        f.write(idx_entries)

        file_end = f.tell()
        movi_list_size = (file_end - 8 - len(idx_entries) - 8) - self._movi_list_pos + 8
        # 精确算: movi LIST 的 size 字段 = 其内容字节数 = ('movi' 4B) + 所有帧块
        frames_bytes = sum(8 + size + (size & 1) for (_, size) in self._frames)
        movi_list_size = 4 + frames_bytes

        f.seek(0, 2)
        total = f.tell()
        # 2) 回填 movi LIST size
        f.seek(self._movi_list_pos + 4)
        f.write(struct.pack("<I", movi_list_size))

        # 3) 回填 avih.dwTotalFrames（avih 在 RIFF(12) + 'LIST'+size+'hdrl'(12) + 'avih'+size(8) + 8）
        avih_data_off = 12 + 12 + 8
        f.seek(avih_data_off + 4 * 4)          # 跳到 dwTotalFrames 字段
        f.write(struct.pack("<I", frame_count))

        # 4) 回填 strh.dwLength / dwSuggestedBufferSize
        #    strh 数据起点 = RIFF(12) + hdrl LIST(12) + avih chunk(8+56) + strl LIST(12) + 'strh'+size(8)
        strh_data_off = 12 + 12 + (8 + 56) + 12 + 8
        _strh_prefix = 4 + 4 + 4 + 2 + 2 + 4 + 4 + 4 + 4   # 到 dwLength 之前的字段总长
        f.seek(strh_data_off + _strh_prefix)
        f.write(struct.pack("<I", frame_count))            # dwLength
        # dwSuggestedBufferSize = 最大帧长
        f.write(struct.pack("<I", self._max_jpeg))

        # 5) 回填 RIFF size
        f.seek(4)
        f.write(struct.pack("<I", total - 8))
        f.flush()
        f.close()
        return self._file_size_guard(total)

    @staticmethod
    def _file_size_guard(total=None):
        return {"bytes": total, "over_1gb": bool(total and total > _GB)}

    @property
    def frame_count(self):
        return len(self._frames)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
