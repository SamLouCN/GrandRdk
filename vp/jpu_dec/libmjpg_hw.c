/*
 * libmjpg_hw.c — RDK S100 USB MJPG 相机硬件解码一体库 (vp5.1 自愈版)
 *
 * 抓帧(V4L2 mmap 拿原始 MJPG 字节, 不经 OpenCV 软解) + JPU 硬解 -> NV12。
 * Python ctypes 调用, 替代 cv2.VideoCapture(...).read() 的 CPU 软解链路。
 *
 * vp5.1 针对 JPU 实例崩溃的加固 (2026-09-13):
 *   0) 【根因级】拷贝先行: DQBUF 取出缓冲后, 先把 JPEG 字节拷到本库自己的
 *      stash 里, 再 QBUF 归还给驱动。旧实现是 DQBUF->QBUF->(之后才)memcpy,
 *      归还后驱动可能立刻写新帧, memcpy 读到撕裂帧, JPU 吃垃圾导致
 *      instance crash / "Unknown interrupt" / "JPU decoded error"。
 *   1) JPEG 帧校验: 每帧必须 SOI(0xFFD8) 开头 + EOI(0xFFD9) 结尾, 坏帧跳过
 *      并继续抓下一帧 (最多 MAX_BAD_JPEG 次), 不让坏帧喂进 JPU;
 *   2) codec 自愈: 任何 hb_mm_mc_* 调用失败(典型: 内核 jpu_free_instances
 *      detect instance crash -> HB_MEDIA_ERR_UNKNOWN 0xF0000001)时,
 *      自动 stop->release->get_default_context->initialize->configure->start
 *      重建 JPU 实例(官方 sample_codec 的 vp_codec_restart 即 stop 重置路径),
 *      随后同一帧重试一次; 重建限速 RESET_MIN_INTERVAL_MS, 避免抖振;
 *   3) 安全释放: mjcam_close 用 stop->release (pause 在 task state=6 时会被
 *      拒绝并刷错误), 出错静默处理, 不再把 codec 的内部错误打到终端;
 *   4) 超时收敛: 抓帧 poll 3s->1.5s, dequeue_in 2s->0.5s, queue_in 1s,
 *      dequeue_out 保持 2s (解码本身很快, 长超时只会掩盖故障)。
 *
 * 编译(板端):
 *   gcc -shared -fPIC -O2 -I/usr/hobot/include libmjpg_hw.c \
 *       -o libmjpg_hw.so -L/usr/hobot/lib -lmultimedia -lhbmem -lpthread
 *
 * API:
 *   void* mjcam_open(const char* device, int w, int h, int fps);
 *   int   mjcam_grab(void* h, uint8_t* nv12, size_t cap, int* ow, int* oh);
 *         // 抓一帧 MJPG -> JPU 解 -> NV12; 返回字节数或 <0
 *   void  mjcam_close(void* h);
 *
 * V4L2 流程参考官方 /app/multimedia_samples/sample_pipeline/uvc_capture_sample;
 * codec 用法参考 /app/multimedia_samples/sample_codec/sample_codec.c。
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include "hb_media_codec.h"

#define MAX_BUFS 4
#define GRAB_TIMEOUT_MS 1500          /* V4L2 poll 超时: 3s->1.5s, 掉线判定更快 */
#define DEQUEUE_IN_MS 500             /* 取输入码流缓冲超时 (正常应瞬时返回)   */
#define QUEUE_IN_MS 1000              /* 送码流缓冲超时                        */
#define DEQUEUE_OUT_MS 2000           /* 取解码输出帧超时 (JPU 解码本身 <1ms)  */
#define MAX_BAD_JPEG 8                /* 连续坏帧(无 SOI/EOI)最大跳过次数      */
#define RESET_MIN_INTERVAL_MS 200     /* 两次 codec 重建的最小间隔, 防抖振     */
#define JPEG_MIN_SIZE 128             /* 小于该长度直接视为坏帧                */

/* 返回码 (均 <0, Python 侧仅区分 ok/fail) */
#define RET_OK                0
#define RET_V4L2_FAIL        -1
#define RET_DEQUEUE_IN       -2
#define RET_BUF_TOO_SMALL    -3
#define RET_QUEUE_IN         -4
#define RET_DEQUEUE_OUT      -5
#define RET_NOT_FRAME        -6
#define RET_OUT_TOO_BIG      -7
#define RET_QUEUE_OUT        -8
#define RET_ALL_BAD_JPEG     -9
#define RET_REBUILD_FAIL    -10

typedef struct {
    int fd;
    int nbufs;
    void* mmap_ptr[MAX_BUFS];
    size_t mmap_len[MAX_BUFS];
    uint32_t buf_size[MAX_BUFS];
    uint32_t w, h;
} v4l2_cam;

typedef struct {
    v4l2_cam cam;
    uint8_t* jpeg_stash;      /* 拷贝区: DQBUF 后立刻把帧拷走再归还缓冲 */
    size_t jpeg_cap;          /* 拷贝区容量 = bitstream_buf_size */
    media_codec_context_t ctx;
    int jpu_ready;            /* codec 已 start, 可解码 */
    int ctx_inited;           /* ctx 已 initialize (stop/release 只在该状态调用) */
    int cam_ready;
    struct timespec last_reset;   /* 上次 codec 重建时刻 (限速用) */
} mjcam;

/* ---------- 工具 ---------- */

static int cam_has_valid_jpeg(const uint8_t* p, size_t len) {
    /* 最基础的 JPEG 完整性: SOI(FF D8) 开头 + EOI(FF D9) 结尾 + 最小长度。
       O(1) 检查, 不解析整帧, 200fps 下无感知开销。 */
    if (!p || len < JPEG_MIN_SIZE) return 0;
    if (p[0] != 0xFF || p[1] != 0xD8) return 0;            /* SOI   */
    if (p[len - 2] != 0xFF || p[len - 1] != 0xD9) return 0; /* EOI   */
    return 1;
}

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

/* ---------- V4L2 ---------- */

static int cam_open_dev(v4l2_cam* c, const char* device, int w, int h, int fps) {
    c->fd = open(device, O_RDWR | O_NONBLOCK);
    if (c->fd < 0) { fprintf(stderr, "[cam] open %s failed: %s\n", device, strerror(errno)); return -1; }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = (uint32_t)w;
    fmt.fmt.pix.height = (uint32_t)h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(c->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[cam] S_FMT MJPG failed: %s\n", strerror(errno));
        close(c->fd); return -1;
    }
    c->w = fmt.fmt.pix.width;
    c->h = fmt.fmt.pix.height;
    fprintf(stderr, "[cam] negotiated %ux%u fmt=0x%08x\n", c->w, c->h, fmt.fmt.pix.pixelformat);

    if (fps > 0) {
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(c->fd, VIDIOC_S_PARM, &parm) == 0) {
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator = (uint32_t)fps;
            ioctl(c->fd, VIDIOC_S_PARM, &parm);
        }
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = MAX_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(c->fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "[cam] REQBUFS failed: %s\n", strerror(errno));
        close(c->fd); return -1;
    }
    c->nbufs = (int)req.count;

    for (int i = 0; i < c->nbufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = (uint32_t)i;
        if (ioctl(c->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "[cam] QUERYBUF %d failed: %s\n", i, strerror(errno));
            return -1;
        }
        c->buf_size[i] = buf.length;
        c->mmap_len[i] = buf.length;
        c->mmap_ptr[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->fd, buf.m.offset);
        if (c->mmap_ptr[i] == MAP_FAILED) {
            fprintf(stderr, "[cam] mmap %d failed: %s\n", i, strerror(errno));
            return -1;
        }
        if (ioctl(c->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[cam] QBUF %d failed: %s\n", i, strerror(errno));
            return -1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(c->fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[cam] STREAMON failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/*
 * 取一帧并拷贝到 m->jpeg_stash, 然后才归还缓冲给驱动。
 * 返回 0=已取到(长度写 *out_len), -1=失败。 超出拷贝区容量的帧会被丢弃。
 */
static int cam_grab_copied(mjcam* m, const uint8_t** out, size_t* out_len) {
    v4l2_cam* c = &m->cam;
    struct pollfd pfd = { .fd = c->fd, .events = POLLIN };
    int pr = poll(&pfd, 1, GRAB_TIMEOUT_MS);
    if (pr <= 0) { fprintf(stderr, "[cam] poll %s\n", pr == 0 ? "timeout" : strerror(errno)); return -1; }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(c->fd, VIDIOC_DQBUF, &buf) < 0) {
        fprintf(stderr, "[cam] DQBUF failed: %s\n", strerror(errno));
        return -1;
    }
    uint32_t used = buf.bytesused;
    int oversized = used > m->jpeg_cap;
    if (oversized) used = (uint32_t)m->jpeg_cap;
    if (used > 0)
        memcpy(m->jpeg_stash, (uint8_t*)c->mmap_ptr[buf.index], used);

    if (ioctl(c->fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "[cam] QBUF(return) failed: %s\n", strerror(errno));
        return -1;
    }
    if (oversized) return -1;      /* 整帧超容, 丢弃 (异常, 上层按坏帧处理) */
    *out = m->jpeg_stash;
    *out_len = used;
    return 0;
}

/* ---------- JPU codec ---------- */

/* 用 m->cam 的当前分辨率重建 codec 实例 (自愈: 崩溃/坏帧后全新实例) */
static int codec_rebuild(mjcam* m) {
    /* 限速: 极短时间内重复触发说明持续异常, 直接失败让上层退出, 避免空转 */
    uint32_t now = now_ms();
    uint32_t last = (uint32_t)(m->last_reset.tv_sec * 1000u + m->last_reset.tv_nsec / 1000000u);
    if (last && (now - last) < RESET_MIN_INTERVAL_MS) {
        fprintf(stderr, "[jpu] rebuild too frequent, give up\n");
        return -1;
    }
    clock_gettime(CLOCK_MONOTONIC, &m->last_reset);
    m->jpu_ready = 0;

    /* 1) 停掉旧实例 (官方 restart 路径; 已崩溃时返回错误, 忽略) */
    if (m->ctx_inited) {
        (void)hb_mm_mc_stop(&m->ctx);
        (void)hb_mm_mc_release(&m->ctx);
        m->ctx_inited = 0;
    }

    /* 2) 全新上下文 + 初始化 + 配置 + 启动 */
    memset(&m->ctx, 0, sizeof(m->ctx));
    if (hb_mm_mc_get_default_context(MEDIA_CODEC_ID_JPEG, 0, &m->ctx) != 0) {
        fprintf(stderr, "[jpu] rebuild: get_default_context failed\n");
        return -1;
    }
    media_codec_context_t* ctx = &m->ctx;
    ctx->encoder = 0;
    ctx->codec_id = MEDIA_CODEC_ID_JPEG;
    mc_video_codec_dec_params_t* p = &ctx->video_dec_params;
    p->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    p->pix_fmt   = MC_PIXEL_FORMAT_NV12;
    p->bitstream_buf_size  = ((uint32_t)m->cam.w * m->cam.h * 3 / 2 + 0xfffu) & ~0xfffu;
    p->bitstream_buf_count = 3;
    p->frame_buf_count     = 3;
    p->jpeg_dec_config.frame_crop_enable = 0;
    p->jpeg_dec_config.rot_degree        = MC_CCW_0;
    p->jpeg_dec_config.mir_direction     = MC_DIRECTION_NONE;

    if (hb_mm_mc_initialize(ctx) != 0) {
        fprintf(stderr, "[jpu] rebuild: initialize failed\n");
        return -1;
    }
    m->ctx_inited = 1;
    if (hb_mm_mc_configure(ctx) != 0) {
        fprintf(stderr, "[jpu] rebuild: configure failed\n");
        (void)hb_mm_mc_release(ctx);
        m->ctx_inited = 0;
        return -1;
    }
    mc_av_codec_startup_params_t st;
    memset(&st, 0, sizeof(st));
    if (hb_mm_mc_start(ctx, &st) != 0) {
        fprintf(stderr, "[jpu] rebuild: start failed\n");
        (void)hb_mm_mc_release(ctx);
        m->ctx_inited = 0;
        return -1;
    }
    m->jpu_ready = 1;
    fprintf(stderr, "[jpu] codec ready (JPU instance ok)\n");
    return 0;
}

/* 单次解码序列 (不负责错误恢复): 返回 >=0 写入字节数 或 错误码 */
static int decode_once(mjcam* m, const uint8_t* jpeg, size_t jpeg_len,
                       uint8_t* nv12, size_t cap, int* ow, int* oh) {
    media_codec_context_t* ctx = &m->ctx;
    media_codec_buffer_t in, out;
    media_codec_output_buffer_info_t info;
    int32_t ret;

    memset(&in, 0, sizeof(in));
    in.type = MC_VIDEO_STREAM_BUFFER;
    ret = hb_mm_mc_dequeue_input_buffer(ctx, &in, DEQUEUE_IN_MS);
    if (ret != 0) { fprintf(stderr, "[jpu] dequeue_in failed %d\n", ret); return RET_DEQUEUE_IN; }
    if (in.vstream_buf.size < jpeg_len) {
        fprintf(stderr, "[jpu] stream buf too small %u < %zu\n", in.vstream_buf.size, jpeg_len);
        (void)hb_mm_mc_queue_input_buffer(ctx, &in, 0);
        return RET_BUF_TOO_SMALL;
    }
    in.vstream_buf.size = (uint32_t)jpeg_len;
    in.vstream_buf.stream_end = 0;
    memcpy(in.vstream_buf.vir_ptr, jpeg, jpeg_len);
    ret = hb_mm_mc_queue_input_buffer(ctx, &in, QUEUE_IN_MS);
    if (ret != 0) { fprintf(stderr, "[jpu] queue_in failed %d\n", ret); return RET_QUEUE_IN; }

    memset(&out, 0, sizeof(out));
    memset(&info, 0, sizeof(info));
    ret = hb_mm_mc_dequeue_output_buffer(ctx, &out, &info, DEQUEUE_OUT_MS);
    if (ret != 0) { fprintf(stderr, "[jpu] dequeue_out failed %d\n", ret); return RET_DEQUEUE_OUT; }
    if (out.type != MC_VIDEO_FRAME_BUFFER) {
        (void)hb_mm_mc_queue_output_buffer(ctx, &out, 0);
        return RET_NOT_FRAME;
    }
    uint32_t w = out.vframe_buf.width;
    uint32_t hgt = out.vframe_buf.height;
    uint32_t need = w * hgt * 3 / 2;
    if (need > cap) { (void)hb_mm_mc_queue_output_buffer(ctx, &out, 0); return RET_OUT_TOO_BIG; }
    memcpy(nv12, out.vframe_buf.vir_ptr[0], w * hgt);
    memcpy(nv12 + w * hgt, out.vframe_buf.vir_ptr[1], w * hgt / 2);
    if (ow) *ow = (int)w;
    if (oh) *oh = (int)hgt;
    ret = hb_mm_mc_queue_output_buffer(ctx, &out, 0);
    if (ret != 0) { fprintf(stderr, "[jpu] queue_out failed %d\n", ret); return RET_QUEUE_OUT; }
    return (int)need;
}

/* 前置声明 (mjcam_open 错误路径会调用 mjcam_close) */
void mjcam_close(void* h);

void* mjcam_open(const char* device, int w, int h, int fps) {
    mjcam* m = (mjcam*)calloc(1, sizeof(mjcam));
    if (!m) return NULL;
    memset(&m->cam, 0, sizeof(m->cam));
    m->cam.fd = -1;

    m->jpeg_cap = ((uint32_t)w * h * 3 / 2 + 0xfffu) & ~0xfffu;   /* 458752 @640x480 */
    if (m->jpeg_cap < 65536) m->jpeg_cap = 65536;
    m->jpeg_stash = (uint8_t*)malloc(m->jpeg_cap);
    if (!m->jpeg_stash) { free(m); return NULL; }

    if (cam_open_dev(&m->cam, device, w, h, fps) != 0) {
        free(m->jpeg_stash);
        free(m);
        return NULL;
    }
    m->cam_ready = 1;

    if (codec_rebuild(m) != 0) {
        mjcam_close(m);
        return NULL;
    }
    return (void*)m;
}

int mjcam_grab(void* h, uint8_t* nv12, size_t cap, int* ow, int* oh) {
    if (!h || !nv12) return RET_V4L2_FAIL;
    mjcam* m = (mjcam*)h;
    if (!m->cam_ready) return RET_V4L2_FAIL;
    if (!m->jpu_ready && codec_rebuild(m) != 0) return RET_REBUILD_FAIL;

    /* 1) 抓一帧 MJPG(已拷入 stash, 缓冲已归还); 坏帧直接跳过抓下一帧 */
    const uint8_t* jpeg = NULL;
    size_t jpeg_len = 0;
    int bad = 0;
    do {
        if (cam_grab_copied(m, &jpeg, &jpeg_len) != 0) return RET_V4L2_FAIL;
        if (cam_has_valid_jpeg(jpeg, jpeg_len)) break;
        bad++;
        if (bad == 1)
            fprintf(stderr, "[jpu] skip bad jpeg frame (len=%zu)\n", jpeg_len);
    } while (bad < MAX_BAD_JPEG);
    if (bad >= MAX_BAD_JPEG) {
        fprintf(stderr, "[jpu] too many bad jpeg frames (%d), give up this grab\n", bad);
        return RET_ALL_BAD_JPEG;
    }

    /* 2) 解码一次; 任何 codec 错误 -> 重建实例后同帧重试一次 */
    if (!m->jpu_ready && codec_rebuild(m) != 0) return RET_REBUILD_FAIL;
    int r = decode_once(m, jpeg, jpeg_len, nv12, cap, ow, oh);
    if (r > 0) return r;                       /* 成功 */
    if (r == RET_BUF_TOO_SMALL) return r;      /* 输入缓冲不足无解, 直接上报 */

    /* 自愈: 先重建, 重试同一帧 */
    fprintf(stderr, "[jpu] decode error rc=%d -> rebuild & retry\n", r);
    if (codec_rebuild(m) != 0) return RET_REBUILD_FAIL;
    int r2 = decode_once(m, jpeg, jpeg_len, nv12, cap, ow, oh);
    if (r2 > 0) return r2;
    fprintf(stderr, "[jpu] retry after rebuild still rc=%d\n", r2);
    return r2;
}

void mjcam_close(void* h) {
    if (!h) return;
    mjcam* m = (mjcam*)h;
    if (m->jpu_ready) {
        /* 用 stop(文档化 reset)->release: 若 task state=6/pause 会被拒时也能走完,
           出错静默 (实例可能已被内核回收) */
        (void)hb_mm_mc_stop(&m->ctx);
        (void)hb_mm_mc_release(&m->ctx);
        m->jpu_ready = 0;
        m->ctx_inited = 0;
    }
    if (m->cam_ready) {
        if (m->cam.fd >= 0) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(m->cam.fd, VIDIOC_STREAMOFF, &type);
            for (int i = 0; i < m->cam.nbufs; i++)
                if (m->cam.mmap_ptr[i] && m->cam.mmap_ptr[i] != MAP_FAILED)
                    munmap(m->cam.mmap_ptr[i], m->cam.mmap_len[i]);
            close(m->cam.fd);
        }
        m->cam_ready = 0;
    }
    if (m->jpeg_stash) { free(m->jpeg_stash); m->jpeg_stash = NULL; }
    free(m);
}