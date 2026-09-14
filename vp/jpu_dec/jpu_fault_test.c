/*
 * jpu_fault_test.c — JPU 实例故障注入 + 自愈配方验证 (vp5.1)
 *
 * 目的: 实证 libmjpg_hw.c 里 codec_rebuild() 的恢复配方在真实 JPU 故障下有效:
 *   喂一个"截断的 JPEG"(模拟相机 USB 掉线/坏帧) -> JPU 报错;
 *   不重建继续解 -> 大概率仍失败;
 *   按 停+释放+重建上下文+初始化+配置+启动 重建 -> 再解完好 JPEG 必须成功。
 *
 * 编译: gcc -O2 -I/usr/hobot/include jpu_fault_test.c -o jpu_fault_test \
 *           -L/usr/hobot/lib -lmultimedia -lhbmem -lpthread
 * 用法: ./jpu_fault_test <完好.jpg>      (默认 /app/res/assets/zebra_cls.jpg)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "hb_media_codec.h"

static media_codec_context_t ctx;

static int init_codec(int w, int h) {
    memset(&ctx, 0, sizeof(ctx));
    if (hb_mm_mc_get_default_context(MEDIA_CODEC_ID_JPEG, 0, &ctx) != 0) {
        printf("  get_default_context FAIL\n"); return -1;
    }
    ctx.encoder = 0;
    ctx.codec_id = MEDIA_CODEC_ID_JPEG;
    mc_video_codec_dec_params_t* p = &ctx.video_dec_params;
    p->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    p->pix_fmt   = MC_PIXEL_FORMAT_NV12;
    p->bitstream_buf_size  = ((uint32_t)w * h * 3 / 2 + 0xfffu) & ~0xfffu;
    p->bitstream_buf_count = 3;
    p->frame_buf_count     = 3;
    p->jpeg_dec_config.frame_crop_enable = 0;
    p->jpeg_dec_config.rot_degree        = MC_CCW_0;
    p->jpeg_dec_config.mir_direction     = MC_DIRECTION_NONE;
    if (hb_mm_mc_initialize(&ctx) != 0) { printf("  initialize FAIL\n"); return -1; }
    if (hb_mm_mc_configure(&ctx) != 0)  { printf("  configure FAIL\n");  return -1; }
    mc_av_codec_startup_params_t st;
    memset(&st, 0, sizeof(st));
    if (hb_mm_mc_start(&ctx, &st) != 0) { printf("  start FAIL\n"); return -1; }
    return 0;
}

/* 返回 0=成功, <0=失败; tag 仅用于打印 */
static int decode(const uint8_t* j, int n, const char* tag) {
    media_codec_buffer_t in, out;
    media_codec_output_buffer_info_t info;
    int32_t r;

    memset(&in, 0, sizeof(in));
    in.type = MC_VIDEO_STREAM_BUFFER;
    r = hb_mm_mc_dequeue_input_buffer(&ctx, &in, 500);
    if (r) { printf("  [%s] dequeue_in failed %d\n", tag, r); return -1; }
    if (in.vstream_buf.size < (uint32_t)n) {
        printf("  [%s] stream buf too small %u < %d\n", tag, in.vstream_buf.size, n);
        (void)hb_mm_mc_queue_input_buffer(&ctx, &in, 0);
        return -1;
    }
    in.vstream_buf.size = (uint32_t)n;
    in.vstream_buf.stream_end = 0;
    memcpy(in.vstream_buf.vir_ptr, j, n);
    r = hb_mm_mc_queue_input_buffer(&ctx, &in, 1000);
    if (r) { printf("  [%s] queue_in failed %d\n", tag, r); return -1; }

    memset(&out, 0, sizeof(out));
    memset(&info, 0, sizeof(info));
    r = hb_mm_mc_dequeue_output_buffer(&ctx, &out, &info, 2000);
    if (r) { printf("  [%s] dequeue_out failed %d\n", tag, r); return -1; }
    if (out.type != MC_VIDEO_FRAME_BUFFER) {
        printf("  [%s] not a frame buffer (type=%d)\n", tag, out.type);
        (void)hb_mm_mc_queue_output_buffer(&ctx, &out, 0);
        return -1;
    }
    printf("  [%s] frame %ux%u decode_result=0x%x\n",
           tag, out.vframe_buf.width, out.vframe_buf.height,
           info.jpeg_frame_info.decode_result);
    r = hb_mm_mc_queue_output_buffer(&ctx, &out, 0);
    if (r) { printf("  [%s] queue_out failed %d\n", tag, r); return -1; }
    /* 0x01=SUCCESS */
    return info.jpeg_frame_info.decode_result == 1 ? 0 : -1;
}

/* 与 libmjpg_hw.c codec_rebuild() 完全一致的恢复配方 */
static int rebuild(int w, int h) {
    (void)hb_mm_mc_stop(&ctx);
    (void)hb_mm_mc_release(&ctx);
    return init_codec(w, h);
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "/app/res/assets/zebra_cls.jpg";
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    fseek(f, 0, SEEK_END);
    long total = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* jpg = (uint8_t*)malloc(total);
    size_t got = fread(jpg, 1, total, f);
    fclose(f);
    if (got != (size_t)total) { fprintf(stderr, "short read\n"); return 2; }
    int n = (int)got;
    int trunc_n = n * 60 / 100;               /* 截断到 60%: 没有 EOI 的坏帧 */
    printf("== JPEG %s: %d bytes, truncated to %d bytes ==\n\n", path, n, trunc_n);

    const int W = 640, H = 480;
    int pass = 1;

    printf("[1] init codec 640x480\n");
    if (init_codec(W, H) != 0) { printf("FATAL: init failed\n"); return 3; }

    printf("[2] decode FULL jpeg (期望 0)\n");
    if (decode(jpg, n, "full") != 0) { printf("  <<< 意外失败, 环境异常, 中止\n"); return 4; }

    printf("[3] decode TRUNCATED jpeg (预期失败或 decode_result=0)\n");
    int r3 = decode(jpg, trunc_n, "trunc");
    printf("    trunc rc=%d  (预期 !=0)\n\n", r3);

    printf("[4] 不重建, 直接再解 FULL (观察是否已被故障污染)\n");
    (void)decode(jpg, n, "full-no-rebuild");

    printf("\n[5] 重建实例 (停+释放+重建+初始化+配置+启动)\n");
    if (rebuild(W, H) != 0) { printf("FATAL: rebuild failed\n"); return 5; }

    printf("[6] 重建后再解 FULL (关键断言, 期望 0)\n");
    if (decode(jpg, n, "full-after-rebuild") != 0) { printf("  <<< 自愈失败\n"); pass = 0; }

    printf("[7] 再解一帧 FULL, 确认稳定 (期望 0)\n");
    if (decode(jpg, n, "full-stability") != 0) { printf("  <<< 不稳定\n"); pass = 0; }

    printf("\n[8] teardown: stop + release (替代 pause, 应无错误刷屏)\n");
    (void)hb_mm_mc_stop(&ctx);
    (void)hb_mm_mc_release(&ctx);

    printf("\n== 结论: %s ==\n", pass ? "PASS: 自愈配方有效" : "FAIL: 自愈配方无效");
    return pass ? 0 : 1;
}