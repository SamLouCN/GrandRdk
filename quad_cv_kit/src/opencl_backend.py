"""Explicit OpenCL kernels for S100 image processing, Hough and batched fitting.

Detection LAB uses the local OpenCV build's exact byte lookup on the GPU;
HSV uses OpenCV's integer convention. CPU HSV saturation may differ by one.
The legacy hybrid path retains CPU LSD, partial groups and motion. The resident
pipeline uses device colour Hough, components, line/model selection and motion.
Ordered merging, split/contrast probes and complete-model geometry are batched
OpenCL search stages; intermediates remain resident until accepted output.
Frame operations defer profiling and synchronization until a host result is
needed; standalone operations remain synchronous.
"""
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time

import cv2
import numpy as np

from .opencl_runtime import LocalMemory, OpenCLError, OpenCLRuntime
from .native_peak_selection import create_native_selector
from .native_postprocess import create_native_postprocess
from .gpu_search import GPUSearchMixin
from .gpu_residency import GPUResidencyMixin
from .gpu_color import lab_a_table


def create_backend(mode='cpu', device_name=None, hough_backend='opencl', blur_mode='pyramid', quality='fast'):
    if mode == 'cpu':
        return None, dict(requested=mode, selected='cpu')
    if mode not in ('opencl', 'auto'):
        raise ValueError(f'Unknown CV backend: {mode}')
    try:
        backend = OpenCLBackend(device_name=device_name, hough_backend=hough_backend, blur_mode=blur_mode, quality=quality)
    except (OpenCLError, OSError) as exc:
        if mode == 'opencl':
            raise
        return None, dict(requested=mode, selected='cpu', fallback_reason=str(exc))
    return backend, dict(requested=mode, **backend.runtime.info, hough_backend=hough_backend, blur_mode=blur_mode, cv_quality=quality)


class OpenCLBackend(GPUResidencyMixin, GPUSearchMixin):
    def __init__(self, device_name=None, *, device_type='gpu', hough_backend='opencl', blur_mode='exact', quality='precise'):
        if quality not in ('fast', 'precise'):
            raise ValueError('CV quality must be fast or precise')
        self.quality = quality
        self.hough_peak_limit = 256
        self.work_counts = {}
        if hough_backend not in ('cpu', 'opencl'):
            raise ValueError('Hough backend must be cpu or opencl')
        self.hough_backend = hough_backend
        if blur_mode not in ('exact', 'pyramid'):
            raise ValueError('Blur mode must be exact or pyramid')
        self.blur_mode = blur_mode
        source = '\n'.join((Path(__file__).parent/'kernels'/name).read_text(encoding='utf-8')
                           for name in ('red_gate.cl', 'search.cl', 'residency.cl', 'color.cl', 'sharpen.cl', 'resident_pipeline.cl'))
        self.runtime = OpenCLRuntime(source, device_name, device_type)
        self._native_peak_selector, self.peak_selector_info = create_native_selector()
        self._native_postprocess, self.postprocess_info = create_native_postprocess()
        self.weights = {}
        self.maps_cache = None
        self.hsv_tables = None
        self.valid_cache = None
        self.operation_ms = {}
        self._hough_executor = None
        self._lsd_executor = None
        angles = np.arange(720, dtype=np.float64)*np.pi/720
        self.angle_values = np.float32(np.column_stack([np.cos(angles), np.sin(angles)]))
        try:
            self.angles = self.runtime.upload('hough_angles', self.angle_values)
            self.fast_angles = self.runtime.upload('hough_fast_angles', self.angle_values[::2])
            self.lab_a = self.runtime.upload('lab_a_table', lab_a_table())
        except BaseException:
            self.runtime.close()
            raise

    @contextmanager
    def _operation(self, name):
        started = time.perf_counter()
        try:
            yield
        finally:
            if getattr(self, '_resident_frame', None) is None:
                self.runtime.finish()
            self.operation_ms[name] = self.operation_ms.get(name, 0.)+(time.perf_counter()-started)*1000

    def reset_stats(self):
        self.runtime.finish()
        self.runtime.reset_stats()
        self.operation_ms.clear()
        self.work_counts.clear()

    def diagnostics(self):
        self.runtime.finish()
        resident = getattr(self, '_resident_core_active', False)
        return dict(**self.runtime.info,
            kernel_ms={key: round(value, 3) for key, value in self.runtime.kernel_ms.items()},
            operation_ms={key: round(value, 3) for key, value in self.operation_ms.items()},
            upload_bytes=self.runtime.upload_bytes, download_bytes=self.runtime.download_bytes,
            transfer_bytes=dict(getattr(self.runtime, 'transfer_bytes', {})),
            transfer_calls=dict(getattr(self.runtime, 'transfer_calls', {})),
            residency='frame' if getattr(self, '_resident_frame', None) is not None else 'operation',
            pipeline_version=7 if getattr(self, '_resident_core_active', False) else 2,
            cv_execution='resident' if getattr(self, '_resident_core_active', False) else 'hybrid',
            residency_strategy='direct-outputs-readonly-aliases',
            synchronization=dict(finish_calls=getattr(self.runtime, 'finish_calls', 0),
                                 read_wait_calls=getattr(self.runtime, 'read_wait_calls', 0)),
            profiling_cpu_ms=round(getattr(self.runtime, 'profiling_cpu_ms', 0.), 3),
            dispatch=dict(kernel_launch_calls=getattr(self.runtime, 'kernel_launch_calls', 0),
                          argument_set_calls=getattr(self.runtime, 'argument_set_calls', 0)),
            cpu_stages=[] if getattr(self, '_resident_core_active', False) else ['LSD LAB/CLAHE', 'Hough peak ordering',
                        'trimmed-line deduplication and partial groups', 'connected components', 'optical flow/RANSAC'],
            search_execution='gpu-resident' if resident else 'gpu-batched',
            hough_backend='opencl' if resident else self.hough_backend,
            hough_algorithm=('resident-color-top8-per-angle' if resident else 'opencv-HoughLinesP' if self.hough_backend == 'cpu' else
                'bounded-peaks-cooperative-runs' if self.quality == 'fast' else 'polar-votes-cooperative-greedy-consumption'),
            cv_quality='resident' if resident else self.quality, work_counts=dict(self.work_counts),
            peak_selector=dict(selected='device-tiled-stable-sort-mask-greedy-nms', neighbors_per_peak=72,
                suppression_words_per_peak=3, suppression_record_bytes=16,
                sort_capacity=2880, sort_tile_slots=256,
                kernel_names=['rg_peak_prepare', 'rg_peak_sort_tiles', 'rg_peak_sort_merge', 'rg_peak_sort_order', 'rg_peak_select'],
                total_gpu_ms=round(sum(self.runtime.kernel_ms.get(k, 0.) for k in
                    ('rg_peak_prepare', 'rg_peak_sort_tiles', 'rg_peak_sort_merge', 'rg_peak_sort_order', 'rg_peak_select')), 3)) if resident else dict(self.peak_selector_info),
            line_ranker=dict(selected='device-tiled-stable-topk', tile_slots=256,
                kernel_names=['rg_rank_tiles', 'rg_rank_merge', 'rg_rank_gather', 'rg_rank'],
                total_gpu_ms=round(sum(self.runtime.kernel_ms.get(k, 0.) for k in
                    ('rg_rank_tiles', 'rg_rank_merge', 'rg_rank_gather', 'rg_rank')), 3)) if resident else None,
            morphology=dict(selected='exact-binary-packed-pass-fusion', pixels_per_word=32,
                tile_size=[256, 8], grayscale_fallback='morph3_fused/morph3',
                kernel_names=['morph_pack', 'morph_packed1', 'morph_packed2', 'morph_packed3', 'morph_packed4'],
                total_gpu_ms=round(sum(self.runtime.kernel_ms.get(k, 0.) for k in
                    ('morph_pack', 'morph_packed1', 'morph_packed2', 'morph_packed3', 'morph_packed4', 'morph3_fused', 'morph3')), 3)),
            components=dict(selected='exact-row-run-union-statistics', connectivity=8,
                capacity_policy='ceil(width/2)-per-row-no-truncation',
                kernel_names=['rg_cc_runs', 'rg_cc_run_link', 'rg_cc_run_stats', 'rg_cc_run_filter', 'rg_white_open2'],
                total_gpu_ms=round(sum(self.runtime.kernel_ms.get(k, 0.) for k in
                    ('rg_cc_runs', 'rg_cc_run_link', 'rg_cc_run_stats', 'rg_cc_run_filter', 'rg_white_open2')), 3)) if resident else None,
            angle_peak_top=dict(selected='exact-lane-top8-merge',
                total_gpu_ms=round(self.runtime.kernel_ms.get('rg_peak_top_tiled', 0.), 3)) if resident else None,
            trimmed_line_merge=dict(selected='device-ordered-merge-and-validation') if resident else dict(self.postprocess_info),
            hough_angles=360 if resident or self.quality == 'fast' and self.hough_backend == 'opencl' else 720,
            hough_peak_limit=256 if resident else self.hough_peak_limit if self.quality == 'fast' and self.hough_backend == 'opencl' else None,
            line_sample_step=2 if resident or self.quality == 'fast' else 1,
            blur_mode=self.blur_mode,
            gpu_stages=(['target association and ROI', 'area canvas and grayscale pyramids',
                        'HSV/LAB and multiscale color', 'row-run eight-connected components',
                        'corner ranking and pyramidal LK', 'similarity RANSAC',
                        'color Hough and device peak selection', 'sampled fitting and ordered merge',
                        'line validation', 'complete/partial model selection', 'planar homography pose'] if resident else
                        ['fused brightness sharpening', 'line fitting', 'line trim',
                        'ordered line merge', 'line split and contrast median',
                        'quad geometry and valid-mask probes', 'model side support',
                        'partial joints and line color probes', 'detection HSV/LAB lookup', 'resident ROI transforms']),
            color_conversion='fused-hsv-exact-lab-a', lab_lookup_bytes=256**3,
            enhancement_mode='none' if getattr(self, '_preprocess_enhancement_removed', False) else 'sharpen-only',
            sharpening_algorithm='disabled' if getattr(self, '_preprocess_enhancement_removed', False) else 'normalized-value-fused-tile16',
            gaussian_algorithm=('fused-small-area4-large-scales' if self.blur_mode == 'pyramid' else 'fused-small-symmetric-tiled'),
            precision='float32')

    @staticmethod
    def _frame(frame):
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3 or not frame.size:
            raise ValueError('OpenCL image operations require a nonempty uint8 BGR frame')

    @staticmethod
    def _mask(mask):
        if not isinstance(mask, np.ndarray) or mask.ndim != 2 or not mask.size:
            raise ValueError('OpenCL line operations require a nonempty 2D mask')

    def _blur(self, source, w, h, sigma, prefix):
        rt = self.runtime
        if self.blur_mode == 'pyramid' and sigma >= 9 and min(w, h) >= 16:
            dw, dh = (w+3)//4, (h+3)//4
            down = rt.buffer(prefix+'_down', dw*dh*4)
            rt.run('area_down4', dw*dh, [source, down, w, h, dw, dh])
            # Area averaging itself adds variance 15/12 in source pixels.
            reduced_sigma = float(np.sqrt(sigma*sigma-15/12)/4)
            small = self._blur_exact(down, dw, dh, reduced_sigma, prefix+'_reduced')
            out = rt.buffer(prefix+'_blur', w*h*4)
            rt.run('linear_up4', w*h, [small, out, w, h, dw, dh])
            return out
        return self._blur_exact(source, w, h, sigma, prefix)

    def _blur_exact(self, source, w, h, sigma, prefix):
        rt = self.runtime
        # OpenCV's automatic support for a float32 source is cvRound(8*sigma+1)|1.
        size = int(np.rint(8*sigma+1)) | 1
        key = (size, sigma)
        weights = self.weights.get(key)
        if weights is None:
            weights = self.weights[key] = rt.upload(f'gaussian_{sigma}', cv2.getGaussianKernel(size, sigma, cv2.CV_32F))
        temp = rt.buffer(prefix+'_temp', w*h*4)
        result = rt.buffer(prefix+'_blur', w*h*4)
        radius = size//2
        if radius <= 5:
            groups = ((w+15)//16)*((h+15)//16)
            extent = 16+2*radius
            rt.run('gaussian_small_fused', groups*64,
                   [source, result, weights, w, h, radius,
                    LocalMemory(extent*extent*4), LocalMemory(extent*16*4),
                    LocalMemory((radius+1)*4)], local=64)
            return result
        horizontal_groups = ((w+255)//256)*h
        vertical_groups = ((w+15)//16)*((h+15)//16)
        taps = LocalMemory((radius+1)*4)
        rt.run('gaussian_tiled', horizontal_groups*64,
               [source, temp, weights, w, h, radius, 1, LocalMemory((256+2*radius)*4), taps], local=64)
        rt.run('gaussian_tiled', vertical_groups*64,
               [temp, result, weights, w, h, radius, 0, LocalMemory(16*(16+2*radius)*4), taps], local=64)
        return result

    def _morph(self, source, w, h, steps, prefix, *, binary=False):
        rt = self.runtime
        if binary and 1 <= len(steps) <= 4:
            words = (w+31)//32
            packed = rt.buffer(prefix+'_packed', words*h*4)
            target = self.device_output(prefix+'_result', w*h)
            operations = sum(int(bool(dilate)) << i for i, dilate in enumerate(steps))
            rt.run('morph_pack', words*h, [source, packed, w, h])
            groups = ((words+7)//8)*((h+7)//8)
            scratch = 10*(8+2*len(steps))*4
            rt.run('morph_packed'+str(len(steps)), groups*64, [packed, target, w, h, operations,
                   LocalMemory(scratch), LocalMemory(scratch)], local=64)
            return target
        if 1 < len(steps) <= 4:
            target = self.device_output(prefix+'_result', w*h)
            extent = 16+2*len(steps)
            operations = sum(int(bool(dilate)) << i for i, dilate in enumerate(steps))
            groups = ((w+15)//16)*((h+15)//16)
            rt.run('morph3_fused', groups*64, [source, target, w, h, len(steps), operations,
                   LocalMemory(extent*extent), LocalMemory(extent*extent)], local=64)
            return target
        buffers = [rt.buffer(prefix+'_a', w*h), rt.buffer(prefix+'_b', w*h)]
        result = source
        for i, dilate in enumerate(steps):
            target = self.device_output(prefix+'_result', w*h) if i == len(steps)-1 else buffers[i % 2]
            rt.run('morph3', w*h, [result, target, w, h, int(dilate)])
            result = target
        return result

    def _local_blur(self, chroma, output, w, h, sigma, first, prefix):
        """Write local evidence directly from the final Gaussian/upscale pass."""
        rt = self.runtime
        if self.blur_mode == 'pyramid' and sigma >= 9 and min(w, h) >= 16:
            dw, dh = (w+3)//4, (h+3)//4
            down = rt.buffer(prefix+'_down', dw*dh*4)
            rt.run('area_down4', dw*dh, [chroma, down, w, h, dw, dh])
            reduced_sigma = float(np.sqrt(sigma*sigma-15/12)/4)
            small = self._blur_exact(down, dw, dh, reduced_sigma, prefix+'_reduced')
            rt.run('linear_up4_local', w*h, [small, chroma, output, w, h, dw, dh, int(first)])
            return
        size = int(np.rint(8*sigma+1)) | 1
        key = (size, sigma)
        weights = self.weights.get(key)
        if weights is None:
            weights = self.weights[key] = rt.upload(f'gaussian_{sigma}', cv2.getGaussianKernel(size, sigma, cv2.CV_32F))
        radius = size//2
        temp = rt.buffer(prefix+'_temp', w*h*4)
        taps = LocalMemory((radius+1)*4)
        rt.run('gaussian_tiled', ((w+255)//256)*h*64,
               [chroma, temp, weights, w, h, radius, 1,
                LocalMemory((256+2*radius)*4), taps], local=64)
        rt.run('gaussian_local', ((w+15)//16)*((h+15)//16)*64,
               [temp, chroma, output, weights, w, h, radius, int(first),
                LocalMemory(16*(16+2*radius)*4), taps], local=64)

    def _tube(self, frame, prefix, profile=None, stage='color'):
        rt = self.runtime
        h, w = frame.shape[:2]
        hsv, chroma = self.color_device(frame, prefix)
        if profile is not None:
            profile.mark(stage+'.enqueue_convert')
        local = rt.buffer(prefix+'_local', w*h*4)
        for i, sigma in enumerate((3, 9, 18)):
            self._local_blur(chroma, local, w, h, sigma, i == 0, prefix)
        if profile is not None:
            profile.mark(stage+'.enqueue_blur')
        mask, score = rt.buffer(prefix+'_mask', w*h), self.device_output(prefix+'_score', w*h*4)
        rt.run('tube_threshold', w*h, [hsv, chroma, local, mask, score, w*h])
        mask = self._morph(mask, w, h, [True, False], prefix+'_close', binary=True)
        if profile is not None:
            profile.mark(stage+'.enqueue_threshold')
        return mask, score

    def combined_evidence(self, enhanced, reference, profile=None, prefix='color'):
        self._frame(enhanced)
        self._frame(reference)
        if enhanced.shape != reference.shape:
            raise ValueError('Color evidence frames must have matching dimensions')
        with self._operation('color_evidence'):
            rt = self.runtime
            h, w = reference.shape[:2]
            clean, score = self._tube(reference, 'reference', profile, prefix+'.reference')
            if enhanced is not reference:
                extra, _ = self._tube(enhanced, 'enhanced', profile, prefix+'.enhanced')
                near = self._morph(clean, w, h, [True], 'near', binary=True)
                combined = self.device_output('combined_mask', w*h)
                rt.run('combine_masks', w*h, [clean, near, extra, combined, w*h])
                clean = combined
            score_buffer = score
            mask, score = rt.read_many([(clean, (h, w), np.uint8), (score_buffer, (h, w), np.float32)])
            # Preserve device results while CPU LSD/tracking reads host views.
            self.register_resident(mask, clean)
            self.register_resident(score, score_buffer)
            if profile is not None:
                profile.mark(prefix+'.readback')
                profile.count('color_evidence_calls')
            return mask, score

    def contrast_signal(self, frame):
        self._frame(frame)
        with self._operation('contrast_signal'):
            h, w = frame.shape[:2]
            rt = self.runtime
            src = self.device_input('log_bgr', frame)
            out = rt.buffer('log_out', w*h*4)
            rt.run('log_signal', w*h, [src, out, w*h])
            return rt.read(out, (h, w), np.float32)

    def red_mask(self, frame):
        self._frame(frame)
        with self._operation('partial_red_mask'):
            rt = self.runtime
            h, w = frame.shape[:2]
            bgr = self.device_input('partial_bgr', frame)
            hsv = self.hsv_device(frame, 'partial_hsv', bgr)
            signal = rt.buffer('partial_signal', w*h*4)
            rt.run('log_signal', w*h, [bgr, signal, w*h])
            blur = self._blur(signal, w, h, 9, 'partial')
            mask = rt.buffer('partial_mask', w*h)
            rt.run('strict_red', w*h, [hsv, signal, blur, mask, w*h])
            mask = self._morph(mask, w, h, [True, False, False, True], 'partial_morph', binary=True)
            result = rt.read(mask, (h, w), np.uint8)
            _, labels, stats, _ = cv2.connectedComponentsWithStats(result)
            keep = (stats[:, cv2.CC_STAT_AREA] >= 35) & (np.maximum(stats[:, 2], stats[:, 3]) >= 25)
            keep[0] = False
            return np.uint8(keep[labels])*255

    def enhance(self, frame, gain=1.2, valid_mask=None, clahe_clip=0, clahe_blend=0,
                sharpen_amount=.6, saturation_gain=1, *, _source=None, _valid=None):
        self._frame(frame)
        with self._operation('enhancement'):
            if gain == 1 and (clahe_clip == 0 or clahe_blend == 0) and sharpen_amount == 0 and saturation_gain == 1:
                return frame.copy()
            rt = self.runtime
            h, w = frame.shape[:2]
            if not (clahe_clip > 0 and clahe_blend > 0):
                return self._enhance_device(frame, gain, valid_mask, sharpen_amount, saturation_gain, _source, _valid)
            hsv_np = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            valid_np = np.ones((h, w), np.uint8) if valid_mask is None else np.uint8(np.asarray(valid_mask, bool))
            if valid_np.shape != (h, w):
                raise ValueError('Enhancement valid mask must match the frame')
            active = (hsv_np[:, :, 2] > 0) & (valid_np > 0)
            hsv, valid = rt.upload('enhance_hsv', hsv_np), rt.upload('enhance_valid', valid_np)
            hist = rt.upload('value_histogram', np.zeros(256, np.uint32))
            rt.run('histogram_value', ((w*h+511)//512)*64, [hsv, valid, hist, w*h, LocalMemory(256*4)], local=64)
            histogram = rt.read(hist, (256,), np.uint32)
            count = int(histogram.astype(np.uint64).sum())
            if not count:
                return frame.copy()
            cumulative = histogram.astype(np.uint64).cumsum()
            low = int(np.searchsorted(cumulative, (count-1)//2+1))
            high = int(np.searchsorted(cumulative, count//2+1))
            pivot = (low+high)/2
            fill = (np.clip((low-pivot)*gain+pivot, 0, 255)+np.clip((high-pivot)*gain+pivot, 0, 255))/2
            initial = rt.buffer('enhance_initial', 4)
            use_initial = int(clahe_clip > 0 and clahe_blend > 0)
            if use_initial:
                histogram_input = hsv_np[:, :, 2].copy()
                histogram_input[~active] = round(pivot)
                local = cv2.createCLAHE(clahe_clip, (8, 8)).apply(histogram_input)
                values = np.float32((1-clahe_blend)*hsv_np[:, :, 2].astype(np.float32)+clahe_blend*local)
                pivot = float(np.median(values[active]))
                fill = float(np.median(np.clip((values[active]-pivot)*gain+pivot, 0, 255)))
                initial = rt.upload('enhance_initial', values)
            value, blur_input = rt.buffer('enhance_value', w*h*4), rt.buffer('enhance_blur_input', w*h*4)
            rt.run('contrast_value', w*h, [hsv, valid, initial, value, blur_input, w*h, float(pivot), float(gain), float(fill), use_initial])
            blur = self._blur(blur_input, w, h, 1.2, 'enhance') if sharpen_amount else value
            out = rt.buffer('enhance_output', w*h*3)
            rt.run('sharpen_value', w*h, [hsv, value, blur, out, w*h, float(sharpen_amount), float(saturation_gain)])
            result = cv2.cvtColor(rt.read(out, (h, w, 3), np.uint8), cv2.COLOR_HSV2BGR)
            result[~active] = frame[~active]
            return result

    def _enhance_device(self, frame, gain, valid_mask, amount, saturation, source=None, valid=None):
        rt = self.runtime
        h, w = frame.shape[:2]
        source = self.device_input('enhance_source', frame) if source is None else source
        if self.hsv_tables is None:
            values = np.arange(1, 256, dtype=float)
            self.hsv_tables = (rt.upload('hsv_sdiv', np.r_[0, np.rint((255 << 12)/values)].astype(np.int32)),
                               rt.upload('hsv_hdiv', np.r_[0, np.rint((180 << 12)/(6*values))].astype(np.int32)))
        if valid is None:
            valid_np = np.ones((h, w), np.uint8) if valid_mask is None else np.uint8(np.asarray(valid_mask, bool))
            if valid_np.shape != (h, w):
                raise ValueError('Enhancement valid mask must match the frame')
            valid = rt.upload('enhance_valid', valid_np)
        hsv = rt.buffer('enhance_hsv', h*w*3)
        rt.run('bgr_hsv', w*h, [source, hsv, *self.hsv_tables, w*h])
        hist = rt.upload('value_histogram', np.zeros(256, np.uint32))
        rt.run('histogram_value', ((w*h+511)//512)*64, [hsv, valid, hist, w*h, LocalMemory(256*4)], local=64)
        params = rt.buffer('enhance_parameters', 12)
        rt.run('median_parameters', 1, [hist, params, float(gain)])
        value, blur_input = rt.buffer('enhance_value', w*h*4), rt.buffer('enhance_blur_input', w*h*4)
        rt.run('contrast_device', w*h, [hsv, valid, params, value, blur_input, w*h])
        blur = self._blur(blur_input, w, h, 1.2, 'enhance') if amount else value
        out = self.device_output('enhance_output_bgr', w*h*3)
        rt.run('sharpen_bgr', w*h, [source, hsv, valid, value, blur, out, w*h, float(amount), float(saturation)])
        return self.capture_read(out, (h, w, 3), np.uint8)

    def sharpen(self, frame, amount=.6, valid_mask=None, *, _source=None, _valid=None):
        """One fused GPU launch, then the single BGR read required by YOLO."""
        self._frame(frame)
        if not np.isfinite(amount) or not 0 <= amount <= 2:
            raise ValueError('Sharpen amount must be finite and in [0, 2]')
        h, w = frame.shape[:2]
        if valid_mask is not None and valid_mask.shape != (h, w):
            raise ValueError('Sharpen valid mask must match the frame')
        with self._operation('enhancement'):
            if amount == 0:
                if _source is not None:
                    return self.capture_read(_source, (h, w, 3), np.uint8)
                return self.copy_host(frame)
            rt = self.runtime
            source = self.device_input('sharpen_source', frame) if _source is None else _source
            valid = source
            if valid_mask is not None:
                valid = _valid if _valid is not None else self.device_input('sharpen_valid', np.uint8(valid_mask))
            key = (11, 1.2)
            weights = self.weights.get(key)
            if weights is None:
                weights = self.weights[key] = rt.upload('gaussian_1.2', cv2.getGaussianKernel(11, 1.2, cv2.CV_32F))
            out = self.device_output('sharpen_output_bgr', frame.size)
            rt.run('sharpen_only', ((w+15)//16)*((h+15)//16)*64,
                   [source, valid, out, weights, w, h, int(valid_mask is not None), float(amount),
                    LocalMemory(26*26*4), LocalMemory(26*26*4),
                    LocalMemory(26*16*4), LocalMemory(26*16*4), LocalMemory(6*4)], local=64)
            return self.capture_read(out, (h, w, 3), np.uint8)

    def preprocess(self, frame, valid_mask, amount=0., maps=None, *, device_reference=False, rotate_180=False):
        """Rotate/correct only; return clean pixels for both CV and external I/O.

        There is no enhancement launch or extra image. Resident consumers keep
        the clean device buffer while BPU/rendering receives one host readback.
        The correction wall checkpoint includes that required output wait.
        amount is an ignored compatibility argument for existing integrations;
        it cannot enable sharpening. New calls should omit it and name maps.
        """
        self._frame(frame)
        if valid_mask is not None and valid_mask.shape != frame.shape[:2]:
            raise ValueError('Preprocessing valid mask must match the frame')
        if device_reference and getattr(self, '_resident_frame', None) is None:
            raise ValueError('Device-only preprocessing requires frame_batch')
        self._preprocess_enhancement_removed = True
        started = time.perf_counter()
        raw = self.rotate180(frame, device_only=device_reference) if rotate_180 else frame
        fixed = raw if maps is None else self.remap(raw, *maps, device_only=device_reference)
        device_frame = self.resident_buffer(fixed)
        if device_frame is None and getattr(self, '_resident_frame', None) is not None:
            device_frame = self.device_input('preprocess_source', fixed)
        output = (self.capture_read(device_frame, fixed.shape, np.uint8)
                  if device_reference and (rotate_180 or maps is not None) else fixed)
        elapsed_ms = (time.perf_counter()-started)*1000
        return fixed, output, elapsed_ms, 0.

    def rotate180(self, frame, *, device_only=False):
        self._frame(frame)
        rt = self.runtime
        src = self.device_input('rotation_source', frame)
        out = self.device_output('rotation_output', frame.size)
        rt.run('rg_rotate_bgr', frame.shape[0]*frame.shape[1], [src, out, frame.shape[0]*frame.shape[1]])
        return (self.device_descriptor(out, frame.shape, np.uint8) if device_only else
                self.capture_read(out, frame.shape, np.uint8))

    def trim_lines(self, lines, mask, score, bounds):
        from .gate_models import merge_trimmed
        if not lines:
            return []
        with self._operation('trim_lines'):
            rt = self.runtime
            h, w = mask.shape
            x0, y0, x1, y1 = bounds
            corners = np.array([[x0, y0], [x1-1, y0], [x1-1, y1-1], [x0, y1-1]], float)
            params = []
            for line in lines:
                span = corners@line['d']
                a = line['n']*line['b']+line['d']*span.min()
                b = line['n']*line['b']+line['d']*span.max()
                count = max(10, int(np.linalg.norm(b-a)/(2 if self.quality == 'fast' else 1)))
                params.append([*a, *b, *line['d'], *line['n'], line['lo'], line['hi'], line['width'], count])
            params = np.float32(params)
            stride = int(params[:, 11].max())
            n = len(lines)
            maxruns = stride//(12 if self.quality == 'fast' else 24)+2
            seeds, src, colors = (rt.upload('trim_params', params),
                self.search_input('mask', mask, np.uint8), self.search_input('score', score, np.float32))
            hits = rt.buffer('trim_hits', n*stride)
            expanded, closed = rt.buffer('trim_expanded', n*stride), rt.buffer('trim_closed', n*stride)
            results, counts = rt.buffer('trim_results', n*maxruns*6*4), rt.buffer('trim_counts', n*4)
            rt.run('trim_probe', n*stride, [src, seeds, hits, w, h, stride, n])
            morph_radius = 4 if self.quality == 'fast' else 9
            rt.run('trim_morph', n*stride, [hits, expanded, seeds, stride, n, 1, morph_radius])
            rt.run('trim_morph', n*stride, [expanded, closed, seeds, stride, n, 0, morph_radius])
            if self.quality == 'fast':
                widths, strong = rt.buffer('trim_widths', n*stride), rt.buffer('trim_strong', n*stride)
                rt.run('trim_sections', n*stride, [src, colors, seeds, widths, strong, w, h, stride, n])
                rt.run('trim_extract_fast', n, [seeds, hits, closed, widths, strong, results, counts, stride, n, maxruns])
            else:
                rt.run('trim_extract', n, [src, colors, seeds, hits, closed, results, counts, w, h, stride, n, maxruns])
            count_rows, rows = rt.read_many([(counts, (n,), np.int32),
                                           (results, (n, maxruns, 6), np.float32)])
            if np.any(count_rows > maxruns):
                raise OpenCLError('Trim run capacity exceeded')
            candidates = []
            for index, line in enumerate(lines):
                for row in rows[index, :count_rows[index]]:
                    candidates.append(dict(line, lo=float(row[0]), hi=float(row[1]), width=float(row[2]),
                        support=float(row[3]), strong_lo=float(row[4]), strong_hi=float(row[5])))
            return (self._native_postprocess.merge_trimmed(candidates) if self._native_postprocess is not None
                    else merge_trimmed(candidates))

    def side_support(self, mask, lines, segments):
        with self._operation('model_side_support'):
            if not lines:
                return []
            rt = self.runtime
            h, w = mask.shape
            params = np.float32([[*np.asarray(segment).reshape(-1), *line['d'], line['width'],
                line['lo'], line['hi'], line.get('strong_lo', line['lo']), line.get('strong_hi', line['hi']), 0]
                for line, segment in zip(lines, segments)])
            src, seeds = self.search_input('mask', mask, np.uint8), rt.upload('side_params', params)
            out = rt.buffer('side_support', len(lines)*4)
            rt.run('side_support', len(lines), [src, seeds, out, w, h, len(lines)])
            return rt.read(out, (len(lines),), np.float32)

    def remap(self, frame, map_x, map_y, *, device_only=False):
        self._frame(frame)
        with self._operation('correction'):
            rt = self.runtime
            h, w = frame.shape[:2]
            if map_x.shape != (h, w) or map_y.shape != (h, w):
                raise ValueError('OpenCL correction maps must match the frame')
            # Keep the source arrays alive: identity comparison cannot alias a
            # newly allocated map whose Python id reused a released object's id.
            if self.maps_cache is None or self.maps_cache[0] is not map_x or self.maps_cache[1] is not map_y:
                self.maps_cache = (map_x, map_y, rt.upload('map_x', np.float32(map_x)), rt.upload('map_y', np.float32(map_y)))
            src, out = self.device_input('remap_source', frame), self.device_output('remap_output', h*w*3)
            rt.run('remap_bgr', w*h, [src, self.maps_cache[2], self.maps_cache[3], out, w, h])
            return (self.device_descriptor(out, (h, w, 3), np.uint8) if device_only else
                    self.capture_read(out, (h, w, 3), np.uint8))

    def hough_segments(self, mask, threshold=35, min_length=55, max_gap=10):
        self._mask(mask)
        if any(not isinstance(value, (int, np.integer)) for value in (threshold, min_length, max_gap)) or threshold < 1 or min_length < 1 or max_gap < 0:
            raise ValueError('Hough threshold/min_length must be positive integers; max_gap must be nonnegative')
        with self._operation('hough'):
            if self.hough_backend == 'cpu':
                segments = cv2.HoughLinesP(np.uint8(mask), 1, np.pi/720, threshold,
                                           minLineLength=min_length, maxLineGap=max_gap)
                return np.empty((0, 4), np.float32) if segments is None else np.float32(segments.reshape(-1, 4))
            if self.quality == 'fast':
                return self._hough_fast(mask, threshold, min_length, max_gap)
            rt = self.runtime
            h, w = mask.shape
            src = self.device_input('hough_mask', mask)
            points, count = rt.buffer('hough_points', w*h*8), rt.upload('hough_count', np.zeros(1, np.uint32))
            rt.run('compact_foreground', w*h, [src, points, count, w, h])
            npoints = int(rt.read(count, (1,), np.uint32)[0])
            if not npoints:
                return np.empty((0, 4), np.float32)
            radius = int(np.ceil(np.hypot(w-1, h-1)))+1
            nrhos = 2*radius+1
            votes = rt.buffer('hough_votes', 720*nrhos*4)
            rt.run('hough_vote', 720*64, [points, npoints, self.angles, votes, nrhos, radius, LocalMemory(nrhos*4)], local=64)
            peaks, count = rt.buffer('hough_peaks', 720*nrhos*16), rt.upload('peak_count', np.zeros(1, np.uint32))
            rt.run('hough_peaks', 720*nrhos, [votes, peaks, count, nrhos, 720, threshold])
            npeaks = int(rt.read(count, (1,), np.uint32)[0])
            if not npeaks:
                return np.empty((0, 4), np.float32)
            peak_rows = rt.read(peaks, (npeaks, 4), np.int32)
            order = np.lexsort((peak_rows[:, 1], peak_rows[:, 0], -peak_rows[:, 2]))
            peaks = rt.upload('hough_peaks_sorted', peak_rows[order])
            maxruns = int(np.ceil(np.hypot(w-1, h-1)/min_length))+2
            out = rt.buffer('hough_segments', npeaks*maxruns*16)
            counts = rt.buffer('hough_run_counts', npeaks*4)
            rt.run('hough_runs', npeaks, [src, peaks, self.angles, out, counts, w, h, npeaks, radius, maxruns, min_length, max_gap])
            counts_np = rt.read(counts, (npeaks,), np.int32)
            if np.any(counts_np > maxruns):
                raise OpenCLError('Hough run capacity exceeded; no segments were silently dropped')
            remaining = rt.upload('hough_unclaimed', np.uint32(mask != 0))
            selected = rt.buffer('hough_selected', npeaks*maxruns*16)
            count = rt.buffer('hough_selected_count', 4)
            rt.run('hough_select_parallel', 64, [remaining, out, counts, selected, count,
                                      w, h, npeaks, maxruns, threshold, LocalMemory(64*4)], local=64)
            nselected = int(rt.read(count, (1,), np.uint32)[0])
            return rt.read(selected, (nselected, 4), np.float32)

    def _hough_fast(self, mask, threshold, min_length, max_gap):
        """Half-density votes and bounded, diverse seeds; fit on the original mask.

        Every peak owns a work group, so pixel probes run in parallel. Spatial
        peak suppression replaces globally ordered pixel consumption. This is
        intentionally approximate; tight parallel rods can lose a seed.
        """
        rt = self.runtime
        h, w = mask.shape
        started = time.perf_counter()
        src = self.device_input('hough_mask', mask)
        points = rt.buffer('hough_points', w*h*8)
        metadata = rt.upload('hough_metadata', np.zeros(2, np.uint32))
        rt.run('compact_foreground_fast', w*h, [src, points, metadata, w, h])
        radius = int(np.ceil(np.hypot(w-1, h-1)))+1
        nrhos = 2*radius+1
        votes = rt.buffer('hough_votes', 360*nrhos*4)
        rt.run('hough_vote_compacted', 360*64, [points, metadata, self.fast_angles, votes, nrhos, radius, LocalMemory(nrhos*4)], local=64)
        peaks = rt.buffer('hough_peaks', 360*nrhos*16)
        rt.run('hough_peaks_compacted', 360*nrhos, [votes, peaks, metadata, nrhos, 360, max(1, (threshold+1)//2)])
        npoints, npeaks = map(int, rt.read(metadata, (2,), np.uint32))
        self.operation_ms['hough.compact_vote_peaks'] = self.operation_ms.get('hough.compact_vote_peaks', 0)+(time.perf_counter()-started)*1000
        self.work_counts.update(hough_foreground_points=npoints, hough_peak_count=npeaks, hough_nms_peaks=0, hough_scanned_peaks=0)
        if not npeaks:
            return np.empty((0, 4), np.float32)
        rows = rt.read(peaks, (npeaks, 4), np.int32)
        started = time.perf_counter()
        rows = self._diverse_peaks(rows, radius, w, h)
        self.operation_ms['hough.peak_select'] = self.operation_ms.get('hough.peak_select', 0)+(time.perf_counter()-started)*1000
        npeaks = len(rows)
        self.work_counts['hough_scanned_peaks'] = npeaks
        if not npeaks:
            return np.empty((0, 4), np.float32)
        started = time.perf_counter()
        peaks = rt.upload('hough_peaks_sorted', rows)
        stride = int(np.ceil(np.hypot(w-1, h-1)/2))+2
        maxruns = int(np.ceil(np.hypot(w-1, h-1)/min_length))+2
        out, counts = rt.buffer('hough_segments', npeaks*maxruns*16), rt.buffer('hough_run_counts', npeaks*4)
        rt.run('hough_runs_fast', npeaks*64, [src, peaks, self.fast_angles, out, counts,
            w, h, npeaks, radius, maxruns, min_length, max_gap, stride, LocalMemory(stride*4)], local=64)
        counts_np, segments = rt.read_many([(counts, (npeaks,), np.int32), (out, (npeaks, maxruns, 4), np.float32)])
        if np.any(counts_np > maxruns):
            raise OpenCLError('Hough run capacity exceeded')
        self.operation_ms['hough.runs_readback'] = self.operation_ms.get('hough.runs_readback', 0)+(time.perf_counter()-started)*1000
        return segments[np.arange(maxruns)[None, :] < counts_np[:, None]]

    def _diverse_peaks(self, rows, radius, w, h):
        normals = self.angle_values[::2][rows[:, 0]].astype(float)
        # Match the orientation rules used by the downstream pole fitter.
        vertical = np.abs(normals[:, 1]) < np.abs(normals[:, 0])*.65
        horizontal = np.abs(normals[:, 0]) < np.abs(normals[:, 1])*.7
        ids = np.flatnonzero(vertical | horizontal)
        ids = ids[np.lexsort((rows[ids, 1], rows[ids, 0], -rows[ids, 2]))]
        # Bound CPU suppression too, even if a noisy mask has tens of
        # thousands of peaks. Keep the best 2048 from EACH orientation.
        ids = ids[(vertical[ids] & (np.cumsum(vertical[ids]) <= 2048)) |
                  (horizontal[ids] & (np.cumsum(horizontal[ids]) <= 2048))]
        rows, normals, horizontal = rows[ids], normals[ids], horizontal[ids]
        self.work_counts['hough_nms_peaks'] = len(rows)
        offsets = rows[:, 1]-radius-normals[:, 0]*((w-1)/2)-normals[:, 1]*((h-1)/2)
        selector = getattr(self, '_native_peak_selector', None)
        if selector is not None:
            indices = selector.select(normals, offsets, horizontal, self.hough_peak_limit, np.cos(np.deg2rad(2)))
            return np.ascontiguousarray(rows[indices])
        # A normal can only suppress normals within 2 degrees, including the
        # 0/180 wrap. Group by angle so each selected peak compares a SMALL
        # neighborhood, retaining the original float64 predicate and order.
        angle_ids = rows[:, 0]
        by_angle = np.argsort(angle_ids, kind='stable')
        starts = np.searchsorted(angle_ids[by_angle], np.arange(361))
        neighbors_cache = {}
        cosine_limit = np.cos(np.deg2rad(2))
        selected, orientation_counts = [], [0, 0]
        suppressed = np.zeros(len(rows), bool)
        for i in range(len(rows)):
            orientation = int(horizontal[i])
            if suppressed[i] or orientation_counts[orientation] >= self.hough_peak_limit//2:
                continue
            selected.append(i)
            orientation_counts[orientation] += 1
            angle = int(angle_ids[i])
            if angle not in neighbors_cache:
                # Five half-degree bins include a numerical guard outside the
                # strict 2 degree test. No suppression decisions are rounded.
                low, high = angle-5, angle+5
                if low < 0:
                    neighbors = np.concatenate((by_angle[starts[360+low]:], by_angle[:starts[high+1]]))
                elif high >= 360:
                    neighbors = np.concatenate((by_angle[starts[low]:], by_angle[:starts[high-359]]))
                else:
                    neighbors = by_angle[starts[low]:starts[high+1]]
                neighbors_cache[angle] = neighbors
            neighbors = neighbors_cache[angle]
            dot = normals[neighbors, 0]*normals[i, 0]+normals[neighbors, 1]*normals[i, 1]
            # Sign-adjust distances at the 0/180 degree wrap. A 2 degree,
            # 4px neighborhood removes nearby peaks from the same thick rod.
            suppressed[neighbors] |= ((np.abs(dot) > cosine_limit) &
                           (np.abs(offsets[neighbors]*np.where(dot < 0, -1, 1)-offsets[i]) < 4))
            if len(selected) >= self.hough_peak_limit:
                break
        return np.ascontiguousarray(rows[selected])

    def fit_segments(self, mask, segments, min_support=.48):
        self._mask(mask)
        if not np.isfinite(min_support) or not 0 <= min_support <= 1:
            raise ValueError('Fit support must be finite and in [0, 1]')
        with self._operation('fit_filter'):
            rt = self.runtime
            segments = np.asarray(segments, np.float32).reshape(-1, 4)
            if not len(segments):
                return []
            if not np.isfinite(segments).all():
                raise ValueError('Fit segments must contain finite coordinates')
            delta = segments[:, 2:]-segments[:, :2]
            lengths = np.linalg.norm(delta.astype(np.float64), axis=1)
            keep = (lengths >= 32) & ((np.abs(delta[:, 0]) < np.abs(delta[:, 1])*.65) |
                                      (np.abs(delta[:, 1]) < np.abs(delta[:, 0])*.7))
            segments, lengths = segments[keep], lengths[keep]
            if not len(segments):
                return []
            h, w = mask.shape
            n = len(segments)
            stride = max(10, int(np.ceil(lengths.max()))+2)
            sortsize = 1 << (max(10, (stride+1)//2)-1).bit_length()
            if stride > 2048:
                raise OpenCLError('OpenCL fitting supports segments up to 2048 samples; use the detection canvas')
            src, seeds = self.search_input('mask', mask, np.uint8), rt.upload('fit_seeds', segments)
            points, widths = rt.buffer('fit_points', n*stride*8), rt.buffer('fit_widths', n*stride*4)
            good = rt.buffer('fit_good', n*stride)
            result, counts = rt.buffer('fit_result', n*9*4), rt.buffer('fit_counts', n*4)
            residuals = rt.buffer('fit_residuals', n*sortsize*4)
            selected_widths = rt.buffer('fit_selected_widths', n*sortsize*4)
            rt.run('fit_sections', n*stride, [src, seeds, points, widths, good, w, h, n, stride])
            rt.run('fit_moments', n, [seeds, points, widths, good, result, residuals, selected_widths,
                                    counts, n, stride, sortsize, float(min_support)])
            rt.run('fit_statistics', n*64, [result, residuals, selected_widths, counts, sortsize,
                                           LocalMemory(sortsize*4), LocalMemory(sortsize*4)], local=64)
            rows = rt.read(result, (n, 9), np.float32)
            lines = []
            for row in rows[rows[:, 8] > 0]:
                d = row[:2].astype(float)
                lines.append(dict(d=d, n=np.array([-d[1], d[0]]), b=float(row[2]),
                    lo=float(row[3]), hi=float(row[4]), width=float(row[5]), support=float(row[6]), vertical=bool(row[7])))
            return lines

    def submit_hough_regions(self, mask, regions):
        """Only Hough owns the GPU until joined; the caller runs CPU LSD."""
        if self._hough_executor is None:
            self._hough_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='quad-hough')
        inputs = []
        for x0, y0, x1, y1 in regions:
            crop = np.ascontiguousarray(mask[y0:y1, x0:x1])
            if getattr(self, '_resident_frame', None) is not None:
                self.register_roi(crop, mask, (x0, y0, x1, y1))
            inputs.append((crop, x0, y0))

        def extract():
            rows = []
            for crop, x0, y0 in inputs:
                found = self.hough_segments(crop)
                if found is not None:
                    rows.extend(found.reshape(-1, 4)+[x0, y0, x0, y0])
            return rows

        return self._hough_executor.submit(extract)

    def submit_lsd_regions(self, image, regions):
        """Independent LSD channels have independent detector instances."""
        if self._lsd_executor is None:
            self._lsd_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix='quad-lsd')
        crops = [(np.ascontiguousarray(image[y0:y1, x0:x1]), x0, y0)
                 for x0, y0, x1, y1 in regions]

        def extract():
            started = time.perf_counter()
            detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
            rows = []
            for crop, x0, y0 in crops:
                found = detector.detect(crop)[0]
                if found is not None:
                    rows.extend(found.reshape(-1, 4)+[x0, y0, x0, y0])
            return rows, (time.perf_counter()-started)*1000

        return self._lsd_executor.submit(extract)

    def close(self):
        if self._hough_executor is not None:
            self._hough_executor.shutdown(wait=True)
        if self._lsd_executor is not None:
            self._lsd_executor.shutdown(wait=True)
        self.runtime.close()
