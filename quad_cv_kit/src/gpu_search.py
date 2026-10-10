"""Batched search stages; device intermediates never become per-probe Python arrays.

The CPU implementation remains the reference. Only accepted lines/models are
materialized as dictionaries, for the existing tracking and selection API.
"""
from contextlib import contextmanager

import numpy as np

from .opencl_runtime import LocalMemory, OpenCLError


def line_rows(lines):
    return np.asarray([[*line['d'], line['b'], line['lo'], line['hi'],
                        line['width'], line['support'], line['vertical'],
                        line.get('strong_lo', line['lo']),
                        line.get('strong_hi', line['hi']), 0, 0]
                       for line in lines], np.float32).reshape(-1, 12)


class GPUSearchMixin:
    @contextmanager
    def search_batch(self):
        """Reuse explicitly read-only search inputs only within this call.

        No identity cache survives a call: callers can mutate/reuse NumPy frame
        arrays on the next frame. Nested band/full searches have separate inputs.
        """
        previous = getattr(self, '_search_inputs', None)
        self._search_inputs = {}
        try:
            yield
        finally:
            if previous is not None:
                previous.clear()  # Nested searches may overwrite named buffers.
            self._search_inputs = previous

    def search_input(self, name, array, dtype):
        resident = self.resident_buffer(array)
        compatible = array.dtype == np.dtype(dtype) or (array.dtype == np.bool_ and np.dtype(dtype) == np.uint8)
        if resident is not None and compatible:
            return resident
        cache = getattr(self, '_search_inputs', None)
        if cache is not None:
            old = cache.get(name)
            if old is not None and old[0] is array and old[1] == np.dtype(dtype):
                return old[2]
        buf = self.device_input('search_'+name, np.asarray(array, dtype=dtype))
        if cache is not None:
            cache[name] = (array, np.dtype(dtype), buf)
        return buf

    def partial_measurements(self, lines, nv, frame):
        """Batch partial-line joints and color probes; graph ownership stays CPU."""
        if not lines:
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.float32)
        rt = self.runtime
        h, w = frame.shape[:2]
        with self._operation('partial_measurements'):
            seeds = rt.upload('partial_lines', line_rows(lines))
            source = self.search_input('contrast_frame_1', frame, np.uint8)
            if self.hsv_tables is None:
                values = np.arange(1, 256, dtype=float)
                self.hsv_tables = (rt.upload('hsv_sdiv', np.r_[0, np.rint((255 << 12)/values)].astype(np.int32)),
                                   rt.upload('hsv_hdiv', np.r_[0, np.rint((180 << 12)/(6*values))].astype(np.int32)))
            measurements = rt.buffer('partial_color', len(lines)*3*4)
            rt.run('search_partial_color', len(lines)*64,
                   [source, *self.hsv_tables, seeds, measurements, w, h, len(lines),
                    LocalMemory(128*4), LocalMemory(64*12)], local=64)
            nh = len(lines)-nv
            pairs = rt.buffer('partial_joints', max(1, nv*nh)*3*4)
            if nv and nh:
                rt.run('search_partial_joints', nv*nh, [seeds, pairs, nv, nh])
            color, joints = rt.read_many([(measurements, (len(lines), 3), np.float32),
                                          (pairs, (nv*nh, 3), np.float32)])
            return color, joints

    def track_support(self, mask, lines, segments, validate_sides, sample_widths=None):
        """One host join for all moved rods instead of per-rod CPU sampling."""
        n = len(lines)
        if not n:
            return np.empty(0, np.float32), np.empty(0, np.float32)
        widths = [line['width'] for line in lines] if sample_widths is None else sample_widths
        params = np.float32([[*np.asarray(segment).reshape(-1), *line['d'], line['width'],
                             line['lo'], line['hi'], line.get('strong_lo', line['lo']),
                             line.get('strong_hi', line['hi']), sample_width]
                            for line, segment, sample_width in zip(lines, segments, widths)])
        rt = self.runtime
        h, w = mask.shape
        with self._operation('track_support'):
            seeds = rt.upload('track_side_params', params)
            src = self.search_input('mask', mask, np.uint8)
            occupancy = rt.buffer('track_occupancy', n*4)
            rt.run('track_occupancy', n*64, [src, seeds, occupancy, w, h, n, LocalMemory(64*4)], local=64)
            if not validate_sides:
                return rt.read(occupancy, (n,), np.float32), np.full(n, -1, np.float32)
            support = rt.buffer('track_side_support', n*4)
            rt.run('search_side_support', n*64, [src, seeds, support, w, h, n, LocalMemory(64*8*4)], local=64)
            return rt.read_many([(occupancy, (n,), np.float32), (support, (n,), np.float32)])

    def merge_split_lines(self, lines, mask, frame, reference, profile):
        """Greedy merge order is retained; samples and run medians are batched."""
        if not lines:
            return [], []
        rt = self.runtime
        h, w = mask.shape
        step = 2 if self.quality == 'fast' else 1
        minimum = 18 if self.quality == 'fast' else 35
        rows = line_rows(lines)
        # Bounds follow actual endpoint length, not the fitted projected span.
        spans = rows[:, 4].astype(float)-rows[:, 3]
        lengths = spans*np.linalg.norm(rows[:, :2].astype(float), axis=1)
        directions = rows[:, :2].astype(float)
        normals = np.column_stack((-directions[:, 1], directions[:, 0]))
        a = normals*rows[:, 2, None]+directions*rows[:, 3, None]
        b = normals*rows[:, 2, None]+directions*rows[:, 4, None]
        extent = np.ptp(np.concatenate((a, b)), axis=0)
        bound = np.linalg.norm(extent)*max(1., np.linalg.norm(directions, axis=1).max()**2)
        stride = max(20, int(np.ceil(max(bound, lengths.max(), spans.max())/step))+3)
        if stride > 2048:
            raise OpenCLError('GPU search supports up to 2048 samples per line')
        with self._operation('merge_split_contrast'):
            seeds = rt.upload('search_line_rows', rows)
            merged = rt.buffer('search_merged', len(lines)*12*4)
            stats = rt.buffer('search_merge_stats', 8)
            rt.run('search_merge', 64, [seeds, merged, stats, len(lines), step,
                                      LocalMemory(64*4)], local=64)
            # Capacity follows the known seed count. Unused merged records are
            # zeroed on device, so probe/morph/run packing need no host count.
            n = len(lines)
            profile.mark('lines.merge')
            mask_buf = self.search_input('mask', mask, np.uint8)
            hits = rt.buffer('search_hits', n*stride)
            expanded = rt.buffer('search_expanded', n*stride)
            closed = rt.buffer('search_closed', n*stride)
            rt.run('search_probe', n*stride, [mask_buf, merged, hits, w, h, stride, n])
            radius = 1 if step == 2 else 3
            rt.run('trim_morph', n*stride, [hits, expanded, merged, stride, n, 1, radius])
            rt.run('trim_morph', n*stride, [expanded, closed, merged, stride, n, 0, radius])
            maxruns = stride//minimum+1
            run_params = rt.buffer('search_run_params', n*maxruns*12*4)
            run_info = rt.buffer('search_run_info', n*maxruns*3*4)
            run_count = rt.buffer('search_run_count', 4)
            # One work item packs runs deterministically. Pixel sampling and
            # morphology have already run in parallel; no atomics/order drift.
            rt.run('search_runs', 1, [merged, hits, closed, run_params, run_info,
                                    run_count, stride, n, minimum, maxruns, step])
            stats_host, count_host = rt.read_many([(stats, (2,), np.int32),
                                                   (run_count, (1,), np.int32)])
            merged_count, comparisons = map(int, stats_host)
            count = int(count_host[0])
            profile.mark('lines.split')
            profile.count('merge_comparisons', comparisons)
            profile.count('merged_lines', merged_count)
            profile.count('contrast_runs', count)
            if not count:
                return [], []
            signals = []
            for index, source in enumerate((reference, frame)):
                if index and source is reference:
                    signals.append(signals[0])
                    continue
                src = self.search_input('contrast_frame_'+str(index), source, np.uint8)
                out = rt.buffer('search_signal_'+str(index), h*w*4)
                rt.run('log_signal', w*h, [src, out, w*h])
                signals.append(out)
                profile.count('contrast_full_image_calls')
            # Power-of-two sorting, one work group per run/channel. Invalid
            # samples sort after finite deltas and do not affect the median.
            sortsize = 1 << (stride-1).bit_length()
            measurements = rt.buffer('search_contrast_results', count*2*2*4)
            rt.run('search_contrast', count*2*64,
                   [signals[0], signals[1], run_params, measurements, w, h,
                    count, sortsize, LocalMemory(sortsize*4), LocalMemory(64*8)], local=64)
            info, contrast, params = rt.read_many([
                (run_info, (count, 3), np.float32),
                (measurements, (count, 2, 2), np.float32),
                (run_params, (count, 12), np.float32)])
            profile.mark('lines.contrast_sample')
            which = (contrast[:, 1, 0] > contrast[:, 0, 0]).astype(int)
            evidence = contrast[np.arange(count), which]
            keep = (evidence[:, 0] > .035) & (evidence[:, 1] > .48)
            accepted = []
            for index in np.flatnonzero(keep):
                p, result = params[index], info[index]
                d = p[4:6].astype(float)
                normal = p[6:8].astype(float)
                accepted.append(dict(d=d, n=normal, b=float((p[0]*normal[0]+p[1]*normal[1])/(normal@normal)),
                    lo=float(result[0]), hi=float(result[1]), width=float(p[10]),
                    support=float(result[2]), vertical=bool(abs(d[0]) < abs(d[1])*.65),
                    contrast=float(evidence[index, 0])))
            accepted.sort(key=lambda line: (line['hi']-line['lo'])*line['support'], reverse=True)
            limit = 8 if step == 2 else 10
            profile.count('supported_lines', len(accepted))
            profile.meta['search_execution'] = 'gpu-batched'
            return ([line for line in accepted if line['vertical']][:limit],
                    [line for line in accepted if not line['vertical']][:limit])

    def complete_models_gpu(self, vertical, horizontal, mask, bounds, valid_mask, profile):
        """Geometry, valid-mask probes and color evidence run without a host join."""
        from .gate_models import _measured_model
        nv, nh = len(vertical), len(horizontal)
        if nv < 2 or nh < 2:
            return []
        lines = vertical+horizontal
        vp = np.array(np.triu_indices(nv, 1)).T
        hp = np.array(np.triu_indices(nh, 1)).T+nv
        pairs = np.ascontiguousarray(np.column_stack((np.repeat(vp, len(hp), axis=0),
                                                     np.tile(hp, (len(vp), 1)))), np.int32)
        n = len(pairs)
        h, w = mask.shape
        if valid_mask is not None and valid_mask.shape != mask.shape:
            raise ValueError('Model valid mask must match the detection mask')
        with self._operation('complete_models'):
            rt = self.runtime
            seeds = rt.upload('model_lines', line_rows(lines))
            pair_buf = rt.upload('model_pairs', pairs)
            valid = self.search_input('valid', valid_mask if valid_mask is not None else
                                      np.ones((1, 1), np.uint8), np.uint8)
            status = rt.buffer('model_status', n*4)
            corners = rt.buffer('model_corners', n*8*4)
            side_params = rt.buffer('model_side_params', n*4*12*4)
            order = rt.buffer('model_order', n*4*4)
            rt.run('search_quads', n, [seeds, pair_buf, valid, status, corners, side_params,
                order, n, w, h, int(valid_mask is not None), *map(int, bounds)])
            support = rt.buffer('model_support', n*4*4)
            src = self.search_input('mask', mask, np.uint8)
            rt.run('search_side_support', n*4*64,
                   [src, side_params, support, w, h, n*4, LocalMemory(64*8*4)], local=64)
            # Only compact accepted models; rejected corners/supports stay on
            # device. The CPU sees final model geometry plus diagnostic counts.
            result = rt.buffer('model_result', n*16*4)
            counts = rt.buffer('model_counts', 4*4)
            rt.run('search_accept', 1, [status, corners, support, order, result, counts, n])
            # Read a small bounded prefix together with counts. Typical frames
            # need one join; unusually many accepted models get a second read
            # without dropping any candidates.
            capacity = min(n, 64)
            counts_host, rows = rt.read_many([(counts, (4,), np.int32),
                                              (result, (capacity, 16), np.float32)])
            total, checked, color_checked, sides = map(int, counts_host)
            rows = rt.read(result, (total, 16), np.float32) if total > capacity else rows[:total]
            if profile is not None:
                profile.count('quad_combinations', checked)
                profile.count('quad_color_checks', color_checked)
                profile.count('model_side_checks', sides)
                profile.meta['model_execution'] = 'gpu-batched'
                profile.mark('models.gpu_batch')
            candidates = []
            for row in rows:
                selected = [lines[int(index)] for index in row[12:16]]
                quad = row[:8].reshape(4, 2)
                segments = np.stack((quad, np.roll(quad, -1, axis=0)), axis=1)
                candidates.append(_measured_model(selected, list(segments),
                    [line['width'] for line in selected], list(quad), quad, row[8:12].tolist()))
            return sorted(candidates, key=lambda candidate: candidate['score'], reverse=True)
