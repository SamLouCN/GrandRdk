"""Resident GPU CV core: device images -> device state -> one final output join.

This is a new detector, not a numerically identical GPU port of OpenCV LSD.
Only external BGR/YOLO inputs and final target records cross the host boundary.
The host assembles output dictionaries, submits kernels and records timings.
"""
import time
from contextlib import contextmanager

import numpy as np

from .opencl_runtime import LocalMemory


class ResidentGatePipeline:
    width, height = 640, 360
    peak_limit, merge_capacity, maxruns = 256, 128, 22
    model_count = 28*28 + 16

    def __init__(self, backend, fps=30., detect_every=3, hold_seconds=.2, padding=.08):
        if not np.isfinite(fps) or fps <= 0 or detect_every < 1 or not 0 <= hold_seconds <= 1:
            raise ValueError('Invalid resident tracking settings')
        self.backend = backend
        self.rt = backend.runtime
        self.interval = int(detect_every)
        self.hold = int(round(fps*hold_seconds))
        self.padding = float(padding)
        self.index = 0
        # Tracker-local names prevent two streams from sharing temporal state.
        self.prefix = 'resident_cv_'+str(id(self))+'_'
        self.size = None
        self.valid_cache = None
        self.last_status = {}
        self.camera = None
        self.last_guidance = None
        self.previous = None
        self.reset()

    def buffer(self, name, size):
        return self.rt.buffer(self.prefix+name, size)

    def floats(self, name, count, clear=False):
        buf = self.buffer(name, count*4)
        if clear:
            self.rt.run('rg_clear', count, [buf, count])
        return buf

    @contextmanager
    def stage(self, name):
        previous = getattr(self.rt, 'current_stage', None)
        self.rt.current_stage = name
        try:
            yield
        finally:
            self.rt.current_stage = previous

    def reset(self):
        self.state = self.rt.upload(self.prefix+'state', np.zeros(128, np.float32))
        self.previous = None
        self.size = None

    def set_camera(self, matrix):
        matrix = np.asarray(matrix, np.float32)
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError('Resident pose camera must be a finite 3x3 matrix')
        self.camera = self.rt.upload(self.prefix+'camera', matrix)

    def _cc(self, source, tag, minimum, extent, control, both=False):
        w, h = self.width, self.height
        n = w*h
        parents = self.buffer(tag+'_parents', n*4)
        stats = self.buffer(tag+'_stats', n*5*4)
        out = self.buffer(tag+'_filtered', n)
        # Exact worst-case capacity, including odd widths and checkerboards.
        capacity = (w+1)//2
        runs = self.buffer(tag+'_runs', capacity*h*8)
        counts = self.buffer(tag+'_row_counts', h*4)
        links = self.buffer(tag+'_run_links', capacity*h*4)
        self.rt.run('rg_cc_runs', h*64, [source, runs, counts, links, parents, stats,
                    w, h, control, LocalMemory(64*4)], local=64)
        slots = ((capacity*h+63)//64)*64
        self.rt.run('rg_cc_run_link', slots, [runs, counts, links, capacity, h, control], local=64)
        self.rt.run('rg_cc_run_stats', slots, [runs, counts, links, stats, capacity, w, h, control], local=64)
        self.rt.run('rg_cc_run_filter', n, [runs, links, parents, stats, out, w, h, capacity,
                    minimum, extent, int(both), control])
        return out

    def _tube(self, bgr, tag):
        rt, backend = self.rt, self.backend
        n = self.width*self.height
        if backend.hsv_tables is None:
            values = np.arange(1, 256, dtype=float)
            backend.hsv_tables = (
                rt.upload('hsv_sdiv', np.r_[0, np.rint((255 << 12)/values)].astype(np.int32)),
                rt.upload('hsv_hdiv', np.r_[0, np.rint((180 << 12)/(6*values))].astype(np.int32)))
        hsv, chroma = self.buffer(tag+'_hsv', n*3), self.buffer(tag+'_chroma', n*4)
        rt.run('bgr_color', n, [bgr, hsv, chroma, backend.lab_a, *backend.hsv_tables, n])
        local = self.buffer(tag+'_local', n*4)
        for i, sigma in enumerate((3, 9, 18)):
            backend._local_blur(chroma, local, self.width, self.height, sigma, i == 0, self.prefix+tag)
        mask, score = self.buffer(tag+'_mask', n), self.buffer(tag+'_score', n*4)
        rt.run('tube_threshold', n, [hsv, chroma, local, mask, score, n])
        mask = backend._morph(mask, self.width, self.height, [True, False], self.prefix+tag+'_close', binary=True)
        return mask, hsv, chroma

    def _motion(self, gray, mask, control):
        rt = self.rt
        current = [gray]
        old = [self.buffer('gray_'+str(1-self.index%2)+'_0', self.width*self.height)]
        for level in range(1, 4):
            w, h = self.width >> (level-1), self.height >> (level-1)
            size = (w//2)*(h//2)
            current.append(self.buffer('gray_'+str(self.index%2)+'_'+str(level), size))
            old.append(self.buffer('gray_'+str(1-self.index%2)+'_'+str(level), size))
            rt.run('rg_gray_down', size, [current[level-1], current[level], w, h])
        tiles = 40*23
        corners = self.floats('corners', tiles*4)
        features, moved = self.floats('features', 64*4, True), self.floats('moved', 64*4, True)
        rt.run('rg_corners', tiles*64, [old[0], self.state, control, corners,
                                      LocalMemory(64*4), LocalMemory(64*4)], local=64)
        rt.run('rg_feature_rank', tiles, [corners, features, tiles])
        rt.run('rg_lk', 64*64, [*old, *current, features, moved, LocalMemory(64*6*4)], local=64)
        hypotheses = self.floats('hypotheses', 256*8, True)
        rt.run('rg_ransac', 256*64, [features, moved, hypotheses, LocalMemory(64*2*4)], local=64)
        prediction = self.floats('prediction', 128, True)
        params = self.floats('motion_sides', 4*12, True)
        rt.run('rg_motion', 64, [hypotheses, features, moved, self.state, prediction,
                               params, control, 256, LocalMemory(64*4), LocalMemory(64*4)], local=64)
        occupancy, support = self.floats('motion_occupancy', 4), self.floats('motion_support', 4)
        rt.run('track_occupancy', 4*64, [mask, params, occupancy, self.width, self.height, 4, LocalMemory(64*4)], local=64)
        rt.run('search_side_support', 4*64, [mask, params, support, self.width, self.height, 4, LocalMemory(64*8*4)], local=64)
        rt.run('rg_track_validate', 1, [prediction, occupancy, support, control])
        return prediction, current, old

    def _lines(self, mask, strict, signal, control):
        rt, n = self.rt, self.width*self.height
        radius = int(np.ceil(np.hypot(self.width-1, self.height-1)))+1
        nrhos = radius*2+1
        points, metadata = self.buffer('points', n*8), self.floats('metadata', 2, True)
        rt.run('compact_foreground_fast', n, [mask, points, metadata, self.width, self.height])
        votes = self.buffer('votes', 360*nrhos*4)
        rt.run('rg_hough_vote', 360*64, [points, metadata, self.backend.fast_angles,
               votes, control, nrhos, radius, LocalMemory(nrhos*4)], local=64)
        peaks, selected = self.buffer('peaks', 2880*16), self.buffer('selected_peaks', self.peak_limit*16)
        self._peak_top(votes, peaks, control, nrhos, 18)
        self._select_peaks(peaks, selected, control, radius)
        stride = int(np.ceil(np.hypot(self.width-1, self.height-1)/2))+2
        seeds = self.floats('segments', self.peak_limit*self.maxruns*4)
        counts = self.buffer('run_counts', self.peak_limit*4)
        rt.run('rg_runs', self.peak_limit*64, [mask, selected, self.backend.fast_angles,
               seeds, counts, control, self.peak_limit, radius, self.maxruns, stride, LocalMemory(stride*4)], local=64)
        total = self.peak_limit*self.maxruns
        fitted = self.floats('fitted', total*12, True)
        rt.run('rg_fit', total*64, [mask, seeds, counts, fitted, control,
               self.maxruns, total, LocalMemory((512+64*6)*4)], local=64)
        ranked = self.floats('ranked', self.merge_capacity*12, True)
        self._rank_lines(fitted, ranked, total, self.merge_capacity, control)
        merged = self.floats('merged', self.merge_capacity*12, True)
        rt.run('rg_merge', 64, [ranked, merged, control, self.merge_capacity, LocalMemory(64*4)], local=64)
        full, partial = (self.floats(name, self.merge_capacity*12, True) for name in ('full', 'partial'))
        rt.run('rg_line_validate', self.merge_capacity*64,
               [merged, mask, strict, signal, full, partial, self.merge_capacity,
                LocalMemory(128*4), LocalMemory(64*5*4)], local=64)
        full_best, partial_best = (self.floats(name, 16*12, True) for name in ('full_best', 'partial_best'))
        for src, out in ((full, full_best), (partial, partial_best)):
            self._rank_lines(src, out, self.merge_capacity, 16, control, per_orientation=True)
        return full_best, partial_best

    def _peak_top(self, votes, peaks, control, nrhos, threshold):
        self.rt.run('rg_peak_top_tiled', 360*64, [votes, peaks, control, self.backend.fast_angles,
                    nrhos, threshold, LocalMemory(1024*4), LocalMemory(1024*4)], local=64)

    def _select_peaks(self, peaks, selected, control, radius):
        rt = self.rt
        order = self.buffer('peak_order', 2880*4)
        neighbors = self.buffer('peak_masks', 2880*16)
        rt.run('rg_peak_prepare', 2880, [peaks, self.backend.fast_angles, order,
               neighbors, control, radius])
        ids = [self.buffer('peak_sort_ids_'+str(i), 4096*4) for i in range(2)]
        rt.run('rg_peak_sort_tiles', 12*64, [peaks, ids[0], control,
               LocalMemory(256*4), LocalMemory(256*4), LocalMemory(256*4)], local=64)
        lists, span, current = 12, 256, 0
        while lists > 1:
            count, other = (lists+1)//2, 1-current
            rt.run('rg_peak_sort_merge', count*span*2, [peaks, ids[current], ids[other], lists, span])
            lists, span, current = count, span*2, other
        rt.run('rg_peak_sort_order', 2880, [ids[current], order])
        rt.run('rg_peak_select', 1, [peaks, self.backend.fast_angles, order,
               neighbors, selected, control, self.peak_limit, LocalMemory(90*4)], local=1)

    def _rank_lines(self, lines, ranked, n, capacity, control, per_orientation=False):
        """Exact stable TopK; all scores, compacted IDs and counts stay on device."""
        rt = self.rt
        if per_orientation:
            rt.run('rg_rank', 64, [lines, ranked, n, capacity, 1, LocalMemory(n*4), control], local=64)
            return
        if not 1 <= capacity <= 256:
            raise ValueError('Resident tiled TopK capacity must be in [1, 256]')
        lists = max(1, (n+255)//256)
        scores = [self.floats('rank_scores_'+str(i), lists*capacity) for i in range(2)]
        ids = [self.buffer('rank_ids_'+str(i), lists*capacity*4) for i in range(2)]
        rt.run('rg_rank_tiles', lists*64, [lines, scores[0], ids[0], n, capacity,
               LocalMemory(256*4), LocalMemory(256*4), control], local=64)
        current = 0
        while lists > 1:
            count = (lists+1)//2
            other = 1-current
            rt.run('rg_rank_merge', count*capacity, [scores[current], ids[current],
                   scores[other], ids[other], lists, capacity])
            lists, current = count, other
        rt.run('rg_rank_gather', capacity, [lines, ids[current], ranked, capacity])

    def _models(self, full, partial, mask, white, bgr, valid, control, prediction):
        rt, n = self.rt, 28*28
        pairs = self.buffer('pairs', n*16)
        if not getattr(self, '_pairs_ready', False):
            rt.run('rg_pairs', n, [pairs, n]); self._pairs_ready = True
        status, corners = self.buffer('quad_status', n*4), self.floats('quad_corners', n*8)
        params, order = self.floats('quad_sides', n*4*12, True), self.buffer('quad_order', n*16)
        rt.run('search_quads', n, [full, pairs, valid, status, corners, params, order,
                                 n, self.width, self.height, 1, 0, 0, self.width, self.height])
        support = self.floats('quad_support', n*4)
        rt.run('search_side_support', n*4*64, [mask, params, support,
                                             self.width, self.height, n*4, LocalMemory(64*8*4)], local=64)
        models = self.floats('models', self.model_count*128, True)
        rt.run('rg_complete', n, [status, corners, support, order, full, control, models, n])
        color = self.floats('partial_color', 16*3)
        rt.run('search_partial_color', 16*64, [bgr, *self.backend.hsv_tables, partial,
               color, self.width, self.height, 16, LocalMemory(128*4), LocalMemory(64*12)], local=64)
        joints = self.floats('joints', 64*3)
        rt.run('rg_joints', 64, [partial, white, joints])
        rt.run('rg_partial', 16, [partial, joints, color, control, models, n])
        rt.run('rg_select', 64, [models, prediction, self.state, control,
               self.model_count, LocalMemory(64*4), LocalMemory(64*4)], local=64)

    @staticmethod
    def candidate(row):
        if not row[0]:
            return None
        count = int(row[2])
        lines, segments = [], []
        for k in range(count):
            p = row[16+k*12:28+k*12].astype(float)
            d = p[:2].copy()
            normal = np.array([-d[1], d[0]])
            segment = row[64+k*4:68+k*4].reshape(2, 2).astype(float)
            # Output formatting only: all measured values were computed on GPU.
            measured = normal*p[2]+np.array([p[3], p[4]])[:, None]*d
            lines.append(dict(d=d, n=normal, b=float(p[2]),
                lo=float(p[3]), hi=float(p[4]), width=float(p[5]), support=float(p[6]),
                vertical=bool(p[7]), strong_lo=float(p[8]), strong_hi=float(p[9]),
                observed_segment=measured))
            segments.append(segment)
        corners = row[8:8+int(row[7])*2].reshape(-1, 2).astype(float)
        return dict(lines=lines, segments=segments, corners=list(corners),
                    quad=corners if row[3] else None, complete=bool(row[3]),
                    score=float(row[4]), width=float(row[5]), apparent_width=float(row[5]),
                    tracked=bool(row[6]), geometry_validated=bool(row[3]),
                    model='resident-four-line' if row[3] else 'resident-endpoint-chain',
                    side_support=row[80:80+count].astype(float).tolist())

    def update(self, reference, enhanced, boxes, target_ids, valid_mask=None):
        started = time.perf_counter()
        started_cpu = time.thread_time()
        backend, rt = self.backend, self.rt
        backend._resident_core_active = True
        stage_before = dict(getattr(rt, 'stage_kernel_ms', {}))
        h, w = reference.shape[:2]
        if reference.shape != enhanced.shape:
            raise ValueError('Resident CV reference and enhanced shapes must match')
        if self.size is not None and self.size != (w, h):
            self.reset()
        self.size = w, h
        ratio = min(self.width/w, self.height/h)
        rw, rh = round(w*ratio), round(h*ratio)
        ox, oy = (self.width-rw)//2, (self.height-rh)//2
        # Detector records are external input. Filtering class IDs is routing.
        inputs = [box for box in boxes if box.get('class_id') in target_ids]
        rows = np.asarray([[*box['bbox'], box['score'], box['class_id']] for box in inputs], np.float32).reshape(-1, 6)
        src = backend.device_input('resident_cv_reference', reference)
        extra = backend.device_input('resident_cv_enhanced', enhanced)
        valid = src
        if valid_mask is not None:
            if valid_mask.shape != (h, w):
                raise ValueError('Resident CV valid mask must match input')
            if self.valid_cache is None or self.valid_cache[0] is not valid_mask:
                self.valid_cache = valid_mask, rt.upload(self.prefix+'input_valid', np.uint8(valid_mask))
            valid = self.valid_cache[1]
        boxes_buf = rt.upload(self.prefix+'yolo_inputs', rows)
        control = self.floats('control', 32)
        with self.stage('input_and_target'):
            rt.run('rg_target', 1, [boxes_buf, valid, self.state, control, len(rows), w, h,
                   int(valid_mask is not None), self.index, self.interval, self.hold,
                   self.padding, ratio, ox, oy])
        n = self.width*self.height
        bgr, gray, canvas_valid = self.buffer('bgr', n*3), self.buffer('gray_'+str(self.index%2)+'_0', n), self.buffer('valid', n)
        with self.stage('input_and_target'):
            rt.run('rg_canvas', n, [src, extra, valid, bgr, gray, canvas_valid, w, h, rw, rh, ox, oy, int(valid_mask is not None)])
        with self.stage('color_evidence'):
            color, hsv, _ = self._tube(bgr, 'color')
            signal = self.floats('signal', n)
            rt.run('log_signal', n, [bgr, signal, n])
            blur = backend._blur(signal, self.width, self.height, 9, self.prefix+'strict')
            mask, strict, white = (self.buffer(name, n) for name in ('mask', 'strict', 'white'))
            rt.run('rg_masks', n, [color, hsv, signal, blur, canvas_valid, control, mask, strict, white])
            strict = backend._morph(strict, self.width, self.height, [True, False, False, True], self.prefix+'strict_morph', binary=True)
        with self.stage('motion'):
            prediction, _, _ = self._motion(gray, mask, control)
        with self.stage('components'):
            strict = self._cc(strict, 'strict_cc', 35, 25, control)
            opened = self.buffer('white_opened', n)
            rt.run('rg_white_open2', n, [white, opened, control])
            white = self._cc(opened, 'white_cc', 18, 3, control, True)
        with self.stage('line_extraction'):
            full, partial = self._lines(mask, strict, signal, control)
        with self.stage('models'):
            self._models(full, partial, mask, white, bgr, canvas_valid, control, prediction)
        pose = self.floats('pose', 40)
        with self.stage('guidance'):
            rt.run('rg_pose', 1, [self.state, self.camera or self.state, pose,
                                 ratio, ox, oy, int(self.camera is not None)])
        # The only CV-core read: final selected geometry + final routing status.
        before_read = time.perf_counter()
        state, result, pose_result = rt.read_many([(self.state, (128,), np.float32),
                                                  (control, (32,), np.float32), (pose, (40,), np.float32)])
        read_finished = time.perf_counter()
        rt.finish()
        finished = time.perf_counter()
        selected = self.candidate(state)
        self.previous = selected
        target = state[96:100].astype(float).tolist() if state[100] else None
        mode = 'search' if result[7] else 'track' if result[18] else 'idle'
        proposal = int(result[5])
        bits = int(state[102])
        boundaries = [side for bit, side in ((1, 'left'), (2, 'right'), (4, 'up'), (8, 'down')) if bits & bit]
        reason = ('clipped-target-continuity' if result[3] == 1 else 'clipped-target-gap' if result[3] == 2 else
                  'largest-yolo-area' if proposal >= 0 else 'yolo-gap')
        profile = dict(total_ms=round((finished-started)*1000, 3),
                       thread_cpu_ms=round((time.thread_time()-started_cpu)*1000, 3),
                       stages_ms={k: round(v-stage_before.get(k, 0.), 3) for k, v in rt.stage_kernel_ms.items()},
                       stages_thread_cpu_ms={}, counts={}, mode=mode,
                       search_reason='device-scheduled-or-tracking-failed' if result[7] else 'interval_tracking',
                       input_size=[w, h], canvas_size=[self.width, self.height],
                       cv_backend=rt.info['selected'], cv_quality='resident',
                       cv_execution='resident', timing_kind='device-events' if rt.info['device_type']=='gpu' else 'host-kernel-emulation',
                       final_join_ms=round((read_finished-before_read)*1000, 3),
                       event_collection_ms=round((finished-read_finished)*1000, 3),
                       cpu_compute_stages=[], intermediate_readbacks=0,
                       candidate_generator='bounded-color-hough', lsd_policy='replaced',
                       candidate_capacity=dict(peaks=self.peak_limit, ranked_runs=self.merge_capacity, lines=16),
                       model_execution='gpu-resident', partial_execution='gpu-resident')
        self.last_status = dict(frame=self.index, candidate_count=int(result[24]) if result[7] else None,
            detection_ran=bool(result[7]), observation='tracked' if selected and selected['tracked'] else 'detected' if selected else 'missing',
            age_frames=self.index-int(state[1]) if selected else None,
            ratio=ratio, offset=[ox, oy], yolo_count=len(inputs), yolo_detections=inputs,
            target_bbox=target, target_score=float(state[104]) if target else None,
            target_clipped=bool(bits) if target else False, target_boundary_sides=boundaries if target else [],
            search_bbox=result[20:24].astype(float).tolist() if target else None,
            selection_reason=reason, target_switched=bool(result[2]),
            yolo_age_frames=self.index-int(state[101]) if target else None,
            cv_enabled=bool(result[0]), cv_profile=profile)
        self.last_guidance = self.guidance(self.last_status, pose_result)
        self.index += 1
        return selected, ([], [])

    @staticmethod
    def guidance(status, pose):
        from .gate_guidance import AXES
        result = dict(mode='no-target', reason='no-current-yolo-target', ui_only=True,
                      coordinate_space='corrected', missing_sides=[], direction_xy=None,
                      completion=None, alignment=None, axes=AXES,
                      geometry_observation=status['observation'],
                      execution='gpu-resident', pose_algorithm='planar-homography')
        if status['target_bbox'] is None:
            return result
        if status['yolo_age_frames'] != 0:
            return dict(result, mode='yolo-gap', reason='wait-for-current-yolo')
        if status['target_clipped']:
            sides = status['target_boundary_sides']
            dx = int('right' in sides)-int('left' in sides)
            dy = int('down' in sides)-int('up' in sides)
            scale = .7071067811865476 if dx and dy else 1
            return dict(result, mode='clipped', reason='target-touches-valid-image-boundary',
                        missing_sides=sides, direction_xy=[dx*scale, dy*scale] if dx or dy else None)
        if not pose[0]:
            return dict(result, mode='pose-unavailable', reason='resident-pose-not-validated')
        alignment = dict(rotation_xyz_deg=pose[24:27].astype(float).tolist(),
            rotation_matrix=pose[3:12].reshape(3, 3).astype(float).tolist(),
            gate_center_model_units=pose[12:15].astype(float).tolist(),
            gate_normal_camera=pose[15:18].astype(float).tolist(),
            alignment_translation_model_units=pose[18:21].astype(float).tolist(),
            translation_pixel_equivalent_xyz=pose[21:24].astype(float).tolist(),
            translation_scaled_xyz=pose[21:24].astype(float).tolist(),
            translation_scale=1., translation_unit='scaled pixel equivalent',
            gate_center_px=pose[27:29].astype(float).tolist(), center_offset_xy_px=pose[29:31].astype(float).tolist(),
            normal_distance_model_units=float(pose[2]), reprojection_rms_px=float(pose[1]),
            gate_size_reference=[.7, .5], axes=AXES, estimated=True, metric_distance_available=False,
            algorithm='planar-homography', ambiguity_test='not-IPPE')
        completion = dict(corners=pose[32:40].reshape(4, 2).astype(float).tolist(), segments=[], observation='measured')
        return dict(result, mode='align', reason='pose-estimated', alignment=alignment, completion=completion)
