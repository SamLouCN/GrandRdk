"""Sequential CV timing and bounded reports; no image/geometry changes."""
from collections import deque
import time

import numpy as np


class CvFrameProfile:
    """Non-overlapping checkpoints: stage times can be added without double counting."""

    def __init__(self, enabled=False):
        self.enabled = enabled
        self.stages = {}
        self.counts = {}
        self.meta = {}
        if enabled:
            self.started = self.last_wall = time.perf_counter()
            self.started_cpu = self.last_cpu = time.thread_time()

    def mark(self, name):
        if not self.enabled:
            return
        wall, cpu = time.perf_counter(), time.thread_time()
        values = self.stages.setdefault(name, [0.0, 0.0])
        values[0] += (wall-self.last_wall)*1000
        values[1] += (cpu-self.last_cpu)*1000
        self.last_wall, self.last_cpu = wall, cpu

    def count(self, name, value=1):
        if self.enabled:
            self.counts[name] = self.counts.get(name, 0)+int(value)

    def finish(self):
        if not self.enabled:
            return None
        self.mark('tracker.finalize')
        return dict(total_ms=round((self.last_wall-self.started)*1000, 3),
                    thread_cpu_ms=round((self.last_cpu-self.started_cpu)*1000, 3),
                    stages_ms={name: round(values[0], 3) for name, values in self.stages.items()},
                    stages_thread_cpu_ms={name: round(values[1], 3) for name, values in self.stages.items()},
                    counts=dict(self.counts), **self.meta)


class CvProfileReporter:
    """Keep the worst frame and distributions per mode, rather than sampling one frame."""

    def __init__(self, interval_s=2.0, slow_ms=100.0):
        self.interval_s = interval_s
        self.slow_ms = slow_ms
        self.samples = deque(maxlen=256)
        self.worst = None
        self.last_report = None
        self.last_slow = None

    @staticmethod
    def details(profile):
        top = sorted(profile['stages_ms'], key=profile['stages_ms'].get, reverse=True)[:6]
        return dict(frame=profile['frame'], mode=profile['mode'],
                    search_reason=profile.get('search_reason'), total_ms=profile['total_ms'],
                    thread_cpu_ms=profile['thread_cpu_ms'],
                    top_ms={name: profile['stages_ms'][name] for name in top},
                    top_thread_cpu_ms={name: profile['stages_thread_cpu_ms'][name] for name in top},
                    counts=profile['counts'], roi_fraction=profile.get('roi_fraction'),
                    cv_backend=profile.get('cv_backend', 'cpu'),
                    cv_quality=profile.get('cv_quality', 'precise'),
                    search_scope=profile.get('search_scope'), lsd_policy=profile.get('lsd_policy'),
                    band_fallback_reason=profile.get('band_fallback_reason'),
                    color_processing_size=profile.get('color_processing_size'),
                    line_extract_parallel=profile.get('line_extract_parallel', False),
                    search_execution=profile.get('search_execution'),
                    model_execution=profile.get('model_execution'),
                    partial_execution=profile.get('partial_execution'),
                    line_extraction_size=profile.get('line_extraction_size'),
                    input_size=profile.get('input_size'), canvas_size=profile.get('canvas_size'),
                    opencv_threads=profile.get('opencv_threads'))

    def add(self, profile, frame_id, now=None):
        if profile is None:
            return []
        now = time.monotonic() if now is None else now
        sample = dict(profile, frame=int(frame_id))
        self.samples.append(sample)
        if self.worst is None or sample['total_ms'] > self.worst['total_ms']:
            self.worst = sample
        messages = []
        if sample['total_ms'] >= self.slow_ms and (self.last_slow is None or now-self.last_slow >= 1):
            messages.append(dict(event='slow', **self.details(sample)))
            self.last_slow = now
        if self.last_report is None:
            self.last_report = now
        elif now-self.last_report >= self.interval_s:
            modes = {}
            for mode in sorted({s['mode'] for s in self.samples}):
                values = [s['total_ms'] for s in self.samples if s['mode'] == mode]
                modes[mode] = dict(n=len(values), mean_ms=round(float(np.mean(values)), 2),
                                   p95_ms=round(float(np.percentile(values, 95)), 2),
                                   max_ms=round(max(values), 2))
            messages.append(dict(event='window', modes=modes, worst=self.details(self.worst)))
            self.samples.clear()
            self.worst = None
            self.last_report = now
        return messages
