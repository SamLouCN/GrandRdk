"""Explicit frame lifetime for device inputs, including CPU read-only consumers.

A downloaded host view is not an instruction to upload the pixels again. Device
copies have stable frame-local slots; scratch buffers may be reused safely.
"""
from contextlib import contextmanager

import cv2
import numpy as np


class GPUResidencyMixin:
    @contextmanager
    def frame_batch(self):
        existing = getattr(self, '_resident_frame', None)
        if existing is not None:
            yield
            return
        self._resident_frame = dict(arrays={}, next_slot=0, stable=set(), color={})
        try:
            yield
        finally:
            try:
                self.runtime.finish()
            finally:
                self._resident_frame = None

    def resident_buffer(self, array):
        state = getattr(self, '_resident_frame', None)
        if state is None:
            return None
        entry = state['arrays'].get(id(array))
        return entry[1] if entry is not None and entry[0] is array else None

    def _resident_slot(self, nbytes, tag=None):
        state = self._resident_frame
        slot = state['next_slot']
        state['next_slot'] += 1
        name = 'resident_'+str(slot)+('' if tag is None else '_'+tag)
        buffer = self.runtime.buffer(name, nbytes)
        state['stable'].add(buffer.handle)
        return buffer

    def device_output(self, name, nbytes):
        """Write final results directly into frame-owned storage."""
        return self._resident_slot(nbytes, name) if getattr(self, '_resident_frame', None) is not None else self.runtime.buffer(name, nbytes)

    def register_resident(self, array, buffer, *, preserve=True):
        if getattr(self, '_resident_frame', None) is None:
            return array
        if preserve and buffer.handle not in self._resident_frame['stable']:
            stable = self._resident_slot(array.nbytes)
            if array.nbytes:
                self.runtime.run('resident_copy', array.nbytes, [buffer, stable, array.nbytes])
            buffer = stable
        self._resident_frame['arrays'][id(array)] = (array, buffer)
        return array

    def device_input(self, name, array):
        buffer = self.resident_buffer(array)
        if buffer is not None:
            return buffer
        if getattr(self, '_resident_frame', None) is not None:
            # Unique slots prevent a nested ROI from overwriting a cached input.
            state = self._resident_frame
            slot = state['next_slot']
            state['next_slot'] += 1
            buffer = self.runtime.upload('resident_'+str(slot)+'_'+name, array)
            self.register_resident(array, buffer, preserve=False)
            return buffer
        return self.runtime.upload(name, array)

    def capture_read(self, buffer, shape, dtype):
        result = self.runtime.read(buffer, shape, dtype)
        return self.register_resident(result, buffer)

    def device_descriptor(self, buffer, shape, dtype):
        """Shape-only reference for an explicitly resident consumer.

        Pixels are not available on this host array. Its read-only broadcast
        storage is never uploaded; device_input resolves the resident buffer.
        The caller must keep frame_batch alive until the consumer finishes.
        """
        if getattr(self, '_resident_frame', None) is None:
            raise ValueError('Device-only references require frame_batch')
        result = np.broadcast_to(np.zeros((), dtype), shape)
        return self.register_resident(result, buffer, preserve=False)

    def hsv_device(self, frame, name, source=None):
        rt = self.runtime
        if self.hsv_tables is None:
            values = np.arange(1, 256, dtype=float)
            self.hsv_tables = (rt.upload('hsv_sdiv', np.r_[0, np.rint((255 << 12)/values)].astype(np.int32)),
                               rt.upload('hsv_hdiv', np.r_[0, np.rint((180 << 12)/(6*values))].astype(np.int32)))
        src = self.device_input(name+'_source', frame) if source is None else source
        out = rt.buffer(name, frame.size)
        rt.run('bgr_hsv', frame.shape[0]*frame.shape[1], [src, out, *self.hsv_tables, frame.shape[0]*frame.shape[1]])
        return out

    def copy_host(self, array):
        result = array.copy()
        source = self.resident_buffer(array)
        if source is not None:
            # Both host arrays are read-only within this frame. Transformations
            # allocate new outputs instead of modifying the shared device data.
            self.register_resident(result, source, preserve=False)
        return result

    def color_device(self, frame, name):
        """Fuse HSV and exact LAB a; cache only read-only frame inputs."""
        state = getattr(self, '_resident_frame', None)
        cached = None if state is None else state['color'].get(id(frame))
        if cached is not None and cached[0] is frame:
            return cached[1:]
        if self.hsv_tables is None:
            values = np.arange(1, 256, dtype=float)
            self.hsv_tables = (self.runtime.upload('hsv_sdiv', np.r_[0, np.rint((255 << 12)/values)].astype(np.int32)),
                               self.runtime.upload('hsv_hdiv', np.r_[0, np.rint((180 << 12)/(6*values))].astype(np.int32)))
        source = self.device_input(name+'_source', frame)
        hsv = self.device_output(name+'_hsv', frame.size)
        chroma = self.device_output(name+'_chroma', frame.shape[0]*frame.shape[1]*4)
        self.runtime.run('bgr_color', frame.shape[0]*frame.shape[1],
                         [source, hsv, chroma, self.lab_a, *self.hsv_tables, frame.shape[0]*frame.shape[1]])
        if state is not None:
            state['color'][id(frame)] = (frame, hsv, chroma)
        return hsv, chroma

    def register_roi(self, result, source, bounds, destination=(0, 0)):
        """Mirror an already constructed CPU ROI using device-to-device pixels."""
        if getattr(self, '_resident_frame', None) is None:
            return result
        if source.dtype != result.dtype or source.shape[2:] != result.shape[2:]:
            raise ValueError('Resident ROI type/channel layout must match')
        x0, y0, x1, y1 = map(int, bounds)
        dx, dy = map(int, destination)
        sh, sw = source.shape[:2]
        dh, dw = result.shape[:2]
        if not (0 <= x0 <= x1 <= sw and 0 <= y0 <= y1 <= sh and
                0 <= dx <= dx+x1-x0 <= dw and 0 <= dy <= dy+y1-y0 <= dh):
            raise ValueError('Resident ROI exceeds its source or destination')
        src = self.device_input('roi_source', source)
        out = self._resident_slot(result.nbytes)
        element_bytes = source.dtype.itemsize*int(np.prod(source.shape[2:]))
        self.runtime.run('resident_roi', dw*dh,
            [src, out, element_bytes, sw, dw, dh, x0, y0, dx, dy, x1-x0, y1-y0])
        return self.register_resident(result, out, preserve=False)

    def restrict_mask(self, mask, valid=None, bounds=None):
        h, w = mask.shape
        x0, y0, x1, y1 = (0, 0, w, h) if bounds is None else tuple(map(int, bounds))
        if valid is not None and valid.shape != mask.shape:
            raise ValueError('Resident valid mask must match the color mask')
        result = np.zeros_like(mask)
        result[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        if valid is not None:
            result[valid == 0] = 0
        if getattr(self, '_resident_frame', None) is not None:
            src = self.device_input('color_mask', mask)
            valid_buf = self.device_input('valid_mask', valid) if valid is not None else src
            out = self._resident_slot(result.nbytes)
            self.runtime.run('resident_restrict', w*h,
                [src, valid_buf, out, w, h, x0, y0, x1, y1, int(valid is not None)])
            self.register_resident(result, out, preserve=False)
        return result

    def mask_and(self, a, b):
        result = cv2.bitwise_and(a, b)
        if getattr(self, '_resident_frame', None) is not None:
            first, second = self.device_input('mask_a', a), self.device_input('mask_b', b)
            out = self._resident_slot(result.nbytes)
            self.runtime.run('resident_and', result.size, [first, second, out, result.size])
            self.register_resident(result, out, preserve=False)
        return result

    def prepare_canvas(self, frame):
        from .detect_red_gate import prepare_detection_frame
        canvas, ratio, offset = prepare_detection_frame(frame)
        src = self.resident_buffer(frame)
        h, w = frame.shape[:2]
        rw, rh = round(w*ratio), round(h*ratio)
        # Common 720p->640x360 and 640x360 inputs have exact integer area rules.
        # Other sizes retain CPU INTER_AREA, then upload the small canvas once.
        scale = w//rw if rw else 0
        if src is not None and scale in (1, 2) and w == rw*scale and h == rh*scale:
            out = self._resident_slot(canvas.nbytes)
            self.runtime.run('resident_canvas', 640*360,
                [src, out, w, rw, rh, int(offset[0]), int(offset[1]), scale])
            self.register_resident(canvas, out, preserve=False)
        return canvas, ratio, offset

    def prepare_valid_canvas(self, valid, width, height, ratio, offset):
        rw, rh = round(width*ratio), round(height*ratio)
        ox, oy = map(int, offset)
        key = (rw, rh, ox, oy)
        cached = getattr(self, '_valid_canvas_cache', None)
        if not valid.flags.writeable and cached is not None and cached[0] is valid and cached[1] == key:
            result, device = cached[2:]
        else:
            result = np.zeros((360, 640), bool)
            result[oy:oy+rh, ox:ox+rw] = cv2.resize(np.uint8(valid), (rw, rh), interpolation=cv2.INTER_NEAREST)>0
            device = None
            if not valid.flags.writeable:
                result.setflags(write=False)
                # A second immutable valid mask can arrive in a nested search.
                # Preserve the first association before reusing this constant
                # buffer; the common single-valid-mask frame needs no copy.
                if cached is not None and self.resident_buffer(cached[2]) is not None:
                    self.register_resident(cached[2], cached[3])
                device = self.runtime.upload('constant_valid_canvas', result)
                self._valid_canvas_cache = (valid, key, result, device)
        if device is not None and self.resident_buffer(result) is None:
            self.register_resident(result, device, preserve=False)
        return result
