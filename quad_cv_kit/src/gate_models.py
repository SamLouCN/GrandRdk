"""Four measured red lines form a gate despite compact white elbow gaps.

Adapted from red_gate_improved/gate_models.py v3. Keep measured red extents
separate from corner-to-corner model edges; no missing side is manufactured.
Partial/foreground selection remains in this project's existing detector.
"""
from itertools import combinations

import cv2
import numpy as np

from .gate_line_geometry import endpoints, intersection, samples


def tube_evidence(frame, profile=None, prefix='color'):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Only the a-channel is used. Keep it contiguous for all three blurs.
    chroma = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[:, :, 1].astype(np.float32)
    if profile is not None:
        profile.mark(prefix+'.convert')
    local = chroma-cv2.GaussianBlur(chroma, (0, 0), 3)
    for sigma in (9, 18):
        np.maximum(local, chroma-cv2.GaussianBlur(chroma, (0, 0), sigma), out=local)
    if profile is not None:
        profile.mark(prefix+'.blur')
    hue, saturation, value = cv2.split(hsv)
    colored = (((hue < 14) | (hue > 118)) & (saturation > 4)) | (
        (saturation < 60) & (chroma > 126) & (local > 3))
    red = colored & (chroma > 125) & (value > 18) & ((local > 1.3) | (chroma > 136))
    mask = cv2.morphologyEx(np.uint8(red)*255, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    score = np.clip((chroma-125)/15, 0, 1)*np.clip(local/4, 0, 1)*colored
    if profile is not None:
        profile.mark(prefix+'.threshold')
    return mask, np.float32(score)


def combined_evidence(enhanced, reference, profile=None, prefix='color', backend=None, bounds=None):
    """Enhanced pixels can strengthen only locally supported reference colors."""
    if bounds is not None:
        # sigma=18: exact support 72px; pyramid interpolation + morphology
        # need additional context. Multiple-of-four origins preserve its grid.
        h, w = reference.shape[:2]
        x0, y0, x1, y1 = map(int, bounds)
        x0, x1 = np.clip([x0, x1], 0, w)
        y0, y1 = np.clip([y0, y1], 0, h)
        mask, score = np.zeros((h, w), np.uint8), np.zeros((h, w), np.float32)
        if x1 <= x0 or y1 <= y0:
            return mask, score
        cx0, cy0 = max(0, (x0-84)//4*4), max(0, (y0-84)//4*4)
        cx1, cy1 = min(w, (x1+87)//4*4), min(h, (y1+87)//4*4)
        clean = np.ascontiguousarray(reference[cy0:cy1, cx0:cx1])
        extra = clean if enhanced is reference else np.ascontiguousarray(enhanced[cy0:cy1, cx0:cx1])
        if backend is not None and hasattr(backend, 'register_roi'):
            backend.register_roi(clean, reference, (cx0, cy0, cx1, cy1))
            if extra is not clean:
                backend.register_roi(extra, enhanced, (cx0, cy0, cx1, cy1))
        local_mask, local_score = combined_evidence(extra, clean, profile, prefix, backend)
        ys, xs = slice(y0-cy0, y1-cy0), slice(x0-cx0, x1-cx0)
        mask[y0:y1, x0:x1] = local_mask[ys, xs]
        score[y0:y1, x0:x1] = local_score[ys, xs]
        if backend is not None and hasattr(backend, 'register_roi'):
            copied = (x0-cx0, y0-cy0, x1-cx0, y1-cy0)
            backend.register_roi(mask, local_mask, copied, (x0, y0))
            backend.register_roi(score, local_score, copied, (x0, y0))
        if profile is not None:
            profile.meta['color_processing_size'] = [int(cx1-cx0), int(cy1-cy0)]
            profile.mark(prefix+'.roi_copy')
        return mask, score
    if backend is not None:
        return backend.combined_evidence(enhanced, reference, profile, prefix)
    mask, score = tube_evidence(reference, profile, prefix+'.reference')
    if enhanced is not reference:
        additional, _ = tube_evidence(enhanced, profile, prefix+'.enhanced')
        mask |= additional & cv2.dilate(mask, np.ones((3, 3), np.uint8))
    if profile is not None:
        profile.mark(prefix+'.combine')
        profile.count('color_evidence_calls')
    return mask, score


def trim_lines(lines, mask, score, bounds=(0, 0, 640, 360), backend=None):
    """Search along the seed line; join short holes and split persistent non-red spans."""
    if backend is not None:
        return backend.trim_lines(lines, mask, score, bounds)
    items = []
    x0, y0, x1, y1 = bounds
    corners = np.array([[x0, y0], [x1-1, y0], [x1-1, y1-1], [x0, y1-1]], float)
    for line in lines:
        span = corners@line['d']
        a = line['n']*line['b']+line['d']*span.min()
        b = line['n']*line['b']+line['d']*span.max()
        points, hits, _ = samples(mask, a, b, max(4, min(8, int(line['width']*.65))))
        occupancy = hits.any(axis=1)
        joined = cv2.morphologyEx(np.uint8(occupancy)[None, :], cv2.MORPH_CLOSE,
                                  np.ones((1, 19), np.uint8))[0] > 0
        ids = np.flatnonzero(joined)
        for run in np.split(ids, np.flatnonzero(np.diff(ids) > 1)+1):
            if len(run) < 24:
                continue
            low, high = points[run[[0, -1]]]@line['d']
            if min(high, line['hi'])-max(low, line['lo']) < min(15, .4*(line['hi']-line['lo'])):
                continue
            if high-low < 24 or occupancy[run].mean() < .5:
                continue
            item = dict(line, lo=float(low), hi=float(high), support=float(occupancy[run].mean()))
            pixels = points[run]
            xs, ys = np.rint(pixels).astype(int).T
            values = score[np.clip(ys, 0, mask.shape[0]-1), np.clip(xs, 0, mask.shape[1]-1)]
            strong = pixels[values > .30]@line['d']
            strong_limits = np.percentile(strong, [2, 98]) if len(strong) > 10 else (low, high)
            item['strong_lo'], item['strong_hi'] = map(float, strong_limits)
            _, cross_sections, _ = samples(mask, *endpoints(item), min(16, max(4, int(line['width']))))
            widths = cross_sections.sum(axis=1)
            widths = widths[widths > 0]
            item['width'] = max(2., float(np.median(widths)) if len(widths) else line['width'])
            items.append(item)
    return merge_trimmed(items)


def merge_trimmed(items):
    result = []
    for item in items:
        duplicate = None
        item_ends = endpoints(item)
        midpoint = item_ends.mean(axis=0)
        for old in result:
            if old['vertical'] != item['vertical'] or old['d']@item['d'] < .995:
                continue
            distance = abs(midpoint@old['n']-old['b'])
            projected = item_ends@old['d']
            gap = max(projected.min()-old['hi'], old['lo']-projected.max(), 0)
            if distance < max(2, .45*min(old['width'], item['width'])) and gap < 20:
                duplicate = old
                break
        if duplicate is None:
            result.append(item)
        else:
            projected = item_ends@duplicate['d']
            duplicate['lo'] = min(duplicate['lo'], float(projected.min()))
            duplicate['hi'] = max(duplicate['hi'], float(projected.max()))
            duplicate['strong_lo'] = min(duplicate['strong_lo'], item['strong_lo'])
            duplicate['strong_hi'] = max(duplicate['strong_hi'], item['strong_hi'])
    return sorted(result, key=lambda line: (line['hi']-line['lo'])*np.sqrt(line['width'])*line['support'],
                   reverse=True)


def side_evidence(mask, line, segment):
    """Skip compact white end caps, validate eight bins and limit extrapolation."""
    a, b = np.asarray(segment, float)
    length = np.linalg.norm(b-a)
    if length < 22:
        return None
    margin = min(.1, max(3, line['width'])/length)
    _, hits, _ = samples(mask, a+margin*(b-a), b-margin*(b-a), max(3, min(8, int(line['width']*.55))))
    occupancy = hits.any(axis=1)
    bins = [part.mean() for part in np.array_split(occupancy, 8) if len(part)]
    if occupancy.mean() < .62 or min(bins[:2]+bins[-2:]) < .35:
        return None
    limits = np.asarray([a, b])@line['d']
    extension = max(line['lo']-limits.min(), limits.max()-line['hi'], 0)
    if extension > max(22, 2.2*line['width']):
        return None
    excess = max(limits.min()-line.get('strong_lo', line['lo']),
                 line.get('strong_hi', line['hi'])-limits.max(), 0)
    if excess > max(65, .45*length):
        return None
    return float(occupancy.mean())


def complete_models(vertical, horizontal, mask, bounds=(0, 0, 640, 360), valid_mask=None, profile=None, backend=None):
    """Enumerate four independent lines, not transitively connected pipe groups."""
    if backend is not None and hasattr(backend, 'complete_models_gpu'):
        return backend.complete_models_gpu(vertical, horizontal, mask, bounds, valid_mask, profile)
    candidates = []
    pending = []
    # Each line participates in many four-line combinations. Its midpoint is
    # constant during enumeration, so avoid allocating endpoints repeatedly.
    midpoints = {id(line): endpoints(line).mean(axis=0) for line in vertical+horizontal}
    crossings = {}
    valid_sides = {}
    side_results = {}
    horizontal_pairs = []
    for hs in combinations(horizontal, 2):
        top, bottom = sorted(hs, key=lambda line: midpoints[id(line)][1])
        horizontal_pairs.append((top, bottom, top['d']@bottom['d'] >= .90))
    for vs in combinations(vertical, 2):
        left, right = sorted(vs, key=lambda line: midpoints[id(line)][0])
        if left['d']@right['d'] < .90:
            continue
        for top, bottom, parallel in horizontal_pairs:
            if profile is not None:
                profile.count('quad_combinations')
            if not parallel:
                if profile is not None:
                    profile.mark('models.precheck')
                continue
            lines = [top, right, bottom, left]
            widths = [line['width'] for line in lines]
            if max(widths)/min(widths) > 2.4:
                if profile is not None:
                    profile.mark('models.precheck')
                continue
            if profile is not None:
                profile.mark('models.precheck')
            corners = []
            for index, line in enumerate(lines):
                other = lines[index-1]
                key = (id(other), id(line))
                if key not in crossings:
                    crossings[key] = intersection(other, line)
                corners.append(crossings[key])
            if any(corner is None for corner in corners):
                if profile is not None:
                    profile.mark('models.geometry')
                continue
            quad = np.float32(corners)
            x0, y0, x1, y1 = bounds
            if (not cv2.isContourConvex(quad) or cv2.contourArea(quad) < 900
                    or np.any(quad < [x0, y0]) or np.any(quad > [x1-1, y1-1])):
                if profile is not None:
                    profile.mark('models.geometry')
                continue
            segments = [np.array([corners[index], corners[(index+1) % 4]]) for index in range(4)]
            lengths = np.array([np.linalg.norm(b-a) for a, b in segments])
            if lengths.min() < 25 or lengths.max()/lengths.min() > 5:
                if profile is not None:
                    profile.mark('models.geometry')
                continue
            if profile is not None:
                profile.mark('models.geometry')
            if valid_mask is not None:
                visible = True
                for line, segment in zip(lines, segments):
                    key = (id(line), segment.tobytes())
                    if key not in valid_sides:
                        probes = np.rint(np.linspace(*segment, 33)).astype(int)
                        valid_sides[key] = valid_mask[probes[:, 1], probes[:, 0]].all()
                    if not valid_sides[key]:
                        visible = False
                        break
                if not visible:
                    if profile is not None:
                        profile.mark('models.valid_mask')
                    continue
            if profile is not None:
                profile.mark('models.valid_mask')
            if backend is not None:
                pending.append((lines, segments, widths, corners, quad))
                continue
            supports = []
            for line, segment in zip(lines, segments):
                key = (id(line), segment.tobytes())
                if key not in side_results:
                    side_results[key] = side_evidence(mask, line, segment)
                support = side_results[key]
                if profile is not None:
                    profile.count('model_side_checks')
                if support is None:
                    break
                supports.append(support)
            if profile is not None:
                profile.mark('models.color_support')
                profile.count('quad_color_checks')
            if len(supports) != 4 or np.mean(supports) < .73:
                continue
            candidates.append(_measured_model(lines, segments, widths, corners, quad, supports))
            if profile is not None:
                profile.mark('models.accept')
    if pending:
        unique, lookup, indices = [], {}, []
        for item in pending:
            for line, segment in zip(item[0], item[1]):
                key = (id(line), segment.tobytes())
                if key not in lookup:
                    lookup[key] = len(unique)
                    unique.append((line, segment))
                indices.append(lookup[key])
        measured = backend.side_support(mask, [item[0] for item in unique], [item[1] for item in unique])
        supports = measured[indices].reshape(-1, 4)
        if profile is not None:
            profile.mark('models.color_support')
            profile.count('model_side_checks', 4*len(pending))
            profile.count('quad_color_checks', len(pending))
            profile.count('unique_model_side_checks', len(unique))
        for item, evidence in zip(pending, supports):
            if np.all(evidence >= 0) and np.mean(evidence) >= .73:
                candidates.append(_measured_model(*item, evidence.tolist()))
        if profile is not None:
            profile.mark('models.accept')
    result = sorted(candidates, key=lambda candidate: candidate['score'], reverse=True)
    if profile is not None:
        profile.mark('models.finish')
    return result


def _measured_model(lines, segments, widths, corners, quad, supports):
    measured_lines = []
    for line, segment in zip(lines, segments):
        low, high = np.sort(segment@line['d'])
        observed = endpoints(dict(line, lo=max(low, line['lo']), hi=min(high, line['hi'])))
        measured_lines.append(dict(line, observed_segment=observed))
    width = float(np.median(widths))
    area = float(cv2.contourArea(quad))
    return dict(lines=measured_lines, segments=segments, corners=list(corners), quad=quad,
        complete=True, width=width, score=float(np.sqrt(area)*width**.7*np.mean(supports)),
        geometry_validated=True, model='four-line-color-validated', side_support=list(supports))
