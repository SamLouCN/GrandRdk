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


def combined_evidence(enhanced, reference, profile=None, prefix='color'):
    """Enhanced pixels can strengthen only locally supported reference colors."""
    mask, score = tube_evidence(reference, profile, prefix+'.reference')
    if enhanced is not reference:
        additional, _ = tube_evidence(enhanced, profile, prefix+'.enhanced')
        mask |= additional & cv2.dilate(mask, np.ones((3, 3), np.uint8))
    if profile is not None:
        profile.mark(prefix+'.combine')
        profile.count('color_evidence_calls')
    return mask, score


def trim_lines(lines, mask, score, bounds=(0, 0, 640, 360)):
    """Search along the seed line; join short holes and split persistent non-red spans."""
    result = []
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
            item['strong_lo'] = float(np.percentile(strong, 2)) if len(strong) > 10 else float(low)
            item['strong_hi'] = float(np.percentile(strong, 98)) if len(strong) > 10 else float(high)
            _, cross_sections, _ = samples(mask, *endpoints(item), min(16, max(4, int(line['width']))))
            widths = cross_sections.sum(axis=1)
            widths = widths[widths > 0]
            item['width'] = max(2., float(np.median(widths)) if len(widths) else line['width'])
            duplicate = None
            for old in result:
                if old['vertical'] != item['vertical'] or old['d']@item['d'] < .995:
                    continue
                distance = abs(endpoints(item).mean(axis=0)@old['n']-old['b'])
                projected = endpoints(item)@old['d']
                gap = max(projected.min()-old['hi'], old['lo']-projected.max(), 0)
                if distance < max(2, .45*min(old['width'], item['width'])) and gap < 20:
                    duplicate = old
                    break
            if duplicate is None:
                result.append(item)
            else:
                projected = endpoints(item)@duplicate['d']
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


def complete_models(vertical, horizontal, mask, bounds=(0, 0, 640, 360), valid_mask=None, profile=None):
    """Enumerate four independent lines, not transitively connected pipe groups."""
    candidates = []
    for vs in combinations(vertical, 2):
        left, right = sorted(vs, key=lambda line: endpoints(line).mean(axis=0)[0])
        if left['d']@right['d'] < .90:
            continue
        for hs in combinations(horizontal, 2):
            if profile is not None:
                profile.count('quad_combinations')
            top, bottom = sorted(hs, key=lambda line: endpoints(line).mean(axis=0)[1])
            if top['d']@bottom['d'] < .90:
                if profile is not None:
                    profile.mark('models.precheck')
                continue
            lines = [top, right, bottom, left]
            widths = np.array([line['width'] for line in lines])
            if widths.max()/widths.min() > 2.4:
                if profile is not None:
                    profile.mark('models.precheck')
                continue
            if profile is not None:
                profile.mark('models.precheck')
            corners = [intersection(lines[index-1], line) for index, line in enumerate(lines)]
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
                probes = np.rint(np.vstack([np.linspace(a, b, 33) for a, b in segments])).astype(int)
                if not valid_mask[probes[:, 1], probes[:, 0]].all():
                    if profile is not None:
                        profile.mark('models.valid_mask')
                    continue
            if profile is not None:
                profile.mark('models.valid_mask')
            supports = []
            for line, segment in zip(lines, segments):
                support = side_evidence(mask, line, segment)
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
            measured_lines = []
            for line, segment in zip(lines, segments):
                low, high = np.sort(segment@line['d'])
                observed = endpoints(dict(line, lo=max(low, line['lo']), hi=min(high, line['hi'])))
                measured_lines.append(dict(line, observed_segment=observed))
            width = float(np.median(widths))
            area = float(cv2.contourArea(quad))
            candidates.append(dict(lines=measured_lines, segments=segments, corners=list(corners), quad=quad,
                complete=True, width=width, score=float(np.sqrt(area)*width**.7*np.mean(supports)),
                geometry_validated=True, model='four-line-color-validated', side_support=list(supports)))
            if profile is not None:
                profile.mark('models.accept')
    result = sorted(candidates, key=lambda candidate: candidate['score'], reverse=True)
    if profile is not None:
        profile.mark('models.finish')
    return result
