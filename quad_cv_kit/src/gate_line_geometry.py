"""Shared red-pipe centerline geometry in the 640 x 360 detection canvas."""
import cv2
import numpy as np


def endpoints(line):
    origin = line['n']*line['b']
    return origin+np.array([line['lo'], line['hi']])[:, None]*line['d']


def intersection(a, b):
    ax, ay = a['n']
    bx, by = b['n']
    det = ax*by-ay*bx
    if abs(det) < .3:
        return None
    return np.array([(a['b']*by-ay*b['b'])/det, (ax*b['b']-a['b']*bx)/det])


def samples(mask, a, b, radius=5, step=1):
    length = np.linalg.norm(b-a)
    points = np.linspace(a, b, max(10, int(length/step)))
    normal = np.array([-(b-a)[1], (b-a)[0]]) / max(length, 1)
    # Keep separate coordinate arrays; the Nx(2r+1)x2 probe tensor and its
    # strided views were allocated for every seed, trim and model side.
    offsets = np.arange(-radius, radius+1)[None, :]
    xs = np.rint(points[:, 0, None] + offsets*normal[0]).astype(np.float32)
    ys = np.rint(points[:, 1, None] + offsets*normal[1]).astype(np.float32)
    # Round in float64 first to preserve the exact nearest-pixel rule. Remap
    # combines bounds handling and gathering without clipped int64 arrays.
    support = cv2.remap(mask, xs, ys, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT) > 0
    return points, support, normal
