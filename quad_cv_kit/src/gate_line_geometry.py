"""Shared red-pipe centerline geometry in the 640 x 360 detection canvas."""
import numpy as np


def endpoints(line):
    return np.array([line['n']*line['b']+line['d']*line[key] for key in ('lo', 'hi')])


def intersection(a, b):
    matrix = np.stack([a['n'], b['n']])
    if abs(np.linalg.det(matrix)) < .3:
        return None
    return np.linalg.solve(matrix, [a['b'], b['b']])


def samples(mask, a, b, radius=5):
    length = np.linalg.norm(b-a)
    points = np.linspace(a, b, max(10, int(length)))
    normal = np.array([-(b-a)[1], (b-a)[0]]) / max(length, 1)
    probes = points[:, None, :] + np.arange(-radius, radius+1)[None, :, None]*normal
    xs, ys = np.rint(probes[..., 0]).astype(int), np.rint(probes[..., 1]).astype(int)
    valid = (xs >= 0) & (xs < mask.shape[1]) & (ys >= 0) & (ys < mask.shape[0])
    support = mask[np.clip(ys, 0, mask.shape[0]-1), np.clip(xs, 0, mask.shape[1]-1)] > 0
    return points, support & valid, normal
