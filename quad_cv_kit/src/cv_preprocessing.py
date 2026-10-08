"""Strong, bounded contrast preprocessing for a clean OpenCV input video."""
import math

import cv2
import numpy as np


def high_contrast_frame(frame, valid_mask=None, clahe_clip=2.5, clahe_blend=.65,
                        contrast=1.6, saturation=1.4, sharpen=.2):
    """Enhance HSV value with percentile stretching and a soft contrast curve.

    Preserve hue and invalid pixels. A soft shoulder keeps high contrast from
    turning wide highlight/shadow regions into flat 255/0 patches. Mild bilateral
    filtering limits amplification of compression noise before local contrast.
    """
    parameters = (clahe_clip, clahe_blend, contrast, saturation, sharpen)
    if (not all(math.isfinite(value) for value in parameters)
            or not 0 < clahe_clip <= 8 or not 0 <= clahe_blend <= 1
            or not 1 <= contrast <= 3 or not 1 <= saturation <= 2
            or not 0 <= sharpen <= 2):
        raise ValueError('Invalid high-contrast preprocessing parameters')
    if valid_mask is not None and np.shape(valid_mask) != frame.shape[:2]:
        raise ValueError('Validity mask must match frame dimensions')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    active = hsv[:, :, 2] > 0
    if valid_mask is not None:
        active &= np.asarray(valid_mask, bool)
    if not active.any():
        return frame.copy()
    value = hsv[:, :, 2].copy()
    value[~active] = round(float(np.median(value[active])))
    smooth = cv2.bilateralFilter(value, 7, 25, 5)
    smooth = cv2.GaussianBlur(smooth, (0, 0), 1.1)
    local = cv2.createCLAHE(clahe_clip, (8, 8)).apply(smooth)
    value = (1-clahe_blend)*smooth.astype(np.float32)+clahe_blend*local
    low, high = np.percentile(value[active], (1., 99.))
    if high-low > 8:
        value = np.clip((value-low)/(high-low), 0, 1)
        # Gain is the center slope relative to a linear curve; endpoints remain bounded.
        value = .5+.5*np.tanh(contrast*(2*value-1))/np.tanh(contrast)
        value = 6+242*value
    if sharpen:
        detail = value-cv2.GaussianBlur(value, (0, 0), 1.2)
        detail[np.abs(detail) < 4] = 0
        value = np.clip(value+sharpen*detail, 4, 250)
    hsv[:, :, 2] = np.rint(value).astype(np.uint8)
    hsv[:, :, 1] = np.rint(np.clip(hsv[:, :, 1].astype(np.float32)*saturation, 0, 255)).astype(np.uint8)
    result = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    result[~active] = frame[~active]
    return result
