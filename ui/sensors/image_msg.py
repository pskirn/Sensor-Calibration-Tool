"""`sensor_msgs/Image` and `CompressedImage` decoding, without cv_bridge.

cv_bridge is only importable when a ROS environment is sourced, which would
force the whole UI to run under ROS just to replay a bag file. Decoding these
two message types by hand is a couple of dozen lines and keeps the offline path
working in a plain virtualenv.
"""

from __future__ import annotations

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - cv2 is a hard dep of the live path only
    cv2 = None  # type: ignore[assignment]


# encoding -> (channels, numpy dtype, cv2 conversion code to reach BGR or None)
_SIMPLE = {
    "bgr8":   (3, np.uint8,   None),
    "rgb8":   (3, np.uint8,   "RGB2BGR"),
    "bgra8":  (4, np.uint8,   "BGRA2BGR"),
    "rgba8":  (4, np.uint8,   "RGBA2BGR"),
    "mono8":  (1, np.uint8,   "GRAY2BGR"),
    "8UC1":   (1, np.uint8,   "GRAY2BGR"),
    "8UC3":   (3, np.uint8,   None),
    "mono16": (1, np.uint16,  "GRAY2BGR"),
    "16UC1":  (1, np.uint16,  "GRAY2BGR"),
}

# Bayer mosaics need demosaicing rather than a channel swap.
_BAYER = {
    "bayer_rggb8": "BayerBG2BGR",
    "bayer_bggr8": "BayerRG2BGR",
    "bayer_gbrg8": "BayerGR2BGR",
    "bayer_grbg8": "BayerGB2BGR",
}


def _require_cv2():
    if cv2 is None:
        raise RuntimeError(
            "OpenCV is required to decode camera frames. "
            "Install it with: pip install opencv-python-headless"
        )


def decode_image(msg) -> np.ndarray:
    """`sensor_msgs/Image` -> (H, W, 3) uint8 BGR."""
    _require_cv2()
    encoding = str(msg.encoding).lower()
    height, width = int(msg.height), int(msg.width)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if encoding in _BAYER:
        mosaic = buf.reshape(height, int(msg.step))[:, :width]
        return cv2.cvtColor(mosaic, getattr(cv2, f"COLOR_{_BAYER[encoding]}"))

    if encoding not in _SIMPLE:
        raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")

    channels, dtype, conversion = _SIMPLE[encoding]
    itemsize = np.dtype(dtype).itemsize
    # `step` is the row stride in bytes and may exceed width*channels*itemsize
    # when the driver pads rows; slice to the real width instead of reshaping
    # blindly, which would shear the image.
    rows = buf.reshape(height, int(msg.step))
    row_bytes = width * channels * itemsize
    arr = rows[:, :row_bytes].copy().view(dtype).reshape(height, width, channels)

    if dtype is np.uint16:
        # Scale 16-bit to 8-bit for display/detection; checkerboard detection
        # works on 8-bit and we never use raw depth values here.
        arr = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    if conversion is None:
        return arr if arr.shape[2] == 3 else cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(arr, getattr(cv2, f"COLOR_{conversion}"))


def decode_compressed(msg) -> np.ndarray:
    """`sensor_msgs/CompressedImage` -> (H, W, 3) uint8 BGR."""
    _require_cv2()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"Could not decode CompressedImage (format={msg.format!r})")
    return img


def decode_any(msg, msgtype: str) -> np.ndarray:
    """Dispatch on ROS message type name."""
    if "CompressedImage" in msgtype:
        return decode_compressed(msg)
    return decode_image(msg)


def camera_info_to_intrinsics(msg) -> dict:
    """`sensor_msgs/CameraInfo` -> the dict shape our config layer expects.

    Lets the live path pick up intrinsics straight off the wire when the driver
    publishes them, instead of making the user run `compute_intrinsics` first.
    """
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    D = np.array(msg.d, dtype=np.float64).ravel()
    return {
        "image_width": int(msg.width),
        "image_height": int(msg.height),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.tolist(),
        "distortion_model": str(getattr(msg, "distortion_model", "plumb_bob")),
    }
