"""Generic `sensor_msgs/PointCloud2` decoding.

This is the piece that makes "works with any LiDAR" true rather than aspirational.
PointCloud2 is a self-describing binary blob: a list of `PointField` entries says
what each field is called, where it starts in the record, and what numeric type
it is. Decode against that description and you handle every vendor's layout
without special-casing any of them.

Real layouts differ a lot, which is why hardcoding is not an option:

    Velodyne VLP-16   x,y,z (f32), intensity (f32), ring (u16), time (f32)
    Ouster OS-1       x,y,z (f32), intensity (f32), t (u32), reflectivity (u16), ...
    Livox Mid-360     x,y,z (f32), t (u32), intensity (f32), tag (u8), line (u8)
    Hesai             x,y,z (f32), intensity (f32), timestamp (f64), ring (u16)

Note the Livox record is 22 bytes with mixed widths and no padding to a 4-byte
boundary -- assuming a uniform float32 record would fail on it outright.

The same decoder works for messages from `rclpy` and from `rosbags`, because
both expose the same duck-typed attributes (`fields`, `data`, `point_step`).
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

# sensor_msgs/PointField datatype constants -> numpy dtypes.
_PF_TO_NP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

# Field names that different vendors use for return strength. Checked in order.
_INTENSITY_ALIASES = ("intensity", "reflectivity", "i", "instensity")


def _structured_dtype(msg) -> np.dtype:
    """Build a numpy structured dtype mirroring the message's own field table."""
    names, formats, offsets = [], [], []
    seen: set[str] = set()
    for f in msg.fields:
        np_type = _PF_TO_NP.get(int(f.datatype))
        if np_type is None:
            continue  # unknown datatype code; skip rather than misread the record
        name = str(f.name)
        # Some drivers emit duplicate field names; keep the first, uniquify the rest
        # so the dtype stays constructible.
        if name in seen:
            suffix = 2
            while f"{name}_{suffix}" in seen:
                suffix += 1
            name = f"{name}_{suffix}"
        seen.add(name)

        count = int(getattr(f, "count", 1) or 1)
        names.append(name)
        formats.append(np_type if count == 1 else (np_type, count))
        offsets.append(int(f.offset))

    byte_order = ">" if bool(getattr(msg, "is_bigendian", False)) else "<"
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(msg.point_step),
    })
    return dtype.newbyteorder(byte_order) if byte_order == ">" else dtype


def decode(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Decode a PointCloud2 message into `(points_xyz, intensity_or_none)`.

    Returns an (N, 3) float32 array of finite points and an optional (N,)
    float32 intensity array. Non-finite points are dropped -- organised clouds
    from depth-style sensors pad invalid returns with NaN, and every downstream
    step (ROI crop, RANSAC, centroid) would otherwise poison itself on them.
    """
    dtype = _structured_dtype(msg)
    raw = np.frombuffer(bytes(msg.data), dtype=dtype)

    missing = [ax for ax in ("x", "y", "z") if ax not in (dtype.names or ())]
    if missing:
        raise ValueError(
            f"PointCloud2 is missing required field(s) {missing}; "
            f"got {list(dtype.names or ())}"
        )

    xyz = np.stack(
        [raw["x"].astype(np.float32),
         raw["y"].astype(np.float32),
         raw["z"].astype(np.float32)],
        axis=1,
    )

    intensity = None
    for alias in _INTENSITY_ALIASES:
        if alias in (dtype.names or ()):
            intensity = raw[alias].astype(np.float32)
            break

    finite = np.isfinite(xyz).all(axis=1)
    # A cloud that is entirely zeros in xyz is also useless; but an exact-zero
    # point is legitimate in principle, so only filter non-finite here.
    if not finite.all():
        xyz = xyz[finite]
        if intensity is not None:
            intensity = intensity[finite]

    return np.ascontiguousarray(xyz), (
        np.ascontiguousarray(intensity) if intensity is not None else None
    )


def describe_fields(msg) -> list[dict]:
    """Human-readable field table, for showing the user what the sensor emits."""
    out = []
    for f in msg.fields:
        np_type = _PF_TO_NP.get(int(f.datatype))
        out.append({
            "name": str(f.name),
            "offset": int(f.offset),
            "type": np_type.__name__ if np_type else f"unknown({f.datatype})",
            "count": int(getattr(f, "count", 1) or 1),
        })
    return out


def stamp_to_seconds(header) -> float:
    """ROS `builtin_interfaces/Time` -> float seconds, across rclpy and rosbags."""
    stamp = header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9
