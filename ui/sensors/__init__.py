"""Vendor-neutral sensor input layer for the calibration tool.

See `base.py` for the design rationale. The short version: the pipeline talks to
`PointCloudSource` and `ImageSource`, and adapters translate whatever the
hardware speaks into plain numpy. The ROS 2 `PointCloud2` adapter is what makes
"any LiDAR" achievable, since every vendor ships a driver for it.
"""

from .base import (
    CloudFrame,
    ImageFrame,
    ImageSource,
    PointCloudSource,
    SensorSource,
    SourceInfo,
)
from .registry import (
    discover,
    find_bags,
    find_intrinsics,
    open_cloud_source,
    open_image_source,
)

__all__ = [
    "CloudFrame",
    "ImageFrame",
    "ImageSource",
    "PointCloudSource",
    "SensorSource",
    "SourceInfo",
    "discover",
    "find_bags",
    "find_intrinsics",
    "open_cloud_source",
    "open_image_source",
]
