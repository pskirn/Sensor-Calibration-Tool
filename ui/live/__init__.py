"""Live calibration: real-time board detection, capture guidance, session export.

`detect` finds the board in each sensor, `plane_fit` supplies the point-cloud
geometry, `quality` scores whether a pose adds information, and `session` ties
them together and exports a `poses.csv` for the existing C++ solver.
"""

from .detect import BoardSpec, CameraDetection, LidarDetection, detect_camera_board, detect_lidar_board
from .quality import PoseSample, SessionReadiness, assess_session, normal_span
from .session import LiveConfig, LiveSession, LiveState

__all__ = [
    "BoardSpec",
    "CameraDetection",
    "LidarDetection",
    "LiveConfig",
    "LiveSession",
    "LiveState",
    "PoseSample",
    "SessionReadiness",
    "assess_session",
    "detect_camera_board",
    "detect_lidar_board",
    "normal_span",
]
