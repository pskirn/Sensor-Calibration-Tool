"""Live checkerboard detection for both sensors.

Produces the same quantities the offline C++ detectors produce -- a plane normal,
a plane distance, and a set of on-plane points -- but fast enough to run on every
preview frame so the operator gets immediate feedback while aiming the board.

Both detectors orient their normal to satisfy the `d > 0` convention described
in `plane_fit`, so a camera detection and a LiDAR detection of the same board
are directly comparable by the solver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np

from . import plane_fit
from .plane_fit import PlaneFit


@dataclass
class BoardSpec:
    """Checkerboard geometry.

    `cols`/`rows` are **internal corner counts**, not square counts -- an 8x6
    squares board has 7x5 internal corners. Getting this wrong is the single
    most common reason detection silently never fires, so the UI states it
    explicitly at the point of entry.
    """

    cols: int
    rows: int
    square_size: float                      # metres
    board_width: Optional[float] = None     # physical panel size, metres
    board_height: Optional[float] = None

    @property
    def pattern(self) -> tuple[int, int]:
        return (self.cols, self.rows)

    @property
    def expected_panel(self) -> tuple[float, float]:
        """Physical board extents as (long side, short side), in metres.

        A board with `cols` internal corners has `cols + 1` squares across, and
        printed boards usually carry some quiet border beyond that. Absent a
        measurement we assume half a square of border per side, which sits in
        the middle of the range real boards occupy; the check that uses this
        allows a generous tolerance precisely because it is an estimate.

        Measure and set `board_width` / `board_height` if your board has an
        unusually wide margin.
        """
        w = self.board_width or (self.cols + 2) * self.square_size
        h = self.board_height or (self.rows + 2) * self.square_size
        return (max(w, h), min(w, h))

    def object_points(self) -> np.ndarray:
        """Corner coordinates in the board's own frame, Z=0."""
        grid = np.zeros((self.rows * self.cols, 3), np.float32)
        grid[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        return grid * self.square_size


@dataclass
class CameraDetection:
    ok: bool
    normal: Optional[np.ndarray] = None      # (3,) unit, camera frame
    distance: float = 0.0                    # metres
    centroid: Optional[np.ndarray] = None    # (3,) camera frame
    corners_3d: Optional[np.ndarray] = None  # (4, 3) pattern extremes, camera frame
    corners_2d: Optional[np.ndarray] = None  # (N, 2) image points, for overlay
    reprojection_error: float = 0.0          # pixels, RMS
    message: str = ""


@dataclass
class LidarDetection:
    ok: bool
    normal: Optional[np.ndarray] = None
    distance: float = 0.0
    centroid: Optional[np.ndarray] = None
    corners_3d: Optional[np.ndarray] = None  # (4, 3) on-plane, LiDAR frame
    inliers: Optional[np.ndarray] = None     # (M, 3) for preview
    num_inliers: int = 0
    rms_error: float = 0.0                   # metres
    panel_size: tuple = (0.0, 0.0)           # measured (long, short), metres
    message: str = ""


# --- camera ---------------------------------------------------------------

_FIND_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
)
_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def detect_camera_board(
    image: np.ndarray,
    spec: BoardSpec,
    K: np.ndarray,
    dist: np.ndarray,
    detect_width: int = 1024,
) -> CameraDetection:
    """Find the board and recover its plane in the camera frame.

    Detection runs on a downscaled copy and sub-pixel refinement on the full
    resolution original. `findChessboardCorners` scales badly -- on a 2448x2048
    frame it can take over a second, which is useless for a live preview -- but
    its output is only ever a seed for `cornerSubPix`, so downscaling costs no
    final accuracy while making the preview interactive.
    """
    if image is None or image.size == 0:
        return CameraDetection(False, message="No image")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image

    scale = 1.0
    search = gray
    if detect_width and gray.shape[1] > detect_width:
        scale = detect_width / gray.shape[1]
        search = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    found, corners = cv2.findChessboardCorners(search, spec.pattern, _FIND_FLAGS)
    if not found:
        return CameraDetection(
            False,
            message=f"No {spec.cols}x{spec.rows} checkerboard found "
                    f"(counts are internal corners, not squares)",
        )

    corners = corners / scale
    corners = cv2.cornerSubPix(gray, corners.astype(np.float32), (11, 11), (-1, -1),
                               _SUBPIX_CRITERIA)

    objp = spec.object_points()
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return CameraDetection(False, corners_2d=corners.reshape(-1, 2),
                               message="solvePnP failed")

    R, _ = cv2.Rodrigues(rvec)
    tvec = tvec.reshape(3)

    # Board points live at Z=0 in board frame, so the board's Z axis -- the third
    # column of R -- is the plane normal expressed in the camera frame.
    normal = R[:, 2].astype(np.float64)
    centroid = (R @ objp.mean(axis=0) + tvec).astype(np.float64)

    d = float(np.dot(normal, centroid))
    if d < 0:
        normal, d = -normal, -d

    # The four extreme internal corners, as a compact description of the pattern.
    grid = objp.reshape(spec.rows, spec.cols, 3)
    extremes = np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
    corners_3d = (extremes @ R.T + tvec).astype(np.float64)

    projected, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    reproj = float(np.sqrt(np.mean(
        np.sum((projected.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1)
    )))

    return CameraDetection(
        ok=True,
        normal=normal,
        distance=d,
        centroid=centroid,
        corners_3d=corners_3d,
        corners_2d=corners.reshape(-1, 2),
        reprojection_error=reproj,
        message="",
    )


# --- lidar ----------------------------------------------------------------

def detect_lidar_board(
    points: np.ndarray,
    spec: BoardSpec,
    roi_min,
    roi_max,
    ransac_threshold: float = 0.02,
    max_iterations: int = 500,
    min_inliers: int = 60,
    cluster_voxel: float = 0.06,
    panel_tolerance: float = 0.25,
) -> LidarDetection:
    """Segment the board plane out of a LiDAR frame.

    Pipeline: crop to ROI, RANSAC the dominant plane, keep the largest connected
    cluster, then check the cluster's footprint against the known board size.
    That last check is what stops a wall from being reported as a confident
    detection -- a wall fits a plane beautifully, it is just the wrong plane.
    """
    if points is None or len(points) == 0:
        return LidarDetection(False, message="Empty cloud")

    roi = plane_fit.crop_roi(points, roi_min, roi_max)
    if roi.shape[0] < min_inliers:
        return LidarDetection(
            False,
            message=f"Only {roi.shape[0]} points inside the ROI box -- widen the ROI "
                    f"or move the board into it",
        )

    fit = plane_fit.fit_plane_ransac(
        roi, threshold=ransac_threshold, max_iterations=max_iterations,
        min_inliers=min_inliers,
    )
    if fit is None:
        return LidarDetection(False, message="RANSAC found no plane in the ROI")

    clustered = plane_fit.largest_cluster(fit.inliers, voxel=cluster_voxel)
    if clustered.shape[0] < min_inliers:
        return LidarDetection(
            False, message=f"Board cluster too small ({clustered.shape[0]} points)"
        )
    fit = plane_fit.refine_plane(clustered)

    long_side, short_side = plane_fit.rectangle_dimensions(fit)
    exp_long, exp_short = spec.expected_panel
    size_off = max(
        abs(long_side - exp_long) / max(exp_long, 1e-6),
        abs(short_side - exp_short) / max(exp_short, 1e-6),
    )
    corners = plane_fit.rectangle_corners(fit)

    if size_off > panel_tolerance:
        return LidarDetection(
            False,
            normal=fit.normal, distance=fit.distance, centroid=fit.centroid,
            corners_3d=corners, inliers=fit.inliers, num_inliers=fit.num_inliers,
            rms_error=fit.rms_error, panel_size=(long_side, short_side),
            message=(
                f"Plane is {long_side:.2f}x{short_side:.2f} m but the board should be "
                f"{exp_long:.2f}x{exp_short:.2f} m -- probably a wall or the floor"
            ),
        )

    return LidarDetection(
        ok=True,
        normal=fit.normal,
        distance=fit.distance,
        centroid=fit.centroid,
        corners_3d=corners,
        inliers=fit.inliers,
        num_inliers=fit.num_inliers,
        rms_error=fit.rms_error,
        panel_size=(long_side, short_side),
        message="",
    )


def draw_camera_overlay(image: np.ndarray, detection: CameraDetection,
                        spec: BoardSpec) -> np.ndarray:
    """Annotate a preview frame with the detection result."""
    canvas = image.copy()
    if detection.corners_2d is None:
        return canvas

    pts = detection.corners_2d.reshape(-1, 1, 2).astype(np.float32)
    cv2.drawChessboardCorners(canvas, spec.pattern, pts, detection.ok)

    if detection.ok:
        hull = cv2.convexHull(detection.corners_2d.astype(np.int32))
        cv2.polylines(canvas, [hull], True, (80, 220, 120), 3)
    return canvas
