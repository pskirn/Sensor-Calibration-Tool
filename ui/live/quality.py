"""Pose quality and coverage scoring for live capture.

This module exists because of a real degeneracy in plane-based extrinsic
calibration, and it is the main thing that makes guided live capture better than
"take 20 pictures and hope".

The solve constrains the transform through two residuals: `R·n_lidar ≈ n_camera`,
and `n_cam·(R·p_lid + t) = d_cam`. Look at what the second one can see. A board
plane only pins down the component of translation *along its own normal* -- slide
the sensor pair parallel to the board and every point-on-plane residual is
unchanged. So a single board orientation leaves two translation directions
completely unobservable, and a whole session of boards held at the same angle
leaves them just as unobservable, no matter how many frames you take.

Recovering all three translation components therefore requires board normals
that span 3D space. The cleanest way to measure that is the singular values of
the stacked normal matrix: if the normals span well, all three singular values
are healthy; if the boards are near-parallel, the smallest collapses toward zero
and the calibration is ill-conditioned however good the residuals look.

That is what `normal_span` reports, and why the UI nags for board *rotation*
rather than merely more captures.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

# Below this, the geometry is too close to degenerate to trust the translation.
MIN_NORMAL_SPAN = 0.15
# Two boards closer than this in orientation add little new information.
NOVELTY_ANGLE_DEG = 8.0
# Fewer than this and a least-squares fit has little redundancy to average noise.
RECOMMENDED_POSES = 9


@dataclass
class PoseSample:
    """One captured pose, in the form the quality metrics need."""

    index: int
    cam_normal: np.ndarray
    cam_distance: float
    cam_centroid: np.ndarray
    lid_normal: np.ndarray
    lid_distance: float
    lid_centroid: np.ndarray
    image_point: Optional[Sequence[float]] = None   # board centre in pixels
    reprojection_error: float = 0.0
    lidar_rms: float = 0.0
    lidar_points: int = 0


def normal_span(normals: Sequence[np.ndarray]) -> float:
    """How well a set of plane normals spans 3D, in [0, 1].

    Ratio of smallest to largest singular value of the stacked normals. Near 0
    means the boards are effectively coplanar in orientation and the translation
    solve is ill-conditioned; toward 1 means they span all three axes well.
    """
    if len(normals) < 3:
        return 0.0
    stacked = np.asarray([np.asarray(n, dtype=np.float64) for n in normals])
    stacked = stacked / np.linalg.norm(stacked, axis=1, keepdims=True)
    singular = np.linalg.svd(stacked, compute_uv=False)
    if singular[0] < 1e-12:
        return 0.0
    return float(singular[-1] / singular[0])


def angular_novelty(normal: np.ndarray, existing: Sequence[np.ndarray]) -> float:
    """Smallest angle (degrees) between `normal` and any already-captured one.

    A large value means this orientation is genuinely new information.
    """
    if not len(existing):
        return 180.0
    n = np.asarray(normal, dtype=np.float64)
    n = n / np.linalg.norm(n)
    others = np.asarray([np.asarray(e, dtype=np.float64) for e in existing])
    others = others / np.linalg.norm(others, axis=1, keepdims=True)
    cos = np.clip(others @ n, -1.0, 1.0)
    return float(np.degrees(np.arccos(np.abs(cos))).min())


def image_coverage(points: Sequence[Optional[Sequence[float]]],
                   image_size: Sequence[int], grid: int = 3) -> float:
    """Fraction of a 3x3 image grid in which a board has been observed.

    Lens distortion is estimated from -- and matters most at -- the frame edges,
    so poses clustered in the middle of the image leave the corners unmodelled.
    """
    width, height = image_size
    if not width or not height:
        return 0.0
    seen = set()
    for pt in points:
        if pt is None:
            continue
        gx = min(int(pt[0] / width * grid), grid - 1)
        gy = min(int(pt[1] / height * grid), grid - 1)
        if 0 <= gx < grid and 0 <= gy < grid:
            seen.add((gx, gy))
    return len(seen) / float(grid * grid)


def occupied_cells(points: Sequence[Optional[Sequence[float]]],
                   image_size: Sequence[int], grid: int = 3) -> List[List[int]]:
    """The grid cells covered so far, for drawing the coverage map."""
    width, height = image_size
    cells: List[List[int]] = []
    if not width or not height:
        return cells
    seen = set()
    for pt in points:
        if pt is None:
            continue
        gx = min(int(pt[0] / width * grid), grid - 1)
        gy = min(int(pt[1] / height * grid), grid - 1)
        if (gx, gy) not in seen:
            seen.add((gx, gy))
            cells.append([gx, gy])
    return cells


@dataclass
class CaptureAssessment:
    """Verdict on the currently-visible board, shown live to the operator."""

    can_capture: bool
    novel: bool
    novelty_deg: float
    reasons: List[str] = field(default_factory=list)
    hints: List[str] = field(default_factory=list)


def assess_live_pose(
    cam_ok: bool,
    lid_ok: bool,
    cam_normal: Optional[np.ndarray],
    captured: Sequence[PoseSample],
    cam_message: str = "",
    lid_message: str = "",
    reprojection_error: float = 0.0,
    max_reprojection_error: float = 1.5,
) -> CaptureAssessment:
    """Decide whether the board as currently held is worth capturing."""
    reasons: List[str] = []
    hints: List[str] = []

    if not cam_ok:
        reasons.append(cam_message or "Board not visible to the camera")
    if not lid_ok:
        reasons.append(lid_message or "Board not found in the LiDAR cloud")

    if cam_ok and reprojection_error > max_reprojection_error:
        reasons.append(
            f"Corner reprojection error {reprojection_error:.2f} px is high -- "
            f"hold the board still, or check the intrinsics"
        )

    novelty = 180.0
    novel = True
    if cam_ok and cam_normal is not None and captured:
        novelty = angular_novelty(cam_normal, [p.cam_normal for p in captured])
        novel = novelty >= NOVELTY_ANGLE_DEG
        if not novel:
            hints.append(
                f"This angle is within {novelty:.0f} deg of a pose you already have -- "
                f"tilt or rotate the board more to add new information"
            )

    return CaptureAssessment(
        can_capture=not reasons,
        novel=novel,
        novelty_deg=novelty,
        reasons=reasons,
        hints=hints,
    )


@dataclass
class SessionReadiness:
    """Whether the captured set is good enough to solve on."""

    ready: bool
    num_poses: int
    normal_span: float
    coverage: float
    distance_range: tuple
    blocking: List[str] = field(default_factory=list)
    advice: List[str] = field(default_factory=list)


def assess_session(captured: Sequence[PoseSample],
                   image_size: Sequence[int]) -> SessionReadiness:
    """Summarise how well-conditioned the captured set is, with actionable advice."""
    blocking: List[str] = []
    advice: List[str] = []
    count = len(captured)

    if count == 0:
        return SessionReadiness(False, 0, 0.0, 0.0, (0.0, 0.0),
                                blocking=["No poses captured yet"])

    normals = [p.cam_normal for p in captured]
    span = normal_span(normals)
    coverage = image_coverage([p.image_point for p in captured], image_size)
    distances = [p.cam_distance for p in captured]
    dist_range = (float(min(distances)), float(max(distances)))

    if count < 3:
        blocking.append(f"Only {count} pose(s); at least 3 are needed to solve")
    elif count < RECOMMENDED_POSES:
        advice.append(
            f"{count} poses will solve, but {RECOMMENDED_POSES}+ averages out "
            f"sensor noise noticeably better"
        )

    if count >= 3 and span < MIN_NORMAL_SPAN:
        blocking.append(
            f"Board orientations are too similar (span {span:.2f}). Translation "
            f"cannot be recovered from near-parallel boards -- capture poses with "
            f"the board tilted left, right, up and down"
        )
    elif span < 0.3:
        advice.append(
            f"Orientation span is {span:.2f}; more extreme board tilts would "
            f"tighten the translation estimate"
        )

    if coverage < 0.5:
        advice.append(
            f"The board has only covered {coverage * 100:.0f}% of the image area -- "
            f"work it into the corners as well as the centre"
        )

    if dist_range[1] - dist_range[0] < 0.5:
        advice.append(
            "All poses are at a similar range; varying board distance improves "
            "the depth-direction estimate"
        )

    return SessionReadiness(
        ready=not blocking,
        num_poses=count,
        normal_span=span,
        coverage=coverage,
        distance_range=dist_range,
        blocking=blocking,
        advice=advice,
    )
