"""Point-cloud geometry: ROI cropping, RANSAC plane fitting, clustering, corners.

This mirrors what `src/lidar_detector.cpp` does with PCL, but in numpy so the
live preview can run it at interactive rates inside the web server without
shelling out to the C++ binary for every frame.

Plane convention matches `PlaneObservation` in the C++ side: the plane is
`n · p = d` with `d > 0`, which means the normal points *away* from the sensor
origin. `PlanePair::isValid()` rejects non-positive distances, so getting this
sign wrong produces a silently useless observation rather than a loud error.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class PlaneFit:
    normal: np.ndarray        # (3,) unit, oriented so `distance` > 0
    distance: float           # d in n·p = d, metres
    inliers: np.ndarray       # (M, 3) points supporting the plane
    centroid: np.ndarray      # (3,)
    rms_error: float          # RMS point-to-plane distance of inliers, metres

    @property
    def num_inliers(self) -> int:
        return int(self.inliers.shape[0])


def crop_roi(points: np.ndarray, roi_min, roi_max) -> np.ndarray:
    """Axis-aligned box crop -- the numpy equivalent of PCL's PassThrough.

    Cropping first is what makes the rest tractable: a Mid-360 sweep is a full
    360 degree scan of the whole room, and RANSAC on that will happily return
    the floor or a wall instead of the calibration board.
    """
    lo = np.asarray(roi_min, dtype=np.float32)
    hi = np.asarray(roi_max, dtype=np.float32)
    mask = np.all((points >= lo) & (points <= hi), axis=1)
    return points[mask]


def _plane_from_three(p: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    normal = np.cross(p[1] - p[0], p[2] - p[0])
    norm = np.linalg.norm(normal)
    if norm < 1e-9:      # collinear sample, no plane defined
        return None
    normal = normal / norm
    return normal, float(np.dot(normal, p[0]))


def fit_plane_ransac(
    points: np.ndarray,
    threshold: float = 0.02,
    max_iterations: int = 500,
    min_inliers: int = 50,
    hypothesis_budget: int = 20000,
    rng: Optional[np.random.Generator] = None,
) -> Optional[PlaneFit]:
    """RANSAC plane fit with a least-squares refinement pass.

    Hypotheses are scored against a random subset (`hypothesis_budget`) rather
    than the full cloud. An accumulated Mid-360 frame is several hundred
    thousand points, and scoring all of them 500 times is seconds of work for no
    extra accuracy -- the inlier *ratio* is what ranks hypotheses, and a 20k
    sample estimates that ratio to well under a percent. The winning model is
    then re-scored and refit on every point, so the final answer uses all data.
    """
    points = np.asarray(points, dtype=np.float32)
    if points.shape[0] < max(3, min_inliers):
        return None
    rng = rng or np.random.default_rng(0)

    scoring = points
    if points.shape[0] > hypothesis_budget:
        scoring = points[rng.choice(points.shape[0], hypothesis_budget, replace=False)]

    best_count, best_model = -1, None
    for _ in range(max_iterations):
        sample = points[rng.choice(points.shape[0], 3, replace=False)]
        model = _plane_from_three(sample)
        if model is None:
            continue
        normal, d = model
        count = int(np.count_nonzero(np.abs(scoring @ normal - d) <= threshold))
        if count > best_count:
            best_count, best_model = count, model

    if best_model is None:
        return None

    # Re-score the winner against the full cloud, then refit to its inliers.
    normal, d = best_model
    inlier_mask = np.abs(points @ normal - d) <= threshold
    if int(np.count_nonzero(inlier_mask)) < min_inliers:
        return None
    return refine_plane(points[inlier_mask])


def refine_plane(inliers: np.ndarray) -> PlaneFit:
    """Total-least-squares plane through `inliers` via SVD.

    The RANSAC winner is defined by three sampled points and is therefore as
    noisy as those three; refitting to the full inlier set is what actually
    determines the normal accuracy, and the normal is what the calibration
    solve is most sensitive to.
    """
    inliers = np.asarray(inliers, dtype=np.float64)
    centroid = inliers.mean(axis=0)
    # Smallest right-singular vector of the centred points = plane normal.
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vt[-1]
    normal /= np.linalg.norm(normal)

    d = float(np.dot(normal, centroid))
    # Enforce the d > 0 convention the C++ PlaneObservation expects.
    if d < 0:
        normal, d = -normal, -d

    residuals = inliers @ normal - d
    return PlaneFit(
        normal=normal.astype(np.float64),
        distance=d,
        inliers=inliers.astype(np.float32),
        centroid=centroid.astype(np.float64),
        rms_error=float(np.sqrt(np.mean(residuals ** 2))),
    )


def largest_cluster(points: np.ndarray, voxel: float = 0.06) -> np.ndarray:
    """Keep the biggest spatially-connected blob, via voxel flood fill.

    RANSAC returns every point on the *infinite* plane, which usually includes
    the wall or floor the board is coplanar-ish with. Connectivity is what
    separates the board itself from those. A voxel grid plus BFS over 26
    neighbours costs O(N) and needs no scipy.
    """
    points = np.asarray(points, dtype=np.float32)
    if points.shape[0] == 0:
        return points

    keys = np.floor(points / voxel).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    occupied = {tuple(k): i for i, k in enumerate(map(tuple, unique))}

    neighbours = [
        (dx, dy, dz)
        for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]

    labels = np.full(len(unique), -1, dtype=np.int64)
    current = 0
    for start in range(len(unique)):
        if labels[start] != -1:
            continue
        stack = [start]
        labels[start] = current
        while stack:
            idx = stack.pop()
            kx, ky, kz = unique[idx]
            for dx, dy, dz in neighbours:
                nbr = occupied.get((kx + dx, ky + dy, kz + dz))
                if nbr is not None and labels[nbr] == -1:
                    labels[nbr] = current
                    stack.append(nbr)
        current += 1

    if current <= 1:
        return points
    sizes = np.bincount(labels[inverse], minlength=current)
    return points[labels[inverse] == int(np.argmax(sizes))]


def plane_basis(normal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Two orthonormal vectors spanning the plane with the given normal."""
    normal = normal / np.linalg.norm(normal)
    # Cross with whichever axis is least parallel to the normal, so the result
    # is numerically well-conditioned for every orientation.
    seed = np.eye(3)[int(np.argmin(np.abs(normal)))]
    u = np.cross(normal, seed)
    u /= np.linalg.norm(u)
    return u, np.cross(normal, u)


def rectangle_corners(fit: PlaneFit) -> np.ndarray:
    """Four corners of the minimum-area rectangle enclosing the plane inliers.

    The solver only requires these points to *lie on* the board plane -- it never
    matches them to specific physical corners -- so what matters is that they are
    on-plane and spread out. Corners of the enclosing rectangle maximise that
    spread, which is what makes translation observable.

    Returned in the plane, so they are exactly on-plane by construction rather
    than being noisy measured points.
    """
    u, v = plane_basis(fit.normal)
    local = np.stack([fit.inliers @ u, fit.inliers @ v], axis=1).astype(np.float32)

    try:
        import cv2
        (cx, cy), (w, h), angle = cv2.minAreaRect(local)
        box = cv2.boxPoints(((cx, cy), (w, h), angle))
    except Exception:
        # Axis-aligned fallback keeps this usable without OpenCV.
        lo, hi = local.min(axis=0), local.max(axis=0)
        box = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])

    # Lift 2D plane coordinates back to 3D. Any point with plane coords (a, b)
    # is a*u + b*v + d*n, which satisfies n·p = d exactly.
    origin = fit.distance * fit.normal
    return np.array([origin + a * u + b * v for a, b in box], dtype=np.float64)


def rectangle_dimensions(fit: PlaneFit) -> Tuple[float, float]:
    """(long side, short side) of the enclosing rectangle, in metres.

    Compared against the known board size, this is a cheap and effective sanity
    check that RANSAC latched onto the board rather than a wall.
    """
    corners = rectangle_corners(fit)
    edges = [float(np.linalg.norm(corners[(i + 1) % 4] - corners[i])) for i in range(4)]
    side_a = (edges[0] + edges[2]) / 2.0
    side_b = (edges[1] + edges[3]) / 2.0
    return (max(side_a, side_b), min(side_a, side_b))
