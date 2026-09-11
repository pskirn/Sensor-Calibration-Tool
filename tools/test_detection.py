#!/usr/bin/env python3
"""Fast checks for the Python layer: decoding, geometry, detection, quality.

Needs no C++ toolchain and no ROS, so it runs in seconds and is the first thing
CI executes. The end-to-end accuracy check lives in `validate_live_pipeline.py`,
which does need the built solver.

    python tools/test_detection.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from ui.live import detect, plane_fit, quality          # noqa: E402
from ui.live.detect import BoardSpec                    # noqa: E402
from ui.sensors import pointcloud2, synthetic           # noqa: E402

_failures: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    status = "ok  " if condition else "FAIL"
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))
    if not condition:
        _failures.append(name)


# --- PointCloud2 decoding -------------------------------------------------

class _Field:
    def __init__(self, name, offset, datatype, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class _FakeCloud:
    """Mimics the Livox Mid-360 layout: mixed widths, 22-byte unpadded record."""

    def __init__(self, points):
        self.fields = [
            _Field("x", 0, 7), _Field("y", 4, 7), _Field("z", 8, 7),
            _Field("t", 12, 6), _Field("intensity", 16, 7),
            _Field("tag", 20, 2), _Field("line", 21, 2),
        ]
        self.point_step = 22
        self.is_bigendian = False
        record = np.zeros(len(points), dtype=np.dtype({
            "names": ["x", "y", "z", "t", "intensity", "tag", "line"],
            "formats": ["<f4", "<f4", "<f4", "<u4", "<f4", "u1", "u1"],
            "offsets": [0, 4, 8, 12, 16, 20, 21],
            "itemsize": 22,
        }))
        record["x"], record["y"], record["z"] = points[:, 0], points[:, 1], points[:, 2]
        record["intensity"] = np.arange(len(points), dtype=np.float32)
        self.data = record.tobytes()


def test_pointcloud2():
    print("PointCloud2 decoding")
    pts = np.random.default_rng(0).normal(size=(500, 3)).astype(np.float32)
    xyz, intensity = pointcloud2.decode(_FakeCloud(pts))
    check("mixed-width 22-byte record decodes", xyz.shape == (500, 3), str(xyz.shape))
    check("xyz values round-trip", np.allclose(xyz, pts, atol=1e-5))
    check("intensity recovered", intensity is not None and len(intensity) == 500)

    # Non-finite points must be dropped, or every downstream centroid is NaN.
    dirty = pts.copy()
    dirty[:10] = np.nan
    xyz2, _ = pointcloud2.decode(_FakeCloud(dirty))
    check("NaN points dropped", len(xyz2) == 490, f"{len(xyz2)} of 500 kept")


# --- plane fitting --------------------------------------------------------

def test_plane_fit():
    print("Plane fitting")
    rng = np.random.default_rng(1)
    normal = np.array([0.3, -0.5, 0.81]); normal /= np.linalg.norm(normal)
    d = 2.0
    u, v = plane_fit.plane_basis(normal)
    a = rng.uniform(-0.4, 0.4, 3000)
    b = rng.uniform(-0.3, 0.3, 3000)
    board = d * normal + a[:, None] * u + b[:, None] * v
    board += rng.normal(0, 0.004, board.shape)
    # Add off-plane clutter RANSAC must reject.
    clutter = rng.uniform(-1, 1, (1500, 3)) * 2.0
    cloud = np.vstack([board, clutter]).astype(np.float32)

    fit = plane_fit.fit_plane_ransac(cloud, threshold=0.02, max_iterations=400)
    check("plane found among clutter", fit is not None)
    if fit is None:
        return
    angle = np.degrees(np.arccos(np.clip(abs(fit.normal @ normal), -1, 1)))
    check("normal accurate", angle < 1.0, f"{angle:.3f} deg")
    check("distance accurate", abs(fit.distance - d) < 0.02, f"{fit.distance:.4f} vs {d}")
    check("d > 0 convention held", fit.distance > 0)

    corners = plane_fit.rectangle_corners(fit)
    on_plane = np.abs(corners @ fit.normal - fit.distance)
    check("corners lie exactly on the plane", float(on_plane.max()) < 1e-9,
          f"max offset {on_plane.max():.2e} m")

    # Footprint is only meaningful after clustering. RANSAC returns inliers of
    # the *infinite* plane, so any clutter that happens to fall within the
    # threshold stretches the enclosing rectangle far beyond the board. The real
    # detector clusters before measuring, and so must this check.
    clustered = plane_fit.largest_cluster(fit.inliers, voxel=0.06)
    refit = plane_fit.refine_plane(clustered)
    long_side, short_side = plane_fit.rectangle_dimensions(refit)
    check("footprint measured after clustering",
          abs(long_side - 0.8) < 0.1 and abs(short_side - 0.6) < 0.1,
          f"{long_side:.3f} x {short_side:.3f} (expected ~0.8 x 0.6)")

    unclustered_long, _ = plane_fit.rectangle_dimensions(fit)
    check("clustering is what makes the footprint meaningful",
          unclustered_long > long_side + 0.2,
          f"{unclustered_long:.2f} m before vs {long_side:.2f} m after")


def test_clustering():
    print("Clustering")
    rng = np.random.default_rng(2)
    big = rng.normal(0, 0.1, (2000, 3))
    far = rng.normal(0, 0.1, (300, 3)) + np.array([5.0, 0, 0])
    kept = plane_fit.largest_cluster(np.vstack([big, far]).astype(np.float32), voxel=0.1)
    check("largest blob kept, distant one dropped",
          1500 < len(kept) < 2200, f"{len(kept)} points kept of 2300")


# --- detection ------------------------------------------------------------

def test_detection():
    print("Board detection on the simulated rig")
    scene = synthetic.reset_scene()
    cfg = synthetic.default_config()
    spec = BoardSpec(cfg["board"]["cols"], cfg["board"]["rows"],
                     cfg["board"]["square_size"],
                     cfg["board"]["board_width"], cfg["board"]["board_height"])
    K = np.array(cfg["camera_matrix"])
    D = np.array(cfg["distortion_coefficients"])
    R_true, _ = synthetic.ground_truth()

    normal_errors, agreements = [], []
    for k in range(4):
        when = scene._t0 + k * scene.config.pose_duration + 0.1
        _idx, (R, centre) = scene.pose_at(when)
        true_n = R[:, 2] / np.linalg.norm(R[:, 2])
        if float(true_n @ centre) < 0:
            true_n = -true_n

        cam = detect.detect_camera_board(scene.render_image(when), spec, K, D)
        lid = detect.detect_lidar_board(scene.render_cloud(when), spec,
                                        cfg["roi_min"], cfg["roi_max"])
        if not (cam.ok and lid.ok):
            check(f"pose {k} detected by both sensors", False,
                  f"cam={cam.message} lid={lid.message}")
            continue
        normal_errors.append(
            np.degrees(np.arccos(np.clip(abs(cam.normal @ true_n), -1, 1))))
        agreements.append(
            np.degrees(np.arccos(np.clip(abs((R_true @ lid.normal) @ cam.normal), -1, 1))))

    check("all poses detected", len(normal_errors) == 4, f"{len(normal_errors)}/4")
    if normal_errors:
        check("camera normal accurate", max(normal_errors) < 0.5,
              f"max {max(normal_errors):.3f} deg")
        # The strongest single check: both sensors' planes must coincide once the
        # true extrinsic is applied. It catches sign, scale and frame errors that
        # each sensor alone would not reveal.
        check("camera and LiDAR planes agree under true extrinsic",
              max(agreements) < 0.5, f"max {max(agreements):.3f} deg")


def test_wall_rejection():
    print("Wall rejection")
    rng = np.random.default_rng(3)
    spec = BoardSpec(7, 5, 0.08, 0.8, 0.64)
    # A big flat wall inside the ROI: fits a plane perfectly, wrong size.
    wall = np.stack([
        np.full(4000, 2.0) + rng.normal(0, 0.005, 4000),
        rng.uniform(-1.5, 1.5, 4000),
        rng.uniform(-0.9, 1.5, 4000),
    ], axis=1).astype(np.float32)
    result = detect.detect_lidar_board(wall, spec, [0.5, -2, -1], [4, 2, 2])
    check("oversized plane rejected", not result.ok, result.message[:70])


# --- quality metrics ------------------------------------------------------

def test_quality():
    print("Quality metrics")
    parallel = [np.array([0.0, 0.0, 1.0]) for _ in range(10)]
    check("near-parallel normals score ~0", quality.normal_span(parallel) < 0.02,
          f"{quality.normal_span(parallel):.4f}")

    spread = [np.array(v) / np.linalg.norm(v) for v in
              ([1, 0, 0.4], [0, 1, 0.4], [0, 0, 1], [-1, 0.3, 0.5], [0.4, -1, 0.6])]
    check("well-spread normals score high", quality.normal_span(spread) > 0.3,
          f"{quality.normal_span(spread):.4f}")

    samples = [quality.PoseSample(i, n, 2.0, np.array([0, 0, 2.0]), n, 2.0,
                                  np.array([0, 0, 2.0]), image_point=[640, 360])
               for i, n in enumerate(parallel)]
    verdict = quality.assess_session(samples, (1280, 720))
    check("degenerate session blocked", not verdict.ready,
          verdict.blocking[0][:60] if verdict.blocking else "")

    check("novelty of a repeated orientation is ~0",
          quality.angular_novelty(np.array([0, 0, 1.0]), parallel) < 1.0)
    check("novelty of a fresh orientation is large",
          quality.angular_novelty(np.array([1.0, 0, 0]), parallel) > 45.0)


def test_capture_stability_gate():
    """A pose captured while the board moves must be refused.

    Regression test. The camera frame and the LiDAR frame are grabbed
    independently; if the board is still moving they can describe *different*
    board positions, and the solver will fit a rigid transform to that
    inconsistent pair without complaint. A handful of such pairs moved a
    recovered translation by ~200 mm and flipped two of its signs, while the
    residuals still looked plausible.
    """
    print("Capture stability gate")
    from ui.live.session import LiveConfig, LiveSession

    session = LiveSession(LiveConfig(
        camera_source="sim:camera", lidar_source="sim:lidar",
        board=BoardSpec(7, 5, 0.08), roi_min=[0, -2, -1], roi_max=[4, 2, 2],
        K=np.eye(3), dist=np.zeros(5), accumulation_window=0.0,
    ))

    def det(normal, centroid):
        d = detect.CameraDetection(True, normal=np.array(normal, dtype=float),
                                   centroid=np.array(centroid, dtype=float))
        lid = detect.LidarDetection(True, normal=np.array(normal, dtype=float),
                                    centroid=np.array(centroid, dtype=float))
        return d, lid

    still = det([0, 0, 1.0], [0, 0, 2.0])
    check("stationary board accepted",
          session._stability_failure(still, det([0, 0, 1.0], [0, 0, 2.0])) is None)

    rotating = det([0.09, 0, 0.996], [0, 0, 2.0])   # ~5 degrees of drift
    problem = session._stability_failure(still, rotating)
    check("rotating board refused", problem is not None,
          (problem or "")[:60])

    translating = det([0, 0, 1.0], [0.0, 0.08, 2.0])   # 80 mm of travel
    problem = session._stability_failure(still, translating)
    check("translating board refused", problem is not None,
          (problem or "")[:60])

    # Sub-threshold jitter is normal sensor noise and must not block capture.
    jitter = det([0.005, 0, 0.99999], [0.0, 0.003, 2.0])
    check("small jitter still accepted",
          session._stability_failure(still, jitter) is None)

    class _Frame:
        def __init__(self, stamp): self.stamp = stamp

    check("large camera/LiDAR time skew refused",
          session._skew_failure(_Frame(100.0), _Frame(101.5)) is not None)
    check("small time skew accepted",
          session._skew_failure(_Frame(100.0), _Frame(100.05)) is None)


def main() -> int:
    for test in (test_pointcloud2, test_plane_fit, test_clustering,
                 test_detection, test_wall_rejection, test_quality,
                 test_capture_stability_gate):
        test()
        print()
    if _failures:
        print(f"{len(_failures)} check(s) FAILED: {', '.join(_failures)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
