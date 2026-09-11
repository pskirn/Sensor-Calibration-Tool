"""Live capture session: sources + detection + captured poses + CSV export.

A session owns one camera source and one LiDAR source, runs board detection on
their latest frames, and accumulates approved poses. Its output is a `poses.csv`
in exactly the ACFR-style layout that `src/poses_csv_loader.cpp` already reads,
so live capture feeds the same validated C++ solver as the offline path rather
than duplicating any of the maths.

Normal orientation
------------------
Both sensors' normals are oriented to give `d = n·centroid > 0`. The reference
ACFR dataset uses the opposite sign (normals pointing back at the sensor), and
both are equally correct: negating a plane's normal *and* its distance negates
each residual without changing any magnitude, so the solve is unaffected. We
pick the positive convention because it also satisfies `PlanePair::isValid()`.
What must never happen is flipping one sensor without the other -- that would
drive the normal residual toward a 180-degree-wrong rotation.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from ..sensors import registry
from ..sensors.base import ImageSource, PointCloudSource
from . import detect, quality
from .detect import BoardSpec, CameraDetection, LidarDetection
from .quality import PoseSample

# Detection is far more expensive than fetching a frame, so results are cached
# for this long and reused across preview requests.
_DETECT_CACHE_S = 0.20

# Capture runs detection twice, this far apart, and refuses the pose if the
# board moved in between. See `_stability_failure` for why this matters.
_STABILITY_DELAY_S = 0.30
_MAX_NORMAL_DRIFT_DEG = 1.5
_MAX_CENTROID_DRIFT_M = 0.02
# How far apart the camera frame and the LiDAR frame may be stamped. The
# accumulation window is added because an accumulated cloud legitimately spans
# that much time.
_MAX_SENSOR_SKEW_S = 0.30


@dataclass
class LiveConfig:
    camera_source: str
    lidar_source: str
    board: BoardSpec
    roi_min: List[float]
    roi_max: List[float]
    K: np.ndarray
    dist: np.ndarray
    accumulation_window: float = 1.0
    ransac_threshold: float = 0.02
    replay_speed: float = 1.0
    preview_max_points: int = 40000


@dataclass
class LiveState:
    """Snapshot of what the session currently sees, serialised straight to JSON."""

    camera_ok: bool = False
    lidar_ok: bool = False
    camera_message: str = ""
    lidar_message: str = ""
    reprojection_error: float = 0.0
    lidar_points: int = 0
    lidar_rms: float = 0.0
    panel_size: tuple = (0.0, 0.0)
    cloud_points_total: int = 0
    can_capture: bool = False
    novelty_deg: float = 180.0
    reasons: List[str] = field(default_factory=list)
    hints: List[str] = field(default_factory=list)


class LiveSession:
    """Owns the live sources and the growing set of captured poses."""

    def __init__(self, config: LiveConfig) -> None:
        self.config = config
        self.poses: List[PoseSample] = []
        self.image_size = (0, 0)
        self._lock = threading.Lock()
        self._camera: Optional[ImageSource] = None
        self._lidar: Optional[PointCloudSource] = None
        self._started_at = 0.0

        self._cache_time = 0.0
        self._cache: Optional[tuple] = None

    # --- lifecycle --------------------------------------------------------

    def start(self) -> None:
        cfg = self.config
        self._camera = registry.open_image_source(cfg.camera_source, speed=cfg.replay_speed)
        self._lidar = registry.open_cloud_source(
            cfg.lidar_source,
            accumulation_window=cfg.accumulation_window,
            speed=cfg.replay_speed,
        )
        self._camera.start()
        self._lidar.start()
        self._started_at = time.time()

    def stop(self) -> None:
        for source in (self._camera, self._lidar):
            if source is not None:
                try:
                    source.stop()
                except Exception:
                    pass
        self._camera = self._lidar = None

    @property
    def running(self) -> bool:
        return self._camera is not None and self._lidar is not None

    # --- detection --------------------------------------------------------

    def _detect(self, full_resolution: bool = False):
        """Run both detectors on the newest frames, with a short result cache.

        `full_resolution` skips the preview subsampling; used at capture time so
        the stored pose is as accurate as the data allows.
        """
        now = time.time()
        if not full_resolution and self._cache and (now - self._cache_time) < _DETECT_CACHE_S:
            return self._cache

        cfg = self.config
        image_frame = self._camera.latest() if self._camera else None
        cloud_frame = self._lidar.accumulate() if self._lidar else None

        cam_det = CameraDetection(False, message="Waiting for camera frames")
        if image_frame is not None:
            self.image_size = image_frame.size
            cam_det = detect.detect_camera_board(
                image_frame.image, cfg.board, cfg.K, cfg.dist,
                detect_width=0 if full_resolution else 1024,
            )

        lid_det = LidarDetection(False, message="Waiting for LiDAR frames")
        total_points = 0
        if cloud_frame is not None:
            points = cloud_frame.points
            total_points = len(points)
            # Preview subsamples so detection keeps up with the stream; capture
            # uses every point because accuracy matters more than latency there.
            if not full_resolution and len(points) > cfg.preview_max_points:
                idx = np.random.default_rng(0).choice(
                    len(points), cfg.preview_max_points, replace=False
                )
                points = points[idx]
            lid_det = detect.detect_lidar_board(
                points, cfg.board, cfg.roi_min, cfg.roi_max,
                ransac_threshold=cfg.ransac_threshold,
            )

        result = (image_frame, cloud_frame, cam_det, lid_det, total_points)
        if not full_resolution:
            self._cache, self._cache_time = result, now
        return result

    def state(self) -> LiveState:
        """Current detection status plus the live capture verdict."""
        with self._lock:
            _img, _cloud, cam_det, lid_det, total = self._detect()
            assessment = quality.assess_live_pose(
                cam_ok=cam_det.ok,
                lid_ok=lid_det.ok,
                cam_normal=cam_det.normal,
                captured=self.poses,
                cam_message=cam_det.message,
                lid_message=lid_det.message,
                reprojection_error=cam_det.reprojection_error,
            )
            return LiveState(
                camera_ok=cam_det.ok,
                lidar_ok=lid_det.ok,
                camera_message=cam_det.message,
                lidar_message=lid_det.message,
                reprojection_error=round(cam_det.reprojection_error, 3),
                lidar_points=lid_det.num_inliers,
                lidar_rms=round(lid_det.rms_error, 4),
                panel_size=tuple(round(v, 3) for v in lid_det.panel_size),
                cloud_points_total=total,
                can_capture=assessment.can_capture,
                novelty_deg=round(assessment.novelty_deg, 1),
                reasons=assessment.reasons,
                hints=assessment.hints,
            )

    def preview_jpeg(self, quality_pct: int = 70, max_width: int = 960) -> Optional[bytes]:
        """Annotated camera frame as JPEG bytes, for the MJPEG preview stream."""
        with self._lock:
            image_frame, _cloud, cam_det, _lid, _total = self._detect()
            if image_frame is None:
                return None
            canvas = detect.draw_camera_overlay(
                image_frame.image, cam_det, self.config.board
            )

        if canvas.shape[1] > max_width:
            scale = max_width / canvas.shape[1]
            canvas = cv2.resize(canvas, None, fx=scale, fy=scale,
                                interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", canvas,
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality_pct])
        return buf.tobytes() if ok else None

    def cloud_preview(self, max_points: int = 6000) -> dict:
        """Subsampled cloud + detected board, for the 3D viewer."""
        with self._lock:
            _img, cloud_frame, _cam, lid_det, total = self._detect()
            if cloud_frame is None:
                return {"points": [], "inliers": [], "corners": [], "total": 0}

            points = cloud_frame.points
            if len(points) > max_points:
                idx = np.random.default_rng(1).choice(len(points), max_points, replace=False)
                points = points[idx]

            inliers = lid_det.inliers if lid_det.inliers is not None else np.empty((0, 3))
            if len(inliers) > max_points:
                idx = np.random.default_rng(2).choice(len(inliers), max_points, replace=False)
                inliers = inliers[idx]

            return {
                "points": np.asarray(points, dtype=np.float32).round(3).tolist(),
                "inliers": np.asarray(inliers, dtype=np.float32).round(3).tolist(),
                "corners": ([] if lid_det.corners_3d is None
                            else np.asarray(lid_det.corners_3d).round(4).tolist()),
                "roi_min": self.config.roi_min,
                "roi_max": self.config.roi_max,
                "total": int(total),
            }

    # --- capture ----------------------------------------------------------

    @staticmethod
    def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
        cos = float(np.clip(abs(np.dot(a, b)), -1.0, 1.0))
        return float(np.degrees(np.arccos(cos)))

    def _stability_failure(self, first, second) -> Optional[str]:
        """Reject a capture taken while the board was still moving.

        The camera frame and the LiDAR frame are grabbed independently and are
        never perfectly simultaneous. While the board is stationary that does
        not matter. But if it is still moving, the two sensors can describe
        *different board positions*, and the solver has no way to know -- it
        will faithfully fit a rigid transform to an inconsistent pair, and the
        result is silently wrong rather than merely noisy.

        A single bad pair is enough to wreck the answer, so we detect twice a
        moment apart and require the board to have held still.
        """
        cam_a, lid_a = first
        cam_b, lid_b = second

        if not (cam_b.ok and lid_b.ok):
            return "Detection was not stable across the capture — hold the board steady"

        cam_drift = self._angle_deg(cam_a.normal, cam_b.normal)
        lid_drift = self._angle_deg(lid_a.normal, lid_b.normal)
        if max(cam_drift, lid_drift) > _MAX_NORMAL_DRIFT_DEG:
            return (f"The board is still moving ({max(cam_drift, lid_drift):.1f}° of "
                    f"rotation during capture) — hold it steady, then capture")

        shift = max(
            float(np.linalg.norm(np.asarray(cam_a.centroid) - np.asarray(cam_b.centroid))),
            float(np.linalg.norm(np.asarray(lid_a.centroid) - np.asarray(lid_b.centroid))),
        )
        if shift > _MAX_CENTROID_DRIFT_M:
            return (f"The board is still moving ({shift * 1000:.0f} mm of travel during "
                    f"capture) — hold it steady, then capture")
        return None

    def _skew_failure(self, image_frame, cloud_frame) -> Optional[str]:
        """Reject a pair whose two frames are stamped too far apart."""
        if image_frame is None or cloud_frame is None:
            return None
        skew = abs(image_frame.stamp - cloud_frame.stamp)
        budget = _MAX_SENSOR_SKEW_S + max(0.0, self.config.accumulation_window)
        if skew > budget:
            return (f"Camera and LiDAR frames are {skew:.2f}s apart (budget "
                    f"{budget:.2f}s) — they may show the board in different places")
        return None

    def capture(self, force: bool = False) -> dict:
        """Detect at full fidelity and, if acceptable and stable, store the pose."""
        # Detection is done outside the lock: it takes a few hundred milliseconds
        # and holding the lock would stall the preview stream and status polling.
        img_a, cloud_a, cam_a, lid_a, _ = self._detect(full_resolution=True)

        assessment = quality.assess_live_pose(
            cam_ok=cam_a.ok, lid_ok=lid_a.ok, cam_normal=cam_a.normal,
            captured=self.poses, cam_message=cam_a.message,
            lid_message=lid_a.message,
            reprojection_error=cam_a.reprojection_error,
        )
        if not assessment.can_capture and not force:
            return {"captured": False, "reasons": assessment.reasons}

        if not (cam_a.ok and lid_a.ok):
            # `force` cannot conjure a detection that does not exist.
            return {
                "captured": False,
                "reasons": assessment.reasons or ["Board not detected by both sensors"],
            }

        # Second look, to confirm the board was stationary. This is a
        # correctness gate, not a nicety, so `force` does not bypass it.
        time.sleep(_STABILITY_DELAY_S)
        image_frame, cloud_frame, cam_det, lid_det, _total = self._detect(full_resolution=True)

        problem = (self._stability_failure((cam_a, lid_a), (cam_det, lid_det))
                   or self._skew_failure(image_frame, cloud_frame))
        if problem:
            return {"captured": False, "reasons": [problem]}

        with self._lock:
            image_point = None
            if cam_det.corners_2d is not None:
                image_point = cam_det.corners_2d.mean(axis=0).tolist()

            sample = PoseSample(
                index=len(self.poses),
                cam_normal=cam_det.normal,
                cam_distance=cam_det.distance,
                cam_centroid=cam_det.centroid,
                lid_normal=lid_det.normal,
                lid_distance=lid_det.distance,
                lid_centroid=lid_det.centroid,
                image_point=image_point,
                reprojection_error=cam_det.reprojection_error,
                lidar_rms=lid_det.rms_error,
                lidar_points=lid_det.num_inliers,
            )
            # Corners are needed for the CSV but are not part of the quality
            # metrics, so they ride along as attributes.
            sample.cam_corners = cam_det.corners_3d      # type: ignore[attr-defined]
            sample.lid_corners = lid_det.corners_3d      # type: ignore[attr-defined]
            self.poses.append(sample)

            return {
                "captured": True,
                "index": sample.index,
                "novelty_deg": round(assessment.novelty_deg, 1),
                "hints": assessment.hints,
            }

    def drop(self, index: int) -> bool:
        """Remove a captured pose and renumber the rest."""
        with self._lock:
            if not (0 <= index < len(self.poses)):
                return False
            self.poses.pop(index)
            for i, pose in enumerate(self.poses):
                pose.index = i
            return True

    def clear(self) -> None:
        with self._lock:
            self.poses.clear()

    def readiness(self) -> quality.SessionReadiness:
        with self._lock:
            return quality.assess_session(self.poses, self.image_size)

    def captured_summary(self) -> List[dict]:
        with self._lock:
            return [
                {
                    "index": p.index,
                    "cam_distance": round(float(p.cam_distance), 3),
                    "lid_distance": round(float(p.lid_distance), 3),
                    "reprojection_error": round(float(p.reprojection_error), 3),
                    "lidar_rms": round(float(p.lidar_rms), 4),
                    "lidar_points": int(p.lidar_points),
                    "image_point": p.image_point,
                    "cam_normal": [round(float(v), 4) for v in p.cam_normal],
                }
                for p in self.poses
            ]

    # --- export -----------------------------------------------------------

    def write_poses_csv(self, path: Path) -> int:
        """Write captured poses in the 19-line ACFR layout the C++ loader reads.

        Every line must parse as an `x,y,z` triple -- including the six metadata
        lines, which the loader skips but still requires to be well formed.
        Positions are millimetres; normals are unit vectors written unscaled.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        M_TO_MM = 1000.0

        def triple(vec) -> str:
            v = np.asarray(vec, dtype=float).ravel()
            return f"{v[0]:.6f},{v[1]:.6f},{v[2]:.6f}"

        with self._lock:
            lines: List[str] = []
            for pose in self.poses:
                cam_corners = getattr(pose, "cam_corners", None)
                lid_corners = getattr(pose, "lid_corners", None)
                if cam_corners is None or lid_corners is None:
                    continue

                lines.append(triple(np.asarray(pose.cam_centroid) * M_TO_MM))
                lines.append(triple(pose.cam_normal))
                for corner in np.asarray(cam_corners)[:4]:
                    lines.append(triple(np.asarray(corner) * M_TO_MM))

                lines.append(triple(np.asarray(pose.lid_centroid) * M_TO_MM))
                lines.append(triple(pose.lid_normal))
                for corner in np.asarray(lid_corners)[:4]:
                    lines.append(triple(np.asarray(corner) * M_TO_MM))

                # Metadata rows 12-17: unused by the loader, but kept meaningful
                # so the file is still readable by a human debugging a session.
                board_long, board_short = self.config.board.expected_panel
                lines.append(triple([board_long * M_TO_MM, board_short * M_TO_MM, 0]))
                lines.append(triple([board_long * M_TO_MM, board_short * M_TO_MM, 0]))
                lines.append(triple([pose.lidar_points, 0, 0]))
                lines.append(triple([pose.lidar_rms * M_TO_MM, 0, 0]))
                lines.append(triple([pose.reprojection_error, 0, 0]))
                lines.append(triple([pose.cam_distance, pose.lid_distance, 0]))
                lines.append(triple([pose.index + 1, 0, 0]))

            path.write_text("\n".join(lines) + ("\n" if lines else ""))
            return len(self.poses)
