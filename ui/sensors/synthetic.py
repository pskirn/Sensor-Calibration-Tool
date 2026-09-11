"""A simulated camera + LiDAR rig with a known ground-truth extrinsic.

This exists for three reasons:

1. **Anyone can try the tool.** Clone the repo, start the server, pick the
   "Simulated rig" sources, and the full live-capture workflow runs with no
   hardware and no ROS.
2. **The pipeline is testable against truth.** The extrinsic used to generate
   the data is known exactly, so detection, capture, CSV export and the C++
   solver can be checked end to end -- not merely for "did it converge" but for
   "did it converge to the right answer".
3. **Failure modes are reproducible.** Noise, board distance and orientation
   spread are all parameters, so the degenerate near-parallel-board case that
   `quality.normal_span` guards against can be produced deliberately.

The scene is a checkerboard held at a sequence of poses in front of the rig.
Both sources derive the current pose from wall-clock time, so they stay
consistent with each other without any explicit synchronisation.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from .base import CloudFrame, ImageFrame, ImageSource, PointCloudSource, SourceInfo

# Ground truth, LiDAR -> camera. The rotation is the conventional mapping from a
# LiDAR frame (x forward, y left, z up) to an optical frame (x right, y down,
# z forward), with a few degrees of mounting error mixed in so the solver has
# something non-trivial to recover.
_BASE_ROTATION = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
], dtype=np.float64)

_MOUNT_ERROR_RPY_DEG = (1.8, -2.5, 1.1)
_TRUE_TRANSLATION = np.array([0.045, -0.085, -0.120])   # metres

DEFAULT_K = np.array([
    [900.0, 0.0, 640.0],
    [0.0, 900.0, 360.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
DEFAULT_IMAGE_SIZE = (1280, 720)
DEFAULT_DIST = np.zeros(5, dtype=np.float64)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic X-Y-Z rotation from radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def ground_truth() -> Tuple[np.ndarray, np.ndarray]:
    """The true (R, t) mapping LiDAR frame -> camera frame."""
    R = _BASE_ROTATION @ _rpy_to_matrix(*np.radians(_MOUNT_ERROR_RPY_DEG))
    return R, _TRUE_TRANSLATION.copy()


@dataclass
class SceneConfig:
    board_cols: int = 7                 # internal corners
    board_rows: int = 5
    square_size: float = 0.08           # metres
    border_squares: float = 1.0         # panel margin, in squares
    pose_duration: float = 2.5          # seconds each pose is held
    num_poses: int = 24
    seed: int = 7
    range_m: Tuple[float, float] = (1.6, 3.2)
    max_tilt_deg: float = 32.0
    lidar_points_on_board: int = 2500
    lidar_noise_m: float = 0.006        # ~6 mm, typical of a mid-range scanner
    lidar_hz: float = 10.0
    camera_hz: float = 10.0
    image_size: Tuple[int, int] = DEFAULT_IMAGE_SIZE

    @property
    def panel_size(self) -> Tuple[float, float]:
        """Physical panel extents, in metres.

        Must stay consistent with `_render_board_texture`, which lays out
        `cols + 1` squares across plus `border_squares` on each side. If this
        formula and that layout disagree, the rendered squares stop being
        square, no rigid planar board can explain the image, and solvePnP
        returns a subtly rotated pose -- a failure that looks like a detector
        bug but is really a scene bug.
        """
        w = (self.board_cols + 1 + 2 * self.border_squares) * self.square_size
        h = (self.board_rows + 1 + 2 * self.border_squares) * self.square_size
        return (w, h)


class SyntheticScene:
    """Generates board poses and renders them for either sensor."""

    def __init__(self, config: Optional[SceneConfig] = None) -> None:
        self.config = config or SceneConfig()
        self.K = DEFAULT_K.copy()
        self.dist = DEFAULT_DIST.copy()
        self.R_lid_to_cam, self.t_lid_to_cam = ground_truth()
        self._poses = self._generate_poses()
        self._texture = self._render_board_texture()
        self._t0 = time.time()

    # --- scene definition -------------------------------------------------

    def _generate_poses(self):
        """A spread of board poses that is well-conditioned by construction.

        Orientations are drawn to cover tilt in both axes and rotation in the
        image plane, and positions sweep the frame, so `normal_span` comes out
        healthy. A deliberately degenerate scene can be made by shrinking
        `max_tilt_deg` toward zero.
        """
        rng = np.random.default_rng(self.config.seed)
        poses = []
        for i in range(self.config.num_poses):
            tilt = np.radians(self.config.max_tilt_deg)
            # Alternate the sign of each tilt axis so consecutive poses differ
            # a lot rather than drifting slowly through similar angles.
            sign_x = 1.0 if i % 2 == 0 else -1.0
            sign_y = 1.0 if (i // 2) % 2 == 0 else -1.0
            roll = sign_x * rng.uniform(0.35, 1.0) * tilt
            pitch = sign_y * rng.uniform(0.35, 1.0) * tilt
            yaw = rng.uniform(-0.6, 0.6) * tilt

            distance = rng.uniform(*self.config.range_m)
            # Keep the board inside the field of view with a margin.
            spread = 0.28 * distance
            centre = np.array([
                rng.uniform(-spread, spread),
                rng.uniform(-spread * 0.6, spread * 0.6),
                distance,
            ])
            R = _rpy_to_matrix(roll, pitch, yaw)
            # The board's own +Z must face the camera, or we would be rendering
            # its back and the LiDAR would see a surface the camera cannot.
            if R[2, 2] > 0:
                R = R @ np.diag([1.0, -1.0, -1.0])
            poses.append((R, centre))
        return poses

    def _render_board_texture(self) -> np.ndarray:
        """A checkerboard panel image, white border included."""
        cfg = self.config
        px_per_square = 60
        squares_x = cfg.board_cols + 1
        squares_y = cfg.board_rows + 1
        border_px = int(cfg.border_squares * px_per_square)

        inner = np.zeros((squares_y * px_per_square, squares_x * px_per_square),
                         dtype=np.uint8)
        for row in range(squares_y):
            for col in range(squares_x):
                if (row + col) % 2 == 0:
                    inner[row * px_per_square:(row + 1) * px_per_square,
                          col * px_per_square:(col + 1) * px_per_square] = 255

        panel = cv2.copyMakeBorder(inner, border_px, border_px, border_px, border_px,
                                   cv2.BORDER_CONSTANT, value=255)
        return cv2.cvtColor(panel, cv2.COLOR_GRAY2BGR)

    def pose_at(self, when: Optional[float] = None):
        """Which board pose is on screen at a given wall-clock time."""
        when = time.time() if when is None else when
        elapsed = when - self._t0
        idx = int(elapsed / self.config.pose_duration) % len(self._poses)
        return idx, self._poses[idx]

    # --- geometry ---------------------------------------------------------

    def panel_corners_cam(self, R: np.ndarray, centre: np.ndarray) -> np.ndarray:
        """The four physical panel corners in the camera frame."""
        w, h = self.config.panel_size
        local = np.array([
            [-w / 2, -h / 2, 0.0],
            [+w / 2, -h / 2, 0.0],
            [+w / 2, +h / 2, 0.0],
            [-w / 2, +h / 2, 0.0],
        ])
        return local @ R.T + centre

    # --- camera rendering -------------------------------------------------

    def render_image(self, when: Optional[float] = None) -> np.ndarray:
        """Render the board into a synthetic camera image."""
        width, height = self.config.image_size
        canvas = np.full((height, width, 3), 60, dtype=np.uint8)
        # A faint gradient stands in for scene texture, so the image does not
        # look like a bug and adaptive thresholding has something to work with.
        gradient = np.linspace(35, 85, width, dtype=np.uint8)
        canvas[:] = np.repeat(gradient[None, :, None], height, axis=0)

        _idx, (R, centre) = self.pose_at(when)
        corners_cam = self.panel_corners_cam(R, centre)
        if np.any(corners_cam[:, 2] < 0.2):
            return canvas   # board behind or too close to the camera

        projected, _ = cv2.projectPoints(
            corners_cam, np.zeros(3), np.zeros(3), self.K, self.dist
        )
        dst = projected.reshape(-1, 2).astype(np.float32)

        th, tw = self._texture.shape[:2]
        src = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]],
                       dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst)

        warped = cv2.warpPerspective(self._texture, H, (width, height))
        mask = cv2.warpPerspective(
            np.full((th, tw), 255, np.uint8), H, (width, height)
        )
        canvas[mask > 0] = warped[mask > 0]

        # Mild blur + sensor noise; sub-pixel corner refinement should still
        # comfortably beat a pixel of error on this.
        canvas = cv2.GaussianBlur(canvas, (3, 3), 0)
        noise = np.random.default_rng().normal(0, 2.0, canvas.shape)
        return np.clip(canvas.astype(np.float64) + noise, 0, 255).astype(np.uint8)

    # --- lidar rendering --------------------------------------------------

    def render_cloud(self, when: Optional[float] = None,
                     rng: Optional[np.random.Generator] = None) -> np.ndarray:
        """Generate a LiDAR sweep containing the board plus background."""
        cfg = self.config
        rng = rng or np.random.default_rng()
        _idx, (R, centre) = self.pose_at(when)

        # Board samples, in the camera frame first.
        w, h = cfg.panel_size
        n = cfg.lidar_points_on_board
        local = np.stack([
            rng.uniform(-w / 2, w / 2, n),
            rng.uniform(-h / 2, h / 2, n),
            np.zeros(n),
        ], axis=1)
        board_cam = local @ R.T + centre

        # Camera -> LiDAR is the inverse of the ground-truth extrinsic.
        R_cl = self.R_lid_to_cam.T
        t_cl = -R_cl @ self.t_lid_to_cam
        board_lid = board_cam @ R_cl.T + t_cl

        parts = [board_lid]

        # Background: a floor and a back wall, so RANSAC and clustering face the
        # same competing planes they will meet on a real robot.
        floor_n = 3000
        floor = np.stack([
            rng.uniform(0.0, 6.0, floor_n),
            rng.uniform(-3.0, 3.0, floor_n),
            np.full(floor_n, -1.25) + rng.normal(0, 0.01, floor_n),
        ], axis=1)
        parts.append(floor)

        wall_n = 2000
        wall = np.stack([
            np.full(wall_n, 5.0) + rng.normal(0, 0.01, wall_n),
            rng.uniform(-3.0, 3.0, wall_n),
            rng.uniform(-1.25, 2.0, wall_n),
        ], axis=1)
        parts.append(wall)

        cloud = np.concatenate(parts, axis=0)
        cloud += rng.normal(0, cfg.lidar_noise_m, cloud.shape)
        return cloud.astype(np.float32)


# --- sources --------------------------------------------------------------

class _TickingSource:
    """Shared background loop that re-renders the scene at a fixed rate."""

    def __init__(self, rate_hz: float, tick) -> None:
        self._interval = 1.0 / max(rate_hz, 0.1)
        self._tick = tick
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._tick()
            except Exception:
                pass
            remaining = self._interval - (time.monotonic() - started)
            if remaining > 0 and self._stop.wait(remaining):
                return


# A single scene instance is shared so the camera and LiDAR sources always agree
# on which board pose is currently being shown.
_SCENE: Optional[SyntheticScene] = None
_SCENE_LOCK = threading.Lock()


def shared_scene() -> SyntheticScene:
    global _SCENE
    with _SCENE_LOCK:
        if _SCENE is None:
            _SCENE = SyntheticScene()
        return _SCENE


def reset_scene(config: Optional[SceneConfig] = None) -> SyntheticScene:
    global _SCENE
    with _SCENE_LOCK:
        _SCENE = SyntheticScene(config)
        return _SCENE


class SyntheticCameraSource(ImageSource):
    def __init__(self) -> None:
        super().__init__(SourceInfo(
            kind="camera", id="sim:camera", label="Simulated camera",
            backend="synthetic", detail="1280x720 pinhole, known intrinsics",
        ))
        self.scene = shared_scene()
        self._ticker = _TickingSource(self.scene.config.camera_hz, self._tick)

    def _tick(self) -> None:
        self._publish(ImageFrame(
            stamp=time.time(), image=self.scene.render_image(), frame_id="sim_camera",
        ))

    def start(self) -> None:
        self._running = True
        self._ticker.start()

    def stop(self) -> None:
        self._running = False
        self._ticker.stop()


class SyntheticLidarSource(PointCloudSource):
    def __init__(self, accumulation_window: float = 0.0) -> None:
        super().__init__(
            SourceInfo(
                kind="lidar", id="sim:lidar", label="Simulated LiDAR",
                backend="synthetic", detail="board + floor + wall, 6 mm noise",
            ),
            accumulation_window=accumulation_window,
        )
        self.scene = shared_scene()
        self._rng = np.random.default_rng()
        self._ticker = _TickingSource(self.scene.config.lidar_hz, self._tick)

    def _tick(self) -> None:
        points = self.scene.render_cloud(rng=self._rng)
        self._publish(CloudFrame(
            stamp=time.time(), points=points, intensity=None, frame_id="sim_lidar",
        ))

    def start(self) -> None:
        self._running = True
        self._ticker.start()

    def stop(self) -> None:
        self._running = False
        self._ticker.stop()


def source_infos() -> list[SourceInfo]:
    """Advertise the simulated rig in the source picker."""
    return [
        SourceInfo(kind="lidar", id="sim:lidar", label="Simulated LiDAR",
                   backend="synthetic", detail="no hardware needed"),
        SourceInfo(kind="camera", id="sim:camera", label="Simulated camera",
                   backend="synthetic", detail="no hardware needed"),
    ]


def default_config() -> dict:
    """Board, ROI and intrinsics that match the simulated scene.

    Lets the UI pre-fill every field for the demo, so the first run needs no
    setup at all.
    """
    scene = shared_scene()
    cfg = scene.config
    return {
        "board": {
            "cols": cfg.board_cols,
            "rows": cfg.board_rows,
            "square_size": cfg.square_size,
            "board_width": cfg.panel_size[0],
            "board_height": cfg.panel_size[1],
        },
        # Excludes the floor (z = -1.25) and the back wall (x = 5.0), leaving the
        # board as the dominant plane inside the box.
        "roi_min": [0.5, -2.0, -1.0],
        "roi_max": [4.0, 2.0, 2.0],
        "camera_matrix": scene.K.tolist(),
        "distortion_coefficients": scene.dist.tolist(),
        "image_width": cfg.image_size[0],
        "image_height": cfg.image_size[1],
        "accumulation_window": 0.0,
    }
