#!/usr/bin/env python3
"""End-to-end self-test: simulated rig -> live capture -> C++ solver -> truth.

Runs the entire live workflow against the synthetic sensor rig, whose extrinsic
is known exactly, then compares what the Ceres solver recovered against it. This
checks the parts the ACFR dataset cannot: board detection in both sensors, the
capture gating, and the `poses.csv` writer -- and it checks them for *accuracy*,
not merely for "it ran".

    python tools/validate_live_pipeline.py [--poses 14] [--tolerance-deg 0.5]

Exits non-zero if the recovered extrinsic misses the tolerance, so it is usable
as a CI gate.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from ui.live.detect import BoardSpec                      # noqa: E402
from ui.live.session import LiveConfig, LiveSession       # noqa: E402
from ui.sensors import synthetic                          # noqa: E402

BINARY = PROJECT_ROOT / "build" / "camera_lidar_calibration"
PARAMS = PROJECT_ROOT / "config" / "params.yaml"
RESULT = PROJECT_ROOT / "results" / "calibration.yaml"
OUT_CSV = PROJECT_ROOT / "data" / "synthetic" / "poses.csv"


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotations, in degrees."""
    R = R_a.T @ R_b
    cos = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def capture_session(num_poses: int, verbose: bool = True) -> LiveSession:
    """Drive the simulated rig and capture a spread of board poses."""
    scene = synthetic.reset_scene(synthetic.SceneConfig(pose_duration=1.5, num_poses=30))
    defaults = synthetic.default_config()
    board = defaults["board"]

    config = LiveConfig(
        camera_source="sim:camera",
        lidar_source="sim:lidar",
        board=BoardSpec(board["cols"], board["rows"], board["square_size"],
                        board["board_width"], board["board_height"]),
        roi_min=defaults["roi_min"],
        roi_max=defaults["roi_max"],
        K=np.array(defaults["camera_matrix"]),
        dist=np.array(defaults["distortion_coefficients"]),
        accumulation_window=0.0,
    )

    session = LiveSession(config)
    session.start()
    try:
        session._camera.wait_for_frame(timeout=10)
        session._lidar.wait_for_frame(timeout=10)

        captured, last_index, deadline = 0, -1, time.time() + 180
        while captured < num_poses and time.time() < deadline:
            index, _pose = scene.pose_at()
            if index == last_index:
                time.sleep(0.05)
                continue
            # Let both sources publish a frame from the *new* pose before
            # capturing. Without this settle time a capture straddling a pose
            # change pairs a camera frame from one board orientation with a
            # LiDAR frame from the next -- which is exactly the cross-sensor
            # sync error this tool should not be silently injecting.
            time.sleep(0.45)
            if scene.pose_at()[0] != index:
                continue

            outcome = session.capture()
            if outcome["captured"]:
                captured += 1
                last_index = index
                if verbose:
                    print(f"  captured pose {captured}/{num_poses} "
                          f"(scene pose {index}, novelty {outcome['novelty_deg']:.0f} deg)")
            else:
                last_index = index
                if verbose:
                    print(f"  skipped scene pose {index}: {'; '.join(outcome['reasons'])}")
    finally:
        session.stop()
    return session


def run_solver(poses_csv: Path) -> dict:
    """Point params.yaml at our CSV, run the C++ binary, restore params."""
    if not BINARY.exists():
        raise SystemExit(f"Binary not found at {BINARY}. Build with: cmake --build build")

    original = PARAMS.read_text()
    try:
        rel = poses_csv.relative_to(PROJECT_ROOT)
        patched = re.sub(r'^(\s*)poses_csv\s*:.*$',
                         lambda m: f'{m.group(1)}poses_csv: "{rel}"',
                         original, count=1, flags=re.MULTILINE)
        PARAMS.write_text(patched)

        proc = subprocess.run([str(BINARY)], cwd=str(BINARY.parent),
                              capture_output=True, text=True, timeout=180)
        if proc.returncode != 0:
            raise SystemExit(f"Solver failed:\n{proc.stdout}\n{proc.stderr}")
    finally:
        PARAMS.write_text(original)

    with RESULT.open() as fh:
        return yaml.safe_load(fh)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=int, default=14)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--tolerance-mm", type=float, default=20.0)
    args = parser.parse_args()

    print(f"Capturing {args.poses} poses from the simulated rig...")
    session = capture_session(args.poses)

    readiness = session.readiness()
    print(f"\nSession readiness: ready={readiness.ready} poses={readiness.num_poses} "
          f"normal_span={readiness.normal_span:.3f} coverage={readiness.coverage:.2f}")
    for item in readiness.blocking:
        print(f"  BLOCKING: {item}")
    for item in readiness.advice:
        print(f"  advice: {item}")

    written = session.write_poses_csv(OUT_CSV)
    print(f"\nWrote {written} poses -> {OUT_CSV.relative_to(PROJECT_ROOT)}")

    result = run_solver(OUT_CSV)
    solved = result["lidar_to_camera"]
    R_solved = np.array(solved["rotation_matrix"], dtype=float).reshape(3, 3)
    t_solved = np.array(solved["translation_m"], dtype=float).ravel()

    R_true, t_true = synthetic.ground_truth()
    rot_err = rotation_angle_deg(R_true, R_solved)
    trans_err = np.linalg.norm(t_true - t_solved) * 1000.0

    print("\n--- recovered vs ground truth (lidar -> camera) ---")
    print(f"  rotation error    : {rot_err:.4f} deg   (tolerance {args.tolerance_deg})")
    print(f"  translation error : {trans_err:.2f} mm    (tolerance {args.tolerance_mm})")
    print(f"  t true   : {np.round(t_true, 4).tolist()}")
    print(f"  t solved : {np.round(t_solved, 4).tolist()}")
    print(f"  per-axis err (mm): {np.round((t_solved - t_true) * 1000, 2).tolist()}")
    print(f"  solver mean residual: {result.get('mean_residual_m', 0) * 1000:.2f} mm "
          f"(max {result.get('max_residual_m', 0) * 1000:.2f} mm)")

    ok = rot_err <= args.tolerance_deg and trans_err <= args.tolerance_mm
    print(f"\n{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
