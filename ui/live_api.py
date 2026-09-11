"""HTTP API for live capture.

Kept in its own router so `app.py` stays focused on the offline dataset flow.
Exactly one capture session exists at a time -- the tool drives physical sensors,
and two sessions fighting over the same camera device would fail in confusing
ways, so the constraint is made explicit rather than left to chance.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field

from .live.detect import BoardSpec
from .live.session import LiveConfig, LiveSession
from .sensors import netscan, registry, ros2_live, synthetic

router = APIRouter(prefix="/api/live", tags=["live"])

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "data"
LIVE_DIR = DATA_DIR / "live"
CONFIG_DIR = PROJECT_ROOT / "config"

# Directories scanned for rosbag2 recordings offered as replay sources.
BAG_SEARCH_DIRS = [PROJECT_ROOT / "livox", DATA_DIR / "bags"]

_session: Optional[LiveSession] = None
_session_lock = threading.Lock()


def _current() -> LiveSession:
    with _session_lock:
        if _session is None or not _session.running:
            raise HTTPException(status_code=409, detail="No live session is running")
        return _session


# --- request models -------------------------------------------------------

class BoardModel(BaseModel):
    cols: int = Field(..., description="Internal corners across (not squares)")
    rows: int = Field(..., description="Internal corners down (not squares)")
    square_size: float = Field(..., description="Square edge length in metres")
    board_width: Optional[float] = None
    board_height: Optional[float] = None


class IntrinsicsModel(BaseModel):
    camera_matrix: List[List[float]]
    distortion_coefficients: List[float]


class StartRequest(BaseModel):
    camera_source: str
    lidar_source: str
    board: BoardModel
    roi_min: List[float]
    roi_max: List[float]
    intrinsics: Optional[IntrinsicsModel] = None
    accumulation_window: float = 1.0
    ransac_threshold: float = 0.02
    replay_speed: float = 1.0


class FinishRequest(BaseModel):
    name: str = "live_session"
    solve: bool = True


# --- intrinsics helpers ---------------------------------------------------

def _load_intrinsics_file(path: Path) -> Optional[dict]:
    """Read `config/camera.yaml`, which uses the OpenCV rows/cols/data layout."""
    if not path.exists():
        return None
    with path.open() as fh:
        raw = yaml.safe_load(fh) or {}

    def matrix(node, shape):
        if node is None:
            return None
        if isinstance(node, dict) and "data" in node:
            return np.array(node["data"], dtype=float).reshape(shape)
        return np.array(node, dtype=float).reshape(shape)

    K = matrix(raw.get("camera_matrix"), (3, 3))
    if K is None:
        return None
    dist_node = raw.get("distortion_coefficients")
    D = matrix(dist_node, (-1,)) if dist_node is not None else np.zeros(5)
    return {
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.ravel().tolist(),
        "image_width": raw.get("image_width"),
        "image_height": raw.get("image_height"),
        "source": str(path.relative_to(PROJECT_ROOT)),
    }


def _resolve_intrinsics(camera_source: str,
                        provided: Optional[IntrinsicsModel]) -> tuple:
    """Pick intrinsics: explicit > published by the source > config file.

    Preferring what the source itself publishes matters because a `camera_info`
    topic is guaranteed to describe the very stream being captured, whereas a
    config file can silently belong to a different camera or resolution.
    """
    if provided is not None:
        return (np.array(provided.camera_matrix, dtype=float),
                np.array(provided.distortion_coefficients, dtype=float),
                "request")

    discovered = registry.find_intrinsics(camera_source)
    if discovered:
        label = ("simulated rig" if camera_source == "sim:camera"
                 else "camera_info topic")
        return (np.array(discovered["camera_matrix"], dtype=float),
                np.array(discovered["distortion_coefficients"], dtype=float),
                label)

    from_file = _load_intrinsics_file(CONFIG_DIR / "camera.yaml")
    if from_file:
        return (np.array(from_file["camera_matrix"], dtype=float),
                np.array(from_file["distortion_coefficients"], dtype=float),
                from_file["source"])

    raise HTTPException(
        status_code=400,
        detail="No camera intrinsics available. Publish a camera_info topic, "
               "run compute_intrinsics to create config/camera.yaml, or pass "
               "intrinsics in the request.",
    )


# --- endpoints ------------------------------------------------------------

@router.get("/sources")
def list_sources() -> dict:
    """Every camera and LiDAR we could stream from right now."""
    found = registry.discover(BAG_SEARCH_DIRS)
    found["session_running"] = _session is not None and _session.running
    return found


@router.get("/diagnose")
def diagnose(seconds: float = 1.5) -> dict:
    """Explain why an Ethernet LiDAR is not in the list.

    Unlike a USB camera, a network LiDAR cannot announce itself to the operating
    system -- it just emits UDP that only its vendor driver understands. Rather
    than show an unexplained empty dropdown, we look at what is actually on the
    wire and say what to do about it.
    """
    ros_topics: List[str] = []
    if ros2_live.available():
        try:
            ros_topics = [s.label for s in ros2_live.list_topics() if s.kind == "lidar"]
        except Exception:
            ros_topics = []

    # Bound the probe so a mistyped query cannot hang the request.
    seconds = max(0.3, min(float(seconds), 5.0))
    return netscan.diagnose(
        duration=seconds,
        ros_available=ros2_live.available(),
        ros_lidar_topics=ros_topics,
    )


@router.get("/defaults")
def defaults(camera_source: str = "", lidar_source: str = "") -> dict:
    """Pre-fill values for the setup form, given the chosen sources."""
    if camera_source == "sim:camera" or lidar_source == "sim:lidar":
        base = synthetic.default_config()
        base["intrinsics_source"] = "simulated rig"
        base["note"] = (
            "Simulated rig: board geometry, ROI and intrinsics are exact, so "
            "this configuration needs no changes."
        )
        return base

    result: dict = {
        "board": {"cols": 7, "rows": 5, "square_size": 0.025},
        "roi_min": [-2.0, -2.0, -1.0],
        "roi_max": [4.0, 2.0, 2.0],
        "accumulation_window": 1.0,
        "note": "",
    }

    intrinsics = registry.find_intrinsics(camera_source) if camera_source else None
    source_label = "camera_info topic"
    if not intrinsics:
        intrinsics = _load_intrinsics_file(CONFIG_DIR / "camera.yaml")
        source_label = intrinsics["source"] if intrinsics else "none"
    if intrinsics:
        result["camera_matrix"] = intrinsics["camera_matrix"]
        result["distortion_coefficients"] = intrinsics["distortion_coefficients"]
        result["image_width"] = intrinsics.get("image_width")
        result["image_height"] = intrinsics.get("image_height")
    result["intrinsics_source"] = source_label

    # A non-repetitive scanner needs accumulation; a spinning one does not. We
    # cannot tell them apart from a topic name, so default to accumulating and
    # explain the knob.
    result["note"] = (
        "Accumulation window is set to 1 s, which suits non-repetitive scanners "
        "such as Livox. For a spinning LiDAR (Velodyne, Ouster, Hesai) one sweep "
        "is already a full frame -- set it to 0 for the lowest latency."
    )
    return result


@router.post("/start")
def start(req: StartRequest) -> dict:
    global _session
    with _session_lock:
        if _session is not None and _session.running:
            raise HTTPException(
                status_code=409,
                detail="A live session is already running. Stop it before starting another.",
            )

    K, dist, intrinsics_source = _resolve_intrinsics(req.camera_source, req.intrinsics)

    config = LiveConfig(
        camera_source=req.camera_source,
        lidar_source=req.lidar_source,
        board=BoardSpec(
            cols=req.board.cols, rows=req.board.rows,
            square_size=req.board.square_size,
            board_width=req.board.board_width, board_height=req.board.board_height,
        ),
        roi_min=req.roi_min,
        roi_max=req.roi_max,
        K=K,
        dist=dist,
        accumulation_window=req.accumulation_window,
        ransac_threshold=req.ransac_threshold,
        replay_speed=req.replay_speed,
    )

    session = LiveSession(config)
    try:
        session.start()
    except Exception as exc:
        session.stop()
        raise HTTPException(status_code=400, detail=f"Could not start sources: {exc}")

    with _session_lock:
        _session = session

    # Give the sources a moment so the first status poll is not all "waiting".
    if session._camera:
        session._camera.wait_for_frame(timeout=5.0)
    if session._lidar:
        session._lidar.wait_for_accumulation(timeout=5.0)

    return {
        "started": True,
        "intrinsics_source": intrinsics_source,
        "camera": req.camera_source,
        "lidar": req.lidar_source,
    }


@router.post("/stop")
def stop() -> dict:
    global _session
    with _session_lock:
        session, _session = _session, None
    if session is not None:
        session.stop()
    return {"stopped": True}


@router.get("/state")
def state() -> dict:
    session = _current()
    live = session.state()
    readiness = session.readiness()
    return {
        "live": vars(live),
        "readiness": {
            "ready": readiness.ready,
            "num_poses": readiness.num_poses,
            "normal_span": round(readiness.normal_span, 3),
            "coverage": round(readiness.coverage, 3),
            "distance_range": [round(v, 2) for v in readiness.distance_range],
            "blocking": readiness.blocking,
            "advice": readiness.advice,
        },
        "poses": session.captured_summary(),
        "image_size": list(session.image_size),
    }


@router.get("/cloud")
def cloud() -> dict:
    return _current().cloud_preview()


@router.get("/preview.mjpg")
def preview() -> StreamingResponse:
    """MJPEG stream of the annotated camera view.

    MJPEG rather than WebSockets because an `<img src>` renders it with no
    client code at all, and the frames are already JPEG-encoded server-side.
    """
    boundary = "frameboundary"

    def generate():
        while True:
            try:
                session = _current()
            except HTTPException:
                return
            frame = session.preview_jpeg()
            if frame is None:
                time.sleep(0.1)
                continue
            yield (
                b"--" + boundary.encode() + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                + frame + b"\r\n"
            )
            time.sleep(0.1)   # ~10 fps is plenty for aiming a board

    return StreamingResponse(
        generate(),
        media_type=f"multipart/x-mixed-replace; boundary={boundary}",
    )


@router.post("/capture")
def capture(force: bool = False) -> dict:
    session = _current()
    outcome = session.capture(force=force)
    readiness = session.readiness()
    outcome["num_poses"] = readiness.num_poses
    outcome["normal_span"] = round(readiness.normal_span, 3)
    outcome["ready"] = readiness.ready
    return outcome


@router.delete("/pose/{index}")
def drop_pose(index: int) -> dict:
    session = _current()
    if not session.drop(index):
        raise HTTPException(status_code=404, detail=f"No captured pose at index {index}")
    return {"dropped": index, "num_poses": len(session.poses)}


@router.post("/finish")
def finish(req: FinishRequest) -> dict:
    """Export the captured poses, and optionally solve immediately."""
    session = _current()
    readiness = session.readiness()
    if not readiness.ready:
        raise HTTPException(
            status_code=400,
            detail={"message": "Session is not ready to solve",
                    "blocking": readiness.blocking},
        )

    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in req.name) or "live_session"
    dest = LIVE_DIR / safe / "poses.csv"
    written = session.write_poses_csv(dest)

    response = {
        "poses_written": written,
        "dataset": f"live/{safe}",
        "poses_csv": str(dest.relative_to(PROJECT_ROOT)),
    }

    if req.solve:
        # Reuse the offline runner so live and offline solves are the same code
        # path, and therefore cannot drift apart.
        from .app import run_calibration, CalibrateRequest
        solved = run_calibration(CalibrateRequest(dataset=f"live/{safe}"))
        import json
        response["result"] = json.loads(bytes(solved.body).decode())

    return response
