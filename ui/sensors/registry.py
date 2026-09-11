"""Source discovery and construction.

Everything the UI knows about a sensor is a `SourceInfo` and its opaque `id`
string. The id encodes which backend to use, so the frontend never has to model
the difference between a ROS topic, a webcam and a bag file:

    ros2:/livox/points
    cv:/dev/video0
    replay:/abs/path/to/bag:/livox/points
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

from . import opencv_cam, replay, ros2_live, synthetic
from .base import ImageSource, PointCloudSource, SourceInfo

_CLOUD_TYPE_HINT = "PointCloud2"
_IMAGE_TYPE_HINTS = ("Image", "CompressedImage")


def find_bags(search_dirs: List[Path]) -> List[Path]:
    """Locate rosbag2 directories (those containing a metadata.yaml)."""
    bags: List[Path] = []
    for root in search_dirs:
        root = Path(root)
        if not root.exists():
            continue
        if (root / "metadata.yaml").exists():
            bags.append(root)
            continue
        for meta in sorted(root.glob("*/metadata.yaml")):
            bags.append(meta.parent)
    return bags


def discover(bag_search_dirs: Optional[List[Path]] = None) -> dict:
    """Enumerate every source we can offer right now, grouped by sensor kind."""
    lidar: List[SourceInfo] = []
    camera: List[SourceInfo] = []
    notes: List[str] = []

    # --- simulated rig ----------------------------------------------------
    # Listed first so the tool is usable the moment it starts, with no hardware.
    for info in synthetic.source_infos():
        (lidar if info.kind == "lidar" else camera).append(info)

    # --- live ROS 2 graph -------------------------------------------------
    if ros2_live.available():
        try:
            for info in ros2_live.list_topics():
                (lidar if info.kind == "lidar" else camera).append(info)
        except Exception as exc:
            notes.append(f"ROS 2 discovery failed: {exc}")
    else:
        notes.append(ros2_live.unavailable_reason())

    # --- direct cameras ---------------------------------------------------
    try:
        camera.extend(opencv_cam.list_devices())
    except Exception as exc:
        notes.append(f"Camera enumeration failed: {exc}")

    # --- bag replay -------------------------------------------------------
    for bag in find_bags(bag_search_dirs or []):
        try:
            info = replay.inspect_bag(bag)
        except Exception as exc:
            notes.append(f"Could not read bag {bag.name}: {type(exc).__name__}: {exc}")
            continue
        for topic in info["topics"]:
            source = SourceInfo(
                kind="", id=f"replay:{bag}:{topic['topic']}",
                label=f"{bag.name} · {topic['topic']}",
                backend="replay",
                detail=f"{topic['msgtype']} · {topic['count']} msgs",
            )
            if _CLOUD_TYPE_HINT in topic["msgtype"]:
                source.kind = "lidar"
                lidar.append(source)
            elif any(h in topic["msgtype"] for h in _IMAGE_TYPE_HINTS):
                source.kind = "camera"
                camera.append(source)

    return {
        "lidar": [vars(s) for s in lidar],
        "camera": [vars(s) for s in camera],
        "ros2_available": ros2_live.available(),
        "notes": notes,
    }


def _split_replay_id(source_id: str) -> tuple[Path, str]:
    """`replay:<bag path>:<topic>` -> (bag, topic).

    Split from the right so absolute bag paths survive; topics always start with
    '/' and never contain ':'.
    """
    body = source_id[len("replay:"):]
    bag_str, _, topic = body.rpartition(":")
    if not bag_str or not topic:
        raise ValueError(f"Malformed replay source id: {source_id!r}")
    return Path(bag_str), topic


def open_cloud_source(source_id: str, accumulation_window: float = 1.0,
                      speed: float = 1.0) -> PointCloudSource:
    if source_id == "sim:lidar":
        return synthetic.SyntheticLidarSource(accumulation_window=accumulation_window)
    if source_id.startswith("ros2:"):
        return ros2_live.RosCloudSource(
            source_id[len("ros2:"):], accumulation_window=accumulation_window
        )
    if source_id.startswith("replay:"):
        bag, topic = _split_replay_id(source_id)
        return replay.BagCloudSource(
            bag, topic, accumulation_window=accumulation_window, speed=speed
        )
    raise ValueError(f"Unknown LiDAR source id: {source_id!r}")


def open_image_source(source_id: str, speed: float = 1.0) -> ImageSource:
    if source_id == "sim:camera":
        return synthetic.SyntheticCameraSource()
    if source_id.startswith("ros2:"):
        return ros2_live.RosImageSource(source_id[len("ros2:"):])
    if source_id.startswith("replay:"):
        bag, topic = _split_replay_id(source_id)
        return replay.BagImageSource(bag, topic, speed=speed)
    if source_id.startswith("cv:"):
        device = source_id[len("cv:"):]
        # Numeric ids address a capture index; anything else is a path or URL.
        return opencv_cam.OpenCVCameraSource(
            int(device) if device.isdigit() else device
        )
    raise ValueError(f"Unknown camera source id: {source_id!r}")


def find_intrinsics(camera_source_id: str) -> Optional[dict]:
    """Best-effort intrinsics lookup for a camera source.

    ROS drivers and bags conventionally publish `<ns>/camera_info` alongside
    `<ns>/image_raw`, so we can often skip asking the user for a calibration
    file entirely.
    """
    if camera_source_id == "sim:camera":
        defaults = synthetic.default_config()
        return {
            "image_width": defaults["image_width"],
            "image_height": defaults["image_height"],
            "camera_matrix": defaults["camera_matrix"],
            "distortion_coefficients": defaults["distortion_coefficients"],
            "distortion_model": "plumb_bob",
        }
    if camera_source_id.startswith("ros2:"):
        topic = camera_source_id[len("ros2:"):]
        return ros2_live.fetch_camera_info(_camera_info_topic(topic))
    if camera_source_id.startswith("replay:"):
        bag, topic = _split_replay_id(camera_source_id)
        try:
            return replay.read_camera_info(bag, _camera_info_topic(topic))
        except Exception:
            return None
    return None


def _camera_info_topic(image_topic: str) -> str:
    """Map an image topic to its sibling camera_info topic."""
    parent = image_topic.rsplit("/", 1)[0]
    return f"{parent}/camera_info" if parent else "/camera_info"
