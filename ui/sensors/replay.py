"""rosbag2 replay sources.

Replay exists so the live capture path is developable, demoable, and testable
without hardware bolted to a robot. It uses the pure-Python `rosbags` package
rather than `rclpy`, so it runs in a plain virtualenv with no ROS installed --
which also means anyone who clones the repo can try the live workflow using a
sample bag, on any OS.

The replay clock honours the bag's own timestamps (scaled by `speed`) so
accumulation windows behave the way they would against a real sensor.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from . import image_msg, pointcloud2
from .base import CloudFrame, ImageFrame, ImageSource, PointCloudSource, SourceInfo

_TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def _open(bag: Path) -> AnyReader:
    """Build a reader for a bag, supplying a typestore for bags with no type defs.

    rosbag2 only embeds message definitions from Humble onward. Older bags need
    a default typestore or `AnyReader` refuses to open them.

    Returns an unopened reader -- every caller uses it as a context manager, and
    `AnyReader.open()` asserts if it is called twice.
    """
    return AnyReader([bag], default_typestore=_TYPESTORE)


def inspect_bag(bag: Path) -> dict:
    """Topic inventory for a bag, used by the source picker."""
    bag = Path(bag)
    with _open(bag) as reader:
        topics = [
            {
                "topic": c.topic,
                "msgtype": c.msgtype,
                "count": c.msgcount,
            }
            for c in reader.connections
        ]
        duration = (reader.end_time - reader.start_time) / 1e9
    return {
        "path": str(bag),
        "name": bag.name,
        "duration_s": round(duration, 2),
        "topics": sorted(topics, key=lambda t: t["topic"]),
    }


class _BagPlayer:
    """Shared replay loop: walks one topic and hands messages to a callback.

    Kept separate from the source classes because the cloud and image sources
    need identical timing behaviour and only differ in how they decode.
    """

    def __init__(self, bag: Path, topic: str, on_message, speed: float = 1.0,
                 loop: bool = True) -> None:
        self.bag = Path(bag)
        self.topic = topic
        self.on_message = on_message
        self.speed = max(speed, 0.01)
        self.loop = loop
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

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
            self._play_once()
            if not self.loop:
                return

    def _play_once(self) -> None:
        with _open(self.bag) as reader:
            conns = [c for c in reader.connections if c.topic == self.topic]
            if not conns:
                return
            wall_start = time.monotonic()
            bag_start: Optional[int] = None
            for conn, stamp_ns, raw in reader.messages(connections=conns):
                if self._stop.is_set():
                    return
                if bag_start is None:
                    bag_start = stamp_ns
                # Sleep until this message is "due" on the scaled replay clock.
                target = (stamp_ns - bag_start) / 1e9 / self.speed
                lag = target - (time.monotonic() - wall_start)
                if lag > 0:
                    if self._stop.wait(lag):
                        return
                try:
                    msg = reader.deserialize(raw, conn.msgtype)
                    self.on_message(msg, conn.msgtype, stamp_ns / 1e9)
                except Exception:
                    # A single malformed message should not kill the replay; the
                    # source's frame counter will show the gap.
                    continue


class BagCloudSource(PointCloudSource):
    """Replays a `PointCloud2` topic out of a rosbag2."""

    def __init__(self, bag: Path, topic: str, accumulation_window: float = 1.0,
                 speed: float = 1.0, loop: bool = True) -> None:
        super().__init__(
            SourceInfo(
                kind="lidar",
                id=f"replay:{bag}:{topic}",
                label=f"{Path(bag).name} · {topic}",
                backend="replay",
                detail=topic,
            ),
            accumulation_window=accumulation_window,
        )
        self._player = _BagPlayer(bag, topic, self._on_message, speed=speed, loop=loop)

    def _on_message(self, msg, msgtype: str, stamp: float) -> None:
        try:
            points, intensity = pointcloud2.decode(msg)
        except Exception as exc:
            self._error = str(exc)
            return
        self._publish(CloudFrame(
            stamp=stamp,
            points=points,
            intensity=intensity,
            frame_id=str(getattr(msg.header, "frame_id", "")),
        ))

    def start(self) -> None:
        self._running = True
        self._player.start()

    def stop(self) -> None:
        self._running = False
        self._player.stop()


class BagImageSource(ImageSource):
    """Replays an `Image` / `CompressedImage` topic out of a rosbag2."""

    def __init__(self, bag: Path, topic: str, speed: float = 1.0,
                 loop: bool = True) -> None:
        super().__init__(
            SourceInfo(
                kind="camera",
                id=f"replay:{bag}:{topic}",
                label=f"{Path(bag).name} · {topic}",
                backend="replay",
                detail=topic,
            )
        )
        self._player = _BagPlayer(bag, topic, self._on_message, speed=speed, loop=loop)

    def _on_message(self, msg, msgtype: str, stamp: float) -> None:
        try:
            image = image_msg.decode_any(msg, msgtype)
        except Exception as exc:
            self._error = str(exc)
            return
        self._publish(ImageFrame(
            stamp=stamp,
            image=image,
            frame_id=str(getattr(msg.header, "frame_id", "")),
        ))

    def start(self) -> None:
        self._running = True
        self._player.start()

    def stop(self) -> None:
        self._running = False
        self._player.stop()


def read_camera_info(bag: Path, topic: str) -> Optional[dict]:
    """Pull the first `CameraInfo` off a bag, if the topic exists.

    Saves the user from calibrating intrinsics by hand when the recording
    already carries them.
    """
    with _open(Path(bag)) as reader:
        conns = [c for c in reader.connections
                 if c.topic == topic and "CameraInfo" in c.msgtype]
        for conn, _stamp, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            return image_msg.camera_info_to_intrinsics(msg)
    return None
