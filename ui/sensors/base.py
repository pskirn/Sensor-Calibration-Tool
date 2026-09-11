"""Sensor source abstractions.

The whole point of this layer is that the calibration pipeline should not know
or care what brand of LiDAR or camera is attached. Every LiDAR ultimately
produces the same thing -- an (N, 3) array of XYZ points -- and every camera
produces an HxWx3 BGR image. What differs is only the transport: Livox speaks
SDK2/UDP, Ouster speaks TCP+JSON, Velodyne speaks raw UDP packets, and so on.

So we define two narrow interfaces here and let adapters deal with transport.
The ROS 2 adapter is the important one: every LiDAR vendor ships a driver that
publishes `sensor_msgs/PointCloud2`, which makes a single adapter cover the
entire market. Direct vendor SDKs are optional extras, not the foundation.

One wrinkle that has to live in the interface rather than in an adapter:
**scan pattern**. A spinning LiDAR delivers a complete sweep every revolution,
so one message is a usable frame. A non-repetitive scanner like the Livox
Mid-360 paints a sparse pattern that only fills in over time, so a usable frame
is "every message from the last 1-2 seconds, concatenated". `accumulate()`
expresses that difference explicitly instead of hiding it.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

import numpy as np


# --- frame types ----------------------------------------------------------

@dataclass
class CloudFrame:
    """A LiDAR observation, normalised to plain numpy."""

    stamp: float                      # seconds since epoch
    points: np.ndarray                # (N, 3) float32, sensor frame, metres
    intensity: Optional[np.ndarray] = None   # (N,) float32 if the sensor has it
    frame_id: str = ""

    @property
    def num_points(self) -> int:
        return int(self.points.shape[0])


@dataclass
class ImageFrame:
    """A camera observation, normalised to an OpenCV-style BGR array."""

    stamp: float                      # seconds since epoch
    image: np.ndarray                 # (H, W, 3) uint8, BGR
    frame_id: str = ""

    @property
    def size(self) -> tuple[int, int]:
        h, w = self.image.shape[:2]
        return (w, h)


@dataclass
class SourceInfo:
    """What a source advertises about itself, for the UI's picker."""

    kind: str          # "lidar" | "camera"
    id: str            # opaque handle the API passes back to open the source
    label: str         # human-readable
    backend: str       # "ros2" | "replay" | "opencv"
    detail: str = ""   # topic name, device path, message type, ...


# --- source interfaces ----------------------------------------------------

class SensorSource(ABC):
    """Common lifecycle: start a background producer, read the latest frame.

    Sources are pull-based from the caller's perspective. The adapter runs its
    own thread (ROS executor, capture loop, replay clock) and drops frames into
    a buffer; readers always get the most recent one rather than queueing up
    stale data. For a calibration UI that is the right trade-off -- we want to
    show what the sensor sees *now*, not replay a backlog.
    """

    def __init__(self, info: SourceInfo) -> None:
        self.info = info
        self._lock = threading.Lock()
        self._running = False
        self._error: Optional[str] = None
        self._frames_seen = 0

    @property
    def running(self) -> bool:
        return self._running

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def frames_seen(self) -> int:
        return self._frames_seen

    @abstractmethod
    def start(self) -> None:
        """Begin producing frames. Must be idempotent."""

    @abstractmethod
    def stop(self) -> None:
        """Stop producing and release the device. Must be idempotent."""

    def __enter__(self) -> "SensorSource":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


class ImageSource(SensorSource):
    """Produces `ImageFrame`s."""

    def __init__(self, info: SourceInfo) -> None:
        super().__init__(info)
        self._latest: Optional[ImageFrame] = None

    def _publish(self, frame: ImageFrame) -> None:
        with self._lock:
            self._latest = frame
            self._frames_seen += 1

    def latest(self) -> Optional[ImageFrame]:
        with self._lock:
            return self._latest

    def wait_for_frame(self, timeout: float = 5.0) -> Optional[ImageFrame]:
        """Block until a frame arrives, or `timeout` elapses."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.latest()
            if frame is not None:
                return frame
            time.sleep(0.02)
        return None


class PointCloudSource(SensorSource):
    """Produces `CloudFrame`s, with an accumulation window for sparse scanners.

    `accumulation_window` is the knob that makes one interface work for both
    scan patterns. Set it to 0 for a spinning LiDAR (one message is already a
    full sweep). Set it to 1.0-2.0 for a Livox-style non-repetitive scanner,
    where density only builds up over time.
    """

    def __init__(self, info: SourceInfo, accumulation_window: float = 0.0,
                 max_buffer: int = 400) -> None:
        super().__init__(info)
        self.accumulation_window = accumulation_window
        self._buffer: Deque[CloudFrame] = deque(maxlen=max_buffer)

    def _publish(self, frame: CloudFrame) -> None:
        with self._lock:
            self._buffer.append(frame)
            self._frames_seen += 1

    def latest(self) -> Optional[CloudFrame]:
        """The most recent single message, no accumulation."""
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def accumulate(self, window: Optional[float] = None) -> Optional[CloudFrame]:
        """Concatenate every buffered frame within `window` seconds of the newest.

        Returns a single `CloudFrame` stamped at the newest contributing frame.
        With `window <= 0` this degenerates to `latest()`, which is what you
        want for a spinning LiDAR.
        """
        win = self.accumulation_window if window is None else window
        with self._lock:
            if not self._buffer:
                return None
            newest = self._buffer[-1]
            if win <= 0:
                return newest
            cutoff = newest.stamp - win
            chunk = [f for f in self._buffer if f.stamp >= cutoff]

        if len(chunk) == 1:
            return chunk[0]

        points = np.concatenate([f.points for f in chunk], axis=0)
        # Intensity is optional and may be missing on some frames; only carry it
        # through if every contributing frame has it, otherwise it would silently
        # misalign with the point array.
        if all(f.intensity is not None for f in chunk):
            intensity = np.concatenate([f.intensity for f in chunk], axis=0)
        else:
            intensity = None
        return CloudFrame(
            stamp=newest.stamp,
            points=points,
            intensity=intensity,
            frame_id=newest.frame_id,
        )

    def wait_for_frame(self, timeout: float = 5.0) -> Optional[CloudFrame]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.latest()
            if frame is not None:
                return frame
            time.sleep(0.02)
        return None

    def wait_for_accumulation(self, timeout: float = 10.0) -> Optional[CloudFrame]:
        """Wait until the buffer spans a full accumulation window, then return it.

        Without this, the first `accumulate()` call right after `start()` returns
        a thin cloud built from however few messages happened to arrive, which
        makes board detection fail for reasons that look like a detector bug.
        """
        win = self.accumulation_window
        if win <= 0:
            return self.wait_for_frame(timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                spanned = (
                    len(self._buffer) >= 2
                    and (self._buffer[-1].stamp - self._buffer[0].stamp) >= win
                )
            if spanned:
                return self.accumulate()
            time.sleep(0.02)
        return self.accumulate()
