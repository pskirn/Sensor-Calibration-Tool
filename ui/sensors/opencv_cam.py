"""Direct camera capture via OpenCV `VideoCapture`.

Covers the non-ROS case: a USB webcam on `/dev/videoN`, a GigE/IP camera over
RTSP, or a video file. Useful when the cameras are plugged straight into the
machine running this tool and nobody wants to stand up a ROS driver for them.
"""

from __future__ import annotations

import glob
import os
import threading
import time
from typing import List, Optional, Union

import cv2

from .base import ImageFrame, ImageSource, SourceInfo


def list_devices(max_probe: int = 8) -> List[SourceInfo]:
    """Enumerate attached V4L2 cameras.

    On Linux we trust `/dev/video*` rather than probing indices, because probing
    opens each device in turn -- which steals it from anything already streaming
    and can take seconds per miss. Note that a single physical camera often
    exposes several nodes (capture + metadata), so not every entry will open.
    """
    infos: List[SourceInfo] = []
    if os.name == "posix" and glob.glob("/dev/video*"):
        for path in sorted(glob.glob("/dev/video*"),
                           key=lambda p: int("".join(filter(str.isdigit, p)) or 0)):
            name = _v4l2_name(path)
            infos.append(SourceInfo(
                kind="camera", id=f"cv:{path}",
                label=f"{path}{f' ({name})' if name else ''}",
                backend="opencv", detail="V4L2",
            ))
        return infos

    for index in range(max_probe):
        cap = cv2.VideoCapture(index)
        opened = cap.isOpened()
        cap.release()
        if opened:
            infos.append(SourceInfo(
                kind="camera", id=f"cv:{index}", label=f"Camera {index}",
                backend="opencv", detail="index",
            ))
    return infos


def _v4l2_name(device_path: str) -> str:
    """Read the friendly product name the kernel exposes in sysfs."""
    node = os.path.basename(device_path)
    try:
        with open(f"/sys/class/video4linux/{node}/name") as fh:
            return fh.read().strip()
    except OSError:
        return ""


class OpenCVCameraSource(ImageSource):
    """Grabs frames from a VideoCapture in a background thread."""

    def __init__(self, device: Union[int, str], width: int = 0, height: int = 0,
                 fps: int = 0) -> None:
        super().__init__(SourceInfo(
            kind="camera", id=f"cv:{device}", label=f"Camera {device}",
            backend="opencv", detail=str(device),
        ))
        self.device = device
        self.width, self.height, self.fps = width, height, fps
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._running:
            return
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open camera {self.device!r}. It may be in use by "
                f"another process, or the path may not be a capture node."
            )
        if self.width and self.height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        # A one-frame driver buffer keeps the preview close to real time; the
        # default queue makes the picture lag behind the board you are holding,
        # which is disorienting when you are trying to aim it.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._cap = cap
        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._running:
            return
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._running = False

    def _run(self) -> None:
        misses = 0
        while not self._stop.is_set() and self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                misses += 1
                if misses > 50:
                    self._error = "Camera stopped delivering frames"
                    return
                time.sleep(0.02)
                continue
            misses = 0
            self._publish(ImageFrame(
                stamp=time.time(), image=frame, frame_id=str(self.device),
            ))
