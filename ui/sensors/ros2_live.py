"""Live ROS 2 sources -- the universal adapter.

Every LiDAR vendor worth using ships a ROS driver that publishes
`sensor_msgs/PointCloud2`: Livox, Ouster, Velodyne, Hesai, RoboSense, Unitree,
and the rest. So this one adapter covers essentially the entire market, and new
sensors work on release day without us writing anything. The same applies to
cameras via `sensor_msgs/Image` and `CompressedImage`.

`rclpy` is imported lazily. The offline and replay paths must keep working in a
plain virtualenv with no ROS installed, so nothing here may be imported at
module scope by the rest of the app.

To use this path, source your ROS setup before starting the server:

    source /opt/ros/$ROS_DISTRO/setup.bash
    ui/.venv/bin/python -m uvicorn ui.app:app

That is enough even for an ordinary virtualenv -- sourcing ROS exports
`PYTHONPATH`, which venvs honour, so `rclpy` resolves without needing the venv
to be created with `--system-site-packages`.
"""

from __future__ import annotations

import threading
from typing import List, Optional

from . import image_msg, pointcloud2
from .base import CloudFrame, ImageFrame, ImageSource, PointCloudSource, SourceInfo

_CLOUD_TYPES = ("sensor_msgs/msg/PointCloud2",)
_IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")


def available() -> bool:
    """True if this process can talk to ROS 2 at all."""
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


def unavailable_reason() -> str:
    return (
        "Live ROS 2 sources are unavailable: rclpy is not importable. Run "
        "`source /opt/ros/$ROS_DISTRO/setup.bash` before starting the server. "
        "Bag replay and direct USB cameras work without ROS."
    )


class _RosContext:
    """One shared node + executor thread for every ROS source in the process.

    rclpy allows only one init per process, and spinning several executors would
    just multiply threads for no benefit. Sources register subscriptions on this
    single node and it is torn down when the last one detaches.
    """

    _instance: Optional["_RosContext"] = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        # `init` may already have been called by an embedding application.
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("calib_ui_sensor_bridge")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._refcount = 0

    @classmethod
    def acquire(cls) -> "_RosContext":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            cls._instance._refcount += 1
            return cls._instance

    @classmethod
    def release(cls) -> None:
        with cls._lock:
            inst = cls._instance
            if inst is None:
                return
            inst._refcount -= 1
            if inst._refcount > 0:
                return
            inst._shutdown()
            cls._instance = None

    def _spin(self) -> None:
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                # An executor exception during shutdown is expected and noisy;
                # the stop flag is the authority on whether we should exit.
                if self._stop.is_set():
                    return

    def _shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass


def _sensor_qos():
    """QoS chosen for maximum compatibility with unknown publishers.

    BEST_EFFORT + VOLATILE is the permissive end of the matrix: a RELIABLE
    publisher can serve a BEST_EFFORT subscriber, and a TRANSIENT_LOCAL
    publisher can serve a VOLATILE one, but not the reverse. Picking the strict
    end would silently fail to connect to sensor drivers -- the classic "topic
    lists fine, no messages arrive" trap.
    """
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _message_class(type_name: str):
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_name)


def list_topics() -> List[SourceInfo]:
    """Discover live PointCloud2 / Image topics currently on the graph."""
    if not available():
        return []
    ctx = _RosContext.acquire()
    try:
        found: List[SourceInfo] = []
        for topic, types in ctx.node.get_topic_names_and_types():
            for type_name in types:
                if type_name in _CLOUD_TYPES:
                    found.append(SourceInfo(
                        kind="lidar", id=f"ros2:{topic}", label=topic,
                        backend="ros2", detail=type_name,
                    ))
                elif type_name in _IMAGE_TYPES:
                    found.append(SourceInfo(
                        kind="camera", id=f"ros2:{topic}", label=topic,
                        backend="ros2", detail=type_name,
                    ))
        return found
    finally:
        _RosContext.release()


def _resolve_type(topic: str, wanted: tuple[str, ...]) -> str:
    """Look up a topic's type from the graph, falling back to the first wanted."""
    ctx = _RosContext.acquire()
    try:
        for name, types in ctx.node.get_topic_names_and_types():
            if name == topic:
                for type_name in types:
                    if type_name in wanted:
                        return type_name
    finally:
        _RosContext.release()
    return wanted[0]


class RosCloudSource(PointCloudSource):
    """Subscribes to a `sensor_msgs/PointCloud2` topic."""

    def __init__(self, topic: str, accumulation_window: float = 1.0) -> None:
        super().__init__(
            SourceInfo(kind="lidar", id=f"ros2:{topic}", label=topic,
                       backend="ros2", detail="sensor_msgs/msg/PointCloud2"),
            accumulation_window=accumulation_window,
        )
        self.topic = topic
        self._ctx: Optional[_RosContext] = None
        self._sub = None

    def start(self) -> None:
        if self._running:
            return
        from sensor_msgs.msg import PointCloud2

        self._ctx = _RosContext.acquire()
        self._sub = self._ctx.node.create_subscription(
            PointCloud2, self.topic, self._on_message, _sensor_qos()
        )
        self._running = True

    def stop(self) -> None:
        if not self._running:
            return
        if self._ctx is not None and self._sub is not None:
            try:
                self._ctx.node.destroy_subscription(self._sub)
            except Exception:
                pass
        self._sub = None
        self._ctx = None
        self._running = False
        _RosContext.release()

    def _on_message(self, msg) -> None:
        try:
            points, intensity = pointcloud2.decode(msg)
        except Exception as exc:
            self._error = str(exc)
            return
        self._publish(CloudFrame(
            stamp=pointcloud2.stamp_to_seconds(msg.header),
            points=points,
            intensity=intensity,
            frame_id=str(msg.header.frame_id),
        ))


class RosImageSource(ImageSource):
    """Subscribes to a `sensor_msgs/Image` or `CompressedImage` topic."""

    def __init__(self, topic: str) -> None:
        type_name = _resolve_type(topic, _IMAGE_TYPES) if available() else _IMAGE_TYPES[0]
        super().__init__(
            SourceInfo(kind="camera", id=f"ros2:{topic}", label=topic,
                       backend="ros2", detail=type_name)
        )
        self.topic = topic
        self.type_name = type_name
        self._ctx: Optional[_RosContext] = None
        self._sub = None

    def start(self) -> None:
        if self._running:
            return
        msg_class = _message_class(self.type_name)
        self._ctx = _RosContext.acquire()
        self._sub = self._ctx.node.create_subscription(
            msg_class, self.topic, self._on_message, _sensor_qos()
        )
        self._running = True

    def stop(self) -> None:
        if not self._running:
            return
        if self._ctx is not None and self._sub is not None:
            try:
                self._ctx.node.destroy_subscription(self._sub)
            except Exception:
                pass
        self._sub = None
        self._ctx = None
        self._running = False
        _RosContext.release()

    def _on_message(self, msg) -> None:
        try:
            image = image_msg.decode_any(msg, self.type_name)
        except Exception as exc:
            self._error = str(exc)
            return
        self._publish(ImageFrame(
            stamp=pointcloud2.stamp_to_seconds(msg.header),
            image=image,
            frame_id=str(msg.header.frame_id),
        ))


def fetch_camera_info(topic: str, timeout: float = 3.0) -> Optional[dict]:
    """Grab one `CameraInfo` message, so live mode can self-configure intrinsics."""
    if not available():
        return None
    import time

    from sensor_msgs.msg import CameraInfo

    result: dict = {}
    done = threading.Event()

    def _cb(msg):
        if not done.is_set():
            result.update(image_msg.camera_info_to_intrinsics(msg))
            done.set()

    ctx = _RosContext.acquire()
    sub = ctx.node.create_subscription(CameraInfo, topic, _cb, _sensor_qos())
    try:
        deadline = time.monotonic() + timeout
        while not done.is_set() and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        try:
            ctx.node.destroy_subscription(sub)
        except Exception:
            pass
        _RosContext.release()
    return result or None
