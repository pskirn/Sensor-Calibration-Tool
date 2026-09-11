"""Detect Ethernet LiDARs that are plugged in but not yet being served.

A USB camera appears as `/dev/video0` because the kernel implements UVC, a
standard every webcam speaks. An Ethernet LiDAR has no equivalent: it simply
sprays proprietary UDP at the NIC, and nothing in the operating system knows a
sensor exists. So "plug it in and it shows up in the list" cannot work the way
it does for cameras -- something must first speak the vendor's protocol, which
in practice means the vendor's ROS driver.

What this module does is remove the confusing part of that: instead of an empty
LiDAR dropdown, the UI can say "a Livox is streaming on port 56300 but no driver
is consuming it -- run this command", or "the cable is in but your host IP is on
the wrong subnet".

Safety property
---------------
Probing must never disturb a driver that is already running. So sockets are
bound **exclusively** -- no SO_REUSEPORT. That gives an unambiguous, harmless
reading:

* bind succeeds, packets arrive -> a LiDAR is streaming and nobody is consuming
  it. This is exactly the "driver not started" case, and stealing those packets
  costs nothing because no one wants them.
* bind fails with EADDRINUSE -> something already holds the port, which is
  almost certainly the driver. We report that and touch nothing.

Under no circumstance do we end up splitting a live stream with a running driver.
"""

from __future__ import annotations

import errno
import ipaddress
import json
import select
import socket
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

# UDP ports the major vendors push point data on. Used only as a hint for
# naming what we found -- detection itself is just "are packets arriving".
KNOWN_LIDAR_PORTS: Dict[int, tuple] = {
    56300: ("Livox", "Mid-360 / HAP point cloud"),
    56400: ("Livox", "Mid-360 IMU"),
    2368: ("Velodyne or Hesai", "point cloud"),
    2369: ("Hesai", "GPS / position"),
    8308: ("Velodyne", "position"),
    7502: ("Ouster", "point cloud"),
    7503: ("Ouster", "IMU"),
    6699: ("RoboSense", "point cloud"),
    7788: ("RoboSense", "DIFOP"),
}

# Default factory networks. A wrong host IP is the single most common reason an
# Ethernet LiDAR appears dead, so it is worth checking explicitly.
VENDOR_NETWORKS = {
    "Livox": {
        "network": "192.168.1.0/24",
        "suggested_host": "192.168.1.50",
        "note": "Mid-360 ships on 192.168.1.1xx (last octet from the serial); "
                "the host must sit on the same /24.",
    },
    "Ouster": {
        "network": None,
        "suggested_host": None,
        "note": "Ouster sensors use DHCP or link-local (169.254.x.x) by default.",
    },
    "Velodyne": {
        "network": "192.168.1.0/24",
        "suggested_host": "192.168.1.100",
        "note": "Velodyne sensors default to 192.168.1.201.",
    },
}


@dataclass
class Interface:
    name: str
    is_up: bool
    has_carrier: bool          # cable physically connected
    addresses: List[str] = field(default_factory=list)

    @property
    def state(self) -> str:
        if not self.has_carrier:
            return "no cable"
        if not self.addresses:
            return "cable connected, no IP"
        return "up"


@dataclass
class PortProbe:
    port: int
    vendor: str
    description: str
    status: str                # "receiving" | "silent" | "in_use" | "error"
    packets: int = 0
    senders: List[str] = field(default_factory=list)
    detail: str = ""


def _is_virtual(name: str) -> bool:
    """Loopback, docker bridges, veth pairs etc. are not sensor links."""
    if name == "lo":
        return True
    return any(name.startswith(p) for p in
               ("docker", "veth", "br-", "virbr", "tun", "tap", "wg"))


def ethernet_interfaces() -> List[Interface]:
    """Wired interfaces, their link state and addresses.

    Reads sysfs for link state (works without extra tooling) and shells out to
    `ip -j addr` for addresses, degrading gracefully if it is unavailable.
    """
    interfaces: List[Interface] = []
    net_dir = Path("/sys/class/net")
    if not net_dir.exists():
        return interfaces

    addresses = _addresses_by_interface()

    for entry in sorted(net_dir.iterdir()):
        name = entry.name
        if _is_virtual(name):
            continue
        # Wireless interfaces have a `wireless` subdirectory; a LiDAR is wired.
        if (entry / "wireless").exists() or (entry / "phy80211").exists():
            continue
        try:
            operstate = (entry / "operstate").read_text().strip()
        except OSError:
            operstate = "unknown"
        try:
            carrier = (entry / "carrier").read_text().strip() == "1"
        except OSError:
            # Reading `carrier` errors when the interface is administratively
            # down, which is itself the answer.
            carrier = False

        interfaces.append(Interface(
            name=name,
            is_up=operstate == "up",
            has_carrier=carrier,
            addresses=addresses.get(name, []),
        ))
    return interfaces


def _addresses_by_interface() -> Dict[str, List[str]]:
    try:
        out = subprocess.run(["ip", "-j", "addr"], capture_output=True,
                             text=True, timeout=5)
        if out.returncode != 0:
            return {}
        result: Dict[str, List[str]] = {}
        for item in json.loads(out.stdout):
            addrs = [f"{a['local']}/{a['prefixlen']}"
                     for a in item.get("addr_info", [])
                     if a.get("family") == "inet"]
            result[item["ifname"]] = addrs
        return result
    except (OSError, ValueError, subprocess.SubprocessError):
        return {}


def probe_ports(ports: Optional[List[int]] = None,
                duration: float = 1.5) -> List[PortProbe]:
    """Listen briefly on known LiDAR data ports and report what is arriving.

    All sockets are polled together via `select`, so the whole sweep costs
    `duration` rather than `duration x len(ports)`.
    """
    ports = ports or sorted(KNOWN_LIDAR_PORTS)
    probes: Dict[int, PortProbe] = {}
    sockets: Dict[int, socket.socket] = {}

    for port in ports:
        vendor, description = KNOWN_LIDAR_PORTS.get(port, ("Unknown", "LiDAR data"))
        probe = PortProbe(port=port, vendor=vendor, description=description,
                          status="silent")
        probes[port] = probe
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Deliberately NOT SO_REUSEPORT: co-binding would split an active
            # stream with a running driver. Failing to bind is the signal we want.
            sock.setblocking(False)
            sock.bind(("0.0.0.0", port))
            sockets[port] = sock
        except OSError as exc:
            if exc.errno in (errno.EADDRINUSE, errno.EACCES):
                probe.status = "in_use"
                probe.detail = ("Another process already holds this port -- "
                                "the vendor driver is most likely running.")
            else:
                probe.status = "error"
                probe.detail = str(exc)

    if sockets:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select(list(sockets.values()), [], [],
                                           max(0.0, min(0.25, remaining)))
            for sock in readable:
                try:
                    _data, addr = sock.recvfrom(65535)
                except OSError:
                    continue
                port = sock.getsockname()[1]
                probe = probes[port]
                probe.status = "receiving"
                probe.packets += 1
                if addr[0] not in probe.senders:
                    probe.senders.append(addr[0])

    for sock in sockets.values():
        try:
            sock.close()
        except OSError:
            pass

    return [probes[p] for p in ports]


def _host_is_on(network: str, interfaces: List[Interface]) -> bool:
    net = ipaddress.ip_network(network)
    for iface in interfaces:
        for addr in iface.addresses:
            try:
                if ipaddress.ip_interface(addr).ip in net:
                    return True
            except ValueError:
                continue
    return False


def diagnose(duration: float = 1.5, ros_available: bool = False,
             ros_lidar_topics: Optional[List[str]] = None) -> dict:
    """Full report: interfaces, port activity, and what to do about it."""
    interfaces = ethernet_interfaces()
    probes = probe_ports(duration=duration)
    ros_lidar_topics = ros_lidar_topics or []

    active = [p for p in probes if p.status == "receiving"]
    held = [p for p in probes if p.status == "in_use"]
    findings: List[dict] = []

    # --- what is on the wire ---------------------------------------------
    for probe in active:
        findings.append({
            "level": "found",
            "title": f"{probe.vendor} LiDAR streaming on UDP {probe.port}",
            "detail": (
                f"{probe.packets} packets from {', '.join(probe.senders)} "
                f"in {duration:.1f}s ({probe.description}), and nothing is "
                f"consuming them."
            ),
            "action": _driver_hint(probe.vendor),
        })

    for probe in held:
        findings.append({
            "level": "info",
            "title": f"UDP {probe.port} is already in use ({probe.vendor})",
            "detail": probe.detail,
            "action": (
                "A driver appears to be running. If its topic is not listed "
                "above, check that the UI process can see the same ROS graph."
            ) if not ros_lidar_topics else "",
        })

    # --- cable / addressing ----------------------------------------------
    wired_up = [i for i in interfaces if i.has_carrier]
    if not wired_up and interfaces:
        findings.append({
            "level": "warn",
            "title": "No Ethernet cable detected",
            "detail": "Wired interfaces: " + ", ".join(
                f"{i.name} ({i.state})" for i in interfaces),
            "action": "Connect the LiDAR to a wired port and check the link LED.",
        })

    for iface in wired_up:
        if not iface.addresses:
            findings.append({
                "level": "warn",
                "title": f"{iface.name}: cable connected but no IP address",
                "detail": "The link is up but the interface has no IPv4 address, "
                          "so the sensor cannot reach this host.",
                "action": (
                    f"Give it a static address on the sensor's subnet, e.g.\n"
                    f"sudo ip addr add 192.168.1.50/24 dev {iface.name}\n"
                    f"sudo ip link set {iface.name} up"
                ),
            })

    # A Livox that is streaming proves addressing already works; only advise on
    # the subnet when we have a cable but no traffic.
    if wired_up and not active and not held:
        livox = VENDOR_NETWORKS["Livox"]
        if not _host_is_on(livox["network"], interfaces):
            names = ", ".join(i.name for i in wired_up)
            findings.append({
                "level": "warn",
                "title": "Host is not on the sensor's default subnet",
                "detail": (
                    f"No interface holds an address in {livox['network']}. "
                    f"{livox['note']}"
                ),
                "action": (
                    f"sudo ip addr add {livox['suggested_host']}/24 dev {names}"
                ),
            })

    # --- ROS state --------------------------------------------------------
    if ros_lidar_topics:
        findings.append({
            "level": "ok",
            "title": f"{len(ros_lidar_topics)} LiDAR topic(s) available over ROS 2",
            "detail": ", ".join(ros_lidar_topics),
            "action": "Select one in the LiDAR dropdown to start streaming.",
        })
    elif not ros_available:
        findings.append({
            "level": "warn",
            "title": "ROS 2 is not available to this process",
            "detail": "Live network LiDARs are served through their ROS driver, "
                      "so the UI needs to see the ROS graph.",
            "action": ("source /opt/ros/$ROS_DISTRO/setup.bash\n"
                       "ui/.venv/bin/python -m uvicorn ui.app:app --port 8000"),
        })

    if not findings:
        findings.append({
            "level": "info",
            "title": "No Ethernet LiDAR detected",
            "detail": "No traffic on known LiDAR ports and no ROS topics found. "
                      "USB cameras and the simulated rig are unaffected.",
            "action": "",
        })

    return {
        "interfaces": [vars(i) | {"state": i.state} for i in interfaces],
        "ports": [vars(p) for p in probes],
        "findings": findings,
        "probe_seconds": duration,
    }


def _driver_hint(vendor: str) -> str:
    """The command that turns raw packets into a topic this tool can consume."""
    hints = {
        "Livox": (
            "Start the Livox driver so the points become a ROS topic:\n"
            "ros2 launch livox_ros_driver2 msg_MID360_launch.py\n"
            "Make sure its config host_net_info IP matches this machine."
        ),
        "Ouster": "ros2 launch ouster_ros driver.launch.py sensor_hostname:=<ip>",
        "Velodyne or Hesai": (
            "Velodyne: ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py\n"
            "Hesai:    ros2 launch hesai_ros_driver start.py"
        ),
        "Velodyne": "ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py",
        "Hesai": "ros2 launch hesai_ros_driver start.py",
        "RoboSense": "ros2 launch rslidar_sdk start.py",
    }
    return hints.get(vendor, "Start the vendor's ROS 2 driver for this sensor.")
