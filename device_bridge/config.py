"""Configuration file: which devices exist and which stream each one feeds.

    robot_name: tb3          # topic namespace TM/<robot_name>/...
    network:                 # optional; these are the defaults
      bind: 0.0.0.0
      command_port: 53333
      state_port: 53334
      camera_port: 53335
    devices:                 # hardware, by a name you choose
      d435:   {driver: realsense, width: 424, height: 240, fps: 15}
      opencr: {driver: turtlebot3_opencr}
    streams:                 # what the controller receives; each one is optional
      camera: {device: d435, frame_id: cam0_optical}
      imu:    {device: opencr}
      wheels: {device: opencr, command_timeout_s: 0.5}
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional

import yaml

from . import protocol


@dataclass
class Network:
    bind: str = "0.0.0.0"
    command_port: int = protocol.COMMAND_PORT
    state_port: int = protocol.STATE_PORT
    camera_port: int = protocol.CAMERA_PORT


@dataclass
class CameraStream:
    device: str
    frame_id: str = "cam0_optical"
    send_capture_time: bool = True   # needs NTP-synchronized clocks; false: the controller uses arrival time


@dataclass
class ImuStream:
    device: str


@dataclass
class WheelStream:
    device: str
    command_timeout_s: float = 0.5   # stop the wheels when commands pause this long


@dataclass
class Config:
    robot_name: str
    network: Network = field(default_factory=Network)
    devices: dict[str, dict] = field(default_factory=dict)
    camera: Optional[CameraStream] = None
    imu: Optional[ImuStream] = None
    wheels: Optional[WheelStream] = None
    state_poll_hz: float = 200.0     # how often the IMU and wheel devices are polled


def load_config(path: str) -> Config:
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    _only(data, "configuration", {"robot_name", "network", "devices", "streams", "state_poll_hz"})

    robot_name = str(data.get("robot_name", ""))
    if not re.fullmatch(r"[A-Za-z0-9_\-]+", robot_name):
        raise ValueError("robot_name is required (letters, digits, _ and -)")

    network = Network(**_only(data.get("network") or {}, "network", set(Network.__dataclass_fields__)))
    devices = data.get("devices") or {}
    if not isinstance(devices, dict) or not devices:
        raise ValueError("devices: list at least one device")

    streams = data.get("streams") or {}
    _only(streams, "streams", {"camera", "imu", "wheels"})
    config = Config(robot_name=robot_name, network=network, devices=devices,
                    state_poll_hz=float(data.get("state_poll_hz", 200.0)))
    if "camera" in streams:
        config.camera = CameraStream(**_only(streams["camera"], "streams.camera",
                                             {"device", "frame_id", "send_capture_time"}))
        if not 0 < len(config.camera.frame_id.encode()) <= protocol.FRAME_ID_MAX:
            raise ValueError(f"streams.camera.frame_id must be 1..{protocol.FRAME_ID_MAX} bytes")
    if "imu" in streams:
        config.imu = ImuStream(**_only(streams["imu"], "streams.imu", {"device"}))
    if "wheels" in streams:
        config.wheels = WheelStream(**_only(streams["wheels"], "streams.wheels", {"device", "command_timeout_s"}))
    if not (config.camera or config.imu or config.wheels):
        raise ValueError("streams: configure at least one of camera, imu, wheels")

    for name, stream in (("camera", config.camera), ("imu", config.imu), ("wheels", config.wheels)):
        if stream is not None and stream.device not in devices:
            raise ValueError(f"streams.{name}.device '{stream.device}' is not listed under devices")
    return config


def _only(block, where: str, allowed: set) -> dict:
    """Reject unknown keys, so a typo fails loudly instead of being ignored."""
    if not isinstance(block, dict):
        raise ValueError(f"{where} must be a mapping")
    unknown = set(block) - allowed
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {', '.join(sorted(unknown))}")
    return block
