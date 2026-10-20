"""What a device driver provides.

A driver is one class for one piece of hardware. It subclasses Device and the
capability classes it supports:

    Camera  aligned RGB-D frames
    Imu     angular velocity, acceleration and attitude
    Wheels  joint states in, joint velocity commands out

One driver may provide several capabilities. The TurtleBot3 OpenCR board, for
example, is both the wheel controller and the IMU, and its serial port can have
only one owner, so a single driver serves both.

Units and frames are fixed by the protocol, so convert inside the driver:
    angles rad, angular velocity rad/s, acceleration m/s^2 (gravity included),
    depth millimetres, times in nanoseconds.
IMU axes are the sensor's own and camera frames are optical (x right, y down,
z forward). Do not rotate into the robot's base frame here: the controller owns
the mounting transforms (extrinsics).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


class Device(ABC):
    """Base class of every driver. `options` is the driver's block from the
    configuration file (everything except `driver`)."""

    def __init__(self, name: str, options: dict):
        self.name = name
        self.options = options

    @abstractmethod
    def open(self) -> None:
        """Connect to the hardware. Raise an exception with a clear message on failure."""

    @abstractmethod
    def close(self) -> None:
        """Stop the hardware and release it. Must be safe to call more than once."""


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
@dataclass
class CameraInfo:
    """Pinhole intrinsics of the images read_frame() returns."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str = "cam0_optical"


@dataclass
class RgbdFrame:
    rgb: np.ndarray        # HxWx3 uint8, RGB order
    depth_mm: np.ndarray   # HxW uint16 millimetres aligned to rgb, 0 = no depth
    capture_unix_ns: int   # capture time on the synchronized wall clock


class Camera(ABC):
    @abstractmethod
    def camera_info(self) -> CameraInfo:
        """Intrinsics of the aligned output; valid after open()."""

    @abstractmethod
    def read_frame(self, timeout_s: float) -> Optional[RgbdFrame]:
        """Block for the next frame. Return None when none arrived within the timeout."""


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------
@dataclass
class ImuSample:
    gyro: Sequence[float]               # rad/s, sensor axes
    accel: Sequence[float]              # m/s^2 including gravity, sensor axes
    orientation_xyzw: Sequence[float]   # sensor attitude in the world, unit quaternion
    source_time_ns: int                 # sensor or host monotonic clock; must increase


class Imu(ABC):
    @abstractmethod
    def read_imu(self) -> Optional[ImuSample]:
        """Return the newest sample, or None when there is nothing new since the
        last call. Must not block for long: the bridge calls it at its state rate."""


# ---------------------------------------------------------------------------
# Wheels (or any velocity-controlled joints)
# ---------------------------------------------------------------------------
@dataclass
class JointState:
    positions: Sequence[float]    # rad, continuous (no wrap at 2*pi)
    velocities: Sequence[float]   # rad/s
    efforts: Sequence[float]      # Nm, or zeros when the hardware cannot measure it
    source_time_ns: int           # monotonic clock; must increase


class Wheels(ABC):
    @property
    @abstractmethod
    def joint_count(self) -> int:
        """Number of joints, in the order the controller's gateway lists them."""

    @abstractmethod
    def read_joints(self) -> Optional[JointState]:
        """Return the newest state, or None when there is nothing new since the last call."""

    @abstractmethod
    def set_velocities(self, rad_per_s: Sequence[float]) -> None:
        """Drive every joint at these velocities (enables the motors if needed)."""

    @abstractmethod
    def stop(self) -> None:
        """Zero velocity and release the motors (torque off)."""
