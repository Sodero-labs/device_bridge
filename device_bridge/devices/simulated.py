"""Hardware-free stand-ins, for trying the bridge and the controller without devices.

simulated_camera   a moving colour pattern in front of a flat wall 2 m away
simulated_imu      a level sensor turning slowly about its z axis
simulated_wheels   joints that follow their velocity commands exactly
"""
from __future__ import annotations

import math
import time
from typing import Optional, Sequence

import numpy as np

from .base import Camera, CameraInfo, Device, Imu, ImuSample, JointState, RgbdFrame, Wheels


class SimulatedCamera(Device, Camera):
    def __init__(self, name: str, options: dict):
        super().__init__(name, options)
        self.width = int(options.get("width", 424))
        self.height = int(options.get("height", 240))
        self.fps = float(options.get("fps", 15))
        self._next_frame = 0.0

    def open(self) -> None:
        self._next_frame = time.monotonic()

    def close(self) -> None:
        pass

    def camera_info(self) -> CameraInfo:
        focal = self.width / (2 * math.tan(math.radians(69.0) / 2))   # D435-like 69 deg field of view
        return CameraInfo(self.width, self.height, focal, focal, self.width / 2, self.height / 2)

    def read_frame(self, timeout_s: float) -> Optional[RgbdFrame]:
        wait = self._next_frame - time.monotonic()
        if wait > timeout_s:
            time.sleep(timeout_s)
            return None
        time.sleep(max(0.0, wait))
        self._next_frame = max(self._next_frame + 1 / self.fps, time.monotonic())
        shift = int(time.monotonic() * 40) % self.width
        columns = (np.arange(self.width) + shift) % 256
        rgb = np.empty((self.height, self.width, 3), np.uint8)
        rgb[..., 0] = columns
        rgb[..., 1] = np.arange(self.height)[:, None] % 256
        rgb[..., 2] = 128
        depth = np.full((self.height, self.width), 2000, np.uint16)
        return RgbdFrame(rgb=rgb, depth_mm=depth, capture_unix_ns=time.time_ns())


class SimulatedImu(Device, Imu):
    def __init__(self, name: str, options: dict):
        super().__init__(name, options)
        self.rate_hz = float(options.get("rate_hz", 100))
        self.yaw_rate = float(options.get("yaw_rate", 0.1))   # rad/s
        self._start = 0.0
        self._next_sample = 0.0

    def open(self) -> None:
        self._start = self._next_sample = time.monotonic()

    def close(self) -> None:
        pass

    def read_imu(self) -> Optional[ImuSample]:
        now = time.monotonic()
        if now < self._next_sample:
            return None
        self._next_sample = max(self._next_sample + 1 / self.rate_hz, now)
        yaw = self.yaw_rate * (now - self._start)
        return ImuSample(gyro=(0.0, 0.0, self.yaw_rate), accel=(0.0, 0.0, 9.81),
                         orientation_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                         source_time_ns=time.monotonic_ns())


class SimulatedWheels(Device, Wheels):
    def __init__(self, name: str, options: dict):
        super().__init__(name, options)
        self._count = int(options.get("joints", 2))
        self.rate_hz = float(options.get("rate_hz", 100))
        self._positions = [0.0] * self._count
        self._velocities = [0.0] * self._count
        self._last = 0.0

    @property
    def joint_count(self) -> int:
        return self._count

    def open(self) -> None:
        self._last = time.monotonic()

    def close(self) -> None:
        pass

    def read_joints(self) -> Optional[JointState]:
        now = time.monotonic()
        if now - self._last < 1 / self.rate_hz:
            return None
        dt, self._last = now - self._last, now
        self._positions = [p + v * dt for p, v in zip(self._positions, self._velocities)]
        return JointState(positions=list(self._positions), velocities=list(self._velocities),
                          efforts=[0.0] * self._count, source_time_ns=time.monotonic_ns())

    def set_velocities(self, rad_per_s: Sequence[float]) -> None:
        self._velocities = [float(v) for v in rad_per_s]

    def stop(self) -> None:
        self._velocities = [0.0] * self._count
