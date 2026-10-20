"""Intel RealSense D400 cameras (tested with the D435) through pyrealsense2.

The depth image is aligned to the colour image, so both share the colour
camera's intrinsics. The D435 colour stream is close to distortion free, which
matches the protocol's rectified pinhole model.

Options:
    serial  device serial number, or "auto" when exactly one camera is connected
    width   image width for colour and depth (D435: 424x240, 640x480, ...)
    height  image height
    fps     frames per second (6, 15, 30, ...)
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs

from .base import Camera, CameraInfo, Device, RgbdFrame


class RealSenseCamera(Device, Camera):
    def __init__(self, name: str, options: dict):
        super().__init__(name, options)
        self.serial = str(options.get("serial", "auto"))
        self.width = int(options.get("width", 424))
        self.height = int(options.get("height", 240))
        self.fps = int(options.get("fps", 15))
        self._pipeline: Optional[rs.pipeline] = None
        self._align: Optional[rs.align] = None
        self._info: Optional[CameraInfo] = None
        self._depth_scale_mm = 1.0   # millimetres per raw depth unit

    def open(self) -> None:
        config = rs.config()
        config.enable_device(self._find_serial())
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        self._depth_scale_mm = profile.get_device().first_depth_sensor().get_depth_scale() * 1000.0
        color = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._info = CameraInfo(width=color.width, height=color.height,
                                fx=color.fx, fy=color.fy, cx=color.ppx, cy=color.ppy)
        self._align = rs.align(rs.stream.color)
        self._pipeline = pipeline

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline = None

    def camera_info(self) -> CameraInfo:
        if self._info is None:
            raise RuntimeError("camera is not open")
        return self._info

    def read_frame(self, timeout_s: float) -> Optional[RgbdFrame]:
        ok, frames = self._pipeline.try_wait_for_frames(int(timeout_s * 1000))
        if not ok:
            return None
        frames = self._align.process(frames)
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            return None

        # Copy out of the SDK's buffers: they are recycled for later frames.
        rgb = np.array(color.get_data(), dtype=np.uint8, copy=True)
        depth_raw = np.asanyarray(depth.get_data())
        if abs(self._depth_scale_mm - 1.0) < 1e-6:
            depth_mm = np.array(depth_raw, dtype=np.uint16, copy=True)
        else:
            depth_mm = np.clip(depth_raw * self._depth_scale_mm, 0, 65535).astype(np.uint16)
        return RgbdFrame(rgb=rgb, depth_mm=depth_mm, capture_unix_ns=self._capture_time_ns(color))

    def _find_serial(self) -> str:
        found = [d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()]
        if self.serial != "auto":
            if self.serial not in found:
                raise RuntimeError(f"RealSense {self.serial} is not connected (found: {found or 'none'})")
            return self.serial
        if len(found) != 1:
            raise RuntimeError(f"serial: auto needs exactly one RealSense camera, found {len(found)}; "
                               "set the camera's serial number")
        return found[0]

    @staticmethod
    def _capture_time_ns(frame: rs.frame) -> int:
        # Global time (the D400 default) maps the camera clock onto the host's
        # wall clock, which is what the protocol carries. A camera that only
        # has its own clock falls back to the arrival time.
        domain = frame.get_frame_timestamp_domain()
        if domain in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
            return int(frame.get_timestamp() * 1_000_000)
        return time.time_ns()
