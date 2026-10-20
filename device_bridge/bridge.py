"""Serves the configured devices to the controller over the LAN.

Each stream runs in its own thread:

    camera   camera device -> camera_pair, and camera_info once a second   port 53335
    state    IMU and wheel devices -> imu_state and sim_state               port 53334
    command  sim_command -> wheel velocities, stopped by a watchdog         port 53333

The bridge binds every socket; the controller's gateways connect to it, exactly
as they connect to the simulator.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import zmq

from . import protocol
from .config import Config
from .devices import Camera, Device, Imu, Wheels

log = logging.getLogger(__name__)


class Bridge:
    def __init__(self, config: Config, devices: dict[str, Device]):
        self.config = config
        self.camera: Optional[Camera] = _stream_device(devices, config.camera, Camera, "camera")
        self.imu: Optional[Imu] = _stream_device(devices, config.imu, Imu, "imu")
        self.wheels: Optional[Wheels] = _stream_device(devices, config.wheels, Wheels, "wheels")
        self._context = zmq.Context()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.counters = {"camera_pair": 0, "imu_state": 0, "sim_state": 0, "sim_command": 0}

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        net = self.config.network
        if self.camera:
            self._spawn("camera", self._camera_loop, self._bind(zmq.PUB, net.camera_port))
        if self.imu or self.wheels:
            self._spawn("state", self._state_loop, self._bind(zmq.PUB, net.state_port))
        if self.wheels:
            self._spawn("command", self._command_loop, self._bind(zmq.SUB, net.command_port))
        log.info("serving robot '%s' on %s (camera %s, state %s, command %s)",
                 self.config.robot_name, net.bind,
                 net.camera_port if self.camera else "-",
                 net.state_port if (self.imu or self.wheels) else "-",
                 net.command_port if self.wheels else "-")

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=3.0)
        self._context.term()

    def _bind(self, kind: int, port: int) -> zmq.Socket:
        socket = self._context.socket(kind)
        socket.setsockopt(zmq.LINGER, 0)
        if kind == zmq.PUB:
            socket.setsockopt(zmq.SNDHWM, 2)   # a slow receiver gets new data, not a backlog
        socket.bind(f"tcp://{self.config.network.bind}:{port}")
        return socket

    def _spawn(self, name: str, target, socket: zmq.Socket) -> None:
        def run():
            try:
                target(socket)
            except Exception:
                log.exception("%s stream stopped", name)
            finally:
                socket.close()
        thread = threading.Thread(target=run, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    # -- camera --------------------------------------------------------------
    def _camera_loop(self, pub: zmq.Socket) -> None:
        robot, frame_id = self.config.robot_name, self.config.camera.frame_id
        info = self.camera.camera_info()
        info.frame_id = frame_id
        log.info("camera %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f frame_id=%s "
                 "(use these intrinsics in the controller's camera settings)",
                 info.width, info.height, info.fx, info.fy, info.cx, info.cy, frame_id)
        pair_topic = protocol.topic(robot, protocol.CAMERA_PAIR)
        info_topic = protocol.topic(robot, protocol.CAMERA_INFO)
        fps = float(getattr(self.camera, "fps", 0.0))
        seq, info_sent, last_frame = 0, 0.0, time.monotonic()

        while not self._stop.is_set():
            frame = self.camera.read_frame(timeout_s=1.0)
            now = time.monotonic()
            if frame is None:
                if now - last_frame > 5.0:
                    log.warning("no camera frame for %.0f s", now - last_frame)
                    last_frame = now
                continue
            last_frame = now
            seq += 1
            stamp = frame.capture_unix_ns if self.config.camera.send_capture_time else 0
            payload = protocol.pack_camera_pair(seq, frame.rgb, frame.depth_mm, stamp, frame_id)
            pub.send(pair_topic + payload, copy=False)
            self.counters["camera_pair"] += 1
            if now - info_sent >= 1.0:
                pub.send(info_topic + protocol.camera_info_json(info, fps, seq))
                info_sent = now

    # -- IMU and wheel state -------------------------------------------------
    def _state_loop(self, pub: zmq.Socket) -> None:
        robot = self.config.robot_name
        imu_topic = protocol.topic(robot, protocol.IMU_STATE)
        state_topic = protocol.topic(robot, protocol.SIM_STATE)
        period = 1.0 / self.config.state_poll_hz
        imu_seq = state_seq = 0

        while not self._stop.is_set():
            started = time.monotonic()
            sample = self.imu.read_imu() if self.imu else None
            if sample is not None:
                imu_seq += 1
                pub.send(imu_topic + protocol.pack_imu_state(
                    sample.source_time_ns, imu_seq, sample.orientation_xyzw, sample.gyro, sample.accel))
                self.counters["imu_state"] += 1
            joints = self.wheels.read_joints() if self.wheels else None
            if joints is not None:
                state_seq += 1
                pub.send(state_topic + protocol.pack_sim_state(
                    joints.source_time_ns, state_seq, joints.positions, joints.velocities, joints.efforts))
                self.counters["sim_state"] += 1
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    # -- wheel commands ------------------------------------------------------
    def _command_loop(self, sub: zmq.Socket) -> None:
        sub.setsockopt(zmq.SUBSCRIBE, b"TM/")
        robot = self.config.robot_name.encode()
        timeout_s = self.config.wheels.command_timeout_s
        count = self.wheels.joint_count
        driving = False          # wheels currently commanded
        last_command = 0.0
        reported_modes: set = set()

        while not self._stop.is_set():
            if sub.poll(timeout=20):
                frames = sub.recv_multipart()
                if len(frames) != 4 or frames[1] != robot or frames[2] != protocol.SIM_COMMAND:
                    continue
                try:
                    command = protocol.unpack_sim_command(frames[3])
                except ValueError as error:
                    log.warning("ignored sim_command: %s", error)
                    continue
                self.counters["sim_command"] += 1
                velocities = wheel_velocities(command, count)
                if velocities is not None:
                    self.wheels.set_velocities(velocities)
                    if not driving:
                        log.info("wheels commanded")
                    driving, last_command = True, time.monotonic()
                else:
                    modes = tuple(command.modes[:count])
                    if modes not in reported_modes and not _is_servo_off(command, count):
                        log.warning("modes %s are not velocity control: wheels stopped", list(modes))
                        reported_modes.add(modes)
                    if driving:
                        self.wheels.stop()
                        driving = False
                        log.info("wheels released (controller servo off)")
            if driving and time.monotonic() - last_command > timeout_s:
                self.wheels.stop()
                driving = False
                log.warning("no wheel command for %.2f s: wheels stopped", timeout_s)


def wheel_velocities(command: protocol.JointCommand, count: int) -> Optional[list[float]]:
    """Velocities to drive with, or None to stop and release the wheels.

    The controller sends VELOCITY while its drive is on and a limp IMPEDANCE
    (kp = 0) when it switches the drive off. Anything else (position or torque
    control, simulator-only modes) is not something these wheels do."""
    if len(command.modes) < count:
        return None
    if all(mode == protocol.MODE_VELOCITY for mode in command.modes[:count]):
        return command.velocities[:count]
    return None


def _is_servo_off(command: protocol.JointCommand, count: int) -> bool:
    return all(mode == protocol.MODE_IMPEDANCE and kp == 0.0
               for mode, kp in zip(command.modes[:count], command.kp[:count]))


def _stream_device(devices: dict[str, Device], stream, capability: type, name: str):
    if stream is None:
        return None
    device = devices[stream.device]
    if not isinstance(device, capability):
        raise ValueError(f"streams.{name}: device '{stream.device}' ({type(device).__name__}) "
                         f"does not provide {capability.__name__}")
    return device
