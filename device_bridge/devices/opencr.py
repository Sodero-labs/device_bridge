"""TurtleBot3 Burger OpenCR board: the two wheels and the built-in IMU.

The board runs the ROBOTIS TurtleBot3 ROS 2 firmware and is reached over USB
with DYNAMIXEL Protocol 2.0 (device ID 200, 1 Mbps). Wheels and IMU share this
one serial port, so this driver provides both and opens the port exclusively.
A worker thread talks to the board at `poll_hz`:

    read the control table 10..149   wheel ticks and speeds, IMU, status
    write cmd_vel (address 150)      the requested twist
    write torque enable (149)
    write the heartbeat (19)         the firmware stops the wheels by itself
                                     when this stops changing for 500 ms

Addresses and conversions follow the official turtlebot3_node and the
actuator_gateway turtlebot3 backend.

Options:
    port                serial device (default /dev/turtlebot3_opencr)
    poll_hz             board I/O rate, 20..100 (default 100)
    max_linear_speed    m/s, at most 0.22 (default 0.15)
    max_angular_speed   rad/s, at most 2.84 (default 1.0)
"""
from __future__ import annotations

import logging
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import serial

from .base import Device, Imu, ImuSample, JointState, Wheels

log = logging.getLogger(__name__)

# Burger geometry and limits, compiled into the stock firmware.
WHEEL_RADIUS = 0.033        # m
WHEEL_SEPARATION = 0.160    # m
MAX_WHEEL_SPEED = 0.22 / WHEEL_RADIUS   # rad/s
FIRMWARE_MAX_LINEAR = 0.22  # m/s
FIRMWARE_MAX_ANGULAR = 2.84  # rad/s

TICK_TO_RAD = 2 * math.pi / 4096
VELOCITY_UNIT_TO_RAD_S = 0.229 * 2 * math.pi / 60   # XL430 velocity unit is 0.229 rpm

# Control table (TurtleBot3 ROS 2 OpenCR firmware).
ADDR_MODEL_INFO = 2
ADDR_DEBUG_MODE = 14
ADDR_HEARTBEAT = 19
ADDR_FEEDBACK = 10          # first byte of the block read every cycle
FEEDBACK_SIZE = 140         # bytes 10..149
ADDR_TORQUE_ENABLE = 149
ADDR_CMD_VEL = 150          # 6 x int32: linear x,y,z, angular x,y,z in 0.01 SI units
ADDR_PROFILE_ACCEL = 174    # 2 x uint32, 0 = no limit
OPENCR_ID = 200
OPENCR_MODEL = 0x5000
BURGER_MODEL_INFO = 1


@dataclass
class Feedback:
    board_millis: int
    battery_voltage: float
    motors_connected: bool
    torque_enabled: bool
    ticks: tuple[int, int]            # left, right
    velocity_units: tuple[int, int]   # left, right
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def decode_feedback(data: bytes) -> Feedback:
    """Decode control table bytes 10..149."""
    def at(address: int, fmt: str):
        return struct.unpack_from("<" + fmt, data, address - ADDR_FEEDBACK)

    w, x, y, z = at(96, "4f")
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not 0.5 < norm < 2.0:
        raise ValueError("OpenCR reported an invalid orientation")
    return Feedback(
        board_millis=at(10, "I")[0],
        battery_voltage=at(42, "I")[0] * 0.01,
        motors_connected=at(148, "B")[0] != 0,
        torque_enabled=at(149, "B")[0] != 0,
        velocity_units=at(128, "2i"),
        ticks=at(136, "2i"),
        gyro=at(60, "3f"),
        accel=at(72, "3f"),
        orientation_xyzw=(x / norm, y / norm, z / norm, w / norm),
    )


def wheels_to_twist(left: float, right: float, max_linear: float, max_angular: float) -> tuple[float, float]:
    """Wheel speeds [rad/s] to (linear m/s, angular rad/s), scaled down together
    so the path's curvature is kept when a limit is reached."""
    scale = max(1.0, abs(left) / MAX_WHEEL_SPEED, abs(right) / MAX_WHEEL_SPEED)
    left, right = left / scale, right / scale
    linear = WHEEL_RADIUS * (left + right) / 2
    angular = WHEEL_RADIUS * (right - left) / WHEEL_SEPARATION
    scale = max(1.0, abs(linear) / max_linear, abs(angular) / max_angular)
    return linear / scale, angular / scale


class TurtleBot3OpenCR(Device, Imu, Wheels):
    def __init__(self, name: str, options: dict):
        super().__init__(name, options)
        self.port = str(options.get("port", "/dev/turtlebot3_opencr"))
        self.poll_hz = float(options.get("poll_hz", 100))
        self.max_linear = min(float(options.get("max_linear_speed", 0.15)), FIRMWARE_MAX_LINEAR)
        self.max_angular = min(float(options.get("max_angular_speed", 1.0)), FIRMWARE_MAX_ANGULAR)
        if not 20 <= self.poll_hz <= 100:
            raise ValueError("poll_hz must be between 20 and 100")

        self._board: Optional[DynamixelBoard] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._feedback: Optional[Feedback] = None
        self._feedback_seq = 0
        self._feedback_time_ns = 0
        self._imu_seen = 0
        self._joints_seen = 0
        self._positions = [0.0, 0.0]     # continuous wheel angles [rad]
        self._twist = (0.0, 0.0)         # requested linear, angular
        self._enable = False

    # -- Device ---------------------------------------------------------------
    def open(self) -> None:
        board = DynamixelBoard(self.port, OPENCR_ID)
        model, firmware = board.ping()
        if model != OPENCR_MODEL or struct.unpack("<I", board.read(ADDR_MODEL_INFO, 4))[0] != BURGER_MODEL_INFO:
            board.close()
            raise RuntimeError(f"{self.port} is not an OpenCR running the TurtleBot3 Burger firmware")
        # Start from a known safe state: wheels stopped, torque off, the
        # firmware's heartbeat watchdog active (debug mode off), no accel limit.
        board.write(ADDR_CMD_VEL, _cmd_vel_bytes(0.0, 0.0))
        board.write(ADDR_TORQUE_ENABLE, b"\x00")
        board.write(ADDR_DEBUG_MODE, b"\x00")
        board.write(ADDR_PROFILE_ACCEL, bytes(8))
        log.info("OpenCR on %s: firmware %d, torque off", self.port, firmware)

        self._board = board
        self._running = True
        self._thread = threading.Thread(target=self._worker, name="opencr", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._board is not None:
            try:
                self._board.write(ADDR_CMD_VEL, _cmd_vel_bytes(0.0, 0.0))
                self._board.write(ADDR_TORQUE_ENABLE, b"\x00")
            except Exception:
                pass   # the firmware's heartbeat watchdog stops the wheels anyway
            self._board.close()
            self._board = None

    # -- Imu -----------------------------------------------------------------
    def read_imu(self) -> Optional[ImuSample]:
        with self._lock:
            if self._feedback is None or self._feedback_seq == self._imu_seen:
                return None
            self._imu_seen = self._feedback_seq
            f = self._feedback
            return ImuSample(gyro=f.gyro, accel=f.accel, orientation_xyzw=f.orientation_xyzw,
                             source_time_ns=self._feedback_time_ns)

    # -- Wheels --------------------------------------------------------------
    @property
    def joint_count(self) -> int:
        return 2   # left, right

    def read_joints(self) -> Optional[JointState]:
        with self._lock:
            if self._feedback is None or self._feedback_seq == self._joints_seen:
                return None
            self._joints_seen = self._feedback_seq
            speeds = [u * VELOCITY_UNIT_TO_RAD_S for u in self._feedback.velocity_units]
            return JointState(positions=list(self._positions), velocities=speeds,
                              efforts=[0.0, 0.0],   # the XL430 reports load, not torque
                              source_time_ns=self._feedback_time_ns)

    def set_velocities(self, rad_per_s: Sequence[float]) -> None:
        left, right = rad_per_s
        with self._lock:
            self._twist = wheels_to_twist(left, right, self.max_linear, self.max_angular)
            self._enable = True

    def stop(self) -> None:
        with self._lock:
            self._twist = (0.0, 0.0)
            self._enable = False

    # -- board I/O -----------------------------------------------------------
    def _worker(self) -> None:
        # Every serial exchange costs a USB round trip, so a cycle reads the
        # board and writes only what changed: cmd_vel on a new value (refreshed
        # every 0.2 s) and the heartbeat every 0.1 s, well inside the
        # firmware's 0.5 s watchdog.
        period = 1.0 / self.poll_hz
        heartbeat = 0
        torque_on: Optional[bool] = None   # unknown until written: forces the first write
        sent_cmd, sent_cmd_at, sent_heartbeat_at = None, 0.0, 0.0
        last_warning = 0.0
        next_cycle = time.monotonic()
        while self._running:
            try:
                self._receive_feedback()
                with self._lock:
                    enable, (linear, angular) = self._enable, self._twist
                    motors_connected = self._feedback.motors_connected
                if enable != torque_on:
                    if enable:   # zero the twist before the motors wake up
                        self._board.write(ADDR_CMD_VEL, _cmd_vel_bytes(0.0, 0.0))
                        sent_cmd = (0.0, 0.0)
                        if not motors_connected:
                            log.warning("wheel motors are not powered: check the robot's power switch")
                    self._board.write(ADDR_TORQUE_ENABLE, b"\x01" if enable else b"\x00")
                    torque_on = enable
                    log.info("wheel torque %s", "on" if enable else "off")
                now = time.monotonic()
                cmd = (linear, angular) if enable else (0.0, 0.0)
                if cmd != sent_cmd or now - sent_cmd_at >= 0.2:
                    self._board.write(ADDR_CMD_VEL, _cmd_vel_bytes(*cmd))
                    sent_cmd, sent_cmd_at = cmd, now
                if now - sent_heartbeat_at >= 0.1:
                    heartbeat = (heartbeat + 1) & 0xFF
                    self._board.write(ADDR_HEARTBEAT, bytes([heartbeat]))
                    sent_heartbeat_at = now
            except (OSError, RuntimeError, ValueError) as error:
                # Without new heartbeats the firmware stops the wheels by itself.
                if time.monotonic() - last_warning > 5.0:
                    log.warning("OpenCR I/O failed: %s", error)
                    last_warning = time.monotonic()
                torque_on, sent_cmd = None, None
                time.sleep(0.2)
            next_cycle += period
            time.sleep(max(0.0, next_cycle - time.monotonic()))
            if time.monotonic() - next_cycle > 0.5:   # fell far behind: resynchronize
                next_cycle = time.monotonic()

    def _receive_feedback(self) -> None:
        feedback = decode_feedback(self._board.read(ADDR_FEEDBACK, FEEDBACK_SIZE))
        now_ns = time.monotonic_ns()
        with self._lock:
            previous = self._feedback
            if previous is not None and feedback.board_millis == previous.board_millis:
                return   # the board has not refreshed its table yet: not a new sample
            if previous is None:
                self._positions = [t * TICK_TO_RAD for t in feedback.ticks]
            elif feedback.torque_enabled == previous.torque_enabled:
                # The XL430 may reset its multi-turn count when torque switches,
                # so only accumulate between samples with the same torque state.
                for i in range(2):
                    delta = (feedback.ticks[i] - previous.ticks[i] + 2**31) % 2**32 - 2**31
                    self._positions[i] += delta * TICK_TO_RAD
            self._feedback = feedback
            self._feedback_seq += 1
            self._feedback_time_ns = now_ns


def _cmd_vel_bytes(linear: float, angular: float) -> bytes:
    """cmd_vel register block; the firmware takes 0.01 m/s and 0.01 rad/s units
    and computes the wheel speeds itself (truncated towards zero, like turtlebot3_node)."""
    return struct.pack("<6i", int(linear * 100), 0, 0, 0, 0, int(angular * 100))


# ---------------------------------------------------------------------------
# DYNAMIXEL Protocol 2.0, just enough for one device: ping, read, write.
# ---------------------------------------------------------------------------
_HEADER = b"\xff\xff\xfd\x00"
_PING, _READ, _WRITE, _STATUS = 0x01, 0x02, 0x03, 0x55


def _crc_table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _crc_table()


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


def _stuff(body: bytes) -> bytes:
    """Insert 0xFD after every FF FF FD in the packet body, as the protocol requires."""
    out = bytearray()
    for byte in body:
        out.append(byte)
        if byte == 0xFD and out[-3:-1] == b"\xff\xff":
            out.append(0xFD)
    return bytes(out)


def _unstuff(body: bytes) -> bytes:
    return body.replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")


class DynamixelBoard:
    """A single DYNAMIXEL Protocol 2.0 device on a serial port, one request at a time."""

    def __init__(self, port: str, device_id: int, baudrate: int = 1_000_000, timeout_s: float = 0.02):
        # exclusive=True: refuse to share the port with another program.
        self._serial = serial.Serial(port, baudrate, timeout=timeout_s, exclusive=True)
        self._id = device_id
        self._timeout_s = timeout_s

    def close(self) -> None:
        self._serial.close()

    def ping(self) -> tuple[int, int]:
        reply = self._transact(_PING, b"")
        model, firmware = struct.unpack("<HB", reply)
        return model, firmware

    def read(self, address: int, size: int) -> bytes:
        reply = self._transact(_READ, struct.pack("<HH", address, size))
        if len(reply) != size:
            raise RuntimeError(f"read {address}: got {len(reply)} of {size} bytes")
        return reply

    def write(self, address: int, data: bytes) -> None:
        self._transact(_WRITE, struct.pack("<H", address) + data)

    def _transact(self, instruction: int, params: bytes) -> bytes:
        body = _stuff(bytes([instruction]) + params)
        packet = _HEADER + bytes([self._id]) + struct.pack("<H", len(body) + 2) + body
        packet += struct.pack("<H", crc16(packet))
        self._serial.reset_input_buffer()   # drop a late reply to an earlier request
        self._serial.write(packet)
        return self._read_status()

    def _read_status(self) -> bytes:
        deadline = time.monotonic() + self._timeout_s
        buffer = bytearray()
        while time.monotonic() < deadline:
            buffer += self._serial.read(max(1, self._serial.in_waiting))
            start = buffer.find(_HEADER)
            if start < 0 or len(buffer) < start + 7:
                continue
            length = struct.unpack_from("<H", buffer, start + 5)[0]
            end = start + 7 + length
            if len(buffer) < end:
                continue
            packet = bytes(buffer[start:end])
            if struct.unpack_from("<H", packet, len(packet) - 2)[0] != crc16(packet[:-2]):
                raise RuntimeError("status packet failed its CRC check")
            body = _unstuff(packet[7:-2])
            if packet[4] != self._id or body[0] != _STATUS:
                raise RuntimeError("unexpected status packet")
            if body[1] != 0:
                raise RuntimeError(f"device reported error 0x{body[1]:02x}")
            return body[2:]
        raise RuntimeError("no reply from the board (timeout)")
