"""Check a running bridge from another computer, the way the controller sees it.

    python -m device_bridge.listen 192.168.60.18 --robot tb3
    python -m device_bridge.listen 192.168.60.18 --robot tb3 --wheels 1.0 1.0 --seconds 2

Prints message rates and the latest values. With --wheels it also drives the
wheels at the given speeds [rad/s] for --seconds and then releases them, as the
controller would: lift the robot or give it room first.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import time

import numpy as np
import zmq

from . import protocol

_FRAME_HDR = struct.Struct("<4s7IqfI16s")
_IMU = struct.Struct("<qQ16d2B6x")
_STATE_HEAD = struct.Struct("<qQi4x")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m device_bridge.listen")
    parser.add_argument("host", help="address of the computer running the bridge")
    parser.add_argument("--robot", default="tb3", help="robot_name of the bridge")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--wheels", type=float, nargs="+", metavar="RAD_S",
                        help="drive the wheels at these speeds while listening")
    args = parser.parse_args(argv)

    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 4)
    for port in (protocol.CAMERA_PORT, protocol.STATE_PORT):
        sub.connect(f"tcp://{args.host}:{port}")
    for kind in (protocol.CAMERA_PAIR, protocol.CAMERA_INFO, protocol.IMU_STATE, protocol.SIM_STATE):
        sub.setsockopt(zmq.SUBSCRIBE, protocol.topic(args.robot, kind))
    pub = None
    if args.wheels:
        pub = context.socket(zmq.PUB)
        pub.connect(f"tcp://{args.host}:{protocol.COMMAND_PORT}")
        time.sleep(0.3)   # let the connection come up before the first command

    counts: dict[str, int] = {}
    latest: dict[str, str] = {}
    start = time.monotonic()
    command_seq, last_command = 0, 0.0
    while time.monotonic() - start < args.seconds:
        if pub is not None and time.monotonic() - last_command >= 0.02:   # 50 Hz, like a controller
            command_seq += 1
            pub.send_multipart(velocity_command(args.robot, command_seq, args.wheels))
            last_command = time.monotonic()
        if not sub.poll(timeout=20):
            continue
        message = sub.recv()
        head, _, payload = message.partition(b"||")
        kind = head.decode(errors="replace").rsplit("/", 1)[-1]
        counts[kind] = counts.get(kind, 0) + 1
        latest[kind] = describe(kind, payload)

    if pub is not None:   # release the wheels the way the controller does on servo off
        for _ in range(5):
            command_seq += 1
            pub.send_multipart(release_command(args.robot, command_seq, len(args.wheels)))
            time.sleep(0.02)

    elapsed = time.monotonic() - start
    if not counts:
        print(f"no messages from {args.host} for robot '{args.robot}' in {elapsed:.1f} s")
        return 1
    for kind in sorted(counts):
        print(f"{kind:12s} {counts[kind] / elapsed:6.1f}/s  {latest[kind]}")
    return 0


def describe(kind: str, payload: bytes) -> str:
    if kind == protocol.CAMERA_PAIR:
        depth = _FRAME_HDR.unpack_from(payload, 0)
        width, height, observed_ns = depth[3], depth[4], depth[8]
        pixels = np.frombuffer(payload, np.uint16, width * height, _FRAME_HDR.size)
        age_ms = (time.time_ns() - observed_ns) / 1e6
        return (f"{width}x{height}, depth valid {np.count_nonzero(pixels) / pixels.size:.0%}, "
                f"median {np.median(pixels[pixels > 0]) if pixels.any() else 0:.0f} mm, "
                f"capture age {age_ms:.0f} ms (meaningful with synchronized clocks)")
    if kind == protocol.CAMERA_INFO:
        intrinsics = json.loads(payload)["intrinsics"]
        return " ".join(f"{key}={intrinsics[key]}" for key in ("width", "height", "fx", "fy", "cx", "cy"))
    if kind == protocol.IMU_STATE:
        values = _IMU.unpack(payload)[2:12]
        gyro, accel = values[4:7], values[7:10]
        return (f"gyro [{', '.join(f'{v:+.3f}' for v in gyro)}] rad/s, "
                f"|accel| {math.sqrt(sum(a * a for a in accel)):.2f} m/s^2")
    if kind == protocol.SIM_STATE:
        _, _, count = _STATE_HEAD.unpack_from(payload, 0)
        values = struct.unpack_from(f"<{3 * protocol.MAX_JOINTS}d", payload, _STATE_HEAD.size)
        positions = values[:count]
        velocities = values[protocol.MAX_JOINTS:protocol.MAX_JOINTS + count]
        return (f"q [{', '.join(f'{v:+.3f}' for v in positions)}] rad, "
                f"dq [{', '.join(f'{v:+.3f}' for v in velocities)}] rad/s")
    return f"{len(payload)} bytes"


def velocity_command(robot: str, seq: int, velocities) -> list[bytes]:
    count = len(velocities)
    modes = [protocol.MODE_VELOCITY] * count
    return _command(robot, seq, modes, list(velocities), [0.0] * count)


def release_command(robot: str, seq: int, count: int) -> list[bytes]:
    modes = [protocol.MODE_IMPEDANCE] * count   # kp = 0: the controller's servo off
    return _command(robot, seq, modes, [0.0] * count, [0.0] * count)


def _command(robot: str, seq: int, modes, velocities, kp) -> list[bytes]:
    n = protocol.MAX_JOINTS
    count = len(modes)

    def padded(values):
        return list(values) + [0.0] * (n - count)

    payload = struct.pack(f"<qQi{n}B4x{5 * n}d", time.monotonic_ns(), seq, count,
                          *(list(modes) + [0] * (n - count)),
                          *padded([0.0] * count), *padded(velocities), *padded([0.0] * count),
                          *padded(kp), *padded([0.0] * count))
    return [b"TM/", robot.encode(), protocol.SIM_COMMAND, payload]


if __name__ == "__main__":
    raise SystemExit(main())
