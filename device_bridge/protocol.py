"""Wire format of the Sodero bridge protocol.

The device bridge speaks exactly the protocol the Isaac Sim bridge speaks, so
the controller's official gateways handle a real robot and a simulated one the
same way: only the address they connect to differs.

Byte layouts (little-endian) mirror these definitions; keep them in sync:
    camera_gateway/include/rc_camera/frame_protocol.hpp   FrameHdr, 64 bytes
    actuator_gateway/include/hardware/sim_types.h         SimCommand, SimState
    isaac_bridge/scripts/protocol.py                      ImuState, envelopes

Messages (ZeroMQ PUB/SUB; the bridge binds, the controller connects):

    port  direction             message
    53333 controller -> bridge  multipart [b"TM/", name, b"sim_command", SimCommand]
    53334 bridge -> controller  b"TM/<name>/sim_state||"   + SimState
                                b"TM/<name>/imu_state||"   + ImuState
    53335 bridge -> controller  b"TM/<name>/camera_pair||" + depth FrameHdr + depth
                                                           + rgb FrameHdr + rgb
                                b"TM/<name>/camera_info||" + JSON (about 1 Hz)
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Envelopes and ports
# ---------------------------------------------------------------------------
COMMAND_PORT = 53333
STATE_PORT = 53334
CAMERA_PORT = 53335

SIM_COMMAND = b"sim_command"
SIM_STATE = "sim_state"
IMU_STATE = "imu_state"
CAMERA_PAIR = "camera_pair"
CAMERA_INFO = "camera_info"


def topic(robot_name: str, kind: str) -> bytes:
    """Prefix of a single-frame message: b"TM/<robot_name>/<kind>||"."""
    return f"TM/{robot_name}/{kind}||".encode()


# ---------------------------------------------------------------------------
# Joints: SimCommand (controller -> bridge) and SimState (bridge -> controller)
# ---------------------------------------------------------------------------
MAX_JOINTS = 64

MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_TORQUE = 2
MODE_IMPEDANCE = 3   # kp = 0 with damping only: the controller's "servo off" (limp)
MODE_IDLE = 0xFE     # slot owned by another gateway: ignore it
MODE_INIT = 0xFF     # simulator-only teleport request: ignore it on hardware

# i64 timestamp_ns | u64 seq | i32 num_joints | u8 mode[64] | pad[4]
# | f64 q_d[64] | f64 dq_d[64] | f64 tau_d[64] | f64 kp[64] | f64 kd[64]
_SIM_COMMAND = struct.Struct(f"<qQi{MAX_JOINTS}B4x{5 * MAX_JOINTS}d")
# i64 source_time_ns | u64 seq | i32 num_joints | pad[4]
# | f64 q[64] | f64 dq[64] | f64 tau[64]
_SIM_STATE = struct.Struct(f"<qQi4x{3 * MAX_JOINTS}d")
assert _SIM_COMMAND.size == 2648 and _SIM_STATE.size == 1560


@dataclass
class JointCommand:
    """One decoded SimCommand, cut to its num_joints entries."""
    seq: int
    modes: list[int]
    positions: list[float]    # q_d   [rad]
    velocities: list[float]   # dq_d  [rad/s]
    efforts: list[float]      # tau_d [Nm]
    kp: list[float]
    kd: list[float]


def unpack_sim_command(payload: bytes) -> JointCommand:
    if len(payload) != _SIM_COMMAND.size:
        raise ValueError(f"sim_command is {len(payload)} bytes, expected {_SIM_COMMAND.size}")
    fields = _SIM_COMMAND.unpack(payload)
    count = fields[2]
    if not 0 <= count <= MAX_JOINTS:
        raise ValueError(f"sim_command num_joints {count} is out of range")
    modes = fields[3:3 + MAX_JOINTS]
    arrays = fields[3 + MAX_JOINTS:]

    def block(index: int) -> list[float]:
        start = index * MAX_JOINTS
        return list(arrays[start:start + count])

    return JointCommand(seq=fields[1], modes=list(modes[:count]),
                        positions=block(0), velocities=block(1), efforts=block(2),
                        kp=block(3), kd=block(4))


def pack_sim_state(source_time_ns: int, seq: int, positions, velocities, efforts) -> bytes:
    """Joint state. source_time_ns is the bridge's monotonic clock; it only has
    to increase. The controller stamps arrival time itself."""
    count = len(positions)
    if count > MAX_JOINTS or len(velocities) != count or len(efforts) != count:
        raise ValueError("positions, velocities and efforts need the same length (at most 64)")
    padding = [0.0] * (MAX_JOINTS - count)
    return _SIM_STATE.pack(int(source_time_ns), int(seq), count,
                           *map(float, positions), *padding,
                           *map(float, velocities), *padding,
                           *map(float, efforts), *padding)


# ---------------------------------------------------------------------------
# IMU: ImuState (bridge -> controller)
# ---------------------------------------------------------------------------
# i64 source_time_ns | u64 seq | f64 orientation_xyzw[4] | f64 gyro[3] | f64 accel[3]
# | f64 lin_vel_w[3] | f64 pos_w[3] | u8 contacts[2] | pad[6]
# The last three fields are simulator ground truth; a real bridge sends zeros,
# and the IMU gateway never reads them.
_IMU_STATE = struct.Struct("<qQ16d2B6x")
assert _IMU_STATE.size == 152


def pack_imu_state(source_time_ns: int, seq: int, orientation_xyzw, gyro, accel) -> bytes:
    """orientation_xyzw: sensor attitude in the world, unit quaternion.
    gyro [rad/s] and accel [m/s^2, gravity included] in the sensor's own axes."""
    values = [*orientation_xyzw, *gyro, *accel]
    if len(values) != 10:
        raise ValueError("orientation needs 4 values, gyro and accel 3 each")
    return _IMU_STATE.pack(int(source_time_ns), int(seq), *map(float, values),
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0)


# ---------------------------------------------------------------------------
# Camera: one RGB-D pair per message, plus camera_info
# ---------------------------------------------------------------------------
# char magic[4] "RCIM" | u32 version | u32 pair_seq | u32 width | u32 height
# | u32 step_bytes | u32 payload_bytes | u32 encoding | i64 observed_at_ns
# | f32 depth_unit_m | u32 flags | char frame_id[16]
_FRAME_HDR = struct.Struct("<4s7IqfI16s")
assert _FRAME_HDR.size == 64

FRAME_MAGIC = b"RCIM"
FRAME_VERSION = 1
ENCODING_RGB8 = 1
ENCODING_DEPTH16 = 2          # uint16 millimetres
FLAG_DEPTH_FIRST = 1 << 1     # the pair carries the depth frame first
FLAG_CAPTURE_UNIX_TIME = 1 << 2   # observed_at_ns is a Unix capture time
FRAME_ID_MAX = 15             # bytes; char[16] with a terminating NUL


def _frame(seq: int, encoding: int, pixels: np.ndarray, frame_id: bytes,
           capture_unix_ns: int, depth_unit_m: float) -> bytes:
    height, width = pixels.shape[:2]
    data = np.ascontiguousarray(pixels).tobytes()
    flags = FLAG_DEPTH_FIRST | (FLAG_CAPTURE_UNIX_TIME if capture_unix_ns else 0)
    header = _FRAME_HDR.pack(FRAME_MAGIC, FRAME_VERSION, seq & 0xFFFFFFFF, width, height,
                             len(data) // height, len(data), encoding, int(capture_unix_ns),
                             depth_unit_m, flags, frame_id)
    return header + data


def pack_camera_pair(seq: int, rgb: np.ndarray, depth_mm: np.ndarray,
                     capture_unix_ns: int, frame_id: str) -> bytes:
    """rgb: HxWx3 uint8 in RGB order. depth_mm: HxW uint16 millimetres, already
    aligned to the RGB image (0 = no depth). Both from the same capture.

    capture_unix_ns > 0 sends the capture time; the controller then needs a
    clock synchronized with this computer (NTP) to within about 20 ms. With 0
    the frame carries no time, like the simulator's, and the controller uses the
    arrival time instead."""
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("rgb must be an HxWx3 uint8 array")
    if depth_mm.dtype != np.uint16 or depth_mm.shape != rgb.shape[:2]:
        raise ValueError("depth must be a uint16 array with the RGB image's height and width")
    name = frame_id.encode()
    if not 0 < len(name) <= FRAME_ID_MAX:
        raise ValueError(f"frame_id must be 1..{FRAME_ID_MAX} bytes")
    return (_frame(seq, ENCODING_DEPTH16, depth_mm, name, capture_unix_ns, 0.001)
            + _frame(seq, ENCODING_RGB8, rgb, name, capture_unix_ns, 0.0))


def camera_info_json(info, fps: float, pair_seq: int) -> bytes:
    """Intrinsics of the published images (rectified pinhole), sent about once
    a second. `info` is a devices.CameraInfo."""
    return json.dumps({
        "frame_id": info.frame_id,
        "intrinsics": {"width": info.width, "height": info.height,
                       "fx": round(info.fx, 4), "fy": round(info.fy, 4),
                       "cx": round(info.cx, 4), "cy": round(info.cy, 4),
                       "distortion_model": "plumb_bob",
                       "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0]},
        "pair_seq": pair_seq,
        "fps": fps,
        "backend": "device_bridge",
    }, separators=(",", ":")).encode()
