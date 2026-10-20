# Bridge protocol

The device bridge and the Sodero simulator (Isaac Sim bridge) publish the same messages. A
robot controller's official gateways subscribe to them; they cannot tell the two apart.

## Transport

ZeroMQ over TCP. The bridge **binds** three sockets; the controller **connects** to them.

| Port | Bridge socket | Controller socket | Messages |
| --- | --- | --- | --- |
| 53335 | PUB | SUB | `camera_pair`, `camera_info` |
| 53334 | PUB | SUB | `imu_state`, `sim_state` |
| 53333 | SUB | PUB | `sim_command` |

Published messages are single-frame: a topic, the two bytes `||`, then the payload.

```
TM/<robot_name>/<kind>||<payload>
```

`<robot_name>` names the robot (the controller's gateways are configured with the same
name); `<kind>` is one of the message kinds below. Commands are multipart:

```
[b"TM/", <robot_name>, b"sim_command", <SimCommand>]
```

The controller's gateways work on the newest message, so publish at the sensor's rate and
do not queue old data (the bridge keeps at most two messages per subscriber). All numbers
are little-endian. Payload layouts are fixed by C++ `static_assert`s on the controller; do
not change sizes or field order.

## camera_pair: one RGB-D capture

`[depth FrameHdr][depth pixels][RGB FrameHdr][RGB pixels]`

FrameHdr, 64 bytes (Python `struct` format `<4s7IqfI16s`):

| Offset | Type | Field | Value |
| --- | --- | --- | --- |
| 0 | char[4] | magic | `RCIM` |
| 4 | u32 | version | 1 |
| 8 | u32 | pair_seq | Capture counter; the same in both headers; increases per capture |
| 12 | u32 | width | Pixels |
| 16 | u32 | height | Pixels |
| 20 | u32 | step_bytes | Bytes per row; rows are tightly packed |
| 24 | u32 | payload_bytes | Pixel bytes after this header |
| 28 | u32 | encoding | 1 = RGB8, 2 = DEPTH16 (uint16 mm), 3 = DEPTH32F (float32 m) |
| 32 | i64 | observed_at_ns | Capture time, Unix ns, when flag 4 is set; otherwise 0 |
| 40 | f32 | depth_unit_m | 0.001 for DEPTH16, 0 for RGB8 |
| 44 | u32 | flags | 2 = depth first (always set); 4 = `observed_at_ns` is a Unix capture time |
| 48 | char[16] | frame_id | Optical frame name, NUL-terminated (at most 15 bytes) |

Requirements:

- Depth is aligned to the RGB image; both have the same size and are rectified to the
  intrinsics in the controller's camera settings (pinhole, no distortion).
- Depth 0 means no measurement.
- With flag 4, the controller rejects frames whose capture time is more than 20 ms in its
  future or more than 2 s old: keep both clocks synchronized (NTP). Without flag 4 it uses
  the arrival time, as for the simulator.

## camera_info: intrinsics

JSON, sent about once a second:

```json
{"frame_id":"cam0_optical",
 "intrinsics":{"width":424,"height":240,"fx":303.217,"fy":303.257,"cx":216.198,"cy":123.175,
               "distortion_model":"plumb_bob","distortion_coefficients":[0.0,0.0,0.0,0.0,0.0]},
 "pair_seq":123,"fps":15,"backend":"device_bridge"}
```

The controller's camera settings hold the intrinsics it uses; this message lets a person or
a tool read them from the bridge.

## imu_state: one IMU sample

152 bytes (`<qQ16d2B6x`):

| Offset | Type | Field | Unit |
| --- | --- | --- | --- |
| 0 | i64 | source_time_ns | Bridge or sensor clock; must increase |
| 8 | u64 | seq | Increases per sample |
| 16 | f64[4] | orientation x, y, z, w | Sensor attitude in the world, unit quaternion |
| 48 | f64[3] | angular velocity | rad/s, sensor axes |
| 72 | f64[3] | linear acceleration | m/s², sensor axes, gravity included (+9.81 up at rest) |
| 96 | f64[3] | linear velocity (world) | Simulator only; 0 from a real sensor |
| 120 | f64[3] | position (world) | Simulator only; 0 from a real sensor |
| 144 | u8[2] | contacts | Simulator only; 0 from a real sensor |
| 146 | pad[6] | | |

The controller stamps IMU samples with their arrival time; `source_time_ns` only has to
increase, so a sample that repeats or goes back is dropped.

## sim_state: joint states (wheels)

1560 bytes (`<qQi4x192d`):

| Offset | Type | Field |
| --- | --- | --- |
| 0 | i64 | source_time_ns, increasing |
| 8 | u64 | seq |
| 16 | i32 | num_joints (at most 64) |
| 20 | pad[4] | |
| 24 | f64[64] | position, rad, continuous |
| 536 | f64[64] | velocity, rad/s |
| 1048 | f64[64] | effort, Nm (0 when unknown) |

The controller's actuator gateway runs one control cycle per received `sim_state`, so send
it at a steady rate (50 to 100 Hz).

## sim_command: joint commands (wheels)

2648 bytes (`<qQi64B4x320d`):

| Offset | Type | Field |
| --- | --- | --- |
| 0 | i64 | timestamp_ns |
| 8 | u64 | seq |
| 16 | i32 | num_joints |
| 20 | u8[64] | mode per joint |
| 84 | pad[4] | |
| 88 | f64[64] | position target q_d, rad |
| 600 | f64[64] | velocity target dq_d, rad/s |
| 1112 | f64[64] | torque target tau_d, Nm |
| 1624 | f64[64] | stiffness kp |
| 2136 | f64[64] | damping kd |

Modes: 0 position, 1 velocity, 2 torque, 3 impedance, 0xFE not this gateway's joint,
0xFF simulator teleport. The controller sends **velocity** while its drive is on and
**impedance with kp = 0** (limp) when it switches the drive off. A wheel bridge drives on
velocity, releases the motors on anything else, and stops on its own when commands pause.
