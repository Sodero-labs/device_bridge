"""Byte layouts and helpers. Run: python -m unittest discover -s tests"""
import struct
import unittest

import numpy as np

from device_bridge import protocol
from device_bridge.devices import opencr
from device_bridge.listen import release_command, velocity_command


class WireLayout(unittest.TestCase):
    def test_sizes_match_the_controller(self):
        # SimCommand 2648 B / SimState 1560 B (sim_types.h), ImuState 152 B, FrameHdr 64 B.
        command = velocity_command("tb3", 1, [0.5, -0.5])[3]
        self.assertEqual(len(command), 2648)
        self.assertEqual(len(protocol.pack_sim_state(1, 1, [0.0], [0.0], [0.0])), 1560)
        self.assertEqual(len(protocol.pack_imu_state(1, 1, (0, 0, 0, 1), (0, 0, 0), (0, 0, 9.81))), 152)

    def test_sim_command_round_trip(self):
        decoded = protocol.unpack_sim_command(velocity_command("tb3", 7, [1.25, -2.5])[3])
        self.assertEqual(decoded.seq, 7)
        self.assertEqual(decoded.modes, [protocol.MODE_VELOCITY] * 2)
        self.assertEqual(decoded.velocities, [1.25, -2.5])
        released = protocol.unpack_sim_command(release_command("tb3", 8, 2)[3])
        self.assertEqual(released.modes, [protocol.MODE_IMPEDANCE] * 2)
        self.assertEqual(released.kp, [0.0, 0.0])

    def test_sim_state_field_offsets(self):
        payload = protocol.pack_sim_state(123, 4, [1.0, 2.0], [3.0, 4.0], [5.0, 6.0])
        time_ns, seq, count = struct.unpack_from("<qQi", payload, 0)
        self.assertEqual((time_ns, seq, count), (123, 4, 2))
        values = struct.unpack_from("<192d", payload, 24)
        self.assertEqual(values[0:2], (1.0, 2.0))      # q
        self.assertEqual(values[64:66], (3.0, 4.0))    # dq
        self.assertEqual(values[128:130], (5.0, 6.0))  # tau

    def test_imu_state_field_offsets(self):
        payload = protocol.pack_imu_state(9, 2, (0.1, 0.2, 0.3, 0.9), (1, 2, 3), (4, 5, 6))
        fields = struct.unpack("<qQ16d2B6x", payload)
        self.assertEqual(fields[:2], (9, 2))
        self.assertEqual(fields[2:6], (0.1, 0.2, 0.3, 0.9))   # orientation x y z w
        self.assertEqual(fields[6:9], (1, 2, 3))              # gyro
        self.assertEqual(fields[9:12], (4, 5, 6))             # accel
        self.assertEqual(fields[12:], (0.0,) * 6 + (0, 0))    # simulator-only fields

    def test_camera_pair_layout(self):
        rgb = np.zeros((2, 3, 3), np.uint8)
        depth = np.full((2, 3), 1500, np.uint16)
        blob = protocol.pack_camera_pair(5, rgb, depth, 1_700_000_000_000_000_000, "cam0_optical")
        header = struct.unpack_from("<4s7IqfI16s", blob, 0)
        magic, version, seq, width, height, step, size, encoding, stamp, unit, flags, frame = header
        self.assertEqual((magic, version, seq, width, height), (b"RCIM", 1, 5, 3, 2))
        self.assertEqual((step, size, encoding, unit), (6, 12, protocol.ENCODING_DEPTH16, np.float32(0.001)))
        self.assertTrue(flags & protocol.FLAG_DEPTH_FIRST and flags & protocol.FLAG_CAPTURE_UNIX_TIME)
        self.assertEqual(frame.rstrip(b"\0"), b"cam0_optical")
        rgb_header = struct.unpack_from("<4s7IqfI16s", blob, 64 + 12)
        self.assertEqual((rgb_header[5], rgb_header[6], rgb_header[7]), (9, 18, protocol.ENCODING_RGB8))
        self.assertEqual(len(blob), 64 + 12 + 64 + 18)

    def test_camera_pair_without_capture_time_matches_the_simulator(self):
        rgb = np.zeros((2, 3, 3), np.uint8)
        depth = np.zeros((2, 3), np.uint16)
        blob = protocol.pack_camera_pair(1, rgb, depth, 0, "cam0_optical")
        for offset in (0, 64 + 12):
            stamp, flags = struct.unpack_from("<qfI", blob, offset + 32)[0::2]
            self.assertEqual((stamp, flags), (0, protocol.FLAG_DEPTH_FIRST))


class OpenCR(unittest.TestCase):
    def test_crc_matches_the_robotis_example(self):
        # Protocol 2.0 manual: ping ID 1 = FF FF FD 00 01 03 00 01 | CRC 19 4E
        packet = bytes([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01])
        self.assertEqual(opencr.crc16(packet), 0x4E19)

    def test_byte_stuffing_round_trip(self):
        body = bytes([0x03, 0xFF, 0xFF, 0xFD, 0x10])
        stuffed = opencr._stuff(body)
        self.assertEqual(stuffed, bytes([0x03, 0xFF, 0xFF, 0xFD, 0xFD, 0x10]))
        self.assertEqual(opencr._unstuff(stuffed), body)

    def test_wheel_speeds_to_twist(self):
        linear, angular = opencr.wheels_to_twist(2.0, 2.0, 0.15, 1.0)
        self.assertAlmostEqual(linear, 0.066)
        self.assertAlmostEqual(angular, 0.0)
        linear, angular = opencr.wheels_to_twist(-10.0, 10.0, 0.15, 1.0)   # limited, still a pure turn
        self.assertAlmostEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 1.0)

    def test_cmd_vel_truncates_like_turtlebot3_node(self):
        values = struct.unpack("<6i", opencr._cmd_vel_bytes(0.129, -0.555))
        self.assertEqual(values, (12, 0, 0, 0, 0, -55))


if __name__ == "__main__":
    unittest.main()
