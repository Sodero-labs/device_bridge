"""The bridge end to end on this computer, with the simulated devices."""
import struct
import time
import unittest

import zmq

from device_bridge import protocol
from device_bridge.bridge import Bridge
from device_bridge.config import CameraStream, Config, ImuStream, Network, WheelStream
from device_bridge.devices import create_device
from device_bridge.listen import release_command, velocity_command

PORTS = Network(bind="127.0.0.1", command_port=54333, state_port=54334, camera_port=54335)


class BridgeEndToEnd(unittest.TestCase):
    def setUp(self):
        options = {"camera": {"driver": "simulated_camera", "width": 64, "height": 48, "fps": 30},
                   "imu": {"driver": "simulated_imu", "rate_hz": 100},
                   "wheels": {"driver": "simulated_wheels", "joints": 2}}
        self.devices = {name: create_device(name, opts) for name, opts in options.items()}
        for device in self.devices.values():
            device.open()
        config = Config(robot_name="tb3", network=PORTS, devices=options,
                        camera=CameraStream("camera"), imu=ImuStream("imu"),
                        wheels=WheelStream("wheels", command_timeout_s=0.3))
        self.bridge = Bridge(config, self.devices)
        self.bridge.start()
        self.context = zmq.Context()
        self.sub = self.context.socket(zmq.SUB)
        for port in (PORTS.camera_port, PORTS.state_port):
            self.sub.connect(f"tcp://127.0.0.1:{port}")
        self.sub.setsockopt(zmq.SUBSCRIBE, b"TM/tb3/")
        self.pub = self.context.socket(zmq.PUB)
        self.pub.connect(f"tcp://127.0.0.1:{PORTS.command_port}")
        time.sleep(0.3)

    def tearDown(self):
        self.sub.close(0)
        self.pub.close(0)
        self.context.term()
        self.bridge.stop()

    def receive(self, seconds):
        kinds, messages = set(), {}
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.sub.poll(50):
                head, _, payload = self.sub.recv().partition(b"||")
                kind = head.decode().rsplit("/", 1)[-1]
                kinds.add(kind)
                messages[kind] = payload
        return kinds, messages

    def test_streams_arrive(self):
        kinds, messages = self.receive(1.5)
        self.assertEqual(kinds, {"camera_pair", "camera_info", "imu_state", "sim_state"})
        width, height = struct.unpack_from("<4s7I", messages["camera_pair"], 0)[3:5]
        self.assertEqual((width, height), (64, 48))

    def test_commands_drive_and_the_watchdog_stops(self):
        wheels = self.devices["wheels"]
        for seq in range(10):
            self.pub.send_multipart(velocity_command("tb3", seq + 1, [1.0, -1.0]))
            time.sleep(0.02)
        self.assertEqual(wheels._velocities, [1.0, -1.0])
        time.sleep(0.6)   # longer than command_timeout_s
        self.assertEqual(wheels._velocities, [0.0, 0.0])

    def test_servo_off_releases(self):
        wheels = self.devices["wheels"]
        self.pub.send_multipart(velocity_command("tb3", 1, [2.0, 2.0]))
        time.sleep(0.1)
        self.pub.send_multipart(release_command("tb3", 2, 2))
        time.sleep(0.1)
        self.assertEqual(wheels._velocities, [0.0, 0.0])

    def test_other_robots_are_ignored(self):
        wheels = self.devices["wheels"]
        self.pub.send_multipart(velocity_command("other", 1, [3.0, 3.0]))
        time.sleep(0.1)
        self.assertEqual(wheels._velocities, [0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
