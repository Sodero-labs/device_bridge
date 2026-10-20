# Device bridge

A Sodero robot controller connects to the rest of the robot through network ports only.
Sensors that need USB, such as an Intel RealSense camera or a USB IMU, go on a small
computer next to them (a Raspberry Pi, any SBC or PC). The device bridge runs on that
computer: it reads the devices and serves their data to the controller over the LAN.

The bridge speaks the same protocol as the Sodero simulator. The controller's official
camera and IMU gateways, visual odometry, localizer and navigator therefore work with real
devices exactly as they work with a simulated robot; only the address they connect to
differs.

## What is official and what is yours

| Part | Runs on | Who owns it |
| --- | --- | --- |
| Device drivers (`device_bridge/devices/`) | Sensor computer | You: adapt or add one per device |
| The rest of the bridge (protocol, streams, safety watchdog) | Sensor computer | Sodero example; use as is |
| Wire protocol ([docs/protocol.md](docs/protocol.md)) | Between the two | Sodero; the simulator uses it too |
| Camera gateway, IMU gateway, actuator gateway | Robot controller | Sodero official; you configure them |
| Visual odometry, localizer, navigator, Strider | Robot controller | Sodero official; you configure them |

Supporting a new camera or IMU means writing one small driver class on the sensor
computer. Nothing on the robot controller is rebuilt or modified.

## How the parts connect

```
 sensor computer                              robot controller (Sodero Station)
 ------------------------------------         -----------------------------------------------
 RealSense --+                         53335   Camera gateway --> Visual odometry --> Localizer
 IMU --------+--> device bridge --------------> IMU gateway ------^                       |
 (wheels) <--+                         53334                                             v
                                <------------- (actuator gateway, optional)     Navigator --> Strider
                                       53333                                             --> motors (EtherCAT)
```

| Port | Direction | Messages |
| --- | --- | --- |
| 53335 | bridge to controller | RGB-D frames (`camera_pair`), camera intrinsics (`camera_info`) |
| 53334 | bridge to controller | IMU samples (`imu_state`), joint states (`sim_state`) |
| 53333 | controller to bridge | joint commands (`sim_command`), only when the bridge drives wheels |

The controller's motors normally use its own EtherCAT port. The `wheels` stream is for
robots whose motor board sits on the sensor computer, such as the TurtleBot3 OpenCR.

## Quick start

### On the sensor computer

Clone this repository on the sensor computer (or copy this directory there), then on that
computer:

```sh
git clone https://github.com/Sodero-labs/device_bridge.git
cd device_bridge
sudo deploy/install.sh config/turtlebot3_d435.yaml     # installs and starts the service
journalctl -u sodero-device-bridge -f                  # follow the log
```

`install.sh` copies the program to `/opt/sodero-device-bridge`, creates a Python environment
and starts the `sodero-device-bridge` service, which also starts at boot. Edit
`/opt/sodero-device-bridge/config.yaml` and restart the service to change a setting.

The log names the camera intrinsics to enter in the controller and then prints the message
rates every 10 seconds:

```
camera 424x240 fx=303.22 fy=303.26 cx=216.20 cy=123.18 frame_id=cam0_optical (use these intrinsics ...)
serving robot 'tb3' on 0.0.0.0 (camera 53335, state 53334, command -)
camera_pair 15.0/s, imu_state 50.0/s, sim_state 0.0/s, sim_command 0.0/s
```

Check what the controller will receive, from any computer on the LAN:

```sh
python -m device_bridge.listen SENSOR_COMPUTER_ADDRESS --robot tb3
```

To try everything without hardware, use `config/simulated.yaml` (a test pattern camera, a
turning IMU and two ideal wheels).

### On the robot controller

On the Station page; the user manual describes every step:
[Device Bridge](https://docs.sodero.io/latest/gateways/device_bridge/).

1. **Camera.** Add a Camera gateway named after the robot (`tb3` above). Set its **Camera name**
   to the bridge's `robot_name`, **Camera host** to the sensor computer's address and **Port** to
   53335, and enter the resolution and intrinsics from the bridge's log line.
2. **IMU.** Add an IMU gateway named after the robot and check that its **Robot in World** is the
   bridge's `robot_name`. The IMU gateway has no address field: it connects to the sensor
   address configured for the controller, which Sodero support sets to the sensor computer.
3. **Wheels (optional).** Only when the bridge serves a `wheels` stream: add a simulation
   actuator gateway with **Robot name** `tb3` and **Sim host** set to the sensor computer's
   address, and connect Strider to it.
4. **Navigation.** Connect the gateways to Visual odometry, Localizer and Navigator as for any
   other robot.
5. **Mounting.** The camera and IMU frames are the bridge's (`frame_id`); where they sit on the
   robot is the controller's extrinsics (in the map bundle). Measure the mounting; do not
   rotate data inside a driver.

## Add your own device

1. Write a driver class in `device_bridge/devices/`. Subclass `Device` and each capability
   the hardware has: `Camera`, `Imu` or `Wheels` ([devices/base.py](device_bridge/devices/base.py)
   documents every method). [devices/simulated.py](device_bridge/devices/simulated.py) is
   the smallest complete example; [realsense.py](device_bridge/devices/realsense.py) and
   [opencr.py](device_bridge/devices/opencr.py) are real ones.
2. Register it under a name in `DRIVERS` ([devices/\_\_init\_\_.py](device_bridge/devices/__init__.py)),
   or name it in the configuration as `my_package.my_module:MyCamera`.
3. Reference it in the configuration:

   ```yaml
   devices:
     front_camera:
       driver: my_camera          # your registered name
       serial: "123456"           # everything else is passed to your class as options
   streams:
     camera:
       device: front_camera
       frame_id: cam0_optical
   ```

A minimal IMU driver:

```python
from device_bridge.devices.base import Device, Imu, ImuSample

class MyImu(Device, Imu):
    def open(self):
        self.sensor = vendor_sdk.open(self.options["port"])

    def close(self):
        self.sensor.close()

    def read_imu(self):
        raw = self.sensor.latest()            # newest sample, never block for long
        if raw is None:
            return None
        return ImuSample(gyro=raw.gyro_rad_s, accel=raw.accel_m_s2,
                         orientation_xyzw=raw.quaternion_xyzw,
                         source_time_ns=raw.timestamp_ns)
```

Rules every driver follows:

- **Units:** radians, rad/s, m/s² with gravity included, depth in millimetres, times in ns.
- **Frames:** IMU data in the sensor's own axes; camera images in the optical frame
  (x right, y down, z forward), depth aligned to the RGB image and both rectified.
- **Time:** `source_time_ns` must increase. A camera's `capture_unix_ns` is the capture time on
  the wall clock (see [Time](#time-and-bandwidth)).
- **Timing:** `read_imu` and `read_joints` return the newest data or `None` at once;
  `read_frame` blocks for at most its timeout.
- **Safety:** `stop()` must stop and release the motors. The bridge calls it when the
  controller switches its drive off and when commands pause for `command_timeout_s`.

## Time and bandwidth

- **Clocks.** With `send_capture_time: true` the camera frames carry their capture time; the
  controller accepts them only when both computers' clocks agree within about 20 ms, so run
  NTP (or chrony) on both. With `false` the controller uses the arrival time, as with the
  simulator; this needs no clock agreement and costs the network delay in accuracy.
- **Bandwidth.** RGB-D frames are sent uncompressed: 424x240 at 15 fps is about 61 Mbit/s,
  640x480 at 15 fps about 185 Mbit/s. Use a wired link for the higher settings.

## Development

```sh
python -m unittest discover -s tests       # protocol layouts and an end-to-end bridge test
python -m device_bridge config/simulated.yaml
```

| Path | Contents |
| --- | --- |
| `device_bridge/protocol.py` | Byte layouts of every message |
| `device_bridge/bridge.py` | The three streams and the wheel watchdog |
| `device_bridge/config.py` | Configuration file format |
| `device_bridge/devices/` | Drivers: `base.py` (interfaces), `realsense.py`, `opencr.py`, `simulated.py` |
| `device_bridge/listen.py` | Receive the streams from another computer, optionally drive the wheels |
| `config/` | Example configurations |
| `deploy/` | Installer and systemd service |
