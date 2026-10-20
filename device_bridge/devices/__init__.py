"""Device drivers, looked up by the `driver` name in the configuration file.

To add your own hardware, write a class as described in base.py and either add
it to DRIVERS below or name it directly in the configuration as
"package.module:ClassName". Drivers are imported only when used, so a vendor
SDK is needed only for the drivers a configuration actually selects.
"""
from __future__ import annotations

import importlib

from .base import Camera, CameraInfo, Device, Imu, ImuSample, JointState, RgbdFrame, Wheels

DRIVERS = {
    "realsense": "device_bridge.devices.realsense:RealSenseCamera",
    "turtlebot3_opencr": "device_bridge.devices.opencr:TurtleBot3OpenCR",
    "simulated_camera": "device_bridge.devices.simulated:SimulatedCamera",
    "simulated_imu": "device_bridge.devices.simulated:SimulatedImu",
    "simulated_wheels": "device_bridge.devices.simulated:SimulatedWheels",
}


def create_device(name: str, options: dict) -> Device:
    """Instantiate the driver selected by options["driver"] (not opened yet)."""
    driver = options.get("driver")
    if not driver:
        raise ValueError(f"device '{name}' has no driver")
    target = DRIVERS.get(driver, driver)
    if ":" not in target:
        raise ValueError(f"device '{name}': unknown driver '{driver}' "
                         f"(known: {', '.join(sorted(DRIVERS))}, or module:Class)")
    module_name, class_name = target.split(":", 1)
    cls = getattr(importlib.import_module(module_name), class_name)
    settings = {key: value for key, value in options.items() if key != "driver"}
    return cls(name, settings)


__all__ = ["Camera", "CameraInfo", "Device", "Imu", "ImuSample", "JointState", "RgbdFrame",
           "Wheels", "DRIVERS", "create_device"]
