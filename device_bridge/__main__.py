"""Run the device bridge:

    python -m device_bridge config/turtlebot3_d435.yaml
"""
from __future__ import annotations

import argparse
import logging
import signal
import threading
import time

from .bridge import Bridge
from .config import load_config
from .devices import Device, create_device

log = logging.getLogger("device_bridge")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m device_bridge",
                                     description="Serve local devices to a Sodero controller over the LAN.")
    parser.add_argument("config", help="configuration file (YAML)")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    config = load_config(args.config)
    devices: dict[str, Device] = {}
    bridge = None
    try:
        for name, options in config.devices.items():
            device = create_device(name, options)
            device.open()
            devices[name] = device
            log.info("device '%s' (%s) is open", name, options["driver"])
        bridge = Bridge(config, devices)
        bridge.start()
        report_rates(bridge, stop)
    except Exception as error:   # report one clear line; systemd restarts the service
        log.error("%s", error, exc_info=args.log_level == "DEBUG")
        return 1
    finally:
        if bridge is not None:
            bridge.stop()
        for device in reversed(list(devices.values())):
            device.close()
    return 0


def report_rates(bridge: Bridge, stop: threading.Event, every_s: float = 10.0) -> None:
    """Log the message rates every few seconds until asked to stop."""
    previous, since = dict(bridge.counters), time.monotonic()
    while not stop.wait(every_s):
        now, current = time.monotonic(), dict(bridge.counters)
        rates = ", ".join(f"{name} {(current[name] - previous[name]) / (now - since):.1f}/s"
                          for name in current)
        log.info("%s", rates)
        previous, since = current, now


if __name__ == "__main__":
    raise SystemExit(main())
