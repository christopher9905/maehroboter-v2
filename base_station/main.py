"""Base station entrypoint: reads local RTCM3 corrections from the base
LC29H and serves them to rovers via NtripServer.

Configuration via environment variables (BASE_NTRIP_USER/PASSWORD are
required — no default):
  BASE_SERIAL_PORT    — serial port to the base LC29H (default: /dev/ttyUSB0)
  BASE_NTRIP_PORT     — TCP port for the caster (default: 2101)
  BASE_MOUNTPOINT     — NTRIP mountpoint name (default: MV2BASE)
  BASE_NTRIP_USER     — required, no default
  BASE_NTRIP_PASSWORD — required, no default

Unlike mower/main.py, this entrypoint does NOT degrade gracefully: without a
working GNSS module this device has no job, so a missing serial port or
missing credentials must fail loudly at startup rather than running a
silently useless process.
"""
import logging
import os
import time

from base_station.rtcm_source import RtcmSerialSource
from base_station.ntrip_server import NtripServer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_NTRIP_PORT = 2101
DEFAULT_MOUNTPOINT = "MV2BASE"


def _require_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"{name} must be set (no default — see base_station/main.py)")
    return value


def main() -> None:
    serial_port = os.environ.get("BASE_SERIAL_PORT", DEFAULT_SERIAL_PORT)
    ntrip_port = int(os.environ.get("BASE_NTRIP_PORT", DEFAULT_NTRIP_PORT))
    mountpoint = os.environ.get("BASE_MOUNTPOINT", DEFAULT_MOUNTPOINT)
    user = _require_env("BASE_NTRIP_USER")
    password = _require_env("BASE_NTRIP_PASSWORD")

    server = NtripServer(host="0.0.0.0", port=ntrip_port, mountpoint=mountpoint,
                         user=user, password=password)
    server.start()
    logger.info("NTRIP caster listening on 0.0.0.0:%d/%s", ntrip_port, mountpoint)

    # Raises immediately if the serial port is unavailable — intentional,
    # see module docstring: no soft fallback for a device with no other job.
    source = RtcmSerialSource(port=serial_port)
    source.on_data = server.broadcast
    source.start()
    logger.info("Relaying RTCM3 from %s", serial_port)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        server.stop()


if __name__ == "__main__":
    main()
