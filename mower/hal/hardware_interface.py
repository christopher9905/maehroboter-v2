import logging
import threading
import itertools
import copy
from typing import Callable, Optional
from mower.hal.serial_driver import SerialDriver
from mower.hal.protocol import (
    CmdType, encode_drive, encode_blade, encode_deck_lift, encode_estop, encode_ping,
    decode_sensors, decode_soc, decode_status,
)

logger = logging.getLogger(__name__)


class HardwareInterface:
    """High-level hardware API. Only this class is used by upper layers.

    Call start() after construction to begin the serial driver and the
    periodic PING sender (10 Hz) that feeds the Teensy watchdog.
    Call stop() on shutdown.
    """

    def __init__(self, driver: SerialDriver):
        self._driver = driver
        self._driver.on_frame = self._on_frame
        # Callbacks — upper layers assign these
        self.on_sensors: Optional[Callable[[dict], None]] = None
        self.on_soc: Optional[Callable[[dict], None]] = None
        self.on_status: Optional[Callable[[dict], None]] = None
        self.on_outputs: Optional[Callable[[dict], None]] = None
        self._output_lock = threading.Lock()
        self._outputs = {
            "speed_command": 0.0,
            "target_speed_kmh": 0.0,
            "speed_mode": "stopped",
            "steering_deg": 0.0,
            "blade_enabled": False,
            "front_deck_raised": False,
            "rear_deck_raised": False,
            "estop_active": False,
        }
        self._ping_seq = itertools.count()
        self._ping_timer: Optional[threading.Timer] = None
        self._running = False
        self.last_drive_direction = 1

    def start(self):
        self._running = True
        self._driver.start()
        self._schedule_ping()

    def stop(self):
        self._running = False
        if self._ping_timer:
            self._ping_timer.cancel()
        self._driver.stop()

    def _schedule_ping(self):
        if not self._running:
            return
        self.ping(next(self._ping_seq) & 0xFFFF)
        self._ping_timer = threading.Timer(0.1, self._schedule_ping)  # 10 Hz
        self._ping_timer.daemon = True
        self._ping_timer.start()

    # --- Commands ---

    def _publish_outputs(self, **values) -> None:
        with self._output_lock:
            self._outputs.update(values)
            snapshot = copy.deepcopy(self._outputs)
        if self.on_outputs:
            self.on_outputs(snapshot)

    def output_snapshot(self) -> dict:
        """Return the latest commands actually handed to the device driver."""
        with self._output_lock:
            return copy.deepcopy(self._outputs)

    def drive(self, speed: float, steering: float):
        """speed: -1.0..1.0, steering: -45..45 degrees"""
        speed = max(-1.0, min(1.0, float(speed)))
        steering = max(-45.0, min(45.0, float(steering)))
        # The wheel encoder is unsigned (single channel, always increments),
        # so odometry cannot sense direction.  Remember the commanded drive
        # direction; the sensor wiring uses it to sign the odometry speed
        # before feeding the localizer — otherwise the UKF is told "forward"
        # while the machine reverses and the fused pose drifts during every
        # reverse maneuver.  Zero speed keeps the previous direction.
        if speed > 1e-6:
            self.last_drive_direction = 1
        elif speed < -1e-6:
            self.last_drive_direction = -1
        self._driver.send(encode_drive(speed, steering))
        self._publish_outputs(speed_command=speed, steering_deg=steering)

    def set_motion_context(self, target_speed_kmh: float, speed_mode: str) -> None:
        """Publish the active motion target without changing device commands."""
        self._publish_outputs(
            target_speed_kmh=max(0.0, float(target_speed_kmh)),
            speed_mode=str(speed_mode),
        )

    def set_blade(self, state: bool):
        state = bool(state)
        self._driver.send(encode_blade(state))
        self._publish_outputs(blade_enabled=state)

    def set_deck_lift(self, front_raised: bool, rear_raised: bool):
        """Raise/lower front and rear mowing decks independently."""
        front_raised, rear_raised = bool(front_raised), bool(rear_raised)
        self._driver.send(encode_deck_lift(front_raised, rear_raised))
        self._publish_outputs(
            front_deck_raised=front_raised,
            rear_deck_raised=rear_raised,
        )

    def estop(self):
        # Priority send: an ESTOP must win serial ordering over any queued
        # drive command (e.g. an in-flight docking manoeuvre).
        self._driver.send(encode_estop(), priority=True)
        self._publish_outputs(
            speed_command=0.0,
            target_speed_kmh=0.0,
            speed_mode="emergency_stop",
            steering_deg=0.0,
            blade_enabled=False,
            estop_active=True,
        )

    def reset_estop(self):
        """Release a device-side ESTOP latch when the driver implements one.

        The current Teensy command is an immediate stop and needs no release;
        stricter devices such as the simulator can expose an explicit reset.
        """
        reset = getattr(self._driver, "reset_estop", None)
        if reset:
            reset()
        self._publish_outputs(estop_active=False)

    def ping(self, seq: int):
        self._driver.send(encode_ping(seq))

    # --- Telemetry dispatcher ---

    def _on_frame(self, cmd_type: CmdType, payload: bytes):
        try:
            if cmd_type == CmdType.SENSORS:
                data = decode_sensors(payload)
                if data['lift']:
                    logger.warning("Lift detected — sending ESTOP")
                    self.estop()
                    self.set_blade(False)
                if self.on_sensors:
                    self.on_sensors(data)
            elif cmd_type == CmdType.SOC:
                if self.on_soc:
                    self.on_soc(decode_soc(payload))
            elif cmd_type == CmdType.STATUS:
                if self.on_status:
                    self.on_status(decode_status(payload))
        except Exception as e:
            logger.error("Frame dispatch error: %s", e)
