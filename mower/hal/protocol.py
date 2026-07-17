import struct
import crcmod
from enum import IntEnum


class CmdType(IntEnum):
    DRIVE = 0x01
    BLADE = 0x02
    ESTOP = 0x03
    PING  = 0x04
    DECK_LIFT = 0x05
    # Telemetry (Teensy → RPi)
    SENSORS = 0x10
    SOC     = 0x11
    STATUS  = 0x12


START_BYTE = 0xAA

# Named constants (Issue 3)
MAX_STEERING_DEG: float = 45.0
PING_SEQ_BYTES: int = 2

# Telemetry payload size constants (Issue 2)
_SENSORS_SIZE = struct.calcsize('<HBI')  # 7
_SOC_SIZE = struct.calcsize('<BH')       # 3
_STATUS_SIZE = struct.calcsize('<BBBB')  # 4 (watchdog_ok, blade_running, error_flags, charging)

# CRC-8/MAXIM: poly=0x31, initCrc=0x00, rev=True, xorOut=0x00
_crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')


class FrameError(Exception):
    pass


def encode_frame(cmd_type: CmdType, payload: bytes) -> bytes:
    header = bytes([START_BYTE, int(cmd_type), len(payload)])
    body = header + payload
    crc = _crc8(body)  # Issue 5: call _crc8 directly, removed _compute_crc wrapper
    return body + bytes([crc])


def decode_frame(data: bytes) -> tuple[CmdType, bytes]:
    if len(data) < 4:
        raise FrameError("Frame too short")
    if data[0] != START_BYTE:
        raise FrameError(f"Invalid start byte: 0x{data[0]:02X}")
    payload_len = data[2]
    expected_len = 3 + payload_len + 1
    if len(data) < expected_len:
        raise FrameError("Frame truncated")
    body = data[:3 + payload_len]
    received_crc = data[3 + payload_len]
    expected_crc = _crc8(body)  # Issue 5: call _crc8 directly
    if received_crc != expected_crc:
        raise FrameError(f"CRC mismatch: got 0x{received_crc:02X}, expected 0x{expected_crc:02X}")
    # Issue 1: wrap unknown CmdType in FrameError instead of bare ValueError
    try:
        cmd_type = CmdType(data[1])
    except ValueError:
        raise FrameError(f"Unknown CmdType: 0x{data[1]:02X}")
    payload = bytes(data[3:3 + payload_len])
    # Issue 6: trailing bytes are intentionally ignored — the serial driver
    # passes exactly one frame at a time, so any bytes beyond expected_len
    # belong to the next frame and are left for the caller to handle.
    return cmd_type, payload


def encode_drive(speed: float, steering: float) -> bytes:
    speed = max(-1.0, min(1.0, speed))
    steering = max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, steering))  # Issue 3
    payload = struct.pack('<ff', speed, steering)
    return encode_frame(CmdType.DRIVE, payload)


def encode_blade(state: bool) -> bytes:
    return encode_frame(CmdType.BLADE, bytes([1 if state else 0]))


def encode_deck_lift(front_raised: bool, rear_raised: bool) -> bytes:
    """Command the independent front and rear mower lift actuators."""
    return encode_frame(
        CmdType.DECK_LIFT,
        bytes([1 if front_raised else 0, 1 if rear_raised else 0]),
    )


def encode_estop() -> bytes:
    return encode_frame(CmdType.ESTOP, b'')


def encode_ping(seq: int) -> bytes:
    return encode_frame(CmdType.PING, seq.to_bytes(PING_SEQ_BYTES, 'little'))  # Issue 3


def decode_sensors(payload: bytes) -> dict:
    # Issue 2: check payload length before unpacking
    if len(payload) != _SENSORS_SIZE:
        raise FrameError(f"SENSORS payload must be {_SENSORS_SIZE} bytes, got {len(payload)}")
    rain_adc, lift_raw, encoder_ticks = struct.unpack('<HBI', payload)
    return {
        'rain_adc': rain_adc,
        'lift': bool(lift_raw),
        'encoder_ticks': encoder_ticks,
    }


def decode_soc(payload: bytes) -> dict:
    # Issue 2: check payload length before unpacking
    if len(payload) != _SOC_SIZE:
        raise FrameError(f"SOC payload must be {_SOC_SIZE} bytes, got {len(payload)}")
    soc_percent, voltage_mv = struct.unpack('<BH', payload)
    return {
        'soc_percent': soc_percent,
        'voltage_mv': voltage_mv,
    }


def decode_status(payload: bytes) -> dict:
    # Issue 2: check payload length before unpacking
    if len(payload) != _STATUS_SIZE:
        raise FrameError(f"STATUS payload must be {_STATUS_SIZE} bytes, got {len(payload)}")
    watchdog_ok, blade_running, error_flags, charging = struct.unpack('<BBBB', payload)
    return {
        'watchdog_ok': bool(watchdog_ok),
        'blade_running': bool(blade_running),
        'error_flags': error_flags,
        'charging': bool(charging),
    }
