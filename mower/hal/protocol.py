import struct
import crcmod
from enum import IntEnum


class CmdType(IntEnum):
    DRIVE = 0x01
    BLADE = 0x02
    ESTOP = 0x03
    PING  = 0x04
    # Telemetry (Teensy → RPi)
    SENSORS = 0x10
    SOC     = 0x11
    STATUS  = 0x12


START_BYTE = 0xAA

# CRC-8/MAXIM: poly=0x31, initCrc=0x00, rev=True, xorOut=0x00
_crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')


class FrameError(Exception):
    pass


def _compute_crc(data: bytes) -> int:
    return _crc8(data)


def encode_frame(cmd_type: CmdType, payload: bytes) -> bytes:
    header = bytes([START_BYTE, int(cmd_type), len(payload)])
    body = header + payload
    crc = _compute_crc(body)
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
    expected_crc = _compute_crc(body)
    if received_crc != expected_crc:
        raise FrameError(f"CRC mismatch: got 0x{received_crc:02X}, expected 0x{expected_crc:02X}")
    cmd_type = CmdType(data[1])
    payload = bytes(data[3:3 + payload_len])
    return cmd_type, payload


def encode_drive(speed: float, steering: float) -> bytes:
    speed = max(-1.0, min(1.0, speed))
    steering = max(-45.0, min(45.0, steering))
    payload = struct.pack('<ff', speed, steering)
    return encode_frame(CmdType.DRIVE, payload)


def encode_blade(state: bool) -> bytes:
    return encode_frame(CmdType.BLADE, bytes([1 if state else 0]))


def encode_estop() -> bytes:
    return encode_frame(CmdType.ESTOP, b'')


def encode_ping(seq: int) -> bytes:
    return encode_frame(CmdType.PING, seq.to_bytes(2, 'little'))


def decode_sensors(payload: bytes) -> dict:
    rain_adc, lift_raw, encoder_ticks = struct.unpack('<HBI', payload)
    return {
        'rain_adc': rain_adc,
        'lift': bool(lift_raw),
        'encoder_ticks': encoder_ticks,
    }


def decode_soc(payload: bytes) -> dict:
    soc_percent, voltage_mv = struct.unpack('<BH', payload)
    return {
        'soc_percent': soc_percent,
        'voltage_mv': voltage_mv,
    }


def decode_status(payload: bytes) -> dict:
    watchdog_ok, blade_running, error_flags = struct.unpack('<BBB', payload)
    return {
        'watchdog_ok': bool(watchdog_ok),
        'blade_running': bool(blade_running),
        'error_flags': error_flags,
    }
