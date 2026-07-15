import struct

import pytest
from mower.hal.protocol import (
    CmdType, encode_frame, decode_frame, FrameError,
    encode_drive, encode_ping, encode_estop, encode_blade,
    decode_sensors, decode_soc, decode_status,
)

START_BYTE = 0xAA


class TestCrcAndFraming:
    def test_encode_frame_starts_with_start_byte(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert frame[0] == START_BYTE

    def test_encode_frame_embeds_cmd_type(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert frame[1] == CmdType.PING

    def test_encode_frame_embeds_payload_length(self):
        payload = b'\x01\x02\x03'
        frame = encode_frame(CmdType.PING, payload)
        assert frame[2] == len(payload)

    def test_encode_frame_crc_is_last_byte(self):
        frame = encode_frame(CmdType.PING, b'\x01')
        assert len(frame) == 5  # header(3) + payload(1) + crc(1)
        # Flipping the payload must change the CRC (last byte)
        frame2 = encode_frame(CmdType.PING, b'\x02')
        assert frame[-1] != frame2[-1]

    def test_decode_valid_frame(self):
        frame = encode_frame(CmdType.PING, b'\x07')
        cmd_type, payload = decode_frame(frame)
        assert cmd_type == CmdType.PING
        assert payload == b'\x07'

    def test_decode_wrong_start_byte_raises(self):
        frame = bytearray(encode_frame(CmdType.PING, b'\x01'))
        frame[0] = 0xBB
        with pytest.raises(FrameError, match="start byte"):
            decode_frame(bytes(frame))

    def test_decode_bad_crc_raises(self):
        frame = bytearray(encode_frame(CmdType.PING, b'\x01'))
        frame[-1] ^= 0xFF  # flip CRC
        with pytest.raises(FrameError, match="CRC"):
            decode_frame(bytes(frame))

    def test_decode_truncated_frame_raises(self):
        with pytest.raises(FrameError):
            decode_frame(b'\xAA\x01')  # too short


class TestCommandEncoding:
    def test_encode_drive(self):
        frame = encode_drive(speed=0.5, steering=15.0)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DRIVE
        assert len(payload) == 8  # two float32 = 8 bytes

    def test_encode_drive_speed_clipped(self):
        # speed must be clamped to [-1.0, 1.0]
        frame = encode_drive(speed=2.5, steering=0.0)
        _, payload = decode_frame(frame)
        speed, _ = struct.unpack('<ff', payload)
        assert speed == pytest.approx(1.0)

    def test_encode_drive_steering_clipped(self):
        # Issue 4: steering must be clamped to [-45.0, 45.0]
        frame = encode_drive(speed=0.0, steering=99.0)
        _, payload = decode_frame(frame)
        _, steering = struct.unpack('<ff', payload)
        assert steering == pytest.approx(45.0)

    def test_encode_blade_on(self):
        frame = encode_blade(True)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.BLADE
        assert payload == b'\x01'

    def test_encode_blade_off(self):
        frame = encode_blade(False)
        _, payload = decode_frame(frame)
        assert payload == b'\x00'

    def test_encode_estop(self):
        frame = encode_estop()
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.ESTOP
        assert payload == b''

    def test_encode_ping(self):
        frame = encode_ping(seq=42)
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.PING
        assert int.from_bytes(payload, 'little') == 42


class TestTelemetryDecoding:
    def test_decode_sensors(self):
        payload = struct.pack('<HBI', 512, 0, 1024)  # rain_adc, lift, encoder
        data = decode_sensors(payload)
        assert data['rain_adc'] == 512
        assert data['lift'] is False
        assert data['encoder_ticks'] == 1024

    def test_decode_sensors_lift_true(self):
        payload = struct.pack('<HBI', 700, 1, 0)
        data = decode_sensors(payload)
        assert data['lift'] is True

    def test_decode_soc(self):
        payload = struct.pack('<BH', 75, 14800)  # 75%, 14.8V
        data = decode_soc(payload)
        assert data['soc_percent'] == 75
        assert data['voltage_mv'] == 14800

    def test_decode_status(self):
        payload = struct.pack('<BBBB', 1, 0, 0, 0)  # watchdog_ok, blade_running, error_flags, charging
        data = decode_status(payload)
        assert data['watchdog_ok'] is True
        assert data['blade_running'] is False
        assert data['error_flags'] == 0
        assert data['charging'] is False

    def test_decode_status_charging_true(self):
        payload = struct.pack('<BBBB', 1, 0, 0, 1)
        data = decode_status(payload)
        assert data['charging'] is True

    def test_decode_status_wrong_length_raises(self):
        with pytest.raises(FrameError, match="STATUS payload"):
            decode_status(struct.pack('<BBB', 1, 0, 0))  # legacy 3-byte payload
