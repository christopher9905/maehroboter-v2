import math
import time
import pytest
from unittest.mock import MagicMock, patch
from mower.nav.imu_reader import ImuReader, ImuReading


def _make_mock_sensor(heading=45.0, pitch=2.0, roll=-1.0,
                      gyro_z_rps=0.1) -> MagicMock:
    """Returns a mock that mimics the BNO08X sensor object."""
    sensor = MagicMock()
    sensor.euler = (heading, pitch, roll)
    sensor.gyro = (0.0, 0.0, gyro_z_rps)
    return sensor


def _make_reader_with_mock_sensor(**sensor_kwargs) -> tuple[ImuReader, list[ImuReading]]:
    readings: list[ImuReading] = []
    mock_i2c = MagicMock()
    mock_sensor = _make_mock_sensor(**sensor_kwargs)

    reader = ImuReader(i2c=mock_i2c)
    reader.on_imu = readings.append

    # Patch the adafruit import so no hardware is needed
    with patch("mower.nav.imu_reader.BNO08X_I2C", return_value=mock_sensor), \
         patch("mower.nav.imu_reader.BNO_REPORT_EULER", 1), \
         patch("mower.nav.imu_reader.BNO_REPORT_GYROSCOPE", 2):
        reader._init_sensor()
        reader._read_once()

    return reader, readings


class TestImuReader:
    def test_heading_published(self):
        _, readings = _make_reader_with_mock_sensor(heading=90.0)
        assert len(readings) == 1
        assert abs(readings[0].heading_deg - 90.0) < 0.01

    def test_pitch_published(self):
        _, readings = _make_reader_with_mock_sensor(pitch=5.0)
        assert abs(readings[0].pitch_deg - 5.0) < 0.01

    def test_roll_published(self):
        _, readings = _make_reader_with_mock_sensor(roll=-3.0)
        assert abs(readings[0].roll_deg - (-3.0)) < 0.01

    def test_yaw_rate_converted_to_dps(self):
        gyro_z = 0.5  # rad/s
        _, readings = _make_reader_with_mock_sensor(gyro_z_rps=gyro_z)
        assert abs(readings[0].yaw_rate_dps - math.degrees(gyro_z)) < 0.01

    def test_heading_wraps_to_0_360(self):
        _, readings = _make_reader_with_mock_sensor(heading=370.0)
        assert 0.0 <= readings[0].heading_deg < 360.0

    def test_timestamp_is_recent(self):
        _, readings = _make_reader_with_mock_sensor()
        assert readings[0].timestamp <= time.monotonic()
        assert readings[0].timestamp > time.monotonic() - 1.0
