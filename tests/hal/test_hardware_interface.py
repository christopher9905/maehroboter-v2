import threading
import time
import pytest
from unittest.mock import MagicMock, patch, call
from mower.hal.protocol import encode_frame, CmdType, encode_drive
from mower.hal.serial_driver import SerialDriver


class TestSerialDriver:
    def test_send_puts_frame_on_serial(self, mock_serial):
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.start()
        frame = encode_drive(0.5, 0.0)
        driver.send(frame)
        time.sleep(0.05)
        driver.stop()
        mock_serial.write.assert_called_with(frame)

    def test_receive_callback_called_on_valid_frame(self, mock_serial):
        import struct
        payload = struct.pack('<HBI', 300, 0, 500)
        frame = encode_frame(CmdType.SENSORS, payload)
        call_count = 0
        def read_side_effect(n):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                return bytes([0xAA])
            elif call_count == 2:
                return frame[1:2]  # CMD_TYPE
            elif call_count == 3:
                return frame[2:3]  # PAYLOAD_LEN
            elif call_count == 4:
                return frame[3:-1]  # PAYLOAD
            elif call_count == 5:
                return frame[-1:]   # CRC
            else:
                time.sleep(1)
                return b''
        mock_serial.read.side_effect = read_side_effect
        received = []
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.on_frame = lambda cmd, payload: received.append((cmd, payload))
        driver.start()
        time.sleep(0.2)
        driver.stop()
        assert len(received) == 1
        assert received[0][0] == CmdType.SENSORS

    def test_stop_terminates_threads(self, mock_serial):
        driver = SerialDriver(port='/dev/ttyACM0', baud=921600)
        driver._serial = mock_serial
        driver.start()
        time.sleep(0.05)
        driver.stop()
        assert not driver._send_thread.is_alive()
        assert not driver._recv_thread.is_alive()


@pytest.fixture
def mock_serial():
    s = MagicMock()
    s.read.return_value = b''
    s.write.return_value = None
    s.in_waiting = 0
    return s


from mower.hal.hardware_interface import HardwareInterface


class TestHardwareInterface:
    def test_drive_sends_drive_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(speed=0.3, steering=10.0)
        mock_driver.send.assert_called_once()
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DRIVE

    def test_estop_sends_estop_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.estop()
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, _ = decode_frame(frame)
        assert cmd == CmdType.ESTOP

    def test_blade_on_sends_blade_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.set_blade(True)
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.BLADE
        assert payload == b'\x01'

    def test_telemetry_callback_on_sensors(self, mock_driver):
        import struct
        from mower.hal.protocol import encode_frame, CmdType, decode_frame
        hw = HardwareInterface(driver=mock_driver)
        received = []
        hw.on_sensors = lambda d: received.append(d)
        payload = struct.pack('<HBI', 300, 0, 1000)
        frame = encode_frame(CmdType.SENSORS, payload)
        cmd, p = decode_frame(frame)
        hw._on_frame(cmd, p)
        assert len(received) == 1
        assert received[0]['rain_adc'] == 300
        assert received[0]['encoder_ticks'] == 1000

    def test_lift_triggers_estop(self, mock_driver):
        import struct
        from mower.hal.protocol import encode_frame, CmdType, decode_frame
        hw = HardwareInterface(driver=mock_driver)
        payload = struct.pack('<HBI', 100, 1, 0)  # lift=True
        frame = encode_frame(CmdType.SENSORS, payload)
        cmd, p = decode_frame(frame)
        hw._on_frame(cmd, p)
        # estop must have been sent
        frames_sent = [mock_driver.send.call_args_list[i][0][0]
                       for i in range(mock_driver.send.call_count)]
        cmds = [decode_frame(f)[0] for f in frames_sent]
        assert CmdType.ESTOP in cmds


@pytest.fixture
def mock_driver():
    d = MagicMock()
    return d
