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
    def test_latest_output_commands_are_observable_and_clamped(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        observed = []
        hw.on_outputs = observed.append

        hw.drive(speed=2.0, steering=-80.0)
        hw.set_blade(True)
        hw.set_deck_lift(True, False)

        outputs = hw.output_snapshot()
        assert outputs["speed_command"] == 1.0
        assert outputs["steering_deg"] == -45.0
        assert outputs["blade_enabled"] is True
        assert outputs["front_deck_raised"] is True
        assert outputs["rear_deck_raised"] is False
        assert observed[-1] == outputs

    def test_drive_sends_drive_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(speed=0.3, steering=10.0)
        mock_driver.send.assert_called_once()
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DRIVE

    def test_motion_context_is_published_without_drive_frame(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        observed = []
        hw.on_outputs = observed.append

        hw.set_motion_context(0.7, "headland")

        assert mock_driver.send.call_count == 0
        assert observed[-1]["target_speed_kmh"] == pytest.approx(0.7)
        assert observed[-1]["speed_mode"] == "headland"

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

    def test_deck_lift_sends_independent_front_rear_state(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.set_deck_lift(True, False)
        frame = mock_driver.send.call_args[0][0]
        from mower.hal.protocol import decode_frame, CmdType
        cmd, payload = decode_frame(frame)
        assert cmd == CmdType.DECK_LIFT
        assert payload == b'\x01\x00'

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

    def test_telemetry_callback_on_status_includes_charging(self, mock_driver):
        import struct
        from mower.hal.protocol import encode_frame, CmdType, decode_frame
        hw = HardwareInterface(driver=mock_driver)
        received = []
        hw.on_status = lambda d: received.append(d)
        payload = struct.pack('<BBBB', 1, 0, 0, 1)  # charging=True
        frame = encode_frame(CmdType.STATUS, payload)
        cmd, p = decode_frame(frame)
        hw._on_frame(cmd, p)
        assert len(received) == 1
        assert received[0]['charging'] is True

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


class TestDriveDirectionTracking:
    """The unsigned encoder cannot sense direction — the commanded drive
    direction signs the odometry speed before it reaches the localizer."""

    def test_initial_direction_is_forward(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        assert hw.last_drive_direction == 1

    def test_reverse_command_sets_negative_direction(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(-0.3, 0.0)
        assert hw.last_drive_direction == -1

    def test_forward_command_sets_positive_direction(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(-0.3, 0.0)
        hw.drive(0.3, 0.0)
        assert hw.last_drive_direction == 1

    def test_zero_speed_keeps_previous_direction(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(-0.3, 0.0)
        hw.drive(0.0, 0.0)  # stop tick during a gear change
        assert hw.last_drive_direction == -1


class TestEstopPriority:
    """ESTOP must win serial ordering over an in-flight drive command
    (Phase 6 docking review — actuation-vs-estop race)."""

    def test_estop_sent_with_priority(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.estop()
        assert mock_driver.send.call_args.kwargs.get('priority') is True

    def test_drive_sent_without_priority(self, mock_driver):
        hw = HardwareInterface(driver=mock_driver)
        hw.drive(0.2, 0.0)
        assert mock_driver.send.call_args.kwargs.get('priority', False) is False

    def test_priority_send_drains_pending_normal_frames(self):
        from mower.hal.protocol import encode_drive, encode_estop
        d = SerialDriver(port='/dev/ttyACM0', baud=921600)
        d.send(encode_drive(0.5, 0.0))                 # queued normal drive
        assert d._send_queue.qsize() == 1
        d.send(encode_estop(), priority=True)          # emergency
        assert d._send_queue.qsize() == 0              # pending drive discarded
        assert d._priority_queue.qsize() == 1

    def test_priority_frame_written_before_normal(self, mock_serial):
        from mower.hal.protocol import encode_drive, encode_estop, decode_frame, CmdType
        d = SerialDriver(port='/dev/ttyACM0', baud=921600)
        d._serial = mock_serial
        d.send(encode_drive(0.5, 0.0))
        d.send(encode_estop(), priority=True)
        d.start()
        time.sleep(0.06)
        d.stop()
        written = [c[0][0] for c in mock_serial.write.call_args_list]
        cmds = [decode_frame(f)[0] for f in written]
        assert CmdType.ESTOP in cmds
        assert CmdType.DRIVE not in cmds               # drained, never sent
