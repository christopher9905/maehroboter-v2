import time
import pytest

from base_station.rtcm_source import RtcmSerialSource


class _FakeSerial:
    """Mimics pyserial's Serial.read(size) -> bytes. A small sleep on empty
    reads mirrors real pyserial's blocking timeout behaviour, so the read
    loop doesn't busy-spin the CPU while a test waits for it."""

    def __init__(self, chunks):
        self._chunks = list(chunks)

    def read(self, size=1024):
        if self._chunks:
            return self._chunks.pop(0)
        time.sleep(0.01)
        return b""


class _FlakySerial:
    """Raises once on the first read(), then behaves like _FakeSerial."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self._raised = False

    def read(self, size=1024):
        if not self._raised:
            self._raised = True
            raise OSError("simulated USB hiccup")
        if self._chunks:
            return self._chunks.pop(0)
        time.sleep(0.01)
        return b""


def test_on_data_fires_with_bytes_read():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([b"\x01\x02\x03"]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.1)
    src.stop()
    assert received == [b"\x01\x02\x03"]


def test_empty_read_does_not_fire_callback():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.05)
    src.stop()
    assert received == []


def test_stop_terminates_thread():
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FakeSerial([]))
    src.start()
    time.sleep(0.05)
    src.stop()
    assert not src._thread.is_alive()


def test_transient_read_error_does_not_kill_thread():
    received = []
    src = RtcmSerialSource(port="/dev/ttyUSB0", serial_backend=_FlakySerial([b"\xaa\xbb"]))
    src.on_data = lambda data: received.append(data)
    src.start()
    time.sleep(0.1)  # first read raises, thread must survive and read again
    src.stop()
    assert received == [b"\xaa\xbb"]
    assert not src._thread.is_alive()
