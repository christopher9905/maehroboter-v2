import threading
import queue
import logging
from typing import Callable, Optional
import serial
from mower.hal.protocol import decode_frame, FrameError, CmdType

logger = logging.getLogger(__name__)


class SerialDriver:
    """Thread-safe serial driver. Sends frames via queue, receives frames via callback."""

    def __init__(self, port: str, baud: int = 921600):
        self._port = port
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._send_queue: queue.Queue = queue.Queue(maxsize=64)
        self._priority_queue: queue.Queue = queue.Queue()  # ESTOP / emergency frames
        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None
        self.on_frame: Optional[Callable[[CmdType, bytes], None]] = None

    def start(self):
        if self._serial is None:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.1
            )
        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread.start()
        self._recv_thread.start()
        logger.info("SerialDriver started on %s @ %d baud", self._port, self._baud)

    def stop(self):
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=1.0)
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("SerialDriver stopped")

    def send(self, frame: bytes, priority: bool = False):
        if priority:
            # Emergency (ESTOP): discard any queued normal frames so an in-flight
            # drive command can't be written after the stop, then jump the queue.
            self._drain(self._send_queue)
            self._priority_queue.put(frame)
            return
        try:
            self._send_queue.put_nowait(frame)
        except queue.Full:
            logger.warning("Send queue full — dropping frame")

    @staticmethod
    def _drain(q: queue.Queue):
        try:
            while True:
                q.get_nowait()
        except queue.Empty:
            pass

    def _send_loop(self):
        while self._running:
            try:
                frame = self._priority_queue.get_nowait()   # ESTOP always first
                self._serial.write(frame)
                continue
            except queue.Empty:
                pass
            try:
                frame = self._send_queue.get(timeout=0.05)
                self._serial.write(frame)
            except queue.Empty:
                pass
            except Exception as e:
                logger.error("Send error: %s", e)

    def _recv_loop(self):
        """Read frames byte-by-byte: wait for start byte, then read header + payload + CRC."""
        while self._running:
            try:
                b = self._serial.read(1)
                if not b or b[0] != 0xAA:
                    continue
                cmd_byte = self._serial.read(1)
                len_byte = self._serial.read(1)
                if not cmd_byte or not len_byte:
                    continue
                payload_len = len_byte[0]
                payload = self._serial.read(payload_len)
                crc_byte = self._serial.read(1)
                if len(payload) != payload_len or not crc_byte:
                    continue
                frame = bytes([0xAA]) + cmd_byte + len_byte + payload + crc_byte
                cmd_type, decoded_payload = decode_frame(frame)
                if self.on_frame:
                    self.on_frame(cmd_type, decoded_payload)
            except FrameError as e:
                logger.warning("Frame error: %s", e)
            except Exception as e:
                if self._running:
                    logger.error("Recv error: %s", e)
