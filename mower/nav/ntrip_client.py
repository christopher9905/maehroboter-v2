import base64
import logging
import queue
import socket
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class NtripClient:
    """NTRIP 2.0 client: receives RTCM3 corrections from a caster.

    Usage:
        client = NtripClient(host, port, mountpoint, user, password)
        client.on_rtcm = lambda data: gps_serial.write(data)
        gps_reader.on_gga_sentence = client.send_gga
        client.start()
    """

    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str, password: str):
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._user = user
        self._password = password
        self._gga_queue: queue.Queue = queue.Queue(maxsize=4)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.on_rtcm: Optional[Callable[[bytes], None]] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def send_gga(self, sentence: str):
        """Forward a GGA sentence to the caster for VRS-based corrections."""
        line = sentence.strip() + "\r\n"
        try:
            self._gga_queue.put_nowait(line)
        except queue.Full:
            pass

    def _connect(self) -> socket.socket:
        credentials = base64.b64encode(
            f"{self._user}:{self._password}".encode()
        ).decode()
        request = (
            f"GET /{self._mountpoint} HTTP/1.0\r\n"
            f"Host: {self._host}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP MowerClient/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            f"\r\n"
        )
        sock = socket.create_connection((self._host, self._port), timeout=10.0)
        sock.sendall(request.encode())
        resp = sock.recv(1024).decode(errors="replace")
        if "ICY 200 OK" not in resp and "200 OK" not in resp:
            sock.close()
            raise ConnectionError(f"NTRIP refused: {resp[:80]}")
        return sock

    def _run(self):
        while self._running:
            try:
                sock = self._connect()
                try:
                    sock.settimeout(5.0)
                    logger.info("NTRIP connected to %s/%s", self._host, self._mountpoint)
                    while self._running:
                        try:
                            gga = self._gga_queue.get_nowait()
                            sock.sendall(gga.encode())
                        except queue.Empty:
                            pass
                        except OSError:
                            break
                        try:
                            data = sock.recv(4096)
                            if data and self.on_rtcm:
                                self.on_rtcm(data)
                            elif data == b"":
                                break  # connection closed
                        except socket.timeout:
                            pass
                        except OSError:
                            break  # socket error — reconnect
                finally:
                    sock.close()
            except (ConnectionError, OSError) as e:
                logger.warning("NTRIP error: %s — reconnecting in 5 s", e)
                time.sleep(5.0)
            except Exception as e:
                logger.warning("NTRIP error: %s — reconnecting in 5 s", e)
                time.sleep(5.0)
