"""Minimal NTRIP caster: TCP server compatible with
mower.nav.ntrip_client.NtripClient's handshake. Broadcasts RTCM3 bytes (fed
via broadcast()) to all connected rovers.

No RTCM3 parsing, no VRS/GGA handling — a single stationary base station
serves the same corrections to every connected rover regardless of position.
"""
import base64
import logging
import socket
import threading
from typing import Optional

logger = logging.getLogger(__name__)

_RECV_BUFSIZE = 2048
_SUCCESS_RESPONSE = b"ICY 200 OK\r\n\r\n"
_REJECT_RESPONSE = b"ERROR - Bad Mountpoint or Credentials\r\n"


class NtripServer:
    def __init__(self, host: str, port: int, mountpoint: str, user: str, password: str):
        self._host = host
        self._port = port
        self._mountpoint = mountpoint
        self._user = user
        self._password = password
        self._listener: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._running = False
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self._host, self._port))
        self._listener.listen(5)
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._listener:
            self._listener.close()
        if self._accept_thread:
            self._accept_thread.join(timeout=2.0)
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()

    def broadcast(self, data: bytes) -> None:
        with self._lock:
            clients = list(self._clients)
        dead = []
        for c in clients:
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        if dead:
            with self._lock:
                for c in dead:
                    if c in self._clients:
                        self._clients.remove(c)
                    try:
                        c.close()
                    except OSError:
                        pass

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, _addr = self._listener.accept()
            except OSError:
                break  # listener closed -> stop() was called
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def _handle_client(self, conn: socket.socket) -> None:
        try:
            request = conn.recv(_RECV_BUFSIZE)
            if self._is_authorized(request):
                conn.sendall(_SUCCESS_RESPONSE)
                with self._lock:
                    self._clients.append(conn)
            else:
                conn.sendall(_REJECT_RESPONSE)
                conn.close()
        except OSError:
            try:
                conn.close()
            except OSError:
                pass

    def _is_authorized(self, request: bytes) -> bool:
        try:
            lines = request.decode(errors="replace").split("\r\n")
            request_line = lines[0]
            _method, path, _proto = request_line.split(" ", 2)
            mountpoint = path.lstrip("/")
            if mountpoint != self._mountpoint:
                return False
            auth_header = next(
                (l for l in lines[1:] if l.lower().startswith("authorization:")), None
            )
            if auth_header is None:
                return False
            b64 = auth_header.split(" ", 2)[-1]
            decoded = base64.b64decode(b64).decode(errors="replace")
            user, _, password = decoded.partition(":")
            return user == self._user and password == self._password
        except Exception:
            return False
