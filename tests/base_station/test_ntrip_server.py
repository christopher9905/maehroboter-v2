import base64
import socket
import time
import pytest

from base_station.ntrip_server import NtripServer
from mower.nav.ntrip_client import NtripClient

HOST = "127.0.0.1"
MOUNTPOINT = "MV2BASE"
USER = "rover"
PASSWORD = "s3cret"


@pytest.fixture
def server():
    srv = NtripServer(host=HOST, port=0, mountpoint=MOUNTPOINT, user=USER, password=PASSWORD)
    srv.start()
    yield srv
    srv.stop()


def _bound_port(srv):
    return srv._listener.getsockname()[1]


def _handshake_request(mountpoint, user, password):
    creds = base64.b64encode(f"{user}:{password}".encode()).decode()
    return (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        f"Host: {HOST}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: NTRIP MowerClient/1.0\r\n"
        f"Authorization: Basic {creds}\r\n"
        f"\r\n"
    ).encode()


def test_valid_handshake_returns_success(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" in resp


def test_wrong_mountpoint_rejected(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request("WRONGPOINT", USER, PASSWORD))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" not in resp


def test_wrong_credentials_rejected(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, "wrong-password"))
    resp = sock.recv(1024)
    sock.close()
    assert b"ICY 200 OK" not in resp


def test_broadcast_delivers_to_connected_client(server):
    sock = socket.create_connection((HOST, _bound_port(server)), timeout=2.0)
    sock.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
    sock.recv(1024)  # consume handshake response
    server.broadcast(b"\x01\x02\x03rtcm")
    data = sock.recv(1024)
    sock.close()
    assert data == b"\x01\x02\x03rtcm"


def test_broadcast_reaches_multiple_clients(server):
    port = _bound_port(server)
    s1 = socket.create_connection((HOST, port), timeout=2.0)
    s2 = socket.create_connection((HOST, port), timeout=2.0)
    for s in (s1, s2):
        s.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
        s.recv(1024)
    server.broadcast(b"corrections")
    d1 = s1.recv(1024)
    d2 = s2.recv(1024)
    s1.close()
    s2.close()
    assert d1 == b"corrections"
    assert d2 == b"corrections"


def test_disconnected_client_does_not_break_broadcast(server):
    port = _bound_port(server)
    s1 = socket.create_connection((HOST, port), timeout=2.0)
    s2 = socket.create_connection((HOST, port), timeout=2.0)
    for s in (s1, s2):
        s.sendall(_handshake_request(MOUNTPOINT, USER, PASSWORD))
        s.recv(1024)
    s1.close()  # s1 goes away before the broadcast
    time.sleep(0.05)
    server.broadcast(b"still-works")  # must not raise
    data = s2.recv(1024)
    s2.close()
    assert data == b"still-works"


def test_real_ntrip_client_receives_broadcast(server):
    """End-to-end interop proof against the actual, unmodified NtripClient."""
    received = []
    client = NtripClient(host=HOST, port=_bound_port(server), mountpoint=MOUNTPOINT,
                         user=USER, password=PASSWORD)
    client.on_rtcm = lambda data: received.append(data)
    client.start()
    time.sleep(0.3)  # allow the client's connect + first recv loop to run
    server.broadcast(b"real-client-rtcm")
    time.sleep(0.2)
    client.stop()
    assert b"real-client-rtcm" in received
