import base64
import queue
import socket
import threading
import time
import pytest
from unittest.mock import MagicMock, patch
from mower.nav.ntrip_client import NtripClient


def _make_client(**kwargs) -> NtripClient:
    defaults = dict(host="ntrip.example.com", port=2101,
                    mountpoint="SAPOS", user="user", password="pass")
    defaults.update(kwargs)
    return NtripClient(**defaults)


class TestNtripClientConnect:
    def test_http_request_contains_mountpoint(self):
        client = _make_client(mountpoint="MY_MOUNT")
        sent_data: list[bytes] = []

        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [b"ICY 200 OK\r\n\r\n", b""]
        mock_sock.sendall.side_effect = lambda d: sent_data.append(d)

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            sock = client._connect()

        combined = b"".join(sent_data).decode()
        assert "GET /MY_MOUNT HTTP/1.0" in combined

    def test_http_request_contains_basic_auth(self):
        client = _make_client(user="testuser", password="secret")
        sent_data: list[bytes] = []

        mock_sock = MagicMock()
        mock_sock.recv.return_value = b"ICY 200 OK\r\n\r\n"
        mock_sock.sendall.side_effect = lambda d: sent_data.append(d)

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            client._connect()

        combined = b"".join(sent_data).decode()
        expected = base64.b64encode(b"testuser:secret").decode()
        assert expected in combined

    def test_rejected_response_raises(self):
        client = _make_client()
        mock_sock = MagicMock()
        mock_sock.recv.return_value = b"HTTP/1.1 401 Unauthorized\r\n\r\n"

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock):
            with pytest.raises(ConnectionError):
                client._connect()


class TestNtripClientSendGga:
    def test_send_gga_queued(self):
        client = _make_client()
        client.send_gga("$GNGGA,123519,...,*47")
        assert not client._gga_queue.empty()
        queued = client._gga_queue.get_nowait()
        assert queued.endswith("\r\n")

    def test_send_gga_queue_does_not_block_when_full(self):
        client = _make_client()
        for _ in range(10):  # overfill the maxsize=4 queue
            client.send_gga("$GNGGA,test")
        # Should not raise or block
        assert client._gga_queue.full()


class TestNtripClientRtcmCallback:
    def test_on_rtcm_called_with_received_data(self):
        client = _make_client()
        received: list[bytes] = []
        client.on_rtcm = received.append

        rtcm_data = b"\xd3\x00\x13\x3e\xd7\xd3\x02\x02\x98\x0e\xde\xef\x34\xb4"
        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [
            b"ICY 200 OK\r\n\r\n",
            rtcm_data,
            socket.timeout(),
            b"",  # empty = connection closed, causes break out of inner loop
        ]
        mock_sock.sendall = MagicMock()

        def stop_on_sleep(seconds):
            client._running = False

        with patch("mower.nav.ntrip_client.socket.create_connection",
                   return_value=mock_sock), \
             patch("mower.nav.ntrip_client.time.sleep", side_effect=stop_on_sleep):
            client._running = True
            client._run()

        assert rtcm_data in received
