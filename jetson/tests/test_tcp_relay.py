"""Hardware-free end-to-end tests for the single-client Jetson TCP relay."""

from __future__ import annotations

import threading
import time

import pytest

from hexapod_jetson_bridge.tcp_relay import TcpRelayServer
from hexapod_protocol.framing import Header, MsgType, decode_frame_body, encode_frame
from transport import FrameExtractor, open_transport
from transport.protocol_client import ProtocolClient
from transport.tcp_proxy import connect_tcp_proxy


class RespondingStream:
    """In-memory MCU byte stream used by the raw relay server."""

    def __init__(self) -> None:
        self._inbound = bytearray()
        self._lock = threading.Lock()
        self._extractor = FrameExtractor()
        self.request_sequences: list[int] = []
        self.closed = False

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            count = min(size, len(self._inbound))
            chunk = bytes(self._inbound[:count])
            del self._inbound[:count]
            return chunk

    def write(self, data: bytes) -> int:
        for frame in self._extractor.push(data):
            header, _ = decode_frame_body(frame[1:-1])
            self.request_sequences.append(header.seq)
            payload = bytes([0, 3, 1, 2, 3]) + b"relay-test".ljust(16, b"\0")
            response = Header(
                msg_type=int(MsgType.RESPONSE),
                msg_id=header.msg_id,
                seq=header.seq,
            )
            with self._lock:
                self._inbound.extend(encode_frame(response, payload))
        return len(data)

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._inbound)

    def close(self) -> None:
        self.closed = True


def endpoint(server: TcpRelayServer, token: str) -> str:
    return f"tcp://127.0.0.1:{server.port}?token={token}"


def test_relay_preserves_protocol_request_response_correlation():
    token = "test-relay-token"
    stream = RespondingStream()
    server = TcpRelayServer(stream, token, port=0)
    server.start()
    link = open_transport(endpoint(server, token))
    assert link is not None
    client = ProtocolClient(link, response_timeout=0.5)
    client.start()
    try:
        hello = client.hello()
        assert hello is not None
        assert hello.device_name == "relay-test"
        assert stream.request_sequences == [1]
    finally:
        client.stop()
        server.close()
    assert stream.closed


def test_relay_rejects_wrong_token_and_second_client():
    token = "test-relay-token"
    server = TcpRelayServer(RespondingStream(), token, port=0)
    server.start()
    first = connect_tcp_proxy(endpoint(server, token))
    try:
        with pytest.raises(ConnectionRefusedError):
            connect_tcp_proxy(endpoint(server, "wrong-token"))
        with pytest.raises(ConnectionRefusedError):
            connect_tcp_proxy(endpoint(server, token))
    finally:
        first.close()
        server.close()


def test_relay_allows_reconnect_after_mac_disconnect():
    token = "test-relay-token"
    server = TcpRelayServer(RespondingStream(), token, port=0)
    server.start()
    first = connect_tcp_proxy(endpoint(server, token))
    first.close()
    deadline = time.monotonic() + 1.0
    while server.client_connected and time.monotonic() < deadline:
        time.sleep(0.01)
    try:
        second = connect_tcp_proxy(endpoint(server, token))
    finally:
        server.close()
    second.close()


def test_relay_shutdown_marks_mac_protocol_client_disconnected():
    token = "test-relay-token"
    server = TcpRelayServer(RespondingStream(), token, port=0)
    server.start()
    link = connect_tcp_proxy(endpoint(server, token))
    client = ProtocolClient(link, response_timeout=0.5)
    client.start()
    try:
        server.close()
        deadline = time.monotonic() + 1.0
        while client.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not client.connected
    finally:
        client.stop()