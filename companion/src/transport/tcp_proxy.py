"""Authenticated TCP transport for a Jetson-owned USB relay."""

from __future__ import annotations

import select
import socket
import threading
from typing import Final
from urllib.parse import parse_qs, urlparse

from diagnostics import print_exception

RELAY_AUTH_PREFIX: Final[bytes] = b"HEXAPOD_RELAY/1 "
RELAY_OK: Final[bytes] = b"OK"
RELAY_UNAUTHORIZED: Final[bytes] = b"UNAUTHORIZED"
RELAY_BUSY: Final[bytes] = b"BUSY"
MAX_RELAY_HANDSHAKE_BYTES: Final[int] = 512


def validate_relay_token(token: str) -> None:
    """Reject ambiguous or excessively large proxy authentication tokens."""
    if not token or len(token) > 256 or any(character.isspace() for character in token):
        raise ValueError("relay token must be a non-empty single token")
    try:
        token.encode("ascii")
    except UnicodeEncodeError as error:
        raise ValueError("relay token must contain ASCII characters only") from error


def build_relay_auth(token: str) -> bytes:
    """Build the one-line preamble consumed before transparent frame relay."""
    validate_relay_token(token)
    return RELAY_AUTH_PREFIX + token.encode("ascii") + b"\n"


def parse_relay_auth(line: bytes) -> str:
    """Parse a client preamble received by the Jetson relay."""
    if not line.startswith(RELAY_AUTH_PREFIX):
        raise ValueError("missing relay authentication preamble")
    try:
        token = line[len(RELAY_AUTH_PREFIX) :].decode("ascii")
    except UnicodeDecodeError as error:
        raise ValueError("relay token must contain ASCII characters only") from error
    validate_relay_token(token)
    return token


def receive_relay_line(connection: socket.socket) -> tuple[bytes, bytes]:
    """Read one bounded newline-delimited preamble and preserve later bytes."""
    buffered = bytearray()
    while True:
        newline = buffered.find(b"\n")
        if newline >= 0:
            return bytes(buffered[:newline]), bytes(buffered[newline + 1 :])
        if len(buffered) >= MAX_RELAY_HANDSHAKE_BYTES:
            raise ValueError("relay authentication preamble is too long")
        chunk = connection.recv(min(256, MAX_RELAY_HANDSHAKE_BYTES - len(buffered)))
        if not chunk:
            raise ConnectionError("relay closed during authentication")
        buffered.extend(chunk)


class TcpProxyLink:
    """A ``ByteStream`` over an authenticated TCP connection to a Jetson relay."""

    min_write_interval = 0.0
    synchronous_requests = False

    def __init__(self, connection: socket.socket, initial_data: bytes = b"") -> None:
        self._connection = connection
        self._initial_data = bytearray(initial_data)
        self._buffer_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False

    def read(self, size: int = 1) -> bytes:
        with self._buffer_lock:
            if self._initial_data:
                count = min(max(1, size), len(self._initial_data))
                chunk = bytes(self._initial_data[:count])
                del self._initial_data[:count]
                return chunk
        try:
            chunk = self._connection.recv(max(1, size))
        except (BlockingIOError, socket.timeout):
            return b""
        if not chunk:
            raise OSError("Jetson TCP relay closed the connection")
        return chunk

    def write(self, data: bytes) -> int:
        if self._closed:
            raise OSError("TCP proxy link is closed")
        self._connection.sendall(data)
        return len(data)

    @property
    def in_waiting(self) -> int:
        with self._buffer_lock:
            if self._initial_data:
                return len(self._initial_data)
        try:
            readable, _, _ = select.select([self._connection], [], [], 0)
        except (OSError, ValueError):
            return 0
        return 512 if readable else 0

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._connection.close()


def is_tcp_proxy_endpoint(endpoint: str) -> bool:
    """Whether an endpoint names the Jetson TCP relay rather than a serial port."""
    return urlparse(endpoint).scheme.lower() == "tcp"


def connect_tcp_proxy(
    endpoint: str,
    *,
    connect_timeout_s: float = 1.0,
    read_timeout_s: float = 0.05,
) -> TcpProxyLink:
    """Open and authenticate a TCP proxy link from a ``tcp://`` endpoint."""
    parsed = urlparse(endpoint)
    if parsed.scheme.lower() != "tcp" or not parsed.hostname:
        raise ValueError("proxy endpoint must use tcp://host:port?token=<token>")
    if parsed.username or parsed.password or parsed.fragment or parsed.path not in ("", "/"):
        raise ValueError("proxy endpoint must not include credentials, paths, or fragments")
    try:
        port = parsed.port
    except ValueError as error:
        raise ValueError("proxy endpoint has an invalid TCP port") from error
    if port is None or not 1 <= port <= 65535:
        raise ValueError("proxy endpoint must include a TCP port")
    tokens = parse_qs(parsed.query, keep_blank_values=True).get("token", [])
    if len(tokens) != 1:
        raise ValueError("proxy endpoint must include exactly one token query value")
    token = tokens[0]
    validate_relay_token(token)

    connection = socket.create_connection(
        (parsed.hostname, port), timeout=connect_timeout_s
    )
    try:
        connection.settimeout(connect_timeout_s)
        connection.sendall(build_relay_auth(token))
        reply, initial_data = receive_relay_line(connection)
        if reply != RELAY_OK:
            raise ConnectionRefusedError("Jetson relay rejected the connection")
        connection.settimeout(read_timeout_s)
        return TcpProxyLink(connection, initial_data)
    except Exception:
        connection.close()
        raise


def open_tcp_proxy(endpoint: str) -> TcpProxyLink | None:
    """Open a proxy endpoint or return ``None`` after a user-visible error."""
    try:
        return connect_tcp_proxy(endpoint)
    except Exception as error:
        print_exception(f"opening Jetson TCP proxy {endpoint} failed", error)
        return None