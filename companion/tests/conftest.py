"""Shared test fixtures: an in-memory loopback byte stream + frame helpers."""

from __future__ import annotations

import threading

from hexapod_protocol import api
from hexapod_protocol.framing import Header, MsgType, encode_frame


class FakeStream:
    """In-memory ByteStream. ``feed`` pushes inbound bytes the client will read;
    ``written`` collects bytes the client wrote."""

    def __init__(self) -> None:
        self._inbound = bytearray()
        self._lock = threading.Lock()
        self.written = bytearray()
        self.closed = False

    def feed(self, data: bytes) -> None:
        with self._lock:
            self._inbound.extend(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            if not self._inbound:
                return b""
            n = min(size, len(self._inbound))
            chunk = bytes(self._inbound[:n])
            del self._inbound[:n]
            return chunk

    def write(self, data: bytes) -> int:
        with self._lock:
            self.written.extend(data)
        return len(data)

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._inbound)

    def close(self) -> None:
        self.closed = True


def make_response(msg_id: int, seq: int, payload: bytes, error: bool = False) -> bytes:
    flags = api.FLAG_ERROR if error else 0
    header = Header(
        msg_type=int(MsgType.RESPONSE), msg_id=msg_id, seq=seq, flags=flags
    )
    return encode_frame(header, payload)


def make_telemetry(stream_id: int, payload: bytes, ts_ms: int = 0) -> bytes:
    header = Header(
        msg_type=int(MsgType.TELEMETRY),
        msg_id=api.MSG_TELEMETRY_BASE + stream_id,
        timestamp_ms=ts_ms,
    )
    return encode_frame(header, payload)
