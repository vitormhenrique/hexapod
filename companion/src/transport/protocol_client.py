"""Typed, threaded protocol client.

Wraps a :class:`ByteStream` with a background reader thread. Commands are sent
synchronously (``request``) with seq-matched response correlation; telemetry
frames are dispatched to registered callbacks. UI-independent: the CLI uses it
directly and the PySide6 app wraps it in a Qt service.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Callable, Optional

from hexapod_protocol import api
from hexapod_protocol.framing import (
    Header,
    MsgType,
    DecodeError,
    decode_frame_body,
)
from hexapod_protocol import telemetry as tlm

from . import ByteStream, FrameExtractor


@dataclass
class Response:
    header: Header
    payload: bytes

    @property
    def is_error(self) -> bool:
        return api.is_error(self.header)


TelemetryCallback = Callable[[int, object, Header], None]
"""Called as ``cb(stream_id, decoded_record, header)`` for each telemetry frame."""

ConnectionCallback = Callable[[bool], None]


class ProtocolClient:
    """Send commands and receive responses / telemetry over a byte stream."""

    def __init__(self, stream: ByteStream, response_timeout: float = 1.0) -> None:
        self._stream = stream
        self._extractor = FrameExtractor()
        self._timeout = response_timeout
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._write_lock = threading.Lock()

        # seq -> single-slot queue for the matching response.
        self._pending: dict[int, "Queue[Response]"] = {}
        self._pending_lock = threading.Lock()

        self._telemetry_cbs: list[TelemetryCallback] = []
        self._connection_cbs: list[ConnectionCallback] = []

        self._reader: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._connected = False

        # Lightweight stats for the diagnostics page.
        self.rx_frames = 0
        self.tx_frames = 0
        self.decode_errors = 0

    # --- lifecycle --------------------------------------------------------

    def start(self) -> None:
        if self._reader and self._reader.is_alive():
            return
        self._running.set()
        self._reader = threading.Thread(
            target=self._read_loop, name="hexapod-reader", daemon=True
        )
        self._reader.start()
        self._set_connected(True)

    def stop(self) -> None:
        self._running.clear()
        if self._reader:
            self._reader.join(timeout=1.0)
        self._set_connected(False)
        try:
            self._stream.close()
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._connected

    # --- callbacks --------------------------------------------------------

    def on_telemetry(self, cb: TelemetryCallback) -> None:
        self._telemetry_cbs.append(cb)

    def on_connection(self, cb: ConnectionCallback) -> None:
        self._connection_cbs.append(cb)

    def _set_connected(self, value: bool) -> None:
        if value == self._connected:
            return
        self._connected = value
        for cb in list(self._connection_cbs):
            try:
                cb(value)
            except Exception:
                pass

    # --- sending ----------------------------------------------------------

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFF
            if self._seq == 0:
                self._seq = 1
            return self._seq

    def request(self, msg_id: int, payload: bytes = b"") -> Optional[Response]:
        """Send a command and wait for the seq-matched response.

        Returns ``None`` on timeout. Thread-safe.
        """
        seq = self._next_seq()
        slot: "Queue[Response]" = Queue(maxsize=1)
        with self._pending_lock:
            self._pending[seq] = slot
        try:
            frame = api.build_command(msg_id, seq=seq, payload=payload)
            self._write(frame)
            return slot.get(timeout=self._timeout)
        except Empty:
            return None
        finally:
            with self._pending_lock:
                self._pending.pop(seq, None)

    def send(self, msg_id: int, payload: bytes = b"") -> None:
        """Fire-and-forget command (no response wait)."""
        seq = self._next_seq()
        self._write(api.build_command(msg_id, seq=seq, payload=payload))

    def _write(self, frame: bytes) -> None:
        with self._write_lock:
            self._stream.write(frame)
            self.tx_frames += 1

    # --- high-level commands ---------------------------------------------

    def hello(self) -> Optional[api.HelloInfo]:
        r = self.request(api.MSG_HELLO)
        return api.parse_hello(r.payload) if r else None

    def get_status(self) -> Optional[api.StatusInfo]:
        r = self.request(api.MSG_GET_STATUS)
        return api.parse_status(r.payload) if r else None

    def get_capabilities(self) -> Optional[api.Capabilities]:
        r = self.request(api.MSG_GET_CAPABILITIES)
        return api.parse_capabilities(r.payload) if r else None

    def subscribe(self, stream_id: int, rate_hz: int) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_SUBSCRIBE, struct_subscribe(stream_id, rate_hz))
        return api.parse_subscribe_result(r.payload) if r else None

    def set_stream_rate(
        self, stream_id: int, rate_hz: int
    ) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_SET_STREAM_RATE, struct_subscribe(stream_id, rate_hz))
        return api.parse_subscribe_result(r.payload) if r else None

    def unsubscribe(self, stream_id: int) -> Optional[api.SubscribeResult]:
        r = self.request(api.MSG_UNSUBSCRIBE, bytes([stream_id]))
        return api.parse_subscribe_result(r.payload) if r else None

    def get_stream_stats(self) -> Optional[api.StreamStats]:
        r = self.request(api.MSG_GET_STREAM_STATS)
        return api.parse_stream_stats(r.payload) if r else None

    # --- reader loop ------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running.is_set():
            try:
                waiting = getattr(self._stream, "in_waiting", 0) or 0
                chunk = self._stream.read(max(1, min(waiting, 512)))
            except Exception:
                self._set_connected(False)
                break
            if not chunk:
                time.sleep(0.005)
                continue
            for frame in self._extractor.push(chunk):
                self._handle_frame(frame)

    def _handle_frame(self, frame: bytes) -> None:
        if len(frame) < 2 or frame[0] != 0x00 or frame[-1] != 0x00:
            return
        try:
            header, payload = decode_frame_body(frame[1:-1])
        except DecodeError:
            self.decode_errors += 1
            return
        self.rx_frames += 1

        if header.msg_type == int(MsgType.TELEMETRY):
            self._dispatch_telemetry(header, payload)
            return

        # Response/event: route to a waiting request by seq.
        with self._pending_lock:
            slot = self._pending.get(header.seq)
        if slot is not None:
            try:
                slot.put_nowait(Response(header, payload))
            except Exception:
                pass

    def _dispatch_telemetry(self, header: Header, payload: bytes) -> None:
        stream_id = header.msg_id - api.MSG_TELEMETRY_BASE
        record = tlm.decode_stream(stream_id, payload)
        if record is None:
            return
        for cb in list(self._telemetry_cbs):
            try:
                cb(stream_id, record, header)
            except Exception:
                pass


def struct_subscribe(stream_id: int, rate_hz: int) -> bytes:
    import struct

    return struct.pack("<BH", stream_id & 0xFF, rate_hz & 0xFFFF)
