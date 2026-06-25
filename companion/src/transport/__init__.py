"""Serial transport for the companion app.

A thin, testable wrapper over a byte stream plus a frame extractor. The
``SerialLink`` owns the pyserial port (real hardware), while ``FrameExtractor``
is pure and reused by tests and replay. Neither imports Qt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Optional, Protocol


class ByteStream(Protocol):
    """Minimal duplex byte stream the protocol client needs."""

    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    @property
    def in_waiting(self) -> int: ...
    def close(self) -> None: ...


class FrameExtractor:
    """Reassemble 0x00-delimited frames from an arbitrary byte stream.

    Feed bytes with :meth:`push`; it yields each complete wire frame (including
    both 0x00 delimiters) as it is found. Empty inter-frame runs are skipped.
    """

    def __init__(self, max_frame: int = 4096) -> None:
        self._buf = bytearray()
        self._max_frame = max_frame
        self._discarding = False

    def push(self, data: bytes) -> Iterator[bytes]:
        for b in data:
            if b == 0x00:
                if self._discarding:
                    # Overflowed frame ended; resync starts at the next byte.
                    self._discarding = False
                    self._buf.clear()
                elif self._buf:
                    # Closing delimiter: emit the frame with both 0x00 bounds.
                    frame = b"\x00" + bytes(self._buf) + b"\x00"
                    self._buf.clear()
                    yield frame
                # else: opening delimiter or run of delimiters; ignore.
            elif self._discarding:
                continue
            else:
                self._buf.append(b)
                if len(self._buf) > self._max_frame:
                    # Runaway frame (noise / desync): drop until next delimiter.
                    self._buf.clear()
                    self._discarding = True

    def reset(self) -> None:
        self._buf.clear()
        self._discarding = False


@dataclass
class PortInfo:
    device: str
    description: str
    hwid: str


def list_serial_ports() -> list[PortInfo]:
    """Discover available serial ports. Returns [] if pyserial is missing."""
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return []
    return [
        PortInfo(p.device, p.description or "", p.hwid or "")
        for p in list_ports.comports()
    ]


class SerialLink:
    """Owns a pyserial port and exposes it as a :class:`ByteStream`."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05) -> None:
        import serial  # type: ignore  # imported lazily so tests need no hardware

        self._serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.port = port

    def read(self, size: int = 1) -> bytes:
        return self._serial.read(size)

    def write(self, data: bytes) -> int:
        n = self._serial.write(data)
        self._serial.flush()
        return n or 0

    @property
    def in_waiting(self) -> int:
        try:
            return self._serial.in_waiting
        except Exception:
            return 0

    def reset_input(self) -> None:
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        return bool(getattr(self._serial, "is_open", False))


def open_serial(port: str, baud: int = 115200) -> Optional[SerialLink]:
    """Open ``port`` or return ``None`` if it cannot be opened."""
    try:
        return SerialLink(port, baud=baud)
    except Exception:
        return None
