"""Frame header packing and frame encode/decode.

Wire format (AGENTS.md 6.1):

    0x00  COBS( header || payload || crc16 )  0x00

14-byte header, little-endian:
    magic(1) ver_major(1) ver_minor(1) msg_type(1) msg_id(1) flags(1)
    seq(2) timestamp_ms(4) payload_len(2)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from .cobs import cobs_decode, cobs_encode
from .crc16 import crc16

MAGIC = 0xA5
VERSION_MAJOR = 0
VERSION_MINOR = 1

HEADER_LEN = 14
CRC_LEN = 2
MAX_PAYLOAD = 256

_HEADER_STRUCT = struct.Struct("<6BHIH")  # 6 bytes, u16 seq, u32 ts, u16 len


class MsgType(IntEnum):
    COMMAND = 0
    RESPONSE = 1
    TELEMETRY = 2
    EVENT = 3


class DecodeError(Exception):
    """Raised when a frame body fails to decode (bad COBS/magic/CRC/length)."""


@dataclass
class Header:
    msg_type: int = 0
    msg_id: int = 0
    flags: int = 0
    seq: int = 0
    timestamp_ms: int = 0
    payload_len: int = 0
    magic: int = MAGIC
    version_major: int = VERSION_MAJOR
    version_minor: int = VERSION_MINOR

    def pack(self) -> bytes:
        return _HEADER_STRUCT.pack(
            self.magic,
            self.version_major,
            self.version_minor,
            self.msg_type,
            self.msg_id,
            self.flags,
            self.seq,
            self.timestamp_ms,
            self.payload_len,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "Header":
        (
            magic,
            ver_major,
            ver_minor,
            msg_type,
            msg_id,
            flags,
            seq,
            ts,
            payload_len,
        ) = _HEADER_STRUCT.unpack(data[:HEADER_LEN])
        return cls(
            msg_type=msg_type,
            msg_id=msg_id,
            flags=flags,
            seq=seq,
            timestamp_ms=ts,
            payload_len=payload_len,
            magic=magic,
            version_major=ver_major,
            version_minor=ver_minor,
        )


def encode_frame(header: Header, payload: bytes = b"") -> bytes:
    """Encode a full on-wire frame (including both 0x00 delimiters)."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds MAX_PAYLOAD")
    header.payload_len = len(payload)
    body = header.pack() + payload
    body += struct.pack("<H", crc16(body))
    return b"\x00" + cobs_encode(body) + b"\x00"


def decode_frame_body(body_cobs: bytes) -> tuple[Header, bytes]:
    """Decode a COBS-encoded frame body (delimiters excluded).

    Returns ``(header, payload)``. Raises ``DecodeError`` on any corruption.
    """
    try:
        body = cobs_decode(body_cobs)
    except ValueError as exc:
        raise DecodeError(f"bad COBS: {exc}") from exc

    if len(body) < HEADER_LEN + CRC_LEN:
        raise DecodeError("frame too short")
    if body[0] != MAGIC:
        raise DecodeError("bad magic")

    header = Header.unpack(body)
    if header.payload_len > MAX_PAYLOAD:
        raise DecodeError("payload_len over MAX_PAYLOAD")
    if HEADER_LEN + header.payload_len + CRC_LEN != len(body):
        raise DecodeError("payload_len inconsistent with frame")

    crc_offset = HEADER_LEN + header.payload_len
    want = crc16(body[:crc_offset])
    (got,) = struct.unpack("<H", body[crc_offset : crc_offset + CRC_LEN])
    if want != got:
        raise DecodeError("CRC mismatch")

    payload = body[HEADER_LEN:crc_offset]
    return header, payload


def version_compatible(proto_major: int, proto_minor: int) -> bool:
    """Whether firmware advertising protocol v``major``.``minor`` is wire-
    compatible with this host build (``VERSION_MAJOR``.``VERSION_MINOR``).

    The major version must match exactly: a different major means an
    incompatible frame/message layout. A differing minor is additive-only and
    tolerated (newer firmware may add messages the host ignores, and vice
    versa). Note the frame decoder does NOT reject a version mismatch -- the
    version rides in the header and is surfaced at the handshake (HELLO /
    GET_CAPABILITIES), so callers use this to diagnose rather than to drop
    frames (4sa.5).
    """
    del proto_minor  # reserved for future minor-based gating; major gates today
    return proto_major == VERSION_MAJOR

