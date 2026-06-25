"""USB API v0 message builders and parsers (host side).

Mirrors ``firmware/openrb150/src/protocol/api.{h,cpp}``. Use ``build_*`` to make
a command frame and ``parse_*`` to decode a response payload.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .framing import Header, MsgType, decode_frame_body, encode_frame

# Message IDs.
MSG_HELLO = 0x01
MSG_HEARTBEAT = 0x02
MSG_GET_STATUS = 0x03
MSG_GET_CAPABILITIES = 0x04

# Header flag bits.
FLAG_ACK_REQ = 0x01
FLAG_ERROR = 0x02
FLAG_FRAGMENT = 0x04

# Error codes.
ERR_NONE = 0
ERR_UNKNOWN_MSG = 1
ERR_BAD_REQUEST = 2

DEVICE_NAME_LEN = 16


def build_command(msg_id: int, seq: int = 0, payload: bytes = b"") -> bytes:
    """Build a full on-wire Command frame for ``msg_id``."""
    header = Header(msg_type=int(MsgType.COMMAND), msg_id=msg_id, seq=seq)
    return encode_frame(header, payload)


@dataclass
class HelloInfo:
    proto_major: int
    proto_minor: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    device_name: str


@dataclass
class StatusInfo:
    uptime_ms: int
    state: int
    dxl_power: bool
    dxl_power_control: bool
    battery_mv: int
    watchdog_missed: int


@dataclass
class Capabilities:
    proto_major: int
    proto_minor: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    feature_bits: int
    device_name: str


def _name(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def parse_response(wire: bytes) -> tuple[Header, bytes]:
    """Strip delimiters and decode a response frame -> (header, payload)."""
    if not wire or wire[0] != 0x00 or wire[-1] != 0x00:
        raise ValueError("frame missing 0x00 delimiters")
    return decode_frame_body(wire[1:-1])


def parse_hello(payload: bytes) -> HelloInfo:
    proto_major, proto_minor, fw_major, fw_minor, fw_patch = payload[:5]
    name = _name(payload[5 : 5 + DEVICE_NAME_LEN])
    return HelloInfo(proto_major, proto_minor, fw_major, fw_minor, fw_patch, name)


def parse_heartbeat(payload: bytes) -> tuple[int, int]:
    (uptime_ms,) = struct.unpack("<I", payload[:4])
    state = payload[4]
    return uptime_ms, state


def parse_status(payload: bytes) -> StatusInfo:
    (uptime_ms,) = struct.unpack("<I", payload[:4])
    state = payload[4]
    flags = payload[5]
    (battery_mv,) = struct.unpack("<H", payload[6:8])
    (watchdog_missed,) = struct.unpack("<I", payload[8:12])
    return StatusInfo(
        uptime_ms=uptime_ms,
        state=state,
        dxl_power=bool(flags & 0x01),
        dxl_power_control=bool(flags & 0x02),
        battery_mv=battery_mv,
        watchdog_missed=watchdog_missed,
    )


def parse_capabilities(payload: bytes) -> Capabilities:
    proto_major, proto_minor, fw_major, fw_minor, fw_patch = payload[:5]
    (feature_bits,) = struct.unpack("<I", payload[5:9])
    name = _name(payload[9 : 9 + DEVICE_NAME_LEN])
    return Capabilities(
        proto_major,
        proto_minor,
        fw_major,
        fw_minor,
        fw_patch,
        feature_bits,
        name,
    )
