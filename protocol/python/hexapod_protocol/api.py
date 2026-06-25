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

# Telemetry / logging command group (mirrors src/protocol/telemetry.h).
MSG_SUBSCRIBE = 0x10
MSG_UNSUBSCRIBE = 0x11
MSG_SET_STREAM_RATE = 0x12
MSG_GET_STREAM_STATS = 0x13

# Config command group (mirrors src/config/config_api.h).
MSG_CFG_GET_SUMMARY = 0x20
MSG_CFG_GET_BLOCK = 0x21
MSG_CFG_SET_BLOCK = 0x22
MSG_CFG_VALIDATE = 0x23
MSG_CFG_COMMIT = 0x24
MSG_CFG_RESET_DEFAULTS = 0x25

# Safety control command group (mirrors src/protocol/control_api.h, 0x30..0x33).
MSG_ESTOP = 0x30
MSG_CLEAR_FAULT = 0x31
MSG_SET_ARMING = 0x32
MSG_SET_MODE = 0x33

# Telemetry frame msg-id base: a telemetry frame for stream s arrives with
# header msg_id = MSG_TELEMETRY_BASE + s (header msg_type == TELEMETRY).
MSG_TELEMETRY_BASE = 0x40

# Header flag bits.
FLAG_ACK_REQ = 0x01
FLAG_ERROR = 0x02
FLAG_FRAGMENT = 0x04

# Error codes.
ERR_NONE = 0
ERR_UNKNOWN_MSG = 1
ERR_BAD_REQUEST = 2

# Subscribe/unsubscribe/set-rate result byte (mirrors SubResult).
SUB_OK = 0
SUB_BAD_STREAM = 1
SUB_BAD_REQUEST = 2

# Arming request byte (SET_ARMING payload; mirrors ArmingRequest).
ARMING_DISARM = 0
ARMING_ARM = 1

# Control response result byte (mirrors CtrlResult).
CTRL_OK = 0
CTRL_REJECTED = 1
CTRL_BAD_REQUEST = 2

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


# --- Telemetry subscription commands --------------------------------------


def build_subscribe(stream_id: int, rate_hz: int, seq: int = 0) -> bytes:
    """Build a SUBSCRIBE command for ``stream_id`` at ``rate_hz``."""
    return build_command(
        MSG_SUBSCRIBE, seq=seq, payload=struct.pack("<BH", stream_id, rate_hz)
    )


def build_set_stream_rate(stream_id: int, rate_hz: int, seq: int = 0) -> bytes:
    """Build a SET_STREAM_RATE command (same wire layout as SUBSCRIBE)."""
    return build_command(
        MSG_SET_STREAM_RATE, seq=seq, payload=struct.pack("<BH", stream_id, rate_hz)
    )


def build_unsubscribe(stream_id: int, seq: int = 0) -> bytes:
    """Build an UNSUBSCRIBE command for ``stream_id``."""
    return build_command(MSG_UNSUBSCRIBE, seq=seq, payload=bytes([stream_id]))


def build_get_stream_stats(seq: int = 0) -> bytes:
    """Build a GET_STREAM_STATS command."""
    return build_command(MSG_GET_STREAM_STATS, seq=seq)


@dataclass
class SubscribeResult:
    result: int
    stream_id: int
    effective_rate_hz: int

    @property
    def ok(self) -> bool:
        return self.result == SUB_OK


def parse_subscribe_result(payload: bytes) -> SubscribeResult:
    """Decode a SUBSCRIBE / SET_STREAM_RATE response payload."""
    if len(payload) >= 4:
        result, stream_id = payload[0], payload[1]
        (rate,) = struct.unpack("<H", payload[2:4])
        return SubscribeResult(result, stream_id, rate)
    if len(payload) >= 2:
        return SubscribeResult(payload[0], payload[1], 0)
    return SubscribeResult(payload[0] if payload else SUB_BAD_REQUEST, 0, 0)


@dataclass
class StreamStat:
    stream_id: int
    enabled: bool
    rate_hz: int
    emitted: int
    dropped: int


@dataclass
class StreamStats:
    tx_backlog: int
    streams: list[StreamStat]


def parse_stream_stats(payload: bytes) -> StreamStats:
    """Decode a GET_STREAM_STATS response payload."""
    if len(payload) < 5:
        return StreamStats(tx_backlog=0, streams=[])
    count = payload[0]
    (tx_backlog,) = struct.unpack("<I", payload[1:5])
    out: list[StreamStat] = []
    off = 5
    for _ in range(count):
        if off + 12 > len(payload):
            break
        sid = payload[off]
        enabled = bool(payload[off + 1])
        (rate,) = struct.unpack("<H", payload[off + 2 : off + 4])
        (emitted,) = struct.unpack("<I", payload[off + 4 : off + 8])
        (dropped,) = struct.unpack("<I", payload[off + 8 : off + 12])
        out.append(StreamStat(sid, enabled, rate, emitted, dropped))
        off += 12
    return StreamStats(tx_backlog=tx_backlog, streams=out)


def is_error(header: Header) -> bool:
    """True if a response header has the ERROR flag set."""
    return bool(header.flags & FLAG_ERROR)


# --- Safety control commands ----------------------------------------------


def build_estop(seq: int = 0) -> bytes:
    """Build an ESTOP command (latch a host emergency stop)."""
    return build_command(MSG_ESTOP, seq=seq)


def build_clear_fault(seq: int = 0) -> bytes:
    """Build a CLEAR_FAULT command (release host E-stop + request clear)."""
    return build_command(MSG_CLEAR_FAULT, seq=seq)


def build_set_arming(arm: bool, seq: int = 0) -> bytes:
    """Build a SET_ARMING command. ``arm=False`` force-disarms (always honored);
    ``arm=True`` only releases the host disarm latch (RC still owns walking-arm).
    """
    value = ARMING_ARM if arm else ARMING_DISARM
    return build_command(MSG_SET_ARMING, seq=seq, payload=bytes([value]))


def build_set_mode(mode: int, seq: int = 0) -> bytes:
    """Build a SET_MODE command. Only safety-reducing modes (2=Disarmed,
    12=Estop) are honored by firmware; others return CTRL_REJECTED.
    """
    return build_command(MSG_SET_MODE, seq=seq, payload=bytes([mode & 0xFF]))


@dataclass
class ControlResult:
    result: int
    state: int
    fault: int

    @property
    def ok(self) -> bool:
        return self.result == CTRL_OK

    @property
    def rejected(self) -> bool:
        return self.result == CTRL_REJECTED


def parse_control_result(payload: bytes) -> ControlResult:
    """Decode an ESTOP/CLEAR_FAULT/SET_ARMING/SET_MODE response payload
    ([result, state, fault]). A 1-byte payload is a malformed-request error.
    """
    if len(payload) >= 3:
        return ControlResult(payload[0], payload[1], payload[2])
    return ControlResult(payload[0] if payload else CTRL_BAD_REQUEST, 0, 0)
