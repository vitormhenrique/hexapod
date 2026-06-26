"""Host-side reference implementation of the hexapod wire protocol.

Mirrors the firmware in ``firmware/openrb150/src/protocol`` (CRC-16/CCITT-FALSE,
COBS, and the 14-byte little-endian frame header). Shared golden vectors in
``protocol/tests/vectors/frames.json`` are validated against both sides.
"""

from .crc16 import crc16
from .cobs import cobs_encode, cobs_decode
from .framing import Header, MsgType, encode_frame, decode_frame_body, DecodeError
from . import api
from . import telemetry
from . import config

__all__ = [
    "crc16",
    "cobs_encode",
    "cobs_decode",
    "Header",
    "MsgType",
    "encode_frame",
    "decode_frame_body",
    "DecodeError",
    "api",
    "telemetry",
    "config",
]
