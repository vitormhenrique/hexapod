"""Host tests for the wire protocol, including shared golden-vector checks."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from hexapod_protocol import (
    DecodeError,
    Header,
    cobs_decode,
    cobs_encode,
    crc16,
    decode_frame_body,
    encode_frame,
)

VECTORS = json.loads(
    (Path(__file__).parent / "vectors" / "frames.json").read_text()
)


# --------------------------------------------------------------------------- #
# CRC16
# --------------------------------------------------------------------------- #
def test_crc16_check_value():
    # The canonical CRC-16/CCITT-FALSE check value for "123456789".
    assert crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize("case", VECTORS["crc16_ccitt_false"])
def test_crc16_golden(case):
    assert crc16(bytes.fromhex(case["input"])) == case["crc"]


# --------------------------------------------------------------------------- #
# COBS
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("case", VECTORS["cobs"])
def test_cobs_golden(case):
    decoded = bytes.fromhex(case["decoded"])
    encoded = bytes.fromhex(case["encoded"])
    assert cobs_encode(decoded) == encoded
    assert cobs_decode(encoded) == decoded
    assert 0 not in encoded  # encoded stream is delimiter-safe


@pytest.mark.parametrize("n", [0, 1, 2, 253, 254, 255, 256, 600])
def test_cobs_roundtrip(n):
    data = bytes((i * 7) & 0xFF for i in range(n))
    assert cobs_decode(cobs_encode(data)) == data


def test_cobs_decode_rejects_zero():
    # A 0x00 landing on a code-byte position is malformed COBS.
    with pytest.raises(ValueError):
        cobs_decode(bytes([0x01, 0x00]))


def test_cobs_decode_rejects_overrun():
    # Code points past the end of the buffer.
    with pytest.raises(ValueError):
        cobs_decode(bytes([0x05, 0x11, 0x22]))


# --------------------------------------------------------------------------- #
# Frames
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("case", VECTORS["frames"])
def test_frame_golden_encode(case):
    h = Header(
        msg_type=case["msg_type"],
        msg_id=case["msg_id"],
        flags=case["flags"],
        seq=case["seq"],
        timestamp_ms=case["timestamp_ms"],
    )
    wire = encode_frame(h, bytes.fromhex(case["payload"]))
    assert wire.hex() == case["wire"]


@pytest.mark.parametrize("case", VECTORS["frames"])
def test_frame_golden_decode(case):
    wire = bytes.fromhex(case["wire"])
    assert wire[0] == 0x00 and wire[-1] == 0x00
    header, payload = decode_frame_body(wire[1:-1])
    assert header.msg_type == case["msg_type"]
    assert header.msg_id == case["msg_id"]
    assert header.flags == case["flags"]
    assert header.seq == case["seq"]
    assert header.timestamp_ms == case["timestamp_ms"]
    assert payload == bytes.fromhex(case["payload"])


def test_frame_roundtrip():
    h = Header(msg_type=2, msg_id=42, flags=0x03, seq=7, timestamp_ms=123456)
    payload = bytes([0x10, 0x00, 0x20, 0x00, 0x30])
    wire = encode_frame(h, payload)
    header, out = decode_frame_body(wire[1:-1])
    assert out == payload
    assert header.seq == 7


# --------------------------------------------------------------------------- #
# Corruption rejection
# --------------------------------------------------------------------------- #
def test_decode_rejects_bit_flip():
    h = Header(msg_type=2, msg_id=1, seq=1, timestamp_ms=10)
    wire = bytearray(encode_frame(h, bytes([0xAA, 0xBB, 0xCC])))
    body = bytearray(wire[1:-1])
    body[-3] ^= 0x01  # flip a bit inside the encoded body
    with pytest.raises(DecodeError):
        decode_frame_body(bytes(body))


def test_decode_rejects_bad_magic():
    h = Header(msg_type=0, msg_id=0, seq=0, timestamp_ms=0)
    wire = encode_frame(h, b"")
    body = cobs_decode(wire[1:-1])
    body = bytearray(body)
    body[0] ^= 0xFF  # corrupt magic
    bad = cobs_encode(bytes(body))
    with pytest.raises(DecodeError):
        decode_frame_body(bad)


def test_decode_rejects_truncation():
    h = Header(msg_type=0, msg_id=0, seq=0, timestamp_ms=0)
    wire = encode_frame(h, bytes([0x01, 0x02, 0x03]))
    body = wire[1:-1]
    with pytest.raises(DecodeError):
        decode_frame_body(body[:5])  # chop the body short
