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
from hexapod_protocol import api

VECTORS = json.loads((Path(__file__).parent / "vectors" / "frames.json").read_text())


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


# --------------------------------------------------------------------------- #
# USB API v0
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("case", VECTORS["api"]["cases"])
def test_api_request_golden(case):
    # build_command must reproduce the golden request bytes.
    wire = api.build_command(case["msg_id"], seq=case["seq"])
    assert wire.hex() == case["request"]


def test_api_parse_hello():
    case = next(c for c in VECTORS["api"]["cases"] if c["name"] == "hello")
    header, payload = api.parse_response(bytes.fromhex(case["response"]))
    info = api.parse_hello(payload)
    dev = VECTORS["api"]["device"]
    assert header.seq == case["seq"]
    assert info.fw_major == dev["fw_major"]
    assert info.fw_minor == dev["fw_minor"]
    assert info.fw_patch == dev["fw_patch"]
    assert info.device_name == dev["device_name"]


def test_api_parse_status():
    case = next(c for c in VECTORS["api"]["cases"] if c["name"] == "get_status")
    header, payload = api.parse_response(bytes.fromhex(case["response"]))
    st = api.parse_status(payload)
    want = VECTORS["api"]["status"]
    assert st.uptime_ms == want["uptime_ms"]
    assert st.state == want["state"]
    assert st.dxl_power == want["dxl_power"]
    assert st.dxl_power_control == want["dxl_power_control"]
    assert st.battery_mv == want["battery_mv"]
    assert st.watchdog_missed == want["watchdog_missed"]


def test_api_parse_capabilities():
    case = next(c for c in VECTORS["api"]["cases"] if c["name"] == "get_capabilities")
    header, payload = api.parse_response(bytes.fromhex(case["response"]))
    caps = api.parse_capabilities(payload)
    dev = VECTORS["api"]["device"]
    assert caps.feature_bits == dev["feature_bits"]
    assert caps.device_name == dev["device_name"]


def test_api_unknown_is_error():
    case = next(c for c in VECTORS["api"]["cases"] if c["name"] == "unknown")
    header, payload = api.parse_response(bytes.fromhex(case["response"]))
    assert header.flags & api.FLAG_ERROR
    assert payload == bytes([api.ERR_UNKNOWN_MSG])


_CONTROL_BUILDERS = {
    "estop": lambda: api.build_estop(seq=1),
    "clear_fault": lambda: api.build_clear_fault(seq=2),
    "set_arming_disarm": lambda: api.build_set_arming(False, seq=3),
    "set_arming_arm": lambda: api.build_set_arming(True, seq=4),
    "set_mode_disarmed": lambda: api.build_set_mode(2, seq=5),
    "set_mode_estop": lambda: api.build_set_mode(12, seq=6),
}


@pytest.mark.parametrize("case", VECTORS["control"]["cases"])
def test_control_request_golden(case):
    # The safety-control builders must reproduce the golden request bytes.
    wire = _CONTROL_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_control_result():
    cr = api.parse_control_result(bytes([api.CTRL_OK, 12, 2]))
    assert cr.ok and cr.state == 12 and cr.fault == 2
    rej = api.parse_control_result(bytes([api.CTRL_REJECTED, 4, 0]))
    assert rej.rejected and not rej.ok
    bad = api.parse_control_result(bytes([api.CTRL_BAD_REQUEST]))
    assert bad.result == api.CTRL_BAD_REQUEST and bad.state == 0


_MOTION_BUILDERS = {
    "set_gait_tripod": lambda: api.build_set_gait(api.GAIT_TRIPOD, seq=1),
    "set_gait_params": lambda: api.build_set_gait_params(40, 60, 30, 128, 160, seq=2),
    "set_body_twist": lambda: api.build_set_body_twist(0.5, -0.25, 1.0, seq=3),
    "set_body_pose": lambda: api.build_set_body_pose(10, -20, 15, 5, -5, 10, seq=4),
    "stop_motion": lambda: api.build_stop_motion(seq=5),
}


@pytest.mark.parametrize("case", VECTORS["motion"]["cases"])
def test_motion_request_golden(case):
    # The motion builders must reproduce the golden request bytes.
    wire = _MOTION_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_motion_result():
    mr = api.parse_motion_result(bytes([api.MOTION_OK, 5, 1]))
    assert mr.ok and mr.state == 5 and mr.motion_allowed
    rej = api.parse_motion_result(bytes([api.MOTION_REJECTED, 5, 0]))
    assert rej.rejected and not rej.motion_allowed
    bad = api.parse_motion_result(bytes([api.MOTION_BAD_REQUEST]))
    assert bad.result == api.MOTION_BAD_REQUEST and not bad.motion_allowed


_MAINT_BUILDERS = {
    "enter_maintenance": lambda: api.build_enter_maintenance(seq=1),
    "exit_maintenance": lambda: api.build_exit_maintenance(0x01020304, seq=2),
    "maint_heartbeat": lambda: api.build_maint_heartbeat(0x01020304, seq=3),
}


@pytest.mark.parametrize("case", VECTORS["maintenance"]["cases"])
def test_maintenance_request_golden(case):
    # The maintenance-lock builders must reproduce the golden request bytes.
    wire = _MAINT_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_maint_result():
    ent = api.parse_maint_result(bytes([api.MAINT_OK, 2, 0x04, 0x03, 0x02, 0x01]))
    assert ent.ok and ent.state == 2 and ent.token == 0x01020304
    busy = api.parse_maint_result(bytes([api.MAINT_BUSY, 2]))
    assert busy.busy and busy.token == 0
    bad = api.parse_maint_result(bytes([api.MAINT_BAD_TOKEN, 8]))
    assert bad.bad_token
    err = api.parse_maint_result(bytes([api.MAINT_BAD_REQUEST]))
    assert err.result == api.MAINT_BAD_REQUEST and err.state == 0
