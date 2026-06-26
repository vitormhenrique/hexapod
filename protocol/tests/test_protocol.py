"""Host tests for the wire protocol, including shared golden-vector checks."""

from __future__ import annotations

import json
import struct
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


_FEATURE_BUILDERS = {
    "feature_get": lambda: api.build_feature_get(seq=1),
    "feature_set_enable": lambda: api.build_feature_set(
        api.FEATURE_FOOT_CONTACT, True, seq=2
    ),
    "feature_set_disable": lambda: api.build_feature_set(
        api.FEATURE_SENSOR_POLLING, False, seq=3
    ),
    "feature_get_reasons": lambda: api.build_feature_get_reasons(seq=4),
    "feature_reset_defaults": lambda: api.build_feature_reset_defaults(seq=5),
}


@pytest.mark.parametrize("case", VECTORS["feature"]["cases"])
def test_feature_request_golden(case):
    # The feature-flag builders must reproduce the golden request bytes.
    wire = _FEATURE_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_feature_list():
    # FEATURE_GET layout: [state, count, {id, available, enabled, reason} x count]
    payload = bytes([2, 2]) + bytes(
        [api.FEATURE_FOOT_CONTACT, 1, 1, api.FEATURE_REASON_NONE]
    ) + bytes(
        [api.FEATURE_TERRAIN_LEVELING, 0, 0, api.FEATURE_REASON_DEPENDENCY_OFF]
    )
    fl = api.parse_feature_list(payload)
    assert fl.state == 2 and len(fl.features) == 2
    fc = fl.get(api.FEATURE_FOOT_CONTACT)
    assert fc.available and fc.enabled and fc.reason == api.FEATURE_REASON_NONE
    tl = fl.get(api.FEATURE_TERRAIN_LEVELING)
    assert not tl.enabled and tl.reason == api.FEATURE_REASON_DEPENDENCY_OFF


def test_parse_feature_list_reset_layout():
    # FEATURE_RESET_DEFAULTS prefixes a result byte before [state, count, ...].
    payload = bytes([api.FEATURE_OK, 3, 1]) + bytes(
        [api.FEATURE_SENSOR_POLLING, 1, 1, api.FEATURE_REASON_NONE]
    )
    fl = api.parse_feature_list(payload)
    assert fl.state == 3 and len(fl.features) == 1
    sp = fl.get(api.FEATURE_SENSOR_POLLING)
    assert sp.available and sp.enabled


def test_parse_feature_reasons():
    payload = bytes([4, 2]) + bytes(
        [api.FEATURE_FOOT_CONTACT, api.FEATURE_REASON_HARDWARE_MISSING]
    ) + bytes([api.FEATURE_PASSIVE_POSE, api.FEATURE_REASON_NOT_IMPLEMENTED])
    fr = api.parse_feature_reasons(payload)
    assert fr.state == 4 and len(fr.reasons) == 2
    assert fr.reasons[0].feature == api.FEATURE_FOOT_CONTACT
    assert fr.reasons[0].reason == api.FEATURE_REASON_HARDWARE_MISSING


def test_parse_feature_set_result():
    ok = api.parse_feature_set_result(
        bytes([api.FEATURE_OK, 2, api.FEATURE_FOOT_CONTACT, 1, 1, api.FEATURE_REASON_NONE])
    )
    assert ok.ok and ok.feature == api.FEATURE_FOOT_CONTACT and ok.enabled
    rej = api.parse_feature_set_result(
        bytes(
            [
                api.FEATURE_REJECTED,
                4,
                api.FEATURE_TERRAIN_LEVELING,
                0,
                0,
                api.FEATURE_REASON_HARDWARE_MISSING,
            ]
        )
    )
    assert rej.rejected and not rej.available
    assert rej.reason == api.FEATURE_REASON_HARDWARE_MISSING
    bad = api.parse_feature_set_result(bytes([api.FEATURE_BAD_REQUEST]))
    assert bad.result == api.FEATURE_BAD_REQUEST and not bad.enabled


_SENSOR_BUILDERS = {
    "contact_enable": lambda: api.build_contact_enable(seq=1),
    "contact_disable": lambda: api.build_contact_disable(seq=2),
    "contact_set_thresholds": lambda: api.build_contact_set_thresholds(
        3, 1200, 800, 1500, seq=3
    ),
    "leveling_enable": lambda: api.build_leveling_enable(seq=4),
    "leveling_disable": lambda: api.build_leveling_disable(seq=5),
    "leveling_set_params": lambda: api.build_leveling_set_params(5000, 200, 64, seq=6),
    "i2c_scan": lambda: api.build_i2c_scan(seq=7),
    "i2c_get_topology": lambda: api.build_i2c_get_topology(seq=8),
    "sensor_get_status": lambda: api.build_sensor_get_status(seq=9),
    "sensor_set_rate": lambda: api.build_sensor_set_rate(50, seq=10),
    "contact_calibrate": lambda: api.build_contact_calibrate(2, seq=11),
    "contact_calibrate_all": lambda: api.build_contact_calibrate(
        api.SENSOR_CALIBRATE_ALL, seq=12
    ),
    "sensor_calibrate": lambda: api.build_sensor_calibrate(seq=13),
}


@pytest.mark.parametrize("case", VECTORS["sensor"]["cases"])
def test_sensor_request_golden(case):
    # The sensor/contact/leveling builders must reproduce the golden bytes.
    wire = _SENSOR_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_sensor_feature_result():
    ok = api.parse_sensor_feature_result(
        bytes([api.SENSOR_OK, 2, 1, 1, api.FEATURE_REASON_NONE])
    )
    assert ok.ok and ok.available and ok.enabled
    rej = api.parse_sensor_feature_result(
        bytes([api.SENSOR_REJECTED, 2, 0, 0, api.FEATURE_REASON_HARDWARE_MISSING])
    )
    assert rej.rejected and not rej.enabled
    assert rej.reason == api.FEATURE_REASON_HARDWARE_MISSING
    bad = api.parse_sensor_feature_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.enabled


def test_parse_contact_threshold_result():
    ok = api.parse_contact_threshold_result(
        struct.pack("<BBHHH", api.SENSOR_OK, 3, 1200, 800, 1500)
    )
    assert ok.ok and ok.foot == 3
    assert ok.near == 1200 and ok.touch == 800 and ok.load == 1500
    bad = api.parse_contact_threshold_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.ok


def test_parse_leveling_params_result():
    ok = api.parse_leveling_params_result(
        struct.pack("<BHHH", api.SENSOR_OK, 5000, 200, 64)
    )
    assert ok.ok and ok.max_tilt_mdeg == 5000
    assert ok.rate_mdeg_s == 200 and ok.response_x255 == 64
    bad = api.parse_leveling_params_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.ok


def test_parse_i2c_scan_result():
    ok = api.parse_i2c_scan_result(struct.pack("<BH", api.SENSOR_OK, 7))
    assert ok.ok and ok.scan_seq == 7
    bad = api.parse_i2c_scan_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.ok


def test_parse_i2c_topology_result():
    payload = bytearray([1, 1, api.SENSOR_NUM_CHANNELS])
    for ch in range(api.SENSOR_NUM_CHANNELS):
        if ch == 2:
            payload += bytes([1, 1, 1, 2, 1])  # present board
        else:
            payload += bytes([0, 0, 0, 0, 0])
    topo = api.parse_i2c_topology_result(bytes(payload))
    assert topo.mux_present and topo.eeprom_present
    assert len(topo.channels) == api.SENSOR_NUM_CHANNELS
    assert topo.channels[2].vcnl_present and topo.channels[2].lps_present
    assert topo.channels[2].device_count == 2 and topo.channels[2].state == 1
    assert not topo.channels[0].scanned
    short = api.parse_i2c_topology_result(b"")
    assert short.channels == []


def test_parse_sensor_status_result():
    payload = bytearray([api.SENSOR_NUM_FEET, 0x05, 1])
    for foot in range(api.SENSOR_NUM_FEET):
        if foot == 2:
            payload += struct.pack("<BBHhB", 3, 210, 1234, -50, 0x04)
        else:
            payload += struct.pack("<BBHhB", 0, 0, 0, 0, 0)
    status = api.parse_sensor_status_result(bytes(payload))
    assert status.present_mask == 0x05 and status.polling_enabled
    assert len(status.feet) == api.SENSOR_NUM_FEET
    f = status.feet[2]
    assert f.state == 3 and f.confidence == 210
    assert f.proximity == 1234 and f.pressure_delta == -50
    assert f.loaded and not f.stale
    short = api.parse_sensor_status_result(b"")
    assert short.feet == []


def test_parse_sensor_rate_result():
    ok = api.parse_sensor_rate_result(struct.pack("<BH", api.SENSOR_OK, 50))
    assert ok.ok and ok.rate_hz == 50
    bad = api.parse_sensor_rate_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.ok


def test_parse_sensor_calibrate_result():
    ok = api.parse_sensor_calibrate_result(bytes([api.SENSOR_OK, 0x3F]))
    assert ok.ok and ok.mask == 0x3F
    rej = api.parse_sensor_calibrate_result(bytes([api.SENSOR_REJECTED, 0x3F]))
    assert not rej.ok and rej.mask == 0x3F
    bad = api.parse_sensor_calibrate_result(bytes([api.SENSOR_BAD_REQUEST]))
    assert bad.result == api.SENSOR_BAD_REQUEST and not bad.ok


_MAINT_BUILDERS = {
    "enter_maintenance": lambda: api.build_enter_maintenance(seq=1),
    "exit_maintenance": lambda: api.build_exit_maintenance(0x01020304, seq=2),
    "maint_heartbeat": lambda: api.build_maint_heartbeat(0x01020304, seq=3),
    "set_leg_target": lambda: api.build_set_leg_target(0, 120, -30, -45, seq=4),
    "set_joint_target": lambda: api.build_set_joint_target(2, 1, 3000, seq=5),
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


def test_parse_leg_target_result():
    ok = api.parse_leg_target_result(
        bytes([api.MAINT_TARGET_OK, 8, 1, 0, 0]) + struct.pack("<HHH", 2048, 2100, 1990)
    )
    assert ok.ok and ok.reachable and ok.ticks == (2048, 2100, 1990)
    unreach = api.parse_leg_target_result(
        bytes([api.MAINT_TARGET_UNREACHABLE, 8, 0, 0, 0])
        + struct.pack("<HHH", 1024, 1024, 3072)
    )
    assert not unreach.reachable and unreach.result == api.MAINT_TARGET_UNREACHABLE
    rej = api.parse_leg_target_result(bytes([api.MAINT_TARGET_REJECTED, 2]))
    assert rej.result == api.MAINT_TARGET_REJECTED and rej.ticks == (0, 0, 0)


def test_parse_joint_target_result():
    ok = api.parse_joint_target_result(
        bytes([api.MAINT_TARGET_OK, 8, 0, 0]) + struct.pack("<H", 2389)
    )
    assert ok.ok and ok.tick == 2389 and not ok.clamped_high
    clamp = api.parse_joint_target_result(
        bytes([api.MAINT_TARGET_OK, 8, 0, 1]) + struct.pack("<H", 3072)
    )
    assert clamp.clamped_high and clamp.tick == 3072
    rej = api.parse_joint_target_result(bytes([api.MAINT_TARGET_REJECTED, 2]))
    assert rej.result == api.MAINT_TARGET_REJECTED and rej.tick == 0


_DXL_BUILDERS = {
    "dxl_scan": lambda: api.build_dxl_scan(1, 18, seq=1),
    "dxl_ping": lambda: api.build_dxl_ping(5, seq=2),
    "dxl_torque": lambda: api.build_dxl_torque(True, seq=3),
    "dxl_get_servo_profile": lambda: api.build_dxl_get_servo_profile(12, seq=4),
    "dxl_get_result": lambda: api.build_dxl_get_result(7, seq=5),
    "dxl_get_param": lambda: api.build_dxl_get_param(
        3, api.DXL_PARAM_CCW_ANGLE_LIMIT, seq=6
    ),
    "dxl_set_param": lambda: api.build_dxl_set_param(
        5, api.DXL_PARAM_MOVING_SPEED, 1000, seq=7
    ),
    "dxl_set_servo_limits": lambda: api.build_dxl_set_servo_limits(7, 100, 3900, seq=8),
    "dxl_read_register": lambda: api.build_dxl_read_register(3, 36, 2, seq=9),
    "dxl_write_register": lambda: api.build_dxl_write_register(
        5, 6, 2, 100, is_eeprom=True, seq=10
    ),
}


@pytest.mark.parametrize("case", VECTORS["dxl"]["cases"])
def test_dxl_request_golden(case):
    # The DXL maintenance builders must reproduce the golden request bytes.
    wire = _DXL_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_dxl_submit():
    acc = api.parse_dxl_submit(
        bytes([api.DXL_SUBMIT_ACCEPTED, 7, api.DXL_SLOT_PENDING])
    )
    assert acc.accepted and acc.job_id == 7 and acc.slot == api.DXL_SLOT_PENDING
    busy = api.parse_dxl_submit(bytes([api.DXL_SUBMIT_BUSY, 0, api.DXL_SLOT_RUNNING]))
    assert not busy.accepted and busy.result == api.DXL_SUBMIT_BUSY
    rej = api.parse_dxl_submit(bytes([api.DXL_SUBMIT_REJECTED]))
    assert rej.result == api.DXL_SUBMIT_REJECTED and rej.job_id == 0


def test_parse_dxl_result_pending_and_scan():
    pend = api.parse_dxl_result(bytes([api.DXL_SLOT_PENDING, api.DXL_CODE_OK, 0]))
    assert not pend.done and pend.slot == api.DXL_SLOT_PENDING and pend.servos() == []

    # A DONE scan result: count=2 then two 6-byte servo records.
    data = bytes([2, 1, 29, 0, 40, 1, 1, 2, 30, 0, 42, 2, 2])
    done = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    assert done.done and done.code == api.DXL_CODE_OK
    servos = done.servos()
    assert len(servos) == 2
    assert servos[0] == api.DxlServoRecord(1, 29, 40, 1, 1)
    assert servos[1] == api.DxlServoRecord(2, 30, 42, 2, 2)


def test_parse_dxl_result_not_found():
    nf = api.parse_dxl_result(bytes([api.DXL_SLOT_DONE, api.DXL_CODE_NOT_FOUND, 0]))
    assert nf.done and nf.code == api.DXL_CODE_NOT_FOUND and nf.servos() == []


def test_parse_dxl_get_param_result():
    # [param, table, len, value(i32)] -> CcwAngleLimit=4, legacy table, 2 bytes.
    data = struct.pack("<BBBi", api.DXL_PARAM_CCW_ANGLE_LIMIT, 1, 2, 3000)
    res = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    pv = res.param()
    assert pv is not None
    assert pv.param == api.DXL_PARAM_CCW_ANGLE_LIMIT
    assert pv.table_kind == 1 and pv.length == 2 and pv.value == 3000
    # A signed value (e.g. HomingOffset) round-trips through the i32 decode.
    sdata = struct.pack("<BBBi", api.DXL_PARAM_HOMING_OFFSET, 2, 4, -128)
    sres = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(sdata)]) + sdata
    )
    assert sres.param().value == -128
    # Pending result yields no param decode.
    assert (
        api.parse_dxl_result(bytes([api.DXL_SLOT_PENDING, api.DXL_CODE_OK, 0])).param()
        is None
    )


def test_parse_dxl_set_param_result():
    # [param, len, written(i32), readback(i32), verified].
    data = struct.pack("<BBiiB", api.DXL_PARAM_MOVING_SPEED, 2, 1000, 1000, 1)
    res = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    sp = res.set_param()
    assert sp is not None
    assert sp.param == api.DXL_PARAM_MOVING_SPEED and sp.length == 2
    assert sp.written == 1000 and sp.readback == 1000 and sp.verified
    # A verify mismatch is surfaced.
    bad = struct.pack("<BBiiB", api.DXL_PARAM_MOVING_SPEED, 2, 1000, 999, 0)
    badres = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_VERIFY_FAILED, len(bad)]) + bad
    )
    assert badres.code == api.DXL_CODE_VERIFY_FAILED
    assert not badres.set_param().verified


def test_parse_dxl_servo_limits_result():
    # [table, min(i32), max(i32), verified].
    data = struct.pack("<BiiB", 2, 100, 3900, 1)
    res = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    lim = res.servo_limits()
    assert lim is not None
    assert lim.table_kind == 2 and lim.min_tick == 100
    assert lim.max_tick == 3900 and lim.verified


def test_parse_dxl_read_register_result():
    # [addr(u16), len, value(i32)].
    data = struct.pack("<HBi", 36, 2, 2048)
    res = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    rv = res.read_register()
    assert rv is not None
    assert rv.address == 36 and rv.length == 2 and rv.value == 2048
    # Pending result yields no decode.
    assert api.parse_dxl_result(
        bytes([api.DXL_SLOT_PENDING, api.DXL_CODE_OK, 0])
    ).read_register() is None


def test_parse_dxl_write_register_result():
    # [addr(u16), len, written(i32), readback(i32), verified].
    data = struct.pack("<HBiiB", 6, 2, 100, 100, 1)
    res = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_OK, len(data)]) + data
    )
    wr = res.write_register()
    assert wr is not None
    assert wr.address == 6 and wr.length == 2
    assert wr.written == 100 and wr.readback == 100 and wr.verified
    # A verify mismatch surfaces.
    bad = struct.pack("<HBiiB", 6, 2, 100, 99, 0)
    badres = api.parse_dxl_result(
        bytes([api.DXL_SLOT_DONE, api.DXL_CODE_VERIFY_FAILED, len(bad)]) + bad
    )
    assert badres.code == api.DXL_CODE_VERIFY_FAILED
    assert not badres.write_register().verified


_PASSIVE_BUILDERS = {
    "passive_enter": lambda: api.build_passive_enter(seq=1),
    "passive_exit": lambda: api.build_passive_exit(seq=2),
    "passive_set_stream_rate": lambda: api.build_passive_set_stream_rate(100, seq=3),
    "passive_zero_reference": lambda: api.build_passive_zero_reference(seq=4),
}


@pytest.mark.parametrize("case", VECTORS["passive"]["cases"])
def test_passive_request_golden(case):
    # The passive pose builders must reproduce the golden request bytes.
    wire = _PASSIVE_BUILDERS[case["name"]]()
    assert wire.hex() == case["request"]


def test_parse_passive_result():
    ok = api.parse_passive_result(bytes([api.PASSIVE_OK, 9]))
    assert ok.ok and ok.state == 9
    rej = api.parse_passive_result(bytes([api.PASSIVE_REJECTED, 4]))
    assert rej.rejected and rej.state == 4
    bad = api.parse_passive_result(bytes([api.PASSIVE_BAD_REQUEST]))
    assert bad.result == api.PASSIVE_BAD_REQUEST and not bad.ok


def test_parse_passive_rate_result():
    ok = api.parse_passive_rate_result(struct.pack("<BBH", api.PASSIVE_OK, 9, 100))
    assert ok.ok and ok.state == 9 and ok.rate_hz == 100
    bad = api.parse_passive_rate_result(bytes([api.PASSIVE_BAD_REQUEST]))
    assert bad.result == api.PASSIVE_BAD_REQUEST and not bad.ok

