"""Generate shared golden test vectors for the wire protocol.

Run from the repo root:  python protocol/tests/gen_vectors.py
Writes protocol/tests/vectors/frames.json. These vectors are consumed by both
the Python tests and the firmware native Unity test to guarantee the two
implementations agree byte-for-byte.
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "protocol" / "python"))

from hexapod_protocol import crc16, cobs_encode, Header, encode_frame  # noqa: E402
from hexapod_protocol.framing import MsgType  # noqa: E402
from hexapod_protocol import api as api_mod  # noqa: E402

VECTORS_PATH = Path(__file__).parent / "vectors" / "frames.json"

# Fixed inputs shared by the firmware native test and the host test so both
# sides assert identical API wire bytes.
API_DEVICE = {
    "fw_major": 0,
    "fw_minor": 1,
    "fw_patch": 0,
    "feature_bits": 0,
    "device_name": "OpenRB150-Hex",
}
API_STATUS = {
    "uptime_ms": 123456,
    "state": 2,  # Disarmed
    "dxl_power": False,
    "dxl_power_control": True,
    "battery_mv": 11800,
    "watchdog_missed": 0,
}


def _name_bytes(name: str) -> bytes:
    raw = name.encode("ascii")[: api_mod.DEVICE_NAME_LEN]
    return raw + b"\x00" * (api_mod.DEVICE_NAME_LEN - len(raw))


def _api_response(msg_id: int, seq: int) -> bytes:
    """Reference implementation of the firmware response builder."""
    dev, st = API_DEVICE, API_STATUS
    flags = 0
    if msg_id == api_mod.MSG_HELLO:
        # proto_major=0, proto_minor=1 (kVersionMajor/Minor)
        payload = bytes([0, 1, dev["fw_major"], dev["fw_minor"], dev["fw_patch"]])
        payload += _name_bytes(dev["device_name"])
    elif msg_id == api_mod.MSG_HEARTBEAT:
        payload = struct.pack("<I", st["uptime_ms"]) + bytes([st["state"]])
    elif msg_id == api_mod.MSG_GET_STATUS:
        sflags = (0x01 if st["dxl_power"] else 0) | (
            0x02 if st["dxl_power_control"] else 0
        )
        payload = (
            struct.pack("<I", st["uptime_ms"])
            + bytes([st["state"], sflags])
            + struct.pack("<H", st["battery_mv"])
            + struct.pack("<I", st["watchdog_missed"])
        )
    elif msg_id == api_mod.MSG_GET_CAPABILITIES:
        payload = bytes([0, 1, dev["fw_major"], dev["fw_minor"], dev["fw_patch"]])
        payload += struct.pack("<I", dev["feature_bits"])
        payload += _name_bytes(dev["device_name"])
    else:
        raise ValueError(msg_id)

    h = Header(
        msg_type=int(MsgType.RESPONSE),
        msg_id=msg_id,
        flags=flags,
        seq=seq,
        timestamp_ms=st["uptime_ms"],
    )
    return encode_frame(h, payload)


def _hex(b: bytes) -> str:
    return b.hex()


def build() -> dict:
    # --- standalone CRC16 vectors (CRC-16/CCITT-FALSE) ---
    crc_cases = [
        {"input": _hex(b""), "crc": crc16(b"")},
        {"input": _hex(b"123456789"), "crc": crc16(b"123456789")},
        {"input": _hex(bytes([0x00])), "crc": crc16(bytes([0x00]))},
        {"input": _hex(bytes(range(16))), "crc": crc16(bytes(range(16)))},
    ]

    # --- standalone COBS vectors ---
    cobs_inputs = [
        b"",
        bytes([0x00]),
        bytes([0x00, 0x00]),
        bytes([0x11, 0x22, 0x00, 0x33]),
        bytes([0x01, 0x02, 0x03, 0x04]),
        bytes(range(1, 255)),  # 254 non-zero bytes -> forces 0xFF block
        bytes(range(1, 256)),  # 255 non-zero bytes
    ]
    cobs_cases = [
        {"decoded": _hex(d), "encoded": _hex(cobs_encode(d))} for d in cobs_inputs
    ]

    # --- full frame vectors ---
    frame_specs = [
        # (msg_type, msg_id, flags, seq, timestamp_ms, payload)
        (0, 0, 0, 0, 0, b""),
        (2, 0x10, 0x01, 0x1234, 0x00ABCDEF, bytes([0xDE, 0xAD, 0xBE, 0xEF])),
        (0, 0x05, 0x00, 0x0001, 1000, bytes([0x00, 0x00, 0x00])),
        (1, 0x7F, 0xFF, 0xFFFF, 0xFFFFFFFF, bytes(range(0, 64))),
    ]
    frame_cases = []
    for msg_type, msg_id, flags, seq, ts, payload in frame_specs:
        h = Header(
            msg_type=msg_type,
            msg_id=msg_id,
            flags=flags,
            seq=seq,
            timestamp_ms=ts,
        )
        wire = encode_frame(h, payload)
        frame_cases.append(
            {
                "msg_type": msg_type,
                "msg_id": msg_id,
                "flags": flags,
                "seq": seq,
                "timestamp_ms": ts,
                "payload": _hex(payload),
                "wire": _hex(wire),
            }
        )

    return {
        "crc16_ccitt_false": crc_cases,
        "cobs": cobs_cases,
        "frames": frame_cases,
        "api": build_api(),
        "control": build_control(),
        "motion": build_motion(),
        "feature": build_feature(),
        "sensor": build_sensor(),
        "maintenance": build_maintenance(),
        "dxl": build_dxl(),
        "passive": build_passive(),
        "telemetry": build_telemetry(),
    }

def build_control() -> dict:
    """Deterministic request frames for the safety control command group.

    Responses echo the live [result, state, fault] the firmware publishes, which
    is not a static fixture, so only the request encoding is pinned here. The
    firmware native test (test_control_api) covers the response behavior.
    """
    cases = [
        {"name": "estop", "request": api_mod.build_estop(seq=1).hex()},
        {"name": "clear_fault", "request": api_mod.build_clear_fault(seq=2).hex()},
        {
            "name": "set_arming_disarm",
            "request": api_mod.build_set_arming(False, seq=3).hex(),
        },
        {
            "name": "set_arming_arm",
            "request": api_mod.build_set_arming(True, seq=4).hex(),
        },
        {
            "name": "set_mode_disarmed",
            "request": api_mod.build_set_mode(2, seq=5).hex(),
        },
        {
            "name": "set_mode_estop",
            "request": api_mod.build_set_mode(12, seq=6).hex(),
        },
    ]
    return {"cases": cases}


def build_motion() -> dict:
    """Deterministic request frames for the motion command group.

    Responses echo the live [result, state, motion_allowed] gate which is not a
    static fixture, so only the request encoding is pinned here. The firmware
    native test (test_motion_api) covers the response/clamping behavior.
    """
    cases = [
        {
            "name": "set_gait_tripod",
            "request": api_mod.build_set_gait(api_mod.GAIT_TRIPOD, seq=1).hex(),
        },
        {
            "name": "set_gait_params",
            "request": api_mod.build_set_gait_params(40, 60, 30, 128, 160, seq=2).hex(),
        },
        {
            "name": "set_body_twist",
            "request": api_mod.build_set_body_twist(0.5, -0.25, 1.0, seq=3).hex(),
        },
        {
            "name": "set_body_pose",
            "request": api_mod.build_set_body_pose(10, -20, 15, 5, -5, 10, seq=4).hex(),
        },
        {
            "name": "stop_motion",
            "request": api_mod.build_stop_motion(seq=5).hex(),
        },
    ]
    return {"cases": cases}


def build_feature() -> dict:
    """Deterministic request frames for the feature flag command group.

    Responses reflect the live availability/reason the firmware publishes, which
    is not a static fixture, so only the request encoding is pinned here. The
    firmware native test (test_feature_api) covers the response behavior.
    """
    cases = [
        {
            "name": "feature_get",
            "request": api_mod.build_feature_get(seq=1).hex(),
        },
        {
            "name": "feature_set_enable",
            "request": api_mod.build_feature_set(
                api_mod.FEATURE_FOOT_CONTACT, True, seq=2
            ).hex(),
        },
        {
            "name": "feature_set_disable",
            "request": api_mod.build_feature_set(
                api_mod.FEATURE_SENSOR_POLLING, False, seq=3
            ).hex(),
        },
        {
            "name": "feature_get_reasons",
            "request": api_mod.build_feature_get_reasons(seq=4).hex(),
        },
        {
            "name": "feature_reset_defaults",
            "request": api_mod.build_feature_reset_defaults(seq=5).hex(),
        },
    ]
    return {"cases": cases}


def build_sensor() -> dict:
    """Deterministic request frames for the sensor / contact / leveling group
    (ubs.5.1 CONTACT_*/LEVELING_* control + ubs.5.2 I2C scan / topology /
    sensor status / rate / calibrate).

    Responses reflect the live feature availability/reason the firmware
    publishes, which is not a static fixture, so only the request encoding is
    pinned here. The firmware native test (test_sensor_api) covers the response
    behavior.
    """
    cases = [
        {
            "name": "contact_enable",
            "request": api_mod.build_contact_enable(seq=1).hex(),
        },
        {
            "name": "contact_disable",
            "request": api_mod.build_contact_disable(seq=2).hex(),
        },
        {
            "name": "contact_set_thresholds",
            "request": api_mod.build_contact_set_thresholds(
                3, 1200, 800, 1500, seq=3
            ).hex(),
        },
        {
            "name": "leveling_enable",
            "request": api_mod.build_leveling_enable(seq=4).hex(),
        },
        {
            "name": "leveling_disable",
            "request": api_mod.build_leveling_disable(seq=5).hex(),
        },
        {
            "name": "leveling_set_params",
            "request": api_mod.build_leveling_set_params(5000, 200, 64, seq=6).hex(),
        },
        {
            "name": "i2c_scan",
            "request": api_mod.build_i2c_scan(seq=7).hex(),
        },
        {
            "name": "i2c_get_topology",
            "request": api_mod.build_i2c_get_topology(seq=8).hex(),
        },
        {
            "name": "sensor_get_status",
            "request": api_mod.build_sensor_get_status(seq=9).hex(),
        },
        {
            "name": "sensor_set_rate",
            "request": api_mod.build_sensor_set_rate(50, seq=10).hex(),
        },
        {
            "name": "contact_calibrate",
            "request": api_mod.build_contact_calibrate(2, seq=11).hex(),
        },
        {
            "name": "contact_calibrate_all",
            "request": api_mod.build_contact_calibrate(
                api_mod.SENSOR_CALIBRATE_ALL, seq=12
            ).hex(),
        },
        {
            "name": "sensor_calibrate",
            "request": api_mod.build_sensor_calibrate(seq=13).hex(),
        },
    ]
    return {"cases": cases}


def build_maintenance() -> dict:
    """Deterministic request frames for the maintenance lock command group.

    Responses carry the live token/state the firmware grants, which is not a
    static fixture, so only the request encoding is pinned here. The firmware
    native test (test_maintenance_api) covers the lock/TTL behavior.
    """
    cases = [
        {
            "name": "enter_maintenance",
            "request": api_mod.build_enter_maintenance(seq=1).hex(),
        },
        {
            "name": "exit_maintenance",
            "request": api_mod.build_exit_maintenance(0x01020304, seq=2).hex(),
        },
        {
            "name": "maint_heartbeat",
            "request": api_mod.build_maint_heartbeat(0x01020304, seq=3).hex(),
        },
        {
            "name": "set_leg_target",
            "request": api_mod.build_set_leg_target(0, 120, -30, -45, seq=4).hex(),
        },
        {
            "name": "set_joint_target",
            "request": api_mod.build_set_joint_target(2, 1, 3000, seq=5).hex(),
        },
    ]
    return {"cases": cases}


def build_dxl() -> dict:
    """Deterministic request frames for the DXL maintenance command group.

    Submit responses carry a live job id and the poll result is produced by the
    Arduino executor on hardware, so only the request encoding is pinned here.
    The firmware native test (test_dxl_job_api) covers the queue + gating.
    """
    cases = [
        {
            "name": "dxl_scan",
            "request": api_mod.build_dxl_scan(1, 18, seq=1).hex(),
        },
        {
            "name": "dxl_ping",
            "request": api_mod.build_dxl_ping(5, seq=2).hex(),
        },
        {
            "name": "dxl_torque",
            "request": api_mod.build_dxl_torque(True, seq=3).hex(),
        },
        {
            "name": "dxl_get_servo_profile",
            "request": api_mod.build_dxl_get_servo_profile(12, seq=4).hex(),
        },
        {
            "name": "dxl_get_result",
            "request": api_mod.build_dxl_get_result(7, seq=5).hex(),
        },
        {
            "name": "dxl_get_param",
            "request": api_mod.build_dxl_get_param(
                3, api_mod.DXL_PARAM_CCW_ANGLE_LIMIT, seq=6
            ).hex(),
        },
        {
            "name": "dxl_set_param",
            "request": api_mod.build_dxl_set_param(
                5, api_mod.DXL_PARAM_MOVING_SPEED, 1000, seq=7
            ).hex(),
        },
        {
            "name": "dxl_set_servo_limits",
            "request": api_mod.build_dxl_set_servo_limits(7, 100, 3900, seq=8).hex(),
        },
        {
            "name": "dxl_read_register",
            "request": api_mod.build_dxl_read_register(3, 36, 2, seq=9).hex(),
        },
        {
            "name": "dxl_write_register",
            "request": api_mod.build_dxl_write_register(
                5, 6, 2, 100, is_eeprom=True, seq=10
            ).hex(),
        },
    ]
    return {"cases": cases}


def build_api() -> dict:
    cases = []
    specs = [
        ("hello", api_mod.MSG_HELLO, 1),
        ("heartbeat", api_mod.MSG_HEARTBEAT, 2),
        ("get_status", api_mod.MSG_GET_STATUS, 3),
        ("get_capabilities", api_mod.MSG_GET_CAPABILITIES, 4),
    ]
    for name, msg_id, seq in specs:
        request = api_mod.build_command(msg_id, seq=seq)
        response = _api_response(msg_id, seq)
        cases.append(
            {
                "name": name,
                "msg_id": msg_id,
                "seq": seq,
                "request": request.hex(),
                "response": response.hex(),
            }
        )

    # Unknown msg_id -> error response (flags has FLAG_ERROR).
    unknown_id = 0x7E
    err_req = api_mod.build_command(unknown_id, seq=9)
    err_h = Header(
        msg_type=int(MsgType.RESPONSE),
        msg_id=unknown_id,
        flags=api_mod.FLAG_ERROR,
        seq=9,
        timestamp_ms=API_STATUS["uptime_ms"],
    )
    err_resp = encode_frame(err_h, bytes([api_mod.ERR_UNKNOWN_MSG]))
    cases.append(
        {
            "name": "unknown",
            "msg_id": unknown_id,
            "seq": 9,
            "request": err_req.hex(),
            "response": err_resp.hex(),
        }
    )

    return {
        "device": API_DEVICE,
        "status": API_STATUS,
        "cases": cases,
    }


def build_passive() -> dict:
    """Deterministic request frames for the passive pose streaming group
    (PASSIVE_ENTER / EXIT / SET_STREAM_RATE / ZERO_REFERENCE).

    Responses echo the live [result, state] the firmware publishes, which is not
    a static fixture, so only the request encoding is pinned here. The firmware
    native test (test_passive_api) covers the response/gating behavior.
    """
    cases = [
        {
            "name": "passive_enter",
            "request": api_mod.build_passive_enter(seq=1).hex(),
        },
        {
            "name": "passive_exit",
            "request": api_mod.build_passive_exit(seq=2).hex(),
        },
        {
            "name": "passive_set_stream_rate",
            "request": api_mod.build_passive_set_stream_rate(100, seq=3).hex(),
        },
        {
            "name": "passive_zero_reference",
            "request": api_mod.build_passive_zero_reference(seq=4).hex(),
        },
    ]
    return {"cases": cases}


def build_telemetry() -> dict:
    """Deterministic telemetry frame payloads + their decoded fields.

    Telemetry flows firmware -> host, so these vectors pin the *decoded* wire
    layout (the host decoder must reproduce the expected fields from the bytes).
    Only the joint_state stream (eax.1) is pinned here; it carries already-mapped
    URDF-zero-relative joint angles in centidegrees so the host renders without
    the servo map.
    """
    # joint_state payload: count(1) then leg(1), joint(1), angle_centideg(int16).
    # Three joints: center (0.00 deg), +30.00 deg, -45.00 deg.
    joints = [
        {"leg": 0, "joint": 0, "angle_centideg": 0},
        {"leg": 1, "joint": 1, "angle_centideg": 3000},
        {"leg": 2, "joint": 2, "angle_centideg": -4500},
    ]
    payload = bytearray([len(joints)])
    for j in joints:
        payload += struct.pack("<BBh", j["leg"], j["joint"], j["angle_centideg"])
    cases = [
        {
            "name": "joint_state",
            "stream": "joint_state",
            "payload": _hex(bytes(payload)),
            "joints": joints,
        },
    ]
    return {"cases": cases}


def main() -> None:
    data = build()
    VECTORS_PATH.parent.mkdir(parents=True, exist_ok=True)
    VECTORS_PATH.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {VECTORS_PATH}")


if __name__ == "__main__":
    main()
