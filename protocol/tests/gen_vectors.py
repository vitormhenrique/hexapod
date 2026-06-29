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
from hexapod_protocol import config as config_mod  # noqa: E402

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
        "config": build_config(),
        "config_cmds": build_config_cmds(),
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
        {
            "name": "dxl_power",
            "request": api_mod.build_dxl_power(True, seq=11).hex(),
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


def build_config_cmds() -> dict:
    """Deterministic request frames for the config command group
    (CFG_GET_SUMMARY / GET_BLOCK / SET_BLOCK / VALIDATE / COMMIT /
    RESET_DEFAULTS). Responses depend on live staged state (firmware
    test_config_api covers behavior), so only request encoding is pinned.
    """
    block_data = bytes(range(8))
    cases = [
        {
            "name": "cfg_get_summary",
            "request": api_mod.build_cfg_get_summary(seq=1).hex(),
        },
        {
            "name": "cfg_get_block",
            "request": api_mod.build_cfg_get_block(16, 32, seq=2).hex(),
        },
        {
            "name": "cfg_set_block",
            "request": api_mod.build_cfg_set_block(16, block_data, seq=3).hex(),
            "offset": 16,
            "data": block_data.hex(),
        },
        {
            "name": "cfg_validate",
            "request": api_mod.build_cfg_validate(seq=4).hex(),
        },
        {
            "name": "cfg_commit",
            "request": api_mod.build_cfg_commit(seq=5).hex(),
        },
        {
            "name": "cfg_reset_defaults",
            "request": api_mod.build_cfg_reset_defaults(seq=6).hex(),
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
    # servo_goals payload (eax.2): count(1) then leg(1), joint(1),
    # angle_centideg(int16), flags(1). flags bit0 = clamped. Three goals: an
    # unclamped center, a clamped +30.00 deg, and an unclamped -45.00 deg.
    goals = [
        {"leg": 0, "joint": 0, "angle_centideg": 0, "clamped": False},
        {"leg": 1, "joint": 1, "angle_centideg": 3000, "clamped": True},
        {"leg": 2, "joint": 2, "angle_centideg": -4500, "clamped": False},
    ]
    gpayload = bytearray([len(goals)])
    for g in goals:
        gpayload += struct.pack(
            "<BBhB", g["leg"], g["joint"], g["angle_centideg"], 1 if g["clamped"] else 0
        )
    cases.append(
        {
            "name": "servo_goals",
            "stream": "servo_goals",
            "payload": _hex(bytes(gpayload)),
            "goals": goals,
        }
    )
    # servo_status payload (eax.6): count(1) then 14 bytes/servo: id, pos(u32),
    # vel(i16), load(i16), volt_mv(u16), temp(i8), err(u8), torque_enable(u8).
    # Two servos: one torque-on (loaded), one torque-off (released).
    servos = [
        {
            "id": 1,
            "position": 2048,
            "velocity": 12,
            "load": -34,
            "voltage_mv": 1200,
            "temperature_c": 31,
            "hardware_error": 0,
            "torque_enabled": True,
        },
        {
            "id": 7,
            "position": 1700,
            "velocity": -5,
            "load": 0,
            "voltage_mv": 1180,
            "temperature_c": 29,
            "hardware_error": 0,
            "torque_enabled": False,
        },
    ]
    spayload = bytearray([len(servos)])
    for s in servos:
        spayload += struct.pack(
            "<BIhhHbBB",
            s["id"],
            s["position"],
            s["velocity"],
            s["load"],
            s["voltage_mv"],
            s["temperature_c"],
            s["hardware_error"],
            1 if s["torque_enabled"] else 0,
        )
    cases.append(
        {
            "name": "servo_status",
            "stream": "servo_status",
            "payload": _hex(bytes(spayload)),
            "servos": servos,
        }
    )
    # leg_state payload (eax.3): count(1) then 8 bytes/leg: leg(1), foot_x(i16),
    # foot_y(i16), foot_z(i16, mm body frame), flags(1). flags bit0 = reachable,
    # bit1 = clamped. Two legs: a reachable/unclamped target and an unreachable
    # one (so the host can flag a foot that left the workspace).
    legs = [
        {
            "leg": 0,
            "foot_x_mm": 127,
            "foot_y_mm": 0,
            "foot_z_mm": -45,
            "reachable": True,
            "clamped": False,
        },
        {
            "leg": 3,
            "foot_x_mm": 400,
            "foot_y_mm": -20,
            "foot_z_mm": 30,
            "reachable": False,
            "clamped": True,
        },
    ]
    lpayload = bytearray([len(legs)])
    for leg in legs:
        flags = (0x01 if leg["reachable"] else 0) | (0x02 if leg["clamped"] else 0)
        lpayload += struct.pack(
            "<BhhhB",
            leg["leg"],
            leg["foot_x_mm"],
            leg["foot_y_mm"],
            leg["foot_z_mm"],
            flags,
        )
    cases.append(
        {
            "name": "leg_state",
            "stream": "leg_state",
            "payload": _hex(bytes(lpayload)),
            "legs": legs,
        }
    )
    return {"cases": cases}


def build_config() -> dict:
    """Golden vectors for the persistent RobotConfig payload + tick<->angle math.

    The serialized payload is produced firmware-side (serializeRobotConfig); the
    host decoder must be its exact inverse. We pin the compiled-default config's
    serialized bytes (and their CRC, which the firmware native test cross-checks
    against its own serializer) plus a set of tick<->angle conversions so both
    implementations agree on the servo-map arithmetic byte-for-byte.
    """
    cfg = config_mod.default_robot_config()
    payload = config_mod.encode_robot_config(cfg)

    # tick<->angle cases over a few default servos (sign +1 leg0, sign -1 leg1).
    smap = config_mod.ServoMap(cfg)
    tick_cases = []
    for leg, joint, tick in [(0, 0, 2048), (0, 1, 2389), (1, 0, 2048), (1, 2, 1024), (4, 1, 3072)]:
        s = smap.servo_for(leg, joint)
        ang = config_mod.tick_to_angle(s, tick)
        tick_cases.append(
            {
                "leg": leg,
                "joint": joint,
                "tick": tick,
                "angle_rad": ang,
                "angle_deg": ang * config_mod.RAD_TO_DEG,
            }
        )

    angle_cases = []
    for leg, joint, deg in [(0, 0, 0.0), (0, 1, 30.0), (1, 0, -45.0), (1, 2, 200.0), (4, 1, -200.0)]:
        s = smap.servo_for(leg, joint)
        cmd = config_mod.angle_to_tick(s, deg * config_mod.DEG_TO_RAD)
        angle_cases.append(
            {
                "leg": leg,
                "joint": joint,
                "angle_deg": deg,
                "tick": cmd.tick,
                "clamped_low": cmd.clamped_low,
                "clamped_high": cmd.clamped_high,
            }
        )

    return {
        "payload_size": config_mod.CONFIG_PAYLOAD_SIZE,
        "default_payload": _hex(payload),
        "default_payload_crc": crc16(payload),
        "schema_version": cfg.schema_version,
        "robot_name": cfg.robot_name,
        "feature_defaults": cfg.feature_defaults,
        "links": {
            "coxa_cmm": cfg.links.coxa_cmm,
            "femur_cmm": cfg.links.femur_cmm,
            "tibia_cmm": cfg.links.tibia_cmm,
        },
        "gait": {
            "body_height_mm": cfg.gait.body_height_mm,
            "stride_len_mm": cfg.gait.stride_len_mm,
            "step_height_mm": cfg.gait.step_height_mm,
            "duty_x255": cfg.gait.duty_x255,
            "speed_x255": cfg.gait.speed_x255,
            "gait": cfg.gait.gait,
        },
        "servos": [
            {
                "index": i,
                "id": s.id,
                "leg": s.leg,
                "joint": s.joint,
                "sign": s.sign,
                "trim_ticks": s.trim_ticks,
                "min_tick": s.min_tick,
                "max_tick": s.max_tick,
            }
            for i, s in enumerate(cfg.servos)
            if i in (0, 1, 2, 15, 17)
        ],
        "tick_to_angle": tick_cases,
        "angle_to_tick": angle_cases,
    }


def main() -> None:
    data = build()
    VECTORS_PATH.parent.mkdir(parents=True, exist_ok=True)
    VECTORS_PATH.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {VECTORS_PATH}")


if __name__ == "__main__":
    main()
