"""Telemetry stream decoders (host side).

Mirrors the per-stream payload layout produced by ``buildTelemetry()`` in
``firmware/openrb150/src/app/tasks.cpp``. Each ``decode_*`` takes the telemetry
frame payload (header stripped) and returns a typed dataclass. UI-independent so
the CLI, tests, and the PySide6 app all share one decoder.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum

NUM_FEET = 6
NUM_CHANNELS = 16
NUM_STREAMS = 9


class StreamId(IntEnum):
    HEALTH = 0
    CONTROL_STATE = 1
    SERVO_STATUS = 2
    CONTACT_STATE = 3
    I2C_SENSORS_RAW = 4
    RC_INPUT = 5
    API_STATS = 6
    JOINT_STATE = 7
    SERVO_GOALS = 8


STREAM_NAMES = {
    StreamId.HEALTH: "health",
    StreamId.CONTROL_STATE: "control_state",
    StreamId.SERVO_STATUS: "servo_status",
    StreamId.CONTACT_STATE: "contact_state",
    StreamId.I2C_SENSORS_RAW: "i2c_sensors_raw",
    StreamId.RC_INPUT: "rc_input",
    StreamId.API_STATS: "api_stats",
    StreamId.JOINT_STATE: "joint_state",
    StreamId.SERVO_GOALS: "servo_goals",
}

_NAME_TO_ID = {name: sid for sid, name in STREAM_NAMES.items()}


def stream_id_from_name(name: str) -> StreamId:
    """Map a stream name (e.g. "servo_status") to its StreamId."""
    try:
        return _NAME_TO_ID[name.strip().lower()]
    except KeyError as exc:
        raise ValueError(f"unknown stream '{name}'") from exc


# Safety state IDs (mirror src/safety/system_state.h).
SAFETY_STATE_NAMES = {
    0: "BOOT",
    1: "CONFIG_LOAD",
    2: "DISARMED",
    3: "ARMING_CHECKS",
    4: "STAND_READY",
    5: "RC_MANUAL",
    6: "CONTACT_TERRAIN",
    7: "JETSON_ASSISTED",
    8: "MAC_MAINTENANCE",
    9: "PASSIVE_POSE_STREAM",
    10: "FAULT_SOFT",
    11: "FAULT_HARD",
    12: "ESTOP",
}

FAULT_REASON_NAMES = {
    0: "NONE",
    1: "RC_KILL",
    2: "HOST_ESTOP",
    3: "RC_LINK_LOST",
    4: "BATTERY_LOW",
    5: "WATCHDOG",
    6: "DXL_HARDWARE",
}

# Command source IDs (mirror safety::CommandSource).
COMMAND_SOURCE_NAMES = {0: "NONE", 1: "RC", 2: "JETSON", 3: "MAC_MAINTENANCE"}

# Per-foot contact state IDs (mirror sensors::ContactState).
CONTACT_STATE_NAMES = {
    0: "AIR",
    1: "NEAR",
    2: "TOUCH",
    3: "LOADED",
    4: "RELEASE",
    5: "STALE",
    6: "FAULT",
}


@dataclass
class HealthTelemetry:
    uptime_ms: int
    state: int
    fault_reason: int
    watchdog_missed: int
    battery_mv: int

    @property
    def state_name(self) -> str:
        return SAFETY_STATE_NAMES.get(self.state, f"0x{self.state:02X}")

    @property
    def fault_name(self) -> str:
        return FAULT_REASON_NAMES.get(self.fault_reason, f"0x{self.fault_reason:02X}")


@dataclass
class ControlStateTelemetry:
    command_source: int
    motion_authorized: bool
    kill_active: bool
    state: int
    fault_reason: int
    motion_gate: bool

    @property
    def source_name(self) -> str:
        return COMMAND_SOURCE_NAMES.get(self.command_source, f"0x{self.command_source:02X}")

    @property
    def state_name(self) -> str:
        return SAFETY_STATE_NAMES.get(self.state, f"0x{self.state:02X}")


@dataclass
class ServoStatus:
    id: int
    position: int
    velocity: int
    load: int
    voltage_mv: int
    temperature_c: int
    hardware_error: int


@dataclass
class ServoStatusTelemetry:
    servos: list[ServoStatus] = field(default_factory=list)


@dataclass
class FootContact:
    state: int
    confidence: int
    pressure_delta: int

    @property
    def state_name(self) -> str:
        return CONTACT_STATE_NAMES.get(self.state, f"0x{self.state:02X}")


@dataclass
class ContactStateTelemetry:
    feet: list[FootContact] = field(default_factory=list)


@dataclass
class FootRaw:
    proximity: int
    pressure_raw: int


@dataclass
class I2cSensorsRawTelemetry:
    feet: list[FootRaw] = field(default_factory=list)


@dataclass
class RcInputTelemetry:
    armed: bool
    kill: bool
    failsafe: bool
    autonomy: bool
    gait_index: int
    channels_us: list[int] = field(default_factory=list)


@dataclass
class ApiStatsTelemetry:
    tx_backlog: int
    dropped_per_stream: list[int] = field(default_factory=list)


# Joint roles (mirror config::JointRole: coxa/femur/tibia).
JOINT_ROLE_NAMES = {0: "coxa", 1: "femur", 2: "tibia"}


@dataclass
class JointAngle:
    leg: int
    joint: int
    angle_centideg: int

    @property
    def angle_deg(self) -> float:
        return self.angle_centideg / 100.0

    @property
    def joint_name(self) -> str:
        return JOINT_ROLE_NAMES.get(self.joint, f"0x{self.joint:02X}")


@dataclass
class JointStateTelemetry:
    joints: list[JointAngle] = field(default_factory=list)


@dataclass
class ServoGoal:
    """One commanded joint goal (after IK + servo-map clamping)."""

    leg: int
    joint: int
    angle_centideg: int
    clamped: bool

    @property
    def angle_deg(self) -> float:
        return self.angle_centideg / 100.0

    @property
    def joint_name(self) -> str:
        return JOINT_ROLE_NAMES.get(self.joint, f"0x{self.joint:02X}")


@dataclass
class ServoGoalsTelemetry:
    goals: list[ServoGoal] = field(default_factory=list)


def _u16(buf: bytes, off: int) -> int:
    return struct.unpack_from("<H", buf, off)[0]


def _u32(buf: bytes, off: int) -> int:
    return struct.unpack_from("<I", buf, off)[0]


def _i32(buf: bytes, off: int) -> int:
    return struct.unpack_from("<i", buf, off)[0]


def decode_health(p: bytes) -> HealthTelemetry:
    uptime = _u32(p, 0)
    state = p[4]
    fault = p[5]
    wd = _u32(p, 6)
    batt = _u16(p, 10)
    return HealthTelemetry(uptime, state, fault, wd, batt)


def decode_control_state(p: bytes) -> ControlStateTelemetry:
    return ControlStateTelemetry(
        command_source=p[0],
        motion_authorized=bool(p[1]),
        kill_active=bool(p[2]),
        state=p[3] if len(p) > 3 else 0,
        fault_reason=p[4] if len(p) > 4 else 0,
        motion_gate=bool(p[5]) if len(p) > 5 else False,
    )


def decode_servo_status(p: bytes) -> ServoStatusTelemetry:
    if not p:
        return ServoStatusTelemetry()
    count = p[0]
    servos: list[ServoStatus] = []
    off = 1
    for _ in range(count):
        if off + 13 > len(p):
            break
        sid = p[off]
        pos = _u32(p, off + 1)
        # velocity/load are signed-ish raw values; surface as signed 16-bit.
        vel = struct.unpack_from("<h", p, off + 5)[0]
        load = struct.unpack_from("<h", p, off + 7)[0]
        volt = _u16(p, off + 9)
        temp = struct.unpack_from("<b", p, off + 11)[0]
        err = p[off + 12]
        servos.append(ServoStatus(sid, pos, vel, load, volt, temp, err))
        off += 13
    return ServoStatusTelemetry(servos)


def decode_contact_state(p: bytes) -> ContactStateTelemetry:
    feet: list[FootContact] = []
    off = 0
    for _ in range(NUM_FEET):
        if off + 4 > len(p):
            break
        state = p[off]
        conf = p[off + 1]
        delta = struct.unpack_from("<h", p, off + 2)[0]
        feet.append(FootContact(state, conf, delta))
        off += 4
    return ContactStateTelemetry(feet)


def decode_i2c_sensors_raw(p: bytes) -> I2cSensorsRawTelemetry:
    feet: list[FootRaw] = []
    off = 0
    for _ in range(NUM_FEET):
        if off + 6 > len(p):
            break
        prox = _u16(p, off)
        pressure = _i32(p, off + 2)
        feet.append(FootRaw(prox, pressure))
        off += 6
    return I2cSensorsRawTelemetry(feet)


def decode_rc_input(p: bytes) -> RcInputTelemetry:
    flags = p[0]
    gait = p[1]
    channels: list[int] = []
    off = 2
    for _ in range(NUM_CHANNELS):
        if off + 2 > len(p):
            break
        channels.append(_u16(p, off))
        off += 2
    return RcInputTelemetry(
        armed=bool(flags & 0x01),
        kill=bool(flags & 0x02),
        failsafe=bool(flags & 0x04),
        autonomy=bool(flags & 0x08),
        gait_index=gait,
        channels_us=channels,
    )


def decode_api_stats(p: bytes) -> ApiStatsTelemetry:
    tx = _u32(p, 0)
    dropped: list[int] = []
    off = 4
    for _ in range(NUM_STREAMS):
        if off + 4 > len(p):
            break
        dropped.append(_u32(p, off))
        off += 4
    return ApiStatsTelemetry(tx, dropped)


def decode_joint_state(p: bytes) -> JointStateTelemetry:
    """Mapped per-joint present angles: count(1) then leg(1), joint(1),
    angle_centideg(int16) per joint. Angles are already URDF-zero-relative
    (the firmware applied the servo map), so no config is needed to render."""
    if not p:
        return JointStateTelemetry()
    count = p[0]
    joints: list[JointAngle] = []
    off = 1
    for _ in range(count):
        if off + 4 > len(p):
            break
        leg = p[off]
        joint = p[off + 1]
        angle = struct.unpack_from("<h", p, off + 2)[0]
        joints.append(JointAngle(leg, joint, angle))
        off += 4
    return JointStateTelemetry(joints)


def decode_servo_goals(p: bytes) -> ServoGoalsTelemetry:
    """Per-joint commanded goals (eax.2): count(1) then leg(1), joint(1),
    angle_centideg(int16), flags(1) per joint. flags bit0 = clamped (the goal
    was saturated against the configured servo travel). Angles are already
    URDF-zero-relative, so they overlay directly on the joint_state pose."""
    if not p:
        return ServoGoalsTelemetry()
    count = p[0]
    goals: list[ServoGoal] = []
    off = 1
    for _ in range(count):
        if off + 5 > len(p):
            break
        leg = p[off]
        joint = p[off + 1]
        angle = struct.unpack_from("<h", p, off + 2)[0]
        flags = p[off + 4]
        goals.append(ServoGoal(leg, joint, angle, bool(flags & 0x01)))
        off += 5
    return ServoGoalsTelemetry(goals)


_DECODERS = {
    StreamId.HEALTH: decode_health,
    StreamId.CONTROL_STATE: decode_control_state,
    StreamId.SERVO_STATUS: decode_servo_status,
    StreamId.CONTACT_STATE: decode_contact_state,
    StreamId.I2C_SENSORS_RAW: decode_i2c_sensors_raw,
    StreamId.RC_INPUT: decode_rc_input,
    StreamId.API_STATS: decode_api_stats,
    StreamId.JOINT_STATE: decode_joint_state,
    StreamId.SERVO_GOALS: decode_servo_goals,
}


def decode_stream(stream_id: int, payload: bytes):
    """Decode a telemetry payload for ``stream_id`` into its typed record.

    Returns ``None`` for an unknown stream id. Decoders are defensive: a short
    payload yields a partial record rather than raising.
    """
    try:
        decoder = _DECODERS[StreamId(stream_id)]
    except (ValueError, KeyError):
        return None
    return decoder(payload)
