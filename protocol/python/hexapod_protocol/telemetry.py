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
NUM_STREAMS = 12


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
    LEG_STATE = 9
    CONTROLLER_STATE = 10
    RC_DIAGNOSTICS = 11


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
    StreamId.LEG_STATE: "leg_state",
    StreamId.CONTROLLER_STATE: "controller_state",
    StreamId.RC_DIAGNOSTICS: "rc_diagnostics",
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
        return COMMAND_SOURCE_NAMES.get(
            self.command_source, f"0x{self.command_source:02X}"
        )

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
    torque_enabled: bool = False


@dataclass
class ServoStatusTelemetry:
    servos: list[ServoStatus] = field(default_factory=list)


# DYNAMIXEL MX(2.0) Hardware Error Status bits (control-table addr 70).
HW_ERROR_BITS = {
    0: "input voltage",
    2: "overheating",
    3: "motor encoder",
    4: "electrical shock",
    5: "overload",
}


def decode_hw_error(err: int) -> list[str]:
    """Return the names of the set hardware-error bits, low bit first."""
    return [name for bit, name in sorted(HW_ERROR_BITS.items()) if err & (1 << bit)]


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
class RcLinkStats:
    """Decoded CRSF LINK_STATISTICS (0x14) signal quality (a8n).

    RSSI fields are the CRSF convention of the positive magnitude of a negative
    dBm (e.g. ``up_rssi_ant1 == 70`` means -70 dBm); use the ``*_dbm`` helpers
    for the signed value. ``up_tx_power`` is the CRSF power-table index, not mW.
    """

    up_rssi_ant1: int = 0
    up_rssi_ant2: int = 0
    up_link_quality: int = 0
    up_snr: int = 0
    active_antenna: int = 0
    rf_mode: int = 0
    up_tx_power: int = 0
    down_rssi: int = 0
    down_link_quality: int = 0
    down_snr: int = 0

    @property
    def up_rssi_dbm(self) -> int:
        """Uplink RSSI of the active antenna as signed dBm."""
        raw = self.up_rssi_ant2 if self.active_antenna else self.up_rssi_ant1
        return -raw

    @property
    def down_rssi_dbm(self) -> int:
        return -self.down_rssi


@dataclass
class RcDiagnosticsTelemetry:
    """Raw CRSF layer for RC troubleshooting (a8n).

    Complements :class:`RcInputTelemetry` (the parsed microsecond channels +
    arm/kill/gait/autonomy flags) with the raw 11-bit ticks, frame-health
    counters, dropout age, and decoded link statistics.
    """

    ever_seen: bool = False
    failsafe: bool = True
    link_stats_valid: bool = False
    raw_ticks: list[int] = field(default_factory=list)
    frames_decoded: int = 0
    crc_errors: int = 0
    link_stats_count: int = 0
    last_frame_age_ms: int = 0xFFFF
    link_stats: RcLinkStats = field(default_factory=RcLinkStats)


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


@dataclass
class LegTarget:
    """One commanded foot target (mm, body frame) and its IK verdict (eax.3)."""

    leg: int
    foot_x_mm: int
    foot_y_mm: int
    foot_z_mm: int
    reachable: bool
    clamped: bool


@dataclass
class LegStateTelemetry:
    legs: list[LegTarget] = field(default_factory=list)


@dataclass
class ControllerRawInputs:
    """Raw ChannelPack inputs carried alongside the decoded controller state."""

    gimbal: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    pot: list[int] = field(default_factory=lambda: [0, 0])
    encoder: list[int] = field(default_factory=lambda: [0, 0])
    switches: list[bool] = field(default_factory=lambda: [False] * 8)
    buttons: list[bool] = field(default_factory=lambda: [False] * 4)
    toggles: list[int] = field(default_factory=lambda: [0, 0])
    nav: list[list[bool]] = field(default_factory=lambda: [[False] * 5, [False] * 5])


@dataclass
class ControllerStateTelemetry:
    """Decoded hand-controller intent + raw inputs (oha.4).

    Mirrors ``ControllerApi::encodeState`` and the CONTROLLER_GET_STATE response.
    Scaled fields are restored to engineering units: twist [-1, 1], pose
    translation mm, pose/trim rotation rad, shape scalars [0, 1].
    """

    valid: bool = False
    failsafe: bool = True
    ever_seen: bool = False
    arm_request: bool = False
    estop: bool = True
    host_authority: bool = False
    feat_foot_contact: bool = False
    feat_terrain_leveling: bool = False
    feat_passive_pose: bool = False
    mode: int = 0
    gait_index: int = 0
    trick: int = 0
    twist_vx: float = 0.0
    twist_vy: float = 0.0
    twist_wz: float = 0.0
    pose_x_mm: float = 0.0
    pose_y_mm: float = 0.0
    pose_z_mm: float = 0.0
    pose_roll: float = 0.0
    pose_pitch: float = 0.0
    pose_yaw: float = 0.0
    trim_roll: float = 0.0
    trim_pitch: float = 0.0
    speed: float = 0.0
    body_height: float = 0.0
    stride: float = 0.0
    step_height: float = 0.0
    raw: ControllerRawInputs = field(default_factory=ControllerRawInputs)


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
        if off + 14 > len(p):
            break
        sid = p[off]
        pos = _u32(p, off + 1)
        # velocity/load are signed-ish raw values; surface as signed 16-bit.
        vel = struct.unpack_from("<h", p, off + 5)[0]
        load = struct.unpack_from("<h", p, off + 7)[0]
        volt = _u16(p, off + 9)
        temp = struct.unpack_from("<b", p, off + 11)[0]
        err = p[off + 12]
        torque = bool(p[off + 13])
        servos.append(ServoStatus(sid, pos, vel, load, volt, temp, err, torque))
        off += 14
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


def decode_rc_diagnostics(p: bytes) -> RcDiagnosticsTelemetry:
    """Decode the rc_diagnostics stream (a8n). Defensive on short payloads.

    Layout: flags(1), 16 x raw_tick(u16), frames_decoded(u32), crc_errors(u32),
    link_stats_count(u32), last_frame_age_ms(u16), then the 10-byte link-stats
    block (up_rssi_ant1, up_rssi_ant2, up_lq, up_snr[i8], active_antenna,
    rf_mode, up_tx_power, down_rssi, down_lq, down_snr[i8]).
    """
    out = RcDiagnosticsTelemetry()
    if len(p) < 1:
        return out
    flags = p[0]
    out.ever_seen = bool(flags & 0x01)
    out.failsafe = bool(flags & 0x02)
    out.link_stats_valid = bool(flags & 0x04)
    off = 1
    ticks: list[int] = []
    for _ in range(NUM_CHANNELS):
        if off + 2 > len(p):
            break
        ticks.append(_u16(p, off))
        off += 2
    out.raw_ticks = ticks
    if off + 4 <= len(p):
        out.frames_decoded = _u32(p, off)
        off += 4
    if off + 4 <= len(p):
        out.crc_errors = _u32(p, off)
        off += 4
    if off + 4 <= len(p):
        out.link_stats_count = _u32(p, off)
        off += 4
    if off + 2 <= len(p):
        out.last_frame_age_ms = _u16(p, off)
        off += 2
    if off + 10 <= len(p):
        out.link_stats = RcLinkStats(
            up_rssi_ant1=p[off],
            up_rssi_ant2=p[off + 1],
            up_link_quality=p[off + 2],
            up_snr=struct.unpack_from("<b", p, off + 3)[0],
            active_antenna=p[off + 4],
            rf_mode=p[off + 5],
            up_tx_power=p[off + 6],
            down_rssi=p[off + 7],
            down_link_quality=p[off + 8],
            down_snr=struct.unpack_from("<b", p, off + 9)[0],
        )
    return out


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


def decode_leg_state(p: bytes) -> LegStateTelemetry:
    """Per-leg commanded foot target (eax.3): count(1) then leg(1),
    foot_x(int16), foot_y(int16), foot_z(int16, mm body frame), flags(1) per
    leg. flags bit0 = reachable, bit1 = clamped (a joint hit its travel). Only
    legs with a recorded SET_LEG_TARGET attempt are present."""
    if not p:
        return LegStateTelemetry()
    count = p[0]
    legs: list[LegTarget] = []
    off = 1
    for _ in range(count):
        if off + 8 > len(p):
            break
        leg = p[off]
        x = struct.unpack_from("<h", p, off + 1)[0]
        y = struct.unpack_from("<h", p, off + 3)[0]
        z = struct.unpack_from("<h", p, off + 5)[0]
        flags = p[off + 7]
        legs.append(LegTarget(leg, x, y, z, bool(flags & 0x01), bool(flags & 0x02)))
        off += 8
    return LegStateTelemetry(legs)


def _i16(buf: bytes, off: int) -> int:
    return struct.unpack_from("<h", buf, off)[0]


def decode_controller_state(p: bytes) -> ControllerStateTelemetry:
    """Decoded hand-controller intent + raw inputs (oha.4): a fixed 57-byte
    layout (31 decoded + 26 raw) mirroring ``ControllerApi::encodeState``.
    Defensive on short payloads (returns a partial/default record)."""
    out = ControllerStateTelemetry()
    if len(p) < 31:
        return out
    f1 = p[0]
    f2 = p[1]
    out.valid = bool(f1 & 0x01)
    out.failsafe = bool(f1 & 0x02)
    out.ever_seen = bool(f1 & 0x04)
    out.arm_request = bool(f1 & 0x08)
    out.estop = bool(f1 & 0x10)
    out.host_authority = bool(f1 & 0x20)
    out.feat_foot_contact = bool(f1 & 0x40)
    out.feat_terrain_leveling = bool(f1 & 0x80)
    out.feat_passive_pose = bool(f2 & 0x01)
    out.mode = p[2]
    out.gait_index = p[3]
    out.trick = p[4]
    out.twist_vx = _i16(p, 5) / 1000.0
    out.twist_vy = _i16(p, 7) / 1000.0
    out.twist_wz = _i16(p, 9) / 1000.0
    out.pose_x_mm = float(_i16(p, 11))
    out.pose_y_mm = float(_i16(p, 13))
    out.pose_z_mm = float(_i16(p, 15))
    out.pose_roll = _i16(p, 17) / 1000.0
    out.pose_pitch = _i16(p, 19) / 1000.0
    out.pose_yaw = _i16(p, 21) / 1000.0
    out.trim_roll = _i16(p, 23) / 1000.0
    out.trim_pitch = _i16(p, 25) / 1000.0
    out.speed = p[27] / 255.0
    out.body_height = p[28] / 255.0
    out.stride = p[29] / 255.0
    out.step_height = p[30] / 255.0
    if len(p) < 57:
        return out
    raw = ControllerRawInputs()
    raw.gimbal = [_i16(p, 31 + 2 * i) for i in range(4)]
    raw.pot = [_i16(p, 39 + 2 * i) for i in range(2)]
    raw.encoder = [struct.unpack_from("<i", p, 43 + 4 * i)[0] for i in range(2)]
    sw = p[51]
    raw.switches = [bool(sw & (1 << i)) for i in range(8)]
    btn = p[52]
    raw.buttons = [bool(btn & (1 << i)) for i in range(4)]
    raw.toggles = [p[53], p[54]]
    nav1 = p[55]
    nav2 = p[56]
    raw.nav = [
        [bool(nav1 & (1 << i)) for i in range(5)],
        [bool(nav2 & (1 << i)) for i in range(5)],
    ]
    out.raw = raw
    return out


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
    StreamId.LEG_STATE: decode_leg_state,
    StreamId.CONTROLLER_STATE: decode_controller_state,
    StreamId.RC_DIAGNOSTICS: decode_rc_diagnostics,
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
