"""USB API v0 message builders and parsers (host side).

Mirrors ``firmware/openrb150/src/protocol/api.{h,cpp}``. Use ``build_*`` to make
a command frame and ``parse_*`` to decode a response payload.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

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

# Motion command group (mirrors src/protocol/motion_api.h, 0x34..0x38).
MSG_SET_GAIT = 0x34
MSG_SET_GAIT_PARAMS = 0x35
MSG_SET_BODY_TWIST = 0x36
MSG_SET_BODY_POSE = 0x37
MSG_STOP_MOTION = 0x38

# Maintenance command group (mirrors src/protocol/maintenance_api.h, 0x50..0x5F).
MSG_ENTER_MAINTENANCE = 0x50
MSG_EXIT_MAINTENANCE = 0x51
MSG_MAINT_HEARTBEAT = 0x52
MSG_SET_LEG_TARGET = 0x53
MSG_SET_JOINT_TARGET = 0x54

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

# Gait ids (mirror config::GaitId / motiongait).
GAIT_STAND = 0
GAIT_SIT = 1
GAIT_TRIPOD = 2
GAIT_RIPPLE = 3
GAIT_WAVE = 4
GAIT_CRAWL = 5

# Motion response result byte (mirrors MotionResult).
MOTION_OK = 0
MOTION_REJECTED = 1
MOTION_BAD_REQUEST = 2

# Maintenance response result byte (mirrors MaintResult).
MAINT_OK = 0
MAINT_REJECTED = 1
MAINT_BAD_REQUEST = 2
MAINT_BUSY = 3
MAINT_BAD_TOKEN = 4

# Maintenance leg/joint target result byte (mirrors MaintTargetResult).
MAINT_TARGET_OK = 0
MAINT_TARGET_REJECTED = 1
MAINT_TARGET_BAD_REQUEST = 2
MAINT_TARGET_UNREACHABLE = 3

# DXL maintenance command group (mirrors src/protocol/dxl_job_api.h, 0x60..0x6F).
MSG_DXL_SCAN = 0x60
MSG_DXL_PING = 0x61
MSG_DXL_TORQUE = 0x62
MSG_DXL_GET_SERVO_PROFILE = 0x63
MSG_DXL_GET_RESULT = 0x64
MSG_DXL_GET_PARAM = 0x65
MSG_DXL_SET_PARAM = 0x66
MSG_DXL_SET_SERVO_LIMITS = 0x67

# DXL submit response byte (mirrors DxlSubmit).
DXL_SUBMIT_ACCEPTED = 0
DXL_SUBMIT_REJECTED = 1
DXL_SUBMIT_BUSY = 2
DXL_SUBMIT_BAD_REQUEST = 3

# DXL job slot lifecycle (mirrors dxljob::Slot).
DXL_SLOT_EMPTY = 0
DXL_SLOT_PENDING = 1
DXL_SLOT_RUNNING = 2
DXL_SLOT_DONE = 3

# DXL job result code (mirrors dxljob::Code).
DXL_CODE_OK = 0
DXL_CODE_NOT_FOUND = 1
DXL_CODE_POWER_OFF = 2
DXL_CODE_BUS_ERROR = 3
DXL_CODE_UNSUPPORTED = 4
DXL_CODE_VERIFY_FAILED = 5

# DXL logical parameters (mirrors dxl::LogicalParam in src/dxl/dxl_params.h).
DXL_PARAM_ID = 0
DXL_PARAM_BAUD_RATE = 1
DXL_PARAM_RETURN_DELAY_TIME = 2
DXL_PARAM_CW_ANGLE_LIMIT = 3
DXL_PARAM_CCW_ANGLE_LIMIT = 4
DXL_PARAM_MIN_POSITION_LIMIT = 5
DXL_PARAM_MAX_POSITION_LIMIT = 6
DXL_PARAM_TEMPERATURE_LIMIT = 7
DXL_PARAM_MIN_VOLTAGE_LIMIT = 8
DXL_PARAM_MAX_VOLTAGE_LIMIT = 9
DXL_PARAM_MAX_TORQUE = 10
DXL_PARAM_STATUS_RETURN_LEVEL = 11
DXL_PARAM_SHUTDOWN = 12
DXL_PARAM_PID_P = 13
DXL_PARAM_PID_I = 14
DXL_PARAM_PID_D = 15
DXL_PARAM_MOVING_SPEED = 16
DXL_PARAM_TORQUE_LIMIT = 17
DXL_PARAM_GOAL_ACCELERATION = 18
DXL_PARAM_HOMING_OFFSET = 19
DXL_PARAM_PROFILE_VELOCITY = 20
DXL_PARAM_PROFILE_ACCELERATION = 21
DXL_PARAM_BUS_WATCHDOG = 22
DXL_PARAM_TORQUE_ENABLE = 23
DXL_PARAM_COUNT = 24

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


# --- Motion commands ------------------------------------------------------


def build_set_gait(gait: int, seq: int = 0) -> bytes:
    """Build a SET_GAIT command (0=stand,1=sit,2=tripod,3=ripple,4=wave,
    5=crawl). Out-of-range ids return MOTION_REJECTED.
    """
    return build_command(MSG_SET_GAIT, seq=seq, payload=bytes([gait & 0xFF]))


def build_set_gait_params(
    body_height_mm: int,
    stride_len_mm: int,
    step_height_mm: int,
    duty_x255: int,
    speed_x255: int,
    seq: int = 0,
) -> bytes:
    """Build a SET_GAIT_PARAMS command. Lengths in mm; duty/speed are 0..255.
    Firmware clamps each to the safe gait-engine limits.
    """
    payload = struct.pack(
        "<HHHBB",
        body_height_mm & 0xFFFF,
        stride_len_mm & 0xFFFF,
        step_height_mm & 0xFFFF,
        duty_x255 & 0xFF,
        speed_x255 & 0xFF,
    )
    return build_command(MSG_SET_GAIT_PARAMS, seq=seq, payload=payload)


def build_set_body_twist(vx: float, vy: float, wz: float, seq: int = 0) -> bytes:
    """Build a SET_BODY_TWIST command. Components are normalised [-1,1]
    (forward, left, yaw-CCW) and sent as signed milli-units.
    """

    def milli(v: float) -> int:
        return max(-1000, min(1000, int(round(v * 1000.0))))

    payload = struct.pack("<hhh", milli(vx), milli(vy), milli(wz))
    return build_command(MSG_SET_BODY_TWIST, seq=seq, payload=payload)


def build_set_body_pose(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    seq: int = 0,
) -> bytes:
    """Build a SET_BODY_POSE command. Translation in mm; rotation in degrees
    (sent as signed milli-degrees). Firmware clamps to the safe pose window.
    """

    def mm(v: float) -> int:
        return max(-32768, min(32767, int(round(v))))

    def mdeg(v: float) -> int:
        return max(-32768, min(32767, int(round(v * 1000.0))))

    payload = struct.pack(
        "<hhhhhh",
        mm(x_mm),
        mm(y_mm),
        mm(z_mm),
        mdeg(roll_deg),
        mdeg(pitch_deg),
        mdeg(yaw_deg),
    )
    return build_command(MSG_SET_BODY_POSE, seq=seq, payload=payload)


def build_stop_motion(seq: int = 0) -> bytes:
    """Build a STOP_MOTION command (zero twist + hold Stand). Always honored."""
    return build_command(MSG_STOP_MOTION, seq=seq)


@dataclass
class MotionResultMsg:
    result: int
    state: int
    motion_allowed: bool

    @property
    def ok(self) -> bool:
        return self.result == MOTION_OK

    @property
    def rejected(self) -> bool:
        return self.result == MOTION_REJECTED


def parse_motion_result(payload: bytes) -> MotionResultMsg:
    """Decode a motion-command response payload ([result, state,
    motion_allowed]). A 1-byte payload is a malformed-request error.
    """
    if len(payload) >= 3:
        return MotionResultMsg(payload[0], payload[1], bool(payload[2]))
    return MotionResultMsg(payload[0] if payload else MOTION_BAD_REQUEST, 0, False)


# --- Maintenance lock commands --------------------------------------------


def build_enter_maintenance(seq: int = 0) -> bytes:
    """Build an ENTER_MAINTENANCE command. Granted only in a safe robot state
    (disarmed / stand-ready); the response carries the lock token.
    """
    return build_command(MSG_ENTER_MAINTENANCE, seq=seq)


def build_exit_maintenance(token: int, seq: int = 0) -> bytes:
    """Build an EXIT_MAINTENANCE command releasing the lock held by ``token``."""
    return build_command(
        MSG_EXIT_MAINTENANCE, seq=seq, payload=struct.pack("<I", token & 0xFFFFFFFF)
    )


def build_maint_heartbeat(token: int, seq: int = 0) -> bytes:
    """Build a MAINT_HEARTBEAT command refreshing the lock TTL for ``token``."""
    return build_command(
        MSG_MAINT_HEARTBEAT, seq=seq, payload=struct.pack("<I", token & 0xFFFFFFFF)
    )


@dataclass
class MaintResultMsg:
    result: int
    state: int
    token: int  # nonzero only on a successful ENTER

    @property
    def ok(self) -> bool:
        return self.result == MAINT_OK

    @property
    def busy(self) -> bool:
        return self.result == MAINT_BUSY

    @property
    def bad_token(self) -> bool:
        return self.result == MAINT_BAD_TOKEN


def parse_maint_result(payload: bytes) -> MaintResultMsg:
    """Decode a maintenance-command response payload. ENTER success is
    [result, state, token(4)]; EXIT/HEARTBEAT are [result, state]; a 1-byte
    payload is a malformed-request error.
    """
    if len(payload) >= 6:
        (token,) = struct.unpack("<I", payload[2:6])
        return MaintResultMsg(payload[0], payload[1], token)
    if len(payload) >= 2:
        return MaintResultMsg(payload[0], payload[1], 0)
    return MaintResultMsg(payload[0] if payload else MAINT_BAD_REQUEST, 0, 0)


# --- Maintenance leg/joint targets ----------------------------------------


def build_set_leg_target(
    leg: int, x_mm: int, y_mm: int, z_mm: int, seq: int = 0
) -> bytes:
    """Build a SET_LEG_TARGET command: move leg ``leg``'s foot to (x, y, z) mm
    in the body frame. Only honored in MacMaintenance with the lock held.
    """
    return build_command(
        MSG_SET_LEG_TARGET,
        seq=seq,
        payload=struct.pack("<Bhhh", leg & 0xFF, x_mm, y_mm, z_mm),
    )


def build_set_joint_target(
    leg: int, joint: int, angle_cdeg: int, seq: int = 0
) -> bytes:
    """Build a SET_JOINT_TARGET command: set one joint (leg, joint) to
    ``angle_cdeg`` centidegrees (URDF-zero-relative). Maintenance-gated.
    """
    return build_command(
        MSG_SET_JOINT_TARGET,
        seq=seq,
        payload=struct.pack("<BBh", leg & 0xFF, joint & 0xFF, angle_cdeg),
    )


@dataclass
class LegTargetResult:
    result: int
    state: int
    reachable: bool
    clamp_low: int  # bitmask over (coxa, femur, tibia)
    clamp_high: int
    ticks: tuple[int, int, int]

    @property
    def ok(self) -> bool:
        return self.result == MAINT_TARGET_OK


@dataclass
class JointTargetResult:
    result: int
    state: int
    clamped_low: bool
    clamped_high: bool
    tick: int

    @property
    def ok(self) -> bool:
        return self.result == MAINT_TARGET_OK


def parse_leg_target_result(payload: bytes) -> LegTargetResult:
    """Decode a SET_LEG_TARGET response
    ([result, state, reachable, clamp_low, clamp_high, 3 x tick(u16)]).
    A short payload is a rejected/error status with no ticks.
    """
    if len(payload) >= 11:
        c, f, t = struct.unpack("<HHH", payload[5:11])
        return LegTargetResult(
            payload[0],
            payload[1],
            bool(payload[2]),
            payload[3],
            payload[4],
            (c, f, t),
        )
    state = payload[1] if len(payload) >= 2 else 0
    return LegTargetResult(
        payload[0] if payload else MAINT_TARGET_BAD_REQUEST,
        state,
        False,
        0,
        0,
        (0, 0, 0),
    )


def parse_joint_target_result(payload: bytes) -> JointTargetResult:
    """Decode a SET_JOINT_TARGET response
    ([result, state, clamp_low, clamp_high, tick(u16)]). A short payload is a
    rejected/error status with no tick.
    """
    if len(payload) >= 6:
        (tick,) = struct.unpack("<H", payload[4:6])
        return JointTargetResult(
            payload[0], payload[1], bool(payload[2]), bool(payload[3]), tick
        )
    state = payload[1] if len(payload) >= 2 else 0
    return JointTargetResult(
        payload[0] if payload else MAINT_TARGET_BAD_REQUEST,
        state,
        False,
        False,
        0,
    )


# --- DXL maintenance commands (async job queue) ----------------------------


def build_dxl_scan(first_id: int = 1, last_id: int = 252, seq: int = 0) -> bytes:
    """Build a DXL_SCAN command over the inclusive id range [first, last]."""
    return build_command(
        MSG_DXL_SCAN,
        seq=seq,
        payload=struct.pack("<BB", first_id & 0xFF, last_id & 0xFF),
    )


def build_dxl_ping(servo_id: int, seq: int = 0) -> bytes:
    """Build a DXL_PING command for a single servo id."""
    return build_command(
        MSG_DXL_PING, seq=seq, payload=struct.pack("<B", servo_id & 0xFF)
    )


def build_dxl_torque(on: bool, seq: int = 0) -> bytes:
    """Build a DXL_TORQUE command (enable/disable torque on all servos)."""
    return build_command(
        MSG_DXL_TORQUE, seq=seq, payload=struct.pack("<B", 1 if on else 0)
    )


def build_dxl_get_servo_profile(servo_id: int, seq: int = 0) -> bytes:
    """Build a DXL_GET_SERVO_PROFILE command for a single servo id."""
    return build_command(
        MSG_DXL_GET_SERVO_PROFILE,
        seq=seq,
        payload=struct.pack("<B", servo_id & 0xFF),
    )


def build_dxl_get_result(job_id: int, seq: int = 0) -> bytes:
    """Build a DXL_GET_RESULT poll for a previously submitted job id."""
    return build_command(
        MSG_DXL_GET_RESULT, seq=seq, payload=struct.pack("<B", job_id & 0xFF)
    )


def build_dxl_get_param(servo_id: int, param: int, seq: int = 0) -> bytes:
    """Build a DXL_GET_PARAM command ([id, param])."""
    return build_command(
        MSG_DXL_GET_PARAM,
        seq=seq,
        payload=struct.pack("<BB", servo_id & 0xFF, param & 0xFF),
    )


def build_dxl_set_param(servo_id: int, param: int, value: int, seq: int = 0) -> bytes:
    """Build a DXL_SET_PARAM command ([id, param, value(i32)])."""
    return build_command(
        MSG_DXL_SET_PARAM,
        seq=seq,
        payload=struct.pack("<BBi", servo_id & 0xFF, param & 0xFF, value),
    )


def build_dxl_set_servo_limits(
    servo_id: int, min_tick: int, max_tick: int, seq: int = 0
) -> bytes:
    """Build a DXL_SET_SERVO_LIMITS command ([id, min(i32), max(i32)])."""
    return build_command(
        MSG_DXL_SET_SERVO_LIMITS,
        seq=seq,
        payload=struct.pack("<Bii", servo_id & 0xFF, min_tick, max_tick),
    )


@dataclass
class DxlSubmitResult:
    result: int  # DXL_SUBMIT_*
    job_id: int  # non-zero only when accepted
    slot: int  # DXL_SLOT_*

    @property
    def accepted(self) -> bool:
        return self.result == DXL_SUBMIT_ACCEPTED


@dataclass
class DxlServoRecord:
    id: int
    model: int
    firmware: int
    protocol: int
    table_kind: int


@dataclass
class DxlJobResult:
    slot: int  # DXL_SLOT_*
    code: int  # DXL_CODE_* (meaningful only when slot == DONE)
    data: bytes  # serialized job payload (empty unless DONE)

    @property
    def done(self) -> bool:
        return self.slot == DXL_SLOT_DONE

    def servos(self) -> list[DxlServoRecord]:
        """Decode a DONE DXL_SCAN result ([count, count x 6-byte records])."""
        if not self.done or not self.data:
            return []
        count = self.data[0]
        out: list[DxlServoRecord] = []
        off = 1
        for _ in range(count):
            if off + 6 > len(self.data):
                break
            sid = self.data[off]
            model = self.data[off + 1] | (self.data[off + 2] << 8)
            out.append(
                DxlServoRecord(
                    sid,
                    model,
                    self.data[off + 3],
                    self.data[off + 4],
                    self.data[off + 5],
                )
            )
            off += 6
        return out

    def param(self) -> Optional["DxlParamValue"]:
        """Decode a DONE GET_PARAM result ([param, table, len, value(i32)])."""
        if not self.done or len(self.data) < 7:
            return None
        param, table, length = self.data[0], self.data[1], self.data[2]
        value = struct.unpack_from("<i", self.data, 3)[0]
        return DxlParamValue(param, table, length, value)

    def set_param(self) -> Optional["DxlSetParamResult"]:
        """Decode a SET_PARAM result ([param, len, written, readback, ok])."""
        if len(self.data) < 11:
            return None
        param, length = self.data[0], self.data[1]
        written = struct.unpack_from("<i", self.data, 2)[0]
        readback = struct.unpack_from("<i", self.data, 6)[0]
        verified = self.data[10] != 0
        return DxlSetParamResult(param, length, written, readback, verified)

    def servo_limits(self) -> Optional["DxlServoLimitsResult"]:
        """Decode a SET_SERVO_LIMITS result ([table, min, max, ok])."""
        if len(self.data) < 10:
            return None
        table = self.data[0]
        min_tick = struct.unpack_from("<i", self.data, 1)[0]
        max_tick = struct.unpack_from("<i", self.data, 5)[0]
        verified = self.data[9] != 0
        return DxlServoLimitsResult(table, min_tick, max_tick, verified)


@dataclass
class DxlParamValue:
    param: int  # DXL_PARAM_*
    table_kind: int
    length: int  # 1, 2, or 4 bytes
    value: int


@dataclass
class DxlSetParamResult:
    param: int  # DXL_PARAM_*
    length: int
    written: int
    readback: int
    verified: bool


@dataclass
class DxlServoLimitsResult:
    table_kind: int
    min_tick: int
    max_tick: int
    verified: bool


def parse_dxl_submit(payload: bytes) -> DxlSubmitResult:
    """Decode a DXL submit response ([result, job_id, slot])."""
    if len(payload) >= 3:
        return DxlSubmitResult(payload[0], payload[1], payload[2])
    return DxlSubmitResult(
        payload[0] if payload else DXL_SUBMIT_BAD_REQUEST, 0, DXL_SLOT_EMPTY
    )


def parse_dxl_result(payload: bytes) -> DxlJobResult:
    """Decode a DXL_GET_RESULT response ([slot, code, len, data...])."""
    if len(payload) >= 3:
        n = payload[2]
        data = bytes(payload[3 : 3 + n])
        return DxlJobResult(payload[0], payload[1], data)
    return DxlJobResult(DXL_SLOT_EMPTY, DXL_CODE_OK, b"")
