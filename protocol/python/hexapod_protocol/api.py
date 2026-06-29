"""USB API v0 message builders and parsers (host side).

Mirrors ``firmware/openrb150/src/protocol/api.{h,cpp}``. Use ``build_*`` to make
a command frame and ``parse_*`` to decode a response payload.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional

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

# Config validate/commit result byte (mirrors config::CfgResult).
CFG_OK = 0
CFG_VALIDATION_FAILED = 1
CFG_VOLATILE = 2
CFG_COMMIT_FAILED = 3

# Config error byte returned with the protocol error flag (mirrors CfgError).
CFG_ERR_NONE = 0
CFG_ERR_BAD_REQUEST = 1
CFG_ERR_BAD_RANGE = 2

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

# Feature flag command group (mirrors src/protocol/feature_api.h, 0x39..0x3C).
MSG_FEATURE_GET = 0x39
MSG_FEATURE_SET = 0x3A
MSG_FEATURE_GET_REASONS = 0x3B
MSG_FEATURE_RESET_DEFAULTS = 0x3C

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

# Feature ids (mirror protocol::Feature; wire-stable, append-only).
FEATURE_FOOT_CONTACT = 0
FEATURE_TERRAIN_LEVELING = 1
FEATURE_SENSOR_POLLING = 2
FEATURE_JETSON_CONTROL = 3
FEATURE_PASSIVE_POSE = 4
FEATURE_COUNT = 5

# Feature unavailability reason byte (mirrors protocol::FeatureReason).
FEATURE_REASON_NONE = 0
FEATURE_REASON_HARDWARE_MISSING = 1
FEATURE_REASON_NOT_CALIBRATED = 2
FEATURE_REASON_UNSAFE_STATE = 3
FEATURE_REASON_STALE_DATA = 4
FEATURE_REASON_DEPENDENCY_OFF = 5
FEATURE_REASON_NOT_IMPLEMENTED = 6

# Feature response result byte (mirrors protocol::FeatureResult).
FEATURE_OK = 0
FEATURE_REJECTED = 1
FEATURE_BAD_REQUEST = 2

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
MSG_DXL_READ_REGISTER = 0x68
MSG_DXL_WRITE_REGISTER = 0x69
MSG_DXL_POWER = 0x6A

# Sensor / contact / leveling command group (mirrors src/protocol/sensor_api.h,
# 0x70..0x7F). ubs.5.1 covers the contact + leveling control subset; the I2C
# scan / topology / sensor status / rate / calibrate commands (0x76..0x7B) are
# added by ubs.5.2.
MSG_CONTACT_ENABLE = 0x70
MSG_CONTACT_DISABLE = 0x71
MSG_CONTACT_SET_THRESHOLDS = 0x72
MSG_LEVELING_ENABLE = 0x73
MSG_LEVELING_DISABLE = 0x74
MSG_LEVELING_SET_PARAMS = 0x75
MSG_I2C_SCAN = 0x76
MSG_I2C_GET_TOPOLOGY = 0x77
MSG_SENSOR_GET_STATUS = 0x78
MSG_SENSOR_SET_RATE = 0x79
MSG_CONTACT_CALIBRATE = 0x7A
MSG_SENSOR_CALIBRATE = 0x7B

# Passive pose streaming commands (mirror protocol::passivemsg, 0x80-0x83).
MSG_PASSIVE_ENTER = 0x80
MSG_PASSIVE_EXIT = 0x81
MSG_PASSIVE_SET_STREAM_RATE = 0x82
MSG_PASSIVE_ZERO_REFERENCE = 0x83

# Passive response result byte (mirrors protocol::PassiveResult).
PASSIVE_OK = 0
PASSIVE_REJECTED = 1
PASSIVE_BAD_REQUEST = 2

# Present-position stream rate bounds (Hz) for PASSIVE_SET_STREAM_RATE
# (mirror protocol::passiverate).
PASSIVE_RATE_DEFAULT_HZ = 50
PASSIVE_RATE_MIN_HZ = 1
PASSIVE_RATE_MAX_HZ = 200

# Sensor response result byte (mirrors protocol::SensorResult).
SENSOR_OK = 0
SENSOR_REJECTED = 1
SENSOR_BAD_REQUEST = 2

# Foot count for the contact sensors (mirrors protocol::kSensorNumFeet).
SENSOR_NUM_FEET = 6

# Mux channel count for the I2C topology (mirrors protocol::kSensorNumChannels).
SENSOR_NUM_CHANNELS = 8

# Calibrate-all foot selector for CONTACT_CALIBRATE.
SENSOR_CALIBRATE_ALL = 0xFF


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


# --- Session commands ------------------------------------------------------


def build_hello(seq: int = 0) -> bytes:
    """Build a HELLO command (handshake; firmware replies with HelloInfo)."""
    return build_command(MSG_HELLO, seq=seq)


def build_heartbeat(seq: int = 0) -> bytes:
    """Build a HEARTBEAT command (liveness ping; replies uptime + state)."""
    return build_command(MSG_HEARTBEAT, seq=seq)


def build_get_status(seq: int = 0) -> bytes:
    """Build a GET_STATUS command (replies with StatusInfo)."""
    return build_command(MSG_GET_STATUS, seq=seq)


def build_get_capabilities(seq: int = 0) -> bytes:
    """Build a GET_CAPABILITIES command (replies with Capabilities)."""
    return build_command(MSG_GET_CAPABILITIES, seq=seq)


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


# --- Feature flag commands ------------------------------------------------


def build_feature_get(seq: int = 0) -> bytes:
    """Build a FEATURE_GET command. The response lists every feature's
    [id, available, enabled, reason].
    """
    return build_command(MSG_FEATURE_GET, seq=seq)


def build_feature_set(feature: int, enable: bool, seq: int = 0) -> bytes:
    """Build a FEATURE_SET command. Enabling a feature the firmware reports as
    unavailable returns FEATURE_REJECTED with the reason echoed; disabling is
    always honored.
    """
    payload = bytes([feature & 0xFF, 1 if enable else 0])
    return build_command(MSG_FEATURE_SET, seq=seq, payload=payload)


def build_feature_get_reasons(seq: int = 0) -> bytes:
    """Build a FEATURE_GET_REASONS command. The response lists every feature's
    [id, reason].
    """
    return build_command(MSG_FEATURE_GET_REASONS, seq=seq)


def build_feature_reset_defaults(seq: int = 0) -> bytes:
    """Build a FEATURE_RESET_DEFAULTS command restoring the compiled enable
    set. The response carries the full feature state list.
    """
    return build_command(MSG_FEATURE_RESET_DEFAULTS, seq=seq)


@dataclass
class FeatureState:
    feature: int
    available: bool
    enabled: bool
    reason: int


@dataclass
class FeatureList:
    state: int
    features: List[FeatureState]

    def get(self, feature: int) -> Optional[FeatureState]:
        for f in self.features:
            if f.feature == feature:
                return f
        return None


def parse_feature_list(payload: bytes) -> FeatureList:
    """Decode a FEATURE_GET / FEATURE_RESET_DEFAULTS state list.

    FEATURE_GET response:            [state, count, {id, available, enabled,
                                      reason} x count]
    FEATURE_RESET_DEFAULTS response: [result, state, count, {id, available,
                                      enabled, reason} x count]
    The leading result byte of RESET is skipped by detecting the layout from
    the count field's position.
    """
    if len(payload) < 2:
        return FeatureList(0, [])
    # RESET_DEFAULTS prefixes a result byte; GET does not. Disambiguate by
    # checking whether the records fit from offset 2 (GET) or 3 (RESET).
    for header in (2, 3):
        if len(payload) < header:
            continue
        count = payload[header - 1]
        if len(payload) == header + 4 * count:
            state = payload[header - 2] if header == 3 else payload[0]
            feats = []
            off = header
            for _ in range(count):
                feats.append(
                    FeatureState(
                        payload[off],
                        bool(payload[off + 1]),
                        bool(payload[off + 2]),
                        payload[off + 3],
                    )
                )
                off += 4
            return FeatureList(state, feats)
    return FeatureList(payload[0], [])


@dataclass
class FeatureReasonEntry:
    feature: int
    reason: int


@dataclass
class FeatureReasons:
    state: int
    reasons: List[FeatureReasonEntry]


def parse_feature_reasons(payload: bytes) -> FeatureReasons:
    """Decode a FEATURE_GET_REASONS response ([state, count, {id, reason} x
    count]).
    """
    if len(payload) < 2:
        return FeatureReasons(0, [])
    state = payload[0]
    count = payload[1]
    out = []
    off = 2
    for _ in range(count):
        if off + 2 > len(payload):
            break
        out.append(FeatureReasonEntry(payload[off], payload[off + 1]))
        off += 2
    return FeatureReasons(state, out)


@dataclass
class FeatureSetResult:
    result: int
    state: int
    feature: int
    available: bool
    enabled: bool
    reason: int

    @property
    def ok(self) -> bool:
        return self.result == FEATURE_OK

    @property
    def rejected(self) -> bool:
        return self.result == FEATURE_REJECTED


def parse_feature_set_result(payload: bytes) -> FeatureSetResult:
    """Decode a FEATURE_SET response ([result, state, id, available, enabled,
    reason]). A 1-byte payload is a malformed-request error.
    """
    if len(payload) >= 6:
        return FeatureSetResult(
            payload[0],
            payload[1],
            payload[2],
            bool(payload[3]),
            bool(payload[4]),
            payload[5],
        )
    return FeatureSetResult(
        payload[0] if payload else FEATURE_BAD_REQUEST, 0, 0, False, False, 0
    )


# --- Sensor / contact / leveling commands ---------------------------------


def build_contact_enable(seq: int = 0) -> bytes:
    """Build a CONTACT_ENABLE command (routes to the FootContact feature).
    Enabling while sensors are missing/stale returns SENSOR_REJECTED with the
    blocking reason echoed.
    """
    return build_command(MSG_CONTACT_ENABLE, seq=seq)


def build_contact_disable(seq: int = 0) -> bytes:
    """Build a CONTACT_DISABLE command (always honored)."""
    return build_command(MSG_CONTACT_DISABLE, seq=seq)


def build_leveling_enable(seq: int = 0) -> bytes:
    """Build a LEVELING_ENABLE command (routes to the TerrainLeveling feature)."""
    return build_command(MSG_LEVELING_ENABLE, seq=seq)


def build_leveling_disable(seq: int = 0) -> bytes:
    """Build a LEVELING_DISABLE command (always honored)."""
    return build_command(MSG_LEVELING_DISABLE, seq=seq)


def build_contact_set_thresholds(
    foot: int, near: int, touch: int, load: int, seq: int = 0
) -> bytes:
    """Build a CONTACT_SET_THRESHOLDS command for one foot. The firmware stages
    the per-foot near/touch/load gates and the contact estimator applies them.
    """
    payload = struct.pack(
        "<BHHH", foot & 0xFF, near & 0xFFFF, touch & 0xFFFF, load & 0xFFFF
    )
    return build_command(MSG_CONTACT_SET_THRESHOLDS, seq=seq, payload=payload)


def build_leveling_set_params(
    max_tilt_mdeg: int, rate_mdeg_s: int, response_x255: int, seq: int = 0
) -> bytes:
    """Build a LEVELING_SET_PARAMS command staging the leveling tunables."""
    payload = struct.pack(
        "<HHH",
        max_tilt_mdeg & 0xFFFF,
        rate_mdeg_s & 0xFFFF,
        response_x255 & 0xFFFF,
    )
    return build_command(MSG_LEVELING_SET_PARAMS, seq=seq, payload=payload)


@dataclass
class SensorFeatureResult:
    """Response to CONTACT_*/LEVELING_* enable/disable ([result, state,
    available, enabled, reason]).
    """

    result: int
    state: int
    available: bool
    enabled: bool
    reason: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK

    @property
    def rejected(self) -> bool:
        return self.result == SENSOR_REJECTED


def parse_sensor_feature_result(payload: bytes) -> SensorFeatureResult:
    """Decode a contact/leveling enable/disable response. A 1-byte payload is a
    malformed-request error.
    """
    if len(payload) >= 5:
        return SensorFeatureResult(
            payload[0], payload[1], bool(payload[2]), bool(payload[3]), payload[4]
        )
    return SensorFeatureResult(
        payload[0] if payload else SENSOR_BAD_REQUEST, 0, False, False, 0
    )


@dataclass
class ContactThresholdResult:
    """Response to CONTACT_SET_THRESHOLDS ([result, foot, near, touch, load])."""

    result: int
    foot: int
    near: int
    touch: int
    load: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK


def parse_contact_threshold_result(payload: bytes) -> ContactThresholdResult:
    """Decode a CONTACT_SET_THRESHOLDS response. A 1-byte payload is an error."""
    if len(payload) >= 8:
        foot = payload[1]
        near, touch, load = struct.unpack_from("<HHH", payload, 2)
        return ContactThresholdResult(payload[0], foot, near, touch, load)
    return ContactThresholdResult(
        payload[0] if payload else SENSOR_BAD_REQUEST, 0, 0, 0, 0
    )


@dataclass
class LevelingParamsResult:
    """Response to LEVELING_SET_PARAMS ([result, max_tilt, rate, response])."""

    result: int
    max_tilt_mdeg: int
    rate_mdeg_s: int
    response_x255: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK


def parse_leveling_params_result(payload: bytes) -> LevelingParamsResult:
    """Decode a LEVELING_SET_PARAMS response. A 1-byte payload is an error."""
    if len(payload) >= 7:
        max_tilt, rate, response = struct.unpack_from("<HHH", payload, 1)
        return LevelingParamsResult(payload[0], max_tilt, rate, response)
    return LevelingParamsResult(
        payload[0] if payload else SENSOR_BAD_REQUEST, 0, 0, 0
    )


# --- I2C scan / topology / sensor status / rate / calibrate (ubs.5.2) ------


def build_i2c_scan(seq: int = 0) -> bytes:
    """Build an I2C_SCAN command. The firmware re-runs its discovery scan
    (owned by i2cTask); poll I2C_GET_TOPOLOGY afterwards for the result.
    """
    return build_command(MSG_I2C_SCAN, seq=seq)


def build_i2c_get_topology(seq: int = 0) -> bytes:
    """Build an I2C_GET_TOPOLOGY command (reads the published topology)."""
    return build_command(MSG_I2C_GET_TOPOLOGY, seq=seq)


def build_sensor_get_status(seq: int = 0) -> bytes:
    """Build a SENSOR_GET_STATUS command (reads the fused foot-status)."""
    return build_command(MSG_SENSOR_GET_STATUS, seq=seq)


def build_sensor_set_rate(rate_hz: int, seq: int = 0) -> bytes:
    """Build a SENSOR_SET_RATE command staging the target poll rate (Hz)."""
    payload = struct.pack("<H", rate_hz & 0xFFFF)
    return build_command(MSG_SENSOR_SET_RATE, seq=seq, payload=payload)


def build_contact_calibrate(foot: int = SENSOR_CALIBRATE_ALL, seq: int = 0) -> bytes:
    """Build a CONTACT_CALIBRATE command. `foot` selects one foot (0..5) or
    SENSOR_CALIBRATE_ALL (0xFF) for every foot. The firmware re-zeroes the
    per-foot pressure baseline to the current reading (foot must be at rest).
    """
    payload = bytes([foot & 0xFF])
    return build_command(MSG_CONTACT_CALIBRATE, seq=seq, payload=payload)


def build_sensor_calibrate(seq: int = 0) -> bytes:
    """Build a SENSOR_CALIBRATE command (re-baselines every foot)."""
    return build_command(MSG_SENSOR_CALIBRATE, seq=seq)


@dataclass
class I2cScanResult:
    """Response to I2C_SCAN ([result, scan_seq u16])."""

    result: int
    scan_seq: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK


def parse_i2c_scan_result(payload: bytes) -> I2cScanResult:
    """Decode an I2C_SCAN response."""
    if len(payload) >= 3:
        return I2cScanResult(payload[0], struct.unpack_from("<H", payload, 1)[0])
    return I2cScanResult(payload[0] if payload else SENSOR_BAD_REQUEST, 0)


@dataclass
class TopologyChannel:
    """One mux channel from I2C_GET_TOPOLOGY."""

    scanned: bool
    vcnl_present: bool
    lps_present: bool
    device_count: int
    state: int  # 0 missing, 1 present, 2 fault


@dataclass
class I2cTopologyResult:
    """Response to I2C_GET_TOPOLOGY (mux/eeprom presence + per-channel state)."""

    mux_present: bool
    eeprom_present: bool
    channels: List[TopologyChannel]


def parse_i2c_topology_result(payload: bytes) -> I2cTopologyResult:
    """Decode an I2C_GET_TOPOLOGY response. A short payload yields no channels."""
    if len(payload) < 3:
        return I2cTopologyResult(False, False, [])
    mux = bool(payload[0])
    eeprom = bool(payload[1])
    n = payload[2]
    channels: List[TopologyChannel] = []
    off = 3
    for _ in range(n):
        if off + 5 > len(payload):
            break
        channels.append(
            TopologyChannel(
                bool(payload[off]),
                bool(payload[off + 1]),
                bool(payload[off + 2]),
                payload[off + 3],
                payload[off + 4],
            )
        )
        off += 5
    return I2cTopologyResult(mux, eeprom, channels)


@dataclass
class FootStatus:
    """One foot from SENSOR_GET_STATUS."""

    state: int
    confidence: int
    proximity: int
    pressure_delta: int
    flags: int

    @property
    def near(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def touch(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def loaded(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def release(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def stale(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def fault(self) -> bool:
        return bool(self.flags & 0x20)


@dataclass
class SensorStatusResult:
    """Response to SENSOR_GET_STATUS (present mask, polling flag, per-foot)."""

    present_mask: int
    polling_enabled: bool
    feet: List[FootStatus]


def parse_sensor_status_result(payload: bytes) -> SensorStatusResult:
    """Decode a SENSOR_GET_STATUS response."""
    if len(payload) < 3:
        return SensorStatusResult(0, False, [])
    n = payload[0]
    present_mask = payload[1]
    polling = bool(payload[2])
    feet: List[FootStatus] = []
    off = 3
    for _ in range(n):
        if off + 7 > len(payload):
            break
        state = payload[off]
        confidence = payload[off + 1]
        proximity, delta = struct.unpack_from("<Hh", payload, off + 2)
        flags = payload[off + 6]
        feet.append(FootStatus(state, confidence, proximity, delta, flags))
        off += 7
    return SensorStatusResult(present_mask, polling, feet)


@dataclass
class SensorRateResult:
    """Response to SENSOR_SET_RATE ([result, rate_hz u16])."""

    result: int
    rate_hz: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK


def parse_sensor_rate_result(payload: bytes) -> SensorRateResult:
    """Decode a SENSOR_SET_RATE response. A 1-byte payload is an error."""
    if len(payload) >= 3:
        return SensorRateResult(payload[0], struct.unpack_from("<H", payload, 1)[0])
    return SensorRateResult(payload[0] if payload else SENSOR_BAD_REQUEST, 0)


@dataclass
class SensorCalibrateResult:
    """Response to CONTACT_CALIBRATE / SENSOR_CALIBRATE ([result, foot_mask])."""

    result: int
    mask: int

    @property
    def ok(self) -> bool:
        return self.result == SENSOR_OK


def parse_sensor_calibrate_result(payload: bytes) -> SensorCalibrateResult:
    """Decode a calibrate response. A 1-byte payload is an error."""
    if len(payload) >= 2:
        return SensorCalibrateResult(payload[0], payload[1])
    return SensorCalibrateResult(
        payload[0] if payload else SENSOR_BAD_REQUEST, 0
    )


# --- Passive pose streaming commands (ubs.6) ------------------------------


def build_passive_enter(seq: int = 0) -> bytes:
    """Build a PASSIVE_ENTER command. The firmware only enters torque-off
    passive pose streaming from a maintenance-safe state (Disarmed /
    MacMaintenance) and once torque is confirmed off.
    """
    return build_command(MSG_PASSIVE_ENTER, seq=seq)


def build_passive_exit(seq: int = 0) -> bytes:
    """Build a PASSIVE_EXIT command (drops the passive request)."""
    return build_command(MSG_PASSIVE_EXIT, seq=seq)


def build_passive_set_stream_rate(rate_hz: int, seq: int = 0) -> bytes:
    """Build a PASSIVE_SET_STREAM_RATE command staging the present-position
    stream rate (Hz). Only honoured while passive streaming is active.
    """
    payload = struct.pack("<H", rate_hz & 0xFFFF)
    return build_command(MSG_PASSIVE_SET_STREAM_RATE, seq=seq, payload=payload)


def build_passive_zero_reference(seq: int = 0) -> bytes:
    """Build a PASSIVE_ZERO_REFERENCE command requesting a present-position
    zero-reference capture. Only honoured while passive streaming is active.
    """
    return build_command(MSG_PASSIVE_ZERO_REFERENCE, seq=seq)


@dataclass
class PassiveResult:
    """Response to PASSIVE_ENTER / PASSIVE_EXIT / PASSIVE_ZERO_REFERENCE
    ([result, state])."""

    result: int
    state: int

    @property
    def ok(self) -> bool:
        return self.result == PASSIVE_OK

    @property
    def rejected(self) -> bool:
        return self.result == PASSIVE_REJECTED


def parse_passive_result(payload: bytes) -> PassiveResult:
    """Decode a passive enter/exit/zero response."""
    if len(payload) >= 2:
        return PassiveResult(payload[0], payload[1])
    return PassiveResult(payload[0] if payload else PASSIVE_BAD_REQUEST, 0)


@dataclass
class PassiveRateResult:
    """Response to PASSIVE_SET_STREAM_RATE ([result, state, rate_hz u16])."""

    result: int
    state: int
    rate_hz: int

    @property
    def ok(self) -> bool:
        return self.result == PASSIVE_OK


def parse_passive_rate_result(payload: bytes) -> PassiveRateResult:
    """Decode a PASSIVE_SET_STREAM_RATE response. A short payload is an error."""
    if len(payload) >= 4:
        return PassiveRateResult(
            payload[0], payload[1], struct.unpack_from("<H", payload, 2)[0]
        )
    return PassiveRateResult(
        payload[0] if payload else PASSIVE_BAD_REQUEST,
        payload[1] if len(payload) >= 2 else 0,
        0,
    )


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


def build_dxl_read_register(
    servo_id: int, address: int, length: int, seq: int = 0
) -> bytes:
    """Build an expert DXL_READ_REGISTER command ([id, addr(u16), len]).

    Raw register access bypasses the logical parameter table and is rejected by
    firmware unless the expert raw-register gate is enabled.
    """
    return build_command(
        MSG_DXL_READ_REGISTER,
        seq=seq,
        payload=struct.pack(
            "<BHB", servo_id & 0xFF, address & 0xFFFF, length & 0xFF
        ),
    )


def build_dxl_write_register(
    servo_id: int,
    address: int,
    length: int,
    value: int,
    is_eeprom: bool = False,
    seq: int = 0,
) -> bytes:
    """Build an expert DXL_WRITE_REGISTER command.

    Payload [id, addr(u16), len, value(i32), flags]; flags bit0 marks an EEPROM
    region so firmware disables torque before writing. Expert-gated.
    """
    flags = 0x01 if is_eeprom else 0x00
    return build_command(
        MSG_DXL_WRITE_REGISTER,
        seq=seq,
        payload=struct.pack(
            "<BHBiB", servo_id & 0xFF, address & 0xFFFF, length & 0xFF,
            value, flags,
        ),
    )


def build_dxl_power(on: bool, seq: int = 0) -> bytes:
    """Build a DXL_POWER command toggling the DYNAMIXEL power FET.

    Payload [on(0/1)]. Maintenance-gated: firmware only accepts it while in the
    MacMaintenance state with the bench lock held, and force-cuts power on any
    exit from maintenance (lock release/expiry, disarm, estop, fault).
    """
    return build_command(
        MSG_DXL_POWER, seq=seq, payload=struct.pack("<B", 1 if on else 0)
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

    def read_register(self) -> Optional["DxlRegisterValue"]:
        """Decode a READ_REGISTER result ([addr(u16), len, value(i32)])."""
        if not self.done or len(self.data) < 7:
            return None
        addr = struct.unpack_from("<H", self.data, 0)[0]
        length = self.data[2]
        value = struct.unpack_from("<i", self.data, 3)[0]
        return DxlRegisterValue(addr, length, value)

    def write_register(self) -> Optional["DxlWriteRegisterResult"]:
        """Decode a WRITE_REGISTER result.

        [addr(u16), len, written(i32), readback(i32), verified].
        """
        if len(self.data) < 12:
            return None
        addr = struct.unpack_from("<H", self.data, 0)[0]
        length = self.data[2]
        written = struct.unpack_from("<i", self.data, 3)[0]
        readback = struct.unpack_from("<i", self.data, 7)[0]
        verified = self.data[11] != 0
        return DxlWriteRegisterResult(addr, length, written, readback, verified)

    def power(self) -> Optional["DxlPowerResult"]:
        """Decode a DONE DXL_POWER result ([power_on(1), has_control(1)])."""
        if not self.done or len(self.data) < 2:
            return None
        return DxlPowerResult(self.data[0] != 0, self.data[1] != 0)


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


@dataclass
class DxlRegisterValue:
    address: int
    length: int      # 1, 2, or 4 bytes
    value: int


@dataclass
class DxlWriteRegisterResult:
    address: int
    length: int
    written: int
    readback: int
    verified: bool


@dataclass
class DxlPowerResult:
    power_on: bool
    has_control: bool


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


# --- Config commands -------------------------------------------------------


def build_cfg_get_summary(seq: int = 0) -> bytes:
    """Build a CFG_GET_SUMMARY command (replies with a ConfigSummary)."""
    return build_command(MSG_CFG_GET_SUMMARY, seq=seq)


def build_cfg_get_block(offset: int, length: int, seq: int = 0) -> bytes:
    """Build a CFG_GET_BLOCK command for a window of the staged payload."""
    return build_command(
        MSG_CFG_GET_BLOCK, seq=seq, payload=struct.pack("<HH", offset, length)
    )


def build_cfg_set_block(offset: int, data: bytes, seq: int = 0) -> bytes:
    """Build a CFG_SET_BLOCK command writing ``data`` into the staging buffer."""
    return build_command(
        MSG_CFG_SET_BLOCK,
        seq=seq,
        payload=struct.pack("<HH", offset, len(data)) + bytes(data),
    )


def build_cfg_validate(seq: int = 0) -> bytes:
    """Build a CFG_VALIDATE command (validate the staged config)."""
    return build_command(MSG_CFG_VALIDATE, seq=seq)


def build_cfg_commit(seq: int = 0) -> bytes:
    """Build a CFG_COMMIT command (persist the staged config to EEPROM)."""
    return build_command(MSG_CFG_COMMIT, seq=seq)


def build_cfg_reset_defaults(seq: int = 0) -> bytes:
    """Build a CFG_RESET_DEFAULTS command (reload compiled defaults)."""
    return build_command(MSG_CFG_RESET_DEFAULTS, seq=seq)


@dataclass
class CfgBlockAck:
    """Decoded CFG_SET_BLOCK ack ([offset u16, len u16])."""

    offset: int
    length: int


def parse_cfg_block_ack(payload: bytes) -> CfgBlockAck:
    """Decode a CFG_SET_BLOCK ack payload ([offset u16, len u16])."""
    if len(payload) >= 4:
        offset, length = struct.unpack("<HH", payload[:4])
        return CfgBlockAck(offset, length)
    return CfgBlockAck(0, 0)


@dataclass
class CfgResult:
    """Decoded CFG_VALIDATE / CFG_COMMIT / CFG_RESET_DEFAULTS response."""

    result: int

    @property
    def ok(self) -> bool:
        return self.result == CFG_OK


def parse_cfg_result(payload: bytes) -> CfgResult:
    """Decode a single-byte config result payload (validate/commit/reset)."""
    return CfgResult(payload[0] if payload else CFG_VALIDATION_FAILED)

