"""HIL observer trace event decoding and ordered v1 record reassembly.

This module is intentionally independent of a serial transport. Callers retain
raw delimited frames first, decode them through :mod:`hexapod_protocol.framing`,
then pass HIL event payloads here for strict logical-record reconstruction.
"""

from __future__ import annotations

import struct
from collections.abc import Iterable
from dataclasses import dataclass, fields, is_dataclass
from enum import IntEnum

from .config import NUM_FOOT_SENSORS, NUM_LEGS, NUM_SERVOS, RobotConfig, decode_robot_config
from .crc16 import crc16
from .framing import DecodeError, MsgType, decode_frame_body

TRACE_SCHEMA_VERSION = 1
TRACE_FRAGMENT_EVENT = 0xA0
OUTPUT_BLOCKED_EVENT = 0xA1
FRAGMENT_PREFIX_BYTES = 21
MAX_EVENT_PAYLOAD = 256
MAX_FRAGMENT_DATA = MAX_EVENT_PAYLOAD - FRAGMENT_PREFIX_BYTES
MAX_LOGICAL_RECORD_BYTES = 960

_FRAGMENT_HEADER = struct.Struct("<HIIIBBBHH")


class TraceDecodeError(ValueError):
    """Raised when a HIL fragment or logical record violates the v1 contract."""


class RecordType(IntEnum):
    BEGIN = 0
    CONFIG = 1
    STEP = 2
    MARKER = 3
    OUTPUT_BLOCKED = 4
    END = 5


class TraceOrigin(IntEnum):
    ATSAMD21_OUTPUT_DISABLED = 1


class CaptureEndReason(IntEnum):
    COMPLETE = 0
    HOST_HEARTBEAT_EXPIRED = 1
    TRANSPORT_OVERFLOW = 2
    TRANSPORT_TIMEOUT = 3
    HOST_ABORTED = 4
    SESSION_CLOSED = 5


@dataclass(frozen=True)
class FragmentHeader:
    schema_version: int
    session_id: int
    capture_id: int
    record_seq: int
    record_type: RecordType
    fragment_index: int
    fragment_count: int
    logical_length: int
    logical_crc16: int


@dataclass(frozen=True)
class TraceFragment:
    header: FragmentHeader
    data: bytes


@dataclass(frozen=True)
class TraceRecord:
    """One CRC-verified logical HIL trace record."""

    header: FragmentHeader
    payload: bytes


@dataclass(frozen=True)
class OutputGuardStatus:
    output_disabled: bool
    power_guard_active: bool
    torque_guard_active: bool
    goal_guard_active: bool
    write_guard_active: bool
    blocked_power_enable: int
    blocked_torque_enable: int
    blocked_goal_write: int
    blocked_dxl_write: int
    last_goal_sequence: int
    last_goal_count: int


@dataclass(frozen=True)
class TraceBegin:
    schema_version: int
    origin: TraceOrigin
    protocol_major: int
    protocol_minor: int
    session_id: int
    capture_id: int
    config_revision: int
    config_payload_crc16: int
    requested_steps: int
    initial_guard: OutputGuardStatus


@dataclass(frozen=True)
class ConfigSnapshot:
    revision: int
    valid: bool
    persistent: bool
    payload_crc16: int
    payload: bytes
    robot_config: RobotConfig | None


@dataclass(frozen=True)
class TraceMarker:
    marker_id: int
    now_ms: int
    safety_state: int
    step_sequence: int


@dataclass(frozen=True)
class OutputBlocked:
    step_sequence: int
    guard: OutputGuardStatus


@dataclass(frozen=True)
class TraceEnd:
    reason: CaptureEndReason
    requested_steps: int
    recorded_steps: int
    queue_high_water: int
    emitted_record_count: int
    emitted_fragment_count: int
    final_guard: OutputGuardStatus


@dataclass(frozen=True)
class ControllerTime:
    now_ms: int
    dt_ms: int
    valid: bool


@dataclass(frozen=True)
class BatterySnapshot:
    millivolts: int
    valid: bool
    validity: int


@dataclass(frozen=True)
class DxlServoStatus:
    id: int
    present_position: int
    present_velocity: int
    present_load: int
    present_voltage_mv: int
    present_temperature_c: int
    hardware_error: int
    torque_enabled: bool
    ok: bool


@dataclass(frozen=True)
class DxlSnapshot:
    servo_count: int
    validity: int
    configured_servo_coverage: bool
    pose_known_mask: int
    config_revision: int
    torque_off: bool
    hard_fault: bool
    servos: tuple[DxlServoStatus, ...]


@dataclass(frozen=True)
class FootContactState:
    timestamp_ms: int
    proximity_raw: int
    pressure_raw: int
    pressure_baseline: int
    pressure_delta: int
    state: int
    confidence: int
    near_surface: bool
    touch: bool
    loaded: bool
    release: bool
    stale: bool
    fault: bool


@dataclass(frozen=True)
class ContactSnapshot:
    present_mask: int
    validity: int
    feet: tuple[FootContactState, ...]


@dataclass(frozen=True)
class RobotState:
    battery: BatterySnapshot
    dxl: DxlSnapshot
    contact: ContactSnapshot
    config_ready: bool
    watchdog_fault: bool


@dataclass(frozen=True)
class ControllerCommand:
    valid: bool
    failsafe: bool
    ever_seen: bool
    frame_ms: int
    arm_request: bool
    estop: bool
    host_authority: bool
    mode: int
    gait_index: int
    twist_vx: float
    twist_vy: float
    twist_wz: float
    pose_x_mm: float
    pose_y_mm: float
    pose_z_mm: float
    pose_roll: float
    pose_pitch: float
    pose_yaw: float
    trim_roll: float
    trim_pitch: float
    speed: float
    body_height: float
    stride: float
    step_height: float
    feat_foot_contact: bool
    feat_terrain_leveling: bool
    feat_passive_pose: bool
    trick: int


@dataclass(frozen=True)
class RcIntent:
    command: ControllerCommand
    ever_seen: bool
    kill: bool
    armed: bool
    failsafe: bool
    autonomy_enabled: bool


@dataclass(frozen=True)
class MotionIntent:
    seq: int
    gait: int
    body_height_mm: int
    stride_len_mm: int
    step_height_mm: int
    duty_x255: int
    speed_x255: int
    twist_vx: float
    twist_vy: float
    twist_wz: float
    pose_x_mm: float
    pose_y_mm: float
    pose_z_mm: float
    pose_roll: float
    pose_pitch: float
    pose_yaw: float


@dataclass(frozen=True)
class MaintenanceLegTarget:
    foot_x_mm: int
    foot_y_mm: int
    foot_z_mm: int
    target_set: bool
    reachable: bool
    clamped: bool


@dataclass(frozen=True)
class MaintenanceTargets:
    seq: int
    tick: tuple[int, ...]
    set: tuple[bool, ...]
    clamped: tuple[bool, ...]
    legs: tuple[MaintenanceLegTarget, ...]


@dataclass(frozen=True)
class MaintenanceIntent:
    lock_held: bool
    lock_token: int
    control_mode: int
    targets: MaintenanceTargets


@dataclass(frozen=True)
class FeatureSnapshot:
    foot_contact_enabled: bool
    terrain_leveling_enabled: bool
    sensor_polling_enabled: bool
    jetson_control_enabled: bool
    passive_pose_enabled: bool


@dataclass(frozen=True)
class ControllerIntent:
    rc: RcIntent
    motion: MotionIntent
    maintenance: MaintenanceIntent
    features: FeatureSnapshot
    host_estop: bool
    host_disarm: bool
    clear_fault_requested: bool
    passive_requested: bool
    jetson_heartbeat_received: bool


@dataclass(frozen=True)
class ControllerConfigSnapshot:
    revision: int
    valid: bool
    persistent: bool


@dataclass(frozen=True)
class ControllerDiagnostics:
    config_revision: int
    intent_sequence: int
    confident_contact_feet: int
    motion_gate_rising: bool
    motion_gate_falling: bool
    config_reapplied: bool
    maintenance_session_started: bool
    clear_maintenance_targets: bool
    clear_maintenance_lock: bool
    clear_passive_request: bool
    any_goal_clamped: bool
    any_goal_unreachable: bool
    any_goal_reach_limited: bool


@dataclass(frozen=True)
class PipelineJoint:
    id: int
    tick: int
    leg: int
    joint: int
    clamped: bool


@dataclass(frozen=True)
class CommandObservation:
    safety_state: int
    fault_reason: int
    command_source: int
    motion_authorized: bool
    motion_gate: bool
    allow_dxl_power: bool
    allow_torque: bool
    goal_valid: bool
    diagnostics: ControllerDiagnostics
    goals: tuple[PipelineJoint, ...]
    any_unreachable: bool
    any_reach_limited: bool


@dataclass(frozen=True)
class ControllerStep:
    step_sequence: int
    time: ControllerTime
    state: RobotState
    intent: ControllerIntent
    config: ControllerConfigSnapshot
    observed: CommandObservation


ParsedRecord = TraceBegin | ConfigSnapshot | ControllerStep | TraceMarker | OutputBlocked | TraceEnd


@dataclass(frozen=True)
class HILTrace:
    """One complete, ordered capture ready for native parity reconstruction."""

    begin: TraceBegin
    config: ConfigSnapshot
    steps: tuple[ControllerStep, ...]
    markers: tuple[TraceMarker, ...]
    output_blocked: tuple[OutputBlocked, ...]
    end: TraceEnd
    records: tuple[ParsedRecord, ...]


def trace_to_artifact(trace: HILTrace) -> dict[str, object]:
    """Convert a parsed trace into deterministic JSON-compatible data.

    Canonical config payload bytes are retained as ``payload_hex`` rather than
    relying on a lossy text conversion or Python's non-serializable ``bytes``.
    The result is intended for an offline parity artifact, not for re-encoding
    a firmware trace.
    """
    converted = _artifact_value(trace)
    if not isinstance(converted, dict):  # pragma: no cover - type invariant
        raise TypeError("trace artifact root must be a dataclass")
    return converted


def _artifact_value(value: object) -> object:
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, IntEnum):
        return int(value)
    if is_dataclass(value) and not isinstance(value, type):
        artifact: dict[str, object] = {}
        for field in fields(value):
            field_value = getattr(value, field.name)
            field_name = f"{field.name}_hex" if isinstance(field_value, bytes) else field.name
            artifact[field_name] = _artifact_value(field_value)
        return artifact
    if isinstance(value, tuple | list):
        return [_artifact_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _artifact_value(item) for key, item in value.items()}
    return value


class TraceAssembler:
    """Validate an ordered HIL event stream into one complete capture.

    A partial capture or one with a terminal reason other than ``COMPLETE`` can
    be inspected, but consumers must reject it for controller parity.
    """

    def __init__(self, expected_session_id: int | None = None) -> None:
        self._reassembler = TraceReassembler(expected_session_id)
        self._expected_session_id = expected_session_id
        self.reset()

    def reset(self) -> None:
        self._begin: TraceBegin | None = None
        self._config: ConfigSnapshot | None = None
        self._end: TraceEnd | None = None
        self._next_record_seq = 1
        self._records: list[ParsedRecord] = []
        self._steps: list[ControllerStep] = []
        self._markers: list[TraceMarker] = []
        self._output_blocked: list[OutputBlocked] = []
        self._fragments_before_end = 0

    @property
    def complete(self) -> bool:
        return self._end is not None

    def accept_event(self, event_id: int, payload: bytes) -> ParsedRecord | None:
        """Accept one validated-frame HIL event payload.

        Callers must pass only the frame payload after verifying the normal wire
        frame's COBS and CRC. The event ID is checked against the logical record
        type so output-block evidence cannot be silently mislabeled.
        """
        if self._end is not None:
            raise TraceDecodeError("trace contains an event after TraceEnd")
        if event_id not in (TRACE_FRAGMENT_EVENT, OUTPUT_BLOCKED_EVENT):
            raise TraceDecodeError(f"unsupported HIL event ID 0x{event_id:02X}")
        fragment = decode_fragment(payload)
        if fragment.header.record_type is RecordType.OUTPUT_BLOCKED:
            if event_id != OUTPUT_BLOCKED_EVENT:
                raise TraceDecodeError("OutputBlocked record used the wrong event ID")
        elif event_id != TRACE_FRAGMENT_EVENT:
            raise TraceDecodeError("non-output trace record used the wrong event ID")
        record = self._reassembler.accept(fragment)
        if record is None:
            return None
        parsed = parse_record(record)
        self._accept_record(record, parsed)
        return parsed

    def _accept_record(self, record: TraceRecord, parsed: ParsedRecord) -> None:
        header = record.header
        if header.record_seq != self._next_record_seq:
            raise TraceDecodeError("trace record sequence is not contiguous")
        self._next_record_seq += 1
        if self._begin is None:
            if not isinstance(parsed, TraceBegin):
                raise TraceDecodeError("TraceBegin must be the first logical record")
            self._begin = parsed
            if self._expected_session_id is not None and parsed.session_id != self._expected_session_id:
                raise TraceDecodeError("TraceBegin belongs to another session")
        else:
            if header.session_id != self._begin.session_id or header.capture_id != self._begin.capture_id:
                raise TraceDecodeError("trace record belongs to another capture")
            if isinstance(parsed, TraceBegin):
                raise TraceDecodeError("trace contains multiple TraceBegin records")
            self._accept_following_record(parsed)
        self._records.append(parsed)
        if not isinstance(parsed, TraceEnd):
            self._fragments_before_end += header.fragment_count

    def _accept_following_record(self, parsed: ParsedRecord) -> None:
        assert self._begin is not None
        if isinstance(parsed, ConfigSnapshot):
            if self._config is not None or self._steps:
                raise TraceDecodeError("ConfigSnapshot must appear exactly once before steps")
            if parsed.revision != self._begin.config_revision:
                raise TraceDecodeError("ConfigSnapshot revision does not match TraceBegin")
            if parsed.payload_crc16 != self._begin.config_payload_crc16:
                raise TraceDecodeError("ConfigSnapshot CRC does not match TraceBegin")
            self._config = parsed
        elif isinstance(parsed, ControllerStep):
            if self._config is None:
                raise TraceDecodeError("ControllerStep appeared before ConfigSnapshot")
            if (
                parsed.config.revision != self._config.revision
                or parsed.config.valid != self._config.valid
                or parsed.config.persistent != self._config.persistent
            ):
                raise TraceDecodeError("ControllerStep config reference does not match ConfigSnapshot")
            expected_step = len(self._steps) + 1
            if parsed.step_sequence != expected_step:
                raise TraceDecodeError("ControllerStep sequence is not contiguous")
            if len(self._steps) >= self._begin.requested_steps:
                raise TraceDecodeError("trace recorded more steps than requested")
            self._steps.append(parsed)
        elif isinstance(parsed, TraceMarker):
            self._require_existing_step(parsed.step_sequence, "Marker")
            self._markers.append(parsed)
        elif isinstance(parsed, OutputBlocked):
            self._require_existing_step(parsed.step_sequence, "OutputBlocked")
            self._output_blocked.append(parsed)
        elif isinstance(parsed, TraceEnd):
            if self._config is None:
                raise TraceDecodeError("TraceEnd appeared before ConfigSnapshot")
            self._validate_end(parsed)
            self._end = parsed
        else:  # pragma: no cover - exhaustive union guard
            raise TraceDecodeError("unknown parsed trace record")

    def _require_existing_step(self, step_sequence: int, record_name: str) -> None:
        if not 1 <= step_sequence <= len(self._steps):
            raise TraceDecodeError(f"{record_name} does not refer to a recorded controller step")

    def _validate_end(self, end: TraceEnd) -> None:
        assert self._begin is not None
        if end.requested_steps != self._begin.requested_steps:
            raise TraceDecodeError("TraceEnd requested count does not match TraceBegin")
        if end.recorded_steps != len(self._steps):
            raise TraceDecodeError("TraceEnd recorded count does not match ControllerStep records")
        if end.emitted_record_count != len(self._records):
            raise TraceDecodeError("TraceEnd logical record counter is inconsistent")
        if end.emitted_fragment_count != self._fragments_before_end:
            raise TraceDecodeError("TraceEnd fragment counter is inconsistent")
        if end.reason is CaptureEndReason.COMPLETE and end.recorded_steps != end.requested_steps:
            raise TraceDecodeError("completed trace does not contain all requested steps")

    def finalize(self) -> HILTrace:
        """Return the fully assembled trace or reject an incomplete capture."""
        if self._begin is None or self._config is None or self._end is None:
            raise TraceDecodeError("trace is incomplete")
        return HILTrace(
            begin=self._begin,
            config=self._config,
            steps=tuple(self._steps),
            markers=tuple(self._markers),
            output_blocked=tuple(self._output_blocked),
            end=self._end,
            records=tuple(self._records),
        )


def decode_trace_frames(
    frames: Iterable[bytes], expected_session_id: int | None = None
) -> HILTrace:
    """Decode one complete HIL capture from retained delimited wire frames.

    Normal command, response, and telemetry frames are ignored. Any malformed
    frame is likewise ignored because a lossless raw session may intentionally
    retain unrelated torn USB frames; malformed *HIL* fragments still fail
    closed once their valid frame envelope is available to this decoder.
    """
    assembler = TraceAssembler(expected_session_id=expected_session_id)
    hIL_events_seen = 0
    for frame in frames:
        if len(frame) < 2 or frame[0] != 0 or frame[-1] != 0:
            continue
        try:
            header, payload = decode_frame_body(frame[1:-1])
        except DecodeError:
            continue
        if header.msg_type != int(MsgType.EVENT):
            continue
        if header.msg_id not in (TRACE_FRAGMENT_EVENT, OUTPUT_BLOCKED_EVENT):
            continue
        hIL_events_seen += 1
        assembler.accept_event(header.msg_id, payload)
    if hIL_events_seen == 0:
        raise TraceDecodeError("raw frame set contains no HIL trace events")
    return assembler.finalize()


def _validate_header(header: FragmentHeader) -> None:
    if header.schema_version != TRACE_SCHEMA_VERSION:
        raise TraceDecodeError(
            f"unsupported trace schema version {header.schema_version}"
        )
    if header.session_id == 0 or header.capture_id == 0:
        raise TraceDecodeError("session_id and capture_id must be nonzero")
    if header.fragment_count == 0 or header.fragment_index >= header.fragment_count:
        raise TraceDecodeError("fragment index/count is invalid")
    if not 0 < header.logical_length <= MAX_LOGICAL_RECORD_BYTES:
        raise TraceDecodeError("logical record length is invalid")
    expected_count = (header.logical_length + MAX_FRAGMENT_DATA - 1) // MAX_FRAGMENT_DATA
    if header.fragment_count != expected_count:
        raise TraceDecodeError("fragment count does not match logical record length")


class _Reader:
    """Bounds-checked little-endian reader for canonical logical records."""

    def __init__(self, data: bytes) -> None:
        self._data = data
        self._offset = 0

    @property
    def remaining(self) -> int:
        return len(self._data) - self._offset

    def _take(self, size: int) -> bytes:
        if size > self.remaining:
            raise TraceDecodeError("logical record is truncated")
        data = self._data[self._offset : self._offset + size]
        self._offset += size
        return data

    def u8(self) -> int:
        return self._take(1)[0]

    def boolean(self) -> bool:
        value = self.u8()
        if value not in (0, 1):
            raise TraceDecodeError("canonical boolean is not 0 or 1")
        return bool(value)

    def u16(self) -> int:
        return struct.unpack("<H", self._take(2))[0]

    def i8(self) -> int:
        return struct.unpack("<b", self._take(1))[0]

    def i16(self) -> int:
        return struct.unpack("<h", self._take(2))[0]

    def u32(self) -> int:
        return struct.unpack("<I", self._take(4))[0]

    def i32(self) -> int:
        return struct.unpack("<i", self._take(4))[0]

    def f32(self) -> float:
        return struct.unpack("<f", self._take(4))[0]

    def bytes(self, length: int) -> bytes:
        return self._take(length)

    def finish(self) -> None:
        if self.remaining:
            raise TraceDecodeError("logical record has trailing bytes")


def _read_guard(reader: _Reader) -> OutputGuardStatus:
    return OutputGuardStatus(
        output_disabled=reader.boolean(),
        power_guard_active=reader.boolean(),
        torque_guard_active=reader.boolean(),
        goal_guard_active=reader.boolean(),
        write_guard_active=reader.boolean(),
        blocked_power_enable=reader.u32(),
        blocked_torque_enable=reader.u32(),
        blocked_goal_write=reader.u32(),
        blocked_dxl_write=reader.u32(),
        last_goal_sequence=reader.u32(),
        last_goal_count=reader.u8(),
    )


def _read_controller_command(reader: _Reader) -> ControllerCommand:
    return ControllerCommand(
        valid=reader.boolean(),
        failsafe=reader.boolean(),
        ever_seen=reader.boolean(),
        frame_ms=reader.u32(),
        arm_request=reader.boolean(),
        estop=reader.boolean(),
        host_authority=reader.boolean(),
        mode=reader.u8(),
        gait_index=reader.u8(),
        twist_vx=reader.f32(),
        twist_vy=reader.f32(),
        twist_wz=reader.f32(),
        pose_x_mm=reader.f32(),
        pose_y_mm=reader.f32(),
        pose_z_mm=reader.f32(),
        pose_roll=reader.f32(),
        pose_pitch=reader.f32(),
        pose_yaw=reader.f32(),
        trim_roll=reader.f32(),
        trim_pitch=reader.f32(),
        speed=reader.f32(),
        body_height=reader.f32(),
        stride=reader.f32(),
        step_height=reader.f32(),
        feat_foot_contact=reader.boolean(),
        feat_terrain_leveling=reader.boolean(),
        feat_passive_pose=reader.boolean(),
        trick=reader.u8(),
    )


def _read_robot_state(reader: _Reader) -> RobotState:
    battery = BatterySnapshot(
        millivolts=reader.u16(),
        valid=reader.boolean(),
        validity=reader.u8(),
    )
    servo_count = reader.u8()
    if servo_count > NUM_SERVOS:
        raise TraceDecodeError("DXL servo count exceeds the portable contract")
    dxl = DxlSnapshot(
        servo_count=servo_count,
        validity=reader.u8(),
        configured_servo_coverage=reader.boolean(),
        pose_known_mask=reader.u32(),
        config_revision=reader.u32(),
        torque_off=reader.boolean(),
        hard_fault=reader.boolean(),
        servos=tuple(
            DxlServoStatus(
                id=reader.u8(),
                present_position=reader.i32(),
                present_velocity=reader.i32(),
                present_load=reader.i32(),
                present_voltage_mv=reader.u16(),
                present_temperature_c=reader.i8(),
                hardware_error=reader.u8(),
                torque_enabled=reader.boolean(),
                ok=reader.boolean(),
            )
            for _ in range(NUM_SERVOS)
        ),
    )
    contact = ContactSnapshot(
        present_mask=reader.u8(),
        validity=reader.u8(),
        feet=tuple(
            FootContactState(
                timestamp_ms=reader.u32(),
                proximity_raw=reader.u16(),
                pressure_raw=reader.i32(),
                pressure_baseline=reader.i32(),
                pressure_delta=reader.i32(),
                state=reader.u8(),
                confidence=reader.u8(),
                near_surface=reader.boolean(),
                touch=reader.boolean(),
                loaded=reader.boolean(),
                release=reader.boolean(),
                stale=reader.boolean(),
                fault=reader.boolean(),
            )
            for _ in range(NUM_FOOT_SENSORS)
        ),
    )
    return RobotState(
        battery=battery,
        dxl=dxl,
        contact=contact,
        config_ready=reader.boolean(),
        watchdog_fault=reader.boolean(),
    )


def _read_motion_intent(reader: _Reader) -> MotionIntent:
    return MotionIntent(
        seq=reader.u32(),
        gait=reader.u8(),
        body_height_mm=reader.u16(),
        stride_len_mm=reader.u16(),
        step_height_mm=reader.u16(),
        duty_x255=reader.u8(),
        speed_x255=reader.u8(),
        twist_vx=reader.f32(),
        twist_vy=reader.f32(),
        twist_wz=reader.f32(),
        pose_x_mm=reader.f32(),
        pose_y_mm=reader.f32(),
        pose_z_mm=reader.f32(),
        pose_roll=reader.f32(),
        pose_pitch=reader.f32(),
        pose_yaw=reader.f32(),
    )


def _read_maintenance_intent(reader: _Reader) -> MaintenanceIntent:
    lock_held = reader.boolean()
    lock_token = reader.u32()
    control_mode = reader.u8()
    targets = MaintenanceTargets(
        seq=reader.u32(),
        tick=tuple(reader.u16() for _ in range(NUM_SERVOS)),
        set=tuple(reader.boolean() for _ in range(NUM_SERVOS)),
        clamped=tuple(reader.boolean() for _ in range(NUM_SERVOS)),
        legs=tuple(
            MaintenanceLegTarget(
                foot_x_mm=reader.i16(),
                foot_y_mm=reader.i16(),
                foot_z_mm=reader.i16(),
                target_set=reader.boolean(),
                reachable=reader.boolean(),
                clamped=reader.boolean(),
            )
            for _ in range(NUM_LEGS)
        ),
    )
    return MaintenanceIntent(lock_held, lock_token, control_mode, targets)


def _read_controller_intent(reader: _Reader) -> ControllerIntent:
    rc = RcIntent(
        command=_read_controller_command(reader),
        ever_seen=reader.boolean(),
        kill=reader.boolean(),
        armed=reader.boolean(),
        failsafe=reader.boolean(),
        autonomy_enabled=reader.boolean(),
    )
    motion = _read_motion_intent(reader)
    maintenance = _read_maintenance_intent(reader)
    features = FeatureSnapshot(
        foot_contact_enabled=reader.boolean(),
        terrain_leveling_enabled=reader.boolean(),
        sensor_polling_enabled=reader.boolean(),
        jetson_control_enabled=reader.boolean(),
        passive_pose_enabled=reader.boolean(),
    )
    return ControllerIntent(
        rc=rc,
        motion=motion,
        maintenance=maintenance,
        features=features,
        host_estop=reader.boolean(),
        host_disarm=reader.boolean(),
        clear_fault_requested=reader.boolean(),
        passive_requested=reader.boolean(),
        jetson_heartbeat_received=reader.boolean(),
    )


def _read_command_observation(reader: _Reader) -> CommandObservation:
    safety_state = reader.u8()
    fault_reason = reader.u8()
    command_source = reader.u8()
    motion_authorized = reader.boolean()
    motion_gate = reader.boolean()
    allow_dxl_power = reader.boolean()
    allow_torque = reader.boolean()
    goal_valid = reader.boolean()
    diagnostics = ControllerDiagnostics(
        config_revision=reader.u32(),
        intent_sequence=reader.u32(),
        confident_contact_feet=reader.u8(),
        motion_gate_rising=reader.boolean(),
        motion_gate_falling=reader.boolean(),
        config_reapplied=reader.boolean(),
        maintenance_session_started=reader.boolean(),
        clear_maintenance_targets=reader.boolean(),
        clear_maintenance_lock=reader.boolean(),
        clear_passive_request=reader.boolean(),
        any_goal_clamped=reader.boolean(),
        any_goal_unreachable=reader.boolean(),
        any_goal_reach_limited=reader.boolean(),
    )
    goal_count = reader.u8()
    any_unreachable = reader.boolean()
    any_reach_limited = reader.boolean()
    if goal_count > NUM_SERVOS:
        raise TraceDecodeError("ControllerStep goal count exceeds the portable contract")
    goals = tuple(
        PipelineJoint(
            id=reader.u8(),
            tick=reader.u16(),
            leg=reader.u8(),
            joint=reader.u8(),
            clamped=reader.boolean(),
        )
        for _ in range(goal_count)
    )
    return CommandObservation(
        safety_state=safety_state,
        fault_reason=fault_reason,
        command_source=command_source,
        motion_authorized=motion_authorized,
        motion_gate=motion_gate,
        allow_dxl_power=allow_dxl_power,
        allow_torque=allow_torque,
        goal_valid=goal_valid,
        diagnostics=diagnostics,
        goals=goals,
        any_unreachable=any_unreachable,
        any_reach_limited=any_reach_limited,
    )


def _read_controller_step(reader: _Reader) -> ControllerStep:
    step_sequence = reader.u32()
    time = ControllerTime(now_ms=reader.u32(), dt_ms=reader.u32(), valid=reader.boolean())
    state = _read_robot_state(reader)
    intent = _read_controller_intent(reader)
    config = ControllerConfigSnapshot(
        revision=reader.u32(), valid=reader.boolean(), persistent=reader.boolean()
    )
    return ControllerStep(
        step_sequence=step_sequence,
        time=time,
        state=state,
        intent=intent,
        config=config,
        observed=_read_command_observation(reader),
    )


def parse_record(record: TraceRecord):
    """Parse a CRC-verified HIL v1 logical record into a typed value.

    The result contains only canonical values captured immediately around one
    controller step. A later trace assembler resolves a step's config reference
    against its preceding :class:`ConfigSnapshot` record.
    """
    if crc16(record.payload) != record.header.logical_crc16:
        raise TraceDecodeError("logical record CRC mismatch")
    reader = _Reader(record.payload)
    record_type = record.header.record_type
    if record_type is RecordType.BEGIN:
        schema_version = reader.u16()
        if schema_version != TRACE_SCHEMA_VERSION:
            raise TraceDecodeError("TraceBegin has an unsupported schema version")
        try:
            origin = TraceOrigin(reader.u8())
        except ValueError as exc:
            raise TraceDecodeError("TraceBegin has an unknown capture origin") from exc
        parsed = TraceBegin(
            schema_version=schema_version,
            origin=origin,
            protocol_major=reader.u8(),
            protocol_minor=reader.u8(),
            session_id=reader.u32(),
            capture_id=reader.u32(),
            config_revision=reader.u32(),
            config_payload_crc16=reader.u16(),
            requested_steps=reader.u8(),
            initial_guard=_read_guard(reader),
        )
        if parsed.session_id != record.header.session_id:
            raise TraceDecodeError("TraceBegin session ID does not match envelope")
        if parsed.capture_id != record.header.capture_id:
            raise TraceDecodeError("TraceBegin capture ID does not match envelope")
        if not 1 <= parsed.requested_steps <= 32:
            raise TraceDecodeError("TraceBegin requested step count is invalid")
    elif record_type is RecordType.CONFIG:
        revision = reader.u32()
        valid = reader.boolean()
        persistent = reader.boolean()
        payload_length = reader.u16()
        payload_crc16 = reader.u16()
        payload = reader.bytes(payload_length)
        if payload_length > MAX_LOGICAL_RECORD_BYTES - 10:
            raise TraceDecodeError("ConfigSnapshot payload is too large")
        if valid:
            if crc16(payload) != payload_crc16:
                raise TraceDecodeError("ConfigSnapshot payload CRC mismatch")
            try:
                robot_config = decode_robot_config(payload)
            except ValueError as exc:
                raise TraceDecodeError(f"ConfigSnapshot is invalid: {exc}") from exc
        elif payload or payload_crc16:
            raise TraceDecodeError("invalid ConfigSnapshot must have an empty payload")
        else:
            robot_config = None
        parsed = ConfigSnapshot(
            revision=revision,
            valid=valid,
            persistent=persistent,
            payload_crc16=payload_crc16,
            payload=payload,
            robot_config=robot_config,
        )
    elif record_type is RecordType.MARKER:
        parsed = TraceMarker(
            marker_id=reader.u32(),
            now_ms=reader.u32(),
            safety_state=reader.u8(),
            step_sequence=reader.u32(),
        )
    elif record_type is RecordType.OUTPUT_BLOCKED:
        parsed = OutputBlocked(step_sequence=reader.u32(), guard=_read_guard(reader))
    elif record_type is RecordType.END:
        try:
            reason = CaptureEndReason(reader.u8())
        except ValueError as exc:
            raise TraceDecodeError("TraceEnd has an unknown reason") from exc
        parsed = TraceEnd(
            reason=reason,
            requested_steps=reader.u8(),
            recorded_steps=reader.u8(),
            queue_high_water=reader.u8(),
            emitted_record_count=reader.u32(),
            emitted_fragment_count=reader.u32(),
            final_guard=_read_guard(reader),
        )
        if parsed.recorded_steps > parsed.requested_steps:
            raise TraceDecodeError("TraceEnd recorded step count exceeds requested count")
    elif record_type is RecordType.STEP:
        parsed = _read_controller_step(reader)
    else:
        raise TraceDecodeError("unknown trace record type")
    reader.finish()
    return parsed


def decode_fragment(payload: bytes) -> TraceFragment:
    """Parse and validate a single HIL v1 event payload.

    Frame COBS/CRC validation belongs to :func:`framing.decode_frame_body`.
    This function validates the HIL fragment envelope and leaves cross-fragment
    ordering and logical CRC verification to :class:`TraceReassembler`.
    """
    if len(payload) < FRAGMENT_PREFIX_BYTES:
        raise TraceDecodeError("trace fragment is shorter than its prefix")
    if len(payload) > MAX_EVENT_PAYLOAD:
        raise TraceDecodeError("trace fragment exceeds the event payload limit")

    (
        schema_version,
        session_id,
        capture_id,
        record_seq,
        record_type,
        fragment_index,
        fragment_count,
        logical_length,
        logical_crc16,
    ) = _FRAGMENT_HEADER.unpack_from(payload)
    try:
        typed_record = RecordType(record_type)
    except ValueError as exc:
        raise TraceDecodeError(f"unknown trace record type {record_type}") from exc

    header = FragmentHeader(
        schema_version=schema_version,
        session_id=session_id,
        capture_id=capture_id,
        record_seq=record_seq,
        record_type=typed_record,
        fragment_index=fragment_index,
        fragment_count=fragment_count,
        logical_length=logical_length,
        logical_crc16=logical_crc16,
    )
    _validate_header(header)
    data = payload[FRAGMENT_PREFIX_BYTES:]
    if not data:
        raise TraceDecodeError("trace fragment has no logical-record data")

    expected_length = min(
        MAX_FRAGMENT_DATA,
        header.logical_length - header.fragment_index * MAX_FRAGMENT_DATA,
    )
    if len(data) != expected_length:
        raise TraceDecodeError("trace fragment data length is inconsistent")
    return TraceFragment(header=header, data=data)


class TraceReassembler:
    """Strictly reassemble one ordered stream of HIL v1 trace fragments."""

    def __init__(self, expected_session_id: int | None = None) -> None:
        if expected_session_id is not None and expected_session_id == 0:
            raise ValueError("expected_session_id must be nonzero when provided")
        self._expected_session_id = expected_session_id
        self.reset()

    @property
    def active(self) -> bool:
        return self._header is not None

    def reset(self) -> None:
        self._header: FragmentHeader | None = None
        self._next_fragment = 0
        self._record = bytearray()

    def accept(self, fragment: TraceFragment) -> TraceRecord | None:
        """Accept one fragment, returning a record only when it completes.

        The state resets after a terminal reassembly error so later, complete
        records can still be processed from a retained raw capture.
        """
        header = fragment.header
        try:
            if (
                self._expected_session_id is not None
                and header.session_id != self._expected_session_id
            ):
                raise TraceDecodeError("trace fragment belongs to another session")
            if self._header is None:
                if header.fragment_index != 0:
                    raise TraceDecodeError("first trace fragment is out of order")
                self._header = header
                self._next_fragment = 0
                self._record = bytearray()
            elif not _same_record(self._header, header):
                raise TraceDecodeError("trace fragment belongs to another logical record")

            if header.fragment_index != self._next_fragment:
                raise TraceDecodeError("trace fragment is duplicated or out of order")
            if len(self._record) + len(fragment.data) > header.logical_length:
                raise TraceDecodeError("trace fragments exceed logical record length")

            self._record.extend(fragment.data)
            self._next_fragment += 1
            if self._next_fragment < header.fragment_count:
                return None

            payload = bytes(self._record)
            if len(payload) != header.logical_length:
                raise TraceDecodeError("trace record is truncated")
            if crc16(payload) != header.logical_crc16:
                raise TraceDecodeError("trace record CRC mismatch")
            completed_header = self._header
            if completed_header is None:  # pragma: no cover - defensive invariant
                raise TraceDecodeError("trace reassembler lost its record header")
            return TraceRecord(header=completed_header, payload=payload)
        except TraceDecodeError:
            self.reset()
            raise
        finally:
            if self._header is not None and self._next_fragment == header.fragment_count:
                self.reset()


def _same_record(left: FragmentHeader, right: FragmentHeader) -> bool:
    return (
        left.schema_version == right.schema_version
        and left.session_id == right.session_id
        and left.capture_id == right.capture_id
        and left.record_seq == right.record_seq
        and left.record_type == right.record_type
        and left.fragment_count == right.fragment_count
        and left.logical_length == right.logical_length
        and left.logical_crc16 == right.logical_crc16
    )