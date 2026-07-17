"""HIL trace v1 fragment decoding and strict reassembly tests."""

from __future__ import annotations

import struct

import pytest

from hexapod_protocol import api
from hexapod_protocol.config import default_robot_config, encode_robot_config
from hexapod_protocol.crc16 import crc16
from hexapod_protocol.framing import Header, MsgType, encode_frame
from hexapod_protocol.hil import (
    FRAGMENT_PREFIX_BYTES,
    CaptureEndReason,
    ConfigSnapshot,
    FragmentHeader,
    MAX_FRAGMENT_DATA,
    OutputBlocked,
    RecordType,
    ControllerStep,
    HILTrace,
    TraceBegin,
    TraceDecodeError,
    TraceEnd,
    TraceMarker,
    TraceOrigin,
    TraceRecord,
    TraceAssembler,
    TraceReassembler,
    decode_fragment,
    decode_trace_frames,
    parse_record,
    trace_to_artifact,
)


def _fragment_payloads(
    logical: bytes,
    *,
    session_id: int = 7,
    capture_id: int = 11,
    record_seq: int = 3,
    record_type: RecordType = RecordType.STEP,
) -> list[bytes]:
    fragment_count = (len(logical) + MAX_FRAGMENT_DATA - 1) // MAX_FRAGMENT_DATA
    prefix = struct.Struct("<HIIIBBBHH")
    logical_crc = crc16(logical)
    return [
        prefix.pack(
            1,
            session_id,
            capture_id,
            record_seq,
            int(record_type),
            index,
            fragment_count,
            len(logical),
            logical_crc,
        )
        + logical[index * MAX_FRAGMENT_DATA : (index + 1) * MAX_FRAGMENT_DATA]
        for index in range(fragment_count)
    ]


def test_hil_trace_fragment_prefix_size_is_stable():
    assert FRAGMENT_PREFIX_BYTES == struct.calcsize("<HIIIBBBHH") == 21


def test_hil_trace_reassembles_ordered_fragmented_record():
    logical = bytes(index & 0xFF for index in range(MAX_FRAGMENT_DATA + 29))
    reassembler = TraceReassembler(expected_session_id=7)

    records = [
        record
        for payload in _fragment_payloads(logical)
        if (record := reassembler.accept(decode_fragment(payload))) is not None
    ]

    assert len(records) == 1
    assert records[0].header.record_type is RecordType.STEP
    assert records[0].header.fragment_index == 0
    assert records[0].payload == logical
    assert reassembler.active is False


def test_hil_trace_rejects_reordered_or_wrong_session_fragments():
    logical = bytes(index & 0xFF for index in range(MAX_FRAGMENT_DATA + 1))
    first, second = _fragment_payloads(logical)

    reassembler = TraceReassembler(expected_session_id=7)
    with pytest.raises(TraceDecodeError, match="out of order"):
        reassembler.accept(decode_fragment(second))
    assert reassembler.active is False

    reassembler.accept(decode_fragment(first))
    wrong_session = _fragment_payloads(logical, session_id=8)[1]
    with pytest.raises(TraceDecodeError, match="another session"):
        reassembler.accept(decode_fragment(wrong_session))
    assert reassembler.active is False


def test_hil_trace_rejects_corrupt_or_truncated_records():
    logical = bytes(index & 0xFF for index in range(MAX_FRAGMENT_DATA + 3))
    first, second = _fragment_payloads(logical)

    corrupt = bytearray(second)
    corrupt[-1] ^= 0x01
    reassembler = TraceReassembler()
    reassembler.accept(decode_fragment(first))
    with pytest.raises(TraceDecodeError, match="CRC"):
        reassembler.accept(decode_fragment(bytes(corrupt)))
    assert reassembler.active is False

    truncated = first[:-1]
    with pytest.raises(TraceDecodeError, match="data length"):
        decode_fragment(truncated)


def test_hil_observer_command_builders_have_exact_token_layouts():
    token = 0x44332211
    marker = 0x88776655
    builders = (
        (api.build_hil_open_session(token, seq=1), api.MSG_HIL_OPEN_SESSION, token.to_bytes(4, "little")),
        (api.build_hil_close_session(token, seq=2), api.MSG_HIL_CLOSE_SESSION, token.to_bytes(4, "little")),
        (api.build_hil_heartbeat(token, seq=3), api.MSG_HIL_HEARTBEAT, token.to_bytes(4, "little")),
        (api.build_hil_capture(token, 32, seq=4), api.MSG_HIL_CAPTURE, token.to_bytes(4, "little") + b"\x20"),
        (api.build_hil_abort_capture(token, seq=5), api.MSG_HIL_ABORT_CAPTURE, token.to_bytes(4, "little")),
        (api.build_hil_mark(token, marker, seq=6), api.MSG_HIL_MARK, struct.pack("<II", token, marker)),
        (api.build_hil_get_session_status(token, seq=7), api.MSG_HIL_GET_SESSION_STATUS, token.to_bytes(4, "little")),
    )
    for wire, msg_id, expected_payload in builders:
        header, payload = api.parse_response(wire)
        assert header.msg_id == msg_id
        assert payload == expected_payload

    header, payload = api.parse_response(api.build_hil_get_capability(seq=8))
    assert header.msg_id == api.MSG_HIL_GET_CAPABILITY
    assert payload == b""
    with pytest.raises(ValueError, match="1..32"):
        api.build_hil_capture(token, 0)


def test_hil_observer_response_parsers_match_firmware_layouts():
    opened = api.parse_hil_open_session(struct.pack("<BIIHB", 0, 7, 9, 1, 0x0F))
    assert opened.ok
    assert (opened.session_id, opened.session_token) == (7, 9)
    assert opened.trace_schema_version == 1
    assert opened.capability_mask == 0x0F

    capture = api.parse_hil_capture(struct.pack("<BI", 0, 12))
    assert capture.ok and capture.capture_id == 12

    status = api.parse_hil_session_status(
        struct.pack("<BBBBIIIHB", 0, 1, 1, 1, 7, 12, 1000, 1, 1)
    )
    assert status.ok
    assert status.session_open and status.capture_active and status.available
    assert (status.session_id, status.capture_id) == (7, 12)
    assert status.last_heartbeat_ms == 1000
    assert status.trace_schema_version == 1
    assert status.output_disabled

    unavailable = api.parse_hil_result(bytes([api.HIL_NOT_AVAILABLE]))
    assert unavailable.ok is False


def _trace_record(
    record_type: RecordType, payload: bytes, record_seq: int = 1
) -> TraceRecord:
    return TraceRecord(
        header=FragmentHeader(
            schema_version=1,
            session_id=7,
            capture_id=11,
            record_seq=record_seq,
            record_type=record_type,
            fragment_index=0,
            fragment_count=1,
            logical_length=len(payload),
            logical_crc16=crc16(payload),
        ),
        payload=payload,
    )


def _guard_bytes() -> bytes:
    return struct.pack("<5B5IB", 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7)


def test_hil_logical_lifecycle_records_parse_with_exact_layouts():
    begin = parse_record(
        _trace_record(
            RecordType.BEGIN,
            struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0x1234, 2)
            + _guard_bytes(),
        )
    )
    assert isinstance(begin, TraceBegin)
    assert begin.origin is TraceOrigin.ATSAMD21_OUTPUT_DISABLED
    assert begin.initial_guard.blocked_goal_write == 4

    config = parse_record(
        _trace_record(RecordType.CONFIG, struct.pack("<IBBHH", 9, 0, 0, 0, 0))
    )
    assert config.revision == 9
    assert config.robot_config is None

    marker = parse_record(
        _trace_record(RecordType.MARKER, struct.pack("<IIBI", 55, 100, 2, 1))
    )
    assert isinstance(marker, TraceMarker)
    assert marker.marker_id == 55 and marker.step_sequence == 1

    blocked = parse_record(
        _trace_record(RecordType.OUTPUT_BLOCKED, struct.pack("<I", 1) + _guard_bytes())
    )
    assert isinstance(blocked, OutputBlocked)
    assert blocked.guard.last_goal_sequence == 6

    end = parse_record(
        _trace_record(
            RecordType.END,
            struct.pack("<BBBBII", int(CaptureEndReason.COMPLETE), 2, 2, 1, 4, 9)
            + _guard_bytes(),
        )
    )
    assert isinstance(end, TraceEnd)
    assert end.reason is CaptureEndReason.COMPLETE
    assert end.emitted_fragment_count == 9


def test_hil_valid_config_snapshot_parses_and_exports_canonical_payload():
    robot_config = default_robot_config()
    payload = encode_robot_config(robot_config)
    config = parse_record(
        _trace_record(
            RecordType.CONFIG,
            struct.pack("<IBBHH", 9, 1, 1, len(payload), crc16(payload)) + payload,
        )
    )

    assert isinstance(config, ConfigSnapshot)
    assert config.robot_config == robot_config
    artifact = trace_to_artifact(config)
    assert artifact["payload_hex"] == payload.hex()
    assert "payload" not in artifact


def test_hil_logical_record_parser_rejects_crc_and_length_violations():
    payload = struct.pack("<IIBI", 55, 100, 2, 1)
    record = _trace_record(RecordType.MARKER, payload)
    bad_crc = TraceRecord(
        header=FragmentHeader(**{**record.header.__dict__, "logical_crc16": 0}),
        payload=payload,
    )
    with pytest.raises(TraceDecodeError, match="CRC"):
        parse_record(bad_crc)

    with pytest.raises(TraceDecodeError, match="truncated"):
        parse_record(_trace_record(RecordType.MARKER, payload[:-1]))


class _StepWriter:
    def __init__(self) -> None:
        self.data = bytearray()

    def u8(self, value: int) -> None:
        self.data += struct.pack("<B", value)

    def boolean(self, value: bool) -> None:
        self.u8(int(value))

    def u16(self, value: int) -> None:
        self.data += struct.pack("<H", value)

    def i8(self, value: int) -> None:
        self.data += struct.pack("<b", value)

    def i16(self, value: int) -> None:
        self.data += struct.pack("<h", value)

    def u32(self, value: int) -> None:
        self.data += struct.pack("<I", value)

    def i32(self, value: int) -> None:
        self.data += struct.pack("<i", value)

    def f32(self, value: float) -> None:
        self.data += struct.pack("<f", value)


def _write_controller_command(writer: _StepWriter) -> None:
    for value in (True, False, True):
        writer.boolean(value)
    writer.u32(77)
    for value in (True, False, True):
        writer.boolean(value)
    writer.u8(2)
    writer.u8(1)
    for value in (0.1, -0.2, 0.3, 1.0, 2.0, 3.0, 0.4, 0.5, 0.6, -0.7, 0.8, 0.9, 0.25, 0.5, 0.75):
        writer.f32(value)
    for value in (True, False, True):
        writer.boolean(value)
    writer.u8(3)


def _controller_step_payload(
    *,
    step_sequence: int = 5,
    config_valid: bool = True,
    config_persistent: bool = True,
) -> bytes:
    writer = _StepWriter()
    writer.u32(step_sequence)
    writer.u32(100)
    writer.u32(10)
    writer.boolean(True)

    writer.u16(12000)
    writer.boolean(True)
    writer.u8(1)
    writer.u8(18)
    writer.u8(1)
    writer.boolean(True)
    writer.u32(0x3FFFF)
    writer.u32(9)
    writer.boolean(False)
    writer.boolean(False)
    for index in range(18):
        writer.u8(index + 1)
        writer.i32(2000 + index)
        writer.i32(-index)
        writer.i32(index * 2)
        writer.u16(12000)
        writer.i8(30)
        writer.u8(0)
        writer.boolean(index % 2 == 0)
        writer.boolean(True)

    writer.u8(0x3F)
    writer.u8(1)
    for leg in range(6):
        writer.u32(100 + leg)
        writer.u16(10 + leg)
        writer.i32(1000 + leg)
        writer.i32(900 + leg)
        writer.i32(100 + leg)
        writer.u8(3)
        writer.u8(200)
        for value in (True, True, True, False, False, False):
            writer.boolean(value)
    writer.boolean(True)
    writer.boolean(False)

    _write_controller_command(writer)
    for value in (True, False, True, False, True):
        writer.boolean(value)

    writer.u32(88)
    writer.u8(3)
    writer.u16(55)
    writer.u16(66)
    writer.u16(22)
    writer.u8(128)
    writer.u8(200)
    for value in (0.5, -0.5, 0.25, 4.0, 5.0, 6.0, 0.7, 0.8, 0.9):
        writer.f32(value)

    writer.boolean(True)
    writer.u32(0x12345678)
    writer.u8(1)
    writer.u32(99)
    for index in range(18):
        writer.u16(2048 + index)
    for index in range(18):
        writer.boolean(index == 0)
    for index in range(18):
        writer.boolean(index == 1)
    for leg in range(6):
        writer.i16(leg)
        writer.i16(-leg)
        writer.i16(leg * 2)
        writer.boolean(True)
        writer.boolean(leg != 5)
        writer.boolean(leg == 2)

    for value in (True, False, True, False, True):
        writer.boolean(value)
    for value in (False, True, False, True, False):
        writer.boolean(value)

    writer.u32(9)
    writer.boolean(config_valid)
    writer.boolean(config_persistent)

    writer.u8(4)
    writer.u8(0)
    writer.u8(3)
    for value in (True, False, True, False, True):
        writer.boolean(value)
    writer.u32(9)
    writer.u32(88)
    writer.u8(6)
    for value in (True, False, True, False, True, False, True, False, True, False):
        writer.boolean(value)
    writer.u8(2)
    writer.boolean(True)
    writer.boolean(False)
    writer.u8(1)
    writer.u16(2048)
    writer.u8(0)
    writer.u8(0)
    writer.boolean(False)
    writer.u8(2)
    writer.u16(2049)
    writer.u8(0)
    writer.u8(1)
    writer.boolean(True)
    return bytes(writer.data)


def test_hil_controller_step_decodes_full_portable_contract():
    step = parse_record(_trace_record(RecordType.STEP, _controller_step_payload()))
    assert isinstance(step, ControllerStep)
    assert step.step_sequence == 5
    assert (step.time.now_ms, step.time.dt_ms, step.time.valid) == (100, 10, True)
    assert step.state.battery.millivolts == 12000
    assert len(step.state.dxl.servos) == 18
    assert step.state.dxl.servos[1].present_velocity == -1
    assert step.state.contact.feet[5].pressure_delta == 105
    assert step.intent.rc.command.mode == 2
    assert step.intent.rc.command.twist_vy == pytest.approx(-0.2)
    assert step.intent.motion.seq == 88
    assert step.intent.maintenance.targets.tick[17] == 2065
    assert step.intent.maintenance.targets.legs[2].clamped is True
    assert step.config.revision == 9
    assert step.observed.diagnostics.intent_sequence == 88
    assert [(goal.id, goal.tick, goal.clamped) for goal in step.observed.goals] == [
        (1, 2048, False),
        (2, 2049, True),
    ]


def test_hil_trace_assembler_validates_complete_capture_lifecycle():
    begin = struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0, 1) + _guard_bytes()
    config = struct.pack("<IBBHH", 9, 0, 0, 0, 0)
    step = _controller_step_payload(
        step_sequence=1, config_valid=False, config_persistent=False
    )
    fragments_before_end = sum(
        len(_fragment_payloads(payload, record_type=record_type, record_seq=sequence))
        for record_type, payload, sequence in (
            (RecordType.BEGIN, begin, 1),
            (RecordType.CONFIG, config, 2),
            (RecordType.STEP, step, 3),
        )
    )
    end = (
        struct.pack(
            "<BBBBII",
            int(CaptureEndReason.COMPLETE),
            1,
            1,
            1,
            3,
            fragments_before_end,
        )
        + _guard_bytes()
    )
    assembler = TraceAssembler(expected_session_id=7)
    records = (
        (RecordType.BEGIN, begin, 1),
        (RecordType.CONFIG, config, 2),
        (RecordType.STEP, step, 3),
        (RecordType.END, end, 4),
    )
    for record_type, payload, sequence in records:
        completed = []
        for fragment in _fragment_payloads(
            payload, record_type=record_type, record_seq=sequence
        ):
            result = assembler.accept_event(0xA0, fragment)
            if result is not None:
                completed.append(result)
        assert len(completed) == 1
        assert type(completed[0]) is {
            RecordType.BEGIN: TraceBegin,
            RecordType.CONFIG: ConfigSnapshot,
            RecordType.STEP: ControllerStep,
            RecordType.END: TraceEnd,
        }[record_type]

    trace = assembler.finalize()
    assert isinstance(trace, HILTrace)
    assert len(trace.steps) == 1
    assert trace.end.reason is CaptureEndReason.COMPLETE


def test_hil_trace_assembler_rejects_step_before_config_snapshot():
    begin = struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0, 1) + _guard_bytes()
    step = _controller_step_payload(config_valid=False, config_persistent=False)
    assembler = TraceAssembler(expected_session_id=7)
    assembler.accept_event(0xA0, _fragment_payloads(begin, record_type=RecordType.BEGIN, record_seq=1)[0])
    with pytest.raises(TraceDecodeError, match="before ConfigSnapshot"):
        for fragment in _fragment_payloads(step, record_type=RecordType.STEP, record_seq=2):
            assembler.accept_event(0xA0, fragment)


def test_hil_trace_assembler_uses_dedicated_output_block_event_id():
    begin = struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0, 1) + _guard_bytes()
    config = struct.pack("<IBBHH", 9, 0, 0, 0, 0)
    step = _controller_step_payload(
        step_sequence=1, config_valid=False, config_persistent=False
    )
    blocked = struct.pack("<I", 1) + _guard_bytes()
    assembler = TraceAssembler(expected_session_id=7)
    for record_type, payload, record_seq in (
        (RecordType.BEGIN, begin, 1),
        (RecordType.CONFIG, config, 2),
        (RecordType.STEP, step, 3),
    ):
        for fragment in _fragment_payloads(
            payload, record_type=record_type, record_seq=record_seq
        ):
            assembler.accept_event(0xA0, fragment)

    parsed = None
    for fragment in _fragment_payloads(
        blocked, record_type=RecordType.OUTPUT_BLOCKED, record_seq=4
    ):
        parsed = assembler.accept_event(0xA1, fragment)
    assert isinstance(parsed, OutputBlocked)
    assert parsed.step_sequence == 1

    wrong_event = _fragment_payloads(
        blocked, record_type=RecordType.OUTPUT_BLOCKED, record_seq=4
    )[0]
    with pytest.raises(TraceDecodeError, match="wrong event ID"):
        TraceAssembler().accept_event(0xA0, wrong_event)
    with pytest.raises(TraceDecodeError, match="wrong event ID"):
        TraceAssembler().accept_event(
            0xA1,
            _fragment_payloads(begin, record_type=RecordType.BEGIN, record_seq=1)[0],
        )


def test_hil_trace_assembler_retains_partial_terminal_trace_as_diagnostic():
    begin = struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0, 2) + _guard_bytes()
    config = struct.pack("<IBBHH", 9, 0, 0, 0, 0)
    step = _controller_step_payload(
        step_sequence=1, config_valid=False, config_persistent=False
    )
    records = (
        (RecordType.BEGIN, begin, 1),
        (RecordType.CONFIG, config, 2),
        (RecordType.STEP, step, 3),
    )
    fragments_before_end = sum(
        len(_fragment_payloads(payload, record_type=record_type, record_seq=record_seq))
        for record_type, payload, record_seq in records
    )
    end = (
        struct.pack(
            "<BBBBII",
            int(CaptureEndReason.HOST_ABORTED),
            2,
            1,
            1,
            3,
            fragments_before_end,
        )
        + _guard_bytes()
    )
    assembler = TraceAssembler(expected_session_id=7)
    for record_type, payload, record_seq in records + ((RecordType.END, end, 4),):
        for fragment in _fragment_payloads(
            payload, record_type=record_type, record_seq=record_seq
        ):
            assembler.accept_event(0xA0, fragment)

    trace = assembler.finalize()
    assert len(trace.steps) == 1
    assert trace.end.reason is CaptureEndReason.HOST_ABORTED


def test_hil_trace_decodes_retained_raw_event_frames():
    begin = struct.pack("<HBBBIIIHB", 1, 1, 0, 2, 7, 11, 9, 0, 1) + _guard_bytes()
    config = struct.pack("<IBBHH", 9, 0, 0, 0, 0)
    step = _controller_step_payload(
        step_sequence=1, config_valid=False, config_persistent=False
    )
    logical_records = (
        (RecordType.BEGIN, begin, 1),
        (RecordType.CONFIG, config, 2),
        (RecordType.STEP, step, 3),
    )
    fragments_before_end = sum(
        len(_fragment_payloads(payload, record_type=record_type, record_seq=sequence))
        for record_type, payload, sequence in logical_records
    )
    end = (
        struct.pack(
            "<BBBBII",
            int(CaptureEndReason.COMPLETE),
            1,
            1,
            1,
            3,
            fragments_before_end,
        )
        + _guard_bytes()
    )
    wire_frames = [
        encode_frame(Header(msg_type=int(MsgType.TELEMETRY), msg_id=0x40), b"\x00")
    ]
    for record_type, payload, sequence in logical_records + ((RecordType.END, end, 4),):
        for fragment in _fragment_payloads(
            payload, record_type=record_type, record_seq=sequence
        ):
            wire_frames.append(
                encode_frame(
                    Header(
                        msg_type=int(MsgType.EVENT),
                        msg_id=0xA0,
                        seq=len(wire_frames),
                        timestamp_ms=100,
                    ),
                    fragment,
                )
            )

    trace = decode_trace_frames(wire_frames, expected_session_id=7)
    assert len(trace.steps) == 1
    assert trace.steps[0].step_sequence == 1