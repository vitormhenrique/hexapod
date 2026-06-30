"""Round-trip tests for the session logger + replay reader (no hardware).

Writes raw frames, decoded telemetry rows, and events through
:class:`SessionLogger`, then reads everything back via :class:`SessionReplay`
and the lossless ``iter_raw_frames`` reader.
"""

from __future__ import annotations

import struct

from hexapod_protocol import telemetry as tlm

from .conftest import make_telemetry


def _health_payload(uptime_ms=1234, state=2, fault=0, watchdog=0, battery=11700):
    # decode_health expects 12 bytes: u32 uptime, u8 state, u8 fault,
    # u32 watchdog, u16 battery.
    return struct.pack("<IBBIH", uptime_ms, state, fault, watchdog, battery)


def test_session_roundtrip(tmp_path) -> None:
    from data import SessionLogger, SessionReplay

    frame = make_telemetry(int(tlm.StreamId.HEALTH), _health_payload())
    record = tlm.decode_stream(int(tlm.StreamId.HEALTH), _health_payload())

    with SessionLogger(
        out_dir=tmp_path, robot_name="hex", firmware={"version": "0.1"}
    ) as logger:
        session_dir = logger.dir
        logger.log_raw_frame(frame)
        logger.log_record("health", record, robot_time_ms=1234)
        logger.mark_event("estop", "operator pressed", source="ui")

    replay = SessionReplay(session_dir)

    # Manifest is complete and counts match.
    meta = replay.meta
    assert meta["robot_name"] == "hex"
    assert meta["firmware"] == {"version": "0.1"}
    assert meta["frame_count"] == 1
    assert meta["record_count"] == 1
    assert meta["event_count"] == 1
    assert meta["stopped_utc"] is not None

    # Decoded telemetry JSONL round-trips.
    records = list(replay.iter_records())
    assert len(records) == 1
    assert records[0]["stream"] == "health"
    assert records[0]["robot_time_ms"] == 1234
    assert records[0]["data"]["uptime_ms"] == 1234

    # Events JSONL round-trips with extra fields.
    events = list(replay.iter_events())
    assert len(events) == 1
    assert events[0]["kind"] == "estop"
    assert events[0]["source"] == "ui"


def test_raw_frames_read_back_byte_exact(tmp_path) -> None:
    from data import SessionLogger, SessionReplay

    frames = [
        make_telemetry(int(tlm.StreamId.HEALTH), _health_payload()),
        make_telemetry(int(tlm.StreamId.HEALTH), _health_payload(uptime_ms=5678)),
    ]
    with SessionLogger(out_dir=tmp_path, robot_name="hex") as logger:
        session_dir = logger.dir
        for fr in frames:
            logger.log_raw_frame(fr)

    replay = SessionReplay(session_dir)
    read_back = [fr for _ts, fr in replay.iter_raw_frames()]
    assert read_back == frames


def test_decoded_frame_replay(tmp_path) -> None:
    from data import SessionLogger, SessionReplay

    frame = make_telemetry(int(tlm.StreamId.HEALTH), _health_payload(uptime_ms=42))
    with SessionLogger(out_dir=tmp_path, robot_name="hex") as logger:
        session_dir = logger.dir
        logger.log_raw_frame(frame)

    replay = SessionReplay(session_dir)
    decoded = list(replay.iter_decoded_frames())
    assert len(decoded) == 1
    df = decoded[0]
    assert df.stream == "health"
    assert df.record is not None
    assert df.record.uptime_ms == 42
    assert df.msg_id == 0x40 + int(tlm.StreamId.HEALTH)


def test_corrupt_frame_is_skipped(tmp_path) -> None:
    from data import SessionLogger, SessionReplay

    good = make_telemetry(int(tlm.StreamId.HEALTH), _health_payload())
    with SessionLogger(out_dir=tmp_path, robot_name="hex") as logger:
        session_dir = logger.dir
        logger.log_raw_frame(b"\x00\x99\x99\x00")  # bad COBS/magic
        logger.log_raw_frame(good)

    replay = SessionReplay(session_dir)
    # Raw reader yields both; decoder drops the corrupt one.
    assert len(list(replay.iter_raw_frames())) == 2
    decoded = list(replay.iter_decoded_frames())
    assert len(decoded) == 1
    assert decoded[0].stream == "health"
