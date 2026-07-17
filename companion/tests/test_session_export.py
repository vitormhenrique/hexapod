"""Tests for hardware-free selected CSV and text summary session exports."""

from __future__ import annotations

import csv

from data.plot_signals import build_signal_registry, registry_by_key
from data.session_export import (
    build_session_summary,
    export_selected_csv,
    render_session_summary,
    write_session_summary,
)

from .replay_fixtures import build_sample_session


def test_selected_signal_csv_preserves_frame_clocks_and_values(tmp_path) -> None:
    replay = build_sample_session(tmp_path, frames_per_stream=2)
    signals = registry_by_key(build_signal_registry())
    output = tmp_path / "selected.csv"

    rows_written = export_selected_csv(
        replay,
        [signals["health.battery_mv"], signals["servo.1.position"]],
        output,
    )

    with output.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert rows_written == 4
    assert rows[0].keys() == {
        "host_time_ns",
        "robot_time_ms",
        "health.battery_mv",
        "servo.1.position",
    }
    assert {row["health.battery_mv"] for row in rows if row["health.battery_mv"]} == {
        "11800.0"
    }
    assert {row["servo.1.position"] for row in rows if row["servo.1.position"]} == {
        "2048.0"
    }


def test_selected_signal_csv_requires_a_selection(tmp_path) -> None:
    replay = build_sample_session(tmp_path)

    try:
        export_selected_csv(replay, [], tmp_path / "empty.csv")
    except ValueError as exc:
        assert "at least one signal" in str(exc)
    else:
        raise AssertionError("an empty signal selection must be rejected")


def test_text_summary_reports_streams_events_and_servo_health(tmp_path) -> None:
    replay = build_sample_session(tmp_path, frames_per_stream=2)

    summary = build_session_summary(replay)
    text = render_session_summary(summary)
    output = tmp_path / "summary.txt"
    written = write_session_summary(replay, output)

    assert summary["stream_counts"]["health"] == 2
    assert summary["events"][0]["kind"] == "connect"
    assert summary["servo_health"]["1"] == {
        "samples": 2,
        "min_voltage_mv": 12000,
        "max_temperature_c": 30,
        "hardware_error_mask": 0,
    }
    assert written == summary
    assert "Session report:" in text
    assert "Servo health:" in text
    assert output.read_text(encoding="utf-8") == text


def test_text_summary_highlights_fault_events(tmp_path) -> None:
    from data import SessionLogger, SessionReplay

    with SessionLogger(tmp_path, robot_name="fault-case") as logger:
        session_dir = logger.dir
        logger.mark_event("fault", "servo 4 overheating")

    summary = build_session_summary(SessionReplay(session_dir))
    text = render_session_summary(summary)

    assert len(summary["fault_events"]) == 1
    assert summary["fault_events"][0]["kind"] == "fault"
    assert summary["fault_events"][0]["detail"] == "servo 4 overheating"
    assert "fault: servo 4 overheating" in text