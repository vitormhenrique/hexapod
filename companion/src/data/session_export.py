"""Selected-signal CSV export and human-readable session summaries.

This module deliberately works from :class:`SessionReplay` rather than from a
Qt page. CLI and UI callers therefore export the same decoded telemetry without
reopening a serial connection or duplicating signal extraction rules.
"""

from __future__ import annotations

import csv
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Iterable

from hexapod_protocol import telemetry as tlm

from .plot_signals import PlotSignal
from .session_replay import SessionReplay


def export_selected_csv(
    replay: SessionReplay, signals: Iterable[PlotSignal], output_path: Path | str
) -> int:
    """Write selected decoded telemetry signals to CSV and return row count.

    Each row represents one decoded telemetry frame that carries at least one
    selected value. ``host_time_ns`` and ``robot_time_ms`` retain both clocks;
    fields from other streams are left blank rather than inventing an alignment
    or interpolation policy during export.
    """
    selected_by_key = {signal.key: signal for signal in signals}
    selected = tuple(selected_by_key.values())
    if not selected:
        raise ValueError("select at least one signal for CSV export")

    by_stream: dict[int, list[PlotSignal]] = {}
    for signal in selected:
        by_stream.setdefault(signal.stream_id, []).append(signal)

    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["host_time_ns", "robot_time_ms", *(signal.key for signal in selected)]
    rows_written = 0
    with target.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for frame in replay.iter_decoded_frames():
            if frame.record is None or frame.stream is None:
                continue
            try:
                stream_id = int(tlm.stream_id_from_name(frame.stream))
            except ValueError:
                continue
            matching_signals = by_stream.get(stream_id)
            if not matching_signals:
                continue

            row: dict[str, int | float] = {
                "host_time_ns": frame.host_time_ns,
                "robot_time_ms": frame.timestamp_ms,
            }
            for signal in matching_signals:
                value = signal.extract(frame.record)
                if value is not None:
                    row[signal.key] = value
            if len(row) == 2:
                continue
            writer.writerow(row)
            rows_written += 1
    return rows_written


def build_session_summary(replay: SessionReplay) -> dict:
    """Return a JSON-compatible operational summary of a recorded session."""
    metadata = replay.meta
    records = list(replay.iter_records())
    events = list(replay.iter_events())
    stream_counts = Counter(
        str(record.get("stream", "unknown")) for record in records
    )
    fault_events = [
        event
        for event in events
        if str(event.get("kind", "")).lower() in {"fault", "estop", "error"}
    ]

    servo_health: dict[str, dict[str, int]] = {}
    decoded_frame_count = 0
    telemetry_frame_count = 0
    for frame in replay.iter_decoded_frames():
        decoded_frame_count += 1
        if frame.record is None:
            continue
        telemetry_frame_count += 1
        if frame.stream != "servo_status":
            continue
        for servo in getattr(frame.record, "servos", []):
            entry = servo_health.setdefault(
                str(servo.id),
                {
                    "samples": 0,
                    "min_voltage_mv": servo.voltage_mv,
                    "max_temperature_c": servo.temperature_c,
                    "hardware_error_mask": 0,
                },
            )
            entry["samples"] += 1
            entry["min_voltage_mv"] = min(entry["min_voltage_mv"], servo.voltage_mv)
            entry["max_temperature_c"] = max(
                entry["max_temperature_c"], servo.temperature_c
            )
            entry["hardware_error_mask"] |= servo.hardware_error

    return {
        "session_id": metadata.get("session_id", replay.dir.name),
        "robot_name": metadata.get("robot_name", "unknown"),
        "started_utc": metadata.get("started_utc"),
        "stopped_utc": metadata.get("stopped_utc"),
        "duration_seconds": _duration_seconds(metadata),
        "frame_count": metadata.get("frame_count", 0),
        "record_count": metadata.get("record_count", 0),
        "event_count": metadata.get("event_count", 0),
        "decoded_frame_count": decoded_frame_count,
        "telemetry_frame_count": telemetry_frame_count,
        "stream_counts": dict(sorted(stream_counts.items())),
        "events": events,
        "fault_events": fault_events,
        "servo_health": dict(sorted(servo_health.items(), key=lambda item: int(item[0]))),
    }


def render_session_summary(summary: dict) -> str:
    """Format :func:`build_session_summary` output as a concise text report."""
    duration = summary.get("duration_seconds")
    duration_text = "unknown" if duration is None else f"{duration:.3f} s"
    lines = [
        f"Session report: {summary.get('session_id', 'unknown')}",
        f"Robot: {summary.get('robot_name', 'unknown')}",
        f"Started: {summary.get('started_utc') or 'unknown'}",
        f"Stopped: {summary.get('stopped_utc') or 'unknown'}",
        f"Duration: {duration_text}",
        (
            "Counts: "
            f"frames={summary.get('frame_count', 0)} "
            f"decoded={summary.get('decoded_frame_count', 0)} "
            f"telemetry={summary.get('telemetry_frame_count', 0)} "
            f"records={summary.get('record_count', 0)} "
            f"events={summary.get('event_count', 0)}"
        ),
        "",
        "Streams:",
    ]
    stream_counts = summary.get("stream_counts", {})
    if stream_counts:
        lines.extend(f"  {stream}: {count}" for stream, count in stream_counts.items())
    else:
        lines.append("  none")

    lines.extend(["", "Faults:"])
    fault_events = summary.get("fault_events", [])
    if fault_events:
        for event in fault_events:
            lines.append(
                f"  {event.get('kind', 'fault')}: {event.get('detail', '')}"
            )
    else:
        lines.append("  none recorded")

    lines.extend(["", "Servo health:"])
    servo_health = summary.get("servo_health", {})
    if servo_health:
        for servo_id, health in servo_health.items():
            lines.append(
                f"  servo {servo_id}: samples={health['samples']} "
                f"min_voltage_mv={health['min_voltage_mv']} "
                f"max_temperature_c={health['max_temperature_c']} "
                f"hardware_error_mask=0x{health['hardware_error_mask']:02X}"
            )
    else:
        lines.append("  no servo status telemetry")
    return "\n".join(lines) + "\n"


def write_session_summary(replay: SessionReplay, output_path: Path | str) -> dict:
    """Write a human-readable summary report and return its structured data."""
    summary = build_session_summary(replay)
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(render_session_summary(summary), encoding="utf-8")
    return summary


def _duration_seconds(metadata: dict) -> float | None:
    started = _parse_utc(metadata.get("started_utc"))
    stopped = _parse_utc(metadata.get("stopped_utc"))
    if started is None or stopped is None:
        return None
    return max(0.0, (stopped - started).total_seconds())


def _parse_utc(value: object) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None