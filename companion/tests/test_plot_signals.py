"""Tests for the Plot Workbench signal registry and replay extraction (nxi.1)."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")

from data.plot_signals import (  # noqa: E402
    build_signal_registry,
    extract_series,
    registry_by_key,
    streams_for,
)

from .replay_fixtures import build_sample_session  # noqa: E402


def test_registry_covers_all_required_stream_families() -> None:
    reg = build_signal_registry()
    groups = {s.group for s in reg}
    # Acceptance: servo, leg, control, RC, and sensor streams are all plottable.
    assert {"Servo", "Leg", "Control", "RC", "Sensor"} <= groups
    # Keys are unique.
    assert len(registry_by_key(reg)) == len(reg)


def test_streams_for_maps_signals_to_stream_ids() -> None:
    reg = registry_by_key(build_signal_registry())
    sigs = [reg["servo.1.position"], reg["control.kill"], reg["rc.gait_index"]]
    assert streams_for(sigs) == {
        int(tlm.StreamId.SERVO_STATUS),
        int(tlm.StreamId.CONTROL_STATE),
        int(tlm.StreamId.RC_INPUT),
    }


def test_servo_extractor_finds_matching_id() -> None:
    reg = registry_by_key(build_signal_registry())
    sig = reg["servo.3.position"]
    record = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(1, 100, 0, 0, 12000, 30, 0),
            tlm.ServoStatus(3, 2048, 0, 0, 12000, 41, 0),
        ]
    )
    assert sig.extract(record) == 2048.0
    # Absent id -> None (won't be plotted).
    assert reg["servo.9.position"].extract(record) is None


def test_bool_and_channel_extractors() -> None:
    reg = registry_by_key(build_signal_registry())
    rc = tlm.RcInputTelemetry(
        armed=True, kill=False, failsafe=False, autonomy=False,
        gait_index=2, channels_us=[1000, 1500, 2000, 1234] + [1500] * 12,
    )
    assert reg["rc.armed"].extract(rc) == 1.0
    assert reg["rc.gait_index"].extract(rc) == 2.0
    assert reg["rc.ch4"].extract(rc) == 1234.0


def test_extract_series_from_replay_session(tmp_path) -> None:
    replay = build_sample_session(tmp_path, frames_per_stream=3)
    reg = registry_by_key(build_signal_registry())
    sigs = [reg["servo.1.position"], reg["contact.0.confidence"], reg["rc.gait_index"]]
    series = extract_series(replay.iter_decoded_frames(), sigs)
    # Each selected signal produced 3 samples (one per recorded frame per stream).
    for sig in sigs:
        xs, ys = series[sig.key]
        assert len(xs) == 3 and len(ys) == 3
    # Contact foot 0 confidence is the fixed fixture value.
    assert series["contact.0.confidence"][1] == [180.0, 180.0, 180.0]
    # x axis is monotonically non-decreasing (robot time in seconds).
    xs = series["servo.1.position"][0]
    assert xs == sorted(xs)
