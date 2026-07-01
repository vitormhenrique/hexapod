"""Qt-free tests for the session event log (nxi.2)."""

from __future__ import annotations

from data.event_log import EventLog, EventMarker, color_for


def test_marker_color_and_label() -> None:
    m = EventMarker(t_s=1.5, kind="gait", detail="Tripod")
    assert m.color == color_for("gait")
    assert m.label == "gait: Tripod"
    assert EventMarker(t_s=0.0, kind="note").label == "note"


def test_unknown_kind_falls_back() -> None:
    assert color_for("mystery") == color_for("mystery")
    assert color_for("gait") != color_for("mystery")


def test_add_and_clear() -> None:
    log = EventLog()
    log.add("gait", "Wave", 1.0)
    log.add("fault", "WATCHDOG", 2.0)
    assert len(log) == 2
    assert [m.kind for m in log.markers()] == ["gait", "fault"]
    log.clear()
    assert len(log) == 0


def test_from_session_events_relative_time() -> None:
    rows = [
        {"host_time_ns": 1_000_000_000, "kind": "connect", "detail": "up"},
        {"host_time_ns": 3_000_000_000, "kind": "gait", "detail": "Tripod"},
        {"kind": "skipped-no-time"},  # dropped: no host_time_ns
        {"host_time_ns": 4_000_000_000},  # dropped: no kind
    ]
    log = EventLog.from_session_events(rows, t0_ns=1_000_000_000)
    markers = log.markers()
    assert len(markers) == 2
    assert markers[0].t_s == 0.0
    assert markers[1].t_s == 2.0
    assert markers[1].kind == "gait"


def test_from_session_events_default_base() -> None:
    rows = [{"host_time_ns": 5_000_000_000, "kind": "note", "detail": "x"}]
    log = EventLog.from_session_events(rows)  # base = first row's own time
    assert log.markers()[0].t_s == 0.0
