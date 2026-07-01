"""Headless tests for the Plot Workbench page (nxi.1)."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from .replay_fixtures import build_sample_session  # noqa: E402


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import PlotWorkbenchPage

    service = ConnectionService()
    page = PlotWorkbenchPage(service)
    qtbot.addWidget(page)
    return page, service


def test_selecting_signals_creates_curves(qtbot) -> None:
    page, _ = _make_page(qtbot)
    page.select_signals(["servo.1.position", "control.kill"])
    assert set(page.selected_keys()) == {"servo.1.position", "control.kill"}
    assert len(page._curves) == 2
    # Deselecting drops the curve + buffer.
    page.select_signals(["servo.1.position"])
    assert set(page._curves) == {"servo.1.position"}


def test_live_telemetry_appends_to_buffers(qtbot) -> None:
    page, service = _make_page(qtbot)
    page.select_signals(["servo.1.position"])
    status = tlm.ServoStatusTelemetry(
        servos=[tlm.ServoStatus(1, 2048, 0, 0, 12000, 33, 0)]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    buf = page._live_buf["servo.1.position"]
    assert len(buf) == 2
    assert buf[-1][1] == 2048.0


def test_live_ignores_unselected_streams(qtbot) -> None:
    page, service = _make_page(qtbot)
    page.select_signals(["servo.1.position"])
    # A contact frame must not land in the servo buffer.
    contact = tlm.ContactStateTelemetry(feet=[tlm.FootContact(3, 200, 40)])
    service.telemetry.emit(int(tlm.StreamId.CONTACT_STATE), contact)
    assert len(page._live_buf["servo.1.position"]) == 0


def test_subscribes_to_needed_streams_on_connect(qtbot) -> None:
    page, service = _make_page(qtbot)
    calls: list[tuple[int, int]] = []
    service.subscribe = lambda sid, rate: calls.append((sid, rate))  # type: ignore
    page.select_signals(["servo.1.position", "rc.gait_index"])
    service.connected.emit(True)
    subscribed = {sid for sid, _ in calls}
    assert int(tlm.StreamId.SERVO_STATUS) in subscribed
    assert int(tlm.StreamId.RC_INPUT) in subscribed


def test_replay_mode_plots_recorded_session(qtbot, tmp_path) -> None:
    page, _ = _make_page(qtbot)
    page.select_signals(["servo.1.position", "contact.0.confidence"])
    replay = build_sample_session(tmp_path, frames_per_stream=4)
    page.load_session(replay.dir)
    assert page._mode == "replay"
    # The servo curve holds the 4 recorded samples.
    xs, ys = page._curves["servo.1.position"].getData()
    assert len(xs) == 4
    assert "Replay:" in page._status.text()


def test_clear_empties_buffers(qtbot) -> None:
    page, service = _make_page(qtbot)
    page.select_signals(["servo.1.position"])
    status = tlm.ServoStatusTelemetry(
        servos=[tlm.ServoStatus(1, 2048, 0, 0, 12000, 33, 0)]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    assert len(page._live_buf["servo.1.position"]) == 1
    page._clear()
    assert len(page._live_buf["servo.1.position"]) == 0


def test_filter_hides_nonmatching_signals(qtbot) -> None:
    from PySide6.QtCore import Qt

    page, _ = _make_page(qtbot)
    page._filter.setText("gait")
    visible = {
        child.data(0, Qt.UserRole): not child.isHidden()
        for child in page._iter_signal_items()
    }
    # The RC gait_index row stays visible; a servo row is hidden.
    assert visible["rc.gait_index"] is True
    assert visible["servo.1.position"] is False


# --- event markers (nxi.2) -------------------------------------------------


def test_live_event_draws_marker(qtbot) -> None:
    page, service = _make_page(qtbot)
    page.select_signals(["servo.1.position"])
    service.event.emit("gait", "Tripod")
    assert page.event_marker_count() == 1
    assert len(page._event_lines) == 1


def test_add_note_emits_event_and_marker(qtbot) -> None:
    page, service = _make_page(qtbot)
    seen: list[tuple[str, str]] = []
    service.event.connect(lambda k, d: seen.append((k, d)))
    page._note_edit.setText("touched leg 3")
    page._add_note()
    assert ("note", "touched leg 3") in seen
    assert page.event_marker_count() == 1
    assert page._note_edit.text() == ""


def test_toggle_hides_and_restores_markers(qtbot) -> None:
    page, service = _make_page(qtbot)
    service.event.emit("fault", "WATCHDOG")
    assert len(page._event_lines) == 1
    page._show_events.setChecked(False)
    assert len(page._event_lines) == 0
    page._show_events.setChecked(True)
    assert len(page._event_lines) == 1


def test_clear_removes_event_markers(qtbot) -> None:
    page, service = _make_page(qtbot)
    service.event.emit("gait", "Wave")
    assert page.event_marker_count() == 1
    page._clear()
    assert page.event_marker_count() == 0
    assert len(page._event_lines) == 0


def test_replay_overlays_recorded_events(qtbot, tmp_path) -> None:
    page, _ = _make_page(qtbot)
    page.select_signals(["servo.1.position"])
    replay = build_sample_session(tmp_path, frames_per_stream=3)
    page.load_session(replay.dir)
    # The fixture records one "connect" event.
    assert page.event_marker_count() == 1
    assert "event(s)" in page._status.text()

