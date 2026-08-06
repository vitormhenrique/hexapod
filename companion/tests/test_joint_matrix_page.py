"""Headless UI tests for the six-leg Joint Matrix maintenance page."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QAbstractItemView

from hexapod_protocol import telemetry as tlm
from theme import DRACULA

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import JointMatrixPage

    service = ConnectionService()
    page = JointMatrixPage(service)
    qtbot.addWidget(page)
    return service, page


def _make_ready_page(qtbot):
    service, page = _make_page(qtbot)
    service.connected.emit(True)
    service.maint_lock_changed.emit(True, 7)
    service.maintenance_setup_changed.emit(False, True, "ready: 18 servos scanned")
    return service, page


def _type(cell, text: str) -> None:
    """Simulate real typing: clear, then send each character to the validator."""
    cell.value.clear()
    for character in text:
        QTest.keyClicks(cell.value, character)


def test_matrix_defaults_to_centered_servo_angles(qtbot) -> None:
    _service, page = _make_page(qtbot)
    assert page.matrix.rowCount() == 6
    assert page.matrix.columnCount() == 4
    assert page.matrix.item(0, 0).text() == "Leg 1"
    assert all(cell.value_int() == 180 for cell in page._cells.values())
    assert all(cell.value.text() == "180" for cell in page._cells.values())
    assert all(cell.value.isEnabled() for cell in page._cells.values())
    assert all(not cell.apply.isEnabled() for cell in page._cells.values())
    assert page.matrix.selectionMode() == QAbstractItemView.SelectionMode.NoSelection


def test_tick_mode_enables_servo_then_sends_calibrated_relative_angle(
    qtbot, monkeypatch
) -> None:
    service, page = _make_ready_page(qtbot)
    sent = []
    monkeypatch.setattr(
        service,
        "set_joint_target_with_torque",
        lambda servo_id, leg, joint, angle: sent.append(
            (servo_id, leg, joint, angle)
        ),
    )
    page.tick_radio.setChecked(True)
    cell = page._cells[(0, 0)]
    cell.value.setText("2048")
    cell.apply.click()
    assert sent == [(1, 0, 0, 0)]


def test_release_targets_only_the_mapped_servo(qtbot, monkeypatch) -> None:
    service, page = _make_ready_page(qtbot)
    released = []
    monkeypatch.setattr(
        service,
        "set_servo_torque",
        lambda servo_id, enabled: released.append((servo_id, enabled)),
    )
    page._cells[(0, 0)].release.click()
    assert released == [(1, False)]


def test_verified_release_grays_only_that_servo(qtbot) -> None:
    service, page = _make_ready_page(qtbot)
    service.servo_torque_changed.emit(1, False, True)
    released = page._cells[(0, 0)]
    assert released.released
    assert released.release.text() == "Passive"
    assert DRACULA.comment in released.value.styleSheet()
    assert not page._cells[(0, 1)].released


def test_stale_status_cannot_undo_verified_release(qtbot) -> None:
    service, page = _make_ready_page(qtbot)
    service.servo_torque_changed.emit(1, False, True)
    service.telemetry.emit(
        int(tlm.StreamId.SERVO_STATUS),
        tlm.ServoStatusTelemetry(
            [tlm.ServoStatus(1, 2048, 0, 0, 12000, 25, 0, True)]
        ),
    )
    assert page._cells[(0, 0)].released


def test_verified_set_reactivates_a_released_servo(qtbot) -> None:
    service, page = _make_ready_page(qtbot)
    service.servo_torque_changed.emit(1, False, True)
    service.servo_torque_changed.emit(1, True, True)
    assert not page._cells[(0, 0)].released


def test_passive_joint_state_updates_the_matching_matrix_value(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.telemetry.emit(
        int(tlm.StreamId.JOINT_STATE),
        tlm.JointStateTelemetry([tlm.JointAngle(leg=2, joint=1, angle_centideg=1234)]),
    )
    assert page._cells[(2, 1)].value_int() == 192


def test_live_update_does_not_overwrite_a_staged_input(qtbot) -> None:
    service, page = _make_page(qtbot)
    cell = page._cells[(2, 1)]
    _type(cell, "200")
    service.telemetry.emit(
        int(tlm.StreamId.JOINT_STATE),
        tlm.JointStateTelemetry([tlm.JointAngle(leg=2, joint=1, angle_centideg=1234)]),
    )
    assert cell.value_int() == 200
    assert cell.value.text() == "200"


# Regression: a range-bound validator (the old QSpinBox) rejected the first
# keystroke whenever the joint travel did not start at 0 -- typing "5" toward
# "50" in a 90..270 range is Invalid, not Intermediate -- so the field silently
# refused input. Any digit sequence must be typeable in either unit.
@pytest.mark.parametrize("ticks", [False, True])
def test_any_digit_sequence_can_be_typed(qtbot, ticks: bool) -> None:
    _service, page = _make_page(qtbot)
    if ticks:
        page.tick_radio.setChecked(True)
    cell = page._cells[(0, 1)]
    for text in ("5", "50", "7", "1234"):
        _type(cell, text)
        assert cell.value.text() == text


def test_out_of_range_entry_is_clamped_to_servo_travel(qtbot, monkeypatch) -> None:
    service, page = _make_ready_page(qtbot)
    sent = []
    monkeypatch.setattr(
        service,
        "set_joint_target_with_torque",
        lambda servo_id, leg, joint, angle: sent.append(angle),
    )
    cell = page._cells[(0, 1)]
    _type(cell, "999")
    cell.apply.click()
    # Default travel is +/-90 deg about the 180 deg centre.
    assert sent == [(270 - 180) * 100]


def test_unparseable_text_falls_back_to_the_last_display(qtbot) -> None:
    _service, page = _make_page(qtbot)
    cell = page._cells[(0, 0)]
    cell.value.clear()
    assert cell.value_int() == 180