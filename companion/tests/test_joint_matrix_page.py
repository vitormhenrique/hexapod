"""Headless UI tests for the six-leg Joint Matrix maintenance page."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
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


def test_matrix_defaults_to_centered_servo_angles(qtbot) -> None:
    _service, page = _make_page(qtbot)
    assert page.matrix.rowCount() == 6
    assert page.matrix.columnCount() == 4
    assert page.matrix.item(0, 0).text() == "Leg 1"
    assert all(cell.value.value() == 180 for cell in page._cells.values())
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
    cell.value.setValue(2048)
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
    assert page._cells[(2, 1)].value.value() == 192


def test_live_update_does_not_overwrite_a_staged_input(qtbot) -> None:
    service, page = _make_page(qtbot)
    cell = page._cells[(2, 1)]
    cell.value.setValue(200)
    service.telemetry.emit(
        int(tlm.StreamId.JOINT_STATE),
        tlm.JointStateTelemetry([tlm.JointAngle(leg=2, joint=1, angle_centideg=1234)]),
    )
    assert cell.value.value() == 200