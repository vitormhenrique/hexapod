"""Headless UI tests for the Passive Pose & Stream page.

The page is built off-screen with a disconnected :class:`ConnectionService`.
Joint rows come from synthetic ``joint_state`` telemetry, and the mode/stream
controls react to ``passive_result``/``passive_rate_result`` emissions.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import PassivePosePage

    service = ConnectionService()
    page = PassivePosePage(service)
    qtbot.addWidget(page)
    return service, page


def test_joint_state_populates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    joints = [
        tlm.JointAngle(leg=0, joint=0, angle_centideg=1234),
        tlm.JointAngle(leg=0, joint=1, angle_centideg=-500),
    ]
    service.telemetry.emit(
        int(tlm.StreamId.JOINT_STATE), tlm.JointStateTelemetry(joints=joints)
    )
    assert page.table.rowCount() == 2
    assert page.table.item(0, 0).text() == "Leg 0"
    assert page.table.item(0, 1).text() == "coxa"
    assert page.table.item(0, 2).text() == "12.34\u00b0"
    # Re-emitting the same joints must not add rows.
    service.telemetry.emit(
        int(tlm.StreamId.JOINT_STATE), tlm.JointStateTelemetry(joints=joints)
    )
    assert page.table.rowCount() == 2


def test_passive_enter_result_sets_badge(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.PassiveResult(api.PASSIVE_OK, 1)
    service.passive_result.emit("enter", res)
    assert "enter" in page.mode_result.text()
    assert "ok" in page.mode_result.text()
    assert "torque off" in page.mode_badge._value.text()


def test_passive_exit_result(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.PassiveResult(api.PASSIVE_OK, 0)
    service.passive_result.emit("exit", res)
    assert "exit" in page.mode_result.text()


def test_rate_result_displays_hz(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.PassiveRateResult(api.PASSIVE_OK, 1, 75)
    service.passive_rate_result.emit(res)
    assert "75 Hz" in page.rate_result.text()
