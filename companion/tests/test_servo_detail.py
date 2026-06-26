"""Headless tests for the per-servo detail inspector (eax.7).

Covers the standalone :class:`ServoDetailPanel` widget and its drill-down wiring
inside the Servo Monitor page: selecting a table row focuses a servo, the
``servo_goals`` stream supplies the commanded target tick, and a torque-limit
``dxl_result`` populates the torque-limit field.
"""

from __future__ import annotations

import os
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def test_decode_hw_error_bits() -> None:
    assert tlm.decode_hw_error(0x00) == []
    assert tlm.decode_hw_error(0x01) == ["input voltage"]
    assert tlm.decode_hw_error(0x20) == ["overload"]
    # bit0 (voltage) + bit5 (overload), low bit first.
    assert tlm.decode_hw_error(0x21) == ["input voltage", "overload"]


def test_detail_panel_select_and_update(qtbot) -> None:
    from ui.widgets import ServoDetailPanel

    panel = ServoDetailPanel()
    qtbot.addWidget(panel)

    # Ignores updates until a servo is selected.
    panel.update_status(tlm.ServoStatus(3, 2048, 0, 9, 11700, 39, 0x00, True))
    assert panel.servo_id is None

    panel.select_servo(3)
    assert panel.servo_id == 3
    panel.set_target_tick(2000)
    panel.set_torque_limit(900)
    panel.update_status(tlm.ServoStatus(3, 2048, 0, 9, 11700, 39, 0x21, True))

    assert panel._fields["position"].text() == "2048"
    assert panel._fields["target"].text() == "2000"
    assert panel._fields["torque"].text() == "ON"
    assert panel._fields["current"].text() == "9"
    assert panel._fields["torque_limit"].text() == "900"
    assert "V" in panel._fields["voltage"].text()
    assert "overload" in panel._fields["error"].text()
    # Sparkline got one present sample and a matching target sample.
    xs, ys = panel._present_curve.getData()
    assert list(ys) == [2048.0]

    # An update for a different servo id is ignored.
    panel.update_status(tlm.ServoStatus(4, 1000, 0, 0, 11700, 30, 0, False))
    xs, ys = panel._present_curve.getData()
    assert list(ys) == [2048.0]


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import ServoTuningPage

    service = ConnectionService()
    page = ServoTuningPage(service)
    qtbot.addWidget(page)
    return service, page


def test_row_selection_drills_into_detail(qtbot) -> None:
    service, page = _make_page(qtbot)
    record = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(1, 2048, 0, 5, 11700, 38, 0x00, True),
            tlm.ServoStatus(2, 1024, -3, 7, 11650, 41, 0x04, False),
        ]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), record)

    page.table.selectRow(0)  # fires itemSelectionChanged -> _on_row_selected
    assert page.detail.servo_id == 1
    assert page.detail._fields["position"].text() == "2048"


def test_servo_goals_supplies_target_tick(qtbot) -> None:
    service, page = _make_page(qtbot)
    status = tlm.ServoStatusTelemetry(
        servos=[tlm.ServoStatus(1, 2048, 0, 5, 11700, 38, 0x00, True)]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    page.table.selectRow(0)

    # Servo id 1 maps to leg 0 / joint 0 in the default servo-map.
    goals = tlm.ServoGoalsTelemetry(
        goals=[tlm.ServoGoal(leg=0, joint=0, angle_centideg=0, clamped=False)]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_GOALS), goals)
    # angle 0 deg -> center tick (2048) for an untrimmed servo.
    assert page.detail._fields["target"].text() == "2048"


def test_torque_limit_dxl_result_feeds_panel(qtbot) -> None:
    service, page = _make_page(qtbot)
    status = tlm.ServoStatusTelemetry(
        servos=[tlm.ServoStatus(1, 2048, 0, 5, 11700, 38, 0x00, True)]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    page.table.selectRow(0)

    blob = bytes([api.DXL_PARAM_TORQUE_LIMIT, 1, 4]) + struct.pack("<i", 880)
    res = api.DxlJobResult(api.DXL_SLOT_DONE, api.DXL_CODE_OK, blob)
    service.dxl_result.emit("get_param", res)
    assert page.detail._fields["torque_limit"].text() == "880"
