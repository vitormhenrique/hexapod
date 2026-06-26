"""Headless UI tests for the Servo Monitor & DXL Tuning page.

The page is built off-screen with a disconnected :class:`ConnectionService`.
Live status rows come from a synthetic ``servo_status`` telemetry record, and
the DXL editor reacts to ``dxl_result`` emissions decoded from real job blobs.
"""

from __future__ import annotations

import os
import struct

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import ServoTuningPage

    service = ConnectionService()
    page = ServoTuningPage(service)
    qtbot.addWidget(page)
    return service, page


def test_servo_status_populates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    record = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(1, 2048, 0, 5, 11700, 38, 0x00, True),
            tlm.ServoStatus(2, 1024, -3, 7, 11650, 41, 0x04, False),
        ]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), record)
    assert page.table.rowCount() == 2
    assert page.table.item(0, 0).text() == "1"
    assert page.table.item(0, 7).text() == "ON"
    assert page.table.item(1, 7).text() == "off"
    # A second update for the same ids must not add rows.
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), record)
    assert page.table.rowCount() == 2


def test_get_param_result_updates_value(qtbot) -> None:
    service, page = _make_page(qtbot)
    # DONE GET_PARAM blob: [param, table, len, value(i32)].
    data = bytes([api.DXL_PARAM_TORQUE_LIMIT, 1, 4]) + struct.pack("<i", 950)
    res = api.DxlJobResult(api.DXL_SLOT_DONE, api.DXL_CODE_OK, data)
    service.dxl_result.emit("get_param", res)
    assert page.param_value.value() == 950
    assert "read ok" in page.param_result.text()


def test_set_param_result_reports_verification(qtbot) -> None:
    service, page = _make_page(qtbot)
    # SET_PARAM blob: [param, len, written(i32), readback(i32), ok].
    data = (
        bytes([api.DXL_PARAM_TORQUE_LIMIT, 4])
        + struct.pack("<i", 950)
        + struct.pack("<i", 950)
        + bytes([1])
    )
    res = api.DxlJobResult(api.DXL_SLOT_DONE, api.DXL_CODE_OK, data)
    service.dxl_result.emit("set_param", res)
    assert "verified=True" in page.param_result.text()


def test_failed_dxl_result_marks_target(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.dxl_result.emit("set_limits", None)
    assert "failed" in page.limit_result.text()


def test_expert_panel_hidden_until_gated(qtbot) -> None:
    service, page = _make_page(qtbot)
    # isHidden() reflects the explicit hidden flag regardless of parent show state.
    assert page.expert_body.isHidden() is True
    # Direct flag flip (bypassing the confirm dialog) reveals the body.
    page.expert_body.setVisible(True)
    assert page.expert_body.isHidden() is False
