"""Headless tests for generic, scan-driven DXL commands."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from hexapod_protocol import api
from PySide6.QtTest import QTest

pytest.importorskip("PySide6")


def _scan_result(*servo_ids: int) -> api.DxlJobResult:
    records = b"".join(
        bytes([servo_id, 29, 0, 42, 1, 1]) for servo_id in servo_ids
    )
    return api.DxlJobResult(
        api.DXL_SLOT_DONE, api.DXL_CODE_OK, bytes([len(servo_ids)]) + records
    )


def _ready_page(qtbot):
    from services import ConnectionService
    from ui.pages import DxlCommandsPage

    service = ConnectionService()
    page = DxlCommandsPage(service)
    qtbot.addWidget(page)
    service.connected.emit(True)
    service.maint_lock_changed.emit(True, 7)
    service.maintenance_setup_changed.emit(False, True, "ready")
    service.dxl_result.emit("scan", _scan_result(7, 2))
    return service, page


def test_scan_builds_generic_rows_sorted_by_servo_id(qtbot) -> None:
    _service, page = _ready_page(qtbot)

    assert page.table.rowCount() == 2
    assert [page.table.item(row, 0).text() for row in range(2)] == ["2", "7"]
    assert page.table.horizontalHeaderItem(0).text() == "ID"
    assert all("Leg" not in column for column in page.COLUMNS)
    assert set(page._editors) == {2, 7}


def test_typing_never_sends_and_incomplete_target_is_rejected(
    qtbot, monkeypatch
) -> None:
    service, page = _ready_page(qtbot)
    commands = []
    monkeypatch.setattr(
        service,
        "set_dxl_positions_with_torque",
        lambda targets: commands.append(targets),
    )
    editor = page._editors[2]

    editor.value.clear()
    QTest.keyClicks(editor.value, "12")
    assert commands == []
    editor.value.clear()
    editor.set_btn.click()
    assert commands == []
    assert "finish entering" in page.status_lbl.text()


def test_single_and_staged_targets_are_keyed_by_servo_id(qtbot, monkeypatch) -> None:
    service, page = _ready_page(qtbot)
    commands = []
    monkeypatch.setattr(
        service,
        "set_dxl_positions_with_torque",
        lambda targets: commands.append(list(targets)),
    )

    first = page._editors[2]
    first.value.clear()
    QTest.keyClicks(first.value, "180")
    first.set_btn.click()
    assert commands == [[(2, 2048)]]

    first.value.clear()
    QTest.keyClicks(first.value, "90")
    second = page._editors[7]
    second.value.clear()
    QTest.keyClicks(second.value, "270")
    assert commands == [[(2, 2048)]]
    page.apply_staged_btn.click()
    assert commands[-1] == [(2, 1024), (7, 3071)]


def test_rescan_clears_discovered_rows_immediately(qtbot, monkeypatch) -> None:
    service, page = _ready_page(qtbot)
    scans = []
    monkeypatch.setattr(service, "dxl_scan", lambda: scans.append(True))

    page.rescan_btn.click()

    assert scans == [True]
    assert page.table.rowCount() == 0
    assert page._editors == {}