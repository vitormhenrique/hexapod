"""Headless UI tests for the Foot Contact & Leveling page.

The page is built off-screen with a disconnected :class:`ConnectionService`.
Contact rows come from synthetic ``contact_state``/``i2c_sensors_raw`` telemetry,
and the enable/threshold/calibrate controls react to result-signal emissions.
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
    from ui.pages import FootContactPage

    service = ConnectionService()
    page = FootContactPage(service)
    qtbot.addWidget(page)
    return service, page


def test_table_has_one_row_per_foot(qtbot) -> None:
    _service, page = _make_page(qtbot)
    assert page.table.rowCount() == tlm.NUM_FEET


def test_contact_state_updates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [
        tlm.FootContact(state=i % 7, confidence=10 * i, pressure_delta=i)
        for i in range(tlm.NUM_FEET)
    ]
    service.telemetry.emit(
        int(tlm.StreamId.CONTACT_STATE), tlm.ContactStateTelemetry(feet=feet)
    )
    assert page.table.item(2, 1).text() == "TOUCH"
    assert page.table.item(3, 2).text() == "30"
    assert page.table.item(4, 3).text() == "4"


def test_i2c_raw_updates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [
        tlm.FootRaw(proximity=100 + i, pressure_raw=200 + i)
        for i in range(tlm.NUM_FEET)
    ]
    service.telemetry.emit(
        int(tlm.StreamId.I2C_SENSORS_RAW), tlm.I2cSensorsRawTelemetry(feet=feet)
    )
    assert page.table.item(0, 4).text() == "100"
    assert page.table.item(5, 5).text() == "205"


def test_feature_result_reports_reason(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.SensorFeatureResult(
        api.SENSOR_REJECTED, 0, False, False, api.FEATURE_REASON_HARDWARE_MISSING
    )
    service.sensor_feature_result.emit("contact", res)
    assert "rejected" in page.contact_result.text()
    assert "hardware missing" in page.contact_result.text()


def test_threshold_result_displays_values(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.ContactThresholdResult(api.SENSOR_OK, 2, 100, 200, 300)
    service.contact_threshold_result.emit(res)
    assert "leg 2" in page.thr_result.text()
    assert "near=100" in page.thr_result.text()


def test_calibrate_result_displays_mask(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.SensorCalibrateResult(api.SENSOR_OK, 0x3F)
    service.sensor_calibrate_result.emit(res)
    assert "0x3F" in page.cal_result.text()
