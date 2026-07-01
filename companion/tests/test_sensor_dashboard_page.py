"""Headless UI tests for the Sensor Dashboard & I2C page.

Topology, sensor-status, rate, and calibrate results are emitted directly as Qt
signals; the live foot table is driven by telemetry emissions. No hardware.
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
    from ui.pages import SensorDashboardPage

    service = ConnectionService()
    page = SensorDashboardPage(service)
    qtbot.addWidget(page)
    return service, page


def _topology(mux=True, eeprom=True):
    return api.I2cTopologyResult(
        mux_present=mux,
        eeprom_present=eeprom,
        channels=[
            api.TopologyChannel(True, True, True, 2, 1),  # present
            api.TopologyChannel(True, False, False, 0, 0),  # missing
            api.TopologyChannel(True, True, False, 1, 2),  # fault
        ],
    )


def test_topology_populates_table_and_badges(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.i2c_topology.emit(_topology())
    assert page.topo_table.rowCount() == 3
    assert page.topo_table.item(0, 5).text() == "present"
    assert page.topo_table.item(2, 5).text() == "fault"
    assert "present" in page.mux_badge._value.text()
    assert "present=1" in page.topo_status.text()
    assert "fault=1" in page.topo_status.text()


def test_topology_none_is_handled(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.i2c_topology.emit(None)
    assert "no topology" in page.topo_status.text()


def test_mux_missing_badge_warns(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.i2c_topology.emit(_topology(mux=False))
    assert "missing" in page.mux_badge._value.text()


def test_live_raw_telemetry_fills_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [tlm.FootRaw(proximity=100 + i, pressure_raw=200 + i) for i in range(6)]
    service.telemetry.emit(
        int(tlm.StreamId.I2C_SENSORS_RAW), tlm.I2cSensorsRawTelemetry(feet)
    )
    assert page.live_table.item(0, 4).text() == "100"
    assert page.live_table.item(5, 5).text() == "205"


def test_live_contact_telemetry_fills_state(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [tlm.FootContact(state=3, confidence=180, pressure_delta=40) for _ in range(6)]
    service.telemetry.emit(
        int(tlm.StreamId.CONTACT_STATE), tlm.ContactStateTelemetry(feet)
    )
    assert page.live_table.item(0, 2).text() == "LOADED"
    assert page.live_table.item(0, 3).text() == "180"


def test_sensor_status_updates_present_and_polling(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [
        api.FootStatus(state=3, confidence=200, proximity=1234, pressure_delta=40, flags=0x04),
        api.FootStatus(state=0, confidence=10, proximity=5, pressure_delta=0, flags=0x00),
    ]
    service.sensor_status.emit(
        api.SensorStatusResult(present_mask=0b01, polling_enabled=True, feet=feet)
    )
    assert page.live_table.item(0, 1).text() == "yes"
    assert page.live_table.item(1, 1).text() == "no"
    assert page.live_table.item(0, 2).text() == "LOADED"
    assert "polling=on" in page.live_status.text()


def test_sensor_status_none_is_handled(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.sensor_status.emit(None)
    assert "no status" in page.live_status.text()


def test_rate_result_message(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.sensor_rate_result.emit(api.SensorRateResult(api.SENSOR_OK, 75))
    assert "75 Hz" in page.rate_result.text()
    service.sensor_rate_result.emit(api.SensorRateResult(api.SENSOR_REJECTED, 0))
    assert "rejected" in page.rate_result.text()


def test_calibrate_result_message(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.sensor_calibrate_result.emit(api.SensorCalibrateResult(api.SENSOR_OK, 0x3F))
    assert "0x3F" in page.cal_result.text()


def test_disconnect_clears_tables(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.i2c_topology.emit(_topology())
    feet = [tlm.FootRaw(proximity=1, pressure_raw=2) for _ in range(6)]
    service.telemetry.emit(
        int(tlm.StreamId.I2C_SENSORS_RAW), tlm.I2cSensorsRawTelemetry(feet)
    )
    page._on_connected(False)
    assert page.topo_table.rowCount() == 0
    assert page.live_table.item(0, 4).text() == "--"
    assert "disconnected" in page.mux_badge._value.text()


def test_actions_safe_when_disconnected(qtbot) -> None:
    service, page = _make_page(qtbot)
    errors = []
    service.error.connect(lambda m: errors.append(m))
    service.refresh_i2c_topology()
    service.refresh_sensor_status()
    service.set_sensor_rate(50)
    assert len(errors) == 3
