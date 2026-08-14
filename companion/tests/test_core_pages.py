"""Headless construction + behavior tests for the core companion pages (qqi.9).

Each page must build with a disconnected :class:`ConnectionService` under
offscreen Qt and react to the service signals it subscribes to.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def _service():
    from services import ConnectionService

    return ConnectionService()


def test_connect_page_builds_and_shows_firmware(qtbot) -> None:
    from ui.pages import ConnectPage

    service = _service()
    page = ConnectPage(service)
    qtbot.addWidget(page)
    # refresh_ports ran during build; combo has at least the placeholder.
    assert page.port_combo.count() >= 1

    hello = api.HelloInfo(0, 1, 0, 2, 3, "HexNav")
    service.hello_received.emit(hello)
    assert page.device_lbl.text() == "HexNav"
    assert page.fw_lbl.text() == "0.2.3"

    caps = api.Capabilities(0, 1, 0, 2, 3, 0x7, "HexNav")
    service.capabilities_received.emit(caps)
    assert page.caps_lbl.text() == "foot_contact, terrain_leveling, sensor_polling"

    # Connected toggles the button enablement.
    service.connected.emit(True)
    assert not page.connect_btn.isEnabled()
    assert page.disconnect_btn.isEnabled()


def test_connect_page_allows_cancel_and_shows_failure(qtbot, monkeypatch) -> None:
    from ui.pages import ConnectPage

    service = _service()
    page = ConnectPage(service)
    qtbot.addWidget(page)
    cancelled = []
    monkeypatch.setattr(service, "disconnect", lambda: cancelled.append(True))

    service.connecting.emit(True)
    assert page.disconnect_btn.text() == "Cancel"
    assert page.disconnect_btn.isEnabled()
    page.disconnect_btn.click()
    assert cancelled == [True]

    service.error.emit("Connection to /dev/fake failed: no HELLO response")
    assert "Connection failed" in page.connection_status.text()


def test_connect_page_prefers_openrb_cu_port(qtbot, monkeypatch) -> None:
    from transport import PortInfo
    from ui.pages import ConnectPage

    service = _service()
    ports = [
        PortInfo("/dev/tty.usbmodem101", "OpenRB-150", "hwid-openrb"),
        PortInfo("/dev/cu.usbmodem101", "OpenRB-150", "hwid-openrb"),
        PortInfo("/dev/cu.Bluetooth-Incoming-Port", "Bluetooth", "hwid-bt"),
    ]
    monkeypatch.setattr(service, "available_ports", lambda: ports)

    page = ConnectPage(service)
    qtbot.addWidget(page)

    assert page.port_combo.currentData() == "/dev/cu.usbmodem101"


def test_overview_page_reflects_status_and_control_state(qtbot) -> None:
    from ui.pages import OverviewPage

    service = _service()
    page = OverviewPage(service)
    qtbot.addWidget(page)

    st = api.StatusInfo(
        uptime_ms=5000,
        state=5,
        dxl_power=True,
        dxl_power_control=True,
        battery_mv=11800,
        watchdog_missed=0,
    )
    service.status_received.emit(st)
    assert page.badges["uptime"]._value.text() == "5 s"
    assert page.badges["watchdog"]._value.text() == "0"

    cs = tlm.ControlStateTelemetry(
        command_source=1,
        motion_authorized=True,
        kill_active=False,
        state=5,
        fault_reason=0,
        motion_gate=True,
    )
    service.telemetry.emit(int(tlm.StreamId.CONTROL_STATE), cs)
    assert page.badges["fault"]._value.text() == "NONE"

    # DXL/I2C health badges populate from live servo + sensor telemetry.
    servos = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(
                id=1,
                position=2048,
                velocity=0,
                load=0,
                voltage_mv=12000,
                temperature_c=30,
                hardware_error=0,
            )
        ]
    )
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), servos)
    assert page.badges["dxl"]._value.text() == "1/18"
    assert page.badges["servo_faults"]._value.text() == "0"

    i2c = tlm.I2cSensorsRawTelemetry(feet=[tlm.FootRaw(proximity=10, pressure_raw=100)])
    service.telemetry.emit(int(tlm.StreamId.I2C_SENSORS_RAW), i2c)
    assert page.badges["i2c"]._value.text() == "1/6"


def test_overview_page_dxl_badge_reflects_power_off(qtbot) -> None:
    from ui.pages import OverviewPage

    service = _service()
    page = OverviewPage(service)
    qtbot.addWidget(page)

    st = api.StatusInfo(
        uptime_ms=10,
        state=2,
        dxl_power=False,
        dxl_power_control=True,
        battery_mv=11800,
        watchdog_missed=0,
    )
    service.status_received.emit(st)
    assert page.badges["dxl"]._value.text() == "bus off"


def test_mode_safety_page_constructs(qtbot) -> None:
    from ui.pages import ModeSafetyPage

    service = _service()
    page = ModeSafetyPage(service)
    qtbot.addWidget(page)
    assert page is not None


def test_mode_safety_disables_hardware_controls_in_simulation(qtbot) -> None:
    from ui.pages import ModeSafetyPage

    service = _service()
    page = ModeSafetyPage(service)
    qtbot.addWidget(page)

    service._set_simulation_mode(True)
    service.connected.emit(True)

    assert not page.enter_maint_btn.isEnabled()


def test_diagnostics_page_shows_telemetry(qtbot) -> None:
    from ui.pages import DiagnosticsPage

    service = _service()
    page = DiagnosticsPage(service)
    qtbot.addWidget(page)

    health = tlm.HealthTelemetry(1000, 5, 0, 2, 11700)
    service.telemetry.emit(int(tlm.StreamId.HEALTH), health)
    page._refresh_timing()
    assert "1.0 s" in page.uptime_card._value.text()
    assert page.watchdog_card._value.text() == "2"
