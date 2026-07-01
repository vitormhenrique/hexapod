"""Headless UI tests for the Diagnostics page.

Telemetry records are emitted directly as Qt signals and the periodic refresh
handlers are invoked explicitly (no timer wait). Protocol stats and the raw
frame inspector read through the service's client, so those tests attach a
lightweight fake client exposing the diagnostics accessors.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import MsgType

pytest.importorskip("PySide6")

from transport.protocol_client import DiagnosticsSnapshot, RawFrameRecord


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import DiagnosticsPage

    service = ConnectionService()
    page = DiagnosticsPage(service)
    qtbot.addWidget(page)
    return service, page


class _FakeClient:
    """Minimal stand-in exposing the diagnostics accessors the page uses."""

    connected = True

    def __init__(self, snap, frames) -> None:
        self._snap = snap
        self._frames = frames
        self.capture = False

    def diagnostics_snapshot(self):
        return self._snap

    def drain_raw_frames(self):
        frames = list(self._frames)
        self._frames = []
        return frames

    def set_raw_capture(self, enabled):
        self.capture = enabled


def test_timing_cards_update(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.HealthTelemetry(
        uptime_ms=12345, state=4, fault_reason=0, watchdog_missed=3, battery_mv=11840
    )
    service.telemetry.emit(int(tlm.StreamId.HEALTH), rec)
    page._refresh_timing()
    assert "12.3 s" in page.uptime_card._value.text()
    assert page.watchdog_card._value.text() == "3"
    assert "11.84 V" in page.battery_card._value.text()


def test_dxl_error_table_lists_faulted(qtbot) -> None:
    service, page = _make_page(qtbot)
    servos = [
        tlm.ServoStatus(1, 0, 0, 0, 12000, 30, 0x00),
        tlm.ServoStatus(2, 0, 0, 0, 12000, 55, 0x20),  # overload
    ]
    service.telemetry.emit(
        int(tlm.StreamId.SERVO_STATUS), tlm.ServoStatusTelemetry(servos)
    )
    page._refresh_dxl()
    assert page.dxl_table.rowCount() == 1
    assert page.dxl_table.item(0, 0).text() == "#2"
    assert "overload" in page.dxl_table.item(0, 1).text()
    assert "1/2" in page.dxl_lbl.text()


def test_dxl_all_ok(qtbot) -> None:
    service, page = _make_page(qtbot)
    servos = [tlm.ServoStatus(1, 0, 0, 0, 12000, 30, 0x00)]
    service.telemetry.emit(
        int(tlm.StreamId.SERVO_STATUS), tlm.ServoStatusTelemetry(servos)
    )
    page._refresh_dxl()
    assert page.dxl_table.rowCount() == 0
    assert "no hardware errors" in page.dxl_lbl.text()


def test_i2c_summary_counts_stale_and_fault(qtbot) -> None:
    service, page = _make_page(qtbot)
    feet = [
        tlm.FootContact(0, 200, 0),  # AIR
        tlm.FootContact(3, 200, 40),  # LOADED
        tlm.FootContact(5, 0, 0),  # STALE
        tlm.FootContact(6, 0, 0),  # FAULT
    ]
    service.telemetry.emit(
        int(tlm.StreamId.CONTACT_STATE), tlm.ContactStateTelemetry(feet)
    )
    page._refresh_i2c()
    text = page.i2c_lbl.text()
    assert "stale=1" in text
    assert "fault=1" in text
    assert "4 feet" in text


def test_protocol_cards_from_snapshot(qtbot) -> None:
    service, page = _make_page(qtbot)
    snap = DiagnosticsSnapshot(
        rx_frames=42, tx_frames=7, decode_errors=2, raw_captured=5, capture_enabled=True
    )
    service._client = _FakeClient(snap, [])
    service.telemetry.emit(
        int(tlm.StreamId.API_STATS),
        tlm.ApiStatsTelemetry(tx_backlog=3, dropped_per_stream=[1, 0, 2]),
    )
    page._refresh_protocol()
    assert page.rx_card._value.text() == "42"
    assert page.tx_card._value.text() == "7"
    assert page.decode_card._value.text() == "2"
    assert page.backlog_card._value.text() == "3"
    assert page.dropped_card._value.text() == "3"


def test_raw_inspector_appends_frames(qtbot) -> None:
    service, page = _make_page(qtbot)
    frames = [
        RawFrameRecord(
            host_time_ns=0,
            length=20,
            msg_type=int(MsgType.TELEMETRY),
            msg_id=api.MSG_TELEMETRY_BASE + int(tlm.StreamId.HEALTH),
            seq=5,
            payload_len=12,
            ok=True,
            head_hex="de ad be ef",
        ),
        RawFrameRecord(
            host_time_ns=0,
            length=4,
            msg_type=None,
            msg_id=None,
            seq=None,
            payload_len=None,
            ok=False,
            head_hex="00 01",
        ),
    ]
    service._client = _FakeClient(None, frames)
    page._refresh_raw()
    text = page.raw_feed.toPlainText()
    assert "health" in text
    assert "decode-error" in text


def test_capture_toggle_forwards_to_client(qtbot) -> None:
    service, page = _make_page(qtbot)
    fake = _FakeClient(None, [])
    service._client = fake
    page.capture_chk.setChecked(True)
    assert fake.capture is True
    page.capture_chk.setChecked(False)
    assert fake.capture is False


def test_disconnect_resets_cards(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.HealthTelemetry(
        uptime_ms=5000, state=4, fault_reason=0, watchdog_missed=0, battery_mv=12000
    )
    service.telemetry.emit(int(tlm.StreamId.HEALTH), rec)
    page._refresh_timing()
    assert page.uptime_card._value.text() != "--"
    page._on_connected(False)
    assert page.uptime_card._value.text() == "--"
    assert page.dxl_table.rowCount() == 0


def test_refresh_safe_when_disconnected(qtbot) -> None:
    service, page = _make_page(qtbot)
    # No client, no telemetry: a full refresh must not raise.
    page._refresh()
    assert page.rx_card._value.text() == "--"
