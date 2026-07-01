"""Tests for the RC Troubleshooting page (a8n).

Drives the page with decoded ``rc_diagnostics`` (raw ticks + link stats) and
``rc_input`` (parsed switches + microseconds) records and checks that both the
raw and parsed views update, without a serial port.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")

MICRO = chr(0xB5)  # µ — must match the page's microsecond suffix exactly.


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import RcTroubleshootingPage

    service = ConnectionService()
    page = RcTroubleshootingPage(service)
    qtbot.addWidget(page)
    return service, page


def test_diagnostics_updates_link_and_raw_ticks(qtbot) -> None:
    service, page = _make_page(qtbot)
    link = tlm.RcLinkStats(
        up_rssi_ant1=70,
        up_rssi_ant2=85,
        up_link_quality=100,
        up_snr=8,
        active_antenna=0,
        rf_mode=6,
        up_tx_power=3,
        down_rssi=60,
        down_link_quality=99,
        down_snr=7,
    )
    rec = tlm.RcDiagnosticsTelemetry(
        ever_seen=True,
        failsafe=False,
        link_stats_valid=True,
        raw_ticks=[172 + i for i in range(16)],
        frames_decoded=1234,
        crc_errors=2,
        link_stats_count=42,
        last_frame_age_ms=12,
        link_stats=link,
    )
    service.telemetry.emit(int(tlm.StreamId.RC_DIAGNOSTICS), rec)

    # Active antenna 0 -> uplink RSSI comes from ant1 (-70 dBm).
    assert page.link["rssi"]._value.text() == "-70 dBm"
    assert page.link["lq"]._value.text() == "100%"
    assert page.link["snr"]._value.text() == "8 dB"
    assert page.link["tx"]._value.text() == "100 mW"
    assert page.link["down_rssi"]._value.text() == "-60 dBm"
    # Frame-health counters.
    assert page.frame["frames"]._value.text() == "1234"
    assert page.frame["crc"]._value.text() == "2"
    assert page.frame["lscount"]._value.text() == "42"
    assert page.frame["age"]._value.text() == "12 ms"
    assert page.frame["link"]._value.text() == "alive"
    # Raw ticks span the whole channel table.
    assert page.table.item(0, 2).text() == "172"
    assert page.table.item(15, 2).text() == "187"


def test_link_stats_invalid_shows_placeholder(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.RcDiagnosticsTelemetry(
        ever_seen=True,
        failsafe=False,
        link_stats_valid=False,
        raw_ticks=[992] * 16,
        frames_decoded=5,
        crc_errors=0,
        link_stats_count=0,
        last_frame_age_ms=8,
    )
    service.telemetry.emit(int(tlm.StreamId.RC_DIAGNOSTICS), rec)

    # No 0x14 frame yet -> link cards stay blank, but counters still update.
    assert page.link["rssi"]._value.text() == "--"
    assert page.link["down_lq"]._value.text() == "--"
    assert page.frame["frames"]._value.text() == "5"


def test_never_seen_marks_link_lost(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.RcDiagnosticsTelemetry(ever_seen=False, last_frame_age_ms=0xFFFF)
    service.telemetry.emit(int(tlm.StreamId.RC_DIAGNOSTICS), rec)

    assert page.frame["link"]._value.text() == "no signal"
    assert page.frame["age"]._value.text() == "never"


def test_rc_input_updates_switches_and_channels(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.RcInputTelemetry(
        armed=True,
        kill=False,
        failsafe=False,
        autonomy=True,
        gait_index=3,
        channels_us=[1000, 1500, 2000, 1250] + [1500] * 12,
    )
    service.telemetry.emit(int(tlm.StreamId.RC_INPUT), rec)

    assert page.sw["armed"]._value.text() == "ARMED"
    assert page.sw["kill"]._value.text() == "clear"
    assert page.sw["autonomy"]._value.text() == "on"
    assert page.sw["gait"]._value.text() == "3"
    # Parsed microseconds + normalized column.
    assert page.table.item(0, 3).text() == f"1000 {MICRO}s"
    assert page.table.item(0, 4).text() == "-1.00"  # (1000-1500)/500
    assert page.table.item(1, 4).text() == "+0.00"  # centered stick
    assert page.table.item(2, 4).text() == "+1.00"  # (2000-1500)/500
    assert page.table.item(3, 4).text() == "-0.50"  # (1250-1500)/500


def test_disconnect_resets_page(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.telemetry.emit(
        int(tlm.StreamId.RC_INPUT),
        tlm.RcInputTelemetry(
            armed=True,
            kill=False,
            failsafe=False,
            autonomy=False,
            gait_index=1,
            channels_us=[1500] * 16,
        ),
    )
    assert page.sw["armed"]._value.text() == "ARMED"

    page._on_connected(False)
    assert page.sw["armed"]._value.text() == "--"
    assert page.table.item(0, 3).text() == "--"
    assert page.table.item(0, 4).text() == "--"
