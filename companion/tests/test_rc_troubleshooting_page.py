"""Tests for the ControllerBridge-only RC Troubleshooting page."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import RcTroubleshootingPage

    service = ConnectionService()
    page = RcTroubleshootingPage(service)
    qtbot.addWidget(page)
    return service, page


def test_connect_subscribes_to_controller_and_radio_health(qtbot, monkeypatch) -> None:
    service, page = _make_page(qtbot)
    subscriptions = []
    monkeypatch.setattr(
        service,
        "acquire_stream",
        lambda owner, stream_id, rate_hz: subscriptions.append(
            (owner, stream_id, rate_hz)
        ),
    )

    page._on_connected(True)

    assert subscriptions == [
        (page, int(tlm.StreamId.CONTROLLER_STATE), 20),
        (page, int(tlm.StreamId.RC_INPUT), 20),
        (page, int(tlm.StreamId.RC_DIAGNOSTICS), 10),
    ]


def test_radio_diagnostics_show_link_quality_and_correct_bindings(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.telemetry.emit(
        int(tlm.StreamId.RC_DIAGNOSTICS),
        tlm.RcDiagnosticsTelemetry(
            ever_seen=True,
            failsafe=False,
            link_stats_valid=True,
            frames_decoded=123,
            crc_errors=2,
            last_frame_age_ms=14,
            link_stats=tlm.RcLinkStats(
                up_rssi_ant1=67,
                up_link_quality=92,
            ),
        ),
    )

    assert page.link["state"]._value.text() == "linked"
    assert page.link["rssi"]._value.text() == "-67 dBm"
    assert page.link["lq"]._value.text() == "92%"
    assert page.link["age"]._value.text() == "14 ms"
    assert page.link["crc"]._value.text() == "2"
    assert page.raw_table.item(page.raw_rows["enc1"], 1).text() == "ENC1"
    assert "capture" in page.raw_table.item(page.raw_rows["btn4"], 1).text()


def test_controller_state_updates_bridge_intent_and_raw_inputs(qtbot) -> None:
    service, page = _make_page(qtbot)
    rec = tlm.ControllerStateTelemetry(
        valid=True,
        failsafe=False,
        ever_seen=True,
        arm_request=True,
        estop=False,
        host_authority=True,
        feat_foot_contact=True,
        feat_terrain_leveling=False,
        feat_passive_pose=True,
        mode=1,
        gait_index=2,
        trick=3,
        twist_vx=0.25,
        twist_vy=-0.5,
        twist_wz=0.75,
        pose_x_mm=10.0,
        pose_y_mm=-20.0,
        pose_z_mm=30.0,
        pose_roll=0.1,
        pose_pitch=-0.2,
        pose_yaw=0.3,
        trim_roll=0.01,
        trim_pitch=-0.02,
        speed=0.4,
        body_height=0.5,
        stride=0.6,
        step_height=0.7,
        raw=tlm.ControllerRawInputs(
            gimbal=[100, -200, 300, -400],
            pot=[400, 500],
            encoder=[12, 34],
            switches=[True, False, True, False, True, True, False, False],
            buttons=[True, False, False, True],
            toggles=[1, 2],
            nav=[
                [True, False, False, False, False],
                [False, True, False, False, True],
            ],
        ),
    )
    service.telemetry.emit(int(tlm.StreamId.CONTROLLER_STATE), rec)

    assert page.bridge["valid"]._value.text() == "valid"
    assert page.bridge["arm"]._value.text() == "REQUESTED"
    assert page.bridge["estop"]._value.text() == "clear"
    assert page.bridge["host"]._value.text() == "requested"
    assert page.bridge["mode"]._value.text() == "Translate body"
    assert page.bridge["gait"]._value.text() == "2"
    assert page.bridge["trick"]._value.text() == "Wave"
    assert page.bridge["contact"]._value.text() == "requested"
    assert page.bridge["leveling"]._value.text() == "off"
    assert page.bridge["passive"]._value.text() == "requested"
    assert page.command["twist"]._value.text() == "+0.25, -0.50, +0.75"
    assert page.raw_table.item(page.raw_rows["sw_a"], 2).text() == "ON"
    assert page.raw_table.item(page.raw_rows["sw_b"], 2).text() == "off"
    assert page.raw_table.item(page.raw_rows["sw_e"], 2).text() == "CENTER"
    assert page.raw_table.item(page.raw_rows["sw_f"], 2).text() == "DOWN"
    assert page.raw_table.item(page.raw_rows["nav2_c"], 2).text() == "ON"


def test_disconnect_resets_page(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.telemetry.emit(
        int(tlm.StreamId.CONTROLLER_STATE),
        tlm.ControllerStateTelemetry(
            valid=True,
            failsafe=False,
            ever_seen=True,
            arm_request=True,
            estop=False,
            raw=tlm.ControllerRawInputs(switches=[True] + [False] * 7),
        ),
    )
    assert page.bridge["arm"]._value.text() == "REQUESTED"
    assert page.raw_table.item(page.raw_rows["sw_a"], 2).text() == "ON"

    page._on_connected(False)
    assert page.bridge["arm"]._value.text() == "--"
    assert page.raw_table.item(page.raw_rows["sw_a"], 2).text() == "--"
