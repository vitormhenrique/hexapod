"""Headless UI tests for the Leg Lab maintenance page.

The page is built off-screen with a disconnected :class:`ConnectionService`.
Leg/joint target results and live ``leg_state`` telemetry are emitted directly
as Qt signals so the page reactions can be exercised without hardware.
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
    from ui.pages import LegLabPage

    service = ConnectionService()
    page = LegLabPage(service)
    qtbot.addWidget(page)
    return service, page


def test_leg_select_lists_all_legs(qtbot) -> None:
    from hexapod_protocol import config as cfg

    _service, page = _make_page(qtbot)
    assert page.leg_combo.count() == cfg.NUM_LEGS
    assert page.leg_combo.itemData(0) == 0
    assert page.leg_combo.itemData(cfg.NUM_LEGS - 1) == cfg.NUM_LEGS - 1


def test_each_leg_defaults_to_configured_safe_home(qtbot) -> None:
    _service, page = _make_page(qtbot)
    for leg in range(page.leg_combo.count()):
        page.leg_combo.setCurrentIndex(leg)
        foot = page._workspace_model.home_foot(leg)
        assert page._foot_spins["x"].value() == round(foot.x)
        assert page._foot_spins["y"].value() == round(foot.y)
        assert page._foot_spins["z"].value() == round(foot.z)
        assert page._target_reachable
        assert page.reach_badge._value.text() == "SAFE"


def test_joint_defaults_to_180_and_sends_relative_centidegrees(
    qtbot, monkeypatch
) -> None:
    service, page = _make_page(qtbot)
    sent = []
    monkeypatch.setattr(
        service,
        "set_joint_target",
        lambda leg, joint, angle: sent.append((leg, joint, angle)),
    )

    assert page.joint_angle.minimum() == 106
    assert page.joint_angle.maximum() == 254
    assert page.joint_angle.value() == 180
    assert "1195–2901 ticks" in page.joint_limit_hint.text()
    page._send_joint_target()
    assert sent[-1] == (0, 0, 0)

    page.joint_angle.setValue(195)
    page._send_joint_target()
    assert sent[-1] == (0, 0, 1500)


def test_unreachable_xyz_is_flagged_and_send_is_blocked(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.connected.emit(True)
    service.maint_lock_changed.emit(True, 7)
    assert page.send_foot_btn.isEnabled()

    for spin in page._foot_spins.values():
        spin.setValue(350)
    assert not page._target_reachable
    assert page.reach_badge._value.text() == "OUTSIDE"
    assert not page.send_foot_btn.isEnabled()
    assert "blocked" in page.reach_detail.text()

    page._set_safe_home()
    assert page._target_reachable
    assert page.send_foot_btn.isEnabled()


def test_leg_target_result_shows_ticks_and_clamp(qtbot) -> None:
    service, page = _make_page(qtbot)
    # bit1 (femur) low-clamped, bit2 (tibia) high-clamped.
    res = api.LegTargetResult(
        result=api.MAINT_TARGET_OK,
        state=8,  # MAC_MAINTENANCE
        reachable=True,
        clamp_low=0b010,
        clamp_high=0b100,
        ticks=(2048, 1500, 2600),
    )
    service.leg_target_result.emit(res)
    text = page.foot_result.text()
    assert "reachable" in text
    assert "coxa=2048" in text and "femur=1500" in text and "tibia=2600" in text
    assert "femur\u2193" in text and "tibia\u2191" in text
    assert "MAC_MAINTENANCE" in text


def test_leg_target_unreachable_not_committed(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.LegTargetResult(
        result=api.MAINT_TARGET_UNREACHABLE,
        state=8,
        reachable=False,
        clamp_low=0,
        clamp_high=0,
        ticks=(0, 0, 0),
    )
    service.leg_target_result.emit(res)
    assert "UNREACHABLE" in page.foot_result.text()


def test_leg_target_failure_emits_message(qtbot) -> None:
    service, page = _make_page(qtbot)
    service.leg_target_result.emit(None)
    assert "failed" in page.foot_result.text()


def test_joint_target_result_shows_tick_and_clamp(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.JointTargetResult(
        result=api.MAINT_TARGET_OK,
        state=8,
        clamped_low=True,
        clamped_high=False,
        tick=1234,
    )
    service.joint_target_result.emit(res)
    text = page.joint_result.text()
    assert "tick=1234" in text
    assert "low" in text


def test_joint_target_rejected(qtbot) -> None:
    service, page = _make_page(qtbot)
    res = api.JointTargetResult(
        result=api.MAINT_TARGET_REJECTED,
        state=2,
        clamped_low=False,
        clamped_high=False,
        tick=0,
    )
    service.joint_target_result.emit(res)
    assert "rejected" in page.joint_result.text()


def test_maint_result_updates_lock_label(qtbot) -> None:
    service, page = _make_page(qtbot)
    # MaintResultMsg(result, state, token); a nonzero token means lock held.
    service.maint_result.emit(api.MaintResultMsg(api.MAINT_OK, 0, 0x1234))
    assert "held" in page.lock_lbl.text()
    assert "4660" in page.lock_lbl.text()  # 0x1234


def test_leg_state_telemetry_populates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    record = tlm.LegStateTelemetry(
        legs=[
            tlm.LegTarget(0, 120, -10, -90, reachable=True, clamped=False),
            tlm.LegTarget(1, 0, 0, 0, reachable=False, clamped=False),
        ]
    )
    service.telemetry.emit(int(tlm.StreamId.LEG_STATE), record)
    assert page.leg_table.rowCount() == 2
    assert page.leg_table.item(0, 0).text() == "0"
    assert page.leg_table.item(0, 1).text() == "120"
    assert page.leg_table.item(0, 4).text() == "reachable"
    assert page.leg_table.item(1, 4).text() == "UNREACHABLE"
    # A repeat update for the same legs must not add rows.
    service.telemetry.emit(int(tlm.StreamId.LEG_STATE), record)
    assert page.leg_table.rowCount() == 2


def test_send_target_when_disconnected_is_safe(qtbot) -> None:
    service, page = _make_page(qtbot)
    errors = []
    service.error.connect(lambda m: errors.append(m))
    page._send_foot_target()
    page._send_joint_target()
    assert any("leg target" in e for e in errors)
    assert any("joint target" in e for e in errors)
