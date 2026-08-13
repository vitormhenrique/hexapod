"""Headless UI tests for the leg-by-leg fixture calibration wizard."""

from __future__ import annotations

import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import config as cfg
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import ServoCalibrationPage

    service = ConnectionService()
    page = ServoCalibrationPage(service)
    qtbot.addWidget(page)
    return service, page


def _make_ready_page(qtbot):
    service, page = _make_page(qtbot)
    service.connected.emit(True)
    service.maint_lock_changed.emit(True, 7)
    service.maintenance_setup_changed.emit(False, True, "ready: 18 servos scanned")
    return service, page


def _ok_result() -> SimpleNamespace:
    return SimpleNamespace(ok=True, tick=None, result=0)


def _make_centered_page(qtbot, monkeypatch):
    """Ready page with center-all done and command/send calls recorded."""
    service, page = _make_ready_page(qtbot)
    calls = []
    monkeypatch.setattr(
        service, "center_all_joints", lambda: calls.append(("center",))
    )
    monkeypatch.setattr(
        service,
        "set_leg_fixture_pose_raw",
        lambda leg, c, f, t, robot_config=None: calls.append(
            ("pose", leg, c, f, t)
        ),
    )
    monkeypatch.setattr(
        service,
        "set_joint_targets_raw_ticks",
        lambda targets, robot_config=None, ensure_setup=True: calls.append(
            ("send", tuple(targets), ensure_setup)
        ),
    )
    page.center_all_btn.click()
    assert calls == [("center",)]
    service.joint_target_result.emit(_ok_result())  # center-all completed
    return service, page, calls


def test_page_defaults_to_leg1_center_fixture(qtbot) -> None:
    _service, page = _make_page(qtbot)
    assert page._leg == 0
    assert page._fixture_deg == 180
    assert page._tick_spins[0].value() == 2048
    assert page._tick_spins[1].value() == 2048
    assert page._tick_spins[2].value() == 2048
    assert "Leg 1" in page.leg_lbl.text()
    assert "180" in page.fixture_lbl.text()
    assert "1 / 18" in page.step_lbl.text()
    assert not page.center_all_btn.isEnabled()
    assert not page.next_btn.isEnabled()
    assert not page.prev_btn.isEnabled()
    assert all(not b.isEnabled() for b in page._send_btns.values())


def test_command_pose_requires_center_all_first(qtbot, monkeypatch) -> None:
    service, page = _make_ready_page(qtbot)
    sent = []
    monkeypatch.setattr(
        service,
        "set_leg_fixture_pose_raw",
        lambda *a, **kw: sent.append(a),
    )
    assert page.center_all_btn.isEnabled()
    assert not page.next_btn.isEnabled()
    assert "Center all first" in page.next_btn.text()
    page._command_pose()  # direct call: button is disabled
    assert sent == []
    assert "center" in page.pose_status.text().lower()


def test_center_all_then_command_pose_sends_ticks(qtbot, monkeypatch) -> None:
    _service, page, calls = _make_centered_page(qtbot, monkeypatch)
    assert page.next_btn.isEnabled()
    assert page.next_btn.text() == "Move leg to pose"
    page._tick_spins[0].setValue(2015)
    calls.clear()
    page.next_btn.click()
    assert calls == [("pose", 0, 2015, 2048, 2048)]


def test_editing_a_tick_never_commands_the_servo(qtbot, monkeypatch) -> None:
    """Regression: typing (e.g. '2' of '2048') must not send anything."""
    service, page, calls = _make_centered_page(qtbot, monkeypatch)
    page.next_btn.click()  # move leg to pose
    service.joint_target_result.emit(_ok_result())  # pose done
    calls.clear()
    spin = page._tick_spins[0]
    assert not spin.keyboardTracking()
    # Simulate the user clearing the field and retyping: value changes and
    # arrow steps must not reach the service without a button press.
    spin.setValue(2)
    spin.setValue(20)
    spin.setValue(204)
    spin.setValue(2048)
    spin.stepUp()
    spin.stepDown()
    assert calls == []


def test_send_button_commands_only_that_joint(qtbot, monkeypatch) -> None:
    service, page, calls = _make_centered_page(qtbot, monkeypatch)
    # Send requires a commanded pose first.
    assert all(not b.isEnabled() for b in page._send_btns.values())
    page.next_btn.click()  # move leg to pose
    service.joint_target_result.emit(_ok_result())  # pose done
    assert page.next_btn.text() == "Capture & Next \u25b6"
    assert all(b.isEnabled() for b in page._send_btns.values())
    calls.clear()
    page._tick_spins[1].setValue(2052)
    page._send_btns[1].click()
    assert calls == [("send", ((0, 1, 2052),), False)]


def test_capture_advances_fixture_then_fits_leg(qtbot, monkeypatch) -> None:
    service, page, calls = _make_centered_page(qtbot, monkeypatch)

    def _capture_at(deg: int, coxa: int, femur: int, tibia: int) -> None:
        assert page._fixture_deg == deg
        assert page.next_btn.text() == "Move leg to pose"
        page.next_btn.click()  # move leg to pose
        service.joint_target_result.emit(_ok_result())
        assert page.next_btn.text() == "Capture & Next \u25b6"
        for jid, tick in ((0, coxa), (1, femur), (2, tibia)):
            page._tick_spins[jid].setValue(tick)
        page.next_btn.click()  # capture and advance

    g180 = cfg.identity_tick_for_servo_deg(180)
    g135 = cfg.identity_tick_for_servo_deg(135)
    g225 = cfg.identity_tick_for_servo_deg(225)
    _capture_at(180, g180, g180, g180)
    assert page._fixture_deg == 135
    assert page.next_btn.text() == "Move leg to pose"  # new pose, new command
    _capture_at(135, g135, g135, g135)
    assert page._fixture_deg == 225
    _capture_at(225, g225, g225, g225)

    # Leg 0 fitted and advanced to leg 1 / 180°; no auto-command was sent.
    assert page._leg == 1
    assert page._fixture_deg == 180
    assert not page._pose_commanded
    assert 0 in page._fits and page._fits[0]
    fit = page._fits[0][0]
    assert fit.ok and fit.sign == 1 and fit.trim_ticks == 0
    assert "Leg 1" in page.results.toPlainText()


def test_previous_pose_steps_back_without_moving(qtbot, monkeypatch) -> None:
    service, page, calls = _make_centered_page(qtbot, monkeypatch)
    page.next_btn.click()  # move leg to pose
    service.joint_target_result.emit(_ok_result())
    page.next_btn.click()  # capture -> advance to pose 2
    assert page._step == 1
    calls.clear()
    page.prev_btn.click()
    assert page._step == 0
    assert calls == []  # going back never moves the robot
    assert page.next_btn.text() == "Move leg to pose"
    assert not page.prev_btn.isEnabled()  # first pose


def test_apply_stages_fitted_sign_trim(qtbot, monkeypatch) -> None:
    service, page = _make_ready_page(qtbot)
    # Inject a finished fit for leg 0 coxa without walking the UI.
    page._fits[0][0] = cfg.ServoCalibrationFit(
        sign=-1, trim_ticks=12, residual_ticks_rms=0.0, ok=True, detail="ok"
    )
    page._apply_gates()
    assert page.apply_btn.isEnabled()
    staged = []
    monkeypatch.setattr(service, "stage_config", lambda config: staged.append(config))
    page.apply_btn.click()
    assert len(staged) == 1
    servo = next(s for s in staged[0].servos if s.leg == 0 and s.joint == 0)
    assert servo.sign == -1
    assert servo.trim_ticks == 12


def test_present_telemetry_fills_use_present(qtbot) -> None:
    service, page = _make_ready_page(qtbot)
    service.telemetry.emit(
        int(tlm.StreamId.SERVO_STATUS),
        tlm.ServoStatusTelemetry(
            [tlm.ServoStatus(1, 2001, 0, 0, 12000, 25, 0, True)]
        ),
    )
    assert page._present_lbls[0].text() == "2001"
    page._use_present(0)
    assert page._tick_spins[0].value() == 2001
