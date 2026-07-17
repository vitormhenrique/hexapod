"""Headless tests for the maintenance Gait Lab page.

The page must build under offscreen Qt, route gait/parameter/twist controls to
the :class:`ConnectionService`, surface the motion gate from control-state
telemetry, and report motion-command verdicts.
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


def _page(qtbot):
    from ui.pages import GaitLabPage

    service = _service()
    page = GaitLabPage(service)
    qtbot.addWidget(page)
    return service, page


def test_gait_lab_builds_with_all_gaits(qtbot) -> None:
    service, page = _page(qtbot)
    assert set(page._gait_buttons) == {
        api.GAIT_STAND,
        api.GAIT_SIT,
        api.GAIT_TRIPOD,
        api.GAIT_RIPPLE,
        api.GAIT_WAVE,
        api.GAIT_CRAWL,
    }
    # Motion gate starts unknown until control state arrives.
    assert page.gate_badge._value.text() == "unknown"


def test_gait_button_sends_set_gait(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait = lambda g: calls.append(g)  # type: ignore[method-assign]
    service.connected.emit(True)
    service.gait_test_changed.emit(True, "running in MacMaintenance")
    page._gait_buttons[api.GAIT_TRIPOD].click()
    assert calls == [api.GAIT_TRIPOD]


def test_motion_controls_require_ready_session(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait = lambda g: calls.append(g)  # type: ignore[method-assign]
    service.connected.emit(True)
    assert not page.gait_box.isEnabled()
    page._gait_buttons[api.GAIT_TRIPOD].click()
    assert calls == []

    service.gait_test_changed.emit(True, "running in MacMaintenance")
    assert page.gait_box.isEnabled()


def test_simulation_session_enables_direct_high_level_motion(qtbot) -> None:
    service, page = _page(qtbot)

    service._set_simulation_mode(True)
    service.connected.emit(True)
    service.gait_test_changed.emit(True, "running in ROS simulation")

    assert page.session_box.title() == "Simulation motion session"
    assert page.start_btn.text() == "Start simulation controls"
    assert page.stop_session_btn.text() == "Stop simulation"
    assert page.gait_box.isEnabled()
    assert page.params_box.isEnabled()
    assert page.walk_box.isEnabled()


def test_start_session_uses_current_gait_parameters(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.start_gait_test = lambda *a: calls.append(a)  # type: ignore[method-assign]
    service.connected.emit(True)
    page._param_spins["body_height"].setValue(45)
    page._param_spins["speed"].setValue(200)
    page.start_btn.click()
    assert calls == [(45, 60, 30, 128, 200)]


def test_busy_session_prevents_duplicate_start_and_allows_stop(qtbot) -> None:
    service, page = _page(qtbot)
    service.connected.emit(True)
    service.gait_test_busy_changed.emit(True)
    assert not page.start_btn.isEnabled()
    assert page.stop_session_btn.isEnabled()


def test_apply_params_sends_gait_params(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait_params = lambda *a: calls.append(a)  # type: ignore[method-assign]
    page._param_spins["body_height"].setValue(45)
    page._param_spins["stride_len"].setValue(70)
    page._param_spins["step_height"].setValue(25)
    page._param_spins["duty"].setValue(120)
    page._param_spins["speed"].setValue(200)
    page._send_gait_params()
    assert calls == [(45, 70, 25, 120, 200)]


def test_pad_press_sends_walking_gait_then_scaled_twist(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait = lambda g: calls.append(("gait", g))  # type: ignore[method-assign]
    service.set_body_twist = lambda *a: calls.append(("twist", *a))  # type: ignore[method-assign]
    page._gait_buttons[api.GAIT_RIPPLE].setChecked(True)
    page._walk_speed.setValue(50)
    page._pad_buttons["forward"].pressed.emit()
    page._pad_buttons["forward"].released.emit()
    assert calls == [
        ("gait", api.GAIT_RIPPLE),
        ("twist", 0.5, 0.0, 0.0),
        ("twist", 0.0, 0.0, 0.0),
    ]


def test_pad_press_auto_selects_tripod_when_posture_checked(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait = lambda g: calls.append(("gait", g))  # type: ignore[method-assign]
    service.set_body_twist = lambda *a: calls.append(("twist", *a))  # type: ignore[method-assign]
    page._gait_buttons[api.GAIT_STAND].setChecked(True)
    page._walk_speed.setValue(100)
    page._pad_buttons["left"].pressed.emit()
    assert calls == [("gait", api.GAIT_TRIPOD), ("twist", 0.0, 1.0, 0.0)]
    assert page._gait_buttons[api.GAIT_TRIPOD].isChecked()


def test_pad_directions_cover_lateral_and_yaw(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_gait = lambda g: None  # type: ignore[method-assign]
    service.set_body_twist = lambda *a: calls.append(a)  # type: ignore[method-assign]
    page._gait_buttons[api.GAIT_WAVE].setChecked(True)
    page._walk_speed.setValue(100)
    page._pad_buttons["left"].pressed.emit()
    page._pad_buttons["yaw_cw"].pressed.emit()
    assert calls == [(0.0, 1.0, 0.0), (0.0, 0.0, -1.0)]


def test_zero_twist_commands_zero(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_body_twist = lambda *a: calls.append(a)  # type: ignore[method-assign]
    page._zero_twist()
    assert calls == [(0.0, 0.0, 0.0)]


def test_control_state_drives_motion_gate(qtbot) -> None:
    service, page = _page(qtbot)
    cs = tlm.ControlStateTelemetry(
        command_source=1,
        motion_authorized=True,
        kill_active=False,
        state=5,
        fault_reason=0,
        motion_gate=True,
    )
    service.telemetry.emit(int(tlm.StreamId.CONTROL_STATE), cs)
    assert page.gate_badge._value.text() == "OPEN"

    cs_closed = tlm.ControlStateTelemetry(
        command_source=0,
        motion_authorized=False,
        kill_active=False,
        state=2,
        fault_reason=0,
        motion_gate=False,
    )
    service.telemetry.emit(int(tlm.StreamId.CONTROL_STATE), cs_closed)
    assert page.gate_badge._value.text() == "CLOSED"


def test_motion_result_updates_authority_label(qtbot) -> None:
    service, page = _page(qtbot)
    rejected = api.MotionResultMsg(api.MOTION_REJECTED, 2, False)
    service.motion_result.emit("set_gait", rejected)
    text = page.authority_lbl.text()
    assert "set_gait" in text and "rejected" in text and "gate CLOSED" in text


def test_disconnect_resets_gate(qtbot) -> None:
    service, page = _page(qtbot)
    cs = tlm.ControlStateTelemetry(
        command_source=1,
        motion_authorized=True,
        kill_active=False,
        state=5,
        fault_reason=0,
        motion_gate=True,
    )
    service.telemetry.emit(int(tlm.StreamId.CONTROL_STATE), cs)
    service.connected.emit(False)
    assert page.gate_badge._value.text() == "unknown"
