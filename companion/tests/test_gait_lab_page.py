"""Headless tests for the Gait Lab page (nzi.3).

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
    page._gait_buttons[api.GAIT_TRIPOD].click()
    assert calls == [api.GAIT_TRIPOD]


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


def test_send_twist_normalises_percentages(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_body_twist = lambda *a: calls.append(a)  # type: ignore[method-assign]
    page._twist_spins["vx"].setValue(50)
    page._twist_spins["vy"].setValue(-25)
    page._twist_spins["wz"].setValue(100)
    page._send_twist()
    assert calls == [(0.5, -0.25, 1.0)]


def test_zero_button_resets_and_commands_zero(qtbot) -> None:
    service, page = _page(qtbot)
    calls = []
    service.set_body_twist = lambda *a: calls.append(a)  # type: ignore[method-assign]
    page._twist_spins["vx"].setValue(80)
    page._zero_twist()
    assert page._twist_spins["vx"].value() == 0
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
