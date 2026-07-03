"""Headless tests for telemetry-stall banners and state-gated controls.

The pages must only offer actions the firmware would accept: maintenance and
passive transitions follow the tracked safety state and lock, and pages that
render live data show a warning banner while their streams are missing/stale.
"""

from __future__ import annotations

import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")

from types import SimpleNamespace

from hexapod_protocol import telemetry as tlm

# --- TelemetryBanner ---------------------------------------------------------


def _make_stub_service():
    from PySide6.QtCore import QObject, Signal

    class StubService(QObject):
        telemetry = Signal(int, object)
        connected = Signal(bool)

        def __init__(self) -> None:
            super().__init__()
            self.link_up = False

        @property
        def is_connected(self) -> bool:
            return self.link_up

    return StubService()


def test_banner_hidden_until_stream_goes_stale(qtbot) -> None:
    from ui.widgets import TelemetryBanner

    svc = _make_stub_service()
    banner = TelemetryBanner(svc, [(int(tlm.StreamId.SERVO_STATUS), "servo_status")])
    qtbot.addWidget(banner)
    banner.STALE_AFTER_S = 0.01

    # Disconnected: never shown.
    banner._evaluate()
    assert banner.isHidden()

    # Connected but no data after the grace window: shown with the stream name.
    svc.link_up = True
    svc.connected.emit(True)
    time.sleep(0.02)
    banner._evaluate()
    assert not banner.isHidden()
    assert "servo_status" in banner._label.text()

    # Data flows: hides again.
    svc.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), object())
    banner._evaluate()
    assert banner.isHidden()

    # Disconnect: hidden regardless.
    svc.link_up = False
    svc.connected.emit(False)
    assert banner.isHidden()


def test_banner_require_any_satisfied_by_one_stream(qtbot) -> None:
    from ui.widgets import TelemetryBanner

    svc = _make_stub_service()
    banner = TelemetryBanner(
        svc,
        [
            (int(tlm.StreamId.JOINT_STATE), "joint_state"),
            (int(tlm.StreamId.SERVO_STATUS), "servo_status"),
        ],
        require_all=False,
    )
    qtbot.addWidget(banner)
    banner.STALE_AFTER_S = 0.01
    svc.link_up = True
    svc.connected.emit(True)
    time.sleep(0.02)
    banner._evaluate()
    assert not banner.isHidden()
    # One fresh stream satisfies require_any.
    svc.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), object())
    banner._evaluate()
    assert banner.isHidden()


def test_banner_set_active_suspends(qtbot) -> None:
    from ui.widgets import TelemetryBanner

    svc = _make_stub_service()
    banner = TelemetryBanner(svc, [(int(tlm.StreamId.JOINT_STATE), "joint_state")])
    qtbot.addWidget(banner)
    banner.STALE_AFTER_S = 0.01
    svc.link_up = True
    svc.connected.emit(True)
    time.sleep(0.02)
    banner.set_active(False)  # e.g. replay mode
    banner._evaluate()
    assert banner.isHidden()
    banner.set_active(True)
    banner._evaluate()
    assert not banner.isHidden()


# --- ConnectionService state tracking ----------------------------------------


def test_service_tracks_robot_state(qtbot) -> None:
    from services import ConnectionService

    svc = ConnectionService()
    assert svc.robot_state == -1
    with qtbot.waitSignal(svc.state_changed, timeout=1000) as blocker:
        svc.status_received.emit(SimpleNamespace(state=8))
    assert blocker.args == [8]
    assert svc.robot_state == 8


# --- Mode & Safety gating -----------------------------------------------------


def _mode_safety_page(qtbot):
    from services import ConnectionService
    from ui.pages import ModeSafetyPage

    svc = ConnectionService()
    page = ModeSafetyPage(svc)
    qtbot.addWidget(page)
    return svc, page


def test_mode_safety_gates_disconnected(qtbot) -> None:
    svc, page = _mode_safety_page(qtbot)
    for btn in (
        page.arm_btn,
        page.disarm_btn,
        page.enter_maint_btn,
        page.exit_maint_btn,
        page.enter_passive_btn,
        page.exit_passive_btn,
        *page._bench_buttons,
    ):
        assert not btn.isEnabled()


def test_mode_safety_gates_follow_state_and_lock(qtbot) -> None:
    svc, page = _mode_safety_page(qtbot)
    svc.connected.emit(True)

    # Unknown state: entry actions available, exits not.
    assert page.arm_btn.isEnabled()
    assert page.enter_maint_btn.isEnabled()
    assert not page.exit_maint_btn.isEnabled()
    assert not page.exit_passive_btn.isEnabled()

    # Passive pose (9): exit passive available, arm/enter-maint not.
    svc.state_changed.emit(9)
    assert page.exit_passive_btn.isEnabled()
    assert not page.arm_btn.isEnabled()
    assert not page.enter_maint_btn.isEnabled()
    assert page.enter_maint_btn.toolTip() != ""

    # Maintenance lock held: bench + exit-maint open up.
    svc.state_changed.emit(8)
    svc.maint_lock_changed.emit(True, 42)
    assert page.exit_maint_btn.isEnabled()
    for btn in page._bench_buttons:
        assert btn.isEnabled()
    assert not page.enter_maint_btn.isEnabled()

    # Lock released in Disarmed: bench closes, enter reopens.
    svc.maint_lock_changed.emit(False, 0)
    svc.state_changed.emit(2)
    for btn in page._bench_buttons:
        assert not btn.isEnabled()
    assert page.enter_maint_btn.isEnabled()

    # Disconnect closes everything again.
    svc.connected.emit(False)
    assert not page.arm_btn.isEnabled()
    assert not page.enter_maint_btn.isEnabled()


def test_mode_safety_clear_fault_gated_to_fault_states(qtbot) -> None:
    svc, page = _mode_safety_page(qtbot)
    svc.connected.emit(True)
    svc.state_changed.emit(2)  # DISARMED: nothing to clear
    assert not page.clear_btn.isEnabled()
    svc.state_changed.emit(12)  # ESTOP
    assert page.clear_btn.isEnabled()


# --- Leg Lab gating -------------------------------------------------------


def test_leg_lab_send_buttons_need_lock(qtbot) -> None:
    from services import ConnectionService
    from ui.pages import LegLabPage

    svc = ConnectionService()
    page = LegLabPage(svc)
    qtbot.addWidget(page)

    assert not page.send_foot_btn.isEnabled()
    svc.connected.emit(True)
    assert not page.send_foot_btn.isEnabled()  # still needs the lock
    svc.maint_lock_changed.emit(True, 7)
    assert page.send_foot_btn.isEnabled()
    assert page.send_joint_btn.isEnabled()
    assert page.exit_maint_btn.isEnabled()
    assert not page.enter_maint_btn.isEnabled()
    svc.maint_lock_changed.emit(False, 0)
    assert not page.send_foot_btn.isEnabled()


# --- Passive pose gating ----------------------------------------------------


def test_passive_page_enter_exit_follow_state(qtbot) -> None:
    from services import ConnectionService
    from ui.pages import PassivePosePage

    svc = ConnectionService()
    page = PassivePosePage(svc)
    qtbot.addWidget(page)

    assert not page.enter_btn.isEnabled()
    svc.connected.emit(True)
    assert page.enter_btn.isEnabled()  # unknown state: allowed
    assert not page.exit_btn.isEnabled()
    svc.state_changed.emit(9)
    assert not page.enter_btn.isEnabled()
    assert page.exit_btn.isEnabled()
    svc.state_changed.emit(2)
    assert page.enter_btn.isEnabled()
    assert not page.exit_btn.isEnabled()
