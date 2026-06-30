"""Headless UI tests for the Mode & Safety Center page.

The page is built off-screen with a disconnected :class:`ConnectionService`, so
control buttons exercise the "not connected" guard (an ``error`` signal) and the
feature cards update purely from a ``feature_list`` emission.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import ModeSafetyPage

    service = ConnectionService()
    page = ModeSafetyPage(service)
    qtbot.addWidget(page)
    return service, page


def test_feature_cards_reflect_feature_list(qtbot) -> None:
    service, page = _make_page(qtbot)
    fl = api.FeatureList(
        state=2,
        features=[
            api.FeatureState(
                api.FEATURE_FOOT_CONTACT, True, True, api.FEATURE_REASON_NONE
            ),
            api.FeatureState(
                api.FEATURE_TERRAIN_LEVELING,
                False,
                False,
                api.FEATURE_REASON_HARDWARE_MISSING,
            ),
        ],
    )
    service.feature_list.emit(fl)

    foot = page._feature_cards[api.FEATURE_FOOT_CONTACT]
    terrain = page._feature_cards[api.FEATURE_TERRAIN_LEVELING]
    assert foot._btn.isChecked() is True
    assert foot._btn.isEnabled() is True
    # Unavailable feature: button disabled + reason shown.
    assert terrain._btn.isEnabled() is False
    assert "hardware missing" in terrain._reason.text()


def test_control_button_when_disconnected_emits_error(qtbot) -> None:
    service, page = _make_page(qtbot)
    with qtbot.waitSignal(service.error, timeout=2000) as blocker:
        # Arm is a non-dangerous control (no confirm dialog) -> calls the service.
        service.set_arming(True)
    (msg,) = blocker.args
    assert "not connected" in msg


def test_control_result_updates_action_label(qtbot) -> None:
    service, page = _make_page(qtbot)
    result = api.ControlResult(api.CTRL_OK, 4, 0)
    service.control_result.emit("arm", result)
    assert "arm" in page.action_lbl.text()
    assert "STAND_READY" in page.action_lbl.text()


def test_maint_result_updates_lock_label(qtbot) -> None:
    service, page = _make_page(qtbot)
    granted = api.MaintResultMsg(api.MAINT_OK, 8, 0x1234)
    service.maint_result.emit(granted)
    assert "1234" in page.lock_lbl.text() or "4660" in page.lock_lbl.text()
    released = api.MaintResultMsg(api.MAINT_OK, 2, 0)
    service.maint_result.emit(released)
    assert "none" in page.lock_lbl.text()
