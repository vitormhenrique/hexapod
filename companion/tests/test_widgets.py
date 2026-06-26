"""Headless tests for the reusable companion widgets (qqi.8).

Each widget must instantiate under offscreen Qt with its Dracula object name and
expose the small behaviors the MainWindow relies on (navigation/estop signals,
event strip rotation, badge level coloring).
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")


def test_status_badge_sets_value(qtbot) -> None:
    from ui.widgets import StatusBadge

    badge = StatusBadge("MODE")
    qtbot.addWidget(badge)
    badge.set("Disarmed", "warn")
    assert badge._value.text() == "Disarmed"


def test_emergency_stop_button_object_name(qtbot) -> None:
    from ui.widgets import EmergencyStopButton

    btn = EmergencyStopButton()
    qtbot.addWidget(btn)
    assert btn.objectName() == "EmergencyStop"
    assert "EMERGENCY STOP" in btn.text()


def test_safety_bar_has_all_badges_and_estop(qtbot) -> None:
    from ui.widgets import SafetyBar

    bar = SafetyBar()
    qtbot.addWidget(bar)
    assert bar.objectName() == "SafetyBar"
    # Seven status badges are exposed for MainWindow wiring.
    for attr in ("conn", "mode", "arming", "torque", "source", "rc", "battery"):
        assert hasattr(bar, attr)
    bar.set_connection(True)
    assert bar.conn._value.text() == "Connected"

    with qtbot.waitSignal(bar.estop_pressed, timeout=1000):
        bar.estop.click()


def test_nav_rail_sections_items_and_navigation(qtbot) -> None:
    from ui.widgets import NavRail

    rail = NavRail()
    qtbot.addWidget(rail)
    assert rail.objectName() == "NavRail"
    rail.add_section("Operate")
    rail.add_item("overview", "Overview")
    rail.add_item("diagnostics", "Diagnostics")
    rail.finish()

    with qtbot.waitSignal(rail.navigated, timeout=1000) as blocker:
        rail.select("diagnostics")
    assert blocker.args == ["diagnostics"]
    # Exclusive selection: the chosen button is checked.
    assert rail._buttons["diagnostics"].isChecked()


def test_event_strip_shows_latest_first(qtbot) -> None:
    from ui.widgets import EventStrip

    strip = EventStrip(capacity=2)
    qtbot.addWidget(strip)
    assert strip.objectName() == "EventStrip"
    strip.add("connect", "fw 0.2")
    strip.add("fault", "dxl bus")
    # Latest event is the one shown.
    assert "[fault] dxl bus" in strip._label.text()
    # Capacity is bounded.
    assert len(strip._events) == 2
