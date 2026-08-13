"""Headless launch test for the Dracula app shell (qqi.10).

Builds the MainWindow under offscreen Qt with the theme applied and verifies the
global safety bar (with the emergency-stop) stays visible across every page, and
that navigation switches the stacked page.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")


def test_app_shell_launches_with_estop_on_all_pages(qtbot) -> None:
    from PySide6.QtWidgets import QApplication

    from main_window import MainWindow
    from theme import apply_theme

    app = QApplication.instance() or QApplication([])
    apply_theme(app)
    assert app.styleSheet()  # theme applied

    window = MainWindow()
    qtbot.addWidget(window)
    window.show()

    # All registered pages are present in the stack.
    assert window.stack.count() == 17
    assert set(window._pages) == {
        "connect",
        "overview",
        "mode_safety",
        "gait_lab",
        "foot_contact",
        "passive_pose",
        "leg_lab",
        "joint_matrix",
        "servo_calibration",
        "servo_config",
        "servo_tuning",
        "plots",
        "model",
        "urdf",
        "sensors",
        "rc_troubleshooting",
        "diagnostics",
    }

    # The emergency-stop lives in the always-on safety bar (outside the stack),
    # so navigating to any page keeps it visible.
    for key in window._pages:
        window.nav.select(key)
        assert window.safety_bar.estop.isVisibleTo(window)

    window.close()


def test_safety_bar_estop_triggers_service(qtbot, monkeypatch) -> None:
    from main_window import MainWindow

    window = MainWindow()
    qtbot.addWidget(window)

    fired = []
    monkeypatch.setattr(window.service, "emergency_stop", lambda: fired.append(True))
    window.safety_bar.estop.click()
    assert fired == [True]


def test_literal_ampersands_are_escaped_for_qt_mnemonics(qtbot) -> None:
    from main_window import MainWindow

    window = MainWindow()
    qtbot.addWidget(window)
    assert window.nav._buttons["mode_safety"].text() == "Mode && Safety"
    assert window.nav._buttons["servo_tuning"].text() == "Servo && DXL"

    connect_page = window.stack.widget(window._pages["connect"])
    mode_page = window.stack.widget(window._pages["mode_safety"])
    assert connect_page.title == "Connect && Setup"
    assert mode_page.title == "Mode && Safety Center"


def test_simulation_disables_hardware_only_navigation(qtbot) -> None:
    from main_window import MainWindow

    window = MainWindow()
    qtbot.addWidget(window)

    window.service._set_simulation_mode(True)

    for key in (
        "foot_contact",
        "passive_pose",
        "leg_lab",
        "joint_matrix",
        "servo_config",
        "servo_tuning",
        "sensors",
        "rc_troubleshooting",
    ):
        assert not window.nav._buttons[key].isEnabled()
        assert "ROS simulated firmware" in window.nav._buttons[key].toolTip()
    assert window.nav._buttons["gait_lab"].isEnabled()
    assert window.stack.currentIndex() == window._pages["gait_lab"]


def test_app_main_entry_point_launches_headless(monkeypatch) -> None:
    """Exercise the hexapod-companion entry point end-to-end without blocking.

    ``app.main`` builds the QApplication, applies the theme, shows the window and
    calls ``app.exec``; stubbing ``exec`` lets the full launch path run headless.
    """
    import app as app_module

    executed = []
    monkeypatch.setattr(
        "PySide6.QtWidgets.QApplication.exec",
        lambda self: executed.append(True) or 0,
    )

    rc = app_module.main()
    assert rc == 0
    assert executed == [True]
