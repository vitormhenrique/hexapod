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

    # The rail exposes workflows, not every implementation page.
    assert window.stack.count() == 5
    assert set(window._workspaces) == {
        "dashboard",
        "motion",
        "servos",
        "analyze",
        "diagnose",
    }

    # Every capability remains directly routable inside its owning workflow.
    assert set(window._page_widgets) == {
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
        "openlog_config",
        "servo_tuning",
        "plots",
        "pose",
        "sensors",
        "rc_troubleshooting",
        "diagnostics",
    }

    # The emergency-stop lives in the always-on safety bar (outside the stack),
    # so routing to any capability keeps it visible.
    for key, page in window._page_widgets.items():
        window.navigate_to(key)
        assert page.isVisibleTo(window)
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
    assert window.nav._buttons["motion"].text() == "Operate"
    assert window.nav._buttons["servos"].text() == "Configure"

    connect_page = window._page_widgets["connect"]
    mode_page = window._page_widgets["mode_safety"]
    assert connect_page.title == "Connection"
    assert mode_page.title == "Safety && Authority"

    pose_page = window._page_widgets["pose"]
    assert pose_page.tabs.count() == 2
    assert pose_page.tabs.tabText(0) == "2D kinematics"
    assert pose_page.tabs.tabText(1) == "3D URDF"


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
        "servo_calibration",
        "servo_config",
        "servo_tuning",
        "sensors",
        "rc_troubleshooting",
    ):
        assert not window.is_page_enabled(key)
        workspace_key, tab_index = window._page_locations[key]
        assert "ROS simulated firmware" in window._workspace_tabs[
            workspace_key
        ].tabToolTip(tab_index)
    assert window.is_page_enabled("gait_lab")
    assert window.stack.currentIndex() == window._workspaces["motion"]
    assert (
        window._workspace_tabs["motion"].currentWidget()
        is window._page_widgets["gait_lab"]
    )


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
