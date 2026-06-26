"""Headless UI smoke tests for the animated model viewer.

Builds the pyqtgraph widget + page off-screen and pushes a pose so the render
path is exercised without a display or a live serial link.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import config as cfg
from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")


def test_hexapod_view_renders_pose(qtbot) -> None:
    from models import HexapodPoseModel
    from ui.widgets import HexapodView

    view = HexapodView()
    qtbot.addWidget(view)
    model = HexapodPoseModel(cfg.default_robot_config())
    view.set_legs(model.legs())
    view._redraw()  # force a synchronous repaint
    # Six leg polylines, each with four chain points.
    assert len(view._top_lines) == 6
    xs, ys = view._top_lines[0].getData()
    assert len(xs) == 4 and len(ys) == 4
    view.stop()


def test_model_viewer_page_consumes_joint_state(qtbot) -> None:
    from services import ConnectionService
    from ui.pages import ModelViewerPage

    service = ConnectionService()
    page = ModelViewerPage(service)
    qtbot.addWidget(page)

    record = tlm.JointStateTelemetry(
        joints=[tlm.JointAngle(leg=0, joint=1, angle_centideg=1500)]
    )
    service.telemetry.emit(int(tlm.StreamId.JOINT_STATE), record)
    page.view._redraw()
    # Source badge flipped to the live joint_state feed.
    assert page._last_joint_state_ms > 0


def test_model_viewer_servo_status_fallback(qtbot) -> None:
    from services import ConnectionService
    from ui.pages import ModelViewerPage

    service = ConnectionService()
    page = ModelViewerPage(service)
    qtbot.addWidget(page)

    config = cfg.default_robot_config()
    smap = cfg.ServoMap(config)
    servo = smap.servo_for(2, 2)
    cmd = cfg.angle_to_tick(servo, 0.1)
    status = tlm.ServoStatusTelemetry(
        servos=[
            tlm.ServoStatus(
                id=servo.id,
                position=cmd.tick,
                velocity=0,
                load=0,
                voltage_mv=12000,
                temperature_c=30,
                hardware_error=0,
            )
        ]
    )
    # No joint_state yet -> servo_status drives the pose.
    service.telemetry.emit(int(tlm.StreamId.SERVO_STATUS), status)
    page.view._redraw()
    assert "servo_status" in page.source_badge._value.text()
