"""Headless tests for the URDF Viewer page (nxi.4)."""

from __future__ import annotations

import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import telemetry as tlm

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from data.urdf_model import find_hexnav_description  # noqa: E402

from .replay_fixtures import build_sample_session  # noqa: E402

# The viewer needs the HexNav description to render anything meaningful.
pytestmark = pytest.mark.skipif(
    find_hexnav_description() is None,
    reason="HexNav_description package not present in this checkout",
)


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import UrdfViewerPage

    service = ConnectionService()
    page = UrdfViewerPage(service)
    qtbot.addWidget(page)
    return page, service


def test_page_loads_urdf_and_renders_meshes(qtbot) -> None:
    page, _ = _make_page(qtbot)
    assert page._ok is True
    # One rendered 3D mesh per HexNav link.
    assert page.mesh_count() == 38


def test_live_joint_state_updates_pose(qtbot) -> None:
    page, service = _make_page(qtbot)
    record = tlm.JointStateTelemetry(
        joints=[tlm.JointAngle(leg=0, joint=1, angle_centideg=3000)]
    )
    service.telemetry.emit(int(tlm.StreamId.JOINT_STATE), record)
    assert page.current_angles()["leg_1_femur_joint"] == pytest.approx(
        math.radians(30.0)
    )


def test_live_ignored_in_replay_mode(qtbot) -> None:
    page, service = _make_page(qtbot)
    page.set_mode("replay")
    record = tlm.JointStateTelemetry(
        joints=[tlm.JointAngle(leg=0, joint=0, angle_centideg=4500)]
    )
    service.telemetry.emit(int(tlm.StreamId.JOINT_STATE), record)
    assert "leg_1_coxa_joint" not in page.current_angles()


def test_subscribes_on_connect(qtbot) -> None:
    page, service = _make_page(qtbot)
    calls: list[tuple[int, int]] = []
    service.subscribe = lambda sid, rate: calls.append((sid, rate))  # type: ignore
    service.connected.emit(True)
    subscribed = {sid for sid, _ in calls}
    assert int(tlm.StreamId.JOINT_STATE) in subscribed
    assert int(tlm.StreamId.SERVO_STATUS) in subscribed


def test_replay_loads_and_scrubs(qtbot, tmp_path) -> None:
    page, _ = _make_page(qtbot)
    replay = build_sample_session(tmp_path, frames_per_stream=4)
    page.load_session(replay.dir)
    assert page._mode == "replay"
    # The fixture records 4 joint_state frames.
    assert page.replay_frame_count() == 4
    page.set_replay_index(2)
    assert page._replay_idx == 2
    # Fixture joint_state carries leg/joint angles -> URDF names populated.
    assert page.current_angles()  # non-empty


def test_replay_play_advances_and_stops(qtbot, tmp_path) -> None:
    page, _ = _make_page(qtbot)
    replay = build_sample_session(tmp_path, frames_per_stream=3)
    page.load_session(replay.dir)
    page.set_replay_index(0)
    # Manually advance through the timer callback (no real wall-clock wait).
    page._advance_replay()
    assert page._replay_idx == 1
    page._advance_replay()
    assert page._replay_idx == 2
    # At the last frame, advancing stops playback.
    page._advance_replay()
    assert page._replay_idx == 2
    assert page._replay_timer.isActive() is False
