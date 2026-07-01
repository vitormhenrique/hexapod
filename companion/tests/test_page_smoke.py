"""Hardware-free UI smoke tests for every companion page (nzi.8).

Two guarantees, both without a serial port:

1. Every page in the navigation registers and constructs off-screen.
2. Every page can consume a full recorded telemetry session (built with
   :mod:`replay_fixtures`) without raising.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")

from .replay_fixtures import build_sample_session, replay_into_service


def _page_classes():
    from ui.pages import (
        ConnectPage,
        DiagnosticsPage,
        FootContactPage,
        GaitLabPage,
        LegLabPage,
        ModelViewerPage,
        ModeSafetyPage,
        OverviewPage,
        PassivePosePage,
        SensorDashboardPage,
        ServoConfigPage,
        ServoTuningPage,
    )

    return [
        ConnectPage,
        OverviewPage,
        ModeSafetyPage,
        GaitLabPage,
        FootContactPage,
        PassivePosePage,
        LegLabPage,
        ServoConfigPage,
        ServoTuningPage,
        ModelViewerPage,
        SensorDashboardPage,
        DiagnosticsPage,
    ]


@pytest.mark.parametrize("page_cls", _page_classes(), ids=lambda c: c.__name__)
def test_page_constructs_headless(qtbot, page_cls) -> None:
    from services import ConnectionService

    service = ConnectionService()
    page = page_cls(service)
    qtbot.addWidget(page)
    assert page.isWidgetType()


@pytest.mark.parametrize("page_cls", _page_classes(), ids=lambda c: c.__name__)
def test_page_consumes_replay_session(qtbot, tmp_path, page_cls) -> None:
    from services import ConnectionService

    service = ConnectionService()
    page = page_cls(service)
    qtbot.addWidget(page)

    replay = build_sample_session(tmp_path)
    emitted = replay_into_service(replay, service)

    # The fixture covers 8 streams x 3 frames each.
    assert emitted == 24
    # Page survived the telemetry storm and is still a live widget.
    assert page.isWidgetType()


def test_sample_session_roundtrips(tmp_path) -> None:
    """The fixture's raw frames must all re-decode through the protocol stack."""
    replay = build_sample_session(tmp_path, frames_per_stream=2)
    decoded = list(replay.iter_decoded_frames())
    assert len(decoded) == 16
    assert all(df.stream is not None and df.record is not None for df in decoded)
    # Manifest and event log are present and readable.
    assert replay.meta["robot_name"] == "fixture"
    assert any(e["kind"] == "connect" for e in replay.iter_events())
