"""Main application window: nav rail + safety bar + pages + event strip."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QHBoxLayout,
    QMainWindow,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from services import ConnectionService
from ui.pages import (
    ConnectPage,
    DiagnosticsPage,
    ModeSafetyPage,
    OverviewPage,
)
from ui.widgets import EventStrip, NavRail, SafetyBar

from hexapod_protocol import telemetry as tlm


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Hexapod Companion")
        self.resize(1180, 760)
        self.service = ConnectionService()

        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Navigation rail.
        self.nav = NavRail()
        self.nav.add_section("Operate")
        self.nav.add_item("connect", "Connect")
        self.nav.add_item("overview", "Overview")
        self.nav.add_item("mode_safety", "Mode & Safety")
        self.nav.add_section("Diagnose")
        self.nav.add_item("diagnostics", "Diagnostics")
        self.nav.finish()
        self.nav.navigated.connect(self._navigate)
        outer.addWidget(self.nav)

        # Right column: safety bar + stacked pages + event strip.
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(0)
        rightw = QWidget()
        rightw.setLayout(right)
        outer.addWidget(rightw, 1)

        self.safety_bar = SafetyBar()
        self.safety_bar.estop_pressed.connect(self.service.emergency_stop)
        right.addWidget(self.safety_bar)

        self.stack = QStackedWidget()
        right.addWidget(self.stack, 1)

        self.event_strip = EventStrip()
        right.addWidget(self.event_strip)

        # Pages.
        self._pages: dict[str, int] = {}
        for key, page in (
            ("connect", ConnectPage(self.service)),
            ("overview", OverviewPage(self.service)),
            ("mode_safety", ModeSafetyPage(self.service)),
            ("diagnostics", DiagnosticsPage(self.service)),
        ):
            self._pages[key] = self.stack.addWidget(page)

        # Wire global chrome to the service.
        self.service.connected.connect(self.safety_bar.set_connection)
        self.service.connected.connect(
            lambda c: self.event_strip.add(
                "connect" if c else "disconnect", "link up" if c else "link down"
            )
        )
        self.service.event.connect(self.event_strip.add)
        self.service.status_received.connect(self._on_status)
        self.service.telemetry.connect(self._on_telemetry)

        self.nav.select("connect")

    def _navigate(self, key: str) -> None:
        idx = self._pages.get(key)
        if idx is not None:
            self.stack.setCurrentIndex(idx)

    def _on_status(self, st) -> None:
        self.safety_bar.mode.set(
            tlm.SAFETY_STATE_NAMES.get(st.state, str(st.state)),
            "ok" if st.state in (2, 4, 5) else "warn",
        )
        self.safety_bar.battery.set(
            f"{st.battery_mv / 1000:.1f} V",
            "ok" if st.battery_mv > 10000 else "warn",
        )
        self.safety_bar.torque.set(
            "OFF" if not st.dxl_power else "ON", "idle" if not st.dxl_power else "warn"
        )

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.CONTROL_STATE):
            self.safety_bar.source.set(
                record.source_name, "active" if record.command_source else "idle"
            )
            self.safety_bar.arming.set(
                "ARMED" if record.motion_gate else "SAFE",
                "warn" if record.motion_gate else "ok",
            )
            if record.fault_reason:
                self.event_strip.add(
                    "fault", tlm.FAULT_REASON_NAMES.get(record.fault_reason, "fault")
                )
        elif stream_id == int(tlm.StreamId.RC_INPUT):
            self.safety_bar.rc.set(
                "FAILSAFE" if record.failsafe else "OK",
                "error" if record.failsafe else "ok",
            )

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self.service.disconnect()
        super().closeEvent(event)
