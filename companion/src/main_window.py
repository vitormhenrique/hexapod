"""Main application window: nav rail + safety bar + pages + event strip."""

from __future__ import annotations

from hexapod_protocol import telemetry as tlm
from PySide6.QtWidgets import (
    QHBoxLayout,
    QMainWindow,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from services import ConnectionService
from ui.pages import (
    ConnectPage,
    DiagnosticsPage,
    DxlCommandsPage,
    FootContactPage,
    GaitLabPage,
    LegLabPage,
    ModeSafetyPage,
    OpenLogConfigFilePage,
    OverviewPage,
    PassivePosePage,
    PlotWorkbenchPage,
    PoseViewerPage,
    RcTroubleshootingPage,
    SensorDashboardPage,
    ServoCalibrationPage,
    ServoConfigPage,
    ServoTuningPage,
)
from ui.widgets import EventStrip, NavRail, SafetyBar

WORKSPACES = (
    (
        "dashboard",
        "Dashboard",
        (
            ("connect", "Connection", ConnectPage),
            ("overview", "Overview", OverviewPage),
            ("mode_safety", "Safety", ModeSafetyPage),
        ),
    ),
    (
        "motion",
        "Operate",
        (
            ("gait_lab", "Gait", GaitLabPage),
            ("foot_contact", "Contact", FootContactPage),
            ("passive_pose", "Passive Pose", PassivePosePage),
        ),
    ),
    (
        "servos",
        "Configure",
        (
            ("leg_lab", "Foot IK", LegLabPage),
            ("joint_matrix", "DXL Commands", DxlCommandsPage),
            ("servo_calibration", "Fixture Calibration", ServoCalibrationPage),
            ("servo_config", "Robot Config", ServoConfigPage),
            ("servo_tuning", "DXL Settings", ServoTuningPage),
        ),
    ),
    (
        "analyze",
        "Analyze",
        (
            ("plots", "Trends", PlotWorkbenchPage),
            ("pose", "Pose", PoseViewerPage),
        ),
    ),
    (
        "diagnose",
        "Diagnose",
        (
            ("sensors", "Sensors", SensorDashboardPage),
            ("rc_troubleshooting", "Radio", RcTroubleshootingPage),
            ("diagnostics", "Protocol", DiagnosticsPage),
            ("openlog_config", "Card Recovery", OpenLogConfigFilePage),
        ),
    ),
)

HARDWARE_ONLY_PAGES = frozenset(
    {
        "foot_contact",
        "passive_pose",
        "leg_lab",
        "joint_matrix",
        "servo_calibration",
        "servo_config",
        "servo_tuning",
        "sensors",
        "rc_troubleshooting",
    }
)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("HexNav")
        self.resize(1180, 760)
        self.service = ConnectionService()

        central = QWidget()
        self.setCentralWidget(central)
        outer = QHBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Navigation exposes operator workflows; related tools live in tabs.
        self.nav = NavRail()
        self.nav.add_section("Workflows")
        for workspace_key, label, _pages in WORKSPACES:
            self.nav.add_item(workspace_key, label)
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

        self._workspaces: dict[str, int] = {}
        self._workspace_tabs: dict[str, QTabWidget] = {}
        self._page_widgets: dict[str, QWidget] = {}
        self._page_locations: dict[str, tuple[str, int]] = {}
        self._active_page: QWidget | None = None
        for workspace_key, _label, page_specs in WORKSPACES:
            tabs = QTabWidget()
            tabs.setObjectName("WorkspaceTabs")
            tabs.setDocumentMode(True)
            tabs.setUsesScrollButtons(True)
            for page_key, tab_label, page_class in page_specs:
                page = page_class(self.service)
                page.set_page_active(False)
                tab_index = tabs.addTab(page, tab_label)
                self._page_widgets[page_key] = page
                self._page_locations[page_key] = (workspace_key, tab_index)
            self._workspace_tabs[workspace_key] = tabs
            self._workspaces[workspace_key] = self.stack.addWidget(tabs)
            tabs.currentChanged.connect(
                lambda index, key=workspace_key: self._on_tab_changed(key, index)
            )

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
        self.service.simulation_mode_changed.connect(self._on_simulation_mode)
        self.service.maint_lock_changed.connect(
            lambda held, _tok: self.safety_bar.lock.set(
                "HELD" if held else "none", "warn" if held else "idle"
            )
        )

        self._last_fault_reason = 0

        self.navigate_to("connect")

    def connect_endpoint(self, endpoint: str) -> None:
        """Select the Connect page and begin an application-startup connection."""
        self.navigate_to("connect")
        page = self._page_widgets["connect"]
        if hasattr(page, "port_combo"):
            page.port_combo.setCurrentText(endpoint)
        self.service.connect_to(endpoint)

    def _navigate(self, key: str) -> None:
        idx = self._workspaces.get(key)
        if idx is not None:
            self.stack.setCurrentIndex(idx)
            self._activate_page(self._workspace_tabs[key].currentWidget())

    def navigate_to(self, page_key: str) -> None:
        """Open a capability in its owning workflow and select its tab."""
        location = self._page_locations.get(page_key)
        if location is None:
            return
        workspace_key, tab_index = location
        tabs = self._workspace_tabs[workspace_key]
        if not tabs.isTabEnabled(tab_index):
            return
        self.nav.select(workspace_key)
        tabs.setCurrentIndex(tab_index)
        self._activate_page(tabs.currentWidget())

    def _on_tab_changed(self, workspace_key: str, tab_index: int) -> None:
        if self.stack.currentIndex() != self._workspaces[workspace_key]:
            return
        self._activate_page(self._workspace_tabs[workspace_key].widget(tab_index))

    def _activate_page(self, page: QWidget | None) -> None:
        if page is self._active_page:
            return
        if self._active_page is not None:
            self._active_page.set_page_active(False)
        self._active_page = page
        if page is not None:
            page.set_page_active(True)

    def _set_page_enabled(self, page_key: str, enabled: bool, tooltip: str) -> None:
        workspace_key, tab_index = self._page_locations[page_key]
        tabs = self._workspace_tabs[workspace_key]
        tabs.setTabEnabled(tab_index, enabled)
        tabs.setTabToolTip(tab_index, "" if enabled else tooltip)

    def is_page_enabled(self, page_key: str) -> bool:
        """Return whether a capability can currently be opened."""
        workspace_key, tab_index = self._page_locations[page_key]
        return self._workspace_tabs[workspace_key].isTabEnabled(tab_index)

    def _on_status(self, st) -> None:
        self.safety_bar.mode.set(
            tlm.SAFETY_STATE_NAMES.get(st.state, str(st.state)),
            "ok" if st.state in tlm.NOMINAL_SAFETY_STATES else "warn",
        )
        self.safety_bar.battery.set(
            f"{st.battery_mv / 1000:.1f} V",
            "ok" if st.battery_mv > 10000 else "warn",
        )
        self.safety_bar.dxl_power.set(
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
            # Emit a fault event only on transition (avoids per-frame spam); the
            # event flows through service.event so both the event strip and the
            # plot markers pick it up.
            if record.fault_reason != self._last_fault_reason:
                self._last_fault_reason = record.fault_reason
                if record.fault_reason:
                    self.service.event.emit(
                        "fault",
                        tlm.FAULT_REASON_NAMES.get(record.fault_reason, "fault"),
                    )
        elif stream_id == int(tlm.StreamId.RC_INPUT):
            self.safety_bar.rc.set(
                "FAILSAFE" if record.failsafe else "OK",
                "error" if record.failsafe else "ok",
            )

    def _on_simulation_mode(self, enabled: bool) -> None:
        tooltip = "Unavailable in the ROS simulated firmware."
        for key in HARDWARE_ONLY_PAGES:
            self._set_page_enabled(key, not enabled, tooltip)
        if enabled:
            self.event_strip.add("simulation", "ROS simulated firmware connected")
            self.navigate_to("gait_lab")

    def closeEvent(self, event) -> None:
        self.service.disconnect()
        super().closeEvent(event)
