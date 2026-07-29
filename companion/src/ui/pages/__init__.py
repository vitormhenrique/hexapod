"""Companion pages. Each page is a QWidget wired to the ConnectionService."""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm
from hexapod_protocol.framing import MsgType

from services import ConnectionService
from theme import DRACULA
from ui.widgets import FeatureToggleCard, StatCard, StatusBadge, TelemetryBanner


class BasePage(QWidget):
    """Common page chrome: scrollable, full-width, padded title + content area.

    Content spans the full page width. Set ``fill = True`` on a subclass to let
    its content area expand and use all vertical space (for pages whose main
    widget is a plot/table/feed); otherwise content stays top-aligned.
    """

    title = "Page"
    subtitle = ""
    fill = False

    def __init__(self, service: ConnectionService, parent=None) -> None:
        super().__init__(parent)
        self.service = service

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        outer.addWidget(scroll)

        canvas = QWidget()
        scroll.setWidget(canvas)
        col = QVBoxLayout(canvas)
        col.setContentsMargins(32, 26, 32, 26)
        col.setSpacing(18)

        header = QVBoxLayout()
        header.setSpacing(3)
        t = QLabel(self.title)
        t.setObjectName("PageTitle")
        header.addWidget(t)
        if self.subtitle:
            s = QLabel(self.subtitle)
            s.setObjectName("PageSubtitle")
            header.addWidget(s)
        col.addLayout(header)

        self.content = QVBoxLayout()
        self.content.setSpacing(16)
        col.addLayout(self.content, 1 if self.fill else 0)
        if not self.fill:
            col.addStretch(1)
        self.build()

    def build(self) -> None:  # override
        ...

    def add_telemetry_banner(
        self, streams, hint: str = "", require_all: bool = True
    ) -> TelemetryBanner:
        """Insert a stale/missing-telemetry warning strip above the content.

        ``streams`` is ``[(StreamId, name), ...]``. The Re-subscribe button
        replays this page's connect-time subscriptions.
        """
        banner = TelemetryBanner(
            self.service,
            streams,
            hint=hint,
            require_all=require_all,
            resubscribe=lambda: self._on_connected(True),
        )
        self.content.insertWidget(0, banner)
        return banner

    def _on_connected(self, connected: bool) -> None:  # override where needed
        ...


class ConnectPage(BasePage):
    title = "Connect && Setup"
    subtitle = "Discover the USB port, handshake, and verify firmware capabilities."

    def build(self) -> None:
        box = QGroupBox("Serial connection")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(14)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(320)
        self.port_combo.setEditable(True)
        self.port_combo.lineEdit().setPlaceholderText(
            "Serial port or tcp://jetson:5555?token=..."
        )
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(self.port_combo, 1)
        row.addWidget(self.refresh_btn)
        form.addRow("Port", self._wrap(row))

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setProperty("accent", True)
        self.connect_btn.clicked.connect(self._toggle)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.service.disconnect)
        self.disconnect_btn.setEnabled(False)
        btnrow = QHBoxLayout()
        btnrow.setSpacing(10)
        btnrow.addWidget(self.connect_btn)
        btnrow.addWidget(self.disconnect_btn)
        btnrow.addStretch(1)
        form.addRow("", self._wrap(btnrow))

        info = QGroupBox("Firmware")
        ilay = QFormLayout(info)
        ilay.setHorizontalSpacing(18)
        ilay.setVerticalSpacing(12)
        ilay.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.device_lbl = QLabel("--")
        self.fw_lbl = QLabel("--")
        self.proto_lbl = QLabel("--")
        self.caps_lbl = QLabel("--")
        for lbl in (self.device_lbl, self.fw_lbl, self.proto_lbl, self.caps_lbl):
            lbl.setObjectName("MonoLabel")
        ilay.addRow("Device", self.device_lbl)
        ilay.addRow("Firmware", self.fw_lbl)
        ilay.addRow("Protocol", self.proto_lbl)
        ilay.addRow("Feature bits", self.caps_lbl)

        # Place the two cards side by side so they use the horizontal space.
        row = QHBoxLayout()
        row.setSpacing(16)
        row.addWidget(box, 1)
        row.addWidget(info, 1)
        self.content.addLayout(row)

        self.service.connected.connect(self._on_connected)
        self.service.hello_received.connect(self._on_hello)
        self.service.capabilities_received.connect(self._on_caps)
        self.service.connecting.connect(self._on_connecting)
        self.refresh_ports()

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = self.service.available_ports()
        for p in ports:
            self.port_combo.addItem(f"{p.device}  —  {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found", None)
            return
        # Prefer the OpenRB-150 controller and, on macOS, the outgoing /dev/cu.*
        # endpoint over /dev/tty.* for application-owned connections.
        best_index = 0
        best_score = -1
        for i, p in enumerate(ports):
            haystack = f"{p.device} {p.description} {p.hwid}".lower()
            score = 0
            if "openrb" in haystack:
                score += 100
            if str(p.device).startswith("/dev/cu."):
                score += 20
            if score > best_score:
                best_index = i
                best_score = score
        self.port_combo.setCurrentIndex(best_index)

    def _toggle(self) -> None:
        endpoint = self.port_combo.currentData()
        if not endpoint:
            endpoint = self.port_combo.currentText().strip()
        if endpoint and endpoint != "No ports found":
            self.service.connect_to(endpoint)

    def _on_connecting(self, busy: bool) -> None:
        if busy:
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Connecting...")
            self.disconnect_btn.setEnabled(False)
            self.refresh_btn.setEnabled(False)
            self.port_combo.setEnabled(False)

    def _on_connected(self, connected: bool) -> None:
        self.connect_btn.setText("Connect")
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.refresh_btn.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        if not connected:
            for lbl in (self.device_lbl, self.fw_lbl, self.proto_lbl, self.caps_lbl):
                lbl.setText("--")

    def _on_hello(self, hello) -> None:
        self.device_lbl.setText(hello.device_name)
        self.fw_lbl.setText(f"{hello.fw_major}.{hello.fw_minor}.{hello.fw_patch}")
        self.proto_lbl.setText(f"{hello.proto_major}.{hello.proto_minor}")

    def _on_caps(self, caps) -> None:
        self.caps_lbl.setText(f"0x{caps.feature_bits:08X}")


class OverviewPage(BasePage):
    title = "Overview"
    subtitle = "Live robot state, battery, safety state, and command source."

    def build(self) -> None:
        grid_box = QGroupBox("Robot state")
        grid = QGridLayout(grid_box)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        grid.setContentsMargins(6, 8, 6, 6)
        self.badges = {
            "state": StatCard("Safety state"),
            "fault": StatCard("Fault"),
            "source": StatCard("Command source"),
            "gate": StatCard("Motion gate"),
            "battery": StatCard("Battery"),
            "uptime": StatCard("Uptime"),
            "dxl": StatCard("DXL bus"),
            "i2c": StatCard("I2C sensors"),
        }
        for i, b in enumerate(self.badges.values()):
            grid.addWidget(b, i // 4, i % 4)
        for c in range(4):
            grid.setColumnStretch(c, 1)
        self.content.addWidget(grid_box)

        self.hint = QLabel(
            "Subscribe on the Mode && Safety page or connect to see live data."
        )
        self.hint.setStyleSheet(f"color: {DRACULA.comment};")
        self.content.addWidget(self.hint)

        self.banner = self.add_telemetry_banner(
            [
                (tlm.StreamId.CONTROL_STATE, "control_state"),
                (tlm.StreamId.HEALTH, "health"),
                (tlm.StreamId.SERVO_STATUS, "servo_status"),
            ]
        )

        self.service.connected.connect(self._on_connected)
        self.service.status_received.connect(self._on_status)
        self.service.telemetry.connect(self._on_telemetry)

    def _on_connected(self, connected: bool) -> None:
        if connected:
            # Subscribe to the streams that drive the overview badges so the
            # page is self-sufficient (state/source plus DXL and I2C health).
            self.service.subscribe(int(tlm.StreamId.CONTROL_STATE), 10)
            self.service.subscribe(int(tlm.StreamId.HEALTH), 2)
            self.service.subscribe(int(tlm.StreamId.SERVO_STATUS), 5)
            self.service.subscribe(int(tlm.StreamId.I2C_SENSORS_RAW), 5)
        else:
            for b in self.badges.values():
                b.set("--", "idle")

    def _on_status(self, st) -> None:
        self.badges["state"].set(
            tlm.SAFETY_STATE_NAMES.get(st.state, str(st.state)),
            "ok" if st.state in tlm.NOMINAL_SAFETY_STATES else "warn",
        )
        self.badges["battery"].set(
            f"{st.battery_mv} mV", "ok" if st.battery_mv > 10000 else "warn"
        )
        self.badges["uptime"].set(f"{st.uptime_ms // 1000} s", "info")
        # DXL bus health: power state + watchdog misses from the status frame.
        if not st.dxl_power:
            self.badges["dxl"].set("power off", "idle")
        elif st.watchdog_missed:
            self.badges["dxl"].set(f"wd miss {st.watchdog_missed}", "warn")
        else:
            self.badges["dxl"].set("power on", "ok")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.CONTROL_STATE):
            self.badges["source"].set(record.source_name, "active")
            self.badges["gate"].set(
                "OPEN" if record.motion_gate else "CLOSED",
                "ok" if record.motion_gate else "idle",
            )
            self.badges["state"].set(
                tlm.SAFETY_STATE_NAMES.get(record.state, str(record.state)),
                "ok" if record.state in tlm.NOMINAL_SAFETY_STATES else "warn",
            )
            self.badges["fault"].set(
                tlm.FAULT_REASON_NAMES.get(
                    record.fault_reason, str(record.fault_reason)
                ),
                "error" if record.fault_reason else "ok",
            )
        elif stream_id == int(tlm.StreamId.HEALTH):
            self.badges["battery"].set(
                f"{record.battery_mv} mV", "ok" if record.battery_mv > 10000 else "warn"
            )
        elif stream_id == int(tlm.StreamId.SERVO_STATUS):
            # Live servo frames confirm the DYNAMIXEL bus is responding.
            self.badges["dxl"].set(f"{len(record.servos)} servos", "ok")
        elif stream_id == int(tlm.StreamId.I2C_SENSORS_RAW):
            self.badges["i2c"].set(f"{len(record.feet)} sensors", "ok")


class ModeSafetyPage(BasePage):
    title = "Mode && Safety Center"
    subtitle = "Arming, fault recovery, maintenance/passive locks, and feature flags."

    STREAMS = [
        ("health", 5),
        ("control_state", 20),
        ("servo_status", 20),
        ("contact_state", 20),
        ("i2c_sensors_raw", 10),
        ("rc_input", 20),
        ("api_stats", 2),
    ]

    FEATURES = [
        (api.FEATURE_FOOT_CONTACT, "Foot Contact"),
        (api.FEATURE_TERRAIN_LEVELING, "Terrain Leveling"),
        (api.FEATURE_SENSOR_POLLING, "Sensor Polling"),
        (api.FEATURE_JETSON_CONTROL, "Jetson Control"),
    ]

    REASON_NAMES = {
        api.FEATURE_REASON_NONE: "NONE",
        api.FEATURE_REASON_HARDWARE_MISSING: "hardware missing",
        api.FEATURE_REASON_NOT_CALIBRATED: "not calibrated",
        api.FEATURE_REASON_UNSAFE_STATE: "unsafe state",
        api.FEATURE_REASON_STALE_DATA: "stale data",
        api.FEATURE_REASON_DEPENDENCY_OFF: "dependency off",
        api.FEATURE_REASON_NOT_IMPLEMENTED: "not implemented",
    }

    def build(self) -> None:
        self._connected = False
        self._state = -1
        self._lock_held = False
        self._simulation_mode = self.service.simulation_mode

        self.content.addWidget(self._safety_controls())
        self.content.addWidget(self._lock_controls())
        self.content.addWidget(self._bench_controls())
        self.content.addWidget(self._feature_flags())
        self.content.addWidget(self._subscriptions())

        safety = QGroupBox("Emergency stop")
        slay = QHBoxLayout(safety)
        estop = QPushButton("\u23fb  EMERGENCY STOP")
        estop.setObjectName("EmergencyStop")
        estop.clicked.connect(self.service.emergency_stop)
        slay.addWidget(estop)
        slay.addStretch(1)
        self.content.addWidget(safety)

        self.service.connected.connect(self._on_connected)
        self.service.state_changed.connect(self._on_state_changed)
        self.service.feature_list.connect(self._on_feature_list)
        self.service.feature_result.connect(lambda _r: self.service.refresh_features())
        self.service.maint_result.connect(self._on_maint_result)
        self.service.maint_lock_changed.connect(self._on_lock_changed)
        self.service.control_result.connect(self._on_control_result)
        self.service.dxl_result.connect(self._on_dxl_result)
        self.service.simulation_mode_changed.connect(self._on_simulation_mode)

        self._apply_gates()

    # --- groups -----------------------------------------------------------

    def _safety_controls(self) -> QGroupBox:
        box = QGroupBox("Safety controls")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        arm = QPushButton("Arm")
        arm.setToolTip("Release the host disarm latch (RC arm switch still required).")
        arm.clicked.connect(lambda: self.service.set_arming(True))
        self.arm_btn = arm

        disarm = QPushButton("Disarm")
        disarm.clicked.connect(
            lambda: self._confirm(
                "Disarm robot",
                "Latch a host force-disarm? This removes motion authority.",
                lambda: self.service.set_arming(False),
            )
        )
        self.disarm_btn = disarm

        clear = QPushButton("Clear Fault")
        clear.clicked.connect(
            lambda: self._confirm(
                "Clear fault",
                "Release the host E-stop latch and request a fault clear?",
                self.service.clear_fault,
            )
        )
        self.clear_btn = clear

        set_disarmed = QPushButton("Set mode: Disarmed")
        set_disarmed.clicked.connect(lambda: self.service.set_mode(2))
        self.set_disarmed_btn = set_disarmed

        force_estop = QPushButton("Set mode: E-stop")
        force_estop.clicked.connect(
            lambda: self._confirm(
                "Force E-stop mode",
                "Drive the safety machine into ESTOP?",
                lambda: self.service.set_mode(12),
            )
        )
        self.force_estop_btn = force_estop

        for i, btn in enumerate((arm, disarm, clear, set_disarmed, force_estop)):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 3, i % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)

        self.action_lbl = QLabel("No command sent yet.")
        self.action_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        self.action_lbl.setWordWrap(True)
        grid.addWidget(self.action_lbl, 2, 0, 1, 3)

        hint = QLabel(
            "Arming for walking requires the RC transmitter arm switch; the "
            "host Arm button only releases a previous host disarm. For bench "
            "work without RC (moving servos from this app), use Enter "
            "Maintenance below."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment}; font-size: 12px;")
        grid.addWidget(hint, 3, 0, 1, 3)
        return box

    def _lock_controls(self) -> QGroupBox:
        box = QGroupBox("Maintenance && passive pose")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        enter_m = QPushButton("Enter Maintenance")
        enter_m.clicked.connect(self.service.enter_maintenance)
        self.enter_maint_btn = enter_m
        exit_m = QPushButton("Exit Maintenance")
        exit_m.clicked.connect(self.service.exit_maintenance)
        self.exit_maint_btn = exit_m

        enter_p = QPushButton("Enter Passive Pose")
        enter_p.clicked.connect(
            lambda: self._confirm(
                "Enter passive pose",
                "Disable all servo torque and stream present positions?",
                self.service.passive_enter,
            )
        )
        self.enter_passive_btn = enter_p
        exit_p = QPushButton("Exit Passive Pose")
        exit_p.clicked.connect(self.service.passive_exit)
        self.exit_passive_btn = exit_p

        for i, btn in enumerate((enter_m, exit_m, enter_p, exit_p)):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 2, i % 2)
        for c in range(2):
            grid.setColumnStretch(c, 1)

        self.lock_lbl = QLabel("Maintenance lock: none")
        self.lock_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        grid.addWidget(self.lock_lbl, 2, 0, 1, 2)
        return box

    def _bench_controls(self) -> QGroupBox:
        box = QGroupBox("Bench servo control (maintenance mode)")
        outer = QVBoxLayout(box)

        hint = QLabel(
            "Bench flow: 1) Enter Maintenance above  2) DXL Power On  "
            "3) Scan Servos  4) Torque On  5) Center All. Targets are clamped "
            "by the configured servo travel limits in firmware."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment}; font-size: 12px;")
        outer.addWidget(hint)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        power_on = QPushButton("DXL Power On")
        power_on.clicked.connect(lambda: self.service.dxl_power(True))
        power_off = QPushButton("DXL Power Off")
        power_off.clicked.connect(lambda: self.service.dxl_power(False))
        scan = QPushButton("Scan Servos")
        scan.clicked.connect(self._on_scan)
        torque_on = QPushButton("Torque On")
        torque_on.clicked.connect(
            lambda: self._confirm(
                "Enable torque",
                "Enable torque on all scanned servos? They will hold and then "
                "follow commanded targets.",
                lambda: self.service.dxl_torque(True),
            )
        )
        torque_off = QPushButton("Torque Off")
        torque_off.clicked.connect(lambda: self.service.dxl_torque(False))
        center = QPushButton("Center All (180\u00b0)")
        center.setProperty("accent", True)
        center.clicked.connect(
            lambda: self._confirm(
                "Center all servos",
                "Move ALL 18 joints to center (tick 2048 / 180\u00b0)? The robot "
                "must be on a stand \u2014 servos WILL move.",
                self.service.center_all_joints,
            )
        )

        for i, btn in enumerate(
            (power_on, power_off, scan, torque_on, torque_off, center)
        ):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 3, i % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        outer.addLayout(grid)

        self.bench_lbl = QLabel("Requires the maintenance lock.")
        self.bench_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        self.bench_lbl.setWordWrap(True)
        outer.addWidget(self.bench_lbl)

        self._bench_buttons = (power_on, power_off, scan, torque_on, torque_off, center)
        for btn in self._bench_buttons:
            btn.setEnabled(False)
        return box

    def _feature_flags(self) -> QGroupBox:
        box = QGroupBox("Feature flags")
        outer = QVBoxLayout(box)
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        self._feature_cards: dict[int, FeatureToggleCard] = {}
        for i, (fid, name) in enumerate(self.FEATURES):
            card = FeatureToggleCard(fid, name)
            card.toggled.connect(self._on_feature_toggle)
            grid.addWidget(card, i // 2, i % 2)
            self._feature_cards[fid] = card
        for c in range(2):
            grid.setColumnStretch(c, 1)
        outer.addLayout(grid)

        refresh = QPushButton("Refresh feature state")
        refresh.setCursor(Qt.PointingHandCursor)
        refresh.clicked.connect(self.service.refresh_features)
        row = QHBoxLayout()
        row.addWidget(refresh)
        row.addStretch(1)
        outer.addLayout(row)
        return box

    def _subscriptions(self) -> QGroupBox:
        box = QGroupBox("Telemetry subscriptions")
        lay = QGridLayout(box)
        lay.setHorizontalSpacing(12)
        lay.setVerticalSpacing(12)
        self._buttons = {}
        for i, (name, rate) in enumerate(self.STREAMS):
            btn = QPushButton(f"Subscribe {name} @ {rate} Hz")
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda checked, n=name, r=rate: self._toggle(n, r, checked)
            )
            lay.addWidget(btn, i // 2, i % 2)
            self._buttons[name] = btn
        for c in range(2):
            lay.setColumnStretch(c, 1)
        return box

    # --- actions / reactions ---------------------------------------------

    def _confirm(self, title: str, text: str, action) -> None:
        reply = QMessageBox.question(
            self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            action()

    def _on_feature_toggle(self, feature: int, enable: bool) -> None:
        if enable and feature == api.FEATURE_JETSON_CONTROL:
            self._confirm(
                "Enable Jetson control",
                "Grant the Jetson high-level motion authority?",
                lambda: self.service.set_feature(feature, True),
            )
            # Revert the optimistic check until the firmware confirms.
            self.service.refresh_features()
        else:
            self.service.set_feature(feature, enable)

    def _on_feature_list(self, fl) -> None:
        for f in fl.features:
            card = self._feature_cards.get(f.feature)
            if card is not None:
                card.set_state(
                    f.available,
                    f.enabled,
                    self.REASON_NAMES.get(f.reason, str(f.reason)),
                )

    def _on_simulation_mode(self, enabled: bool) -> None:
        self._simulation_mode = enabled
        if enabled:
            self.action_lbl.setText(
                "ROS simulation: maintenance, DXL, and passive-pose controls are unavailable."
            )
        self._apply_gates()

    def _on_maint_result(self, res) -> None:
        if res.token:
            self.lock_lbl.setText(f"Maintenance lock: held (token {res.token})")
        elif res.ok:
            self.lock_lbl.setText("Maintenance lock: none")
        else:
            self.lock_lbl.setText(f"Maintenance lock: rejected (result {res.result})")

    def _on_lock_changed(self, held: bool, token: int) -> None:
        self._lock_held = held
        if held:
            self.lock_lbl.setText(
                f"Maintenance lock: held (token {token}, heartbeating)"
            )
        else:
            self.lock_lbl.setText("Maintenance lock: none")
        self.bench_lbl.setText("Ready." if held else "Requires the maintenance lock.")
        self._apply_gates()

    def _on_scan(self) -> None:
        self.bench_lbl.setText("Scanning IDs 1-30 (takes a few seconds)\u2026")
        self.service.dxl_scan()

    def _on_dxl_result(self, kind: str, res) -> None:
        if kind not in ("power", "torque", "scan"):
            return
        if res is None:
            self.bench_lbl.setText(f"DXL {kind}: no result (rejected or timed out)")
            return
        if not res.done:
            self.bench_lbl.setText(f"DXL {kind}: not finished (slot {res.slot})")
            return
        if kind == "scan":
            servos = res.servos()
            ids = ", ".join(str(s.id) for s in servos)
            self.bench_lbl.setText(
                f"Scan: {len(servos)} servo(s) found"
                + (f" \u2014 ids {ids}" if servos else " (power on first?)")
            )
        elif kind == "power":
            pr = res.power()
            if pr is None:
                self.bench_lbl.setText("DXL power: no decode")
            else:
                self.bench_lbl.setText(f"DXL power: {'ON' if pr.power_on else 'OFF'}")
        elif len(res.data) >= 2:
            self.bench_lbl.setText(
                f"Torque {'ON' if res.data[0] else 'OFF'} \u2014 acked by "
                f"{res.data[1]} servo(s)"
            )
        else:
            self.bench_lbl.setText(f"Torque command acknowledged (code {res.code}).")

    def _on_control_result(self, kind: str, res) -> None:
        state = tlm.SAFETY_STATE_NAMES.get(res.state, str(res.state))
        verdict = "ok" if res.ok else f"rejected ({res.result})"
        text = f"{kind}: {verdict} — state {state}"
        if kind == "arm" and res.ok and res.state == tlm.SafetyState.DISARMED:
            text += (
                " — host arm only releases the disarm latch; walking requires "
                "the RC arm switch. For bench servo control use Enter "
                "Maintenance."
            )
        self.action_lbl.setText(text)

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.refresh_features()
        else:
            self._state = -1
            self._lock_held = False
            self.lock_lbl.setText("Maintenance lock: none")
            self.action_lbl.setText("No command sent yet.")
        self._apply_gates()

    def _on_state_changed(self, state: int) -> None:
        self._state = state
        self._apply_gates()

    def _apply_gates(self) -> None:
        """Enable only the controls that make sense right now.

        Gating mirrors the firmware safety machine so the UI does not offer
        actions the firmware would reject. E-stop is never gated. State -1
        (unknown before the first status poll) keeps entry actions enabled; the
        firmware still validates every request.
        """
        con = self._connected
        st = self._state
        held = self._lock_held

        def gate(btn, enabled: bool, why: str) -> None:
            btn.setEnabled(enabled)
            btn.setToolTip("" if enabled else why)

        for btn in (self.disarm_btn, self.set_disarmed_btn, self.force_estop_btn):
            gate(btn, con, "Connect to the robot first.")
        gate(
            self.arm_btn,
            con and st != tlm.SafetyState.PASSIVE_POSE_STREAM,
            "Connect first; exit passive pose before arming.",
        )
        gate(
            self.clear_btn,
            con
            and st
            in (
                -1,
                tlm.SafetyState.FAULT_SOFT,
                tlm.SafetyState.FAULT_HARD,
                tlm.SafetyState.ESTOP,
            ),
            "No latched fault/E-stop to clear.",
        )
        gate(
            self.enter_maint_btn,
            not self._simulation_mode
            and con
            and not held
            and st in (-1, tlm.SafetyState.DISARMED),
            "Unavailable in the ROS simulated firmware.",
        )
        gate(
            self.exit_maint_btn,
            not self._simulation_mode and con and held,
            "Unavailable in the ROS simulated firmware.",
        )
        gate(
            self.enter_passive_btn,
            not self._simulation_mode
            and con
            and st
            in (
                -1,
                tlm.SafetyState.DISARMED,
                tlm.SafetyState.MAC_MAINTENANCE,
                tlm.SafetyState.PASSIVE_POSE_STREAM,
            ),
            "Unavailable in the ROS simulated firmware.",
        )
        gate(
            self.exit_passive_btn,
            not self._simulation_mode
            and con
            and st == tlm.SafetyState.PASSIVE_POSE_STREAM,
            "Unavailable in the ROS simulated firmware.",
        )
        for btn in self._bench_buttons:
            gate(
                btn,
                not self._simulation_mode and con and held,
                "Unavailable in the ROS simulated firmware.",
            )

    def _toggle(self, name: str, rate: int, checked: bool) -> None:
        sid = int(tlm.stream_id_from_name(name))
        if checked:
            self.service.subscribe(sid, rate)
        else:
            self.service.unsubscribe(sid)


class GaitLabPage(BasePage):
    title = "Gait Lab"
    subtitle = "Bench-test gait families and body modes under maintenance authority."

    # (label, GAIT_* id). Stand/Sit are postures; the rest are walking gaits.
    GAITS = [
        ("Stand", api.GAIT_STAND),
        ("Sit", api.GAIT_SIT),
        ("Tripod", api.GAIT_TRIPOD),
        ("Ripple", api.GAIT_RIPPLE),
        ("Wave", api.GAIT_WAVE),
        ("Crawl", api.GAIT_CRAWL),
    ]

    def build(self) -> None:
        self._connected = self.service.is_connected
        self._session_active = False
        self._session_busy = False
        self._simulation_mode = self.service.simulation_mode
        self._session_controls = []

        self.session_box = self._session_panel()
        self.content.addWidget(self.session_box)
        self.gait_box = self._gait_select()
        self.params_box = self._gait_params()
        self.mode_box = self._mode_select()
        self.walk_box = self._body_twist()
        self.translate_box = self._body_translation()
        self.rotate_box = self._body_rotation()
        for box in (
            self.gait_box,
            self.params_box,
            self.mode_box,
            self.walk_box,
            self.translate_box,
            self.rotate_box,
        ):
            self.content.addWidget(box)
            self._session_controls.append(box)
        self.translate_box.hide()
        self.rotate_box.hide()

        # Always-available stop + emergency stop row.
        safety = QGroupBox("Stop")
        slay = QHBoxLayout(safety)
        stop = QPushButton("Hold Stand")
        stop.clicked.connect(self._hold_stand)
        estop = QPushButton("\u23fb  EMERGENCY STOP")
        estop.setObjectName("EmergencyStop")
        estop.clicked.connect(self.service.emergency_stop)
        slay.addWidget(stop)
        slay.addWidget(estop)
        slay.addStretch(1)
        self.content.addWidget(safety)

        self.banner = self.add_telemetry_banner(
            [(tlm.StreamId.CONTROL_STATE, "control_state")]
        )

        self.service.connected.connect(self._on_connected)
        self.service.gait_test_changed.connect(self._on_session_changed)
        self.service.gait_test_busy_changed.connect(self._on_session_busy)
        self.service.motion_result.connect(self._on_motion_result)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.simulation_mode_changed.connect(self._on_simulation_mode)
        self._on_simulation_mode(self._simulation_mode)
        self._apply_session_gate()

    # --- groups -----------------------------------------------------------

    def _session_panel(self) -> QGroupBox:
        box = QGroupBox("Maintenance gait session")
        lay = QGridLayout(box)
        lay.setHorizontalSpacing(12)
        lay.setVerticalSpacing(10)
        self.session_badge = StatusBadge("bench session")
        self.session_badge.set("stopped", "idle")
        self.gate_badge = StatusBadge("motion gate")
        self.gate_badge.set("unknown", "idle")
        self.start_btn = QPushButton("Start gait test")
        self.start_btn.setProperty("accent", True)
        self.start_btn.clicked.connect(self._start_session)
        self.stop_session_btn = QPushButton("Stop && torque off")
        self.stop_session_btn.clicked.connect(self.service.stop_gait_test)
        self.authority_lbl = QLabel("Disconnected")
        self.authority_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(self.session_badge, 0, 0)
        lay.addWidget(self.gate_badge, 0, 1)
        lay.addWidget(self.start_btn, 1, 0)
        lay.addWidget(self.stop_session_btn, 1, 1)
        lay.addWidget(self.authority_lbl, 2, 0, 1, 2)
        lay.setColumnStretch(0, 1)
        lay.setColumnStretch(1, 1)
        return box

    def _gait_select(self) -> QGroupBox:
        box = QGroupBox("Gait selection")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        self._gait_buttons = {}
        self._gait_group = QButtonGroup(self)
        self._gait_group.setExclusive(True)
        for i, (label, gait) in enumerate(self.GAITS):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(
                lambda _=False, g=gait, name=label: self._select_gait(g, name)
            )
            self._gait_group.addButton(btn)
            grid.addWidget(btn, i // 3, i % 3)
            self._gait_buttons[gait] = btn
        self._gait_buttons[api.GAIT_STAND].setChecked(True)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        return box

    def _select_gait(self, gait: int, label: str) -> None:
        """Command a gait and annotate the timeline with a gait-change event."""
        self.service.set_gait(gait)
        self.service.event.emit("gait", label)

    def _gait_params(self) -> QGroupBox:
        box = QGroupBox("Gait parameters")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        # (attr, label, min, max, default) — defaults mirror firmware gait config.
        self._param_spins = {}
        for attr, label, lo, hi, default in (
            ("body_height", "Body height (mm)", 0, 120, 40),
            ("stride_len", "Stride length (mm)", 0, 80, 60),
            ("step_height", "Step height (mm)", 0, 50, 30),
            ("duty", "Duty cycle (0-255)", 0, 255, 128),
            ("speed", "Speed (0-255)", 0, 255, 128),
        ):
            spin = QSpinBox()
            spin.setRange(lo, hi)
            spin.setValue(default)
            self._param_spins[attr] = spin
            form.addRow(label, spin)
        send = QPushButton("Apply gait parameters")
        send.clicked.connect(self._send_gait_params)
        form.addRow("", send)
        presets = QHBoxLayout()
        for label, value in (("Slow", 64), ("Medium", 128), ("Fast", 220)):
            btn = QPushButton(label)
            btn.clicked.connect(lambda _=False, v=value: self._set_speed(v))
            presets.addWidget(btn)
        presets.addStretch(1)
        form.addRow("Speed preset", self._wrap(presets))
        return box

    def _mode_select(self) -> QGroupBox:
        box = QGroupBox("Control mode")
        row = QHBoxLayout(box)
        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._mode_buttons = {}
        for mode, label in (("walk", "Walk"), ("translate", "Translate"), ("rotate", "Rotate")):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _=False, m=mode: self._set_mode(m))
            self._mode_group.addButton(btn)
            self._mode_buttons[mode] = btn
            row.addWidget(btn)
        self._mode_buttons["walk"].setChecked(True)
        row.addStretch(1)
        return box

    # (attribute, button label, command-frame twist direction, grid cell).
    # Pressing a pad button walks in that direction until release. The
    # firmware holds home stance and ignores twist while the gait is a
    # posture (Stand/Sit), so a press always re-asserts a walking gait first.
    WALK_PAD = [
        ("yaw_ccw", "\u21ba", (0.0, 0.0, 1.0), (0, 0)),
        ("forward", "\u25b2", (1.0, 0.0, 0.0), (0, 1)),
        ("yaw_cw", "\u21bb", (0.0, 0.0, -1.0), (0, 2)),
        ("left", "\u25c0", (0.0, 1.0, 0.0), (1, 0)),
        ("back", "\u25bc", (-1.0, 0.0, 0.0), (1, 1)),
        ("right", "\u25b6", (0.0, -1.0, 0.0), (1, 2)),
    ]
    WALKING_GAITS = (api.GAIT_TRIPOD, api.GAIT_RIPPLE, api.GAIT_WAVE, api.GAIT_CRAWL)

    def _body_twist(self) -> QGroupBox:
        box = QGroupBox("Walk (hold a direction; release to stop)")
        row = QHBoxLayout(box)
        row.setSpacing(18)

        pad = QGridLayout()
        pad.setHorizontalSpacing(8)
        pad.setVerticalSpacing(8)
        self._pad_buttons = {}
        for attr, label, direction, (r, c) in self.WALK_PAD:
            btn = QPushButton(label)
            btn.setAutoRepeat(False)
            btn.setFixedSize(72, 48)
            btn.setToolTip(attr.replace("_", " ").title())
            btn.pressed.connect(lambda d=direction: self._pad_pressed(*d))
            btn.released.connect(self._pad_released)
            pad.addWidget(btn, r, c)
            self._pad_buttons[attr] = btn
        row.addLayout(pad)

        side = QFormLayout()
        side.setHorizontalSpacing(12)
        self._walk_speed = QSpinBox()
        self._walk_speed.setRange(10, 100)
        self._walk_speed.setValue(60)
        self._walk_speed.setSuffix(" %")
        side.addRow("Walk speed", self._walk_speed)
        legend = QLabel(
            "\u25b2\u25bc forward/back   \u25c0\u25b6 sideways   \u21ba\u21bb turn\n"
            "Uses the selected walking gait (Tripod if a posture is selected)."
        )
        legend.setStyleSheet(f"color: {DRACULA.comment};")
        side.addRow(legend)
        row.addLayout(side)
        row.addStretch(1)
        return box

    def _body_translation(self) -> QGroupBox:
        box = QGroupBox("Body translation with feet planted")
        form = QFormLayout(box)
        self._translate_spins = {}
        for attr, label in (("x", "Forward (mm)"), ("y", "Left (mm)"), ("z", "Up (mm)")):
            spin = QSpinBox()
            spin.setRange(-50, 50)
            self._translate_spins[attr] = spin
            form.addRow(label, spin)
        send = QPushButton("Apply translation")
        send.clicked.connect(self._send_translation)
        form.addRow("", send)
        return box

    def _body_rotation(self) -> QGroupBox:
        box = QGroupBox("Body rotation with feet planted")
        form = QFormLayout(box)
        self._rotate_spins = {}
        for attr, label in (("roll", "Roll (deg)"), ("pitch", "Pitch (deg)"), ("yaw", "Yaw (deg)")):
            spin = QSpinBox()
            spin.setRange(-25, 25)
            self._rotate_spins[attr] = spin
            form.addRow(label, spin)
        send = QPushButton("Apply rotation")
        send.clicked.connect(self._send_rotation)
        form.addRow("", send)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- actions ----------------------------------------------------------

    def _start_session(self) -> None:
        self.service.start_gait_test(
            self._param_spins["body_height"].value(),
            self._param_spins["stride_len"].value(),
            self._param_spins["step_height"].value(),
            self._param_spins["duty"].value(),
            self._param_spins["speed"].value(),
        )

    def _set_speed(self, value: int) -> None:
        self._param_spins["speed"].setValue(value)
        if self._session_active:
            self._send_gait_params()

    def _set_mode(self, mode: str) -> None:
        self.walk_box.setVisible(mode == "walk")
        self.translate_box.setVisible(mode == "translate")
        self.rotate_box.setVisible(mode == "rotate")
        if not self._session_active:
            return
        self.service.set_body_twist(0.0, 0.0, 0.0)
        self.service.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if mode != "walk":
            self.service.set_gait(api.GAIT_STAND)
            self._gait_buttons[api.GAIT_STAND].setChecked(True)

    def _hold_stand(self) -> None:
        self._zero_twist()
        self.service.set_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.service.stop_motion()
        # STOP_MOTION resets the firmware gait to Stand; mirror that here.
        self._gait_buttons[api.GAIT_STAND].setChecked(True)

    def _send_gait_params(self) -> None:
        self.service.set_gait_params(
            self._param_spins["body_height"].value(),
            self._param_spins["stride_len"].value(),
            self._param_spins["step_height"].value(),
            self._param_spins["duty"].value(),
            self._param_spins["speed"].value(),
        )

    def _walking_gait(self) -> int:
        """The checked walking gait, or Tripod when a posture is selected."""
        for gait in self.WALKING_GAITS:
            if self._gait_buttons[gait].isChecked():
                return gait
        return api.GAIT_TRIPOD

    def _pad_pressed(self, vx: float, vy: float, wz: float) -> None:
        # Always re-assert the walking gait: postures ignore twist, and Stop /
        # mode switches silently reset the firmware to Stand behind the UI.
        gait = self._walking_gait()
        if not self._gait_buttons[gait].isChecked():
            self._gait_buttons[gait].setChecked(True)
        self.service.set_gait(gait)
        magnitude = self._walk_speed.value() / 100.0
        self.service.set_body_twist(vx * magnitude, vy * magnitude, wz * magnitude)

    def _pad_released(self) -> None:
        self.service.set_body_twist(0.0, 0.0, 0.0)

    def _send_translation(self) -> None:
        self.service.set_gait(api.GAIT_STAND)
        self.service.set_body_pose(
            self._translate_spins["x"].value(),
            self._translate_spins["y"].value(),
            self._translate_spins["z"].value(),
            0.0,
            0.0,
            0.0,
        )

    def _send_rotation(self) -> None:
        self.service.set_gait(api.GAIT_STAND)
        self.service.set_body_pose(
            0.0,
            0.0,
            0.0,
            self._rotate_spins["roll"].value(),
            self._rotate_spins["pitch"].value(),
            self._rotate_spins["yaw"].value(),
        )

    def _zero_twist(self) -> None:
        self.service.set_body_twist(0.0, 0.0, 0.0)

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.subscribe(int(tlm.StreamId.CONTROL_STATE), 10)
            self.authority_lbl.setText("Ready to start maintenance gait session")
        else:
            self._session_active = False
            self._session_busy = False
            self.gate_badge.set("unknown", "idle")
            self.session_badge.set("stopped", "idle")
            self.authority_lbl.setText("Disconnected")
        self._apply_session_gate()

    def _on_session_busy(self, busy: bool) -> None:
        self._session_busy = busy
        self._apply_session_gate()

    def _on_simulation_mode(self, enabled: bool) -> None:
        self._simulation_mode = enabled
        if enabled:
            self.session_box.setTitle("Simulation motion session")
            self.start_btn.setText("Start simulation controls")
            self.stop_session_btn.setText("Stop simulation")
        else:
            self.session_box.setTitle("Maintenance gait session")
            self.start_btn.setText("Start gait test")
            self.stop_session_btn.setText("Stop && torque off")
        self._apply_session_gate()

    def _on_session_changed(self, active: bool, detail: str) -> None:
        self._session_active = active
        self.session_badge.set(
            "RUNNING" if active else detail,
            "warn" if active else "idle",
        )
        self.authority_lbl.setText(detail)
        self._apply_session_gate()

    def _apply_session_gate(self) -> None:
        self.start_btn.setEnabled(
            self._connected and not self._session_active and not self._session_busy
        )
        self.stop_session_btn.setEnabled(
            self._connected and (self._session_active or self._session_busy)
        )
        for control in self._session_controls:
            control.setEnabled(self._session_active)

    def _on_motion_result(self, kind: str, res) -> None:
        state = tlm.SAFETY_STATE_NAMES.get(res.state, str(res.state))
        verdict = "accepted" if res.ok else f"rejected ({res.result})"
        gate = "gate OPEN" if res.motion_allowed else "gate CLOSED"
        self.authority_lbl.setText(f"{kind}: {verdict} — state {state}, {gate}")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.CONTROL_STATE):
            self.gate_badge.set(
                "OPEN" if record.motion_gate else "CLOSED",
                "ok" if record.motion_gate else "warn",
            )


class LegLabPage(BasePage):
    title = "Leg Lab"
    subtitle = (
        "Per-leg foot and per-joint maintenance controls with live IK and "
        "clamp feedback. Honored only in Mac Maintenance with the lock held."
    )

    # Joint roles in firmware order (0=coxa, 1=femur, 2=tibia).
    JOINTS = [("Coxa", 0), ("Femur", 1), ("Tibia", 2)]

    def build(self) -> None:
        from hexapod_protocol import config as cfg
        from models import HexapodPoseModel

        self._num_legs = cfg.NUM_LEGS
        self._leg_rows: dict[int, int] = {}  # leg index -> live table row
        self._workspace_config = cfg.default_robot_config()
        self._workspace_model = HexapodPoseModel(self._workspace_config)
        self._target_reachable = True

        self.content.addWidget(self._maintenance_box())
        self.content.addWidget(self._leg_select())
        self.content.addWidget(self._foot_target())
        self.content.addWidget(self._joint_target())
        self.content.addWidget(self._live_legs())

        safety = QGroupBox("Stop")
        slay = QHBoxLayout(safety)
        stop = QPushButton("Stop motion (hold Stand)")
        stop.clicked.connect(self.service.stop_motion)
        estop = QPushButton("\u23fb  EMERGENCY STOP")
        estop.setObjectName("EmergencyStop")
        estop.clicked.connect(self.service.emergency_stop)
        slay.addWidget(stop)
        slay.addWidget(estop)
        slay.addStretch(1)
        self.content.addWidget(safety)

        self.banner = self.add_telemetry_banner([(tlm.StreamId.LEG_STATE, "leg_state")])

        self._connected = False
        self._state = -1
        self._lock_held = False

        self.service.connected.connect(self._on_connected)
        self.service.state_changed.connect(self._on_state_changed)
        self.service.maint_result.connect(self._on_maint_result)
        self.service.maint_lock_changed.connect(self._on_lock_changed)
        self.service.config_loaded.connect(self._on_config_loaded)
        self.service.leg_target_result.connect(self._on_leg_result)
        self.service.joint_target_result.connect(self._on_joint_result)
        self.service.telemetry.connect(self._on_telemetry)

        self._set_safe_home()
        self._update_joint_limits()
        self._apply_gates()

    # --- groups -----------------------------------------------------------

    def _maintenance_box(self) -> QGroupBox:
        box = QGroupBox("Maintenance lock")
        lay = QVBoxLayout(box)
        note = QLabel(
            "Leg and joint commands are rejected unless the robot is in Mac "
            "Maintenance with the lock held. Acquire the lock, command slowly, "
            "and release it when done."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(note)

        row = QHBoxLayout()
        enter = QPushButton("Enter Maintenance")
        enter.clicked.connect(self.service.enter_maintenance)
        self.enter_maint_btn = enter
        leave = QPushButton("Exit Maintenance")
        leave.clicked.connect(self.service.exit_maintenance)
        self.exit_maint_btn = leave
        row.addWidget(enter)
        row.addWidget(leave)
        row.addStretch(1)
        lay.addLayout(row)

        self.lock_lbl = QLabel("Maintenance lock: none")
        self.lock_lbl.setObjectName("MonoLabel")
        lay.addWidget(self.lock_lbl)
        return box

    def _leg_select(self) -> QGroupBox:
        box = QGroupBox("Leg selection")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        self.leg_combo = QComboBox()
        names = (
            "rear left",
            "rear right",
            "middle right",
            "front right",
            "front left",
            "middle left",
        )
        for leg, name in enumerate(names[: self._num_legs]):
            self.leg_combo.addItem(f"Leg {leg + 1} — {name}", leg)
        self.leg_combo.currentIndexChanged.connect(self._on_leg_changed)
        form.addRow("Active leg", self.leg_combo)
        return box

    def _foot_target(self) -> QGroupBox:
        box = QGroupBox("Foot target (mm, body frame)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        hint = QLabel(
            "X moves forward/back, Y moves left/right, and Z moves up/down. "
            "Start from Safe home and make small changes; the preview uses the "
            "active robot geometry before a command can be sent."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        form.addRow(hint)
        self._foot_spins = {}
        for attr, label in (("x", "X (forward)"), ("y", "Y (left)"), ("z", "Z (up)")):
            spin = QSpinBox()
            spin.setRange(-350, 350)
            spin.setValue(0)
            spin.setSuffix(" mm")
            spin.setSingleStep(1)
            spin.setAccelerated(True)
            spin.valueChanged.connect(self._update_reach_preview)
            self._foot_spins[attr] = spin
            form.addRow(label, spin)

        self.reach_badge = StatusBadge("Reach preview")
        self.reach_badge.set("checking", "idle")
        self.reach_detail = QLabel("")
        self.reach_detail.setObjectName("MonoLabel")
        self.reach_detail.setWordWrap(True)
        reach_row = QHBoxLayout()
        reach_row.addWidget(self.reach_badge)
        reach_row.addWidget(self.reach_detail, 1)
        form.addRow("Workspace", self._wrap(reach_row))

        reset = QPushButton("Use safe home")
        reset.setToolTip(
            "Load the selected leg's configured all-joints-at-180° home target."
        )
        reset.clicked.connect(self._set_safe_home)
        self.safe_home_btn = reset
        send = QPushButton("Send foot target")
        send.setProperty("accent", True)
        send.clicked.connect(self._send_foot_target)
        self.send_foot_btn = send
        actions = QHBoxLayout()
        actions.addWidget(reset)
        actions.addWidget(send)
        actions.addStretch(1)
        form.addRow("", self._wrap(actions))
        self.foot_result = QLabel("--")
        self.foot_result.setObjectName("MonoLabel")
        self.foot_result.setWordWrap(True)
        form.addRow("Result", self.foot_result)
        return box

    def _joint_target(self) -> QGroupBox:
        box = QGroupBox("Joint target (physical servo angle)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        note = QLabel(
            "180° is the safe calibrated center (firmware-relative 0° / nominal "
            "MX-28 center tick 2048). Move one joint at a time and watch clamp feedback."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DRACULA.comment};")
        form.addRow(note)
        self.joint_combo = QComboBox()
        for label, jid in self.JOINTS:
            self.joint_combo.addItem(label, jid)
        self.joint_combo.currentIndexChanged.connect(self._update_joint_limits)
        form.addRow("Joint", self.joint_combo)
        self.joint_angle = QSpinBox()
        self.joint_angle.setRange(0, 360)
        self.joint_angle.setValue(180)
        self.joint_angle.setSuffix(" \u00b0")
        self.joint_angle.setToolTip(
            "Displayed as a physical servo angle; 180° is sent as 0° relative."
        )
        form.addRow("Servo angle", self.joint_angle)
        self.joint_limit_hint = QLabel("")
        self.joint_limit_hint.setObjectName("MonoLabel")
        form.addRow("Configured travel", self.joint_limit_hint)
        center = QPushButton("Reset to 180° center")
        center.clicked.connect(lambda: self.joint_angle.setValue(180))
        send = QPushButton("Send joint target")
        send.setProperty("accent", True)
        send.clicked.connect(self._send_joint_target)
        self.send_joint_btn = send
        actions = QHBoxLayout()
        actions.addWidget(center)
        actions.addWidget(send)
        actions.addStretch(1)
        form.addRow("", self._wrap(actions))
        self.joint_result = QLabel("--")
        self.joint_result.setObjectName("MonoLabel")
        self.joint_result.setWordWrap(True)
        form.addRow("Result", self.joint_result)
        return box

    def _live_legs(self) -> QGroupBox:
        box = QGroupBox("Live leg state")
        lay = QVBoxLayout(box)
        self.leg_table = QTableWidget(0, 5)
        self.leg_table.setHorizontalHeaderLabels(
            ["Leg", "Foot X", "Foot Y", "Foot Z", "IK"]
        )
        self.leg_table.verticalHeader().setVisible(False)
        self.leg_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.leg_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.leg_table.setMinimumHeight(180)
        lay.addWidget(self.leg_table)
        return box

    # --- actions ----------------------------------------------------------

    def _send_foot_target(self) -> None:
        self.foot_result.setText("sending\u2026")
        self.service.set_leg_target(
            self.leg_combo.currentData(),
            self._foot_spins["x"].value(),
            self._foot_spins["y"].value(),
            self._foot_spins["z"].value(),
        )

    def _send_joint_target(self) -> None:
        self.joint_result.setText("sending\u2026")
        # The UI shows the physical servo convention (180° center); firmware
        # expects a URDF-zero-relative centidegree angle.
        self.service.set_joint_target(
            self.leg_combo.currentData(),
            self.joint_combo.currentData(),
            (self.joint_angle.value() - 180) * 100,
        )

    def _on_leg_changed(self, _index: int) -> None:
        self.joint_angle.setValue(180)
        self._set_safe_home()
        self._update_joint_limits()

    def _update_joint_limits(self, _index: int | None = None) -> None:
        import math

        from hexapod_protocol import config as cfg

        leg = self.leg_combo.currentData()
        joint = self.joint_combo.currentData()
        if leg is None or joint is None:
            return
        servo = cfg.ServoMap(self._workspace_config).servo_for(int(leg), int(joint))
        if servo is None:
            self.joint_angle.setRange(180, 180)
            self.joint_limit_hint.setText("unmapped joint")
            return
        physical = sorted(
            (
                180.0 + math.degrees(cfg.tick_to_angle(servo, servo.min_tick)),
                180.0 + math.degrees(cfg.tick_to_angle(servo, servo.max_tick)),
            )
        )
        low = max(0, math.ceil(physical[0]))
        high = min(360, math.floor(physical[1]))
        self.joint_angle.setRange(low, high)
        self.joint_angle.setValue(min(high, max(low, 180)))
        self.joint_limit_hint.setText(
            f"{low}°–{high}° from servo {servo.id} limits "
            f"({servo.min_tick}–{servo.max_tick} ticks)"
        )

    def _set_safe_home(self) -> None:
        leg = self.leg_combo.currentData()
        if leg is None:
            return
        foot = self._workspace_model.home_foot(int(leg))
        self._foot_spins["x"].setValue(round(foot.x))
        self._foot_spins["y"].setValue(round(foot.y))
        self._foot_spins["z"].setValue(round(foot.z))
        self._update_reach_preview()

    def _update_reach_preview(self, _value: int | None = None) -> None:
        leg = self.leg_combo.currentData()
        if leg is None:
            return
        reach = self._workspace_model.assess_foot_target(
            int(leg),
            self._foot_spins["x"].value(),
            self._foot_spins["y"].value(),
            self._foot_spins["z"].value(),
        )
        self._target_reachable = reach.reachable
        if reach.inside_safe_margin:
            self.reach_badge.set("SAFE", "ok")
            verdict = "inside recommended margin"
        elif reach.reachable:
            self.reach_badge.set("EDGE", "warn")
            verdict = "reachable, but near an IK boundary"
        else:
            self.reach_badge.set("OUTSIDE", "error")
            verdict = "outside geometric reach — command blocked"
        self.reach_detail.setText(
            f"{verdict}; planar reach {reach.distance_mm:.1f} mm · "
            f"recommended {reach.safe_minimum_mm:.1f}–{reach.safe_maximum_mm:.1f} mm"
        )
        if hasattr(self, "send_foot_btn"):
            self._apply_gates()

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.subscribe(int(tlm.StreamId.LEG_STATE), 10)
        else:
            self._state = -1
            self._lock_held = False
            self.lock_lbl.setText("Maintenance lock: none")
            self.foot_result.setText("--")
            self.joint_result.setText("--")
            self.leg_table.setRowCount(0)
            self._leg_rows.clear()
        self._apply_gates()

    def _on_config_loaded(self, config) -> None:
        if config is None:
            return
        from models import HexapodPoseModel

        self._workspace_config = config
        self._workspace_model = HexapodPoseModel(config)
        self._set_safe_home()
        self._update_joint_limits()

    def _on_state_changed(self, state: int) -> None:
        self._state = state
        self._apply_gates()

    def _on_lock_changed(self, held: bool, token: int) -> None:
        self._lock_held = held
        if held:
            self.lock_lbl.setText(
                f"Maintenance lock: held (token {token}, heartbeating)"
            )
        else:
            self.lock_lbl.setText("Maintenance lock: none")
        self._apply_gates()

    def _apply_gates(self) -> None:
        """Only offer commands the firmware would accept right now."""
        con = self._connected
        held = self._lock_held

        def gate(btn, enabled: bool, why: str) -> None:
            btn.setEnabled(enabled)
            btn.setToolTip("" if enabled else why)

        gate(
            self.enter_maint_btn,
            con and not held and self._state in (-1, tlm.SafetyState.DISARMED),
            "Requires connection and a Disarmed robot.",
        )
        gate(self.exit_maint_btn, con and held, "The maintenance lock is not held.")
        gate(
            self.send_foot_btn,
            con and held and self._target_reachable,
            "Adjust XYZ into the reachable workspace."
            if con and held
            else "Enter Maintenance first — targets need the lock.",
        )
        gate(
            self.send_joint_btn,
            con and held,
            "Enter Maintenance first — targets need the lock.",
        )

    def _on_maint_result(self, res) -> None:
        if res.token:
            self.lock_lbl.setText(f"Maintenance lock: held (token {res.token})")
        elif res.ok:
            self.lock_lbl.setText("Maintenance lock: none")
        else:
            self.lock_lbl.setText(f"Maintenance lock: rejected (result {res.result})")

    def _on_leg_result(self, res) -> None:
        if res is None:
            self.foot_result.setText("failed (rejected or timed out)")
            return
        state = tlm.SAFETY_STATE_NAMES.get(res.state, str(res.state))
        if res.ok:
            c, f, t = res.ticks
            clamp = self._clamp_text(res.clamp_low, res.clamp_high)
            self.foot_result.setText(
                f"reachable \u2014 ticks coxa={c} femur={f} tibia={t}; "
                f"clamp {clamp}; state {state}"
            )
        elif res.result == api.MAINT_TARGET_UNREACHABLE:
            self.foot_result.setText(f"UNREACHABLE \u2014 not committed; state {state}")
        else:
            self.foot_result.setText(f"rejected (result {res.result}); state {state}")

    def _on_joint_result(self, res) -> None:
        if res is None:
            self.joint_result.setText("failed (rejected or timed out)")
            return
        state = tlm.SAFETY_STATE_NAMES.get(res.state, str(res.state))
        # center-all sends an aggregate (AllJointTargetResult) with a stored
        # count and an 18-bit clamp mask; single-joint sends a per-joint result.
        if hasattr(res, "stored_count"):
            if res.ok:
                clamped = bin(res.clamp_mask).count("1")
                self.joint_result.setText(
                    f"ok \u2014 {res.stored_count}/18 joints; "
                    f"{clamped} clamped; state {state}"
                )
            else:
                self.joint_result.setText(
                    f"rejected (result {res.result}); state {state}"
                )
            return
        if res.ok:
            flags = []
            if res.clamped_low:
                flags.append("low")
            if res.clamped_high:
                flags.append("high")
            clamp = "+".join(flags) if flags else "none"
            self.joint_result.setText(
                f"ok \u2014 tick={res.tick}; clamp {clamp}; state {state}"
            )
        else:
            self.joint_result.setText(f"rejected (result {res.result}); state {state}")

    def _clamp_text(self, low: int, high: int) -> str:
        names = ("coxa", "femur", "tibia")
        hit = []
        for j, name in enumerate(names):
            if low & (1 << j):
                hit.append(f"{name}\u2193")
            if high & (1 << j):
                hit.append(f"{name}\u2191")
        return "+".join(hit) if hit else "none"

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id != int(tlm.StreamId.LEG_STATE):
            return
        for leg in record.legs:
            row = self._leg_rows.get(leg.leg)
            if row is None:
                row = self.leg_table.rowCount()
                self.leg_table.insertRow(row)
                self._leg_rows[leg.leg] = row
            if leg.reachable:
                ik = "clamped" if leg.clamped else "reachable"
            else:
                ik = "UNREACHABLE"
            values = [
                str(leg.leg),
                str(leg.foot_x_mm),
                str(leg.foot_y_mm),
                str(leg.foot_z_mm),
                ik,
            ]
            for col, text in enumerate(values):
                self.leg_table.setItem(row, col, QTableWidgetItem(text))

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w


class ServoConfigPage(BasePage):
    title = "Robot Calibration && Config"
    subtitle = (
        "Calibrate servo zero/sign/travel, leg geometry, and foot sensors; then "
        "validate, stage, commit, or export the EEPROM-backed robot config."
    )

    COLUMNS = ["ID", "Leg", "Joint", "Sign", "Trim", "Min tick", "Max tick"]
    GEOMETRY_COLUMNS = [
        "Leg",
        "Mount X (mm)",
        "Mount Y (mm)",
        "Mount Z (mm)",
        "Yaw (deg)",
    ]
    SENSOR_COLUMNS = ["Foot", "Baseline (raw)", "Near", "Touch", "Load", "Enabled"]

    # (column, attribute, lo, hi) for in-table validation of edited servo rows.
    FIELDS = [
        (0, "id", 1, 253),
        (1, "leg", 0, 5),
        (2, "joint", 0, 2),
        (3, "sign", -1, 1),
        (4, "trim_ticks", -2048, 2048),
        (5, "min_tick", 0, 4095),
        (6, "max_tick", 0, 4095),
    ]

    def build(self) -> None:
        from hexapod_protocol import config as cfg

        self._cfg = cfg
        self._loaded = None  # last config read from the robot / reset (the base)
        self._edited = None  # config last sent to the staging buffer
        self._connected = False
        self._lock_held = False
        self._latest_pressure_raw = [None] * tlm.NUM_FEET
        self._latest_pressure_time = 0.0

        self.content.addWidget(self._summary_box())
        self.content.addWidget(self._servo_table())
        self.content.addWidget(self._geometry_editor())
        self.content.addWidget(self._sensor_calibration_editor())
        self.content.addWidget(self._actions_box())
        self.content.addWidget(self._diff_box())

        self.service.connected.connect(self._on_connected)
        self.service.config_loaded.connect(self._on_config_loaded)
        self.service.config_summary.connect(self._on_config_summary)
        self.service.config_staged.connect(self._on_config_staged)
        self.service.config_result.connect(self._on_config_result)
        self.service.maint_lock_changed.connect(self._on_lock_changed)
        self.service.telemetry.connect(self._on_telemetry)
        self._apply_gates()

    # --- groups -----------------------------------------------------------

    def _summary_box(self) -> QGroupBox:
        box = QGroupBox("Config summary")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(10)
        self.name_edit = QLineEdit()
        self.name_edit.setMaxLength(15)
        form.addRow("Robot name", self.name_edit)
        self.schema_lbl = QLabel("--")
        self.schema_lbl.setObjectName("MonoLabel")
        form.addRow("Schema / size", self.schema_lbl)
        self.persist_lbl = QLabel("--")
        self.persist_lbl.setObjectName("MonoLabel")
        form.addRow("Persistence", self.persist_lbl)

        row = QHBoxLayout()
        load = QPushButton("Load from robot")
        load.setProperty("accent", True)
        load.clicked.connect(self.service.load_config)
        self.reset_btn = QPushButton("Reset to defaults")
        self.reset_btn.clicked.connect(self._reset_defaults)
        row.addWidget(load)
        row.addWidget(self.reset_btn)
        row.addStretch(1)
        form.addRow("", self._wrap(row))
        return box

    def _servo_table(self) -> QGroupBox:
        box = QGroupBox("Servo map")
        lay = QVBoxLayout(box)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMinimumHeight(320)
        lay.addWidget(self.table)
        hint = QLabel(
            "Double-click a cell to edit. Sign must be +1 or -1; ticks are "
            "0\u20134095. Edits stay local until you stage and commit them."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(hint)
        return box

    def _geometry_editor(self) -> QGroupBox:
        box = QGroupBox("Leg geometry")
        lay = QVBoxLayout(box)

        form = QFormLayout()
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(10)
        self.link_spins = {
            "coxa": self._mm_spin(0.01, 655.35),
            "femur": self._mm_spin(0.01, 655.35),
            "tibia": self._mm_spin(0.01, 655.35),
        }
        form.addRow("Coxa link", self.link_spins["coxa"])
        form.addRow("Femur link", self.link_spins["femur"])
        form.addRow("Tibia link", self.link_spins["tibia"])

        self.home_radius_spin = self._mm_spin(0.01, 655.35)
        self.home_foot_z_spin = self._mm_spin(-327.68, 327.67)
        self.coxa_lift_spin = self._mm_spin(0.0, 655.35)
        form.addRow("Home radius", self.home_radius_spin)
        form.addRow("Home foot Z", self.home_foot_z_spin)
        form.addRow("Coxa lift", self.coxa_lift_spin)
        lay.addLayout(form)

        self.geometry_table = QTableWidget(0, len(self.GEOMETRY_COLUMNS))
        self.geometry_table.setHorizontalHeaderLabels(self.GEOMETRY_COLUMNS)
        self.geometry_table.verticalHeader().setVisible(False)
        self.geometry_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.geometry_table.setMinimumHeight(220)
        lay.addWidget(self.geometry_table)

        hint = QLabel(
            "Edit the measured body-to-coxa mounts and hip yaw. The configured "
            "neutral stance must remain inside the two-link reach envelope."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(hint)
        return box

    def _mm_spin(self, minimum: float, maximum: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(0.1)
        spin.setSuffix(" mm")
        return spin

    def _sensor_calibration_editor(self) -> QGroupBox:
        box = QGroupBox("Foot sensor calibration")
        lay = QVBoxLayout(box)
        self.sensor_table = QTableWidget(0, len(self.SENSOR_COLUMNS))
        self.sensor_table.setHorizontalHeaderLabels(self.SENSOR_COLUMNS)
        self.sensor_table.verticalHeader().setVisible(False)
        self.sensor_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.sensor_table.setMinimumHeight(220)
        lay.addWidget(self.sensor_table)

        row = QHBoxLayout()
        self.capture_baselines_btn = QPushButton("Copy live pressure baselines")
        self.capture_baselines_btn.clicked.connect(self._capture_live_baselines)
        row.addWidget(self.capture_baselines_btn)
        row.addStretch(1)
        self.sensor_capture_lbl = QLabel("Waiting for raw sensor telemetry.")
        self.sensor_capture_lbl.setObjectName("MonoLabel")
        row.addWidget(self.sensor_capture_lbl)
        lay.addLayout(row)

        hint = QLabel(
            "Copy captures the latest raw pressure values into the local config. "
            "Set nonzero near/touch/load thresholds before enabling a foot, then "
            "stage, validate, and commit to apply and persist the calibration."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(hint)
        return box

    def _actions_box(self) -> QGroupBox:
        box = QGroupBox("Config actions")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        diff = QPushButton("Diff vs loaded")
        diff.clicked.connect(self._show_diff)
        self.stage_btn = QPushButton("Stage to robot")
        self.stage_btn.setProperty("accent", True)
        self.stage_btn.clicked.connect(self._stage)
        validate = QPushButton("Validate staged")
        validate.clicked.connect(self.service.validate_config)
        self.commit_btn = QPushButton("Commit to EEPROM")
        self.commit_btn.clicked.connect(self._commit)
        export = QPushButton("Export JSON\u2026")
        export.clicked.connect(self._export)
        importb = QPushButton("Import JSON\u2026")
        importb.clicked.connect(self._import)
        buttons = [diff, self.stage_btn, validate, self.commit_btn, export, importb]
        for i, btn in enumerate(buttons):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 3, i % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        self.action_lbl = QLabel("Load the config to begin.")
        self.action_lbl.setObjectName("MonoLabel")
        self.action_lbl.setWordWrap(True)
        grid.addWidget(self.action_lbl, 2, 0, 1, 3)
        return box

    def _diff_box(self) -> QGroupBox:
        box = QGroupBox("Pending changes")
        lay = QVBoxLayout(box)
        self.diff_text = QPlainTextEdit()
        self.diff_text.setReadOnly(True)
        self.diff_text.setMinimumHeight(120)
        self.diff_text.setPlaceholderText("No changes staged.")
        lay.addWidget(self.diff_text)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- table <-> config -------------------------------------------------

    def _populate_table(self, config) -> None:
        servos = config.servos
        self.table.setRowCount(len(servos))
        for row, s in enumerate(servos):
            values = [
                s.id,
                s.leg,
                s.joint,
                s.sign,
                s.trim_ticks,
                s.min_tick,
                s.max_tick,
            ]
            for col, val in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(str(val)))
        self._populate_geometry(config)
        self._populate_sensor_calibration(config)

    def _populate_geometry(self, config) -> None:
        self.link_spins["coxa"].setValue(config.links.coxa_cmm / 100.0)
        self.link_spins["femur"].setValue(config.links.femur_cmm / 100.0)
        self.link_spins["tibia"].setValue(config.links.tibia_cmm / 100.0)
        self.home_radius_spin.setValue(config.geometry.home_radius_cmm / 100.0)
        self.home_foot_z_spin.setValue(config.geometry.home_foot_z_cmm / 100.0)
        self.coxa_lift_spin.setValue(config.geometry.coxa_lift_cmm / 100.0)

        self.geometry_table.setRowCount(len(config.legs))
        for row, leg in enumerate(config.legs):
            label = QTableWidgetItem(f"Leg {row + 1}")
            label.setFlags(label.flags() & ~Qt.ItemIsEditable)
            self.geometry_table.setItem(row, 0, label)
            values = (
                leg.mount_x_dmm / 10.0,
                leg.mount_y_dmm / 10.0,
                leg.mount_z_dmm / 10.0,
                leg.mount_yaw_cdeg / 100.0,
            )
            for column, value in enumerate(values, start=1):
                self.geometry_table.setItem(
                    row, column, QTableWidgetItem(f"{value:.2f}")
                )

    def _populate_sensor_calibration(self, config) -> None:
        self.sensor_table.setRowCount(len(config.feet))
        for row, foot in enumerate(config.feet):
            label = QTableWidgetItem(f"Foot {row + 1}")
            label.setFlags(label.flags() & ~Qt.ItemIsEditable)
            self.sensor_table.setItem(row, 0, label)
            for column, value in enumerate(
                (
                    foot.pressure_baseline,
                    foot.near_thresh,
                    foot.touch_thresh,
                    foot.load_thresh,
                ),
                start=1,
            ):
                self.sensor_table.setItem(row, column, QTableWidgetItem(str(value)))
            enabled = QCheckBox()
            enabled.setChecked(bool(foot.enabled))
            enabled.setToolTip("Enable fused contact classification for this foot")
            self.sensor_table.setCellWidget(row, 5, enabled)

    def _read_table(self):
        """Build a RobotConfig from the loaded base + edited servo rows.

        Returns ``(config, None)`` on success or ``(None, error)`` if a cell is
        non-numeric or out of range.
        """
        import copy

        if self._loaded is None:
            return None, "no config loaded"
        config = copy.deepcopy(self._loaded)
        config.robot_name = self.name_edit.text().strip()
        for row in range(self.table.rowCount()):
            if row >= len(config.servos):
                break
            s = config.servos[row]
            for col, attr, lo, hi in self.FIELDS:
                item = self.table.item(row, col)
                text = item.text().strip() if item is not None else ""
                try:
                    val = int(text)
                except ValueError:
                    return None, f"row {row} {attr}: '{text}' is not an integer"
                if attr == "sign":
                    if val not in (-1, 1):
                        return None, f"row {row} sign must be +1 or -1"
                elif not (lo <= val <= hi):
                    return None, f"row {row} {attr}: {val} out of [{lo}, {hi}]"
                setattr(s, attr, val)
            if s.min_tick >= s.max_tick:
                return None, f"row {row}: min tick must be < max tick"
        geometry_error = self._read_geometry(config)
        if geometry_error is not None:
            return None, geometry_error
        sensor_error = self._read_sensor_calibration(config)
        if sensor_error is not None:
            return None, sensor_error
        validation_errors = self._cfg.validate_robot_config(config)
        if validation_errors:
            return None, validation_errors[0]
        return config, None

    def _read_geometry(self, config) -> str | None:
        config.links.coxa_cmm = round(self.link_spins["coxa"].value() * 100)
        config.links.femur_cmm = round(self.link_spins["femur"].value() * 100)
        config.links.tibia_cmm = round(self.link_spins["tibia"].value() * 100)
        config.geometry.home_radius_cmm = round(self.home_radius_spin.value() * 100)
        config.geometry.home_foot_z_cmm = round(self.home_foot_z_spin.value() * 100)
        config.geometry.coxa_lift_cmm = round(self.coxa_lift_spin.value() * 100)
        if self.geometry_table.rowCount() != len(config.legs):
            return "leg geometry row count does not match config"

        fields = (
            ("mount_x_dmm", "mount X", 10),
            ("mount_y_dmm", "mount Y", 10),
            ("mount_z_dmm", "mount Z", 10),
            ("mount_yaw_cdeg", "yaw", 100),
        )
        for row, leg in enumerate(config.legs):
            for column, (attribute, label, scale) in enumerate(fields, start=1):
                item = self.geometry_table.item(row, column)
                text = item.text().strip() if item is not None else ""
                try:
                    value = round(float(text) * scale)
                except (OverflowError, ValueError):
                    return f"leg {row + 1} {label}: '{text}' is not a number"
                if not -0x8000 <= value <= 0x7FFF:
                    return f"leg {row + 1} {label} is outside the signed range"
                setattr(leg, attribute, value)
        return None

    def _read_sensor_calibration(self, config) -> str | None:
        if self.sensor_table.rowCount() != len(config.feet):
            return "foot calibration row count does not match config"
        fields = (
            ("pressure_baseline", "baseline", -0x80000000, 0x7FFFFFFF),
            ("near_thresh", "near threshold", 0, 0xFFFF),
            ("touch_thresh", "touch threshold", 0, 0xFFFF),
            ("load_thresh", "load threshold", 0, 0xFFFF),
        )
        for row, foot in enumerate(config.feet):
            for column, (attribute, label, minimum, maximum) in enumerate(
                fields, start=1
            ):
                item = self.sensor_table.item(row, column)
                text = item.text().strip() if item is not None else ""
                try:
                    value = int(text)
                except ValueError:
                    return f"foot {row + 1} {label}: '{text}' is not an integer"
                if not minimum <= value <= maximum:
                    return (
                        f"foot {row + 1} {label}: {value} out of "
                        f"[{minimum}, {maximum}]"
                    )
                setattr(foot, attribute, value)
            enabled = self.sensor_table.cellWidget(row, 5)
            if not isinstance(enabled, QCheckBox):
                return f"foot {row + 1} enabled control is unavailable"
            foot.enabled = 1 if enabled.isChecked() else 0
        return None

    def _diff_lines(self, edited) -> list:
        lines = []
        if self._loaded is None:
            return lines
        if edited.robot_name != self._loaded.robot_name:
            lines.append(
                f"robot_name: {self._loaded.robot_name!r} -> {edited.robot_name!r}"
            )
        for i, (old, new) in enumerate(zip(self._loaded.servos, edited.servos)):
            for _col, attr, _lo, _hi in self.FIELDS:
                ov, nv = getattr(old, attr), getattr(new, attr)
                if ov != nv:
                    lines.append(f"servo[{i}].{attr}: {ov} -> {nv}")
        geometry_fields = (
            ("links.coxa_cmm", self._loaded.links.coxa_cmm, edited.links.coxa_cmm),
            ("links.femur_cmm", self._loaded.links.femur_cmm, edited.links.femur_cmm),
            ("links.tibia_cmm", self._loaded.links.tibia_cmm, edited.links.tibia_cmm),
            (
                "geometry.home_radius_cmm",
                self._loaded.geometry.home_radius_cmm,
                edited.geometry.home_radius_cmm,
            ),
            (
                "geometry.home_foot_z_cmm",
                self._loaded.geometry.home_foot_z_cmm,
                edited.geometry.home_foot_z_cmm,
            ),
            (
                "geometry.coxa_lift_cmm",
                self._loaded.geometry.coxa_lift_cmm,
                edited.geometry.coxa_lift_cmm,
            ),
        )
        for name, old, new in geometry_fields:
            if old != new:
                lines.append(f"{name}: {old} -> {new}")
        for index, (old, new) in enumerate(zip(self._loaded.legs, edited.legs)):
            for attribute in (
                "mount_x_dmm",
                "mount_y_dmm",
                "mount_z_dmm",
                "mount_yaw_cdeg",
            ):
                old_value, new_value = getattr(old, attribute), getattr(new, attribute)
                if old_value != new_value:
                    lines.append(f"leg[{index}].{attribute}: {old_value} -> {new_value}")
        for index, (old, new) in enumerate(zip(self._loaded.feet, edited.feet)):
            for attribute in (
                "pressure_baseline",
                "near_thresh",
                "touch_thresh",
                "load_thresh",
                "enabled",
            ):
                old_value, new_value = getattr(old, attribute), getattr(new, attribute)
                if old_value != new_value:
                    lines.append(f"foot[{index}].{attribute}: {old_value} -> {new_value}")
        return lines

    # --- actions ----------------------------------------------------------

    def _reset_defaults(self) -> None:
        if self._confirm(
            "Reset config to defaults",
            "Reload the compiled SAFE defaults into the staging buffer? This "
            "discards staged edits (commit separately to persist).",
        ):
            self.service.reset_config_defaults()

    def _show_diff(self) -> None:
        edited, err = self._read_table()
        if err is not None:
            self.diff_text.setPlainText(f"cannot diff: {err}")
            return
        lines = self._diff_lines(edited)
        self.diff_text.setPlainText(
            "\n".join(lines) if lines else "No changes vs loaded config."
        )

    def _capture_live_baselines(self) -> None:
        if self._loaded is None:
            self.sensor_capture_lbl.setText("Load a config before capturing.")
            return
        if time.monotonic() - self._latest_pressure_time > 2.0:
            self.sensor_capture_lbl.setText("Raw pressure telemetry is stale.")
            return
        copied = 0
        for foot, pressure in enumerate(self._latest_pressure_raw):
            if pressure is None or foot >= self.sensor_table.rowCount():
                continue
            self.sensor_table.item(foot, 1).setText(str(pressure))
            copied += 1
        if copied:
            self.sensor_capture_lbl.setText(
                f"Copied {copied} live baseline(s); stage and commit to persist."
            )
        else:
            self.sensor_capture_lbl.setText("No raw pressure samples are available.")

    def _stage(self) -> None:
        edited, err = self._read_table()
        if err is not None:
            self.action_lbl.setText(f"stage blocked: {err}")
            return
        lines = self._diff_lines(edited)
        summary = f"{len(lines)} field(s) changed" if lines else "no changes"
        if not self._confirm(
            "Stage config to robot",
            f"Write the edited config to the staging buffer ({summary})? "
            "It is not persisted until you commit.",
        ):
            return
        self._edited = edited
        self.action_lbl.setText("staging\u2026")
        self.service.stage_config(edited)

    def _commit(self) -> None:
        if self._confirm(
            "Commit config to EEPROM",
            "Persist the staged config to the 24LC32 EEPROM? The firmware "
            "requires Maintenance with all servo torque off and rejects an "
            "invalid staged config.",
        ):
            self.action_lbl.setText("committing\u2026")
            self.service.commit_config()

    def _export(self) -> None:
        edited, err = self._read_table()
        if err is not None:
            self.action_lbl.setText(f"export blocked: {err}")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export config JSON", "robot_config.json", "JSON (*.json)"
        )
        if not path:
            return
        import dataclasses
        import json

        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(dataclasses.asdict(edited), fh, indent=2)
        except OSError as exc:
            self.action_lbl.setText(f"export failed: {exc}")
            return
        self.action_lbl.setText(f"exported to {path}")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Import config JSON", "", "JSON (*.json)"
        )
        if not path:
            return
        import json

        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
            config = self._config_from_dict(data)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.action_lbl.setText(f"import failed: {exc}")
            return
        self._loaded = config
        self.name_edit.setText(config.robot_name)
        self._populate_table(config)
        self._apply_gates()
        self.action_lbl.setText(f"imported {path} (stage to apply)")
        self.diff_text.setPlainText("Imported config loaded as the new base.")

    def _config_from_dict(self, data: dict):
        cfg = self._cfg
        rc_data = data.get("rc_input", {})
        rc_input = (
            cfg.RcInputCalibration(
                channels=[
                    cfg.RcChannelCalibration(**channel)
                    for channel in rc_data["channels"]
                ]
            )
            if "channels" in rc_data
            else cfg.default_rc_input_calibration()
        )
        return cfg.RobotConfig(
            schema_version=cfg.SCHEMA_VERSION,
            robot_name=data.get("robot_name", ""),
            links=cfg.LinkLengths(**data["links"]),
            geometry=cfg.BodyGeometry(**data["geometry"]),
            legs=[cfg.LegGeometry(**d) for d in data["legs"]],
            servos=[cfg.ServoConfig(**d) for d in data["servos"]],
            gait=cfg.GaitDefaults(**data["gait"]),
            rc_input=rc_input,
            body_command=cfg.BodyCommandLimits(**data.get("body_command", {})),
            feet=[cfg.FootSensorCal(**d) for d in data["feet"]],
            feature_defaults=data.get("feature_defaults", 0),
        )

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.load_config()
            self.service.subscribe(int(tlm.StreamId.I2C_SENSORS_RAW), 10)
        else:
            self.table.setRowCount(0)
            self.sensor_table.setRowCount(0)
            self._loaded = None
            self._latest_pressure_raw = [None] * tlm.NUM_FEET
            self._latest_pressure_time = 0.0
            self.schema_lbl.setText("--")
            self.persist_lbl.setText("--")
            self.action_lbl.setText("Load the config to begin.")
        self._apply_gates()

    def _on_lock_changed(self, held: bool, _token: int) -> None:
        self._lock_held = held
        self._apply_gates()

    def _apply_gates(self) -> None:
        allowed = self._connected and self._lock_held
        why = "Enter Maintenance first; Commit/Reset also require torque off."
        for button in (self.reset_btn, self.stage_btn, self.commit_btn):
            button.setEnabled(allowed)
            button.setToolTip("" if allowed else why)
        capture_allowed = (
            self._connected
            and self._loaded is not None
            and any(value is not None for value in self._latest_pressure_raw)
        )
        self.capture_baselines_btn.setEnabled(capture_allowed)
        self.capture_baselines_btn.setToolTip(
            "" if capture_allowed else "Connect and wait for raw sensor telemetry."
        )

    def _on_config_loaded(self, config) -> None:
        if config is None:
            self.action_lbl.setText("config read failed")
            return
        self._loaded = config
        self.name_edit.setText(config.robot_name)
        self.schema_lbl.setText(
            f"v{config.schema_version}  ({len(config.servos)} servos)"
        )
        self._populate_table(config)
        self.action_lbl.setText("config loaded")
        self.diff_text.setPlainText("")
        self._apply_gates()

    def _on_config_summary(self, summary) -> None:
        if summary is None:
            self.persist_lbl.setText("--")
            return
        persist = "persistent" if summary.persistent else "VOLATILE (no EEPROM)"
        staged = "staged valid" if summary.staged_valid else "staged invalid"
        self.persist_lbl.setText(f"{persist} \u2014 {staged}")

    def _on_config_staged(self, ok: bool) -> None:
        if ok:
            self._loaded = self._edited
            self.action_lbl.setText("staged ok \u2014 validate, then commit")
            self.diff_text.setPlainText("Staged. Validate and commit to persist.")
        else:
            self.action_lbl.setText("stage failed (a block was not acked)")

    def _on_config_result(self, kind: str, res) -> None:
        if res is None:
            self.action_lbl.setText(f"{kind}: no response")
            return
        verdict = "ok" if res.ok else f"failed (result {res.result})"
        self.action_lbl.setText(f"{kind}: {verdict}")
        if kind == "reset" and res.ok:
            # The firmware reloaded defaults into the staging buffer; pull them.
            self.service.load_config()

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id != int(tlm.StreamId.I2C_SENSORS_RAW):
            return
        self._latest_pressure_raw = [None] * tlm.NUM_FEET
        for foot, raw in enumerate(record.feet):
            if foot >= tlm.NUM_FEET:
                break
            self._latest_pressure_raw[foot] = raw.pressure_raw
        self._latest_pressure_time = time.monotonic()
        self._apply_gates()

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class ServoTuningPage(BasePage):
    title = "Servo Monitor && DXL Tuning"
    subtitle = (
        "Live per-servo status and safe logical-parameter writes "
        "(staged, verified, read-back)."
    )

    # Logical parameters surfaced in the editor combo (label, DXL_PARAM_* id).
    # Limit pairs are handled by the dedicated limits group below.
    PARAMS = [
        ("Return delay time", api.DXL_PARAM_RETURN_DELAY_TIME),
        ("Temperature limit", api.DXL_PARAM_TEMPERATURE_LIMIT),
        ("Min voltage limit", api.DXL_PARAM_MIN_VOLTAGE_LIMIT),
        ("Max voltage limit", api.DXL_PARAM_MAX_VOLTAGE_LIMIT),
        ("Max torque", api.DXL_PARAM_MAX_TORQUE),
        ("Status return level", api.DXL_PARAM_STATUS_RETURN_LEVEL),
        ("PID P gain", api.DXL_PARAM_PID_P),
        ("PID I gain", api.DXL_PARAM_PID_I),
        ("PID D gain", api.DXL_PARAM_PID_D),
        ("Moving speed", api.DXL_PARAM_MOVING_SPEED),
        ("Torque limit", api.DXL_PARAM_TORQUE_LIMIT),
        ("Goal acceleration", api.DXL_PARAM_GOAL_ACCELERATION),
        ("Profile velocity", api.DXL_PARAM_PROFILE_VELOCITY),
        ("Profile acceleration", api.DXL_PARAM_PROFILE_ACCELERATION),
        ("Bus watchdog", api.DXL_PARAM_BUS_WATCHDOG),
    ]

    # EEPROM-region params need a torque-off write; warn before committing.
    EEPROM_PARAMS = {
        api.DXL_PARAM_RETURN_DELAY_TIME,
        api.DXL_PARAM_TEMPERATURE_LIMIT,
        api.DXL_PARAM_MIN_VOLTAGE_LIMIT,
        api.DXL_PARAM_MAX_VOLTAGE_LIMIT,
        api.DXL_PARAM_MAX_TORQUE,
        api.DXL_PARAM_STATUS_RETURN_LEVEL,
    }

    COLUMNS = [
        "ID",
        "Position",
        "Velocity",
        "Load",
        "Voltage",
        "Temp",
        "Error",
        "Torque",
    ]
    INT32_MIN = -(2**31)
    INT32_MAX = 2**31 - 1

    def build(self) -> None:
        self._rows: dict[int, int] = {}  # servo id -> table row
        # Servo-map for id -> (leg, joint) so servo_goals targets can be matched.
        from hexapod_protocol import config as cfg

        self._servo_map = cfg.ServoMap(cfg.default_robot_config())
        self._last_status: dict[int, object] = {}  # servo id -> ServoStatus
        self._goal_ticks: dict[int, int] = {}  # servo id -> commanded target tick

        self.content.addWidget(self._status_table())
        self.content.addWidget(self._detail_panel())
        self.content.addWidget(self._param_editor())
        self.content.addWidget(self._limits_editor())
        self.content.addWidget(self._expert_panel())

        self.banner = self.add_telemetry_banner(
            [
                (tlm.StreamId.SERVO_STATUS, "servo_status"),
                (tlm.StreamId.SERVO_GOALS, "servo_goals"),
            ],
            hint="Servo frames also require DXL power on and servos scanned.",
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.dxl_result.connect(self._on_dxl_result)

    # --- groups -----------------------------------------------------------

    def _status_table(self) -> QGroupBox:
        box = QGroupBox("Live servo status")
        lay = QVBoxLayout(box)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMinimumHeight(240)
        self.table.itemSelectionChanged.connect(self._on_row_selected)
        lay.addWidget(self.table)
        return box

    def _detail_panel(self) -> QWidget:
        from ui.widgets import ServoDetailPanel

        self.detail = ServoDetailPanel()
        return self.detail

    def _param_editor(self) -> QGroupBox:
        box = QGroupBox("Logical parameter editor")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        self.param_servo = QSpinBox()
        self.param_servo.setRange(1, 253)
        form.addRow("Servo ID", self.param_servo)

        self.param_combo = QComboBox()
        for label, pid in self.PARAMS:
            self.param_combo.addItem(label, pid)
        form.addRow("Parameter", self.param_combo)

        read = QPushButton("Read")
        read.clicked.connect(self._read_param)
        self.param_current = QLabel("--")
        self.param_current.setObjectName("MonoLabel")
        rrow = QHBoxLayout()
        rrow.addWidget(read)
        rrow.addWidget(self.param_current, 1)
        form.addRow("Current", self._wrap(rrow))

        self.param_value = QSpinBox()
        self.param_value.setRange(self.INT32_MIN, self.INT32_MAX)
        form.addRow("New value", self.param_value)

        write = QPushButton("Stage && write")
        write.setProperty("accent", True)
        write.clicked.connect(self._write_param)
        form.addRow("", write)

        self.param_result = QLabel("--")
        self.param_result.setObjectName("MonoLabel")
        form.addRow("Result", self.param_result)
        return box

    def _limits_editor(self) -> QGroupBox:
        box = QGroupBox("Servo position limits (legacy CW/CCW or MX2.0 min/max)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        self.limit_servo = QSpinBox()
        self.limit_servo.setRange(1, 253)
        form.addRow("Servo ID", self.limit_servo)

        self.limit_min = QSpinBox()
        self.limit_min.setRange(0, 4095)
        self.limit_max = QSpinBox()
        self.limit_max.setRange(0, 4095)
        self.limit_max.setValue(4095)
        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("min"))
        mrow.addWidget(self.limit_min)
        mrow.addWidget(QLabel("max"))
        mrow.addWidget(self.limit_max)
        mrow.addStretch(1)
        form.addRow("Limits (ticks)", self._wrap(mrow))

        write = QPushButton("Write limits")
        write.setProperty("accent", True)
        write.clicked.connect(self._write_limits)
        form.addRow("", write)

        self.limit_result = QLabel("--")
        self.limit_result.setObjectName("MonoLabel")
        form.addRow("Result", self.limit_result)
        return box

    def _expert_panel(self) -> QGroupBox:
        box = QGroupBox("Expert: raw register access")
        outer = QVBoxLayout(box)
        self.expert_gate = QCheckBox(
            "Enable raw register read/write (bypasses logical table — dangerous)"
        )
        self.expert_gate.toggled.connect(self._toggle_expert)
        outer.addWidget(self.expert_gate)

        self.expert_body = QWidget()
        form = QFormLayout(self.expert_body)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        self.reg_servo = QSpinBox()
        self.reg_servo.setRange(1, 253)
        form.addRow("Servo ID", self.reg_servo)
        self.reg_addr = QSpinBox()
        self.reg_addr.setRange(0, 0xFFFF)
        form.addRow("Address", self.reg_addr)
        self.reg_len = QComboBox()
        for n in (1, 2, 4):
            self.reg_len.addItem(f"{n} byte(s)", n)
        form.addRow("Length", self.reg_len)
        self.reg_value = QSpinBox()
        self.reg_value.setRange(self.INT32_MIN, self.INT32_MAX)
        form.addRow("Value", self.reg_value)
        self.reg_eeprom = QCheckBox("EEPROM region (torque-off write)")
        form.addRow("", self.reg_eeprom)

        read = QPushButton("Read register")
        read.clicked.connect(self._read_register)
        write = QPushButton("Write register")
        write.clicked.connect(self._write_register)
        brow = QHBoxLayout()
        brow.addWidget(read)
        brow.addWidget(write)
        brow.addStretch(1)
        form.addRow("", self._wrap(brow))

        self.reg_result = QLabel("--")
        self.reg_result.setObjectName("MonoLabel")
        form.addRow("Result", self.reg_result)

        self.expert_body.setVisible(False)
        outer.addWidget(self.expert_body)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- live status ------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.SERVO_STATUS), 20)
            self.service.subscribe(int(tlm.StreamId.SERVO_GOALS), 20)
        else:
            self.table.setRowCount(0)
            self._rows.clear()
            self._last_status.clear()
            self._goal_ticks.clear()
            self.detail.select_servo(None)

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.SERVO_GOALS):
            self._on_servo_goals(record)
            return
        if stream_id != int(tlm.StreamId.SERVO_STATUS):
            return
        for s in record.servos:
            self._last_status[s.id] = s
            row = self._rows.get(s.id)
            if row is None:
                row = self.table.rowCount()
                self.table.insertRow(row)
                self.table.setItem(row, 0, QTableWidgetItem(str(s.id)))
                self._rows[s.id] = row
            values = [
                str(s.id),
                str(s.position),
                str(s.velocity),
                str(s.load),
                f"{s.voltage_mv / 1000:.1f} V",
                f"{s.temperature_c} \u00b0C",
                f"0x{s.hardware_error:02X}",
                "ON" if s.torque_enabled else "off",
            ]
            for col, text in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(text))
            self.detail.update_status(s)

    def _on_servo_goals(self, record) -> None:
        """Map each commanded (leg, joint) goal back to a servo id + target tick."""
        for g in record.goals:
            servo = self._servo_map.servo_for(g.leg, g.joint)
            if servo is None:
                continue
            cmd = self._servo_map.angle_to_tick(
                g.leg, g.joint, g.angle_deg * 3.141592653589793 / 180.0
            )
            self._goal_ticks[servo.id] = cmd.tick
            if self.detail.servo_id == servo.id:
                self.detail.set_target_tick(cmd.tick)

    def _on_row_selected(self) -> None:
        items = self.table.selectedItems()
        if not items:
            return
        row = items[0].row()
        id_item = self.table.item(row, 0)
        if id_item is None:
            return
        servo_id = int(id_item.text())
        self.detail.select_servo(servo_id)
        self.detail.set_target_tick(self._goal_ticks.get(servo_id))
        status = self._last_status.get(servo_id)
        if status is not None:
            self.detail.update_status(status)
        # Torque limit isn't streamed; read it on demand when connected.
        if self.service.is_connected:
            self.service.dxl_get_param(servo_id, api.DXL_PARAM_TORQUE_LIMIT)

    # --- parameter editor -------------------------------------------------

    def _read_param(self) -> None:
        self.param_result.setText("reading…")
        self.service.dxl_get_param(
            self.param_servo.value(), self.param_combo.currentData()
        )

    def _write_param(self) -> None:
        pid = self.param_combo.currentData()
        eeprom = pid in self.EEPROM_PARAMS
        warn = (
            "This parameter lives in servo EEPROM and requires torque-off; "
            "the firmware will disable torque to write it.\n\n"
            if eeprom
            else ""
        )
        if not self._confirm(
            "Write servo parameter",
            f"{warn}Write {self.param_combo.currentText()} = "
            f"{self.param_value.value()} to servo {self.param_servo.value()}?",
        ):
            return
        self.param_result.setText("writing…")
        self.service.dxl_set_param(
            self.param_servo.value(), pid, self.param_value.value()
        )

    def _write_limits(self) -> None:
        lo, hi = self.limit_min.value(), self.limit_max.value()
        if lo >= hi:
            self.limit_result.setText("min must be < max")
            return
        if not self._confirm(
            "Write servo limits",
            f"Write position limits [{lo}, {hi}] to servo "
            f"{self.limit_servo.value()}? Torque will be disabled to write EEPROM.",
        ):
            return
        self.limit_result.setText("writing…")
        self.service.dxl_set_servo_limits(self.limit_servo.value(), lo, hi)

    # --- expert raw register ---------------------------------------------

    def _toggle_expert(self, on: bool) -> None:
        if on and not self._confirm(
            "Enable expert raw register access",
            "Raw register writes bypass the logical parameter table and can "
            "brick a servo. Continue?",
        ):
            self.expert_gate.setChecked(False)
            return
        self.expert_body.setVisible(on)

    def _read_register(self) -> None:
        self.reg_result.setText("reading…")
        self.service.dxl_read_register(
            self.reg_servo.value(), self.reg_addr.value(), self.reg_len.currentData()
        )

    def _write_register(self) -> None:
        if not self._confirm(
            "Write raw register",
            f"Write {self.reg_value.value()} to address {self.reg_addr.value()} "
            f"on servo {self.reg_servo.value()}?",
        ):
            return
        self.reg_result.setText("writing…")
        self.service.dxl_write_register(
            self.reg_servo.value(),
            self.reg_addr.value(),
            self.reg_len.currentData(),
            self.reg_value.value(),
            self.reg_eeprom.isChecked(),
        )

    # --- result routing ---------------------------------------------------

    def _on_dxl_result(self, kind: str, res) -> None:
        if res is None:
            target = {
                "get_param": self.param_result,
                "set_param": self.param_result,
                "set_limits": self.limit_result,
                "read_register": self.reg_result,
                "write_register": self.reg_result,
            }.get(kind)
            if target is not None:
                target.setText("failed (rejected or timed out)")
            return
        if kind == "get_param":
            pv = res.param()
            if pv is not None:
                self.param_current.setText(f"{pv.value}  (table {pv.table_kind})")
                self.param_value.setValue(pv.value)
                self.param_result.setText("read ok")
                # Feed the per-servo detail panel when this is a torque-limit read.
                if pv.param == api.DXL_PARAM_TORQUE_LIMIT:
                    self.detail.set_torque_limit(pv.value)
            else:
                self.param_result.setText(f"code {res.code}")
        elif kind == "set_param":
            sp = res.set_param()
            if sp is not None:
                self.param_result.setText(
                    f"wrote {sp.written}, read-back {sp.readback}, "
                    f"verified={sp.verified}"
                )
            else:
                self.param_result.setText(f"code {res.code}")
        elif kind == "set_limits":
            sl = res.servo_limits()
            if sl is not None:
                self.limit_result.setText(
                    f"table {sl.table_kind}: [{sl.min_tick}, {sl.max_tick}] "
                    f"verified={sl.verified}"
                )
            else:
                self.limit_result.setText(f"code {res.code}")
        elif kind == "read_register":
            rv = res.read_register()
            if rv is not None:
                self.reg_result.setText(
                    f"addr {rv.address} len {rv.length} = {rv.value}"
                )
            else:
                self.reg_result.setText(f"code {res.code}")
        elif kind == "write_register":
            wr = res.write_register()
            if wr is not None:
                self.reg_result.setText(
                    f"wrote {wr.written}, read-back {wr.readback}, "
                    f"verified={wr.verified}"
                )
            else:
                self.reg_result.setText(f"code {res.code}")

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class FootContactPage(BasePage):
    title = "Foot Contact && Leveling"
    subtitle = (
        "Per-leg touchdown state, live proximity/pressure, runtime threshold "
        "tuning, and contact/leveling enable with reasons."
    )

    COLUMNS = ["Leg", "State", "Conf", "Δpressure", "Proximity", "Pressure"]

    REASON_NAMES = {
        api.FEATURE_REASON_NONE: "NONE",
        api.FEATURE_REASON_HARDWARE_MISSING: "hardware missing",
        api.FEATURE_REASON_NOT_CALIBRATED: "not calibrated",
        api.FEATURE_REASON_UNSAFE_STATE: "unsafe state",
        api.FEATURE_REASON_STALE_DATA: "stale data",
        api.FEATURE_REASON_DEPENDENCY_OFF: "dependency off",
        api.FEATURE_REASON_NOT_IMPLEMENTED: "not implemented",
    }

    def build(self) -> None:
        self.content.addWidget(self._contact_table())
        self.content.addWidget(self._feature_controls())
        self.content.addWidget(self._threshold_editor())
        self.content.addWidget(self._calibrate_controls())

        self.banner = self.add_telemetry_banner(
            [(tlm.StreamId.CONTACT_STATE, "contact_state")]
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.sensor_feature_result.connect(self._on_feature_result)
        self.service.contact_threshold_result.connect(self._on_threshold_result)
        self.service.sensor_calibrate_result.connect(self._on_calibrate_result)

    # --- groups -----------------------------------------------------------

    def _contact_table(self) -> QGroupBox:
        box = QGroupBox("Per-leg contact state")
        lay = QVBoxLayout(box)
        self.table = QTableWidget(tlm.NUM_FEET, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for foot in range(tlm.NUM_FEET):
            self.table.setItem(foot, 0, QTableWidgetItem(f"Leg {foot}"))
            for col in range(1, len(self.COLUMNS)):
                self.table.setItem(foot, col, QTableWidgetItem("--"))
        self.table.setMinimumHeight(220)
        lay.addWidget(self.table)
        return box

    def _feature_controls(self) -> QGroupBox:
        box = QGroupBox("Contact && terrain leveling")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        crow = QHBoxLayout()
        self.contact_on = QPushButton("Enable contact")
        self.contact_on.setProperty("accent", True)
        self.contact_on.clicked.connect(lambda: self.service.set_contact(True))
        self.contact_off = QPushButton("Disable contact")
        self.contact_off.clicked.connect(lambda: self.service.set_contact(False))
        crow.addWidget(self.contact_on)
        crow.addWidget(self.contact_off)
        crow.addStretch(1)
        form.addRow("Foot contact", self._wrap(crow))
        self.contact_result = QLabel("--")
        self.contact_result.setObjectName("MonoLabel")
        form.addRow("", self.contact_result)

        lrow = QHBoxLayout()
        self.level_on = QPushButton("Enable leveling")
        self.level_on.setProperty("accent", True)
        self.level_on.clicked.connect(lambda: self.service.set_leveling(True))
        self.level_off = QPushButton("Disable leveling")
        self.level_off.clicked.connect(lambda: self.service.set_leveling(False))
        lrow.addWidget(self.level_on)
        lrow.addWidget(self.level_off)
        lrow.addStretch(1)
        form.addRow("Terrain leveling", self._wrap(lrow))
        self.level_result = QLabel("--")
        self.level_result.setObjectName("MonoLabel")
        form.addRow("", self.level_result)
        return box

    def _threshold_editor(self) -> QGroupBox:
        box = QGroupBox("Contact thresholds")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        self.thr_foot = QSpinBox()
        self.thr_foot.setRange(0, tlm.NUM_FEET - 1)
        form.addRow("Leg", self.thr_foot)

        self.thr_near = QSpinBox()
        self.thr_near.setRange(0, 0xFFFF)
        form.addRow("Near (proximity)", self.thr_near)
        self.thr_touch = QSpinBox()
        self.thr_touch.setRange(0, 0xFFFF)
        form.addRow("Touch (Δpressure)", self.thr_touch)
        self.thr_load = QSpinBox()
        self.thr_load.setRange(0, 0xFFFF)
        form.addRow("Load (Δpressure)", self.thr_load)

        write = QPushButton("Apply runtime thresholds")
        write.setProperty("accent", True)
        write.clicked.connect(self._write_thresholds)
        form.addRow("", write)

        self.thr_result = QLabel("--")
        self.thr_result.setObjectName("MonoLabel")
        form.addRow("Result", self.thr_result)
        return box

    def _calibrate_controls(self) -> QGroupBox:
        box = QGroupBox("Baseline calibration")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        self.cal_foot = QComboBox()
        self.cal_foot.addItem("All feet", api.SENSOR_CALIBRATE_ALL)
        for foot in range(tlm.NUM_FEET):
            self.cal_foot.addItem(f"Leg {foot}", foot)
        form.addRow("Target", self.cal_foot)

        cal = QPushButton("Re-zero pressure baseline")
        cal.clicked.connect(self._calibrate)
        form.addRow("", cal)

        self.cal_result = QLabel("Feet must be at rest before calibrating.")
        self.cal_result.setObjectName("MonoLabel")
        form.addRow("Result", self.cal_result)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- actions ----------------------------------------------------------

    def _write_thresholds(self) -> None:
        near = self.thr_near.value()
        touch = self.thr_touch.value()
        load = self.thr_load.value()
        if not near or not touch or not load:
            self.thr_result.setText(
                "blocked: near, touch, and load thresholds must be nonzero"
            )
            return
        if load < touch:
            self.thr_result.setText("blocked: load threshold must be at least touch")
            return
        self.thr_result.setText("writing…")
        self.service.set_contact_thresholds(
            self.thr_foot.value(),
            near,
            touch,
            load,
        )

    def _calibrate(self) -> None:
        target = self.cal_foot.currentData()
        name = self.cal_foot.currentText()
        if not self._confirm(
            "Calibrate contact baseline",
            f"Re-zero the pressure baseline for {name}? The selected foot/feet "
            "must be unloaded and at rest.",
        ):
            return
        self.cal_result.setText("calibrating…")
        self.service.calibrate_contact(target)

    # --- telemetry & results ---------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.CONTACT_STATE), 20)
            self.service.subscribe(int(tlm.StreamId.I2C_SENSORS_RAW), 10)
        else:
            for foot in range(tlm.NUM_FEET):
                for col in range(1, len(self.COLUMNS)):
                    self.table.item(foot, col).setText("--")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.CONTACT_STATE):
            for foot, fc in enumerate(record.feet):
                if foot >= tlm.NUM_FEET:
                    break
                self.table.item(foot, 1).setText(fc.state_name)
                self.table.item(foot, 2).setText(str(fc.confidence))
                self.table.item(foot, 3).setText(str(fc.pressure_delta))
        elif stream_id == int(tlm.StreamId.I2C_SENSORS_RAW):
            for foot, fr in enumerate(record.feet):
                if foot >= tlm.NUM_FEET:
                    break
                self.table.item(foot, 4).setText(str(fr.proximity))
                self.table.item(foot, 5).setText(str(fr.pressure_raw))

    def _format_feature(self, res) -> str:
        reason = self.REASON_NAMES.get(res.reason, str(res.reason))
        verdict = "ok" if res.ok else ("rejected" if res.rejected else "error")
        return (
            f"{verdict}: available={res.available} enabled={res.enabled} "
            f"reason={reason}"
        )

    def _on_feature_result(self, kind: str, res) -> None:
        label = self.contact_result if kind == "contact" else self.level_result
        label.setText(self._format_feature(res))

    def _on_threshold_result(self, res) -> None:
        if res.ok:
            self.thr_result.setText(
                f"leg {res.foot}: near={res.near} touch={res.touch} load={res.load}"
            )
        else:
            self.thr_result.setText(f"rejected (code {res.result})")

    def _on_calibrate_result(self, res) -> None:
        if res.ok:
            self.cal_result.setText(
                f"calibrated runtime (mask 0x{res.mask:02X}); copy and commit "
                "baselines in Robot Calibration to persist"
            )
        else:
            self.cal_result.setText(f"rejected (code {res.result})")

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class SensorDashboardPage(BasePage):
    title = "Sensor Dashboard && I2C"
    subtitle = (
        "Root I2C topology, TCA9548A mux channels, live Robotic Finger Sensor v2 "
        "values, poll-rate control, and baseline calibration."
    )

    TOPO_COLUMNS = ["Ch", "Scanned", "VCNL4040", "LPS25HB", "Devices", "State"]
    LIVE_COLUMNS = ["Ch", "Present", "State", "Conf", "Proximity", "Pressure"]
    CHANNEL_STATE_NAMES = {0: "missing", 1: "present", 2: "fault"}
    CHANNEL_STATE_LEVEL = {0: "idle", 1: "ok", 2: "warn"}

    def build(self) -> None:
        self.content.addWidget(self._topology_group())
        self.content.addWidget(self._live_group())
        self.content.addWidget(self._controls_group())

        self.banner = self.add_telemetry_banner(
            [
                (tlm.StreamId.I2C_SENSORS_RAW, "i2c_sensors_raw"),
                (tlm.StreamId.CONTACT_STATE, "contact_state"),
            ],
            hint="Sensor streams also require sensor polling to be enabled.",
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.i2c_topology.connect(self._apply_topology)
        self.service.sensor_status.connect(self._apply_sensor_status)
        self.service.sensor_rate_result.connect(self._on_rate_result)
        self.service.sensor_calibrate_result.connect(self._on_calibrate_result)

    # --- groups -----------------------------------------------------------

    def _topology_group(self) -> QGroupBox:
        box = QGroupBox("Root I2C topology")
        lay = QVBoxLayout(box)

        badges = QHBoxLayout()
        self.mux_badge = StatusBadge("TCA9548A mux")
        self.eeprom_badge = StatusBadge("24LC32 EEPROM")
        badges.addWidget(self.mux_badge)
        badges.addWidget(self.eeprom_badge)
        badges.addStretch(1)
        lay.addLayout(badges)

        self.topo_table = QTableWidget(0, len(self.TOPO_COLUMNS))
        self.topo_table.setHorizontalHeaderLabels(self.TOPO_COLUMNS)
        self.topo_table.verticalHeader().setVisible(False)
        self.topo_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.topo_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.topo_table.setMinimumHeight(200)
        lay.addWidget(self.topo_table)

        row = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh topology")
        self.refresh_btn.clicked.connect(
            lambda: self.service.refresh_i2c_topology(rescan=False)
        )
        self.rescan_btn = QPushButton("Rescan I2C bus")
        self.rescan_btn.clicked.connect(self._rescan)
        row.addWidget(self.refresh_btn)
        row.addWidget(self.rescan_btn)
        row.addStretch(1)
        self.topo_status = QLabel("Not connected.")
        self.topo_status.setObjectName("MonoLabel")
        row.addWidget(self.topo_status)
        lay.addLayout(row)
        return box

    def _live_group(self) -> QGroupBox:
        box = QGroupBox("Live foot sensors (channels 0-5)")
        lay = QVBoxLayout(box)
        self.live_table = QTableWidget(tlm.NUM_FEET, len(self.LIVE_COLUMNS))
        self.live_table.setHorizontalHeaderLabels(self.LIVE_COLUMNS)
        self.live_table.verticalHeader().setVisible(False)
        self.live_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.live_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for ch in range(tlm.NUM_FEET):
            self.live_table.setItem(ch, 0, QTableWidgetItem(f"Ch {ch}"))
            for col in range(1, len(self.LIVE_COLUMNS)):
                self.live_table.setItem(ch, col, QTableWidgetItem("--"))
        self.live_table.setMinimumHeight(220)
        lay.addWidget(self.live_table)

        row = QHBoxLayout()
        status_btn = QPushButton("Refresh status")
        status_btn.clicked.connect(self.service.refresh_sensor_status)
        row.addWidget(status_btn)
        row.addStretch(1)
        self.live_status = QLabel("--")
        self.live_status.setObjectName("MonoLabel")
        row.addWidget(self.live_status)
        lay.addLayout(row)
        return box

    def _controls_group(self) -> QGroupBox:
        box = QGroupBox("Polling && calibration")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        rate_row = QHBoxLayout()
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 200)
        self.rate_spin.setValue(50)
        self.rate_spin.setSuffix(" Hz")
        apply_btn = QPushButton("Apply rate")
        apply_btn.setProperty("accent", True)
        apply_btn.clicked.connect(
            lambda: self.service.set_sensor_rate(self.rate_spin.value())
        )
        rate_row.addWidget(self.rate_spin)
        rate_row.addWidget(apply_btn)
        rate_row.addStretch(1)
        form.addRow("Poll rate", self._wrap(rate_row))
        self.rate_result = QLabel("--")
        self.rate_result.setObjectName("MonoLabel")
        form.addRow("", self.rate_result)

        cal = QPushButton("Re-zero all foot baselines")
        cal.clicked.connect(self._calibrate)
        form.addRow("Calibration", cal)
        self.cal_result = QLabel("Feet must be at rest before calibrating.")
        self.cal_result.setObjectName("MonoLabel")
        form.addRow("", self.cal_result)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- actions ----------------------------------------------------------

    def _rescan(self) -> None:
        self.topo_status.setText("rescanning…")
        self.service.refresh_i2c_topology(rescan=True)

    def _calibrate(self) -> None:
        if not self._confirm(
            "Calibrate all baselines",
            "Re-zero the pressure baseline for every foot? All feet must be "
            "unloaded and at rest.",
        ):
            return
        self.cal_result.setText("calibrating…")
        self.service.calibrate_contact(api.SENSOR_CALIBRATE_ALL)

    # --- telemetry & results ---------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.I2C_SENSORS_RAW), 10)
            self.service.subscribe(int(tlm.StreamId.CONTACT_STATE), 10)
            self.service.refresh_i2c_topology(rescan=False)
            self.service.refresh_sensor_status()
        else:
            self.topo_table.setRowCount(0)
            self.mux_badge.set("disconnected", "idle")
            self.eeprom_badge.set("disconnected", "idle")
            self.topo_status.setText("Not connected.")
            for ch in range(tlm.NUM_FEET):
                for col in range(1, len(self.LIVE_COLUMNS)):
                    self.live_table.item(ch, col).setText("--")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.I2C_SENSORS_RAW):
            for ch, fr in enumerate(record.feet):
                if ch >= tlm.NUM_FEET:
                    break
                self.live_table.item(ch, 4).setText(str(fr.proximity))
                self.live_table.item(ch, 5).setText(str(fr.pressure_raw))
        elif stream_id == int(tlm.StreamId.CONTACT_STATE):
            for ch, fc in enumerate(record.feet):
                if ch >= tlm.NUM_FEET:
                    break
                self.live_table.item(ch, 2).setText(fc.state_name)
                self.live_table.item(ch, 3).setText(str(fc.confidence))

    def _apply_topology(self, res) -> None:
        if res is None:
            self.topo_status.setText("no topology response")
            return
        self.mux_badge.set(
            "present" if res.mux_present else "missing",
            "ok" if res.mux_present else "warn",
        )
        self.eeprom_badge.set(
            "present" if res.eeprom_present else "missing",
            "ok" if res.eeprom_present else "warn",
        )
        self.topo_table.setRowCount(len(res.channels))
        for ch, chan in enumerate(res.channels):
            state_name = self.CHANNEL_STATE_NAMES.get(chan.state, str(chan.state))
            values = [
                str(ch),
                "yes" if chan.scanned else "no",
                "yes" if chan.vcnl_present else "no",
                "yes" if chan.lps_present else "no",
                str(chan.device_count),
                state_name,
            ]
            for col, text in enumerate(values):
                self.topo_table.setItem(ch, col, QTableWidgetItem(text))
        present = sum(1 for c in res.channels if c.state == 1)
        fault = sum(1 for c in res.channels if c.state == 2)
        self.topo_status.setText(
            f"{len(res.channels)} channels — present={present} fault={fault}"
        )

    def _apply_sensor_status(self, res) -> None:
        if res is None:
            self.live_status.setText("no status response")
            return
        for ch, foot in enumerate(res.feet):
            if ch >= tlm.NUM_FEET:
                break
            present = bool(res.present_mask & (1 << ch))
            self.live_table.item(ch, 1).setText("yes" if present else "no")
            self.live_table.item(ch, 2).setText(
                tlm.CONTACT_STATE_NAMES.get(foot.state, str(foot.state))
            )
            self.live_table.item(ch, 3).setText(str(foot.confidence))
            self.live_table.item(ch, 4).setText(str(foot.proximity))
        polling = "on" if res.polling_enabled else "off"
        self.live_status.setText(
            f"present_mask=0x{res.present_mask:02X}  polling={polling}"
        )

    def _on_rate_result(self, res) -> None:
        if res.ok:
            self.rate_result.setText(f"poll rate set to {res.rate_hz} Hz")
        else:
            self.rate_result.setText(f"rejected (code {res.result})")

    def _on_calibrate_result(self, res) -> None:
        if res.ok:
            self.cal_result.setText(f"calibrated (mask 0x{res.mask:02X})")
        else:
            self.cal_result.setText(f"rejected (code {res.result})")

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class PassivePosePage(BasePage):

    title = "Passive Pose && Stream"
    subtitle = (
        "Torque-off passive streaming: hand-pose the robot and watch joint "
        "angles update live for calibration and URDF capture."
    )

    COLUMNS = ["Leg", "Joint", "Angle"]

    def build(self) -> None:
        self._rows: dict[tuple[int, int], int] = {}  # (leg, joint) -> row

        self.content.addWidget(self._mode_controls())
        self.content.addWidget(self._stream_controls())
        self.content.addWidget(self._joint_table())

        self.banner = self.add_telemetry_banner(
            [(tlm.StreamId.JOINT_STATE, "joint_state")],
            hint=(
                "Joint angles stream only while passive pose mode is active "
                "— use Enter passive above."
            ),
        )

        self._connected = False
        self._state = -1

        self.service.connected.connect(self._on_connected)
        self.service.state_changed.connect(self._on_state_changed)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.passive_result.connect(self._on_passive_result)
        self.service.passive_rate_result.connect(self._on_rate_result)

        self._apply_gates()

    # --- groups -----------------------------------------------------------

    def _mode_controls(self) -> QGroupBox:
        box = QGroupBox("Passive mode (all servo torque off)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        row = QHBoxLayout()
        self.enter_btn = QPushButton("Enter passive")
        self.enter_btn.setProperty("accent", True)
        self.enter_btn.clicked.connect(self._enter)
        self.exit_btn = QPushButton("Exit passive")
        self.exit_btn.clicked.connect(lambda: self.service.passive_exit())
        row.addWidget(self.enter_btn)
        row.addWidget(self.exit_btn)
        row.addStretch(1)
        form.addRow("", self._wrap(row))

        self.mode_badge = StatusBadge("Passive state")
        self.mode_badge.set("unknown", "idle")
        form.addRow("Status", self.mode_badge)
        self.mode_result = QLabel("--")
        self.mode_result.setObjectName("MonoLabel")
        form.addRow("Last result", self.mode_result)
        return box

    def _stream_controls(self) -> QGroupBox:
        box = QGroupBox("Stream control")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)

        rrow = QHBoxLayout()
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 200)
        self.rate_spin.setValue(50)
        self.rate_spin.setSuffix(" Hz")
        apply_rate = QPushButton("Apply rate")
        apply_rate.clicked.connect(
            lambda: self.service.passive_set_stream_rate(self.rate_spin.value())
        )
        rrow.addWidget(self.rate_spin)
        rrow.addWidget(apply_rate)
        rrow.addStretch(1)
        form.addRow("Stream rate", self._wrap(rrow))

        zero = QPushButton("Capture zero reference")
        zero.clicked.connect(lambda: self.service.passive_zero_reference())
        form.addRow("", zero)
        self.rate_result = QLabel("--")
        self.rate_result.setObjectName("MonoLabel")
        form.addRow("Result", self.rate_result)
        return box

    def _joint_table(self) -> QGroupBox:
        box = QGroupBox("Live joint angles (present position)")
        lay = QVBoxLayout(box)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMinimumHeight(280)
        lay.addWidget(self.table)
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- actions ----------------------------------------------------------

    def _enter(self) -> None:
        if not self._confirm(
            "Enter passive pose mode",
            "This disables torque on all servos so the robot can be moved by "
            "hand. The robot will go limp — support it before continuing.",
        ):
            return
        self.mode_result.setText("entering…")
        self.service.passive_enter()

    # --- telemetry & results ---------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.subscribe(int(tlm.StreamId.JOINT_STATE), 50)
        else:
            self._state = -1
            self.table.setRowCount(0)
            self._rows.clear()
            self.mode_badge.set("disconnected", "idle")
        self._apply_gates()

    def _on_state_changed(self, state: int) -> None:
        self._state = state
        if state == tlm.SafetyState.PASSIVE_POSE_STREAM:
            self.mode_badge.set("passive (torque off)", "warn")
        elif self._connected:
            self.mode_badge.set(
                tlm.SAFETY_STATE_NAMES.get(state, str(state)).lower(), "idle"
            )
        self._apply_gates()

    def _apply_gates(self) -> None:
        """Enter needs a Disarmed/Maintenance robot; Exit only in passive."""
        con = self._connected
        st = self._state
        enter_ok = con and st in (
            -1,
            tlm.SafetyState.DISARMED,
            tlm.SafetyState.MAC_MAINTENANCE,
            tlm.SafetyState.PASSIVE_POSE_STREAM,
        )
        self.enter_btn.setEnabled(enter_ok)
        self.enter_btn.setToolTip(
            ""
            if enter_ok
            else "Requires a Disarmed, Maintenance, or Passive robot."
        )
        exit_ok = con and st == tlm.SafetyState.PASSIVE_POSE_STREAM
        self.exit_btn.setEnabled(exit_ok)
        self.exit_btn.setToolTip(
            "" if exit_ok else "The robot is not in passive pose mode."
        )

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id != int(tlm.StreamId.JOINT_STATE):
            return
        for j in record.joints:
            key = (j.leg, j.joint)
            row = self._rows.get(key)
            if row is None:
                row = self.table.rowCount()
                self.table.insertRow(row)
                self._rows[key] = row
                self.table.setItem(row, 0, QTableWidgetItem(f"Leg {j.leg}"))
                self.table.setItem(row, 1, QTableWidgetItem(j.joint_name))
                self.table.setItem(row, 2, QTableWidgetItem("--"))
            self.table.item(row, 2).setText(f"{j.angle_deg:.2f}\u00b0")

    def _on_passive_result(self, kind: str, res) -> None:
        verdict = "ok" if res.ok else ("rejected" if res.rejected else "error")
        self.mode_result.setText(f"{kind}: {verdict} (state {res.state})")
        if kind == "enter" and res.ok:
            self.mode_badge.set("passive (torque off)", "warn")
        elif kind == "exit" and res.ok:
            self.mode_badge.set("inactive", "ok")

    def _on_rate_result(self, res) -> None:
        if res.ok:
            self.rate_result.setText(f"stream rate {res.rate_hz} Hz")
        else:
            self.rate_result.setText(f"rejected (code {res.result})")

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class RcTroubleshootingPage(BasePage):
    title = "RC Troubleshooting"
    subtitle = "Live ControllerBridge intent and decoded physical controls."
    MODE_NAMES = tlm.CONTROLLER_MODE_NAMES
    TRICK_NAMES = {
        0: "None",
        1: "Stand up",
        2: "Sit down",
        3: "Wave",
        4: "Crouch toggle",
        5: "Twirl",
        6: "Stretch",
        7: "Lean/look",
        8: "Dance loop",
    }
    RAW_INPUTS = [
        ("gimbal_lx", "Gimbal", "Left X"),
        ("gimbal_ly", "Gimbal", "Left Y"),
        ("gimbal_rx", "Gimbal", "Right X"),
        ("gimbal_ry", "Gimbal", "Right Y"),
        ("pot1", "Shape", "POT1 / speed"),
        ("pot2", "Shape", "POT2 / body height"),
        ("enc1", "Shape", "ENC1 / stride"),
        ("enc2", "Shape", "ENC2 / step height"),
        ("sw_a", "2-position", "SW_A / arm"),
        ("sw_b", "2-position", "SW_B / estop"),
        ("sw_c", "2-position", "SW_C / foot contact"),
        ("sw_d", "2-position", "SW_D / terrain leveling"),
        ("sw_g", "2-position", "SW_G / passive pose"),
        ("sw_h", "2-position", "SW_H / host authority"),
        ("sw_e", "3-position", "SW_E / control mode"),
        ("sw_f", "3-position", "SW_F / gait"),
        ("btn1", "Button", "BTN_1 / stand up"),
        ("btn2", "Button", "BTN_2 / sit down"),
        ("btn3", "Button", "BTN_3 / wave"),
        ("btn4", "Button", "BTN_4 / crouch"),
        ("nav1_u", "NAV1 trim", "Up / pitch +"),
        ("nav1_d", "NAV1 trim", "Down / pitch -"),
        ("nav1_l", "NAV1 trim", "Left / roll +"),
        ("nav1_r", "NAV1 trim", "Right / roll -"),
        ("nav1_c", "NAV1 trim", "Center / reset"),
        ("nav2_u", "NAV2 trick", "Up / twirl"),
        ("nav2_d", "NAV2 trick", "Down / stretch"),
        ("nav2_l", "NAV2 trick", "Left / lean/look"),
        ("nav2_r", "NAV2 trick", "Right / unbound"),
        ("nav2_c", "NAV2 trick", "Center / dance"),
    ]

    def build(self) -> None:
        self.content.addWidget(self._bridge_group())
        self.content.addWidget(self._command_group())
        self.content.addWidget(self._raw_inputs_group())

        self.banner = self.add_telemetry_banner(
            [(tlm.StreamId.CONTROLLER_STATE, "controller_state")]
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)

    # --- groups -----------------------------------------------------------

    def _grid_of_cards(self, box: QGroupBox, keys, per_row: int = 4) -> dict:
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        grid.setContentsMargins(6, 8, 6, 6)
        cards: dict[str, StatCard] = {}
        for i, (key, caption) in enumerate(keys):
            card = StatCard(caption)
            cards[key] = card
            grid.addWidget(card, i // per_row, i % per_row)
        for c in range(per_row):
            grid.setColumnStretch(c, 1)
        return cards

    def _bridge_group(self) -> QGroupBox:
        box = QGroupBox("ControllerBridge decoded intent")
        self.bridge = self._grid_of_cards(
            box,
            [
                ("valid", "Command valid"),
                ("failsafe", "Failsafe"),
                ("seen", "Controller seen"),
                ("arm", "SW_A arm request"),
                ("estop", "SW_B estop"),
                ("host", "SW_H host authority"),
                ("mode", "SW_E control mode"),
                ("gait", "Gait index"),
                ("trick", "Trick trigger"),
                ("contact", "SW_C foot contact"),
                ("leveling", "SW_D leveling"),
                ("passive", "SW_G passive pose"),
            ],
            per_row=4,
        )
        return box

    def _command_group(self) -> QGroupBox:
        box = QGroupBox("ControllerBridge command values")
        self.command = self._grid_of_cards(
            box,
            [
                ("twist", "Walk twist (vx, vy, wz)"),
                ("pose_xyz", "Body translation (x, y, z)"),
                ("pose_rpy", "Body rotation (roll, pitch, yaw)"),
                ("trim", "Persistent trim (roll, pitch)"),
                ("speed", "Speed"),
                ("height", "Body height"),
                ("stride", "Stride"),
                ("step", "Step height"),
            ],
            per_row=4,
        )
        return box

    def _raw_inputs_group(self) -> QGroupBox:
        box = QGroupBox("ControllerBridge raw ChannelPack inputs")
        lay = QVBoxLayout(box)
        self.raw_table = QTableWidget(len(self.RAW_INPUTS), 3)
        self.raw_table.setHorizontalHeaderLabels(["Group", "Physical input", "Value"])
        self.raw_table.verticalHeader().setVisible(False)
        self.raw_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.raw_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.raw_rows = {}
        for row, (key, group, name) in enumerate(self.RAW_INPUTS):
            self.raw_rows[key] = row
            self.raw_table.setItem(row, 0, QTableWidgetItem(group))
            self.raw_table.setItem(row, 1, QTableWidgetItem(name))
            self.raw_table.setItem(row, 2, QTableWidgetItem("--"))
        self.raw_table.setMinimumHeight(620)
        lay.addWidget(self.raw_table)
        return box

    # --- telemetry --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.CONTROLLER_STATE), 20)
        else:
            for card in (
                *self.bridge.values(),
                *self.command.values(),
            ):
                card.set("--", "idle")
            for row in self.raw_rows.values():
                self.raw_table.item(row, 2).setText("--")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.CONTROLLER_STATE):
            self._apply_controller(record)

    def _apply_controller(self, rec) -> None:
        self.bridge["valid"].set(
            "valid" if rec.valid else "invalid", "ok" if rec.valid else "error"
        )
        self.bridge["failsafe"].set(
            "FAILSAFE" if rec.failsafe else "clear",
            "error" if rec.failsafe else "ok",
        )
        self.bridge["seen"].set(
            "seen" if rec.ever_seen else "never", "ok" if rec.ever_seen else "warn"
        )
        self.bridge["arm"].set(
            "REQUESTED" if rec.arm_request else "off",
            "active" if rec.arm_request else "idle",
        )
        self.bridge["estop"].set(
            "ESTOP" if rec.estop else "clear", "error" if rec.estop else "ok"
        )
        self.bridge["host"].set(
            "requested" if rec.host_authority else "off",
            "active" if rec.host_authority else "idle",
        )
        self.bridge["mode"].set(self.MODE_NAMES.get(rec.mode, f"Unknown ({rec.mode})"), "info")
        self.bridge["gait"].set(str(rec.gait_index), "info")
        self.bridge["trick"].set(self.TRICK_NAMES.get(rec.trick, f"Unknown ({rec.trick})"), "info")
        self._set_request("contact", rec.feat_foot_contact)
        self._set_request("leveling", rec.feat_terrain_leveling)
        self._set_request("passive", rec.feat_passive_pose)

        self.command["twist"].set(
            f"{rec.twist_vx:+.2f}, {rec.twist_vy:+.2f}, {rec.twist_wz:+.2f}", "info"
        )
        self.command["pose_xyz"].set(
            f"{rec.pose_x_mm:+.0f}, {rec.pose_y_mm:+.0f}, {rec.pose_z_mm:+.0f} mm",
            "info",
        )
        self.command["pose_rpy"].set(
            f"{rec.pose_roll:+.3f}, {rec.pose_pitch:+.3f}, {rec.pose_yaw:+.3f} rad",
            "info",
        )
        self.command["trim"].set(
            f"{rec.trim_roll:+.3f}, {rec.trim_pitch:+.3f} rad", "info"
        )
        self.command["speed"].set(f"{rec.speed:.3f}", "info")
        self.command["height"].set(f"{rec.body_height:.3f}", "info")
        self.command["stride"].set(f"{rec.stride:.3f}", "info")
        self.command["step"].set(f"{rec.step_height:.3f}", "info")

        raw = rec.raw
        values = {
            "gimbal_lx": raw.gimbal[0],
            "gimbal_ly": raw.gimbal[1],
            "gimbal_rx": raw.gimbal[2],
            "gimbal_ry": raw.gimbal[3],
            "pot1": raw.pot[0],
            "pot2": raw.pot[1],
            "enc1": raw.encoder[0],
            "enc2": raw.encoder[1],
            "sw_a": raw.switches[0],
            "sw_b": raw.switches[1],
            "sw_c": raw.switches[2],
            "sw_d": raw.switches[3],
            "sw_g": raw.switches[4],
            "sw_h": raw.switches[5],
            "sw_e": raw.toggles[0],
            "sw_f": raw.toggles[1],
            "btn1": raw.buttons[0],
            "btn2": raw.buttons[1],
            "btn3": raw.buttons[2],
            "btn4": raw.buttons[3],
        }
        nav_names = ("u", "d", "l", "r", "c")
        for nav_index in range(2):
            for direction, active in zip(nav_names, raw.nav[nav_index]):
                values[f"nav{nav_index + 1}_{direction}"] = active
        for key, value in values.items():
            self.raw_table.item(self.raw_rows[key], 2).setText(
                self._raw_value(key, value)
            )

    def _set_request(self, key: str, enabled: bool) -> None:
        self.bridge[key].set(
            "requested" if enabled else "off", "active" if enabled else "idle"
        )

    @staticmethod
    def _raw_value(key: str, value) -> str:
        if key in ("sw_e", "sw_f"):
            return {0: "UP", 1: "CENTER", 2: "DOWN"}.get(value, f"INVALID ({value})")
        if isinstance(value, bool):
            return "ON" if value else "off"
        return str(value)


class DiagnosticsPage(BasePage):
    title = "Diagnostics"
    subtitle = (
        "Protocol stats, DXL/I2C errors, firmware timing, and a raw frame inspector."
    )
    fill = True

    # Streams the page needs; (stream_id, rate_hz).
    _STREAMS = (
        (int(tlm.StreamId.HEALTH), 5),
        (int(tlm.StreamId.API_STATS), 2),
        (int(tlm.StreamId.SERVO_STATUS), 10),
        (int(tlm.StreamId.CONTACT_STATE), 10),
    )

    def build(self) -> None:
        self._last: dict[int, object] = {}

        # --- protocol stats -------------------------------------------------
        proto = QGroupBox("Protocol")
        pg = QGridLayout(proto)
        pg.setHorizontalSpacing(12)
        pg.setVerticalSpacing(12)
        self.rx_card = StatCard("RX frames")
        self.tx_card = StatCard("TX frames")
        self.decode_card = StatCard("Decode errors")
        self.backlog_card = StatCard("TX backlog")
        self.dropped_card = StatCard("Dropped frames")
        # Firmware-side USB rx health from api_stats (hexapod_src-lv6): frames
        # the MCU received, frames it failed to decode (corruption), and frames
        # dropped for overflowing its reader buffer.
        self.fw_rx_card = StatCard("FW RX frames")
        self.fw_bad_card = StatCard("FW RX bad")
        self.fw_overflow_card = StatCard("FW RX overflow")
        for col, card in enumerate(
            (
                self.rx_card,
                self.tx_card,
                self.decode_card,
                self.backlog_card,
                self.dropped_card,
                self.fw_rx_card,
                self.fw_bad_card,
                self.fw_overflow_card,
            )
        ):
            pg.addWidget(card, col // 5, col % 5)
            pg.setColumnStretch(col % 5, 1)
        self.content.addWidget(proto)

        # --- firmware timing ------------------------------------------------
        timing = QGroupBox("Firmware timing")
        tg = QGridLayout(timing)
        tg.setHorizontalSpacing(12)
        tg.setVerticalSpacing(12)
        self.uptime_card = StatCard("Uptime")
        self.watchdog_card = StatCard("Watchdog missed")
        self.battery_card = StatCard("Battery")
        for col, card in enumerate(
            (self.uptime_card, self.watchdog_card, self.battery_card)
        ):
            tg.addWidget(card, 0, col)
            tg.setColumnStretch(col, 1)
        self.content.addWidget(timing)

        # --- DXL errors -----------------------------------------------------
        dxl = QGroupBox("DYNAMIXEL hardware errors")
        dv = QVBoxLayout(dxl)
        self.dxl_lbl = QLabel("Waiting for servo_status...")
        self.dxl_lbl.setObjectName("StatCaption")
        dv.addWidget(self.dxl_lbl)
        self.dxl_table = QTableWidget(0, 2)
        self.dxl_table.setHorizontalHeaderLabels(["Servo", "Error bits"])
        self.dxl_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents
        )
        self.dxl_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.dxl_table.verticalHeader().setVisible(False)
        self.dxl_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.dxl_table.setMaximumHeight(160)
        dv.addWidget(self.dxl_table)
        self.content.addWidget(dxl)

        # --- I2C / contact errors ------------------------------------------
        i2c = QGroupBox("I2C foot sensors")
        iv = QVBoxLayout(i2c)
        self.i2c_lbl = QLabel("Waiting for contact_state...")
        self.i2c_lbl.setObjectName("StatCaption")
        iv.addWidget(self.i2c_lbl)
        self.content.addWidget(i2c)

        # --- raw frame inspector -------------------------------------------
        raw = QGroupBox("Raw frame inspector")
        rv = QVBoxLayout(raw)
        ctl = QHBoxLayout()
        self.capture_chk = QCheckBox("Capture raw frames")
        self.capture_chk.toggled.connect(self._on_capture_toggled)
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(lambda: self.raw_feed.clear())
        ctl.addWidget(self.capture_chk)
        ctl.addStretch(1)
        ctl.addWidget(clear_btn)
        rv.addLayout(ctl)
        self.raw_feed = QPlainTextEdit()
        self.raw_feed.setReadOnly(True)
        self.raw_feed.setMaximumBlockCount(500)
        self.raw_feed.setObjectName("MonoLabel")
        rv.addWidget(self.raw_feed, 1)
        self.content.addWidget(raw, 1)

        # --- wiring ---------------------------------------------------------
        self._timer = QTimer(self)
        self._timer.setInterval(300)
        self._timer.timeout.connect(self._refresh)
        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)

    # --- reactions ---------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            for stream_id, rate in self._STREAMS:
                self.service.subscribe(stream_id, rate)
            if self.capture_chk.isChecked():
                self.service.set_raw_capture(True)
            self._timer.start()
        else:
            self._timer.stop()
            self._last.clear()
            self._reset_cards()

    def _on_capture_toggled(self, checked: bool) -> None:
        self.service.set_raw_capture(checked)
        if not checked:
            self.raw_feed.appendPlainText("-- capture stopped")

    def _on_telemetry(self, stream_id: int, record) -> None:
        self._last[stream_id] = record

    # --- periodic refresh --------------------------------------------------

    def _refresh(self) -> None:
        self._refresh_protocol()
        self._refresh_timing()
        self._refresh_dxl()
        self._refresh_i2c()
        self._refresh_raw()

    def _refresh_protocol(self) -> None:
        snap = self.service.diagnostics_snapshot()
        if snap is None:
            return
        self.rx_card.set(str(snap.rx_frames), "ok")
        self.tx_card.set(str(snap.tx_frames), "ok")
        self.decode_card.set(
            str(snap.decode_errors), "warn" if snap.decode_errors else "ok"
        )
        stats = self._last.get(int(tlm.StreamId.API_STATS))
        if stats is not None:
            self.backlog_card.set(
                str(stats.tx_backlog), "warn" if stats.tx_backlog else "ok"
            )
            total = sum(stats.dropped_per_stream)
            self.dropped_card.set(str(total), "warn" if total else "ok")
            self.fw_rx_card.set(str(stats.rx_frames), "ok")
            self.fw_bad_card.set(str(stats.rx_bad), "warn" if stats.rx_bad else "ok")
            self.fw_overflow_card.set(
                str(stats.rx_overflow), "warn" if stats.rx_overflow else "ok"
            )

    def _refresh_timing(self) -> None:
        health = self._last.get(int(tlm.StreamId.HEALTH))
        if health is None:
            return
        self.uptime_card.set(f"{health.uptime_ms / 1000:.1f} s", "ok")
        self.watchdog_card.set(
            str(health.watchdog_missed),
            "warn" if health.watchdog_missed else "ok",
        )
        self.battery_card.set(f"{health.battery_mv / 1000:.2f} V", "ok")

    def _refresh_dxl(self) -> None:
        record = self._last.get(int(tlm.StreamId.SERVO_STATUS))
        if record is None:
            return
        faulted = [s for s in record.servos if s.hardware_error]
        self.dxl_table.setRowCount(len(faulted))
        for row, servo in enumerate(faulted):
            bits = ", ".join(tlm.decode_hw_error(servo.hardware_error)) or (
                f"0x{servo.hardware_error:02X}"
            )
            self.dxl_table.setItem(row, 0, QTableWidgetItem(f"#{servo.id}"))
            self.dxl_table.setItem(row, 1, QTableWidgetItem(bits))
        n = len(record.servos)
        if faulted:
            self.dxl_lbl.setText(f"{len(faulted)}/{n} servos reporting errors")
        else:
            self.dxl_lbl.setText(f"{n} servos OK — no hardware errors")

    def _refresh_i2c(self) -> None:
        record = self._last.get(int(tlm.StreamId.CONTACT_STATE))
        if record is None:
            return
        stale = sum(1 for f in record.feet if f.state == 5)  # STALE
        fault = sum(1 for f in record.feet if f.state == 6)  # FAULT
        states = " ".join(f.state_name[:1] for f in record.feet)
        summary = (
            f"{len(record.feet)} feet  [{states}]  " f"stale={stale}  fault={fault}"
        )
        self.i2c_lbl.setText(summary)

    def _refresh_raw(self) -> None:
        for rec in self.service.drain_raw_frames():
            if rec.ok:
                if rec.msg_type == int(MsgType.TELEMETRY):
                    sid = rec.msg_id - api.MSG_TELEMETRY_BASE
                    name = tlm.STREAM_NAMES.get(sid, f"stream:{sid}")
                else:
                    name = f"type:{rec.msg_type} id:0x{rec.msg_id:02X}"
                line = (
                    f"[{name}] seq={rec.seq} len={rec.length} "
                    f"plen={rec.payload_len}  {rec.head_hex}"
                )
            else:
                line = f"[decode-error] len={rec.length}  {rec.head_hex}"
            self.raw_feed.appendPlainText(line)

    # --- helpers -----------------------------------------------------------

    def _reset_cards(self) -> None:
        for card in (
            self.rx_card,
            self.tx_card,
            self.decode_card,
            self.backlog_card,
            self.dropped_card,
            self.uptime_card,
            self.watchdog_card,
            self.battery_card,
        ):
            card.set("--", "idle")
        self.dxl_table.setRowCount(0)
        self.dxl_lbl.setText("Waiting for servo_status...")
        self.i2c_lbl.setText("Waiting for contact_state...")


class ModelViewerPage(BasePage):
    title = "Model Viewer"
    subtitle = "Live animated hexapod pose from joint telemetry (servo-map fallback)."
    fill = True

    # Prefer joint_state; fall back to servo_status only when joint_state stalls.
    _JOINT_STATE_TIMEOUT_MS = 750

    def build(self) -> None:
        from models import HexapodPoseModel
        from hexapod_protocol import config as cfg
        from ui.widgets import HexapodView

        self._model = HexapodPoseModel(cfg.default_robot_config())
        self._last_joint_state_ms = 0

        self.source_badge = StatusBadge("Pose source")
        self.source_badge.set("waiting", "idle")
        self.content.addWidget(self.source_badge)

        self.view = HexapodView()
        self.view.set_legs(self._model.legs())
        self.content.addWidget(self.view, 1)

        self.banner = self.add_telemetry_banner(
            [
                (tlm.StreamId.JOINT_STATE, "joint_state"),
                (tlm.StreamId.SERVO_STATUS, "servo_status"),
            ],
            hint="servo_status frames require DXL power on.",
            require_all=False,
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)

    def _now_ms(self) -> int:
        import time

        return int(time.monotonic() * 1000)

    def _on_connected(self, connected: bool) -> None:
        if connected:
            # Ask the firmware for joint poses; servo_status is the fallback feed.
            self.service.subscribe(int(tlm.StreamId.JOINT_STATE), 50)
            self.service.subscribe(int(tlm.StreamId.SERVO_STATUS), 20)
        else:
            self.source_badge.set("disconnected", "idle")

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id == int(tlm.StreamId.JOINT_STATE):
            self._last_joint_state_ms = self._now_ms()
            self._model.update_from_joint_state(record)
            self.source_badge.set("joint_state", "ok")
            self.view.set_legs(self._model.legs())
        elif stream_id == int(tlm.StreamId.SERVO_STATUS):
            # Only drive the pose from ticks when joint_state is absent/stale.
            stale = (
                self._now_ms() - self._last_joint_state_ms
                > self._JOINT_STATE_TIMEOUT_MS
            )
            if stale:
                self._model.update_from_servo_status(record)
                self.source_badge.set("servo_status (fallback)", "warn")
                self.view.set_legs(self._model.legs())


# Curve colours cycled across selected signals (Dracula-ish palette).
_PLOT_COLORS = [
    "#8be9fd",
    "#ff79c6",
    "#50fa7b",
    "#ffb86c",
    "#bd93f9",
    "#f1fa8c",
    "#ff5555",
    "#6272a4",
    "#8affff",
    "#ff78e0",
]


class PlotWorkbenchPage(BasePage):
    title = "Plot Workbench"
    subtitle = "Plot live telemetry or a recorded session across servo, leg, control, RC, and sensor streams."
    fill = True

    # Live redraw cadence and default rolling window (samples per signal).
    _REDRAW_MS = 100
    _DEFAULT_WINDOW = 600
    _LIVE_RATE_HZ = 20

    def build(self) -> None:
        from collections import deque
        import time as _time

        import pyqtgraph as pg

        from data.plot_signals import (
            build_signal_registry,
            registry_by_key,
            streams_for,
        )
        from data.event_log import EventLog

        self._pg = pg
        self._deque = deque
        self._monotonic = _time.monotonic
        self._registry = build_signal_registry()
        self._by_key = registry_by_key(self._registry)
        self._streams_for = streams_for

        self._selected: list = []
        self._live_buf: dict[str, object] = {}
        self._curves: dict[str, object] = {}
        self._subscribed: set[int] = set()
        self._replay = None
        self._t0 = self._monotonic()
        self._mode = "live"  # or "replay"

        # Event markers overlaying the timeline (nxi.2).
        self._event_log = EventLog()
        self._event_lines: list = []

        # --- top controls -------------------------------------------------
        controls = QHBoxLayout()
        controls.setSpacing(10)
        self._live_radio = QRadioButton("Live")
        self._live_radio.setChecked(True)
        self._replay_radio = QRadioButton("Replay")
        mode_group = QButtonGroup(self)
        mode_group.addButton(self._live_radio)
        mode_group.addButton(self._replay_radio)
        self._live_radio.toggled.connect(self._on_mode_toggled)
        controls.addWidget(QLabel("Source:"))
        controls.addWidget(self._live_radio)
        controls.addWidget(self._replay_radio)

        self._load_btn = QPushButton("Load session\u2026")
        self._load_btn.clicked.connect(self._choose_session)
        self._load_btn.setEnabled(False)
        controls.addWidget(self._load_btn)

        self._export_csv_btn = QPushButton("Export selected CSV")
        self._export_csv_btn.clicked.connect(self._choose_csv_export)
        self._export_csv_btn.setEnabled(False)
        controls.addWidget(self._export_csv_btn)

        self._export_report_btn = QPushButton("Export session report")
        self._export_report_btn.clicked.connect(self._choose_report_export)
        self._export_report_btn.setEnabled(False)
        controls.addWidget(self._export_report_btn)

        controls.addSpacing(16)
        controls.addWidget(QLabel("Window:"))
        self._window_spin = QSpinBox()
        self._window_spin.setRange(60, 5000)
        self._window_spin.setValue(self._DEFAULT_WINDOW)
        self._window_spin.setSuffix(" samples")
        self._window_spin.valueChanged.connect(self._on_window_changed)
        controls.addWidget(self._window_spin)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._clear)
        controls.addWidget(self._clear_btn)
        controls.addStretch(1)
        self._status = QLabel("No signals selected.")
        self._status.setObjectName("PageSubtitle")
        controls.addWidget(self._status)
        self.content.addLayout(controls)

        # --- event annotation row (nxi.2) ---------------------------------
        events = QHBoxLayout()
        events.setSpacing(10)
        self._show_events = QCheckBox("Show event markers")
        self._show_events.setChecked(True)
        self._show_events.toggled.connect(self._on_show_events_toggled)
        events.addWidget(self._show_events)
        events.addWidget(QLabel("Note:"))
        self._note_edit = QLineEdit()
        self._note_edit.setPlaceholderText(
            "Add an operator note at the current time\u2026"
        )
        self._note_edit.returnPressed.connect(self._add_note)
        events.addWidget(self._note_edit, 1)
        self._note_btn = QPushButton("Add note")
        self._note_btn.clicked.connect(self._add_note)
        events.addWidget(self._note_btn)
        self.content.addLayout(events)

        # --- splitter: picker | plot --------------------------------------
        splitter = QSplitter(Qt.Horizontal)

        picker = QWidget()
        pv = QVBoxLayout(picker)
        pv.setContentsMargins(0, 0, 0, 0)
        pv.setSpacing(8)
        self._filter = QLineEdit()
        self._filter.setPlaceholderText("Filter signals\u2026")
        self._filter.textChanged.connect(self._apply_filter)
        pv.addWidget(self._filter)
        self._tree = QTreeWidget()
        self._tree.setHeaderLabels(["Signal", "Unit"])
        self._tree.setColumnWidth(0, 210)
        self._tree.itemChanged.connect(self._on_item_changed)
        pv.addWidget(self._tree, 1)
        self._populate_tree()
        splitter.addWidget(picker)

        pg.setConfigOptions(antialias=True)
        self._plot = pg.PlotWidget(background="#1e1f29")
        self._plot.showGrid(x=True, y=True, alpha=0.2)
        self._plot.getAxis("left").setPen("#6272a4")
        self._plot.getAxis("bottom").setPen("#6272a4")
        self._plot.setLabel("bottom", "time", units="s")
        self._plot.setLabel("left", "value")
        self._legend = self._plot.addLegend()
        splitter.addWidget(self._plot)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([300, 720])
        self.content.addWidget(splitter, 1)

        self.service.telemetry.connect(self._on_telemetry)
        self.service.connected.connect(self._on_connected)
        self.service.event.connect(self._on_event)

        self._redraw_timer = QTimer(self)
        self._redraw_timer.setInterval(self._REDRAW_MS)
        self._redraw_timer.timeout.connect(self._redraw_live)
        self._redraw_timer.start()

    # --- tree / selection --------------------------------------------------

    def _populate_tree(self) -> None:
        groups: dict[str, QTreeWidgetItem] = {}
        for sig in self._registry:
            parent = groups.get(sig.group)
            if parent is None:
                parent = QTreeWidgetItem(self._tree, [sig.group, ""])
                parent.setFlags(Qt.ItemIsEnabled)
                parent.setExpanded(False)
                groups[sig.group] = parent
            item = QTreeWidgetItem(parent, [sig.label, sig.unit])
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsUserCheckable)
            item.setCheckState(0, Qt.Unchecked)
            item.setData(0, Qt.UserRole, sig.key)

    def _iter_signal_items(self):
        for i in range(self._tree.topLevelItemCount()):
            parent = self._tree.topLevelItem(i)
            for j in range(parent.childCount()):
                yield parent.child(j)

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for parent_idx in range(self._tree.topLevelItemCount()):
            parent = self._tree.topLevelItem(parent_idx)
            any_visible = False
            for j in range(parent.childCount()):
                child = parent.child(j)
                key = child.data(0, Qt.UserRole)
                sig = self._by_key.get(key)
                match = (
                    not needle
                    or needle in sig.label.lower()
                    or needle in sig.group.lower()
                    or needle in sig.key.lower()
                )
                child.setHidden(not match)
                any_visible = any_visible or match
            parent.setHidden(not any_visible)
            if needle and any_visible:
                parent.setExpanded(True)

    def _on_item_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if item.data(0, Qt.UserRole) is None:
            return
        self._rebuild_selection()

    def select_signals(self, keys) -> None:
        """Programmatically check a set of signal keys (used by tests)."""
        wanted = set(keys)
        self._tree.blockSignals(True)
        for child in self._iter_signal_items():
            key = child.data(0, Qt.UserRole)
            child.setCheckState(0, Qt.Checked if key in wanted else Qt.Unchecked)
        self._tree.blockSignals(False)
        self._rebuild_selection()

    def selected_keys(self) -> list[str]:
        return [s.key for s in self._selected]

    def _rebuild_selection(self) -> None:
        selected = []
        for child in self._iter_signal_items():
            if child.checkState(0) == Qt.Checked:
                key = child.data(0, Qt.UserRole)
                sig = self._by_key.get(key)
                if sig is not None:
                    selected.append(sig)
        self._selected = selected

        # Drop curves/buffers for signals no longer selected.
        keep = {s.key for s in selected}
        for key in list(self._curves):
            if key not in keep:
                self._plot.removeItem(self._curves.pop(key))
                self._live_buf.pop(key, None)
        # Create curves/buffers for new signals.
        window = self._window_spin.value()
        for idx, sig in enumerate(selected):
            if sig.key not in self._curves:
                color = _PLOT_COLORS[idx % len(_PLOT_COLORS)]
                self._curves[sig.key] = self._plot.plot(
                    [], [], pen=self._pg.mkPen(color, width=2), name=sig.label
                )
                self._live_buf[sig.key] = self._deque(maxlen=window)

        if not selected:
            self._status.setText("No signals selected.")
        else:
            self._status.setText(f"{len(selected)} signal(s) \u00b7 {self._mode} mode")
        if self._mode == "live":
            self._ensure_subscriptions()
        else:
            self._replot_replay()
        self._update_export_gates()

    # --- mode --------------------------------------------------------------

    def _on_mode_toggled(self, live_checked: bool) -> None:
        self._mode = "live" if live_checked else "replay"
        self._load_btn.setEnabled(self._mode == "replay")
        self._clear()
        self._update_export_gates()
        if self._mode == "live":
            self._ensure_subscriptions()
            self._status.setText(f"{len(self._selected)} signal(s) \u00b7 live mode")
        else:
            self._status.setText("Replay mode \u2014 load a recorded session.")

    def set_mode(self, mode: str) -> None:
        """Programmatic mode switch (used by tests)."""
        if mode == "replay":
            self._replay_radio.setChecked(True)
        else:
            self._live_radio.setChecked(True)

    def _on_window_changed(self, value: int) -> None:
        for key, buf in list(self._live_buf.items()):
            self._live_buf[key] = self._deque(buf, maxlen=value)

    def _clear(self) -> None:
        for key in self._live_buf:
            self._live_buf[key] = self._deque(maxlen=self._window_spin.value())
        for curve in self._curves.values():
            curve.setData([], [])
        self._clear_event_markers()
        self._t0 = self._monotonic()

    # --- event markers (nxi.2) --------------------------------------------

    def _add_note(self) -> None:
        text = self._note_edit.text().strip()
        if not text:
            return
        self.service.mark_note(text)
        self._note_edit.clear()

    def _on_event(self, kind: str, detail: str) -> None:
        """Handle a live event from the service: annotate the live timeline."""
        if self._mode != "live":
            return
        t_s = self._monotonic() - self._t0
        marker = self._event_log.add(kind, detail, t_s)
        self._draw_event_marker(marker)

    def _on_show_events_toggled(self, checked: bool) -> None:
        if checked:
            for marker in self._event_log.markers():
                self._draw_event_marker(marker)
        else:
            for line in self._event_lines:
                self._plot.removeItem(line)
            self._event_lines.clear()

    def _draw_event_marker(self, marker) -> None:
        if not self._show_events.isChecked():
            return
        pg = self._pg
        line = pg.InfiniteLine(
            pos=marker.t_s,
            angle=90,
            movable=False,
            pen=pg.mkPen(marker.color, width=1, style=Qt.DashLine),
            label=marker.label,
            labelOpts={"position": 0.92, "color": marker.color, "rotateAxis": (1, 0)},
        )
        self._plot.addItem(line)
        self._event_lines.append(line)

    def _clear_event_markers(self) -> None:
        for line in self._event_lines:
            self._plot.removeItem(line)
        self._event_lines.clear()
        self._event_log.clear()

    def event_marker_count(self) -> int:
        """Number of active event markers (used by tests)."""
        return len(self._event_log)

    # --- live --------------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._subscribed.clear()
        if connected and self._mode == "live":
            self._ensure_subscriptions()

    def _ensure_subscriptions(self) -> None:
        for sid in self._streams_for(self._selected):
            if sid not in self._subscribed:
                self.service.subscribe(sid, self._LIVE_RATE_HZ)
                self._subscribed.add(sid)

    def _on_telemetry(self, stream_id: int, record) -> None:
        if self._mode != "live" or not self._selected:
            return
        x = self._monotonic() - self._t0
        for sig in self._selected:
            if sig.stream_id != stream_id:
                continue
            y = sig.extract(record)
            if y is None:
                continue
            self._live_buf[sig.key].append((x, y))

    def _redraw_live(self) -> None:
        if self._mode != "live":
            return
        for key, buf in self._live_buf.items():
            curve = self._curves.get(key)
            if curve is None or not buf:
                continue
            xs = [p[0] for p in buf]
            ys = [p[1] for p in buf]
            curve.setData(xs, ys)

    # --- replay ------------------------------------------------------------

    def _choose_session(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open recorded session")
        if path:
            self.load_session(path)

    def _choose_csv_export(self) -> None:
        if self._replay is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export selected session signals",
            str(self._replay.dir / "selected_signals.csv"),
            "CSV (*.csv)",
        )
        if path:
            self.export_selected_csv(path)

    def _choose_report_export(self) -> None:
        if self._replay is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export session report",
            str(self._replay.dir / "session_summary.txt"),
            "Text (*.txt)",
        )
        if path:
            self.export_session_report(path)

    def export_selected_csv(self, output_path) -> bool:
        """Export selected replay signals to ``output_path`` for UI/tests."""
        if self._replay is None or not self._selected:
            self._status.setText("Load a replay and select at least one signal first.")
            return False
        from data.session_export import export_selected_csv

        try:
            rows = export_selected_csv(self._replay, self._selected, output_path)
        except (OSError, ValueError) as exc:
            self._status.setText(f"CSV export failed: {exc}")
            return False
        self._status.setText(f"Exported {rows} CSV row(s) to {output_path}")
        return True

    def export_session_report(self, output_path) -> bool:
        """Export the loaded replay's human-readable report for UI/tests."""
        if self._replay is None:
            self._status.setText("Load a replay before exporting a report.")
            return False
        from data.session_export import write_session_summary

        try:
            summary = write_session_summary(self._replay, output_path)
        except (OSError, ValueError) as exc:
            self._status.setText(f"Report export failed: {exc}")
            return False
        self._status.setText(
            f"Exported report for {summary['session_id']} to {output_path}"
        )
        return True

    def load_session(self, session_dir) -> None:
        """Load a recorded session directory and plot the selected signals."""
        from data.session_replay import SessionReplay

        try:
            self._replay = SessionReplay(session_dir)
        except Exception as exc:  # noqa: BLE001 - surfaced to the operator
            self._status.setText(f"Load failed: {exc}")
            self._replay = None
            return
        self.set_mode("replay")
        self._replot_replay()
        self._update_export_gates()

    def _update_export_gates(self) -> None:
        replay_loaded = self._mode == "replay" and self._replay is not None
        self._export_csv_btn.setEnabled(replay_loaded and bool(self._selected))
        self._export_report_btn.setEnabled(replay_loaded)
        self._export_csv_btn.setToolTip(
            "" if replay_loaded and self._selected else "Load a replay and select signals."
        )
        self._export_report_btn.setToolTip(
            "" if replay_loaded else "Load a replay session first."
        )

    def _replot_replay(self) -> None:
        if self._mode != "replay" or self._replay is None:
            return
        from data.plot_signals import extract_series
        from data.event_log import EventLog

        frames = list(self._replay.iter_decoded_frames())
        t0_ns = min(
            (getattr(f, "host_time_ns", 0) for f in frames),
            default=0,
        )
        series = extract_series(frames, self._selected, t0_ns=t0_ns)
        points = 0
        for sig in self._selected:
            curve = self._curves.get(sig.key)
            if curve is None:
                continue
            xs, ys = series.get(sig.key, ([], []))
            curve.setData(xs, ys)
            points += len(xs)

        # Overlay recorded events on the same host timeline.
        self._clear_event_markers()
        self._event_log = EventLog.from_session_events(
            self._replay.iter_events(), t0_ns=t0_ns
        )
        for marker in self._event_log.markers():
            self._draw_event_marker(marker)

        meta = self._replay.meta
        self._status.setText(
            f"Replay: {meta.get('session_id', '?')} \u2014 "
            f"{len(self._selected)} signal(s), {points} points, "
            f"{len(self._event_log)} event(s)"
        )


class UrdfViewerPage(BasePage):
    title = "URDF Viewer"
    subtitle = (
        "Render the actual HexNav URDF meshes in 3D and animate them from "
        "live or replay joint telemetry."
    )
    fill = True

    # Prefer joint_state; fall back to servo_status only when joint_state stalls.
    _JOINT_STATE_TIMEOUT_MS = 750
    _REPLAY_FPS = 20

    def build(self) -> None:
        import time as _time

        from data.urdf_fk import UrdfForwardKinematics, joint_state_to_urdf_angles
        from data.urdf_model import find_hexnav_description, load_hexnav

        self._monotonic = _time.monotonic
        self._map_angles = joint_state_to_urdf_angles

        self._angles: dict = {}
        self._last_joint_state_ms = 0
        self._mode = "live"  # or "replay"
        self._replay_frames: list = []  # list[dict] of joint-name -> rad
        self._replay_idx = 0
        self._ok = False
        self._view = None

        self.source_badge = StatusBadge("Pose source")
        self.source_badge.set("waiting", "idle")
        self.content.addWidget(self.source_badge)

        if find_hexnav_description() is None:
            self._show_unavailable(
                "HexNav_description not found. Set HEXAPOD_ROBOT_DESCRIPTION "
                "to the package path to enable the URDF viewer."
            )
            return

        try:
            self._model = load_hexnav()
            self._fk = UrdfForwardKinematics(self._model)

            from data.mesh_loader import build_link_meshes
            from ui.widgets.urdf_gl_view import UrdfGLView

            meshes = build_link_meshes(self._model)
            self._view = UrdfGLView(self._model, self._fk, meshes)
        except Exception as exc:  # noqa: BLE001 - degrade instead of crashing
            self._show_unavailable(f"3D view unavailable: {exc}")
            return

        self._ok = True

        self.content.addLayout(self._controls())
        self.content.addWidget(self._view, 1)

        self.banner = self.add_telemetry_banner(
            [
                (tlm.StreamId.JOINT_STATE, "joint_state"),
                (tlm.StreamId.SERVO_STATUS, "servo_status"),
            ],
            hint="servo_status frames require DXL power on.",
            require_all=False,
        )

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)

        self._replay_timer = QTimer(self)
        self._replay_timer.setInterval(int(1000 / self._REPLAY_FPS))
        self._replay_timer.timeout.connect(self._advance_replay)

    # --- construction helpers ---------------------------------------------

    def _show_unavailable(self, message: str) -> None:
        self._fk = None
        self.source_badge.set("URDF unavailable", "error")
        self.content.addWidget(QLabel(message))
        self.content.addStretch(1)

    def _controls(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)
        self._live_radio = QRadioButton("Live")
        self._live_radio.setChecked(True)
        self._replay_radio = QRadioButton("Replay")
        group = QButtonGroup(self)
        group.addButton(self._live_radio)
        group.addButton(self._replay_radio)
        self._live_radio.toggled.connect(self._on_mode_toggled)
        row.addWidget(QLabel("Source:"))
        row.addWidget(self._live_radio)
        row.addWidget(self._replay_radio)

        self._load_btn = QPushButton("Load session\u2026")
        self._load_btn.clicked.connect(self._choose_session)
        self._load_btn.setEnabled(False)
        row.addWidget(self._load_btn)

        self._play_btn = QPushButton("Play")
        self._play_btn.setEnabled(False)
        self._play_btn.clicked.connect(self._toggle_play)
        row.addWidget(self._play_btn)

        self._scrub = QSlider(Qt.Horizontal)
        self._scrub.setEnabled(False)
        self._scrub.setRange(0, 0)
        self._scrub.valueChanged.connect(self._on_scrub)
        row.addWidget(self._scrub, 1)

        self._frame_lbl = QLabel("\u2014")
        self._frame_lbl.setObjectName("PageSubtitle")
        row.addWidget(self._frame_lbl)
        return row

    def _now_ms(self) -> int:
        return int(self._monotonic() * 1000)

    # --- mode --------------------------------------------------------------

    def _on_mode_toggled(self, live_checked: bool) -> None:
        self._mode = "live" if live_checked else "replay"
        self._load_btn.setEnabled(self._mode == "replay")
        replay_ready = self._mode == "replay" and bool(self._replay_frames)
        self._play_btn.setEnabled(replay_ready)
        self._scrub.setEnabled(replay_ready)
        # The stale-telemetry banner only applies to the live feed.
        self.banner.set_active(self._mode == "live")
        if self._mode == "live":
            self._stop_play()
            self._ensure_subscriptions()

    def set_mode(self, mode: str) -> None:
        """Programmatic mode switch (used by tests)."""
        (self._replay_radio if mode == "replay" else self._live_radio).setChecked(True)

    # --- live feed ---------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected and self._ok:
            self._ensure_subscriptions()
        elif not connected:
            self.source_badge.set("disconnected", "idle")

    def _ensure_subscriptions(self) -> None:
        self.service.subscribe(int(tlm.StreamId.JOINT_STATE), 50)
        self.service.subscribe(int(tlm.StreamId.SERVO_STATUS), 20)

    def _on_telemetry(self, stream_id: int, record) -> None:
        if not self._ok or self._mode != "live":
            return
        if stream_id == int(tlm.StreamId.JOINT_STATE):
            self._last_joint_state_ms = self._now_ms()
            self._apply_angles(self._map_angles(record))
            self.source_badge.set("joint_state", "ok")
        elif stream_id == int(tlm.StreamId.SERVO_STATUS):
            stale = (
                self._now_ms() - self._last_joint_state_ms
                > self._JOINT_STATE_TIMEOUT_MS
            )
            if stale:
                self._apply_angles(self._servo_status_angles(record))
                self.source_badge.set("servo_status (fallback)", "warn")

    def _servo_status_angles(self, record) -> dict:
        from hexapod_protocol import config as cfg

        joints = cfg.servo_status_to_joint_angles(cfg.default_robot_config(), record)
        return self._map_angles(tlm.JointStateTelemetry(joints=joints))

    # --- pose application / redraw ----------------------------------------

    def _render(self) -> None:
        if self._ok and self._view is not None:
            self._view.set_angles(self._angles)

    def _apply_angles(self, angles: dict) -> None:
        """Merge new joint angles into the pose and repose the model."""
        if angles:
            self._angles.update(angles)
            self._render()

    def apply_angles(self, angles: dict) -> None:
        """Public test hook: set angles and repose synchronously."""
        self._apply_angles(angles)

    def current_angles(self) -> dict:
        return dict(self._angles)

    def mesh_count(self) -> int:
        """Number of rendered link meshes (used by tests)."""
        if not self._ok or self._view is None:
            return 0
        return self._view.mesh_count()

    # --- replay ------------------------------------------------------------

    def _choose_session(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open recorded session")
        if path:
            self.load_session(path)

    def load_session(self, session_dir) -> None:
        """Load a recorded session and expose its joint_state frames for scrub."""
        from data.session_replay import SessionReplay

        try:
            replay = SessionReplay(session_dir)
        except Exception as exc:  # noqa: BLE001 - surfaced to the operator
            self.source_badge.set(f"load failed: {exc}", "error")
            return
        frames: list = []
        js_stream = int(tlm.StreamId.JOINT_STATE)
        for df in replay.iter_decoded_frames():
            if df.record is None:
                continue
            if df.msg_id - api.MSG_TELEMETRY_BASE != js_stream:
                continue
            frames.append(self._map_angles(df.record))
        self._replay_frames = frames
        self._replay_idx = 0
        self.set_mode("replay")
        has_frames = bool(frames)
        self._scrub.setEnabled(has_frames)
        self._play_btn.setEnabled(has_frames)
        self._scrub.setRange(0, max(0, len(frames) - 1))
        if has_frames:
            self.set_replay_index(0)
            self.source_badge.set(f"replay ({len(frames)} frames)", "active")
        else:
            self.source_badge.set("replay: no joint_state frames", "warn")

    def replay_frame_count(self) -> int:
        return len(self._replay_frames)

    def set_replay_index(self, index: int) -> None:
        if not self._replay_frames:
            return
        index = max(0, min(index, len(self._replay_frames) - 1))
        self._replay_idx = index
        self._angles = dict(self._replay_frames[index])
        self._render()
        self._frame_lbl.setText(f"{index + 1}/{len(self._replay_frames)}")
        if self._scrub.value() != index:
            self._scrub.blockSignals(True)
            self._scrub.setValue(index)
            self._scrub.blockSignals(False)

    def _on_scrub(self, value: int) -> None:
        self.set_replay_index(value)

    def _toggle_play(self) -> None:
        if self._replay_timer.isActive():
            self._stop_play()
        elif self._replay_frames:
            self._play_btn.setText("Pause")
            self._replay_timer.start()

    def _stop_play(self) -> None:
        self._replay_timer.stop()
        self._play_btn.setText("Play")

    def _advance_replay(self) -> None:
        if not self._replay_frames:
            self._stop_play()
            return
        nxt = self._replay_idx + 1
        if nxt >= len(self._replay_frames):
            self._stop_play()
            return
        self.set_replay_index(nxt)
