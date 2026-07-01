"""Companion pages. Each page is a QWidget wired to the ConnectionService."""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
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
from ui.widgets import FeatureToggleCard, StatCard, StatusBadge


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


class ConnectPage(BasePage):
    title = "Connect & Setup"
    subtitle = "Discover the USB port, handshake, and verify firmware capabilities."

    def build(self) -> None:
        box = QGroupBox("Serial connection")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(14)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(320)
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
        self.refresh_ports()

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        for p in self.service.available_ports():
            self.port_combo.addItem(f"{p.device}  —  {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found", None)

    def _toggle(self) -> None:
        port = self.port_combo.currentData()
        if port:
            self.service.connect_to(port)

    def _on_connected(self, connected: bool) -> None:
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
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
            "Subscribe on the Mode & Safety page or connect to see live data."
        )
        self.hint.setStyleSheet(f"color: {DRACULA.comment};")
        self.content.addWidget(self.hint)

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
            "ok" if st.state in (2, 4, 5) else "warn",
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
                "ok" if record.state in (2, 4, 5) else "warn",
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
    title = "Mode & Safety Center"
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
        (api.FEATURE_PASSIVE_POSE, "Passive Pose"),
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
        self.content.addWidget(self._safety_controls())
        self.content.addWidget(self._lock_controls())
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
        self.service.feature_list.connect(self._on_feature_list)
        self.service.feature_result.connect(lambda _r: self.service.refresh_features())
        self.service.maint_result.connect(self._on_maint_result)
        self.service.control_result.connect(self._on_control_result)

    # --- groups -----------------------------------------------------------

    def _safety_controls(self) -> QGroupBox:
        box = QGroupBox("Safety controls")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        arm = QPushButton("Arm")
        arm.setToolTip("Release the host disarm latch (RC arm switch still required).")
        arm.clicked.connect(lambda: self.service.set_arming(True))

        disarm = QPushButton("Disarm")
        disarm.clicked.connect(
            lambda: self._confirm(
                "Disarm robot",
                "Latch a host force-disarm? This removes motion authority.",
                lambda: self.service.set_arming(False),
            )
        )

        clear = QPushButton("Clear Fault")
        clear.clicked.connect(
            lambda: self._confirm(
                "Clear fault",
                "Release the host E-stop latch and request a fault clear?",
                self.service.clear_fault,
            )
        )

        set_disarmed = QPushButton("Set mode: Disarmed")
        set_disarmed.clicked.connect(lambda: self.service.set_mode(2))

        force_estop = QPushButton("Set mode: E-stop")
        force_estop.clicked.connect(
            lambda: self._confirm(
                "Force E-stop mode",
                "Drive the safety machine into ESTOP?",
                lambda: self.service.set_mode(12),
            )
        )

        for i, btn in enumerate((arm, disarm, clear, set_disarmed, force_estop)):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 3, i % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)

        self.action_lbl = QLabel("No command sent yet.")
        self.action_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        grid.addWidget(self.action_lbl, 2, 0, 1, 3)
        return box

    def _lock_controls(self) -> QGroupBox:
        box = QGroupBox("Maintenance & passive pose")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        enter_m = QPushButton("Enter Maintenance")
        enter_m.clicked.connect(self.service.enter_maintenance)
        exit_m = QPushButton("Exit Maintenance")
        exit_m.clicked.connect(self.service.exit_maintenance)

        enter_p = QPushButton("Enter Passive Pose")
        enter_p.clicked.connect(
            lambda: self._confirm(
                "Enter passive pose",
                "Disable all servo torque and stream present positions?",
                self.service.passive_enter,
            )
        )
        exit_p = QPushButton("Exit Passive Pose")
        exit_p.clicked.connect(self.service.passive_exit)

        for i, btn in enumerate((enter_m, exit_m, enter_p, exit_p)):
            btn.setCursor(Qt.PointingHandCursor)
            grid.addWidget(btn, i // 2, i % 2)
        for c in range(2):
            grid.setColumnStretch(c, 1)

        self.lock_lbl = QLabel("Maintenance lock: none")
        self.lock_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        grid.addWidget(self.lock_lbl, 2, 0, 1, 2)
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

    def _on_maint_result(self, res) -> None:
        if res.token:
            self.lock_lbl.setText(f"Maintenance lock: held (token {res.token})")
        elif res.ok:
            self.lock_lbl.setText("Maintenance lock: none")
        else:
            self.lock_lbl.setText(f"Maintenance lock: rejected (result {res.result})")

    def _on_control_result(self, kind: str, res) -> None:
        state = tlm.SAFETY_STATE_NAMES.get(res.state, str(res.state))
        verdict = "ok" if res.ok else f"rejected ({res.result})"
        self.action_lbl.setText(f"{kind}: {verdict} — state {state}")

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.refresh_features()
        else:
            self.lock_lbl.setText("Maintenance lock: none")
            self.action_lbl.setText("No command sent yet.")

    def _toggle(self, name: str, rate: int, checked: bool) -> None:
        sid = int(tlm.stream_id_from_name(name))
        if checked:
            self.service.subscribe(sid, rate)
        else:
            self.service.unsubscribe(sid)


class GaitLabPage(BasePage):
    title = "Gait Lab"
    subtitle = "Select a gait, tune gait parameters, and command safe body motion."

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
        self.content.addWidget(self._arming_banner())
        self.content.addWidget(self._gait_select())
        self.content.addWidget(self._gait_params())
        self.content.addWidget(self._body_twist())

        # Always-available stop + emergency stop row.
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

        self.service.connected.connect(self._on_connected)
        self.service.motion_result.connect(self._on_motion_result)
        self.service.telemetry.connect(self._on_telemetry)

    # --- groups -----------------------------------------------------------

    def _arming_banner(self) -> QGroupBox:
        box = QGroupBox("Command authority")
        lay = QVBoxLayout(box)
        self.authority_lbl = QLabel(
            "Connect and arm the robot (RC arm switch required) before commanding "
            "motion. The firmware rejects motion commands unless the motion gate "
            "is OPEN."
        )
        self.authority_lbl.setWordWrap(True)
        self.authority_lbl.setStyleSheet(f"color: {DRACULA.comment};")
        self.gate_badge = StatusBadge("motion gate")
        self.gate_badge.set("unknown", "idle")
        lay.addWidget(self.gate_badge)
        lay.addWidget(self.authority_lbl)
        return box

    def _gait_select(self) -> QGroupBox:
        box = QGroupBox("Gait selection")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        self._gait_buttons = {}
        for i, (label, gait) in enumerate(self.GAITS):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.clicked.connect(
                lambda _=False, g=gait, name=label: self._select_gait(g, name)
            )
            grid.addWidget(btn, i // 3, i % 3)
            self._gait_buttons[gait] = btn
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
            ("stride_len", "Stride length (mm)", 0, 160, 60),
            ("step_height", "Step height (mm)", 0, 80, 30),
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
        return box

    def _body_twist(self) -> QGroupBox:
        box = QGroupBox("Body twist (normalised -100..100 %)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        self._twist_spins = {}
        for attr, label in (
            ("vx", "Forward"),
            ("vy", "Left"),
            ("wz", "Yaw (CCW)"),
        ):
            spin = QSpinBox()
            spin.setRange(-100, 100)
            spin.setValue(0)
            self._twist_spins[attr] = spin
            form.addRow(label, spin)
        row = QHBoxLayout()
        send = QPushButton("Send twist")
        send.clicked.connect(self._send_twist)
        zero = QPushButton("Zero")
        zero.clicked.connect(self._zero_twist)
        row.addWidget(send)
        row.addWidget(zero)
        row.addStretch(1)
        form.addRow("", self._wrap(row))
        return box

    def _wrap(self, layout) -> QWidget:
        w = QWidget()
        w.setLayout(layout)
        return w

    # --- actions ----------------------------------------------------------

    def _send_gait_params(self) -> None:
        self.service.set_gait_params(
            self._param_spins["body_height"].value(),
            self._param_spins["stride_len"].value(),
            self._param_spins["step_height"].value(),
            self._param_spins["duty"].value(),
            self._param_spins["speed"].value(),
        )

    def _send_twist(self) -> None:
        self.service.set_body_twist(
            self._twist_spins["vx"].value() / 100.0,
            self._twist_spins["vy"].value() / 100.0,
            self._twist_spins["wz"].value() / 100.0,
        )

    def _zero_twist(self) -> None:
        for spin in self._twist_spins.values():
            spin.setValue(0)
        self.service.set_body_twist(0.0, 0.0, 0.0)

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.CONTROL_STATE), 10)
        else:
            self.gate_badge.set("unknown", "idle")

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
            for gait, btn in self._gait_buttons.items():
                btn.setChecked(False)


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

        self._num_legs = cfg.NUM_LEGS
        self._leg_rows: dict[int, int] = {}  # leg index -> live table row

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

        self.service.connected.connect(self._on_connected)
        self.service.maint_result.connect(self._on_maint_result)
        self.service.leg_target_result.connect(self._on_leg_result)
        self.service.joint_target_result.connect(self._on_joint_result)
        self.service.telemetry.connect(self._on_telemetry)

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
        leave = QPushButton("Exit Maintenance")
        leave.clicked.connect(self.service.exit_maintenance)
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
        for leg in range(self._num_legs):
            self.leg_combo.addItem(f"Leg {leg}", leg)
        form.addRow("Active leg", self.leg_combo)
        return box

    def _foot_target(self) -> QGroupBox:
        box = QGroupBox("Foot target (mm, body frame)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        self._foot_spins = {}
        for attr, label in (("x", "X (forward)"), ("y", "Y (left)"), ("z", "Z (up)")):
            spin = QSpinBox()
            spin.setRange(-400, 400)
            spin.setValue(0)
            spin.setSuffix(" mm")
            self._foot_spins[attr] = spin
            form.addRow(label, spin)
        send = QPushButton("Send foot target")
        send.setProperty("accent", True)
        send.clicked.connect(self._send_foot_target)
        form.addRow("", send)
        self.foot_result = QLabel("--")
        self.foot_result.setObjectName("MonoLabel")
        self.foot_result.setWordWrap(True)
        form.addRow("Result", self.foot_result)
        return box

    def _joint_target(self) -> QGroupBox:
        box = QGroupBox("Joint target (URDF-zero relative)")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        self.joint_combo = QComboBox()
        for label, jid in self.JOINTS:
            self.joint_combo.addItem(label, jid)
        form.addRow("Joint", self.joint_combo)
        self.joint_angle = QSpinBox()
        self.joint_angle.setRange(-180, 180)
        self.joint_angle.setValue(0)
        self.joint_angle.setSuffix(" \u00b0")
        form.addRow("Angle", self.joint_angle)
        send = QPushButton("Send joint target")
        send.setProperty("accent", True)
        send.clicked.connect(self._send_joint_target)
        form.addRow("", send)
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
        # Firmware expects centidegrees; the spinbox is whole degrees.
        self.service.set_joint_target(
            self.leg_combo.currentData(),
            self.joint_combo.currentData(),
            self.joint_angle.value() * 100,
        )

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.subscribe(int(tlm.StreamId.LEG_STATE), 10)
        else:
            self.lock_lbl.setText("Maintenance lock: none")
            self.foot_result.setText("--")
            self.joint_result.setText("--")
            self.leg_table.setRowCount(0)
            self._leg_rows.clear()

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
    title = "Servo Map & Config"
    subtitle = (
        "View, edit, validate, diff, stage, and commit the EEPROM-backed servo "
        "map. Export/import the full robot config as JSON."
    )

    COLUMNS = ["ID", "Leg", "Joint", "Sign", "Trim", "Min tick", "Max tick"]

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

        self.content.addWidget(self._summary_box())
        self.content.addWidget(self._servo_table())
        self.content.addWidget(self._actions_box())
        self.content.addWidget(self._diff_box())

        self.service.connected.connect(self._on_connected)
        self.service.config_loaded.connect(self._on_config_loaded)
        self.service.config_summary.connect(self._on_config_summary)
        self.service.config_staged.connect(self._on_config_staged)
        self.service.config_result.connect(self._on_config_result)

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
        reset = QPushButton("Reset to defaults")
        reset.clicked.connect(self._reset_defaults)
        row.addWidget(load)
        row.addWidget(reset)
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

    def _actions_box(self) -> QGroupBox:
        box = QGroupBox("Config actions")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        diff = QPushButton("Diff vs loaded")
        diff.clicked.connect(self._show_diff)
        stage = QPushButton("Stage to robot")
        stage.setProperty("accent", True)
        stage.clicked.connect(self._stage)
        validate = QPushButton("Validate staged")
        validate.clicked.connect(self.service.validate_config)
        commit = QPushButton("Commit to EEPROM")
        commit.clicked.connect(self._commit)
        export = QPushButton("Export JSON\u2026")
        export.clicked.connect(self._export)
        importb = QPushButton("Import JSON\u2026")
        importb.clicked.connect(self._import)
        buttons = [diff, stage, validate, commit, export, importb]
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
        return config, None

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
            "rejects a commit while walking or if the staged config is invalid.",
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
        self.action_lbl.setText(f"imported {path} (stage to apply)")
        self.diff_text.setPlainText("Imported config loaded as the new base.")

    def _config_from_dict(self, data: dict):
        cfg = self._cfg
        return cfg.RobotConfig(
            schema_version=data.get("schema_version", cfg.SCHEMA_VERSION),
            robot_name=data.get("robot_name", ""),
            links=cfg.LinkLengths(**data["links"]),
            geometry=cfg.BodyGeometry(**data["geometry"]),
            legs=[cfg.LegGeometry(**d) for d in data["legs"]],
            servos=[cfg.ServoConfig(**d) for d in data["servos"]],
            gait=cfg.GaitDefaults(**data["gait"]),
            feet=[cfg.FootSensorCal(**d) for d in data["feet"]],
            feature_defaults=data.get("feature_defaults", 0),
        )

    # --- reactions --------------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        if connected:
            self.service.load_config()
        else:
            self.table.setRowCount(0)
            self._loaded = None
            self.schema_lbl.setText("--")
            self.persist_lbl.setText("--")
            self.action_lbl.setText("Load the config to begin.")

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

    def _confirm(self, title: str, text: str) -> bool:
        return (
            QMessageBox.question(
                self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            == QMessageBox.Yes
        )


class ServoTuningPage(BasePage):
    title = "Servo Monitor & DXL Tuning"
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

        write = QPushButton("Stage & write")
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
    title = "Foot Contact & Leveling"
    subtitle = (
        "Per-leg touchdown state, live proximity/pressure, threshold tuning, "
        "and contact/leveling enable with reasons."
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
        box = QGroupBox("Contact & terrain leveling")
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

        write = QPushButton("Stage & write thresholds")
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
        self.thr_result.setText("writing…")
        self.service.set_contact_thresholds(
            self.thr_foot.value(),
            self.thr_near.value(),
            self.thr_touch.value(),
            self.thr_load.value(),
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


class SensorDashboardPage(BasePage):
    title = "Sensor Dashboard & I2C"
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
        box = QGroupBox("Polling & calibration")
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

    title = "Passive Pose & Stream"
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

        self.service.connected.connect(self._on_connected)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.passive_result.connect(self._on_passive_result)
        self.service.passive_rate_result.connect(self._on_rate_result)

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
        if connected:
            self.service.subscribe(int(tlm.StreamId.JOINT_STATE), 50)
        else:
            self.table.setRowCount(0)
            self._rows.clear()
            self.mode_badge.set("disconnected", "idle")

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


class DiagnosticsPage(BasePage):
    title = "Diagnostics"
    subtitle = "Protocol stats, DXL/I2C errors, firmware timing, and a raw frame inspector."
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
        for col, card in enumerate(
            (
                self.rx_card,
                self.tx_card,
                self.decode_card,
                self.backlog_card,
                self.dropped_card,
            )
        ):
            pg.addWidget(card, 0, col)
            pg.setColumnStretch(col, 1)
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
        self.dxl_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.Stretch
        )
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
            f"{len(record.feet)} feet  [{states}]  "
            f"stale={stale}  fault={fault}"
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
    "#8be9fd", "#ff79c6", "#50fa7b", "#ffb86c", "#bd93f9",
    "#f1fa8c", "#ff5555", "#6272a4", "#8affff", "#ff78e0",
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
        self._note_edit.setPlaceholderText("Add an operator note at the current time\u2026")
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
            child.setCheckState(
                0, Qt.Checked if key in wanted else Qt.Unchecked
            )
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
            self._status.setText(
                f"{len(selected)} signal(s) \u00b7 {self._mode} mode"
            )
        if self._mode == "live":
            self._ensure_subscriptions()
        else:
            self._replot_replay()

    # --- mode --------------------------------------------------------------

    def _on_mode_toggled(self, live_checked: bool) -> None:
        self._mode = "live" if live_checked else "replay"
        self._load_btn.setEnabled(self._mode == "replay")
        self._clear()
        if self._mode == "live":
            self._ensure_subscriptions()
            self._status.setText(
                f"{len(self._selected)} signal(s) \u00b7 live mode"
            )
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
