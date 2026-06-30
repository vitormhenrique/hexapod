"""Companion pages. Each page is a QWidget wired to the ConnectionService."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import api
from hexapod_protocol import telemetry as tlm

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
            btn.clicked.connect(lambda _=False, g=gait: self.service.set_gait(g))
            grid.addWidget(btn, i // 3, i % 3)
            self._gait_buttons[gait] = btn
        for c in range(3):
            grid.setColumnStretch(c, 1)
        return box

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
    subtitle = "Raw telemetry feed and protocol stats."
    fill = True

    def build(self) -> None:
        self.feed = QPlainTextEdit()
        self.feed.setReadOnly(True)
        self.feed.setMaximumBlockCount(500)
        self.feed.setObjectName("MonoLabel")
        self.content.addWidget(self.feed, 1)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.event.connect(self._on_event)

    def _on_telemetry(self, stream_id: int, record) -> None:
        name = tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id))
        self.feed.appendPlainText(f"{name}: {record}")

    def _on_event(self, kind: str, detail: str) -> None:
        self.feed.appendPlainText(f"-- [{kind}] {detail}")


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
