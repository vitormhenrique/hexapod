"""Companion pages. Each page is a QWidget wired to the ConnectionService."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import telemetry as tlm

from services import ConnectionService
from theme import DRACULA
from ui.widgets import StatCard, StatusBadge


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
        }
        for i, b in enumerate(self.badges.values()):
            grid.addWidget(b, i // 3, i % 3)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        self.content.addWidget(grid_box)

        self.hint = QLabel(
            "Subscribe on the Mode & Safety page or connect to see live data."
        )
        self.hint.setStyleSheet(f"color: {DRACULA.comment};")
        self.content.addWidget(self.hint)

        self.service.status_received.connect(self._on_status)
        self.service.telemetry.connect(self._on_telemetry)

    def _on_status(self, st) -> None:
        self.badges["state"].set(
            tlm.SAFETY_STATE_NAMES.get(st.state, str(st.state)),
            "ok" if st.state in (2, 4, 5) else "warn",
        )
        self.badges["battery"].set(
            f"{st.battery_mv} mV", "ok" if st.battery_mv > 10000 else "warn"
        )
        self.badges["uptime"].set(f"{st.uptime_ms // 1000} s", "info")

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


class ModeSafetyPage(BasePage):
    title = "Mode & Safety Center"
    subtitle = "Telemetry subscriptions and safety controls."

    STREAMS = [
        ("health", 5),
        ("control_state", 20),
        ("servo_status", 20),
        ("contact_state", 20),
        ("i2c_sensors_raw", 10),
        ("rc_input", 20),
        ("api_stats", 2),
    ]

    def build(self) -> None:
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
        self.content.addWidget(box)

        safety = QGroupBox("Safety")
        slay = QHBoxLayout(safety)
        estop = QPushButton("\u23fb  EMERGENCY STOP")
        estop.setObjectName("EmergencyStop")
        estop.clicked.connect(self.service.emergency_stop)
        slay.addWidget(estop)
        slay.addStretch(1)
        self.content.addWidget(safety)

    def _toggle(self, name: str, rate: int, checked: bool) -> None:
        sid = int(tlm.stream_id_from_name(name))
        if checked:
            self.service.subscribe(sid, rate)
        else:
            self.service.unsubscribe(sid)


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
