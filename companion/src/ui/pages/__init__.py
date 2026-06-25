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
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import telemetry as tlm

from ...services import ConnectionService
from ...theme import DRACULA
from ..widgets import StatusBadge


class BasePage(QWidget):
    """Common page chrome: title + subtitle + content area."""

    title = "Page"
    subtitle = ""

    def __init__(self, service: ConnectionService, parent=None) -> None:
        super().__init__(parent)
        self.service = service
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(14)
        header = QVBoxLayout()
        header.setSpacing(2)
        t = QLabel(self.title)
        t.setObjectName("PageTitle")
        header.addWidget(t)
        if self.subtitle:
            s = QLabel(self.subtitle)
            s.setObjectName("PageSubtitle")
            header.addWidget(s)
        root.addLayout(header)
        self.content = QVBoxLayout()
        self.content.setSpacing(14)
        root.addLayout(self.content)
        root.addStretch(1)
        self.build()

    def build(self) -> None:  # override
        ...


class ConnectPage(BasePage):
    title = "Connect & Setup"
    subtitle = "Discover the USB port, handshake, and verify firmware capabilities."

    def build(self) -> None:
        box = QGroupBox("Serial connection")
        form = QFormLayout(box)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(280)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row = QHBoxLayout()
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
        btnrow.addWidget(self.connect_btn)
        btnrow.addWidget(self.disconnect_btn)
        btnrow.addStretch(1)
        form.addRow("", self._wrap(btnrow))
        self.content.addWidget(box)

        info = QGroupBox("Firmware")
        ilay = QFormLayout(info)
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
        self.content.addWidget(info)

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
        self.badges = {
            "state": StatusBadge("SAFETY STATE"),
            "fault": StatusBadge("FAULT"),
            "source": StatusBadge("CMD SOURCE"),
            "gate": StatusBadge("MOTION GATE"),
            "battery": StatusBadge("BATTERY"),
            "uptime": StatusBadge("UPTIME"),
        }
        for i, b in enumerate(self.badges.values()):
            grid.addWidget(b, i // 3, i % 3)
        self.content.addWidget(grid_box)

        self.hint = QLabel("Subscribe on the Mode & Safety page or connect to see live data.")
        self.hint.setStyleSheet(f"color: {DRACULA.comment};")
        self.content.addWidget(self.hint)

        self.service.status_received.connect(self._on_status)
        self.service.telemetry.connect(self._on_telemetry)

    def _on_status(self, st) -> None:
        self.badges["state"].set(
            tlm.SAFETY_STATE_NAMES.get(st.state, str(st.state)),
            "ok" if st.state in (2, 4, 5) else "warn",
        )
        self.badges["battery"].set(f"{st.battery_mv} mV",
                                   "ok" if st.battery_mv > 10000 else "warn")
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
                tlm.FAULT_REASON_NAMES.get(record.fault_reason, str(record.fault_reason)),
                "error" if record.fault_reason else "ok",
            )
        elif stream_id == int(tlm.StreamId.HEALTH):
            self.badges["battery"].set(f"{record.battery_mv} mV",
                                       "ok" if record.battery_mv > 10000 else "warn")


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
        self._buttons = {}
        for i, (name, rate) in enumerate(self.STREAMS):
            btn = QPushButton(f"Subscribe {name} @ {rate} Hz")
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, n=name, r=rate: self._toggle(n, r, checked))
            lay.addWidget(btn, i // 2, i % 2)
            self._buttons[name] = btn
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

    def build(self) -> None:
        self.feed = QPlainTextEdit()
        self.feed.setReadOnly(True)
        self.feed.setMaximumBlockCount(500)
        self.feed.setObjectName("MonoLabel")
        self.content.addWidget(self.feed)
        self.service.telemetry.connect(self._on_telemetry)
        self.service.event.connect(self._on_event)

    def _on_telemetry(self, stream_id: int, record) -> None:
        name = tlm.STREAM_NAMES.get(tlm.StreamId(stream_id), str(stream_id))
        self.feed.appendPlainText(f"{name}: {record}")

    def _on_event(self, kind: str, detail: str) -> None:
        self.feed.appendPlainText(f"-- [{kind}] {detail}")
