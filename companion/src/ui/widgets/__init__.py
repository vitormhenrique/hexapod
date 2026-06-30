"""Reusable companion widgets (status badge, safety bar, nav rail, event strip)."""

from __future__ import annotations

import time
from collections import deque

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from theme import DRACULA, status_color


class StatusBadge(QWidget):
    """A small colored dot + caption/value stack used across the safety bar."""

    def __init__(self, caption: str, parent=None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 0, 4, 0)
        lay.setSpacing(8)
        self._dot = QLabel("\u25cf")
        self._dot.setStyleSheet(f"color: {DRACULA.comment}; font-size: 11px;")
        self._dot.setFixedWidth(12)
        self._dot.setAlignment(Qt.AlignVCenter | Qt.AlignHCenter)
        self._caption = QLabel(caption)
        self._caption.setObjectName("BadgeCaption")
        self._value = QLabel("--")
        self._value.setObjectName("BadgeValue")
        col = QVBoxLayout()
        col.setSpacing(1)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self._caption)
        col.addWidget(self._value)
        lay.addWidget(self._dot)
        lay.addLayout(col)

    def set(self, value: str, level: str = "idle") -> None:
        self._value.setText(value)
        self._dot.setStyleSheet(f"color: {status_color(level)}; font-size: 11px;")


class StatCard(QFrame):
    """A larger stat tile (caption + big value + status dot) that fills its cell.

    Shares the ``set(value, level)`` API with :class:`StatusBadge` so it is a
    drop-in for dashboard grids that should use the available space.
    """

    def __init__(self, caption: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("StatCard")
        self.setMinimumHeight(84)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setSpacing(6)

        top = QHBoxLayout()
        top.setSpacing(8)
        self._dot = QLabel("\u25cf")
        self._dot.setStyleSheet(f"color: {DRACULA.comment}; font-size: 11px;")
        self._dot.setFixedWidth(12)
        self._caption = QLabel(caption.upper())
        self._caption.setObjectName("StatCaption")
        top.addWidget(self._dot)
        top.addWidget(self._caption)
        top.addStretch(1)
        lay.addLayout(top)

        self._value = QLabel("--")
        self._value.setObjectName("StatValue")
        lay.addWidget(self._value)
        lay.addStretch(1)

    def set(self, value: str, level: str = "idle") -> None:
        self._value.setText(value)
        self._dot.setStyleSheet(f"color: {status_color(level)}; font-size: 11px;")


class FeatureToggleCard(QFrame):
    """A feature-flag tile: name, availability/enabled state + reason, toggle.

    The button is disabled when the firmware reports the feature unavailable and
    shows the unavailability reason, so the operator always sees *why* a feature
    cannot be turned on (AGENTS.md 1.3 / 7.3).
    """

    toggled = Signal(int, bool)  # feature id, desired-enable

    def __init__(self, feature_id: int, name: str, parent=None) -> None:
        super().__init__(parent)
        self._feature = feature_id
        self.setObjectName("StatCard")
        self.setMinimumHeight(96)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(8)
        self._dot = QLabel("\u25cf")
        self._dot.setStyleSheet(f"color: {DRACULA.comment}; font-size: 11px;")
        self._dot.setFixedWidth(12)
        cap = QLabel(name.upper())
        cap.setObjectName("StatCaption")
        top.addWidget(self._dot)
        top.addWidget(cap)
        top.addStretch(1)
        self._btn = QPushButton("Enable")
        self._btn.setCheckable(True)
        self._btn.setCursor(Qt.PointingHandCursor)
        self._btn.clicked.connect(
            lambda checked: self.toggled.emit(self._feature, checked)
        )
        top.addWidget(self._btn)
        lay.addLayout(top)

        self._reason = QLabel("--")
        self._reason.setObjectName("StatCaption")
        lay.addWidget(self._reason)
        lay.addStretch(1)

    @property
    def feature(self) -> int:
        return self._feature

    def set_state(self, available: bool, enabled: bool, reason: str) -> None:
        self._btn.setEnabled(available)
        self._btn.blockSignals(True)
        self._btn.setChecked(enabled)
        self._btn.blockSignals(False)
        self._btn.setText("Enabled" if enabled else "Enable")
        if not available:
            level = "idle"
            self._reason.setText(f"unavailable — {reason}")
        elif enabled:
            level = "ok"
            self._reason.setText("enabled")
        else:
            level = "warn"
            self._reason.setText(reason if reason and reason != "NONE" else "available")
        self._dot.setStyleSheet(f"color: {status_color(level)}; font-size: 11px;")


def _badge_separator() -> QFrame:
    sep = QFrame()
    sep.setObjectName("BadgeSep")
    sep.setFrameShape(QFrame.VLine)
    sep.setFixedHeight(30)
    sep.setStyleSheet(f"color: {DRACULA.border_soft};")
    return sep


class EmergencyStopButton(QPushButton):
    def __init__(self, parent=None) -> None:
        super().__init__("\u23fb  EMERGENCY STOP", parent)
        self.setObjectName("EmergencyStop")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(40)


class SafetyBar(QFrame):
    """Always-visible top bar: connection, mode, arming, torque, RC, battery, estop."""

    estop_pressed = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("SafetyBar")
        self.setFixedHeight(72)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(20, 10, 20, 10)
        lay.setSpacing(14)

        self.conn = StatusBadge("CONNECTION")
        self.mode = StatusBadge("MODE")
        self.arming = StatusBadge("ARMING")
        self.torque = StatusBadge("TORQUE")
        self.source = StatusBadge("CMD SOURCE")
        self.rc = StatusBadge("RC LINK")
        self.battery = StatusBadge("BATTERY")
        badges = (
            self.conn,
            self.mode,
            self.arming,
            self.torque,
            self.source,
            self.rc,
            self.battery,
        )
        for i, b in enumerate(badges):
            if i:
                lay.addWidget(_badge_separator())
            lay.addWidget(b)
        lay.addStretch(1)
        self.estop = EmergencyStopButton()
        self.estop.clicked.connect(self.estop_pressed)
        lay.addWidget(self.estop)

        self.conn.set("Disconnected", "idle")

    # Convenience setters used by MainWindow.
    def set_connection(self, connected: bool) -> None:
        self.conn.set(
            "Connected" if connected else "Disconnected", "ok" if connected else "idle"
        )


class NavRail(QFrame):
    """Left navigation rail. Emits the page key when a button is selected."""

    navigated = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("NavRail")
        self.setFixedWidth(220)
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 12)
        self._lay.setSpacing(0)

        header = QWidget()
        hrow = QHBoxLayout(header)
        hrow.setContentsMargins(18, 20, 18, 6)
        hrow.setSpacing(10)
        from ui.app_icon import app_icon

        logo = QLabel()
        logo.setPixmap(app_icon().pixmap(26, 26))
        title = QLabel("HEXNAV")
        title.setObjectName("NavRailTitle")
        hrow.addWidget(logo)
        hrow.addWidget(title)
        hrow.addStretch(1)
        self._lay.addWidget(header)

        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: dict[str, QPushButton] = {}

    def add_section(self, label: str) -> None:
        sec = QLabel(label.upper())
        sec.setObjectName("NavSection")
        self._lay.addWidget(sec)

    def add_item(self, key: str, label: str) -> None:
        btn = QPushButton(label)
        btn.setCheckable(True)
        btn.setCursor(Qt.PointingHandCursor)
        btn.clicked.connect(lambda: self.navigated.emit(key))
        self._group.addButton(btn)
        self._buttons[key] = btn
        self._lay.addWidget(btn)

    def finish(self) -> None:
        self._lay.addStretch(1)

    def select(self, key: str) -> None:
        btn = self._buttons.get(key)
        if btn:
            btn.setChecked(True)
            self.navigated.emit(key)


class EventStrip(QFrame):
    """Bottom strip showing recent faults, commits, mode changes, etc."""

    def __init__(self, parent=None, capacity: int = 8) -> None:
        super().__init__(parent)
        self.setObjectName("EventStrip")
        self.setFixedHeight(30)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(20, 2, 20, 2)
        self._label = QLabel("Ready.")
        lay.addWidget(self._label)
        lay.addStretch(1)
        self._events: deque = deque(maxlen=capacity)

    def add(self, kind: str, detail: str) -> None:
        ts = time.strftime("%H:%M:%S")
        color = {
            "fault": DRACULA.red,
            "estop": DRACULA.red,
            "error": DRACULA.red,
            "connect": DRACULA.green,
            "disconnect": DRACULA.orange,
            "commit": DRACULA.cyan,
        }.get(kind, DRACULA.comment)
        self._events.appendleft(f"{ts}  [{kind}] {detail}")
        self._label.setText(self._events[0])
        self._label.setStyleSheet(f"color: {color};")


from ui.widgets.hexapod_view import HexapodView  # noqa: E402  (re-export)
from ui.widgets.servo_detail import ServoDetailPanel  # noqa: E402  (re-export)

__all__ = [
    "StatusBadge",
    "StatCard",
    "FeatureToggleCard",
    "EmergencyStopButton",
    "SafetyBar",
    "NavRail",
    "EventStrip",
    "HexapodView",
    "ServoDetailPanel",
]
