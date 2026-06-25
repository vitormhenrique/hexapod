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

from ..theme import DRACULA, status_color


class StatusBadge(QWidget):
    """A small colored dot + label used across the safety bar and overview."""

    def __init__(self, caption: str, parent=None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 2, 8, 2)
        lay.setSpacing(6)
        self._dot = QLabel("\u25cf")
        self._dot.setStyleSheet(f"color: {DRACULA.comment}; font-size: 14px;")
        self._caption = QLabel(caption)
        self._caption.setStyleSheet(f"color: {DRACULA.comment}; font-size: 11px;")
        self._value = QLabel("--")
        self._value.setStyleSheet(f"color: {DRACULA.foreground}; font-weight: 600;")
        col = QVBoxLayout()
        col.setSpacing(0)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self._caption)
        row = QHBoxLayout()
        row.setSpacing(6)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self._dot)
        row.addWidget(self._value)
        col.addLayout(row)
        lay.addLayout(col)

    def set(self, value: str, level: str = "idle") -> None:
        self._value.setText(value)
        self._dot.setStyleSheet(f"color: {status_color(level)}; font-size: 14px;")


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
        self.setFixedHeight(64)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 8, 14, 8)
        lay.setSpacing(6)

        self.conn = StatusBadge("CONNECTION")
        self.mode = StatusBadge("MODE")
        self.arming = StatusBadge("ARMING")
        self.torque = StatusBadge("TORQUE")
        self.source = StatusBadge("CMD SOURCE")
        self.rc = StatusBadge("RC LINK")
        self.battery = StatusBadge("BATTERY")
        for b in (self.conn, self.mode, self.arming, self.torque,
                  self.source, self.rc, self.battery):
            lay.addWidget(b)
        lay.addStretch(1)
        self.estop = EmergencyStopButton()
        self.estop.clicked.connect(self.estop_pressed)
        lay.addWidget(self.estop)

        self.conn.set("Disconnected", "idle")

    # Convenience setters used by MainWindow.
    def set_connection(self, connected: bool) -> None:
        self.conn.set("Connected" if connected else "Disconnected",
                      "ok" if connected else "idle")


class NavRail(QFrame):
    """Left navigation rail. Emits the page key when a button is selected."""

    navigated = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("NavRail")
        self.setFixedWidth(210)
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 12)
        self._lay.setSpacing(0)
        title = QLabel("HEXAPOD")
        title.setObjectName("NavRailTitle")
        self._lay.addWidget(title)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._buttons: dict[str, QPushButton] = {}

    def add_section(self, label: str) -> None:
        sec = QLabel(label.upper())
        sec.setStyleSheet(
            f"color: {DRACULA.comment}; font-size: 10px; font-weight: 700;"
            "padding: 12px 16px 4px 16px; letter-spacing: 1px;"
        )
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
        self.setFixedHeight(28)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 2, 14, 2)
        self._label = QLabel("Ready.")
        self._label.setStyleSheet(f"color: {DRACULA.comment};")
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
