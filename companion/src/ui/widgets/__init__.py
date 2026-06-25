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
        title = QLabel("HEXAPOD")
        title.setObjectName("NavRailTitle")
        self._lay.addWidget(title)
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
