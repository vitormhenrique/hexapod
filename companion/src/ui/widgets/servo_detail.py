"""Per-servo detail inspector widget (eax.7).

A drill-down panel for a single DYNAMIXEL servo: measured position, commanded
target, torque-enable, present current/PWM (load proxy), torque limit, voltage,
temperature, and decoded hardware-error bits, with a live position-vs-target
history sparkline. Fed from the ``servo_status`` (eax.6) and ``servo_goals``
(eax.2) telemetry streams plus an on-demand torque-limit read.
"""

from __future__ import annotations

from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import telemetry as tlm

_PRESENT = "#8be9fd"
_TARGET = "#ff79c6"
_BG = "#1e1f29"
_FG = "#6272a4"


class ServoDetailPanel(QGroupBox):
    """Live detail view for one selected servo id."""

    def __init__(self, history: int = 240, parent=None) -> None:
        super().__init__("Per-servo detail", parent)
        self._servo_id: int | None = None
        self._target_tick: int | None = None
        self._pos_hist: deque[float] = deque(maxlen=history)
        self._tgt_hist: deque[float] = deque(maxlen=history)

        outer = QVBoxLayout(self)

        self._heading = QLabel("No servo selected")
        self._heading.setObjectName("PageSubtitle")
        outer.addWidget(self._heading)

        form = QFormLayout()
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(8)
        self._fields: dict[str, QLabel] = {}
        for key, label in (
            ("position", "Measured position"),
            ("target", "Commanded target"),
            ("torque", "Torque enable"),
            ("current", "Present current/PWM"),
            ("load", "Load"),
            ("torque_limit", "Torque limit"),
            ("voltage", "Voltage"),
            ("temperature", "Temperature"),
            ("error", "Hardware errors"),
        ):
            value = QLabel("--")
            value.setObjectName("MonoLabel")
            self._fields[key] = value
            form.addRow(label, value)
        outer.addLayout(form)

        pg.setConfigOptions(antialias=True)
        self._plot = pg.PlotWidget(background=_BG)
        self._plot.setMinimumHeight(150)
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._plot.getAxis("left").setPen(_FG)
        self._plot.getAxis("bottom").setPen(_FG)
        self._plot.setLabel("left", "ticks")
        self._present_curve = self._plot.plot(
            [], [], pen=pg.mkPen(_PRESENT, width=2), name="present"
        )
        self._target_curve = self._plot.plot(
            [], [], pen=pg.mkPen(_TARGET, width=2, style=Qt.DashLine), name="target"
        )
        outer.addWidget(self._plot)

    # --- selection --------------------------------------------------------

    @property
    def servo_id(self) -> int | None:
        return self._servo_id

    def select_servo(self, servo_id: int | None) -> None:
        """Focus a servo id; clears history and resets fields."""
        self._servo_id = servo_id
        self._target_tick = None
        self._pos_hist.clear()
        self._tgt_hist.clear()
        for value in self._fields.values():
            value.setText("--")
        self._present_curve.setData([], [])
        self._target_curve.setData([], [])
        self._heading.setText(
            "No servo selected" if servo_id is None else f"Servo #{servo_id}"
        )

    # --- live feeds -------------------------------------------------------

    def update_status(self, status: tlm.ServoStatus) -> None:
        """Apply a ``servo_status`` record for the selected servo (else ignore)."""
        if self._servo_id is None or status.id != self._servo_id:
            return
        self._fields["position"].setText(str(status.position))
        self._fields["torque"].setText("ON" if status.torque_enabled else "off")
        self._fields["current"].setText(str(status.load))
        self._fields["load"].setText(str(status.load))
        self._fields["voltage"].setText(f"{status.voltage_mv / 1000:.1f} V")
        self._fields["temperature"].setText(f"{status.temperature_c} \u00b0C")
        bits = tlm.decode_hw_error(status.hardware_error)
        self._fields["error"].setText(
            "none" if not bits else f"0x{status.hardware_error:02X}: " + ", ".join(bits)
        )
        self._pos_hist.append(float(status.position))
        self._tgt_hist.append(
            float(self._target_tick) if self._target_tick is not None else float("nan")
        )
        self._redraw()

    def set_target_tick(self, tick: int | None) -> None:
        """Set the commanded target tick (from a mapped ``servo_goals`` entry)."""
        self._target_tick = tick
        self._fields["target"].setText("--" if tick is None else str(tick))

    def set_torque_limit(self, value: int | None) -> None:
        self._fields["torque_limit"].setText("--" if value is None else str(value))

    # --- internals --------------------------------------------------------

    def _redraw(self) -> None:
        xs = list(range(len(self._pos_hist)))
        self._present_curve.setData(xs, list(self._pos_hist))
        tgt = list(self._tgt_hist)
        if any(v == v for v in tgt):  # at least one non-NaN
            self._target_curve.setData(xs, tgt)
        else:
            self._target_curve.setData([], [])
