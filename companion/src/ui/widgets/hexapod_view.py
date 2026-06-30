"""Animated 2D hexapod model widget (top-down + side view) via pyqtgraph.

Renders all six legs / 18 joints from :class:`LegPose` snapshots produced by the
UI-independent :class:`HexapodPoseModel`. Redraw is decoupled from the telemetry
rate by an internal timer (default ~30 FPS) that only repaints when new pose
data has arrived, so the GUI thread is never flooded or blocked.

Coordinate frames (body frame B, mm):
* top-down view -> X (forward) right, Y (left) up
* side view     -> X (forward) right, Z (up) up
"""

from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QHBoxLayout, QWidget

from models.pose_model import LegPose

# Distinct per-leg colors (front-right around to front-left).
_LEG_COLORS = [
    "#ff5555",
    "#ffb86c",
    "#f1fa8c",
    "#50fa7b",
    "#8be9fd",
    "#bd93f9",
]
_BG = "#1e1f29"
_FG = "#6272a4"
_FOOT = "#ff79c6"
_BODY = "#44475a"


class HexapodView(QWidget):
    """Live top-down + side animation of the hexapod from pose snapshots."""

    def __init__(self, fps: int = 30, parent=None) -> None:
        super().__init__(parent)
        pg.setConfigOptions(antialias=True)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        self._top = self._make_plot("Top-down (X fwd, Y left)", locked=True)
        self._side = self._make_plot("Side (X fwd, Z up)", locked=True)
        lay.addWidget(self._top, 1)
        lay.addWidget(self._side, 1)

        # One polyline per leg per view, plus a foot scatter and body outline.
        self._top_lines: list[pg.PlotDataItem] = []
        self._side_lines: list[pg.PlotDataItem] = []
        for color in _LEG_COLORS:
            pen = pg.mkPen(color=color, width=3)
            self._top_lines.append(
                self._top.plot(
                    [],
                    [],
                    pen=pen,
                    symbol="o",
                    symbolSize=6,
                    symbolBrush=color,
                    symbolPen=None,
                )
            )
            self._side_lines.append(
                self._side.plot(
                    [],
                    [],
                    pen=pen,
                    symbol="o",
                    symbolSize=6,
                    symbolBrush=color,
                    symbolPen=None,
                )
            )
        self._top_body = self._top.plot(
            [], [], pen=pg.mkPen(color=_BODY, width=2, style=Qt.DashLine)
        )
        self._top_feet = pg.ScatterPlotItem(size=11, brush=_FOOT, pen=None)
        self._top.addItem(self._top_feet)
        self._side_feet = pg.ScatterPlotItem(size=11, brush=_FOOT, pen=None)
        self._side.addItem(self._side_feet)

        self._legs: list[LegPose] = []
        self._dirty = False

        self._timer = QTimer(self)
        self._timer.setInterval(max(1, int(1000 / max(1, fps))))
        self._timer.timeout.connect(self._redraw)
        self._timer.start()

    def _make_plot(self, title: str, locked: bool) -> pg.PlotWidget:
        plot = pg.PlotWidget(background=_BG, title=title)
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.getAxis("left").setPen(_FG)
        plot.getAxis("bottom").setPen(_FG)
        plot.setMenuEnabled(False)
        if locked:
            plot.setAspectLocked(True)
        return plot

    # --- feed -------------------------------------------------------------

    def set_legs(self, legs: list[LegPose]) -> None:
        """Store the latest pose snapshot; the timer repaints on the next tick."""
        self._legs = legs
        self._dirty = True

    def stop(self) -> None:
        self._timer.stop()

    # --- render -----------------------------------------------------------

    def _redraw(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        legs = self._legs

        hips_x: list[float] = []
        hips_y: list[float] = []
        feet_top: list[tuple[float, float]] = []
        feet_side: list[tuple[float, float]] = []

        for i, leg in enumerate(legs):
            if i >= len(self._top_lines):
                break
            pts = leg.points
            xs = [p.x for p in pts]
            ys = [p.y for p in pts]
            zs = [p.z for p in pts]
            self._top_lines[i].setData(xs, ys)
            self._side_lines[i].setData(xs, zs)
            hips_x.append(leg.hip.x)
            hips_y.append(leg.hip.y)
            feet_top.append((leg.foot.x, leg.foot.y))
            feet_side.append((leg.foot.x, leg.foot.z))

        # Close the hip ring for a body outline in the top-down view.
        if hips_x:
            self._top_body.setData(hips_x + hips_x[:1], hips_y + hips_y[:1])
        self._top_feet.setData([p[0] for p in feet_top], [p[1] for p in feet_top])
        self._side_feet.setData([p[0] for p in feet_side], [p[1] for p in feet_side])
