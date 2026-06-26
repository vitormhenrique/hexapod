"""Programmatic application icon (no binary asset dependency).

Draws a hexapod-themed mark -- a rounded hexagon with six legs in the Dracula
purple/pink palette -- as a :class:`QIcon`, used for the dock / taskbar running
icon and the window title bar.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QBrush,
    QColor,
    QIcon,
    QLinearGradient,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)

from theme import DRACULA


def _hexagon(cx: float, cy: float, r: float, rot_deg: float = 0.0) -> QPolygonF:
    poly = QPolygonF()
    for i in range(6):
        a = math.radians(rot_deg + 60 * i)
        poly.append(QPointF(cx + r * math.cos(a), cy + r * math.sin(a)))
    return poly


def _render(size: int) -> QPixmap:
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)

    cx = cy = size / 2.0
    body_r = size * 0.30

    # Six legs radiating from the body.
    leg_pen = QPen(QColor(DRACULA.pink))
    leg_pen.setWidthF(size * 0.045)
    leg_pen.setCapStyle(Qt.RoundCap)
    p.setPen(leg_pen)
    for i in range(6):
        a = math.radians(30 + 60 * i)
        knee = QPointF(cx + body_r * 1.25 * math.cos(a), cy + body_r * 1.25 * math.sin(a))
        foot = QPointF(cx + body_r * 1.95 * math.cos(a), cy + body_r * 1.95 * math.sin(a))
        hip = QPointF(cx + body_r * 0.85 * math.cos(a), cy + body_r * 0.85 * math.sin(a))
        p.drawLine(hip, knee)
        p.drawLine(knee, foot)

    # Body: filled hexagon with a vertical gradient + soft outline.
    grad = QLinearGradient(0, cy - body_r, 0, cy + body_r)
    grad.setColorAt(0.0, QColor(DRACULA.purple))
    grad.setColorAt(1.0, QColor("#7d5bd6"))
    p.setBrush(QBrush(grad))
    p.setPen(QPen(QColor(DRACULA.foreground), size * 0.02))
    p.drawPolygon(_hexagon(cx, cy, body_r, rot_deg=0))

    # Inner accent hexagon.
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(QColor(DRACULA.cyan), size * 0.02))
    p.drawPolygon(_hexagon(cx, cy, body_r * 0.55, rot_deg=0))

    p.end()
    return pm


def app_icon() -> QIcon:
    """Build a multi-resolution application icon."""
    icon = QIcon()
    for s in (16, 24, 32, 48, 64, 128, 256):
        icon.addPixmap(_render(s))
    return icon
