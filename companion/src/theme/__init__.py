"""Dracula dark theme for the PySide6 companion UI.

Exposes the official Dracula palette (https://draculatheme.com) plus a modern Qt
stylesheet and a ``QPalette`` so the whole app renders in a consistent dark
scheme. Importing this module does not require Qt; ``apply_theme`` does.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Dracula:
    background = "#282a36"
    current_line = "#44475a"
    selection = "#44475a"
    foreground = "#f8f8f2"
    comment = "#6272a4"
    cyan = "#8be9fd"
    green = "#50fa7b"
    orange = "#ffb86c"
    pink = "#ff79c6"
    purple = "#bd93f9"
    red = "#ff5555"
    yellow = "#f1fa8c"

    # Derived surfaces for a layered modern look.
    surface = "#21222c"
    surface_alt = "#1e1f29"
    elevated = "#343746"
    border = "#191a21"


DRACULA = Dracula()


# Semantic status colors (badges, safety bar).
STATUS_COLORS = {
    "ok": DRACULA.green,
    "good": DRACULA.green,
    "warn": DRACULA.yellow,
    "warning": DRACULA.yellow,
    "error": DRACULA.red,
    "danger": DRACULA.red,
    "fault": DRACULA.red,
    "info": DRACULA.cyan,
    "idle": DRACULA.comment,
    "active": DRACULA.purple,
}


def status_color(level: str) -> str:
    return STATUS_COLORS.get(level.lower(), DRACULA.comment)


STYLESHEET = f"""
* {{
    color: {DRACULA.foreground};
    font-family: -apple-system, "SF Pro Text", "Segoe UI", "Inter", sans-serif;
    font-size: 13px;
}}
QWidget {{
    background-color: {DRACULA.background};
}}
QMainWindow, QDialog {{
    background-color: {DRACULA.background};
}}

/* --- Navigation rail --- */
#NavRail {{
    background-color: {DRACULA.surface_alt};
    border-right: 1px solid {DRACULA.border};
}}
#NavRail QPushButton {{
    text-align: left;
    padding: 10px 16px;
    margin: 2px 8px;
    border: none;
    border-radius: 8px;
    color: {DRACULA.comment};
    font-size: 13px;
    font-weight: 500;
    background: transparent;
}}
#NavRail QPushButton:hover {{
    background-color: {DRACULA.current_line};
    color: {DRACULA.foreground};
}}
#NavRail QPushButton:checked {{
    background-color: {DRACULA.purple};
    color: {DRACULA.background};
    font-weight: 600;
}}
#NavRailTitle {{
    color: {DRACULA.purple};
    font-size: 16px;
    font-weight: 700;
    padding: 16px 16px 8px 16px;
}}

/* --- Safety top bar --- */
#SafetyBar {{
    background-color: {DRACULA.surface};
    border-bottom: 1px solid {DRACULA.border};
}}
#SafetyBar QLabel {{
    color: {DRACULA.comment};
    font-size: 12px;
}}

/* --- Event strip --- */
#EventStrip {{
    background-color: {DRACULA.surface_alt};
    border-top: 1px solid {DRACULA.border};
    color: {DRACULA.comment};
    font-family: "SF Mono", "JetBrains Mono", Menlo, monospace;
    font-size: 12px;
}}

/* --- Cards / group boxes --- */
QGroupBox {{
    background-color: {DRACULA.surface};
    border: 1px solid {DRACULA.border};
    border-radius: 10px;
    margin-top: 14px;
    padding: 12px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 6px;
    color: {DRACULA.purple};
}}
#Card {{
    background-color: {DRACULA.surface};
    border: 1px solid {DRACULA.border};
    border-radius: 10px;
}}

/* --- Buttons --- */
QPushButton {{
    background-color: {DRACULA.elevated};
    border: 1px solid {DRACULA.border};
    border-radius: 8px;
    padding: 8px 16px;
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: {DRACULA.current_line};
    border-color: {DRACULA.comment};
}}
QPushButton:pressed {{
    background-color: {DRACULA.selection};
}}
QPushButton:disabled {{
    color: {DRACULA.comment};
    background-color: {DRACULA.surface_alt};
}}
QPushButton[accent="true"] {{
    background-color: {DRACULA.purple};
    color: {DRACULA.background};
    border: none;
}}
QPushButton[accent="true"]:hover {{
    background-color: {DRACULA.pink};
}}
#EmergencyStop {{
    background-color: {DRACULA.red};
    color: #ffffff;
    border: none;
    border-radius: 8px;
    padding: 10px 22px;
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 1px;
}}
#EmergencyStop:hover {{
    background-color: #ff6e6e;
}}
#EmergencyStop:pressed {{
    background-color: #e23d3d;
}}

/* --- Inputs --- */
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTextEdit {{
    background-color: {DRACULA.surface_alt};
    border: 1px solid {DRACULA.border};
    border-radius: 6px;
    padding: 6px 8px;
    selection-background-color: {DRACULA.purple};
    selection-color: {DRACULA.background};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {DRACULA.purple};
}}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{
    background-color: {DRACULA.surface};
    border: 1px solid {DRACULA.border};
    selection-background-color: {DRACULA.purple};
    selection-color: {DRACULA.background};
}}

/* --- Sliders --- */
QSlider::groove:horizontal {{
    height: 5px; background: {DRACULA.current_line}; border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {DRACULA.purple}; width: 16px; height: 16px;
    margin: -6px 0; border-radius: 8px;
}}
QSlider::sub-page:horizontal {{ background: {DRACULA.purple}; border-radius: 2px; }}

/* --- Tables --- */
QTableView, QTableWidget, QTreeView, QListView {{
    background-color: {DRACULA.surface};
    alternate-background-color: {DRACULA.surface_alt};
    gridline-color: {DRACULA.border};
    border: 1px solid {DRACULA.border};
    border-radius: 8px;
    selection-background-color: {DRACULA.purple};
    selection-color: {DRACULA.background};
}}
QHeaderView::section {{
    background-color: {DRACULA.surface_alt};
    color: {DRACULA.comment};
    padding: 6px 8px;
    border: none;
    border-bottom: 1px solid {DRACULA.border};
    font-weight: 600;
}}

/* --- Scrollbars --- */
QScrollBar:vertical {{
    background: transparent; width: 10px; margin: 2px;
}}
QScrollBar::handle:vertical {{
    background: {DRACULA.current_line}; border-radius: 5px; min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{ background: {DRACULA.comment}; }}
QScrollBar:horizontal {{
    background: transparent; height: 10px; margin: 2px;
}}
QScrollBar::handle:horizontal {{
    background: {DRACULA.current_line}; border-radius: 5px; min-width: 24px;
}}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}

/* --- Checkboxes / labels --- */
QCheckBox::indicator, QRadioButton::indicator {{
    width: 16px; height: 16px; border-radius: 4px;
    border: 1px solid {DRACULA.comment}; background: {DRACULA.surface_alt};
}}
QCheckBox::indicator:checked {{
    background: {DRACULA.green}; border-color: {DRACULA.green};
}}
QToolTip {{
    background-color: {DRACULA.elevated};
    color: {DRACULA.foreground};
    border: 1px solid {DRACULA.purple};
    border-radius: 6px;
    padding: 6px;
}}
#PageTitle {{ font-size: 22px; font-weight: 700; color: {DRACULA.foreground}; }}
#PageSubtitle {{ color: {DRACULA.comment}; font-size: 13px; }}
#MonoLabel {{ font-family: "SF Mono", Menlo, monospace; }}
"""


def apply_theme(app) -> None:
    """Apply the Dracula palette + stylesheet to a ``QApplication``."""
    from PySide6.QtGui import QColor, QPalette

    pal = QPalette()
    bg = QColor(DRACULA.background)
    surface = QColor(DRACULA.surface)
    fg = QColor(DRACULA.foreground)
    pal.setColor(QPalette.Window, bg)
    pal.setColor(QPalette.WindowText, fg)
    pal.setColor(QPalette.Base, surface)
    pal.setColor(QPalette.AlternateBase, QColor(DRACULA.surface_alt))
    pal.setColor(QPalette.Text, fg)
    pal.setColor(QPalette.Button, QColor(DRACULA.elevated))
    pal.setColor(QPalette.ButtonText, fg)
    pal.setColor(QPalette.Highlight, QColor(DRACULA.purple))
    pal.setColor(QPalette.HighlightedText, bg)
    pal.setColor(QPalette.ToolTipBase, QColor(DRACULA.elevated))
    pal.setColor(QPalette.ToolTipText, fg)
    pal.setColor(QPalette.PlaceholderText, QColor(DRACULA.comment))
    pal.setColor(QPalette.Link, QColor(DRACULA.cyan))
    app.setPalette(pal)
    app.setStyleSheet(STYLESHEET)
    try:
        app.setStyle("Fusion")
    except Exception:
        pass
