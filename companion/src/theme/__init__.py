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

    # Derived surfaces for a layered modern look (darkest base -> lightest card).
    base = "#1a1b23"  # app canvas (behind everything)
    surface = "#21222c"  # nav rail / bars
    surface_alt = "#272935"  # cards / panels (lighter than base for separation)
    input_bg = "#1e1f29"  # inputs sit recessed
    elevated = "#343746"  # buttons / hover
    border = "#13141b"  # crisp 1px separators
    border_soft = "#33354a"  # subtle inner borders


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
    font-family: "Helvetica Neue", "Segoe UI", "Inter", sans-serif;
    font-size: 13px;
    outline: 0;
}}

/* Labels and most leaf widgets must be transparent so they don't paint
   opaque rectangles over the cards beneath them. */
QLabel, QCheckBox, QRadioButton, QGroupBox::indicator {{
    background: transparent;
}}
QWidget {{
    color: {DRACULA.foreground};
}}
QMainWindow, QDialog {{
    background-color: {DRACULA.base};
}}
QStackedWidget > QWidget {{
    background-color: {DRACULA.base};
}}

/* --- Navigation rail --- */
#NavRail {{
    background-color: {DRACULA.surface};
    border-right: 1px solid {DRACULA.border};
}}
#NavRail QPushButton {{
    text-align: left;
    padding: 10px 14px;
    margin: 2px 12px;
    border: none;
    border-radius: 8px;
    color: {DRACULA.comment};
    font-size: 13px;
    font-weight: 500;
    background: transparent;
}}
#NavRail QPushButton:hover {{
    background-color: {DRACULA.elevated};
    color: {DRACULA.foreground};
}}
#NavRail QPushButton:checked {{
    background-color: {DRACULA.purple};
    color: #ffffff;
    font-weight: 600;
}}
#NavRailTitle {{
    color: {DRACULA.purple};
    font-size: 17px;
    font-weight: 800;
    letter-spacing: 2px;
    padding: 0;
}}
#NavSection {{
    color: {DRACULA.comment};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1.5px;
    padding: 16px 20px 6px 20px;
}}

/* --- Safety top bar --- */
#SafetyBar {{
    background-color: {DRACULA.surface};
    border-bottom: 1px solid {DRACULA.border};
}}
#BadgeCaption {{
    color: {DRACULA.comment};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}}
#BadgeValue {{
    color: {DRACULA.foreground};
    font-size: 13px;
    font-weight: 600;
}}
#BadgeSep {{
    color: {DRACULA.border_soft};
}}

/* --- Event strip --- */
#EventStrip {{
    background-color: {DRACULA.surface};
    border-top: 1px solid {DRACULA.border};
}}
#EventStrip QLabel {{
    color: {DRACULA.comment};
    font-family: "SF Mono", "JetBrains Mono", Menlo, monospace;
    font-size: 12px;
}}

/* --- Cards / group boxes --- */
QGroupBox {{
    background-color: {DRACULA.surface_alt};
    border: 1px solid {DRACULA.border_soft};
    border-radius: 12px;
    margin-top: 22px;
    padding: 22px 18px 18px 18px;
    font-size: 13px;
    font-weight: 700;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 4px;
    top: 0px;
    padding: 0 2px;
    color: {DRACULA.comment};
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 1.2px;
}}
#Card {{
    background-color: {DRACULA.surface_alt};
    border: 1px solid {DRACULA.border_soft};
    border-radius: 12px;
}}
#StatCard {{
    background-color: {DRACULA.input_bg};
    border: 1px solid {DRACULA.border_soft};
    border-radius: 10px;
}}
#StatCaption {{
    color: {DRACULA.comment};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}}
#StatValue {{
    color: {DRACULA.foreground};
    font-size: 22px;
    font-weight: 700;
}}

/* --- Buttons --- */
QPushButton {{
    background-color: {DRACULA.elevated};
    border: 1px solid {DRACULA.border_soft};
    border-radius: 8px;
    padding: 8px 18px;
    font-weight: 600;
    min-height: 18px;
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
    background-color: {DRACULA.surface};
    border-color: {DRACULA.border};
}}
QPushButton[accent="true"] {{
    background-color: {DRACULA.purple};
    color: #ffffff;
    border: none;
}}
QPushButton[accent="true"]:hover {{
    background-color: {DRACULA.pink};
}}
QPushButton[accent="true"]:disabled {{
    background-color: {DRACULA.elevated};
    color: {DRACULA.comment};
}}
QPushButton:checked {{
    background-color: {DRACULA.purple};
    color: #ffffff;
    border: none;
}}
#EmergencyStop {{
    background-color: {DRACULA.red};
    color: #ffffff;
    border: none;
    border-radius: 8px;
    padding: 10px 22px;
    font-size: 13px;
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
    background-color: {DRACULA.input_bg};
    border: 1px solid {DRACULA.border};
    border-radius: 8px;
    padding: 8px 10px;
    selection-background-color: {DRACULA.purple};
    selection-color: #ffffff;
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus,
QDoubleSpinBox:focus, QPlainTextEdit:focus {{
    border-color: {DRACULA.purple};
}}
QComboBox::drop-down {{ border: none; width: 24px; }}
QComboBox QAbstractItemView {{
    background-color: {DRACULA.surface_alt};
    border: 1px solid {DRACULA.border_soft};
    border-radius: 8px;
    padding: 4px;
    selection-background-color: {DRACULA.purple};
    selection-color: #ffffff;
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
    background-color: {DRACULA.input_bg};
    alternate-background-color: {DRACULA.surface_alt};
    gridline-color: {DRACULA.border};
    border: 1px solid {DRACULA.border};
    border-radius: 10px;
    selection-background-color: {DRACULA.purple};
    selection-color: #ffffff;
}}
QHeaderView::section {{
    background-color: {DRACULA.surface};
    color: {DRACULA.comment};
    padding: 8px 10px;
    border: none;
    border-bottom: 1px solid {DRACULA.border};
    font-weight: 600;
}}

/* --- Scrollbars --- */
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{
    background: transparent; width: 12px; margin: 2px;
}}
QScrollBar::handle:vertical {{
    background: {DRACULA.current_line}; border-radius: 5px; min-height: 28px;
}}
QScrollBar::handle:vertical:hover {{ background: {DRACULA.comment}; }}
QScrollBar:horizontal {{
    background: transparent; height: 12px; margin: 2px;
}}
QScrollBar::handle:horizontal {{
    background: {DRACULA.current_line}; border-radius: 5px; min-width: 28px;
}}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

/* --- Checkboxes --- */
QCheckBox::indicator, QRadioButton::indicator {{
    width: 16px; height: 16px; border-radius: 4px;
    border: 1px solid {DRACULA.comment}; background: {DRACULA.input_bg};
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

/* --- Form rows --- */
QFormLayout QLabel {{ color: {DRACULA.comment}; }}

#PageTitle {{ font-size: 24px; font-weight: 800; color: {DRACULA.foreground}; }}
#PageSubtitle {{ color: {DRACULA.comment}; font-size: 13px; }}
#MonoLabel {{
    font-family: "SF Mono", "JetBrains Mono", Menlo, monospace;
    color: {DRACULA.foreground};
}}
"""


def apply_theme(app) -> None:
    """Apply the Dracula palette + stylesheet to a ``QApplication``."""
    from PySide6.QtGui import QColor, QPalette

    pal = QPalette()
    bg = QColor(DRACULA.base)
    surface = QColor(DRACULA.input_bg)
    fg = QColor(DRACULA.foreground)
    pal.setColor(QPalette.Window, bg)
    pal.setColor(QPalette.WindowText, fg)
    pal.setColor(QPalette.Base, surface)
    pal.setColor(QPalette.AlternateBase, QColor(DRACULA.surface_alt))
    pal.setColor(QPalette.Text, fg)
    pal.setColor(QPalette.Button, QColor(DRACULA.elevated))
    pal.setColor(QPalette.ButtonText, fg)
    pal.setColor(QPalette.Highlight, QColor(DRACULA.purple))
    pal.setColor(QPalette.HighlightedText, QColor("#ffffff"))
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
