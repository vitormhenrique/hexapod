"""Theme module tests: palette constants are pure, apply_theme runs headless."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest


def test_palette_and_stylesheet_need_no_qt() -> None:
    # Importing the theme module must not require Qt.
    from theme import DRACULA, STYLESHEET, status_color

    assert DRACULA.background == "#282a36"
    assert DRACULA.foreground == "#f8f8f2"
    # Stylesheet is built from the palette and references derived surfaces.
    assert DRACULA.base in STYLESHEET
    assert "#NavRail" in STYLESHEET
    # Semantic status colors map to palette entries with a safe fallback.
    assert status_color("ok") == DRACULA.green
    assert status_color("fault") == DRACULA.red
    assert status_color("totally-unknown") == DRACULA.comment


def test_apply_theme_runs_offscreen() -> None:
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication

    from theme import DRACULA, apply_theme

    app = QApplication.instance() or QApplication([])
    apply_theme(app)

    assert app.styleSheet()  # stylesheet installed
    # Palette window color reflects the Dracula base canvas.
    assert app.palette().window().color().name() == DRACULA.base
