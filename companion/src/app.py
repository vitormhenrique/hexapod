"""PySide6 application entry point for the hexapod companion."""

from __future__ import annotations

import sys


def main() -> int:
    from diagnostics import install_exception_hooks
    from PySide6.QtWidgets import QApplication

    from main_window import MainWindow
    from theme import apply_theme
    from ui.app_icon import app_icon

    install_exception_hooks()

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("HexNav")
    app.setApplicationDisplayName("HexNav")
    app.setOrganizationName("Hexapod")
    app.setDesktopFileName("HexNav")
    app.setWindowIcon(app_icon())
    apply_theme(app)

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
