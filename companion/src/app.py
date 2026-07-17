"""PySide6 application entry point for the hexapod companion."""

from __future__ import annotations

import argparse
import os
import sys


def main(argv: list[str] | None = None) -> int:
    from diagnostics import install_exception_hooks
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    from main_window import MainWindow
    from theme import apply_theme
    from ui.app_icon import app_icon

    parser = argparse.ArgumentParser(description="HexNav companion application")
    parser.add_argument(
        "--endpoint",
        default=os.environ.get("HEXAPOD_ENDPOINT"),
        help="Serial device or authenticated tcp:// endpoint to connect on startup.",
    )
    options, qt_args = parser.parse_known_args(argv)

    install_exception_hooks()

    app = QApplication.instance() or QApplication([sys.argv[0], *qt_args])
    app.setApplicationName("HexNav")
    app.setApplicationDisplayName("HexNav")
    app.setOrganizationName("Hexapod")
    app.setDesktopFileName("HexNav")
    app.setWindowIcon(app_icon())
    apply_theme(app)

    window = MainWindow()
    window.show()
    if options.endpoint:
        QTimer.singleShot(0, lambda: window.connect_endpoint(options.endpoint))
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
