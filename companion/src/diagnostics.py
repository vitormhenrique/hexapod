"""Terminal diagnostics for the companion app."""

from __future__ import annotations

import sys
import threading
import traceback
from types import TracebackType


def print_exception(context: str, exc: BaseException) -> None:
    """Print a labeled traceback to stderr so GUI failures are visible."""
    print(f"[hexapod-companion] {context}: {exc!r}", file=sys.stderr, flush=True)
    traceback.print_exception(type(exc), exc, exc.__traceback__, file=sys.stderr)


def install_exception_hooks() -> None:
    """Ensure uncaught GUI/thread exceptions are printed to the terminal."""

    def excepthook(
        exc_type: type[BaseException],
        exc: BaseException,
        tb: TracebackType | None,
    ) -> None:
        print(f"[hexapod-companion] uncaught exception: {exc!r}", file=sys.stderr)
        traceback.print_exception(exc_type, exc, tb, file=sys.stderr)

    def thread_excepthook(args: threading.ExceptHookArgs) -> None:
        name = args.thread.name if args.thread else "unknown-thread"
        print(
            f"[hexapod-companion] uncaught thread exception in {name}: "
            f"{args.exc_value!r}",
            file=sys.stderr,
        )
        traceback.print_exception(
            args.exc_type,
            args.exc_value,
            args.exc_traceback,
            file=sys.stderr,
        )

    sys.excepthook = excepthook
    threading.excepthook = thread_excepthook
