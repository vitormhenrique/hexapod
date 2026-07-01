"""Session event log and plot markers (nxi.2).

A small, UI-independent model for the events that annotate a telemetry timeline:
gait changes, faults, config commits, connect/disconnect, and free-form operator
notes. The Plot Workbench overlays these as vertical markers; the session logger
persists them to ``events.jsonl``. Pure stdlib -- no Qt, no serial port.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

# Marker colour per event kind (Dracula-ish); unknown kinds fall back to comment.
EVENT_KIND_COLORS = {
    "gait": "#50fa7b",
    "fault": "#ff5555",
    "estop": "#ff5555",
    "error": "#ffb86c",
    "commit": "#bd93f9",
    "note": "#f1fa8c",
    "connect": "#8be9fd",
    "disconnect": "#6272a4",
    "version": "#ff79c6",
}
_DEFAULT_COLOR = "#6272a4"


def color_for(kind: str) -> str:
    """Marker colour for an event kind."""
    return EVENT_KIND_COLORS.get(kind, _DEFAULT_COLOR)


@dataclass(frozen=True)
class EventMarker:
    """One annotation on the plot timeline."""

    t_s: float  # seconds on the active plot timeline
    kind: str
    detail: str = ""

    @property
    def color(self) -> str:
        return color_for(self.kind)

    @property
    def label(self) -> str:
        return self.kind if not self.detail else f"{self.kind}: {self.detail}"


class EventLog:
    """An ordered collection of :class:`EventMarker`."""

    def __init__(self) -> None:
        self._markers: list[EventMarker] = []

    def __len__(self) -> int:
        return len(self._markers)

    def add(self, kind: str, detail: str, t_s: float) -> EventMarker:
        marker = EventMarker(t_s=float(t_s), kind=kind, detail=detail)
        self._markers.append(marker)
        return marker

    def markers(self) -> list[EventMarker]:
        return list(self._markers)

    def clear(self) -> None:
        self._markers.clear()

    @classmethod
    def from_session_events(
        cls, rows: Iterable[dict], t0_ns: Optional[int] = None
    ) -> "EventLog":
        """Build a log from recorded ``events.jsonl`` rows.

        Each row carries ``host_time_ns``, ``kind`` and ``detail``. ``t_s`` is
        computed relative to ``t0_ns`` (the session's first frame host time) so
        markers share the replay timeline with the telemetry curves. Rows
        without a host timestamp or kind are skipped.
        """
        log = cls()
        for row in rows:
            host_ns = row.get("host_time_ns")
            kind = row.get("kind")
            if host_ns is None or not kind:
                continue
            base = t0_ns if t0_ns is not None else host_ns
            log.add(kind, row.get("detail", ""), (host_ns - base) / 1e9)
        return log
