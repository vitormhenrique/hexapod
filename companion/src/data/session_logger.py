"""Session logging for the companion app.

Writes a self-describing session directory (AGENTS.md 7.5):

    data/sessions/<ts>_<name>/
        session.json        manifest (start/stop, firmware, notes)
        raw_frames.bin       length-prefixed raw wire frames (lossless)
        telemetry.jsonl      decoded long-form records (one JSON per line)
        events.jsonl         operator marks / fault events

Parquet export is layered on top later; JSONL keeps the logger dependency-free
and replayable without hardware.
"""

from __future__ import annotations

import json
import struct
import threading
import time
from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


def _now_ns() -> int:
    return time.time_ns()


def _record_to_dict(record: Any) -> Any:
    if is_dataclass(record) and not isinstance(record, type):
        return {k: _record_to_dict(v) for k, v in asdict(record).items()}
    if isinstance(record, list):
        return [_record_to_dict(v) for v in record]
    return record


@dataclass
class SessionMeta:
    session_id: str
    started_utc: str
    robot_name: str = "hexapod"
    firmware: Optional[dict] = None
    notes: str = ""
    stopped_utc: Optional[str] = None
    frame_count: int = 0
    record_count: int = 0
    event_count: int = 0


@dataclass
class SessionLogger:
    """Append-only session recorder. Thread-safe across the three sinks."""

    out_dir: Path
    robot_name: str = "hexapod"
    firmware: Optional[dict] = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def __post_init__(self) -> None:
        ts = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
        self.session_id = f"{ts}_{self.robot_name}"
        self.dir = Path(self.out_dir) / self.session_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self._raw = open(self.dir / "raw_frames.bin", "wb")
        self._tlm = open(self.dir / "telemetry.jsonl", "w", encoding="utf-8")
        self._evt = open(self.dir / "events.jsonl", "w", encoding="utf-8")
        self._meta = SessionMeta(
            session_id=self.session_id,
            started_utc=datetime.now(timezone.utc).isoformat(),
            robot_name=self.robot_name,
            firmware=self.firmware,
        )
        self._write_manifest()

    # --- sinks ------------------------------------------------------------

    def log_raw_frame(self, frame: bytes) -> None:
        """Append a raw wire frame, length-prefixed (u32 host-ns + u16 len)."""
        with self._lock:
            self._raw.write(struct.pack("<QH", _now_ns(), len(frame)))
            self._raw.write(frame)
            self._meta.frame_count += 1

    def log_record(self, stream: str, record: Any, robot_time_ms: int = 0) -> None:
        """Append a decoded telemetry record as one JSON line."""
        row = {
            "host_time_ns": _now_ns(),
            "robot_time_ms": robot_time_ms,
            "stream": stream,
            "data": _record_to_dict(record),
        }
        with self._lock:
            self._tlm.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._meta.record_count += 1

    def mark_event(self, kind: str, detail: str = "", **extra: Any) -> None:
        """Append an operator mark or fault event."""
        row = {"host_time_ns": _now_ns(), "kind": kind, "detail": detail, **extra}
        with self._lock:
            self._evt.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._meta.event_count += 1

    # --- manifest / close -------------------------------------------------

    def _write_manifest(self) -> None:
        (self.dir / "session.json").write_text(
            json.dumps(_record_to_dict(self._meta), indent=2), encoding="utf-8"
        )

    def set_notes(self, notes: str) -> None:
        with self._lock:
            self._meta.notes = notes

    def close(self) -> None:
        with self._lock:
            self._meta.stopped_utc = datetime.now(timezone.utc).isoformat()
            for f in (self._raw, self._tlm, self._evt):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
            self._write_manifest()

    def __enter__(self) -> "SessionLogger":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def iter_raw_frames(path: Path):
    """Yield ``(host_time_ns, frame_bytes)`` from a raw_frames.bin file."""
    with open(path, "rb") as f:
        header = f.read(10)
        while len(header) == 10:
            ts, length = struct.unpack("<QH", header)
            frame = f.read(length)
            if len(frame) != length:
                break
            yield ts, frame
            header = f.read(10)
