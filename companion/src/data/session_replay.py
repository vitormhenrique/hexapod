"""Session replay: read back a recorded session without hardware.

Pairs with :mod:`data.session_logger`. Reads the manifest, decoded telemetry
and event JSONL, and (losslessly) re-decodes ``raw_frames.bin`` through the
protocol stack so a recorded session can drive plots/UI exactly like a live
link. Pure stdlib + ``hexapod_protocol``; never touches a serial port.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Optional

from hexapod_protocol import MsgType, decode_frame_body
from hexapod_protocol import telemetry as tlm

from .session_logger import iter_raw_frames


@dataclass
class DecodedFrame:
    """One re-decoded raw frame from ``raw_frames.bin``."""

    host_time_ns: int
    msg_type: int
    msg_id: int
    seq: int
    timestamp_ms: int
    payload: bytes
    stream: Optional[str] = None  # stream name when this is a telemetry frame
    record: Any = None  # decoded telemetry record when decodable


class SessionReplay:
    """Read-only view over a recorded session directory."""

    def __init__(self, session_dir: Path | str) -> None:
        self.dir = Path(session_dir)
        if not self.dir.is_dir():
            raise FileNotFoundError(f"not a session directory: {self.dir}")

    # --- manifest ---------------------------------------------------------

    @property
    def meta(self) -> dict:
        """The ``session.json`` manifest as a dict."""
        return json.loads((self.dir / "session.json").read_text(encoding="utf-8"))

    # --- decoded JSONL sinks ---------------------------------------------

    def iter_records(self) -> Iterator[dict]:
        """Yield decoded telemetry rows from ``telemetry.jsonl``."""
        yield from self._iter_jsonl("telemetry.jsonl")

    def iter_events(self) -> Iterator[dict]:
        """Yield operator marks / fault rows from ``events.jsonl``."""
        yield from self._iter_jsonl("events.jsonl")

    def _iter_jsonl(self, name: str) -> Iterator[dict]:
        path = self.dir / name
        if not path.exists():
            return
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    yield json.loads(line)

    # --- lossless raw replay ---------------------------------------------

    def iter_raw_frames(self) -> Iterator[tuple[int, bytes]]:
        """Yield ``(host_time_ns, frame_bytes)`` straight from disk."""
        yield from iter_raw_frames(self.dir / "raw_frames.bin")

    def iter_decoded_frames(self) -> Iterator[DecodedFrame]:
        """Re-decode every raw frame through the protocol stack.

        Telemetry frames additionally carry their stream name and decoded
        record. Frames that fail to decode are skipped (mirrors the live
        reader, which drops corrupt frames).
        """
        for host_ns, frame in self.iter_raw_frames():
            if len(frame) < 2 or frame[0] != 0x00 or frame[-1] != 0x00:
                continue
            try:
                header, payload = decode_frame_body(frame[1:-1])
            except Exception:
                continue
            df = DecodedFrame(
                host_time_ns=host_ns,
                msg_type=header.msg_type,
                msg_id=header.msg_id,
                seq=header.seq,
                timestamp_ms=header.timestamp_ms,
                payload=payload,
            )
            if header.msg_type == MsgType.TELEMETRY:
                stream_id = header.msg_id - 0x40  # MSG_TELEMETRY_BASE
                df.stream = tlm.STREAM_NAMES.get(stream_id)
                try:
                    df.record = tlm.decode_stream(stream_id, payload)
                except Exception:
                    df.record = None
            yield df
