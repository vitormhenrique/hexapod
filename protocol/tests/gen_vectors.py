"""Generate shared golden test vectors for the wire protocol.

Run from the repo root:  python protocol/tests/gen_vectors.py
Writes protocol/tests/vectors/frames.json. These vectors are consumed by both
the Python tests and the firmware native Unity test to guarantee the two
implementations agree byte-for-byte.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "protocol" / "python"))

from hexapod_protocol import crc16, cobs_encode, Header, encode_frame  # noqa: E402

VECTORS_PATH = Path(__file__).parent / "vectors" / "frames.json"


def _hex(b: bytes) -> str:
    return b.hex()


def build() -> dict:
    # --- standalone CRC16 vectors (CRC-16/CCITT-FALSE) ---
    crc_cases = [
        {"input": _hex(b""), "crc": crc16(b"")},
        {"input": _hex(b"123456789"), "crc": crc16(b"123456789")},
        {"input": _hex(bytes([0x00])), "crc": crc16(bytes([0x00]))},
        {"input": _hex(bytes(range(16))), "crc": crc16(bytes(range(16)))},
    ]

    # --- standalone COBS vectors ---
    cobs_inputs = [
        b"",
        bytes([0x00]),
        bytes([0x00, 0x00]),
        bytes([0x11, 0x22, 0x00, 0x33]),
        bytes([0x01, 0x02, 0x03, 0x04]),
        bytes(range(1, 255)),          # 254 non-zero bytes -> forces 0xFF block
        bytes(range(1, 256)),          # 255 non-zero bytes
    ]
    cobs_cases = [
        {"decoded": _hex(d), "encoded": _hex(cobs_encode(d))} for d in cobs_inputs
    ]

    # --- full frame vectors ---
    frame_specs = [
        # (msg_type, msg_id, flags, seq, timestamp_ms, payload)
        (0, 0, 0, 0, 0, b""),
        (2, 0x10, 0x01, 0x1234, 0x00ABCDEF, bytes([0xDE, 0xAD, 0xBE, 0xEF])),
        (0, 0x05, 0x00, 0x0001, 1000, bytes([0x00, 0x00, 0x00])),
        (1, 0x7F, 0xFF, 0xFFFF, 0xFFFFFFFF, bytes(range(0, 64))),
    ]
    frame_cases = []
    for msg_type, msg_id, flags, seq, ts, payload in frame_specs:
        h = Header(
            msg_type=msg_type,
            msg_id=msg_id,
            flags=flags,
            seq=seq,
            timestamp_ms=ts,
        )
        wire = encode_frame(h, payload)
        frame_cases.append(
            {
                "msg_type": msg_type,
                "msg_id": msg_id,
                "flags": flags,
                "seq": seq,
                "timestamp_ms": ts,
                "payload": _hex(payload),
                "wire": _hex(wire),
            }
        )

    return {
        "crc16_ccitt_false": crc_cases,
        "cobs": cobs_cases,
        "frames": frame_cases,
    }


def main() -> None:
    data = build()
    VECTORS_PATH.parent.mkdir(parents=True, exist_ok=True)
    VECTORS_PATH.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {VECTORS_PATH}")


if __name__ == "__main__":
    main()
