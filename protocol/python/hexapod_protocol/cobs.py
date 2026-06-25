"""Consistent Overhead Byte Stuffing (COBS).

Encodes data so it contains no 0x00 bytes, letting 0x00 delimit frames. Matches
the firmware implementation byte-for-byte.
"""

from __future__ import annotations


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode ``data`` (output contains no 0x00 bytes)."""
    out = bytearray()
    code_idx = 0
    out.append(0)  # placeholder for the first code byte
    code = 1

    for byte in data:
        if byte == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1

    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS-decode ``data`` (which must not include the 0x00 delimiters).

    Raises ``ValueError`` on malformed input.
    """
    out = bytearray()
    read_idx = 0
    n = len(data)

    while read_idx < n:
        code = data[read_idx]
        if code == 0:
            raise ValueError("unexpected zero byte in COBS stream")
        if read_idx + code > n:
            raise ValueError("COBS code runs past end of buffer")
        read_idx += 1
        for _ in range(code - 1):
            out.append(data[read_idx])
            read_idx += 1
        if code != 0xFF and read_idx < n:
            out.append(0)

    return bytes(out)
