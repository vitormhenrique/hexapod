"""CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflection, xorout=0)."""

from __future__ import annotations

CRC16_INIT = 0xFFFF


def crc16(data: bytes, crc: int = CRC16_INIT) -> int:
    """Compute CRC-16/CCITT-FALSE over ``data``."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF
