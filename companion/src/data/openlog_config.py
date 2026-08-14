"""Read and append the Qwiic OpenLog ``CONFIG.TXT`` config journal.

The firmware stores binary journal records as uppercase ASCII hex because the
OpenLog file transport is text-oriented. The physical file can contain an
arbitrary damaged/non-hex prefix; firmware decodes each invalid nibble as zero
and scans forward until it finds a CRC-valid record. This module preserves that
behavior so a file edited on the companion can be copied back to the card.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct

from hexapod_protocol import config as cfg
from hexapod_protocol.crc16 import crc16

CONFIG_MAGIC = 0x48455843  # ``CXEH`` bytes on disk, little-endian uint32.
CONFIG_VERSION = 1
COMMIT_MAGIC = 0x54494D43  # ``CMIT`` bytes on disk, little-endian uint32.
HEADER_SIZE = 16
TRAILER_SIZE = 8
MAX_PAYLOAD_SIZE = 2032


class OpenLogConfigError(ValueError):
    """The card file cannot safely be parsed or written as a config journal."""


@dataclass(frozen=True)
class JournalRecord:
    """One complete CRC-valid config record discovered in ``CONFIG.TXT``."""

    offset: int
    sequence: int
    payload: bytes
    config: cfg.RobotConfig

    @property
    def end_offset(self) -> int:
        return self.offset + HEADER_SIZE + len(self.payload) + TRAILER_SIZE


@dataclass(frozen=True)
class OpenLogConfigJournal:
    """Decoded card file plus every valid record it contains."""

    path: Path | None
    raw_text: bytes
    decoded_bytes: bytes
    records: tuple[JournalRecord, ...]

    @property
    def newest(self) -> JournalRecord:
        if not self.records:
            raise OpenLogConfigError("CONFIG.TXT contains no valid config record")
        return max(self.records, key=lambda record: record.sequence)

    @property
    def next_sequence(self) -> int:
        return self.newest.sequence + 1

    @property
    def prefix_bytes(self) -> int:
        """Decoded bytes before the first valid record (usually transport text)."""
        return min(record.offset for record in self.records) if self.records else 0


def _nibble(value: int) -> int:
    if 48 <= value <= 57:  # 0..9
        return value - 48
    if 65 <= value <= 70:  # A..F; mirrors QwiicConfigFile exactly.
        return value - 65 + 10
    return 0


def decode_openlog_hex(raw_text: bytes) -> bytes:
    """Mirror ``QwiicConfigFile::read`` over the complete raw text file."""
    if len(raw_text) % 2:
        raise OpenLogConfigError(
            "CONFIG.TXT has an odd number of bytes; firmware would reject it"
        )
    return bytes(
        (_nibble(raw_text[index]) << 4) | _nibble(raw_text[index + 1])
        for index in range(0, len(raw_text), 2)
    )


def _parse_record(data: bytes, offset: int) -> JournalRecord | None:
    """Return a complete valid record at ``offset``, or ``None`` if invalid."""
    if len(data) - offset < HEADER_SIZE + TRAILER_SIZE:
        return None
    header = data[offset : offset + HEADER_SIZE]
    magic, version, sequence, length, payload_crc, header_crc = struct.unpack(
        "<IHIHHH", header
    )
    if (
        magic != CONFIG_MAGIC
        or version != CONFIG_VERSION
        or length > MAX_PAYLOAD_SIZE
        or crc16(header[:14]) != header_crc
    ):
        return None
    payload_start = offset + HEADER_SIZE
    payload_end = payload_start + length
    trailer_end = payload_end + TRAILER_SIZE
    if trailer_end > len(data):
        return None
    payload = data[payload_start:payload_end]
    trailer_magic, trailer_sequence = struct.unpack(
        "<II", data[payload_end:trailer_end]
    )
    if crc16(payload) != payload_crc or (
        trailer_magic != COMMIT_MAGIC or trailer_sequence != sequence
    ):
        return None
    try:
        config = cfg.decode_robot_config(payload)
    except cfg.ConfigDecodeError:
        return None
    return JournalRecord(offset, sequence, payload, config)


def parse_openlog_config(raw_text: bytes, path: Path | None = None) -> OpenLogConfigJournal:
    """Parse all valid records from a raw OpenLog ``CONFIG.TXT`` file."""
    data = decode_openlog_hex(raw_text)
    records: list[JournalRecord] = []
    offset = 0
    while len(data) - offset >= HEADER_SIZE + TRAILER_SIZE:
        record = _parse_record(data, offset)
        if record is None:
            offset += 1
            continue
        records.append(record)
        offset = record.end_offset
    if not records:
        raise OpenLogConfigError("CONFIG.TXT contains no valid CRC-protected record")
    return OpenLogConfigJournal(path, raw_text, data, tuple(records))


def load_openlog_config(path: str | Path) -> OpenLogConfigJournal:
    """Load and parse a card file without modifying it."""
    source = Path(path)
    try:
        raw_text = source.read_bytes()
    except OSError as exc:
        raise OpenLogConfigError(f"cannot read {source}: {exc}") from exc
    return parse_openlog_config(raw_text, source)


def encode_record(config: cfg.RobotConfig, sequence: int) -> bytes:
    """Build one firmware-compatible binary record for an edited config."""
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise OpenLogConfigError("record sequence must fit in uint32")
    validation_errors = cfg.validate_robot_config(config)
    if validation_errors:
        raise OpenLogConfigError(validation_errors[0])
    payload = cfg.encode_robot_config(config)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise OpenLogConfigError("serialized config exceeds the OpenLog limit")
    payload_crc = crc16(payload)
    header = bytearray(
        struct.pack(
            "<IHIHHH",
            CONFIG_MAGIC,
            CONFIG_VERSION,
            sequence,
            len(payload),
            payload_crc,
            0,
        )
    )
    struct.pack_into("<H", header, 14, crc16(header[:14]))
    return bytes(header) + payload + struct.pack("<II", COMMIT_MAGIC, sequence)


def append_config_record(journal: OpenLogConfigJournal, config: cfg.RobotConfig) -> bytes:
    """Return the complete text file with an appended next valid record.

    The pre-existing prefix and append-only history remain byte-for-byte
    unchanged. The new record is uppercase ASCII hex, exactly as the firmware's
    ``QwiicConfigFile::append`` emits it.
    """
    record = encode_record(config, journal.next_sequence)
    return journal.raw_text + record.hex().upper().encode("ascii")


def save_openlog_config(
    journal: OpenLogConfigJournal, config: cfg.RobotConfig, destination: str | Path
) -> Path:
    """Append an edited record and write a replacement card file."""
    target = Path(destination)
    try:
        target.write_bytes(append_config_record(journal, config))
    except OSError as exc:
        raise OpenLogConfigError(f"cannot write {target}: {exc}") from exc
    return target