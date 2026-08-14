"""Tests for the offline Qwiic OpenLog CONFIG.TXT journal codec."""

from __future__ import annotations

from data.openlog_config import (
    HEADER_SIZE,
    OpenLogConfigError,
    append_config_record,
    encode_record,
    parse_openlog_config,
)
from hexapod_protocol import config as cfg


PREFIX = b"42,26,3,0\r\ni2c_address,escape,esc#,mode\n"


def test_parses_prefix_and_selects_newest_record() -> None:
    first = cfg.default_robot_config()
    second = cfg.default_robot_config()
    second.robot_name = "Bench"
    raw = PREFIX + (encode_record(first, 5) + encode_record(second, 6)).hex().upper().encode("ascii")

    journal = parse_openlog_config(raw)

    assert journal.prefix_bytes == len(PREFIX) // 2
    assert [record.sequence for record in journal.records] == [5, 6]
    assert journal.newest.config.robot_name == "Bench"
    assert journal.next_sequence == 7


def test_append_preserves_history_and_round_trips_edited_config() -> None:
    source = cfg.default_robot_config()
    journal = parse_openlog_config(PREFIX + encode_record(source, 6).hex().upper().encode("ascii"))
    edited = cfg.default_robot_config()
    edited.robot_name = "Edited"
    edited.servos[0].trim_ticks = 24

    rendered = append_config_record(journal, edited)
    reloaded = parse_openlog_config(rendered)

    assert rendered.startswith(journal.raw_text)
    assert len(reloaded.records) == 2
    assert reloaded.newest.sequence == 7
    assert reloaded.newest.config.robot_name == "Edited"
    assert reloaded.newest.config.servos[0].trim_ticks == 24


def test_rejects_odd_length_card_file() -> None:
    try:
        parse_openlog_config(b"ABC")
    except OpenLogConfigError as exc:
        assert "odd" in str(exc)
    else:
        raise AssertionError("expected odd raw CONFIG.TXT length to be rejected")


def test_header_size_matches_serialized_record() -> None:
    record = encode_record(cfg.default_robot_config(), 1)
    assert len(record) == HEADER_SIZE + cfg.CONFIG_PAYLOAD_SIZE + 8