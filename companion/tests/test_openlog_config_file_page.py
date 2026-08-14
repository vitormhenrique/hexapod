"""Headless tests for the offline OpenLog CONFIG.TXT editor page."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from hexapod_protocol import config as cfg

from data.openlog_config import encode_record, load_openlog_config, parse_openlog_config

pytest.importorskip("PySide6")


PREFIX = b"42,26,3,0\r\ni2c_address,escape,esc#,mode\n"


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import OpenLogConfigFilePage

    page = OpenLogConfigFilePage(ConnectionService())
    qtbot.addWidget(page)
    return page


def _journal():
    config = cfg.default_robot_config()
    return parse_openlog_config(PREFIX + encode_record(config, 6).hex().upper().encode("ascii"))


def _two_record_file(path):
    older = cfg.default_robot_config()
    older.robot_name = "Older"
    newer = cfg.default_robot_config()
    newer.robot_name = "Newer"
    path.write_bytes(
        PREFIX
        + (encode_record(older, 5) + encode_record(newer, 6))
        .hex()
        .upper()
        .encode("ascii")
    )
    return older, newer


def test_load_journal_populates_human_readable_editor(qtbot) -> None:
    page = _make_page(qtbot)
    page._load_journal(_journal())

    assert page.record_table.rowCount() == 1
    assert page.name_edit.text() == "HexNav"
    assert page.servo_table.rowCount() == cfg.NUM_SERVOS
    assert page.leg_table.rowCount() == cfg.NUM_LEGS
    assert page.rc_table.rowCount() == cfg.NUM_RC_ANALOG_INPUTS
    assert page.foot_table.rowCount() == cfg.NUM_FOOT_SENSORS
    assert page.append_btn.isEnabled() is False  # no physical source path
    assert page.save_as_btn.isEnabled()


def test_editor_builds_edited_config_and_appends_valid_record(qtbot) -> None:
    page = _make_page(qtbot)
    journal = _journal()
    page._load_journal(journal)
    page.name_edit.setText("Bench")
    page.servo_table.item(0, 4).setText("24")

    edited = page._read_editor()
    reloaded = parse_openlog_config(
        PREFIX + (encode_record(cfg.default_robot_config(), 6)).hex().upper().encode("ascii")
    )
    from data.openlog_config import append_config_record

    after = parse_openlog_config(append_config_record(reloaded, edited))
    assert after.newest.sequence == 7
    assert after.newest.config.robot_name == "Bench"
    assert after.newest.config.servos[0].trim_ticks == 24


def test_editor_reports_invalid_servo_sign(qtbot) -> None:
    page = _make_page(qtbot)
    page._load_journal(_journal())
    page.servo_table.item(0, 3).setText("0")

    with pytest.raises(ValueError, match="sign"):
        page._read_editor()


def test_save_writes_a_reloadable_next_record(qtbot, tmp_path) -> None:
    source = tmp_path / "CONFIG.TXT"
    source.write_bytes(PREFIX + encode_record(cfg.default_robot_config(), 6).hex().upper().encode("ascii"))
    page = _make_page(qtbot)
    page._load_journal(load_openlog_config(source))
    page.name_edit.setText("CardEdit")

    page._save_to_path(source)

    reloaded = load_openlog_config(source)
    assert reloaded.newest.sequence == 7
    assert reloaded.newest.config.robot_name == "CardEdit"
    assert "readback verified" in page.action_lbl.text()


def test_activate_selected_history_entry_makes_it_firmware_active(qtbot, tmp_path, monkeypatch) -> None:
    source = tmp_path / "CONFIG.TXT"
    older, _newer = _two_record_file(source)
    page = _make_page(qtbot)
    page._load_journal(load_openlog_config(source))
    page.record_table.selectRow(0)
    monkeypatch.setattr(page, "_confirm", lambda *_args: True)

    page._activate_selected_record()

    reloaded = load_openlog_config(source)
    assert [record.sequence for record in reloaded.records] == [5, 6, 7]
    assert reloaded.newest.config.robot_name == older.robot_name
    assert "activated sequence 5 as sequence 7" in page.action_lbl.text()


def test_delete_selected_history_entry_preserves_firmware_valid_record(qtbot, tmp_path, monkeypatch) -> None:
    source = tmp_path / "CONFIG.TXT"
    older, _newer = _two_record_file(source)
    page = _make_page(qtbot)
    page._load_journal(load_openlog_config(source))
    page.record_table.selectRow(1)
    monkeypatch.setattr(page, "_confirm", lambda *_args: True)

    page._delete_selected_record()

    reloaded = load_openlog_config(source)
    assert [record.sequence for record in reloaded.records] == [5]
    assert reloaded.newest.config.robot_name == older.robot_name
    assert "firmware now boots sequence 5" in page.action_lbl.text()