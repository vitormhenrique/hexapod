"""Headless UI tests for the Servo Map & Config page.

The page is built off-screen with a disconnected :class:`ConnectionService`.
Config load/stage/commit results are emitted directly as Qt signals so the page
reactions, table editing, diff, and JSON export/import can be exercised without
hardware.
"""

from __future__ import annotations

import json
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from hexapod_protocol import api
from hexapod_protocol import config as cfg

pytest.importorskip("PySide6")


def _make_page(qtbot):
    from services import ConnectionService
    from ui.pages import ServoConfigPage

    service = ConnectionService()
    page = ServoConfigPage(service)
    qtbot.addWidget(page)
    return service, page


def _load_default(service, page):
    config = cfg.default_robot_config()
    service.config_summary.emit(
        cfg.ConfigSummary(
            schema_version=config.schema_version,
            payload_size=cfg.CONFIG_PAYLOAD_SIZE,
            block_max=cfg.CFG_BLOCK_MAX,
            persistent=True,
            staged_valid=True,
            feature_defaults=config.feature_defaults,
            robot_name=config.robot_name,
        )
    )
    service.config_loaded.emit(config)
    return config


def test_config_load_populates_table(qtbot) -> None:
    service, page = _make_page(qtbot)
    config = _load_default(service, page)
    assert page.table.rowCount() == len(config.servos)
    assert page.table.item(0, 0).text() == str(config.servos[0].id)
    assert "persistent" in page.persist_lbl.text()
    assert page.name_edit.text() == config.robot_name


def test_edit_and_diff_reports_changes(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    # Edit servo 0 trim (column 4) and the robot name.
    page.table.item(0, 4).setText("33")
    page.name_edit.setText("Edited")
    page._show_diff()
    text = page.diff_text.toPlainText()
    assert "servo[0].trim_ticks" in text
    assert "robot_name" in text


def test_diff_rejects_bad_cell(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    page.table.item(0, 4).setText("not-a-number")
    page._show_diff()
    assert "cannot diff" in page.diff_text.toPlainText()


def test_read_table_range_validation(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    # Sign must be +1/-1.
    page.table.item(0, 3).setText("2")
    _config, err = page._read_table()
    assert err is not None and "sign" in err


def test_read_table_min_max_ordering(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    page.table.item(0, 5).setText("3000")  # min
    page.table.item(0, 6).setText("2000")  # max
    _config, err = page._read_table()
    assert err is not None and "min tick" in err


def test_staged_ok_updates_base(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    page.table.item(0, 4).setText("9")
    edited, err = page._read_table()
    assert err is None
    page._edited = edited
    service.config_staged.emit(True)
    assert page._loaded is edited
    assert "staged ok" in page.action_lbl.text()


def test_staged_failure_message(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    service.config_staged.emit(False)
    assert "stage failed" in page.action_lbl.text()


def test_config_result_routing(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    service.config_result.emit("validate", api.CfgResult(api.CFG_OK))
    assert "validate: ok" in page.action_lbl.text()
    service.config_result.emit("commit", api.CfgResult(api.CFG_COMMIT_FAILED))
    assert "commit: failed" in page.action_lbl.text()


def test_export_import_roundtrip(qtbot, tmp_path) -> None:
    service, page = _make_page(qtbot)
    config = _load_default(service, page)
    # Export the current (default) config straight to a file, bypassing the
    # file dialog by writing what _export would serialize.
    import dataclasses

    edited, err = page._read_table()
    assert err is None
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(dataclasses.asdict(edited)))

    # Reconstruct via the page's importer and confirm it round-trips.
    data = json.loads(path.read_text())
    restored = page._config_from_dict(data)
    assert restored.robot_name == config.robot_name
    assert len(restored.servos) == len(config.servos)
    assert restored.servos[3].id == config.servos[3].id
    # Encoding the restored config must match the original wire payload.
    assert cfg.encode_robot_config(restored) == cfg.encode_robot_config(config)


def test_actions_safe_when_disconnected(qtbot) -> None:
    service, page = _make_page(qtbot)
    errors = []
    service.error.connect(lambda m: errors.append(m))
    service.load_config()
    service.validate_config()
    service.commit_config()
    assert any("config" in e for e in errors)
