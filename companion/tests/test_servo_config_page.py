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


def test_geometry_edits_stage_into_the_shared_config(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    page.link_spins["coxa"].setValue(57.25)
    page.home_radius_spin.setValue(126.5)
    page.geometry_table.item(0, 1).setText("-70.50")
    page.geometry_table.item(0, 4).setText("136.25")

    edited, err = page._read_table()

    assert err is None
    assert edited.links.coxa_cmm == 5725
    assert edited.geometry.home_radius_cmm == 12650
    assert edited.legs[0].mount_x_dmm == -705
    assert edited.legs[0].mount_yaw_cdeg == 13625
    page._show_diff()
    assert "links.coxa_cmm" in page.diff_text.toPlainText()
    assert "leg[0].mount_yaw_cdeg" in page.diff_text.toPlainText()


def test_sensor_calibration_edits_stage_into_the_shared_config(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    page.sensor_table.item(0, 1).setText("123456")
    page.sensor_table.item(0, 2).setText("100")
    page.sensor_table.item(0, 3).setText("200")
    page.sensor_table.item(0, 4).setText("300")
    enabled = page.sensor_table.cellWidget(0, 5)
    assert enabled is not None
    enabled.setChecked(True)

    edited, err = page._read_table()

    assert err is None
    assert edited.feet[0].pressure_baseline == 123456
    assert edited.feet[0].near_thresh == 100
    assert edited.feet[0].touch_thresh == 200
    assert edited.feet[0].load_thresh == 300
    assert edited.feet[0].enabled == 1
    page._show_diff()
    assert "foot[0].pressure_baseline" in page.diff_text.toPlainText()


def test_sensor_calibration_rejects_invalid_enabled_thresholds(qtbot) -> None:
    service, page = _make_page(qtbot)
    _load_default(service, page)
    enabled = page.sensor_table.cellWidget(0, 5)
    assert enabled is not None
    enabled.setChecked(True)
    page.sensor_table.item(0, 2).setText("100")
    page.sensor_table.item(0, 3).setText("300")
    page.sensor_table.item(0, 4).setText("200")

    _edited, err = page._read_table()

    assert err is not None and "load threshold" in err


def test_sensor_baseline_capture_uses_latest_raw_telemetry(qtbot) -> None:
    from hexapod_protocol import telemetry as tlm

    service, page = _make_page(qtbot)
    _load_default(service, page)
    service.telemetry.emit(
        int(tlm.StreamId.I2C_SENSORS_RAW),
        tlm.I2cSensorsRawTelemetry(
            feet=[
                tlm.FootRaw(proximity=100 + foot, pressure_raw=1000 + foot)
                for foot in range(tlm.NUM_FEET)
            ]
        ),
    )

    page._capture_live_baselines()

    assert page.sensor_table.item(0, 1).text() == "1000"
    assert page.sensor_table.item(5, 1).text() == "1005"
    assert "Copied 6" in page.sensor_capture_lbl.text()


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
    page.link_spins["femur"].setValue(66.75)
    page.geometry_table.item(1, 2).setText("-93.40")
    page.sensor_table.item(0, 1).setText("123456")
    page.sensor_table.item(0, 2).setText("100")
    page.sensor_table.item(0, 3).setText("200")
    page.sensor_table.item(0, 4).setText("300")
    page.sensor_table.cellWidget(0, 5).setChecked(True)

    # Export the calibrated local config straight to a file, bypassing the
    # file dialog while preserving the same dataclass JSON structure.
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
    assert restored.links.femur_cmm == 6675
    assert restored.legs[1].mount_y_dmm == -934
    assert restored.feet[0].pressure_baseline == 123456
    assert restored.feet[0].load_thresh == 300
    assert restored.feet[0].enabled == 1
    # Encoding the restored config must match the original wire payload.
    assert cfg.encode_robot_config(restored) == cfg.encode_robot_config(edited)


def test_actions_safe_when_disconnected(qtbot) -> None:
    service, page = _make_page(qtbot)
    errors = []
    service.error.connect(lambda m: errors.append(m))
    service.load_config()
    service.validate_config()
    service.commit_config()
    assert any("config" in e for e in errors)


def test_config_mutation_controls_require_maintenance_lock(qtbot) -> None:
    service, page = _make_page(qtbot)
    for button in (page.reset_btn, page.stage_btn, page.commit_btn):
        assert not button.isEnabled()

    service.connected.emit(True)
    for button in (page.reset_btn, page.stage_btn, page.commit_btn):
        assert not button.isEnabled()
        assert button.toolTip()

    service.maint_lock_changed.emit(True, 42)
    for button in (page.reset_btn, page.stage_btn, page.commit_btn):
        assert button.isEnabled()

    service.maint_lock_changed.emit(False, 0)
    for button in (page.reset_btn, page.stage_btn, page.commit_btn):
        assert not button.isEnabled()
