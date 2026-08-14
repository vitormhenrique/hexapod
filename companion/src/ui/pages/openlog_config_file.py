"""Offline editor for the Qwiic OpenLog ``CONFIG.TXT`` journal."""

from __future__ import annotations

import copy
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from data.openlog_config import (
    OpenLogConfigError,
    OpenLogConfigJournal,
    append_config_record,
    load_openlog_config,
    parse_openlog_config,
    save_openlog_config,
)
from theme import DRACULA

from ui.pages import BasePage  # noqa: E402


class OpenLogConfigFilePage(BasePage):
    """View, edit, and append records to the physical card config journal."""

    title = "OpenLog Config File"
    subtitle = (
        "Open a card CONFIG.TXT, inspect its append-only history, edit the "
        "newest readable robot config, and append a firmware-compatible record."
    )

    SERVO_COLUMNS = ("ID", "Leg", "Joint", "Sign", "Trim", "Min tick", "Max tick")
    LEG_COLUMNS = ("Leg", "Mount X (mm)", "Mount Y (mm)", "Mount Z (mm)", "Yaw (deg)")
    FOOT_COLUMNS = ("Foot", "Baseline", "Near", "Touch", "Load", "Enabled")
    RC_COLUMNS = (
        "Input",
        "Source",
        "Type",
        "Min",
        "Center",
        "Max",
        "Reversed",
        "Deadband",
        "Expo",
        "Filter (ms)",
        "Debounce (ms)",
    )
    BODY_FIELDS = (
        ("Max forward", "max_forward_milli"),
        ("Max reverse", "max_reverse_milli"),
        ("Max lateral", "max_lateral_milli"),
        ("Max yaw", "max_yaw_milli"),
        ("Forward accel", "forward_accel_milli_per_s"),
        ("Forward decel", "forward_decel_milli_per_s"),
        ("Lateral accel", "lateral_accel_milli_per_s"),
        ("Lateral decel", "lateral_decel_milli_per_s"),
        ("Yaw accel", "yaw_accel_milli_per_s"),
        ("Yaw decel", "yaw_decel_milli_per_s"),
        ("Height rise (mm/s)", "height_rise_mm_per_s"),
        ("Height lower (mm/s)", "height_lower_mm_per_s"),
        ("Pose translation (mm/s)", "pose_translation_rate_mm_per_s"),
        ("Pose rotation (mrad/s)", "pose_rotation_rate_millirad_per_s"),
    )
    FEATURES = (
        ("Foot contact", "FEAT_FOOT_CONTACT"),
        ("Terrain leveling", "FEAT_TERRAIN_LEVELING"),
        ("Sensor polling", "FEAT_SENSOR_POLLING"),
        ("Passive pose stream", "FEAT_PASSIVE_POSE_STREAM"),
        ("Jetson control", "FEAT_JETSON_CONTROL"),
    )

    def build(self) -> None:
        from hexapod_protocol import config as cfg

        self._cfg = cfg
        self._journal: OpenLogConfigJournal | None = None
        self._record_sequence: int | None = None
        self._base_config = None

        self.content.addWidget(self._source_box())
        self.content.addWidget(self._record_history_box())
        self.content.addWidget(self._editor_box())
        self.content.addWidget(self._actions_box())

    # --- screen construction --------------------------------------------

    def _source_box(self) -> QGroupBox:
        box = QGroupBox("Card file")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(10)

        self.path_lbl = QLabel("No CONFIG.TXT loaded")
        self.path_lbl.setObjectName("MonoLabel")
        self.path_lbl.setWordWrap(True)
        form.addRow("Source", self.path_lbl)
        self.file_status_lbl = QLabel("Choose a file to inspect its records.")
        self.file_status_lbl.setObjectName("MonoLabel")
        self.file_status_lbl.setWordWrap(True)
        form.addRow("Journal", self.file_status_lbl)

        actions = QHBoxLayout()
        open_button = QPushButton("Open CONFIG.TXT")
        open_button.setProperty("accent", True)
        open_button.clicked.connect(self._open_file)
        actions.addWidget(open_button)
        actions.addStretch(1)
        form.addRow("File", self._wrap(actions))
        note = QLabel(
            "The editor preserves the card's transport prefix and append-only "
            "history. It writes one uppercase-hex journal record, matching "
            "the firmware Qwiic OpenLog format."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DRACULA.comment};")
        form.addRow(note)
        return box

    def _record_history_box(self) -> QGroupBox:
        box = QGroupBox("Valid journal records")
        lay = QVBoxLayout(box)
        self.record_table = QTableWidget(0, 5)
        self.record_table.setHorizontalHeaderLabels(
            ("Sequence", "Offset", "Schema", "Payload", "Selected")
        )
        self.record_table.verticalHeader().setVisible(False)
        self.record_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.record_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.record_table.setSelectionMode(QTableWidget.SingleSelection)
        self.record_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.record_table.setMinimumHeight(180)
        self.record_table.itemSelectionChanged.connect(self._select_record)
        lay.addWidget(self.record_table)
        return box

    def _editor_box(self) -> QGroupBox:
        box = QGroupBox("Human-readable configuration")
        lay = QVBoxLayout(box)
        self.tabs = QTabWidget()
        self.tabs.addTab(self._general_tab(), "General")
        self.tabs.addTab(self._servo_tab(), "Servos")
        self.tabs.addTab(self._geometry_tab(), "Geometry && Gait")
        self.tabs.addTab(self._input_tab(), "RC && Limits")
        self.tabs.addTab(self._sensor_tab(), "Foot Sensors")
        lay.addWidget(self.tabs)
        return box

    def _general_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        form.setContentsMargins(12, 12, 12, 12)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(10)
        self.name_edit = QLineEdit()
        self.name_edit.setMaxLength(15)
        form.addRow("Robot name", self.name_edit)
        self.schema_lbl = QLabel("--")
        self.schema_lbl.setObjectName("MonoLabel")
        form.addRow("Schema / payload", self.schema_lbl)
        self.sequence_lbl = QLabel("--")
        self.sequence_lbl.setObjectName("MonoLabel")
        form.addRow("Selected record", self.sequence_lbl)

        feature_row = QHBoxLayout()
        self.feature_checks: dict[str, QCheckBox] = {}
        for label, constant in self.FEATURES:
            check = QCheckBox(label)
            self.feature_checks[constant] = check
            feature_row.addWidget(check)
        feature_row.addStretch(1)
        form.addRow("Default features", self._wrap(feature_row))
        return tab

    def _servo_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(12, 12, 12, 12)
        self.servo_table = self._table(self.SERVO_COLUMNS, 320)
        lay.addWidget(self.servo_table)
        hint = QLabel(
            "Each row is one logical servo map record. Leg and joint are "
            "zero-based in firmware; the editor displays them as 1-based."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(hint)
        return tab

    def _geometry_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(12, 12, 12, 12)
        form = QFormLayout()
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(8)
        self.link_spins = {
            "coxa": self._mm_spin(0.01, 655.35),
            "femur": self._mm_spin(0.01, 655.35),
            "tibia": self._mm_spin(0.01, 655.35),
        }
        form.addRow("Coxa link", self.link_spins["coxa"])
        form.addRow("Femur link", self.link_spins["femur"])
        form.addRow("Tibia link", self.link_spins["tibia"])
        self.home_radius_spin = self._mm_spin(0.01, 655.35)
        self.home_foot_z_spin = self._mm_spin(-327.68, 327.67)
        self.coxa_lift_spin = self._mm_spin(0.0, 655.35)
        form.addRow("Home radius", self.home_radius_spin)
        form.addRow("Home foot Z", self.home_foot_z_spin)
        form.addRow("Coxa lift", self.coxa_lift_spin)
        self.body_height_spin = self._integer_spin(40, 160)
        self.stride_spin = self._integer_spin(0, 80)
        self.step_height_spin = self._integer_spin(0, 50)
        self.duty_spin = self._integer_spin(0, 255)
        self.speed_spin = self._integer_spin(0, 255)
        self.gait_combo = QComboBox()
        for gait_id, gait_name in self._cfg.GAIT_NAMES.items():
            self.gait_combo.addItem(gait_name.title(), gait_id)
        form.addRow("Body height (mm)", self.body_height_spin)
        form.addRow("Stride (mm)", self.stride_spin)
        form.addRow("Step height (mm)", self.step_height_spin)
        form.addRow("Duty", self.duty_spin)
        form.addRow("Speed", self.speed_spin)
        form.addRow("Gait", self.gait_combo)
        lay.addLayout(form)
        self.leg_table = self._table(self.LEG_COLUMNS, 220)
        lay.addWidget(self.leg_table)
        return tab

    def _input_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(12, 12, 12, 12)
        self.rc_table = self._table(self.RC_COLUMNS, 250)
        lay.addWidget(self.rc_table)
        type_hint = QLabel(
            "Input type: 0 = centered analog, 1 = unipolar analog, 2 = "
            "relative encoder. All RC values are stored as integers."
        )
        type_hint.setWordWrap(True)
        type_hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(type_hint)
        self.body_table = self._table(("Limit", "Value"), 310)
        lay.addWidget(self.body_table)
        return tab

    def _sensor_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(12, 12, 12, 12)
        self.foot_table = self._table(self.FOOT_COLUMNS, 250)
        lay.addWidget(self.foot_table)
        return tab

    @staticmethod
    def _table(columns: tuple[str, ...], minimum_height: int) -> QTableWidget:
        table = QTableWidget(0, len(columns))
        table.setHorizontalHeaderLabels(columns)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setMinimumHeight(minimum_height)
        return table

    @staticmethod
    def _integer_spin(minimum: int, maximum: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        return spin

    @staticmethod
    def _mm_spin(minimum: float, maximum: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(0.1)
        spin.setSuffix(" mm")
        return spin

    def _actions_box(self) -> QGroupBox:
        box = QGroupBox("Write journal record")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        self.append_btn = QPushButton("Append to loaded CONFIG.TXT")
        self.append_btn.setProperty("accent", True)
        self.append_btn.clicked.connect(self._append_to_loaded_file)
        self.save_as_btn = QPushButton("Save copy as CONFIG.TXT")
        self.save_as_btn.clicked.connect(self._save_copy)
        self.preview_btn = QPushButton("Preview new record")
        self.preview_btn.clicked.connect(self._preview_record)
        grid.addWidget(self.append_btn, 0, 0)
        grid.addWidget(self.save_as_btn, 0, 1)
        grid.addWidget(self.preview_btn, 0, 2)
        for column in range(3):
            grid.setColumnStretch(column, 1)
        self.action_lbl = QLabel("Open a card file to begin.")
        self.action_lbl.setObjectName("MonoLabel")
        self.action_lbl.setWordWrap(True)
        grid.addWidget(self.action_lbl, 1, 0, 1, 3)
        self.preview_text = QPlainTextEdit()
        self.preview_text.setReadOnly(True)
        self.preview_text.setMinimumHeight(100)
        self.preview_text.setPlaceholderText("Record preview appears here.")
        grid.addWidget(self.preview_text, 2, 0, 1, 3)
        self._apply_gates()
        return box

    @staticmethod
    def _wrap(layout) -> QWidget:
        widget = QWidget()
        widget.setLayout(layout)
        return widget

    # --- file / journal loading -----------------------------------------

    def _open_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open OpenLog CONFIG.TXT", "", "Config files (*.TXT *.txt);;All files (*)"
        )
        if not path:
            return
        try:
            journal = load_openlog_config(path)
        except OpenLogConfigError as exc:
            self.action_lbl.setText(f"open failed: {exc}")
            return
        self._load_journal(journal)

    def _load_journal(self, journal: OpenLogConfigJournal) -> None:
        self._journal = journal
        self.path_lbl.setText(str(journal.path) if journal.path else "in-memory journal")
        self.file_status_lbl.setText(
            f"{len(journal.records)} valid record(s), prefix {journal.prefix_bytes} decoded byte(s), "
            f"next sequence {journal.next_sequence}"
        )
        self.record_table.setRowCount(len(journal.records))
        newest = journal.newest.sequence
        for row, record in enumerate(journal.records):
            values = (
                record.sequence,
                record.offset,
                record.config.schema_version,
                len(record.payload),
                "newest" if record.sequence == newest else "",
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.record_table.setItem(row, column, item)
        newest_row = next(
            row for row, record in enumerate(journal.records) if record.sequence == newest
        )
        self.record_table.selectRow(newest_row)
        self._apply_gates()

    def _select_record(self) -> None:
        if self._journal is None:
            return
        selected = self.record_table.selectedItems()
        if not selected:
            return
        row = selected[0].row()
        if not 0 <= row < len(self._journal.records):
            return
        record = self._journal.records[row]
        self._record_sequence = record.sequence
        self._base_config = copy.deepcopy(record.config)
        self._populate_editor(self._base_config)
        self.sequence_lbl.setText(f"{record.sequence} (new writes use {self._journal.next_sequence})")
        self.action_lbl.setText(
            f"Loaded record {record.sequence}; edits append sequence {self._journal.next_sequence}."
        )
        self.preview_text.clear()
        self._apply_gates()

    # --- config <-> editable UI -----------------------------------------

    def _populate_editor(self, config) -> None:
        self.name_edit.setText(config.robot_name)
        self.schema_lbl.setText(f"v{config.schema_version} / {self._cfg.CONFIG_PAYLOAD_SIZE} bytes")
        for _label, constant in self.FEATURES:
            self.feature_checks[constant].setChecked(
                bool(config.feature_defaults & getattr(self._cfg, constant))
            )

        self.servo_table.setRowCount(len(config.servos))
        for row, servo in enumerate(config.servos):
            values = (
                servo.id,
                servo.leg + 1,
                servo.joint + 1,
                servo.sign,
                servo.trim_ticks,
                servo.min_tick,
                servo.max_tick,
            )
            self._set_row(self.servo_table, row, values)

        self.link_spins["coxa"].setValue(config.links.coxa_cmm / 100.0)
        self.link_spins["femur"].setValue(config.links.femur_cmm / 100.0)
        self.link_spins["tibia"].setValue(config.links.tibia_cmm / 100.0)
        self.home_radius_spin.setValue(config.geometry.home_radius_cmm / 100.0)
        self.home_foot_z_spin.setValue(config.geometry.home_foot_z_cmm / 100.0)
        self.coxa_lift_spin.setValue(config.geometry.coxa_lift_cmm / 100.0)
        gait = config.gait
        self.body_height_spin.setValue(gait.body_height_mm)
        self.stride_spin.setValue(gait.stride_len_mm)
        self.step_height_spin.setValue(gait.step_height_mm)
        self.duty_spin.setValue(gait.duty_x255)
        self.speed_spin.setValue(gait.speed_x255)
        gait_index = self.gait_combo.findData(gait.gait)
        self.gait_combo.setCurrentIndex(max(0, gait_index))

        self.leg_table.setRowCount(len(config.legs))
        for row, leg in enumerate(config.legs):
            self._set_row(
                self.leg_table,
                row,
                (
                    row + 1,
                    f"{leg.mount_x_dmm / 10.0:.2f}",
                    f"{leg.mount_y_dmm / 10.0:.2f}",
                    f"{leg.mount_z_dmm / 10.0:.2f}",
                    f"{leg.mount_yaw_cdeg / 100.0:.2f}",
                ),
                read_only_columns=(0,),
            )

        self.rc_table.setRowCount(len(config.rc_input.channels))
        for row, channel in enumerate(config.rc_input.channels):
            self._set_row(
                self.rc_table,
                row,
                (
                    row + 1,
                    channel.source,
                    channel.type,
                    channel.min_raw,
                    channel.center_raw,
                    channel.max_raw,
                    channel.reversed,
                    channel.deadband_x255,
                    channel.expo_x255,
                    channel.filter_tau_ms,
                    channel.switch_debounce_ms,
                ),
                read_only_columns=(0,),
            )

        self.body_table.setRowCount(len(self.BODY_FIELDS))
        for row, (label, attribute) in enumerate(self.BODY_FIELDS):
            self._set_row(
                self.body_table,
                row,
                (label, getattr(config.body_command, attribute)),
                read_only_columns=(0,),
            )

        self.foot_table.setRowCount(len(config.feet))
        for row, foot in enumerate(config.feet):
            self._set_row(
                self.foot_table,
                row,
                (
                    row + 1,
                    foot.pressure_baseline,
                    foot.near_thresh,
                    foot.touch_thresh,
                    foot.load_thresh,
                    foot.enabled,
                ),
                read_only_columns=(0,),
            )

    @staticmethod
    def _set_row(table, row: int, values, read_only_columns: tuple[int, ...] = ()) -> None:
        for column, value in enumerate(values):
            item = QTableWidgetItem(str(value))
            if column in read_only_columns:
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, column, item)

    @staticmethod
    def _table_int(table, row: int, column: int, label: str) -> int:
        item = table.item(row, column)
        text = item.text().strip() if item is not None else ""
        try:
            return int(text)
        except ValueError as exc:
            raise OpenLogConfigError(f"{label}: '{text}' is not an integer") from exc

    @staticmethod
    def _table_scaled(table, row: int, column: int, label: str, scale: int) -> int:
        item = table.item(row, column)
        text = item.text().strip() if item is not None else ""
        try:
            return round(float(text) * scale)
        except ValueError as exc:
            raise OpenLogConfigError(f"{label}: '{text}' is not a number") from exc

    def _read_editor(self):
        if self._base_config is None:
            raise OpenLogConfigError("select a valid journal record first")
        config = copy.deepcopy(self._base_config)
        config.robot_name = self.name_edit.text().strip()
        feature_defaults = 0
        for _label, constant in self.FEATURES:
            if self.feature_checks[constant].isChecked():
                feature_defaults |= getattr(self._cfg, constant)
        config.feature_defaults = feature_defaults

        for row, servo in enumerate(config.servos):
            servo.id = self._table_int(self.servo_table, row, 0, f"servo {row + 1} ID")
            servo.leg = self._table_int(self.servo_table, row, 1, f"servo {row + 1} leg") - 1
            servo.joint = self._table_int(self.servo_table, row, 2, f"servo {row + 1} joint") - 1
            servo.sign = self._table_int(self.servo_table, row, 3, f"servo {row + 1} sign")
            servo.trim_ticks = self._table_int(self.servo_table, row, 4, f"servo {row + 1} trim")
            servo.min_tick = self._table_int(self.servo_table, row, 5, f"servo {row + 1} minimum")
            servo.max_tick = self._table_int(self.servo_table, row, 6, f"servo {row + 1} maximum")

        config.links.coxa_cmm = round(self.link_spins["coxa"].value() * 100)
        config.links.femur_cmm = round(self.link_spins["femur"].value() * 100)
        config.links.tibia_cmm = round(self.link_spins["tibia"].value() * 100)
        config.geometry.home_radius_cmm = round(self.home_radius_spin.value() * 100)
        config.geometry.home_foot_z_cmm = round(self.home_foot_z_spin.value() * 100)
        config.geometry.coxa_lift_cmm = round(self.coxa_lift_spin.value() * 100)
        config.gait.body_height_mm = self.body_height_spin.value()
        config.gait.stride_len_mm = self.stride_spin.value()
        config.gait.step_height_mm = self.step_height_spin.value()
        config.gait.duty_x255 = self.duty_spin.value()
        config.gait.speed_x255 = self.speed_spin.value()
        config.gait.gait = int(self.gait_combo.currentData())

        for row, leg in enumerate(config.legs):
            leg.mount_x_dmm = self._table_scaled(self.leg_table, row, 1, f"leg {row + 1} X", 10)
            leg.mount_y_dmm = self._table_scaled(self.leg_table, row, 2, f"leg {row + 1} Y", 10)
            leg.mount_z_dmm = self._table_scaled(self.leg_table, row, 3, f"leg {row + 1} Z", 10)
            leg.mount_yaw_cdeg = self._table_scaled(self.leg_table, row, 4, f"leg {row + 1} yaw", 100)

        for row, channel in enumerate(config.rc_input.channels):
            channel.source = self._table_int(self.rc_table, row, 1, f"RC {row + 1} source")
            channel.type = self._table_int(self.rc_table, row, 2, f"RC {row + 1} type")
            channel.min_raw = self._table_int(self.rc_table, row, 3, f"RC {row + 1} min")
            channel.center_raw = self._table_int(self.rc_table, row, 4, f"RC {row + 1} center")
            channel.max_raw = self._table_int(self.rc_table, row, 5, f"RC {row + 1} max")
            channel.reversed = self._table_int(self.rc_table, row, 6, f"RC {row + 1} reversed")
            channel.deadband_x255 = self._table_int(self.rc_table, row, 7, f"RC {row + 1} deadband")
            channel.expo_x255 = self._table_int(self.rc_table, row, 8, f"RC {row + 1} expo")
            channel.filter_tau_ms = self._table_int(self.rc_table, row, 9, f"RC {row + 1} filter")
            channel.switch_debounce_ms = self._table_int(self.rc_table, row, 10, f"RC {row + 1} debounce")

        for row, (_label, attribute) in enumerate(self.BODY_FIELDS):
            setattr(
                config.body_command,
                attribute,
                self._table_int(self.body_table, row, 1, attribute),
            )

        for row, foot in enumerate(config.feet):
            foot.pressure_baseline = self._table_int(self.foot_table, row, 1, f"foot {row + 1} baseline")
            foot.near_thresh = self._table_int(self.foot_table, row, 2, f"foot {row + 1} near")
            foot.touch_thresh = self._table_int(self.foot_table, row, 3, f"foot {row + 1} touch")
            foot.load_thresh = self._table_int(self.foot_table, row, 4, f"foot {row + 1} load")
            foot.enabled = self._table_int(self.foot_table, row, 5, f"foot {row + 1} enabled")

        errors = self._cfg.validate_robot_config(config)
        if errors:
            raise OpenLogConfigError(errors[0])
        return config

    # --- write actions ----------------------------------------------------

    def _preview_record(self) -> None:
        try:
            config = self._read_editor()
            rendered = append_config_record(self._require_journal(), config)
            new_chars = len(rendered) - len(self._journal.raw_text)
        except OpenLogConfigError as exc:
            self.action_lbl.setText(f"preview blocked: {exc}")
            return
        self.preview_text.setPlainText(
            f"Ready to append sequence {self._journal.next_sequence}: "
            f"{new_chars} uppercase hex characters / {new_chars // 2} decoded bytes.\n"
            "Existing file prefix and journal records are preserved unchanged."
        )
        self.action_lbl.setText("record preview is valid")

    def _append_to_loaded_file(self) -> None:
        journal = self._require_journal_or_warn()
        if journal is None or journal.path is None:
            return
        if not self._confirm(
            "Append config record to card",
            f"Append sequence {journal.next_sequence} to {journal.path.name}? "
            "The old records stay intact; copy this file back to the robot card.",
        ):
            return
        self._save_to_path(journal.path)

    def _save_copy(self) -> None:
        journal = self._require_journal_or_warn()
        if journal is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save updated CONFIG.TXT",
            "CONFIG.TXT",
            "Config files (*.TXT *.txt);;All files (*)",
        )
        if path:
            self._save_to_path(Path(path))

    def _save_to_path(self, path: Path) -> None:
        try:
            config = self._read_editor()
            saved = save_openlog_config(self._require_journal(), config, path)
            reloaded = load_openlog_config(saved)
        except OpenLogConfigError as exc:
            self.action_lbl.setText(f"save blocked: {exc}")
            return
        self._load_journal(reloaded)
        self.action_lbl.setText(
            f"saved sequence {reloaded.newest.sequence} to {saved}; readback verified"
        )

    def _require_journal(self) -> OpenLogConfigJournal:
        if self._journal is None:
            raise OpenLogConfigError("open CONFIG.TXT first")
        return self._journal

    def _require_journal_or_warn(self) -> OpenLogConfigJournal | None:
        try:
            return self._require_journal()
        except OpenLogConfigError as exc:
            self.action_lbl.setText(str(exc))
            return None

    def _confirm(self, title: str, text: str) -> bool:
        return QMessageBox.question(
            self,
            title,
            text,
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) == QMessageBox.Yes

    def _apply_gates(self) -> None:
        editable = self._journal is not None and self._base_config is not None
        self.append_btn.setEnabled(editable and self._journal.path is not None)
        self.save_as_btn.setEnabled(editable)
        self.preview_btn.setEnabled(editable)