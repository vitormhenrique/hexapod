"""Leg-by-leg fixture calibration wizard (physical 180° / 135° / 225°).

Linear Next/Previous workflow (18 poses = 6 legs × 3 fixture angles):
  1. Enter maintenance + scan.
  2. Center ALL servos at raw tick 2048 (physical 180°).
  3. One primary button drives the walk-through:
       "Move leg to pose"   commands the shown ticks for the active pose;
       "Capture & Next"     stores the corrected ticks and advances.
     Editing a tick never moves a servo; per-joint Send applies fine
     adjustments. "Previous pose" steps back at any time.
  4. Completing a leg's three fixtures fits sign + trim for its servos.
  5. When legs are done, stage + validate + commit the config.

Raw ticks are inverted through the LIVE servo map before sending, because the
firmware maps SET_JOINT_TARGET angles through the configured sign/trim.
"""

from __future__ import annotations

import copy

from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from hexapod_protocol import telemetry as tlm
from theme import DRACULA

# BasePage lives in ui.pages.__init__. Importing the package here is safe when
# this module is loaded from the end of that file (BasePage already defined).
from ui.pages import BasePage  # noqa: E402


class ServoCalibrationPage(BasePage):
    title = "Servo Fixture Calibration"
    subtitle = (
        "Walk leg-by-leg through fixture angles 180°, 135°, and 225°. Correct "
        "the raw tick guesses, then sign/trim are fit and written into the "
        "OpenLog robot config."
    )

    LEG_NAMES = (
        "rear left",
        "rear right",
        "middle right",
        "front right",
        "front left",
        "middle left",
    )
    JOINTS = (("Coxa", 0), ("Femur", 1), ("Tibia", 2))
    # Center first so the operator parks the horn at 180, then the two extremes
    # that resolve direction (sign).
    FIXTURES = (180, 135, 225)

    def build(self) -> None:
        from hexapod_protocol import config as cfg

        self._cfg = cfg
        self._robot_config = cfg.default_robot_config()
        self._servo_map = cfg.ServoMap(self._robot_config)
        self._connected = False
        self._lock_held = False
        self._setup_busy = False
        self._setup_ready = False
        self._state = -1
        self._busy = False
        self._centered = False  # center-all completed this session
        self._pose_commanded = False  # active fixture pose sent at least once
        self._present_ticks: dict[int, int] = {}  # servo id -> present tick

        # leg -> joint -> servo_deg -> captured tick
        self._samples: dict[int, dict[int, dict[int, int]]] = {
            leg: {j: {} for j in range(cfg.JOINTS_PER_LEG)}
            for leg in range(cfg.NUM_LEGS)
        }
        # leg -> joint -> ServoCalibrationFit
        self._fits: dict[int, dict[int, object]] = {
            leg: {} for leg in range(cfg.NUM_LEGS)
        }

        self._leg = 0
        self._fixture_index = 0
        self._tick_spins: dict[int, QSpinBox] = {}
        self._guess_lbls: dict[int, QLabel] = {}
        self._present_lbls: dict[int, QLabel] = {}
        self._send_btns: dict[int, QPushButton] = {}

        self.content.addWidget(self._maintenance_box())
        self.content.addWidget(self._progress_box())
        self.content.addWidget(self._pose_box())
        self.content.addWidget(self._results_box())
        self.content.addWidget(self._actions_box())

        self.banner = self.add_telemetry_banner(
            [(tlm.StreamId.SERVO_STATUS, "servo_status")],
            hint="Present ticks come from servo_status while you adjust fixtures.",
        )

        self.service.connected.connect(self._on_connected)
        self.service.state_changed.connect(self._on_state_changed)
        self.service.maint_lock_changed.connect(self._on_lock_changed)
        self.service.maint_result.connect(self._on_maint_result)
        self.service.maintenance_setup_changed.connect(self._on_setup_changed)
        self.service.config_loaded.connect(self._on_config_loaded)
        self.service.config_staged.connect(self._on_config_staged)
        self.service.config_result.connect(self._on_config_result)
        self.service.joint_target_result.connect(self._on_joint_result)
        self.service.dxl_result.connect(self._on_dxl_result)
        self.service.telemetry.connect(self._on_telemetry)
        self._refresh_pose_ui()
        self._apply_gates()

    # --- groups -----------------------------------------------------------

    def _maintenance_box(self) -> QGroupBox:
        box = QGroupBox("Bench setup")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        note = QLabel(
            "Calibration commands require Mac Maintenance with the lock held "
            "and a successful DXL scan. Start by centering all servos, then "
            "mount the fixture for the active leg before capturing each angle. "
            "Raw ticks are converted through the live servo map so the "
            "physical servo lands exactly on the requested tick."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DRACULA.comment};")
        form.addRow(note)
        actions = QHBoxLayout()
        self.enter_maint_btn = QPushButton("Enter Maintenance and Scan")
        self.enter_maint_btn.setProperty("accent", True)
        self.enter_maint_btn.clicked.connect(self.service.enter_maintenance)
        self.exit_maint_btn = QPushButton("Exit Maintenance")
        self.exit_maint_btn.clicked.connect(self.service.exit_maintenance)
        self.load_cfg_btn = QPushButton("Reload config")
        self.load_cfg_btn.clicked.connect(self.service.load_config)
        actions.addWidget(self.enter_maint_btn)
        actions.addWidget(self.exit_maint_btn)
        actions.addWidget(self.load_cfg_btn)
        actions.addStretch(1)
        form.addRow("Lock", self._wrap(actions))

        start_row = QHBoxLayout()
        self.center_all_btn = QPushButton("1. Center all servos (180\u00b0)")
        self.center_all_btn.setProperty("accent", True)
        self.center_all_btn.setToolTip(
            "Arm every servo and command raw tick 2048 on all 18 joints. "
            "Required once per session before the leg walk-through."
        )
        self.center_all_btn.clicked.connect(self._center_all)
        start_row.addWidget(self.center_all_btn)
        start_row.addStretch(1)
        form.addRow("Start", self._wrap(start_row))

        self.maintenance_lbl = QLabel("Maintenance lock: none")
        self.maintenance_lbl.setObjectName("MonoLabel")
        form.addRow("Status", self.maintenance_lbl)
        return box

    def _progress_box(self) -> QGroupBox:
        box = QGroupBox("Progress")
        form = QFormLayout(box)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(10)
        self.step_lbl = QLabel("")
        self.step_lbl.setObjectName("MonoLabel")
        form.addRow("Pose", self.step_lbl)
        self.leg_lbl = QLabel("")
        self.leg_lbl.setObjectName("MonoLabel")
        form.addRow("Active leg", self.leg_lbl)
        self.fixture_lbl = QLabel("")
        self.fixture_lbl.setObjectName("MonoLabel")
        form.addRow("Fixture angle", self.fixture_lbl)
        self.progress_lbl = QLabel("")
        self.progress_lbl.setObjectName("MonoLabel")
        form.addRow("Captured", self.progress_lbl)
        self.step_hint = QLabel("")
        self.step_hint.setWordWrap(True)
        self.step_hint.setStyleSheet(f"color: {DRACULA.comment};")
        form.addRow("Next step", self.step_hint)
        return box

    def _pose_box(self) -> QGroupBox:
        box = QGroupBox("Active leg joint ticks")
        lay = QVBoxLayout(box)
        hint = QLabel(
            "Raw-tick guesses for the fixture angle are pre-filled. The big "
            "button below always shows the next action: first it moves the "
            "leg to the shown ticks, then it captures and advances. Editing "
            "a tick NEVER moves the servo \u2014 press that joint's Send to "
            "apply a fine adjustment."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {DRACULA.comment};")
        lay.addWidget(hint)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        for col, title in enumerate(
            ("Joint", "Guess", "Corrected tick", "", "Present", "")
        ):
            lbl = QLabel(title)
            lbl.setStyleSheet(f"color: {DRACULA.comment};")
            grid.addWidget(lbl, 0, col)
        for row, (name, jid) in enumerate(self.JOINTS, start=1):
            grid.addWidget(QLabel(name), row, 0)
            guess = QLabel("--")
            guess.setObjectName("MonoLabel")
            self._guess_lbls[jid] = guess
            grid.addWidget(guess, row, 1)
            spin = QSpinBox()
            spin.setRange(0, 4095)
            spin.setSingleStep(1)
            spin.setAccelerated(True)
            spin.setValue(2048)
            # Never react to per-keystroke edits: the value is only read when
            # a Send/Command/Capture button is pressed. Keyboard tracking off
            # also keeps value() stable while typing ("2" of "2048" is never
            # observable as the committed value).
            spin.setKeyboardTracking(False)
            self._tick_spins[jid] = spin
            grid.addWidget(spin, row, 2)
            send = QPushButton("Send")
            send.setToolTip(
                "Command only this joint to the corrected tick shown."
            )
            send.clicked.connect(lambda _=False, j=jid: self._send_joint(j))
            self._send_btns[jid] = send
            grid.addWidget(send, row, 3)
            present = QLabel("--")
            present.setObjectName("MonoLabel")
            self._present_lbls[jid] = present
            grid.addWidget(present, row, 4)
            use_present = QPushButton("Use present")
            use_present.clicked.connect(lambda _=False, j=jid: self._use_present(j))
            grid.addWidget(use_present, row, 5)
        lay.addLayout(grid)

        row = QHBoxLayout()
        self.prev_btn = QPushButton("\u25c0 Previous pose")
        self.prev_btn.setToolTip(
            "Step back one pose (does not move the robot until you press the "
            "primary button)."
        )
        self.prev_btn.clicked.connect(self._go_previous)
        self.next_btn = QPushButton("Move leg to pose")
        self.next_btn.setProperty("accent", True)
        self.next_btn.setMinimumWidth(220)
        self.next_btn.clicked.connect(self._primary_action)
        self.reset_guess_btn = QPushButton("Reset to guess")
        self.reset_guess_btn.clicked.connect(self._reset_to_guess)
        row.addWidget(self.prev_btn)
        row.addWidget(self.next_btn)
        row.addWidget(self.reset_guess_btn)
        row.addStretch(1)
        lay.addLayout(row)

        self.pose_status = QLabel("--")
        self.pose_status.setObjectName("MonoLabel")
        self.pose_status.setWordWrap(True)
        lay.addWidget(self.pose_status)
        return box

    def _results_box(self) -> QGroupBox:
        box = QGroupBox("Fit results")
        lay = QVBoxLayout(box)
        self.results = QPlainTextEdit()
        self.results.setReadOnly(True)
        self.results.setObjectName("MonoLabel")
        self.results.setMinimumHeight(160)
        self.results.setPlaceholderText(
            "Per-leg sign/trim fits appear here as each leg is completed."
        )
        lay.addWidget(self.results)
        return box

    def _actions_box(self) -> QGroupBox:
        box = QGroupBox("Session")
        lay = QHBoxLayout(box)
        self.skip_leg_btn = QPushButton("Skip leg \u25b6")
        self.skip_leg_btn.setToolTip(
            "Advance without fitting this leg (keeps previous config values)."
        )
        self.skip_leg_btn.clicked.connect(self._skip_leg)
        self.apply_btn = QPushButton("Stage + commit fits")
        self.apply_btn.setProperty("accent", True)
        self.apply_btn.setToolTip(
            "Write fitted sign/trim into the robot config, stage, validate, and commit."
        )
        self.apply_btn.clicked.connect(self._apply_and_commit)
        self.reset_session_btn = QPushButton("Reset session")
        self.reset_session_btn.clicked.connect(self._reset_session)
        lay.addWidget(self.skip_leg_btn)
        lay.addWidget(self.apply_btn)
        lay.addWidget(self.reset_session_btn)
        lay.addStretch(1)
        return box

    def _wrap(self, layout) -> QWidget:
        widget = QWidget()
        widget.setLayout(layout)
        return widget

    # --- state helpers ----------------------------------------------------

    @property
    def _fixture_deg(self) -> int:
        return int(self.FIXTURES[self._fixture_index])

    @property
    def _step(self) -> int:
        """Linear pose index 0..17 (leg-major, fixture-minor)."""
        return self._leg * len(self.FIXTURES) + self._fixture_index

    @property
    def _total_steps(self) -> int:
        return self._cfg.NUM_LEGS * len(self.FIXTURES)

    def _guess_tick(self, _joint: int) -> int:
        return self._cfg.identity_tick_for_servo_deg(self._fixture_deg)

    def _refresh_pose_ui(self) -> None:
        leg = self._leg
        name = self.LEG_NAMES[leg] if leg < len(self.LEG_NAMES) else ""
        self.step_lbl.setText(f"{self._step + 1} / {self._total_steps}")
        self.leg_lbl.setText(f"Leg {leg + 1} — {name}  ({leg + 1}/6)")
        self.fixture_lbl.setText(
            f"{self._fixture_deg}\u00b0  "
            f"(step {self._fixture_index + 1}/{len(self.FIXTURES)})"
        )
        captured = []
        for jname, jid in self.JOINTS:
            n = len(self._samples[leg][jid])
            captured.append(f"{jname[0]}:{n}/{len(self.FIXTURES)}")
        done_legs = sum(1 for lg in range(self._cfg.NUM_LEGS) if self._fits[lg])
        self.progress_lbl.setText(
            f"this leg [{', '.join(captured)}]   legs fitted {done_legs}/6"
        )
        for _name, jid in self.JOINTS:
            guess = self._guess_tick(jid)
            self._guess_lbls[jid].setText(str(guess))
            prior = self._samples[leg][jid].get(self._fixture_deg)
            self._tick_spins[jid].blockSignals(True)
            self._tick_spins[jid].setValue(prior if prior is not None else guess)
            self._tick_spins[jid].blockSignals(False)
            self._update_present_label(jid)
        if done_legs == self._cfg.NUM_LEGS:
            self.step_hint.setText(
                "All six legs have fits. Review results, then Stage + commit."
            )
        elif not self._centered:
            self.step_hint.setText(
                "Center all servos first (step 1), then seat the Leg "
                f"{leg + 1} fixture at {self._fixture_deg}\u00b0."
            )
        elif not self._pose_commanded:
            self.step_hint.setText(
                f"Press 'Move leg to pose' to put Leg {leg + 1} at "
                f"{self._fixture_deg}\u00b0, then seat the fixture."
            )
        else:
            self.step_hint.setText(
                "Adjust ticks (Send per joint) until the fixture seats, then "
                "press 'Capture & Next'."
            )

    def _update_present_label(self, joint: int) -> None:
        servo = self._servo_map.servo_for(self._leg, joint)
        if servo is None:
            self._present_lbls[joint].setText("unmapped")
            return
        tick = self._present_ticks.get(servo.id)
        self._present_lbls[joint].setText("--" if tick is None else str(tick))

    def _use_present(self, joint: int) -> None:
        servo = self._servo_map.servo_for(self._leg, joint)
        if servo is None:
            return
        tick = self._present_ticks.get(servo.id)
        if tick is None:
            self.pose_status.setText("no present-position sample for that servo yet")
            return
        self._tick_spins[joint].setValue(int(tick))

    def _reset_to_guess(self) -> None:
        for _name, jid in self.JOINTS:
            self._tick_spins[jid].setValue(self._guess_tick(jid))
        self.pose_status.setText("ticks reset to fixture guesses")

    def _center_all(self) -> None:
        """Step 1: arm every servo and center all 18 joints at tick 2048."""
        if not self._bench_ready(ignore_centered=True):
            self.pose_status.setText("not ready: enter maintenance and scan first")
            return
        self._busy = True
        self._centered = True
        self._pose_commanded = False
        self.pose_status.setText("centering all 18 servos at 180\u00b0...")
        self.service.center_all_joints()
        self._refresh_pose_ui()
        self._apply_gates()

    def _send_joint(self, joint: int) -> None:
        """Explicitly command one joint to the corrected tick (button only)."""
        if not self._bench_ready() or not self._pose_commanded:
            self.pose_status.setText(
                "send: command the pose first (step 2), then adjust and Send"
            )
            return
        # Ensure any in-progress typed edit is committed to value().
        self._tick_spins[joint].interpretText()
        tick = self._tick_spins[joint].value()
        self.pose_status.setText(
            f"sending {self.JOINTS[joint][0].lower()} = {tick}..."
        )
        # Setup (mode/torque) was done by Command pose; skip it for speed.
        self.service.set_joint_targets_raw_ticks(
            [(self._leg, joint, tick)],
            robot_config=self._robot_config,
            ensure_setup=False,
        )

    def _primary_action(self) -> None:
        """One button drives the walk-through: move to pose, then capture."""
        if not self._pose_commanded:
            self._command_pose()
        else:
            self._capture_fixture()

    def _command_pose(self) -> None:
        if not self._bench_ready():
            if not self._centered:
                self.pose_status.setText("center all servos first (step 1)")
            else:
                self.pose_status.setText(
                    "not ready: enter maintenance and scan first"
                )
            return
        ticks = [self._tick_spins[j].value() for _n, j in self.JOINTS]
        self._busy = True
        self._pose_commanded = True
        self.pose_status.setText(
            f"commanding leg {self._leg + 1} @ {self._fixture_deg}\u00b0 "
            f"ticks={ticks}..."
        )
        self.service.set_leg_fixture_pose_raw(
            self._leg, *ticks, robot_config=self._robot_config
        )
        self._apply_gates()

    def _capture_fixture(self) -> None:
        leg = self._leg
        deg = self._fixture_deg
        for _name, jid in self.JOINTS:
            self._samples[leg][jid][deg] = int(self._tick_spins[jid].value())
        self.pose_status.setText(
            f"captured leg {leg + 1} @ {deg}\u00b0: "
            + ", ".join(f"{n}={self._samples[leg][j][deg]}" for n, j in self.JOINTS)
        )
        if self._fixture_index + 1 < len(self.FIXTURES):
            self._fixture_index += 1
            self._pose_commanded = False
            self._refresh_pose_ui()
            self._apply_gates()
            return
        self._fit_current_leg()

    def _fit_current_leg(self) -> None:
        leg = self._leg
        lines = [f"Leg {leg + 1} ({self.LEG_NAMES[leg]}):"]
        ok_all = True
        for name, jid in self.JOINTS:
            samples = [
                (float(deg), tick)
                for deg, tick in sorted(self._samples[leg][jid].items())
            ]
            fit = self._cfg.fit_servo_sign_trim(samples)
            self._fits[leg][jid] = fit
            if not fit.ok:
                ok_all = False
            lines.append(
                f"  {name:5s}  sign={fit.sign:+d}  trim={fit.trim_ticks:+5d}  "
                f"rms={fit.residual_ticks_rms:.2f}  {fit.detail}"
            )
        block = "\n".join(lines)
        existing = self.results.toPlainText().strip()
        self.results.setPlainText((existing + "\n\n" if existing else "") + block)
        if ok_all and leg + 1 < self._cfg.NUM_LEGS:
            self._leg += 1
            self._fixture_index = 0
            self._pose_commanded = False
            self.pose_status.setText(
                f"leg {leg + 1} fitted \u2014 advancing to leg {self._leg + 1}"
            )
            self._refresh_pose_ui()
            self._apply_gates()
            return
        elif ok_all:
            self.pose_status.setText(
                "all six legs fitted — review results, then Stage + commit"
            )
        else:
            self.pose_status.setText(
                f"leg {leg + 1} fit had errors — adjust captures and re-capture "
                "the last fixture, or Skip leg"
            )
        self._refresh_pose_ui()
        self._apply_gates()

    def _go_previous(self) -> None:
        if self._fixture_index > 0:
            self._fixture_index -= 1
        elif self._leg > 0:
            self._leg -= 1
            self._fixture_index = len(self.FIXTURES) - 1
        else:
            self.pose_status.setText("already at the first pose")
            return
        self._pose_commanded = False
        self.pose_status.setText(
            f"back to pose {self._step + 1}/{self._total_steps} \u2014 press "
            "'Move leg to pose' to re-command it"
        )
        self._refresh_pose_ui()
        self._apply_gates()

    def _skip_leg(self) -> None:
        if self._leg + 1 < self._cfg.NUM_LEGS:
            self.pose_status.setText(f"skipped leg {self._leg + 1}")
            self._leg += 1
            self._fixture_index = 0
            self._pose_commanded = False
        else:
            self.pose_status.setText("already on the last leg")
        self._refresh_pose_ui()
        self._apply_gates()

    def _reset_session(self) -> None:
        self._samples = {
            leg: {j: {} for j in range(self._cfg.JOINTS_PER_LEG)}
            for leg in range(self._cfg.NUM_LEGS)
        }
        self._fits = {leg: {} for leg in range(self._cfg.NUM_LEGS)}
        self._leg = 0
        self._fixture_index = 0
        self._centered = False
        self._pose_commanded = False
        self.results.clear()
        self.pose_status.setText("session reset \u2014 center all servos to begin")
        self._refresh_pose_ui()
        self._apply_gates()

    def _apply_and_commit(self) -> None:
        if not self._fits_ready():
            self.pose_status.setText("no complete leg fits to apply yet")
            return
        if self._robot_config is None:
            self.pose_status.setText("load the robot config first")
            return
        edited = copy.deepcopy(self._robot_config)
        applied = 0
        for leg, joints in self._fits.items():
            for jid, fit in joints.items():
                if fit is None or not getattr(fit, "ok", False):
                    continue
                servo = None
                for candidate in edited.servos:
                    if candidate.leg == leg and candidate.joint == jid:
                        servo = candidate
                        break
                if servo is None:
                    continue
                servo.sign = int(fit.sign)
                servo.trim_ticks = int(fit.trim_ticks)
                applied += 1
        if applied == 0:
            self.pose_status.setText("no successful fits to write")
            return
        self._busy = True
        self.pose_status.setText(f"staging {applied} servo sign/trim update(s)...")
        self.service.stage_config(edited)
        self._apply_gates()

    def _fits_ready(self) -> bool:
        return any(
            any(getattr(fit, "ok", False) for fit in joints.values())
            for joints in self._fits.values()
        )

    def _bench_ready(self, ignore_centered: bool = False) -> bool:
        return (
            self._connected
            and self._lock_held
            and self._setup_ready
            and not self._setup_busy
            and not self._busy
            and (self._centered or ignore_centered)
            and self._state != tlm.SafetyState.PASSIVE_POSE_STREAM
        )

    # --- service reactions ------------------------------------------------

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            self.service.subscribe(int(tlm.StreamId.SERVO_STATUS), 25)
            self.service.load_config()
        else:
            self._lock_held = False
            self._setup_busy = False
            self._setup_ready = False
            self._state = -1
            self._present_ticks.clear()
            self._busy = False
            self._centered = False
            self._pose_commanded = False
        self._apply_gates()

    def _on_state_changed(self, state: int) -> None:
        self._state = state
        self._apply_gates()

    def _on_lock_changed(self, held: bool, token: int) -> None:
        self._lock_held = held
        if not held:
            self._setup_ready = False
            self._centered = False
            self._pose_commanded = False
        self.maintenance_lbl.setText(
            f"Maintenance lock: {'held' if held else 'none'}"
            + (f" ({token})" if held else "")
        )
        self._apply_gates()

    def _on_setup_changed(self, busy: bool, ready: bool, detail: str) -> None:
        self._setup_busy = busy
        self._setup_ready = ready
        self.enter_maint_btn.setText(
            detail
            if busy
            else ("Maintenance ready" if ready else "Retry Maintenance and Scan")
        )
        self.pose_status.setText(detail)
        self._apply_gates()

    def _on_maint_result(self, res) -> None:
        if res is None:
            return
        if res.ok and res.token:
            self.maintenance_lbl.setText(f"Maintenance lock: held ({res.token})")
        elif not res.ok:
            self.pose_status.setText(f"maintenance rejected ({res.result})")

    def _on_config_loaded(self, config) -> None:
        if config is None:
            return
        self._robot_config = config
        self._servo_map = self._cfg.ServoMap(config)
        for _name, jid in self.JOINTS:
            self._update_present_label(jid)
        name = getattr(config, "robot_name", "robot")
        if isinstance(name, (bytes, bytearray)):
            name = name.decode("utf-8", errors="ignore")
        self.pose_status.setText(f"config loaded ({str(name).strip() or 'robot'})")

    def _on_config_staged(self, ok: bool) -> None:
        if not ok:
            self._busy = False
            self.pose_status.setText("config stage failed")
            self._apply_gates()
            return
        self.pose_status.setText("staged — validating...")
        self.service.validate_config()

    def _on_config_result(self, kind: str, res) -> None:
        if kind == "validate":
            if res is not None and res.ok:
                self.pose_status.setText("validated — committing to OpenLog...")
                self.service.commit_config()
            else:
                self._busy = False
                code = getattr(res, "result", "?")
                self.pose_status.setText(f"validate failed ({code})")
                self._apply_gates()
        elif kind == "commit":
            self._busy = False
            if res is not None and res.ok:
                self.pose_status.setText("commit ok — reloading config")
                self.service.load_config()
            else:
                code = getattr(res, "result", "?")
                self.pose_status.setText(f"commit failed ({code})")
            self._apply_gates()

    def _on_joint_result(self, res) -> None:
        self._busy = False
        if res is None:
            self.pose_status.setText("joint command: no response")
        elif getattr(res, "ok", False):
            tick = getattr(res, "tick", None)
            self.pose_status.setText(
                "joint command accepted"
                + (f" (last tick {tick})" if tick is not None else "")
            )
        else:
            self.pose_status.setText(
                f"joint command rejected ({getattr(res, 'result', '?')})"
            )
        self._apply_gates()

    def _on_dxl_result(self, kind: str, res) -> None:
        if kind != "scan":
            return
        count = len(res.servos()) if res is not None and res.done else 0
        self.pose_status.setText(f"servo scan: {count} found")
        if count and self._lock_held and not self._setup_busy:
            self._setup_ready = True
            self.enter_maint_btn.setText("Maintenance ready")
            self._apply_gates()

    def _on_telemetry(self, stream_id: int, record) -> None:
        if stream_id != int(tlm.StreamId.SERVO_STATUS):
            return
        for status in getattr(record, "servos", []):
            pos = getattr(status, "position", None)
            if pos is None or pos < 0:
                continue
            self._present_ticks[int(status.id)] = int(pos)
        for _name, jid in self.JOINTS:
            self._update_present_label(jid)

    def _apply_gates(self) -> None:
        ready = self._bench_ready()
        setup_ok = self._bench_ready(ignore_centered=True)
        self.enter_maint_btn.setEnabled(
            self._connected
            and not self._setup_busy
            and not self._setup_ready
            and self._state != tlm.SafetyState.PASSIVE_POSE_STREAM
        )
        self.exit_maint_btn.setEnabled(
            self._connected and self._lock_held and not self._setup_busy
        )
        self.load_cfg_btn.setEnabled(self._connected and not self._busy)
        self.center_all_btn.setEnabled(setup_ok)
        self.center_all_btn.setText(
            "Re-center all servos (180\u00b0)"
            if self._centered
            else "1. Center all servos (180\u00b0)"
        )
        # Primary button: one action at a time, label says which.
        self.next_btn.setEnabled(ready)
        if not self._centered:
            self.next_btn.setText("Center all first")
        elif not self._pose_commanded:
            self.next_btn.setText("Move leg to pose")
        else:
            self.next_btn.setText("Capture & Next \u25b6")
        self.prev_btn.setEnabled(
            not self._busy and self._centered and self._step > 0
        )
        self.reset_guess_btn.setEnabled(ready)
        for btn in self._send_btns.values():
            btn.setEnabled(ready and self._pose_commanded)
        self.skip_leg_btn.setEnabled(not self._busy and self._centered)
        self.reset_session_btn.setEnabled(not self._busy)
        self.apply_btn.setEnabled(
            self._connected
            and self._lock_held
            and self._fits_ready()
            and not self._busy
        )
        for spin in self._tick_spins.values():
            spin.setEnabled(ready or self._connected)
