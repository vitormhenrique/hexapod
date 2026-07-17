# Hexapod Companion

macOS development console for the OpenRB-150 hexapod. Provides a PySide6 GUI and
a scriptable CLI over the USB serial protocol, plus session logging for offline
analysis. The transport/protocol/data/theme layers are UI-independent, so the
CLI and tests run without Qt or hardware.

## Requirements

- Python `>=3.10,<3.14` (PySide6 wheels; pin with `uv python pin 3.12` if needed)
- [uv](https://docs.astral.sh/uv/) for environment management
- [just](https://github.com/casey/just) (optional, for the task runner)

## Project-local Developer Environment

```bash
cd companion
uv sync --extra dev      # or: just sync
```

This installs the companion package together with the local `hexapod-protocol`
path dependency in editable mode.

## Local macOS Command Install

For a source-based developer install that provides commands outside the project
directory, run this from the monorepo root:

```bash
uv tool install --editable ./companion --with-editable ./protocol/python
```

This installs the current checkout and its unpublished local protocol dependency
into one uv-managed tool environment. It provides:

```text
hexapod-companion  # PySide6 desktop application
hexapod-cli        # command-line client
hexapod-hil        # output-disabled HIL helper
```

If uv reports that its tool bin directory is not on `PATH`, run
`uv tool update-shell` once and start a new shell. Verify the installation
without a robot:

```bash
hexapod-cli --help
hexapod-cli ports
```

Use `uv tool uninstall hexapod-companion` to remove this tool environment. This
is intentionally an editable developer installation, not a signed standalone
macOS app. When package metadata changes, rerun the install command.

Build source and wheel artifacts for local packaging validation with:

```bash
just companion-package
```

The artifacts in `companion/dist/` are checked for buildability in CI. Because
the protocol package is a local monorepo dependency, use the editable install
above for a developer machine instead of treating the wheel as a public
standalone installer.

## Run

```bash
just launch              # PySide6 app (Dracula dark theme)
# or
uv run hexapod-companion
```

CLI:

```bash
uv run hexapod-cli ports                       # list serial ports
uv run hexapod-cli status                       # handshake + firmware status
uv run hexapod-cli stream health,servo_status   # live telemetry
uv run hexapod-cli log --streams servo_status   # record a session
uv run hexapod-cli stream-stats                 # firmware emit/drop counters
uv run hexapod-cli export-csv data/sessions/<session> \
  --signals health.battery_mv,servo.1.position
uv run hexapod-cli export-report data/sessions/<session>
```

When the Jetson owns the OpenRB-150 USB device, the same CLI and desktop app
can connect through its authenticated TCP relay instead of a local serial port:

```bash
uv run hexapod-cli status \
  --port 'tcp://<jetson-host>:5555?token=<relay-token>'
```

Enter the same endpoint in the editable connection field in **Connect & Setup**
for the desktop app. Start the relay and review its single-client and
trusted-network constraints in the [Jetson bridge guide](../jetson/README.md#mac-tcp-relay).

## Session exports

Open a recorded session in **Plot Workbench** replay mode, check the signals to
export, then use **Export selected CSV**. The CSV preserves the host and robot
timestamps and leaves unrelated stream columns blank rather than interpolating
between different telemetry rates. **Export session report** writes a text
summary of stream counts, recorded events/faults, and per-servo voltage,
temperature, and hardware-error observations.

The same artifacts are available without Qt through `export-csv` and
`export-report`; their default output paths are `selected_signals.csv` and
`session_summary.txt` within the session directory.

## Calibration workflow

Use **Robot Calibration & Config** for all persistent calibration data. It
loads the complete EEPROM-backed `RobotConfig`, so servo and geometry changes
share one staged diff, firmware validation, commit, and JSON export path.

1. Enter maintenance and confirm servo torque is off before changing servo
  zero trims, signs, travel limits, or geometry.
2. Load the config, edit the servo map and leg geometry in their displayed
  units, then use **Diff vs loaded** to inspect the fixed-point values that
  will be stored.
3. With each foot unloaded and raw sensor telemetry current, use **Copy live
  pressure baselines**. Set nonzero near/touch/load thresholds, keep `load >=
  touch`, and enable only calibrated feet.
4. Use **Stage to robot**, **Validate staged**, and **Commit to EEPROM** in
  that order. Export JSON after staging to archive the complete calibration.

The Foot Contact page can apply thresholds and re-zero an estimator at runtime
for bench tuning. Those runtime commands are not persistent; copy the chosen
values into Robot Calibration & Config and commit them to retain the result
over a power cycle.

## Test / lint

```bash
just test                # QT_QPA_PLATFORM=offscreen uv run pytest
just lint                # uvx ruff check
just fmt                 # uvx ruff format
```

## Layout

Flat `src/` layout — modules are top-level imports (`import transport`, `import
data`, ...), no extra package folder. `src/` is on the path via the editable
install and `[tool.pytest.ini_options] pythonpath`.

```
src/
  transport/        frame extractor, serial/TCP links, threaded protocol client
  data/             session logger + raw replay
  theme/            Dracula palette + Qt stylesheet
  models/           hexapod pose model (forward kinematics for the viewer)
  services/         Qt bridge over the protocol client (signals)
  ui/widgets/       nav rail, safety bar, event strip, emergency stop,
                    status badges, servo table/detail, hexapod view
  ui/pages/         connect, overview, mode & safety, foot contact,
                    passive pose, servo tuning, model viewer, diagnostics
  app.py            PySide6 entry point  (hexapod-companion)
  main_window.py    nav rail + safety bar + pages + event strip shell
  cli.py            Typer CLI            (hexapod-cli)
tests/              hardware-free unit + UI smoke tests
```

A `.envrc` (direnv) auto-syncs and activates the environment on `cd companion`
— run `direnv allow` once to enable it.
