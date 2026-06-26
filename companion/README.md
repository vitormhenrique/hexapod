# Hexapod Companion

macOS development console for the OpenRB-150 hexapod. Provides a PySide6 GUI and
a scriptable CLI over the USB serial protocol, plus session logging for offline
analysis. The transport/protocol/data/theme layers are UI-independent, so the
CLI and tests run without Qt or hardware.

## Requirements

- Python `>=3.10,<3.14` (PySide6 wheels; pin with `uv python pin 3.12` if needed)
- [uv](https://docs.astral.sh/uv/) for environment management
- [just](https://github.com/casey/just) (optional, for the task runner)

## Install

```bash
cd companion
uv sync --extra dev      # or: just sync
```

This installs the companion package together with the local `hexapod-protocol`
path dependency in editable mode.

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
```

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
  transport/        frame extractor, serial link, threaded protocol client
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
