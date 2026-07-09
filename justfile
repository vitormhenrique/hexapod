# Hexapod monorepo task runner
# Run `just` or `just list` to see the available shortcuts.

set shell := ["zsh", "-cu"]

root := justfile_directory()
companion_dir := root / "companion"
firmware_dir := root / "firmware/openrb150"
protocol_dir := root / "protocol"
pio := env_var_or_default("PIO", env_var("HOME") + "/.platformio/penv/bin/pio")

# List all available recipes.
default: list

# List all available recipes.
list:
    @just --list

# Show the main tool versions and configured PlatformIO executable.
doctor:
    @echo "just:       $(just --version)"
    @echo "uv:         $(uv --version)"
    @echo "PlatformIO: {{pio}}"
    @{{pio}} --version

# Install/synchronize companion dependencies, including test tools.
companion-sync:
    cd "{{companion_dir}}" && uv sync --extra dev

# Start the PySide6 companion application.
companion-run:
    cd "{{companion_dir}}" && uv run hexapod-companion

# Short alias for companion-run.
companion: companion-run

# Run the companion CLI; example: just companion-cli status
companion-cli *args:
    cd "{{companion_dir}}" && uv run hexapod-cli {{args}}

# List serial ports visible to the companion CLI.
companion-ports:
    cd "{{companion_dir}}" && uv run hexapod-cli ports

# Run the companion test suite with headless Qt.
companion-test:
    cd "{{companion_dir}}" && QT_QPA_PLATFORM=offscreen uv run pytest

# Lint companion source and tests with Ruff.
companion-lint:
    cd "{{companion_dir}}" && uvx ruff check src tests

# Format companion source and tests with Ruff.
companion-format:
    cd "{{companion_dir}}" && uvx ruff format src tests

# Run a hardware-in-loop page; example: just companion-hil all --port /dev/cu.usbmodem2101
companion-hil *args:
    cd "{{companion_dir}}" && uv run hexapod-hil {{args}}

# Build firmware for the real OpenRB-150 target.
firmware-build:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150

# Compile both OpenRB-150 and MKR Zero target environments.
firmware-check:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150 -e mkrzero

# Run native firmware unit tests.
firmware-test:
    cd "{{firmware_dir}}" && "{{pio}}" test -e native

# Upload firmware to the connected OpenRB-150.
firmware-upload:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150 -t upload

# Alias matching the project command convention.
firmware-flash: firmware-upload

# Open the OpenRB-150 USB serial monitor (default 115200 baud).
firmware-monitor baud="115200":
    cd "{{firmware_dir}}" && "{{pio}}" device monitor -b {{baud}}

# Remove PlatformIO firmware build artifacts.
firmware-clean:
    cd "{{firmware_dir}}" && "{{pio}}" run -t clean

# Run shared protocol tests and golden-vector checks.
protocol-test:
    cd "{{protocol_dir}}" && uv run --project python --extra dev pytest tests

# Run protocol, companion, and native firmware tests.
test: protocol-test companion-test firmware-test

# Run tests and compile firmware for the OpenRB-150.
check: test firmware-build
