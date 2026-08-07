# Hexapod monorepo task runner
# Run `just` or `just list` to see the available shortcuts.

set shell := ["zsh", "-cu"]

root := justfile_directory()
companion_dir := root / "companion"
firmware_dir := root / "firmware/openrb150"
protocol_dir := root / "protocol"
ros_simulation_dir := root / "robot_ros_simulation"
pio := env_var_or_default("PIO", env_var("HOME") + "/.platformio/penv/bin/pio")
openrb_port_finder := root / "tools/find_openrb_port.py"

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

# Install companion commands into uv's user tool directory from this checkout.
companion-install:
    uv tool install --editable "{{companion_dir}}" --with-editable "{{protocol_dir}}/python"

# Build source and wheel artifacts for local package validation.
companion-package:
    cd "{{companion_dir}}" && uv build

# Start the PySide6 companion application.
companion-run:
    cd "{{companion_dir}}" && uv run hexapod-companion

# Short alias for companion-run.
companion: companion-run

# Build the ROS SIL graph, RViz, and companion app connected to its local
# simulated-firmware endpoint. Example: just sim-companion 127.0.0.1 5561
# Pass false as the fourth positional argument to keep the same graph headless.
sim-companion host="127.0.0.1" port="5560" token="hexapod-sim" rviz="true":
    cd "{{ros_simulation_dir}}" && just companion-sim "{{host}}" "{{port}}" "{{token}}" "{{rviz}}"

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

# Install/synchronize the Jetson bridge package and its test tools.
jetson-sync:
    cd "{{root}}/jetson" && uv sync --extra dev

# Run the hardware-free Jetson bridge tests.
jetson-test:
    cd "{{root}}/jetson" && uv run pytest

# Expose a Jetson-owned OpenRB-150 USB link to one authenticated Mac client.
# Example: just jetson-relay --serial-port /dev/ttyACM0 --host 0.0.0.0
jetson-relay *args:
    cd "{{root}}/jetson" && uv run hexapod-jetson-relay {{args}}

# Run a hardware-in-loop page; example: just companion-hil all --port /dev/cu.usbmodem2101
companion-hil *args:
    cd "{{companion_dir}}" && uv run hexapod-hil {{args}}

# Build firmware for the real OpenRB-150 target.
firmware-build:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150

# Alias for firmware-build.
build: firmware-build

# Compile both OpenRB-150 and MKR Zero target environments.
firmware-check:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150 -e mkrzero

# Run native firmware unit tests.
firmware-test:
    cd "{{firmware_dir}}" && "{{pio}}" test -e native

# Upload firmware to the connected OpenRB-150.
firmware-upload:
    #!/usr/bin/env zsh
    port="$(python3 "{{openrb_port_finder}}" "{{pio}}")"
    echo "Uploading OpenRB-150 on $port"
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150 -t upload --upload-port "$port"

# Alias matching the project command convention.
firmware-flash: firmware-upload

# Alias for firmware-upload.
upload: firmware-upload

# Open the OpenRB-150 USB serial monitor (default 115200 baud).
firmware-monitor baud="115200":
    #!/usr/bin/env zsh
    port="$(python3 "{{openrb_port_finder}}" "{{pio}}")"
    cd "{{firmware_dir}}" && "{{pio}}" device monitor --port "$port" -b {{baud}}

# Remove PlatformIO firmware build artifacts.
firmware-clean:
    cd "{{firmware_dir}}" && "{{pio}}" run -t clean

# Alias for firmware-clean.
clean: firmware-clean

# Print the unique USB CDC port belonging to the connected OpenRB-150.
firmware-port:
    @python3 "{{openrb_port_finder}}" "{{pio}}"

# Run shared protocol tests and golden-vector checks.
protocol-test:
    cd "{{protocol_dir}}" && uv run --project python --extra dev pytest tests

# Lint the shared protocol implementation and vector tests.
protocol-lint:
    cd "{{protocol_dir}}" && uvx ruff check python/hexapod_protocol tests

# Type-check the shared protocol implementation.
protocol-typecheck:
    cd "{{protocol_dir}}" && uv run --project python --extra dev pyright python/hexapod_protocol

# Lint the pure-Python Jetson bridge and its loopback tests.
jetson-lint:
    cd "{{root}}/jetson" && uvx ruff check src tests

# Type-check the pure-Python Jetson bridge against its editable dependencies.
jetson-typecheck:
    cd "{{root}}/jetson" && uv run --extra dev pyright src/hexapod_jetson_bridge

# Run protocol, companion, and native firmware tests.
test: protocol-test companion-test jetson-test firmware-test

# Run Ruff across protocol, companion, and Jetson Python code.
lint: protocol-lint companion-lint jetson-lint
typecheck: protocol-typecheck jetson-typecheck

# Run all local software quality gates and compile the real firmware target.
check: test lint typecheck firmware-build
