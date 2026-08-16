# Hexapod monorepo task runner
# Run `just` or `just list` to see the available shortcuts.

set shell := ["zsh", "-cu"]

root := justfile_directory()
companion_dir := root / "companion"
firmware_dir := root / "firmware/openrb150"
protocol_dir := root / "protocol"
ros_simulation_dir := root / "robot_ros_simulation"
pio := env_var_or_default("PIO", env_var("HOME") + "/.platformio/penv/bin/pio")
board_detector := root / "tools/detect_firmware_board.py"

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

# Build firmware for the connected board (auto-detects OpenRB-150 vs Teensy).
# With no board connected, builds both targets. Override: just firmware-build teensymm
firmware-build env="":
    #!/usr/bin/env zsh
    env="{{env}}"
    if [[ -z "$env" ]]; then
        env="$(python3 "{{board_detector}}" --env-only "{{pio}}" 2>/dev/null || true)"
    fi
    cd "{{firmware_dir}}"
    if [[ -n "$env" ]]; then
        echo "Building $env"
        "{{pio}}" run -e "$env"
    else
        echo "No board detected; building both targets"
        "{{pio}}" run -e openrb150 -e teensymm
    fi

# Alias for firmware-build.
build: firmware-build

# Compile both supported target environments.
firmware-check:
    cd "{{firmware_dir}}" && "{{pio}}" run -e openrb150 -e teensymm

# Build the instrumented benchmark image for the connected board. With no
# board connected, build both images for later flashing. Override with the base
# board name: `just firmware-benchmark-build teensymm`.
firmware-benchmark-build env="":
    #!/usr/bin/env zsh
    env="{{env}}"
    if [[ -z "$env" ]]; then
        env="$(python3 "{{board_detector}}" --env-only "{{pio}}" 2>/dev/null || true)"
    fi
    cd "{{firmware_dir}}"
    if [[ -n "$env" ]]; then
        echo "Building ${env}_benchmark"
        "{{pio}}" run -e "${env}_benchmark"
    else
        echo "No board detected; building both benchmark targets"
        "{{pio}}" run -e openrb150_benchmark -e teensymm_benchmark
    fi

# Flash the build-gated benchmark image to the detected board. The Teensy
# loader detects its device itself; OpenRB-150 requires its selected CDC port.
firmware-benchmark-upload env="":
    #!/usr/bin/env zsh
    env="{{env}}"
    port=""
    if [[ -z "$env" ]]; then
        read -r env port <<< "$(python3 "{{board_detector}}" "{{pio}}")"
    fi
    cd "{{firmware_dir}}"
    if [[ "$env" == teensymm ]]; then
        echo "Uploading teensymm_benchmark"
        "{{pio}}" run -e teensymm_benchmark -t upload
    else
        [[ -z "$port" ]] && port="$(python3 "{{board_detector}}" --port-only "{{pio}}")"
        echo "Uploading ${env}_benchmark on $port"
        "{{pio}}" run -e "${env}_benchmark" -t upload --upload-port "$port"
    fi

# Collect the current benchmark snapshot over USB. Flash the matching
# *_benchmark image first; run once per board and compare the printed metrics.
firmware-benchmark seconds="15" rate="5" port="":
    #!/usr/bin/env zsh
    port="{{port}}"
    if [[ -z "$port" ]]; then
        if ! port="$(python3 "{{board_detector}}" --port-only "{{pio}}")"; then
            exit 1
        fi
    fi
    cd "{{companion_dir}}" && uv run hexapod-cli benchmark --seconds {{seconds}} --rate {{rate}} --port "$port"

# Run native firmware unit tests.
firmware-test:
    cd "{{firmware_dir}}" && "{{pio}}" test -e native

# Upload firmware to the connected board (auto-detects OpenRB-150 vs Teensy).
# Override when the board cannot be detected: just firmware-upload teensymm
firmware-upload env="":
    #!/usr/bin/env zsh
    env="{{env}}"
    port=""
    if [[ -z "$env" ]]; then
        read -r env port <<< "$(python3 "{{board_detector}}" "{{pio}}")"
    fi
    cd "{{firmware_dir}}"
    if [[ "$env" == teensymm ]]; then
        # The Teensy loader autodetects its own device (including bootloader
        # mode, which has no serial port), so no --upload-port is passed.
        echo "Uploading teensymm (Teensy loader autodetects the board)"
        "{{pio}}" run -e teensymm -t upload
    else
        [[ -z "$port" ]] && port="$(python3 "{{board_detector}}" --port-only "{{pio}}")"
        echo "Uploading $env on $port"
        "{{pio}}" run -e "$env" -t upload --upload-port "$port"
    fi

# Alias matching the project command convention.
firmware-flash: firmware-upload

# Alias for firmware-upload.
upload: firmware-upload

# Open the connected board's USB serial monitor (default 115200 baud).
firmware-monitor baud="115200":
    #!/usr/bin/env zsh
    port="$(python3 "{{board_detector}}" --port-only "{{pio}}")"
    cd "{{firmware_dir}}" && "{{pio}}" device monitor --port "$port" -b {{baud}}

# Remove PlatformIO firmware build artifacts.
firmware-clean:
    cd "{{firmware_dir}}" && "{{pio}}" run -t clean

# Alias for firmware-clean.
clean: firmware-clean

# Print the detected board environment and USB CDC port.
firmware-port:
    @python3 "{{board_detector}}" "{{pio}}"

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
