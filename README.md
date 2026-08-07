# HexNav OpenRB-150 Hexapod

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Buy Me A Coffee](https://img.shields.io/badge/Buy%20Me%20A%20Coffee-support-orange?logo=buy-me-a-coffee&logoColor=white)](https://buymeacoffee.com/vitormhenrique)

HexNav is a six-legged DYNAMIXEL MX-28AT robot built around a ROBOTIS
OpenRB-150. Its goal is to provide a safe, inspectable path from RC-controlled
walking to logged calibration, live model visualization, and optional
Jetson-assisted autonomy.

The OpenRB-150 is the final authority for arming, emergency stop, DYNAMIXEL
power and torque, gait timing, inverse kinematics, joint limits, and actuator
commands. The RC transmitter, Mac companion, Jetson, and ROS 2 processes may
request high-level behavior, but none can bypass firmware safety checks or
write raw servo goals during normal walking.

![HexNav URDF viewer showing the live or replayed robot model](img/URDF%20Viewer.png)

## What It Does Today

### Safety-Gated Robot Control

- Runs deterministic OpenRB-150 firmware with DYNAMIXEL power off at boot,
  safety state transitions, command-source arbitration, and enforced servo
  travel limits.
- Supports ExpressLRS CRSF control for arm/disarm, kill, walking, gait choice,
  body pose, gait tuning, and bounded choreography.
- Routes every high-level command through firmware validation, gait generation,
  body and leg IK, the servo map, and DYNAMIXEL goal limits before the bus is
  written.
- Reads six muxed Robotic Finger Sensor v2 foot sensors and publishes raw and
  fused contact data without blocking the motion-control path.

### Mac Engineering Console and CLI

The PySide6 companion and scriptable CLI discover USB ports, perform the
versioned protocol handshake, surface firmware capabilities, and keep serial,
logging, and rendering work off the UI thread.

![Connect and Setup page showing USB discovery and firmware capabilities](img/Connect%20%26%20Setup.png)

- Inspect connection, battery, DYNAMIXEL, I2C, RC, and command-source state.
- Enter safety-gated maintenance, torque-off passive pose, and DYNAMIXEL
  configuration workflows.
- Stage, validate, compare, commit, export, and restore EEPROM-backed robot
  configuration and calibration data.
- Use logical DYNAMIXEL parameters rather than table-specific register
  assumptions; torque-off EEPROM changes require read-back verification.

![Mode and Safety Center with emergency stop, maintenance, and passive-pose controls](img/Mode%20%26%20Safety%20Center.png)

### Telemetry, Recording, and Analysis

- Uses one COBS-framed, CRC16-protected, versioned USB protocol for firmware,
  companion tools, tests, and the Jetson bridge.
- Streams rate-limited health, control, servo, contact, I2C, RC, and API
  diagnostics with firmware-enforced backpressure behavior.
- Records raw frames and decoded sessions for replay, plot inspection, event
  markers, CSV export, and fault analysis without requiring connected hardware.

### Model and URDF Visualization

- Visualizes live or replayed joint telemetry in a kinematic model for rapid
  maintenance and pose inspection.
- Loads and poses the actual HexNav URDF meshes in the companion's 3D viewer,
  including playback for recorded sessions.
- Keeps ROS 2 and URDF tooling on the client side; these views diagnose and
  visualize firmware state, not replace actuator control.

![Telemetry-driven HexNav model viewer](img/Model%20Viewer.png)

## Getting Started

Read the [operator guide](docs/operator_guide.md) before wiring or powering the
robot. Keep the robot mechanically supported, the RC kill active, and the
external DYNAMIXEL supply safe throughout first connection and flashing.

1. Check the local toolchain and run hardware-free validation:

   ```bash
   just doctor
   just test
   ```

2. Build and flash firmware only when the robot is mechanically safe. Follow
   the board-specific instructions in the
   [firmware README](firmware/openrb150/README.md#commands):

   ```bash
   just firmware-build
   just firmware-flash
   ```

3. Install the [companion app](companion/README.md#project-local-developer-environment),
   then verify the binary USB protocol before maintenance or motion:

   ```bash
   just companion-sync
   just companion-cli status
   ```

4. Use the companion's **Connect & Setup** and **Overview** pages to verify the
   firmware identity, safety state, battery, DYNAMIXEL and I2C health, and
   active command source. A visible serial port alone is not sufficient.

## Safety Contract

- DYNAMIXEL power is off at firmware boot. This does not replace a stable
  mechanical setup, accessible RC kill control, or a correctly fused supply.
- Never route all 18 servo currents through the OpenRB-150 DYNAMIXEL port path.
  Use a properly rated fused 12 V DYNAMIXEL distribution system with a common
  ground.
- RC kill/disarm and firmware faults override all Mac, Jetson, and ROS
  requests.
- Passive pose mode turns torque off, stops goal writes, and rejects gait/IK
  commands until the mode exits.
- EEPROM/configuration writes are maintenance-only, torque-off, read-back
  verified operations. Stage, validate, review, and commit calibration changes.
- Unit tests, simulation, and output-disabled builds do not prove a physical
  DYNAMIXEL power rail or actuator is safe. Hardware-in-the-loop claims require
  physical evidence.

## Guides

| Need | Read |
| --- | --- |
| Safe wiring, first connection, calibration, passive pose, logging, and troubleshooting | [Operator guide](docs/operator_guide.md) |
| Architecture, task ownership, simulation, Jetson boundaries, and HIL evidence | [Developer guide](docs/developer_guide.md) |
| Repeatable local and CI checks | [Testing guide](docs/testing.md) |
| Firmware build environments and board-specific upload notes | [Firmware README](firmware/openrb150/README.md) |
| Desktop companion app and CLI reference | [Companion README](companion/README.md) |
| Binary framing and golden vectors | [Protocol README](protocol/README.md) |
| RC controller mapping and failsafe behavior | [Controller bridge reference](docs/controller_bridge.md) |
| ROS 2/SIL joint map and ownership boundary | [ROS 2 interface mapping](docs/ros2_controller_interface_mapping.md) |
| DYNAMIXEL detail-polling and baud-rate decision | [DXL polling evaluation](docs/dxl_status_polling_evaluation.md) |

## Development

The root task runner expects `just`, `uv`, and PlatformIO. Use `just doctor` to
inspect the installed tools and the selected PlatformIO executable. The most
common hardware-free and development workflows are:

```bash
# Install companion and test dependencies.
just companion-sync

# Start the desktop companion or use its CLI.
just companion-run
just companion-cli ports
just companion-cli status

# Build firmware and run software checks.
just firmware-build
just test
just check
```

`just test` runs the hardware-free protocol, companion, Jetson, and native
firmware suites. `just check` adds linting, type checking, and the OpenRB-150
firmware build. Refer to the [testing guide](docs/testing.md) before treating
any result as hardware evidence.

## Repository Layout

```text
firmware/openrb150/        OpenRB-150 firmware, board variant, native tests
protocol/                  Shared binary framing and Python reference package
companion/                 PySide6 engineering console, CLI, logging, replay
jetson/                    Optional high-level Jetson serial bridge
robot_ros_simulation/      ROS 2 Jazzy software-in-the-loop workspace and URDF
docs/                      Operator, developer, testing, and safety references
img/                       Companion and model-view screenshots used above
```

## Following Work

The immediate priority is physical validation: complete one-leg, suspended
full-robot, and low-speed ground tests; record fault-injection evidence; and
finish the guided calibration workflow for geometry, servo limits, and foot
sensors. The firmware remains deliberately conservative until these tests are
performed and signed off.

Reliability work will improve post-fault diagnosis and bus behavior, including
latched fault history, bounded DYNAMIXEL scan handling, and evaluation of
detail-polling performance across all 18 servos.

The longer-term optional autonomy path adds repeatable macOS packaging and
Jetson/ROS 2 integration: telemetry and safe flat-floor high-level motion,
camera and TF calibration, state estimation, local mapping, traversability,
and RC-supervised route and gait assistance. The Jetson will remain an advisor;
the OpenRB-150 will retain final safety and actuator authority.

Work is tracked with Beads. Run `bd ready` to find the current actionable work
item; the issue tracker, rather than this overview, is the source of truth for
implementation status.