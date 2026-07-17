# OpenRB-150 Hexapod

Firmware, host tools, simulation, and future Jetson autonomy support for a
six-legged robot built around a ROBOTIS OpenRB-150 and MX-28AT servos.

The OpenRB-150 is the final authority for arming, emergency stop, DYNAMIXEL
power and torque, gait timing, inverse kinematics, joint limits, and actuator
commands. A Mac, Jetson, or ROS 2 process may request high-level motion, but
none can bypass firmware safety checks.

## Start Here

1. Read the [operator guide](docs/operator_guide.md) before wiring or powering
   the robot. It covers safe wiring, flashing, calibration, operation,
   passive-pose recording, plotting, URDF viewing, and fault recovery.
2. Run hardware-free checks before connecting a robot:

   ```bash
   just doctor
   just test
   ```

3. Follow [firmware flashing](firmware/openrb150/README.md#commands) only with
   the robot mechanically supported, RC kill active, and a safe 12 V DYNAMIXEL
   power arrangement.
4. Install the [companion app](companion/README.md#install), then use
   `hexapod-cli status` to verify the USB protocol before attempting any
   maintenance or motion workflow.

## Guides

| Need | Read |
| --- | --- |
| Safe wiring, first connection, calibration, passive pose, logging, and troubleshooting | [Operator guide](docs/operator_guide.md) |
| Architecture, tests, protocol changes, simulation, Jetson, and HIL boundaries | [Developer guide](docs/developer_guide.md) |
| Repeatable local and CI checks | [Testing guide](docs/testing.md) |
| Firmware build environments and board-specific upload notes | [Firmware README](firmware/openrb150/README.md) |
| Desktop companion app and CLI reference | [Companion README](companion/README.md) |
| Binary framing and golden vectors | [Protocol README](protocol/README.md) |
| RC controller mapping and failsafe behavior | [Controller bridge reference](docs/controller_bridge.md) |
| ROS 2/SIL joint map and ownership boundary | [ROS 2 interface mapping](docs/ros2_controller_interface_mapping.md) |
| DYNAMIXEL detail-polling and baud-rate decision | [DXL polling evaluation](docs/dxl_status_polling_evaluation.md) |

## Safety Baseline

- DYNAMIXEL power is off at firmware boot. Keep the robot mechanically safe
  before intentionally enabling it.
- Never route the current for all 18 servos through the OpenRB-150 DYNAMIXEL
  port path. Use a fused high-current 12 V distribution system and a common
  ground.
- RC kill/disarm and firmware faults always override USB, Jetson, and ROS
  requests.
- Passive pose mode is torque-off and rejects gait/IK commands until it exits.
- EEPROM/configuration writes are maintenance-only, torque-off, read-back
  verified operations. Stage, validate, review, and commit calibration changes.
- Hardware-in-the-loop evidence is physical evidence. A passing unit test or
  output-disabled build does not prove a DYNAMIXEL rail or actuator is safe.

## Repository Layout

```text
firmware/openrb150/        OpenRB-150 firmware, board variant, native tests
protocol/                  Shared binary framing and Python reference package
companion/                 PySide6 engineering console, CLI, logging, replay
jetson/                    High-level Jetson serial bridge (no raw servo API)
robot_ros_simulation/      ROS 2 Jazzy SIL workspace and HexNav URDF
docs/                      Operator, developer, testing, and safety references
```

Work is tracked with Beads. Run `bd ready` to find the next actionable issue.