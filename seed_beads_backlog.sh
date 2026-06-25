#!/usr/bin/env bash
set -euo pipefail

# Seed Beads issues for the hexapod monorepo.
# Run this from the repository root after installing Beads and running `bd init`.
# The script creates phase epics and child issues using verified bd create flags.

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

need bd

create_epic() {
  local title="$1"
  local priority="$2"
  local labels="$3"
  local description="$4"
  bd create "$title" -t epic -p "$priority" -l "$labels" --description "$description" --silent
}

create_child() {
  local parent="$1"
  local title="$2"
  local type="$3"
  local priority="$4"
  local labels="$5"
  local description="$6"
  local acceptance="$7"
  bd create "$title" -t "$type" -p "$priority" -l "$labels" --parent "$parent" --description "$description" --acceptance "$acceptance" --silent
}

echo "Seeding Phase 1..."
phase1=$(create_epic \
  "Phase 1 - Firmware foundation and safe I/O" \
  0 \
  "phase:1,area:firmware,priority:safety,hw:openrb150" \
  "Boot OpenRB-150 firmware safely, establish bus ownership, prove DXL/I2C/RC/API basics, and create a validated configuration store.")

create_child "$phase1" "Create monorepo skeleton and firmware build target" task 0 "phase:1,area:firmware" \
  "Create the monorepo layout and make the OpenRB-150 firmware build from the command line." \
  "Repo layout exists; firmware compiles for OpenRB-150; README contains flash command; local check command is documented."

create_child "$phase1" "Implement board HAL and safe boot defaults" task 0 "phase:1,area:firmware,priority:safety,hw:openrb150" \
  "Centralize board pins and safe boot behavior." \
  "DXL power remains off at boot; board pins/constants are centralized; battery ADC and status hooks exist."

create_child "$phase1" "Bring up FreeRTOS task skeleton" task 0 "phase:1,area:firmware" \
  "Start the firmware RTOS tasks with bounded stacks and visible health reporting." \
  "Control, DXL, API, I2C, RC, and health tasks start; stack high-water marks are visible; watchdog stub exists."

create_child "$phase1" "Implement protocol framing golden vectors" task 0 "phase:1,area:protocol,area:firmware" \
  "Implement COBS plus CRC16 framing and shared protocol test vectors." \
  "COBS plus CRC16 encode/decode works in firmware and host tests; corrupt frames are rejected."

create_child "$phase1" "Implement USB API v0" feature 0 "phase:1,area:firmware,area:protocol" \
  "Expose the first USB CDC API commands needed by host tools." \
  "HELLO, HEARTBEAT, GET_STATUS, and GET_CAPABILITIES work over USB CDC."

create_child "$phase1" "Implement DYNAMIXEL bus manager maintenance scan" feature 0 "phase:1,area:firmware,hw:dxl,priority:safety" \
  "Create the single-owner DXL bus manager and maintenance-safe scan/ping path." \
  "Scan, ping, and status work with torque off; bus errors are counted; no other task touches DXL directly."

create_child "$phase1" "Implement TCA9548A and root I2C scan" feature 1 "phase:1,area:firmware,hw:i2c" \
  "Create I2C mux selection and scan tooling for root and mux channels." \
  "Root bus scan finds mux and EEPROM; channel scan finds sensor devices; mux channel selection is exclusive."

create_child "$phase1" "Implement 24LC32 page driver and transactional config store" feature 0 "phase:1,area:firmware,hw:i2c,priority:safety" \
  "Implement EEPROM page IO and two-slot CRC-protected config transactions." \
  "Config load, validate, commit, power-cycle recovery, corrupt-slot rejection, and defaults fallback are tested."

create_child "$phase1" "Implement CRSF parser and RC failsafe status" feature 0 "phase:1,area:firmware,hw:crsf,priority:safety" \
  "Parse ExpressLRS CRSF channel frames and expose RC/failsafe status." \
  "Channels normalize to stable units; stale RC triggers failsafe flag; arming, kill, and gait channels are mapped."

create_child "$phase1" "Create Phase 1 hardware-in-loop smoke tests" task 1 "phase:1,area:test,area:firmware" \
  "Document and script the initial hardware smoke checks." \
  "Checklist covers boot, USB, DXL, I2C, EEPROM, CRSF, and fault observations."

echo "Seeding Phase 2..."
phase2=$(create_epic \
  "Phase 2 - Firmware motion, sensors, config API, and telemetry" \
  0 \
  "phase:2,area:firmware,priority:safety,hw:dxl,hw:i2c" \
  "Make the robot controller useful without the companion app: safe walking, servo mapping, IK/gait control, sensor polling, config updates, telemetry, and command arbitration.")

create_child "$phase2" "Define persistent robot config schema" feature 0 "phase:2,area:firmware,area:protocol" \
  "Define the EEPROM-backed schema for robot geometry, servo map, sensor calibration, and gait defaults." \
  "Schema includes servo ID mapping, leg geometry, servo signs/offsets/min/max, gait defaults, sensor calibration, and config version."

create_child "$phase2" "Implement config API over USB protocol" feature 0 "phase:2,area:firmware,area:protocol,priority:safety" \
  "Expose config get, set, validate, commit, and reset through the host protocol." \
  "RAM shadow and EEPROM transaction are used; invalid configs are rejected with errors."

create_child "$phase2" "Implement servo map and joint limit enforcement" feature 0 "phase:2,area:firmware,hw:dxl,priority:safety" \
  "Convert logical leg and joint targets into DXL ticks using the validated config." \
  "Leg/joint target converts to DXL ticks; sign, zero, min, and max are applied; clamp flags are reported."

create_child "$phase2" "Implement 3-DOF leg IK and body pose transform" feature 0 "phase:2,area:firmware" \
  "Implement deterministic leg inverse kinematics and body transforms for the hexapod." \
  "IK returns joint angles and reachability; tests cover nominal, edge, and unreachable foot targets."

create_child "$phase2" "Implement gait engine v1" feature 0 "phase:2,area:firmware" \
  "Generate bounded foot targets for initial gait modes." \
  "Stand, sit, tripod, ripple, and crawl or wave produce bounded foot targets at 50 Hz."

create_child "$phase2" "Implement DXL Sync Write and status telemetry" feature 0 "phase:2,area:firmware,hw:dxl" \
  "Update all servo goals efficiently and read status for telemetry and safety." \
  "All servo goals update in one control cycle; status read includes position, velocity, load or PWM, voltage, temperature, and error bits."

create_child "$phase2" "Implement Robotic Finger Sensor v2 polling" feature 1 "phase:2,area:firmware,hw:i2c" \
  "Poll all six muxed finger sensors without blocking control." \
  "All six mux channels produce pressure, proximity, and contact telemetry at configured rate; I2C errors do not block control."

create_child "$phase2" "Implement RC Jetson Mac command arbitration" feature 0 "phase:2,area:firmware,area:protocol,priority:safety" \
  "Implement command-source priority and heartbeat-based authority management." \
  "RC kill overrides all; Jetson requires fresh heartbeat and RC-enabled autonomy; Mac maintenance lock expires safely."

create_child "$phase2" "Implement rate-limited telemetry subscriptions" feature 0 "phase:2,area:firmware,area:protocol" \
  "Allow host clients to subscribe to telemetry streams at safe rates." \
  "Host can subscribe and unsubscribe streams; stream rates are enforced; dropped frames and backlog counters are visible."

create_child "$phase2" "Complete firmware safety state machine" feature 0 "phase:2,area:firmware,priority:safety" \
  "Complete safe state transitions for boot, arm, walk, maintenance, faults, and estop." \
  "Boot, disarmed, arming checks, stand ready, RC manual, Jetson assisted, Mac maintenance, soft fault, hard fault, and estop transitions are tested."

create_child "$phase2" "Run Phase 2 bench suspended and ground tests" task 0 "phase:2,area:test,area:firmware,priority:safety" \
  "Validate motion gradually from one leg through suspended full robot to low-speed ground tests." \
  "One-leg test, suspended full-body test, low-speed ground gait test, fault injection, and telemetry log review are documented."

echo "Seeding Phase 3..."
phase3=$(create_epic \
  "Phase 3 - Companion transport, protocol client, and data logger" \
  1 \
  "phase:3,area:companion,area:protocol,area:data" \
  "Create a scriptable Python package that connects to the robot, decodes telemetry, sends safe commands, and records sessions before building the full UI.")

create_child "$phase3" "Create Python companion package scaffold" task 1 "phase:3,area:companion" \
  "Set up the companion Python package and command entry point." \
  "Package installs locally; lint and test commands are documented; CLI entry point exists."

create_child "$phase3" "Implement Python protocol codec" feature 1 "phase:3,area:companion,area:protocol" \
  "Implement COBS, CRC, framing, and schema-version handling in Python." \
  "Codec matches firmware golden vectors; corrupt frames are rejected; schema version detection works."

create_child "$phase3" "Implement serial connection service" feature 1 "phase:3,area:companion" \
  "Create non-blocking USB serial connection lifecycle management." \
  "Discovers USB serial ports, connects, reconnects, heartbeats, and reports link state without blocking."

create_child "$phase3" "Implement typed protocol client" feature 1 "phase:3,area:companion,area:protocol" \
  "Expose high-level Python methods for the robot serial API." \
  "Supports hello, status, capabilities, subscribe, config get, and safe error handling."

create_child "$phase3" "Implement session logger" feature 1 "phase:3,area:companion,area:data" \
  "Record raw frames, decoded telemetry, events, and config snapshots." \
  "Logger writes a complete session directory with raw frames, decoded telemetry, events, and config snapshot."

create_child "$phase3" "Implement replay system" feature 2 "phase:3,area:companion,area:data" \
  "Replay saved sessions into services and tests without hardware." \
  "Replay can feed decoded telemetry to tests and future UI from saved sessions."

create_child "$phase3" "Implement companion CLI tools" feature 1 "phase:3,area:companion" \
  "Add CLI commands for status, streaming, logging, replay, and config transfer." \
  "status, stream, log, replay, config dump, and config restore commands work."

echo "Seeding Phase 4..."
phase4=$(create_epic \
  "Phase 4 - Companion UI foundation and core pages" \
  1 \
  "phase:4,area:companion,area:ui" \
  "Build the first usable macOS development UI with clear navigation and safe controls.")

create_child "$phase4" "Build PySide6 application shell" feature 1 "phase:4,area:companion,area:ui" \
  "Build a native-style application shell with navigation and global safety affordances." \
  "App launches on macOS with navigation, global status bar, and global estop widget."

create_child "$phase4" "Build Connect and Overview pages" feature 1 "phase:4,area:companion,area:ui" \
  "Create connection and status pages for daily development." \
  "User can connect, see firmware capabilities, robot state, battery, DXL/I2C health, and active command source."

create_child "$phase4" "Build Gait Lab page" feature 1 "phase:4,area:companion,area:ui" \
  "Create safe controls for gait selection and body movement." \
  "Gait selection and safe body twist controls work with visible arming and mode requirements."

create_child "$phase4" "Build Leg Lab page" feature 1 "phase:4,area:companion,area:ui" \
  "Create individual leg and joint maintenance controls." \
  "Individual leg, foot, and joint controls work only in maintenance mode and show IK and clamp feedback."

create_child "$phase4" "Build Servo Config page" feature 1 "phase:4,area:companion,area:ui" \
  "Create EEPROM-backed servo map and config editor UI." \
  "User can view, edit, validate, diff, commit, export, and import servo map/config."

create_child "$phase4" "Build Sensor Dashboard page" feature 2 "phase:4,area:companion,area:ui,hw:i2c" \
  "Create live sensor panels for the muxed Robotic Finger Sensor v2 boards." \
  "All six sensor channels show live values, health, and calibration actions."

create_child "$phase4" "Build Diagnostics page" feature 1 "phase:4,area:companion,area:ui" \
  "Create developer diagnostics for raw frames, protocol, DXL, I2C, and firmware timing." \
  "Raw frame inspector, protocol stats, DXL errors, I2C errors, and firmware timing counters are visible."

create_child "$phase4" "Add UI smoke tests and replay fixtures" task 2 "phase:4,area:companion,area:ui,area:test" \
  "Make UI pages testable without hardware by using replay fixtures." \
  "Pages construct in tests and can consume replay telemetry without hardware."

echo "Seeding Phase 5..."
phase5=$(create_epic \
  "Phase 5 - Plotting, session analysis, and URDF visualization" \
  2 \
  "phase:5,area:companion,area:data,area:urdf" \
  "Turn the companion app into the main troubleshooting and calibration tool.")

create_child "$phase5" "Build Plot Workbench page" feature 2 "phase:5,area:companion,area:data,area:ui" \
  "Create live and replay plotting for robot telemetry." \
  "Live and replay plots can display selected fields from servo, leg, control, RC, and sensor streams."

create_child "$phase5" "Add event markers and session notes" feature 2 "phase:5,area:companion,area:data" \
  "Record and visualize user and robot events in sessions." \
  "Gait changes, faults, config commits, and user notes appear on plots and in logs."

create_child "$phase5" "Create hexapod URDF package" feature 2 "phase:5,area:urdf" \
  "Create a URDF model for body, legs, and actuated joints." \
  "URDF loads successfully and includes links and joints for body, six legs, and 18 actuated joints."

create_child "$phase5" "Build URDF Viewer page" feature 2 "phase:5,area:companion,area:urdf,area:ui" \
  "Render the URDF and update it from live or replay joint telemetry." \
  "Model renders and updates from live or replay joint telemetry."

create_child "$phase5" "Implement calibration workflows" feature 1 "phase:5,area:companion,area:firmware,area:ui,priority:safety" \
  "Create guided calibration for servo geometry and sensor baselines." \
  "Servo zero, sign, min/max, leg geometry, and sensor baselines can be calibrated, validated, saved, and exported."

create_child "$phase5" "Implement session export reporting" feature 3 "phase:5,area:companion,area:data" \
  "Export selected data and summaries from recorded sessions." \
  "Selected session data can export to CSV and a human-readable summary report."

echo "Seeding Phase 6..."
phase6=$(create_epic \
  "Phase 6 - Jetson bridge, integration, packaging, and release hardening" \
  2 \
  "phase:6,area:jetson,area:test,area:docs" \
  "Integrate autonomy workflows and make the tools repeatable for daily use.")

create_child "$phase6" "Implement Jetson Python bridge" feature 2 "phase:6,area:jetson,area:protocol" \
  "Use the same protocol client on Jetson for high-level commands and telemetry." \
  "Jetson can send body twist/gait commands and receive telemetry using the same protocol client."

create_child "$phase6" "Prototype optional ROS 2 bridge" feature 3 "phase:6,area:jetson" \
  "Prototype ROS 2 integration on Jetson only." \
  "ROS 2 node maps high-level messages to serial API without changing MCU firmware."

create_child "$phase6" "Implement Mac to Jetson proxy mode" feature 3 "phase:6,area:jetson,area:companion" \
  "Allow Mac companion access through Jetson when direct MCU USB is unavailable." \
  "Mac companion can monitor or control through Jetson when MCU USB is owned by Jetson."

create_child "$phase6" "Add CI and quality gates" task 2 "phase:6,area:test" \
  "Add repeatable local and CI checks." \
  "Firmware unit tests, protocol vectors, Python tests, linting, and type checks run in CI where possible."

create_child "$phase6" "Package companion app for local macOS install" task 3 "phase:6,area:companion" \
  "Create a documented local install/package path for the Mac app." \
  "Documented install and run flow works from a clean Mac developer machine."

create_child "$phase6" "Complete operator and developer documentation" task 2 "phase:6,area:docs" \
  "Write user-facing and developer-facing project docs." \
  "Docs cover wiring, flashing, config, calibration, operation, troubleshooting, logging, plotting, and URDF use."

echo "Seed complete. Created epics:"
echo "  Phase 1: $phase1"
echo "  Phase 2: $phase2"
echo "  Phase 3: $phase3"
echo "  Phase 4: $phase4"
echo "  Phase 5: $phase5"
echo "  Phase 6: $phase6"
echo
bd epic status || true
