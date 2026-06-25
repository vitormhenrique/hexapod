# Companion App Product Plan

## Purpose

The companion app is the main development console for the hexapod. It should make firmware, servo, gait, contact-sensor, logging, calibration, passive-pose, URDF, and ROS 2 model workflows clear enough to use every day without writing one-off scripts.

Python is the default implementation language. Use PySide6 / Qt for Python for the native UI, pyserial for USB, pyqtgraph for live plots, Parquet/DuckDB for session analysis, and a protocol package that is independent of the UI.

## Product principles

- The app is safety-first. Emergency stop, torque state, arming state, active command source, RC status, battery, DXL health, I2C health, and feature state are always visible.
- The app is hardware-aware. It shows what the firmware actually detected: servo profiles, I2C topology, EEPROM availability, foot sensor presence, and feature fallback reasons.
- The app is staged for dangerous actions. Servo limit writes, DXL register writes, config commits, and calibration saves require diff, confirmation, write, read-back verification, and logging.
- The app is replayable. Every screen that displays live data should also be able to consume a saved session replay.
- The app is non-blocking. Serial I/O, logging, ROS 2, plotting, DXL operations, and file writes must not freeze the UI.

## Global shell

Use a left navigation rail and a global top bar.

Global top bar fields:

- Connection state.
- Firmware/protocol version.
- Robot mode.
- Arming state.
- Torque state.
- Active command source.
- RC link/failsafe state.
- Jetson heartbeat.
- Battery state.
- DXL bus health.
- I2C health.
- Global emergency stop.

Global event strip:

- Faults.
- Feature auto-disables.
- Config commits.
- DXL writes.
- Contact mode transitions.
- Passive pose enter/exit.
- I2C device loss/recovery.

## Screens

### 1. Connect and Setup

Features:

- USB serial port discovery.
- Connect/disconnect/reconnect.
- Handshake and protocol version check.
- Firmware capabilities and build info.
- Hardware topology summary.
- First-run checklist.

### 2. Overview

Features:

- Robot state summary.
- Battery, uptime, loop timing, task stack margins.
- DXL bus and per-servo health badges.
- I2C topology and per-channel health badges.
- Current gait, body command, mode, and command source.
- Top-down hexapod diagram with contact and servo states.

### 3. Mode and Safety Center

Features:

- Arm/disarm.
- Estop and clear fault.
- Maintenance lock request/release.
- Passive pose enter/exit.
- Feature toggles:
  - foot contact detection,
  - terrain leveling,
  - I2C sensor polling,
  - Jetson authority,
  - passive pose streaming.
- Unavailable-feature reasons from firmware.
- Safety checklist for each mode.

### 4. Gait Lab

Features:

- Stand, sit, tripod, ripple, wave, crawl.
- Speed, stride length, step height, stance width, body height, duty factor.
- Body twist command pad.
- Keyboard/gamepad control.
- RC/Mac/Jetson command-source mix display.
- Event marking and session recording.

### 5. Foot Contact and Leveling

Features:

- API buttons to enable/disable contact detection and terrain leveling.
- Per-leg state: AIR, NEAR, TOUCH, LOADED, RELEASE, STALE, FAULT.
- Raw proximity, pressure, baseline, delta, confidence, estimated ground height.
- Threshold and debounce editor.
- Calibration capture.
- Support-plane visualization.
- Leveling roll/pitch/z correction display.
- Auto-disable reasons and I2C fallback state.

### 6. Leg Lab

Features:

- Select one leg, mirrored leg pairs, or all legs.
- Foot target x/y/z controls.
- IK visualization.
- Reachability and clamp flags.
- Maintenance-only command gate.
- Servo tick and angle view.

### 7. Servo Map and DXL Tuning

Features:

- DXL scan and per-servo profile table.
- Servo-to-leg mapping editor.
- Servo sign, zero, min/max ticks, speed limits.
- Logical servo limit editor:
  - legacy MX-28 CW Angle Limit / CCW Angle Limit,
  - MX(2.0) Min Position Limit / Max Position Limit.
- Other DXL parameters:
  - return delay,
  - PID gains,
  - moving speed / profile velocity,
  - acceleration,
  - torque limit,
  - status return level,
  - voltage and temperature limits.
- Staged diff.
- Torque-off write workflow.
- Read-back verification.
- 24LC32 config commit.
- Expert raw register editor disabled by default.

### 8. Sensor Dashboard and I2C Explorer

Features:

- Root I2C devices.
- TCA9548A mux channels.
- Per-channel scan result.
- Six Robotic Finger Sensor v2 live panels.
- Polling rates and stale timers.
- Error counters.
- TCA reset/recovery action if wired.

### 9. Passive Pose and ROS 2 Model Stream

Features:

- Enter torque-off passive stream mode.
- Show present position for all servos.
- Show mapped joint angles.
- Record passive pose sessions.
- Publish or export joint states for ROS 2.
- Feed the URDF viewer with live manually posed joints.
- Calibration comparison against neutral pose.

### 10. Plot Workbench

Features:

- Live and replay plotting.
- Plot presets for:
  - servo goal vs present,
  - load/velocity/voltage/temperature,
  - contact state and pressure,
  - gait phase,
  - RC input,
  - feature state,
  - loop timing,
  - API drops/errors.
- Event markers.
- Plot notes.
- CSV export.

### 11. Session Browser

Features:

- List sessions by date, robot name, firmware, config hash, notes, faults.
- Replay into every compatible page.
- Compare two sessions.
- Export selected telemetry.

### 12. URDF / 3D Viewer

Features:

- Load hexapod URDF.
- Display live, replay, or passive joint pose.
- Overlay foot contact markers.
- Overlay commanded foot targets.
- Overlay estimated ground plane and body pose.
- Use for servo zero/sign/min/max calibration.

### 13. Diagnostics

Features:

- Raw frame inspector.
- Protocol statistics.
- CRC failures and dropped frames.
- DXL errors by servo.
- I2C errors by bus/channel.
- Firmware task timing and stack margins.
- Feature auto-disable history.
- Expert DXL register read/write panel.

### 14. Jetson and ROS 2

Features:

- Jetson heartbeat.
- ROS 2 bridge topic status.
- Autonomy enabled/disabled.
- Command TTL and stale-command state.
- RC/autonomy mixer display.
- Current autonomy action: stop, straight over, strafe around, arc around, backup.
- Mac-to-Jetson proxy status.

### 15. Settings

Features:

- Log directory.
- UI update rate.
- Plot retention window.
- Theme.
- Safety confirmation settings.
- Default stream presets.

## Implementation architecture

```text
Serial reader thread / async task
  -> frame decoder
  -> protocol client
  -> typed telemetry bus
  -> state store
  -> Qt models
  -> pages/widgets

Command UI
  -> command service
  -> safety policy
  -> protocol client
  -> serial writer
```

Core services:

- ConnectionService.
- ProtocolClient.
- TelemetryService.
- FeatureService.
- ConfigService.
- DxlParamService.
- ContactService.
- PassivePoseService.
- LoggerService.
- ReplayService.
- PlotDataService.
- Ros2BridgeService.

## Minimum first usable app

A strong first UI milestone is:

1. Connect and Overview.
2. Mode and Safety Center.
3. Sensor Dashboard with I2C topology.
4. Passive Pose page.
5. Servo Map and DXL Tuning page.
6. Plot Workbench with live/replay support.

That gives immediate value for development before all walking UI features are complete.
