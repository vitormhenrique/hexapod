# AGENTS.md - Hexapod Robot Monorepo Agent Guide

## 0. Mission

Build a monorepo for a DYNAMIXEL MX-28AT hexapod controlled by an OpenRB-150 / SAMD21 Cortex-M0+ MCU, with an optional Jetson high-level autonomy controller and a Python-first macOS companion application for development, troubleshooting, calibration, data capture, plotting, ROS 2 model streaming, and URDF visualization.

The project must remain useful in these operating modes:

1. **Standalone RC mode** - ExpressLRS commands drive basic walking, sit/stand, gait selection, speed, and emergency stop. The Jetson may be disconnected or powered off.
2. **Contact-aware terrain mode** - the MCU uses six Robotic Finger Sensor v2 foot sensors through a TCA9548A mux to detect touchdown, missed ground, release, and basic support-plane leveling. This mode must be runtime-toggleable through the API.
3. **Jetson-assisted mode** - the Jetson sends high-level motion/autonomy commands while the MCU still owns real-time safety, gait timing, inverse kinematics, servo limits, and final command validation.
4. **Mac development mode** - a Mac connected by USB can inspect, configure, command, plot, log, replay, calibrate, and visualize the robot.
5. **Passive pose streaming mode** - all servo torque is disabled, the MCU streams present servo positions and sensor data, and the Mac/ROS 2/URDF viewer shows the live manually posed robot without commanding movement.

The MCU is the deterministic safety and motion controller. The Jetson and Mac are clients. No client is allowed to bypass MCU safety limits.

## 1. Non-negotiable project rules

### 1.1 Safety rules

- The OpenRB-150 must own arming, disarming, failsafe, torque enable, DYNAMIXEL power enable, DYNAMIXEL bus watchdog, servo position limits, gait timing, and emergency stop.
- The Jetson and Mac may request actions, but every action must be validated by firmware limits before reaching a servo.
- The robot must safely stop if RC input is lost, Jetson heartbeat is stale, USB API client disconnects, DXL errors occur, battery voltage is unsafe, I2C confidence is too low for a requested terrain mode, or firmware watchdog checks fail.
- Never power all 18 MX-28AT servos through the OpenRB-150 DYNAMIXEL port current path. Use a proper high-current 12 V DYNAMIXEL power distribution system, fusing, and common ground.
- Never write DYNAMIXEL EEPROM/registers that require torque-off while torque is enabled. Servo EEPROM writes are maintenance-mode only, torque-off only, and must be read back for verification.
- Never perform unbounded work inside ISRs, serial callbacks, or RTOS high-priority tasks.
- I2C sensor failures must degrade features, not freeze walking. The control loop must never block waiting for an I2C transaction.
- Contact-aware gait and body leveling must be bounded and slow. Contact sensors modify gait timing, foot descent, and body-leveling inputs before IK; they must not directly command raw servo positions after IK.

### 1.2 Engineering rules

- Prefer simple deterministic systems over generic frameworks on the MCU.
- Use static memory on firmware after boot. Avoid heap use, Arduino `String`, dynamic STL containers, and JSON parsers on the MCU.
- Keep protocol schemas versioned and shared across firmware, Jetson bridge, Mac companion, CLI tools, and tests.
- Treat the serial API as a public interface: add tests, examples, and backwards-compatible versioning.
- Use hardware-in-loop tests for firmware features that touch DYNAMIXEL, I2C, RC, power, USB serial, contact sensors, or passive pose streaming.
- Do not use Markdown TODO lists as the source of truth. All work must be represented in Beads issues.
- Any newly discovered task, bug, safety concern, calibration issue, or hardware observation must be captured with `bd create` or `bd remember`.

### 1.3 Feature flags and graceful degradation

The firmware must expose runtime capabilities and feature states. Examples:

```text
feature.i2c_root_present
feature.eeprom_config_present
feature.tca_mux_present
feature.foot_sensor_0_present ... feature.foot_sensor_5_present
feature.foot_contact_available
feature.foot_contact_enabled
feature.terrain_leveling_available
feature.terrain_leveling_enabled
feature.passive_pose_stream_available
feature.passive_pose_stream_enabled
feature.jetson_control_available
feature.jetson_control_enabled
feature.dxl_param_write_available
```

Rules:

- At boot, firmware scans expected I2C devices and DYNAMIXEL devices before enabling features that depend on them.
- Missing optional sensors deactivate dependent features and publish a capability/fault reason.
- Missing 24LC32 config EEPROM must fall back to a compiled safe default config, mark the config as volatile, and reject config commit until EEPROM returns.
- Missing TCA9548A or missing foot sensors must disable contact-aware gait and terrain leveling, while default gait and IK remain usable.
- Runtime I2C failures must mark affected channels stale, reduce confidence, and deactivate contact-aware modes if the data is no longer safe.
- API clients can request feature enable/disable, but firmware may reject or auto-disable a feature if hardware capabilities, robot state, or safety rules do not allow it.

## 2. Beads workflow for agents

This project uses `bd` / Beads as the source of truth for work tracking, decisions, bugs, follow-ups, and project memory.

At session start:

```bash
bd prime
bd ready
```

During work:

```bash
bd show <id>
bd update <id> --claim
bd create "Found issue" -t bug -p 1 -l "area:firmware" --description "..."
bd remember "Important durable project fact"
bd close <id> --reason "Implemented and tested"
```

Project rules:

- Start from `bd ready`, not from comments in code or Markdown task lists.
- Claim work before editing.
- Create child issues when a task grows too large.
- Use labels consistently: `phase:1`, `area:firmware`, `area:companion`, `area:protocol`, `area:ui`, `area:data`, `area:jetson`, `hw:dxl`, `hw:i2c`, `priority:safety`.
- Close issues only after acceptance criteria are satisfied or explicitly superseded by another issue.
- Use `bd remember` for durable context that future agents should know.

## 3. Recommended monorepo layout

Use a single repo so firmware, protocol definitions, robot description, companion app, Jetson integration, and tests evolve together.

```text
hexapod/
  AGENTS.md
  README.md
  pyproject.toml
  uv.lock
  justfile
  .gitignore
  .beads/

  firmware/
    openrb150/
      README.md
      platformio.ini
      arduino-cli.yaml
      src/
        main.cpp
        app/
          control_task.cpp/.h
          dxl_task.cpp/.h
          rc_crsf_task.cpp/.h
          api_task.cpp/.h
          i2c_task.cpp/.h
          health_task.cpp/.h
        board/
          openrb150_pins.h
          board_init.cpp/.h
        config/
          config_schema.h
          config_store.cpp/.h
          eeprom_24lc32.cpp/.h
          defaults.cpp/.h
        dxl/
          dxl_bus.cpp/.h
          dxl_model_detect.cpp/.h
          mx28_legacy_table.h
          mx28_2_table.h
          servo_map.cpp/.h
          servo_maintenance.cpp/.h
          servo_param_api.cpp/.h
        gait/
          gait_engine.cpp/.h
          gait_tables.cpp/.h
          leg_ik.cpp/.h
          body_ik.cpp/.h
          contact_terrain_adapter.cpp/.h
          body_leveling.cpp/.h
        input/
          crsf_parser.cpp/.h
        protocol/
          cobs.cpp/.h
          crc16.cpp/.h
          api_messages.cpp/.h
          feature_flags.h
          telemetry_schema.h
        sensors/
          i2c_discovery.cpp/.h
          tca9548a.cpp/.h
          robotic_finger_v2.cpp/.h
          contact_estimator.cpp/.h
        safety/
          safety_state.cpp/.h
          fault_codes.h
      test/
        unit/
        hil/

  protocol/
    README.md
    schema/
      robot_api.yaml
      robot_config.yaml
      robot_telemetry.yaml
      dxl_param_schema.yaml
      feature_flags.yaml
    generated/
      c/
      python/
      rust/
    tests/
      vectors/

  companion/
    README.md
    src/hexapod_companion/
      __init__.py
      app.py
      main_window.py
      navigation.py
      theme.py
      transport/
        serial_link.py
        cobs.py
        crc16.py
        framing.py
        protocol_client.py
        subscriptions.py
      models/
        robot_state.py
        servo_state.py
        sensor_state.py
        contact_state.py
        config_models.py
        feature_models.py
        session_models.py
      services/
        connection_service.py
        command_service.py
        telemetry_service.py
        logger_service.py
        replay_service.py
        config_service.py
        dxl_param_service.py
        feature_service.py
        ros2_bridge_service.py
      ui/
        pages/
          connect_page.py
          overview_page.py
          mode_safety_page.py
          gait_lab_page.py
          foot_contact_page.py
          leg_lab_page.py
          servo_config_page.py
          servo_tuning_page.py
          sensor_dashboard_page.py
          i2c_devices_page.py
          passive_pose_page.py
          plot_workbench_page.py
          session_browser_page.py
          urdf_viewer_page.py
          diagnostics_page.py
          jetson_ros_page.py
          settings_page.py
        widgets/
          status_badge.py
          emergency_stop.py
          feature_toggle_card.py
          servo_table.py
          live_plot.py
          command_pad.py
          hexapod_topdown.py
          contact_foot_widget.py
          dxl_register_editor.py
      data/
        session_store.py
        parquet_writer.py
        csv_export.py
      urdf/
        urdf_loader.py
        robot_scene.py
        pose_bridge.py
      ros2/
        joint_state_bridge.py
        urdf_state_publisher.py
      cli.py
    tests/
      unit/
      protocol/
      replay/
      ui_smoke/

  robot_description/
    hexapod.urdf.xacro
    hexapod.urdf
    meshes/
    materials/
    calibration/
      default_servo_map.yaml
      default_leg_geometry.yaml
      default_sensor_baselines.yaml

  jetson/
    README.md
    pyproject.toml
    launch/
      camera_gemini336.launch.py
      jetson_bridge.launch.py
      autonomy_stack.launch.py
    config/
      camera_gemini336.yaml
      tf_static.yaml
      autonomy_params.yaml
      traversability_params.yaml
      nav2_params.yaml
      nvblox_params.yaml
    src/hexapod_jetson_bridge/
      serial_client.py
      ros2_bridge.py
      telemetry_publisher.py
      command_publisher.py
      time_sync.py
      watchdog.py
    src/hexapod_autonomy/
      autonomy_supervisor.py
      rc_autonomy_mixer.py
      heading_lock.py
      traversability_grid.py
      obstacle_classifier.py
      local_route_planner.py
      gait_hint_generator.py
      terrain_probe.py
      debug_markers.py
    tests/
      unit/
      replay/
      bag_tests/

  tools/
    flash_firmware.py
    scan_dxl.py
    dump_config.py
    restore_config.py
    record_session.py
    replay_session.py
    plot_session.py
    seed_beads_backlog.sh

  docs/
    architecture.md
    safety.md
    protocol.md
    config_schema.md
    dxl_maintenance.md
    contact_sensors.md
    passive_pose_streaming.md
    calibration.md
    companion_ui.md
    jetson_autonomy.md
    testing.md
    beads_workflow.md

  data/
    sessions/
    exports/
```

If PlatformIO cannot cleanly target OpenRB-150, keep Arduino CLI as the firmware build source of truth. The monorepo should not depend on the Arduino IDE GUI.

## 4. Hardware and firmware baseline

### 4.1 Controller

Target board:

- ROBOTIS OpenRB-150.
- MCU: SAMD21 Cortex-M0+ 32-bit ARM, 48 MHz.
- SRAM: 32 KB.
- Flash: 256 KB with bootloader reserve.
- No internal EEPROM.
- Logic voltage: 3.3 V.
- DYNAMIXEL TTL ports are one TTL bus exposed on four physical ports.
- DYNAMIXEL bus is on `Serial1` with automatic direction control when using Dynamixel2Arduino.
- External serial port is `Serial2`.
- USB CDC `Serial` is available for host API and programming.
- `BDPIN_DXL_PWR_EN` controls DYNAMIXEL power enable on OpenRB-150.
- `ADC_BATTERY` is available for battery monitoring.

Recommended connection assignment:

| Function | Connection |
| --- | --- |
| DYNAMIXEL MX-28AT bus | `Serial1` / OpenRB DXL TTL bus |
| ExpressLRS receiver | `Serial2`, CRSF, uninverted UART |
| Mac companion API | USB CDC `Serial` |
| Jetson API | USB CDC when connected directly, or Mac-to-Jetson proxy when Jetson owns USB |
| 6 Robotic Finger Sensor v2 boards | TCA9548A channels 0-5 |
| 24LC32 EEPROM | Root I2C bus at `0x50`, not behind mux |
| TCA9548A mux | Root I2C bus at `0x70` |

If Mac and Jetson must both be connected at the same time, do not share one MCU USB CDC port directly. Use one of these designs:

1. Jetson connects to the MCU; Mac connects to the Jetson companion proxy over TCP.
2. Mac connects to the MCU; Jetson disabled during development.
3. Add a USB-to-UART bridge or upgrade to a controller with more UARTs.
4. Add a small companion bridge MCU later.

### 4.2 DYNAMIXEL MX-28AT abstraction

The firmware must not hard-code one MX-28 control table without detection. MX-28AT servos may appear as:

1. **Legacy MX-28 Protocol 1.0 table**, where `CW Angle Limit` and `CCW Angle Limit` are the joint angle bounds.
2. **MX firmware upgraded to Protocol 2.0 / MX(2.0) table**, where the equivalent single-turn bounds are `Min Position Limit` and `Max Position Limit`.

At boot or maintenance scan, create a per-servo capability profile:

```text
ServoProfile {
  id
  model_number
  firmware_version
  protocol_version
  table_kind              # mx28_legacy or mx28_2
  supports_sync_read
  supports_fast_sync_read
  supports_cw_ccw_angle_limits
  supports_min_max_position_limits
  supports_profile_velocity
  supports_bus_watchdog
  torque_enabled
  last_error
}
```

Legacy MX-28 table fields used by this project:

| Logical field | Legacy address | Size | Region | Notes |
| --- | ---: | ---: | --- | --- |
| `id` | 3 | 1 | EEPROM | maintenance only |
| `baud_rate` | 4 | 1 | EEPROM | maintenance only |
| `return_delay_time` | 5 | 1 | EEPROM | optional tuning |
| `cw_angle_limit` | 6 | 2 | EEPROM | minimum Goal Position |
| `ccw_angle_limit` | 8 | 2 | EEPROM | maximum Goal Position |
| `temperature_limit` | 11 | 1 | EEPROM | do not raise above safe default |
| `min_voltage_limit` | 12 | 1 | EEPROM | units are 0.1 V |
| `max_voltage_limit` | 13 | 1 | EEPROM | units are 0.1 V |
| `max_torque` | 14 | 2 | EEPROM | maintenance only |
| `status_return_level` | 16 | 1 | EEPROM | maintenance only |
| `shutdown` | 18 | 1 | EEPROM | maintenance only |
| `torque_enable` | 24 | 1 | RAM | 0 off, 1 on |
| `pid_d/i/p` | 26/27/28 | 1 each | RAM | optional tuning |
| `goal_position` | 30 | 2 | RAM | walking command |
| `moving_speed` | 32 | 2 | RAM | optional speed limit |
| `torque_limit` | 34 | 2 | RAM | runtime torque clamp |
| `present_position` | 36 | 2 | RAM | passive pose stream |
| `present_speed` | 38 | 2 | RAM | telemetry |
| `present_load` | 40 | 2 | RAM | inferred load only |
| `present_voltage` | 42 | 1 | RAM | telemetry |
| `present_temperature` | 43 | 1 | RAM | telemetry |
| `moving` | 46 | 1 | RAM | telemetry |
| `punch` | 48 | 2 | RAM | optional tuning |
| `realtime_tick` | 50 | 2 | RAM | optional telemetry |
| `goal_acceleration` | 73 | 1 | RAM | optional tuning |

MX-28(2.0) table fields used by this project:

| Logical field | MX(2.0) address | Size | Region | Notes |
| --- | ---: | ---: | --- | --- |
| `id` | 7 | 1 | EEPROM | maintenance only |
| `baud_rate` | 8 | 1 | EEPROM | maintenance only |
| `operating_mode` | 11 | 1 | EEPROM | position mode initially |
| `protocol_type` | 13 | 1 | EEPROM | verify before normal use |
| `homing_offset` | 20 | 4 | EEPROM | calibration |
| `max_position_limit` | 48 | 4 | EEPROM | maximum single-turn position |
| `min_position_limit` | 52 | 4 | EEPROM | minimum single-turn position |
| `torque_enable` | 64 | 1 | RAM | 0 off, 1 on; EEPROM locks when on |
| `hardware_error_status` | 70 | 1 | RAM | fault telemetry |
| `bus_watchdog` | 98 | 1 | RAM | safety |
| `profile_acceleration` | 108 | 4 | RAM | optional smoothing |
| `profile_velocity` | 112 | 4 | RAM | optional smoothing |
| `goal_position` | 116 | 4 | RAM | walking command |
| `moving` | 122 | 1 | RAM | telemetry |
| `moving_status` | 123 | 1 | RAM | telemetry |
| `present_pwm` | 124 | 2 | RAM | telemetry |
| `present_load` | 126 | 2 | RAM | inferred load only |
| `present_velocity` | 128 | 4 | RAM | telemetry |
| `present_position` | 132 | 4 | RAM | passive pose stream |
| `present_input_voltage` | 144 | 2 | RAM | telemetry |
| `present_temperature` | 146 | 1 | RAM | telemetry |

The firmware API must expose logical parameters, not raw register assumptions. Example: `SET_SERVO_LIMITS(id, min_tick, max_tick)` writes legacy `CW/CCW Angle Limit` on legacy MX-28, and writes `Min/Max Position Limit` on MX(2.0), after torque is disabled and safety checks pass.

### 4.3 Sensors and EEPROM

I2C root bus devices:

- TCA9548A-compatible I2C mux at `0x70`.
- 24LC32/CAT24C32-compatible EEPROM at `0x50`.

Robotic Finger Sensor v2 setup:

- One sensor per TCA9548A channel, channels 0 through 5.
- Channels 6 and 7 reserved for future sensors.
- The sensor board includes VCNL4040 proximity/IR and LPS25HB pressure sensing.
- Keep only one mux channel active while reading fixed-address sensor boards.
- Sensor reads are asynchronous. The gait and IK task consumes latest contact snapshots only.

I2C discovery behavior:

```text
boot:
  scan root bus
  detect EEPROM at 0x50
  detect TCA mux at 0x70
  for mux channel 0..5:
    select channel exclusively
    scan expected sensor addresses
    run lightweight identity/status checks
    mark foot sensor present/missing/faulty
  publish capabilities and feature availability
```

Fallback rules:

- EEPROM missing: load compiled default config, mark config as volatile, reject `CFG_COMMIT`, continue in conservative disarmed/manual-capable mode.
- TCA missing: disable all foot sensor polling, contact-aware gait, and body leveling.
- One foot sensor missing: allow default walking; allow contact mode only if configured policy permits partial sensors. Default policy should reject terrain leveling unless enough stance-leg contact confidence exists.
- Runtime I2C errors: mark channel stale, increment counters, reset TCA if needed, and auto-disable features whose data is unsafe.

EEPROM usage:

- Capacity: 4096 bytes.
- Page write size: 32 bytes.
- Use a two-slot transactional config store with magic, version, sequence, length, payload CRC, and header CRC.
- Load the newest valid slot at boot.
- Write only on explicit config commit.
- Keep a RAM shadow config and validate before commit.
- Do not write EEPROM during active walking.

## 5. Firmware architecture

### 5.1 RTOS baseline

Use FreeRTOS on Arduino SAMD21 for the first implementation. Keep the system small and deterministic.

Initial tasks:

| Task | Priority | Frequency / trigger | Responsibilities |
| --- | ---: | --- | --- |
| `task_control` | high | 50-100 Hz | mode arbitration, safety state, gait phase, contact terrain adapter, body leveling, IK, servo target generation |
| `task_dxl` | high | 50 Hz write in active modes, 10-50 Hz read | sync write goals, passive read-only streaming, status reads, DXL error handling, torque state, maintenance writes |
| `task_rc_crsf` | medium-high | UART event driven | parse ExpressLRS CRSF, normalize channels, failsafe detection |
| `task_api` | medium | USB serial event driven | host protocol, Jetson/Mac heartbeats, commands, config, feature toggles, DXL parameter requests |
| `task_i2c` | medium-low | staggered 50-100 Hz proximity target, 25 Hz pressure target, EEPROM jobs on demand | mux selection, sensor reads, EEPROM config store, I2C discovery/recovery |
| `task_health` | low | 1-10 Hz | watchdog, stack high-water marks, battery, uptime, fault summary, feature health |

Ownership rules:

- Only `task_dxl` directly touches Dynamixel2Arduino and `Serial1`.
- Only `task_i2c` directly touches `Wire`, TCA9548A, 24LC32, and I2C sensors.
- Only `task_rc_crsf` directly reads CRSF bytes from `Serial2`.
- Only `task_api` directly parses and writes USB CDC host frames.
- Cross-task data moves through bounded static queues, ring buffers, event groups, or fixed-size state snapshots.
- `task_control` must never wait on I2C, DXL, EEPROM writes, USB serial, or CRSF parsing.

### 5.2 Control model

Firmware command flow:

```text
RC / Jetson / Mac command
  -> command validation
  -> mode arbiter
  -> gait engine
  -> contact terrain adapter
  -> body leveling transform
  -> body and leg IK
  -> servo map and firmware limits
  -> DXL goal frame
  -> sync write
```

Default command abstraction:

- Body twist: forward, lateral, yaw.
- Body pose: roll, pitch, yaw, x/y/z offsets.
- Gait selection: ripple, tripod, wave, crawl, stand, sit.
- Gait parameters: stride length, step height, duty factor, body height, speed scalar.
- Feature toggles: contact detection, terrain-aware gait, terrain leveling, passive pose stream.
- Maintenance command: individual leg or individual joint control, gated by maintenance mode.

Do not let Jetson or Mac stream raw servo positions in normal walking mode. Raw/individual joint control is allowed only in maintenance mode with lower speed limits, torque guards, and visible UI warnings.

### 5.3 Safety state machine

Required states:

```text
BOOT
CONFIG_LOAD
DISARMED
ARMING_CHECKS
STAND_READY
RC_MANUAL
CONTACT_TERRAIN
JETSON_ASSISTED
MAC_MAINTENANCE
PASSIVE_POSE_STREAM
FAULT_SOFT
FAULT_HARD
ESTOP
```

Important transitions:

- `BOOT -> CONFIG_LOAD` after hardware init.
- `CONFIG_LOAD -> DISARMED` after valid EEPROM config or default config fallback.
- `DISARMED -> ARMING_CHECKS` only with RC arming switch or explicit maintenance command.
- `ARMING_CHECKS -> STAND_READY` only when battery, DXL scan, config, and pose checks pass.
- `STAND_READY -> RC_MANUAL` when RC command is active.
- `STAND_READY -> CONTACT_TERRAIN` only when contact features are enabled and sensor confidence is valid.
- `STAND_READY -> JETSON_ASSISTED` only when RC grants autonomy and Jetson heartbeat is fresh.
- `DISARMED -> PASSIVE_POSE_STREAM` only when torque is disabled and the command source has monitor permission.
- `STAND_READY -> MAC_MAINTENANCE` only when robot is physically safe and maintenance lock is active.
- Any state -> `ESTOP` on RC kill, host estop, critical DXL error, unsafe voltage, or watchdog failure.
- Any state -> `FAULT_HARD` on repeated bus failures or servo hardware errors.

### 5.4 Foot contact detection and terrain leveling

Purpose: use six muxed foot/finger sensors to detect ground contact, adapt gait timing, and maintain slow body leveling on uneven terrain.

The feature must be API-toggleable:

```text
FEATURE_SET foot_contact_enabled true|false
FEATURE_SET terrain_leveling_enabled true|false
```

Firmware may reject enable requests when sensors are missing, stale, or not calibrated.

Per-leg contact state:

```cpp
struct LegContactState {
  uint32_t timestamp_ms;
  uint16_t proximity_raw;
  int32_t pressure_raw;
  int32_t pressure_baseline;
  int32_t pressure_delta;
  uint8_t state;        // AIR, NEAR, TOUCH, LOADED, RELEASE, STALE, FAULT
  uint8_t confidence;   // 0..255
  bool near_surface;
  bool touch;
  bool loaded;
  bool release;
  bool sensor_stale;
  bool sensor_fault;
  int16_t estimated_ground_z_mm;
};
```

State machine:

```text
AIR -> NEAR      proximity crosses near threshold
NEAR -> TOUCH    proximity or pressure delta crosses touch threshold with debounce
TOUCH -> LOADED  pressure remains above load threshold during stance
LOADED -> RELEASE pressure/proximity drops below release threshold
ANY -> STALE     no valid sample before stale timeout
ANY -> FAULT     repeated I2C/device errors
```

Gait modifications:

- Early touchdown during swing: stop lowering that foot, latch estimated ground height, and transition to stance if safe.
- No touchdown by expected ground height: continue lowering slowly up to `extra_drop_limit_mm`.
- Still no touchdown after limit: mark missed ground/drop risk, slow or stop.
- Unexpected contact during swing: abort or retry with higher clearance.
- Unexpected release during stance: mark slip/drop, reduce stride or stop.
- Strong support-plane slope from stance feet: update body roll/pitch/body height slowly with low-pass filtering and rate limits.

Rules:

- Start with wave or crawl/ripple terrain gait, not fast tripod.
- Terrain leveling must use filtered support-plane estimates, not instant pose snapping.
- Contact confidence must be included in telemetry and logs.
- When contact features are disabled, firmware must return to nominal gait behavior and continue streaming raw sensor data if sensor polling is enabled.

### 5.5 Passive pose streaming mode

Purpose: allow a user to physically move the robot while all servos are torque-off, and see the model update in the Mac companion app and ROS 2/URDF tools.

Mode behavior:

```text
enter passive pose stream:
  require disarmed or maintenance-safe state
  set all servo torque off
  stop all goal writes
  continue DXL present-position/status reads
  continue I2C sensor reads if enabled
  publish joint-state telemetry at configured rate
  reject walking/IK/gait commands until mode exits
```

Required telemetry:

- Raw present position per servo.
- Converted joint angle per configured servo map.
- Servo status: voltage, temperature, load/current proxy, error bits when available.
- Contact sensor raw and fused state.
- Robot mode, torque state, and stream rate.

API clients must be able to:

- Enter/exit passive mode.
- Set passive stream rate.
- Subscribe to `passive_joint_state`, `servo_status`, `leg_state`, and `i2c_sensors`.
- Export recorded passive motion sessions for URDF calibration, ROS 2 visualization, and analysis.

### 5.6 DYNAMIXEL maintenance and parameter API

The firmware must expose a safe logical servo-parameter API. It must support setting legacy MX-28 `CW Angle Limit` and `CCW Angle Limit`, and support equivalent MX(2.0) min/max position limits when detected.

Required API operations:

```text
DXL_SCAN
DXL_PING
DXL_GET_SERVO_PROFILE
DXL_GET_PARAM
DXL_SET_PARAM
DXL_SET_SERVO_LIMITS
DXL_SET_TORQUE
DXL_READ_REGISTER          # guarded developer/diagnostic mode
DXL_WRITE_REGISTER         # guarded developer/diagnostic mode, never normal UI default
DXL_REBOOT_SERVO           # maintenance only, optional
DXL_FACTORY_RESET_SERVO    # disabled by default, compile-time or expert-gated
```

Safe write transaction:

```text
request maintenance lock
validate robot is disarmed or safe maintenance state
validate servo ID is known and unique
read current servo profile and table kind
if parameter is EEPROM/torque-off required:
  torque off target servo or all servos as policy requires
  confirm torque off
write logical parameter using correct control table
read back parameter
compare expected vs actual
update RAM config shadow if parameter is part of robot config
emit event and telemetry
require explicit config commit if value should be stored in 24LC32 robot config
```

Logical parameters should include at minimum:

```text
id
baud_rate
return_delay_time
cw_angle_limit
ccw_angle_limit
min_position_limit
max_position_limit
temperature_limit
min_voltage_limit
max_voltage_limit
max_torque
status_return_level
shutdown_mask
pid_p
pid_i
pid_d
moving_speed
torque_limit
goal_acceleration
homing_offset
profile_velocity
profile_acceleration
bus_watchdog
```

The companion UI must prefer logical parameter widgets over a raw register editor. Raw register writes belong in an expert diagnostics panel with warnings and read-back verification.

## 6. Serial API and telemetry

### 6.1 Transport

Use one binary protocol for Mac, Jetson, and test tools.

Recommended frame:

```text
0x00 COBS(versioned_header + payload + crc16) 0x00
```

Header fields:

| Field | Size | Notes |
| --- | ---: | --- |
| magic | 1 | protocol family marker |
| version_major | 1 | incompatible schema changes |
| version_minor | 1 | compatible additions |
| msg_type | 1 | command, response, telemetry, event |
| msg_id | 1 | command or telemetry ID |
| flags | 1 | ack requested, error, fragmented, etc. |
| seq | 2 | monotonically increasing per sender |
| timestamp_ms | 4 | sender uptime if available |
| payload_len | 2 | bytes after header before crc |

Use little-endian integer encoding unless the schema says otherwise.

### 6.2 Required command groups

| Group | Commands |
| --- | --- |
| Session | `HELLO`, `HEARTBEAT`, `GET_CAPABILITIES`, `GET_FEATURES`, `SET_STREAM_RATE`, `SUBSCRIBE`, `UNSUBSCRIBE` |
| Safety | `ESTOP`, `CLEAR_FAULT`, `SET_ARMING`, `SET_MODE`, `GET_STATUS` |
| Feature flags | `FEATURE_GET`, `FEATURE_SET`, `FEATURE_GET_REASONS`, `FEATURE_RESET_DEFAULTS` |
| Motion | `SET_BODY_TWIST`, `SET_BODY_POSE`, `SET_GAIT`, `SET_GAIT_PARAMS`, `STOP_MOTION` |
| Terrain/contact | `CONTACT_ENABLE`, `CONTACT_DISABLE`, `CONTACT_CALIBRATE`, `CONTACT_SET_THRESHOLDS`, `LEVELING_ENABLE`, `LEVELING_DISABLE`, `LEVELING_SET_PARAMS` |
| Passive pose | `PASSIVE_ENTER`, `PASSIVE_EXIT`, `PASSIVE_SET_STREAM_RATE`, `PASSIVE_ZERO_REFERENCE` |
| Maintenance | `ENTER_MAINTENANCE`, `EXIT_MAINTENANCE`, `SET_LEG_TARGET`, `SET_JOINT_TARGET`, `DXL_SCAN`, `DXL_PING`, `DXL_TORQUE` |
| DXL parameters | `DXL_GET_SERVO_PROFILE`, `DXL_GET_PARAM`, `DXL_SET_PARAM`, `DXL_SET_SERVO_LIMITS`, `DXL_READ_REGISTER`, `DXL_WRITE_REGISTER` |
| Config | `CFG_GET_SUMMARY`, `CFG_GET_BLOCK`, `CFG_SET_BLOCK`, `CFG_VALIDATE`, `CFG_COMMIT`, `CFG_RESET_DEFAULTS` |
| I2C/sensors | `I2C_SCAN`, `I2C_GET_TOPOLOGY`, `SENSOR_GET_STATUS`, `SENSOR_CALIBRATE`, `SENSOR_SET_RATE` |
| Logging | `MARK_EVENT`, `GET_LOG_SCHEMA`, `GET_STREAM_STATS` |

### 6.3 Required telemetry streams

The firmware must support rate-limited subscriptions so the Mac can collect data without overloading USB or the MCU.

| Stream | Typical rate | Contents |
| --- | ---: | --- |
| `health` | 1-10 Hz | uptime, state, faults, stack marks, battery, loop timing |
| `capabilities` | event/1 Hz on change | detected DXL profiles, I2C topology, feature availability |
| `feature_state` | event/1-10 Hz | enabled/available/reason for each feature flag |
| `control_state` | 25-100 Hz | active mode, command source, gait phase, body command |
| `servo_status` | 10-50 Hz | id, present position, velocity, load/current proxy, voltage, temperature, error bits |
| `servo_goals` | 25-100 Hz | target joint angle/tick per servo, clamp flags |
| `passive_joint_state` | 10-100 Hz | torque-off present positions and mapped joint angles |
| `servo_params` | event/on request | servo profile and selected logical parameter values |
| `leg_state` | 25-100 Hz | foot targets, IK result, reachability flags |
| `i2c_topology` | event/on request | root devices, mux channels, per-sensor presence/fault state |
| `i2c_sensors_raw` | 10-100 Hz | raw proximity/IR/pressure by channel |
| `contact_state` | 10-100 Hz | AIR/NEAR/TOUCH/LOADED/RELEASE/STALE/FAULT, confidence, estimated ground height |
| `leveling_state` | 5-25 Hz | support plane, body roll/pitch/z correction, confidence |
| `rc_input` | 10-50 Hz | normalized RC channels, link quality if available, failsafe |
| `api_stats` | 1-10 Hz | rx/tx frames, CRC failures, dropped frames, stream backlog |
| `fault_event` | event | fault transitions and causes |

Telemetry must include schema version and units. Companion logs must preserve raw frames when possible, plus decoded typed records.

### 6.4 USB client arbitration

Only one active high-authority USB client may control motion at a time. If the Mac is connected in development mode, the UI must show whether it is:

- Monitor only.
- Maintenance controller.
- Motion controller.
- Config editor.
- Passive pose streaming monitor.

The firmware should expose a lock token for maintenance/config/DXL writes:

1. Client requests lock.
2. Firmware grants only if state allows it.
3. Lock expires unless heartbeat is fresh.
4. Any RC kill switch or estop immediately revokes the lock.

## 7. Companion app product definition

### 7.1 Language and framework choice

Use **Python first** for the companion app.

Recommended stack:

| Area | Recommended choice | Why |
| --- | --- | --- |
| Desktop UI | PySide6 / Qt for Python | Native macOS app feel, mature widgets, good model/view architecture |
| Live plots | pyqtgraph | Fast Qt-native plotting for engineering telemetry |
| Serial | pyserial | Cross-platform serial access and port discovery |
| Protocol models | dataclasses + pydantic where helpful | Simple typed structures and validation |
| Dataframes | polars or pandas | Offline analysis and export |
| Log storage | Parquet via pyarrow, plus DuckDB query support | Efficient session logs and local analytics |
| URDF parsing | yourdfpy first; urdfpy only if needed | Practical URDF load/manipulate/visualize workflow |
| 3D view | embedded Qt/OpenGL or webview-backed local viewer | Keep first version simple; upgrade after profiling |
| CLI tools | Typer or argparse | Scriptable flashing, scanning, logging, replay |
| Packaging | pyproject.toml, optional uv lock, PyInstaller or briefcase later | Repeatable macOS developer install |

Rust is optional later for a shared protocol crate, single-binary host tools, or high-performance logging if Python becomes a proven bottleneck. Do not start with Rust for the UI unless profiling demands it.

### 7.2 Modern UI principles

The companion app should feel like an engineering control console, not a pile of scripts.

Global layout:

- Left navigation rail with clear sections: Connect, Operate, Tune, Analyze, Visualize, Diagnose, Jetson/ROS.
- Global top safety bar visible on every screen: connection state, robot mode, arming state, torque state, active command source, RC status, Jetson heartbeat, battery, and emergency stop.
- Global event strip for recent faults, feature auto-disables, config commits, DXL writes, and sensor disconnects.
- Command buttons must show whether they are monitor-only, require maintenance lock, require disarmed state, or require arming.
- Dangerous operations require staged changes, clear diff, explicit confirmation, read-back verification, and session log events.
- UI must never freeze while serial, logging, plotting, ROS 2, or file writes are active.

### 7.3 Companion app screens

1. **Connect and Setup**
   - USB port discovery.
   - Handshake, firmware version, protocol version, capability view.
   - Connect/disconnect/reconnect controls.
   - First-run checklist and hardware topology summary.

2. **Overview**
   - Robot state, arming state, active command source, RC status, Jetson status, Mac lock status.
   - Battery, firmware uptime, DXL bus health, I2C health, current gait.
   - Top-down hexapod mini-map with leg contact states and servo health badges.

3. **Mode and Safety Center**
   - Arm/disarm, estop, clear fault.
   - Enter/exit passive pose stream.
   - Enter/exit maintenance lock.
   - Enable/disable feature flags: contact detection, terrain leveling, sensor polling, Jetson authority.
   - Show why a mode or feature is unavailable.

4. **Gait Lab**
   - Select gait: stand, sit, tripod, ripple, wave, crawl.
   - Adjust speed, stride length, step height, body height, stance width, duty factor.
   - Keyboard/gamepad/onscreen body twist controls.
   - Show command source mix: RC, Mac, Jetson.
   - Record gait changes as session events.

5. **Foot Contact and Leveling**
   - Enable/disable touchdown detection and terrain leveling through API.
   - Show each leg state: AIR, NEAR, TOUCH, LOADED, RELEASE, STALE, FAULT.
   - Live raw proximity/pressure, pressure baseline, pressure delta, confidence.
   - Threshold editor with staged changes and calibration capture.
   - Support-plane visualization and body roll/pitch/z correction.
   - Auto-disable reasons and fallback state.

6. **Leg Lab**
   - Select one leg or all legs.
   - Move foot target in x/y/z with sliders and numeric entry.
   - Show IK result, reachability, joint clamps, servo ticks.
   - Maintenance-mode gate required before sending commands.

7. **Servo Map and DXL Tuning**
   - Scan DYNAMIXEL IDs and profiles.
   - View servo-to-leg mapping from EEPROM shadow config.
   - Edit ID, leg, joint role, sign, zero offset, min/max ticks, speed limits.
   - Set legacy `CW Angle Limit` and `CCW Angle Limit` through logical fields.
   - Set MX(2.0) `Min/Max Position Limit` through the same logical UI when applicable.
   - Edit safe parameters such as return delay, PID gains, speed/acceleration limits, torque limit, and status return level.
   - Stage, diff, write, read-back verify, commit to 24LC32 config, export/import YAML/JSON.
   - Raw register editor is expert-only and disabled by default.

8. **Sensor Dashboard and I2C Explorer**
   - Show root I2C scan, TCA9548A mux state, channel presence, error counters.
   - Show live Robotic Finger Sensor v2 values on channels 0-5.
   - Sensor polling rate controls.
   - TCA reset/recovery action if wired.

9. **Passive Pose and ROS 2 Model Stream**
   - Enter torque-off passive mode.
   - Display live joint positions from DXL present position telemetry.
   - Stream or export joint states for ROS 2 and URDF visualization.
   - Record manually posed calibration sessions.
   - Compare measured pose against neutral/expected pose.

10. **Plot Workbench**
    - Select live stream or recorded session.
    - Plot servo goal vs present, load, velocity, temperature, voltage, contact states, pressure, proximity, gait phase, RC input, feature state, loop timing.
    - Add event markers and annotations.
    - Export plot presets and CSV slices.

11. **Session Browser**
    - List recorded sessions with metadata, robot config hash, firmware version, notes, and fault summary.
    - Replay sessions into plots, overview, contact page, and URDF viewer.
    - Compare two sessions.

12. **URDF / 3D Viewer**
    - Load `robot_description/hexapod.urdf`.
    - Show static robot model.
    - Overlay live or replayed joint states.
    - Show foot contact markers, commanded foot targets, estimated ground plane, and body pose.
    - Provide calibration overlay for servo zero/sign/min/max work.

13. **Diagnostics**
    - Raw frame inspector.
    - Protocol stats, CRC failures, dropped frames.
    - DXL errors by servo.
    - I2C scan and per-channel errors.
    - Firmware performance counters and task stack margins.
    - Feature auto-disable history.

14. **Jetson and ROS 2**
    - Show Jetson heartbeat, autonomy state, command TTL, RC/autonomy mix, and selected route action.
    - ROS 2 topic status when the bridge is active.
    - Option to proxy Mac companion traffic through Jetson when Jetson owns the MCU USB cable.

15. **Settings**
    - Default log directory.
    - UI update rate.
    - Plot retention window.
    - Safety prompts.
    - Theme and window settings.

The emergency stop control must remain globally accessible from every page.

### 7.4 Companion architecture

Use a service-oriented UI architecture:

```text
Serial reader thread / async task
  -> frame decoder
  -> protocol client
  -> typed telemetry bus
  -> state store
  -> UI models
  -> pages/widgets

Command UI
  -> command service
  -> safety confirmation if needed
  -> protocol client
  -> framed serial write
```

Rules:

- The UI thread must never block on serial reads, file writes, DXL operations, config commits, ROS 2, or plotting.
- Log writes must be asynchronous and buffered.
- Use immutable or copy-on-write snapshots for robot state shown in UI.
- Keep protocol encode/decode independent of PySide6 so CLI tools can use it.
- Keep session replay independent of live serial so plots and UI can be tested without hardware.
- Use a typed state store so pages do not parse raw protocol payloads directly.

### 7.5 Data logging model

Each recording session should create:

```text
data/sessions/YYYY-MM-DD_HHMMSS_<robot-name>/
  session.json
  raw_frames.bin
  telemetry.parquet
  events.parquet
  config_snapshot.yaml
  i2c_topology.json
  dxl_profiles.json
  robot_state_summary.json
```

Minimum long-form telemetry fields:

| Field | Notes |
| --- | --- |
| `session_id` | stable ID |
| `host_time_ns` | Mac timestamp when decoded |
| `robot_time_ms` | MCU timestamp |
| `stream` | stream name |
| `entity_type` | servo, leg, sensor, control, health, feature, i2c |
| `entity_id` | servo id, leg index, sensor channel, etc. |
| `field` | present_position, goal_position, pressure, contact_state, feature_enabled, etc. |
| `value` | numeric value when applicable |
| `unit` | ticks, degrees, volts, celsius, raw, enum, etc. |
| `quality` | ok, stale, clamped, estimated, dropped, fallback |

Use Parquet for efficient long-form logs and DuckDB for local queries. Export CSV for interoperability.

## 8. Jetson autonomy design

The Jetson layer is an optional autonomy advisor. It may perceive the world, plan short-horizon routes, and request body-level motion from the MCU, but it must not send raw servo ticks during normal walking. The OpenRB-150 firmware remains the final safety gate for arming, gait timing, IK, joint limits, servo torque, watchdogs, and RC override.

Recommended Jetson responsibilities:

- Gemini 336 camera bring-up through ROS 2.
- Camera-to-body TF calibration.
- Visual odometry or visual-inertial odometry.
- Local 3D mapping and 2D/2.5D traversability grid.
- Climbable-vs-bypass obstacle classifier.
- Heading-preserving local route planner.
- RC/autonomy mixer.
- Gait hint generator: speed, clearance, stride, gait mode.
- Contact-feedback terrain confidence using MCU telemetry from servos and foot sensors.

The MCU and Jetson contract:

- Jetson sends high-level body twist, gait hints, and autonomy intent only.
- Jetson commands must have a TTL and heartbeat.
- Firmware rejects stale or unsafe commands.
- RC kill/disarm always wins.
- Contact-aware terrain features on the MCU remain usable with or without Jetson.

## 9. Phase plan

The first two phases are firmware-only by design. Companion work starts only after the firmware exposes a stable enough USB protocol and telemetry stream.

### Phase 1 - Firmware foundation and safe I/O

Goal: boot the OpenRB-150 firmware reliably, establish bus ownership, prove safe DXL/I2C/RC/API basics, and create a validated configuration store.

Deliverables:

- Build system for OpenRB-150.
- FreeRTOS boot and health task.
- Board HAL and DXL power enable control.
- USB serial protocol skeleton with `HELLO`, `HEARTBEAT`, `GET_STATUS`, `GET_CAPABILITIES`, and feature-state reporting.
- COBS + CRC framing with tests/golden vectors.
- DYNAMIXEL scan/ping/status read in maintenance-safe mode.
- DYNAMIXEL protocol/model/table detection.
- Passive torque-off servo status read prototype.
- TCA9548A mux select and root/channel I2C scan.
- I2C device discovery, capabilities, and feature fallback reasons.
- 24LC32 page read/write and transactional config store.
- CRSF parser with channel normalization and failsafe detection.
- Fault state machine skeleton.
- Hardware-in-loop smoke test checklist.

Exit criteria:

- Firmware boots, reports version/status/capabilities over USB, and does not enable DXL power until commanded.
- DXL scan works with one servo and then full bus, including per-servo profile detection.
- I2C scan can identify root mux/EEPROM and per-channel sensor devices.
- Missing I2C devices cause feature deactivation and clear reported reasons, not boot failure.
- EEPROM config load/commit survives power cycle and rejects corrupt slots.
- RC failsafe transitions are visible in status telemetry.
- No heap allocations after boot in normal operation.

### Phase 2 - Firmware motion, sensors, config API, telemetry, and maintenance features

Goal: make the robot controller useful without the companion app: safe walking, servo mapping, IK/gait control, contact sensing, feature toggles, servo maintenance API, passive pose streaming, config updates, telemetry, and RC/Jetson/Mac arbitration.

Deliverables:

- EEPROM-backed servo map, servo profiles, sensor calibration, feature defaults, and leg geometry config.
- Config API: get/set/validate/commit/reset.
- Feature API: enable/disable contact detection, terrain leveling, sensor polling, Jetson authority, and passive pose streaming where safe.
- 3-DOF leg IK and body transform model.
- Gait engine: stand, sit, tripod, ripple, wave/crawl.
- Contact estimator for six Robotic Finger Sensor v2 boards.
- Terrain-aware gait adapter for early touchdown, late touchdown, missed ground, slip/release, and retry/stop behavior.
- Body leveling from stance-foot support plane with filtering and clamps.
- Servo target limiting and clamp reporting.
- DXL Sync Write for all goal positions.
- DXL status telemetry for position, velocity, load/PWM, voltage, temperature, and errors.
- Passive pose streaming mode: torque off, no goal writes, present-position stream active.
- DXL parameter API for logical parameters, including legacy `CW/CCW Angle Limit` and MX(2.0) `Min/Max Position Limit` handling.
- Robust I2C runtime recovery and feature fallback.
- Control arbitration among RC, Jetson, and Mac maintenance lock.
- Rate-limited telemetry subscriptions for raw and fused data.
- Robust safety/fault transitions.
- Bench tests for one leg, then suspended robot, then ground testing.

Exit criteria:

- RC can arm/disarm, sit/stand, select gait, and command low-speed walking.
- Jetson can be off with no negative effect.
- Host can read and update EEPROM config through protocol commands.
- Host can toggle foot contact detection and terrain leveling; firmware rejects unsafe enable attempts with clear reasons.
- Host can stream raw sensor data and fused contact state.
- Passive pose mode disables all servo torque and streams positions to CLI/UI/ROS 2 model consumers.
- Host can set servo logical limits, including CW/CCW angle limits on legacy MX-28, only in safe maintenance state with read-back verification.
- Telemetry streams are decoded by a CLI test client.
- Servo temperature/voltage/error faults trigger safe behavior.
- Maintenance mode can command one leg with visible clamp and reachability feedback.

### Phase 3 - Companion transport, protocol client, and data logger

Goal: create a scriptable Python package that connects to the robot, decodes telemetry, sends safe commands, and records sessions before building the full UI.

Deliverables:

- Python package scaffold under `companion/`.
- Shared protocol schema import/generation or manual first-pass models.
- COBS + CRC encode/decode matching firmware golden vectors.
- Serial port discovery and connection lifecycle.
- Typed protocol client for status, subscribe, config, feature toggles, DXL parameters, passive pose mode, and logging.
- CLI tools: connect, status, stream, log, replay, config dump/restore, features, i2c scan, contact calibrate, passive pose, DXL params.
- Session logger writing raw frames and decoded Parquet/CSV.
- Replay reader that feeds decoded telemetry to tests and future UI.

Exit criteria:

- `hexapod-cli status` connects and prints firmware capabilities.
- `hexapod-cli features` shows available/enabled/unavailable feature reasons.
- `hexapod-cli contact enable` and `hexapod-cli contact disable` exercise the API safely.
- `hexapod-cli passive enter --stream-rate 50` records present joint positions with torque off.
- `hexapod-cli dxl set-limits <id> <min> <max>` stages/writes/verifies safe servo limits in maintenance mode.
- `hexapod-cli log --streams servo_status,contact_state,i2c_sensors_raw` records a session.
- Replay tests can run without hardware.

### Phase 4 - Companion UI foundation and core pages

Goal: build the first usable macOS development UI with clear navigation and safe controls.

Deliverables:

- PySide6 app shell with navigation rail, theme, status bar, event strip, and global estop.
- Connect and Setup page.
- Overview page.
- Mode and Safety Center page.
- Gait Lab page.
- Foot Contact and Leveling page.
- Leg Lab page.
- Servo Map and DXL Tuning page.
- Sensor Dashboard and I2C Explorer page.
- Passive Pose page.
- Diagnostics/raw frame page.
- UI smoke tests for page construction.

Exit criteria:

- App launches on macOS from the monorepo.
- UI can connect/disconnect without freezing.
- Live status and feature state updates are visible.
- Commands require correct safety state/lock.
- Contact detection can be enabled/disabled through UI and its data streams live.
- Passive pose mode can be entered/exited through UI and servo positions update the model state.
- DXL parameter writes are staged, warned, verified, and logged.
- Emergency stop is visible and functional from all pages.

### Phase 5 - Plotting, session analysis, URDF visualization, and ROS 2 model stream

Goal: turn the companion app into the main troubleshooting and calibration tool.

Deliverables:

- Plot Workbench with live and replay modes.
- Servo goal vs present position plots.
- Load/temperature/voltage plots.
- I2C raw sensor and fused contact plots.
- Feature-state and auto-disable event plots.
- Event markers and session notes.
- Session Browser.
- URDF model under `robot_description/`.
- URDF Viewer page with live/replay/passive joint-state overlay.
- ROS 2 joint-state bridge for passive pose/model streaming.
- Calibration workflow for servo zero offsets, signs, min/max limits, leg geometry, and sensor baselines.
- Export report for a session.

Exit criteria:

- A recorded walking or passive-pose session can be replayed and plotted.
- URDF viewer shows the robot and updates joint pose from telemetry.
- ROS 2 model consumers can receive joint states from passive pose telemetry.
- Calibration edits can be staged, validated, committed, exported, and restored.
- Data from all servos and all I2C sensors can be saved and plotted.

### Phase 6 - Jetson bridge, integration, packaging, and release hardening

Goal: make Jetson integration possible without yet depending on autonomy. The robot should still be safe and useful if the camera/autonomy stack is disabled.

Deliverables:

- Jetson Python bridge using the same protocol client as the Mac app.
- ROS 2 bridge that maps MCU telemetry to ROS topics and safe high-level ROS commands back to the MCU.
- Mac-to-Jetson proxy mode for cases where the Jetson owns the MCU USB cable.
- Mac companion packaging for local install.
- CI for firmware unit tests, protocol golden vectors, Python tests, linting, and type checks.
- Hardware-in-loop test scripts and checklists.
- Documentation for wiring, flashing, calibration, operation, troubleshooting, data analysis, passive streaming, DXL maintenance, and contact mode.

Exit criteria:

- Jetson can command high-level body twist/gait while MCU retains safety authority.
- Mac can monitor via Jetson proxy if direct USB is unavailable.
- ROS 2 bridge can publish MCU telemetry and accept safe body-level commands.
- A new developer can clone the repo, install tools, flash firmware, connect UI, enter passive pose, record data, and plot it by following README instructions.

### Phase 7 - Jetson autonomy, depth perception, and terrain-aware navigation

Goal: implement optional Jetson autonomy: avoid non-climbable obstacles, climb small safe obstacles, preserve mission heading during detours, and blend autonomy with RC input.

Deliverables:

- Gemini 336 ROS 2 camera bring-up and recording profiles.
- Camera-to-body TF calibration and URDF integration.
- Visual odometry / state estimation pipeline.
- Local 3D mapping with a costmap for obstacle avoidance.
- Robot-centric terrain/traversability grid from depth and pose.
- Climbable-vs-bypass obstacle classifier.
- Heading-preserving local route planner.
- RC/autonomy mixer with manual nudge, speed scale, yaw override, and RC-owned kill/disarm.
- Gait hint generator for speed, clearance, stride, and gait mode.
- Contact-feedback terrain probing from servo telemetry and foot/finger sensor data.
- Replay tests using recorded bags and MCU logs.
- RViz/Foxglove/Mac companion debug overlays for map, route, selected action, risk, and command source.

Exit criteria:

- Jetson can walk the robot on flat floor under RC-enabled autonomy while preserving firmware failsafes.
- Jetson can detect an obstacle in the intended corridor and choose stop, around, or over from deterministic risk scores.
- Around-route tests keep the original mission heading unless the RC yaw stick or explicit heading-reset command changes it.
- Known low-step obstacles can be climbed slowly only when height, slope, foothold, confidence, and contact checks pass.
- Unsafe, unknown, reflective, moving, or drop-off scenarios result in slow/stop or bypass, not blind climbing.
- Every autonomy behavior has replay tests before ground testing.

## 10. Beads backlog requirements

Seed the project using `tools/seed_beads_backlog.sh`. The script must create epics for all phases and child issues for:

- Firmware safe boot, RTOS task skeleton, protocol framing, USB API, DXL discovery, I2C discovery, EEPROM config, CRSF, and HIL smoke tests.
- Firmware IK/gait, contact estimator, terrain-aware gait, body leveling, API feature toggles, passive pose mode, DXL parameter writes, I2C fallback, telemetry, and safety tests.
- Companion protocol client, CLI, logger, replay, UI shell, contact page, passive pose page, DXL tuning page, plots, session browser, URDF/ROS 2 model streaming, and calibration workflows.
- Jetson bridge and later autonomy features.

Agents may split tasks further, but must not delete the firmware-first phase ordering.

## 11. Development command conventions

Suggested root commands:

```bash
just firmware-build
just firmware-test
just firmware-flash
just protocol-test
just companion-test
just companion-run
just companion-log
just seed-beads
```

If `just` is not used, document equivalent commands in `README.md`.

Firmware commands should be repeatable without opening Arduino IDE.

Companion commands should run from a clean Python environment:

```bash
uv sync
uv run hexapod-cli status
uv run hexapod-companion
```

## 12. Whole-project acceptance checklist

The project is considered healthy when:

- Firmware boots safely with DXL power off by default.
- Firmware reports capabilities, feature flags, DXL profiles, and I2C topology.
- Robot can walk slowly from RC without Jetson.
- Contact detection and terrain leveling can be enabled/disabled by API and UI.
- Missing or failing I2C sensors disable dependent features and fall back to safe defaults.
- Passive pose stream disables all servo torque, reads present positions, and drives the companion/ROS 2 model.
- Servo parameter writes, including MX-28 legacy CW/CCW limits, are maintenance-only, torque-off, read-back verified, and logged.
- Mac companion can connect, show live status, command safe modes, log all servo/sensor data, plot sessions, and visualize URDF.
- Jetson can be disconnected without affecting basic RC walking.
- Jetson autonomy can be enabled only through RC-approved, heartbeat-checked control.
- All dangerous actions are explicit, staged, logged, and recoverable.
- Beads contains current project work, decisions, bugs, and follow-ups.

<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:ca08a54f -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

## Session Completion

**When ending a work session**, you MUST complete ALL steps below. Work is NOT complete until `git push` succeeds.

**MANDATORY WORKFLOW:**

1. **File issues for remaining work** - Create issues for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **PUSH TO REMOTE** - This is MANDATORY:
   ```bash
   git pull --rebase
   bd dolt push
   git push
   git status  # MUST show "up to date with origin"
   ```
5. **Clean up** - Clear stashes, prune remote branches
6. **Verify** - All changes committed AND pushed
7. **Hand off** - Provide context for next session

**CRITICAL RULES:**
- Work is NOT complete until `git push` succeeds
- NEVER stop before pushing - that leaves work stranded locally
- NEVER say "ready to push when you are" - YOU must push
- If push fails, resolve and retry until it succeeds
<!-- END BEADS INTEGRATION -->
