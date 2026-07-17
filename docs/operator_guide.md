# Operator Guide

This guide is for bench setup, calibration, operation, session capture, and
first-line troubleshooting. Read it before connecting DYNAMIXEL power.

## Safety Before Wiring

- Keep the robot supported so legs cannot strike a person or bench if a setup
  mistake occurs. Keep the RC kill/disarm control active until the intended
  test explicitly calls for arming.
- Firmware holds the OpenRB-150 DYNAMIXEL power FET off at boot. This is not a
  substitute for a fused external 12 V supply, a safe mechanical setup, or a
  reachable RC kill switch.
- Do not power all 18 MX-28AT servos through the OpenRB-150 DYNAMIXEL port
  current path. Use a properly rated 12 V DYNAMIXEL distribution system with
  fusing and a common ground to the controller.
- Use 3.3 V logic only. The OpenRB-150 logic pins are not 5 V tolerant.
- Do not flash, reset, or power-cycle the controller while a torque-enabled
  robot is unsupported.

## Wiring Checklist

| Connection | Required arrangement |
| --- | --- |
| DYNAMIXEL TTL bus | Connect the servo data bus to the OpenRB-150 `Serial1` DYNAMIXEL TTL bus. The firmware controls the board power FET, while the servo supply must be the external fused 12 V distribution described above. |
| Servo IDs | The default logical map is IDs 1 through 18, leg-major, coxa/femur/tibia. Confirm the actual map before motion; see the [canonical map](ros2_controller_interface_mapping.md#canonical-joint-map). |
| RC receiver | Connect the ExpressLRS CRSF UART to `Serial3`: D14 TX / D13 RX, crossed as a normal UART link, at 420000 baud. Keep the receiver and controller logic at 3.3 V. |
| Foot sensors | Root I2C has the TCA9548A mux at `0x70` and the 24LC32 config EEPROM at `0x50`. Put one Robotic Finger Sensor v2 on each mux channel 0 through 5; channels 6 and 7 are reserved. Each board exposes VCNL4040 proximity at `0x60` and LPS25HB pressure at `0x5C`. |
| Host connection | Use the OpenRB-150 USB CDC port directly for a Mac companion or Jetson bridge. When the Jetson owns USB, one Mac may use its authenticated TCP relay on a trusted network; see the [Jetson relay guide](../jetson/README.md#mac-tcp-relay). Only one high-authority USB client may control motion at once. |

The battery monitor uses `ADC_BATTERY`; validate the measured battery voltage
before any armed test. Do not infer a safe battery from a USB connection alone.

## Build, Flash, and First Connection

Install `just`, `uv`, and PlatformIO. If PlatformIO is installed outside its
default location, set `PIO` to its executable before using root recipes.

```bash
cd <clone>/hexapod
just doctor
just firmware-test
just firmware-build
```

Flash only when the robot is mechanically safe and DYNAMIXEL power is not
needed for the upload:

```bash
just firmware-flash
```

If the SAMD21 bootloader is not detected, double-tap the OpenRB-150 RESET
button, then retry. A reset cuts the board-controlled DYNAMIXEL power path.

Install the companion and confirm a framed-protocol handshake. The USB stream
is binary; prefer `hexapod-cli status` over a plain serial monitor for normal
verification.

```bash
cd companion
uv sync --extra dev
uv run hexapod-cli ports
uv run hexapod-cli status --port <USB_PORT>
```

`status` should report the protocol/firmware identity, safety state, features,
and capabilities. A serial device appearing in `ports` is not proof that it is
the OpenRB-150; a successful `status` response is the required check.

## Safe Operating Order

1. Inspect wiring, mechanical clearance, the external supply, and the RC kill
   state. Start disarmed.
2. Connect USB and run `hexapod-cli status`. Use the companion **Connect** and
   **Overview** pages to confirm state, battery, DXL/I2C health, and command
   source.
3. Use **Mode & Safety** to inspect why a feature or mode is unavailable. An
   unavailable feature is a safety signal, not an invitation to retry blindly.
4. Start with passive pose or a supported bench procedure. Do not send raw
   servo targets during normal walking.
5. For RC walking, arm only after the firmware reports ready state and the
   physical area is clear. RC kill/disarm always wins over host and Jetson.
6. Use **Gait Lab** for high-level gait/twist requests. Firmware may clamp or
   reject a request based on its current authority, limits, battery, DXL, and
   sensor state.

If an E-stop or fault occurs, remove the cause before requesting recovery. Do
not treat `CLEAR_FAULT` as a way to override an active hardware or RC safety
condition.

## Config and Calibration

Persistent configuration belongs in **Servo Config** / **Robot Calibration &
Config**. It contains the EEPROM-backed servo map, zero trims, signs, travel
limits, leg geometry, gait defaults, and foot-sensor calibration.

Use this exact lifecycle:

1. Enter maintenance only while the robot is physically safe. Confirm target
   servo torque is off before any EEPROM-backed servo parameter change.
2. Load the current configuration and make changes in displayed units.
3. Inspect **Diff vs loaded**, then **Stage to robot** and **Validate staged**.
4. Commit only after firmware validation succeeds. Commit is intentionally
   unavailable when the 24LC32 EEPROM is missing or unsafe state prevents it.
5. Export the staged/committed JSON as a calibration backup.

The DXL UI and CLI use logical parameter names. The firmware automatically
maps legacy MX-28 CW/CCW angle limits or MX(2.0) min/max position limits based
on the detected servo profile. Do not use raw register writes as a normal
calibration workflow. EEPROM-requiring writes must be torque-off and read-back
verified.

### Foot Sensor Calibration

With each foot unloaded and live raw pressure telemetry current, use **Copy
live pressure baselines** in Robot Calibration & Config. Set nonzero `near`,
`touch`, and `load` thresholds, keep `load >= touch`, enable only calibrated
feet, then stage, validate, and commit.

The **Foot Contact** page and CLI can re-zero the live estimator for bench
tuning:

```bash
uv run hexapod-cli contact calibrate --foot 255 --port <USB_PORT>
uv run hexapod-cli contact enable --port <USB_PORT>
uv run hexapod-cli feature get --port <USB_PORT>
```

Runtime baseline/threshold changes are not persistent. Copy successful values
back into the staged RobotConfig and commit them if they must survive a reboot.
Firmware may reject contact or terrain-leveling enable requests when the mux,
sensor confidence, calibration, or safety state is unsuitable. Nominal gait
and IK remain available when optional contact features degrade.

## Passive Pose, Logging, Plotting, and URDF

Passive pose mode is the first physical workflow to use for a new robot. It
disables torque and stops goal writes while present positions continue to be
read. Do not manually pose the robot until passive mode is confirmed.

### CLI Workflow

```bash
cd companion

# Request torque-off passive pose mode.
uv run hexapod-cli passive enter --port <USB_PORT>

# Physically pose the torque-off robot while this command records telemetry.
uv run hexapod-cli log \
  --streams health,joint_state,servo_status,contact_state \
  --rate 20 --seconds 60 --name passive_pose --port <USB_PORT>

# Return to normal disarmed operation when recording is complete.
uv run hexapod-cli passive exit --port <USB_PORT>
```

The logger prints the session directory. Replay it in **Plot Workbench** to
compare servo goal/present position, voltage, temperature, contact state, and
other selected signals. From the CLI, export a selected CSV or text report:

```bash
uv run hexapod-cli export-csv data/sessions/<session> \
  --signals health.battery_mv,servo.1.position
uv run hexapod-cli export-report data/sessions/<session>
```

Use **Model Viewer** or **URDF Viewer** to inspect the same `joint_state`
telemetry against the flattened
`robot_ros_simulation/HexNav_description/urdf/HexNav.urdf` model. The viewer
uses the active servo map for tick-to-angle conversion; incorrect sign, trim,
or joint mapping is a calibration problem, not a visual-only issue.

## Troubleshooting

| Symptom | First safe checks |
| --- | --- |
| No upload port | Double-tap RESET, reconnect a known data-capable USB cable, then retry `just firmware-flash`. Do not reset an unsupported torque-enabled robot. |
| Port exists but `status` times out | Check that the selected port is the OpenRB-150, close competing clients, reconnect USB, then retry. A plain serial monitor can corrupt or consume binary frames. |
| No DXL scan or status | DXL power is expected to be off at boot. First inspect the external fused 12 V rail, common ground, bus connection, and maintenance/safety state. Do not repeatedly force power or torque. |
| EEPROM commit rejected | Check capabilities/feature reasons and I2C topology. Missing EEPROM uses volatile safe defaults and intentionally rejects commit. |
| Contact feature unavailable | Use **Sensor Dashboard** or `feature get`; inspect mux `0x70`, each channel 0 through 5, stale/fault state, and calibration before retrying. |
| Telemetry gaps | Lower requested stream rates and inspect `uv run hexapod-cli stream-stats --port <USB_PORT>` for transmit backlog or dropped frames. |
| Heat, voltage, or hardware fault | Stop motion, disarm, remove power if necessary, and diagnose supply capacity, wiring, temperature, and DXL status before clearing a fault. |
| URDF pose looks wrong | Verify the active servo map, sign, trim, and neutral geometry. Do not compensate by changing only the viewer. |

For board-specific details, see the [firmware README](../firmware/openrb150/README.md).
For RC mapping and failsafe semantics, see the
[controller bridge reference](controller_bridge.md).