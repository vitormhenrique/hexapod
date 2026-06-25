# Phase 1 Hardware-in-Loop (HIL) Smoke Test

Manual + scripted bring-up checks for the OpenRB-150 firmware foundation
(Phase 1: safe boot, USB API, DXL bus manager, I2C/EEPROM, CRSF). Run these
after every firmware change that touches I/O, before any Phase 2 motion work.

The automated USB portion is driven by [`tools/hil_smoke.py`](../tools/hil_smoke.py);
everything that needs physical interaction or 12 V power is a manual step here.

## Safety preconditions

- **No servo torque, no motion in Phase 1.** Nothing in this checklist commands
  a servo to move. DXL power stays OFF unless a step explicitly enables it on a
  bench with a single servo.
- Keep the robot on a stand or with legs clear of surfaces.
- Have the RC kill switch reachable before powering the DXL bus.
- Use a current-limited 12 V supply for the first DXL power-on.

## Equipment

- OpenRB-150 flashed with the current `openrb150` firmware build.
- USB-C cable to the host (Mac).
- 12 V DXL power distribution (only for the DXL bench step).
- At least one MX-28AT servo for the DXL step.
- TCA9548A + 24LC32 + at least one Robotic Finger Sensor v2 (for I2C/EEPROM).
- ExpressLRS receiver + bound transmitter (for the CRSF step).

## 1. Boot (USB power only)

1. Power the board from USB only (DXL power supply OFF).
2. Observe LEDs:
   - [ ] USER LED blinks (the `blink` task / scheduler is alive).
   - [ ] DXL bus LED is **RED** (DXL power FET is OFF at boot, as required).
3. [ ] No brown-out reset loop (LED pattern is steady, not restarting).

## 2. USB API handshake (automated)

Run:

```bash
uv run python tools/hil_smoke.py --port <PORT>
# or: PYTHONPATH=protocol/python python tools/hil_smoke.py --port <PORT>
```

Expected automated checks (all PASS):

- [ ] **USB HELLO protocol** — reports protocol v0.1.
- [ ] **USB HELLO device identity** — name `OpenRB150-Hex`, firmware version.
- [ ] **DXL power OFF at boot** — `GET_STATUS` shows `dxl_power=False`.
- [ ] **No watchdog misses** — `watchdog_missed == 0` at idle.
- [ ] **Status telemetry decodes** — state / battery mV / uptime are plausible.
- [ ] **Heartbeat uptime advances** — proves the RTOS scheduler is running.
- [ ] **USB GET_CAPABILITIES** — feature bitmap decodes.

If `--port` is omitted the script auto-detects a single USB CDC port. Use
`--list` to enumerate ports.

## 3. DYNAMIXEL bus (bench, single servo)

> Requires 12 V DXL power. Keep the servo unloaded.

1. Connect one MX-28AT to the DXL bus; apply 12 V DXL power.
2. Enable DXL power via firmware (Phase 2 command or bench scan path), then run
   a maintenance scan (`DxlBus::scan` / Phase 2 `DXL_SCAN`).
3. Confirm:
   - [ ] Servo is discovered at its expected ID.
   - [ ] Model/protocol detected correctly (MX-28 legacy = Protocol 1.0 by
         default; MX-28(2.0) if reflashed).
   - [ ] Torque reported **OFF** (scan is read-only / maintenance-safe).
   - [ ] Present position / voltage / temperature read back sane values.
   - [ ] Bus error counters stay at 0 during a clean scan.
4. Power the DXL bus back OFF before continuing.

## 4. I2C topology (mux + sensors)

1. With the I2C tree wired (mux 0x70, EEPROM 0x50, foot boards on channels 0–5):
2. Confirm the boot discovery scan results (via diagnostics / status):
   - [ ] TCA9548A detected at `0x70`.
   - [ ] 24LC32 EEPROM detected at `0x50`.
   - [ ] Each populated foot channel (0–5) reports **Present** (both VCNL4040
         `0x60` and LPS25HB `0x5C` ACK).
   - [ ] A channel with only one device present reports **Fault**, not Present.
   - [ ] Unpopulated channels report **Missing**, and reserved channels 6–7 are
         not scanned.
3. Degradation check:
   - [ ] Disconnect the mux → foot sensing reported unavailable, firmware still
         boots and the USB API still responds.

## 5. EEPROM config persistence

1. Commit a known config payload (Phase 2 `CFG_COMMIT` / bench harness).
2. Power-cycle the board.
3. Confirm:
   - [ ] Config reloads from EEPROM after the power cycle (sequence preserved).
   - [ ] Corrupting one slot (bench tool) → firmware falls back to the other
         valid slot.
   - [ ] With **no** valid slot, config is marked **volatile**, defaults load,
         and commits are rejected until EEPROM holds a valid slot.

## 6. CRSF / RC failsafe

1. Power the ExpressLRS receiver; bind the transmitter.
2. Confirm via `rc_input` telemetry / diagnostics:
   - [ ] Channels move and normalize to ~988–2012 µs.
   - [ ] Arm / kill / 3-position gait channels map to the expected logical state.
   - [ ] Powering the TX off (or unplugging the RX) raises the **failsafe** flag
         within ~250 ms and forces disarm + kill.
   - [ ] Restoring the link clears failsafe.

## 7. Fault observations

- [ ] An estop / unsafe condition transitions the safety state and is reported
      over the API.
- [ ] No task misses its watchdog deadline during the full run
      (`watchdog_missed` stays 0).
- [ ] Removing a previously-present I2C device at runtime degrades the dependent
      feature (does not freeze the control loop or the USB API).

## Recording results

Capture the automated script output and tick the manual boxes above for each
run. File any failure or surprising observation as a Beads issue
(`bd create ... -l area:firmware`) before proceeding to Phase 2.
