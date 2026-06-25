# Phase 2 Hardware-in-the-Loop (HIL) bench & ground test checklist

Status: **procedure documented; physical execution pending** (bd `hexapod_src-22l.11`).

This checklist validates Phase 2 motion **gradually**, from a single leg through a
suspended full robot to low-speed ground gait, then exercises fault injection and a
telemetry log review. It is the Phase 2 analogue of `tools/hil_smoke.py` /
`docs/hil_smoke_phase1.md`. Every motion step is a **manual, operator-supervised**
procedure: the firmware is the safety controller, but a human must be ready on the
RC kill switch and the DYNAMIXEL power cutoff at all times.

> SAFETY FIRST. Never run a motion step without: (1) the RC transmitter on and the
> kill switch reachable, (2) the 12 V DYNAMIXEL power on a fused, switched feed you
> can cut instantly, (3) clear space around the robot, and (4) eye protection for
> the first powered tests. Stop on any unexpected motion, buzzing, heat, or smell.

## 0. Preconditions (run once per session)

- [ ] Firmware built and flashed: `cd firmware/openrb150 && ~/.platformio/penv/bin/pio run -e openrb150 -t upload`.
- [ ] Native test suite green: `~/.platformio/penv/bin/pio test -e native` (expect all cases passing).
- [ ] USB CDC enumerates; record the port (e.g. `/dev/tty.usbmodemXXXX`).
- [ ] Phase 1 boot-safety smoke passes: `uv run python tools/hil_smoke.py --port <PORT>`
      (HELLO ok, DXL power OFF, no watchdog misses, uptime advancing).
- [ ] 12 V DXL bus on its own fused, switched, high-current feed with a common ground
      to the OpenRB-150 (AGENTS.md 1.1 — never power 18 servos through the controller).
- [ ] Battery charged; note resting pack voltage. Confirm the safety state machine's
      low-battery cutoff (`StateParams.battery_min_mv`, default 10000 mV) is below the
      pack's loaded voltage but above its safe floor.

For each step below, record: date/time, firmware git hash, pack voltage, result
(PASS/FAIL), and a saved telemetry capture filename.

## 1. Single-leg test (one leg, robot clamped, feet clear of ground)

Goal: prove servo mapping, IK direction, joint limits, and the maintenance command
path on the least-dangerous configuration.

- [ ] Mechanically secure the body; ensure the **one** leg under test can move freely
      and the other five are unpowered or blocked.
- [ ] DXL power ON. Scan the bus (`DXL_SCAN`): confirm the expected servo IDs and that
      each `ServoProfile` reports the correct table kind (legacy MX-28 vs MX 2.0).
- [ ] Verify all servos report **torque OFF** and the safety state is `DISARMED`.
- [ ] Acquire the maintenance lock; confirm state goes to `MAC_MAINTENANCE` only with
      the lock held (and back to `DISARMED` on release).
- [ ] Command a small single-joint move on the test leg via `SET_JOINT_TARGET`
      (coxa, then femur, then tibia). Confirm:
  - [ ] each joint moves in the **expected direction** (fix sign in the servo map if not),
  - [ ] motion stops at the configured min/max ticks (no mechanical hard-stop slamming),
  - [ ] `servo_status` telemetry shows present position tracking the goal,
  - [ ] clamp/reachability flags appear when a target is out of range.
- [ ] Command a small Cartesian foot move via `SET_LEG_TARGET`; confirm IK output is
      sane and reachability flags behave at the workspace edges.
- [ ] Release the maintenance lock; confirm torque drops OFF and state returns to `DISARMED`.

## 2. Suspended full-body test (all six legs, robot hanging, feet off the ground)

Goal: prove gait timing, all-leg sync write, and arming/stand without ground reaction.

- [ ] Suspend the robot so every foot is clear of all surfaces and tethers cannot snag.
- [ ] DXL power ON; `DXL_SCAN` shows all 18 servos with correct profiles.
- [ ] Arm via the RC arm switch; confirm `DISARMED → ARMING_CHECKS → STAND_READY`
      (arming only proceeds when config/battery/scan checks pass).
- [ ] Command **stand**; confirm a smooth, symmetric stance pose and that
      `g_motionGate` is true only in a motion state with authority.
- [ ] Command **sit**; confirm controlled descent.
- [ ] Cycle each gait at the **lowest** speed: tripod, ripple, wave/crawl. For each,
      confirm: correct leg phasing, no servo over-temperature, goal-vs-present tracking
      within tolerance, and no clamp storms.
- [ ] Disarm; confirm torque OFF and `STAND_READY/RcManual → DISARMED`.

## 3. Low-speed ground gait test (on the ground, minimal speed)

Goal: first real ground walking with full failsafe coverage.

- [ ] Place the robot on a flat, non-slip surface with clear space; operator on the kill switch.
- [ ] Arm and **stand**; confirm stable static stance under its own weight.
- [ ] Walk forward at minimum speed/stride/step-height for a short distance; confirm:
  - [ ] stable gait, no foot dragging or tipping,
  - [ ] body stays roughly level,
  - [ ] servo temperatures and loads stay in range (watch `servo_status`),
  - [ ] RC body-twist (forward/lateral/yaw) responds correctly.
- [ ] (If foot contact sensors are populated and calibrated) enable contact detection
      and confirm `CONTACT_TERRAIN` is entered only with valid sensor confidence, that
      `contact_state` transitions look right per foot, and that disabling it falls back
      to nominal gait. Skip if sensors are absent — feature must auto-disable with a reason.
- [ ] Disarm and confirm a safe stop.

## 4. Fault injection (verify every safe-stop path)

Run each with the robot suspended first, then (optionally) on the ground. Confirm the
robot **stops safely**, the safety state and `FaultReason` are reported in telemetry,
and recovery requires the documented action.

- [ ] **RC kill switch** during walk → `ESTOP`, reason `RcKill`, torque OFF. Releasing
      kill returns to `DISARMED` (must re-arm).
- [ ] **RC link loss** (power off the transmitter) while operational → failsafe →
      `ESTOP`, reason `RcLinkLost`.
- [ ] **Host estop** over USB → `ESTOP`, reason `HostEstop`.
- [ ] **Low battery** (or simulate by raising `battery_min_mv`) → `ESTOP`, reason `BatteryLow`.
- [ ] **Watchdog** (a task missing its deadline, e.g. induced stall) → `ESTOP`, reason `Watchdog`.
- [ ] **DXL hard fault** (unplug a servo / induce repeated bus errors) → `FAULT_HARD`,
      reason `DxlHardware`; confirm it **latches** and only clears on `CLEAR_FAULT`
      after the condition is gone.
- [ ] **I2C sensor loss** (disconnect the mux or a foot sensor) → dependent contact
      features auto-disable with a published reason; default gait remains usable.
- [ ] **Maintenance lock expiry** (stop heartbeating) → lock revoked, returns to `DISARMED`.

## 5. Telemetry log review

Goal: confirm the Phase 2 telemetry subscriptions capture enough to debug a run.

- [ ] Subscribe to `health`, `control_state`, `servo_status`, `servo_goals`,
      `contact_state`, `i2c_sensors_raw`, `rc_input`, and `api_stats` (`SUBSCRIBE` /
      `SET_STREAM_RATE`); confirm each arrives at the requested rate, clamped to its max.
- [ ] Record a full session (one-leg, suspended, and ground runs) to a log file
      (raw frames + decoded). A decoded CLI/companion client lands in Phase 3; until
      then capture raw delimited frames and decode offline.
- [ ] Review the log for: goal-vs-present tracking, clamp/reachability events, contact
      transitions, RC input vs commanded motion, safety-state/FaultReason transitions
      around each injected fault, and `api_stats` dropped-frame / TX-backlog counters.
- [ ] Confirm `GET_STREAM_STATS` shows non-zero `emitted` for subscribed streams and
      that `dropped`/`tx_backlog` stay low at the chosen rates.

## Sign-off

Record per test (one-leg / suspended / ground / fault-injection / telemetry-review):
PASS/FAIL, operator, date, firmware hash, and log filename. File any defect, surprise,
or hardware observation as a `bd` issue. Close `hexapod_src-22l.11` only after all five
sections have a recorded PASS.
