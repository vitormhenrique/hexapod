# Firmware Controller-to-Servo Motion Review

**Date:** 2026-07-22

**Scope:** Read-only review of `firmware/openrb150`. No firmware behavior,
servo configuration, calibration, or hardware state was changed as part of
this review.

**Review basis:** Current source, PlatformIO configuration, native Unity tests,
and existing firmware documentation. No live radio, DYNAMIXEL bus, or
mechanical measurements were available. A finding is *confirmed* only when the
stated behavior follows directly from source code. Whether a confirmed behavior
is visible as jitter on a particular robot still depends on the mechanism,
servo configuration, radio rate, and load.

## Executive Summary

The normal RC path is correctly high-level: gimbals do not directly command raw
servo positions. They produce a `ControllerCommand` containing body twist or
body pose, which then passes through safety arbitration, local gait generation,
body/leg IK, a servo map, and a DYNAMIXEL Sync Write. Packet arrival is
decoupled from output transmission: `rcTask` runs nominally at 100 Hz,
`controlTask` at 100 Hz, and `dxlTask` at 50 Hz.

The existing implementation already has several useful smoothing safeguards:

- A 5% rescaled gimbal deadband.
- A time-based 0.25 s filter for stride, height, step height, and speed.
- A time-based critically damped twist tracker rather than direct stick-to-foot
  assignment.
- A time-based 0.12 s body-pose filter.
- C1 foot trajectories at liftoff, touchdown, and phase wrap.
- IK reach-margin limiting, joint travel clamps, and a torque-enable ramp from
  measured servo positions.
- A single task and a mutex-protected frame for normal DYNAMIXEL goal writes.

The main remaining movement risks are transition and output-boundary issues:

1. Walking-to-stand, gait-family, and stop/restart transitions are not
   phase-safe. They can replace a moving leg target with the home target or a
   different gait's target in one control cycle.
2. Steady-state post-IK joints have no velocity or acceleration constraint;
   the only joint tick slew limiter operates while torque is being enabled.
3. A radio timeout deliberately enters E-stop and then disables torque/cuts
   DYNAMIXEL power without first completing swing legs or establishing a stable
   stance.
4. The default 57,600 baud DYNAMIXEL setting is marginal for 18 Protocol 1.0
   servos at 50 Hz plus position polling. It is feasible only if measured
   return-delay/turnaround/error behavior leaves enough margin; 100 Hz is not
   feasible at that baud.

## 1. Current Architecture

### Runtime tasks and ownership

The firmware starts FreeRTOS from [main.cpp](../firmware/openrb150/src/main.cpp#L28).
Periods and priorities are centralized in
[task_config.h](../firmware/openrb150/src/app/task_config.h#L42):

| Task | Nominal period | Nominal rate | Priority | Owner / purpose |
| --- | ---: | ---: | ---: | --- |
| `healthTask` | 500 ms | 2 Hz | 4 | Watchdog and health evaluation |
| `controlTask` | 10 ms | 100 Hz | 3 | Safety, arbitration, command filtering, gait, IK, goal generation |
| `dxlTask` | 20 ms | 50 Hz | 3 | Sole `Serial1` owner; torque, goal Sync Write, DXL reads |
| `rcTask` | 10 ms | 100 Hz | 3 | Sole `Serial3` owner; CRSF parsing and controller bridge |
| `apiTask` | 5 ms | 200 Hz | 2 | USB protocol and telemetry |
| `i2cTask` | 5-100 ms | 10-200 Hz | 1 | Sensors, mux, EEPROM, contact estimation |

`app::start()` creates these tasks in
[tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L2694). No application ISR,
timer callback, or serial callback directly computes a gait or writes a servo
goal; the serial and FreeRTOS implementation may use lower-level interrupts
outside this source tree.

### Complete RC-to-servo path

```text
ELRS/CRSF byte stream on Serial3 at 420000 baud
  -> rcTask (100 Hz, max 128 bytes per cycle)
  -> crsf::Parser::push() validates frame CRC and unpacks 16 x 11-bit channels
  -> controller::ControllerBridge::update()
  -> g_ctrlCmd / g_rcStatus latest snapshots
  -> controlTask (100 Hz)
  -> ControllerCore::step(): arbiter + StateMachine + RC/host selection
  -> GaitPipeline::setParams(), setGait(), setTwist(), setBodyPose()
  -> GaitEngine::update(dt): filtered command + six local foot trajectories
  -> BodyKinematics + LegIk: foot target to three joint angles per leg
  -> ServoMap::angleToTick(): sign, trim, rounding, configured travel clamp
  -> GoalFrame under g_goalMutex
  -> dxlTask (50 Hz)
  -> DxlBus::writeGoalPositions()
  -> one or more Protocol 1.0 Sync Write packets to Goal Position @ address 30
```

The normal path is visible end-to-end in
[rcTask](../firmware/openrb150/src/app/tasks.cpp#L2097),
[controlTask](../firmware/openrb150/src/app/tasks.cpp#L1286),
[ControllerCore::step](../firmware/openrb150/src/controller/controller_core.cpp#L100),
[GaitPipeline::update](../firmware/openrb150/src/gait/gait_pipeline.cpp#L73),
and [dxlTask](../firmware/openrb150/src/app/tasks.cpp#L1818).

### One concrete gimbal trace: left-Y forward walking

With the default binding map, `GimbalLY` is `walk_forward`
([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L108)).
For either custom ChannelPack or TX16S direct CRSF layouts, the bridge converts
the channel to a signed `-1000..1000` gimbal value, then maps it to `[-1, 1]`.
`readAxisBipolar()` applies the configured 0.05 deadband and rescales the
remaining range continuously ([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L365)).

In Walk mode `ControllerBridge::update()` sets `cmd_.twist_vx` from that axis
([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L539)).
`ControllerCore::step()` selects that high-level twist only while RC owns the
motion gate, converts command frame forward/left to the URDF body frame, then
calls `pipeline_.setTwist()` ([controller_core.cpp](../firmware/openrb150/src/controller/controller_core.cpp#L257)).
The gait engine filters and applies the resulting twist to foot strokes; it
does not interpret the stick as a joint tick or individual servo position.

The final per-joint tick is `2048 + trim + sign * round(angle_deg * ticks/deg)`,
clamped to each configured travel window in
[ServoMap::angleToTick](../firmware/openrb150/src/dxl/servo_map.cpp#L25).
The normal `dxlTask` then copies the latest complete frame and invokes exactly
one `DxlBus::writeGoalPositions()` call for that cycle
([tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L1971),
[dxl_bus.cpp](../firmware/openrb150/src/dxl/dxl_bus.cpp#L187)).

### What each controller input controls

| Input / source | Normalized output | Downstream effect |
| --- | --- | --- |
| Left Y, Walk | `twist_vx` | Forward/backward gait stroke |
| Right X, Walk | `twist_vy` | Lateral/strafe gait stroke |
| Left X, Walk | `twist_wz` | Tangential yaw component in every foot stroke |
| Gimbals, TranslateBody | body X/Y/Z pose | Body transform over nominal planted targets |
| Gimbals, RotateBody | roll/pitch/yaw pose | Body transform over nominal planted targets |
| Pot 1 | `speed` | Cadence, twist response, and torque-enable goal ramp rate |
| Pot 2 | `body_height` | Body-height target and neutral stance radius |
| Encoder / slider 1 | `stride` | Stride-length target |
| Encoder / slider 2 | `step_height` | Swing-lift target |
| Gait toggle | `gait_index` | Stand, Tripod, or Ripple for RC operation |
| Buttons/nav | `TrickId` / trim | High-level trick pose/twist/gait, never raw servo ticks |

The detailed default mapping is documented in
[controller_bridge.md](../firmware/openrb150/docs/controller_bridge.md#L20).
Raw or individual joint positions are available only through host maintenance
APIs, gated by `MacMaintenance`; they are not produced by gimbals
([controller_core.cpp](../firmware/openrb150/src/controller/controller_core.cpp#L184)).

### Radio parsing, profiles, and reconnect behavior

- `rcTask` uses `Serial3` at 420,000 baud and drains at most 128 bytes per 10
  ms cycle ([tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L2102)).
- `crsf::Parser` validates CRC-8 before accepting an RC channels frame
  ([crsf_parser.cpp](../firmware/openrb150/src/input/crsf_parser.cpp#L116)).
- The bridge recognizes custom packed ChannelPack and TX16S direct layouts.
  It requires three agreeing frames before choosing a profile, safely holding
  failsafe during detection ([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L250)).
- Once a profile is locked, link loss retains it; a reconnect does not
  redetect a different transmitter layout ([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L464)).
- CRSF RC channels have no sender sequence counter in this path. A duplicate
  valid frame repeats the same target and is harmless. A true out-of-order RC
  frame cannot occur on one UART stream; malformed/reassembled byte sequences
  are discarded by CRC or length checks.

## 2. Input Conditioning and Motion Smoothing

### Present conditioning

The implemented conceptual pipeline is:

```text
CRSF tick
  -> ChannelPack or TX16S layout decoding
  -> fixed-range normalization to [-1, 1] / [0, 1]
  -> per-binding deadband and rescale for bipolar axes
  -> high-level target command
  -> time-based gait parameter filter / pose filter / twist tracker
  -> foot trajectory and IK
```

Implemented safeguards:

- Fixed CRSF endpoint clamping for TX16S direct input is in
  [controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L39).
- The default gimbal deadband is 0.05 and is rescaled after the deadband;
  centered values do not create a reduced but nonzero walking command
  ([controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L108),
  [controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L18)).
- Gait shape parameters use a `dt`-based first-order filter with a 0.25 s time
  constant ([gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L200)).
- Twist uses a `dt`-based critically damped second-order tracker with
  speed-dependent natural frequency, not a fixed-coefficient input filter
  ([gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L222)).
- Body pose uses a `dt`-based first-order target filter with a 0.12 s time
  constant ([gait_pipeline.cpp](../firmware/openrb150/src/gait/gait_pipeline.cpp#L76)).

Not implemented in the firmware:

- Persisted per-axis raw minimum, center, and maximum calibration.
- Per-axis exponential response.
- A separate configurable input low-pass stage before motion-command shaping.
- Per-axis input range/outlier diagnostics beyond CRSF frame CRC and static
  endpoint clamping.

This is not necessarily wrong for a transmitter that has already calibrated
its gimbals, but the firmware cannot prove or record that calibration. The
existing deadband plus downstream command filters reduce center noise; they do
not correct a biased center or asymmetric endpoint.

### Existing command dynamics

`GaitEngine` turns the filtered speed setting into both gait frequency
(0.25--1.20 Hz) and twist response (natural frequency 3--10 rad/s)
([gait_engine.h](../firmware/openrb150/src/gait/gait_engine.h#L46)). This has
continuous command position and velocity for ordinary stick changes.

There is no raw controller-packet-to-servo packet coupling. `ControllerBridge`
only runs when a frame is parsed, but `ControllerCore::step()` and the servo
writer run periodic loops that consume the latest snapshot independently.

## 3. Gait, Foot Trajectory, and IK Review

### Gait representation and phase

This is an analytical foot-trajectory gait, not a fixed servo table or a
joint-angle keyframe gait. `GaitEngine` maintains one normalized phase,
per-gait leg offsets, a duty factor, stroke, and vertical swing lift
([gait_engine.h](../firmware/openrb150/src/gait/gait_engine.h#L11)).

Tripod offsets are `{0, 0.5, 0, 0.5, 0, 0.5}` for leg order rear-left,
rear-right, middle-right, front-right, front-left, middle-left. This forms the
intended groups:

```text
Group A: rear-left, middle-right, front-left
Group B: rear-right, front-right, middle-left
```

This is the requested physical grouping, expressed in the repository's leg
order ([gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L67)).

Normal twist and parameter changes do not reset phase. `ControllerCore` calls
`resetPhase()` only on a motion-gate rising edge
([controller_core.cpp](../firmware/openrb150/src/controller/controller_core.cpp#L284)).
Changing gait ID itself retains the scalar phase but immediately changes its
leg offsets and duty interpretation.

### Foot trajectory continuity

The stance trajectory is linear in stroke position for constant ground-speed.
The swing trajectory uses a cubic Hermite-like horizontal return whose endpoint
slopes match stance velocity, plus a `sin^2(pi*u)` vertical lift
([gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L292)).

Confirmed continuity properties:

- Position is continuous at liftoff, touchdown, and phase wrap.
- Horizontal foot velocity is intentionally matched across liftoff and
  touchdown; a native test exercises touchdown velocity continuity
  ([test_gait_engine.cpp](../firmware/openrb150/test/test_gait_engine/test_gait_engine.cpp#L174)).
- Vertical velocity is zero at swing endpoints.

The trajectory is C1, not C2. Stance acceleration is zero while the cubic
swing and `sin^2` lift have nonzero endpoint acceleration. This is a valid
low-cost trajectory, but fast/high-load robots can feel an acceleration step
as a torque or mechanical jerk. A quintic polynomial or cycloidal profile
would improve that later without requiring a heavier runtime architecture.

### IK and servo conversion

The leg solver uses `atan2f`, clamps the law-of-cosines term to `[-1, 1]`, uses
a fixed knee-up branch, and reports reachability
([leg_ik.cpp](../firmware/openrb150/src/gait/leg_ik.cpp#L24)). Before solving,
the body layer pulls targets into a 95% reach annulus where needed
([leg_ik.h](../firmware/openrb150/src/gait/leg_ik.h#L29),
[body_ik.cpp](../firmware/openrb150/src/gait/body_ik.cpp#L91)).

Joint direction inversion and trim happen once, in `ServoMap`; the current
main gait path does not apply another direction inversion or post-IK offset.
The conversion uses `lroundf`, so it avoids truncation bias but can still
alternate between adjacent integer ticks when an otherwise small physical
motion straddles a rounding boundary.

Invalid IK output is not sent unchecked: foot targets are reach-limited,
unreachable/reach-limited flags are carried in the pipeline output, and every
final tick is clamped to the configured servo window. The code currently
records aggregate flags but does not expose a per-leg singularity distance,
per-joint target delta, or a rejected-goal counter.

## 4. Confirmed Jitter and Abrupt-Motion Risks

The following are source-proven behaviors with a plausible direct route to
abrupt motion, instability, or timing-induced discontinuity. Severity rates
the potential physical consequence, not proof that every robot exhibits it.

| ID | Severity / category | Source location | Current behavior and trigger | Why it can create visible instability | Recommended correction |
| --- | --- | --- | --- | --- | --- |
| C1 | High - gait transition | [controller_core.cpp](../firmware/openrb150/src/controller/controller_core.cpp#L266), [gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L176) | A gait toggle, Walk-to-body-mode change, or a trick can call `setGait()` immediately. `setGait()` only replaces the gait enum; it does not remap phase or transition support legs. Selecting Stand produces all home targets on the next cycle. | The same phase has different duty/offset semantics in Tripod, Ripple, and Stand. A swing leg can become stance, or vice versa, with a foot-target jump. | Add a gait-transition state that preserves support legs, completes current swing legs, and changes offsets only at a compatible support boundary. Treat Walk/body/trick gait changes through the same transition manager. |
| C2 | High - stop/restart transition | [gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L239) | When the twist tracker falls below its park thresholds, the engine returns every foot to home immediately and stops phase. The stored phase is retained; the next nonzero command resumes from that old phase. | The final active walking target is generally not home, and the resumed phase can put one or more legs partway through swing. Both events can create Cartesian and joint-target steps. | Implement a stopping substate: reduce stride while finishing active swings, place feet, then enter a defined stance. Start from a defined phase/support condition and ramp stride/lift from zero. |
| C3 | High - joint command limiting | [gait_pipeline.cpp](../firmware/openrb150/src/gait/gait_pipeline.cpp#L125) | `last_tick_` and `goal_slew_ticks_per_s_` limit only a torque-enable ramp. Once a joint reaches the trajectory, `ramping_[slot]` turns off permanently. Steady-state IK targets are sent without per-joint velocity, acceleration, or delta bounds. | A phase/mode discontinuity, nonlinear IK region, or delayed cycle can become a large 50 Hz tick step. Servo-side moving-speed values are not written with goal positions. | Add per-joint velocity limits at minimum, with optional acceleration limits. Constrain the final command after IK and before `GoalFrame`; report target-vs-limited delta. |
| C4 | High - link-loss transition | [controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L598), [state_machine.cpp](../firmware/openrb150/src/safety/state_machine.cpp#L126), [tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L1910) | More than 250 ms without a valid RC frame enters bridge failsafe, then `Estop`. On the DXL task's next 20 ms cycle the falling motion gate disables torque and the state policy removes DXL power. | A short RF blackout during swing can immediately remove support rather than completing a step or holding a stable stance. This is intentional E-stop behavior, but it is not a controlled gait stop. | Keep a physical kill switch and critical-fault E-stop immediate. For stale command data, add a staged, state-aware link-loss policy: hold latest command briefly, ramp target velocity to zero, finish/land swing legs, then disarm if the link does not recover. |
| C5 | Medium - control overrun/filter discontinuity | [controller_time.cpp](../firmware/openrb150/src/controller/controller_time.cpp#L13), [gait_pipeline.cpp](../firmware/openrb150/src/gait/gait_pipeline.cpp#L76), [gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L200) | Real `dt` is propagated after ordinary loop overruns. The pose filter snaps to target when `dt >= 120 ms`; shape filters snap when `dt >= 250 ms`. | A blocked equal-priority task or long bus operation can turn a single late cycle into a body-pose or gait-parameter step, even though normal-rate behavior is filtered. | Use `alpha = 1 - exp(-dt/tau)` for all first-order filters, including late samples. Add an explicit overrun recovery policy and record missed-cycle count/dt. |
| C6 | Medium - trajectory acceleration | [gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L292) | Horizontal swing/stance velocity is continuous and vertical endpoint velocity is zero, but acceleration is discontinuous at liftoff/touchdown. | Torque demand can step at support transitions, especially with high step height, a stiff servo P gain, backlash, or a compliant frame. | Retain the current C1 path initially; offer a quintic smoothstep or cycloidal lift as a selectable low-cost trajectory after baseline HIL logs establish that it matters. |
| C7 | Medium - bus timing | [dxl_bus.h](../firmware/openrb150/src/dxl/dxl_bus.h#L50), [dxl_bus.cpp](../firmware/openrb150/src/dxl/dxl_bus.cpp#L187), [task_config.h](../firmware/openrb150/src/app/task_config.h#L48) | The configured default DXL baud is 57,600 and the goal path runs at 50 Hz. Eighteen targets are Sync Written in chunks if the installed library node cap is below 18; each cycle also performs a position read. | The nominal bus load is high enough that return delay, direction turnaround, a slower servo, error responses, or maintenance work can make output intervals irregular. | Measure bus timing first. Use 40 Hz as a conservative initial legacy-bus rate, or migrate all servos and host tooling to a verified higher baud before seeking 50--100 Hz. Do not raise control frequency just because it is CPU-feasible. |
| C8 | Medium - DXL fault timing | [dxl_bus.cpp](../firmware/openrb150/src/dxl/dxl_bus.cpp#L247), [tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L2009) | A powered but nonresponsive position read can busy-wait for up to 20 ms in the 50 Hz, priority-3 DXL task. It executes after goal output and before yielding. | An unresponsive bus can consume an entire DXL period and delay same-priority control/RC work, producing a larger next `dt` and therefore a larger unbounded joint target delta. | Keep reads bounded, but split/reduce diagnostic reads under motion, prioritize `controlTask` above DXL I/O, and add per-cycle bus-time/overrun telemetry. Fault after a bounded window without allowing a failed read to monopolize a full period. |
| C9 | Medium - coupled tuning | [gait_pipeline.cpp](../firmware/openrb150/src/gait/gait_pipeline.cpp#L31), [gait_engine.cpp](../firmware/openrb150/src/gait/gait_engine.cpp#L222) | Pot 1 simultaneously controls cadence, twist response, and the torque-enable tick ramp. | An operator asking for faster gait cadence also changes acceleration feel and recovery/arming behavior, making stable tuning difficult and potentially too abrupt at high speed settings. | Split cadence, command acceleration, and torque-enable recovery slew into separate configuration parameters. Keep an optional operator speed scale, but map it through calibrated independent envelopes. |
| C10 | Medium - input calibration gap | [controller_bridge.cpp](../firmware/openrb150/src/input/controller_bridge.cpp#L18), [controller_bridge.h](../firmware/openrb150/src/input/controller_bridge.h#L116) | Normalization assumes fixed `-1000..1000` or standard CRSF endpoints; only the 5% deadband is configurable per binding. No per-radio min/center/max or expo is persisted. | A biased or asymmetrical transmitter can enter motion outside the deadband, distort speed, or give unequal direction response. The downstream tracker masks some but not all of this. | Add persisted per-axis calibration and validation, then apply deadband/rescale, optional expo, and a time-aware input filter before the desired-motion state. |

### Important non-findings

- No gimbal path directly writes a raw servo position or individual joint goal.
- No normal path sends a servo packet from `rcTask` or a radio callback.
- No normal goal writer exists outside `dxlTask`; torque-enable hold writes and
  maintenance hold writes also remain in that same task.
- The normal gait implementation is not fixed servo tables or raw joint
  keyframes.
- Contact sensing currently does not modify foot timing or raw servo output, so
  it is not an active source of contact-feedback oscillation. Feature reporting
  intentionally marks terrain gait/leveling integration unavailable.

## 5. Suspected Risks Requiring Runtime Evidence

| Topic | What source proves | Missing evidence / measurement needed |
| --- | --- | --- |
| DXL return delay and status return level | Firmware can read/write logical parameters but does not establish or record their startup values. Normal goal Sync Writes are broadcast and do not request replies. | Read every servo's `ReturnDelayTime`, `StatusReturnLevel`, baud, moving speed, PID, torque limit, temperature, voltage, and error state on the bench. |
| Actual RF packet rate and receiver buffering | `rcTask` can consume at most 12,800 bytes/s and processes the newest complete frame(s) once per 10 ms task cycle. | Log inter-frame intervals and UART backlog at configured ELRS packet rate. A 26-byte RC frame at 500 Hz alone is about 13,000 bytes/s, above the drain budget before link-stat frames. |
| DXL write duration/node chunk limit | The source chunks at `DXL_MAX_NODE`, but the PlatformIO dependency range does not pin the installed macro/source in this repository. | Record exact Sync Write packet count, bytes, duration, UART errors, and servo response behavior from the built artifact. |
| Adjacent-tick chatter | `lroundf` can alternate adjacent ticks; repeated writes intentionally send unchanged goals to feed a possible bus watchdog. | Log final tick deltas per joint while stationary, while centered, and at slow walking. Correlate oscillations with present position/load and physical vibration. |
| Servo tuning and mechanics | The firmware does not configure legacy moving speed, PID/compliance, torque limit, or return delay on the motion path. | Capture actual EEPROM/RAM values plus load, voltage, temperature, backlash, horn tightness, frame flex, and supply sag under load. |
| Snapshot coherence | `g_goalFrame` is mutex-protected. Several other cross-task aggregate snapshots (`g_ctrlCmd`, RC status, contact and servo arrays) are plain latest-value copies without a sequence lock or critical copy. | Stress with high RC/API/telemetry load and instrument inconsistent snapshot reads. On the single-core scheduler these copies are brief, so this is a concurrency concern, not evidence of current jitter. |
| Controller type replacement | A locked bridge profile remains locked across link loss and reconnect. | Reconnect a different transmitter layout while safely torque-off and verify decoded fields. Decide whether profile must be explicitly reset or revalidated before rearming. |

## 6. Timing and DYNAMIXEL Bus Analysis

### Control timing

- **Radio receive:** asynchronous UART bytes; parser is serviced at 100 Hz.
  Actual ELRS packet rate is not configured in firmware.
- **Input decode:** whenever a complete frame is seen in `rcTask`; only the
  latest decoded snapshot is used by the next control iteration.
- **Gait, pose filter, IK, servo map:** 100 Hz `controlTask` with measured
  `dt`, normally 10 ms.
- **DYNAMIXEL goal output:** 50 Hz `dxlTask`, normally 20 ms.
- **Position/status read:** one present-position read per DXL cycle; each
  servo position is therefore refreshed around 2.5 Hz for 18 servos. One
  detailed status read is also round-robin and slower per servo.
- **I2C/contact:** nominally 50 Hz, but the gait pipeline currently does not
  consume contact state for terrain adaptation.

All periodic tasks use `vTaskDelayUntil()` and the motion/DXL loops rebase after
overruns ([tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L1362),
[tasks.cpp](../firmware/openrb150/src/app/tasks.cpp#L2081)). This prevents a
permanent busy loop, but it does not make an individual late control step
physically smooth.

### Protocol 1.0 byte budget at 57,600 baud

For a legacy MX-28 Goal Position Sync Write with two data bytes per servo:

```text
one Protocol 1.0 Sync Write = 8 + 3N bytes
N = 18 => 62 bytes if one packet can contain all servos
```

The driver explicitly chunks at its library node limit. If 18 legacy targets
need two packets, the total is:

```text
(8 + 3N1) + (8 + 3N2) = 70 bytes, where N1 + N2 = 18
```

At 57,600 baud with conventional 8N1 serial framing:

```text
goal write, two packets: 70 bytes * 10 bits * 50 Hz = 35,000 bit/s = 60.8%
goal write, one packet: 62 bytes * 10 bits * 50 Hz = 31,000 bit/s = 53.8%
one legacy position read: request 8 + response 8 bytes at 50 Hz = 8,000 bit/s = 13.9%
combined nominal traffic = approximately 67.7% to 74.7%
```

These figures exclude servo return delay, TTL direction turnaround, line idle,
library overhead, protocol errors, retries from lower layers, maintenance
traffic, and any V2 devices. The normal writer itself does not immediately
retry a failed Sync Write. Read timeouts are explicitly 20 ms; the Sync Write
call does not pass an explicit timeout in this source.

**Conclusion:** 50 Hz is possible on a clean, correctly tuned 57,600 baud
legacy bus but has little engineering margin. A 100 Hz 18-servo goal update
would require roughly 108--122% of baud capacity before reads and is not
realistic. Start at 40 Hz for the current baud unless HIL measurements prove
50 Hz has acceptable P99 write intervals and error margin. A verified
higher-baud migration is required before considering 50--100 Hz output.

## 7. Recommended Architecture

Keep the existing task ownership and `GoalFrame` boundary. Add explicit
command and transition state within `ControllerCore`/`GaitPipeline` rather
than moving servo logic into the radio task.

```text
rcTask / host API
  -> validated latest RawInputSnapshot + timestamp + sequence
  -> 100 Hz controlTask
     -> per-axis calibration, deadband, expo, time-aware input filter
     -> DesiredMotionState (vx, vy, yaw rate, body pose, gait request)
     -> independent velocity/acceleration limits
     -> gait transition + start/stop state machine
     -> fixed-phase local GaitEngine
     -> foot targets + IK + reach diagnostics
     -> per-joint velocity/acceleration limiter
     -> one coherent GoalFrame with timestamp/sequence
  -> 40--50 Hz dxlTask
     -> final safety validity/age check
     -> coordinated Sync Write
     -> rate-limited, noncritical status reads
  -> telemetry sampler / ring buffer
```

Design rules:

- Store the newest valid command and timestamp; do not let packet arrival
  invoke gait or DXL work.
- Give each transition a state, entry condition, timeout, and telemetry reason.
- Preserve gait phase for normal steering/magnitude changes, but do not assume
  that preserving a scalar phase makes a gait-family change continuous.
- Apply joint constraints after IK, not a blind low-pass filter on tick output.
  This gives bounded motion without silently obscuring Cartesian tracking
  error.
- Make `controlTask` the highest regular motion priority, leave DXL I/O below
  it, and bound all bus diagnostics so a failed read cannot delay the next
  control decision.
- Use fixed-size snapshots, sequence counters, and short critical copies for
  cross-task aggregate data. Do not allocate or print from real-time paths.

### Proposed staged radio-loss policy

Keep the current physical kill switch and critical hardware fault behavior as
immediate safety actions. For stale RC data, make timeouts derived from the
measured RC period rather than hard-coding a generic example:

```text
age <= T_hold:       retain last valid desired motion
T_hold < age <= T_brake:
                      target velocity/yaw ramps toward zero
T_brake < age <= T_land:
                      gait manager completes/lands swing legs and enters stance
age > T_land:         torque-off/disarm according to support and fault policy
```

For a nominal packet period `P`, choose `T_hold` above measured jitter, for
example from a multiple of `P` and observed P99 inter-arrival time. Then choose
braking and landing windows from measured gait phase duration and support
stability, not an arbitrary radio timeout.

## 8. Prioritized Implementation Plan

### Must fix first

| Task | Affected files | Approach and dependencies | Acceptance criteria / tests | Risk |
| --- | --- | --- | --- | --- |
| Add gait transition and stop/start manager | `controller_core.*`, `gait_pipeline.*`, `gait_engine.*`, tests | Introduce explicit `Walking`, `Stopping`, `Standing`, and `TransitioningGait` states. Preserve/land current swings before changing gait/mode or removing stroke. Depends on a way to expose swing phase per leg. | No foot-target or final-tick step above configured limit during gait toggle, walk/body mode switch, stop, or restart in torque-off replay. | High |
| Add post-IK joint velocity limits | `gait_pipeline.*`, config schema/API, telemetry/tests | Maintain limited ticks and apply role-specific `max_delta = v_max * dt`; optionally add acceleration state. Keep reach/clamp and Cartesian error diagnostics. | Every output delta is bounded under phase, mode, and overrun tests; configured maxima survive config validation; no out-of-range goal is emitted. | High |
| Separate RC stale command from immediate kill | bridge/core/state machine/tasks, protocol/docs/tests | Preserve immediate physical kill/E-stop. Add stale-command phase with a gait-aware brake/land policy before final torque-off. Requires a clear motion-state contract. | Inject late/lost packets in replay; robot reaches stable stance before normal link-loss torque-off; physical kill still cuts authority in one cycle. | High |
| Instrument timing and final goals | telemetry, HIL trace, companion decoder/tests | Add sampled control/dxl duration, `dt`, goal-frame age/sequence, packet interval/age, final per-joint delta, and bus error counters. Use fixed records/ring buffer. | Output-disabled HIL replay captures a complete input-to-goal trace without changing normal control timing materially. | Medium |

### Next improvements

| Task | Affected files | Approach and dependencies | Acceptance criteria / tests | Risk |
| --- | --- | --- | --- | --- |
| Make filters late-sample safe | `gait_engine.*`, `gait_pipeline.*`, tests | Replace snap-on-large-`dt` first-order updates with exponential alpha. Define an explicit overrun recovery behavior. | 10, 20, 75, 120, and 250 ms injected `dt` traces remain bounded and do not create a pose/shape step. | Medium |
| Decouple speed tuning | bridge bindings/config, core, pipeline, docs/tests | Separate operator speed scale from cadence, twist acceleration, and torque-enable slew. Preserve existing profile as compatibility mapping. | One setting can change cadence without changing acceleration or recovery slew; defaults reproduce current behavior where desired. | Medium |
| Persist input calibration and response | config schema/API, controller bridge, companion/CLI, tests | Store min/center/max and deadband/expo/filter parameters per analog axis. Validate calibration at commit. | Center-noise stays still after calibration; endpoints map symmetrically; malformed calibration fails validation. | Medium |
| Establish DXL bus configuration baseline | DXL job/telemetry/docs/HIL scripts | Read and log all legacy tuning/status parameters, packet timing, baud, and response errors. Do not write servo EEPROM during walking. | HIL report contains every servo's settings, packet durations, P99 intervals, and error rates. | Medium |
| Move noncritical DXL reads out of critical output budget | `tasks.cpp`, `dxl_bus.*`, tests | Schedule one bounded read only when bus budget permits, or lower DXL I/O priority beneath control. | Injected read timeouts do not prevent the next 100 Hz control update or violate DXL write interval target. | Medium |

### Optional advanced improvements

| Task | Affected files | Approach and dependencies | Acceptance criteria / tests | Risk |
| --- | --- | --- | --- | --- |
| C2 trajectories | `gait_engine.*`, tests | Offer quintic or cycloidal swing profiles with matching endpoint acceleration. | Calculated position, velocity, and acceleration are continuous at liftoff/touchdown; cycle CPU remains within budget. | Low |
| Jerk-limited Cartesian command profiles | core/pipeline, config/tests | Add S-curve velocity/acceleration/jerk states above the gait engine. | Step and reversal traces satisfy configured jerk bounds with acceptable latency. | Medium |
| Contact-aware gait adaptation | contact adapter, gait pipeline, safety/tests | Use existing contact estimates to adapt touchdown and support only after baseline gait is stable. | Sensor faults gracefully disable the feature; no direct raw-servo contact writes; replay proves bounded early/late touchdown behavior. | High |
| Higher DXL baud and synchronized status strategy | tooling/docs/DXL config/HIL | Perform maintenance-only, verified whole-bus baud migration; reevaluate 50--100 Hz control rate. | Packet budget, return settings, and all actuator acknowledgements are verified before normal walking. | High |

## 9. Proposed Configuration Parameters

These are starting ranges for bench tuning, not final robot constants. Record
the selected values with the servo map and firmware version.

| Parameter | Initial range / formula | Tuning guidance |
| --- | --- | --- |
| Per-axis center deadband | 0.03--0.06 normalized | Set above P99 centered-stick noise plus thermal drift, then rescale outside the deadband. Do not use a large output deadband to hide a bad input. |
| Exponential response | 0.20--0.45 blend | Use more expo for yaw and fine body pose only if it does not compromise emergency maneuver authority. |
| Input LPF cutoff | 8--15 Hz at 100 Hz control | Use `alpha = 1 - exp(-2*pi*f_c*dt)`. The existing twist tracker already adds smoothing, so start lightly or omit this stage after clean calibration. |
| Linear acceleration | 100--250 mm/s^2 initial bench envelope | Calibrate from actual stride/frequency to body speed. Use a separate deceleration value if braking needs to be stronger. |
| Linear deceleration | 150--350 mm/s^2 initial bench envelope | Must still allow swing completion; do not equate it to immediate zero-torque E-stop. |
| Yaw acceleration | 0.5--1.5 rad/s^2 initial bench envelope | Tune below the point where stance feet scrub, body oscillates, or current/load spikes. |
| Gait/IK update | 100 Hz initially | Existing CPU architecture supports this target. Confirm P99 task runtime before considering 200 Hz. |
| DXL goal update at 57,600 baud | 40 Hz initial, 50 Hz only after HIL proof | Keep average bus utilization and P99 packet interval within measured margin. 100 Hz is not viable for 18 legacy goals plus reads at this baud. |
| DXL goal update at verified higher baud | 50--100 Hz | Increase only after a maintenance-verified all-servo baud migration and bus-load test. |
| Joint velocity limits | Start role-specific, approximately 600--1,200 ticks/s for recovery/low-speed bench motion | Convert from measured safe joint angular speed; increase only after monitoring load, tracking error, and support stability. |
| Joint acceleration limits | Start approximately 3,000--8,000 ticks/s^2 | Use separate coxa/femur/tibia values if logs show different load margins. |
| Packet timeout thresholds | `T_hold > max(3P, P99 interval + margin)`; brake/land from measured gait timing | Preserve immediate physical kill separately. Do not copy the current 250 ms timeout blindly into the staged policy. |
| Diagnostic rate | 10--20 Hz continuous summaries; 100 Hz fixed-size capture on trigger | Never print per-cycle text logs. Stream binary/sample records or use an output-disabled HIL ring buffer. |

## 10. Instrumentation Recommendations

Existing useful streams are `ControllerState`, `RcDiagnostics`, `ServoGoals`,
`JointState`, `ServoStatus`, `Health`, and `ApiStats`
([telemetry.h](../firmware/openrb150/src/protocol/telemetry.h#L25)). They already
cover raw CRSF ticks, decoded controller command, packet age, final goal angle,
present joint state, and many servo health fields.

Add a sampled or trigger-captured diagnostic record with:

```text
control_timestamp, control_dt, control_runtime, control_overrun_count
radio_frame_timestamp, radio_interval, packet_age, parser_crc_errors
raw_axis, calibrated_axis, deadband_axis, filtered_axis
desired_vx/vy/wz, limited_vx/vy/wz, desired_body_pose, limited_body_pose
gait_id, gait_phase, gait_transition_state, stride, frequency, step_height
per-leg foot target, swing/stance, reach margin, IK result
per-joint raw IK tick, limited tick, final transmitted tick, tick delta
GoalFrame sequence, goal age, DXL write duration, packet count, bytes
DXL reads/writes/errors, bus timeout count, voltage, temperature, load
```

Use a fixed-size ring buffer owned by `controlTask`/`dxlTask`, export through
the existing HIL trace or rate-limited binary telemetry, and trigger high-rate
capture on a packet gap, goal delta threshold, clamp, task overrun, or DXL
error. Do not use `Serial.print` inside the control, RC, or DXL task.

## 11. Safe Test Plan

Run the first seven tests with output-disabled HIL firmware or with DYNAMIXEL
torque disabled. Log generated foot targets, joint targets, and final ticks;
do not use ground walking until output traces and suspended-leg checks pass.

| Test | Setup | Procedure | Acceptance criteria |
| --- | --- | --- | --- |
| Centered-noise | Output disabled, recorded real transmitter samples | Feed 5--10 minutes of centered raw channels. | Desired/limited twist remains zero; no gait phase advance; no repeating nonzero tick changes. |
| Step input | Output disabled | Apply neutral-to-full forward/strafe/yaw records. | Bounded command acceleration, foot velocity, joint delta, and no clamp/unreachable event. |
| Direction reversal | Output disabled, then suspended robot | Full forward to full reverse and yaw reversals at several speed settings. | Command changes follow configured bounds; no foot/joint target discontinuity beyond limits. |
| Packet jitter | Replay with nominal interval plus measured P95/P99 gaps and duplicates | Keep command constant and vary input arrival timing. | Fixed-rate output remains smooth; packet count does not alter phase rate; no false failsafe within configured hold window. |
| Packet loss | Replay one, several, and sustained gaps | Exercise staged hold/brake/land policy and physical kill separately. | Link loss lands in a support-safe stance before ordinary torque-off; kill remains immediate. |
| Gait-phase continuity | Output disabled | Toggle Tripod/Ripple/Stand, Walk/body modes, and tricks at every phase bucket. | Transition manager prevents leg target steps and does not strand swing feet. |
| Phase wrap | Output disabled | Sweep all gait types and high stride/yaw over many wraps. | Position/velocity continuity at wrap; no NaN, reach failure, or unexpected tick delta. |
| IK boundary | Native plus output-disabled HIL | Sweep body pose, stride, height, and yaw to reach margin. | No trig NaN; targets clamp/report before singularity; final ticks stay in travel. |
| Tick quantization | Torque disabled, high-rate trace | Hold low-amplitude commands around half-tick boundaries. | Quantify alternation; only add a quantization policy if it reduces vibration without unacceptable Cartesian stair steps. |
| DXL bus load | Servos torque off or suspended | Measure packet bytes/duration/errors at 40/50 Hz and configured return values. | P99 write interval meets target, no CRC/timeouts, and documented utilization retains margin. |
| Standing-on-support | Suspended first, then supervised ground stance | Arm/stand, start/stop, and mode/gait transitions at low speed. | No snap at torque enable, no collapse on controlled stop, stable support polygon. |
| Servos-disabled telemetry | Passive pose/output-disabled build | Move joints manually and replay controller inputs. | Telemetry remains responsive, calculated goals are observable, and no DXL write/torque enable occurs. |

Existing local quality gates are documented in [testing.md](testing.md#L7).
The relevant baseline native suites are `test_controller_bridge`,
`test_controller_core`, `test_controller_time`, `test_gait_engine`,
`test_gait_pipeline`, `test_leg_ik`, `test_servo_map`, `test_dxl_sync`, and
`test_state_machine`. They are valuable component coverage but do not yet
exercise the transition, packet-jitter, or bus-timing cases above.

## 12. Mechanical and Electrical Separation Checklist

Use the following classifications when interpreting a symptom:

| Observable symptom | First software evidence to inspect | Non-software evidence to collect |
| --- | --- | --- |
| Motion while sticks are centered | Raw/decoded channels, deadband result, filtered twist, phase | Gimbal center drift, transmitter calibration |
| Sharp change at gait/mode/stop | Transition state, foot target delta, final joint delta | Servo load/current, foot contact, frame flex |
| Oscillation near a pose | IK reach margin, tick alternation, servo-goal vs present error | Servo P/I/D/compliance, horn backlash, linkage play |
| Random freezes or jumps under load | DXL write/read intervals, errors, task overrun, packet age | Supply voltage at servo bus, connector resistance, brownout/reset cause |
| One leg behaves differently | Servo sign/trim/limits, IK angle, final tick, present state | Incorrect horn indexing, wrong servo ID/config, binding/link geometry |
| Heat, overload, or loss of holding force | Temperature/voltage/load/error telemetry, torque cycles | Mechanical binding, insufficient 12 V current capacity, poor cooling |

The firmware already reports much of the basic servo health state. It cannot,
from source alone, distinguish firmware-command jitter from servo PID tuning,
mechanical backlash, calibration error, or power-integrity faults. Those need
the HIL captures above.

## Review Conclusion

The architecture is already close to the intended controller-to-motion model:
controller inputs become robot-level command targets, and a local, fixed-rate
gait/IK pipeline owns servo goals. The immediate value is not a wholesale
rewrite. It is to complete the missing transition and post-IK limiting layers,
measure the legacy DXL bus margin, and make the timing/goal evidence visible.
Those changes address the most credible sources of abrupt movement without
weakening the existing single-owner bus and safety boundaries.
