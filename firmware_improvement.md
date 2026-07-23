You are planning work on an existing RC-controlled hexapod robot codebase.

Primary goal:
Remove jerky motion while preserving responsive RC control. Do NOT implement everything in one shot. Instead, create a new Beads epic (`bd create "Remove jerky motion from RC control pipeline" -t epic -p 1 -l "area:firmware,priority:safety"`) and break ALL of the work described in this document into individual child issues under that epic, with dependencies, labels, priorities, and per-issue acceptance criteria. The technical sections below are the specification each issue must reference.

## Unified Execution Plan (2026-07-22)

This document remains the detailed behavioral specification. The source-backed
baseline and confirmed risks are in
[docs/firmware_controller_to_servo_review_2026-07-22.md](docs/firmware_controller_to_servo_review_2026-07-22.md).
Beads epic `hexapod_src-22l.15` is the implementation source of truth; its
dependency graph, not this Markdown file, determines what is actionable.

The current firmware already has a fixed-rate task split, high-level
gimbal-to-body commands, local Cartesian gait/IK, configured joint travel
clamps, and one DYNAMIXEL Sync Write owner. The plan extends those seams rather
than replacing them.

### Reconciled Decisions

- Preserve the current ChannelPack and TX16S bindings until the configurable
  mapping path has compatibility coverage. Gimbals continue to command body
  motion, never normal raw servo targets.
- Keep physical kill, host E-stop, unsafe voltage, watchdog, and hard DXL
  faults immediate. A stale valid RC frame becomes a controlled
  `LINK_LOSS_STOP` only after gait landing support exists.
- Use stop-settle-switch-resume for the first gait-transition implementation.
  Master phase remains continuous while walking; a canonical phase may be set
  only after every foot reaches a defined stable stance.
- Add one explicit RC conditioning and body-command-shaping contract. Do not
  stack undocumented EMA, tracker, gait, and servo-internal profiles.
- Retain compute-all-then-Sync-Write. Do not change 57,600-baud rate, return
  delay, internal DYNAMIXEL tuning, or EEPROM settings until output-disabled
  HIL and suspended-bench timing evidence supports a specific change.
- Defer terrain adaptation, force control, nonlinear CPGs, whole-body
  optimization, and automatic tuning. They are not prerequisites for a
  predictable RC motion baseline.

### Beads Delivery Order

1. `hexapod_src-22l.15.1`: timing observability, whole-frame RC snapshots,
   and output-disabled traces.
2. `hexapod_src-22l.15.2` and `.3`: calibrated normalization, then bumpless
   analog filtering/expo and switch conditioning.
3. `hexapod_src-22l.15.4`: centralized limits and the production
   acceleration-limited body command shaper.
4. `hexapod_src-2qd` and `hexapod_src-22l.15.5`: phase-safe gait state,
   stopping, transitions, and latched swing trajectories.
5. `hexapod_src-22l.15.6`, `hexapod_src-e93`, and `hexapod_src-3cm`: IK jump
   diagnostics, final joint-rate guard, and staged RC link-loss landing.
6. `hexapod_src-22l.15.7`: measured DXL command-epoch and internal-profiling
   policy; this waits for the existing legacy polling benchmark.
7. `hexapod_src-22l.15.9` and `.8`: optional median/jerk strategies, then
   safe runtime mappings, presets, and tuning gates.
8. `hexapod_src-22l.15.10`: complete replay/HIL evidence, tuning guide, and
   physical validation. It is intentionally blocked by the preceding work and
   existing hardware bench validation.

No implementation issue should be claimed outside this order unless its Beads
dependencies have been deliberately revised after new source or hardware
evidence.

Each implementation session then works one issue at a time from `bd ready`, claims it with `bd update <id> --claim`, and closes it only when its acceptance criteria pass.

Use the repository's existing language, build system, coding style, servo driver, inverse kinematics, and gait implementation. Do not rewrite working subsystems unnecessarily.

Before creating issues:
1. Inspect the repository and identify:
   - RC receiver type and protocol: PWM, PPM, SBUS, CRSF, serial, etc.
   - RC update rate and how packet loss/failsafe is reported.
   - Main control-loop rate and timing source.
   - Existing gait state machine and phase representation.
   - Existing foot trajectory generation.
   - Existing inverse kinematics.
   - Servo type and output protocol.
   - Whether the servo API supports a synchronized or batch write.
   - Whether servos already perform internal speed/acceleration profiling.
2. Preserve existing channel assignments where practical.
3. Record these findings in the epic description (and `bd remember` durable facts), then file child issues that implement the following architecture with minimal invasive changes. If inspection reveals additional sources of jerk not listed here, file an issue for each one.

==================================================
REQUIRED CONTROL PIPELINE
==================================================

Implement this pipeline:

RC frame
    -> frame validation and timeout handling
    -> per-channel calibration and normalization
    -> analog-channel glitch filtering
    -> low-pass filtering
    -> deadband and expo
    -> mapping to desired body velocity and body pose
    -> acceleration- or jerk-limited command shaping
    -> gait state machine with continuous phase
    -> smooth Cartesian foot trajectories
    -> continuous inverse kinematics
    -> final joint velocity safety guard
    -> one synchronized servo command epoch

Never map raw RC values directly to servo positions.

Sample the entire RC frame atomically once per control tick. Do not read individual channels at different times.

==================================================
IMPLEMENTATION ORDER AND DEPENDENCIES
==================================================

Structure the epic's issue dependencies so these items land first, in this order:

1. Fixed-rate control loop with measured dt.
2. RC validation, calibration, timeout, deadband, expo, and EMA.
3. Acceleration-limited body velocity commands.
4. Continuous gait phase and safe gait start/stop.
5. Smooth swing-foot trajectory with latched endpoints.
6. Synchronized output of every servo target.
7. Joint rate guard as a final safety layer.
8. Optional median filtering and optional jerk-limited command shaping.
9. Runtime mode selection through configurable RC channels.

Do not file implementation issues yet for:
- Kalman filtering.
- Full nonlinear CPGs.
- Terrain adaptation.
- Force control.
- Whole-body optimization.
- Automatic parameter tuning.

Leave clean extension interfaces for these features (an issue may cover defining those interfaces).

==================================================
REQUIRED BEADS EPIC BREAKDOWN
==================================================

Create the epic, then create at minimum the following child issues. Every issue must: reference the relevant specification section of this document, carry acceptance criteria, carry labels (`area:firmware`, plus `priority:safety` where applicable), and declare dependencies matching the implementation order above. Split any issue that grows too large.

Audit and discovery:
- Audit codebase and document every current source of jerky motion (RC path, gait, IK, servo output, timing). Findings go into the epic description and follow-up issues.

Control loop and timing:
- Fixed-rate control loop with monotonic clock, measured dt, dt clamping, overrun logging.
- Atomic once-per-tick RC frame sampling.
- Remove heap allocation, blocking I/O, and slow formatting from the real-time loop.

RC input conditioning:
- Per-channel calibration config struct and asymmetric normalization with clamping and invalid-value rejection.
- Analog glitch filtering: input-filter strategy interface (NONE_DIAGNOSTIC, EMA, MEDIAN3_EMA) with time-based EMA and median-of-three.
- Deadband with output rescaling and expo shaping.
- Switch debouncing with stable-position requirement.
- Bumpless input-filter mode switching (initialize new filter from current output).

Command shaping:
- Central configuration object for all body-command limits.
- Body-level command mapping (BodyTwist, BodyPose) — never raw RC to servo positions.
- Command-shaper strategy interface (DIRECT_DIAGNOSTIC, ACCEL_LIMITED, JERK_LIMITED).
- ACCEL_LIMITED shaper with direction-aware accel/decel limits per axis.
- JERK_LIMITED critically damped bounded-jerk tracker with target-crossing protection.
- Bumpless shaper mode switching preserving value and rate state.

Failsafe:
- RC timeout tracking, LINK_LOSS_STOP behavior, disarm-after-1s policy, configurable timeouts, rearm requirement.
- Configurable emergency-stop policy (HOLD_POSITION, CONTROLLED_STOP, TORQUE_OFF).

Gait continuity:
- Gait state machine (DISARMED, STANDING, STARTING, WALKING, STOPPING, GAIT_TRANSITION, LINK_LOSS_STOP, FAULT) with continuous phase — no phase resets on stick-center, speed change, gait change, or late packets.
- STARTING/STOPPING ramps: amplitude ramp-in, finish active swings, settle to stable stance.
- Stop-settle-switch-resume runtime gait transitions, with extension interface for future blended transitions.

Foot trajectories:
- Latched swing endpoints (lift-off/touchdown position, duration, step height, boundary velocities) — no endpoint teleporting on new RC frames.
- Cubic Hermite horizontal swing path with matched boundary velocities.
- Minimum-jerk vertical swing profile with zero velocity at lift-off, apex, touchdown.
- Stance-path foot velocity from body twist (preserve existing world-frame pinning if present).

IK and joint safety:
- IK branch continuity: closest-valid-solution selection, wrapped angular distance, angle unwrapping, trig input clamping, unreachable-target detection/reporting, jump detection if single-branch.
- Final per-joint velocity guard with configurable limits and activation diagnostics counter.

Servo output:
- Compute-all-then-transmit with synchronized/batch write; document inter-servo skew if batch is unavailable.
- Deliberate policy decision on internal servo profiling vs external trajectory generation, documented.

RC mapping and tuning:
- Configurable 16-channel mapping with debounced mode switches and safety gating of diagnostic modes.
- Tuning layer (Channel 14) with safe-state gating, filtering, and conservative clamping.
- SMOOTH/BALANCED/RESPONSIVE presets with smooth interpolated preset transitions.
- Compile-time options and production defaults.
- Optional protected persistent tuning storage.

Diagnostics and testing:
- Non-blocking telemetry/diagnostics for all listed signals plus live diagnostic view.
- Unit tests per the UNIT AND SIMULATION TESTS section (may be split per subsystem, ideally paired with or blocking each implementation issue).
- Tuning document covering every parameter, unit, default, safe range, compile-time option, and RC assignment.
- Safe physical test procedure execution checklist as the final issue, blocked by all implementation issues.

==================================================
CONTROL LOOP
==================================================

Use a monotonic clock.

Default control-loop target:
- 100 Hz unless the existing hardware clearly supports and benefits from another rate.
- Make this configurable.

Use actual measured dt for all filters, gait updates, and trajectory integration.

Clamp unreasonable dt values:
- Small jitter should use the measured dt.
- Large overruns should be logged.
- Do not integrate one extremely large step after a pause.
- Define configurable minimum and maximum dt values.

No heap allocation, blocking telemetry, file writes, or slow formatting in the real-time control loop.

Generate every leg and joint target first. Only then transmit the servo command.

==================================================
RC CHANNEL CALIBRATION
==================================================

Create a configurable description for every RC channel:

struct RcChannelConfig:
- channel index
- minimum raw value
- center raw value
- maximum raw value
- reversed flag
- channel type:
  - analog centered
  - analog unipolar
  - two-position switch
  - three-position switch
  - momentary switch
- deadband
- expo
- low-pass time constant
- switch debounce time

Normalize centered analog channels to [-1, 1].

Use asymmetric calibration:

if raw >= center:
    normalized = (raw - center) / (maximum - center)
else:
    normalized = (raw - center) / (center - minimum)

Clamp the result to [-1, 1].

Normalize knobs and sliders to [0, 1].

Reject clearly invalid values rather than passing them into the controller.

Do not low-pass discrete switches. Debounce them instead.

==================================================
ANALOG RC FILTERING
==================================================

Implement a strategy interface with these modes:

enum InputFilterMode:
- NONE_DIAGNOSTIC
- EMA
- MEDIAN3_EMA

Default production mode:
- EMA

NONE_DIAGNOSTIC must be disabled in production builds or selectable only while the robot is disarmed.

Processing order for centered analog controls:

1. Normalize.
2. Optional median-of-three glitch rejection.
3. Time-based EMA.
4. Deadband with output rescaling.
5. Expo.
6. Clamp.

Median-of-three:
- Store the latest three normalized samples.
- Return the median.
- Use this only for analog controls.
- Do not add a long moving-average window.

EMA implementation:

alpha = 1 - exp(-dt / tau)
filtered += alpha * (input - filtered)

Handle tau <= 0 as bypass mode.

Suggested initial values:
- Translation and yaw stick tau: 0.05 to 0.08 seconds.
- Body height and body pose tau: 0.08 to 0.15 seconds.
- Default translation/yaw tau: 0.06 seconds.

Deadband implementation:

if abs(x) <= deadband:
    y = 0
else:
    y = sign(x) * (abs(x) - deadband) / (1 - deadband)

Suggested default deadband:
- 0.04

Expo implementation:

y = (1 - expo) * x + expo * x^3

Suggested default expo:
- 0.35

Apply expo after the deadband.

When a filter mode changes at runtime:
- Initialize the new filter using the current filtered output.
- The first output of the new mode must not jump to the raw input.
- Mode switching must be continuous.

==================================================
BODY-LEVEL COMMANDS
==================================================

Map the conditioned controls to body-level commands, not joint commands:

BodyTwist:
- forward velocity vx
- sideways velocity vy
- yaw rate wz

BodyPose:
- body height
- optional roll
- optional pitch
- optional body yaw offset

Use existing safe maximum speeds from the project where available.

Put all maximum values in one central configuration object:
- max forward speed
- max reverse speed
- max sideways speed
- max yaw rate
- body height minimum and maximum
- step height minimum and maximum
- gait frequency minimum and maximum
- stance width minimum and maximum

Do not scatter tuning constants throughout the code.

==================================================
COMMAND SHAPERS
==================================================

Create a command-shaper strategy interface.

Implement these modes:

enum CommandShaperMode:
- DIRECT_DIAGNOSTIC
- ACCEL_LIMITED
- JERK_LIMITED

Default production mode:
- ACCEL_LIMITED

DIRECT_DIAGNOSTIC:
- Compile only in development builds, or permit selection only while disarmed.
- Never make it the production default.

--------------------------------
ACCEL_LIMITED MODE
--------------------------------

Treat the body command as velocity. Limit the rate of change of that velocity.

For each axis:

delta = target - current
delta = clamp(delta, -decelerationLimit * dt,
                     accelerationLimit * dt)
current += delta

Implement direction-aware limits:
- When increasing speed in the same direction, use accelerationLimit.
- When reducing speed or reversing direction, use decelerationLimit.
- A reversal should decelerate toward zero before accelerating strongly in the opposite direction.

Use separate limits for:
- forward/reverse translation
- sideways translation
- yaw
- body height

Keep acceleration and deceleration separately configurable.

Suggested initial behavior:
- Deceleration can be approximately 1.25 to 1.75 times acceleration.
- Use conservative values derived from the robot's current maximum speed.
- Do not hard-code aggressive values without examining the existing robot limits.

--------------------------------
JERK_LIMITED MODE
--------------------------------

Implement a bounded-acceleration, bounded-jerk tracking filter.

For each shaped value maintain:
- current value
- current rate

For a body velocity command:
- current value is velocity.
- current rate is acceleration.

A suitable implementation is a critically damped second-order tracker with bounded jerk and bounded acceleration:

error = target - value
jerkCommand =
    omega * omega * error
    - 2 * dampingRatio * omega * acceleration

jerkCommand = clamp(jerkCommand, -maxJerk, maxJerk)

acceleration += jerkCommand * dt
acceleration = clamp(acceleration, -maxAcceleration, maxAcceleration)

nextValue = value + acceleration * dt

Use:
- dampingRatio = 1.0 by default.
- configurable response time or omega.
- target-crossing protection to prevent small overshoot around a stationary target.
- numerical guards for abnormal dt.

Unit-test step inputs, ramps, sign reversals, and returning to zero.

When changing command-shaper mode:
- Preserve the current output.
- Preserve the current acceleration where possible.
- Do not initialize the new mode from zero.
- Do not permit a direct diagnostic mode change while walking.

==================================================
RC TIMEOUT AND FAILSAFE
==================================================

Track:
- timestamp of latest valid RC frame
- receiver frame-loss flag
- receiver failsafe flag
- packet age

Suggested initial policy:

At approximately 150 ms without a valid frame:
- Enter LINK_LOSS_STOP.
- Ignore new movement intent.
- Set desired body velocity to zero.
- Use the normal acceleration/deceleration shaper.
- Continue active swing trajectories safely.
- Do not abruptly reset gait phase.

At approximately 1 second without a valid frame:
- Enter a stable disarmed or hold state.
- Require an arm-switch low-to-high cycle before walking resumes.

Make both timeouts configurable.

Do not treat packet loss as an immediate phase reset.

Provide a configurable emergency-stop policy:
- HOLD_POSITION
- CONTROLLED_STOP
- TORQUE_OFF

Use the project's existing safety behavior if one exists.

A radio kill switch is not a substitute for a physical emergency stop.

==================================================
GAIT STATE MACHINE
==================================================

Use or extend a state machine with states similar to:

- DISARMED
- STANDING
- STARTING
- WALKING
- STOPPING
- GAIT_TRANSITION
- LINK_LOSS_STOP
- FAULT

Do not instantly start or stop the oscillator based on stick deadband.

STARTING:
- Ramp stride amplitude and commanded body speed from zero.
- Preserve phase continuity.
- Lift a leg only when support conditions are valid.

STOPPING:
- Ramp body velocity to zero.
- Finish any active leg swing.
- Place all feet into a stable stance.
- Stop or park the oscillator only after all feet are safely down.

Never reset gait phase merely because:
- the stick crossed center
- speed changed
- gait mode changed
- an RC packet arrived late

==================================================
RUNTIME GAIT CHANGES
==================================================

Support existing gait modes, for example:
- wave
- ripple or tetrapod
- tripod

For the first implementation, use a conservative transition:

1. Queue the newly requested gait.
2. Ramp walking velocity to zero.
3. Finish all active swing motions.
4. Reach a stable all-feet stance.
5. Change gait phase offsets and duty factor.
6. Resume toward the latest requested body command.

Do not instantly replace phase offsets while multiple feet are in swing.

This stop-settle-switch-resume method is acceptable for version one and is safer than an incomplete phase-blending implementation.

Keep the architecture open for future blended gait transitions.

==================================================
FOOT TRAJECTORIES
==================================================

Generate foot motion in Cartesian space before inverse kinematics.

At leg lift-off, latch:
- actual lift-off position
- intended touchdown position
- swing duration
- step height
- start velocity
- expected touchdown velocity
- gait mode
- relevant body command

Do not recompute and teleport the swing endpoint every time a new RC frame arrives.

New RC commands should primarily affect:
- stance velocity
- future steps
- safely bounded adjustments

--------------------------------
HORIZONTAL SWING PATH
--------------------------------

Use cubic Hermite interpolation for x and y as the first implementation.

For normalized time u in [0, 1]:

h00 =  2u^3 - 3u^2 + 1
h10 =    u^3 - 2u^2 + u
h01 = -2u^3 + 3u^2
h11 =    u^3 -   u^2

p(u) =
    h00 * p0
    + h10 * duration * v0
    + h01 * p1
    + h11 * duration * v1

Set:
- v0 to the foot's relative velocity at lift-off.
- v1 to the expected stance-relative velocity at touchdown.

This avoids an obvious velocity discontinuity when entering or leaving stance.

If the codebase already has a tested quintic Hermite implementation, it may be used instead.

--------------------------------
VERTICAL SWING PATH
--------------------------------

Use the minimum-jerk scalar:

q(u) = 10u^3 - 15u^4 + 6u^5

For the first half of swing:
- rise from lift-off height to the apex using q(2u).

For the second half:
- descend from the apex to touchdown using 1 - q(2u - 1).

Adapt signs to the coordinate convention.

Vertical velocity should be zero or very close to zero at:
- lift-off
- apex
- touchdown

--------------------------------
STANCE PATH
--------------------------------

During stance, approximate the foot's body-relative velocity as:

footVelocity =
    -bodyLinearVelocity
    -cross(bodyAngularVelocity, footPosition)

Use the repository's coordinate conventions.

If the code already pins stance feet in an odometry or world frame, preserve that implementation.

==================================================
INVERSE KINEMATICS CONTINUITY
==================================================

Do not permit IK to jump between mathematical branches.

For each leg:
- Generate all valid IK solutions if the solver supports multiple branches.
- Reject solutions outside configured joint limits.
- Select the valid solution closest to the previous joint configuration.
- Use wrapped angular distance.
- Unwrap angles before filtering or limiting.

Guard trigonometric inputs:
- Clamp acos/asin inputs to [-1, 1].
- Detect unreachable foot targets.
- Report clipping or projection to the reachable workspace.

Avoid commanding the leg exactly at full extension where possible.

If only one IK branch is currently supported, add checks that detect sudden joint jumps and report them.

==================================================
FINAL JOINT SAFETY GUARD
==================================================

After IK, apply a final per-joint velocity guard:

maximumDelta = maximumJointVelocity * dt
command =
    clamp(target,
          previousCommand - maximumDelta,
          previousCommand + maximumDelta)

This is a safety backstop, not the primary trajectory generator.

Requirements:
- Make joint velocity limits configurable per joint or per joint type.
- Record a diagnostic counter whenever the guard activates.
- If it activates frequently, report that the gait command is exceeding safe joint dynamics.
- Do not silently depend on this layer for every step.

An optional joint acceleration guard may be added if the existing architecture makes it straightforward, but it is not required for the first working version.

==================================================
SERVO OUTPUT
==================================================

Compute all servo targets before transmitting any target.

Preferred output:

servoDriver.writeAll(jointTargets)

Use:
- synchronized write
- group write
- batch write
- one controller packet containing all channels

If the hardware does not support a batch operation:
- Prepare the complete target buffer first.
- Send targets consecutively without delays or blocking reads.
- Measure and document total inter-servo skew.

Do not perform:
- write servo 1
- wait
- read servo 1
- write servo 2
- wait
- etc.

Determine whether the servo or servo controller already performs internal speed or acceleration profiling.

Choose one deliberate policy:
1. External trajectory generation with internal servo profiling disabled or made sufficiently fast.
2. Internal servo profiling with lower-rate goal updates.

Do not accidentally stack a slow internal trajectory profile on top of a slow external trajectory profile without documenting why.

==================================================
16-CHANNEL RC DEFAULT MAPPING
==================================================

Make every channel assignment configurable. Preserve current assignments where they already exist.

Use this mapping as a default proposal:

Channel 1:
- Forward/reverse velocity

Channel 2:
- Sideways velocity

Channel 3:
- Yaw rate

Channel 4:
- Body height

Channel 5:
- Three-position gait selection:
  - low: wave
  - middle: ripple/tetrapod
  - high: tripod

Channel 6:
- Maximum walking-speed scale
- Suggested range: 25% to 100% of configured safe maximum

Channel 7:
- Step-height control

Channel 8:
- Responsiveness/smoothing control
- Map this to safe presets rather than completely unbounded values
- Example:
  - low: smooth
  - middle: balanced
  - high: responsive

Channel 9:
- Input-filter mode:
  - low: NONE_DIAGNOSTIC
  - middle: EMA
  - high: MEDIAN3_EMA

Channel 10:
- Command-shaper mode:
  - low: DIRECT_DIAGNOSTIC
  - middle: ACCEL_LIMITED
  - high: JERK_LIMITED

Channel 11:
- Gait-frequency scale or stride/frequency balance

Channel 12:
- Stance-width adjustment

Channel 13:
- Three-position robot-state request:
  - low: sit/rest
  - middle: stand
  - high: walking enabled

Channel 14:
- Tuning-layer modifier

Channel 15:
- Arm/disarm

Channel 16:
- Emergency or radio-kill request using the configured emergency-stop policy

Safety rules:
- Direct/raw diagnostic modes must not be selectable while walking in production.
- Debounce every switch.
- Require a switch position to remain stable before accepting a new mode.
- Apply mode changes only on a detected position change.
- Use current-state initialization so switching modes does not create an output jump.
- Filter continuous parameter changes before applying them to the gait.
- Clamp every remotely adjustable parameter to a conservative range.
- Do not allow tuning controls to set zero, negative, NaN, or unreasonable limits.

==================================================
TUNING LAYER
==================================================

When Channel 14 enables the tuning layer, allow additional runtime tuning only when:
- the robot is disarmed, standing, or moving below a very small speed threshold

Suggested tuning-layer assignments:

Channel 6:
- EMA time constant

Channel 7:
- translation acceleration limit

Channel 8:
- translation jerk limit or response time

Channel 11:
- gait duty factor

Channel 12:
- joint-rate guard scale or stance width

Mode channels 9 and 10 may retain their filter/shaper selection behavior.

Store runtime tuning in RAM initially.

Persistent storage is optional and must be protected against accidental writes. If implemented:
- allow saving only while disarmed
- require a deliberate multi-switch gesture or host command
- validate values and include a configuration version/checksum
- retain compile-time defaults as a recovery option

==================================================
CONFIGURATION PRESETS
==================================================

Create three safe responsiveness presets:

SMOOTH:
- larger EMA tau
- lower acceleration
- lower jerk
- slower gait-frequency ramp

BALANCED:
- default production preset

RESPONSIVE:
- smaller EMA tau
- higher but still bounded acceleration
- higher but still bounded jerk

Changing presets must transition parameters smoothly. Do not instantly replace all limits while walking.

Put exact values in one configuration file and document their units.

==================================================
COMPILE-TIME OPTIONS
==================================================

Use the project's idiomatic build configuration. Examples of required options:

- ENABLE_RUNTIME_RC_TUNING
- ENABLE_MEDIAN3_FILTER
- ENABLE_JERK_LIMITER
- ENABLE_LEGACY_DIAGNOSTIC_MODES
- ENABLE_PERSISTENT_TUNING
- DEFAULT_INPUT_FILTER_MODE
- DEFAULT_COMMAND_SHAPER_MODE
- DEFAULT_GAIT_MODE
- CONTROL_LOOP_FREQUENCY_HZ

Production defaults:
- runtime tuning enabled only if safely implemented
- EMA filter
- acceleration-limited command shaper
- diagnostic direct modes disabled
- joint-rate safety guard enabled
- RC timeout enabled
- synchronized servo writes enabled

Avoid a large set of preprocessor conditionals inside the control loop. Prefer compiled strategy classes or constexpr configuration where appropriate.

==================================================
MODE-SWITCH CONTINUITY
==================================================

All runtime mode changes must be bumpless.

Input-filter change:
- new filter output starts at the old filter output

Command-shaper change:
- new shaper value starts at the current shaped value
- carry acceleration/rate state where possible

Gait change:
- use stop-settle-switch-resume for version one

Responsiveness-preset change:
- interpolate the tuning values over a short configurable interval

No runtime mode change may:
- reset the body command to zero for one frame
- reset gait phase
- reset filtered input to raw input
- reset joint targets to a default pose
- create a large servo command step

==================================================
TELEMETRY AND DIAGNOSTICS
==================================================

Add lightweight diagnostics for:

- raw RC values
- normalized RC values
- filtered RC values
- requested body command
- shaped body command
- current acceleration in jerk-limited mode
- current gait
- requested gait
- gait state
- master phase
- each leg's stance/swing state
- foot targets
- joint targets
- joint-rate guard activations
- RC packet age
- failsafe state
- control-loop dt
- control-loop overruns
- servo packet duration
- IK clipping or unreachable-target count

Do not print all of this synchronously from the real-time loop.

Use:
- a ring buffer
- lower-rate telemetry
- an existing logging task
- an existing serial telemetry mechanism

Provide a concise live diagnostic view that can show:
- selected filter mode
- selected command-shaper mode
- selected gait
- current tuning preset
- RC link state

==================================================
UNIT AND SIMULATION TESTS
==================================================

Add tests for at least the following.

RC normalization:
- minimum maps to -1
- center maps to 0
- maximum maps to +1
- asymmetric calibration
- reversed channel
- invalid values

Deadband:
- values inside the deadband produce zero
- values outside are rescaled continuously
- no discontinuity at the edge beyond floating-point tolerance

Expo:
- zero remains zero
- endpoints remain endpoints
- sign is preserved

EMA:
- step response is monotonic
- behavior remains similar under different loop dt values
- reset starts from the requested initial value
- tau zero bypasses safely

Median-of-three:
- rejects a single isolated spike
- does not reorder stable input

Switch debounce:
- ignores contact or receiver jitter
- emits one mode change per real switch transition

Acceleration limiter:
- output change never exceeds configured acceleration/deceleration times dt
- sign reversal decelerates safely
- returning to zero is monotonic

Jerk limiter:
- acceleration remains bounded
- change in acceleration remains bounded by maxJerk * dt
- no sustained oscillation at zero
- no unacceptable overshoot after a step
- sign reversals converge correctly

Mode switching:
- first output after switching equals the previous mode output within tolerance

Gait:
- phase does not reset during speed changes
- stop completes active swings
- gait transition reaches stable stance before switching offsets
- swing endpoint remains latched through the swing

Foot trajectory:
- position is continuous
- horizontal velocity is continuous at Hermite endpoints
- vertical position and velocity are continuous
- apex equals requested step height
- no NaN values for u in [0,1]

Joint guard:
- no command delta exceeds maxJointVelocity * dt
- activation counter increments

RC loss:
- body target becomes zero
- shaped command decelerates rather than jumping
- active feet return safely to stance
- rearming policy works

==================================================
ACCEPTANCE CRITERIA
==================================================

Distribute these criteria across the child issues; the epic is closed only when every child issue is closed and all of the following hold:

1. A noisy centered RC input remains zero after filtering and deadband.
2. A full stick step does not create an immediate full body-velocity step.
3. Acceleration-limited mode respects configured rate limits.
4. Jerk-limited mode respects configured acceleration and jerk limits.
5. Switching filter or shaper modes produces no output discontinuity.
6. Gait phase is not reset by stick-center crossings.
7. A leg's swing start and touchdown targets are latched for that swing.
8. Foot targets are continuous through swing.
9. Gait changes use a safe controlled transition.
10. RC loss causes a controlled stop.
11. Every servo target is computed before the output operation starts.
12. A synchronized/batch servo write is used when supported.
13. No joint command exceeds the final configured joint-rate guard.
14. The code builds and existing tests still pass.
15. New unit tests pass.
16. A tuning document explains every parameter, its unit, default, safe range, compile-time option, and RC assignment.

==================================================
DELIVERABLES
==================================================

For this planning session, deliver:

1. One Beads epic capturing the goal, the audit findings on current sources of jerk, and the control-pipeline description.
2. Child issues covering every item in REQUIRED BEADS EPIC BREAKDOWN, each with acceptance criteria, labels, priority, and dependencies matching the implementation order.
3. `bd remember` entries for durable hardware/architecture facts discovered during the audit.
4. A verification pass: `bd ready` shows the correct first actionable issues and no dependency cycles exist.

Across the life of the epic, the implementation issues must collectively produce:

1. A short description of the original source of jerk found in the code.
2. A list of modified and added files.
3. A diagram or concise description of the final control pipeline.
4. The final 16-channel mapping.
5. Compile-time options and their defaults.
6. Runtime-selectable modes.
7. All default tuning values and units.
8. Test results.
9. Any hardware limitations, especially:
   - servo bus throughput
   - lack of synchronized write
   - internal servo profiling
   - control-loop jitter
10. A safe physical test procedure.

==================================================
SAFE PHYSICAL TEST PROCEDURE
==================================================

Document and follow this order:

1. Run all unit tests without servo output.
2. Run telemetry with RC input but servo output disabled.
3. Verify filtering and shaper plots for step and center-noise inputs.
4. Test with the robot securely supported so the feet cannot carry full weight.
5. Use the lowest speed, gait frequency, and step height.
6. Test stand/sit transitions.
7. Test one gait at a time.
8. Test controlled stopping.
9. Test RC timeout.
10. Test runtime mode changes while standing.
11. Only then test low-speed walking on the ground.
12. Increase limits gradually while monitoring supply voltage, loop overruns, IK clipping, and joint-rate guard activations.

Prioritize predictable, testable behavior over cleverness.