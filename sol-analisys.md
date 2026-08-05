# Hexapod Walking Failure Analysis

Date: 2026-08-04

Scope: `firmware/openrb150`, checked against the current HexNav model in
`robot_ros_simulation/HexNav_description/urdf`. The Mark III source in
`temp_example/Mark_III_CM9-main/Phantom_Phoenix` is used only as a reference
for gait state, sequencing, and IK architecture.

Beads issues: original analysis `hexapod_src-5eg`; URDF-grounded review
`hexapod_src-d6z`.

## Executive conclusion

The current firmware does contain the Mark III tripod/ripple/wave phase tables,
and their leg ordering is correctly translated to this project's logical leg
order. The main failure is not the phase table. It is the reduction of the
HexNav URDF chain to an incorrect point-foot model.

The active constants `56.08 / 66.51 / 24.86 mm`, the hip mounts, the hard-coded
home XY points, and the `40 mm` home height are not arbitrary Mark III values.
They reproduce the HexNav URDF's **`leg_n_tibia` frame origin** at all joint
angles equal to zero. The crucial error is treating that internal link frame as
the physical ground-contact point and assuming that matching its neutral XYZ is
enough to reproduce its motion.

It is not enough. Exact URDF FK and firmware FK agree near
`(127, 0, -44.55) mm` at zero, but their joint-space derivatives disagree. A
positive tibia command moves the URDF endpoint radially outward at about
`+8.55 mm/rad`; firmware predicts about `-3.23 mm/rad`. The firmware selects a
synthetic `+53.38 deg` knee rest from a generic two-link triangle instead of
representing the URDF's fixed `140 deg` knee transform and structural offsets.
The robot can therefore stand at an apparently correct neutral pose and still
move each leg along the wrong curve as soon as walking begins.

The URDF has no explicit foot/toe link. Its tibia mesh supplies the missing
physical extent. Applying the exact current URDF transforms to that mesh gives:

- lowest zero-pose mesh point in the coxa frame:
   approximately `(131.59, 0.49, -153.39) mm`;
- distance from knee axis to that lowest point: approximately `120.25 mm`;
- farthest mesh point from the knee axis: approximately `124.47 mm`;
- lowest point in the centred body frame: approximately `-148.89 mm` Z because
   the coxa joint frame is `+4.5 mm` above the centred body origin.

These values come from the current checked-in URDF and DAE mesh. They supersede
the earlier report's unsupported `123.14 mm` number. A rubber foot, sensor, or
other installed contact part may extend beyond the mesh and still requires a
hardware measurement.

With the short internal endpoint, a normal `60 mm` requested stroke exceeds the
modeled workspace. `GaitPipeline` then scales every leg's horizontal motion to
the most constrained leg. The existing regression test explicitly accepts only
`6 mm` of horizontal sweep for a `60 mm` request. This directly explains the
tiny steps, but it is a consequence of the wrong controlled endpoint and IK,
not evidence that the HexNav hardware should use Mark III dimensions.

The incoherent or apparently random leg motion has a second likely cause: the
active profile assumes all 18 joints use `sign = +1`, `trim_ticks = 0`, and
identical symmetric limits. A schema-v6/v7 migration also replaces the entire
servo profile instead of preserving measured signs, trims, and stricter limits.
Software tests prove that the firmware is self-consistent with those
assumptions; they do not prove that those assumptions match the assembled
robot.

The recent commit history is significant, but neither profile should be copied
unchanged:

- `ad03f39` rebuilt the motion profile from Mark III with a `133 mm` tibia,
  taller stance, side-aware output signs, horn offsets, and joint-specific
  limits. That improved workspace but used PhantomX dimensions and calibration,
  so it is not the correct HexNav solution.
- `ad9124b` replaced that profile with the current `24.86 mm` endpoint,
  `40 mm` stance, all-positive signs, zero trims, and generic limits.
- The current physical symptoms are consistent with replacing one mismatched
   model with another incomplete HexNav reduction.

Do not copy Mark III dimensions, servo IDs, horn offsets, or signs. Retain the
verified sequential ID map, derive kinematics from the complete HexNav URDF
chain and selected physical contact frame, then verify signs, trims, and limits
against the assembled hardware.

## Active movement path

The physical path is:

```text
CRSF channels
  -> ControllerBridge (operator command frame: forward, left, yaw)
  -> ControllerCore / BodyCommandShaper
  -> commandPlanarToBody (forward -> body +Y, left -> body -X)
  -> GaitEngine (Mark III phase/keyframe positions)
  -> GaitPipeline global reach-envelope scaling
  -> BodyKinematics / LegIk
  -> ServoMap (id, sign, trim, limits)
  -> DXL Sync Write
```

The command-frame conversion is correct. `ControllerBridge` stores forward in
`twist_vx`, but that field is explicitly the first command-frame component;
`ControllerCore::commandPlanarToBody` later maps it to mechanical body `+Y`.
This is not the source of the observed direction error.

## Confirmed findings

### 1. Critical: neutral pose matches the URDF, but movement does not

`config_schema.cpp` currently sets:

```text
coxa  = 56.08 mm
femur = 66.51 mm
tibia = 24.86 mm
```

Those numbers are correctly derived from portions of the HexNav URDF:

| Firmware value | HexNav URDF meaning | Verdict |
| --- | --- | --- |
| `56.08 mm` | horizontal reduction of hip-yaw axis to femur axis | consistent |
| `66.51 mm` | femur axis to tibia axis | consistent |
| `24.86 mm` | tibia axis to `leg_n_tibia` **frame origin** | consistent, but not a toe |

The firmware and exact URDF both place that frame at approximately
`(127, 0, -44.55) mm` in the coxa frame at zero. However, the URDF chain also
contains lateral/vertical offsets and a fixed `2.443461 rad` (`140 deg`) knee
rotation. `LegIk` discards those transforms, solves a generic triangle, selects
the positive `acos` branch, and subtracts synthetic rest angles so the one home
point happens to match.

The resulting local motion is wrong:

| Positive joint perturbation at zero | Exact URDF endpoint derivative (mm/rad) | Firmware derivative (mm/rad) |
| --- | --- | --- |
| femur | `(radial +23.55, vertical +74.99)` | `(radial +44.55, vertical +70.92)` |
| tibia | `(radial +8.55, vertical +10.19)` | `(radial -3.23, vertical +24.65)` |

The tibia radial component has the opposite sign. This is a direct software
explanation for legs moving in unexpected directions despite a correct-looking
stand pose.

The implementation should use the exact URDF transform chain for FK and an IK
that inverts the same chain. A carefully derived reduced analytic model is also
possible, but it must include the selected contact-point offset, fixed frame
rotations, the correct knee branch, and zero-angle offsets. Replacing only `L3`
with Mark III's `133 mm` or a mesh length is insufficient.

### 2. Critical: the controlled endpoint is not the physical contact point

The URDF ends kinematically at the `leg_n_tibia` frame. Its own
`inverse_kinematics.md` explicitly says the `24.86 mm` value is not necessarily
the rubber foot tip and must be extended for physical contact control.

The current DAE mesh reaches a lowest zero-pose point about `120.25 mm` from the
knee axis. That point is around `109 mm` lower than the URDF tibia frame origin
in the coxa frame. The gait therefore commands an internal frame to rise by
`30 mm`; it does not guarantee that the physical toe rises by `30 mm` or follows
the requested horizontal line.

Add a named fixed frame such as `leg_n_foot_contact` or `leg_n_toe` to the Xacro
at the actual sensor/rubber contact point. Firmware geometry should be generated
from or validated against that frame. A mesh extremity is useful for diagnosis,
but the installed contact point must be measured on hardware.

### 3. Critical: reach protection collapses the requested stride

The current neutral coxa-frame foot is approximately:

```text
radius = 127.0 mm
z      = -44.5 mm
```

After removing the `56.08 mm` coxa link, the planar two-link distance is about:

```text
sqrt((127.0 - 56.08)^2 + 44.5^2) = 83.7 mm
```

The pipeline's 95 percent maximum is:

```text
0.95 * (66.51 + 24.86) = 86.8 mm
```

Only about `3 mm` of workspace margin remains at neutral. At a `60 mm` stride,
the forward middle leg asks for a `30 mm` radial half-stroke. At the same foot
height, the permitted radial movement is only about `3.6 mm`, yielding a path
scale around `3.6 / 30 = 0.12`.

`GaitPipeline` computes one minimum `path_scale` across every leg and every
sample, then applies it to all legs. One constrained sample therefore shrinks
the complete gait. The safety behavior is reasonable, but with the wrong model
it guarantees tiny steps.

The existing `test_large_stride_is_reach_limited_not_unreachable` treats this
collapse as success: it only requires the output to remain reachable and to
report that limiting occurred. The newer
`test_default_forward_walk_takes_real_steps` still accepts only `6 mm` of
horizontal sweep for a `60 mm` command. Its comment mentions a stance-bias
implementation that is no longer present in the active `GaitEngine`, so the
test and implementation have drifted apart.

### 4. High: configuration migration can erase physical calibration

For legacy schema v6 or v7, `deserializeRobotConfig` calls
`applyRobotMotionProfile(out)`. That function overwrites all servo IDs, signs,
trims, and limits with the new defaults. The prior Mark III migration explicitly
preserved calibration by physical ID; the active migration does not.

Consequences include:

- measured zero offsets can silently return to zero;
- mirrored joint signs can silently become `+1`;
- joint-specific safe limits can become generic `1024..3072` limits;
- a robot that stood correctly under an older EEPROM profile can move
  incorrectly immediately after a firmware/schema upgrade.

This is both a motion defect and a safety defect. Schema migration must preserve
verified per-servo calibration unless a deliberately versioned transformation
is required. It must never silently replace calibration with assumptions.

### 5. High: the default physical output profile is unverified by tests

The active defaults assign sequential IDs in logical leg-major order, all
positive signs, zero trims, and symmetric limits. Sequential IDs are recorded
as physically verified, including leg 1 femur on ID 2. The remaining properties
are not established by the current unit tests.

Phoenix explicitly has installation-dependent inversion and horn calibration:

- `Hex_Cfg.h` defines side-specific tibia inversion;
- Phoenix applies femur and tibia horn offsets;
- Phoenix uses joint-specific coxa/femur/tibia angle limits;
- `_Phoenix_Code.h` applies inversion before sending each joint to the driver.

This robot may legitimately use different signs because its servo installation
is different. However, that must be demonstrated one joint at a time with
torque-limited, suspended hardware. A test that expects every stand goal to be
tick `2048` only proves the configured assumption, not the physical result.

### 6. High: the Mark III parity test is circular and too small

`test_mark_iii_gait_parity` is not an independent Mark III oracle:

- it duplicates the firmware keyframe equations;
- it duplicates the firmware body-to-coxa and leg IK equations;
- it uses the same incomplete `24.86 mm` internal-frame endpoint;
- it uses only `2 mm` stride and `5 mm` lift;
- it therefore stays inside the narrow workspace that fails at normal stride;
- it compares calculated ticks, not physical foot trajectories or measured
  joint direction.

The test passes now, but it cannot falsify the reported hardware failure. A
useful parity fixture should contain golden Phoenix outputs captured from the
reference implementation, transformed once into this robot's leg order and
coordinate frame. Separate geometry tests should use this robot's measured
dimensions and assert realized foot displacement through FK.

### 7. High: small travel commands reduce lift and encourage foot drag

Phoenix treats leg lift height as a gait parameter independent of travel
amplitude once `TravelRequest` crosses its dead zone. The current
`GaitEngine::footAt` multiplies step height by:

```text
clamp(max(|vx|, |vy|, |wz|) * 4, 0, 1)
```

Commands below `0.25` therefore produce proportionally reduced clearance. With
geometry already causing horizontal scaling, low-stick walking can drag or
catch feet instead of cleanly transferring support. Preserve a minimum safe
swing clearance after motion starts, or make lift scaling an explicit tested
policy rather than coupling it to travel magnitude.

### 8. Medium: stopping a gait snaps all feet to home

When command magnitude reaches zero, `GaitEngine::update` immediately emits the
home position for all six feet and stops phase advancement. Phoenix retains
`fWalking`/`TravelRequest` while gait offsets settle and returns each leg toward
home through the gait sequence.

The current body-command shaper reduces the frequency of this discontinuity,
but once the command crosses the deadband, any residual per-leg gait offset is
discarded in one control cycle. Multiple legs can then move toward home at once
instead of completing a supported touchdown/stance transition. This can look
like unrelated or random leg motion when the stick is released or the link
briefly jitters around neutral.

Implement a phase-safe stop: finish or shorten active swings, place those feet,
settle stance offsets, then park the gait.

### 9. Medium: gait changes have no stop-settle-switch-resume transition

`setGait` changes the active table immediately while preserving the current
numeric phase. Tripod, ripple, and wave have different step counts and leg
origins, so the same phase does not represent the same support state after a
switch. Switching while moving can abruptly reclassify swing and stance legs.

Phoenix-style behavior should stop and settle before changing gait, or perform
an explicit support-aware phase mapping. Do not switch origin tables in place
while feet are loaded.

### 10. Medium: gait home XY and configurable geometry are separate authorities

`GaitEngine` hard-codes six body-frame home XY points, while `RobotConfig`
separately stores hip mounts, mount yaw, and home radius. Changing the geometry
configuration does not regenerate the gait home points. The IK rest pose can
therefore describe one geometry while the gait generator commands another.

The current constants happen to match the URDF tibia-frame home pose. Once a
real contact frame is added, derive each home foot from the URDF/contact model
or store the complete per-leg contact vectors in versioned configuration.
Validate not only the neutral point but also FK Jacobians and representative
trajectories.

## What is already correct

- The six firmware hip mount positions and yaws match the current URDF after
  subtracting the CAD body centre `(76.463, 126.463) mm`.
- The firmware coxa-frame Z origin is correct: mount `-16.5 mm` plus coxa-joint
  lift `+21.0 mm` gives `+4.5 mm` in the centred body frame.
- `56.08 mm`, `66.51 mm`, and the hard-coded home feet reproduce the URDF
  tibia-frame origin at zero. The failure is away from zero and at the physical
  contact point.
- The active gait origins match Phoenix after translating Phoenix leg order
  `RR, RM, RF, LR, LM, LF` to firmware order `LR, RR, RM, RF, LF, LM`.
- Tripod groups alternate `{LR, RM, LF}` and `{RR, RF, LM}` as expected.
- The HexNav IK document's alternative tripod `{1,3,4}/{2,5,6}` has one support
   edge through the body centre and does not match the Mark III APG table; do not
   replace the firmware grouping with that stale prose.
- Forward/left operator commands are converted to body `+Y/-X` at the
  controller-to-gait boundary.
- DXL goals pass through the servo map and configured limits; clients do not
  directly bypass the firmware safety path.
- Reach limiting keeps the bad model from commanding unreachable points. The
  problem is that the model makes normal walking collapse into that limiter.

## Recommended correction sequence

### Stage 1: establish the controlled HexNav contact frame

1. Add an explicit `foot_contact`/`toe` fixed link to each leg in the HexNav
   Xacro at the actual installed contact point.
2. Generate exact FK from the URDF chain and verify the zero pose, Jacobian, and
   toe path independently of firmware `LegIk`.
3. Keep the robot suspended and start torque-off.
4. Read and archive the current EEPROM config and all present servo positions.
5. Confirm the sequential ID-to-leg/joint map one joint at a time.
6. Jog each joint by a small positive logical angle and record the physical
   direction. Derive `sign`; do not infer it from left/right alone.
7. Place every leg in the intended URDF-zero stance and record the tick for each
   mechanical zero. Derive `trim_ticks`.
8. Measure the installed knee-axis-to-contact transform and compare it with the
   new URDF contact frame.
9. Establish joint-specific collision-safe limits at low speed and torque.

### Stage 2: repair the model and configuration contract

1. Implement exact URDF/contact-frame FK in a portable module.
2. Implement IK against that same FK, preferably first with a bounded numerical
   solver seeded from the previous cycle. An analytic reduction is acceptable
   only after it matches exact FK position and Jacobian tests.
3. Use URDF joint zero directly. Do not synthesize a different rest geometry
   merely to force one home point to match.
4. Make home contact vectors and IK geometry share one source of truth.
5. Preserve IDs, signs, trims, and stricter limits during schema migration.
6. Reject a gait profile whose normal configured stride requires severe global
   scaling. Configuration validation should check the full gait envelope, not
   only the neutral annulus.
7. Report requested versus realized stride and the limiting leg/sample in
   telemetry.

### Stage 3: reproduce Mark III motion semantics with this geometry

1. Keep the current verified Mark III APG phase origins and support ordering.
2. Generate translation/yaw **contact-point** offsets in the HexNav body frame.
3. Keep safe swing clearance independent of small travel amplitude.
4. Add phase-safe start, stop, and gait switching.
5. Keep Cartesian interpolation if desired, but validate support timing and
   touchdown behavior against Phoenix golden traces.
6. Start with wave or ripple at low speed; enable tripod only after every leg's
   direction, lift, touchdown, and stance travel are verified.

### Stage 4: replace self-referential tests

Add independent checks for:

- golden Phoenix gait offsets for tripod, ripple, and wave over complete
  cycles;
- leg-order conversion from Phoenix order to firmware order;
- exact URDF FK golden points and finite-difference Jacobians;
- URDF-contact-frame IK/FK round trips at neutral and stroke extrema;
- explicit regression for positive tibia radial/vertical direction at zero;
- minimum realized stride at configured normal commands;
- no global reach limiting during the approved normal gait envelope;
- one-joint positive-direction truth for all 18 hardware joints;
- migration preservation of every servo ID/sign/trim/limit;
- phase-safe command stop and gait switch;
- suspended-robot traces of commanded tick versus present tick;
- ground tests only after suspended tests pass.

## Acceptance evidence for a real fix

A software-only green test suite is insufficient. A corrected firmware should
produce all of the following evidence:

1. With torque off, the logical map matches all 18 physical joints.
2. In suspended single-joint tests, positive coxa/femur/tibia commands move in
   the documented direction for every leg.
3. Stand reaches the URDF/contact-frame neutral toe positions without servo
   clamping or IK reach limiting.
4. At low-speed wave gait, each leg performs a coherent sequence: unload,
   lift, move forward, descend, load, and move backward in stance.
5. A straight forward command produces symmetric left/right trajectories and
   no unintended lateral/yaw component.
6. Realized contact-point stride is close to requested stride and is not merely
   the current test minimum of `6 mm` for a `60 mm` request.
7. Releasing the stick lands active swing legs and settles to stand without a
   simultaneous snap of all feet.
8. Changing gait settles first or maps support phase without abrupt leg
   reclassification.
9. EEPROM migration preserves measured servo calibration exactly.
10. Ground testing begins with a tether/stand, low torque/speed, and wave or
    ripple gait before tripod.

## Immediate safety recommendation

Do not continue full-body ground walking with the active profile. The exact
URDF comparison proves that its endpoint Jacobian is wrong even before unknown
hardware signs and trims are considered. Until the contact frame, IK branch,
fixed offsets, and per-joint physical mapping are verified, use torque-off
inspection followed by one-joint and one-leg suspended tests. The present reach
limiter prevents some impossible targets in the wrong reduced model; it cannot
protect against an incorrect toe path, servo sign, zero offset, ID assignment,
or collision limit.