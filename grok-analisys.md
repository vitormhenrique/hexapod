# Hexapod Movement Firmware Analysis

**Date:** 2026-08-04  
**Scope:** `firmware/openrb150` gait / IK / geometry vs HexNav URDF and Mark III Phoenix reference  
**Symptom:** robot takes tiny steps; legs do not move straight; motion looks random / incorrect  
**Goal of this note:** document root causes only (no Beads tasks, no code changes)

---

## 1. Executive summary

The current movement stack is **not wrong only in “tuning.”** Several independent geometry and mapping bugs stack on top of each other. The dominant failure mode is:

1. Firmware treats the **URDF tibia-link frame** as the foot tip (`L_TIBIA = 24.86 mm`).
2. That makes the reachable workspace tiny and places the home stance at the edge of full extension.
3. Reach limiting then collapses a 60 mm commanded stride to roughly **~10 mm**.
4. The physical rubber toe is much farther from the knee than 24.86 mm, so every IK solution moves a **phantom point near the knee**, while the real toe swings on a long lever arm.
5. Default servo signs are **all `+1`**, ignoring HexNav’s left/right mirror requirement, so half the robot pushes the wrong way for the same body command.

Mark III is a good **algorithm reference** (gait keyframes, body FK, planar leg IK, travel/lift separation). It is **not** a geometry source. HexNav dimensions must come from `robot_ros_simulation/HexNav_description/urdf` (and a measured toe tip beyond that URDF frame).

Until tibia length, home toe pose, servo signs, and reach model are corrected together, no gait-table tweak will make the robot walk straight.

---

## 2. Sources reviewed

| Source | Role |
| --- | --- |
| `firmware/openrb150/src/config/config_schema.cpp` | Compiled geometry + servo defaults |
| `firmware/openrb150/src/gait/leg_ik.*` | 3-DOF planar IK |
| `firmware/openrb150/src/gait/body_ik.*` | Body → coxa transform + body pose |
| `firmware/openrb150/src/gait/gait_engine.*` | Mark III-style foot trajectory |
| `firmware/openrb150/src/gait/gait_pipeline.*` | Reach envelope + angle→tick path |
| `firmware/openrb150/src/dxl/servo_map.*` | Angle → MX-28 ticks |
| `firmware/openrb150/src/controller/controller_core.cpp` | Command-frame → body-frame twist |
| `robot_ros_simulation/HexNav_description/urdf/HexNav.urdf` | True mechanical model |
| `robot_ros_simulation/HexNav_description/docs/inverse_kinematics.md` | Extracted HexNav IK reference |
| `temp_example/Mark_III_CM9-main/Phantom_Phoenix/*` | Working Phoenix gait/IK reference |

---

## 3. What “correct” should look like

### 3.1 Mark III idea (keep this structure)

Mark III / Phantom Phoenix does roughly:

```text
Travel sticks
  -> Gait() relative foot offsets (GaitPosX/Y/Z, GaitRotY)
  -> BodyFK (body translation/rotation about body centre + coxa offsets)
  -> LegIK (coxa yaw + 2-link femur/tibia)
  -> servo PWM/ticks with per-leg inversions and horn offsets
```

Important Mark III properties:

- Feet are commanded as **body-relative travel**, not raw joint wiggles.
- Stance pushes opposite the travel request; swing lifts then advances.
- Leg IK uses **real link lengths to the foot tip**.
- Per-leg coxa mount angles and **servo inversions** are first-class.
- Geometry constants live in `Hex_Cfg.h` and are robot-specific.

Mark III Hexapod geometry (for comparison only):

| Quantity | Mark III |
| --- | ---: |
| Coxa | 52 mm |
| Femur | 66 mm |
| Tibia (to tip) | **133 mm** |
| Init foot radial XZ | 147 mm |
| Init foot height Y | 25 mm |
| Body half-length / half-width style offsets | ~60 / 100 / 120 mm class |

### 3.2 HexNav geometry (use this robot)

From URDF + `inverse_kinematics.md`, body-centred frame **B** (`base_link − (76.463, 126.463, 0) mm`):

**Coxa mounts in B**

| Leg | Role | Mount XY (mm) | Mount Z (mm) | Mount yaw |
| ---: | --- | ---: | ---: | ---: |
| 1 | rear-left | (−65.58, −115.58) | −16.5 | +135° |
| 2 | rear-right | (+65.58, −115.58) | −16.5 | −135° |
| 3 | mid-right | (+69.78, 0.00) | −16.5 | −90° |
| 4 | front-right | (+65.58, +115.58) | −16.5 | −45° |
| 5 | front-left | (−65.58, +115.58) | −16.5 | +45° |
| 6 | mid-left | (−69.78, 0.00) | −16.5 | +90° |

**Reduced leg links (URDF frames)**

| Link | Firmware/doc value | Meaning |
| --- | ---: | --- |
| `L_COXA` horizontal | 56.08 mm | hip yaw axis → femur axis, radial |
| `L_FEMUR` | 66.51 mm | femur axis → tibia axis |
| `L_TIBIA` | **24.86 mm** | tibia axis → **`leg_n_tibia` frame origin only** |

The HexNav IK doc itself warns:

> `L_TIBIA` is the distance to the `leg_n_tibia` link frame origin … **not necessarily the rubber foot tip**. … **This is the one number to calibrate on hardware.**

**URDF all-zero home feet in B** (tibia-frame endpoints, not rubber tips):

| Leg | Foot XY (mm) | Z (mm) |
| ---: | ---: | ---: |
| 1 | (−155.4, −205.4) | −40.0 |
| 2 | (+155.4, −205.4) | −40.0 |
| 3 | (+196.8, 0.0) | −40.0 |
| 4 | (+155.4, +205.4) | −40.0 |
| 5 | (−155.4, +205.4) | −40.0 |
| 6 | (−196.8, 0.0) | −40.0 |

Per-leg coxa-frame home (tibia frame): **r = 127.0 mm, z = −44.55 mm**.

**Exact chain extras the planar model currently ignores**

- Coxa joint origin lift: +21.0 mm above mount.
- Femur axis in coxa frame is not pure radial: **(52.0, −21.0, −21.0) mm**.
- Tibia joint has a baked rest yaw of **140°** in the URDF zero pose.

---

## 4. Current firmware path (what actually runs)

```text
RC / host twist (operator frame: forward/left)
  -> ControllerCore commandPlanarToBody()
  -> GaitEngine foot targets in body frame B
  -> optional global horizontal path_scale (reach envelope)
  -> BodyKinematics body->coxa
  -> LegIk solve (relative to home rest angles)
  -> ServoMap angleToTick (center 2048 + sign*deg + trim)
  -> DXL sync write
```

This structure is fine. The numbers and conventions fed into it are not.

---

## 5. Root causes ranked by impact

### P0 — `L_TIBIA` is the wrong physical length

**Where**

- `config_schema.cpp` → `cfg.links.tibia_cmm = 2486` (= 24.86 mm)
- `leg_ik.h` comments and parity tests hard-code the same value
- HexNav docs explicitly say 24.86 mm is **not** the toe tip

**Why it breaks walking**

Mark III’s tibia is **133 mm to the tip**. HexNav’s URDF tibia frame is only **24.86 mm** past the knee axis. The printed tibia + rubber toe continues far beyond that frame.

With current numbers:

```text
L2 + L3 = 66.51 + 24.86 = 91.37 mm
home planar reach d ≈ 83.75 mm
95% safe reach ≈ 86.80 mm
radial home = 127 mm
radial outer margin ≈ 3.6 mm
```

So the robot is commanded as if every foot tip lives almost at the knee. Consequences:

1. **Standing at all-zero can still look plausible**, because servo centre = URDF zero, and the mechanical home pose is real.
2. **Any commanded foot displacement is solved for a phantom point** ~25 mm from the knee.
3. The real toe, being much farther out, **amplifies that joint motion** into large, wrong Cartesian arcs.
4. The reachable annulus is so small that normal walking strokes are mostly illegal.

This single bug alone explains “tiny steps + weird leg flailing.”

**What should happen**

- Measure the true knee-axis → contact-tip length on hardware (or from mesh lowest point in the tibia/toe frame).
- Store that as `L_TIBIA`.
- Recompute home foot XYZ in body/coxa frames from **true tip FK**, not from the tibia-link origin.
- Recompute femur/tibia rest angles from that true home tip.

A previous internal note claimed toe-referenced `L_TIBIA ≈ 123.14 mm`. Current source still uses **24.86 mm**. That fix is not present in the live motion profile.

---

### P0 — Reach limiting collapses stride to ~10 mm

**Where**

- `gait_pipeline.cpp` global `bodyTargetPathScale()` over Mark III envelope samples
- `leg_ik.cpp` `kReachMarginFrac = 0.95`
- default gait `stride_len_mm = 60`

**Measured effect with current geometry**

For a full forward command (`vy = 1`, stride 60 mm, height 40 mm, step 30 mm), the pipeline’s global horizontal path scale bottoms out around:

```text
path_scale ≈ 0.164
effective stride ≈ 60 × 0.164 ≈ 9.8 mm
```

The worst samples are rear-corner legs at the trailing stroke endpoint (`longitudinal = -0.5`), which leave the tiny annulus.

So even before wrong toe leverage, the commanded walk is forced into **centimetre-scale shuffles**. That matches the observed “tiny steps.”

Because the scale is global, all legs are shortened together. The motion can still look incoherent once combined with wrong tibia length and wrong servo signs.

---

### P0 — Default servo signs ignore left/right mirroring

**Where**

- `config_schema.cpp` sets every `servo.sign = 1`
- `servo_map.cpp` uses `center + sign * deg(angle)`
- HexNav IK doc recommended starting map:

| Joint | Left legs 1/5/6 | Right legs 2/3/4 |
| --- | :---: | :---: |
| coxa / femur / tibia | +1 | −1 |

**Why it breaks walking**

Body-frame IK produces URDF-signed angles. On a sagittally mirrored hexapod, the same positive coxa command is **not** the same physical horn direction on left and right legs.

With all signs `+1`:

- one side tracks the intended body motion
- the opposite side drives joints the wrong way for the same Cartesian target
- net behaviour is shearing, spinning, or “random” looking leg fights instead of a rigid stance push

Mark III encodes this with `cCoxaInv[]`, `cFemurInv[]`, `cTibiaInv[]` per leg. HexNav firmware currently does not ship a mirrored default map.

Unit tests also assume `sign = +1` everywhere, so this bug is invisible in native parity tests.

---

### P1 — Home pose / tip frame are inconsistent with physical ground contact

**Where**

- gait home XY hardcoded in `gait_engine.cpp` as URDF tibia-frame feet
- `home_radius_cmm = 12700`, `home_foot_z_cmm = -4455`
- body height default 40 mm

These values are consistent **with each other and with the tibia-frame URDF endpoint**. They are not consistent with a toe-extended leg.

If `L_TIBIA` is corrected without also moving home tip XYZ / rest angles / body-height envelope, IK rest offsets become wrong and the robot will stand at the wrong height or immediately saturate.

Correct package of constants (must change together):

1. `links.tibia_cmm` (true tip)
2. home foot XYZ in body frame
3. `geometry.home_radius_cmm` / `home_foot_z_cmm`
4. body-height min/neutral/max
5. femur/tibia rest angles derived from the new home tip
6. stride/step defaults sized for the new workspace

---

### P1 — Planar IK drops real coxa-frame offsets

**Where**

- `leg_ik.cpp` assumes pure planar model:
  - `planar_r = hypot(x,y) - L1`
  - `dz = z`
- URDF femur axis is at **(52.0, −21.0, −21.0) mm** in coxa frame, not `(56.08, 0, 0)`

HexNav docs say the simplified model is “accurate to a few mm” for gait, and exact chain FK/IK is needed for high precision. That is acceptable later.

But today this approximation is being applied on top of an already wrong tibia length. After tibia/home/sign fixes, residual foot error from the −21 mm lateral/vertical offsets will still remain and should be either:

- folded into a calibrated effective model, or
- solved with the exact URDF transform chain.

This is secondary to P0, but it is a real model mismatch versus the CAD.

---

### P1 — Out-of-reach targets can still become joint commands

**Where**

- walking path uses `solveBody()` after path scaling, not a final hard clamp on every tick
- `LegIk::solveRaw()` still returns angles when `reachable == false` by clamping `cos_k` into `[-1, 1]`

If any leg still leaves the annulus (pose mode, body shift, calibration error, stale config), the solver saturates to a straight/folded knee and keeps commanding that pose. That produces sudden knee snaps rather than a held last-good foot.

Mark III raises `IKSolution` / warning / error flags and is much more conservative about impossible feet.

---

### P2 — Gait tables are Mark III-shaped, and mostly remapped, but not HexNav-validated

Firmware tripod/ripple/wave keyframe machinery is intentionally Phoenix-like. Leg-order remap from Mark III `(RR,RM,RF,LR,LM,LF)` into HexNav `(1..6 = LR,RR,RM,RF,LF,LM)` appears intentional:

| Gait | Mark III origins (RR..LF) | Firmware origins (leg1..6) | Remap status |
| --- | --- | --- | --- |
| Tripod-8 | `{1,5,1,5,1,5}` | `{5,1,5,1,5,1}` | consistent remap |
| Ripple-12 | `{7,11,3,1,5,9}` | `{1,7,11,3,9,5}` | consistent remap |
| Wave-24 | `{13,17,21,1,5,9}` | `{1,13,17,21,9,5}` | consistent remap |

Firmware tripod groups become:

- group A: legs **1,3,5** (rear-left, mid-right, front-left)
- group B: legs **2,4,6** (rear-right, front-right, mid-left)

That is a stable alternating triangle. HexNav’s own doc suggests a different valid pairing `{1,3,4}` / `{2,5,6}`. Difference here is **not** the main bug.

However:

- firmware never validated these timings against HexNav mass, stance width, or toe clearance
- crawl is aliased to wave
- duty factor from config is ignored (`duty_x255` is a no-op for pattern safety)
- step period is 20–80 ms per keyframe, Phoenix-like, which is aggressive once real tibia length restores large joint throw

After geometry is fixed, gait amplitude/timing will need re-tuning for HexNav, but the keyframe engine itself is a reasonable Mark III port.

---

### P2 — Body-frame conventions are mostly handled, but easy to re-break

Current controller comments and code:

- operator command frame: **x forward, y left**
- mechanical/IK frame B: **x right, y forward, z up**
- `commandPlanarToBody()` rotates twist/pose before the gait engine

Gait engine then does:

```text
x = home_x * cos(yaw) - home_y * sin(yaw) + vx * stride * longitudinal
y = home_x * sin(yaw) + home_y * cos(yaw) + vy * stride * longitudinal
z = -body_height + step * lift_scale * lift
```

That is the right structure for frame B.

Mark III uses a different axis convention entirely (classic Phoenix: X/Z ground plane, Y up). Directly copying Mark III formulas without the frame remap would destroy locomotion. The current port did remap frames; do not “simplify” this back to raw Phoenix axes.

---

### P2 — Tests currently protect the broken model

Examples:

- `test_mark_iii_gait_parity` hard-codes `kTibiaMm = 24.86`, home radius 127, all-positive servo math
- config schema tests expect tibia 2486 and sign `+1`
- gait engine tests assert tiny bounded motion under the short-tibia envelope

So CI can be green while the physical robot cannot walk. Any geometry fix must update tests to the **toe-referenced HexNav model**, not to Mark III lengths and not to the phantom tibia-frame tip.

---

## 6. Side-by-side geometry comparison

| Item | Mark III reference | HexNav URDF truth | Current firmware |
| --- | ---: | ---: | ---: |
| Coxa length | 52 mm | 56.08 mm radial (plus −21/−21 offsets) | 56.08 mm radial only |
| Femur length | 66 mm | 66.51 mm | 66.51 mm |
| Tibia length | **133 mm to tip** | 24.86 mm to tibia frame; tip longer | **24.86 mm** |
| Init/home radial | 147 mm | 127 mm to tibia frame; tip TBD | 127 mm |
| Init/home height | 25 mm down | 40 mm body / −44.55 mm coxa-frame | 40 / −44.55 |
| Body size | ~120×100 mm class coxa span | ~252.9 × 152.9 mm chassis | URDF mounts loaded |
| Servo centre | horn-offset degrees | MX-28 2048 = 180° = URDF 0 | 2048 centre |
| Side inversions | per-leg inv tables | left/right mirror required | **all +1** |
| Gait engine | Phoenix APG tables | n/a | Phoenix-style port |
| Foot target meaning | real contact tip | should be real contact tip | phantom tibia-frame point |

---

## 7. Why the failure looks like “random legs”

Operators often describe the bug as random motion. Mechanically it is more specific:

1. **Tiny shuffled stance travel** from path-scale collapse (~10 mm).
2. **Large wrong swing arcs** because joint angles were computed for a near-knee phantom tip.
3. **Left/right fight** from missing mirror signs.
4. Corner legs and mid legs have different body radii, so the same broken tip model produces **different error magnitudes per leg**. That looks chaotic even though the code is deterministic.

It is not a scheduler race and not primarily a DXL write bug. The Cartesian motion chain is deterministic and wrong.

---

## 8. What is already good

Keep these pieces; they are not the root cause:

- FreeRTOS split: control generates goals, DXL owns the bus
- Operator-frame → body-frame conversion in `ControllerCore`
- Body pose filter and gait parameter lag
- Mark III keyframe interpolation idea
- COBS/CRC protocol, safety state machine, torque seed/ramp
- Using config for mounts instead of hard-coding Mark III body offsets
- Hard-coded HexNav home XY values matching the URDF tibia-frame feet

The architecture can host a correct HexNav walker. The kinematic constants and servo sign map cannot.

---

## 9. Recommended fix order (documentation only)

Do these as one coherent geometry correction, not as isolated tweaks:

1. **Measure true toe tip**
   - From mesh and/or hardware: knee/tibia axis → ground contact point at URDF-zero pose.
   - Record coxa-frame tip `(x, y, z)` for one leg; confirm all six match by symmetry.

2. **Replace link + home constants together**
   - `L_TIBIA` = measured tip length
   - home feet in B from tip FK
   - `home_radius` / `home_foot_z`
   - body-height envelope from real standing clearance
   - rest angles from tip home

3. **Install mirrored default servo signs**
   - start from HexNav doc left `+1`, right `−1`
   - verify each joint with a +10° URDF command before walking

4. **Recompute workspace and gait defaults**
   - choose stride/step that stay inside ~90% of true annulus
   - keep Phoenix keyframes, but retune amplitude/timing for HexNav

5. **Only then compare qualitative walk to Mark III**
   - same gait ideas
   - different dimensions
   - no more copying Mark III’s 52/66/133 or 147/25 home

6. **Update native tests** so they fail if phantom tibia-frame geometry returns

Optional later:

- exact URDF chain IK instead of pure planar reduction
- contact-based touchdown adaptation
- per-leg trim from calibration UI

---

## 10. Concrete incorrect constants in live firmware

From `applyRobotMotionProfile()` in `config_schema.cpp`:

```text
links.coxa_cmm        = 5608      # 56.08 mm   OK as radial reduced model
links.femur_cmm       = 6651      # 66.51 mm   OK
links.tibia_cmm       = 2486      # 24.86 mm   WRONG for contact tip
geometry.home_radius  = 127.00 mm           WRONG once tip is corrected
geometry.home_foot_z  = -44.55 mm           WRONG once tip is corrected
geometry.coxa_lift    = 21.00 mm            OK vs URDF coxa joint z
gait.body_height_mm   = 40                  tied to phantom home
gait.stride_len_mm    = 60                  too large for current workspace
gait.step_height_mm   = 30                  meaningful only after tip fix
servo.sign            = +1 for all 18       WRONG default for right legs
```

Mount seeds are essentially correct vs URDF body-centred mounts.

Gait home XY table in `gait_engine.cpp` matches URDF tibia-frame feet, not rubber tips.

---

## 11. Minimal acceptance checks after a future fix

A corrected implementation should satisfy all of the following on the bench:

1. All-zero command holds the URDF/RViz home pose.
2. A pure body-Z crouch moves body height with feet planted; no yaw twist.
3. A pure forward walk:
   - opposite tripod/ripple phases lift cleanly
   - stance feet push backward straight in body frame
   - robot translates forward without continuous spin
4. Left and right mirror pairs are opposite in joint space for the same Cartesian foot motion.
5. Commanded stride of tens of mm is visible in odometry / floor marks, not ~1 cm shuffles.
6. FK of commanded joint angles lands within a few mm of the commanded tip targets in RViz or a host simulator using the same URDF + tip offset.

If check 3 fails but 1–2 pass, inspect gait tables/signs.  
If 1–2 already fail, geometry/sign/rest angles are still wrong.

---

## 12. Bottom line

The firmware tries to walk HexNav using:

- Mark III gait ideas ✅
- HexNav mount layout mostly ✅
- HexNav **tibia-frame** length instead of **toe** length ❌
- no left/right servo mirror map ❌
- reach limiting that hides the workspace bug by shrinking steps to ~10 mm ❌
- tests that freeze the broken constants into “correct” behaviour ❌

Mark III should remain the movement **algorithm reference**.  
`robot_ros_simulation/HexNav_description/urdf` plus a measured toe offset must remain the **dimension reference**.

Until those two roles stop being mixed, the robot will keep taking tiny, non-straight, uncoordinated steps.
