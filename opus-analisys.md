# HexNav locomotion analysis — why `firmware/openrb150` does not walk

**Date:** 2026-08-04
**Scope:** `firmware/openrb150/src/gait/**`, `firmware/openrb150/src/dxl/servo_map.*`,
`firmware/openrb150/src/config/config_schema.*`, `firmware/openrb150/src/controller/**`
**References:** `robot_ros_simulation/HexNav_description/urdf/HexNav.urdf` (authoritative geometry),
`robot_ros_simulation/HexNav_description/docs/inverse_kinematics.md`,
`temp_example/Mark_III_CM9-main/Phantom_Phoenix/**` (gait behaviour reference only)

---

## 0. Executive summary

The **gait sequencer is correct**. It is a faithful port of the Mark III / Phoenix
APG tables — leg phase offsets, keyframe shape, stance travel divisors and duty
factors all match the reference exactly (verified in §5).

The **kinematic model is wrong**, and everything downstream of it is therefore wrong.

Three independent, compounding defects:

| # | Defect | Effect |
|---|---|---|
| **F1** | `L_TIBIA = 24.86 mm` instead of the real **118.33 mm** | The firmware models a leg **4.8× shorter** than the physical leg. Commanded 60 mm of stance push produces **107 mm sideways + 45 mm backwards + 32 mm vertical** at the real toe on four of the six legs. |
| **F2** | `L_COXA = 56.08 mm` with **no femur-axis vertical offset** | The femur pivot is modelled 21 mm above where it actually is, adding a second systematic distortion. |
| **F3** | IK picks the **`+acos` knee branch**; HexNav's URDF rest pose is the **`−acos`** branch (knee angle at home = **−69.64°**) | Femur and tibia are commanded on the **mirrored** solution. Not fixable with a servo sign flip. |

Because of F1/F2 the modelled leg has a two-link reach of only 91.4 mm and the
home stance sits at **83.7 mm — 92 % of full extension, 3.1 mm from the clamp
boundary**. The reachability limiter in
[gait_pipeline.cpp](firmware/openrb150/src/gait/gait_pipeline.cpp#L124-L153)
therefore scales the whole stride down by a global factor:

```
command            path_scale    effective stride (60 mm commanded)
full forward         0.164            9.8 mm      <-- "tiny steps"
half forward         0.332           19.9 mm
full strafe          0.117            7.0 mm
full yaw             0.426           25.6 mm
forward + yaw        0.148            8.9 mm
```

So: **tiny steps** = F1/F2 collapsing the workspace and the limiter throttling the
stride. **Random-looking leg motion** = F1/F2/F3 making each leg's real toe path
diverge from the commanded path by a *different* amount depending on that leg's
mounting azimuth, so the six legs fight each other instead of translating the
body.

A fourth family of defects (F4–F10) covers the derived constants that were all
sized against the broken model (ride height 31–45 mm, foot-Z floor −120 mm, a
fixed ±16° yaw stroke, etc.). Fixing F1–F3 without also fixing those will not
produce a working robot.

---

## 1. Method

Everything below is derived from the URDF by forward kinematics, not estimated:

1. The full URDF chain `mx28_coxa → coxa_joint → femur_joint → femur → tibia_joint → tibia`
   was composed symbolically and evaluated numerically.
2. The physical toe was located by parsing `meshes/HexNav/leg_1_tibia.dae`
   (7383 vertices) and taking the vertex farthest from the tibia rotation axis.
   Result: toe at `(-73.7, -80.2, ~0)` mm in the `leg_n_tibia` link frame.
3. The leg was verified to be **exactly planar**: with `q_coxa = 0`, the toe's
   lateral coordinate is `0.00 mm` for every `(q_femur, q_tibia)` combination
   tested. A 2-link planar model is therefore *exact* for this robot — no
   approximation error, unlike what `inverse_kinematics.md` §8 warns about.
4. The true planar parameters were recovered by circle-fitting the toe locus.
5. The corrected model was round-tripped (target → IK → true URDF FK) over a
   100 × 100 mm workspace patch: **worst-case error 0.005 mm**.

---

## 2. Ground truth vs. what the firmware believes

### 2.1 Leg link parameters (coxa-joint frame: `r` = radial from hip-yaw axis, `z` up)

| Quantity | URDF ground truth | Firmware ([config_schema.cpp](firmware/openrb150/src/config/config_schema.cpp#L79-L85)) | Verdict |
|---|---:|---:|---|
| Hip-yaw axis → femur axis, radial | **52.00 mm** | `coxa_cmm = 5608` → 56.08 mm | wrong |
| Hip-yaw axis → femur axis, vertical | **−21.00 mm** | *not modelled at all* | missing |
| Femur axis → tibia axis (`L2`) | **66.51 mm** | `femur_cmm = 6651` → 66.51 mm | correct |
| Tibia axis → **toe** (`L3`) | **118.33 mm** | `tibia_cmm = 2486` → 24.86 mm | **wrong, 4.8×** |
| Two-link reach `L2+L3` | **184.84 mm** | 91.37 mm | — |
| Folded limit `|L2−L3|` | **51.82 mm** | 41.65 mm | — |

The 24.86 mm value is the distance to the **`leg_n_tibia` link frame origin**,
which is the mesh mounting point — not the foot. `inverse_kinematics.md` §4
flags this explicitly:

> † `L_TIBIA` is the distance to the `leg_n_tibia` link frame origin, which is
> the kinematic end of the URDF — **not necessarily the rubber foot tip**. […]
> **This is the one number to calibrate on hardware.**

That calibration was never applied. The firmware copied the placeholder.

Note also that `56.08 = hypot(52, 21)`: the 3-D distance was used where the
**radial** component (52 mm) belongs, and the 21 mm that should have become a
*vertical* femur-axis offset was silently folded into the radius. The
`tibia_cmm = 2486 = hypot(13.3, 21)` value has the same defect.

### 2.2 Zero-servo pose (all 18 MX-28 at 180° / tick 2048)

| | model "foot" (tibia link origin) | **real toe** |
|---|---:|---:|
| radius from hip-yaw axis | 126.99 mm | **131.89 mm** |
| height below coxa joint | −44.55 mm | **−153.36 mm** |
| leg-3 position in body frame B | (196.8, 0.0, **−40.0**) | (201.7, 0.0, **−148.9**) |

**The firmware's entire kinematic model tracks a virtual point 109 mm above the
actual foot.** `home_foot_z_cmm = -4455`
([config_schema.cpp:84](firmware/openrb150/src/config/config_schema.cpp#L84)) and
`kHomeFootZMm = -44.55f`
([leg_ik.h:30](firmware/openrb150/src/gait/leg_ik.h#L30)) encode that virtual
point. So does the 40 mm default ride height
([config_schema.cpp:106](firmware/openrb150/src/config/config_schema.cpp#L106)) —
the robot physically stands at ~149 mm.

### 2.3 Body layout — this part is correct

`kLegSeeds` ([config_schema.cpp:69-77](firmware/openrb150/src/config/config_schema.cpp#L69))
and `kHomeFootXy` ([gait_engine.cpp:12](firmware/openrb150/src/gait/gait_engine.cpp#L12))
both match the URDF exactly, and the body→coxa transform in
[body_ik.cpp](firmware/openrb150/src/gait/body_ik.cpp#L37-L50) maps every home
foot to `(r = 127.00, y = 0.00)` in its coxa frame. The `-(mount_yaw + 90°)`
rotation is right (`coxa_joint` adds a `+π/2` yaw, so the coxa-frame radial axis
is at `mount_yaw + 90°` in B).

> **Documentation hazard:** `inverse_kinematics.md` §2 claims REP-103
> (X forward, Y left), but its §3 tables and diagram — and the URDF itself —
> use **X right, Y forward**. The firmware consistently uses X-right/Y-forward,
> which matches the URDF. The doc's §2 statement is wrong; do not "fix" the
> firmware to match it.

---

## 3. Findings

### F1 — `tibia_cmm` is the mesh-mount offset, not the leg length (**critical**)

**Where:** [config_schema.cpp:81](firmware/openrb150/src/config/config_schema.cpp#L81)
`cfg.links.tibia_cmm = 2486;`

**Should be:** `11833` (118.33 mm)

**Consequence — measured.** Command: pure forward, 60 mm stride, tripod stance
phase (foot travels from `+30 mm` to `−30 mm` in body Y). Firmware IK is run,
then the resulting joint angles are pushed through the *true* URDF forward
kinematics to find where the real toe actually goes:

| leg | commanded toe delta (mm) | **actual** toe delta (mm) |
|---|---|---|
| 1 rear-left | `dx 0.00  dy −60.00  dz 0.00` | `dx +107.39  dy +44.70  dz +31.57` |
| 2 rear-right | `dx 0.00  dy −60.00  dz 0.00` | `dx −107.39  dy +44.70  dz +31.57` |
| 3 mid-right | `dx 0.00  dy −60.00  dz 0.00` | `dx 0.00  dy −53.64  dz 0.00` |
| 4 front-right | `dx 0.00  dy −60.00  dz 0.00` | `dx +107.39  dy +44.70  dz −31.57` |
| 5 front-left | `dx 0.00  dy −60.00  dz 0.00` | `dx −107.39  dy +44.70  dz −31.57` |
| 6 mid-left | `dx 0.00  dy −60.00  dz 0.00` | `dx 0.00  dy −53.64  dz 0.00` |

For a body to translate, **every grounded foot must move by the same vector**.
Here legs 3/6 push backwards (roughly right), legs 1/2/4/5 push *forwards* and
sideways, and legs 1/2 rise 31 mm while 4/5 dig in 31 mm.

This is exactly the reported symptom: the robot twitches, scrubs, and the legs
look like they are moving randomly.

Legs 3 and 6 behave least badly because their radial axis is parallel to the
body X axis, so a forward command is almost pure hip yaw and never touches the
mis-modelled femur/tibia pair. Corner legs split the command ~50/50 between hip
yaw and the planar arm, so they take the full error.

### F2 — `coxa_cmm` mixes radial and vertical; the femur-axis Z offset is missing (**critical**)

**Where:** [config_schema.cpp:79](firmware/openrb150/src/config/config_schema.cpp#L79)
and [leg_ik.cpp:37-38](firmware/openrb150/src/gait/leg_ik.cpp#L37)

```cpp
const float planar_r = horiz - l1_;   // l1_ = 56.08
const float dz = z_mm;                // <-- femur axis assumed at z = 0
```

The femur rotation axis is at `(r = 52.00, z = −21.00)` in the coxa-joint frame,
not `(56.08, 0)`. The correct reduction is:

```cpp
const float planar_r = horiz - kL1;          // kL1 = 52.00
const float dz       = z_mm - kFemurAxisZ;   // kFemurAxisZ = -21.00
```

`LegIk` has no field for the vertical offset — it must be added (or `BodyGeometry`
must carry it, since it belongs to the calibrated model, not a compile-time
constant).

### F3 — wrong IK elbow branch (**critical**)

**Where:** [leg_ik.cpp:51](firmware/openrb150/src/gait/leg_ik.cpp#L51)

```cpp
const float beta = acosf(cos_k);  // knee interior angle (knee-up branch)
```

Measured from the URDF at the zero pose:

* femur vector (femur axis → tibia axis) = `(64.80, −15.00)` → `α = −13.03°`
* tibia vector (tibia axis → toe) = `(15.09, −117.36)` → `−82.67°`
* therefore **`β = −69.64°`**

HexNav's rest pose is on the **negative** branch. The firmware selects `+acos`.

`solve()` subtracts `tibia_rest_`, so the *home* pose still maps to 0 and the
servos still centre correctly — which is why this bug hides until the robot
moves. Off-home the leg tracks the mirrored solution:

| target offset from home (r, z) | real toe error `dr` | real toe error `dz` |
|---|---:|---:|
| `(−30, −30)` | −12.5 mm | +32.7 mm |
| `(  0, −30)` | −40.7 mm | +60.9 mm |
| `(+30, −30)` | −83.5 mm | +86.2 mm |
| `(+30,   0)` | −28.6 mm | +29.5 mm |
| `(  0,   0)` |   0.0 mm |   0.0 mm |

**This is not fixable by flipping servo signs.** For the two IK branches:
`β_B = −β_A` exactly, but `α_B − α_B,rest = (a − a_rest) + (b − b_rest)` whereas
`α_A − α_A,rest = (a − a_rest) − (b − b_rest)`. The femur command is *not* a
negation. The branch itself has to change:

```cpp
const float beta = -acosf(cos_k);   // HexNav knee-out / negative branch
```

with `forwardRaw()` unchanged (it already handles either sign correctly).

### F4 — the reachability limiter destroys the stride (**critical, consequence of F1/F2**)

**Where:** [gait_pipeline.cpp:109-153](firmware/openrb150/src/gait/gait_pipeline.cpp#L109),
[leg_ik.h:36](firmware/openrb150/src/gait/leg_ik.h#L36) `kReachMarginFrac = 0.95f`

With the broken links the safe annulus is `d ∈ [46.2, 86.8] mm` — only 40.6 mm
wide — and the home stance already sits at `d = 83.72 mm`, i.e. **3.08 mm from
the outer clamp**. Reproducing `bodyTargetPathScale()` exactly:

```
full forward  -> path_scale 0.164   (60 mm commanded  ->   9.8 mm delivered)
full strafe   -> path_scale 0.117   (60 mm commanded  ->   7.0 mm delivered)
full yaw      -> path_scale 0.426
fwd + yaw     -> path_scale 0.148
```

With the corrected geometry the annulus is `d ∈ [61.1, 175.6] mm` (114 mm wide)
and there is real head-room:

| foot radius from hip | ride height | `d` at home | outward margin | max symmetric stride |
|---:|---:|---:|---:|---:|
| 130 mm | 110 mm | 121.8 | 53.8 mm | **140 mm** |
| 140 mm | 120 mm | 135.9 | 39.7 mm | **106 mm** |
| 150 mm | 120 mm | 142.5 | 33.1 mm | **86 mm** |
| 160 mm | 130 mm | 156.7 | 18.9 mm | 50 mm |

Secondary problems in the limiter itself, which remain even after F1–F3 are fixed:

* **`path_scale` is a single global minimum** across all six legs and all five
  envelope samples, applied to every leg. One leg near its boundary throttles
  the whole machine. This is the correct choice for *coordination* but it means
  a marginal yaw command silently kills forward travel.
* **It is recomputed every 10 ms from the instantaneous twist** and applied to
  the current targets — including feet that are **already planted**. Any stick
  movement instantly translates the stance feet, which is a direct
  ground-scrubbing mechanism.
* [gait_pipeline.cpp:130-134](firmware/openrb150/src/gait/gait_pipeline.cpp#L130)
  passes `target_z` as the anchor's Z:
  `bodyTargetPathScale(leg, anchor_x, anchor_y, target_z, target_x, target_y, target_z)`.
  If the home XY at the *lifted* Z falls outside the annulus, `bodyTargetPathScale()`
  returns `0.0` and the stride collapses to zero entirely. Latent today, easy to
  trigger with a large step height.
* The non-pose path calls `solveBody()`
  ([gait_pipeline.cpp:172](firmware/openrb150/src/gait/gait_pipeline.cpp#L172)),
  **not** `solveBodyLimited()`. If `path_scale` under-corrects, `cos_k`
  saturates and the leg snaps to full extension with only an `any_unreachable`
  flag.

### F5 — body-pose mode clamps each leg independently (**high**)

**Where:** [gait_pipeline.cpp:109](firmware/openrb150/src/gait/gait_pipeline.cpp#L109) / [:159-170](firmware/openrb150/src/gait/gait_pipeline.cpp#L159)

```cpp
if (!apply_pose_) { ...global coordinated path_scale... }
...
if (apply_pose_) {
  ik = body_.solveBodyPoseLimited(leg, pose_, ...);   // per-leg clampToReach()
}
```

When a body pose is active the coordinated stroke limiter is **skipped** and each
leg is instead clamped **independently** by `LegIk::clampToReach()`, which
radially drags each foot onto the annulus boundary. Six independent radial
projections = six different foot displacements = uncoordinated legs.

`apply_pose_` is latched by any non-zero pose, which in RC Walk mode includes the
**persistent operator trim** (`body_pose.roll += rc.trim_roll` at
[controller_core.cpp:317](firmware/openrb150/src/controller/controller_core.cpp#L317)).
A single trim nudge permanently moves walking onto the uncoordinated path.

### F6 — ride-height envelope is sized for the virtual foot (**high**)

**Where:** [gait_engine.h:55-57](firmware/openrb150/src/gait/gait_engine.h#L55)

```cpp
constexpr float kRcBodyHeightMinMm     = 31.0f;
constexpr float kRcBodyHeightNeutralMm = 40.0f;
constexpr float kRcBodyHeightMaxMm     = 45.0f;
```

A 14 mm total range around 40 mm. The real robot stands at ~149 mm at servo home
and its usable range is roughly **100–160 mm**. Every RC height command is
therefore meaningless on hardware.

### F7 — foot-Z clamps block the real stance (**high**)

**Where:** [gait_engine.h:29-33](firmware/openrb150/src/gait/gait_engine.h#L29)

```cpp
constexpr float kMaxStrideMm = 80.0f;
constexpr float kMaxStepMm   = 50.0f;
constexpr float kMinFootZMm  = -120.0f;   // <-- real home foot is at -148.9
constexpr float kMaxFootZMm  = -5.0f;
constexpr float kSitFootZMm  = -8.0f;     // <-- physically unreachable
```

`kMinFootZMm = -120` clamps *above* the natural standing height. As soon as F1–F3
are fixed the robot will be unable to reach its own stance. `kSitFootZMm = -8`
asks for the toe 8 mm below the body plate, which the corrected leg cannot do.
All four constants must be re-derived from the corrected workspace.

Also note [config_schema.h](firmware/openrb150/src/config/config_schema.h#L133-L136):
`kMaxGaitBodyHeightMm = 120` will *reject* a correct 130–150 mm ride height at
config-validation time.

### F8 — yaw stroke is a fixed body angle, ignoring leg radius (**high**)

**Where:** [gait_engine.h:36](firmware/openrb150/src/gait/gait_engine.h#L36),
[gait_engine.cpp:147-152](firmware/openrb150/src/gait/gait_engine.cpp#L147)

```cpp
constexpr float kMarkIiiYawTravelRad = 0.55850536f;  // 32 degrees end-to-end
...
const float yaw = twist_.wz * kMarkIiiYawTravelRad * longitudinal;
x = home_x * cos - home_y * sin + twist_.vx * stride_mm_ * longitudinal;
y = home_x * sin + home_y * cos + twist_.vy * stride_mm_ * longitudinal;
```

±16° about the body centre, regardless of the configured stride and regardless of
how far the foot is from the centre:

| foot radius from body centre | end-to-end arc travel at ±16° |
|---:|---:|
| 196.8 mm (mid legs, current home) | 108.5 mm |
| 257.5 mm (corner legs, current home) | **142.0 mm** |
| 219.8 mm (mid legs, proposed home) | 121.2 mm |

That is roughly 2.4× the 60 mm translation stride, on the largest-radius legs, so
any yaw input immediately saturates the limiter and crushes translation (see the
`fwd + yaw → 0.148` row in F4). Mark III does not do this: its `TravelLength.y`
is a small rotation increment (`-(ps4.rightH)/4`, i.e. ≈ ±6 units) integrated in
`GaitRotY` exactly like X/Z, so yaw and translation are on comparable scales.

**Fix:** derive the yaw angle from the configured stride and the *maximum* leg
radius, e.g. `yaw_half = asin(stride_mm / (2 · R_max))`, so a yaw command
produces the same foot arc length as a translation command of the same stick
deflection.

### F9 — swing lift is decoupled from stride magnitude (**medium**)

**Where:** [gait_engine.cpp:154-158](firmware/openrb150/src/gait/gait_engine.cpp#L154)

```cpp
const float command_scale = fmaxf(fabsf(vx), fmaxf(fabsf(vy), fabsf(wz)));
const float lift_scale    = clampf(command_scale * 4.0f, 0.0f, 1.0f);
z = clampf(home_z + step_mm_ * lift_scale * clampf(lift_fraction, 0, 1), ...);
```

At 25 % stick the lift is already **100 %** while the stride is only 25 % — and
after F4's `path_scale` throttling the horizontal travel is smaller still. The
robot high-steps in place. Lift should scale with the *delivered* stride, not
with a 4× saturating ramp on raw stick.

### F10 — servo signs are all `+1` and unverified (**medium**)

**Where:** [config_schema.cpp:99](firmware/openrb150/src/config/config_schema.cpp#L99)
`servo.sign = 1;`

Kinematically this is defensible: the URDF gives all six legs **identical** local
chains (`femur_joint` rpy `(π/2, −π/2, 0)`, `tibia_joint` yaw `2.443461` on every
leg) with only `leg_n_coxa_mount` yaw differing, so a rotationally-symmetric build
does take a uniform sign. Note this **contradicts** `inverse_kinematics.md` §7,
which recommends `+1` for legs 1/5/6 and `−1` for legs 2/3/4 — that
recommendation is about *body-level* motion, not about per-leg-frame IK output,
and applying it would break the model.

Still needs one-time hardware confirmation (per §7's calibration procedure) that
the MX-28 positive tick direction matches the URDF joint axis direction. Until
that is done, sign errors are indistinguishable from F1–F3 on the bench.

Related: `min_tick = 1024 / max_tick = 3072`
([config_schema.cpp:101-102](firmware/openrb150/src/config/config_schema.cpp#L101))
is ±90°. With the corrected model the tibia excursion around its −69.64° rest is
larger than with the toy model; re-check the required travel before trusting
these clamps.

### F11 — 100 Hz IK vs 50 Hz DXL write vs 20 ms gait step (**low**)

`period_ms::kControl = 10` (100 Hz) and `period_ms::kDxl = 20` (50 Hz)
([task_config.h:63-64](firmware/openrb150/src/app/task_config.h#L63)), while
`stepPeriodMs()` ([gait_engine.cpp:169-179](firmware/openrb150/src/gait/gait_engine.cpp#L169))
goes down to `kMinStepPeriodMs = 20 ms` per Phoenix keyframe. At maximum speed
there is exactly **one Sync Write per gait keyframe** — the keyframe interpolation
is fully aliased away and the servos receive discrete jumps. Cap the minimum step
period at ≥ 60 ms, or raise the DXL write rate.

### F12 — `duty_x255` is accepted and discarded (**low**)

`setParams()` takes `duty_x255` and `GaitEngine::dutyFactor()` returns
`minimumGaitDuty(gait_)` — a hard-coded per-gait constant
([gait_engine.cpp:63-77](firmware/openrb150/src/gait/gait_engine.cpp#L63)). This
is documented as intentional (the APG tables own the support pattern), but the
API and telemetry advertise a knob that does nothing. Either reject the field or
document it as reserved.

---

## 4. What Mark III does that HexNav should copy — and what it must not

### Copy

1. **Integrated stance travel.** Phoenix's `Gait()` accumulates
   `GaitPosX -= TravelLength.x / TLDivFactor` from the previous value
   ([_Phoenix_Code.h:1398](temp_example/Mark_III_CM9-main/Phantom_Phoenix/_Phoenix_Code.h#L1398)).
   A planted foot is therefore only ever moved by a *small increment* per step.
   The firmware recomputes an absolute position from `longitudinal` every tick,
   which is equivalent in steady state but lets any parameter change (stride,
   `path_scale`, body height) teleport a planted foot — see F4.
2. **Yaw and translation on the same scale.** `TravelLength.y = -(ps4.rightH)/4`
   is a *small* rotation, integrated identically to X/Z. See F8.
3. **Rotation applied about the body centre to the full body→foot radius** — the
   firmware already does this correctly in `footAt()`.
4. **Bounded, single-source-of-truth leg lengths.** `cXXTibiaLength = 133`
   ([Hex_Cfg.h:244](temp_example/Mark_III_CM9-main/Phantom_Phoenix/Hex_Cfg.h#L244))
   — a *measured toe* length, with the comment `// MEASURE THIS!!!`. HexNav's
   real value is 118.33 mm; the leg geometries are otherwise very close
   (Mark III 52 / 66 / 133 vs HexNav 52.00 / 66.51 / 118.33), which is why the
   Mark III gait tables port cleanly once the lengths are right.

### Do **not** copy

* Mark III's `cHexInitXZ = 147`, `CHexInitY = 25` stance constants. The firmware's
  header comments still cite "Mark III home foot (147 mm radial, 25 mm down)"
  ([leg_ik.h:14-16](firmware/openrb150/src/gait/leg_ik.h#L14)) — HexNav's stance
  must come from its own URDF (131.89 mm radial, 153.36 mm down at servo home).
* Mark III's body offsets (`X_COXA 60 / Y_COXA 60 / M_COXA 100`). HexNav's hips
  are at ±65.58/±115.58 (corner) and ±69.78 (mid) — already correct in the config.

---

## 5. What is already correct (do not "fix" these)

Verified against `APG[]` in
[_Phoenix_Code.h:655-661](temp_example/Mark_III_CM9-main/Phantom_Phoenix/_Phoenix_Code.h#L655),
accounting for the leg-index permutation
(Phoenix `{RR,RM,RF,LR,LM,LF}` vs firmware `{LR,RR,RM,RF,LF,LM}`):

| gait | Mark III `GaitLegNr` | firmware `origin[]` | match |
|---|---|---|---|
| Tripod 8 | RR1 RM5 RF1 LR5 LM1 LF5 | `{5,1,5,1,5,1}` → LR5 RR1 RM5 RF1 LF5 LM1 | ✅ |
| Ripple 12 | RR7 RM11 RF3 LR1 LM5 LF9 | `{1,7,11,3,9,5}` | ✅ |
| Wave 24 | RR13 RM17 RF21 LR1 LM5 LF9 | `{1,13,17,21,9,5}` | ✅ |

Also verified correct:

* `travel_divisor` = Phoenix `TLDivFactor` (4 / 8 / 20) ✅
* Keyframe shape: `relative 0` → lift 1.0 / longi 0; `relative 1` → longi +0.5,
  half lift; `relative 2` (= `FrontDownPos`) → longi +0.5, planted;
  `relative steps−1` → longi −0.5, half lift; stance decrement `0.5 − (rel−2)/div` ✅
* Duty factors 5/8, 9/12, 21/24 = Phoenix `StepsInGait − NrLiftedPos` ✅
* Body→coxa transform `Rz(−(mount_yaw + 90°))`, `z_off = mount_z + coxa_lift` ✅
  (maps every home foot to `r = 127.00, y = 0.00`)
* `kHomeFootXy` and `kLegSeeds` match the URDF ✅
* Command-frame → body-frame axis mapping in
  [command_frame.h](firmware/openrb150/src/controller/command_frame.h#L21) ✅
* `ServoMap::angleToTick()` maths (`2048 + trim + sign·deg·11.3778`) ✅
* `LegIk::forwardRaw()` — correct for either branch ✅

---

## 6. Recommended corrected parameter set

### 6.1 Model constants

```
L1  (hip-yaw axis -> femur axis, radial)      = 52.00 mm     coxa_cmm   = 5200
Zf  (hip-yaw axis -> femur axis, vertical)    = -21.00 mm    NEW FIELD  (femur_axis_z_cmm = -2100)
L2  (femur axis -> tibia axis)                = 66.51 mm     femur_cmm  = 6651   (unchanged)
L3  (tibia axis -> toe)                       = 118.33 mm    tibia_cmm  = 11833  <-- CALIBRATE ON HARDWARE
knee branch                                   = -acos(cos_k)
```

Reduction step becomes:

```cpp
const float planar_r = hypotf(x_mm, y_mm) - l1_;
const float dz       = z_mm - femur_axis_z_;      // NEW
const float d        = hypotf(planar_r, dz);
const float beta     = -acosf(clampf(cos_k, -1, 1));
```

`femur_rest_ / tibia_rest_` are then derived automatically from the configured
home, as today — no other change to `solve()` is needed.

### 6.2 Home / stance

Two options.

**(a) Keep the URDF zero pose as home** (all servos at 2048 when standing):

```
home_radius_cmm =  13189   (131.89 mm)
home_foot_z_cmm = -15336   (-153.36 mm, coxa-joint frame)
body_height_mm  =  149
```
`d_home = 154.6 mm`, outward margin only 21 mm at 95 % — usable but tight.

**(b) Recommended: a slightly wider, lower stance with real margin**

```
home_radius_cmm = 14000    (140 mm)
home_foot_z_cmm = ~ -14550 (see note)
body_height_mm  = 120      -> d_home = 135.9 mm, outward margin 39.7 mm
stride_len_mm   = 90       (max symmetric stride at this stance is 106 mm)
step_height_mm  = 35
```

Note: `home_foot_z` is expressed in the **coxa-joint** frame, i.e.
`-(body_height_mm) - (mount_z + coxa_lift)` = `-(120) - (-16.5 + 21.0)` = `-124.5 mm`
→ `home_foot_z_cmm = -12450`. (Option (a)'s −153.36 follows the same relation:
`-(148.86) - 4.5`.)

Corresponding home feet in body frame B for `r = 140 mm`:

| leg | X (mm) | Y (mm) |
|---|---:|---:|
| 1 rear-left | −164.57 | −214.57 |
| 2 rear-right | +164.57 | −214.57 |
| 3 mid-right | +209.78 | 0.00 |
| 4 front-right | +164.57 | +214.57 |
| 5 front-left | −164.57 | +214.57 |
| 6 mid-left | −209.78 | 0.00 |

(General form: `hip_xy + r · (cos(mount_yaw + 90°), sin(mount_yaw + 90°))`.
`kHomeFootXy` in [gait_engine.cpp:12](firmware/openrb150/src/gait/gait_engine.cpp#L12)
should be **derived from `cfg.legs[] + home_radius`, not hard-coded**, so it can
never drift from the config again.)

### 6.3 Envelope constants to re-derive

| constant | current | proposed |
|---|---:|---:|
| `kMaxStrideMm` | 80 | 110 |
| `kMaxStepMm` | 50 | 45 |
| `kMinFootZMm` | −120 | −185 |
| `kMaxFootZMm` | −5 | −60 |
| `kSitFootZMm` | −8 | −55 |
| `kRcBodyHeightMinMm` | 31 | 100 |
| `kRcBodyHeightNeutralMm` | 40 | 120 |
| `kRcBodyHeightMaxMm` | 45 | 150 |
| `kMaxGaitBodyHeightMm` (config_schema.h) | 120 | 170 |
| `kMaxGaitStrideMm` (config_schema.h) | 80 | 110 |
| `kMinStepPeriodMs` | 20 | 60 |
| `kMarkIiiYawTravelRad` | 0.5585 fixed | `asin(stride / (2·R_max))`, computed |

---

## 7. Suggested fix order

1. **F1 + F2 + F3 together** in `leg_ik.*` / `config_schema.cpp`. These are one
   logical change (correct kinematic model) and fixing any one alone will make
   the robot behave *differently* wrong, not better.
2. **F7 + F6** — re-derive the Z / ride-height envelope, otherwise step 1 makes
   the robot unable to reach its own stance.
3. **F8** — make the yaw stroke stride-derived.
4. Re-measure `path_scale` (F4). It should be `1.0` for full forward/strafe at
   the recommended stance. If it is not, the stance or `stride_len_mm` is wrong.
5. **F5** — route the pose path through the same coordinated stroke limiter, or
   apply a single global scale to pose targets too.
6. **F9**, **F10**, **F11**, **F12**.

### Regression tests worth adding (all host-runnable, `pio test -e native`)

* **`ik_matches_urdf_fk`** — for a grid of targets, `solve()` → true URDF FK
  (constants baked from the chain in §1) must return the target to < 0.5 mm.
  With the corrected model the achieved error is 0.005 mm.
* **`stance_feet_translate_rigidly`** — for a pure forward command, the
  *body-frame* displacement of every grounded foot over a stance phase must be
  identical across all six legs to within 1 mm. This test fails today with a
  ~150 mm spread and is the single best guard against F1/F2/F3 regressing.
* **`full_stride_is_not_reach_limited`** — `path_scale == 1.0` for full
  forward, full strafe, and full yaw at the configured default stance.
* **`zero_angles_equal_urdf_home`** — `solve(home) == (0, 0, 0)` and
  `forwardRaw(rest) == home`.
* **`gait_tables_match_phoenix`** — lock in the (already correct) APG port.
