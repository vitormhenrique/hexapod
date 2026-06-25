# HexNav — Kinematics & Gait Reference for Firmware

> Authoritative source: `HexNav_description/urdf/HexNav.urdf`.
> Every number in this document was extracted directly from that URDF by forward
> kinematics, not hand-measured. Lengths are in **millimetres**, angles in
> **radians** unless a `°` is shown. SI (metres/radians) is used in all equations.

This document gives a firmware engineer everything needed to implement closed‑loop
leg control and body locomotion for the **HexNav** hexapod without opening the
CAD or the URDF: the body layout, the per‑leg kinematic chain, the segment
lengths, the servo (Dynamixel MX‑28) mapping, the forward/inverse kinematics, the
reachable workspace, and the standard hexapod gaits.

---

## 1. Platform summary

| Property | Value |
|---|---|
| Legs | 6, each **3‑DOF** (coxa / femur / tibia) |
| Actuators | 18 × Dynamixel **MX‑28** (Joint Mode, 0–360°, 4096 pulse/rev, 0.088°/pulse) |
| Servo home / center | **180°** ⇒ URDF joint zero (0 rad) |
| Total mass | 4.663 kg |
| Root frame | `base_link` |
| Body centre (CoM, in `base_link`) | (76.463, 126.463, −0.093) mm |
| Body bounding box | 152.9 × 252.9 × 3.0 mm (flat hex chassis) |

### Joint command order (ROS `position_controller`)
The ros2_control command vector and `/joint_states` use this exact order
(see `config/ros2_controllers.yaml`). Firmware should keep the same index map:

```
[ leg1_coxa, leg1_femur, leg1_tibia,
  leg2_coxa, leg2_femur, leg2_tibia,
  leg3_coxa, leg3_femur, leg3_tibia,
  leg4_coxa, leg4_femur, leg4_tibia,
  leg5_coxa, leg5_femur, leg5_tibia,
  leg6_coxa, leg6_femur, leg6_tibia ]   # 18 values
```

---

## 2. Coordinate frames

All frames follow ROS/REP‑103 (right‑handed, **X forward, Y left, Z up**),
matching the URDF.

- **`base_link`** — body frame, origin at the chassis corner used by the CAD
  export. The geometric body centre is at **(76.463, 126.463, 0) mm** in this
  frame; subtract that to get a centred body frame `B` (recommended for gait
  maths, used throughout §3 and §9).
- **Leg base frame `Hₙ`** — at leg *n*'s coxa **mount** on the body
  (`leg_n_coxa_mount`), see §3.
- **Coxa frame `Cₙ`** — rotates with the coxa (hip‑yaw) joint about **+Z**.
- The femur and tibia joints rotate about a **horizontal** axis (knee‑type
  pitch), see §6.

> ⚠️ The raw `base_link` origin is *not* the body centre. Use the centred body
> frame `B` = `base_link` − (76.463, 126.463, 0) mm for symmetric gait planning.

---

## 3. Body layout (leg mounting)

Leg coxa mounts (`base_link → leg_n_mx28_coxa`, all fixed), expressed **relative
to the body centre `B`**. `radius` is the horizontal distance from body centre to
the hip; `azimuth` is measured from +X (forward), CCW positive (toward +Y/left).

| Leg | Position in B (mm) | Radius (mm) | Azimuth | Mount Z (mm) | Side / role |
|----:|:-------------------|:-----------:|:-------:|:------------:|:------------|
| 1 | (−65.58, −115.58) | 132.88 | −119.57° | −16.5 | rear‑left  |
| 2 | (+65.58, −115.58) | 132.88 |  −60.43° | −16.5 | rear‑right |
| 3 | (+69.78,   0.00 ) |  69.78 |    0.00° | −16.5 | mid‑right  |
| 4 | (+65.58, +115.58) | 132.88 |  +60.43° | −16.5 | front‑right|
| 5 | (−65.58, +115.58) | 132.88 | +119.57° | −16.5 | front‑left |
| 6 | (−69.78,   0.00 ) |  69.78 | 180.00° | −16.5 | mid‑left   |

The hip‑yaw home azimuth of each leg (the `leg_n_coxa_mount` yaw, i.e. the
direction the coxa points at servo 180°):

| Leg | 1 | 2 | 3 | 4 | 5 | 6 |
|----:|:--:|:--:|:--:|:--:|:--:|:--:|
| Mount yaw | +135° | −135° | −90° | −45° | +45° | +90° |

### Layout diagram (top view, +Y = forward/left handedness)

```
                 FRONT (+Y)
        leg5 \                 / leg4
              \               /
   leg6 ------  BODY CENTRE B  ------ leg3      (+X = right)
              /               \
        leg1 /                 \ leg2
                  REAR (−Y)
```

This is a **symmetric hexagon**: middle legs (3, 6) stick straight out along ±X
at 69.78 mm; the four corner legs sit at ±115.58 mm fore/aft and ±65.58 mm
lateral, 132.88 mm from centre.

### Sagittal mirror pairs
The platform is mirror‑symmetric about the **fore/aft plane (X = 0 in B)**.
Mirror pairs (left ↔ right):

| Pair | Left leg | Right leg |
|---|---|---|
| Rear   | 1 | 2 |
| Middle | 6 | 3 |
| Front  | 5 | 4 |

**Left legs:** 1, 5, 6  **Right legs:** 2, 3, 4. (See §7 for the servo
inversion this implies.)

---

## 4. Per‑leg kinematic chain (exact)

Every leg is built from the **same** local transforms — only the
`leg_n_coxa_mount` (body placement) differs. The chain from body to foot, with
each transform as **xyz (mm)** and **rpy (rad)** exactly as in the URDF:

| # | Joint | Type | Parent → Child | xyz (mm) | rpy (rad) | Axis |
|--:|:------|:-----|:---------------|:---------|:----------|:-----|
| 1 | `coxa_mount`  | fixed | base_link → mx28_coxa | per‑leg, see §3 | per‑leg yaw | — |
| 2 | `coxa_joint`  | **revolute** | mx28_coxa → coxa | (0, 0, **21.0**) | (0, 0, **+π/2**) | **+Z** |
| 3 | `femur_joint` | **revolute** | coxa → mx28_femur | (**52.0**, −21.0, −21.0) | (+π/2, −π/2, 0) | **+Z**(local) |
| 4 | `femur_mount` | fixed | mx28_femur → femur | (0, −35.8, −21.0) | (0, 0, −π) | — |
| 5 | `tibia_joint` | **revolute** | femur → mx28_tibia | (15.0, 29.0, 21.0) | (0, 0, **2.443461**) | **+Z**(local) |
| 6 | `tibia_mount` | fixed | mx28_tibia → tibia | (0, −13.3, −21.0) | (±π, 0, +π/2) | — |

Notes:
- The three actuated joints are **`coxa_joint`, `femur_joint`, `tibia_joint`**.
  The `*_mx28_*` links are the servo bodies; the `*_mount` joints are fixed
  structural offsets to the printed leg segments.
- `tibia_joint` rpy yaw = **2.443461 rad = 140.0°** is the built‑in knee rest
  angle baked into the URDF zero pose.
- `leg_1` and `leg_2` differ only by a sign in `tibia_mount` roll (±π) and in the
  reported `axis` sign of femur/tibia (±Z); these are equivalent representations
  of the same physical axis — see §11 "Symmetry verification".

### Reduced (DH‑like) leg parameters
For an analytic model, each leg reduces to a hip‑yaw followed by a 2‑link planar
arm. Measured in the **coxa frame `Cₙ`** (origin on the hip‑yaw axis, after
`coxa_joint`), at all‑zero joint angles:

| Quantity | Symbol | Value |
|---|---|---|
| Hip → femur‑axis, horizontal offset | `L_COXA`  | **56.08 mm** (radial) with −21.0 mm lateral, −21.0 mm vertical |
| Femur‑axis → tibia‑axis (link length) | `L_FEMUR` | **66.51 mm** |
| Tibia‑axis → foot frame (link length) | `L_TIBIA` | **24.86 mm** † |

† `L_TIBIA` is the distance to the **`leg_n_tibia` link frame origin**, which is
the kinematic end of the URDF — **not necessarily the rubber foot tip**. If the
physical toe extends beyond this frame, measure that extra offset on the real leg
and add it along the tibia direction. **This is the one number to calibrate on
hardware.**

True 3‑D link lengths are **identical for all six legs** (verified, §11):
`L_COXA = 59.88 mm` (3‑D hip→femur), `L_FEMUR = 66.51 mm`, `L_TIBIA = 24.86 mm`.

---

## 5. Home pose & default stance

At **all joints = 0 rad (all servos = 180°)** the feet rest at (in centred body
frame `B`):

| Leg | Foot in B (mm) | Horiz. radius (mm) | Azimuth | Foot height (mm) |
|----:|:---------------|:------------------:|:-------:|:----------------:|
| 1 | (−155.4, −205.4, −40.0) | 257.5 | −127.1° | −40.0 |
| 2 | (+155.4, −205.4, −40.0) | 257.5 |  −52.9° | −40.0 |
| 3 | (+196.8,    0.0, −40.0) | 196.8 |    0.0° | −40.0 |
| 4 | (+155.4, +205.4, −40.0) | 257.5 |  +52.9° | −40.0 |
| 5 | (−155.4, +205.4, −40.0) | 257.5 | +127.1° | −40.0 |
| 6 | (−196.8,    0.0, −40.0) | 196.8 | 180.0° | −40.0 |

- Default **body ride height** at the home pose: **40.0 mm** (feet 40 mm below
  the hip mounts; mounts are 16.5 mm below `base_link`).
- In each leg's own coxa frame the home foot is at **r = 127.0 mm, z = −44.55 mm**
  (i.e. 127 mm radially out, 44.6 mm down) — identical for all legs.

---

## 6. Joint axes & sign conventions

In the body frame at home pose the joint axes are:

| Joint | Rotation axis | Effect |
|---|---|---|
| coxa  | **vertical (+Z)** | hip yaw — swings the leg fore/aft (protraction/retraction) |
| femur | **horizontal**, perpendicular to the leg's radial direction | lifts/lowers the thigh |
| tibia | **horizontal**, parallel to femur axis | flexes/extends the knee |

The femur/tibia axes point along the leg's local tangent; per leg (body frame):

| Leg | femur & tibia axis (body XY) |
|----:|:-----------------------------|
| 1 | (−0.71, +0.71) |
| 2 | (−0.71, −0.71) |
| 3 | ( 0.00, −1.00) |
| 4 | (+0.71, −0.71) |
| 5 | (+0.71, +0.71) |
| 6 | ( 0.00, +1.00) |

Positive joint angle (URDF, right‑hand rule about the listed axis):
- **+coxa** → leg swings toward +azimuth (CCW seen from above).
- **+femur** → thigh rotates (see §8 for the planar sign used in IK).
- **+tibia** → knee flexes.

---

## 7. Servo mapping (Dynamixel MX‑28)

Each URDF joint angle `q` (rad) maps to an MX‑28 goal angle `S` (deg):

```
S = 180 + sign · degrees(q)             # command (rad → servo deg)
q = sign · radians(S − 180)             # feedback (servo deg → rad)
```

- **Center:** servo **180°** = URDF zero. Joint Mode range 0–360°, so usable
  travel about home is roughly ±180° before the servo dead‑band/limits.
- **`sign` (inversion):** `+1` or `−1` per servo, set so that a positive URDF
  angle and the physical motion agree. Because the chassis is sagittally
  mirror‑symmetric, **left and right legs of a pair turn opposite directions for
  the same body motion**, so their coxa (and usually femur/tibia) signs are
  opposite.

Recommended starting inversion map (verify on hardware with the GUI `inv`
toggles in `scripts/joint_gui.py`):

| Servo | Left legs (1, 5, 6) | Right legs (2, 3, 4) |
|---|:--:|:--:|
| coxa  | `+1` | `−1` |
| femur | `+1` | `−1` |
| tibia | `+1` | `−1` |

> The GUI (`joint_gui.py`) already exposes a per‑joint **`inv`** checkbox and a
> **Center (180°)** button; use it to confirm each servo's `sign` before burning
> the map into firmware. Store the 18 signs and 18 home offsets (trim) in EEPROM
> or config.

### Calibration procedure (per servo)
1. Power the leg, set servo to **180°**, confirm the segment is at its URDF‑zero
   pose (use RViz with `/joint_states` as the reference picture).
2. Command **+10° in URDF terms** (`S = 180 + sign·10`). If the segment moves the
   wrong way, flip `sign`.
3. Record any mechanical offset between the servo's mechanical 180° and the true
   zero pose as a **trim** added to `S`.

---

## 8. Inverse kinematics (single leg, 3‑DOF)

Goal: given a desired **foot position `p = (x, y, z)` in the leg base frame `Hₙ`**
(origin at the hip, X radially outward at home, Z up), find `(q_coxa, q_femur,
q_tibia)`.

Constants (SI):
```
L1 = 0.05608   # L_COXA  horizontal hip→femur offset (m)
L2 = 0.06651   # L_FEMUR (m)
L3 = 0.02486   # L_TIBIA to URDF foot frame (m) — add measured toe offset
```

**Step 1 — Hip yaw (coxa):**
```
q_coxa = atan2(y, x)
```

**Step 2 — Reduce to the leg's vertical plane.** Horizontal distance from the
hip axis to the foot, minus the coxa offset:
```
r   = hypot(x, y) − L1
dz  = z                      # foot height relative to femur axis (offset −21 mm baked in, see note)
d   = hypot(r, dz)           # straight‑line femur‑axis → foot
```

**Step 3 — Two‑link planar IK (femur L2, tibia L3):**
```
# law of cosines
cos_k = (d² − L2² − L3²) / (2·L2·L3)
cos_k = clamp(cos_k, −1, +1)            # reject unreachable targets
q_tibia =  ±acos(cos_k)                 # knee; pick sign for "knee‑up" posture

# femur angle
a   = atan2(dz, r)
b   = atan2(L3·sin(q_tibia), L2 + L3·cos(q_tibia))
q_femur = a − b
```

**Step 4 — Apply URDF zero offsets and servo map.** Add the URDF rest offsets
(femur/tibia have non‑zero baked rest angles, notably the tibia's **140.0°**),
then convert with §7:
```
S_coxa  = 180 + sign_c · degrees(q_coxa)
S_femur = 180 + sign_f · degrees(q_femur  − FEMUR_REST)
S_tibia = 180 + sign_t · degrees(q_tibia  − TIBIA_REST)   # TIBIA_REST = 2.443461 rad
```

> **Reachability test:** the target is valid only if
> `|L2 − L3| ≤ d ≤ (L2 + L3)` **and** `−1 ≤ cos_k ≤ 1`. Clamp/saturate before
> commanding servos.

> **Offset note:** the URDF has a −21 mm lateral and −21 mm vertical offset
> between the hip axis and the femur axis (§4). For most gait work the simplified
> planar model above is accurate to a few mm. For high‑precision foot placement,
> use the **exact transform chain in §4** (multiply the homogeneous matrices and
> solve numerically, or fold the offsets into `L1`/`dz`).

### Forward kinematics (for verification / feedback)
Compose the §4 transforms in order with the measured joint angles inserted at
`coxa_joint`, `femur_joint`, `tibia_joint`:
```
T_foot = T_mount(n) · Rz(q_coxa) · T_23 · Rz(q_femur) · T_34 · Rz(q_tibia) · T_56
```
where `T_23, T_34, T_56` are the fixed parts of rows 3–6 in §4. The foot is the
translation of `T_foot`. Use this to validate IK output before driving hardware.

---

## 9. Workspace & limits

- **Leg reach (horizontal, from hip):** home at **127 mm**; max ≈ `L1+L2+L3 ≈
  146 mm` fully extended, min ≈ `L1+|L2−L3| ≈ 98 mm` fully folded (planar model;
  add the measured toe offset to `L3`).
- **Foot height envelope:** about **±60 mm** around the −40 mm home height before
  the 2‑link arm saturates.
- **Servo travel:** MX‑28 Joint Mode 0–360°; keep each joint within **±90° of the
  180° home** to stay clear of the dead‑band and self‑collision.
- **Recommended per‑joint soft limits** (tune on hardware):
  - coxa: ±35° about home (avoid adjacent‑leg collision; corner legs are 60.43°
    apart in azimuth).
  - femur: −45°…+45°.
  - tibia: −60°…+60° about its 140° rest.

> ⚠️ The URDF marks the actuated joints as **`continuous`** (no hard limits).
> Enforce the soft limits **in firmware** — the model will not stop you.

---

## 10. Gaits

A hexapod is statically stable while **≥ 3 non‑collinear feet** are grounded.
Common gaits, in order of speed vs. stability:

### 10.1 Tripod gait (fastest, 2 phases)
Two alternating tripods, each a stable triangle:
- **Tripod A:** legs **1, 4, 3**  (rear‑left, front‑right, mid‑right)
- **Tripod B:** legs **2, 5, 6**  (rear‑right, front‑left, mid‑left)

> Pick the two opposing triangles so each contains one front, one middle and one
> rear from alternating sides. With HexNav's numbering, **{1,3,4}** and
> **{2,5,6}** form two balanced tripods straddling the centre.

Cycle: while tripod A swings (lifts, moves forward, plants), tripod B is in
stance (feet down, pushing the body forward), then swap. Duty factor 0.5.

```
phase 0: A=SWING   B=STANCE
phase 1: A=STANCE  B=SWING
```

### 10.2 Ripple gait (medium)
Legs move in an overlapping sequence; **2 legs in swing at a time**, offset by
1/6 cycle. Order (one common ripple): `1 → 4 → 5 → 2 → 3 → 6` with each leg
phase‑shifted by ~0.33. Duty factor ≈ 0.67. Smoother than tripod, more stable.

### 10.3 Wave gait (slowest, most stable)
**One leg swings at a time**; five always grounded. Sequence around the body,
e.g. `1 → 2 → 3 → 4 → 5 → 6`. Duty factor ≈ 0.83. Use on rough terrain or for
heavy payloads.

### 10.4 Gait parameters (firmware knobs)
| Parameter | Typical | Notes |
|---|---|---|
| Step length | 30–60 mm | foot travel per stride in body X |
| Step height | 20–40 mm | swing‑phase lift |
| Cycle time  | 0.5–2.0 s | shorter = faster |
| Duty factor | 0.5 (tripod) … 0.83 (wave) | stance fraction |
| Stance trajectory | straight line, body‑relative | feet move opposite body |
| Swing trajectory  | semi‑ellipse / Bézier | lift → forward → plant |

### 10.5 Gait → IK loop (per control tick)
```
for each leg n:
    if leg n in STANCE:
        foot_B = home_foot[n] − body_velocity · dt        # push body forward
    else: # SWING
        foot_B = swing_curve(home_foot[n], step_len, step_h, phase)
    foot_H = transform foot_B from body frame B into leg base frame Hn   # §3
    (qc,qf,qt) = leg_IK(foot_H)                                          # §8
    command servos with §7 map
```

---

## 11. Body pose control (no stepping)

With all six feet planted you can pose the body in 6‑DOF by re‑solving IK for the
fixed foot positions while moving the **body** frame:
```
foot_H(n) = R_body⁻¹ · (foot_world(n) − body_pos) ,  expressed in Hn
```
Sweep `body_pos` (x, y, z) and `R_body` (roll, pitch, yaw) and run §8 per leg.
This gives RViz‑style body translate/tilt and is the basis for terrain
adaptation and balance.

---

## 12. Symmetry verification (extracted, not assumed)

Confirmed directly from the URDF — **all six legs are geometrically identical**;
they differ only by the body‑placement transform in §3.

| Check | Result |
|---|---|
| Coxa‑frame femur axis position | **(52.0, −21.0, −21.0) mm — identical, all 6 legs** |
| Coxa‑frame tibia axis position | **(116.8, −21.0, −36.0) mm — identical, all 6 legs** |
| Coxa‑frame foot position | **(127.0, 0.0, −44.5) mm — identical, all 6 legs** |
| `L_COXA` / `L_FEMUR` / `L_TIBIA` | **59.88 / 66.51 / 24.86 mm — identical, all 6 legs** |
| Segment masses (mx28/coxa/femur/tibia) | **0.0739 / 0.193 / 0.127 / 0.155 kg — identical, all 6 legs** |
| Leg base radius (corner / middle) | **132.88 mm / 69.78 mm — mirror‑exact** |
| Sagittal mirror pairs | **(1↔2), (6↔3), (5↔4) — positions match to ±0.00001 m** |

**Consequence for firmware:** implement **one** leg IK/FK routine and **one** set
of constants. Per leg, parameterise only: (a) the body mount transform from §3,
and (b) the servo `sign`/trim from §7. Nothing else changes between legs.

---

## 13. Quick‑reference constants

```c
// ---- Leg geometry (same for all 6 legs), millimetres ----
#define L_COXA    56.08f   // horizontal hip -> femur-axis offset
#define L_FEMUR   66.51f   // femur-axis -> tibia-axis
#define L_TIBIA   24.86f   // tibia-axis -> URDF foot frame (ADD measured toe!)
#define COXA_Z_OFF  21.0f  // mx28_coxa -> coxa lift
#define LATERAL_OFF 21.0f  // femur-axis lateral offset in coxa frame
#define TIBIA_REST_DEG 140.0f  // baked knee rest (2.443461 rad)

// ---- Servo (MX-28) ----
#define SERVO_CENTER_DEG 180.0f
#define SERVO_MIN_DEG      0.0f
#define SERVO_MAX_DEG    360.0f
#define SERVO_DEG_PER_PULSE 0.087890625f   // 360 / 4096

// ---- Body, centred frame B (mm) ----
// {x, y, mount_yaw_deg} per leg, Z mount = -16.5 mm
const float LEG_BASE[6][3] = {
  {-65.58f, -115.58f, +135.0f},  // leg1 rear-left
  {+65.58f, -115.58f, -135.0f},  // leg2 rear-right
  {+69.78f,    0.00f,  -90.0f},  // leg3 mid-right
  {+65.58f, +115.58f,  -45.0f},  // leg4 front-right
  {-65.58f, +115.58f,  +45.0f},  // leg5 front-left
  {-69.78f,    0.00f, +180.0f},  // leg6 mid-left
};

// ---- Home stance, foot in B (mm) ----
const float HOME_FOOT[6][3] = {
  {-155.4f, -205.4f, -40.0f},
  {+155.4f, -205.4f, -40.0f},
  {+196.8f,    0.0f, -40.0f},
  {+155.4f, +205.4f, -40.0f},
  {-155.4f, +205.4f, -40.0f},
  {-196.8f,    0.0f, -40.0f},
};
```

---

## 14. Open calibration items (must verify on hardware)

1. **Foot tip offset** beyond the `leg_n_tibia` frame (`L_TIBIA` is to the URDF
   frame, not the rubber toe) — §4.
2. **18 servo `sign` values** and **18 home trims** — §7. Use the
   `scripts/joint_gui.py` `inv` toggles + Center (90°) to confirm.
3. **Per‑joint soft limits** — §9 (URDF joints are `continuous`).
4. **Knee posture choice** (`±acos` branch in §8) — pick knee‑up consistently.

---

*Generated from `HexNav_description/urdf/HexNav.urdf`. If the URDF geometry
changes, re‑extract: all tables here come from forward kinematics on that file.*
