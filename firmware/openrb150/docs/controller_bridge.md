# Controller → Hexapod Action Mapping

This is the authoritative reference for the **default** binding layout produced by
`controller::defaultBindings()` in
[`src/input/controller_bridge.cpp`](../src/input/controller_bridge.cpp). It maps
every physical control on the ChannelPack CRSF transmitter to the high-level
hexapod action the `ControllerBridge` emits each control cycle.

- The wire format (which physical input lands in which CRSF channel) is defined
  once in [`lib/ChannelPack/README.md`](../lib/ChannelPack/README.md). This
  document layers the **robot behaviour** on top of that channel map.
- Every binding here is **remappable at runtime** by a USB host via
  `setBindings()` (oha.4). The table below is only the safe out-of-box default.
- The bridge produces *intent* only. The control task applies it through the
  same safety gate as every other command source; nothing here bypasses arming,
  E-stop, joint limits, or failsafe.

---

## 1. Proportional controls (gimbals & pots)

The two gimbals have **fixed jobs**:

- **Left gimbal = walking.** Forward/backward plus strafe, in every switch
  position. `SW_E` changes *how* it walks by selecting the gait pattern.
- **Right gimbal = turning and body motion.** `SW_F` selects which, because the
  right gimbal only has two axes.

Neither toggle can take walking away from the left gimbal, and body translation
or rotation never requires a dedicated gait: the `SW_E` gait keeps running in
all three `SW_F` positions. That is why `SW_F` position 0 is called `Yaw`, not
`Walk`.

### Gimbals per right-gimbal mode

| Gimbal axis | CRSF ch | **Yaw** mode | **TranslateBody** mode | **RotateBody** mode |
|-------------|:------:|---------------|------------------------|---------------------|
| Left X  | CH1 | Strafe left / right (`twist_vy`) | Strafe (`twist_vy`) | Strafe (`twist_vy`) |
| Left Y  | CH2 | Forward / backward (`twist_vx`) | Forward / backward (`twist_vx`) | Forward / backward (`twist_vx`) |
| Right X | CH3 | **Turn** the robot (`twist_wz`) | Body **Y** shift (`pose_y_mm`) | Body **roll** (`pose_roll`) |
| Right Y | CH4 | — | Body **X** shift (`pose_x_mm`) | Body **pitch** (`pose_pitch`) |

Turning is available only in `Yaw` mode: in the two body modes both right-hand
axes carry the pose overlay. Flick `SW_F` back to UP to steer.

`body_z` and `body_yaw` are unbound by default; body height lives on Pot 2.
A USB binding override can attach the unused pose axes to another source.

Notes:
- Gimbals are bipolar: centre = 0, with a 5% deadband to kill jitter.
- Walking outputs are normalised twist in `[-1, 1]`; the gait engine scales
  them to real body velocity.
- Centred walking gimbals hold all feet at the planted home stance and park
  gait phase. Direction and magnitude changes are slew-limited rather than
  applied as instantaneous foot-target jumps.
- Body-pose outputs are clamped to the safe envelope (`±50 mm` translation,
  `±25°` rotation — `poselim::kMaxTransMm` / `kMaxRotRad`) so a stick command can
  never exceed what `SET_BODY_POSE` accepts over USB.

### Logical RC Calibration

The persisted robot configuration carries a calibration record for each
logical analog source used by this bridge: `GimbalLX`, `GimbalLY`, `GimbalRX`,
`GimbalRY`, `Pot1`, `Pot2`, `Enc1`, and `Enc2`. This is deliberately above the
custom/Tx16S physical CRSF layouts: the two profiles decode their wire data to
the same logical source before calibration, so changing transmitter profiles
does not reinterpret a stored calibration.

Current bridge behavior applies calibrated minimum/center/maximum, reversal,
and a time-based EMA before mapping to body commands. Centered controls then
use source-level deadband/rescale and expo; a nonzero legacy binding deadband
overrides the source deadband rather than stacking with it. Values far outside
the calibrated range are rejected to neutral rather than becoming a full-speed
command. The compiled defaults reproduce the previous `[-1000, 0, 1000]`
gimbal range and `[0, 1000]` pot range, retain approximately 5% center
deadband, and apply a 60 ms gimbal / 120 ms pot EMA. Expo defaults to zero for
compatibility and can be tuned in the persisted profile.

The EMA and optional median-of-three filter modes are bridge-owned and preserve
the current filtered output when switching modes. Raw diagnostic bypass is
rejected unless the caller has established that the robot is disarmed. Gait and
control-mode three-position switches require a stable 40 ms position before
changing; physical E-stop remains immediate. Valid schema-v3/v4/v5 EEPROM
payloads migrate to the Mark III motion profile; schema-v6 payloads retain
their logical-slot calibration while schema v7 restores verified IDs 1..18.
An explicit config commit persists the migration.

### Pots & encoders (live shape parameters, all modes)

These are read in **every** mode and continuously shape the gait, normalised to
`[0, 1]`.

| Control | CRSF ch | Type | Hexapod parameter |
|---------|:------:|------|-------------------|
| POT 1 | CH5 | absolute | **Speed and response** scalar (`speed`) |
| POT 2 | CH6 | absolute | **Body height** (`body_height`) |
| NAV1 | CH11 | edge-nudged | **Step height**, **stride**, **duty** (gait-tune editor) |
| ENC 1 | CH7 | *(unbound)* | free — stride moved to the gait-tune editor |
| ENC 2 | CH8 | *(unbound)* | free — step height moved to the gait-tune editor |

Pots are absolute. `stride`, `step_height`, and `duty` come from the NAV1
gait-tune editor (§2, "5-way nav clusters") unless a host explicitly binds an
axis to them through `SET_BINDINGS`; a bound axis always wins. On the TX16S
direct profile, which has no NAV cluster, the LS/RS sliders drive the same
stride / step-height values absolutely.
POT 1 controls gait cadence and the torque-enable goal recovery slew. It no
longer controls body-twist acceleration: the central body-command shaper owns
that rate contract so an operator speed change cannot silently change braking
or reversal behavior.

### Body Command Shaping

After controller conditioning, `ControllerCore` applies the persisted
`BodyCommandLimits` profile before converting the command frame to the gait
body frame. The production path is acceleration-limited:

- Forward, lateral, and yaw twist use independent maximum command scales and
  separate acceleration/deceleration limits.
- Reversals decelerate to zero before applying acceleration in the new
  direction.
- Body-height and body-pose offsets use bounded physical rates.
- The gait engine consumes this already-shaped twist directly; it does not add
  a second twist spring-damper.

Compiled defaults use normalized command maxima of `1.0`, forward
acceleration/deceleration of `1.2` / `1.8` per second, lateral `1.0` / `1.5`,
yaw `1.5` / `2.0`, height rise/lower rates of `20` / `30 mm/s`, body translation
at `60 mm/s`, and attitude at `0.5 rad/s`. These are conservative starting
limits, not final hardware tuning values. `DIRECT_DIAGNOSTIC` exists only in
the portable shaper and is gated; normal firmware selects acceleration-limited
output.

---

## 2. Digital controls

### 2-position switches — CH9 compact mask (safety & feature levels)

Read as a **level** (not an edge). Feature switches are *requests*: the control
layer honours them only when the hardware capability is available.

The custom ESP controller scales compact CH9–CH11 physical-state values across
the valid CRSF range before sending them through ELRS. The receiver reverses
that scaling with round-to-nearest; raw low integers and densely packed fields
do not survive an ELRS RF link reliably. The OpenRB build explicitly selects
this custom profile with `HEXAPOD_FORCE_CUSTOM_CHANNELPACK`.

| Switch | Hexapod action |
|--------|----------------|
| SW_A | **ARM** request (motion allowed when set and not failsafe) |
| SW_B | **E-STOP** / kill (also forced by failsafe) |
| SW_C | Enable **foot-contact** detection feature |
| SW_D | Enable **terrain-leveling** feature |
| SW_G | ON: **gait-tune editor**; OFF: normal NAV1 pose trim |
| SW_H | Hand **motion authority to the USB host** (Mac/Jetson) |

### 3-position toggles — CH10 compact state (selectors)

Positions are UP = 0 / CENTER = 1 / DOWN = 2.

| Toggle | Hexapod action |
|--------|----------------|
| SW_E | **Gait select** — how the LEFT gimbal walks: Wave (0) / Ripple (1) / Tripod (2) |
| SW_F | **Right-gimbal select**: Yaw (0) / TranslateBody (1) / RotateBody (2) |

The two selectors are independent. The gait family is live in every SW_F
position, so the operator keeps walking with the chosen gait while the right
gimbal steers, translates, or tilts the body — body motion needs no dedicated
gait. Stand and Sit remain explicit robot-state/choreography actions rather
than gait-family choices.

### Push buttons — CH10 compact state (trick triggers, rising edge)

Buttons fire a one-shot trick on the **rising edge** (debounced with a 150 ms
refractory window). The choreography itself lives in the control-task trick
engine; the bridge only emits the trigger.

| Button | Trick |
|--------|-------|
| BTN 1 | **Stand up** |
| BTN 2 | **Sit down** |
| BTN 3 | **Wave** |
| BTN 4 | **Crouch toggle** |

### 5-way nav clusters — CH11 compact state

**NAV1 is dual-purpose.** With the gait-tune editor closed it is the persistent
operator pose trim (edge-nudged, fixed `1°` step per press, clamped to `±25°`),
biasing the robot's attitude on top of the gimbal pose:

| NAV1 direction | Hexapod action |
|----------------|----------------|
| Up | Trim **pitch up** |
| Down | Trim **pitch down** |
| Left | Trim **roll left** |
| Right | Trim **roll right** |
| Center | **Reset** all pose trim to zero |

With the editor engaged (`SW_G` ON) the same cluster edits gait shape:

| NAV1 direction | Hexapod action |
|----------------|----------------|
| Up / Down | Next / previous parameter (step height → stride → duty) |
| Left / Right | Decrease / increase by `kGaitTuneStepFrac` (5% of range) |
| Center | **Save** the applied gait shape to the 24LC32 config |

While the editor is engaged, the sticks are centred, and motion is authorised,
`GaitPipeline` runs a **single-leg preview**: leg 1 (index 0) traces the swing
arc once every 2 s from the live stride and step height while the other five
legs hold stance. The preview target goes through the same reach-margin and
servo travel clamps as a walking target, and any stick input cancels it.

Saving is a full transactional EEPROM commit of the gait block only
(`ConfigApi::saveGaitDefaults`). `ControllerCore` refuses it while the robot is
moving; `apiTask` runs the transaction and reports a rejection through the
deduplicated error journal. The save request is carried as a monotonic
sequence, so a press is never lost to a task boundary nor applied twice.

**NAV2 → additional trick triggers** (rising edge, same debounce as buttons).

| NAV2 direction | Trick |
|----------------|-------|
| Up | **Twirl** |
| Down | **Stretch** |
| Left | **Lean / look** |
| Center | **Dance loop** |
| Right | Unassigned |

---

## 3. Failsafe behaviour

- If no fresh CRSF frame arrives within `kDefaultFailsafeMs` (250 ms), or the
  receiver reports link-down, the bridge asserts a **safe hold**: disarmed,
  E-stop asserted, all motion outputs zeroed. Persistent pose trim is preserved
  but not applied while failsafed.
- The RC kill switch (`SW_B`) and failsafe **always win** over any USB host
  command, regardless of the `SW_H` authority handoff.

---

## 4. Quick channel reference

| CRSF ch | Physical input | Default function |
|--------:|----------------|------------------|
| CH1 | Gimbal Left X | Strafe (all modes) |
| CH2 | Gimbal Left Y | Forward / backward (all modes) |
| CH3 | Gimbal Right X | Yaw (Walk) / body Y (Translate) / roll (Rotate) |
| CH4 | Gimbal Right Y | Body X (Translate) / pitch (Rotate) |
| CH5 | Pot 1 | Speed scalar |
| CH6 | Pot 2 | Body height |
| CH7 | Encoder 1 | Stride length trim |
| CH8 | Encoder 2 | Step height trim |
| CH9 | 6× 2-pos switches | Arm / E-stop / feature toggles / host authority |
| CH10 | 4 buttons + 2 toggles | Mode & gait select, stand/sit/wave/crouch tricks |
| CH11 | 2× 5-way nav | Pose trim / gait tuning (NAV1), tricks + tune toggle (NAV2) |
| CH12–CH16 | reserved | unused (centered) |
