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

The two gimbals are **mode-sensitive**: what a gimbal axis does depends on the
active control mode selected by the `SW_E` toggle. This is the "move the core
without moving the legs" requirement — the same sticks either walk the robot or
pose its body with the feet planted.

### Gimbals per control mode

| Gimbal axis | CRSF ch | **Walk** mode | **TranslateBody** mode | **RotateBody** mode |
|-------------|:------:|---------------|------------------------|---------------------|
| Left X  | CH1 | Yaw rate — turn left / right (`twist_wz`) | — | Body **yaw** (`pose_yaw`) |
| Left Y  | CH2 | Forward / backward (`twist_vx`) | Body **Z** up / down (`pose_z_mm`) | — |
| Right X | CH3 | Strafe left / right (`twist_vy`) | Body **Y** shift (`pose_y_mm`) | Body **roll** (`pose_roll`) |
| Right Y | CH4 | — | Body **X** shift (`pose_x_mm`) | Body **pitch** (`pose_pitch`) |

Notes:
- Gimbals are bipolar: centre = 0, with a 5% deadband to kill jitter.
- Walk-mode outputs are normalised twist in `[-1, 1]`; the gait engine scales
  them to real body velocity.
- Body-pose outputs are clamped to the safe envelope (`±50 mm` translation,
  `±25°` rotation — `poselim::kMaxTransMm` / `kMaxRotRad`) so a stick command can
  never exceed what `SET_BODY_POSE` accepts over USB.

### Pots & encoders (live shape parameters, all modes)

These are read in **every** mode and continuously shape the gait, normalised to
`[0, 1]`.

| Control | CRSF ch | Type | Hexapod parameter |
|---------|:------:|------|-------------------|
| POT 1 | CH5 | absolute | **Speed** scalar (`speed`) |
| POT 2 | CH6 | absolute | **Body height** (`body_height`) |
| ENC 1 | CH7 | relative trim | **Stride** length (`stride`) |
| ENC 2 | CH8 | relative trim | **Step height** / foot clearance (`step_height`) |

Encoders integrate their relative delta into a 0..1 trim
(`kEncoderCountsFullScale = 1024` counts = full sweep); pots are absolute.

---

## 2. Digital controls

### 2-position switches — CH9 bitfield (safety & feature levels)

Read as a **level** (not an edge). Feature switches are *requests*: the control
layer honours them only when the hardware capability is available.

| Switch | Hexapod action |
|--------|----------------|
| SW_A | **ARM** request (motion allowed when set and not failsafe) |
| SW_B | **E-STOP** / kill (also forced by failsafe) |
| SW_C | Enable **foot-contact** detection feature |
| SW_D | Enable **terrain-leveling** feature |
| SW_G | Enable **passive-pose** streaming feature |
| SW_H | Hand **motion authority to the USB host** (Mac/Jetson) |

### 3-position toggles — CH10 bitfield (selectors)

Positions are UP = 0 / CENTER = 1 / DOWN = 2.

| Toggle | Hexapod action |
|--------|----------------|
| SW_E | **Control-mode select**: Walk (0) / TranslateBody (1) / RotateBody (2) |
| SW_F | **Gait-family select**: `gait_index` 0 / 1 / 2 |

### Push buttons — CH10 bitfield (trick triggers, rising edge)

Buttons fire a one-shot trick on the **rising edge** (debounced with a 150 ms
refractory window). The choreography itself lives in the control-task trick
engine; the bridge only emits the trigger.

| Button | Trick |
|--------|-------|
| BTN 1 | **Stand up** |
| BTN 2 | **Sit down** |
| BTN 3 | **Wave** |
| BTN 4 | **Crouch toggle** |

### 5-way nav clusters — CH11 bitfield

**NAV1 → persistent operator pose trim** (edge-nudged, fixed `1°` step per press,
clamped to `±25°`). This biases the robot's attitude on top of the gimbal pose.

| NAV1 direction | Hexapod action |
|----------------|----------------|
| Up | Trim **pitch up** |
| Down | Trim **pitch down** |
| Left | Trim **roll left** |
| Right | Trim **roll right** |
| Center | **Reset** all pose trim to zero |

**NAV2 → additional trick triggers** (rising edge, same debounce as buttons).

| NAV2 direction | Trick |
|----------------|-------|
| Up | **Twirl** |
| Down | **Stretch** |
| Left | **Lean / look** |
| Center | **Dance loop** |
| Right | *(unassigned)* |

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
| CH1 | Gimbal Left X | Yaw (Walk) / body yaw (Rotate) |
| CH2 | Gimbal Left Y | Forward (Walk) / body Z (Translate) |
| CH3 | Gimbal Right X | Strafe (Walk) / body Y (Translate) / roll (Rotate) |
| CH4 | Gimbal Right Y | Body X (Translate) / pitch (Rotate) |
| CH5 | Pot 1 | Speed scalar |
| CH6 | Pot 2 | Body height |
| CH7 | Encoder 1 | Stride length trim |
| CH8 | Encoder 2 | Step height trim |
| CH9 | 6× 2-pos switches | Arm / E-stop / feature toggles / host authority |
| CH10 | 4 buttons + 2 toggles | Mode & gait select, stand/sit/wave/crouch tricks |
| CH11 | 2× 5-way nav | Pose trim (NAV1), tricks (NAV2) |
| CH12–CH16 | reserved | unused (centered) |
