# Controller Bridge — Mapping, Modes, Tricks, and USB API

Status: **design + portable module landed** (epic `oha`). Firmware wiring
(`oha.3`), USB `CONTROLLER_*` API (`oha.4`), and the trick/choreography engine
(`oha.5`) are tracked as follow-ups.

This document defines how the hand controller drives the hexapod so it has the
feature set of common open-source hexapods: walking, changing gaits, moving the
core/body without stepping the legs, live shape tuning, feature toggles, an
emergency kill, and scripted "tricks" — all also reachable over USB.

---

## 1. Hardware link

The controller is an **ESP32-S3 TX** that packs every physical input into a
**16-channel CRSF frame** (11-bit channels, 50 Hz, 420000 baud 8N1) and the
**OpenRB-150 is the RX**. The shared wire layout lives in the vendored
`firmware/openrb150/lib/ChannelPack/ChannelPack.h` (single source of truth for
both ends) and `lib/AlfredoCRSF/`.

Decode path on the robot:

```
Serial3 (D14 TX / D13 RX) bytes ──► crsf::Parser ──► raw 11-bit ticks ch[16]
                                        │
                                        ▼
                           controller::ControllerBridge.update(ch, link_up, now_ms)
                                        │
                                        ▼
                              controller::ControllerCommand  (decoded intent)
```

- `crsf::Parser` already exists and produces `ChannelData.channels[16]` raw
  ticks (ChannelPack range 191..1792).
- `controller::ControllerBridge` (this feature) turns those ticks into a
  high-level `ControllerCommand`. It depends **only** on the vendored
  `ChannelPack.h` and is fully host-testable (40 native tests in
  `test/test_controller_bridge/`).

### Physical inputs available (from `ChannelPackInputs_t`)

| Group | Inputs | Wire range |
| --- | --- | --- |
| Gimbals | LX, LY, RX, RY (2 sticks) | -1000..+1000 |
| Pots | POT1, POT2 | 0..1000 |
| Encoders | ENC1, ENC2 (continuous, wrap at 11-bit) | raw counts |
| 2-pos switches | SW_A, SW_B, SW_C, SW_D, SW_G, SW_H | bool |
| 3-pos toggles | SW_E, SW_F | 0=UP / 1=CENTER / 2=DOWN |
| Buttons | BTN_1..BTN_4 | bool |
| Nav clusters | NAV1, NAV2 each U/D/L/R/Center | bool |

---

## 2. What each gimbal does

The two gimbals have **fixed jobs**:

- **Left gimbal = walking.** LY walks forward/back, LX strafes. This is true in
  every switch position — nothing can take walking away from the left stick.
- **Right gimbal = turning and body motion.** `SW_F` picks which of the two it
  does, because the right gimbal only has two axes.

The two toggles are therefore fully independent:

| Toggle | What it changes | Positions |
| --- | --- | --- |
| `SW_E` | How the **left** gimbal walks — the gait pattern | Wave (0) / Ripple (1) / Tripod (2) |
| `SW_F` | What the **right** gimbal does | Yaw (0) / TranslateBody (1) / RotateBody (2) |

| `SW_F` | `ControlMode` | Left gimbal (always walking) | Right gimbal |
| --- | --- | --- | --- |
| UP | `Yaw` | LY = forward/back, LX = strafe | RX = turn the robot left/right |
| CENTER | `TranslateBody` | LY = forward/back, LX = strafe | RY = body X, RX = body Y |
| DOWN | `RotateBody` | LY = forward/back, LX = strafe | RX = body roll, RY = body pitch |

Key points:

- **Body translation and rotation do not need a special gait.** The gait chosen
  by `SW_E` keeps running in all three `SW_F` positions, so the robot walks
  normally while the body is shifted or tilted.
- `SW_F` never decides *whether* the robot walks — that is why position 0 is
  named `Yaw`, not `Walk`.
- Turning is only available in `SW_F` = `Yaw`: in the body modes both right-hand
  axes are taken by the pose overlay. Flick `SW_F` back to UP to steer.
- Translate limits: ±`poselim::kMaxTransMm` = **±50 mm** on each axis.
- Rotate limits: ±`poselim::kMaxRotRad` = **±0.4363 rad (±25°)** on each axis.
- These mirror `protocol::motionlim`, so a controller body pose can never exceed
  what `SET_BODY_POSE` allows over USB.
- With the left stick centred the gait holds the planted home stance, so the
  body modes still "move the core without moving the legs".

---

## 3. Gait selection and live shape tuning

| Input | Action | Range |
| --- | --- | --- |
| `SW_E` (3-pos) | `gait_index` — Wave (0) / Ripple (1) / Tripod (2). Live in **every** `SW_F` position | 0..2 |
| `POT1` | `speed` scalar | 0..1 |
| `POT2` | `body_height` scalar — maps to `65..150 mm` ride height, neutral `132 mm` at centre | 0..1 |
| `NAV1` (gait-tune mode) | `step_height`, `stride`, `duty` | 0..1 each |

`ENC1` and `ENC2` are **no longer bound** in `defaultBindings()`: stride and step
height moved to the NAV1 gait-tune editor, so both encoders are free for future
use. A host may still bind an axis to `stride` / `step_height` / `duty` through
`SET_BINDINGS`; when a binding is present it overrides the editor value.

### 3b. Gait-tune editor (SW_G + NAV1)

`SW_G` is a maintained tuning-mode switch: ON enters the editor and OFF leaves
it. While it is engaged, NAV1 edits gait parameters instead of pose trim; while
it is disengaged NAV1 keeps its normal trim role.

| Input | Action |
| --- | --- |
| `SW_G` | ON: engage gait tuning; OFF: leave gait tuning |
| `NAV1 Up` / `Down` | Next / previous parameter (step height → stride → duty) |
| `NAV1 Right` / `Left` | Increase / decrease by `kGaitTuneStepFrac` (5% of range, 20 presses end to end) |
| `NAV1 Center` | Save the live gait shape to the 24LC32 config |

While the editor is engaged and the sticks are centred, **leg 1 traces the swing
arc** at a slow 2 s cycle using the live stride and step height, so the operator
can see the effect on the robot itself. The other five legs hold their stance.
The preview target passes through the same reach-margin clamp and servo travel
clamp as a walking target, and any stick input immediately cancels it.

The handset also shows the live values: the CRSF status frame carries the
selected parameter and the applied stride / step height / duty, and its rate is
raised from 5 Hz to 20 Hz for 3 s after every edit so the readout tracks the
knob without perceptible lag.

Saving is refused while the robot is moving (`AGENTS.md` 4.3 forbids EEPROM
writes during walking) and when the config store is volatile; the reason is
reported on the downlink through the deduplicated error journal.

On boot — and after any config commit — `rcTask` seeds the editor from the
persisted `GaitDefaults`, so the handset always starts from what is stored.

---

## 4. Feature toggles, arm, and kill

2-position switches map directly to safety/feature requests. The firmware is
still the final authority and may reject any request (see safety rules in
`AGENTS.md`); these are *requests*, not commands.

| Switch | `ControllerCommand` field | Meaning |
| --- | --- | --- |
| `SW_A` | `arm_request` | Arm (ignored while kill is active) |
| `SW_B` | `estop` | **Kill / emergency stop** — also forces `arm_request=false` |
| `SW_C` | `feat_foot_contact` | Request foot-contact detection |
| `SW_D` | `feat_terrain_leveling` | Request terrain leveling |
| `SW_G` | `gait_tune_active` | ON: gait-tune editor; OFF: normal NAV1 trim controls |
| `SW_H` | `host_authority` | Hand high-level authority to a host/Jetson |

`NAV1` is the **body-pose trim** cluster while the gait-tune editor is closed
(persistent offset applied on top of the live pose):

| NAV1 | Action |
| --- | --- |
| Up / Down | pitch trim ± `kTrimStepRad` (1° per press) |
| Left / Right | roll trim ± 1° per press |
| Center | reset all trim to 0 |

Trim accumulates on **rising edges only** (one step per press), clamped to
±`kTrimMaxRad`. See §3b for what the same cluster does while the gait-tune
editor is engaged.

---

## 5. Tricks

`NAV2` and the four buttons fire scripted tricks. The bridge emits only the
**rising-edge `TrickId` trigger** (debounced with a 150 ms refractory window,
one trick per frame); the actual choreography (timed joint/body sequences) is
implemented by the control-task trick engine in `oha.5`.

| Input | `TrickId` | Description |
| --- | --- | --- |
| `BTN_1` | `StandUp` | Stand up from rest |
| `BTN_2` | `SitDown` | Sit/rest down |
| `BTN_3` | `Wave` | Rock the standing body side to side |
| `BTN_4` | `CrouchToggle` | Toggle crouch height |
| `NAV2 Up` | `Twirl` | Rotate body 360° in place |
| `NAV2 Down` | `Stretch` | Full-body stretch sequence |
| `NAV2 Left` | `LeanLook` | Lean/look around |
| `NAV2 Center` | `DanceLoop` | Looping dance until cancelled |

`NAV2 Right` is unassigned by the default profile.

All seven+ tricks from the request are covered. Bindings are remappable (any
boolean source → any `TrickId`, up to `kMaxTrickBindings` = 8).

---

## 5b. Input profiles: custom ChannelPack vs. TX16S MK3 direct

The bridge supports **two input profiles**, selected automatically:

1. **`CustomControllerChannelPack`** — the ESP32-S3 TX described above, which
   packs switches/buttons/toggles/nav into the CH9–CH11 bitfields and centres
   CH12–CH16. This is the original behaviour and is unchanged.
2. **`Tx16sMk3Direct`** — a RadioMaster **TX16S MK3** running ELRS with a saved
   EdgeTX model, driving the robot directly. Its channels carry conventional
   per-channel stick/knob/switch values, which the bridge decodes into the
   **same** `ChannelPackInputs_t` fields so all binding/command logic downstream
   is identical.

**Both arrive as ordinary CRSF *RC channels* frames** — the CRSF frame *type* is
the same for both. Detection therefore happens at the **decoded 16-channel
layout** level, never from the frame type:

- `looksLikeCustomChannelPack(ch)` validates the packed-bitfield shape: CH9 uses
  only bits 0–5 (bits 6–7 reserved/clear), CH10's two tri-fields are ≤ 2 with
  nothing above bit 7, CH11 uses only bits 0–9, and CH12–CH16 sit near CRSF
  centre. A frame that fails this is treated as TX16S direct.
- Detection requires `kProfileDetectFrames` (3) consecutive agreeing link-up
  frames before it **locks**, so a glitchy startup frame cannot pick the wrong
  profile. Until a profile locks the bridge holds **failsafe**.
- Once locked the profile **never auto-switches** until `reset()`. A link drop
  after lock keeps the same profile (no re-detection on return). A link drop
  *before* lock clears the pending streak so stale startup frames cannot combine
  with later frames.

`ControllerBridge::detectedProfile()` / `profileLocked()` expose the state.

### TX16S MK3 direct channel map

Saved from EdgeTX model `ROBOT_TX16S_DIRECT` (see
`firmware/openrb150/docs/ROBOT_TX16S_channel_map.csv`):

| CRSF ch | EdgeTX source | Robot use | Decoded into |
| --- | --- | --- | --- |
| CH1 | Rud | left stick X | `gimbal[0]` (LX) |
| CH2 | Thr | left stick Y | `gimbal[1]` (LY) |
| CH3 | Ail | right stick X | `gimbal[2]` (RX) |
| CH4 | Ele | right stick Y | `gimbal[3]` (RY) |
| CH5 | S1 | speed | `pot[0]` |
| CH6 | S2 | body height | `pot[1]` |
| CH7 | LS | stride | `encoder[0]` + `enc_accum_[0]` (absolute) |
| CH8 | RS | step height | `encoder[1]` + `enc_accum_[1]` (absolute) |
| CH9 | SE | gait select | `toggles[0]` / SwE (0/1/2) |
| CH10 | SD | mode select | `toggles[1]` / SwF (0/1/2) |
| CH11 | SA/SF mix | safety mask (2 bits) | bit0→SwA arm, bit1→SwB estop |
| CH12 | SB/SC/SG mix | feature mask (4 bits) | SwC/SwD/SwG/SwH |
| CH13 | GV1 ACT | action selector (13 pos) | picks one button/nav |
| CH14 | SH | action fire (momentary) | gates the CH13 selection |
| CH15 | MAX 0 | reserved centre | (none) |
| CH16 | MAX 0 | reserved centre | (none) |

The TX16S has no NAV cluster, so the gait-tune editor is unreachable from it.
Instead the LS/RS sliders feed the same stride / step-height values the editor
edits (absolute, not integrated), so tuning, telemetry, and the save path behave
identically on both profiles. Duty stays at the persisted default on the TX16S.

Notes on the direct profile:

- **Virtual encoders (CH7/CH8):** the TX16S has no physical relative encoders,
  so the LS/RS sliders are decoded as **absolute** 0..1 values written straight
  into `enc_accum_[]` (and a 0..2047 debug value into `encoder[]`). The relative
  wrap-delta integration runs **only** for the custom controller.
- **Safety mask (CH11):** `mask 0`=disarmed/no-estop, `1`=armed, `2`=estop,
  `3`=armed+estop. Existing command logic still applies: `arm_request = SwA &&
  !SwB`, `estop = SwB`, so mask 2 and 3 both force estop and disarm.
- **Feature mask (CH12):** always parsed as a full 4-bit / 16-position mask on
  the robot side. If the current EdgeTX mix only drives bits 0–2, bit 3
  (`host_authority`) simply stays false until the radio mix/Lua helper is
  updated — the robot-side mapping is not changed to work around that.
- **Action selector/fire (CH13/CH14):** CH13 selects *which* logical
  button/nav boolean to arm; CH14 (SH) fires it. When SH is inactive **none**
  fire. This reuses the existing rising-edge/debounce logic for trims and
  tricks. `Nav2Right` is deliberately unmapped and always stays false.

### Saved EdgeTX model profile

```
Model name: ROBOT_TX16S_DIRECT

RF:
- Internal RF: CRSF (built-in ELRS)
- External RF: Off (unless using an external ELRS module)
- Channel range: CH1–CH16
- ELRS Lua:
  - Packet rate: 100Hz Full or 333Hz Full
  - Switch mode: 16ch Rate/2 Full Res

Mixes:
- CH01 = Rud = left stick X
- CH02 = Thr = left stick Y
- CH03 = Ail = right stick X
- CH04 = Ele = right stick Y
- CH05 = S1 knob
- CH06 = S2 knob
- CH07 = LS slider
- CH08 = RS slider
- CH09 = SE, 3-position mode selector
- CH10 = SD, 3-position gait selector
- CH11 = SA/SF mix, safety mask, 4 positions / 2 bits
- CH12 = SB/SC/SG mix, feature mask, 16 positions / 4 bits (robot side)
- CH13 = GV1 ACT, action selector, 13 positions
- CH14 = SH, action fire momentary
- CH15 = MAX 0, center/reserved
- CH16 = MAX 0, center/reserved
```

The mask/selector channels (CH11/CH12/CH13) need discrete positions, produced by
EdgeTX mixes/logical switches or a small EdgeTX Lua/mixer helper on the radio.
The robot-side parser is implemented cleanly regardless; missing radio positions
just leave the corresponding features/actions inactive.

---

## 6. Failsafe

- If the CRSF link is down (`link_up=false`) the bridge enters failsafe: `valid=false`,
  `failsafe=true`, `estop=true`, `arm_request=false`, **all motion zeroed**, no
  trick. Trim, mode, and "ever seen the link" state are preserved so recovery is
  clean.
- `evaluateFailsafe(now_ms, timeout)` also trips failsafe if no fresh frame has
  arrived within `timeout` (`kDefaultFailsafeMs` = 250 ms) or the link has never
  been seen.
- RC kill (`SW_B`) and failsafe always win over any host command — the firmware
  arbiter must honour this.

---

## 7. Remappable bindings

Every binding above is data, not code: `controller::BindingConfig` holds an
`AxisBinding`/`BoolSource`/`TriSource`/`TrickBinding` for each function, and
`defaultBindings()` fills the documented layout. `AxisBinding` carries
`{source, invert, deadband}` so any proportional function can be moved to any
axis, inverted, and given a centre deadband (default 0.05 on gimbal axes to kill
jitter). The whole config is trivially-copyable so a USB handler can read/replace
it wholesale.

---

## 8. Proposed USB `CONTROLLER_*` API (oha.4)

Exposed through the existing COBS+CRC framed protocol. Three commands plus one
telemetry stream let a host inspect the live controller intent, watch raw
inputs, and remap the bindings — so everything the physical controller can do is
also scriptable over USB.

| Message | Dir | Payload | Purpose |
| --- | --- | --- | --- |
| `CONTROLLER_GET_STATE` | host→mcu / mcu→host | decoded `ControllerCommand` | Current high-level intent: mode, twist, body pose, trim, gait, shape params, feature requests, arm/estop, last trick, `valid`/`failsafe`. |
| `CONTROLLER_GET_BINDINGS` | host→mcu / mcu→host | `BindingConfig` | Read the current binding table. |
| `CONTROLLER_SET_BINDINGS` | host→mcu | `BindingConfig` (+ validate/commit flags) | Replace the binding table; firmware validates ranges then applies (and optionally persists to config EEPROM). |
| `controller_state` (telemetry) | mcu→host stream | `ControllerCommand` + raw `ChannelPackInputs_t` snapshot | Rate-limited subscription for the companion Diagnostics/“RC input” page and binding UI. |

Notes:

- The decoded `controller_state` stream should carry a schema version and units
  (ticks for raw, normalized -1..1 / 0..1 for axes, rad/mm for pose), consistent
  with the rest of the telemetry contract.
- `CONTROLLER_SET_BINDINGS` is a configuration write: stage → validate →
  read-back, and gate behind the same safe-state rules as other config writes.
- A host with `host_authority` may also synthesize intent equivalent to the
  controller, but the firmware arbiter keeps RC kill/failsafe authoritative.

---

## 9. Module map and follow-ups

| Item | Location / Beads |
| --- | --- |
| Portable bridge (this pass) | `firmware/openrb150/src/input/controller_bridge.{h,cpp}` |
| Native tests (40) | `firmware/openrb150/test/test_controller_bridge/` |
| Wire into `rcTask`/`controlTask` (move core without stepping) | `oha.3` |
| USB `CONTROLLER_*` API + `controller_state` stream | `oha.4` |
| Trick/choreography engine | `oha.5` |

The bridge is intentionally decoupled from FreeRTOS, Serial, and the gait
pipeline so it can be unit-tested on the host and wired into the real tasks
without changing its logic.
