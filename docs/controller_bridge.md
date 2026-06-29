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
Serial2 bytes ──► crsf::Parser ──► raw 11-bit ticks ch[16]
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
  `ChannelPack.h` and is fully host-testable (20 native tests in
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

## 2. Control modes ("move the core without moving the legs")

`SW_E` (3-pos toggle) selects the control mode. The same two gimbals are
re-interpreted per mode; in the two body modes the **feet stay planted** and
only the body shifts/tilts on the standing legs (clamped to the IK envelope).

| `SW_E` | `ControlMode` | Left gimbal | Right gimbal |
| --- | --- | --- | --- |
| UP | `Walk` | LY = forward/back, LX = yaw turn | RX = strafe |
| CENTER | `TranslateBody` | LY = body Z (height) | RY = body X, RX = body Y |
| DOWN | `RotateBody` | LX = body yaw | RX = body roll, RY = body pitch |

- Translate limits: ±`poselim::kMaxTransMm` = **±50 mm** on each axis.
- Rotate limits: ±`poselim::kMaxRotRad` = **±0.4363 rad (±25°)** on each axis.
- These mirror `protocol::motionlim`, so a controller body pose can never exceed
  what `SET_BODY_POSE` allows over USB.
- In `Walk` mode no body pose is emitted; in the body modes no twist is emitted.

---

## 3. Gait selection and live shape tuning

| Input | Action | Range |
| --- | --- | --- |
| `SW_F` (3-pos) | `gait_index` 0/1/2 — selects gait family (e.g. tripod / ripple / wave) | 0..2 |
| `POT1` | `speed` scalar | 0..1 |
| `POT2` | `body_height` scalar | 0..1 |
| `ENC1` | `stride` length scalar (relative; integrated from encoder delta) | 0..1 |
| `ENC2` | `step_height` scalar (relative; integrated from encoder delta) | 0..1 |

Encoders are **continuous** and have no absolute zero, so the bridge integrates
their wrapped delta (shortest path across the 0↔2047 boundary) into a value
clamped to 0..1, seeded at 0.5 (mid) on reset. This makes ENC1/ENC2 act like
infinite-turn trim knobs for stride and step height.

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
| `SW_G` | `feat_passive_pose` | Request passive-pose streaming |
| `SW_H` | `host_authority` | Hand high-level authority to a host/Jetson |

`NAV1` is the **body-pose trim** cluster (persistent offset applied on top of
the live pose):

| NAV1 | Action |
| --- | --- |
| Up / Down | pitch trim ± `kTrimStepRad` (1° per press) |
| Left / Right | roll trim ± 1° per press |
| Center | reset all trim to 0 |

Trim accumulates on **rising edges only** (one step per press), clamped to
±`kTrimMaxRad`.

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
| `BTN_3` | `Wave` | Lift a front leg and wave |
| `BTN_4` | `CrouchToggle` | Toggle crouch height |
| `NAV2 Up` | `Twirl` | Rotate body 360° in place |
| `NAV2 Down` | `Stretch` | Full-body stretch sequence |
| `NAV2 Left` | `LeanLook` | Lean/look around |
| `NAV2 Center` | `DanceLoop` | Looping dance until cancelled |

All seven+ tricks from the request are covered. Bindings are remappable (any
boolean source → any `TrickId`, up to `kMaxTrickBindings` = 8).

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
| Native tests (20) | `firmware/openrb150/test/test_controller_bridge/` |
| Wire into `rcTask`/`controlTask` (move core without stepping) | `oha.3` |
| USB `CONTROLLER_*` API + `controller_state` stream | `oha.4` |
| Trick/choreography engine | `oha.5` |

The bridge is intentionally decoupled from FreeRTOS, Serial, and the gait
pipeline so it can be unit-tested on the host and wired into the real tasks
without changing its logic.
