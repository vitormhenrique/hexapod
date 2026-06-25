# OpenRB-150 on PlatformIO: custom board, and how it differs from `mkrzero`

**Read this before writing any peripheral code.** This project targets a
ROBOTIS **OpenRB-150**. PlatformIO ships no OpenRB-150 board package, so this
repo provides its own:

- `boards/openrb150.json` — custom board definition (`board = openrb150`).
- `variants/OpenRB-150/variant.h` / `variant.cpp` — the **exact** OpenRB-150 pin
  map taken from the ROBOTIS board package.
- `variants/OpenRB-150/linker_scripts/gcc/*.ld` — SAMD21G18A linker scripts.

**Build with `[env:openrb150]` (the default env).** It uses the same SAMD21 MCU
and the same bossac upload flow as `mkrzero`, but with the **correct OpenRB-150
peripheral mapping** — so `Serial1` really is the DYNAMIXEL bus, `Serial2`/
`Serial3` exist, and `BDPIN_DXL_PWR_EN` is the real power-FET pin.

A second env, `[env:mkrzero]`, is kept only as a bare "does the toolchain work?"
fallback. **Do not rely on its serial / DXL-power pin mapping** — the rest of
this document explains exactly why.

```bash
pio run -e openrb150            # build the real board (default)
pio run -e openrb150 -t upload  # flash it
pio run -e mkrzero              # toolchain-only fallback
```

Sources:
- OpenRB-150 e-Manual: <https://emanual.robotis.com/docs/en/parts/controller/openrb-150/>
- OpenRB-150 board package: <https://github.com/ROBOTIS-GIT/OpenRB-150>
- Dynamixel2Arduino: <https://github.com/ROBOTIS-GIT/Dynamixel2Arduino>

---

## TL;DR for agents

`[env:openrb150]` is the source of truth. The table shows what each env gives you:

| Capability | `openrb150` (custom) | `mkrzero` (fallback) |
| --- | --- | --- |
| Build / link / upload | ✅ | ✅ same MCU + upload flow |
| `LED_BUILTIN` blink (pin 32) | ✅ | ✅ matches exactly |
| USB CDC `Serial` | ✅ | ✅ both alias `Serial` → `SerialUSB` |
| I2C `Wire` (SDA/SCL) | ✅ | ✅ MKR header pins (11/12) |
| Battery ADC (`ADC_BATTERY`, pin 33) | ✅ | ⚠️ defined, verify scaling |
| **DYNAMIXEL bus via `Serial1`** | ✅ SERCOM2, pins 26/27 | ❌ MKR header UART (13/14) |
| **DXL power FET `BDPIN_DXL_PWR_EN`** | ✅ pin 31 | ❌ undefined |
| **`Serial2` (ExpressLRS/CRSF)** | ✅ SERCOM4, pins 28/29 | ❌ does not exist |
| **`Serial3` (expansion UART)** | ✅ SERCOM5, pins 13/14 | ❌ does not exist |
| OpenRB build defines | ✅ `__OPENRB_150__` set | ❌ absent |

**Always build firmware with `-e openrb150`.** If you build with `-e mkrzero`,
anything in the ❌ column will silently target the wrong physical pins, or fail
to compile (`Serial2`/`Serial3`).

---

## Why the MCU match is enough for uploading

| Spec | OpenRB-150 | MKR Zero | Match |
| --- | --- | --- | --- |
| MCU | SAMD21G18A Cortex-M0+ | SAMD21G18A Cortex-M0+ | ✅ |
| Clock | 48 MHz | 48 MHz | ✅ |
| Flash | 256 KB (8 KB bootloader) | 256 KB | ✅ |
| SRAM | 32 KB | 32 KB | ✅ |
| Internal EEPROM | none | none | ✅ |
| I/O voltage | 3.3 V | 3.3 V | ✅ |
| Form factor | Arduino MKR | Arduino MKR | ✅ |
| Upload | bossac, `use_1200bps_touch`, native USB | bossac, 1200 bps touch | ✅ |

Upload tip: if an upload fails, **double-tap the RESET button** to force the
SAMD21 into the bootloader (the sketch does not run in bootloader mode), then
upload again.

> ⚠️ Resetting the MCU also momentarily cuts DXL power. Only reset when the
> actuators are in a mechanically safe position.

---

## The serial-port trap (most important difference)

The OpenRB-150 variant rewires the SAMD21 SERCOMs to the on-board connectors.
The stock `mkrzero` variant does not. The same `SerialN` name points at
**different physical pins** depending on which env you build:

| Name | `openrb150` (custom variant) | `mkrzero` (stock) |
| --- | --- | --- |
| `Serial` | USB CDC (`SerialUSB`) | USB CDC (`SerialUSB`) — same |
| `Serial1` | **Internal DYNAMIXEL TTL bus** — SERCOM2, pins 26/27 (PA12/PA13) | MKR header UART (RX=13 / TX=14) |
| `Serial2` | External 4-pin UART connector — SERCOM4, pins 28/29 (PA14/PA15) | **not defined** |
| `Serial3` | Expansion UART header — SERCOM5, pins 13/14 (PB23/PB22) | **not defined** |

This is why the custom board exists. With `[env:openrb150]`:

- `Dynamixel2Arduino dxl(Serial1);` reaches the servos on the real internal DXL
  bus, matching the AGENTS.md wiring plan (DXL on `Serial1`, ExpressLRS/CRSF on
  `Serial2`, USB API on `Serial`).
- `Serial2` / `Serial3` exist and compile.

With `[env:mkrzero]` (fallback only):

- `Dynamixel2Arduino dxl(Serial1);` **will not reach the servos** — bytes go to
  the MKR header UART pins, and there is no automatic half-duplex direction
  control. Do not assume servo bring-up works.
- Any code referencing `Serial2` / `Serial3` **fails to compile**.

> Note on DXL direction control: the OpenRB-150 hardware auto-toggles the TTL
> bus direction; Dynamixel2Arduino's `DXL_DIR_PIN` is not needed when you
> construct it with `Serial1` on this board. Pass `DXL_DIR_PIN = -1` (default)
> unless you wire an external transceiver.

---

## LEDs

The OpenRB-150 has three LEDs:

| LED | Color | Controllable? | Pin |
| --- | --- | --- | --- |
| `PWR` | Green | No — hardwired, on whenever powered | — |
| `USER` | Orange | **Yes** | `PIN_LED` / `LED_BUILTIN` = **32** |
| `DXL` | Red | Indirect — lit when DXL power FET is ON | driven by `BDPIN_DXL_PWR_EN` (31) |

- `LED_BUILTIN` (pin 32) is **identical** on both envs, so the blink smoke test
  in `src/main.cpp` is valid on real hardware. Use it as the "is my sketch
  running?" heartbeat.
- The red `DXL` LED is not a GPIO; it tracks the DYNAMIXEL power FET. You cannot
  blink it directly — it turns on when you enable DXL power (OpenRB-150 only).
- DYNAMIXEL servos also have their own LED, toggled via the Dynamixel2Arduino
  `ledOn()` / `ledOff()` API (not a board pin).

---

## OpenRB-150 pin definitions

These come from the bundled `variants/OpenRB-150/variant.h` and are available
automatically when building `[env:openrb150]`. They are **absent** under
`[env:mkrzero]` (except `PIN_LED`/`LED_BUILTIN` and `ADC_BATTERY`, which happen
to match):

```c
// Battery monitor ADC
#define ADC_BATTERY        (33u)   // also defined on mkrzero — verify scaling

// LEDs
#define PIN_LED            (32u)   // USER orange LED — matches mkrzero
#define LED_BUILTIN        PIN_LED

// DYNAMIXEL power FET enable (OpenRB-150 ONLY — internal FET)
#define BDPIN_DXL_PWR_EN   (31u)   // OFF by default at power-on; HIGH = DXL power on

// Internal DYNAMIXEL TTL bus (Serial1 on the OpenRB-150 variant)
#define PIN_SERIAL1_TX     (26ul)
#define PIN_SERIAL1_RX     (27ul)

// External 4-pin UART connector (Serial2 on the OpenRB-150 variant)
#define PIN_SERIAL2_TX     (28ul)
#define PIN_SERIAL2_RX     (29ul)

// Expansion UART on header pins 13/14 (Serial3 on the OpenRB-150 variant)
#define PIN_SERIAL3_TX     (14ul)
#define PIN_SERIAL3_RX     (13ul)
```

Build flags: `[env:openrb150]` sets
`-DARDUINO_OpenRB -D__SAMD21G18A__ -DUSE_ARDUINO_MKR_PIN_LAYOUT -D__OPENRB_150__`
(see `boards/openrb150.json`). `[env:mkrzero]` sets `ARDUINO_SAMD_MKRZERO`
instead and omits `__OPENRB_150__`. Use `#if defined(__OPENRB_150__)` to guard
any board-specific code.

---

## DYNAMIXEL power FET (`BDPIN_DXL_PWR_EN`, pin 31)

On the OpenRB-150 a FET gates 12 V power to the four DYNAMIXEL ports:

- **OFF by default** at power-on (servos unpowered until firmware enables it).
- Drive `BDPIN_DXL_PWR_EN` HIGH to enable; the red `DXL` LED then lights.
- Defined under `[env:openrb150]`. On `[env:mkrzero]` the symbol is undefined and
  pin 31 does nothing equivalent on a bare MKR Zero. Keeping DXL power off until
  armed is a firmware safety responsibility.

> ⚠️ The OpenRB-150 DYNAMIXEL ports can pass up to 3 A. Do not power all 18
> MX-28AT servos through this path — use a proper external 12 V distribution with
> fusing and a common ground (see AGENTS.md §1.1).

---

## Power, voltage, and I/O cautions (hardware, board-independent)

- Logic is **3.3 V**; I/O pins are **not 5 V tolerant**. >3.3 V on any I/O pin
  can destroy the SAMD21.
- USB current is fuse-limited to 500 mA — do not run servos off USB power; use
  the terminal VIN / XT60 with the power-source jumper set correctly.
- Never connect/disconnect DYNAMIXEL cables while powered.
- DYNAMIXEL TTL baud accuracy on the SAMD21: 1 Mbps and 9.6 kbps are exact;
  57.6 k / 115.2 k have ~-0.16% error (within tolerance).

---

## Recommended workflow

1. **Always build/flash with `[env:openrb150]`** (the default env). It is the
   correct mapping for DYNAMIXEL, ExpressLRS/CRSF, DXL power, LEDs, and battery.
2. Use the blink sketch in `src/main.cpp` as the first upload smoke test
   (toolchain + USB + USER LED).
3. Only fall back to `[env:mkrzero]` to isolate a toolchain problem from a
   variant problem. Never rely on its serial / DXL-power pins.

## Maintaining the custom variant

The variant files are vendored copies from the ROBOTIS OpenRB-150 board package
(`variants/OpenRB-150/variant.{h,cpp}`). If ROBOTIS updates them, re-fetch from
<https://github.com/ROBOTIS-GIT/OpenRB-150/tree/master/variants/OpenRB-150> and
keep the local `linker_scripts/gcc/*.ld` (copied from the SAMD core's `mkrzero`
variant; identical SAMD21G18A layout). The board JSON
(`boards/openrb150.json`) is modeled on the SAMD `mkrzero.json` with OpenRB-150
USB VID/PID (0x2F5D/0x2202), `variant = OpenRB-150`, and the OpenRB build flags.
