# OpenRB-150 Hexapod Firmware

Deterministic safety + motion controller for the hexapod, running on a ROBOTIS
**OpenRB-150** (Microchip SAMD21G18A, Cortex-M0+ @ 48 MHz, 256 KB flash,
32 KB SRAM). Built with **PlatformIO**.

> Before touching DYNAMIXEL / Serial2 / DXL-power code, read
> [doc/mkrzero-vs-openrb150.md](doc/mkrzero-vs-openrb150.md). This repo ships a
> custom OpenRB-150 board + variant so peripheral pins map to the real hardware.

## Build environments

| Env | Board | Use |
| --- | --- | --- |
| `openrb150` (default) | custom `boards/openrb150.json` + `variants/OpenRB-150` | **All real firmware work.** Correct pin map: `Serial1`=DXL bus, `Serial2`=CRSF, DXL power FET, LEDs, battery ADC. |
| `mkrzero` | stock MKR Zero | Toolchain-only fallback. WRONG serial / DXL-power pins. |

## Commands

```bash
# Build (default env = openrb150)
pio run

# Local check: compile both envs to confirm toolchain + variant are healthy
pio run -e openrb150 -e mkrzero

# Flash the OpenRB-150 over USB
pio run -e openrb150 -t upload

# Serial monitor (USB CDC, 115200)
pio device monitor -b 115200
```

If `pio` is not on PATH, use the bundled binary: `~/.platformio/penv/bin/pio`.

### Upload troubleshooting

- If upload does not catch the port, **double-tap the RESET button** to force the
  SAMD21 bootloader, then re-run upload.
- Resetting the MCU momentarily cuts DXL power — only reset when actuators are in
  a mechanically safe position.

## Safety-first boot behavior

- The DYNAMIXEL power FET (`BDPIN_DXL_PWR_EN`, pin 31) is held **OFF at boot**.
  Servos are unpowered until firmware explicitly arms them.
- Logic is 3.3 V and **not 5 V tolerant**. Do not run servos from USB power.

## Layout

```text
firmware/openrb150/
  platformio.ini            PlatformIO config (openrb150 + mkrzero envs)
  boards/openrb150.json     custom board definition
  variants/OpenRB-150/      exact OpenRB-150 pin map + linker scripts
  doc/                      board notes (read mkrzero-vs-openrb150.md)
  src/
    board/                  board pin map + HAL / safe boot
    main.cpp                entry point
```

## Intentionally not implemented (`#NOTIMPLEMENTED`)

These enhancements were raised by the 2026-06-29 Phase 1/2 audit
([docs/firmware_phase2_beads_audit_2026-06-29.md](../../docs/firmware_phase2_beads_audit_2026-06-29.md))
but are deliberately **not** on the backlog. They are micro-optimizations or
cleanups that are not required for a safe, working robot at this stage. Revisit
only if a profiler or hardware-in-loop run proves a concrete need.

- `#NOTIMPLEMENTED` **Precomputed servo lookup tables** (audit 22l.3). `ServoMap::servoFor()`/
  `servoForId()` linearly scan all servos. With only 18 servos the scan cost is
  negligible; a cached slot/id map adds state to maintain for no measurable gain.
- `#NOTIMPLEMENTED` **Coherent-snapshot / sequence guards on multi-field telemetry** (audit 22l.9).
  `buildTelemetry()` reads `g_servoStatus` while `dxlTask` writes it. A torn record
  is cosmetic for bring-up telemetry. Add a copy/sequence guard only if an HIL run
  shows visibly torn snapshots.
- `#NOTIMPLEMENTED` **Consolidate/remove the unused `CommandArbiter` Mac-lock path** (audit 22l.8).
  `MaintenanceApi` is the single source of truth for the maintenance lock; the
  dormant `CommandArbiter` Mac-lock methods are harmless dead code. Leave them
  until a broader arbiter refactor, rather than churning safety code for cleanup.

> Bench / hardware-in-loop sign-off (audit 22l.11) is tracked separately and is
> **deferred, not rejected** — it is pending hardware, not a non-goal.

See the repo-root `AGENTS.md` for the full architecture, safety rules, and phase
plan. Work is tracked in Beads (`bd ready`), not markdown TODOs.
