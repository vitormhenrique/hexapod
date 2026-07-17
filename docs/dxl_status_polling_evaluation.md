# DYNAMIXEL Detail Polling Evaluation

## Scope

This note evaluates `hexapod_src-31s`: improving refresh of velocity, load,
voltage, temperature, and error detail for an 18-servo MX-28AT bus. It is a
source-level evaluation only. No DYNAMIXEL EEPROM values, bus baud rates, or
firmware polling behavior were changed, and no hardware was connected.

## Current Behavior

`task_dxl` runs at 50 Hz. Each eligible cycle:

1. `DxlBus::syncReadStatus()` performs one direct Present Position read for one
   servo, despite its historic name. Protocol 1.0 does not support Sync Read.
2. `DxlBus::readStatus()` refreshes the same servo's detail fields with separate
   register reads.

For a legacy MX-28, the detail registers are contiguous from address 36 through
43:

| Fields | Address range | Bytes |
| --- | ---: | ---: |
| Position, speed, load, voltage, temperature | 36-43 | 8 |

The current round-robin implementation keeps each DXL task iteration bounded
but spreads a full detail refresh across 18 cycles, nominally about 360 ms at a
50 Hz task period.

## Protocol 1 Bulk Read

Dynamixel2Arduino 0.8.1 supports Protocol 1 `bulkRead`; it explicitly rejects
Protocol 1 `syncRead` and `fastSyncRead`. Its modern
`DYNAMIXEL::InfoBulkReadInst_t` API accepts a caller-owned array, so an
18-servo request can use static memory. Do not use the deprecated
`ParamForBulkReadInst_t` helper for this robot: its compile-time
`DXL_MAX_NODE` is 16.

One legacy 18-servo bulk-read transaction for the contiguous eight-byte range
has this wire-time lower bound, excluding return delay, direction turnaround,
parser time, and retry/failure handling:

$$
\begin{aligned}
B_{\mathrm{request}} &= 7 + 3(18) = 61\ \mathrm{bytes}, \\
B_{\mathrm{responses}} &= 18(6 + 8) = 252\ \mathrm{bytes}, \\
B_{\mathrm{total}} &= 313\ \mathrm{bytes}, \\
t_{57{,}600} &= \frac{313 \cdot 10}{57{,}600}
                 \approx 54.3\ \mathrm{ms}.
\end{aligned}
$$

That is materially faster than five direct detail reads per servo, but it is
not compatible with a 20 ms high-priority DXL task slot. The library receives
status packets synchronously in sequence, so a late or absent servo extends the
call further by its supplied timeout. A direct replacement would violate the
existing bounded-work and watchdog rationale.

The MX-28 Protocol 2 table needs a separate design. Its relevant values are not
the same compact contiguous block: load starts at 126, position is at 132, and
voltage/temperature are at 144/146. A legacy-only bulk-read decoder must never
be applied to a mixed Protocol 1/Protocol 2 bus.

## Higher Baud Evaluation

The SAMD21/OpenRB-150 documentation records exact 1 Mbps UART timing; 57.6 k
and 115.2 k are also within tolerance. At 1 Mbps, the $313$-byte lower bound
falls to about $3.13\ \mathrm{ms}$ before servo return delay and software
overhead.

The current firmware still initializes `DxlBus` at its compiled 57,600 baud
default. It exposes a logical EEPROM `BaudRate` parameter per servo, but has no
transactional bus-baud migration state machine or persistent bus-baud setting.
Changing servo EEPROM baud alone would strand the host after a reboot or leave
a mixed-baud bus while a multi-servo migration is incomplete.

A future baud migration must therefore:

1. Require disarmed maintenance state, confirmed torque off, and DXL power.
2. Record every profile and original baud value before modifying EEPROM.
3. Change one known servo at a time, switch the host baud as needed, and read
   back the value at the new baud before proceeding.
4. Persist the selected bus baud only after every expected servo verifies.
5. Define a power-cycle and recovery procedure for an interrupted or mixed-baud
   migration.

## Decision

Do not change normal firmware polling or the global bus baud from this
evaluation alone.

The lowest-risk experiment is a legacy-only, static-memory modern `bulkRead`
prototype in the output-disabled image. It must run at a deliberately measured
rate below the DXL task budget, not as an unconditional 50 Hz call. Retain the
existing one-servo direct read as the fallback when profiles are mixed, a bulk
response is incomplete, or the measured transaction budget is exceeded.

Only consider a 1 Mbps migration after the bulk-read benchmark establishes a
clear need and the migration transaction above is implemented and reviewed.

## Required Hardware Benchmark

With the robot mechanically supported, torque off, and the output-disabled
firmware image loaded, collect evidence for all 18 expected servos:

1. Current round-robin detail freshness, DXL task timing, error counters, and
   watchdog state at 57,600 baud.
2. Legacy-only modern bulk-read transaction time, success count, status packet
   error bits, and worst-case behavior with one servo temporarily unavailable.
3. A rate sweep that proves health/watchdog progress and preserves the existing
   position snapshot fallback.
4. Only if justified, a staged 115,200 or 1 Mbps migration on a recoverable
   bench setup, including reboot/recovery evidence.

The connected robot is required for this benchmark; simulation and native tests
cannot validate half-duplex turnaround, servo return delay, or EEPROM baud
migration recovery.