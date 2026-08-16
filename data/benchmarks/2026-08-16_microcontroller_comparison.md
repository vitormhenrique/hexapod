# Microcontroller Benchmark Comparison

Date: 2026-08-16

## Scope

This document compares the idle benchmark snapshots saved for the two firmware
controllers:

- [Teensy result](2026-08-16_teensymm_idle_600mhz.txt): full benchmark image
  with the debug OLED enabled.
- [OpenRB result](2026-08-16_openrb150_idle_48mhz.txt): SRAM-safe benchmark
  image with the debug OLED compiled out. The normal OLED-enabled OpenRB image
  was separately flashed and verified API-stable after its SRAM fix.

Both samples used a 15-second benchmark subscription at 5 Hz. The robot was
Disarmed with DYNAMIXEL power off, so the `motion_*` counters in the idle files
are expected to be zero.

## What An Overrun Means

Each periodic task has a nominal period $T$ and a measured loop-body runtime
$E$ in microseconds. The benchmark increments `overruns` when:

$$
E > T
$$

For example, the I2C task normally has $T = 20\,ms$. A $36\,ms$ OLED flush is
one overrun: the next `vTaskDelayUntil()` cannot wait for the missed deadline,
so that task begins its next pass immediately. This is not by itself a watchdog
failure; it is a scheduling-capacity signal. `jitter_us` is separate: it is the
largest deviation of one task release interval from its nominal period.

## Hardware And Memory

| Property | MicroMod Teensy | OpenRB-150 benchmark | OpenRB-150 normal OLED image |
|---|---:|---:|---:|
| MCU | NXP IMXRT1062 | Microchip SAMD21G18A | Microchip SAMD21G18A |
| CPU clock | 600 MHz | 48 MHz | 48 MHz |
| Clock ratio | $12.5\times$ OpenRB | 1x | 1x |
| Addressable RAM reported by build | 1 MiB, split into two 512 KiB banks | 32 KiB | 32 KiB |
| Static RAM used | RAM1: 204,416 B; RAM2: 12,416 B | 30,232 B | 30,424 B |
| Free RAM reported by build | RAM1 local: 319,872 B; RAM2 malloc/new: 511,872 B | 2,536 B | 2,344 B |
| Combined free RAM reported | 831,744 B | 2,536 B | 2,344 B |
| Runtime control stack high-water free | 488 B | 396 B | 368 B at final API verification |
| Runtime DXL stack high-water free | 876 B | 1,016 B | 1,048 B at final API verification |

The OpenRB benchmark intentionally removes the 1,196-byte SparkFun OLED object
and framebuffer. This is necessary because the full benchmark collector plus
OLED leaves insufficient SAMD21 interrupt-stack margin. The full OLED firmware
now uses a corrected FreeRTOS heap of 11,840 B and is stable at 30,424 B static
RAM use.

## Idle Task Timing

| Metric | Teensy | OpenRB-150 | Interpretation |
|---|---:|---:|---|
| Control loop mean / max | 108 / 1,004 us | 838 / 1,652 us | Teensy is $7.8\times$ faster on mean loop cost. |
| Controller step mean / max | 5 / 6 us | 330 / 336 us | Teensy is about $66\times$ faster for idle controller computation. |
| DXL task mean / max | 10 / 10 us | 35 / 42 us | Teensy is $3.5\times$ faster when DXL power is off. |
| RC task mean / max | 2 / 2 us | 203 / 210 us | Teensy has much more serial-processing headroom. |
| API task mean / max | 998 / 1,001 us | 130 / 5,219 us | USB transport behavior differs by board; this is not pure CPU performance. |
| Health task mean / max | 4 / 32 us | 10 / 20 us | Both are negligible. |
| Control/DXL/RC overruns | 0 / 0 / 0 | 0 / 0 / 0 | Both boards meet the idle motion deadlines. |

## I2C Timing Caveat

The I2C figures are not an equivalent workload:

| I2C metric | Teensy | OpenRB-150 |
|---|---:|---:|
| I2C task mean / max | 675 / 36,011 us | 161 / 168 us |
| I2C overruns | 14 | 0 |
| OLED samples | 14, mean 35,995 us | 0, OLED disabled |
| IMU samples | 0 | 0 |
| OpenLog/storage samples | 0 | 0 |

The Teensy result includes 14 OLED page flushes. Each official SparkFun OLED
flush took roughly $36\,ms$, exceeding the 20 ms I2C period and accounting for
all 14 I2C overruns. The OpenRB benchmark removed OLED intentionally to fit in
SAMD21 SRAM, so its lower I2C timing cannot be used to claim a faster I2C bus.

The Teensy firmware was improved before this sample: it now tracks dirty OLED
pages and redraws only changed rows. In comparable short windows that reduced
OLED flushes from 32 to 18 and I2C mean loop time from about 6.7 ms to 4.2 ms.

## Motion Measurement

A mechanically supported Teensy maintenance-gait run produced:

| Motion metric | Teensy result |
|---|---:|
| Controller step mean / max | 39 / 55 us |
| Motion controller mean / max | 40 / 55 us |
| DXL motion cycle mean / max | 32,444 / 197,821 us |
| Controller computation overruns | 0 |

The DXL timing is dominated by the physical half-duplex DYNAMIXEL TTL bus and
servo transactions, not by Teensy CPU time. An equivalent OpenRB motion run is
still needed for direct actuator-path comparison.

## Conclusion

The Teensy is the preferred controller for higher-rate motion, future terrain
processing, and Jetson-facing work. It has large RAM headroom and substantial
control/IK timing margin.

The OpenRB-150 remains viable for the current 100 Hz control loop: its idle
control mean of 838 us stays far below the 10 ms period, and its idle benchmark
reported no control, DXL, RC, or I2C overruns. Its 32 KiB SRAM is the primary
constraint. Keep the benchmark image OLED-free and use the normal full image
for OLED operation.

## Reproduction

```zsh
# Flash the appropriate benchmark image to the connected board.
just firmware-benchmark-upload teensymm
# or
just firmware-benchmark-upload openrb150

# Save an idle benchmark result.
just firmware-benchmark 15 5

# On a mechanically supported robot, run the standard low-speed gait profile.
cd companion
uv run hexapod-cli benchmark-motion --seconds 10 --port <USB_PORT>

# Restore normal firmware after a benchmark run.
cd ..
just firmware-upload teensymm
# or
just firmware-upload openrb150
```
