# OpenRB Firmware SRAM Budget

This budget applies to `firmware/openrb150`, built with
`pio run -e openrb150` for the SAMD21G18A's 32 KiB SRAM.

## Result

| Image | `.data` | `.bss` | `.noinit` | Total RAM | Usage |
| --- | ---: | ---: | ---: | ---: | ---: |
| Baseline before this reduction | - | - | - | 28,728 B | 87.7% |
| Production after reduction | 2,432 B | 22,040 B | 104 B | 24,576 B | 75.0% |
| Output-disabled HIL | - | - | - | 27,044 B | 82.5% |

The production reduction is 4,152 B. PlatformIO's total includes the retained
`.noinit` fault records, so those records remain part of the explicit budget.

## Largest Production Allocations

The final largest RAM-backed symbols are:

| Symbol | Bytes | Reason retained |
| --- | ---: | --- |
| `ucHeap` | 11,584 | Fixed FreeRTOS boot allocation pool |
| `g_dxlBus` | 940 | DYNAMIXEL profiles and Sync Write scratch owned by `dxlTask` |
| `g_controllerCore` | 916 | Gait, IK, and safety-owned controller state |
| `g_configApi` | 696 | Validated editable configuration shadow |
| `g_controllerState` | 628 | Cross-task controller snapshot |
| `Serial1`, `Serial3` | 580 each | Required DXL and CRSF UART buffers |
| USB endpoint caches | 448 each | Arduino USB CDC transport |
| `g_servoStatus` | 432 | One present-status snapshot per physical actuator |

`Serial2` is now opt-in through `HEXAPOD_ENABLE_SERIAL2`; the default USB,
CRSF, and DXL image does not retain its 580-byte UART object.

## Structural Reductions

- Bound all DXL and HIL target storage to the robot's fixed 18 servos rather
  than 24 entries.
- Omit `Serial2` and its SERCOM wrapper unless a deployment explicitly needs
  the external UART.
- Remove temporary plaintext USB heartbeat, watchdog-hold switch, standalone
  LED blink task, and unconsumed task-loop counter.
- Use a project-owned FreeRTOS configuration that omits the unused timer daemon,
  timer queue, queue registry, runtime statistics, and related unused APIs.
- Keep the normal image's HIL observer response stateless: reserved observer
  commands return `NotAvailable` without allocating the HIL session state.
- Reduce `task_api` to 576 words after compiler stack-use analysis. Its request
  and response buffers are static; the deepest normal dispatch path has 1,912 B
  of reported frames within its 2,304 B allocation. The HIL image retains a
  768-word API stack for trace fragmentation.

## Heap and Stack Guardrails

`tasks.cpp` has a compile-time allocation budget that accounts for all six
application tasks, the FreeRTOS idle task, three semaphore objects, allocator
headers/alignment, and a 512-byte minimum free-heap reserve. It fails the build
if a task stack, semaphore count, or heap configuration no longer fits.

The production heap is 11,584 B. The output-disabled HIL heap is 12,544 B to
cover its larger API stack. The motion-critical stacks were not reduced:

- `controlTask`: increased from 384 to 448 words after the measured 353-word
  peak, leaving 95 words (380 B) headroom.
- `dxlTask`: unchanged at 512 words.

## Probe Classification

Removed probe-only behavior:

- `HEXAPOD_DEBUG_SERIAL_HEARTBEAT`, which injected plaintext into the framed
  USB protocol.
- `HEXAPOD_WATCHDOG_HOLD_PROBE`, which suppressed watchdog evaluation.
- `blinkTask` and its FreeRTOS stack/priority/period configuration.
- The never-read per-task `g_loops` counter.

Retained production safety behavior:

- `.noinit` retained HardFault, stack-overflow, and malloc-failure capture.
- Strong wrapped FreeRTOS fatal hooks and immediate DXL power cutoff.
- Allocation-failure LED indication before the scheduler starts.
- Immutable output-disabled HIL guard and trace capture in the HIL image.

## Hardware-Free Validation

```text
pio test -e native                         515 passed
pio run -e openrb150 -t clean && pio run -e openrb150
                                             24,576 / 32,768 B RAM
pio run -e openrb150_hil_output_disabled -t clean && \
  pio run -e openrb150_hil_output_disabled  27,044 / 32,768 B RAM
```

These checks do not flash a board, enable DYNAMIXEL power, torque servos, arm
the robot, or command motion. A future hardware session should inspect the
reported FreeRTOS high-water marks and status telemetry before reducing any
further task stack or heap margin.