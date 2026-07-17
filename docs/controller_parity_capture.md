# ControllerCore Parity Capture and Comparison

Status: the native comparator is implemented; no ATSAMD21 trace has been
captured yet. A synthetic trace validates the comparison machinery only. It is
not hardware-in-the-loop evidence and must not be used to close a bench or HIL
acceptance gate.

## Scope

This procedure compares the portable decision boundary of
`controller::ControllerCore` on two executions:

1. The ATSAMD21 execution records the exact input snapshots supplied by
   `app::controlTask` and the `RobotCommand` produced by that same call.
2. Native tests replay the recorded input snapshots through a fresh
   `ControllerCore` and compare its command with the recorded command.

It answers whether controller state, command authority, safety policy, and
final calibrated DYNAMIXEL target ticks agree. It does not compare USB timing,
FreeRTOS scheduling, DXL bus writes, servo present positions, or physical gait
quality. Those remain adapter, bus, and mechanical checks.

## Capture Point

The target capture point is in `controlTask`, immediately after:

```cpp
g_controllerCore.step(g_controllerState, g_controllerIntent,
                      g_controllerConfig, cycle_time, g_controllerCommand);
```

and before `publishControllerCommand(now_ms)`. At that point the input and
output belong to one control step, while DXL power sequencing, torque changes,
maintenance-target clearing, goal-frame publication, Sync Write, and physical
servo behavior have not yet changed the adapter state.

The future recorder must copy only fixed-size snapshots. It must not allocate,
block the 100 Hz control task, use a serial callback, or write a peripheral
from the capture point. The HIL serial framing and authority mechanism are
specified by `hexapod_src-4ju.17`; the target recorder and output-disabled
execution are implemented by `hexapod_src-4ju.18`.

## Logical Schema V1

The native schema is defined by
`firmware/openrb150/test/support/controller_parity.h`.

Each `Trace` has a `TraceHeader`:

| Field | Meaning |
| --- | --- |
| `format_version` | Must be `1`. Any other value is rejected. |
| `origin` | `Synthetic`, `Atsamd21OutputDisabled`, or `Atsamd21Bench`. |
| `frame_count` | Number of records, from 1 through 32. |
| `nominal_period_ms` | Expected controller cadence. It must be nonzero. |

Each `TraceFrame` contains:

| Record | Contents |
| --- | --- |
| `ControllerTime` | `now_ms`, `dt_ms`, and validity from the adapter clock. |
| `RobotState` | Battery, DXL feedback/readiness, contact snapshot, config-ready, and watchdog state. |
| `ControllerIntent` | Decoded RC, host motion, maintenance, effective feature, estop, disarm, passive, clear-fault, and Jetson-heartbeat inputs. |
| `ControllerConfigSnapshot` | Validated config value, revision, and persistence state. |
| `CommandObservation` | The controller command directly after `step`: safety/fault/source, policy booleans, diagnostics, and final calibrated goal ticks. |

This is a logical schema, not a C++ object-memory wire format. The later USB
trace protocol must use explicit versioned field encoding, canonical endianness,
and explicit numeric units; it must never transmit compiler padding or raw
object memory. It may de-duplicate unchanged configuration values on the wire,
but its decoder must reconstruct the full per-frame `ControllerStepInput` before
native comparison.

## Comparison Rules

`controller::parity::replayAndCompare()` creates a fresh core and replays every
captured input in sequence. The following fields must match exactly:

- safety state, fault reason, command source, authority, motion gate, DXL
  power policy, torque policy, and goal validity;
- config revision and all controller diagnostics, including gate edges, contact
  confidence, maintenance-clear requests, and clamp/reachability verdicts;
- goal count and ordered `(id, tick, leg, joint, clamped)` records.

Final goal ticks are integers after the existing gait, IK, servo-map, and travel
limit stages. There is no tick tolerance: a one-tick difference is an actionable
controller parity failure. The comparator returns the first frame and, for a
goal mismatch, the goal index and field category.

There is no controller-output float tolerance because the comparison terminates
at calibrated integer ticks. Transport code must preserve the input values that
the target passed to the core; it must not round them to display units before
replay. A record with `time.valid == true` and `dt_ms > 1000` is rejected because
the adapter clock declares that gap invalid rather than stepping a giant delta.
Recorded `now_ms` and `dt_ms` are replayed exactly, including normal scheduler
overruns and unsigned millisecond wraparound. No wall-clock or ROS timestamp
offset is part of this comparison.

## Physical Response Is Separate

`servo_status.present_position` measures a physical response after controller
output and can lag or diverge for legitimate mechanical reasons. A servo
tracking analysis may compare it to a target tick with a joint-specific,
experimentally justified tolerance, but it must report that result separately
from ControllerCore parity. A physical tracking discrepancy must never be
treated as evidence that a native replay differs from the controller command.

## Future ATSAMD21 Procedure

1. Run all native tests and build the normal target before testing.
2. Use the explicit output-disabled HIL mode from `hexapod_src-4ju.18`; confirm
   DXL power, torque enable, and Sync Write are blocked and reported.
3. Start an authorized HIL trace session using the `hexapod_src-4ju.17`
   contract. Preserve raw serial frames, decoded records, firmware revision,
   config revision/hash, trace origin, and output-disabled evidence.
4. Decode the capture into a version-1 `Trace` fixture and add a native test
   that calls `replayAndCompare()`.
5. Treat the first mismatch as the investigation anchor. Compare its input,
   command observation, target task stage, and raw frame before widening the
   trace or changing controller code.

The native test `test_controller_parity` demonstrates the comparator with a
synthetic arm-to-stand, walk, authority-loss, and E-stop sequence. A corrupted
goal tick fails with an explicit frame, goal index, and `GoalTick` reason. A
real `Atsamd21OutputDisabled` or `Atsamd21Bench` fixture is still required
before this document can serve as HIL completion evidence.