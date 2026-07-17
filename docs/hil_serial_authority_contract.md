# ATSAMD21 HIL Serial Trace and Authority Contract

Status: the immutable output-disabled build, v0.3 capability/status fields,
`health` extension, `hil_status` telemetry, and board smoke harness are
implemented by `hexapod_src-4ju.18`. The observer-session command group,
fixed-memory trace recorder, ordered fragment encoder, and trace events are
implemented by `hexapod_src-4ju.21`. A synthetic or native-only trace is not
ATSAMD21 or physical HIL evidence; retained board evidence and parity replay
remain work for `hexapod_src-4ju.19`.

## Purpose and Scope

This contract lets a host collect deterministic `ControllerCore` input/output
traces from the OpenRB-150 before physical DYNAMIXEL output is permitted. It
uses the existing USB CDC COBS/CRC transport and the existing safety APIs. It
does not create a second controller, a raw-servo route, or a ROS-to-servo
bypass.

The initial HIL topology is one host connected directly to the OpenRB-150 USB
CDC port. A later Jetson proxy may carry the same framed protocol, but it is a
single client of the MCU and does not create a second control authority. The
MCU remains the safety and motion authority in either topology.

The first HIL implementation supports output-disabled, maintenance-safe
controller execution. It can exercise the normal high-level gait pipeline
through the existing maintenance lock and `MaintControlMode::GaitPipeline`.
RC and Jetson scenarios remain subject to their existing physical RC,
feature-enable, and heartbeat conditions.

## Non-Negotiable Safety Rules

1. HIL trace control is an observer capability. It does not grant motion,
   arm the robot, refresh a maintenance lock, refresh Jetson authority, or
   alter the state machine.
2. The normal build must not contain a runtime command that enables or
   disables actuator output. The output-disabled HIL build is selected at
   compile time, reports that fact, and cannot re-enable output over USB.
3. The only ordinary motion inputs remain the existing validated APIs:
   `SET_GAIT`, `SET_GAIT_PARAMS`, `SET_BODY_TWIST`, `SET_BODY_POSE`,
   `STOP_MOTION`, RC input, and the existing Jetson heartbeat path. They are
   still subject to the arbiter, state machine, IK, ServoMap, and DXL task.
4. `ESTOP`, `SET_ARMING(disarm)`, RC kill, RC failsafe, unsafe battery,
   watchdog fault, and DXL hard fault retain their current precedence over a
   HIL observer session.
5. HIL never accepts raw DYNAMIXEL goal ticks, a raw joint-target topic, a raw
   `RobotCommand`, or an arbitrary `ControllerStepInput` as a normal motion
   command. Existing maintenance targets remain the exception only where they
   already pass through their maintenance lock, IK/ServoMap limits, and state
   gate.

## Existing Protocol Reuse

The wire frame remains exactly the current framed protocol:

```text
0x00 COBS(header || payload || crc16-ccitt-false) 0x00
```

Header fields, little-endian integers, the 256-byte payload cap, frame
sequence numbers, CRC behavior, and error handling remain those defined in
`protocol/README.md` and `firmware/openrb150/src/protocol/framing.h`.

The following existing APIs are reused rather than duplicated:

| Existing surface | HIL meaning |
| --- | --- |
| `HELLO`, `GET_CAPABILITIES`, `GET_STATUS` | Discover the protocol version, build identity, current safety state, and ordinary hardware status. |
| `ESTOP`, `CLEAR_FAULT`, `SET_ARMING`, `SET_MODE` | Existing safety intent only. HIL adds no alternate state transition. |
| `ENTER_MAINTENANCE`, `EXIT_MAINTENANCE`, `MAINT_HEARTBEAT` | Existing low-authority maintenance lock. It is required to open the first HIL observer session and remains independent afterwards. |
| `SET_GAIT`, `SET_GAIT_PARAMS`, `SET_BODY_TWIST`, `SET_BODY_POSE`, `STOP_MOTION` | Bounded high-level stimulus. These commands never grant authority by themselves. |
| `SET_CONTROL_MODE(GaitPipeline)` | Existing maintenance-safe way to exercise the normal gait/IK pipeline without accepting raw servo ticks. |
| `JETSON_HEARTBEAT` | Retains the existing RC-autonomy and feature-enable gates. A generic or HIL heartbeat must never refresh Jetson authority. |
| `SUBSCRIBE`, `GET_STREAM_STATS` | Existing regular telemetry and transport-backlog diagnostics. |
| `health`, `hil_status`, `control_state`, `servo_status`, `contact_state`, `rc_input`, `api_stats`, `servo_goals` | Supporting timeline data. They do not replace the canonical trace record. |

The generic session `HEARTBEAT` remains informational. It must never refresh a
maintenance lock, HIL session, or Jetson authority.

## Additive Versioning and Reserved IDs

The output guard and observer additions are compatible protocol v0.3 features:
the frame major remains `0`, `GET_CAPABILITIES` reports immutable build flags,
and `GET_STATUS`/`health` append guard state plus the retained last-fault
reason and controller timestamp. The observer range is always recognized: a
normal build responds `NotAvailable`, while an output-disabled build applies
the session checks below. No current message ID or telemetry stream number is
renumbered.

Reserve the following currently unused command IDs for the HIL session group.
They are requests with `MsgType::Command` and normal same-sequence responses.

| ID | Command | Purpose |
| ---: | --- | --- |
| `0x14` | `HIL_GET_CAPABILITY` | Report whether this firmware image was built with immutable output disable and the supported trace schema versions. |
| `0x15` | `HIL_OPEN_SESSION` | Create one observer session from a verified output-disabled image. Requires a current maintenance lock token and `Disarmed` or maintenance-gated `MacMaintenance` state. |
| `0x16` | `HIL_CLOSE_SESSION` | Close a session with its HIL session token. It never releases the separate maintenance lock implicitly. |
| `0x17` | `HIL_HEARTBEAT` | Refresh only the observer-session TTL using its session token. |
| `0x18` | `HIL_CAPTURE` | Capture the next bounded sequence of complete controller steps. |
| `0x19` | `HIL_ABORT_CAPTURE` | Stop the active capture and emit an incomplete-capture record. |
| `0x1A` | `HIL_MARK` | Associate a host-chosen numeric marker with the next controller-step boundary. |
| `0x1B` | `HIL_GET_SESSION_STATUS` | Read session/capture activity, IDs, observer heartbeat time, schema version, and output-disabled availability. |

Reserve event IDs `0xA0` through `0xAF` for HIL records. Initial records use
`0xA0` (`HIL_TRACE_FRAGMENT`) and `0xA1` (`HIL_OUTPUT_BLOCKED`). HIL trace
records are events rather than regular telemetry because one logical trace
record may need multiple 256-byte frames and must retain ordering.

`StreamId::HilStatus = 12` is implemented. Its telemetry message ID is `0x4C`
and its maximum rate is 10 Hz. The payload is `flags(u8), goal_count(u8),
last_goal_sequence(u32), blocked_power_enable(u32),
blocked_torque_enable(u32), blocked_goal_write(u32),
blocked_dxl_write(u32)`, followed by bounded `id(u8), tick(i32)` records from
the last blocked Sync Write. This is a last-frame observation, not the future
complete trace recorder.

## Session and Authority Lifecycle

### Opening an observer session

`HIL_OPEN_SESSION` takes a live maintenance-lock token. The firmware accepts
it only when all of the following are true:

- the build reports `output_disabled = true` and all three output guards are
  installed: power enable, torque enable, and Sync Write goal output;
- the current safety state is `Disarmed` or maintenance-gated
   `MacMaintenance`;
- the supplied maintenance token is the current, unexpired token; and
- there is no existing HIL observer session.

The response returns a nonzero `session_id`, a distinct nonzero
`session_token`, the HIL schema version, and an observer capability mask. The
host reads live output-guard counters through existing `GET_STATUS` or
`hil_status` telemetry. The session token authorizes only the HIL command
group. It is not a maintenance token and must never be accepted by maintenance,
config, DXL, or motion APIs.

The maintenance lock can then be deliberately retained for a maintenance
scenario or released so an RC/Jetson scenario can proceed under its normal
authority rules. Releasing or expiring the maintenance lock does not make the
HIL session a replacement authority.

The host sends `HIL_HEARTBEAT` at 4 Hz or faster. The session TTL is 1000 ms.
On expiry, firmware aborts any capture, emits an `HIL_TRACE_FRAGMENT` terminal
record with `HostHeartbeatExpired`, and destroys the observer session. It does
not synthesize an E-stop; output is permanently disabled in this build, and
the normal state machine continues to own safety state. A host must open a new
session before starting another capture.

### Host capture and decode tools

`hexapod-cli hil-capture --steps N --out data/sessions` is the companion-side
observer workflow. It first verifies both immutable `GET_CAPABILITIES` build
flags and the live `HIL_GET_CAPABILITY` response, checks the supported trace
schema, and refuses a firmware status that reports DXL power enabled. It then
acquires the existing maintenance lock, opens a distinct observer session,
retains every incoming delimited frame before sending `HIL_CAPTURE`, and sends
both `HIL_HEARTBEAT` and the separately scoped `MAINT_HEARTBEAT` every 200 ms
until the terminal trace event arrives. It closes the observer and releases
the maintenance lock on completion or host-side failure.

The generated session directory contains incoming raw frames in
`raw_frames.bin`, decoded trace records in `telemetry.jsonl`, and observer
lifecycle records in `events.jsonl` (open, requested capture, completion, or
host error). The raw-frame tap is inbound-only; lifecycle events are an audit
trail, not a byte-for-byte outbound serial transcript.

`hexapod-cli hil-decode <session-dir>` is offline: it never opens a serial
port. It strictly reassembles retained `0xA0` and `0xA1` events, validates
their fragment ordering and CRCs, and writes `hil_trace.json`. Binary
`ConfigSnapshot` data is represented deterministically as `payload_hex`; no
Python byte representation or text decoding is used. A non-`COMPLETE`
`TraceEnd` still produces an inspectable diagnostic artifact, but the command
returns nonzero so it cannot be mistaken for parity evidence.

The JSON artifact contains the canonical controller inputs and observations
needed to construct a native `controller::parity::Trace`, but it does not yet
run `replayAndCompare()` itself. Physical HIL closure still requires a native
fixture importer/replay, retained outbound-command evidence where required,
and independent power-rail and DXL-bus measurement.

### Precedence and revocation

| Condition | Required outcome |
| --- | --- |
| Host `ESTOP` or RC kill/failsafe | Existing safety path wins, maintenance lock is revoked, output remains disabled, and the trace records the resulting controller step when transport permits. |
| Host `SET_ARMING(disarm)` | Existing force-disarm path wins and clears maintenance/passive requests. The observer can continue only as a monitor. |
| HIL heartbeat expiry | Abort trace and remove observer capability only; never grant or remove normal motion authority. |
| Maintenance heartbeat expiry | Existing lock expiry path wins. A HIL session may observe it but cannot refresh it. |
| Jetson heartbeat expiry | Existing 250 ms Jetson authority expiry applies. HIL or generic heartbeats cannot substitute for it. |
| CRC, framing, or authorization error | Reject the request, increment existing link diagnostics where applicable, and do not mutate controller or output-guard state. |

## Immutable Output-Disabled HIL Build

`hexapod_src-4ju.18` implements a distinct PlatformIO build profile, for
example `openrb150_hil_output_disabled`, with a compile-time output-disable
macro. The normal `openrb150` build remains behaviorally unchanged.

In the HIL build, the desired controller policy is still calculated and
recorded, but the final adapter boundary must enforce all of the following:

| Requested operation | HIL behavior |
| --- | --- |
| DXL power enable | Do not assert `BDPIN_DXL_PWR_EN`; retain power-off state and increment `blocked_power_enable`. |
| Torque enable | Do not send torque-on; retain reported torque-off state and increment `blocked_torque_enable`. |
| Goal Sync Write | Do not call the DXL goal-write path; record the bounded command that would have been sent and increment `blocked_goal_write`. |
| DXL torque maintenance job | Reject torque-on with an explicit output-disabled result. Torque-off remains safety-reducing and may be a no-op. |
| DXL read-only status work | May be supported only if it never enables DXL power or torque. Unsupported read work must fail explicitly rather than silently changing power policy. |

The implemented `GET_CAPABILITIES` build bit and `GET_STATUS`/`health`
`hil_flags` report `build_output_disabled`, `power_guard_active`,
`torque_guard_active`, `goal_guard_active`, and `write_guard_active`.
`GET_STATUS` and `hil_status` also publish cumulative blocked-operation
counters. `hil_status` retains the exact bounded targets from the most recent
blocked Sync Write. During a trace capture, changed blocked-operation counters
also produce an ordered `HIL_OUTPUT_BLOCKED` event carrying the canonical
`OutputBlocked` logical record.

There is no `HIL_ENABLE_OUTPUT`, no configuration bit, and no session command
that can relax these guards. A build with `build_output_disabled = false`
rejects every HIL command with `NotAvailable`.

## Allowed Stimuli and Explicit Rejections

HIL captures real task scheduling and normal controller adapters. For a
no-hardware motion scenario, the host can retain the normal maintenance lock,
select `MaintControlMode::GaitPipeline`, then send the existing bounded
high-level motion APIs. The controller still emits only calibrated,
ServoMap-limited goals; the output guard records and blocks their final DXL
side effect.

The contract deliberately rejects these proposed interfaces:

| Rejected interface | Reason |
| --- | --- |
| `HIL_SET_GOAL_TICKS`, `HIL_SET_JOINTS`, or a raw ROS joint command | It would bypass gait, IK, ServoMap travel limits, and normal command arbitration. |
| Direct `ControllerCore::step()` RPC | It would bypass task timing, adapter ownership, and the MCU safety boundary. |
| Host-controlled DXL power or torque flag | It could turn a trace harness into an actuator authority. |
| Generic-heartbeat authority refresh | It weakens the distinct maintenance and Jetson liveness policies. |
| Arbitrary raw sensor/RC replacement in the initial HIL protocol | It can hide the adapter behavior that HIL is intended to test. Simulated contact inputs are owned by `hexapod_src-4ju.16` and must remain clearly labeled simulated. |

An optional future feedback fixture may supply only bounded *observed* DXL
present positions to an output-disabled adapter. If added, it must accept
unique configured IDs and ticks in `[0, 4095]`, derive coverage and config
revision from firmware-owned configuration, expire within 100 ms, and never
appear in a normal build. It is not required for the initial maintenance-gait
trace workflow and is not a raw goal command.

## Trace Capture Contract V1

### Capture control and transport budget

`HIL_CAPTURE` requests `1..32` consecutive controller steps. The one-slot
recorder first delivers `TraceBegin` and `ConfigSnapshot`; its requested step
sequence starts only after those mandatory setup records have drained. From
that point, it records every actual call to `ControllerCore::step()` at the
existing 100 Hz cadence. It does not downsample, coalesce steps, or substitute
wall-clock timestamps. Only one capture may be active. A capture start is
rate-limited to one per second and is rejected when the previous capture did
not finish cleanly.

During an active capture, regular noncritical subscribed telemetry is
temporarily suppressed while retaining its subscription state. The firmware
retains subscribed `health` and `hil_status` streams so a stalled host remains
visible. Trace records have transport priority. At capture end, prior
subscriptions resume and the terminal record reports the completed record and
fragment counts.

The recorder uses fixed-size static storage, never allocates, never blocks
`controlTask`, and never writes USB from `controlTask`. If its bounded handoff
queue, event fragment encoder, or USB transport cannot preserve the complete
ordered capture, it aborts with `TransportOverflow` or `TransportTimeout`.
It must not silently drop or decimate a step. An incomplete capture is invalid
for controller parity even if its individual fragments decode.

Each on-wire event payload is at most 256 bytes. One logical record is split
into ordered `HIL_TRACE_FRAGMENT` events as needed. The common fragment prefix
is little-endian:

| Field | Size | Meaning |
| --- | ---: | --- |
| `trace_schema_version` | 2 | `1` for this contract. |
| `session_id` | 4 | Observer session that produced the record. |
| `capture_id` | 4 | Monotonic nonzero capture identifier. |
| `record_seq` | 4 | Monotonic record identifier within the capture. |
| `record_type` | 1 | Begin, config, step, marker, output-blocked, or end. |
| `fragment_index` | 1 | Zero-based fragment number. |
| `fragment_count` | 1 | Total fragments for this logical record. |
| `logical_length` | 2 | Length after reassembly, before record CRC. |
| `logical_crc16` | 2 | CRC-16/CCITT-FALSE of the reassembled logical bytes. |
| `fragment_bytes` | variable | Ordered slice of the logical record. |

The frame CRC protects every fragment; `logical_crc16` protects host
reassembly. The host rejects duplicate, missing, out-of-order, wrong-session,
or failed-logical-CRC records. It persists raw delimited frames before decoding
them. `OutputBlocked` records use event ID `0xA1`; all other record types use
`0xA0`. Both use this same prefix and reassembly rule.

### Logical records

All logical records use canonical little-endian scalar encoding. They never
serialize C++ object memory, padding, pointers, enums by width assumption, or
floating-point display strings. Boolean values are explicit bytes, and floating
values use IEEE-754 binary32 bits in the same field order as the portable
contracts.

| Record type | Required content |
| --- | --- |
| `TraceBegin` | Trace schema version, origin `Atsamd21OutputDisabled`, firmware protocol major/minor, session/capture IDs, active config revision, config payload CRC-16, requested step count, and initial output-guard counters. |
| `ConfigSnapshot` | Config revision, `valid` and `persistent` flags, serialized `RobotConfig` bytes from `serializeRobotConfig`, and its CRC-16. It appears before any step referencing that revision. |
| `ControllerStep` | One complete `ControllerStepInput` followed by the `CommandObservation` immediately returned by that same `ControllerCore::step()` call. Includes monotonically increasing controller-step sequence and the adapter-provided `now_ms`/`dt_ms`. |
| `Marker` | Host marker ID, receipt time, current safety state, and the associated controller-step sequence. A one-slot recorder emits this immediately after its associated step; markers contain no free-form text. |
| `OutputBlocked` | Output guard operation and aggregate counter snapshot. It is evidence of an attempted blocked side effect, not evidence that a physical bus stayed unpowered. |
| `TraceEnd` | Completion or abort reason, expected/recorded step counts, emitted logical-record and fragment counts before `TraceEnd`, queue high-water mark, and final output-guard counters. |

The `ControllerStep` input is field-for-field canonical with the value
contracts in `controller_contract.h`:

- `ControllerTime`: `now_ms`, `dt_ms`, and `valid`.
- `RobotState`: battery value/validity; every `DxlSnapshot` scalar and each
  `ServoStatus` scalar; contact mask/validity and every `LegContactState`
  scalar; `config_ready`; and `watchdog_fault`.
- `ControllerIntent`: full decoded `ControllerCommand` plus RC flags; full
  `MotionIntent`; maintenance lock, control mode, and all `MaintTargetSet`
  fields; feature flags; host E-stop/disarm; clear-fault edge; passive request;
  and Jetson-heartbeat edge.
- `ControllerConfigSnapshot`: revision, validity, and persistence flags. The
  associated `RobotConfig` value is resolved from the preceding
  `ConfigSnapshot` record with matching revision and CRC.

The output portion is canonical with `controller::parity::CommandObservation`:
safety state, fault reason, command source, authority and policy booleans,
every `ControllerDiagnostics` scalar, goal count, and every ordered
`PipelineJoint` field (`id`, `tick`, `leg`, `joint`, `clamped`). A native
decoder reconstructs the version-1 `Trace` fixture before calling
`replayAndCompare()`; no float display rounding or servo-angle conversion is
allowed between target capture and native comparison.

The serial schema may de-duplicate unchanged config records, but it may not
omit a `ControllerStep` field or substitute a later telemetry value. The HIL
adapter rejects config staging and commit while a capture is active, and the
recorder fail-closes if it nevertheless sees a revision change. Any schema
change adds a new trace schema version and is rejected by a version-1 decoder.

## HIL Capture Procedure and Required Evidence

The future board procedure is intentionally staged. It must save the raw
serial capture, decoded trace, configuration payload, command transcript, and
firmware artifact identity together.

1. Run native firmware tests and build both the normal target and the explicit
   output-disabled HIL target. The normal target must retain its existing
   behavior.
2. Flash only the output-disabled image. Use `GET_CAPABILITIES`, `GET_STATUS`,
   and `tools/hil_smoke.py --hil-output-disabled` to retain the response
   showing all guards active before an observer session is opened.
3. Prove the physical output boundary separately from software counters: record
   DXL power-enable state at boot and through a command-producing capture,
   verify no torque-enable or Sync Write packet is emitted, and retain the
   instrument trace or equivalent board-level evidence. Keep the DXL power
   feed safely isolated for the first execution.
4. Open the HIL observer session under a maintenance lock. The companion
   workflow is `hexapod-cli hil-capture --steps N --out data/sessions`; it uses
   only the existing high-level protocol. For a command-producing scenario,
   select maintenance gait-pipeline mode with the existing command APIs and
   add numeric host markers before each scenario boundary.
5. Capture a bounded sequence with zero fragment loss, no queue overflow, and
   a `TraceEnd` record whose recorded count equals the requested count. Retain
   blocked-operation counters when a command-producing scenario was used.
6. Run `hexapod-cli hil-decode <session-dir>` to retain the validated JSON
   parity artifact, then import it into the native parity fixture and run
   `replayAndCompare()`. State, fault, authority, goal validity, diagnostics,
   and every calibrated goal tick must match exactly. The first mismatch is a
   failure report with capture ID, controller-step sequence, field, and goal
   index where applicable.
7. Do not treat a native synthetic trace, a ROS SIL test, an output-blocked
   counter alone, or a decoded partial serial log as physical HIL completion.
   `hexapod_src-4ju.19` closes only after a real ATSAMD21 output-disabled
   capture satisfies all evidence above.

## Implementation Boundaries

`hexapod_src-4ju.18` owns the implemented compile-time output guard,
output-block instrumentation, capability/status visibility, `hil_status`
telemetry, and the smoke harness. `hexapod_src-4ju.21` owns the implemented
observer command handler, fixed-memory recorder, fragment codec, API-owned
event emission, malformed-token and capacity checks, reassembly coverage, and
the embedded size check. It does not constitute hardware proof.

`hexapod_src-4ju.16` owns explicitly simulated contact/IMU inputs for ROS SIL;
those inputs must be labeled simulated and must not silently become HIL sensor
truth. `hexapod_src-4ju.19` owns board execution, host raw-frame retention,
native trace reconstruction, and exact parity comparison. The normal
controller, serial framing, and actuator safety boundary remain single-owner
firmware responsibilities throughout.