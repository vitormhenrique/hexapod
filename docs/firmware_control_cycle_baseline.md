# Firmware Control-Cycle Baseline

Status: observed implementation baseline for `hexapod_src-4ju.1`.

This document describes the current OpenRB-150 firmware behavior before the
controller orchestration is extracted for native simulation, ROS 2, or
hardware-in-the-loop use. It does not change timing, gains, calibration, servo
limits, pin assignments, or safety policy.

## Scope and Ownership

`firmware/openrb150/src/main.cpp` performs board initialization and calls
`app::start()`. `app::start()` creates the FreeRTOS tasks once at boot, then
starts the scheduler. `firmware/openrb150/src/app/tasks.cpp` is the current
orchestration boundary.

| Task | Period | Priority | Sole peripheral or control ownership |
| --- | ---: | ---: | --- |
| `controlTask` | 10 ms / 100 Hz | 3 | Command arbitration, safety transition, battery ADC read, gait/IK/servo-map goal generation |
| `dxlTask` | 20 ms / 50 Hz | 3 | `Serial1`, DYNAMIXEL discovery, power/torque sequencing, Sync Write, status reads |
| `rcTask` | 10 ms / 100 Hz | 3 | `Serial3`, CRSF parsing, controller bridge snapshots |
| `apiTask` | 5 ms / 200 Hz | 2 | USB CDC framing, host command APIs, telemetry transmission |
| `i2cTask` | 5-100 ms / nominal 20 ms | 1 | `Wire`, mux/sensor reads, EEPROM transactions, contact estimator updates |
| `healthTask` | 500 ms / 2 Hz | 4 | Watchdog evaluation |
| `blinkTask` | 100 ms / 5 Hz | 1 | User LED liveness indication |

Every periodic task uses `vTaskDelayUntil()` and rebases after an overrun where
needed. `controlTask` and `dxlTask` deliberately remain separate at the same
priority: the former determines an eligible command, and the latter remains the
final actuator gate.

## One Motion Cycle

The observed data flow for an active motion cycle is:

```text
RC / USB host / Jetson heartbeat / I2C / DXL status
  -> published task snapshots
  -> controlTask: authority arbitration
  -> controlTask: StateMachine update
  -> motion gate = stateAllowsMotion && authority
  -> GaitPipeline: gait -> body IK -> leg IK -> ServoMap
  -> GoalFrame publication under g_goalMutex
  -> dxlTask: power, torque, and goal-write enforcement
  -> DxlBus Sync Write and status snapshot publication
```

### `controlTask` order

On each 10 ms cycle, `controlTask`:

1. Derives `now_ms` from the FreeRTOS tick count and forwards an allowed Jetson
   heartbeat to `g_arbiter`.
2. Reads the RC snapshot, host disarm/estop state, and maintenance lock state;
   refreshes the `CommandArbiter` output.
3. Reads battery voltage as the sole ADC owner and builds `StateInputs` from
   the latest configuration, DXL, RC, contact, watchdog, passive, and
   maintenance snapshots.
4. Advances `g_stateMachine`, publishes the current state/fault, and revokes
   maintenance/passive requests on fault or E-stop.
5. Calculates `g_motionGate` as the conjunction of a motion-capable safety
   state and authorized command source.
6. When the gate is open, selects RC or host intent, reconfigures the pipeline
   when the config revision changes, applies gait/body/twist parameters,
   handles trick state, seeds goals from measured positions on a gate rising
   edge, then calls `g_pipeline.update(10 ms, goals)`.
7. Publishes the bounded `PipelineOutput` into `g_goalFrame` under
   `g_goalMutex`. When the gate closes, invalidates `g_goalValid`, cancels a
   running trick, and forces gait/intent reseeding on the next authorization.
8. Publishes maintenance/DXL-job eligibility and feature availability for the
   API task, then delays to the next period.

Mac maintenance has a separate mode within the same cycle. Fresh maintenance
authority begins by clearing stored targets; in joint-target mode only explicitly
set targets are copied into `g_goalFrame`. Unset joints are not commanded and
later hold their measured torque-on seed position.

### `dxlTask` order

On each 20 ms cycle, `dxlTask`:

1. Uses the published safety state to run the ArmingChecks discovery sequence:
   power on, wait 500 ms, discover configured IDs, and establish fresh pose
   evidence.
2. Executes at most one queued DXL maintenance job and refreshes readiness
   evidence.
3. Cuts DXL power when `stateAllowsDxlPower()` is false.
4. Uses the published `g_motionGate` to disable torque on a falling edge.
   On a non-maintenance rising edge it first Sync Writes measured hold targets,
   then enables torque only if all configured servos acknowledge.
5. Publishes torque-off confirmation for passive-mode safety gating.
6. Copies a valid `g_goalFrame` with a bounded mutex wait, then writes its
   ticks only while authorized, torque is ready, and a servo table exists.
7. Performs read-only status work on a powered, scanned bus: present-position
   Sync Read plus one round-robin detailed servo read. It publishes readiness,
   pose, torque, and hard-fault snapshots for the next control cycles.
8. Converts sustained failed reads, hardware error bits, or failed torque
   enablement into the published DXL hard-fault input.

The 50 Hz task is intentionally capable of rejecting a stale or otherwise
ineligible goal even if `controlTask` generated it previously.

## Stateful Components and Cross-Task Snapshots

The following items participate in one control cycle. Their current ownership
is part of the behavior that future extraction must preserve.

| Item | Owner | Consumers | Current role |
| --- | --- | --- | --- |
| `g_arbiter` | `controlTask` | `dxlTask` through published authority/gate | Selects RC, Jetson, or Mac maintenance authority and kill state |
| `g_stateMachine` | `controlTask` | `dxlTask`, API, telemetry | Authoritative safety mode/fault transition state |
| `g_pipeline` | `controlTask` | `dxlTask` through `GoalFrame` | Stateful gait phase, body/leg IK, and ServoMap goal production |
| `g_trickEngine` | `controlTask` | `StateInputs` and pipeline selection | Timed body/twist choreography; cancelled when the gate closes |
| `g_contact` | `i2cTask` | `controlTask`, API, telemetry | Portable contact estimator whose published feet contribute confidence evidence |
| `g_bridge` | `rcTask` | `controlTask`, API | Decodes RC channels into controller command and safety-related RC state |
| `g_configApi` and config revision | `apiTask` for edits; `i2cTask` for EEPROM commit | `controlTask`, `dxlTask`, telemetry | Live validated RAM configuration and calibration source |
| `g_rcStatus`, `g_ctrlCmd` | `rcTask` | `controlTask`, API, telemetry | Latest RC/bridge input snapshot |
| `g_controlApi`, `g_motionApi`, `g_maintApi`, `g_maintTargetApi`, `g_dxlJobApi`, `g_featureApi`, `g_passiveApi` | `apiTask` | `controlTask`, `dxlTask` as applicable | Validated host intent, locks, maintenance work, and feature requests |
| I2C topology and foot snapshots | `i2cTask` | `controlTask`, API, telemetry | Config readiness, sensor presence, raw/fused contact data |
| Servo status/readiness snapshots | `dxlTask` | `controlTask`, API, telemetry | Present pose, coverage, torque, and hard-fault evidence |
| `g_goalFrame`, `g_goalValid`, `g_goalSeq`, `g_goalClamped` | `controlTask` | `dxlTask`, API telemetry | Latest gated servo target frame, protected by `g_goalMutex` |
| `g_batteryMv` | `controlTask` | API, telemetry | Latest ADC voltage; no other task calls `analogRead()` |

`controlTask` also retains cycle history in function-local static state:
applied motion-intent sequence, applied gait, applied configuration revision,
previous motion-gate state, previous maintenance-authority state, previous RC
trick trigger, and idle-activity intent sequence. `dxlTask` retains discovery
progress, authorization edge state, torque-seed/fault state, DXL failure count,
and round-robin servo index. These are controller semantics, not incidental
scratch variables, and must become explicit owned state before a portable step
can reproduce behavior deterministically.

## Current Sources of Truth

| Concern | Source of truth | Enforcement point |
| --- | --- | --- |
| Safety state and fault reason | `g_stateMachine` output published as `g_safetyState` and `g_faultReason` | `controlTask` transition; `dxlTask` power/torque enforcement |
| Command authority and kill state | `g_arbiter` output, informed by RC and the external maintenance lock | `controlTask`; published gate is consumed by `dxlTask` |
| Maintenance lock lifetime | `g_maintApi` token/TTL | `controlTask` mirrors it into the arbiter and state inputs |
| Active robot configuration | `g_configApi.config()` and revision | Pipeline reconfiguration, servo map, DXL configured-ID checks |
| Latest physical joint evidence | `g_servoStatus` published by `dxlTask` | Arming checks, goal slew seeding, hold-target generation |
| Actuable target | Valid `g_goalFrame` | `dxlTask` copies and Sync Writes only after its own safety checks |

## Gates, Limits, and Clamp Points

1. Protocol APIs and `ControllerBridge` range-check and store intent before the
   control cycle consumes it.
2. The arbiter selects one high-level authority and reports kill/authorization.
3. The safety state machine decides whether a state permits DXL power, torque,
   and motion. `g_motionGate` requires both a motion-capable state and command
   authority.
4. The arming path additionally requires current configuration, a valid battery
   reading, all configured servo IDs, fresh present positions, and no DXL hard
   fault.
5. `GaitPipeline` constrains gait inputs, applies body and leg IK, flags
   reachability, applies ServoMap travel limits, and records per-joint clamp
   flags in `PipelineJoint`.
6. Goal slew is seeded from actual present positions on an authorization edge;
   a closed gate invalidates the target frame rather than retaining stale motion.
7. `dxlTask` is the final physical gate: it can cut power, disable torque,
   require a hold-target seed before torque enable, skip missing frames, and
   reject output when the bus is not ready.

## Existing Native Coverage

`platformio.ini` environment `native` builds Arduino-free sources only. The
following existing Unity suites cover the portable pieces that feed the current
cycle:

- Authority and safety: `test_command_arbiter`, `test_state_machine`, and
  `test_watchdog`.
- RC and host intent: `test_crsf`, `test_controller_bridge`, `test_control_api`,
  `test_motion_api`, `test_maintenance_api`, `test_maintenance_target_api`,
  `test_feature_api`, `test_sensor_api`, and `test_passive_api`.
- Configuration and mapping: `test_config_schema`, `test_config_api`,
  `test_config_store`, `test_servo_map`, and `test_hold_targets`.
- Motion generation: `test_leg_ik`, `test_gait_engine`, `test_gait_pipeline`,
  and `test_trick_engine`.
- Contact and DXL-adjacent portable logic: `test_contact_estimator`,
  `test_dxl_model`, `test_dxl_sync`, `test_dxl_params`, and `test_dxl_job_api`.

These suites exercise component behavior but do not execute the FreeRTOS task
interleaving or the complete `controlTask` to `dxlTask` path. Future
ControllerCore regression and replay tests must establish that cross-component
equivalence.

## Extraction Constraints

The later portable boundary must accept explicit state, intent, configuration,
and time; emit bounded goal/diagnostic output; and own the controller-state
listed above. It must not own `Serial1`, `Wire`, ADC, EEPROM transactions,
FreeRTOS delays/mutexes, power FET control, torque sequencing, or USB framing.
Those remain adapter responsibilities until parity evidence demonstrates an
intentional behavior change.