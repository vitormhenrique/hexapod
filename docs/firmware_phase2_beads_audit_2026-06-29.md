# Firmware Phase 2 Beads Audit - 2026-06-29

Beads epic audited: `hexapod_src-22l` - Phase 2 firmware motion, sensors, config API, and telemetry.

Verification run during this audit:

- `cd firmware/openrb150 && ~/.platformio/penv/bin/pio test -e native` - 291/291 native tests passed.
- `~/.platformio/penv/bin/pio run -e openrb150` - OpenRB-150 target build succeeded, RAM 75.5%, flash 26.4%.
- Physical HIL was not run. `hexapod_src-22l.11` is still in progress and explicitly documented as pending.

Overall conclusion: the Phase 2 epic is not 100% end-to-end complete. Most closed child tasks have strong portable modules and tests, but several task-level integrations are still partial: gait output is not wired to DXL goal writes, DXL hard faults are not fed into the safety machine, Jetson heartbeat authority is not wired, contact-terrain inputs are forced false, sensor rate commands are not consumed, and HIL sign-off is pending.

## hexapod_src-22l.1 - Define persistent robot config schema

Audit status: implemented and correctly covered by native tests for the stated schema acceptance criteria.

Possible enhancements:

- Strengthen `validateRobotConfig()` to reject unknown `feature_defaults` bits and unsafe persisted gait defaults. `feature_defaults` is serialized/deserialized but not range-checked at [firmware/openrb150/src/config/config_schema.cpp:235](../firmware/openrb150/src/config/config_schema.cpp#L235), while validation starts at [firmware/openrb150/src/config/config_schema.cpp:240](../firmware/openrb150/src/config/config_schema.cpp#L240) and only checks body height/gait enum plus servo invariants around [firmware/openrb150/src/config/config_schema.cpp:250](../firmware/openrb150/src/config/config_schema.cpp#L250).
- Validate enabled foot-sensor calibration values. The schema stores `near_thresh`, `touch_thresh`, `load_thresh`, and `enabled` at [firmware/openrb150/src/config/config_schema.cpp:229-L232](../firmware/openrb150/src/config/config_schema.cpp#L229-L232), but validation does not reject enabled sensors with zero or inconsistent thresholds. A good rule would be `enabled => near/touch/load configured and load >= touch`, with project-specific calibration tolerances.

## hexapod_src-22l.2 - Implement config API over USB protocol

Audit status: core CFG_* protocol, RAM shadow, validation, and transactional commit are implemented and tested. Runtime adoption/commit propagation is partial.

Possible enhancements:

- Re-apply adopted or committed config to runtime consumers. `apiTask` adopts persisted config at [firmware/openrb150/src/app/tasks.cpp:1041](../firmware/openrb150/src/app/tasks.cpp#L1041), and CFG_COMMIT reaches EEPROM at [firmware/openrb150/src/app/tasks.cpp:1220](../firmware/openrb150/src/app/tasks.cpp#L1220), but `i2cTask` seeds contact calibration and motion defaults from compiled defaults at [firmware/openrb150/src/app/tasks.cpp:1170-L1175](../firmware/openrb150/src/app/tasks.cpp#L1170-L1175). The contact estimator, motion defaults, and feature defaults should be refreshed from the active config after boot adoption and after successful commit.
- Apply `RobotConfig.feature_defaults` to `FeatureApi`. The config schema has `feature_defaults` at [firmware/openrb150/src/config/config_schema.cpp:132](../firmware/openrb150/src/config/config_schema.cpp#L132), but `FeatureApi` uses compiled defaults from [firmware/openrb150/src/protocol/feature_api.h:87](../firmware/openrb150/src/protocol/feature_api.h#L87). That means persisted feature defaults currently do not drive runtime desired state.

## hexapod_src-22l.3 - Implement servo map and joint limit enforcement

Audit status: implemented and correctly covered by native tests. The conversion path applies sign, trim, min/max, and device clamp flags.

Possible enhancements:

- Consider precomputed lookup tables for high-rate consumers. `ServoMap::servoFor()` and `ServoMap::servoForId()` linearly scan all servos at [firmware/openrb150/src/dxl/servo_map.cpp:8](../firmware/openrb150/src/dxl/servo_map.cpp#L8) and [firmware/openrb150/src/dxl/servo_map.cpp:16](../firmware/openrb150/src/dxl/servo_map.cpp#L16). Eighteen servos is small, so this is not urgent, but a cached slot/id map would reduce repeated scans in telemetry, maintenance targets, and future gait loops.
- Add end-to-end clamp telemetry tests once task-level DXL writes are wired. `ServoMap::angleToTick()` is the right final gate at [firmware/openrb150/src/dxl/servo_map.cpp:23](../firmware/openrb150/src/dxl/servo_map.cpp#L23), but the current full robot goal-write path is not yet active, so clamp flags are not HIL-proven through actual bus commands.

## hexapod_src-22l.4 - Implement 3-DOF leg IK and body pose transform

Audit status: implemented and correctly covered by native tests for nominal, edge, and unreachable targets.

Possible enhancements:

- Move hard-coded geometric calibration constants into persisted config or a calibration document once hardware measurements are available. `kHomeRadiusMm` and `kHomeFootZMm` are fixed in [firmware/openrb150/src/gait/leg_ik.h:27](../firmware/openrb150/src/gait/leg_ik.h#L27), and `kCoxaLiftMm` is fixed in [firmware/openrb150/src/gait/body_ik.h:35](../firmware/openrb150/src/gait/body_ik.h#L35). That is fine for the current reference model, but Phase 2 HIL may need measured offsets.
- Add HIL-backed regression cases for the final calibrated geometry after `hexapod_src-22l.11`, especially because the default stance is known to sit close to the reach boundary on some legs.

## hexapod_src-22l.5 - Implement gait engine v1

Audit status: portable gait engine is implemented and tested, but the robot-level walking path is not end-to-end implemented.

Possible enhancements:

- Instantiate and consume `GaitEngine` in `controlTask`. `MotionApi` only stores intent by design at [firmware/openrb150/src/protocol/motion_api.h:13](../firmware/openrb150/src/protocol/motion_api.h#L13). The current task code also documents the gait goal source as future work at [firmware/openrb150/src/app/tasks.cpp:398](../firmware/openrb150/src/app/tasks.cpp#L398). The missing integration is: `MotionApi.intent()` -> `GaitEngine.update()` -> `BodyKinematics` -> `ServoMap` -> target queue for `dxlTask`.
- Add reachability-aware stride limiting or a safer default stance before ground tests. The gait engine clamps generated target boxes, but practical reach margin is small for the documented home stance. The stroke vector is generated around [firmware/openrb150/src/gait/gait_engine.cpp:145](../firmware/openrb150/src/gait/gait_engine.cpp#L145); this is a good place to add per-leg stride scaling or a config-backed retracted stance.

## hexapod_src-22l.6 - Implement DXL Sync Write and status telemetry

Audit status: driver-level sync write/status code exists and tests pass. Full 18-servo task-level goal writing and all-servo status refresh need work.

Possible enhancements:

- Wire `DxlBus::writeGoalPositions()` into `dxlTask` under `g_motionGate`. The function is implemented at [firmware/openrb150/src/dxl/dxl_bus.cpp:169](../firmware/openrb150/src/dxl/dxl_bus.cpp#L169), but `dxlTask` currently says task-level activation is still future/gated at [firmware/openrb150/src/app/tasks.cpp:958-L961](../firmware/openrb150/src/app/tasks.cpp#L958-L961), and there is no source call site outside the definition. This means accepted gait/maintenance targets are not actually sent as DXL goal positions yet.
- Chunk `syncReadStatus()` for buses with more than `DXL_MAX_NODE` servos of the same table kind. The current sync-read collection stops at `DXL_MAX_NODE` at [firmware/openrb150/src/dxl/dxl_bus.cpp:231](../firmware/openrb150/src/dxl/dxl_bus.cpp#L231) and performs one `syncRead` per table at [firmware/openrb150/src/dxl/dxl_bus.cpp:242](../firmware/openrb150/src/dxl/dxl_bus.cpp#L242). On an all-legacy 18-servo bus, this can refresh only 16 present positions per cycle.
- Derive passive torque-off confirmation from actual servo torque state, not only motion authorization. `g_dxlTorqueOff` is inferred at [firmware/openrb150/src/app/tasks.cpp:953](../firmware/openrb150/src/app/tasks.cpp#L953). If a maintenance DXL_TORQUE job turns torque on while motion is otherwise unauthorized, passive entry can be told torque is off even though the bus may still have torque enabled.

## hexapod_src-22l.7 - Implement Robotic Finger Sensor v2 polling

Audit status: I2C discovery, bounded one-channel polling, raw telemetry, and the portable contact estimator are implemented. Runtime rate/config/contact enable behavior is partial.

Possible enhancements:

- Consume `SENSOR_SET_RATE` in `i2cTask`. The API stores rate intent at [firmware/openrb150/src/protocol/sensor_api.cpp:253](../firmware/openrb150/src/protocol/sensor_api.cpp#L253) and [firmware/openrb150/src/protocol/sensor_api.cpp:269](../firmware/openrb150/src/protocol/sensor_api.cpp#L269), and exposes it at [firmware/openrb150/src/protocol/sensor_api.h:155-L156](../firmware/openrb150/src/protocol/sensor_api.h#L155-L156). The task still runs at the fixed 50 Hz period from [firmware/openrb150/src/app/task_config.h:49](../firmware/openrb150/src/app/task_config.h#L49) and delays at [firmware/openrb150/src/app/tasks.cpp:1312](../firmware/openrb150/src/app/tasks.cpp#L1312).
- Make calibration enable classification or stage a config update. Default foot calibrations are disabled at [firmware/openrb150/src/config/config_schema.cpp:128](../firmware/openrb150/src/config/config_schema.cpp#L128). `ContactEstimator` copies that enabled flag at [firmware/openrb150/src/sensors/contact_estimator.cpp:22](../firmware/openrb150/src/sensors/contact_estimator.cpp#L22), and disabled feet always stay `Air` at [firmware/openrb150/src/sensors/contact_estimator.cpp:131-L132](../firmware/openrb150/src/sensors/contact_estimator.cpp#L131-L132). `CONTACT_CALIBRATE` currently captures baseline at [firmware/openrb150/src/app/tasks.cpp:1272](../firmware/openrb150/src/app/tasks.cpp#L1272), but does not set the foot enabled flag or persist thresholds.
- Mark mux select failures as sensor failures. The polling path only calls `g_contact.update()` after `selectChannel(ch)` succeeds at [firmware/openrb150/src/app/tasks.cpp:1287](../firmware/openrb150/src/app/tasks.cpp#L1287) and [firmware/openrb150/src/app/tasks.cpp:1300](../firmware/openrb150/src/app/tasks.cpp#L1300). Feeding a failed sample when channel selection fails would let fault counters and telemetry reflect mux/runtime I2C problems faster.

## hexapod_src-22l.8 - Implement RC Jetson Mac command arbitration

Audit status: RC and Mac maintenance authority rules are implemented in portable modules; Jetson authority and maintenance target actuation are not end-to-end wired.

Possible enhancements:

- Wire a real Jetson heartbeat/control source into the arbiter. `CommandArbiter::jetsonHeartbeat()` exists at [firmware/openrb150/src/safety/command_arbiter.cpp:23](../firmware/openrb150/src/safety/command_arbiter.cpp#L23), and `controlTask` checks freshness at [firmware/openrb150/src/app/tasks.cpp:561](../firmware/openrb150/src/app/tasks.cpp#L561), but no firmware source calls `jetsonHeartbeat()`. `Feature::JetsonControl` is also reported unavailable/not implemented at [firmware/openrb150/src/app/tasks.cpp:506](../firmware/openrb150/src/app/tasks.cpp#L506).
- Consolidate the unused `CommandArbiter` Mac lock APIs with `MaintenanceApi` or remove the unused path. `CommandArbiter` still has Mac lock methods at [firmware/openrb150/src/safety/command_arbiter.cpp:39](../firmware/openrb150/src/safety/command_arbiter.cpp#L39), [firmware/openrb150/src/safety/command_arbiter.cpp:53](../firmware/openrb150/src/safety/command_arbiter.cpp#L53), and [firmware/openrb150/src/safety/command_arbiter.cpp:64](../firmware/openrb150/src/safety/command_arbiter.cpp#L64), while the actual API path uses `MaintenanceApi`.
- Drive stored maintenance targets to DXL goals once `22l.6` is integrated. `MaintTargetApi` documents that control/dxl tasks consume stored ticks at [firmware/openrb150/src/protocol/maintenance_target_api.h:20](../firmware/openrb150/src/protocol/maintenance_target_api.h#L20), but the active task-level DXL write path is still missing.

## hexapod_src-22l.9 - Implement rate-limited telemetry subscriptions

Audit status: implemented and correctly covered by native tests. Subscribe/unsubscribe, rate clamps, dropped counters, and backlog counters are present.

Possible enhancements:

- Add coherent snapshot protection or sequence counters for multi-field telemetry records if HIL shows torn snapshots. `buildTelemetry()` reads `g_servoStatusCount` at [firmware/openrb150/src/app/tasks.cpp:302](../firmware/openrb150/src/app/tasks.cpp#L302) and then walks `g_servoStatus` at [firmware/openrb150/src/app/tasks.cpp:304](../firmware/openrb150/src/app/tasks.cpp#L304) while `dxlTask` is the writer. The current approach is probably acceptable for bring-up, but a lightweight copy/sequence guard would improve record consistency.
- Add integration tests for emitted payload builders, not only the subscription manager. Current native tests cover `SubscriptionManager`, but the concrete payload encoders in `buildTelemetry()` are mostly target-task code.

## hexapod_src-22l.10 - Complete firmware safety state machine

Audit status: portable state machine is implemented and tested. Runtime integration leaves several safety inputs stubbed or too shallow.

Possible enhancements:

- Expand arming checks beyond config readiness. `arming_checks_pass` is currently just `g_configReady` at [firmware/openrb150/src/app/tasks.cpp:558](../firmware/openrb150/src/app/tasks.cpp#L558). Phase 2 safety should include at least battery validity, DXL scan/servo profile coverage, sane present pose, and no active DXL/I2C hard fault before `StandReady`.
- Feed real DXL hard faults into the state machine. `si.dxl_hard_fault` is hard-coded false at [firmware/openrb150/src/app/tasks.cpp:552](../firmware/openrb150/src/app/tasks.cpp#L552), so `FaultHard` cannot be triggered by repeated bus failures or servo hardware errors in live firmware even though the portable state machine supports it.
- Wire contact feature/confidence into `ContactTerrain`. `si.contact_enabled` and `si.contact_confident` are forced false at [firmware/openrb150/src/app/tasks.cpp:572-L573](../firmware/openrb150/src/app/tasks.cpp#L572-L573), so the tested `ContactTerrain` transition is unreachable in the integrated task loop.
- Improve torque state truth for passive mode. The state machine expects `torque_off` to mean all servo torque is confirmed off at [firmware/openrb150/src/safety/state_machine.h:73](../firmware/openrb150/src/safety/state_machine.h#L73), but `dxlTask` currently derives it from authorization at [firmware/openrb150/src/app/tasks.cpp:953](../firmware/openrb150/src/app/tasks.cpp#L953). This should be backed by actual DXL torque state/status.

## hexapod_src-22l.11 - Run Phase 2 bench suspended and ground tests

Audit status: not complete. The issue is correctly in progress, and only the procedure is documented.

Possible enhancements:

- Execute and record the HIL sign-off before closing the epic. The checklist explicitly says physical execution is pending at [tools/hil_phase2.md:3](../tools/hil_phase2.md#L3) and says to close `hexapod_src-22l.11` only after all sections pass at [tools/hil_phase2.md:128](../tools/hil_phase2.md#L128).
- Add a small machine-readable sign-off artifact or script output for HIL results, so future audits can distinguish procedure text from completed hardware evidence.

## Recommended follow-up order

1. Wire gait/maintenance target generation into a bounded DXL goal-write path (`22l.5`, `22l.6`, `22l.8`).
2. Fix DXL status/torque truth: chunk sync reads, derive hard faults, and confirm torque-off from actual servo state (`22l.6`, `22l.10`).
3. Propagate active config to runtime consumers and feature defaults after boot adoption/commit (`22l.2`, `22l.7`).
4. Consume sensor-rate intent and make calibration enable/persist contact classification (`22l.7`).
5. Wire Jetson heartbeat/control only after the core RC/manual path is moving safely (`22l.8`).
6. Run and record `22l.11` HIL sign-off before treating the Phase 2 epic as complete.

# Firmware Phase 1 Beads Audit - 2026-06-29 Append

Beads epic audited: `hexapod_src-rbg` - Phase 1 firmware foundation and safe I/O. The epic is closed with all ten children complete.

Verification run during this append:

- Existing firmware verification from this audit still applies: `pio test -e native` passed 291/291 and `pio run -e openrb150` succeeded.
- `PYTHONPATH=protocol/python python3 tools/hil_smoke.py --list` failed under system Python because `pyserial` is not installed.
- `cd companion && PYTHONPATH=../protocol/python uv run python ../tools/hil_smoke.py --list` succeeded and listed serial ports.

Working-robot verdict: Phase 1 is correctly implemented as a safe boot and I/O foundation, but Phase 1 plus current Phase 2 is still not enough for a working walking hexapod. The firmware should boot, answer USB, parse RC, scan I2C, and expose tested protocol/config pieces. It should not be expected to walk safely yet because no task-level gait-to-DXL goal-write loop exists, the RC gait switch is only decoded/telemetry-visible, DXL hard faults are not live safety inputs, DXL power enable is not exposed through a complete safe command path, and physical HIL motion sign-off is still pending.

## hexapod_src-rbg.1 - Create monorepo skeleton and firmware build target

Audit status: implemented. The custom OpenRB-150 PlatformIO environment and README commands exist, and the target build succeeded in this audit.

Possible enhancements:

- Keep the MKR Zero environment visibly marked as compile-only. The PlatformIO file correctly says `[env:openrb150]` is the real hardware mapping and `[env:mkrzero]` has wrong serial/DXL pins at [firmware/openrb150/platformio.ini:24-L29](../firmware/openrb150/platformio.ini#L24-L29). That warning is important because testing motion on the fallback env would invalidate DXL/CRSF assumptions.
- Update the firmware README layout once task files are split out. It still describes the early skeleton layout at [firmware/openrb150/README.md:42-L49](../firmware/openrb150/README.md#L42-L49), while the repo now has many Phase 1/2 modules under `src/`.

## hexapod_src-rbg.2 - Implement board HAL and safe boot defaults

Audit status: implemented for safe boot. DXL power is forced off during board init, and battery/status hooks exist.

Possible enhancements:

- Calibrate the battery divider before relying on low-voltage safety. The divider is explicitly provisional at [firmware/openrb150/src/board/board.h:28](../firmware/openrb150/src/board/board.h#L28), and pack voltage is computed from that fixed ratio at [firmware/openrb150/src/board/board.cpp:71](../firmware/openrb150/src/board/board.cpp#L71).
- Expose a complete, safety-gated DXL power command path. The HAL has `setDxlPower()` at [firmware/openrb150/src/board/board.cpp:45](../firmware/openrb150/src/board/board.cpp#L45), but no source outside the HAL currently calls it. DXL maintenance jobs check `board::dxlPowerEnabled()` at [firmware/openrb150/src/app/tasks.cpp:706](../firmware/openrb150/src/app/tasks.cpp#L706), so a robot with firmware-only control may never get past `PowerOff` without an added enable path or external bench step.

## hexapod_src-rbg.3 - Bring up FreeRTOS task skeleton

Audit status: implemented for task bring-up. Control, DXL, RC, API, I2C, health, and blink tasks are created at [firmware/openrb150/src/app/tasks.cpp:1359-L1374](../firmware/openrb150/src/app/tasks.cpp#L1359-L1374).

Possible enhancements:

- Replace the software-watchdog-only stub with real SAMD21 hardware WDT behavior before free walking. The current watchdog code records stalled tasks, but the hardware WDT pet/reset is still a TODO at [firmware/openrb150/src/safety/watchdog.cpp:42](../firmware/openrb150/src/safety/watchdog.cpp#L42).
- Add automated stack-margin assertions once HIL logs exist. Stack sizes are centralized in [firmware/openrb150/src/app/task_config.h](../firmware/openrb150/src/app/task_config.h), but the current automated tests do not enforce minimum high-water marks from hardware runs.

## hexapod_src-rbg.4 - Implement protocol framing golden vectors

Audit status: implemented and covered by native/Python tests. COBS, CRC16, frame headers, and corrupt-frame rejection are in place.

Possible enhancements:

- Add explicit version-mismatch behavior to host and firmware integration tests if protocol major version changes. The frame layer is versioned, but current runtime API handling mostly drops undecodable frames rather than surfacing richer diagnostics.
- Preserve golden-vector discipline for every new command group. Later Phase 2/companion work has done this for many groups; keep treating protocol additions as public API changes.

## hexapod_src-rbg.5 - Implement USB API v0

Audit status: implemented for Phase 1 session/status commands. USB CDC command handling is exclusive to `apiTask`, and HELLO/HEARTBEAT/GET_STATUS/GET_CAPABILITIES are implemented.

Possible enhancements:

- Populate capability bits from real runtime capabilities. `GET_CAPABILITIES` returns `feature_bits`, but initialization still sets it to zero at [firmware/openrb150/src/app/tasks.cpp:243](../firmware/openrb150/src/app/tasks.cpp#L243). Feature state is richer through later APIs, but Phase 1 capabilities remain under-informative.
- Add automated HIL assertions for state progression beyond boot API responses. The smoke script checks safe boot and uptime, but deeper I/O checks are manual or Phase 2-dependent.

## hexapod_src-rbg.6 - Implement DYNAMIXEL bus manager maintenance scan

Audit status: driver-level scan/ping/status profile logic is implemented. It is not enough by itself to produce walking motion.

Possible enhancements:

- Add a safe DXL power enable and scan workflow that can run from the normal API. `DxlBus` begins in `dxlTask` at [firmware/openrb150/src/app/tasks.cpp:925](../firmware/openrb150/src/app/tasks.cpp#L925), but scans require DXL power and Phase 1 intentionally deferred scan commands. Current DXL jobs can report `PowerOff` when `board::dxlPowerEnabled()` is false at [firmware/openrb150/src/app/tasks.cpp:706](../firmware/openrb150/src/app/tasks.cpp#L706).
- Do not treat bus-manager completion as robot motion readiness. Goal writing exists only as a driver method at [firmware/openrb150/src/dxl/dxl_bus.cpp:169](../firmware/openrb150/src/dxl/dxl_bus.cpp#L169), with no task-level walking call site yet.

## hexapod_src-rbg.7 - Implement TCA9548A and root I2C scan

Audit status: implemented for root/mux/channel discovery. I2C ownership and one-hot mux selection are correct for safe bring-up.

Possible enhancements:

- Add runtime recovery and fault surfacing around mux select failures. The scan/polling layer counts mux errors, but the high-level safety machine does not yet consume I2C confidence as a feature/fault input.
- Treat the scan as topology discovery, not contact-aware walking readiness. Sensor presence is necessary but not enough; contact calibration, rate handling, terrain inputs, and gait adaptation remain Phase 2+ integration gaps.

## hexapod_src-rbg.8 - Implement 24LC32 page driver and transactional config store

Audit status: implemented at storage/transaction level and covered by native tests. EEPROM I/O is bounded and page-aware.

Possible enhancements:

- Confirm 24LC32 write timing under actual hardware and keep the result with HIL evidence. The driver ACK-polls with a bounded loop in [firmware/openrb150/src/config/eeprom_24lc32.cpp:15-L24](../firmware/openrb150/src/config/eeprom_24lc32.cpp#L15-L24), which is fine in the I2C task but still needs physical confirmation over repeated commits.
- Connect persistence evidence to runtime config adoption. The lower-level store works, but the Phase 2 audit notes that adopted/committed config is not fully propagated to contact, motion, and feature runtime consumers.

## hexapod_src-rbg.9 - Implement CRSF parser and RC failsafe status

Audit status: parser and failsafe logic are implemented and tested. RC input does not yet select or execute walking gaits end to end.

Possible enhancements:

- Wire `gait_index` into the actual motion/gait controller. CRSF maps the gait switch at [firmware/openrb150/src/input/crsf_parser.h:83](../firmware/openrb150/src/input/crsf_parser.h#L83), stores a three-position `gait_index` at [firmware/openrb150/src/input/crsf_parser.h:93](../firmware/openrb150/src/input/crsf_parser.h#L93), and publishes it in RC telemetry at [firmware/openrb150/src/app/tasks.cpp:344](../firmware/openrb150/src/app/tasks.cpp#L344). The integrated control loop does not consume it to select stand/sit/tripod/ripple/wave/crawl.
- Expand RC command mapping for body twist once walking is wired. The parser normalizes all channels and maps arm/kill/gait/autonomy, but `controlTask` currently only feeds arm/kill/autonomy into the arbiter and safety machine.

## hexapod_src-rbg.10 - Create Phase 1 hardware-in-loop smoke tests

Audit status: script and checklist exist; automated coverage is USB-only. This is useful but not proof that the full robot works.

Possible enhancements:

- Record completed physical HIL evidence, not just the checklist. The Phase 1 checklist says everything needing physical interaction or 12 V power is manual at [docs/hil_smoke_phase1.md:8](../docs/hil_smoke_phase1.md#L8), and explicitly states no servo torque/no motion in Phase 1 at [docs/hil_smoke_phase1.md:12](../docs/hil_smoke_phase1.md#L12).
- Make the smoke script dependency path obvious. Bare system Python failed here because `pyserial` was missing, while the documented `uv` environment succeeded. The script itself says deeper DXL/I2C/EEPROM/CRSF checks need physical interaction or Phase 2 API commands at [tools/hil_smoke.py:13](../tools/hil_smoke.py#L13).
- Do not close walking readiness from Phase 1 HIL. The checklist’s DXL step depends on a Phase 2 command or bench scan path at [docs/hil_smoke_phase1.md:63-L64](../docs/hil_smoke_phase1.md#L63-L64), so it is not a standalone proof of Phase 1 bus control.

## Walking Gaits And Real-Work Readiness

Different walking gaits are implemented in the portable Phase 2 `GaitEngine`: stand/sit are static, and tripod/ripple/wave/crawl generate bounded foot targets. The implementation entry point is [firmware/openrb150/src/gait/gait_engine.cpp:103](../firmware/openrb150/src/gait/gait_engine.cpp#L103), and the native tests passed in this audit.

However, the real robot walking loop is not implemented end to end. The current blockers are:

- `GaitEngine` is not instantiated/consumed by `controlTask`; the Phase 2 report already points to the missing intent -> gait -> IK -> servo target queue.
- `DxlBus::writeGoalPositions()` is not called by `dxlTask`, so generated or stored goal ticks are not sent to servos.
- RC `gait_index` is decoded but not used to select the active gait.
- Arming checks do not verify DXL scan/servo coverage or present pose; they are currently `g_configReady` only at [firmware/openrb150/src/app/tasks.cpp:558](../firmware/openrb150/src/app/tasks.cpp#L558).
- DXL hard faults are not live inputs to the safety machine at [firmware/openrb150/src/app/tasks.cpp:552](../firmware/openrb150/src/app/tasks.cpp#L552).
- Contact terrain mode is unreachable in the integrated loop because contact inputs are forced false at [firmware/openrb150/src/app/tasks.cpp:572](../firmware/openrb150/src/app/tasks.cpp#L572).

Conclusion: the codebase has real, tested foundational work, and the gait math itself exists. It does not yet have the complete real work needed for a walking robot. The next fix should still be the bounded task-level goal pipeline: RC/Mac/Jetson intent -> gait selection/twist -> gait engine -> body/leg IK -> servo map -> DXL sync write -> status/fault feedback -> safety state machine.
