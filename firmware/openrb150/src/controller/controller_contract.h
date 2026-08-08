#pragma once

// ===========================================================================
// Portable ControllerCore contract (hexapod_src-4ju.2).
//
// This header defines the value snapshots exchanged between hardware/ROS
// adapters and the future deterministic ControllerCore. It is deliberately
// Arduino-free, heap-free, and wire-format-free. All arrays have compile-time
// capacities from config_schema.h.
//
// Planned step API:
//
//   void step(const RobotState& state, const ControllerIntent& intent,
//             const ControllerConfigSnapshot& config,
//             const ControllerTime& time, RobotCommand& command);
//
// `step()` will not mutate or retain aliases to any input. An adapter owns
// snapshot collection and actuator I/O; the core owns only portable controller
// state. Identical input values and controller state must produce identical
// command values.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "../dxl/dxl_status.h"
#include "../gait/gait_pipeline.h"
#include "../input/controller_bridge.h"
#include "../protocol/maintenance_target_api.h"
#include "../protocol/motion_api.h"
#include "../safety/command_arbiter.h"
#include "../safety/state_machine.h"
#include "../sensors/contact_estimator.h"

namespace controller {

// Freshness is explicit at adapter boundaries. Individual DXL entries retain
// ServoStatus::ok and individual contact entries retain their state/stale/fault
// flags; this value describes the aggregate snapshot delivered to one step.
enum class SnapshotValidity : uint8_t {
  Unknown = 0,
  Fresh = 1,
  Stale = 2,
  Fault = 3,
};

// Monotonic controller time. `now_ms` and `dt_ms` use unsigned milliseconds;
// `valid` is false until an adapter has established a trustworthy monotonic
// timeline. `overrun` marks a valid elapsed interval that exceeded the
// adapter's nominal control period. The core must fail closed for invalid time
// instead of inferring a clock from FreeRTOS, ROS, or wall time.
struct ControllerTime {
  uint32_t now_ms = 0;
  uint32_t dt_ms = 0;
  bool valid = false;
  bool overrun = false;
};

struct BatterySnapshot {
  uint16_t millivolts = 0;
  bool valid = false;
  SnapshotValidity validity = SnapshotValidity::Unknown;
};

// Status acquired from the DXL adapter. `configured_servo_coverage`,
// `pose_known_mask`, and `config_revision` preserve the current arming
// evidence contract: only status read against the active config can authorize
// a normal arming path.
struct DxlSnapshot {
  dxl::ServoStatus servos[config::kNumServos] = {};
  uint8_t servo_count = 0;
  SnapshotValidity validity = SnapshotValidity::Unknown;
  bool configured_servo_coverage = false;
  uint32_t pose_known_mask = 0;
  uint32_t config_revision = 0;
  bool torque_off = true;
  bool hard_fault = false;
};

struct ContactSnapshot {
  sensors::LegContactState feet[sensors::kNumFeet] = {};
  uint8_t present_mask = 0;
  SnapshotValidity validity = SnapshotValidity::Unknown;
};

// Normalized RC state, already decoded by the adapter. ControllerCommand
// retains its validated twist, pose, shape, and trick fields; the separate
// safety fields preserve the existing RcStatus semantics without exposing a
// UART parser to the core.
struct RcIntent {
  controller::ControllerCommand command{};
  bool ever_seen = false;
  bool kill = true;
  bool armed = false;
  bool failsafe = true;
  bool autonomy_enabled = false;
};

// Effective feature state after the adapter has combined a user's request with
// hardware availability. The core consumes this snapshot rather than a
// FeatureApi object, so feature availability remains an adapter/API concern.
struct FeatureSnapshot {
  bool foot_contact_enabled = false;
  bool terrain_leveling_enabled = false;
  bool sensor_polling_enabled = false;
  bool jetson_control_enabled = false;
  bool passive_pose_enabled = false;
};

// Host-maintenance intent is already range-checked by MaintTargetApi. The
// stored targets are final DXL ticks produced by its shared IK + ServoMap path,
// never raw host-provided ticks.
struct MaintenanceIntent {
  bool lock_held = false;
  uint32_t lock_token = 0;
  protocol::MaintControlMode control_mode =
      protocol::MaintControlMode::JointTargets;
  protocol::MaintTargetSet targets{};
};

// Validated non-physical command input for one controller step. A received
// Jetson heartbeat and a clear-fault request are edge events: the adapter sets
// them for exactly the step that consumes them. Other fields are latest-value
// snapshots and may be reused for multiple steps.
struct ControllerIntent {
  RcIntent rc{};
  protocol::MotionIntent motion{};
  MaintenanceIntent maintenance{};
  FeatureSnapshot features{};
  bool host_estop = false;
  bool host_disarm = false;
  bool clear_fault_requested = false;
  bool passive_requested = false;
  bool jetson_heartbeat_received = false;
};

// Immutable physical/health snapshot. The adapter owns all peripheral access
// and must populate this before invoking the core. `config_ready` means that
// a persisted configuration or compiled safe fallback was adopted; it does not
// imply the configuration is persistent.
struct RobotState {
  BatterySnapshot battery{};
  DxlSnapshot dxl{};
  ContactSnapshot contact{};
  bool config_ready = false;
  bool watchdog_fault = false;
};

// Full fixed-size configuration and calibration value for an input step. A
// value copy makes the snapshot self-contained and replayable. Adapters may
// reuse a static instance between steps, but must replace it atomically when
// `revision` changes. `persistent` reports whether storage-backed commits are
// currently available, while `valid` means the value passed schema validation.
struct ControllerConfigSnapshot {
  config::RobotConfig robot{};
  uint32_t revision = 0;
  bool valid = false;
  bool persistent = false;
};

// Output-only diagnostics. These flags describe the result of the current
// decision and never request hardware I/O directly.
struct ControllerDiagnostics {
  uint32_t config_revision = 0;
  uint32_t intent_sequence = 0;
  uint8_t confident_contact_feet = 0;
  // Gait shape actually commanded this step, after clamping. Adapters publish
  // this on the CRSF downlink and persist it when `gait_save_requested` is set,
  // so the handset, the log, and persisted config can never disagree.
  uint16_t applied_body_height_mm = 0;
  uint16_t applied_stride_mm = 0;
  uint16_t applied_step_height_mm = 0;
  uint8_t applied_duty_x255 = 0;
  uint8_t applied_speed_x255 = 0;
  // RC gait-tune editor state (controller::GaitTuneParam value).
  uint8_t gait_tune_param = 0;
  bool gait_tune_active = false;
  // The single-leg preview is running (operator is watching a parameter).
  bool gait_tune_preview = false;
  // The operator asked to persist the applied gait shape. The adapter owns the
  // storage transaction and applies `gait_save_seq` de-duplication: it acts only
  // when the sequence differs from the last one it persisted.
  bool gait_save_requested = false;
  uint32_t gait_save_seq = 0;
  bool motion_gate_rising = false;
  bool motion_gate_falling = false;
  bool config_reapplied = false;
  bool maintenance_session_started = false;
  bool clear_maintenance_targets = false;
  bool clear_maintenance_lock = false;
  bool clear_passive_request = false;
  bool any_goal_clamped = false;
  bool any_goal_unreachable = false;
  bool any_goal_reach_limited = false;
};

// Controller decision for one step. `allow_dxl_power` and `allow_torque` are
// state-policy outputs, not direct GPIO/torque commands. The target adapter
// owns DXL power sequencing, hold-position seeding, torque writes, Sync Write,
// and final actuator fault handling.
//
// `goals` intentionally contains only final calibrated DXL ticks. GaitPipeline
// owns angle-to-tick mapping and ServoMap owns travel clamps, so a second
// joint-angle command representation cannot bypass or duplicate those limits.
// ROS and HIL adapters may derive display/transport angles from this same
// configuration snapshot, but must not replace these actuator goals.
struct RobotCommand {
  safety::State safety_state = safety::State::Boot;
  safety::FaultReason fault_reason = safety::FaultReason::None;
  safety::CommandSource command_source = safety::CommandSource::None;
  bool motion_authorized = false;
  bool motion_gate = false;
  bool allow_dxl_power = false;
  bool allow_torque = false;
  bool goal_valid = false;
  gait::PipelineOutput goals{};
  ControllerDiagnostics diagnostics{};
};

// Convenience value object for deterministic replays and native tests. The
// future core API accepts the four members as const references to avoid a
// per-step copy on the MCU; this aggregate is optional transport/test storage.
struct ControllerStepInput {
  RobotState state{};
  ControllerIntent intent{};
  ControllerConfigSnapshot config{};
  ControllerTime time{};
};

}  // namespace controller