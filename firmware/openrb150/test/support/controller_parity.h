#pragma once

// ===========================================================================
// ControllerCore native-to-target trace parity support (hexapod_src-4ju.12).
//
// This test-only format records the exact portable inputs passed to one
// ControllerCore step and the command observed directly after that step. It is
// deliberately transport-neutral: the later HIL protocol work is responsible
// for serial framing, while this runner owns the native semantic comparison.
// ===========================================================================

#include "../../src/controller/controller_core.h"
#include "../../src/controller/controller_time.h"

namespace controller {
namespace parity {

constexpr uint16_t kFormatVersion = 1;
constexpr uint8_t kMaxFrames = 32;

enum class CaptureOrigin : uint8_t {
  Synthetic = 0,
  Atsamd21OutputDisabled = 1,
  Atsamd21Bench = 2,
};

struct TraceHeader {
  uint16_t format_version = kFormatVersion;
  CaptureOrigin origin = CaptureOrigin::Synthetic;
  uint8_t frame_count = 0;
  uint16_t nominal_period_ms = 10;
};

// A fully scalar view of RobotCommand. The future target capture point is
// immediately after ControllerCore::step() and before publishControllerCommand
// applies adapter-owned lock, target, power, torque, or DXL side effects.
struct CommandObservation {
  safety::State safety_state = safety::State::Boot;
  safety::FaultReason fault_reason = safety::FaultReason::None;
  safety::CommandSource command_source = safety::CommandSource::None;
  bool motion_authorized = false;
  bool motion_gate = false;
  bool allow_dxl_power = false;
  bool allow_torque = false;
  bool goal_valid = false;
  ControllerDiagnostics diagnostics{};
  uint8_t goal_count = 0;
  gait::PipelineJoint goals[config::kNumServos] = {};
};

struct TraceFrame {
  ControllerStepInput input{};
  CommandObservation observed{};
};

struct Trace {
  TraceHeader header{};
  TraceFrame frames[kMaxFrames] = {};
};

enum class Failure : uint8_t {
  None = 0,
  HeaderVersion,
  HeaderOrigin,
  HeaderFrameCount,
  HeaderPeriod,
  InputTiming,
  SafetyState,
  FaultReason,
  CommandSource,
  MotionAuthorization,
  MotionGate,
  DxlPowerPolicy,
  TorquePolicy,
  GoalValidity,
  ConfigRevision,
  IntentSequence,
  ContactConfidence,
  GateRising,
  GateFalling,
  ConfigReapplied,
  MaintenanceSession,
  ClearMaintenanceTargets,
  ClearMaintenanceLock,
  ClearPassiveRequest,
  GoalClamped,
  GoalUnreachable,
  GoalReachLimited,
  GoalCountRange,
  GoalCount,
  GoalId,
  GoalTick,
  GoalLeg,
  GoalJoint,
  GoalClamp,
};

struct Result {
  bool passed = false;
  uint8_t frame_index = 0xFF;
  uint8_t goal_index = 0xFF;
  Failure failure = Failure::None;
};

inline CommandObservation capture(const RobotCommand& command) {
  CommandObservation observed;
  observed.safety_state = command.safety_state;
  observed.fault_reason = command.fault_reason;
  observed.command_source = command.command_source;
  observed.motion_authorized = command.motion_authorized;
  observed.motion_gate = command.motion_gate;
  observed.allow_dxl_power = command.allow_dxl_power;
  observed.allow_torque = command.allow_torque;
  observed.goal_valid = command.goal_valid;
  observed.diagnostics = command.diagnostics;
  observed.goal_count = command.goals.count;
  const uint8_t count = command.goals.count > config::kNumServos
                            ? config::kNumServos
                            : command.goals.count;
  for (uint8_t index = 0; index < count; ++index) {
    observed.goals[index] = command.goals.joints[index];
  }
  return observed;
}

inline Failure compare(const RobotCommand& expected,
                       const CommandObservation& observed,
                       uint8_t& goal_index) {
  if (expected.safety_state != observed.safety_state) {
    return Failure::SafetyState;
  }
  if (expected.fault_reason != observed.fault_reason) {
    return Failure::FaultReason;
  }
  if (expected.command_source != observed.command_source) {
    return Failure::CommandSource;
  }
  if (expected.motion_authorized != observed.motion_authorized) {
    return Failure::MotionAuthorization;
  }
  if (expected.motion_gate != observed.motion_gate) {
    return Failure::MotionGate;
  }
  if (expected.allow_dxl_power != observed.allow_dxl_power) {
    return Failure::DxlPowerPolicy;
  }
  if (expected.allow_torque != observed.allow_torque) {
    return Failure::TorquePolicy;
  }
  if (expected.goal_valid != observed.goal_valid) {
    return Failure::GoalValidity;
  }

  const ControllerDiagnostics& a = expected.diagnostics;
  const ControllerDiagnostics& b = observed.diagnostics;
  if (a.config_revision != b.config_revision) return Failure::ConfigRevision;
  if (a.intent_sequence != b.intent_sequence) return Failure::IntentSequence;
  if (a.confident_contact_feet != b.confident_contact_feet) {
    return Failure::ContactConfidence;
  }
  if (a.motion_gate_rising != b.motion_gate_rising) {
    return Failure::GateRising;
  }
  if (a.motion_gate_falling != b.motion_gate_falling) {
    return Failure::GateFalling;
  }
  if (a.config_reapplied != b.config_reapplied) {
    return Failure::ConfigReapplied;
  }
  if (a.maintenance_session_started != b.maintenance_session_started) {
    return Failure::MaintenanceSession;
  }
  if (a.clear_maintenance_targets != b.clear_maintenance_targets) {
    return Failure::ClearMaintenanceTargets;
  }
  if (a.clear_maintenance_lock != b.clear_maintenance_lock) {
    return Failure::ClearMaintenanceLock;
  }
  if (a.clear_passive_request != b.clear_passive_request) {
    return Failure::ClearPassiveRequest;
  }
  if (a.any_goal_clamped != b.any_goal_clamped) {
    return Failure::GoalClamped;
  }
  if (a.any_goal_unreachable != b.any_goal_unreachable) {
    return Failure::GoalUnreachable;
  }
  if (a.any_goal_reach_limited != b.any_goal_reach_limited) {
    return Failure::GoalReachLimited;
  }

  if (observed.goal_count > config::kNumServos ||
      expected.goals.count > config::kNumServos) {
    return Failure::GoalCountRange;
  }
  if (expected.goals.count != observed.goal_count) {
    return Failure::GoalCount;
  }
  for (uint8_t index = 0; index < expected.goals.count; ++index) {
    goal_index = index;
    const gait::PipelineJoint& a_goal = expected.goals.joints[index];
    const gait::PipelineJoint& b_goal = observed.goals[index];
    if (a_goal.id != b_goal.id) return Failure::GoalId;
    if (a_goal.tick != b_goal.tick) return Failure::GoalTick;
    if (a_goal.leg != b_goal.leg) return Failure::GoalLeg;
    if (a_goal.joint != b_goal.joint) return Failure::GoalJoint;
    if (a_goal.clamped != b_goal.clamped) return Failure::GoalClamp;
  }
  goal_index = 0xFF;
  return Failure::None;
}

inline Result replayAndCompare(const Trace& trace) {
  Result result;
  if (trace.header.format_version != kFormatVersion) {
    result.failure = Failure::HeaderVersion;
    return result;
  }
  if (trace.header.origin != CaptureOrigin::Synthetic &&
      trace.header.origin != CaptureOrigin::Atsamd21OutputDisabled &&
      trace.header.origin != CaptureOrigin::Atsamd21Bench) {
    result.failure = Failure::HeaderOrigin;
    return result;
  }
  if (trace.header.frame_count == 0 ||
      trace.header.frame_count > kMaxFrames) {
    result.failure = Failure::HeaderFrameCount;
    return result;
  }
  if (trace.header.nominal_period_ms == 0) {
    result.failure = Failure::HeaderPeriod;
    return result;
  }

  ControllerCore core;
  RobotCommand expected;
  for (uint8_t index = 0; index < trace.header.frame_count; ++index) {
    const TraceFrame& frame = trace.frames[index];
    result.frame_index = index;
    if (frame.input.time.valid &&
        frame.input.time.dt_ms > kDefaultMaxControllerElapsedMs) {
      result.failure = Failure::InputTiming;
      return result;
    }

    core.step(frame.input.state, frame.input.intent, frame.input.config,
              frame.input.time, expected);
    result.failure = compare(expected, frame.observed, result.goal_index);
    if (result.failure != Failure::None) return result;
  }

  result.passed = true;
  result.frame_index = trace.header.frame_count - 1;
  return result;
}

}  // namespace parity
}  // namespace controller