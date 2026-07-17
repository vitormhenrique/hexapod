#pragma once

// ===========================================================================
// Fixed-size ControllerCore replay format (hexapod_src-4ju.11).
//
// Test-only, transport-independent input/output records for deterministic
// controller replays. Fixtures capture fully decoded contract snapshots; raw
// serial, ROS, Arduino, and hardware time sources remain outside this format.
// ===========================================================================

#include "../../src/controller/controller_core.h"

namespace controller {
namespace replay {

constexpr uint16_t kFormatVersion = 1;
constexpr uint8_t kMaxFrames = 16;

enum class TimeUnit : uint8_t {
  Milliseconds = 1,
};

struct FixtureHeader {
  uint16_t format_version = kFormatVersion;
  TimeUnit time_unit = TimeUnit::Milliseconds;
  uint8_t frame_count = 0;
};

struct ExpectedOutput {
  safety::State safety_state = safety::State::Boot;
  safety::FaultReason fault_reason = safety::FaultReason::None;
  safety::CommandSource command_source = safety::CommandSource::None;
  uint32_t config_revision = 0;
  uint8_t goal_count = 0;
  bool motion_gate = false;
  bool goal_valid = false;
  bool allow_torque = false;
  bool gate_rising = false;
  bool gate_falling = false;
  bool require_goal_change = false;
};

struct ReplayFrame {
  ControllerTime time{};
  RobotState state{};
  ControllerIntent intent{};
  ControllerConfigSnapshot config{};
  ExpectedOutput expected{};
};

struct Fixture {
  FixtureHeader header{};
  ReplayFrame frames[kMaxFrames] = {};
};

enum class Failure : uint8_t {
  None = 0,
  HeaderVersion,
  HeaderTimeUnit,
  FrameCount,
  SafetyState,
  FaultReason,
  CommandSource,
  ConfigRevision,
  MotionGate,
  GoalValidity,
  TorquePolicy,
  GoalCount,
  GateRising,
  GateFalling,
  GoalUnchanged,
};

struct Result {
  bool passed = false;
  uint8_t frame_index = 0xFF;
  Failure failure = Failure::None;
};

inline bool goalsChanged(const gait::PipelineOutput& previous,
                         const gait::PipelineOutput& current) {
  if (previous.count != current.count) return true;
  for (uint8_t index = 0; index < current.count; ++index) {
    if (previous.joints[index].id != current.joints[index].id ||
        previous.joints[index].tick != current.joints[index].tick ||
        previous.joints[index].clamped != current.joints[index].clamped) {
      return true;
    }
  }
  return false;
}

inline Result run(const Fixture& fixture) {
  Result result;
  if (fixture.header.format_version != kFormatVersion) {
    result.failure = Failure::HeaderVersion;
    return result;
  }
  if (fixture.header.time_unit != TimeUnit::Milliseconds) {
    result.failure = Failure::HeaderTimeUnit;
    return result;
  }
  if (fixture.header.frame_count == 0 ||
      fixture.header.frame_count > kMaxFrames) {
    result.failure = Failure::FrameCount;
    return result;
  }

  ControllerCore core;
  RobotCommand command;
  gait::PipelineOutput previous_goals;
  bool have_previous_goals = false;
  for (uint8_t index = 0; index < fixture.header.frame_count; ++index) {
    const ReplayFrame& frame = fixture.frames[index];
    const ExpectedOutput& expected = frame.expected;
    core.step(frame.state, frame.intent, frame.config, frame.time, command);

    result.frame_index = index;
    if (command.safety_state != expected.safety_state) {
      result.failure = Failure::SafetyState;
      return result;
    }
    if (command.fault_reason != expected.fault_reason) {
      result.failure = Failure::FaultReason;
      return result;
    }
    if (command.command_source != expected.command_source) {
      result.failure = Failure::CommandSource;
      return result;
    }
    if (command.diagnostics.config_revision != expected.config_revision) {
      result.failure = Failure::ConfigRevision;
      return result;
    }
    if (command.motion_gate != expected.motion_gate) {
      result.failure = Failure::MotionGate;
      return result;
    }
    if (command.goal_valid != expected.goal_valid) {
      result.failure = Failure::GoalValidity;
      return result;
    }
    if (command.allow_torque != expected.allow_torque) {
      result.failure = Failure::TorquePolicy;
      return result;
    }
    if (command.goals.count != expected.goal_count) {
      result.failure = Failure::GoalCount;
      return result;
    }
    if (command.diagnostics.motion_gate_rising != expected.gate_rising) {
      result.failure = Failure::GateRising;
      return result;
    }
    if (command.diagnostics.motion_gate_falling != expected.gate_falling) {
      result.failure = Failure::GateFalling;
      return result;
    }
    if (expected.require_goal_change &&
        (!have_previous_goals || !goalsChanged(previous_goals, command.goals))) {
      result.failure = Failure::GoalUnchanged;
      return result;
    }

    previous_goals = command.goals;
    have_previous_goals = true;
  }

  result.passed = true;
  result.frame_index = fixture.header.frame_count - 1;
  result.failure = Failure::None;
  return result;
}

}  // namespace replay
}  // namespace controller