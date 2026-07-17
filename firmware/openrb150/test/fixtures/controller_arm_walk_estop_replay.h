#pragma once

#include "../support/controller_replay.h"

namespace controller {
namespace replay {
namespace fixtures {

inline RobotState readyState(const ControllerConfigSnapshot& config_snapshot) {
  RobotState state;
  state.config_ready = true;
  state.battery.millivolts = 12000;
  state.battery.valid = true;
  state.battery.validity = SnapshotValidity::Fresh;
  state.dxl.servo_count = config::kNumServos;
  state.dxl.validity = SnapshotValidity::Fresh;
  state.dxl.configured_servo_coverage = true;
  state.dxl.config_revision = config_snapshot.revision;
  state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
  state.dxl.torque_off = true;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    state.dxl.servos[index].id = config_snapshot.robot.servos[index].id;
    state.dxl.servos[index].present_position = config::kServoCenterTick;
    state.dxl.servos[index].ok = true;
  }
  return state;
}

inline ControllerIntent safeRcIntent() {
  ControllerIntent intent;
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  intent.rc.ever_seen = true;
  intent.rc.command.valid = true;
  intent.rc.command.body_height = 0.5f;
  intent.rc.command.speed = 0.5f;
  intent.rc.command.stride = 0.5f;
  intent.rc.command.step_height = 0.5f;
  intent.motion.duty_x255 = 128;
  return intent;
}

inline ExpectedOutput expected(safety::State state,
                               safety::CommandSource source,
                               bool motion_gate, bool goal_valid,
                               bool allow_torque, uint8_t goal_count,
                               bool gate_rising = false,
                               bool gate_falling = false,
                               bool require_goal_change = false) {
  ExpectedOutput output;
  output.safety_state = state;
  output.command_source = source;
  output.config_revision = 31;
  output.motion_gate = motion_gate;
  output.goal_valid = goal_valid;
  output.allow_torque = allow_torque;
  output.goal_count = goal_count;
  output.gate_rising = gate_rising;
  output.gate_falling = gate_falling;
  output.require_goal_change = require_goal_change;
  return output;
}

inline const Fixture& armWalkEstop() {
  static const Fixture fixture = [] {
    Fixture out;
    out.header.format_version = kFormatVersion;
    out.header.time_unit = TimeUnit::Milliseconds;
    out.header.frame_count = 9;

    ControllerConfigSnapshot config_snapshot;
    config::defaultRobotConfig(config_snapshot.robot);
    config_snapshot.revision = 31;
    config_snapshot.valid = config::validateRobotConfig(config_snapshot.robot);
    config_snapshot.persistent = false;
    const RobotState state = readyState(config_snapshot);
    const ControllerIntent base_intent = safeRcIntent();

    for (uint8_t index = 0; index < out.header.frame_count; ++index) {
      ReplayFrame& frame = out.frames[index];
      frame.time.now_ms = static_cast<uint32_t>(index + 1) * 10;
      frame.time.dt_ms = 10;
      frame.time.valid = true;
      frame.state = state;
      frame.intent = base_intent;
      frame.config = config_snapshot;
    }

    out.frames[0].expected = expected(safety::State::ConfigLoad,
                                      safety::CommandSource::None, false,
                                      false, false, 0);
    out.frames[1].expected = expected(safety::State::Disarmed,
                                      safety::CommandSource::None, false,
                                      false, false, 0);
    out.frames[2].expected = expected(safety::State::Disarmed,
                                      safety::CommandSource::None, false,
                                      false, false, 0);

    out.frames[3].intent.rc.armed = true;
    out.frames[3].expected = expected(safety::State::ArmingChecks,
                                      safety::CommandSource::Rc, false,
                                      false, false, 0);
    out.frames[4].intent.rc.armed = true;
    out.frames[4].expected = expected(safety::State::StandReady,
                                      safety::CommandSource::Rc, false,
                                      false, true, 0);
    out.frames[5].intent.rc.armed = true;
    out.frames[5].expected = expected(safety::State::RcManual,
                                      safety::CommandSource::Rc, true,
                                      true, true, config::kNumServos,
                                      true);
    out.frames[6].intent.rc.armed = true;
    out.frames[6].intent.rc.command.gait_index = 1;
    out.frames[6].intent.rc.command.twist_vx = 0.65f;
    out.frames[6].expected = expected(safety::State::RcManual,
                                      safety::CommandSource::Rc, true,
                                      true, true, config::kNumServos,
                                      false, false, true);
    out.frames[7].intent.rc.armed = false;
    out.frames[7].expected = expected(safety::State::Disarmed,
                                      safety::CommandSource::None, false,
                                      false, false, 0, false, true);
    out.frames[8].intent.host_estop = true;
    out.frames[8].expected = expected(safety::State::Estop,
                                      safety::CommandSource::None, false,
                                      false, false, 0);
    out.frames[8].expected.fault_reason = safety::FaultReason::HostEstop;
    return out;
  }();
  return fixture;
}

}  // namespace fixtures
}  // namespace replay
}  // namespace controller