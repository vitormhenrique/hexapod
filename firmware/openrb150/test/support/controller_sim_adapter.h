#pragma once

// ===========================================================================
// Native ControllerCore fixture (hexapod_src-4ju.8).
//
// This is deliberately test-only: it owns contract snapshots, not Arduino,
// FreeRTOS, DXL, Wire, or ROS transport. Tests may mutate every snapshot field
// directly, while helpers model the same fresh-status goal-seeding evidence the
// ATSAMD21 adapter supplies to ControllerCore.
// ===========================================================================

#include "../../src/controller/controller_core.h"

namespace controller {
namespace sim {

class ControllerSimAdapter {
 public:
  ControllerSimAdapter() { reset(); }

  void reset() {
    core_.reset();
    input_ = ControllerStepInput{};
    command_ = RobotCommand{};
  }

  bool configureDefault(uint32_t revision = 1, bool persistent = false) {
    config::defaultRobotConfig(input_.config.robot);
    input_.config.revision = revision;
    input_.config.persistent = persistent;
    input_.config.valid = config::validateRobotConfig(input_.config.robot);
    return input_.config.valid;
  }

  // Populate the same arming evidence the DXL adapter publishes after a fresh
  // full-bus status read. A test may subsequently override any field directly.
  bool setReadyDxl(uint16_t present_tick = config::kServoCenterTick) {
    if (!input_.config.valid) return false;
    RobotState& state = input_.state;
    state.config_ready = true;
    state.battery.millivolts = 12000;
    state.battery.valid = true;
    state.battery.validity = SnapshotValidity::Fresh;
    state.dxl.servo_count = config::kNumServos;
    state.dxl.validity = SnapshotValidity::Fresh;
    state.dxl.configured_servo_coverage = true;
    state.dxl.config_revision = input_.config.revision;
    state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
    state.dxl.torque_off = true;
    state.dxl.hard_fault = false;
    for (uint8_t index = 0; index < config::kNumServos; ++index) {
      dxl::ServoStatus& servo = state.dxl.servos[index];
      servo = dxl::ServoStatus{};
      servo.id = input_.config.robot.servos[index].id;
      servo.present_position = present_tick;
      servo.ok = true;
    }
    return true;
  }

  void setTime(uint32_t now_ms, uint32_t dt_ms, bool valid = true) {
    input_.time.now_ms = now_ms;
    input_.time.dt_ms = dt_ms;
    input_.time.valid = valid;
  }

  const RobotCommand& step() {
    core_.step(input_.state, input_.intent, input_.config, input_.time,
               command_);
    return command_;
  }

  const RobotCommand& advance(uint32_t dt_ms) {
    input_.time.now_ms += dt_ms;
    input_.time.dt_ms = dt_ms;
    input_.time.valid = true;
    return step();
  }

  RobotState& state() { return input_.state; }
  ControllerIntent& intent() { return input_.intent; }
  ControllerConfigSnapshot& config() { return input_.config; }
  ControllerTime& time() { return input_.time; }
  const RobotCommand& command() const { return command_; }

 private:
  ControllerCore core_;
  ControllerStepInput input_;
  RobotCommand command_;
};

}  // namespace sim
}  // namespace controller