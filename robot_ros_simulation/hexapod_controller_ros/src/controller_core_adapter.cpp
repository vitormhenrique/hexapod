#include "hexapod_controller_ros/controller_core_adapter.hpp"

namespace hexapod_controller_ros {

void ControllerCoreAdapter::reset() {
  core_.reset();
  command_ = controller::RobotCommand{};
}

void ControllerCoreAdapter::configureSafety(
    const safety::StateParams& params) {
  core_.configureSafety(params);
}

const controller::RobotCommand& ControllerCoreAdapter::step(
    const controller::ControllerStepInput& input) {
  core_.step(input.state, input.intent, input.config, input.time, command_);
  return command_;
}

}  // namespace hexapod_controller_ros