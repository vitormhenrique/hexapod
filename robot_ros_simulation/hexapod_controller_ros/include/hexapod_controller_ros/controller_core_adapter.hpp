#pragma once

// ROS-facing shell around the portable controller. This header intentionally
// exposes only controller contracts: serial, DXL, FreeRTOS, Gazebo, and ROS
// message conversion belong to injected adapters outside ControllerCore.

#include "controller/controller_core.h"

namespace hexapod_controller_ros {

class ControllerInputAdapter {
 public:
  virtual ~ControllerInputAdapter() = default;

  // Copy one self-consistent snapshot into input. The lifecycle timer is the
  // only caller. Return false when no complete snapshot is available.
  virtual bool sample(controller::ControllerStepInput& input) = 0;
};

class ControllerCommandAdapter {
 public:
  virtual ~ControllerCommandAdapter() = default;

  // Consume the command from the same input snapshot. Implementations may
  // publish observation data or feed SIL only; they must not bypass safety.
  virtual void publish(const controller::ControllerStepInput& input,
                       const controller::RobotCommand& command) = 0;
};

class ControllerCoreAdapter {
 public:
  void reset();

  // Forward safety tunables to the portable core. Params survive reset().
  void configureSafety(const safety::StateParams& params);

  // ControllerCore fully overwrites command_ for each call. The caller must
  // serialize calls; ControllerCore state is intentionally not thread-safe.
  const controller::RobotCommand& step(
      const controller::ControllerStepInput& input);

  const controller::RobotCommand& command() const { return command_; }

 private:
  controller::ControllerCore core_;
  controller::RobotCommand command_;
};

}  // namespace hexapod_controller_ros