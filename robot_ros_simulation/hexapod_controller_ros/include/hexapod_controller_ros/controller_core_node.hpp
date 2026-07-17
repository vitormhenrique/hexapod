#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "hexapod_controller_ros/controller_core_adapter.hpp"

namespace hexapod_controller_ros {

class ControllerCoreNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ControllerCoreNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
      std::shared_ptr<ControllerInputAdapter> input_adapter = nullptr,
      std::shared_ptr<ControllerCommandAdapter> command_adapter = nullptr);

    // Adapters may be installed only before configuration. This lets the
    // standalone executable construct a ROS-facing adapter with a reference to
    // this node while preserving injected adapters for deterministic tests.
    bool setAdapters(std::shared_ptr<ControllerInputAdapter> input_adapter,
                                     std::shared_ptr<ControllerCommandAdapter> command_adapter);

  // Executes one adapter/core/output cycle while active. This is public so
  // deterministic adapter tests can exercise the exact timer boundary without
  // relying on a wall-clock executor.
  bool stepOnce();

    const controller::RobotCommand& lastCommand() const {
        return core_adapter_.command();
    }

 protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state)
      override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state)
      override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state)
      override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state)
      override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state)
      override;

 private:
  ControllerCoreAdapter core_adapter_;
  std::shared_ptr<ControllerInputAdapter> input_adapter_;
  std::shared_ptr<ControllerCommandAdapter> command_adapter_;
  rclcpp::CallbackGroup::SharedPtr cycle_group_;
  rclcpp::TimerBase::SharedPtr cycle_timer_;
  int period_ms_ = 10;
  bool active_ = false;
};

}  // namespace hexapod_controller_ros