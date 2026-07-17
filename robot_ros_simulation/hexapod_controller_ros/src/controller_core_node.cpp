#include "hexapod_controller_ros/controller_core_node.hpp"

#include <chrono>
#include <utility>

namespace hexapod_controller_ros {
namespace {

class NullInputAdapter final : public ControllerInputAdapter {
 public:
  bool sample(controller::ControllerStepInput&) override { return false; }
};

class NullCommandAdapter final : public ControllerCommandAdapter {
 public:
  void publish(const controller::ControllerStepInput&,
               const controller::RobotCommand&) override {}
};

}  // namespace

ControllerCoreNode::ControllerCoreNode(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<ControllerInputAdapter> input_adapter,
    std::shared_ptr<ControllerCommandAdapter> command_adapter)
    : rclcpp_lifecycle::LifecycleNode("hexapod_controller_core", options),
      input_adapter_(std::move(input_adapter)),
      command_adapter_(std::move(command_adapter)) {
  if (!input_adapter_) input_adapter_ = std::make_shared<NullInputAdapter>();
  if (!command_adapter_) {
    command_adapter_ = std::make_shared<NullCommandAdapter>();
  }
}

bool ControllerCoreNode::setAdapters(
    std::shared_ptr<ControllerInputAdapter> input_adapter,
    std::shared_ptr<ControllerCommandAdapter> command_adapter) {
  if (!input_adapter || !command_adapter || active_ || cycle_timer_) {
    return false;
  }
  input_adapter_ = std::move(input_adapter);
  command_adapter_ = std::move(command_adapter);
  return true;
}

ControllerCoreNode::CallbackReturn ControllerCoreNode::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!has_parameter("period_ms")) {
    declare_parameter<int>("period_ms", 10);
  }
  if (!get_parameter("period_ms", period_ms_) || period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "period_ms must be positive");
    return CallbackReturn::FAILURE;
  }

  // Firmware defaults to a 60 s idle auto-disarm to stop 18 physical MX-28s
  // from holding torque unattended. A simulation has no such power concern,
  // and its constant simulated rc_armed input can never provide the arm
  // release edge the state machine demands after an auto-disarm, so the SIL
  // launch disables the timeout (0). The parameter keeps firmware parity
  // available for tests that want it.
  if (!has_parameter("idle_disarm_ms")) {
    declare_parameter<int>("idle_disarm_ms", 60000);
  }
  int idle_disarm_ms = 60000;
  if (!get_parameter("idle_disarm_ms", idle_disarm_ms) || idle_disarm_ms < 0) {
    RCLCPP_ERROR(get_logger(), "idle_disarm_ms must be zero or positive");
    return CallbackReturn::FAILURE;
  }

  core_adapter_.reset();
  safety::StateParams safety_params;
  safety_params.idle_disarm_ms = static_cast<uint32_t>(idle_disarm_ms);
  core_adapter_.configureSafety(safety_params);
  active_ = false;
  cycle_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cycle_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_), [this]() { stepOnce(); },
      cycle_group_);
  cycle_timer_->cancel();
  return CallbackReturn::SUCCESS;
}

ControllerCoreNode::CallbackReturn ControllerCoreNode::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!cycle_timer_) return CallbackReturn::FAILURE;
  active_ = true;
  cycle_timer_->reset();
  return CallbackReturn::SUCCESS;
}

ControllerCoreNode::CallbackReturn ControllerCoreNode::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  if (cycle_timer_) cycle_timer_->cancel();
  return CallbackReturn::SUCCESS;
}

ControllerCoreNode::CallbackReturn ControllerCoreNode::on_cleanup(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  if (cycle_timer_) cycle_timer_->cancel();
  cycle_timer_.reset();
  cycle_group_.reset();
  core_adapter_.reset();
  return CallbackReturn::SUCCESS;
}

ControllerCoreNode::CallbackReturn ControllerCoreNode::on_shutdown(
    const rclcpp_lifecycle::State&) {
  return on_cleanup(rclcpp_lifecycle::State{});
}

bool ControllerCoreNode::stepOnce() {
  if (!active_) return false;

  controller::ControllerStepInput input;
  if (!input_adapter_->sample(input)) return false;

  const controller::RobotCommand& command = core_adapter_.step(input);
  command_adapter_->publish(input, command);
  return true;
}

}  // namespace hexapod_controller_ros