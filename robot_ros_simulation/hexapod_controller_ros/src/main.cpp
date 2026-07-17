#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hexapod_controller_ros/controller_core_node.hpp"
#include "hexapod_controller_ros/ros2_control_sil_adapter.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_controller_ros::ControllerCoreNode>();
  auto sil_adapter =
      std::make_shared<hexapod_controller_ros::Ros2ControlSilAdapter>(*node);
  if (!node->setAdapters(sil_adapter, sil_adapter)) {
    RCLCPP_ERROR(node->get_logger(), "Unable to install the SIL adapters");
    rclcpp::shutdown();
    return 1;
  }

  const bool autostart = node->declare_parameter<bool>("autostart", true);
  if (autostart) {
    if (node->configure().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(node->get_logger(), "SIL node configuration failed");
      rclcpp::shutdown();
      return 1;
    }
    if (node->activate().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(node->get_logger(), "SIL node activation failed");
      rclcpp::shutdown();
      return 1;
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}