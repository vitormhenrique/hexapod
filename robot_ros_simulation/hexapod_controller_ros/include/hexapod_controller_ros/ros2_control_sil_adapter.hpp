#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "hexapod_msgs/msg/controller_status.hpp"
#include "hexapod_msgs/msg/foot_contact_array.hpp"
#include "hexapod_msgs/msg/joint_command.hpp"
#include "hexapod_msgs/msg/motion_command.hpp"
#include "hexapod_msgs/msg/sil_safety_input.hpp"

#include "hexapod_controller_ros/controller_core_adapter.hpp"

namespace hexapod_controller_ros {

// ROS-facing adapter for the mock ros2_control SIL path. It owns ROS message
// conversion and mailbox synchronization; ControllerCore remains ROS-free.
class Ros2ControlSilAdapter final : public ControllerInputAdapter,
                                   public ControllerCommandAdapter {
 public:
  explicit Ros2ControlSilAdapter(rclcpp_lifecycle::LifecycleNode& node);

  bool sample(controller::ControllerStepInput& input) override;
  void publish(const controller::ControllerStepInput& input,
               const controller::RobotCommand& command) override;

  // Subscription callbacks delegate to these methods so tests can exercise
  // the same named-message conversion without requiring a DDS round trip.
  bool acceptJointState(const sensor_msgs::msg::JointState& message);
  bool acceptMotionCommand(const hexapod_msgs::msg::MotionCommand& message);
  void acceptSafetyInput(const hexapod_msgs::msg::SilSafetyInput& message);
    bool acceptFootContacts(
            const hexapod_msgs::msg::FootContactArray& message);

  uint32_t simulationOutputCount() const;
  std::array<double, config::kNumServos> lastSimulationCommand() const;

 private:
  using SteadyClock = std::chrono::steady_clock;

  struct SilSafetyState {
    bool rc_seen = false;
    bool rc_kill = true;
    bool rc_armed = false;
    bool rc_autonomy_enabled = false;
    bool host_estop = false;
    bool battery_valid = true;
    uint16_t battery_millivolts = 12000;
    bool watchdog_fault = false;
    bool dxl_hard_fault = false;
        bool foot_contact_enabled = false;
        bool terrain_leveling_enabled = false;
    };

    struct ContactMailbox {
        controller::ContactSnapshot snapshot{};
        SteadyClock::time_point received{};
        bool received_valid = false;
  };

  static int jointIndexForName(const std::string& name);
  static const char* jointNameForIndex(uint8_t index);
  static bool isFinite(const hexapod_msgs::msg::MotionCommand& message);
  static bool convertMotionCommand(
      const hexapod_msgs::msg::MotionCommand& message,
      protocol::MotionIntent& intent,
      std::chrono::milliseconds& validity);

  uint32_t elapsedMilliseconds(const SteadyClock::time_point& now) const;

  rclcpp_lifecycle::LifecycleNode& node_;
  config::RobotConfig config_;
  dxl::ServoMap servo_map_;
  bool config_valid_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<hexapod_msgs::msg::MotionCommand>::SharedPtr
      motion_subscription_;
  rclcpp::Subscription<hexapod_msgs::msg::SilSafetyInput>::SharedPtr
      safety_subscription_;
  rclcpp::Subscription<hexapod_msgs::msg::FootContactArray>::SharedPtr
      foot_contact_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      simulation_command_publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::JointCommand>::SharedPtr
      joint_command_publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::ControllerStatus>::SharedPtr
      status_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  mutable std::mutex mutex_;
  std::array<double, config::kNumServos> feedback_positions_rad_ = {};
  bool feedback_valid_ = false;
  SteadyClock::time_point feedback_received_{};
  protocol::MotionIntent motion_{};
  bool motion_valid_ = false;
  SteadyClock::time_point motion_expires_{};
  SilSafetyState safety_{};
    ContactMailbox contacts_{};
  uint32_t last_sample_ms_ = 0;
  bool sampled_once_ = false;
  uint32_t feedback_timeout_ms_ = 250;
    uint32_t contact_timeout_ms_ = 250;
  uint32_t simulation_output_count_ = 0;
  std::array<double, config::kNumServos> last_simulation_command_ = {};
  // Planar odometry integrated from the commanded twist so the simulated
  // robot translates across the RViz world instead of walking in place.
  double odom_x_m_ = 0.0;
  double odom_y_m_ = 0.0;
  double odom_yaw_rad_ = 0.0;
  bool odom_time_valid_ = false;
  SteadyClock::time_point odom_last_update_{};
  const SteadyClock::time_point epoch_ = SteadyClock::now();
};

}  // namespace hexapod_controller_ros