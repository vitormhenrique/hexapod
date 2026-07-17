#include <memory>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "config/config_schema.h"
#include "hexapod_controller_ros/controller_core_node.hpp"

namespace {

class FixedInputAdapter final
    : public hexapod_controller_ros::ControllerInputAdapter {
 public:
  controller::ControllerStepInput input;

  bool sample(controller::ControllerStepInput& destination) override {
    destination = input;
    return true;
  }
};

class RecordingCommandAdapter final
    : public hexapod_controller_ros::ControllerCommandAdapter {
 public:
  uint32_t count = 0;
  controller::RobotCommand command;

  void publish(const controller::ControllerStepInput&,
               const controller::RobotCommand& source) override {
    command = source;
    ++count;
  }
};

void configureReadyInput(controller::ControllerStepInput& input) {
  config::defaultRobotConfig(input.config.robot);
  input.config.revision = 9;
  input.config.valid = config::validateRobotConfig(input.config.robot);
  input.config.persistent = false;

  input.state.config_ready = true;
  input.state.battery.millivolts = 12000;
  input.state.battery.valid = true;
  input.state.battery.validity = controller::SnapshotValidity::Fresh;
  input.state.dxl.servo_count = config::kNumServos;
  input.state.dxl.validity = controller::SnapshotValidity::Fresh;
  input.state.dxl.configured_servo_coverage = true;
  input.state.dxl.pose_known_mask = (1u << config::kNumServos) - 1u;
  input.state.dxl.config_revision = input.config.revision;
  input.state.dxl.torque_off = true;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    input.state.dxl.servos[index].id = input.config.robot.servos[index].id;
    input.state.dxl.servos[index].present_position = config::kServoCenterTick;
    input.state.dxl.servos[index].ok = true;
  }

  input.intent.rc.kill = false;
  input.intent.rc.failsafe = false;
  input.intent.rc.ever_seen = true;
  input.intent.rc.command.valid = true;
  input.intent.rc.command.body_height = 0.5f;
  input.intent.rc.command.speed = 0.5f;
  input.intent.rc.command.stride = 0.5f;
  input.intent.rc.command.step_height = 0.5f;
  input.intent.motion.duty_x255 = 128;
  input.time.valid = true;
}

void advance(FixedInputAdapter& input,
             hexapod_controller_ros::ControllerCoreNode& node) {
  input.input.time.now_ms += 10;
  input.input.time.dt_ms = 10;
  ASSERT_TRUE(node.stepOnce());
}

}  // namespace

TEST(ControllerCoreNode, StepsInjectedSnapshotOnlyWhileLifecycleActive) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  auto input = std::make_shared<FixedInputAdapter>();
  auto output = std::make_shared<RecordingCommandAdapter>();
  configureReadyInput(input->input);
  auto node = std::make_shared<hexapod_controller_ros::ControllerCoreNode>(
      rclcpp::NodeOptions(), input, output);

  EXPECT_FALSE(node->stepOnce());
  EXPECT_EQ(node->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  advance(*input, *node);  // Boot -> ConfigLoad
  advance(*input, *node);  // ConfigLoad -> Disarmed
  advance(*input, *node);  // arm-release qualification
  input->input.intent.rc.armed = true;
  advance(*input, *node);  // ArmingChecks
  advance(*input, *node);  // StandReady
  advance(*input, *node);  // RcManual, seeded goals

  EXPECT_EQ(output->count, 6u);
  EXPECT_EQ(output->command.safety_state, safety::State::RcManual);
  EXPECT_TRUE(output->command.motion_gate);
  EXPECT_TRUE(output->command.goal_valid);
  EXPECT_EQ(output->command.goals.count, config::kNumServos);

  EXPECT_EQ(node->deactivate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(node->stepOnce());
  EXPECT_EQ(node->cleanup().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node.reset();
  rclcpp::shutdown();
}