#include <array>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hexapod_controller_ros/controller_core_node.hpp"
#include "hexapod_controller_ros/ros2_control_sil_adapter.hpp"

namespace {

constexpr std::array<const char*, config::kNumServos> kJointNames = {
    "leg_1_coxa_joint", "leg_1_femur_joint", "leg_1_tibia_joint",
    "leg_2_coxa_joint", "leg_2_femur_joint", "leg_2_tibia_joint",
    "leg_3_coxa_joint", "leg_3_femur_joint", "leg_3_tibia_joint",
    "leg_4_coxa_joint", "leg_4_femur_joint", "leg_4_tibia_joint",
    "leg_5_coxa_joint", "leg_5_femur_joint", "leg_5_tibia_joint",
    "leg_6_coxa_joint", "leg_6_femur_joint", "leg_6_tibia_joint",
};

sensor_msgs::msg::JointState completeFeedback() {
  sensor_msgs::msg::JointState feedback;
  for (uint8_t index = 0; index < config::kNumServos; ++index) {
    const uint8_t reversed = config::kNumServos - 1u - index;
    feedback.name.emplace_back(kJointNames[reversed]);
    feedback.position.emplace_back(static_cast<double>(reversed) * 0.002);
  }
  return feedback;
}

hexapod_msgs::msg::SilSafetyInput safeSilSafety(bool armed) {
  hexapod_msgs::msg::SilSafetyInput safety;
  safety.rc_seen = true;
  safety.rc_kill = false;
  safety.rc_armed = armed;
  safety.rc_autonomy_enabled = true;
  safety.battery_valid = true;
  safety.battery_millivolts = 12000;
  return safety;
}

hexapod_msgs::msg::FootContactArray contactSnapshot(
    uint8_t validity =
        hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH,
    uint8_t present_mask = (1u << sensors::kNumFeet) - 1u,
    uint8_t state = hexapod_msgs::msg::FootContact::STATE_LOADED,
    uint8_t confidence = 255u) {
  hexapod_msgs::msg::FootContactArray contacts;
  contacts.source_time_ms = 321u;
  contacts.validity = validity;
  contacts.present_mask = present_mask;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    auto& foot = contacts.feet[leg];
    foot.leg_index = leg;
    foot.state = state;
    foot.confidence = confidence;
    foot.proximity_raw = 200u + leg;
    foot.pressure_raw = 1000 + leg;
    foot.pressure_baseline = 900;
    foot.pressure_delta = 100 + leg;
  }
  return contacts;
}

hexapod_msgs::msg::MotionCommand walkingMotion() {
  hexapod_msgs::msg::MotionCommand motion;
  motion.sequence = 1;
  motion.valid_for.sec = 1;
  motion.gait = hexapod_msgs::msg::MotionCommand::GAIT_TRIPOD;
  motion.body_height_m = 0.05f;
  motion.stride_length_m = 0.06f;
  motion.step_height_m = 0.025f;
  motion.duty_factor = 0.5f;
  motion.speed_scale = 0.6f;
  motion.normalized_vx = 0.6f;
  return motion;
}

void step(hexapod_controller_ros::ControllerCoreNode& node, uint32_t count) {
  for (uint32_t index = 0; index < count; ++index) {
    ASSERT_TRUE(node.stepOnce());
  }
}

}  // namespace

TEST(Ros2ControlSilAdapter, NamedFeedbackDrivesOnlyMotionGatedOutput) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  auto node = std::make_shared<hexapod_controller_ros::ControllerCoreNode>();
  auto adapter =
      std::make_shared<hexapod_controller_ros::Ros2ControlSilAdapter>(*node);
  ASSERT_TRUE(node->setAdapters(adapter, adapter));

  const sensor_msgs::msg::JointState feedback = completeFeedback();
  ASSERT_TRUE(adapter->acceptJointState(feedback));
  sensor_msgs::msg::JointState incomplete = feedback;
  incomplete.name.pop_back();
  incomplete.position.pop_back();
  EXPECT_FALSE(adapter->acceptJointState(incomplete));
  ASSERT_TRUE(adapter->acceptMotionCommand(walkingMotion()));
  adapter->acceptSafetyInput(safeSilSafety(false));

  ASSERT_EQ(node->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  step(*node, 3);  // Boot -> ConfigLoad -> Disarmed and arm release.
  adapter->acceptSafetyInput(safeSilSafety(true));
  step(*node, 6);  // Arming -> StandReady -> Jetson-assisted gait.

  ASSERT_EQ(node->lastCommand().safety_state, safety::State::JetsonAssisted);
  const uint32_t commanded_count = adapter->simulationOutputCount();
  ASSERT_GT(commanded_count, 0u);
  const auto command = adapter->lastSimulationCommand();
  bool any_nonzero = false;
  for (double position_rad : command) {
    EXPECT_TRUE(std::isfinite(position_rad));
    EXPECT_LE(std::fabs(position_rad), dxl::kPi / 2.0 + 0.001);
    any_nonzero = any_nonzero || std::fabs(position_rad) > 0.0001;
  }
  EXPECT_TRUE(any_nonzero);

  adapter->acceptSafetyInput(safeSilSafety(false));
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::Disarmed);
  EXPECT_EQ(adapter->simulationOutputCount(), commanded_count);

  auto estop = safeSilSafety(false);
  estop.host_estop = true;
  adapter->acceptSafetyInput(estop);
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::Estop);
  EXPECT_EQ(adapter->simulationOutputCount(), commanded_count);

  EXPECT_EQ(node->deactivate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node.reset();
  rclcpp::shutdown();
}

TEST(Ros2ControlSilAdapter, ContactMailboxPreservesAndFailsClosedSnapshots) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  auto node = std::make_shared<hexapod_controller_ros::ControllerCoreNode>();
  auto adapter =
      std::make_shared<hexapod_controller_ros::Ros2ControlSilAdapter>(*node);
  ASSERT_TRUE(adapter->acceptJointState(completeFeedback()));

  controller::ControllerStepInput input;
  auto fresh = contactSnapshot();
  ASSERT_TRUE(adapter->acceptFootContacts(fresh));
  ASSERT_TRUE(adapter->sample(input));
  EXPECT_EQ(input.state.contact.validity, controller::SnapshotValidity::Fresh);
  EXPECT_EQ(input.state.contact.present_mask,
            (1u << sensors::kNumFeet) - 1u);
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    const auto& foot = input.state.contact.feet[leg];
    EXPECT_EQ(foot.timestamp_ms, fresh.source_time_ms);
    EXPECT_EQ(foot.state, sensors::ContactState::Loaded);
    EXPECT_TRUE(foot.loaded);
    EXPECT_FALSE(foot.stale);
    EXPECT_FALSE(foot.fault);
    EXPECT_EQ(foot.confidence, 255u);
  }

  auto low_confidence = contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH,
      (1u << sensors::kNumFeet) - 1u,
      hexapod_msgs::msg::FootContact::STATE_LOADED, 127u);
  ASSERT_TRUE(adapter->acceptFootContacts(low_confidence));
  ASSERT_TRUE(adapter->sample(input));
  EXPECT_EQ(input.state.contact.feet[0].state, sensors::ContactState::Loaded);
  EXPECT_EQ(input.state.contact.feet[0].confidence, 127u);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_STALE)));
  ASSERT_TRUE(adapter->sample(input));
  EXPECT_EQ(input.state.contact.validity, controller::SnapshotValidity::Stale);
  for (const auto& foot : input.state.contact.feet) {
    EXPECT_EQ(foot.state, sensors::ContactState::Stale);
    EXPECT_TRUE(foot.stale);
    EXPECT_EQ(foot.confidence, 0u);
  }

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FAULT)));
  ASSERT_TRUE(adapter->sample(input));
  EXPECT_EQ(input.state.contact.validity, controller::SnapshotValidity::Fault);
  for (const auto& foot : input.state.contact.feet) {
    EXPECT_EQ(foot.state, sensors::ContactState::Fault);
    EXPECT_TRUE(foot.fault);
    EXPECT_EQ(foot.confidence, 0u);
  }

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH, 0u)));
  ASSERT_TRUE(adapter->sample(input));
  EXPECT_EQ(input.state.contact.validity, controller::SnapshotValidity::Fresh);
  EXPECT_EQ(input.state.contact.present_mask, 0u);
  for (const auto& foot : input.state.contact.feet) {
    EXPECT_EQ(foot.state, sensors::ContactState::Stale);
    EXPECT_TRUE(foot.stale);
    EXPECT_EQ(foot.confidence, 0u);
  }

  auto malformed = contactSnapshot();
  malformed.feet[1].leg_index = malformed.feet[0].leg_index;
  EXPECT_FALSE(adapter->acceptFootContacts(malformed));
  malformed = contactSnapshot();
  malformed.validity = 4u;
  EXPECT_FALSE(adapter->acceptFootContacts(malformed));

  node.reset();
  rclcpp::shutdown();
}

TEST(Ros2ControlSilAdapter, ContactConfidenceControlsTerrainFallback) {
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  auto node = std::make_shared<hexapod_controller_ros::ControllerCoreNode>();
  auto adapter =
      std::make_shared<hexapod_controller_ros::Ros2ControlSilAdapter>(*node);
  ASSERT_TRUE(node->setAdapters(adapter, adapter));
  ASSERT_TRUE(adapter->acceptJointState(completeFeedback()));
  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot()));

  auto safety = safeSilSafety(false);
  safety.rc_autonomy_enabled = false;
  safety.foot_contact_enabled = true;
  adapter->acceptSafetyInput(safety);

  ASSERT_EQ(node->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  step(*node, 3);  // Boot -> ConfigLoad -> Disarmed and arm release.

  safety.rc_armed = true;
  adapter->acceptSafetyInput(safety);
  step(*node, 3);  // Arming -> StandReady -> ContactTerrain.
  ASSERT_EQ(node->lastCommand().safety_state, safety::State::ContactTerrain);
  ASSERT_EQ(node->lastCommand().diagnostics.confident_contact_feet,
            sensors::kNumFeet);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH,
      (1u << sensors::kNumFeet) - 1u,
      hexapod_msgs::msg::FootContact::STATE_LOADED, 127u)));
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::RcManual);
  EXPECT_EQ(node->lastCommand().diagnostics.confident_contact_feet, 0u);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot()));
  step(*node, 1);
  ASSERT_EQ(node->lastCommand().safety_state, safety::State::ContactTerrain);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FAULT)));
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::RcManual);
  EXPECT_EQ(node->lastCommand().diagnostics.confident_contact_feet, 0u);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot()));
  step(*node, 1);
  ASSERT_EQ(node->lastCommand().safety_state, safety::State::ContactTerrain);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_STALE)));
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::RcManual);
  EXPECT_EQ(node->lastCommand().diagnostics.confident_contact_feet, 0u);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot()));
  step(*node, 1);
  ASSERT_EQ(node->lastCommand().safety_state, safety::State::ContactTerrain);

  ASSERT_TRUE(adapter->acceptFootContacts(contactSnapshot(
      hexapod_msgs::msg::FootContactArray::VALIDITY_FRESH, 0u)));
  step(*node, 1);
  EXPECT_EQ(node->lastCommand().safety_state, safety::State::RcManual);
  EXPECT_EQ(node->lastCommand().diagnostics.confident_contact_feet, 0u);

  EXPECT_EQ(node->deactivate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->cleanup().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node.reset();
  rclcpp::shutdown();
}