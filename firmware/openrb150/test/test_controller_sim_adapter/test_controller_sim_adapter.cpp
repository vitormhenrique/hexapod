// Native tests for the test-only ControllerCore simulation adapter.
// Run with: pio test -e native -f test_controller_sim_adapter

#include <unity.h>

#include "../support/controller_sim_adapter.h"

namespace {

void configureSafeRcIntent(controller::ControllerIntent& intent) {
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  intent.rc.ever_seen = true;
  intent.rc.command.valid = true;
  intent.rc.command.body_height = 0.5f;
  intent.rc.command.speed = 0.5f;
  intent.rc.command.stride = 0.5f;
  intent.rc.command.step_height = 0.5f;
  intent.motion.duty_x255 = 128;
}

void driveToRcManual(controller::sim::ControllerSimAdapter& adapter) {
  adapter.advance(10);  // Boot -> ConfigLoad
  adapter.advance(10);  // ConfigLoad -> Disarmed
  adapter.advance(10);  // release arm qualification
  adapter.intent().rc.armed = true;
  adapter.advance(10);  // Disarmed -> ArmingChecks
  adapter.advance(10);  // ArmingChecks -> StandReady
  adapter.advance(10);  // StandReady -> RcManual and first goals
}

}  // namespace

void test_adapter_exposes_seeded_servo_feedback_to_first_goal_frame() {
  controller::sim::ControllerSimAdapter adapter;
  TEST_ASSERT_TRUE(adapter.configureDefault(7));
  TEST_ASSERT_TRUE(adapter.setReadyDxl(1500));
  TEST_ASSERT_TRUE(adapter.state().dxl.torque_off);
  configureSafeRcIntent(adapter.intent());

  driveToRcManual(adapter);

  const controller::RobotCommand& command = adapter.command();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::RcManual),
                          static_cast<uint8_t>(command.safety_state));
  TEST_ASSERT_TRUE(command.motion_gate);
  TEST_ASSERT_TRUE(command.goal_valid);
  TEST_ASSERT_EQUAL_UINT8(config::kNumServos, command.goals.count);
  for (uint8_t index = 0; index < command.goals.count; ++index) {
    const int32_t delta = static_cast<int32_t>(command.goals.joints[index].tick) -
                          1500;
    TEST_ASSERT_TRUE(delta >= -26 && delta <= 26);
  }
}

void test_stale_contact_snapshot_falls_back_to_nominal_rc_mode() {
  controller::sim::ControllerSimAdapter adapter;
  TEST_ASSERT_TRUE(adapter.configureDefault());
  TEST_ASSERT_TRUE(adapter.setReadyDxl());
  configureSafeRcIntent(adapter.intent());
  adapter.intent().features.foot_contact_enabled = true;
  adapter.state().contact.validity = controller::SnapshotValidity::Stale;
  for (uint8_t leg = 0; leg < sensors::kNumFeet; ++leg) {
    adapter.state().contact.feet[leg].confidence = 255;
    adapter.state().contact.feet[leg].stale = true;
  }

  driveToRcManual(adapter);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::RcManual),
                          static_cast<uint8_t>(adapter.command().safety_state));
  TEST_ASSERT_TRUE(adapter.command().motion_gate);
}

void test_missing_servo_feedback_is_representable_and_blocks_arming() {
  controller::sim::ControllerSimAdapter adapter;
  TEST_ASSERT_TRUE(adapter.configureDefault());
  TEST_ASSERT_TRUE(adapter.setReadyDxl());
  configureSafeRcIntent(adapter.intent());
  adapter.state().dxl.servo_count = config::kNumServos - 1;
  adapter.state().dxl.configured_servo_coverage = false;
  adapter.state().dxl.pose_known_mask = 0;
  adapter.intent().rc.armed = true;

  adapter.advance(10);  // Boot -> ConfigLoad
  adapter.advance(10);  // ConfigLoad -> Disarmed
  adapter.advance(10);  // arm qualification is still held, release first
  adapter.intent().rc.armed = false;
  adapter.advance(10);
  adapter.intent().rc.armed = true;
  adapter.advance(10);  // Disarmed -> ArmingChecks
  adapter.advance(10);  // remains in ArmingChecks: missing evidence

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(safety::State::ArmingChecks),
                          static_cast<uint8_t>(adapter.command().safety_state));
  TEST_ASSERT_FALSE(adapter.command().motion_gate);
  TEST_ASSERT_FALSE(adapter.command().allow_torque);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_adapter_exposes_seeded_servo_feedback_to_first_goal_frame);
  RUN_TEST(test_stale_contact_snapshot_falls_back_to_nominal_rc_mode);
  RUN_TEST(test_missing_servo_feedback_is_representable_and_blocks_arming);
  return UNITY_END();
}