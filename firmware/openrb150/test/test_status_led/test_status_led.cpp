#include <unity.h>

#include "app/status_led.h"

using app::status_led::Inputs;
using app::status_led::Pattern;

void test_healthy_state_patterns() {
  Inputs inputs;
  inputs.state = safety::State::Boot;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::BootFast),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 0));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 100));

  inputs.state = safety::State::Disarmed;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::DisarmedHeartbeat),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 0));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 100));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 2000));

  inputs.state = safety::State::RcManual;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::ArmedSolid),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 1733));
}

void test_arming_distinguishes_discovery_and_pose_wait() {
  Inputs inputs;
  inputs.state = safety::State::ArmingChecks;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::ArmingDiscoverIds),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 0));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 300));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 600));

  inputs.configured_servo_coverage = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::ArmingReadPoses),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 600));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 900));

  inputs.all_servo_poses_known = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::BootFast),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
}

void test_maintenance_and_passive_patterns() {
  Inputs inputs;
  inputs.state = safety::State::MacMaintenance;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::MaintenanceDouble),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 0));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 300));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 600));

  inputs.state = safety::State::PassivePoseStream;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Pattern::PassiveSlow),
                          static_cast<uint8_t>(app::status_led::patternFor(inputs)));
  TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, 0));
  TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, 500));
}

void test_fault_codes_match_fault_reason_values() {
  Inputs inputs;
  inputs.state = safety::State::FaultSoft;
  for (uint8_t reason = 1; reason <= 7; ++reason) {
    inputs.fault = static_cast<safety::FaultReason>(reason);
    TEST_ASSERT_EQUAL_UINT8(reason, app::status_led::faultPulseCount(inputs));
    for (uint8_t pulse = 0; pulse < reason; ++pulse) {
      TEST_ASSERT_TRUE(app::status_led::ledOn(inputs, pulse * 300u));
      TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, pulse * 300u + 100u));
    }
    TEST_ASSERT_FALSE(app::status_led::ledOn(inputs, reason * 300u));
  }

  inputs.state = safety::State::Disarmed;
  inputs.fault = safety::FaultReason::None;
  inputs.watchdog_stalled = true;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(safety::FaultReason::Watchdog),
      app::status_led::faultPulseCount(inputs));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_healthy_state_patterns);
  RUN_TEST(test_arming_distinguishes_discovery_and_pose_wait);
  RUN_TEST(test_maintenance_and_passive_patterns);
  RUN_TEST(test_fault_codes_match_fault_reason_values);
  return UNITY_END();
}
