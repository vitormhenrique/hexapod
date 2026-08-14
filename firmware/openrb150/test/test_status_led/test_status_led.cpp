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

namespace {
void assertColor(const app::status_led::Inputs& inputs, uint8_t r, uint8_t g,
                 uint8_t b) {
  const app::status_led::Rgb c = app::status_led::colorFor(inputs);
  TEST_ASSERT_EQUAL_UINT8(r, c.r);
  TEST_ASSERT_EQUAL_UINT8(g, c.g);
  TEST_ASSERT_EQUAL_UINT8(b, c.b);
}
}  // namespace

void test_colors_distinguish_states_and_modes() {
  Inputs inputs;
  inputs.state = safety::State::Boot;
  assertColor(inputs, 255, 255, 255);  // boot: white

  inputs.state = safety::State::Disarmed;
  assertColor(inputs, 0, 0, 255);  // disarmed: blue

  inputs.state = safety::State::ArmingChecks;
  assertColor(inputs, 255, 180, 0);  // discovering IDs: yellow
  inputs.configured_servo_coverage = true;
  assertColor(inputs, 255, 64, 0);  // waiting on poses: amber

  inputs.state = safety::State::StandReady;
  assertColor(inputs, 0, 255, 0);  // armed: green
  inputs.state = safety::State::RcManual;
  assertColor(inputs, 0, 255, 0);
  inputs.state = safety::State::ContactTerrain;
  assertColor(inputs, 128, 255, 0);  // terrain mode: chartreuse
  inputs.state = safety::State::JetsonAssisted;
  assertColor(inputs, 128, 0, 255);  // Jetson autonomy: violet

  inputs.state = safety::State::MacMaintenance;
  assertColor(inputs, 255, 0, 255);  // maintenance: magenta
  inputs.state = safety::State::PassivePoseStream;
  assertColor(inputs, 0, 255, 255);  // passive stream: cyan

  inputs.state = safety::State::Estop;
  assertColor(inputs, 255, 0, 0);  // estop/fault: red
  inputs.state = safety::State::RcManual;
  inputs.watchdog_stalled = true;
  assertColor(inputs, 255, 0, 0);  // watchdog stall overrides armed green
}

void test_rgb_frame_applies_blink_gate() {
  Inputs inputs;
  inputs.state = safety::State::Disarmed;  // heartbeat: on 0-99, off 100+
  const app::status_led::Rgb on = app::status_led::rgbFrame(inputs, 0);
  TEST_ASSERT_EQUAL_UINT8(0, on.r);
  TEST_ASSERT_EQUAL_UINT8(0, on.g);
  TEST_ASSERT_EQUAL_UINT8(255, on.b);
  const app::status_led::Rgb off = app::status_led::rgbFrame(inputs, 100);
  TEST_ASSERT_EQUAL_UINT8(0, off.r);
  TEST_ASSERT_EQUAL_UINT8(0, off.g);
  TEST_ASSERT_EQUAL_UINT8(0, off.b);

  inputs.state = safety::State::RcManual;  // armed: solid, never gated off
  const app::status_led::Rgb armed = app::status_led::rgbFrame(inputs, 12345);
  TEST_ASSERT_EQUAL_UINT8(0, armed.r);
  TEST_ASSERT_EQUAL_UINT8(255, armed.g);
  TEST_ASSERT_EQUAL_UINT8(0, armed.b);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_healthy_state_patterns);
  RUN_TEST(test_arming_distinguishes_discovery_and_pose_wait);
  RUN_TEST(test_maintenance_and_passive_patterns);
  RUN_TEST(test_fault_codes_match_fault_reason_values);
  RUN_TEST(test_colors_distinguish_states_and_modes);
  RUN_TEST(test_rgb_frame_applies_blink_gate);
  return UNITY_END();
}
