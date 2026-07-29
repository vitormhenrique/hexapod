// Native tests for the portable RC conditioning primitives.

#include <unity.h>

#include "config/config_schema.h"
#include "input/rc_input_conditioner.h"

namespace {

config::RcInputCalibration calibration(uint16_t tau_ms = 100) {
  config::RcInputCalibration result;
  config::defaultRcInputCalibration(result);
  for (uint8_t index = 0; index < 4; ++index) {
    result.channels[index].filter_tau_ms = tau_ms;
    result.channels[index].deadband_x255 = 0;
    result.channels[index].expo_x255 = 0;
  }
  return result;
}

}  // namespace

void test_ema_is_consistent_across_equivalent_dt() {
  const config::RcInputCalibration cfg = calibration(/*tau_ms=*/100);
  controller::RcInputConditioner fine;
  controller::RcInputConditioner coarse;
  fine.configure(cfg);
  coarse.configure(cfg);
  fine.update(1, 0.0f, 0);
  coarse.update(1, 0.0f, 0);

  float fine_output = 0.0f;
  for (int index = 0; index < 10; ++index) {
    fine_output = fine.update(1, 1.0f, 10);
  }
  float coarse_output = 0.0f;
  for (int index = 0; index < 5; ++index) {
    coarse_output = coarse.update(1, 1.0f, 20);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.002f, fine_output, coarse_output);
  TEST_ASSERT_TRUE(fine_output > 0.0f && fine_output < 1.0f);
}

void test_zero_tau_bypasses_ema() {
  config::RcInputCalibration cfg = calibration();
  cfg.channels[0].filter_tau_ms = 0;
  controller::RcInputConditioner conditioner;
  conditioner.configure(cfg);
  conditioner.update(1, 0.0f, 0);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f,
                           conditioner.update(1, 0.75f, 10));
}

void test_median3_rejects_one_isolated_spike() {
  config::RcInputCalibration cfg = calibration(/*tau_ms=*/0);
  controller::RcInputConditioner conditioner;
  conditioner.configure(cfg);
  TEST_ASSERT_TRUE(conditioner.setMode(
      controller::InputFilterMode::Median3Ema, false));
  conditioner.update(1, 0.0f, 0);
  conditioner.update(1, 0.0f, 10);
  conditioner.update(1, 0.0f, 10);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, conditioner.update(1, 1.0f, 10));
}

void test_deadband_and_expo_preserve_continuity_sign_and_endpoints() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f,
                          controller::RcInputConditioner::applyDeadband(0.05f,
                                                                          0.05f));
  TEST_ASSERT_FLOAT_WITHIN(2e-6f, 0.0f,
                           controller::RcInputConditioner::applyDeadband(
                               0.050001f, 0.05f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f,
                           controller::RcInputConditioner::applyExpo(1.0f,
                                                                      0.35f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f,
                           controller::RcInputConditioner::applyExpo(-1.0f,
                                                                      0.35f));
  TEST_ASSERT_TRUE(controller::RcInputConditioner::applyExpo(-0.5f, 0.35f) <
                   0.0f);
}

void test_filter_mode_switch_is_bumpless_and_diagnostic_bypass_is_gated() {
  const config::RcInputCalibration cfg = calibration(/*tau_ms=*/100);
  controller::RcInputConditioner conditioner;
  conditioner.configure(cfg);
  conditioner.update(1, 0.0f, 0);
  const float before = conditioner.update(1, 0.7f, 10);
  TEST_ASSERT_TRUE(conditioner.setMode(
      controller::InputFilterMode::Median3Ema, false));
  const float after = conditioner.update(1, 1.0f, 10);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, before, after);
  TEST_ASSERT_FALSE(conditioner.setMode(
      controller::InputFilterMode::NoneDiagnostic, false));
  TEST_ASSERT_TRUE(conditioner.setMode(
      controller::InputFilterMode::NoneDiagnostic, true));
}

void test_tri_switch_requires_a_stable_position() {
  controller::RcTriSwitchDebouncer debouncer;
  TEST_ASSERT_EQUAL_UINT8(0, debouncer.update(0, 0, 40));
  TEST_ASSERT_EQUAL_UINT8(0, debouncer.update(1, 10, 40));
  TEST_ASSERT_EQUAL_UINT8(0, debouncer.update(0, 20, 40));
  TEST_ASSERT_EQUAL_UINT8(0, debouncer.update(1, 30, 40));
  TEST_ASSERT_EQUAL_UINT8(0, debouncer.update(1, 69, 40));
  TEST_ASSERT_EQUAL_UINT8(1, debouncer.update(1, 70, 40));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_ema_is_consistent_across_equivalent_dt);
  RUN_TEST(test_zero_tau_bypasses_ema);
  RUN_TEST(test_median3_rejects_one_isolated_spike);
  RUN_TEST(test_deadband_and_expo_preserve_continuity_sign_and_endpoints);
  RUN_TEST(test_filter_mode_switch_is_bumpless_and_diagnostic_bypass_is_gated);
  RUN_TEST(test_tri_switch_requires_a_stable_position);
  return UNITY_END();
}