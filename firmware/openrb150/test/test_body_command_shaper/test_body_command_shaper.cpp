// Native tests for high-level body command acceleration and pose limits.

#include <unity.h>

#include "config/config_schema.h"
#include "controller/body_command_shaper.h"

namespace {

config::BodyCommandLimits limits() {
  config::BodyCommandLimits value;
  config::defaultBodyCommandLimits(value);
  value.forward_accel_milli_per_s = 1000;
  value.forward_decel_milli_per_s = 2000;
  value.lateral_accel_milli_per_s = 1000;
  value.lateral_decel_milli_per_s = 2000;
  value.yaw_accel_milli_per_s = 1000;
  value.yaw_decel_milli_per_s = 2000;
  value.height_rise_mm_per_s = 10;
  value.height_lower_mm_per_s = 20;
  value.pose_translation_rate_mm_per_s = 50;
  value.pose_rotation_rate_millirad_per_s = 500;
  return value;
}

controller::BodyCommand desired(float vx, float vy, float wz) {
  controller::BodyCommand value;
  value.vx = vx;
  value.vy = vy;
  value.wz = wz;
  value.body_height_mm = 40.0f;
  return value;
}

}  // namespace

void test_step_respects_axis_acceleration_limits() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  const controller::BodyCommand& output =
      shaper.update(desired(1.0f, 1.0f, 1.0f), 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, output.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, output.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, output.wz);
}

void test_return_and_reversal_use_deceleration_before_acceleration() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  shaper.update(desired(1.0f, 0.0f, 0.0f), 10);  // +0.01

  const controller::BodyCommand& braking =
        shaper.update(desired(0.0f, 0.0f, 0.0f), 10);
      TEST_ASSERT_EQUAL_FLOAT(0.0f, braking.vx);  // decel is 0.02 / 10 ms

  const controller::BodyCommand& reverse =
        shaper.update(desired(-1.0f, 0.0f, 0.0f), 10);
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.01f, reverse.vx);
}

void test_axis_maxima_clamp_the_desired_command() {
  config::BodyCommandLimits config = limits();
  config.max_forward_milli = 500;
  config.max_reverse_milli = 250;
  controller::BodyCommandShaper shaper;
  shaper.configure(config);
  shaper.reset(40.0f);
  for (int index = 0; index < 10; ++index) {
    shaper.update(desired(1.0f, 0.0f, 0.0f), 100);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, shaper.current().vx);
  for (int index = 0; index < 10; ++index) {
    shaper.update(desired(-1.0f, 0.0f, 0.0f), 100);
  }
  TEST_ASSERT_TRUE(shaper.current().vx >= -0.25f);
}

void test_height_and_pose_are_rate_limited() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  controller::BodyCommand target = desired(0.0f, 0.0f, 0.0f);
  target.body_height_mm = 50.0f;
  target.pose.x_mm = 20.0f;
  target.pose.roll = 0.5f;
  const controller::BodyCommand& output = shaper.update(target, 10);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 40.1f, output.body_height_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, output.pose.x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.005f, output.pose.roll);
}

void test_height_override_is_hard_capped() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  controller::BodyCommand target = desired(0.0f, 0.0f, 0.0f);
  target.body_height_mm = 100.0f;
  target.height_rate_override_mm_per_s = 1000.0f;
  const controller::BodyCommand& output = shaper.update(target, 10);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-6f, 42.0f, output.body_height_mm);  // 200 mm/s hard ceiling
}

void test_large_dt_cannot_integrate_a_giant_command_step() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  const controller::BodyCommand& output =
      shaper.update(desired(1.0f, 0.0f, 0.0f), 500);
  // Integration is bounded to 50 ms: 1.0 normalized/s * 0.05 s.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, output.vx);
}

void test_direct_diagnostic_is_gated_and_bumpless() {
  controller::BodyCommandShaper shaper;
  shaper.configure(limits());
  shaper.reset(40.0f);
  shaper.update(desired(1.0f, 0.0f, 0.0f), 100);
  const float before = shaper.current().vx;
  TEST_ASSERT_FALSE(shaper.setMode(controller::CommandShaperMode::DirectDiagnostic,
                                   false));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, before, shaper.current().vx);
  TEST_ASSERT_TRUE(shaper.setMode(controller::CommandShaperMode::DirectDiagnostic,
                                  true));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f,
                           shaper.update(desired(0.5f, 0.0f, 0.0f), 100).vx);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_step_respects_axis_acceleration_limits);
  RUN_TEST(test_return_and_reversal_use_deceleration_before_acceleration);
  RUN_TEST(test_axis_maxima_clamp_the_desired_command);
  RUN_TEST(test_height_and_pose_are_rate_limited);
  RUN_TEST(test_height_override_is_hard_capped);
  RUN_TEST(test_large_dt_cannot_integrate_a_giant_command_step);
  RUN_TEST(test_direct_diagnostic_is_gated_and_bumpless);
  return UNITY_END();
}