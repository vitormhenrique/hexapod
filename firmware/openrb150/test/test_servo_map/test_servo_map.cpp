// Native (host) unit tests for the servo map + joint-limit enforcement.
// No Arduino deps.
//
// Run with: pio test -e native

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/servo_map.h"

using namespace dxl;
using namespace config;

namespace {

// Approx tick equality (rounding tolerance).
void assertTickNear(uint16_t expected, uint16_t got, uint16_t tol = 1) {
  const int d = static_cast<int>(expected) - static_cast<int>(got);
  TEST_ASSERT_TRUE(d <= static_cast<int>(tol) && d >= -static_cast<int>(tol));
}

}  // namespace

void test_lookup_by_slot_matches_wiring() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);

  // Wiring: leg1 coxa id 1, femur id 7, tibia id 13; leg6 tibia id 18.
  TEST_ASSERT_EQUAL_PTR(nullptr, map.servoFor(99, 0));
  const ServoConfig* leg1_coxa = map.servoFor(0, 0);
  TEST_ASSERT_NOT_NULL(leg1_coxa);
  TEST_ASSERT_EQUAL_UINT8(1, leg1_coxa->id);
  TEST_ASSERT_EQUAL_UINT8(7, map.servoFor(0, 1)->id);
  TEST_ASSERT_EQUAL_UINT8(13, map.servoFor(0, 2)->id);
  TEST_ASSERT_EQUAL_UINT8(18, map.servoFor(5, 2)->id);
}

void test_lookup_by_id() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);

  const ServoConfig* s = map.servoForId(11);  // leg5 femur
  TEST_ASSERT_NOT_NULL(s);
  TEST_ASSERT_EQUAL_UINT8(4, s->leg);   // leg index 4 == "leg 5"
  TEST_ASSERT_EQUAL_UINT8(1, s->joint); // femur
  TEST_ASSERT_EQUAL_PTR(nullptr, map.servoForId(200));
}

void test_zero_angle_maps_to_center() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);

  JointCommand c = map.angleToTick(0, 0, 0.0f);
  TEST_ASSERT_FALSE(c.unmapped);
  TEST_ASSERT_FALSE(c.clamped_low);
  TEST_ASSERT_FALSE(c.clamped_high);
  TEST_ASSERT_EQUAL_UINT16(kServoCenterTick, c.tick);
}

void test_positive_angle_applies_sign() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);

  // +45 deg. Left leg (leg 0) sign +1 -> tick above center; right leg (leg 1)
  // sign -1 -> tick below center. Offset magnitude = 45 * 4096/360 = 512.
  const float angle = 45.0f * kDegToRad;
  JointCommand left = map.angleToTick(0, 0, angle);
  JointCommand right = map.angleToTick(1, 0, angle);
  assertTickNear(kServoCenterTick + 512, left.tick);
  assertTickNear(kServoCenterTick - 512, right.tick);
  TEST_ASSERT_FALSE(left.clamped_high);
  TEST_ASSERT_FALSE(right.clamped_low);
}

void test_trim_offsets_center() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.servos[0].trim_ticks = 30;  // leg1 coxa mechanical trim
  ServoMap map(cfg);

  JointCommand c = map.angleToTick(0, 0, 0.0f);
  TEST_ASSERT_EQUAL_UINT16(kServoCenterTick + 30, c.tick);
}

void test_clamp_high_reported() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  // Default travel is [1024, 3072]. +120 deg => offset ~1365 => 3413 > 3072.
  ServoMap map(cfg);
  JointCommand c = map.angleToTick(0, 0, 120.0f * kDegToRad);
  TEST_ASSERT_TRUE(c.clamped_high);
  TEST_ASSERT_FALSE(c.clamped_low);
  TEST_ASSERT_EQUAL_UINT16(3072, c.tick);
}

void test_clamp_low_reported() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);
  // Left leg, -120 deg => below 1024 floor.
  JointCommand c = map.angleToTick(0, 0, -120.0f * kDegToRad);
  TEST_ASSERT_TRUE(c.clamped_low);
  TEST_ASSERT_FALSE(c.clamped_high);
  TEST_ASSERT_EQUAL_UINT16(1024, c.tick);
}

void test_unmapped_slot() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);
  JointCommand c = map.angleToTick(0, 5, 0.3f);  // joint 5 does not exist
  TEST_ASSERT_TRUE(c.unmapped);
  TEST_ASSERT_EQUAL_UINT16(kServoCenterTick, c.tick);
}

void test_round_trip_angle_tick_angle() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  cfg.servos[3].trim_ticks = -25;  // exercise trim in the inverse too
  ServoMap map(cfg);

  // leg index 1 (right, sign -1), coxa. Pick an in-range angle.
  const float angle = 30.0f * kDegToRad;
  JointCommand c = map.angleToTick(1, 0, angle);
  TEST_ASSERT_FALSE(c.clamped_low);
  TEST_ASSERT_FALSE(c.clamped_high);
  const float back = map.tickToAngle(1, 0, c.tick);
  // Within one tick of rounding (~0.088 deg ~= 0.0015 rad).
  TEST_ASSERT_FLOAT_WITHIN(0.003f, angle, back);
}

void test_tick_to_angle_unmapped_is_zero() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, map.tickToAngle(0, 9, 3000));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_lookup_by_slot_matches_wiring);
  RUN_TEST(test_lookup_by_id);
  RUN_TEST(test_zero_angle_maps_to_center);
  RUN_TEST(test_positive_angle_applies_sign);
  RUN_TEST(test_trim_offsets_center);
  RUN_TEST(test_clamp_high_reported);
  RUN_TEST(test_clamp_low_reported);
  RUN_TEST(test_unmapped_slot);
  RUN_TEST(test_round_trip_angle_tick_angle);
  RUN_TEST(test_tick_to_angle_unmapped_is_zero);
  return UNITY_END();
}
