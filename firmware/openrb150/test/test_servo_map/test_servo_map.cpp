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

  // Mark III wiring normalized to firmware leg order LR, RR, RM, RF, LF, LM.
  TEST_ASSERT_EQUAL_PTR(nullptr, map.servoFor(99, 0));
  const ServoConfig* leg1_coxa = map.servoFor(0, 0);
  TEST_ASSERT_NOT_NULL(leg1_coxa);
  TEST_ASSERT_EQUAL_UINT8(7, leg1_coxa->id);
  TEST_ASSERT_EQUAL_UINT8(9, map.servoFor(0, 1)->id);
  TEST_ASSERT_EQUAL_UINT8(11, map.servoFor(0, 2)->id);
  TEST_ASSERT_EQUAL_UINT8(17, map.servoFor(5, 2)->id);
}

void test_lookup_by_id() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);

  const ServoConfig* s = map.servoForId(11);  // left-rear tibia
  TEST_ASSERT_NOT_NULL(s);
  TEST_ASSERT_EQUAL_UINT8(0, s->leg);
  TEST_ASSERT_EQUAL_UINT8(2, s->joint);
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

  // +45 deg. Mark III mirrors the right side at the servo output.
  const float angle = 45.0f * kDegToRad;
  JointCommand left = map.angleToTick(0, 0, angle);
  JointCommand right = map.angleToTick(1, 0, angle);
  assertTickNear(kServoCenterTick + 512, left.tick);
  assertTickNear(kServoCenterTick - 512, right.tick);
  TEST_ASSERT_FALSE(left.clamped_high);
  TEST_ASSERT_FALSE(right.clamped_high);
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
  // Left-rear coxa uses the Mark III +/-75 degree travel window.
  ServoMap map(cfg);
  JointCommand c = map.angleToTick(0, 0, 120.0f * kDegToRad);
  TEST_ASSERT_TRUE(c.clamped_high);
  TEST_ASSERT_FALSE(c.clamped_low);
  TEST_ASSERT_EQUAL_UINT16(2901, c.tick);
}

void test_clamp_low_reported() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  ServoMap map(cfg);
  // Left-rear coxa, -120 deg => below the -75 degree floor.
  JointCommand c = map.angleToTick(0, 0, -120.0f * kDegToRad);
  TEST_ASSERT_TRUE(c.clamped_low);
  TEST_ASSERT_FALSE(c.clamped_high);
  TEST_ASSERT_EQUAL_UINT16(1195, c.tick);
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

  // Right-rear coxa is inverted. Pick an in-range angle.
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
