// Native (host) unit tests for the gait engine v1. No Arduino deps.
// Run with: pio test -e native

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/gait/body_ik.h"
#include "../../src/gait/gait_engine.h"

using namespace gait;
using namespace config;

namespace {

constexpr float kHomeXy[kNumLegs][2] = {
    {-155.4f, -205.4f}, {155.4f, -205.4f}, {196.8f, 0.0f},
    {155.4f, 205.4f},   {-155.4f, 205.4f}, {-196.8f, 0.0f},
};

GaitDefaults defaultGait() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  return cfg.gait;
}

}  // namespace

void test_stand_holds_home_stance() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Stand);
  ge.configure(d);
  GaitOutput out;
  ge.update(20, out);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FALSE(out.feet[leg].swing);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][0], out.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][1], out.feet[leg].y_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -static_cast<float>(d.body_height_mm),
                             out.feet[leg].z_mm);
  }
}

void test_sit_lowers_body() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Sit);
  ge.configure(d);
  GaitOutput out;
  ge.update(20, out);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FALSE(out.feet[leg].swing);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][0], out.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kSitFootZMm, out.feet[leg].z_mm);
  }
}

void test_tripod_groups_are_opposite_at_phase_zero() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  ge.configure(d);
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
  ge.reset();
  // First tiny tick leaves phase ~0: group A {0,2,3} stance, B {1,4,5} swing.
  GaitOutput out;
  ge.update(1, out);
  TEST_ASSERT_FALSE(out.feet[0].swing);
  TEST_ASSERT_FALSE(out.feet[2].swing);
  TEST_ASSERT_FALSE(out.feet[3].swing);
  TEST_ASSERT_TRUE(out.feet[1].swing);
  TEST_ASSERT_TRUE(out.feet[4].swing);
  TEST_ASSERT_TRUE(out.feet[5].swing);
}

void test_duty_factors() {
  GaitEngine ge;
  ge.setGait(GaitId::Tripod);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, ge.dutyFactor());
  ge.setGait(GaitId::Ripple);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.667f, ge.dutyFactor());
  ge.setGait(GaitId::Wave);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.833f, ge.dutyFactor());
  ge.setGait(GaitId::Stand);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, ge.dutyFactor());
}

void test_targets_bounded_over_full_cycle() {
  const GaitId gaits[] = {GaitId::Tripod, GaitId::Ripple, GaitId::Wave,
                          GaitId::Crawl};
  for (GaitId g : gaits) {
    GaitEngine ge;
    GaitDefaults d = defaultGait();
    d.gait = static_cast<uint8_t>(g);
    d.speed_x255 = 255;  // fastest
    ge.configure(d);
    BodyTwist t;
    t.vx = 1.0f;
    t.vy = 1.0f;
    t.wz = 1.0f;  // worst-case combined command
    ge.setTwist(t);
    ge.reset();

    for (int i = 0; i < 200; ++i) {
      GaitOutput out;
      ge.update(20, out);  // 50 Hz
      for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
        const float dx = out.feet[leg].x_mm - kHomeXy[leg][0];
        const float dy = out.feet[leg].y_mm - kHomeXy[leg][1];
        TEST_ASSERT_TRUE(fabsf(dx) <= kMaxStrideMm + 1.0f);
        TEST_ASSERT_TRUE(fabsf(dy) <= kMaxStrideMm + 1.0f);
        TEST_ASSERT_TRUE(out.feet[leg].z_mm >= kMinFootZMm - 1e-3f);
        TEST_ASSERT_TRUE(out.feet[leg].z_mm <= kMaxFootZMm + 1e-3f);
        TEST_ASSERT_FALSE(isnan(out.feet[leg].x_mm));
      }
    }
  }
}

void test_swing_lift_reaches_step_height() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Wave);
  d.speed_x255 = 255;
  ge.configure(d);
  ge.reset();
  const float home_z = -static_cast<float>(d.body_height_mm);
  float max_lift = 0.0f;
  for (int i = 0; i < 400; ++i) {
    GaitOutput out;
    ge.update(10, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      if (out.feet[leg].swing) {
        max_lift = fmaxf(max_lift, out.feet[leg].z_mm - home_z);
      }
    }
  }
  // Peak lift should approach the configured step height (30 mm default).
  TEST_ASSERT_FLOAT_WITHIN(2.0f, static_cast<float>(d.step_height_mm), max_lift);
}

void test_forward_twist_moves_stance_foot_backward() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Wave);
  ge.configure(d);
  BodyTwist t;
  t.vx = 1.0f;  // forward
  ge.setTwist(t);
  ge.reset();
  // Leg 0 offset 0: starts stance at L=+0.5 -> foot x ahead of home, sweeping
  // back. Sample early stance and confirm x is ahead of home (positive stroke).
  GaitOutput out;
  ge.update(1, out);
  TEST_ASSERT_FALSE(out.feet[0].swing);
  TEST_ASSERT_TRUE(out.feet[0].x_mm > kHomeXy[0][0]);
}

void test_all_gait_targets_are_ik_reachable() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  GaitEngine ge;
  GaitDefaults d = cfg.gait;
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.stride_len_mm = 8;  // small stride: home stance sits near full leg reach
  ge.configure(d);
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
  ge.reset();

  for (int i = 0; i < 100; ++i) {
    GaitOutput out;
    ge.update(20, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      IkResult r = bk.solveBody(leg, out.feet[leg].x_mm, out.feet[leg].y_mm,
                                out.feet[leg].z_mm);
      TEST_ASSERT_TRUE(r.reachable);
    }
  }
}

void test_phase_wraps_and_advances() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.speed_x255 = 255;
  ge.configure(d);
  ge.reset();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ge.phase());
  for (int i = 0; i < 500; ++i) {
    GaitOutput out;
    ge.update(20, out);
    TEST_ASSERT_TRUE(ge.phase() >= 0.0f);
    TEST_ASSERT_TRUE(ge.phase() < 1.0f);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_holds_home_stance);
  RUN_TEST(test_sit_lowers_body);
  RUN_TEST(test_tripod_groups_are_opposite_at_phase_zero);
  RUN_TEST(test_duty_factors);
  RUN_TEST(test_targets_bounded_over_full_cycle);
  RUN_TEST(test_swing_lift_reaches_step_height);
  RUN_TEST(test_forward_twist_moves_stance_foot_backward);
  RUN_TEST(test_all_gait_targets_are_ik_reachable);
  RUN_TEST(test_phase_wraps_and_advances);
  return UNITY_END();
}
