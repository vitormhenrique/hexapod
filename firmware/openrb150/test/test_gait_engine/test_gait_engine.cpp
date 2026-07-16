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

void test_requested_duty_is_honored_above_stable_minimum() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.duty_x255 = 204;  // 0.8
  ge.configure(d);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.8f, ge.dutyFactor());

  // The request follows gait changes but cannot undercut a gait's nominal
  // support requirement.
  ge.setGait(GaitId::Wave);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.833f, ge.dutyFactor());
}

void test_requested_duty_is_bounded_to_leave_swing_time() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.duty_x255 = 255;
  ge.configure(d);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kMaxDutyFactor, ge.dutyFactor());
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
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
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

void test_swing_finishes_with_vertical_touchdown_approach() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Wave);
  d.speed_x255 = 255;
  ge.configure(d);
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
  ge.reset();

  // At 1.2 Hz, these samples put leg 0 late in its short Wave-gait swing.
  // Horizontal placement is already complete, while the foot is descending.
  GaitOutput early;
  ge.update(800, early);
  GaitOutput late;
  ge.update(25, late);
  TEST_ASSERT_TRUE(early.feet[0].swing);
  TEST_ASSERT_TRUE(late.feet[0].swing);
  // Horizontal placement is complete (sub-0.1 mm residual comes only from the
  // twist tracker's converging tail scaling the stroke), while the foot is
  // clearly descending.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, early.feet[0].x_mm, late.feet[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, early.feet[0].y_mm, late.feet[0].y_mm);
  TEST_ASSERT_TRUE(late.feet[0].z_mm < early.feet[0].z_mm);
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

void test_centered_twist_holds_planted_home_stance() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.speed_x255 = 255;
  ge.configure(d);
  BodyTwist t;
  t.vx = kMotionDeadband * 0.5f;
  t.vy = -kMotionDeadband * 0.5f;
  ge.setTwist(t);
  ge.reset();

  const float initial_phase = ge.phase();
  for (int i = 0; i < 20; ++i) {
    GaitOutput out;
    ge.update(50, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, initial_phase, ge.phase());
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      TEST_ASSERT_FALSE(out.feet[leg].swing);
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][0], out.feet[leg].x_mm);
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][1], out.feet[leg].y_mm);
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, -static_cast<float>(d.body_height_mm),
                               out.feet[leg].z_mm);
    }
  }
}

void test_twist_is_slew_limited_by_speed_setting() {
  GaitDefaults slow_defaults = defaultGait();
  slow_defaults.gait = static_cast<uint8_t>(GaitId::Tripod);
  slow_defaults.speed_x255 = 0;
  GaitEngine slow;
  slow.configure(slow_defaults);
  BodyTwist command;
  command.vx = 1.0f;
  slow.setTwist(command);
  GaitOutput slow_out;
  slow.update(100, slow_out);

  GaitDefaults fast_defaults = slow_defaults;
  fast_defaults.speed_x255 = 255;
  GaitEngine fast;
  fast.configure(fast_defaults);
  fast.setTwist(command);
  GaitOutput fast_out;
  fast.update(100, fast_out);

  const float slow_stroke = fabsf(slow_out.feet[0].x_mm - kHomeXy[0][0]);
  const float fast_stroke = fabsf(fast_out.feet[0].x_mm - kHomeXy[0][0]);
  TEST_ASSERT_TRUE(slow_stroke > 0.0f);
  TEST_ASSERT_TRUE(fast_stroke > slow_stroke);
  TEST_ASSERT_TRUE(fast_stroke < static_cast<float>(fast_defaults.stride_len_mm));
}

void test_centering_command_settles_smoothly_then_parks_phase() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.speed_x255 = 128;
  ge.configure(d);
  BodyTwist command;
  command.vx = 1.0f;
  ge.setTwist(command);
  GaitOutput out;
  for (int i = 0; i < 30; ++i) ge.update(20, out);

  command.vx = 0.0f;
  ge.setTwist(command);
  const float phase_before_settle = ge.phase();
  ge.update(20, out);
  TEST_ASSERT_TRUE(ge.phase() != phase_before_settle);

  for (int i = 0; i < 40; ++i) ge.update(20, out);
  const float parked_phase = ge.phase();
  ge.update(200, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, parked_phase, ge.phase());
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FALSE(out.feet[leg].swing);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][0], out.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][1], out.feet[leg].y_mm);
  }
}

void test_rc_body_height_pot_neutral_at_center() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kRcBodyHeightNeutralMm, rcBodyHeightMm(0.5f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kRcBodyHeightMinMm, rcBodyHeightMm(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kRcBodyHeightMaxMm, rcBodyHeightMm(1.0f));
  // Monotonic through the centre.
  TEST_ASSERT_TRUE(rcBodyHeightMm(0.25f) < kRcBodyHeightNeutralMm);
  TEST_ASSERT_TRUE(rcBodyHeightMm(0.75f) > kRcBodyHeightNeutralMm);
}

// The whole Pot2 sweep must keep the planted home-XY feet inside the reach
// annulus WITHOUT engaging the reach clamp -- the clamp slides feet radially
// inward, which is exactly the "feet don't stay planted" failure.
void test_rc_body_height_envelope_keeps_feet_planted() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  for (int i = 0; i <= 10; ++i) {
    const float frac = static_cast<float>(i) / 10.0f;
    const float bh = rcBodyHeightMm(frac);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      bool reach_limited = false;
      IkResult r = bk.solveBodyLimited(leg, kHomeXy[leg][0], kHomeXy[leg][1],
                                       -bh, reach_limited);
      TEST_ASSERT_TRUE(r.reachable);
      TEST_ASSERT_FALSE(reach_limited);
    }
  }
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
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
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
  RUN_TEST(test_requested_duty_is_honored_above_stable_minimum);
  RUN_TEST(test_requested_duty_is_bounded_to_leave_swing_time);
  RUN_TEST(test_targets_bounded_over_full_cycle);
  RUN_TEST(test_swing_lift_reaches_step_height);
  RUN_TEST(test_swing_finishes_with_vertical_touchdown_approach);
  RUN_TEST(test_forward_twist_moves_stance_foot_backward);
  RUN_TEST(test_centered_twist_holds_planted_home_stance);
  RUN_TEST(test_twist_is_slew_limited_by_speed_setting);
  RUN_TEST(test_centering_command_settles_smoothly_then_parks_phase);
  RUN_TEST(test_rc_body_height_pot_neutral_at_center);
  RUN_TEST(test_rc_body_height_envelope_keeps_feet_planted);
  RUN_TEST(test_all_gait_targets_are_ik_reachable);
  RUN_TEST(test_phase_wraps_and_advances);
  return UNITY_END();
}
