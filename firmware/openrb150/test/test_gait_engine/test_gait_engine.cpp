// Native (host) unit tests for the gait engine v1. No Arduino deps.
// Run with: pio test -e native

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/gait/body_ik.h"
#include "../../src/gait/gait_engine.h"
#include "../../src/gait/leg_ik.h"

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
  // First tiny tick leaves phase ~0: alternating tripods {1,3,5} (legs
  // 0,2,4: rear-left, mid-right, front-left) stance, {2,4,6} (legs 1,3,5)
  // swing. Each group is a balanced triangle straddling the body centre.
  GaitOutput out;
  ge.update(1, out);
  TEST_ASSERT_FALSE(out.feet[0].swing);
  TEST_ASSERT_FALSE(out.feet[2].swing);
  TEST_ASSERT_FALSE(out.feet[4].swing);
  TEST_ASSERT_TRUE(out.feet[1].swing);
  TEST_ASSERT_TRUE(out.feet[3].swing);
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

// The swing profile's endpoint slopes match the stance sweep rate, so the
// foot's body-frame velocity is continuous through touchdown (zero scuff in
// the world frame). Sample x-velocity across leg 0's swing->stance flip and
// require no visible step.
void test_touchdown_velocity_is_continuous() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Wave);
  d.speed_x255 = 255;
  ge.configure(d);
  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
  ge.reset();

  // Let the twist tracker converge so the stroke is steady.
  GaitOutput prev;
  for (int i = 0; i < 150; ++i) ge.update(10, prev);

  // Walk in fine 2 ms steps until leg 0 flips swing -> stance; compare the
  // per-step x velocity just before and across the flip.
  float vel_before = 0.0f;
  for (int i = 0; i < 2000; ++i) {
    GaitOutput cur;
    ge.update(2, cur);
    const float step_x = cur.feet[0].x_mm - prev.feet[0].x_mm;
    if (prev.feet[0].swing && !cur.feet[0].swing) {
      // Touchdown happened inside this step. The velocity across the flip
      // must be close to the last in-swing velocity (no jolt).
      TEST_ASSERT_FLOAT_WITHIN(0.12f, vel_before, step_x);
      return;
    }
    vel_before = step_x;
    prev = cur;
  }
  TEST_FAIL_MESSAGE("leg 0 never touched down");
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

// The whole Pot2 sweep rides the URDF-derived constant distal-link orientation
// locus: every stance target must stay reachable WITHOUT engaging the reach
// clamp (the clamp would drag feet off the locus), and the radius must shrink
// as the body rises (legs reconfigure, not just the knee).
void test_rc_body_height_envelope_keeps_feet_reachable() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  float prev_r = 1e9f;
  for (int i = 0; i <= 10; ++i) {
    const float frac = static_cast<float>(i) / 10.0f;
    const float bh = rcBodyHeightMm(frac);
    GaitEngine ge;
    GaitDefaults d = defaultGait();
    d.gait = static_cast<uint8_t>(GaitId::Stand);
    d.body_height_mm = static_cast<uint16_t>(bh + 0.5f);
    ge.configure(d);
    GaitOutput out;
    ge.update(20, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      bool reach_limited = false;
      IkResult r = bk.solveBodyLimited(leg, out.feet[leg].x_mm,
                                       out.feet[leg].y_mm,
                                       out.feet[leg].z_mm, reach_limited);
      TEST_ASSERT_TRUE(r.reachable);
      TEST_ASSERT_FALSE(reach_limited);
    }
    // Stance radius shrinks monotonically as the body rises (leg 2 is the
    // pure +X mid-right leg, so its x is the hip-relative radius + mount).
    TEST_ASSERT_TRUE(out.feet[2].x_mm < prev_r);
    prev_r = out.feet[2].x_mm;
  }
}

void test_rc_body_height_keeps_urdf_distal_direction() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
           cfg.links.tibia_cmm / 100.0f, cfg.geometry.home_radius_cmm / 100.0f,
           cfg.geometry.home_foot_z_cmm / 100.0f);

  float neutral_direction = 0.0f;
  for (int i = 0; i <= 10; ++i) {
    const float fraction = static_cast<float>(i) / 10.0f;
    const float height = rcBodyHeightMm(fraction);
    GaitEngine ge;
    GaitDefaults d = defaultGait();
    d.gait = static_cast<uint8_t>(GaitId::Stand);
    d.body_height_mm = static_cast<uint16_t>(height + 0.5f);
    ge.configure(d);
    GaitOutput out;
    ge.update(20, out);

    // Leg 3 is the middle-right leg, whose stance vector is purely radial in
    // the body frame. Reconstruct the raw URDF planar angles from the command.
    IkResult result = bk.solveBody(2, out.feet[2].x_mm, out.feet[2].y_mm,
                                   out.feet[2].z_mm);
    TEST_ASSERT_TRUE(result.reachable);
    const float direction = result.femur + ik.femurRest() + result.tibia +
                            ik.tibiaRest();
    if (i == 0) {
      neutral_direction = direction;
    } else {
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, neutral_direction, direction);
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
  RUN_TEST(test_touchdown_velocity_is_continuous);
  RUN_TEST(test_forward_twist_moves_stance_foot_backward);
  RUN_TEST(test_centered_twist_holds_planted_home_stance);
  RUN_TEST(test_twist_is_slew_limited_by_speed_setting);
  RUN_TEST(test_centering_command_settles_smoothly_then_parks_phase);
  RUN_TEST(test_rc_body_height_pot_neutral_at_center);
  RUN_TEST(test_rc_body_height_envelope_keeps_feet_reachable);
  RUN_TEST(test_rc_body_height_keeps_urdf_distal_direction);
  RUN_TEST(test_all_gait_targets_are_ik_reachable);
  RUN_TEST(test_phase_wraps_and_advances);
  return UNITY_END();
}
