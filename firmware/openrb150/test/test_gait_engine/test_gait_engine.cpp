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

// Home feet = measured coxa mount + 126.75 mm along each leg's home azimuth
// (dimensions.md documents these rounded to 0.1 mm: +-155.2 / +-205.2 /
// +-196.5). The engine derives them from the mounts so the stance radius can
// be a live parameter; the exact values are asserted here.
constexpr float kHomeRadial45 = 126.75f * 0.70710678f;  // 89.626
constexpr float kHomeXy[kNumLegs][2] = {
  {-65.6f - kHomeRadial45, -115.6f - kHomeRadial45},
  {65.6f + kHomeRadial45, -115.6f - kHomeRadial45},
  {69.8f + 126.75f, 0.0f},
  {65.6f + kHomeRadial45, 115.6f + kHomeRadial45},
  {-65.6f - kHomeRadial45, 115.6f + kHomeRadial45},
  {-69.8f - 126.75f, 0.0f},
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
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f / 8.0f, ge.dutyFactor());
  ge.setGait(GaitId::Ripple);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 9.0f / 12.0f, ge.dutyFactor());
  ge.setGait(GaitId::Wave);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 21.0f / 24.0f, ge.dutyFactor());
  ge.setGait(GaitId::Stand);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, ge.dutyFactor());
}

void test_requested_duty_does_not_change_mark_iii_pattern() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.duty_x255 = 255;
  ge.configure(d);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f / 8.0f, ge.dutyFactor());
  ge.setGait(GaitId::Wave);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 21.0f / 24.0f, ge.dutyFactor());
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
        // Combined full translation + radius-correct yaw can exceed one
        // translational stride while remaining bounded by the Mark III arc.
        TEST_ASSERT_TRUE(fabsf(dx) <= 130.0f);
        TEST_ASSERT_TRUE(fabsf(dy) <= 130.0f);
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

void test_crouched_walk_keeps_configured_swing_clearance() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Wave);
  d.body_height_mm = static_cast<uint16_t>(kRcBodyHeightMinMm);
  d.speed_x255 = 255;
  ge.configure(d);

  BodyTwist t;
  t.vx = 1.0f;
  ge.setTwist(t);
  ge.reset();

  float max_lift = 0.0f;
  for (int i = 0; i < 400; ++i) {
    GaitOutput out;
    ge.update(10, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      if (out.feet[leg].swing) {
        max_lift = fmaxf(max_lift,
                          out.feet[leg].z_mm + d.body_height_mm);
      }
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(2.0f, static_cast<float>(d.step_height_mm),
                           max_lift);
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
  t.vy = 1.0f;  // mechanical body +Y is forward
  ge.setTwist(t);
  ge.reset();
  float x, y, z;
  ge.motionEnvelopeFoot(0, 0.5f, 0.0f, x, y, z);
  TEST_ASSERT_TRUE(y > kHomeXy[0][1]);
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

void test_twist_is_consumed_without_a_second_response_filter() {
  GaitDefaults slow_defaults = defaultGait();
  slow_defaults.gait = static_cast<uint8_t>(GaitId::Tripod);
  slow_defaults.speed_x255 = 0;
  GaitEngine slow;
  slow.configure(slow_defaults);
  BodyTwist command;
  command.vx = 1.0f;
  slow.setTwist(command);
  float slow_x, slow_y, slow_z;
  slow.motionEnvelopeFoot(0, 0.5f, 0.0f, slow_x, slow_y, slow_z);

  GaitDefaults fast_defaults = slow_defaults;
  fast_defaults.speed_x255 = 255;
  GaitEngine fast;
  fast.configure(fast_defaults);
  fast.setTwist(command);
  float fast_x, fast_y, fast_z;
  fast.motionEnvelopeFoot(0, 0.5f, 0.0f, fast_x, fast_y, fast_z);

  const float slow_stroke = fabsf(slow_x - kHomeXy[0][0]);
  const float fast_stroke = fabsf(fast_x - kHomeXy[0][0]);
  TEST_ASSERT_TRUE(slow_stroke > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, slow_stroke, fast_stroke);
}

void test_centering_shaped_command_parks_phase() {
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
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, phase_before_settle, ge.phase());
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FALSE(out.feet[leg].swing);
  }
  // Once the shaped command reaches zero every foot remains planted at home.
  for (int i = 0; i < 150; ++i) {
    ge.update(20, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      TEST_ASSERT_FALSE(out.feet[leg].swing);
    }
  }
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FLOAT_WITHIN(0.5f, kHomeXy[leg][0], out.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, kHomeXy[leg][1], out.feet[leg].y_mm);
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

// Height changes keep stance X/Y fixed and vary only foot Z. Every
// point in the configured Pot2 envelope must remain inside the safe annulus.
void test_rc_body_height_envelope_keeps_feet_reachable() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
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
      TEST_ASSERT_FLOAT_WITHIN(0.1f, kHomeXy[leg][0], out.feet[leg].x_mm);
      TEST_ASSERT_FLOAT_WITHIN(0.1f, kHomeXy[leg][1], out.feet[leg].y_mm);
    }
  }
}

// Pot2 fully down is the measured full-crouch pose: belly near the ground with
// the stance X/Y unchanged. Pin the joint commands it produces (the operator
// reads these on the Joint Matrix page as 180 + deg) and prove every joint
// still fits inside the configured servo travel, so lowering the envelope can
// never silently start clamping a leg.
void test_rc_body_height_minimum_is_the_measured_crouch() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kRcBodyHeightMinMm, rcBodyHeightMm(0.0f));

  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Stand);
  d.body_height_mm = static_cast<uint16_t>(kRcBodyHeightMinMm + 0.5f);
  ge.configure(d);
  GaitOutput out;
  ge.update(20, out);

  constexpr float kRadToDeg = 57.2957795f;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    bool reach_limited = false;
    const IkResult r = bk.solveBodyLimited(leg, out.feet[leg].x_mm,
                                           out.feet[leg].y_mm,
                                           out.feet[leg].z_mm, reach_limited);
    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FALSE(reach_limited);
    // Joint Matrix reads 180 + these: femur ~240 deg, tibia ~128 deg.
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, r.coxa * kRadToDeg);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 60.0f, r.femur * kRadToDeg);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -52.0f, r.tibia * kRadToDeg);
    // Configured travel is +/-90 deg about the 2048 centre; stay inside it.
    TEST_ASSERT_TRUE(fabsf(r.femur * kRadToDeg) < 90.0f);
    TEST_ASSERT_TRUE(fabsf(r.tibia * kRadToDeg) < 90.0f);
  }
}

// Swing clearance is a gait parameter, not a function of stick deflection: a
// barely-above-deadband command must still lift swing feet by the full
// configured step height so slow walking never drags feet.
void test_low_command_keeps_full_swing_clearance() {
  GaitEngine ge;
  GaitDefaults defaults = defaultGait();
  defaults.gait = static_cast<uint8_t>(GaitId::Tripod);
  defaults.speed_x255 = 128;
  ge.configure(defaults);

  BodyTwist command;
  command.vy = 0.05f;  // just above the motion deadband
  ge.setTwist(command);
  GaitOutput out;
  float peak_lift = 0.0f;
  for (int step = 0; step < 400; ++step) {
    ge.update(10, out);
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      const float lift = out.feet[leg].z_mm + defaults.body_height_mm;
      if (lift > peak_lift) peak_lift = lift;
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, static_cast<float>(defaults.step_height_mm),
                           peak_lift);
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

// --- Live shape tunables ----------------------------------------------------

namespace {

// Run the engine long enough for every first-order parameter filter to settle.
void settle(GaitEngine& ge, GaitOutput& out) {
  for (int i = 0; i < 200; ++i) ge.update(20, out);
}

// Radial distance of a foot from its coxa mount (compiled table).
float radiusFromMount(uint8_t leg, float x, float y) {
  constexpr float kMount[kNumLegs][2] = {
    {-65.6f, -115.6f}, {65.6f, -115.6f}, {69.8f, 0.0f},
    {65.6f, 115.6f},   {-65.6f, 115.6f}, {-69.8f, 0.0f},
  };
  const float dx = x - kMount[leg][0];
  const float dy = y - kMount[leg][1];
  return sqrtf(dx * dx + dy * dy);
}

// Body yaw swept by one full gait cycle for a pure yaw command: the angle
// between the stance foot's extreme positions, measured on a mid leg.
float sweptYawDeg(GaitEngine& ge) {
  BodyTwist t;
  t.wz = 1.0f;
  ge.setTwist(t);
  ge.reset();
  float min_angle = 10.0f, max_angle = -10.0f;
  for (int i = 0; i < 400; ++i) {
    GaitOutput out;
    ge.update(5, out);
    const float a = atan2f(out.feet[2].y_mm, out.feet[2].x_mm);
    if (a < min_angle) min_angle = a;
    if (a > max_angle) max_angle = a;
  }
  return (max_angle - min_angle) * 180.0f / 3.14159265f;
}

}  // namespace

void test_shape_defaults_match_compiled_gait() {
  GaitShape s;
  TEST_ASSERT_EQUAL_UINT16(60, s.get(TuneId::StrideMm));
  TEST_ASSERT_EQUAL_UINT16(30, s.get(TuneId::StepHeightMm));
  TEST_ASSERT_EQUAL_UINT16(20, s.get(TuneId::YawStrokeDeg));
  TEST_ASSERT_EQUAL_UINT16(127, s.get(TuneId::StanceRadiusMm));
  TEST_ASSERT_EQUAL_UINT16(60, s.get(TuneId::StepPeriodFastMs));
  TEST_ASSERT_EQUAL_UINT16(120, s.get(TuneId::StepPeriodSlowMs));
  TEST_ASSERT_EQUAL_UINT16(50, s.get(TuneId::HalfLiftPct));
  // The stride / step maxima are the single envelope shared with the config.
  TEST_ASSERT_EQUAL_UINT16(kMaxGaitStrideMm, kTuneLimits[1].max);
  TEST_ASSERT_EQUAL_UINT16(kMaxGaitStepMm, kTuneLimits[2].max);
  TEST_ASSERT_EQUAL_FLOAT(kMaxStrideMm, static_cast<float>(kMaxGaitStrideMm));
  TEST_ASSERT_EQUAL_FLOAT(kMaxStepMm, static_cast<float>(kMaxGaitStepMm));
  // Clamping is table-driven.
  s.set(TuneId::StanceRadiusMm, 5);
  TEST_ASSERT_EQUAL_UINT16(kTuneLimits[4].min, s.get(TuneId::StanceRadiusMm));
  s.set(TuneId::YawStrokeDeg, 500);
  TEST_ASSERT_EQUAL_UINT16(kTuneLimits[3].max, s.get(TuneId::YawStrokeDeg));
}

void test_shape_stance_radius_moves_home_foot_radially() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Stand);
  d.body_height_mm = 110;  // plenty of radial room
  ge.configure(d);
  GaitShape s;
  s.set(TuneId::StanceRadiusMm, 150);
  ge.setShape(s);
  GaitOutput out;
  settle(ge, out);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 150.0f, ge.stanceRadiusMm());
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 150.0f,
                             radiusFromMount(leg, out.feet[leg].x_mm,
                                             out.feet[leg].y_mm));
  }
  // Mid legs stay exactly on the body X axis.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out.feet[2].y_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out.feet[5].y_mm);
  // Narrowing works the same way and is filtered, not stepped.
  s.set(TuneId::StanceRadiusMm, 110);
  ge.setShape(s);
  ge.update(20, out);
  TEST_ASSERT_TRUE(ge.stanceRadiusMm() < 150.0f);
  TEST_ASSERT_TRUE(ge.stanceRadiusMm() > 140.0f);
  settle(ge, out);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 110.0f, ge.stanceRadiusMm());
}

void test_shape_stance_radius_is_capped_by_workspace_at_height() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Stand);
  d.body_height_mm = static_cast<uint16_t>(kRcBodyHeightMaxMm);
  ge.configure(d);
  GaitShape s;
  s.set(TuneId::StanceRadiusMm, kTuneLimits[4].max);  // 165: unreachable here
  ge.setShape(s);
  GaitOutput out;
  settle(ge, out);
  TEST_ASSERT_TRUE(ge.stanceRadiusMm() < static_cast<float>(kTuneLimits[4].max));
  // Whatever the cap chose, the home foot is inside the IK margin.
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    bool limited = false;
    const IkResult r = bk.solveBodyLimited(leg, out.feet[leg].x_mm,
                                           out.feet[leg].y_mm,
                                           out.feet[leg].z_mm, limited);
    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FALSE(limited);
  }
  // The default radius is NOT capped anywhere in the Pot2 envelope.
  for (int i = 0; i <= 10; ++i) {
    GaitEngine ge2;
    GaitDefaults d2 = defaultGait();
    d2.body_height_mm =
        static_cast<uint16_t>(rcBodyHeightMm(static_cast<float>(i) / 10.0f) + 0.5f);
    ge2.configure(d2);
    settle(ge2, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 126.75f, ge2.stanceRadiusMm());
  }
}

void test_shape_yaw_stroke_sets_body_yaw_per_cycle() {
  GaitEngine ge;
  GaitDefaults d = defaultGait();
  d.gait = static_cast<uint8_t>(GaitId::Tripod);
  d.speed_x255 = 255;
  ge.configure(d);
  // Default 20 deg per cycle (keyframes span -0.5..+0.5 of the stroke).
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, sweptYawDeg(ge));
  GaitShape s;
  s.set(TuneId::YawStrokeDeg, 40);
  ge.setShape(s);
  GaitOutput out;
  settle(ge, out);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 40.0f, sweptYawDeg(ge));
  s.set(TuneId::YawStrokeDeg, 0);
  ge.setShape(s);
  settle(ge, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, sweptYawDeg(ge));
}

void test_shape_cadence_endpoints_scale_cycle_time() {
  // Count update() ticks per full tripod cycle at speed 1 and speed 0.
  auto cycle_ms = [](uint16_t fast, uint16_t slow, uint8_t speed) {
    GaitEngine ge;
    GaitDefaults d = defaultGait();
    d.gait = static_cast<uint8_t>(GaitId::Tripod);
    d.speed_x255 = speed;
    ge.configure(d);
    GaitShape s;
    s.set(TuneId::StepPeriodFastMs, fast);
    s.set(TuneId::StepPeriodSlowMs, slow);
    ge.setShape(s);
    BodyTwist t;
    t.vy = 1.0f;
    ge.setTwist(t);
    GaitOutput out;
    settle(ge, out);
    ge.reset();
    float prev = ge.phase();
    int ms = 0;
    for (;;) {
      ge.update(1, out);
      ++ms;
      if (ge.phase() < prev) break;  // wrapped
      prev = ge.phase();
      if (ms > 20000) break;
    }
    return ms;
  };
  // Defaults: 8 steps x 60 ms = 480 ms at full speed, 8 x 120 = 960 ms at 0.
  TEST_ASSERT_INT_WITHIN(3, 480, cycle_ms(60, 120, 255));
  TEST_ASSERT_INT_WITHIN(3, 960, cycle_ms(60, 120, 0));
  TEST_ASSERT_INT_WITHIN(3, 640, cycle_ms(60, 120, 128));  // neutral 80 ms
  // Phoenix-like 35 ms steps: 280 ms tripod cycle.
  TEST_ASSERT_INT_WITHIN(3, 280, cycle_ms(35, 200, 255));
  // A crossed pair degrades to the fast value instead of inverting the knob.
  TEST_ASSERT_INT_WITHIN(3, 400, cycle_ms(50, 40, 0));
}

void test_shape_half_lift_changes_mid_swing_height_only() {
  auto mid_swing_lift = [](uint16_t half_pct) {
    GaitEngine ge;
    GaitDefaults d = defaultGait();
    d.gait = static_cast<uint8_t>(GaitId::Tripod);
    d.speed_x255 = 128;  // 80 ms steps
    ge.configure(d);
    GaitShape s;
    s.set(TuneId::HalfLiftPct, half_pct);
    ge.setShape(s);
    BodyTwist t;
    t.vy = 1.0f;
    ge.setTwist(t);
    GaitOutput out;
    settle(ge, out);
    ge.reset();
    // Leg 1 (origin 1) has its full-lift keyframe at step 1 and the half-lift
    // keyframe at step 2; sample the exact step-2 boundary (80 ms).
    float peak = -1000.0f;
    float at_half = 0.0f;
    for (int ms = 1; ms <= 160; ++ms) {
      ge.update(1, out);
      const float lift = out.feet[1].z_mm + static_cast<float>(d.body_height_mm);
      if (lift > peak) peak = lift;
      if (ms == 80) at_half = lift;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, peak);  // full step height intact
    return at_half;
  };
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 15.0f, mid_swing_lift(50));
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 22.5f, mid_swing_lift(75));
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 0.0f, mid_swing_lift(0));
}

void test_leg_geometry_from_config_matches_compiled_table() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  GaitEngine from_cfg;
  from_cfg.setLegGeometry(cfg);
  from_cfg.configure(cfg.gait);
  GaitEngine compiled;
  compiled.configure(cfg.gait);
  GaitOutput a, b;
  from_cfg.update(20, a);
  compiled.update(20, b);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, b.feet[leg].x_mm, a.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, b.feet[leg].y_mm, a.feet[leg].y_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][0], a.feet[leg].x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kHomeXy[leg][1], a.feet[leg].y_mm);
  }
}

// Golden vector shared with the handset (simulator/test_hexapod_gait_params.c):
// defaults 60,30,20,127,60,120,50 as little-endian uint16 ->
// 3C 00 1E 00 14 00 7F 00 3C 00 78 00 32 00, CRC-16/CCITT-FALSE = 0x1319.
void test_gait_shape_crc_vector() {
  GaitShape s;
  TEST_ASSERT_EQUAL_HEX16(0x1319, gaitShapeCrc16(s));
  s.set(TuneId::StrideMm, 61);
  TEST_ASSERT_NOT_EQUAL(0x1319, gaitShapeCrc16(s));
  s.set(TuneId::StrideMm, 60);
  TEST_ASSERT_EQUAL_HEX16(0x1319, gaitShapeCrc16(s));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stand_holds_home_stance);
  RUN_TEST(test_sit_lowers_body);
  RUN_TEST(test_tripod_groups_are_opposite_at_phase_zero);
  RUN_TEST(test_duty_factors);
  RUN_TEST(test_requested_duty_does_not_change_mark_iii_pattern);
  RUN_TEST(test_targets_bounded_over_full_cycle);
  RUN_TEST(test_swing_lift_reaches_step_height);
  RUN_TEST(test_crouched_walk_keeps_configured_swing_clearance);
  RUN_TEST(test_touchdown_velocity_is_continuous);
  RUN_TEST(test_forward_twist_moves_stance_foot_backward);
  RUN_TEST(test_centered_twist_holds_planted_home_stance);
  RUN_TEST(test_twist_is_consumed_without_a_second_response_filter);
  RUN_TEST(test_centering_shaped_command_parks_phase);
  RUN_TEST(test_rc_body_height_pot_neutral_at_center);
  RUN_TEST(test_rc_body_height_envelope_keeps_feet_reachable);
  RUN_TEST(test_rc_body_height_minimum_is_the_measured_crouch);
  RUN_TEST(test_low_command_keeps_full_swing_clearance);
  RUN_TEST(test_all_gait_targets_are_ik_reachable);
  RUN_TEST(test_phase_wraps_and_advances);
  // Live shape tunables (handset parameter uplink).
  RUN_TEST(test_shape_defaults_match_compiled_gait);
  RUN_TEST(test_shape_stance_radius_moves_home_foot_radially);
  RUN_TEST(test_shape_stance_radius_is_capped_by_workspace_at_height);
  RUN_TEST(test_shape_yaw_stroke_sets_body_yaw_per_cycle);
  RUN_TEST(test_shape_cadence_endpoints_scale_cycle_time);
  RUN_TEST(test_shape_half_lift_changes_mid_swing_height_only);
  RUN_TEST(test_leg_geometry_from_config_matches_compiled_table);
  RUN_TEST(test_gait_shape_crc_vector);
  return UNITY_END();
}
