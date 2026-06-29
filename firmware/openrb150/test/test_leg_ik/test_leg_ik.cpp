// Native (host) unit tests for 3-DOF leg IK and the body pose transform.
// No Arduino deps.
//
// Run with: pio test -e native

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/gait/body_ik.h"
#include "../../src/gait/leg_ik.h"

using namespace gait;
using namespace config;

namespace {

// Default HexNav link lengths (mm), IK ref section 13.
constexpr float kL1 = 56.08f;
constexpr float kL2 = 66.51f;
constexpr float kL3 = 24.86f;

// Home foot positions in body-centered frame B (mm), IK ref section 13.
struct Vec3 {
  float x, y, z;
};
constexpr Vec3 kHomeFootB[kNumLegs] = {
    {-155.4f, -205.4f, -40.0f}, {155.4f, -205.4f, -40.0f},
    {196.8f, 0.0f, -40.0f},     {155.4f, 205.4f, -40.0f},
    {-155.4f, 205.4f, -40.0f},  {-196.8f, 0.0f, -40.0f},
};

}  // namespace

// ---- LegIk ---------------------------------------------------------------

void test_home_foot_maps_to_zero_angles() {
  LegIk ik(kL1, kL2, kL3);
  IkResult r = ik.solve(kHomeRadiusMm, 0.0f, kHomeFootZMm);
  TEST_ASSERT_TRUE(r.reachable);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r.coxa);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r.femur);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r.tibia);
}

void test_coxa_yaw_from_lateral_offset() {
  LegIk ik(kL1, kL2, kL3);
  // Foot pushed to +Y should yield a positive coxa yaw; -Y negative.
  IkResult left = ik.solve(120.0f, 40.0f, -44.55f);
  IkResult right = ik.solve(120.0f, -40.0f, -44.55f);
  TEST_ASSERT_TRUE(left.coxa > 0.05f);
  TEST_ASSERT_TRUE(right.coxa < -0.05f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, atan2f(40.0f, 120.0f), left.coxa);
}

void test_fk_round_trip_nominal() {
  LegIk ik(kL1, kL2, kL3);
  const float tx = 120.0f, ty = 10.0f, tz = -50.0f;
  IkResult r = ik.solve(tx, ty, tz);
  TEST_ASSERT_TRUE(r.reachable);

  // Reconstruct raw angles and run FK; must reproduce the coxa-frame target.
  float fx, fy, fz;
  ik.forwardRaw(r.coxa, r.femur + ik.femurRest(), r.tibia + ik.tibiaRest(), fx,
                fy, fz);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, tx, fx);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, ty, fy);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, tz, fz);
}

void test_edge_full_extension_is_reachable_boundary() {
  LegIk ik(kL1, kL2, kL3);
  // d exactly L2+L3 along the radial axis: planar_r = L2+L3, z = 0.
  const float x = kL1 + (kL2 + kL3);  // 147.45
  IkResult r = ik.solve(x, 0.0f, 0.0f);
  TEST_ASSERT_TRUE(r.reachable);
  // Knee straight: raw tibia ~ 0 -> relative tibia ~ -tibiaRest.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -ik.tibiaRest(), r.tibia);
}

void test_edge_too_far_is_unreachable() {
  LegIk ik(kL1, kL2, kL3);
  IkResult r = ik.solve(kL1 + (kL2 + kL3) + 12.0f, 0.0f, 0.0f);
  TEST_ASSERT_FALSE(r.reachable);
}

void test_edge_too_close_is_unreachable() {
  LegIk ik(kL1, kL2, kL3);
  // d below |L2-L3| -> folded, unreachable.
  const float x = kL1 + 30.0f;  // planar_r = 30 < 41.65
  IkResult r = ik.solve(x, 0.0f, 0.0f);
  TEST_ASSERT_FALSE(r.reachable);
}

void test_unreachable_still_returns_clamped_angles() {
  LegIk ik(kL1, kL2, kL3);
  IkResult r = ik.solve(400.0f, 0.0f, 0.0f);  // way out
  TEST_ASSERT_FALSE(r.reachable);
  // acos clamps cos_k to 1 -> raw tibia 0 -> relative tibia = -tibiaRest.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -ik.tibiaRest(), r.tibia);
  // No NaNs leaked.
  TEST_ASSERT_FALSE(isnan(r.femur));
  TEST_ASSERT_FALSE(isnan(r.tibia));
}

// lmt.11: the home-pose ctor params seed the rest offset. A LegIk built with a
// lowered home foot maps that lowered foot (not the default home) to zero, and
// the default home foot then reads non-zero -- proving the configured stance,
// not a compiled constant, drives the zero reference.
void test_leg_ik_home_params_shift_rest_offset() {
  LegIk def(kL1, kL2, kL3);  // default home (kHomeRadiusMm, kHomeFootZMm)
  LegIk low(kL1, kL2, kL3, kHomeRadiusMm, kHomeFootZMm - 10.0f);  // home 10 mm down

  // Default maps the documented home foot to ~zero.
  IkResult d = def.solve(kHomeRadiusMm, 0.0f, kHomeFootZMm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, d.femur);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, d.tibia);

  // The lowered-home solver reads the SAME foot as non-zero (its zero moved).
  IkResult l = low.solve(kHomeRadiusMm, 0.0f, kHomeFootZMm);
  TEST_ASSERT_TRUE(fabsf(l.femur) > 1e-3f || fabsf(l.tibia) > 1e-3f);

  // ...and maps ITS configured home (10 mm lower) to ~zero.
  IkResult l0 = low.solve(kHomeRadiusMm, 0.0f, kHomeFootZMm - 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, l0.femur);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, l0.tibia);
}

// ---- clampToReach (lmt.14 reachability-aware stride limiting) -------------

// The documented home stance (~92% of full reach) is already inside the 95%
// margin, so it must pass through untouched.
void test_clamp_reach_leaves_home_unchanged() {
  LegIk ik(kL1, kL2, kL3);
  float x = kHomeRadiusMm, y = 0.0f, z = kHomeFootZMm;
  const bool clamped = ik.clampToReach(x, y, z);
  TEST_ASSERT_FALSE(clamped);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kHomeRadiusMm, x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kHomeFootZMm, z);
}

// An over-reaching radial target is pulled in to exactly the reach margin, with
// the foot height preserved and the result solidly reachable.
void test_clamp_reach_pulls_overreach_to_margin_keeping_height() {
  LegIk ik(kL1, kL2, kL3);
  float x = 200.0f, y = 0.0f, z = kHomeFootZMm;  // far past full reach
  const bool clamped = ik.clampToReach(x, y, z);
  TEST_ASSERT_TRUE(clamped);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kHomeFootZMm, z);  // height kept
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, y);          // stays on radial

  // Planar reach distance now equals the margin (95% of l2+l3).
  const float d_max = kReachMarginFrac * (kL2 + kL3);
  const float planar_r = sqrtf(x * x + y * y) - kL1;
  const float d = sqrtf(planar_r * planar_r + z * z);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, d_max, d);

  // And the clamped target is comfortably reachable.
  TEST_ASSERT_TRUE(ik.solve(x, y, z).reachable);
}

// A diagonal over-reach keeps its hip-yaw direction (x and y scale together).
void test_clamp_reach_preserves_hip_yaw() {
  LegIk ik(kL1, kL2, kL3);
  const float yaw_in = atan2f(90.0f, 130.0f);
  float x = 130.0f, y = 90.0f, z = kHomeFootZMm;  // |xy| ~ 158 mm, over-reach
  const bool clamped = ik.clampToReach(x, y, z);
  TEST_ASSERT_TRUE(clamped);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, yaw_in, atan2f(y, x));  // direction kept
  TEST_ASSERT_TRUE(ik.solve(x, y, z).reachable);
}

// ---- BodyKinematics ------------------------------------------------------

void test_body_to_coxa_maps_home_to_radial() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    float cx, cy, cz;
    bk.footBodyToCoxa(leg, kHomeFootB[leg].x, kHomeFootB[leg].y,
                      kHomeFootB[leg].z, cx, cy, cz);
    // Every leg's home foot lands on the coxa +X radial axis at ~127 mm.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, kHomeRadiusMm, cx);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, cy);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, kHomeFootZMm, cz);
  }
}

void test_solve_body_home_is_near_zero_all_legs() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    IkResult r = bk.solveBody(leg, kHomeFootB[leg].x, kHomeFootB[leg].y,
                              kHomeFootB[leg].z);
    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r.coxa);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r.femur);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r.tibia);
  }
}

void test_apply_body_pose_identity() {
  BodyPose pose;  // all zero
  float bx, by, bz;
  BodyKinematics::applyBodyPose(pose, 12.0f, -34.0f, 56.0f, bx, by, bz);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 12.0f, bx);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -34.0f, by);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 56.0f, bz);
}

void test_apply_body_pose_pure_yaw() {
  BodyPose pose;
  pose.yaw = static_cast<float>(M_PI) / 2.0f;  // +90 deg
  float bx, by, bz;
  // World point (100,0,0) in the body-yawed frame: R^T * v = Rz(-90)*(100,0,0).
  BodyKinematics::applyBodyPose(pose, 100.0f, 0.0f, 0.0f, bx, by, bz);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, bx);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -100.0f, by);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, bz);
}

void test_apply_body_pose_translation() {
  BodyPose pose;
  pose.z_mm = 10.0f;
  float bx, by, bz;
  BodyKinematics::applyBodyPose(pose, 0.0f, 0.0f, -40.0f, bx, by, bz);
  // Body raised 10 mm -> foot sits 10 mm lower in the body frame.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -50.0f, bz);
}

void test_body_pose_raise_lowers_foot_in_coxa_frame() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  const uint8_t leg = 2;  // mid-right
  BodyPose neutral;
  BodyPose raised;
  raised.z_mm = 6.0f;  // lift body 6 mm (stays within reach)

  IkResult n = bk.solveBodyPose(leg, neutral, kHomeFootB[leg].x,
                                kHomeFootB[leg].y, kHomeFootB[leg].z);
  IkResult u = bk.solveBodyPose(leg, raised, kHomeFootB[leg].x,
                                kHomeFootB[leg].y, kHomeFootB[leg].z);
  TEST_ASSERT_TRUE(n.reachable);
  TEST_ASSERT_TRUE(u.reachable);
  // Raising the body must change the leg pose (foot effectively lower).
  TEST_ASSERT_TRUE(fabsf(u.tibia - n.tibia) > 0.01f);
}

void test_solve_body_invalid_leg() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  float cx = 9, cy = 9, cz = 9;
  bk.footBodyToCoxa(99, 1, 2, 3, cx, cy, cz);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cz);
}

// lmt.11: BodyKinematics reads the stance/coxa geometry from the persisted
// config, not compiled constants. Raising the configured coxa lift drops the
// home foot's coxa-frame Z by the same amount; the radial X/Y are unaffected.
void test_body_geometry_from_config_coxa_lift() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics base(cfg);

  const uint8_t leg = 2;  // mid-right
  float x0, y0, z0;
  base.footBodyToCoxa(leg, kHomeFootB[leg].x, kHomeFootB[leg].y,
                      kHomeFootB[leg].z, x0, y0, z0);

  cfg.geometry.coxa_lift_cmm += 1000;  // +10.00 mm
  BodyKinematics lifted(cfg);
  float x1, y1, z1;
  lifted.footBodyToCoxa(leg, kHomeFootB[leg].x, kHomeFootB[leg].y,
                        kHomeFootB[leg].z, x1, y1, z1);

  TEST_ASSERT_FLOAT_WITHIN(0.05f, z0 - 10.0f, z1);  // foot 10 mm lower in coxa Z
  TEST_ASSERT_FLOAT_WITHIN(0.05f, x0, x1);          // radial unchanged
  TEST_ASSERT_FLOAT_WITHIN(0.05f, y0, y1);
}

// lmt.14: solveBodyLimited applies the reach clamp before IK. A body-frame foot
// that over-reaches in the coxa frame is reported reach_limited AND solved
// reachable; the documented home stance is neither limited nor unreachable.
void test_solve_body_limited_flags_and_recovers_overreach() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);

  const uint8_t leg = 2;  // mid-right, home foot purely +X in body frame
  bool limited = true;
  IkResult home = bk.solveBodyLimited(leg, kHomeFootB[leg].x, kHomeFootB[leg].y,
                                      kHomeFootB[leg].z, limited);
  TEST_ASSERT_FALSE(limited);       // home stance is inside the margin
  TEST_ASSERT_TRUE(home.reachable);

  // Push the foot 70 mm further out along +X (past the reach boundary).
  bool limited2 = false;
  IkResult far = bk.solveBodyLimited(leg, kHomeFootB[leg].x + 70.0f,
                                     kHomeFootB[leg].y, kHomeFootB[leg].z,
                                     limited2);
  TEST_ASSERT_TRUE(limited2);       // clamp engaged
  TEST_ASSERT_TRUE(far.reachable);  // and the result is back inside the annulus
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_home_foot_maps_to_zero_angles);
  RUN_TEST(test_coxa_yaw_from_lateral_offset);
  RUN_TEST(test_fk_round_trip_nominal);
  RUN_TEST(test_edge_full_extension_is_reachable_boundary);
  RUN_TEST(test_edge_too_far_is_unreachable);
  RUN_TEST(test_edge_too_close_is_unreachable);
  RUN_TEST(test_unreachable_still_returns_clamped_angles);
  RUN_TEST(test_leg_ik_home_params_shift_rest_offset);
  RUN_TEST(test_clamp_reach_leaves_home_unchanged);
  RUN_TEST(test_clamp_reach_pulls_overreach_to_margin_keeping_height);
  RUN_TEST(test_clamp_reach_preserves_hip_yaw);
  RUN_TEST(test_body_to_coxa_maps_home_to_radial);
  RUN_TEST(test_solve_body_home_is_near_zero_all_legs);
  RUN_TEST(test_apply_body_pose_identity);
  RUN_TEST(test_apply_body_pose_pure_yaw);
  RUN_TEST(test_apply_body_pose_translation);
  RUN_TEST(test_body_pose_raise_lowers_foot_in_coxa_frame);
  RUN_TEST(test_solve_body_invalid_leg);
  RUN_TEST(test_body_geometry_from_config_coxa_lift);
  RUN_TEST(test_solve_body_limited_flags_and_recovers_overreach);
  return UNITY_END();
}
