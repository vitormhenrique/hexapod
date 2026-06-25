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

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_home_foot_maps_to_zero_angles);
  RUN_TEST(test_coxa_yaw_from_lateral_offset);
  RUN_TEST(test_fk_round_trip_nominal);
  RUN_TEST(test_edge_full_extension_is_reachable_boundary);
  RUN_TEST(test_edge_too_far_is_unreachable);
  RUN_TEST(test_edge_too_close_is_unreachable);
  RUN_TEST(test_unreachable_still_returns_clamped_angles);
  RUN_TEST(test_body_to_coxa_maps_home_to_radial);
  RUN_TEST(test_solve_body_home_is_near_zero_all_legs);
  RUN_TEST(test_apply_body_pose_identity);
  RUN_TEST(test_apply_body_pose_pure_yaw);
  RUN_TEST(test_apply_body_pose_translation);
  RUN_TEST(test_body_pose_raise_lowers_foot_in_coxa_frame);
  RUN_TEST(test_solve_body_invalid_leg);
  return UNITY_END();
}
