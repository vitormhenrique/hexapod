// Native (host) verification of the firmware kinematic model against the
// measured CAD, the dimension source of truth (dimensions.md at the repo
// root). Every expected number below is a direct CAD measurement (converted
// from the CAD frame "x forward / y left" into the firmware body frame B
// "x right / y forward": x_B = -y_cad, y_B = x_cad).
//
// The core promise: with all 18 servos centered (tick 2048 == all joint
// angles zero) the model must place every joint axis and every foot tip at
// the measured CAD coordinates -- and, inversely, asking the IK for the
// measured tip must return exactly zero joint angles / centered ticks.
//
// Run with: pio test -e native -f test_cad_geometry

#include <math.h>
#include <unity.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/body_ik.h"
#include "../../src/gait/gait_engine.h"
#include "../../src/gait/gait_pipeline.h"
#include "../../src/gait/leg_ik.h"

using namespace gait;
using namespace config;

namespace {

struct Vec3 {
  float x, y, z;
};

// Measured CAD data, body frame B (x right, y forward, z up), origin at the
// body centre (middle of the top and bottom plates). All mm.
//
// Firmware leg index order: 0 rear-left, 1 rear-right, 2 mid-right,
// 3 front-right, 4 front-left, 5 mid-left.

// Coxa rotation centres ("center of coxa rotation"): corners at
// (+-65.577, +-115.577, 0), mids at (+-69.78, 0, 0).
constexpr Vec3 kCadCoxaAxis[kNumLegs] = {
    {-65.577f, -115.577f, 0.0f}, {65.577f, -115.577f, 0.0f},
    {69.78f, 0.0f, 0.0f},        {65.577f, 115.577f, 0.0f},
    {-65.577f, 115.577f, 0.0f},  {-69.78f, 0.0f, 0.0f},
};

// Femur rotation centres: 52.00 mm radially out from the coxa axis, same
// height (52/sqrt(2) = 36.770 on each axis for the corner legs).
constexpr Vec3 kCadFemurAxis[kNumLegs] = {
    {-102.347f, -152.347f, 0.0f}, {102.347f, -152.347f, 0.0f},
    {121.78f, 0.0f, 0.0f},        {102.347f, 152.347f, 0.0f},
    {-102.347f, 152.347f, 0.0f},  {-121.78f, 0.0f, 0.0f},
};

// Tibia (knee) rotation centres: 64.80 mm further out radially, 15.00 mm
// down (64.80/sqrt(2) = 45.820 per axis on the corner legs).
constexpr Vec3 kCadTibiaAxis[kNumLegs] = {
    {-148.167f, -198.167f, -15.0f}, {148.167f, -198.167f, -15.0f},
    {186.58f, 0.0f, -15.0f},        {148.167f, 198.167f, -15.0f},
    {-148.167f, 198.167f, -15.0f},  {-186.58f, 0.0f, -15.0f},
};

// Foot tips with all servos centered.
constexpr Vec3 kCadFootTip[kNumLegs] = {
    {-155.205f, -205.205f, -131.734f}, {155.205f, -205.205f, -131.734f},
    {196.534f, 0.0f, -131.734f},       {155.205f, 205.205f, -131.734f},
    {-155.205f, 205.205f, -131.734f},  {-196.534f, 0.0f, -131.734f},
};

// Measured straight-line distances from the body centre to the foot tips.
constexpr float kCadMidTipDistanceMm = 236.60f;
constexpr float kCadCornerTipDistanceMm = 289.053f;

constexpr float kPi = 3.14159265358979323846f;

// Convert a coxa-frame point back into body frame B using the persisted leg
// geometry (inverse of BodyKinematics::footBodyToCoxa).
void coxaToBody(const RobotConfig& cfg, uint8_t leg, float cx, float cy,
                float cz, float& bx, float& by, float& bz) {
  const LegGeometry& g = cfg.legs[leg];
  const float yaw = g.mount_yaw_cdeg * (kPi / 18000.0f);
  const float a = -(yaw + kPi / 2.0f);
  const float cos_a = cosf(a);
  const float sin_a = sinf(a);
  // footBodyToCoxa applies R(a); invert with R(-a) = R(a)^T.
  bx = g.mount_x_dmm / 10.0f + cos_a * cx + sin_a * cy;
  by = g.mount_y_dmm / 10.0f - sin_a * cx + cos_a * cy;
  bz = cz + g.mount_z_dmm / 10.0f + cfg.geometry.coxa_lift_cmm / 100.0f;
}

LegIk makeLegIk(const RobotConfig& cfg) {
  return LegIk(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
               cfg.links.tibia_cmm / 100.0f,
               cfg.geometry.home_radius_cmm / 100.0f,
               cfg.geometry.home_foot_z_cmm / 100.0f);
}

}  // namespace

// The persisted default mounts are the measured coxa rotation centres.
void test_config_mounts_match_cad_coxa_axes() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadCoxaAxis[leg].x,
                             cfg.legs[leg].mount_x_dmm / 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadCoxaAxis[leg].y,
                             cfg.legs[leg].mount_y_dmm / 10.0f);
    // Coxa axis on the body mid-plane: mount z + coxa lift == 0.
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadCoxaAxis[leg].z,
                             cfg.legs[leg].mount_z_dmm / 10.0f +
                                 cfg.geometry.coxa_lift_cmm / 100.0f);
  }
}

// L1 places the femur axis at the measured femur rotation centre (checks the
// 52.00 mm radial coxa link and the per-leg home azimuth).
void test_centered_femur_axes_match_cad() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  const float l1 = cfg.links.coxa_cmm / 100.0f;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    float bx, by, bz;
    coxaToBody(cfg, leg, l1, 0.0f, 0.0f, bx, by, bz);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadFemurAxis[leg].x, bx);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadFemurAxis[leg].y, by);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, kCadFemurAxis[leg].z, bz);
  }
}

// The model's rest decomposition must put the knee at the measured tibia
// rotation centre. This is a REAL constraint, not true by construction: the
// femur rest angle falls out of the law of cosines from (L2, L3, home foot),
// so a wrong tibia length or a wrong knee branch moves the computed knee far
// from the CAD point (the old 24.86 mm tibia missed it by ~100 mm).
void test_centered_knee_axes_match_cad() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  LegIk ik = makeLegIk(cfg);
  const float l1 = cfg.links.coxa_cmm / 100.0f;
  const float l2 = cfg.links.femur_cmm / 100.0f;
  const float knee_r = l1 + l2 * cosf(ik.femurRest());
  const float knee_z = l2 * sinf(ik.femurRest());
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    float bx, by, bz;
    coxaToBody(cfg, leg, knee_r, 0.0f, knee_z, bx, by, bz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadTibiaAxis[leg].x, bx);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadTibiaAxis[leg].y, by);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadTibiaAxis[leg].z, bz);
  }
}

// Centered servos (all joint angles zero) put every foot tip at the measured
// CAD coordinates.
void test_centered_foot_tips_match_cad() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  LegIk ik = makeLegIk(cfg);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    float cx, cy, cz;
    ik.forwardRaw(0.0f, ik.femurRest(), ik.tibiaRest(), cx, cy, cz);
    float bx, by, bz;
    coxaToBody(cfg, leg, cx, cy, cz, bx, by, bz);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadFootTip[leg].x, bx);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadFootTip[leg].y, by);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, kCadFootTip[leg].z, bz);

    // And the measured body-centre-to-tip distances hold.
    const float dist = sqrtf(bx * bx + by * by + bz * bz);
    const bool mid = (leg == 2 || leg == 5);
    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, mid ? kCadMidTipDistanceMm : kCadCornerTipDistanceMm, dist);
  }
}

// Inverse direction: asking the IK for the measured tip returns zero angles,
// and the servo map emits centered ticks (2048) for all 18 joints.
void test_cad_foot_tips_solve_to_centered_servos() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  BodyKinematics bk(cfg);
  dxl::ServoMap map(cfg);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    const IkResult r = bk.solveBody(leg, kCadFootTip[leg].x, kCadFootTip[leg].y,
                                    kCadFootTip[leg].z);
    TEST_ASSERT_TRUE(r.reachable);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.coxa);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.femur);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.tibia);
    const float angles[kJointsPerLeg] = {r.coxa, r.femur, r.tibia};
    for (uint8_t j = 0; j < kJointsPerLeg; ++j) {
      const dxl::JointCommand jc = map.angleToTick(leg, j, angles[j]);
      TEST_ASSERT_UINT16_WITHIN(1, kServoCenterTick, jc.tick);
    }
  }
}

// The measured knee rest angle is on the NEGATIVE (knee-out) branch: -72.1
// degrees from the CAD joint centres. Guards the acos branch selection.
void test_knee_rest_angle_matches_cad_branch() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  LegIk ik = makeLegIk(cfg);
  constexpr float kDeg = 180.0f / kPi;
  TEST_ASSERT_FLOAT_WITHIN(0.3f, -72.1f, ik.tibiaRest() * kDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, -13.0f, ik.femurRest() * kDeg);
}

// The gait engine's hard-coded home stance must agree with the config-derived
// stance (mount + home_radius along the home azimuth) so the two sources of
// truth can never drift apart again.
void test_gait_engine_home_matches_config_geometry() {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  GaitEngine ge;
  ge.configure(cfg.gait);
  const float r = cfg.geometry.home_radius_cmm / 100.0f;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    const float yaw = cfg.legs[leg].mount_yaw_cdeg * (kPi / 18000.0f);
    const float azimuth = yaw + kPi / 2.0f;  // coxa +X direction in B
    const float expected_x = cfg.legs[leg].mount_x_dmm / 10.0f +
                             r * cosf(azimuth);
    const float expected_y = cfg.legs[leg].mount_y_dmm / 10.0f +
                             r * sinf(azimuth);
    float x, y, z;
    ge.nominalFoot(leg, x, y, z);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_x, x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_y, y);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -static_cast<float>(cfg.gait.body_height_mm),
                             z);
  }
}

// With the measured geometry the default stride at the default stance must
// use the full commanded stroke: the reach limiter stays disengaged for full
// forward, full strafe, and full yaw (the old model collapsed a 60 mm stride
// to ~10 mm here).
void test_default_stride_is_not_reach_limited() {
  const float commands[][3] = {
      {0.0f, 1.0f, 0.0f},  // full forward
      {1.0f, 0.0f, 0.0f},  // full strafe
      {0.0f, 0.0f, 1.0f},  // full yaw
      {0.0f, 0.7f, 0.7f},  // combined
  };
  for (const float* cmd : commands) {
    RobotConfig cfg;
    defaultRobotConfig(cfg);
    GaitPipeline pipe(cfg);
    pipe.setGait(GaitId::Tripod);
    pipe.setTwist(cmd[0], cmd[1], cmd[2]);
    for (int i = 0; i < 200; ++i) {  // several full cycles
      PipelineOutput out;
      pipe.update(10, out);
      TEST_ASSERT_FALSE(out.any_unreachable);
      TEST_ASSERT_FALSE(out.any_reach_limited);
    }
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_config_mounts_match_cad_coxa_axes);
  RUN_TEST(test_centered_femur_axes_match_cad);
  RUN_TEST(test_centered_knee_axes_match_cad);
  RUN_TEST(test_centered_foot_tips_match_cad);
  RUN_TEST(test_cad_foot_tips_solve_to_centered_servos);
  RUN_TEST(test_knee_rest_angle_matches_cad_branch);
  RUN_TEST(test_gait_engine_home_matches_config_geometry);
  RUN_TEST(test_default_stride_is_not_reach_limited);
  return UNITY_END();
}
