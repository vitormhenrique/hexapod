// Native regression test: a commanded straight walk must BE straight.
//
// Drives the full host-motion path (arbiter -> shaper -> ControllerCore ->
// gait -> IK -> servo map -> quantized ticks), reconstructs foot positions
// from the emitted ticks via forward kinematics, and rigid-fits the implied
// body odometry over many gait cycles. Any left/right asymmetry anywhere in
// the chain (spatial or timing) integrates into lateral drift or yaw here.
//
// Bounds (32 s of walking, ~3.6 m of travel):
//   |lateral|  < 2 mm       (measured: ~0.6 mm from tick quantization)
//   |net yaw|  < 0.05 deg   (measured: ~0.000 deg)
// Run with: pio test -e native -f test_gait_straightness
#include <math.h>
#include <unity.h>

#include "../support/controller_sim_adapter.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/leg_ik.h"

using namespace controller;
using namespace config;

namespace {

struct Foot {
  float x, y, z;
  bool valid;
};

Foot goalsFootBody(const RobotConfig& cfg, const dxl::ServoMap& map,
                   const gait::LegIk& ik, const RobotCommand& cmd,
                   uint8_t leg) {
  float angles[kJointsPerLeg] = {};
  bool found[kJointsPerLeg] = {};
  for (uint8_t index = 0; index < cmd.goals.count; ++index) {
    const auto& goal = cmd.goals.joints[index];
    if (goal.leg == leg && goal.joint < kJointsPerLeg) {
      angles[goal.joint] = map.tickToAngle(leg, goal.joint, goal.tick);
      found[goal.joint] = true;
    }
  }
  Foot f{0, 0, 0, false};
  if (!(found[0] && found[1] && found[2])) return f;
  float cx, cy, cz;
  ik.forwardRaw(angles[0], angles[1] + ik.femurRest(),
                angles[2] + ik.tibiaRest(), cx, cy, cz);
  constexpr float kPi = 3.14159265358979323846f;
  const LegGeometry& g = cfg.legs[leg];
  const float yaw = g.mount_yaw_cdeg * (kPi / 18000.0f);
  const float a = -(yaw + kPi / 2.0f);
  const float ca = cosf(a), sa = sinf(a);
  f.x = g.mount_x_dmm / 10.0f + ca * cx + sa * cy;
  f.y = g.mount_y_dmm / 10.0f - sa * cx + ca * cy;
  f.z = cz + g.mount_z_dmm / 10.0f + cfg.geometry.coxa_lift_cmm / 100.0f;
  f.valid = true;
  return f;
}

void runStraightWalk(uint8_t gait_wire_id, double& lateral_mm,
                     double& forward_mm, double& yaw_deg) {
  sim::ControllerSimAdapter adapter;
  adapter.configureDefault();
  adapter.setReadyDxl();
  adapter.state().dxl.torque_off = false;
  adapter.setTime(1000, 10);

  ControllerIntent& intent = adapter.intent();
  intent.rc.ever_seen = false;
  intent.rc.armed = false;
  intent.rc.kill = false;
  intent.rc.failsafe = true;
  intent.maintenance.lock_held = true;
  intent.maintenance.lock_token = 7;
  intent.maintenance.control_mode = protocol::MaintControlMode::GaitPipeline;

  protocol::MotionIntent& motion = intent.motion;
  motion.gait = gait_wire_id;
  motion.body_height_mm = 132;
  motion.stride_len_mm = 60;
  motion.step_height_mm = 30;
  motion.duty_x255 = 159;
  motion.speed_x255 = 128;
  motion.seq = 1;
  for (int i = 0; i < 800; ++i) adapter.advance(10);
  motion.twist_vx = 0.6f;  // command-frame FORWARD
  motion.seq = 2;
  for (int i = 0; i < 400; ++i) adapter.advance(10);

  const RobotConfig& cfg = adapter.config().robot;
  const dxl::ServoMap map(cfg);
  const gait::LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
                       cfg.links.tibia_cmm / 100.0f,
                       cfg.geometry.home_radius_cmm / 100.0f,
                       cfg.geometry.home_foot_z_cmm / 100.0f);

  Foot prev[kNumLegs];
  for (uint8_t leg = 0; leg < kNumLegs; ++leg)
    prev[leg] = goalsFootBody(cfg, map, ik, adapter.command(), leg);
  float zmin = 0;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) zmin = fminf(zmin, prev[leg].z);
  const float stance_thresh = zmin + 1.0f;

  double bx = 0, by = 0, byaw = 0;
  for (int i = 0; i < 3200; ++i) {
    const RobotCommand& cmd = adapter.advance(10);
    TEST_ASSERT_TRUE(cmd.goal_valid);
    double px[kNumLegs], py[kNumLegs], dx[kNumLegs], dy[kNumLegs];
    int n = 0;
    Foot cur[kNumLegs];
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      cur[leg] = goalsFootBody(cfg, map, ik, cmd, leg);
      if (!cur[leg].valid || !prev[leg].valid) continue;
      if (cur[leg].z > stance_thresh || prev[leg].z > stance_thresh) continue;
      px[n] = 0.5 * (cur[leg].x + prev[leg].x);
      py[n] = 0.5 * (cur[leg].y + prev[leg].y);
      dx[n] = cur[leg].x - prev[leg].x;
      dy[n] = cur[leg].y - prev[leg].y;
      ++n;
    }
    if (n >= 3) {
      double pbx = 0, pby = 0, dbx = 0, dby = 0;
      for (int k = 0; k < n; ++k) {
        pbx += px[k]; pby += py[k]; dbx += dx[k]; dby += dy[k];
      }
      pbx /= n; pby /= n; dbx /= n; dby /= n;
      double num = 0, den = 0;
      for (int k = 0; k < n; ++k) {
        const double cxx = px[k] - pbx, cyy = py[k] - pby;
        const double ex = dx[k] - dbx, ey = dy[k] - dby;
        num += ex * cyy - ey * cxx;
        den += cxx * cxx + cyy * cyy;
      }
      const double dth = den > 0 ? num / den : 0.0;
      const double dcx = dth * pby - dbx;
      const double dcy = -dth * pbx - dby;
      const double c = cos(byaw), s = sin(byaw);
      bx += c * dcx - s * dcy;
      by += s * dcx + c * dcy;
      byaw += dth;
    }
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) prev[leg] = cur[leg];
  }
  lateral_mm = bx;
  forward_mm = by;
  yaw_deg = byaw * 57.29577951308232;
}

}  // namespace

void test_tripod_forward_walk_is_straight() {
  double lat, fwd, yaw;
  runStraightWalk(2, lat, fwd, yaw);  // tripod
  TEST_ASSERT_TRUE_MESSAGE(fwd > 3000.0, "expected >3 m of forward travel");
  TEST_ASSERT_TRUE_MESSAGE(fabs(lat) < 2.0,
                           "commanded walk drifts laterally >2 mm per ~3.6 m");
  TEST_ASSERT_TRUE_MESSAGE(fabs(yaw) < 0.05,
                           "commanded walk accumulates yaw >0.05 deg per ~3.6 m");
}

void test_wave_forward_walk_is_straight() {
  double lat, fwd, yaw;
  runStraightWalk(4, lat, fwd, yaw);  // wave
  TEST_ASSERT_TRUE_MESSAGE(fwd > 500.0, "expected forward travel");
  TEST_ASSERT_TRUE_MESSAGE(fabs(lat) < 2.0,
                           "wave walk drifts laterally >2 mm");
  TEST_ASSERT_TRUE_MESSAGE(fabs(yaw) < 0.05, "wave walk accumulates yaw");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_tripod_forward_walk_is_straight);
  RUN_TEST(test_wave_forward_walk_is_straight);
  return UNITY_END();
}
