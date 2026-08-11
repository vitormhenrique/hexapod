// Host probe: full ControllerCore path (RC intent -> arbiter -> shaper ->
// trick engine -> pipeline) at various body-height fractions. Diagnoses the
// "drags feet at 65 mm crouch" report end-to-end.
#include <math.h>
#include <stdio.h>

#include "controller_sim_adapter.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/leg_ik.h"

using namespace controller;
using namespace config;

static bool goalsFootBody(const RobotConfig& cfg, const RobotCommand& cmd,
                          uint8_t leg, float& body_x, float& body_y,
                          float& body_z) {
  dxl::ServoMap map(cfg);
  float angles[kJointsPerLeg] = {};
  bool found[kJointsPerLeg] = {};
  for (uint8_t index = 0; index < cmd.goals.count; ++index) {
    const auto& goal = cmd.goals.joints[index];
    // Identify leg/joint via the servo config for this DXL id.
    for (uint8_t k = 0; k < kNumServos; ++k) {
      if (cfg.servos[k].id == goal.id) {
        if (cfg.servos[k].leg != leg) break;
        const uint8_t joint = cfg.servos[k].joint;
        angles[joint] = map.tickToAngle(leg, joint, goal.tick);
        found[joint] = true;
        break;
      }
    }
  }
  for (uint8_t joint = 0; joint < kJointsPerLeg; ++joint) {
    if (!found[joint]) return false;
  }
  gait::LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
                 cfg.links.tibia_cmm / 100.0f,
                 cfg.geometry.home_radius_cmm / 100.0f,
                 cfg.geometry.home_foot_z_cmm / 100.0f);
  float coxa_x, coxa_y, coxa_z;
  ik.forwardRaw(angles[0], angles[1] + ik.femurRest(),
                angles[2] + ik.tibiaRest(), coxa_x, coxa_y, coxa_z);
  constexpr float kPi = 3.14159265358979323846f;
  const LegGeometry& geometry = cfg.legs[leg];
  const float yaw = geometry.mount_yaw_cdeg * (kPi / 18000.0f);
  const float a = -(yaw + kPi / 2.0f);
  const float cos_a = cosf(a);
  const float sin_a = sinf(a);
  body_x = geometry.mount_x_dmm / 10.0f + cos_a * coxa_x + sin_a * coxa_y;
  body_y = geometry.mount_y_dmm / 10.0f - sin_a * coxa_x + cos_a * coxa_y;
  body_z = coxa_z + geometry.mount_z_dmm / 10.0f +
           cfg.geometry.coxa_lift_cmm / 100.0f;
  return true;
}

static void walkAt(float height_fraction, float trim_pitch_rad) {
  sim::ControllerSimAdapter adapter;
  adapter.configureDefault();
  adapter.setReadyDxl();
  adapter.state().dxl.torque_off = false;
  adapter.setTime(1000, 10);

  ControllerIntent& intent = adapter.intent();
  ControllerCommand& rc = intent.rc.command;
  intent.rc.ever_seen = true;
  intent.rc.armed = false;
  intent.rc.kill = false;
  intent.rc.failsafe = false;
  rc.valid = true;
  rc.failsafe = false;
  rc.ever_seen = true;
  rc.estop = false;
  rc.arm_request = true;
  rc.mode = ControlMode::Yaw;
  rc.gait_index = 2;  // tripod
  rc.body_height = height_fraction;
  rc.stride = 0.75f;       // 60 mm of 80
  rc.step_height = 0.6f;   // 30 mm of 50
  rc.duty = 159.0f / 255.0f;
  rc.speed = 0.5f;
  rc.twist_vx = 0.0f;
  rc.twist_vy = 0.0f;
  rc.twist_wz = 0.0f;
  rc.trim_pitch = trim_pitch_rad;

  // Arm and settle: neutral sticks while the shaper walks the height down.
  for (int i = 0; i < 5; ++i) adapter.advance(10);
  intent.rc.armed = true;
  for (int i = 0; i < 1500; ++i) adapter.advance(10);  // 15 s

  // Walk forward.
  rc.twist_vy = 1.0f;
  const RobotConfig& cfg = adapter.config().robot;
  float min_z[kNumLegs], max_z[kNumLegs];
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    min_z[leg] = 1e9f;
    max_z[leg] = -1e9f;
  }
  bool clamped = false;
  uint16_t applied_height = 0;
  for (int i = 0; i < 600; ++i) {  // 6 s of walking
    const RobotCommand& cmd = adapter.advance(10);
    if (!cmd.motion_gate || !cmd.goal_valid) {
      printf("frac=%.2f: motion gate closed at i=%d (state=%u)\n",
             (double)height_fraction, i,
             (unsigned)cmd.safety_state);
      return;
    }
    if (cmd.diagnostics.any_goal_clamped) clamped = true;
    applied_height = cmd.diagnostics.applied_body_height_mm;
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      float x, y, z;
      if (goalsFootBody(cfg, cmd, leg, x, y, z)) {
        if (z < min_z[leg]) min_z[leg] = z;
        if (z > max_z[leg]) max_z[leg] = z;
      }
    }
  }
  printf("frac=%.2f trim=%.3f applied_height=%u clamped=%d\n",
         (double)height_fraction, (double)trim_pitch_rad, applied_height,
         clamped);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    printf("  leg%u z=[%7.2f .. %7.2f] lift=%6.2f mm\n", leg + 1,
           (double)min_z[leg], (double)max_z[leg],
           (double)(max_z[leg] - min_z[leg]));
  }
}

int main() {
  walkAt(0.5f, 0.0f);            // neutral 132 mm, no trim
  walkAt(0.0f, 0.0f);            // full crouch, no trim
  walkAt(0.0f, 0.0175f);         // full crouch, 1 deg pitch trim
  walkAt(0.0f, 0.0524f);         // full crouch, 3 deg pitch trim
  walkAt(0.5f, 0.0524f);         // neutral, 3 deg pitch trim (control)
  return 0;
}
