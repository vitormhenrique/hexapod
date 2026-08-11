// Host probe: measure the NET COMMANDED body motion per gait cycle by
// rigid-fitting stance-foot displacements from the emitted servo ticks
// (post servo-map quantization). A straight forward walk must show zero
// lateral velocity and zero yaw rate. Mirrors the host-motion (companion)
// path: tripod, h=132, stride 60, step 30, duty 159, speed 128, vy=0.6.
#include <math.h>
#include <stdio.h>

#include "controller_sim_adapter.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/leg_ik.h"

using namespace controller;
using namespace config;

struct Foot {
  float x, y, z;
  bool valid;
};

static Foot goalsFootBody(const RobotConfig& cfg, const RobotCommand& cmd,
                          uint8_t leg) {
  dxl::ServoMap map(cfg);
  float angles[kJointsPerLeg] = {};
  bool found[kJointsPerLeg] = {};
  for (uint8_t index = 0; index < cmd.goals.count; ++index) {
    const auto& goal = cmd.goals.joints[index];
    if (goal.leg == leg && goal.joint < kJointsPerLeg) {
      angles[goal.joint] = dxl::ServoMap(cfg).tickToAngle(leg, goal.joint, goal.tick);
      found[goal.joint] = true;
    }
  }
  Foot f{0, 0, 0, false};
  if (!(found[0] && found[1] && found[2])) return f;
  gait::LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
                 cfg.links.tibia_cmm / 100.0f,
                 cfg.geometry.home_radius_cmm / 100.0f,
                 cfg.geometry.home_foot_z_cmm / 100.0f);
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

int main() {
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
  motion.gait = 2;  // tripod
  motion.body_height_mm = 132;
  motion.stride_len_mm = 60;
  motion.step_height_mm = 30;
  motion.duty_x255 = 159;
  motion.speed_x255 = 128;
  motion.seq = 1;

  for (int i = 0; i < 800; ++i) adapter.advance(10);  // settle standing
  motion.twist_vx = 0.6f;  // motion-intent vx = FORWARD (command frame)
  motion.seq = 2;
  for (int i = 0; i < 400; ++i) adapter.advance(10);  // reach steady state

  const RobotConfig& cfg = adapter.config().robot;
  Foot prev[kNumLegs];
  for (uint8_t leg = 0; leg < kNumLegs; ++leg)
    prev[leg] = goalsFootBody(cfg, adapter.command(), leg);

  // Integrate body pose from stance-foot rigid fit over N cycles.
  double body_x = 0, body_y = 0, body_yaw = 0;
  double sum_w = 0;  // instantaneous yaw-rate accumulation for reporting
  const int kSteps = 3200;  // 32 s of walking
  float stance_z_thresh = 0.0f;
  {
    // stance plane = min commanded z + 1 mm
    float zmin = 0;
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) zmin = fminf(zmin, prev[leg].z);
    stance_z_thresh = zmin + 1.0f;
  }
  int frames_with_goals = 0, frames_fit = 0;
  int first_no_goal = -1;
  for (int i = 0; i < kSteps; ++i) {
    const RobotCommand& cmd = adapter.advance(10);
    if (cmd.goals.count > 0) ++frames_with_goals;
    else if (first_no_goal < 0) first_no_goal = i;
    // Proper small-motion rigid fit (centered least squares):
    //   foot displacement d_i = dtheta * J * p_i - dc   (J = 90deg rotation)
    // Solve dtheta from centered cross terms, then dc from centroids.
    double px[kNumLegs], py[kNumLegs], dx[kNumLegs], dy[kNumLegs];
    int n = 0;
    Foot cur[kNumLegs];
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      cur[leg] = goalsFootBody(cfg, cmd, leg);
      if (!cur[leg].valid || !prev[leg].valid) continue;
      if (cur[leg].z > stance_z_thresh || prev[leg].z > stance_z_thresh)
        continue;
      px[n] = 0.5 * (cur[leg].x + prev[leg].x);
      py[n] = 0.5 * (cur[leg].y + prev[leg].y);
      dx[n] = cur[leg].x - prev[leg].x;
      dy[n] = cur[leg].y - prev[leg].y;
      ++n;
    }
    if (n >= 3) {
      ++frames_fit;
      double pbx = 0, pby = 0, dbx = 0, dby = 0;
      for (int k = 0; k < n; ++k) {
        pbx += px[k]; pby += py[k]; dbx += dx[k]; dby += dy[k];
      }
      pbx /= n; pby /= n; dbx /= n; dby /= n;
      double num = 0, den = 0;
      for (int k = 0; k < n; ++k) {
        const double cx = px[k] - pbx, cy = py[k] - pby;
        const double ex = dx[k] - dbx, ey = dy[k] - dby;
        // d = dtheta * (py, -px)... from d_i = dtheta*J*p_i - dc with
        // J*p = (py, -px) per the derivation in the analysis notes:
        num += ex * cy - ey * cx;
        den += cx * cx + cy * cy;
      }
      const double dtheta = den > 0 ? num / den : 0.0;
      // dc (body translation in body frame) from centroids:
      const double dcx = dtheta * pby - dbx;
      const double dcy = -dtheta * pbx - dby;
      // Integrate in world frame.
      const double c = cos(body_yaw), s = sin(body_yaw);
      body_x += c * dcx - s * dcy;
      body_y += s * dcx + c * dcy;
      body_yaw += dtheta;
      sum_w += dtheta;
    }
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) prev[leg] = cur[leg];
  }
  const double dist = sqrt(body_x * body_x + body_y * body_y);
  printf("debug: frames_with_goals=%d/%d first_no_goal=%d frames_fit=%d\n",
         frames_with_goals, kSteps, first_no_goal, frames_fit);
  printf("commanded body motion over %.1f s (body frame: +Y fwd, +X right):\n",
         kSteps * 0.01);
  printf("  forward=%+.2f mm  lateral=%+.2f mm (neg=left)  |d|=%.1f mm\n",
         body_y, body_x, dist);
  printf("  net yaw=%+.5f rad (%+.3f deg)\n", body_yaw, body_yaw * 57.2958);
  if (fabs(body_y) > 1.0) {
    printf("  lateral drift: %+.2f mm per meter forward\n",
           body_x / (fabs(body_y) / 1000.0));
    printf("  heading error: %+.3f deg\n", atan2(-body_x, body_y) * 57.2958);
  }
  return 0;
}
