// Host probe: measure actual foot lift (via FK of emitted servo ticks) while
// walking at various body heights. Diagnoses the "drags feet at 65 mm crouch"
// report. Build/run: see probe command.
#include <math.h>
#include <stdio.h>

#include "../../src/config/config_schema.h"
#include "../../src/dxl/servo_map.h"
#include "../../src/gait/gait_pipeline.h"
#include "../../src/gait/leg_ik.h"

using namespace gait;
using namespace config;

static bool emittedFootBody(const RobotConfig& cfg, const PipelineOutput& out,
                            uint8_t leg, float& body_x, float& body_y,
                            float& body_z) {
  dxl::ServoMap map(cfg);
  float angles[kJointsPerLeg] = {};
  bool found[kJointsPerLeg] = {};
  for (uint8_t index = 0; index < out.count; ++index) {
    const PipelineJoint& goal = out.joints[index];
    if (goal.leg != leg || goal.joint >= kJointsPerLeg) continue;
    angles[goal.joint] = map.tickToAngle(leg, goal.joint, goal.tick);
    found[goal.joint] = true;
  }
  for (uint8_t joint = 0; joint < kJointsPerLeg; ++joint) {
    if (!found[joint]) return false;
  }
  LegIk ik(cfg.links.coxa_cmm / 100.0f, cfg.links.femur_cmm / 100.0f,
           cfg.links.tibia_cmm / 100.0f, cfg.geometry.home_radius_cmm / 100.0f,
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

static void walkAt(uint16_t body_height_mm, uint16_t step_height_mm,
                   float pose_z_mm) {
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(body_height_mm, 60, step_height_mm, 128, 128);
  pipe.setTwist(0.0f, 1.0f, 0.0f);  // walk forward (body +Y)
  BodyPose pose;
  pose.z_mm = pose_z_mm;
  pipe.setBodyPose(pose);

  // Let the shape filters settle at the target height first (stand ~1 s).
  for (int i = 0; i < 100; ++i) {
    PipelineOutput out;
    pipe.update(20, out);
  }

  float min_z[kNumLegs], max_z[kNumLegs];
  bool unreachable = false, reach_limited = false, clamped = false;
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    min_z[leg] = 1e9f;
    max_z[leg] = -1e9f;
  }
  for (int i = 0; i < 300; ++i) {  // ~6 s of walking
    PipelineOutput out;
    pipe.update(20, out);
    if (out.any_unreachable) unreachable = true;
    if (out.any_reach_limited) reach_limited = true;
    for (uint8_t k = 0; k < out.count; ++k) {
      if (out.joints[k].clamped) clamped = true;
    }
    for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
      float x, y, z;
      if (emittedFootBody(cfg, out, leg, x, y, z)) {
        if (z < min_z[leg]) min_z[leg] = z;
        if (z > max_z[leg]) max_z[leg] = z;
      }
    }
  }
  printf("body_height=%3u step=%2u pose_z=%5.1f  unreachable=%d reach_limited=%d clamped=%d\n",
         body_height_mm, step_height_mm, (double)pose_z_mm, unreachable,
         reach_limited, clamped);
  for (uint8_t leg = 0; leg < kNumLegs; ++leg) {
    printf("  leg%u  z=[%7.2f .. %7.2f]  lift=%6.2f mm\n", leg + 1,
           (double)min_z[leg], (double)max_z[leg],
           (double)(max_z[leg] - min_z[leg]));
  }
}

int main() {
  walkAt(132, 30, 0.0f);
  walkAt(65, 30, 0.0f);

  // Detail trace: leg 1 joint ticks + clamps over one swing at 65 mm crouch.
  RobotConfig cfg;
  defaultRobotConfig(cfg);
  GaitPipeline pipe(cfg);
  pipe.setGait(GaitId::Tripod);
  pipe.setParams(65, 60, 30, 159, 128);
  pipe.setTwist(0.0f, 1.0f, 0.0f);
  for (int i = 0; i < 200; ++i) {
    PipelineOutput out;
    pipe.update(20, out);
  }
  printf("\n t(ms) leg1: coxa femur tibia  clamp  foot_z\n");
  for (int i = 0; i < 40; ++i) {
    PipelineOutput out;
    pipe.update(20, out);
    uint16_t t[3] = {0, 0, 0};
    bool cl[3] = {false, false, false};
    for (uint8_t k = 0; k < out.count; ++k) {
      const PipelineJoint& j = out.joints[k];
      if (j.leg == 0) {
        t[j.joint] = j.tick;
        cl[j.joint] = j.clamped;
      }
    }
    float x, y, z;
    emittedFootBody(cfg, out, 0, x, y, z);
    printf("%5d       %4u %4u %4u   %c%c%c   %7.2f\n", i * 20, t[0], t[1], t[2],
           cl[0] ? 'C' : '-', cl[1] ? 'F' : '-', cl[2] ? 'T' : '-', (double)z);
  }
  return 0;
}
