// Gait -> servo goal pipeline (portable, host-tested). See gait_pipeline.h.

#include "gait_pipeline.h"

namespace gait {

GaitPipeline::GaitPipeline(const config::RobotConfig& cfg)
    : cfg_(cfg), body_(cfg), map_(cfg) {
  configureFromConfig();
}

void GaitPipeline::configureFromConfig() { engine_.configure(cfg_.gait); }

void GaitPipeline::reconfigure() {
  // Body IK caches link lengths + per-leg geometry by value, so rebuild it from
  // the (updated) config. The servo map holds a live reference and needs no
  // action. Re-seed the gait engine defaults last.
  body_ = BodyKinematics(cfg_);
  configureFromConfig();
}

void GaitPipeline::setGait(config::GaitId g) { engine_.setGait(g); }

void GaitPipeline::setParams(uint16_t body_height_mm, uint16_t stride_len_mm,
                             uint16_t step_height_mm, uint8_t duty_x255,
                             uint8_t speed_x255) {
  config::GaitDefaults d = cfg_.gait;
  d.body_height_mm = body_height_mm;
  d.stride_len_mm = stride_len_mm;
  d.step_height_mm = step_height_mm;
  d.duty_x255 = duty_x255;
  d.speed_x255 = speed_x255;
  // Preserve the currently selected gait; configure() would otherwise reset it
  // to the config default.
  const config::GaitId g = engine_.gait();
  engine_.configure(d);
  engine_.setGait(g);
}

void GaitPipeline::setTwist(float vx, float vy, float wz) {
  BodyTwist t;
  t.vx = vx;
  t.vy = vy;
  t.wz = wz;
  engine_.setTwist(t);
}

void GaitPipeline::setBodyPose(const BodyPose& pose) {
  pose_ = pose;
  apply_pose_ = (pose.x_mm != 0.0f || pose.y_mm != 0.0f || pose.z_mm != 0.0f ||
                 pose.roll != 0.0f || pose.pitch != 0.0f || pose.yaw != 0.0f);
}

void GaitPipeline::resetPhase() { engine_.reset(); }

void GaitPipeline::update(uint32_t dt_ms, PipelineOutput& out) {
  GaitOutput feet;
  engine_.update(dt_ms, feet);

  out.count = 0;
  out.any_unreachable = false;
  out.any_reach_limited = false;
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    const FootTarget& f = feet.feet[leg];
    IkResult ik;
    if (apply_pose_) {
      // Body-pose mode: the gait foot target is treated as a world-fixed
      // foothold and re-expressed in the moved body frame, so the body shifts
      // and tilts over planted feet (oha.3). The pose is already clamped to a
      // safe envelope by the caller, so no reach-margin pull-in is applied;
      // unreachable targets are still reported.
      ik = body_.solveBodyPose(leg, pose_, f.x_mm, f.y_mm, f.z_mm);
    } else {
      bool reach_limited = false;
      ik = body_.solveBodyLimited(leg, f.x_mm, f.y_mm, f.z_mm, reach_limited);
      if (reach_limited) {
        out.any_reach_limited = true;
      }
    }
    if (!ik.reachable) {
      out.any_unreachable = true;
    }
    // JointRole order: Coxa=0, Femur=1, Tibia=2 (config_schema.h), matching the
    // IkResult fields so the angle index lines up with the joint index.
    const float angles[config::kJointsPerLeg] = {ik.coxa, ik.femur, ik.tibia};
    for (uint8_t j = 0; j < config::kJointsPerLeg; ++j) {
      const config::ServoConfig* sc = map_.servoFor(leg, j);
      if (sc == nullptr) {
        continue;  // no servo mapped for this slot; emit nothing
      }
      const dxl::JointCommand jc = map_.angleToTick(leg, j, angles[j]);
      PipelineJoint& pj = out.joints[out.count++];
      pj.id = sc->id;
      pj.tick = jc.tick;
      pj.leg = leg;
      pj.joint = j;
      pj.clamped = jc.clamped_low || jc.clamped_high;
    }
  }
}

}  // namespace gait
