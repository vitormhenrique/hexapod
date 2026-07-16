// Gait -> servo goal pipeline (portable, host-tested). See gait_pipeline.h.

#include "gait_pipeline.h"

#include <math.h>

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
  engine_.setGait(g);  // The speed knob (Pot1) also drives the goal slew limiter, so one remote
  // control scales cadence, twist response AND how briskly the servos are
  // allowed to chase their targets.
  const float s = static_cast<float>(speed_x255) / 255.0f;
  goal_slew_ticks_per_s_ =
      kMinGoalSlewTicksPerSec +
      (kMaxGoalSlewTicksPerSec - kMinGoalSlewTicksPerSec) * s;}

void GaitPipeline::setTwist(float vx, float vy, float wz) {
  BodyTwist t;
  t.vx = vx;
  t.vy = vy;
  t.wz = wz;
  engine_.setTwist(t);
}

void GaitPipeline::setBodyPose(const BodyPose& pose) {
  // Store the raw command as the filter target; update() advances the filtered
  // pose toward it so stick noise never jumps the body.
  pose_target_ = pose;
}

void GaitPipeline::resetPhase() { engine_.reset(); }

void GaitPipeline::seedGoal(uint8_t id, uint16_t present_tick) {
  const config::ServoConfig* sc = map_.servoForId(id);
  if (sc == nullptr) return;
  const uint8_t slot =
      static_cast<uint8_t>(sc->leg * config::kJointsPerLeg + sc->joint);
  if (slot >= config::kNumServos) return;
  last_tick_[slot] = present_tick;
  ramping_[slot] = true;
}

void GaitPipeline::update(uint32_t dt_ms, PipelineOutput& out) {
  // Advance the body-pose filter toward the latest command, snapping to exact
  // neutral once the residual is imperceptible so the normal walk path (and
  // its reachability limiting) is restored bit-exactly.
  {
    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
    const float k = (dt_s >= kPoseFilterTau) ? 1.0f : dt_s / kPoseFilterTau;
    pose_.x_mm += (pose_target_.x_mm - pose_.x_mm) * k;
    pose_.y_mm += (pose_target_.y_mm - pose_.y_mm) * k;
    pose_.z_mm += (pose_target_.z_mm - pose_.z_mm) * k;
    pose_.roll += (pose_target_.roll - pose_.roll) * k;
    pose_.pitch += (pose_target_.pitch - pose_.pitch) * k;
    pose_.yaw += (pose_target_.yaw - pose_.yaw) * k;
    const bool target_neutral =
        pose_target_.x_mm == 0.0f && pose_target_.y_mm == 0.0f &&
        pose_target_.z_mm == 0.0f && pose_target_.roll == 0.0f &&
        pose_target_.pitch == 0.0f && pose_target_.yaw == 0.0f;
    if (target_neutral && fabsf(pose_.x_mm) < kPoseSnapMm &&
        fabsf(pose_.y_mm) < kPoseSnapMm && fabsf(pose_.z_mm) < kPoseSnapMm &&
        fabsf(pose_.roll) < kPoseSnapRad &&
        fabsf(pose_.pitch) < kPoseSnapRad &&
        fabsf(pose_.yaw) < kPoseSnapRad) {
      pose_ = BodyPose{};
    }
    apply_pose_ =
        (pose_.x_mm != 0.0f || pose_.y_mm != 0.0f || pose_.z_mm != 0.0f ||
         pose_.roll != 0.0f || pose_.pitch != 0.0f || pose_.yaw != 0.0f);
  }

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
      // and tilts over planted feet (oha.3). The API envelope alone cannot
      // guarantee every transformed leg target is reachable, so apply the same
      // final safe-annulus clamp used by ordinary gait targets.
      bool reach_limited = false;
      ik = body_.solveBodyPoseLimited(leg, pose_, f.x_mm, f.y_mm, f.z_mm,
                                      reach_limited);
      if (reach_limited) {
        out.any_reach_limited = true;
      }
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
      // Arm-transition ramp only: after seedGoal() the joint walks from its
      // present position toward the live trajectory at the Pot1-mapped rate;
      // the moment it reaches the trajectory the limiter disengages, so
      // steady-state gait output is never rate-clipped (no flatten-and-catch-
      // up distortion).
      const uint8_t slot =
          static_cast<uint8_t>(leg * config::kJointsPerLeg + j);
      uint16_t tick = jc.tick;
      if (ramping_[slot]) {
        const float max_step =
            goal_slew_ticks_per_s_ * static_cast<float>(dt_ms) / 1000.0f;
        const long limit = max_step < 1.0f ? 1L : static_cast<long>(max_step);
        const long prev = static_cast<long>(last_tick_[slot]);
        const long want = static_cast<long>(tick);
        if (want > prev + limit) {
          tick = static_cast<uint16_t>(prev + limit);
        } else if (want < prev - limit) {
          tick = static_cast<uint16_t>(prev - limit);
        } else {
          ramping_[slot] = false;  // reached the trajectory: hand over
        }
      }
      last_tick_[slot] = tick;
      PipelineJoint& pj = out.joints[out.count++];
      pj.id = sc->id;
      pj.tick = tick;
      pj.leg = leg;
      pj.joint = j;
      pj.clamped = jc.clamped_low || jc.clamped_high;
    }
  }
}

}  // namespace gait
