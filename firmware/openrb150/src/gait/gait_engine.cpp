// Mark III Phoenix gait engine (portable, host-tested). See gait_engine.h.

#include "gait_engine.h"

#include <math.h>

namespace gait {
namespace {

// Mark III feet converted from Phoenix coordinates to body frame B. Firmware
// leg order is LR, RR, RM, RF, LF, LM so physical legs 3 and 6 remain the two
// middle legs used by existing telemetry and calibration.
constexpr float kHomeFootXy[config::kNumLegs][2] = {
    {-164.0f, -224.0f}, {164.0f, -224.0f}, {247.0f, 0.0f},
    {164.0f, 224.0f},   {-164.0f, 224.0f}, {-247.0f, 0.0f},
};

struct MarkIiiGait {
  uint8_t steps;
  uint8_t travel_divisor;
  uint8_t origin[config::kNumLegs];
};

constexpr MarkIiiGait kTripod = {8, 4, {5, 1, 5, 1, 5, 1}};
constexpr MarkIiiGait kRipple = {12, 8, {1, 7, 11, 3, 9, 5}};
constexpr MarkIiiGait kWave = {24, 20, {1, 13, 17, 21, 9, 5}};

const MarkIiiGait& gaitDefinition(config::GaitId gait) {
  switch (gait) {
    case config::GaitId::Tripod:
      return kTripod;
    case config::GaitId::Ripple:
      return kRipple;
    case config::GaitId::Wave:
    case config::GaitId::Crawl:
    default:
      return kWave;
  }
}

void keyframe(const MarkIiiGait& gait, uint8_t leg, uint8_t gait_step,
              float& longitudinal, float& lift) {
  const uint8_t relative = static_cast<uint8_t>(
      (gait_step + gait.steps - gait.origin[leg]) % gait.steps);
  lift = 0.0f;
  if (relative == 0) {
    longitudinal = 0.0f;
    lift = 1.0f;
  } else if (relative == 1) {
    longitudinal = 0.5f;
    lift = 0.5f;
  } else if (relative == 2) {
    longitudinal = 0.5f;
  } else if (relative == gait.steps - 1) {
    longitudinal = -0.5f;
    lift = 0.5f;
  } else {
    longitudinal =
        0.5f - static_cast<float>(relative - 2) /
                   static_cast<float>(gait.travel_divisor);
  }
}

// Per-gait minimum stance duty factor (fraction of the cycle grounded). A
// requested duty may lengthen stance but must never destabilize a gait by
// dropping below this nominal support pattern.
float minimumGaitDuty(config::GaitId g) {
  switch (g) {
    case config::GaitId::Tripod:
      return 5.0f / 8.0f;
    case config::GaitId::Ripple:
      return 9.0f / 12.0f;
    case config::GaitId::Wave:
    case config::GaitId::Crawl:
      return 21.0f / 24.0f;
    default:  // Stand / Sit: always grounded
      return 1.0f;
  }
}

// Per-leg phase offset within the cycle for each gait (IK ref section 10).
inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

GaitEngine::GaitEngine() {}

void GaitEngine::configure(const config::GaitDefaults& d) {
  gait_ = static_cast<config::GaitId>(d.gait);
  // Live shape values are filter TARGETS: the RC/host feeds raw knob values
  // every cycle and update()'s first-order lag delivers them to the trajectory
  // smoothly, so pot/ADC noise never jumps the body or the stride.
  stride_target_ = clampf(static_cast<float>(d.stride_len_mm), 0.0f, kMaxStrideMm);
  step_target_ = clampf(static_cast<float>(d.step_height_mm), 0.0f, kMaxStepMm);
  height_target_ = static_cast<float>(d.body_height_mm);
  speed_target_ = clampf(static_cast<float>(d.speed_x255) / 255.0f, 0.0f, 1.0f);
  if (!params_seeded_) {
    // First configuration (boot / pipeline construction): start the filters
    // exactly on target so the initial pose is the configured one.
    stride_mm_ = stride_target_;
    step_mm_ = step_target_;
    body_height_mm_ = height_target_;
    speed_ = speed_target_;
    params_seeded_ = true;
  }
}

void GaitEngine::setGait(config::GaitId g) { gait_ = g; }

void GaitEngine::setTwist(const BodyTwist& t) {
  twist_.vx = clampf(t.vx, -1.0f, 1.0f);
  twist_.vy = clampf(t.vy, -1.0f, 1.0f);
  twist_.wz = clampf(t.wz, -1.0f, 1.0f);
  if (fabsf(twist_.vx) <= kMotionDeadband) twist_.vx = 0.0f;
  if (fabsf(twist_.vy) <= kMotionDeadband) twist_.vy = 0.0f;
  if (fabsf(twist_.wz) <= kMotionDeadband) twist_.wz = 0.0f;
}

void GaitEngine::reset() {
  phase_ = 0.0f;
  gait_step_phase_ = 0.0f;
}

float GaitEngine::dutyFactor() const {
  return minimumGaitDuty(gait_);
}

void GaitEngine::baseHomeFoot(uint8_t leg, float& x, float& y,
                              float& z) const {
  x = kHomeFootXy[leg][0];
  y = kHomeFootXy[leg][1];
  z = -body_height_mm_;
}

void GaitEngine::homeFoot(uint8_t leg, float& x, float& y, float& z) const {
  baseHomeFoot(leg, x, y, z);
}

void GaitEngine::footAt(uint8_t leg, float longitudinal,
                        float lift_fraction, float& x, float& y,
                        float& z) const {
  float home_x, home_y, home_z;
  homeFoot(leg, home_x, home_y, home_z);
  const float yaw = twist_.wz * kMarkIiiYawTravelRad * longitudinal;
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  x = home_x * cosine - home_y * sine +
      twist_.vx * stride_mm_ * longitudinal;
  y = home_x * sine + home_y * cosine +
      twist_.vy * stride_mm_ * longitudinal;
    const float command_scale =
      fmaxf(fabsf(twist_.vx), fmaxf(fabsf(twist_.vy), fabsf(twist_.wz)));
    const float lift_scale = clampf(command_scale * 4.0f, 0.0f, 1.0f);
    z = clampf(home_z + step_mm_ * lift_scale *
                clampf(lift_fraction, 0.0f, 1.0f),
             kMinFootZMm, kMaxFootZMm);
}

void GaitEngine::motionEnvelopeFoot(uint8_t leg, float longitudinal,
                                    float lift_fraction, float& x, float& y,
                                    float& z) const {
  footAt(leg, longitudinal, lift_fraction, x, y, z);
}

float GaitEngine::stepPeriodMs() const {
  constexpr float kNeutralSpeed = 128.0f / 255.0f;
  if (speed_ <= kNeutralSpeed) {
    return kMaxStepPeriodMs -
           (kMaxStepPeriodMs - kNeutralStepPeriodMs) *
               (speed_ / kNeutralSpeed);
  }
  return kNeutralStepPeriodMs -
         (kNeutralStepPeriodMs - kMinStepPeriodMs) *
             ((speed_ - kNeutralSpeed) / (1.0f - kNeutralSpeed));
}

void GaitEngine::update(uint32_t dt_ms, GaitOutput& out) {
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

  // Advance the shape-parameter filters (body height, stride, step, speed).
  const float pk = (dt_s >= kParamFilterTau) ? 1.0f : dt_s / kParamFilterTau;
  stride_mm_ += (stride_target_ - stride_mm_) * pk;
  step_mm_ += (step_target_ - step_mm_) * pk;
  body_height_mm_ += (height_target_ - body_height_mm_) * pk;
  speed_ += (speed_target_ - speed_) * pk;

  // Static poses: no stepping, no twist.
  if (gait_ == config::GaitId::Stand || gait_ == config::GaitId::Sit) {
    for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
      float hx, hy, hz;
      homeFoot(leg, hx, hy, hz);
      out.feet[leg].x_mm = hx;
      out.feet[leg].y_mm = hy;
      out.feet[leg].z_mm =
          (gait_ == config::GaitId::Sit) ? kSitFootZMm : hz;
      out.feet[leg].swing = false;
    }
    return;
  }

  if (fabsf(twist_.vx) < kTwistParkPos &&
      fabsf(twist_.vy) < kTwistParkPos &&
      fabsf(twist_.wz) < kTwistParkPos) {
    twist_.vx = twist_.vy = twist_.wz = 0.0f;
  }

  const float command_scale =
      fmaxf(fabsf(twist_.vx), fmaxf(fabsf(twist_.vy), fabsf(twist_.wz)));
  if (command_scale <= 0.0f) {
    for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
      FootTarget& foot = out.feet[leg];
      homeFoot(leg, foot.x_mm, foot.y_mm, foot.z_mm);
      foot.swing = false;
    }
    return;
  }

  const MarkIiiGait& gait = gaitDefinition(gait_);
  gait_step_phase_ += static_cast<float>(dt_ms) / stepPeriodMs();
  while (gait_step_phase_ >= static_cast<float>(gait.steps)) {
    gait_step_phase_ -= static_cast<float>(gait.steps);
  }
  phase_ = gait_step_phase_ / static_cast<float>(gait.steps);
  const uint8_t current_step =
      static_cast<uint8_t>(floorf(gait_step_phase_)) + 1;
  const uint8_t next_step =
      current_step == gait.steps ? 1 : static_cast<uint8_t>(current_step + 1);
  const float interpolation = gait_step_phase_ - floorf(gait_step_phase_);

  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    float current_longitudinal, current_lift;
    float next_longitudinal, next_lift;
    keyframe(gait, leg, current_step, current_longitudinal, current_lift);
    keyframe(gait, leg, next_step, next_longitudinal, next_lift);
    const float longitudinal =
        current_longitudinal +
        (next_longitudinal - current_longitudinal) * interpolation;
    const float lift =
        current_lift + (next_lift - current_lift) * interpolation;
    FootTarget& foot = out.feet[leg];
    footAt(leg, longitudinal, lift, foot.x_mm, foot.y_mm, foot.z_mm);
    foot.swing = lift > 1e-4f;
  }
}

}  // namespace gait
