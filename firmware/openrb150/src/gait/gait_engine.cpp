// Mark III Phoenix gait engine (portable, host-tested). See gait_engine.h.

#include "gait_engine.h"

#include <math.h>

namespace gait {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kQ15 = 32767.0f;

// Measured CAD coxa mounts in body frame B (dimensions.md, 0.1 mm) and the
// home hip azimuth each leg points along. The documented home feet (corners
// at (+-155.2, +-205.2), mids at (+-196.5, 0)) are mount + 126.75 mm along the
// azimuth. Mark III supplies gait phase/keyframe behavior only; physical
// dimensions always come from this robot's measured model (or the adopted
// RobotConfig via setLegGeometry, which overrides this table).
constexpr int16_t kDefaultMountDmm[config::kNumLegs][2] = {
  {-656, -1156}, {656, -1156}, {698, 0},
  {656, 1156},   {-656, 1156}, {-698, 0},
};
constexpr float kDefaultAzimuthDeg[config::kNumLegs] = {
  225.0f, 315.0f, 0.0f, 45.0f, 135.0f, 180.0f,
};

inline int16_t toQ15(float v) {
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  // Snap float noise on axis-aligned legs so a mid leg's home y is exactly 0.
  if (v > -1e-6f && v < 1e-6f) return 0;
  return static_cast<int16_t>(v * kQ15 + (v >= 0.0f ? 0.5f : -0.5f));
}

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
              float half_lift, float& longitudinal, float& lift) {
  const uint8_t relative = static_cast<uint8_t>(
      (gait_step + gait.steps - gait.origin[leg]) % gait.steps);
  lift = 0.0f;
  if (relative == 0) {
    longitudinal = 0.0f;
    lift = 1.0f;
  } else if (relative == 1) {
    longitudinal = 0.5f;
    lift = half_lift;
  } else if (relative == 2) {
    longitudinal = 0.5f;
  } else if (relative == gait.steps - 1) {
    longitudinal = -0.5f;
    lift = half_lift;
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

GaitEngine::GaitEngine() {
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    mount_x_dmm_[leg] = kDefaultMountDmm[leg][0];
    mount_y_dmm_[leg] = kDefaultMountDmm[leg][1];
    azimuth_cos_q15_[leg] = toQ15(cosf(kDefaultAzimuthDeg[leg] * kDegToRad));
    azimuth_sin_q15_[leg] = toQ15(sinf(kDefaultAzimuthDeg[leg] * kDegToRad));
  }
}

void GaitEngine::setLegGeometry(const config::RobotConfig& cfg) {
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    const config::LegGeometry& g = cfg.legs[leg];
    mount_x_dmm_[leg] = g.mount_x_dmm;
    mount_y_dmm_[leg] = g.mount_y_dmm;
    // The persisted mount yaw is the servo-180 pointing direction; the home
    // foot azimuth in B is a quarter turn CCW from it (body_ik.cpp applies the
    // inverse rotation -(yaw + 90 deg) to bring B vectors into the coxa frame).
    const float azimuth =
        static_cast<float>(g.mount_yaw_cdeg) * (kDegToRad / 100.0f) +
        kPi / 2.0f;
    azimuth_cos_q15_[leg] = toQ15(cosf(azimuth));
    azimuth_sin_q15_[leg] = toQ15(sinf(azimuth));
  }
  if (cfg.links.coxa_cmm > 0 && cfg.links.femur_cmm > 0 &&
      cfg.links.tibia_cmm > 0) {
    l1_mm_ = static_cast<float>(cfg.links.coxa_cmm) / 100.0f;
    l23_mm_ = static_cast<float>(cfg.links.femur_cmm + cfg.links.tibia_cmm) /
              100.0f;
  }
}

void GaitEngine::setShape(const GaitShape& shape) {
  // Stride / step height stay owned by configure(); only the remaining slots
  // are read here.
  stance_radius_target_ =
      static_cast<float>(shape.get(TuneId::StanceRadiusMm));
  yaw_stroke_target_rad_ =
      static_cast<float>(shape.get(TuneId::YawStrokeDeg)) * kDegToRad;
  half_lift_target_ =
      clampf(static_cast<float>(shape.get(TuneId::HalfLiftPct)) / 100.0f,
             0.0f, 1.0f);
  uint16_t fast = shape.get(TuneId::StepPeriodFastMs);
  uint16_t slow = shape.get(TuneId::StepPeriodSlowMs);
  // "Fast" must not be slower than "slow"; a crossed pair degrades to a fixed
  // cadence rather than inverting the speed knob.
  if (slow < fast) slow = fast;
  step_period_fast_ms_ = fast;
  step_period_slow_ms_ = slow;
  if (!params_seeded_) {
    stance_radius_mm_ = stance_radius_target_;
    yaw_stroke_rad_ = yaw_stroke_target_rad_;
    half_lift_ = half_lift_target_;
  }
}

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
    stance_radius_mm_ = stance_radius_target_;
    yaw_stroke_rad_ = yaw_stroke_target_rad_;
    half_lift_ = half_lift_target_;
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

float GaitEngine::maxStanceRadiusMm(float body_height_mm) const {
  // Home foot distance from the femur axis must stay inside
  // kHomeReachFrac * (IK reach margin). Solve the planar radial component at
  // the current ride height; with the body too high for any radial room the
  // cap collapses onto the coxa link, which the table minimum then overrides.
  const float d_limit = kHomeReachFrac * kIkReachMarginFrac * l23_mm_;
  const float h = fabsf(body_height_mm);
  if (h >= d_limit) return l1_mm_;
  return l1_mm_ + sqrtf(d_limit * d_limit - h * h);
}

void GaitEngine::baseHomeFoot(uint8_t leg, float& x, float& y,
                              float& z) const {
  x = static_cast<float>(mount_x_dmm_[leg]) * 0.1f +
      stance_radius_mm_ * (static_cast<float>(azimuth_cos_q15_[leg]) / kQ15);
  y = static_cast<float>(mount_y_dmm_[leg]) * 0.1f +
      stance_radius_mm_ * (static_cast<float>(azimuth_sin_q15_[leg]) / kQ15);
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
  // Yaw stroke is an explicit operator parameter: the body yaw swept over one
  // full cycle at full yaw stick. Phoenix Mark III used a fixed 32 deg; that
  // moved corner feet ~2.4x the translation stride at the old 60 mm default
  // and saturated the reach limiter, so the stroke is tunable (default 20 deg)
  // instead of being derived from stride.
  const float yaw = twist_.wz * yaw_stroke_rad_ * longitudinal;
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  x = home_x * cosine - home_y * sine +
      twist_.vx * stride_mm_ * longitudinal;
  y = home_x * sine + home_y * cosine +
      twist_.vy * stride_mm_ * longitudinal;
  // Swing clearance is a gait parameter, not a function of stick deflection:
  // a small travel command still gets the full configured step height so slow
  // walking never drags feet (Phoenix keeps LegLiftHeight fixed the same way).
  z = clampf(home_z + step_mm_ * clampf(lift_fraction, 0.0f, 1.0f),
             kMinFootZMm, kMaxFootZMm);
}

void GaitEngine::motionEnvelopeFoot(uint8_t leg, float longitudinal,
                                    float lift_fraction, float& x, float& y,
                                    float& z) const {
  footAt(leg, longitudinal, lift_fraction, x, y, z);
}

void GaitEngine::previewFoot(uint8_t leg, float phase, float& x, float& y,
                             float& z) const {
  float home_x, home_y, home_z;
  homeFoot(leg, home_x, home_y, home_z);
  phase = phase - floorf(phase);
  float longitudinal = 0.0f;
  float lift = 0.0f;
  if (phase < 0.5f) {
    // Swing: forward through the full configured clearance.
    const float u = phase * 2.0f;
    longitudinal = -0.5f + u;
    lift = sinf(3.14159265f * u);
  } else {
    // Stance: back along the ground, so the stroke length is equally visible.
    const float u = (phase - 0.5f) * 2.0f;
    longitudinal = 0.5f - u;
  }
  x = home_x;
  y = home_y + stride_mm_ * longitudinal;  // body +Y is forward
  z = clampf(home_z + step_mm_ * lift, kMinFootZMm, kMaxFootZMm);
}

float GaitEngine::stepPeriodMs() const {
  // Piecewise-linear about a neutral point two thirds of the way from slow to
  // fast (120 / 60 ms -> 80 ms at speed 0.5, the historical cadence), so the
  // lower half of the knob stretches the period and the upper half tightens.
  constexpr float kNeutralSpeed = 128.0f / 255.0f;
  const float slow = static_cast<float>(step_period_slow_ms_);
  const float fast = static_cast<float>(step_period_fast_ms_);
  const float neutral = slow - (slow - fast) * (2.0f / 3.0f);
  if (speed_ <= kNeutralSpeed) {
    return slow - (slow - neutral) * (speed_ / kNeutralSpeed);
  }
  return neutral -
         (neutral - fast) * ((speed_ - kNeutralSpeed) / (1.0f - kNeutralSpeed));
}

void GaitEngine::update(uint32_t dt_ms, GaitOutput& out) {
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

  // Advance the shape-parameter filters (body height, stride, step, speed,
  // stance radius, yaw stroke, half lift).
  const float pk = (dt_s >= kParamFilterTau) ? 1.0f : dt_s / kParamFilterTau;
  stride_mm_ += (stride_target_ - stride_mm_) * pk;
  step_mm_ += (step_target_ - step_mm_) * pk;
  body_height_mm_ += (height_target_ - body_height_mm_) * pk;
  speed_ += (speed_target_ - speed_) * pk;
  // The stance radius target is capped against the workspace at the filtered
  // ride height every cycle, so lowering/raising the body via Pot2 widens or
  // narrows the reachable stance smoothly instead of leaving the home foot
  // outside the IK annulus.
  const float radius_cap = maxStanceRadiusMm(body_height_mm_);
  const float radius_goal = stance_radius_target_ < radius_cap
                                ? stance_radius_target_
                                : radius_cap;
  stance_radius_mm_ += (radius_goal - stance_radius_mm_) * pk;
  yaw_stroke_rad_ += (yaw_stroke_target_rad_ - yaw_stroke_rad_) * pk;
  half_lift_ += (half_lift_target_ - half_lift_) * pk;

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
    keyframe(gait, leg, current_step, half_lift_, current_longitudinal,
             current_lift);
    keyframe(gait, leg, next_step, half_lift_, next_longitudinal, next_lift);
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
