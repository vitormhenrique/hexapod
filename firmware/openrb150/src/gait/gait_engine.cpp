// Gait engine v1 (portable, host-tested). See gait_engine.h.
// Reference: HexNav_description/docs/inverse_kinematics.md section 10/13.

#include "gait_engine.h"

#include <math.h>

namespace gait {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Home foot XY in body frame B (mm), IK ref section 13 HOME_FOOT. The Z comes
// from the configured body height.
constexpr float kHomeFootXy[config::kNumLegs][2] = {
    {-155.4f, -205.4f}, {155.4f, -205.4f}, {196.8f, 0.0f},
    {155.4f, 205.4f},   {-155.4f, 205.4f}, {-196.8f, 0.0f},
};

// Coxa mount XY in body frame B (mm), mirroring config kLegSeeds. Used to
// scale the stance radius about each hip for the height locus below.
constexpr float kCoxaMountXy[config::kNumLegs][2] = {
    {-65.6f, -115.6f}, {65.6f, -115.6f}, {69.8f, 0.0f},
    {65.6f, 115.6f},   {-65.6f, 115.6f}, {-69.8f, 0.0f},
};

// Reference-model constants for the constant-tibia-orientation stance locus.
// Changing body height must NOT simply bend the knee at a fixed foot radius:
// the whole leg reconfigures -- femur rotates, knee follows -- while the
// distal link keeps its home orientation (the calibrated tibia-vertical
// contact). That locus is a circle of radius L_FEMUR about the fixed distal
// offset: r(alpha) = L1 + L2*cos(alpha) + Cr, z(alpha) = L2*sin(alpha) + Cz,
// with Cr/Cz = L3*cos/sin(theta_d) and theta_d = 7.45 deg the model's distal
// orientation at the documented home pose.
constexpr float kIkL1Mm = 56.08f;
constexpr float kIkL2Mm = 66.51f;
constexpr float kDistalCrMm = 24.650f;  // L3 * cos(theta_d)
constexpr float kDistalCzMm = 3.224f;   // L3 * sin(theta_d)
constexpr float kCoxaZOffMm = 4.55f;    // coxa-frame z = body z - 4.55

// Stance radius (mm, horizontal distance from the hip axis) that keeps the
// distal link at its home orientation for a given body height.
inline float stanceRadiusForHeight(float bh_mm) {
  float s = (-bh_mm - kCoxaZOffMm - kDistalCzMm) / kIkL2Mm;
  if (s < -1.0f) s = -1.0f;
  if (s > 0.0f) s = 0.0f;
  return kIkL1Mm + kIkL2Mm * sqrtf(1.0f - s * s) + kDistalCrMm;
}

// Per-gait minimum stance duty factor (fraction of the cycle grounded). A
// requested duty may lengthen stance but must never destabilize a gait by
// dropping below this nominal support pattern.
float minimumGaitDuty(config::GaitId g) {
  switch (g) {
    case config::GaitId::Tripod:
      return 0.5f;
    case config::GaitId::Ripple:
      return 0.667f;
    case config::GaitId::Wave:
    case config::GaitId::Crawl:
      return 0.833f;
    default:  // Stand / Sit: always grounded
      return 1.0f;
  }
}

// Per-leg phase offset within the cycle for each gait (IK ref section 10).
float legOffset(config::GaitId g, uint8_t leg) {
  switch (g) {
    case config::GaitId::Tripod: {
      // Alternating tripods {1,3,5} / {2,4,6} (rear-left + mid-right +
      // front-left vs. the mirror). NOTE: the IK reference doc 10.1 grouping
      // {1,4,3}/{2,5,6} is geometrically defective -- its rear-left ->
      // front-right support diagonal passes exactly through the body centre,
      // leaving zero stability margin, so the robot teeters every half cycle.
      static const float kOff[config::kNumLegs] = {0.0f, 0.5f, 0.0f,
                                                   0.5f, 0.0f, 0.5f};
      return kOff[leg];
    }
    case config::GaitId::Ripple: {
      // Sequence 1->4->5->2->3->6 (legs 0,3,4,1,2,5), shift = position/6.
      static const float kOff[config::kNumLegs] = {
          0.0f, 3.0f / 6.0f, 4.0f / 6.0f, 1.0f / 6.0f, 2.0f / 6.0f, 5.0f / 6.0f};
      return kOff[leg];
    }
    case config::GaitId::Wave:
    case config::GaitId::Crawl: {
      // One leg at a time, sequence 1->2->3->4->5->6.
      return static_cast<float>(leg) / 6.0f;
    }
    default:
      return 0.0f;
  }
}

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline float frac01(float v) {
  v -= floorf(v);
  if (v >= 1.0f) v -= 1.0f;
  if (v < 0.0f) v += 1.0f;
  return v;
}

inline float smoothstep(float v) { return v * v * (3.0f - 2.0f * v); }

// One tracker axis step: critically-damped spring-damper pulled toward
// `target` with natural frequency `w`. Integrated in bounded substeps so the
// semi-implicit Euler stays stable for any caller dt, with the state clamped
// to the valid normalised twist range.
inline void trackAxis(float& x, float& v, float target, float w, float dt) {
  while (dt > 0.0f) {
    const float h = dt > 0.02f ? 0.02f : dt;
    dt -= h;
    const float a = w * w * (target - x) - 2.0f * w * v;
    v += a * h;
    x += v * h;
  }
  if (x > 1.0f) {
    x = 1.0f;
    if (v > 0.0f) v = 0.0f;
  } else if (x < -1.0f) {
    x = -1.0f;
    if (v < 0.0f) v = 0.0f;
  }
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
  requested_duty_ = clampf(static_cast<float>(d.duty_x255) / 255.0f, 0.0f,
                           kMaxDutyFactor);
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
  target_twist_.vx = clampf(t.vx, -1.0f, 1.0f);
  target_twist_.vy = clampf(t.vy, -1.0f, 1.0f);
  target_twist_.wz = clampf(t.wz, -1.0f, 1.0f);
  if (fabsf(target_twist_.vx) <= kMotionDeadband) target_twist_.vx = 0.0f;
  if (fabsf(target_twist_.vy) <= kMotionDeadband) target_twist_.vy = 0.0f;
  if (fabsf(target_twist_.wz) <= kMotionDeadband) target_twist_.wz = 0.0f;
}

void GaitEngine::reset() { phase_ = 0.0f; }

float GaitEngine::dutyFactor() const {
  const float minimum = minimumGaitDuty(gait_);
  if (minimum >= 1.0f) return 1.0f;
  // x255 cannot encode 0.5 exactly (128/255 is ~0.502). Treat values within
  // one wire quantization step of the nominal duty as nominal so the tripod
  // groups remain exactly opposite at the conventional value 128.
  constexpr float kDutyQuantization = 1.0f / 255.0f;
  return requested_duty_ > minimum + kDutyQuantization ? requested_duty_
                                                        : minimum;
}

void GaitEngine::homeFoot(uint8_t leg, float& x, float& y, float& z) const {
  // Height changes ride the constant-tibia-orientation locus: the stance
  // radius scales about each hip so femur AND knee reconfigure together while
  // the distal link keeps its calibrated ground orientation. Normalised to
  // exactly 1.0 at the neutral height so the documented home stance is exact.
  const float scale = stanceRadiusForHeight(body_height_mm_) /
                      stanceRadiusForHeight(kRcBodyHeightNeutralMm);
  x = kCoxaMountXy[leg][0] +
      (kHomeFootXy[leg][0] - kCoxaMountXy[leg][0]) * scale;
  y = kCoxaMountXy[leg][1] +
      (kHomeFootXy[leg][1] - kCoxaMountXy[leg][1]) * scale;
  z = -body_height_mm_;
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

  // Track the stick target with the critically-damped spring-damper. The
  // working twist therefore has continuous position and velocity: direction
  // and magnitude changes become S-curves instead of rate-limited ramps.
  const float omega =
      kMinTwistOmega + (kMaxTwistOmega - kMinTwistOmega) * speed_;
  trackAxis(twist_.vx, twist_vel_.vx, target_twist_.vx, omega, dt_s);
  trackAxis(twist_.vy, twist_vel_.vy, target_twist_.vy, omega, dt_s);
  trackAxis(twist_.wz, twist_vel_.wz, target_twist_.wz, omega, dt_s);

  // Park: once the target is neutral and the tracker tail has decayed inside
  // the park window, snap to exactly zero so the phase stops advancing and
  // the feet hold the planted home stance (no bobbing at centre sticks).
  const bool target_neutral = target_twist_.vx == 0.0f &&
                              target_twist_.vy == 0.0f &&
                              target_twist_.wz == 0.0f;
  if (target_neutral &&
      fabsf(twist_.vx) < kTwistParkPos && fabsf(twist_.vy) < kTwistParkPos &&
      fabsf(twist_.wz) < kTwistParkPos &&
      fabsf(twist_vel_.vx) < kTwistParkVel &&
      fabsf(twist_vel_.vy) < kTwistParkVel &&
      fabsf(twist_vel_.wz) < kTwistParkVel) {
    twist_.vx = twist_.vy = twist_.wz = 0.0f;
    twist_vel_.vx = twist_vel_.vy = twist_vel_.wz = 0.0f;
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

  // Advance cycle phase.
  const float freq = kMinFreqHz + (kMaxFreqHz - kMinFreqHz) * speed_;
  phase_ = frac01(phase_ + freq * dt_s);

  const float beta = dutyFactor();
  const float swing_span = (beta < 1.0f) ? (1.0f - beta) : 1.0f;

  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    float hx, hy, hz;
    homeFoot(leg, hx, hy, hz);

    // Commanded stroke vector for this leg (mm). Yaw adds a tangential
    // component perpendicular to the leg's home radial direction.
    const float r = sqrtf(hx * hx + hy * hy);
    float tang_x = 0.0f, tang_y = 0.0f;
    if (r > 1e-3f) {
      tang_x = -hy / r;
      tang_y = hx / r;
    }
    float sx = twist_.vx * stride_mm_ + twist_.wz * stride_mm_ * tang_x;
    float sy = twist_.vy * stride_mm_ + twist_.wz * stride_mm_ * tang_y;
    sx = clampf(sx, -kMaxStrideMm, kMaxStrideMm);
    sy = clampf(sy, -kMaxStrideMm, kMaxStrideMm);

    const float leg_phase = frac01(phase_ + legOffset(gait_, leg));
    bool swing;
    float L;     // longitudinal sweep in [-0.5, +0.5]
    float lift;  // swing Z lift (mm)
    if (leg_phase < beta) {
      // Stance: LINEAR sweep +0.5 -> -0.5. Constant push velocity is what
      // moves the body at constant speed; easing the stance would make the
      // whole body surge twice per cycle.
      const float s = (beta > 0.0f) ? (leg_phase / beta) : 0.0f;
      L = 0.5f - s;
      lift = 0.0f;
      swing = false;
    } else {
      // Swing: cubic Hermite return whose endpoint slopes MATCH the stance
      // sweep rate, so the foot's body-frame velocity is continuous through
      // liftoff and touchdown -- i.e. zero world-frame velocity at ground
      // contact (no scuff, no per-step jolt). The squared sine gives zero
      // vertical velocity at both ends; a lift floor keeps real clearance
      // even at small commands.
      const float u = (leg_phase - beta) / swing_span;
      const float m = -swing_span / beta;  // matched endpoint slope (in u)
      L = -0.5f + smoothstep(u) + m * u * (u - 1.0f) * (2.0f * u - 1.0f);
      const float lift_wave = sinf(kPi * u);
      const float lift_scale =
          command_scale > 0.25f ? 1.0f : command_scale * 4.0f;
      lift = step_mm_ * lift_scale * lift_wave * lift_wave;
      swing = true;
    }

    float fx = hx + sx * L;
    float fy = hy + sy * L;
    float fz = hz + lift;
    fz = clampf(fz, kMinFootZMm, kMaxFootZMm);

    out.feet[leg].x_mm = fx;
    out.feet[leg].y_mm = fy;
    out.feet[leg].z_mm = fz;
    out.feet[leg].swing = swing;
  }
}

}  // namespace gait
