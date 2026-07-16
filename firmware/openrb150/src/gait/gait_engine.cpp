// Gait engine v1 (portable, host-tested). See gait_engine.h.
// Reference: HexNav_description/docs/inverse_kinematics.md section 10/13.

#include "gait_engine.h"

#include <math.h>

namespace gait {
namespace {

constexpr float kPi = 3.14159265358979323846f;
// Finish horizontal placement before touchdown so the final swing segment is
// a vertical descent. This reduces scuffing and gives the calibrated tibia
// home geometry the best chance to meet the floor near-perpendicularly.
constexpr float kSwingPlacementFraction = 0.70f;

// Home foot XY in body frame B (mm), IK ref section 13 HOME_FOOT. The Z comes
// from the configured body height.
constexpr float kHomeFootXy[config::kNumLegs][2] = {
    {-155.4f, -205.4f}, {155.4f, -205.4f}, {196.8f, 0.0f},
    {155.4f, 205.4f},   {-155.4f, 205.4f}, {-196.8f, 0.0f},
};

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
      // Tripod A {1,3,4}=legs{0,2,3} at 0.0; Tripod B {2,5,6}=legs{1,4,5} at 0.5
      static const float kOff[config::kNumLegs] = {0.0f, 0.5f, 0.0f,
                                                   0.0f, 0.5f, 0.5f};
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

inline float slewToward(float current, float target, float max_delta) {
  const float delta = target - current;
  if (delta > max_delta) return current + max_delta;
  if (delta < -max_delta) return current - max_delta;
  return target;
}

}  // namespace

GaitEngine::GaitEngine() {}

void GaitEngine::configure(const config::GaitDefaults& d) {
  gait_ = static_cast<config::GaitId>(d.gait);
  stride_mm_ = clampf(static_cast<float>(d.stride_len_mm), 0.0f, kMaxStrideMm);
  step_mm_ = clampf(static_cast<float>(d.step_height_mm), 0.0f, kMaxStepMm);
  body_height_mm_ = static_cast<float>(d.body_height_mm);
  requested_duty_ = clampf(static_cast<float>(d.duty_x255) / 255.0f, 0.0f,
                           kMaxDutyFactor);
  speed_ = clampf(static_cast<float>(d.speed_x255) / 255.0f, 0.0f, 1.0f);
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
  x = kHomeFootXy[leg][0];
  y = kHomeFootXy[leg][1];
  z = -body_height_mm_;
}

void GaitEngine::update(uint32_t dt_ms, GaitOutput& out) {
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

  // Slew the working twist toward the latest stick/host target. Pot1 controls
  // this response together with cadence: low speed is deliberately gentle;
  // high speed remains responsive. Returning to centre uses a modestly faster
  // rate so the robot settles promptly without an abrupt target jump.
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
  const bool target_neutral = target_twist_.vx == 0.0f &&
                              target_twist_.vy == 0.0f &&
                              target_twist_.wz == 0.0f;
  float slew_rate = kMinTwistSlewPerSec +
                    (kMaxTwistSlewPerSec - kMinTwistSlewPerSec) * speed_;
  if (target_neutral) slew_rate *= 1.5f;
  const float max_delta = slew_rate * dt_s;
  twist_.vx = slewToward(twist_.vx, target_twist_.vx, max_delta);
  twist_.vy = slewToward(twist_.vy, target_twist_.vy, max_delta);
  twist_.wz = slewToward(twist_.wz, target_twist_.wz, max_delta);

  // A selected walking gait is still a planted stance when both the command
  // and its filtered tail are centred. Phase stays parked, so RC jitter cannot
  // make the robot cycle its feet in place after arming.
  const float command_scale =
      fmaxf(fabsf(twist_.vx), fmaxf(fabsf(twist_.vy), fabsf(twist_.wz)));
  const bool motion_commanded = command_scale > 0.0f;
  if (!motion_commanded) {
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
      // Stance: foot moves +0.5 -> -0.5 (body pushed forward).
      const float s = (beta > 0.0f) ? (leg_phase / beta) : 0.0f;
      L = 0.5f - smoothstep(s);
      lift = 0.0f;
      swing = false;
    } else {
      // Swing: zero-velocity easing avoids horizontal reversals and a squared
      // sine gives zero vertical velocity at liftoff and touchdown. Horizontal
      // placement finishes early, leaving a vertical final approach.
      const float u = (leg_phase - beta) / swing_span;
      const float placement_u =
          clampf(u / kSwingPlacementFraction, 0.0f, 1.0f);
      L = -0.5f + smoothstep(placement_u);
      const float lift_wave = sinf(kPi * u);
      lift = step_mm_ * command_scale * lift_wave * lift_wave;
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
