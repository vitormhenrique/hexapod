// Gait engine v1 (portable, host-tested). See gait_engine.h.
// Reference: HexNav_description/docs/inverse_kinematics.md section 10/13.

#include "gait_engine.h"

#include <math.h>

#include "leg_ik.h"  // kReachMarginFrac: same margin the pipeline clamps to

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
constexpr float kIkL3Mm = 24.86f;       // L_TIBIA (reference model, section 4)
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
  twist_.vx = clampf(t.vx, -1.0f, 1.0f);
  twist_.vy = clampf(t.vy, -1.0f, 1.0f);
  twist_.wz = clampf(t.wz, -1.0f, 1.0f);
  if (fabsf(twist_.vx) <= kMotionDeadband) twist_.vx = 0.0f;
  if (fabsf(twist_.vy) <= kMotionDeadband) twist_.vy = 0.0f;
  if (fabsf(twist_.wz) <= kMotionDeadband) twist_.wz = 0.0f;
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

void GaitEngine::baseHomeFoot(uint8_t leg, float& x, float& y,
                              float& z) const {
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

void GaitEngine::homeFoot(uint8_t leg, float& x, float& y, float& z) const {
  baseHomeFoot(leg, x, y, z);
  // Walking stance bias: while a stroke is commanded, slide the stance centre
  // toward this leg's own hip so the stroke envelope fits inside the reach
  // annulus instead of being collapsed by the pipeline's reach limiting.
  if (stance_bias_mm_ > 0.01f) {
    const float dx = x - kCoxaMountXy[leg][0];
    const float dy = y - kCoxaMountXy[leg][1];
    const float r = sqrtf(dx * dx + dy * dy);
    if (r > stance_bias_mm_ + 1.0f) {
      const float k = (r - stance_bias_mm_) / r;
      x = kCoxaMountXy[leg][0] + dx * k;
      y = kCoxaMountXy[leg][1] + dy * k;
    }
  }
}

float GaitEngine::strokeBiasTarget() const {
  if (twist_.vx == 0.0f && twist_.vy == 0.0f && twist_.wz == 0.0f) {
    return 0.0f;
  }
  // Reachable annulus radii at the current foot height, projected onto the
  // horizontal plane through each hip (reference-model geometry; the pipeline
  // re-checks against the live config, this bias just keeps it from having to
  // shrink anything in the nominal case). A small pad keeps the fitted stroke
  // extremes strictly inside the margin despite filter lag and float noise.
  constexpr float kBiasPadMm = 1.0f;
  const float zc = -body_height_mm_ - kCoxaZOffMm;  // foot z in coxa frame
  const float d_max = kReachMarginFrac * (kIkL2Mm + kIkL3Mm);
  const float outer_sq = d_max * d_max - zc * zc;
  if (outer_sq <= 0.0f) return 0.0f;  // height alone exceeds reach: no help
  const float r_outer = kIkL1Mm + sqrtf(outer_sq) - kBiasPadMm;
  const float d_min = fabsf(kIkL2Mm - kIkL3Mm) +
                      (1.0f - kReachMarginFrac) * (kIkL2Mm + kIkL3Mm);
  const float inner_sq = d_min * d_min - zc * zc;
  const float r_inner =
      kIkL1Mm + (inner_sq > 0.0f ? sqrtf(inner_sq) : 0.0f) + kBiasPadMm;

  float need = 0.0f;               // bias required by the outer boundary
  float allow = kMaxStanceBiasMm;  // bias allowed by the inner boundary
  const float beta = dutyFactor();
  const float swing_span = (beta < 1.0f) ? (1.0f - beta) : 1.0f;
  const float m = (beta > 0.0f) ? -swing_span / beta : 0.0f;
  // Swing-path samples for the inner bound: lifting the foot shrinks |z| in
  // the coxa frame, pulling d toward the folded boundary exactly where the
  // swing also sweeps inward, so the stance/lift-zero extremes alone are not
  // the binding inner constraint.
  constexpr float kSwingU[] = {0.2f, 0.35f, 0.5f, 0.65f, 0.8f};
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    float hx, hy, hz;
    baseHomeFoot(leg, hx, hy, hz);
    float sx, sy, lift_scale;
    strokeForLeg(leg, sx, sy, lift_scale);
    // Hip -> stance-centre unit direction (the bias axis for this leg).
    const float cx = hx - kCoxaMountXy[leg][0];
    const float cy = hy - kCoxaMountXy[leg][1];
    const float cr = sqrtf(cx * cx + cy * cy);
    if (cr < 1.0f) continue;
    const float ux = cx / cr;
    const float uy = cy / cr;
    for (int8_t sign = -1; sign <= 1; sign += 2) {
      // Stroke extreme relative to the hip, before any bias.
      const float px = cx + static_cast<float>(sign) * kStrokeEnvelopeFrac * sx;
      const float py = cy + static_cast<float>(sign) * kStrokeEnvelopeFrac * sy;
      const float along = px * ux + py * uy;   // component on the bias axis
      const float perp_sq = px * px + py * py - along * along;
      // Outer boundary: smallest b with |p - b*u| <= r_outer (exact solve;
      // sliding the centre inward moves the extreme along -u).
      const float outer_rem = r_outer * r_outer - perp_sq;
      if (outer_rem <= 0.0f) {
        // No inward shift can bring this extreme inside; take the cap and let
        // the pipeline's reach limiting absorb the (shorter) remainder.
        need = kMaxStanceBiasMm;
      } else {
        const float b = along - sqrtf(outer_rem);
        if (b > need) need = b;
      }
      // Inner boundary at ground level: largest b keeping |p-b*u| >= r_inner.
      const float inner_rem = r_inner * r_inner - perp_sq;
      if (inner_rem > 0.0f) {
        const float b_in = along - sqrtf(inner_rem);
        if (b_in < allow) allow = b_in;
      }
    }
    // Inner boundary along the lifted swing return.
    for (const float u : kSwingU) {
      const float ss = u * u * (3.0f - 2.0f * u);  // smoothstep
      const float L = -0.5f + ss + m * u * (u - 1.0f) * (2.0f * u - 1.0f);
      const float lift_wave = sinf(kPi * u);
      float zc_u = zc + step_mm_ * lift_scale * lift_wave * lift_wave;
      const float zc_cap = kMaxFootZMm - kCoxaZOffMm;  // foot Z clamp in coxa
      if (zc_u > zc_cap) zc_u = zc_cap;
      const float in_sq = d_min * d_min - zc_u * zc_u;
      const float r_in_u =
          kIkL1Mm + (in_sq > 0.0f ? sqrtf(in_sq) : 0.0f) + kBiasPadMm;
      const float px = cx + L * sx;
      const float py = cy + L * sy;
      const float along = px * ux + py * uy;
      const float perp_sq = px * px + py * py - along * along;
      const float inner_rem = r_in_u * r_in_u - perp_sq;
      if (inner_rem > 0.0f) {
        const float b_in = along - sqrtf(inner_rem);
        if (b_in < allow) allow = b_in;
      }
    }
  }
  if (allow < 0.0f) allow = 0.0f;
  return clampf(need, 0.0f,
                allow < kMaxStanceBiasMm ? allow : kMaxStanceBiasMm);
}

void GaitEngine::strokeForLeg(uint8_t leg, float& x, float& y,
                              float& lift_scale) const {
  float hx, hy, hz;
  homeFoot(leg, hx, hy, hz);
  const float radius = sqrtf(hx * hx + hy * hy);
  float tangent_x = 0.0f;
  float tangent_y = 0.0f;
  if (radius > 1e-3f) {
    tangent_x = -hy / radius;
    tangent_y = hx / radius;
  }
  x = clampf(twist_.vx * stride_mm_ +
                 twist_.wz * stride_mm_ * tangent_x,
             -kMaxStrideMm, kMaxStrideMm);
  y = clampf(twist_.vy * stride_mm_ +
                 twist_.wz * stride_mm_ * tangent_y,
             -kMaxStrideMm, kMaxStrideMm);
  const float command_scale =
      fmaxf(fabsf(twist_.vx), fmaxf(fabsf(twist_.vy), fabsf(twist_.wz)));
  lift_scale = command_scale > 0.25f ? 1.0f : command_scale * 4.0f;
}

void GaitEngine::motionEnvelopeFoot(uint8_t leg, float longitudinal,
                                    float lift_fraction, float& x, float& y,
                                    float& z) const {
  float home_x, home_y, home_z;
  homeFoot(leg, home_x, home_y, home_z);
  float stroke_x, stroke_y, lift_scale;
  strokeForLeg(leg, stroke_x, stroke_y, lift_scale);
  x = home_x + stroke_x * longitudinal;
  y = home_y + stroke_y * longitudinal;
  z = home_z + step_mm_ * lift_scale * clampf(lift_fraction, 0.0f, 1.0f);
}

void GaitEngine::update(uint32_t dt_ms, GaitOutput& out) {
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

  // Advance the shape-parameter filters (body height, stride, step, speed).
  const float pk = (dt_s >= kParamFilterTau) ? 1.0f : dt_s / kParamFilterTau;
  stride_mm_ += (stride_target_ - stride_mm_) * pk;
  step_mm_ += (step_target_ - step_mm_) * pk;
  body_height_mm_ += (height_target_ - body_height_mm_) * pk;
  speed_ += (speed_target_ - speed_) * pk;

  // Advance the walking stance bias toward what the current stroke envelope
  // needs. Static poses always relax back to the documented home stance.
  const bool stepping =
      gait_ != config::GaitId::Stand && gait_ != config::GaitId::Sit;
  const float bias_target = stepping ? strokeBiasTarget() : 0.0f;
  stance_bias_mm_ += (bias_target - stance_bias_mm_) * pk;

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
    float sx, sy, lift_scale;
    strokeForLeg(leg, sx, sy, lift_scale);

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
