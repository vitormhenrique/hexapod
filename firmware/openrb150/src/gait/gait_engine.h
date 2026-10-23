#pragma once

// ===========================================================================
// Mark III Phoenix gait engine (portable, no Arduino deps).
//
// Generates bounded foot targets in mechanical body frame B (X right, Y
// forward, Z up) for the standard hexapod gaits. The output feeds the body
// transform + leg IK (gait/body_ik.h) which produces joint angles for the
// servo map.
//
// Ripple, tripod, and wave use the exact Mark III APG step counts and leg
// origins. Linear interpolation between adjacent Phoenix keyframes reproduces
// the reference servo move while allowing the 100 Hz control loop to remain
// smooth. Translation follows the reference longitudinal coefficients and yaw
// rotates the complete body-centre-to-foot radius.
//
// Stand and Sit are static poses (no stepping): Stand holds the home stance,
// Sit holds the feet at home XY with the body lowered. Deterministic, static
// memory, float math, safe to run at >= 50 Hz.
//
// The operator-facing *shape* of the walk (stroke, clearance, yaw stroke,
// stance radius, cadence endpoints, swing mid-height) comes from a GaitShape
// (gait_shape.h). Everything positional is first-order filtered so a value
// arriving from the handset can never jump the feet.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "gait_shape.h"

namespace gait {

// Safety clamps for generated targets (mm). Derived from the measured CAD
// workspace (dimensions.md): centered servos put the foot 131.73 mm below the
// body mid-plane; the two-link annulus (66.51 + 117.16 mm at a 95% margin)
// allows foot depths to about -158 mm at the home radius. Stride / lift maxima
// mirror config::kMaxGaitStrideMm / kMaxGaitStepMm and gait_shape.h; the
// reach limiter in the pipeline, not these caps, keeps a large stroke inside
// the leg workspace at a given ride height.
constexpr float kMaxStrideMm = 140.0f;  // max per-axis foot stroke
constexpr float kMaxStepMm = 80.0f;     // max swing lift
constexpr float kMinFootZMm = -158.0f;  // lowest commanded foot Z in B
// The full swing must remain available at the 65 mm crouch limit:
// -65 + 50 = -15 mm. The previous -40 mm cap silently shortened a 30 mm
// configured lift to 25 mm (and worse for larger lifts) as the body lowered.
constexpr float kMaxFootZMm = -15.0f;   // highest commanded foot Z in B
constexpr float kSitFootZMm = -60.0f;   // body-down sit pose

constexpr float kStrokeEnvelopeFrac = 0.5f;
// Largest body-centre-to-foot radius of the measured home stance (corner legs
// at sqrt(155.2^2 + 205.2^2) mm). Kept for callers that size envelopes
// against the nominal stance; the yaw stroke itself is now an explicit
// GaitShape parameter (degrees per cycle) rather than derived from stride.
constexpr float kMaxLegRadiusMm = 257.3f;
// Compiled cadence endpoints (also GaitShape defaults): Mark III step period
// at speed 1 / speed 0, and the neutral period reached at speed 0.5.
constexpr float kMinStepPeriodMs = 60.0f;
constexpr float kNeutralStepPeriodMs = 80.0f;
constexpr float kMaxStepPeriodMs = 120.0f;
// BodyCommandShaper owns acceleration/deceleration. The gait engine receives
// an already-shaped twist and keeps only this small neutral threshold so
// imperceptible transport residue cannot make the feet bob in place.
constexpr float kTwistParkPos = 0.03f;
// First-order filter time constant for the live shape parameters (body
// height, stride, step height, speed, stance radius, yaw stroke, half lift).
// RC pot/ADC noise and handset edits land on the filter target, never
// directly on the servo goals.
constexpr float kParamFilterTau = 0.25f;  // seconds
// Ignore residual stick/transport noise around centre. A stepping gait with a
// neutral command must hold the planted home stance rather than bob in place.
constexpr float kMotionDeadband = 0.03f;
// Mark III APG tables own duty/support timing; duty_x255 remains a compatible
// wire input but does not alter those fixed safety patterns.

// The home (anchor) foot must itself sit inside the leg workspace, otherwise
// the pipeline's reach limiter has nothing to scale from. The commanded
// stance radius is capped so the home foot never exceeds this fraction of the
// IK reach margin at the current ride height (see
// GaitEngine::maxStanceRadiusMm). 0.98 leaves the measured default stance
// untouched across the whole Pot2 height envelope (home d = 167.7 mm of the
// 174.5 mm margin at 150 mm) and only bites for operator radii beyond it.
constexpr float kHomeReachFrac = 0.98f;
// Leg IK reach margin (mirrors leg_ik.h kReachMarginFrac without a dependency).
constexpr float kIkReachMarginFrac = 0.95f;

// Reach-safe ride-height envelope around the measured 131.73 mm centered-servo
// stance (dimensions.md). At the home radius the leg keeps a comfortable
// workspace margin across this whole range.
//
// The bottom of the range is the measured full-crouch pose: with the stance
// X/Y held at home, 65 mm puts the femur at about +60 deg and the tibia at
// about -50 deg from their URDF-zero rest (Joint Matrix "240 deg" / "128 deg"),
// which is the belly-near-the-ground pose. That is deliberately much further
// from neutral than the 18 mm of lift above it, so Pot2's lower half buys real
// crouch travel while its upper half stays a fine trim.
constexpr float kRcBodyHeightMinMm = 65.0f;
constexpr float kRcBodyHeightNeutralMm = 132.0f;
constexpr float kRcBodyHeightMaxMm = 150.0f;

// Map the Pot2 0..1 fraction onto the reach-safe height envelope, piecewise
// linear about the neutral centre.
inline float rcBodyHeightMm(float frac) {
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  return frac < 0.5f
             ? kRcBodyHeightNeutralMm -
                   (0.5f - frac) * 2.0f *
                       (kRcBodyHeightNeutralMm - kRcBodyHeightMinMm)
             : kRcBodyHeightNeutralMm +
                   (frac - 0.5f) * 2.0f *
                       (kRcBodyHeightMaxMm - kRcBodyHeightNeutralMm);
}

// Normalised body twist command. Each component is clamped to [-1, 1].
struct BodyTwist {
  float vx = 0.0f;  // body right (+) / left (-)
  float vy = 0.0f;  // body forward (+) / backward (-)
  float wz = 0.0f;  // yaw CCW (+) / CW (-)
};

// One foot target in body frame B (mm) plus its swing/stance state.
struct FootTarget {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float z_mm = 0.0f;
  bool swing = false;
};

struct GaitOutput {
  FootTarget feet[config::kNumLegs];
};

class GaitEngine {
 public:
  GaitEngine();

  // Apply gait defaults from config (gait id, stride, step height, speed, body
  // height). Stride and step height are clamped to safe maxima.
  void configure(const config::GaitDefaults& d);

  // Adopt the leg placement + link lengths the home stance is built from. The
  // measured CAD table is the compiled fallback, so an engine that is never
  // given a config still produces the documented stance.
  void setLegGeometry(const config::RobotConfig& cfg);

  // Apply the live shape tunables (yaw stroke, stance radius, cadence,
  // half-lift). Stride / step height are taken from configure(); the matching
  // GaitShape slots are ignored here so one path owns each number.
  void setShape(const GaitShape& shape);

  void setGait(config::GaitId g);
  void setTwist(const BodyTwist& t);

  // Reset phase to 0.
  void reset();

  // Advance the cycle by dt_ms and fill `out` with bounded foot targets.
  void update(uint32_t dt_ms, GaitOutput& out);

  config::GaitId gait() const { return gait_; }
  float phase() const { return phase_; }
  float dutyFactor() const;
  // Stance radius actually in use after the workspace cap (mm, filtered).
  float stanceRadiusMm() const { return stance_radius_mm_; }
  void nominalFoot(uint8_t leg, float& x, float& y, float& z) const {
    homeFoot(leg, x, y, z);
  }
  void motionEnvelopeFoot(uint8_t leg, float longitudinal,
                          float lift_fraction, float& x, float& y,
                          float& z) const;

  // Single-leg tuning preview (RC gait-tune editor). Traces the configured
  // swing arc for `leg` at cycle `phase` in [0, 1): the first half swings the
  // foot forward through the full step height, the second half returns it along
  // the ground. Uses only the filtered stride / step-height / body-height
  // parameters and ignores the twist command, so it demonstrates exactly what a
  // parameter change does while the robot stands still. Const and phase-free:
  // it never touches the gait cycle.
  void previewFoot(uint8_t leg, float phase, float& x, float& y,
                   float& z) const;

 private:
  void baseHomeFoot(uint8_t leg, float& x, float& y, float& z) const;
  void homeFoot(uint8_t leg, float& x, float& y, float& z) const;
  void footAt(uint8_t leg, float longitudinal, float lift_fraction,
              float& x, float& y, float& z) const;
  float stepPeriodMs() const;
  // Largest stance radius whose home foot stays inside kHomeReachFrac of the
  // IK reach margin at `body_height_mm`.
  float maxStanceRadiusMm(float body_height_mm) const;

  config::GaitId gait_ = config::GaitId::Stand;
  float phase_ = 0.0f;
  float gait_step_phase_ = 0.0f;
  // Filtered live shape parameters (first-order lag toward the *_target_
  // values below, advanced in update()). Seeded from the first configure().
  float stride_mm_ = 60.0f;
  float step_mm_ = 30.0f;
  float body_height_mm_ = 132.0f;
  float speed_ = 0.5f;  // 0..1 normalised
  float stance_radius_mm_ = 126.75f;
  float yaw_stroke_rad_ = 0.349066f;  // 20 deg
  float half_lift_ = 0.5f;
  // Raw configure()/setShape() targets for the filtered parameters.
  float stride_target_ = 60.0f;
  float step_target_ = 30.0f;
  float height_target_ = 132.0f;
  float speed_target_ = 0.5f;
  float stance_radius_target_ = 126.75f;
  float yaw_stroke_target_rad_ = 0.349066f;
  float half_lift_target_ = 0.5f;
  // Cadence endpoints (ms per Mark III step), applied directly.
  uint16_t step_period_fast_ms_ = static_cast<uint16_t>(kMinStepPeriodMs);
  uint16_t step_period_slow_ms_ = static_cast<uint16_t>(kMaxStepPeriodMs);
  bool params_seeded_ = false;
  // Per-leg home geometry: coxa mount in B (0.1 mm) and home azimuth as Q15
  // cos/sin, plus the link lengths the stance-radius cap is computed from.
  // Kept as 16-bit fixed point: this engine lives in SAMD21 .bss, where every
  // 32-bit float per leg is paid six times.
  int16_t mount_x_dmm_[config::kNumLegs];
  int16_t mount_y_dmm_[config::kNumLegs];
  int16_t azimuth_cos_q15_[config::kNumLegs];
  int16_t azimuth_sin_q15_[config::kNumLegs];
  float l1_mm_ = 52.0f;
  float l23_mm_ = 66.51f + 117.16f;
  BodyTwist twist_;  // already-shaped command from ControllerCore
};

}  // namespace gait
