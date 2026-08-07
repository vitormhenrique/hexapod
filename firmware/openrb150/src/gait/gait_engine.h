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
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"

namespace gait {

// Safety clamps for generated targets (mm). Derived from the measured CAD
// workspace (dimensions.md): centered servos put the foot 131.73 mm below the
// body mid-plane; the two-link annulus (66.51 + 117.16 mm at a 95% margin)
// allows foot depths to about -158 mm at the home radius.
constexpr float kMaxStrideMm = 80.0f;   // max per-axis foot stroke
constexpr float kMaxStepMm = 50.0f;     // max swing lift
constexpr float kMinFootZMm = -158.0f;  // lowest commanded foot Z in B
// The full 50 mm swing must remain available at the 65 mm crouch limit:
// -65 + 50 = -15 mm. The previous -40 mm cap silently shortened a 30 mm
// configured lift to 25 mm (and worse for larger lifts) as the body lowered.
constexpr float kMaxFootZMm = -15.0f;   // highest commanded foot Z in B
constexpr float kSitFootZMm = -60.0f;   // body-down sit pose

constexpr float kStrokeEnvelopeFrac = 0.5f;
// Largest body-centre-to-foot radius of the home stance (corner legs at
// sqrt(155.2^2 + 205.2^2) mm). Yaw strokes are sized against it so a full yaw
// command sweeps the same arc length a full translation command strides.
constexpr float kMaxLegRadiusMm = 257.3f;
constexpr float kMinStepPeriodMs = 60.0f;
constexpr float kNeutralStepPeriodMs = 80.0f;
constexpr float kMaxStepPeriodMs = 120.0f;
// BodyCommandShaper owns acceleration/deceleration. The gait engine receives
// an already-shaped twist and keeps only this small neutral threshold so
// imperceptible transport residue cannot make the feet bob in place.
constexpr float kTwistParkPos = 0.03f;
// First-order filter time constant for the live shape parameters (body
// height, stride, step height, speed). RC pot/ADC noise lands on the filter
// target, never directly on the servo goals.
constexpr float kParamFilterTau = 0.25f;  // seconds
// Ignore residual stick/transport noise around centre. A stepping gait with a
// neutral command must hold the planted home stance rather than bob in place.
constexpr float kMotionDeadband = 0.03f;
// Mark III APG tables own duty/support timing; duty_x255 remains a compatible
// wire input but does not alter those fixed safety patterns.

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

  void setGait(config::GaitId g);
  void setTwist(const BodyTwist& t);

  // Reset phase to 0.
  void reset();

  // Advance the cycle by dt_ms and fill `out` with bounded foot targets.
  void update(uint32_t dt_ms, GaitOutput& out);

  config::GaitId gait() const { return gait_; }
  float phase() const { return phase_; }
  float dutyFactor() const;
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

  config::GaitId gait_ = config::GaitId::Stand;
  float phase_ = 0.0f;
  float gait_step_phase_ = 0.0f;
  // Filtered live shape parameters (first-order lag toward the *_target_
  // values below, advanced in update()). Seeded from the first configure().
  float stride_mm_ = 60.0f;
  float step_mm_ = 30.0f;
  float body_height_mm_ = 132.0f;
  float speed_ = 0.5f;  // 0..1 normalised
  // Raw configure() targets for the filtered parameters.
  float stride_target_ = 60.0f;
  float step_target_ = 30.0f;
  float height_target_ = 132.0f;
  float speed_target_ = 0.5f;
  bool params_seeded_ = false;
  BodyTwist twist_;  // already-shaped command from ControllerCore
};

}  // namespace gait
