#pragma once

// ===========================================================================
// Gait engine v1 (portable, no Arduino deps).
//
// Generates bounded foot targets in the body-centred frame B (REP-103: X
// forward, Y left, Z up; see HexNav_description/docs/inverse_kinematics.md
// section 10) for the standard hexapod gaits. The output feeds the body
// transform + leg IK (gait/body_ik.h) which produces joint angles for the
// servo map.
//
// Cycle model: a single normalised phase in [0,1) advances each tick at a
// frequency derived from the speed knob. Each leg has a fixed phase offset and
// the gait defines a stance duty factor beta:
//
//   leg_phase = frac(phase + offset[leg])
//   stance if leg_phase < beta      stance s = leg_phase / beta
//   swing  otherwise                swing  u = (leg_phase - beta)/(1 - beta)
//
// A longitudinal parameter L in [-0.5, +0.5] sweeps the foot along the
// commanded stroke (stance pushes the body forward; swing returns the foot with
// a sinusoidal lift). The stroke vector combines body twist (forward/lateral)
// and yaw (tangential to each leg's radius). All magnitudes are clamped so foot
// targets stay inside a safe, IK-reachable box -- the engine never emits an
// unbounded or runaway target.
//
// Stand and Sit are static poses (no stepping): Stand holds the home stance,
// Sit holds the feet at home XY with the body lowered. Deterministic, static
// memory, float math, safe to run at >= 50 Hz.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"

namespace gait {

// Safety clamps for generated targets (mm). Keep well inside the leg workspace.
constexpr float kMaxStrideMm = 80.0f;   // max per-axis foot stroke
constexpr float kMaxStepMm = 50.0f;     // max swing lift
constexpr float kMinFootZMm = -120.0f;  // lowest commanded foot Z in B
constexpr float kMaxFootZMm = -5.0f;    // highest commanded foot Z in B
constexpr float kSitFootZMm = -8.0f;    // body-down sit pose

// Horizontal stroke envelope fraction: the largest |L| the swing-return
// profile produces (slope-matched Hermite overshoot past +/-0.5). Shared with
// GaitPipeline's reach-envelope check so both layers bound the same extremes.
constexpr float kStrokeEnvelopeFrac = 0.56f;

// Maximum inward walking stance bias (mm). The documented home stance stands
// at ~92% leg extension -- only a few mm inside the IK reach margin -- so any
// real stride pushes the outward half of the stroke off the workspace and the
// pipeline's reach limiting used to collapse the whole gait into a shuffle.
// While a twist is commanded the engine slides each stance centre toward its
// own hip just far enough for the stroke envelope to fit the reachable
// annulus. The cap keeps the inward extreme clear of the folded (inner)
// boundary and bounds worst-case deviation from the documented home.
constexpr float kMaxStanceBiasMm = 25.0f;

// Cycle frequency range mapped from the speed knob (0..255).
constexpr float kMinFreqHz = 0.25f;
constexpr float kMaxFreqHz = 1.20f;
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
// Keep a non-zero swing interval even when the host requests 100% stance.
constexpr float kMaxDutyFactor = 0.95f;

// RC body-height envelope for the Pot2 knob. Pot CENTRE is the neutral stance
// height, turning down lowers and turning up raises the body. Height rides
// the constant-tibia-orientation locus (gait_engine.cpp homeFoot): the stance
// radius adjusts with height so femur and knee reconfigure together while the
// distal link keeps its calibrated ground orientation. Both ends stay inside
// the leg-reach annulus, so height commands never engage the reach clamp.
constexpr float kRcBodyHeightMinMm = 31.0f;
constexpr float kRcBodyHeightNeutralMm = 40.0f;
constexpr float kRcBodyHeightMaxMm = 55.0f;

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
  float vx = 0.0f;  // forward (+) / backward (-)
  float vy = 0.0f;  // left (+) / right (-)
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

 private:
  // Home stance before the walking bias (height locus only).
  void baseHomeFoot(uint8_t leg, float& x, float& y, float& z) const;
  void homeFoot(uint8_t leg, float& x, float& y, float& z) const;
  void strokeForLeg(uint8_t leg, float& x, float& y,
                    float& lift_scale) const;
  // Inward stance bias (mm) needed for the current stroke envelope to fit the
  // reference-model reach annulus. 0 when no twist is commanded.
  float strokeBiasTarget() const;

  config::GaitId gait_ = config::GaitId::Stand;
  float phase_ = 0.0f;
  // Filtered inward walking stance bias (mm), toward each leg's own hip.
  float stance_bias_mm_ = 0.0f;
  // Filtered live shape parameters (first-order lag toward the *_target_
  // values below, advanced in update()). Seeded from the first configure().
  float stride_mm_ = 60.0f;
  float step_mm_ = 30.0f;
  float body_height_mm_ = 40.0f;
  float requested_duty_ = 0.5f;
  float speed_ = 0.5f;  // 0..1 normalised
  // Raw configure() targets for the filtered parameters.
  float stride_target_ = 60.0f;
  float step_target_ = 30.0f;
  float height_target_ = 40.0f;
  float speed_target_ = 0.5f;
  bool params_seeded_ = false;
  BodyTwist twist_;  // already-shaped command from ControllerCore
};

}  // namespace gait
