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

// Cycle frequency range mapped from the speed knob (0..255).
constexpr float kMinFreqHz = 0.25f;
constexpr float kMaxFreqHz = 1.20f;
// Pot1 speed also controls how quickly the filtered body twist follows the
// sticks. Full-scale response ranges from ~1.3 s to ~0.33 s.
constexpr float kMinTwistSlewPerSec = 0.75f;
constexpr float kMaxTwistSlewPerSec = 3.0f;
// Ignore residual stick/transport noise around centre. A stepping gait with a
// neutral command must hold the planted home stance rather than bob in place.
constexpr float kMotionDeadband = 0.03f;
// Keep a non-zero swing interval even when the host requests 100% stance.
constexpr float kMaxDutyFactor = 0.95f;

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

 private:
  void homeFoot(uint8_t leg, float& x, float& y, float& z) const;

  config::GaitId gait_ = config::GaitId::Stand;
  float phase_ = 0.0f;
  float stride_mm_ = 60.0f;
  float step_mm_ = 30.0f;
  float body_height_mm_ = 40.0f;
  float requested_duty_ = 0.5f;
  float speed_ = 0.5f;  // 0..1 normalised
  BodyTwist target_twist_;
  BodyTwist twist_;  // slew-limited command used to generate foot targets
};

}  // namespace gait
