#pragma once

// ===========================================================================
// 3-DOF leg inverse kinematics (portable, no Arduino deps).
//
// Solves a single hexapod leg: given a foot target in the leg's COXA frame
// (X radially outward at home, Y tangential/left, Z up; origin on the hip-yaw
// axis), return the three URDF-zero-relative joint angles (coxa/femur/tibia, in
// radians, where 0 == the 180deg servo home pose) plus a reachability flag.
//
// Math follows HexNav_description/docs/inverse_kinematics.md section 8 (hip yaw
// + 2-link planar arm, law of cosines). The raw planar solution is offset so
// that the documented home foot (127 mm radial, -44.55 mm down -> section 5/12)
// maps to all-zero joint angles; this makes the output directly consumable by
// the servo map (dxl/servo_map.h: tick = 2048 + sign*deg(angle)).
//
// Deterministic, branch-stable ("knee-up" = +acos), no heap, float math. The
// two-link reachable annulus is |L2 - L3| <= d <= (L2 + L3); targets outside it
// are clamped (straight or fully folded) and reported unreachable so the caller
// (gait/safety) can react instead of commanding an impossible pose.
// ===========================================================================

#include <stdint.h>

namespace gait {

// Foot-tip offset baked into L_TIBIA points to the URDF foot frame; the home
// foot in the coxa frame (identical for all six legs, IK ref section 12).
constexpr float kHomeRadiusMm = 127.0f;
constexpr float kHomeFootZMm = -44.55f;

// Result of a single-leg IK solve. Angles are URDF-zero-relative radians.
struct IkResult {
  float coxa = 0.0f;
  float femur = 0.0f;
  float tibia = 0.0f;
  bool reachable = false;
};

class LegIk {
 public:
  // Link lengths in millimetres (L1 hip->femur radial offset, L2 femur, L3
  // tibia-to-foot). `home_radius_mm`/`home_foot_z_mm` are the neutral foot
  // position in the coxa frame; the rest offsets are computed from them so
  // solve() returns URDF-zero-relative angles (home -> 0). They default to the
  // documented reference stance and are overridden by the persisted robot
  // config (config::BodyGeometry) on real hardware.
  LegIk(float l1_mm, float l2_mm, float l3_mm,
        float home_radius_mm = kHomeRadiusMm,
        float home_foot_z_mm = kHomeFootZMm);

  // Solve for a foot target (mm) in the coxa frame. Always returns angles
  // (clamped/saturated when out of reach); `reachable` is false if the target
  // is outside the two-link annulus.
  IkResult solve(float x_mm, float y_mm, float z_mm) const;

  // Raw planar rest angles at the home foot (radians), exposed for tests/FK.
  float femurRest() const { return femur_rest_; }
  float tibiaRest() const { return tibia_rest_; }

  // Forward kinematics for verification: raw planar angles (NOT relative) ->
  // foot position in the coxa frame (mm).
  void forwardRaw(float coxa, float femur_raw, float tibia_raw, float& x_mm,
                  float& y_mm, float& z_mm) const;

 private:
  // Planar+yaw solve returning RAW angles (coxa, femur=alpha, tibia=beta).
  IkResult solveRaw(float x_mm, float y_mm, float z_mm) const;

  float l1_;
  float l2_;
  float l3_;
  float femur_rest_;  // raw femur angle at the home foot
  float tibia_rest_;  // raw tibia angle at the home foot
};

}  // namespace gait
