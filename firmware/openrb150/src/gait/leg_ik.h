#pragma once

// ===========================================================================
// 3-DOF leg inverse kinematics (portable, no Arduino deps).
//
// Solves a single hexapod leg: given a foot target in the leg's COXA frame
// (X radially outward at home, Y tangential/left, Z up; origin on the hip-yaw
// axis), return the three URDF-zero-relative joint angles (coxa/femur/tibia, in
// radians, where 0 == the 180deg servo home pose) plus a reachability flag.
//
// Math is a hip yaw + 2-link planar arm (law of cosines). Link lengths and the
// home stance come from the measured CAD (dimensions.md, the dimension source
// of truth): L1 = 52.00 mm hip-yaw axis -> femur axis (femur axis at the same
// height as the coxa axis), L2 = 66.51 mm femur axis -> tibia axis, L3 =
// 117.16 mm tibia (knee) axis -> foot tip. The raw planar solution is offset
// so the measured centered-servo foot (126.75 mm radial, 131.73 mm down) maps
// to all-zero joint angles; this makes the output directly consumable by the
// servo map (dxl/servo_map.h: tick = 2048 + sign*deg(angle)).
//
// Deterministic, branch-stable knee: beta = -acos(...) selects the knee-out /
// negative branch, which is the branch the physical HexNav leg rests on (at
// the centered pose the measured knee interior angle is -72.1 deg). No heap,
// float math. The two-link reachable annulus is |L2 - L3| <= d <= (L2 + L3);
// targets outside it are clamped (straight or fully folded) and reported
// unreachable so the caller (gait/safety) can react instead of commanding an
// impossible pose.
// ===========================================================================

#include <stdint.h>

namespace gait {

// Measured centered-servo foot TIP in the coxa frame (identical for all six
// legs; dimensions.md). The coxa frame origin sits on the hip-yaw axis at the
// body mid-plane (z = 0 in body frame B), so the tip is 131.73 mm below it.
constexpr float kHomeRadiusMm = 126.75f;
constexpr float kHomeFootZMm = -131.73f;

// Reachability-aware stride limit (lmt.14): generated foot targets are pulled
// radially inward by clampToReach() so the planar reach distance never exceeds
// this fraction of the full two-link extension (l2 + l3). HexNav home is near
// full extension, so retain a conservative 5% workspace margin.
constexpr float kReachMarginFrac = 0.95f;

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

  // Reachability-aware limiter (lmt.14): clamp a coxa-frame foot target into a
  // safe annulus, clear of both full extension and the fully-folded boundary.
  // Foot height and hip-yaw direction are preserved when possible, so a stance
  // foot stays on its ground plane. Returns true if the target was modified.
  bool clampToReach(float& x_mm, float& y_mm, float& z_mm) const;

  // True when a target is already inside the same conservative annulus used
  // by clampToReach(). This lets the body-level pipeline shorten a Cartesian
  // stroke along its original path instead of radially redirecting it.
  bool withinReachMargin(float x_mm, float y_mm, float z_mm) const;

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
