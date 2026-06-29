// 3-DOF leg inverse kinematics (portable, host-tested).
// See leg_ik.h. Math from HexNav_description/docs/inverse_kinematics.md section 8.

#include "leg_ik.h"

#include <math.h>

namespace gait {
namespace {

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

LegIk::LegIk(float l1_mm, float l2_mm, float l3_mm, float home_radius_mm,
             float home_foot_z_mm)
    : l1_(l1_mm), l2_(l2_mm), l3_(l3_mm), femur_rest_(0.0f), tibia_rest_(0.0f) {
  // Compute the raw planar angles at the configured home foot so solve() can
  // report URDF-zero-relative angles (home -> 0,0,0).
  const IkResult home = solveRaw(home_radius_mm, 0.0f, home_foot_z_mm);
  femur_rest_ = home.femur;
  tibia_rest_ = home.tibia;
}

IkResult LegIk::solveRaw(float x_mm, float y_mm, float z_mm) const {
  IkResult r;

  // Step 1: hip yaw.
  r.coxa = atan2f(y_mm, x_mm);

  // Step 2: reduce to the leg's vertical plane.
  const float horiz = sqrtf(x_mm * x_mm + y_mm * y_mm);
  const float planar_r = horiz - l1_;        // radial distance past the coxa
  const float dz = z_mm;                      // vertical
  const float d = sqrtf(planar_r * planar_r + dz * dz);

  // Step 3: two-link planar IK (law of cosines). Reachable annulus, with a
  // small epsilon so exact-boundary targets count as reachable despite float
  // rounding.
  constexpr float kReachEpsMm = 1e-2f;
  const float reach_max = l2_ + l3_;
  const float reach_min = fabsf(l2_ - l3_);
  r.reachable = (d <= reach_max + kReachEpsMm) && (d >= reach_min - kReachEpsMm);

  float cos_k = (d * d - l2_ * l2_ - l3_ * l3_) / (2.0f * l2_ * l3_);
  cos_k = clampf(cos_k, -1.0f, 1.0f);
  const float beta = acosf(cos_k);  // knee interior angle (knee-up branch)

  // Step 4: femur angle.
  const float a = atan2f(dz, planar_r);
  const float b = atan2f(l3_ * sinf(beta), l2_ + l3_ * cosf(beta));
  r.femur = a - b;
  r.tibia = beta;
  return r;
}

IkResult LegIk::solve(float x_mm, float y_mm, float z_mm) const {
  IkResult r = solveRaw(x_mm, y_mm, z_mm);
  // Offset to URDF-zero-relative (home foot -> 0).
  r.femur -= femur_rest_;
  r.tibia -= tibia_rest_;
  return r;
}

bool LegIk::clampToReach(float& x_mm, float& y_mm, float& z_mm) const {
  const float horiz = sqrtf(x_mm * x_mm + y_mm * y_mm);
  const float planar_r = horiz - l1_;  // radial distance past the coxa
  const float dz = z_mm;
  const float d = sqrtf(planar_r * planar_r + dz * dz);
  const float d_max = kReachMarginFrac * (l2_ + l3_);
  if (d <= d_max) {
    return false;  // already inside the safe reach margin
  }

  // Prefer to keep the foot height (dz) fixed and shorten only the radial reach,
  // so a stance foot stays on its ground plane while the stride is bounded. If
  // the foot is lower than the whole reach margin (no radial solution at that
  // height), fall back to scaling both components toward the reach centre.
  float planar_r_new;
  float dz_new = dz;
  if (fabsf(dz) < d_max) {
    planar_r_new = sqrtf(d_max * d_max - dz * dz);
    if (planar_r < 0.0f) planar_r_new = -planar_r_new;  // preserve radial sign
  } else {
    const float s = d_max / d;
    planar_r_new = planar_r * s;
    dz_new = dz * s;
  }

  const float horiz_new = planar_r_new + l1_;
  if (horiz > 1e-6f) {
    const float k = horiz_new / horiz;  // preserves hip-yaw (atan2 unchanged)
    x_mm *= k;
    y_mm *= k;
  } else {
    x_mm = horiz_new;
    y_mm = 0.0f;
  }
  z_mm = dz_new;
  return true;
}

void LegIk::forwardRaw(float coxa, float femur_raw, float tibia_raw,
                       float& x_mm, float& y_mm, float& z_mm) const {
  // Planar arm: femur at angle alpha, tibia continues at alpha+beta.
  const float alpha = femur_raw;
  const float beta = tibia_raw;
  const float planar_r = l2_ * cosf(alpha) + l3_ * cosf(alpha + beta);
  const float dz = l2_ * sinf(alpha) + l3_ * sinf(alpha + beta);
  const float horiz = planar_r + l1_;
  x_mm = horiz * cosf(coxa);
  y_mm = horiz * sinf(coxa);
  z_mm = dz;
}

}  // namespace gait
