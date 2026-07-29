// Body pose transform + full-chain leg solve (portable, host-tested).
// See body_ik.h for the frame relations.

#include "body_ik.h"

#include <math.h>

namespace gait {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = kPi / 2.0f;
constexpr float kCentiDegToRad = (kPi / 180.0f) / 100.0f;  // 0.01 deg -> rad

}  // namespace

BodyKinematics::BodyKinematics(const config::RobotConfig& cfg)
    : ik_(cfg.links.coxa_cmm / 100.0f,    // 0.01 mm -> mm
          cfg.links.femur_cmm / 100.0f,
          cfg.links.tibia_cmm / 100.0f,
          cfg.geometry.home_radius_cmm / 100.0f,
          cfg.geometry.home_foot_z_cmm / 100.0f) {
  const float coxa_lift_mm = cfg.geometry.coxa_lift_cmm / 100.0f;
  for (uint8_t leg = 0; leg < config::kNumLegs; ++leg) {
    const config::LegGeometry& g = cfg.legs[leg];
    LegXform& x = legs_[leg];
    x.hip_x_mm = g.mount_x_dmm / 10.0f;  // 0.1 mm -> mm
    x.hip_y_mm = g.mount_y_dmm / 10.0f;
    x.z_off_mm = g.mount_z_dmm / 10.0f + coxa_lift_mm;
    const float yaw_rad = g.mount_yaw_cdeg * kCentiDegToRad;
    const float a = -(yaw_rad + kHalfPi);  // rotate B-vector into coxa frame
    x.cos_a = cosf(a);
    x.sin_a = sinf(a);
  }
}

void BodyKinematics::footBodyToCoxa(uint8_t leg, float bx, float by, float bz,
                                    float& cx, float& cy, float& cz) const {
  if (leg >= config::kNumLegs) {
    cx = cy = cz = 0.0f;
    return;
  }
  const LegXform& x = legs_[leg];
  const float dx = bx - x.hip_x_mm;
  const float dy = by - x.hip_y_mm;
  cx = x.cos_a * dx - x.sin_a * dy;
  cy = x.sin_a * dx + x.cos_a * dy;
  cz = bz - x.z_off_mm;
}

void BodyKinematics::applyBodyPose(const BodyPose& pose, float wx, float wy,
                                   float wz, float& bx, float& by, float& bz) {
  // R = Rz(yaw) * Ry(pitch) * Rx(roll); we apply R^T * (w - t).
  const float cy = cosf(pose.yaw), sy = sinf(pose.yaw);
  const float cp = cosf(pose.pitch), sp = sinf(pose.pitch);
  const float cr = cosf(pose.roll), sr = sinf(pose.roll);

  // R rows.
  const float r00 = cy * cp;
  const float r01 = cy * sp * sr - sy * cr;
  const float r02 = cy * sp * cr + sy * sr;
  const float r10 = sy * cp;
  const float r11 = sy * sp * sr + cy * cr;
  const float r12 = sy * sp * cr - cy * sr;
  const float r20 = -sp;
  const float r21 = cp * sr;
  const float r22 = cp * cr;

  const float vx = wx - pose.x_mm;
  const float vy = wy - pose.y_mm;
  const float vz = wz - pose.z_mm;

  // R^T * v (transpose: columns of R become rows).
  bx = r00 * vx + r10 * vy + r20 * vz;
  by = r01 * vx + r11 * vy + r21 * vz;
  bz = r02 * vx + r12 * vy + r22 * vz;
}

IkResult BodyKinematics::solveBody(uint8_t leg, float bx, float by,
                                   float bz) const {
  float cx, cy, cz;
  footBodyToCoxa(leg, bx, by, bz, cx, cy, cz);
  return ik_.solve(cx, cy, cz);
}

IkResult BodyKinematics::solveBodyLimited(uint8_t leg, float bx, float by,
                                          float bz, bool& reach_limited) const {
  float cx, cy, cz;
  footBodyToCoxa(leg, bx, by, bz, cx, cy, cz);
  reach_limited = ik_.clampToReach(cx, cy, cz);
  return ik_.solve(cx, cy, cz);
}

float BodyKinematics::bodyTargetPathScale(
    uint8_t leg, float anchor_bx, float anchor_by, float anchor_bz,
    float target_bx, float target_by, float target_bz) const {
  if (leg >= config::kNumLegs) return 0.0f;

  float target_x, target_y, target_z;
  footBodyToCoxa(leg, target_bx, target_by, target_bz,
                 target_x, target_y, target_z);
  if (ik_.withinReachMargin(target_x, target_y, target_z)) return 1.0f;

  float anchor_x, anchor_y, anchor_z;
  footBodyToCoxa(leg, anchor_bx, anchor_by, anchor_bz,
                 anchor_x, anchor_y, anchor_z);
  if (!ik_.withinReachMargin(anchor_x, anchor_y, anchor_z)) return 0.0f;

  float reachable = 0.0f;
  float unreachable = 1.0f;
  // Eight iterations bound the path scale within 1/256. At the maximum 80 mm
  // stroke this is below 0.32 mm while keeping Cortex-M0+ work predictable.
  for (uint8_t iteration = 0; iteration < 8; ++iteration) {
    const float scale = 0.5f * (reachable + unreachable);
    const float x = anchor_x + (target_x - anchor_x) * scale;
    const float y = anchor_y + (target_y - anchor_y) * scale;
    const float z = anchor_z + (target_z - anchor_z) * scale;
    if (ik_.withinReachMargin(x, y, z)) {
      reachable = scale;
    } else {
      unreachable = scale;
    }
  }
  return reachable;
}

IkResult BodyKinematics::solveBodyPose(uint8_t leg, const BodyPose& pose,
                                       float wx, float wy, float wz) const {
  float bx, by, bz;
  applyBodyPose(pose, wx, wy, wz, bx, by, bz);
  return solveBody(leg, bx, by, bz);
}

IkResult BodyKinematics::solveBodyPoseLimited(
    uint8_t leg, const BodyPose& pose, float wx, float wy, float wz,
    bool& reach_limited) const {
  float bx, by, bz;
  applyBodyPose(pose, wx, wy, wz, bx, by, bz);
  return solveBodyLimited(leg, bx, by, bz, reach_limited);
}

}  // namespace gait
