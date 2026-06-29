#pragma once

// ===========================================================================
// Body pose transform + full-chain leg solve (portable, no Arduino deps).
//
// Bridges the body-centered frame B (REP-103: X forward, Y left, Z up; origin
// at the body geometric centre) and each leg's coxa frame, then runs leg IK.
// Per-leg placement comes from the validated robot config (config_schema.h):
// coxa mount XY/Z and home yaw.
//
// Frame relation (verified against HexNav IK ref sections 3/5/12): a leg's
// coxa-frame radial +X direction sits at (mount_yaw + 90deg) in B, and the coxa
// joint axis is lifted +21 mm above the mount. So a point in B maps to the coxa
// frame by translating to the hip, rotating by -(mount_yaw + 90deg) about Z,
// and shifting Z down by (mount_z + 21 mm):
//
//   c = Rz(-(yaw+90)) * (p_B.xy - hip_xy);  c.z = p_B.z - (mount_z + 21)
//
// This maps every leg's documented home foot to (127, 0, -44.55) -> all-zero
// joint angles.
//
// Body pose control (IK ref section 11): with feet world-fixed, moving the body
// by translation t and rotation R(roll,pitch,yaw) re-expresses each foot in the
// moved body frame as  p_B = R^T * (p_world - t),  which then maps to the coxa
// frame as above. Rotation uses the REP-103 ZYX (yaw*pitch*roll) convention.
//
// No heap; per-leg transform constants are precomputed once from the config.
// ===========================================================================

#include <stdint.h>

#include "../config/config_schema.h"
#include "leg_ik.h"

namespace gait {

// Coxa joint axis lift above the mount (IK ref section 4/13, COXA_Z_OFF). This
// is the documented reference nominal; the runtime value comes from the
// persisted config (config::BodyGeometry::coxa_lift_cmm), whose compiled default
// mirrors this constant.
constexpr float kCoxaLiftMm = 21.0f;

// 6-DOF body pose relative to the neutral stance. Translation in mm, rotation
// in radians (REP-103 roll/pitch/yaw). All-zero == neutral.
struct BodyPose {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float z_mm = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

class BodyKinematics {
 public:
  explicit BodyKinematics(const config::RobotConfig& cfg);

  // Transform a foot target expressed in body frame B (mm) into leg `leg`'s
  // coxa frame (mm).
  void footBodyToCoxa(uint8_t leg, float bx, float by, float bz, float& cx,
                      float& cy, float& cz) const;

  // Apply a body pose to a world-fixed foot position (given in the neutral body
  // frame, mm) and return its coordinates in the moved body frame:
  //   out = R(pose)^T * (in - t(pose))
  static void applyBodyPose(const BodyPose& pose, float wx, float wy, float wz,
                            float& bx, float& by, float& bz);

  // Full chain: foot target in body frame B -> coxa frame -> leg IK.
  IkResult solveBody(uint8_t leg, float bx, float by, float bz) const;

  // Full chain with reachability-aware stride limiting (lmt.14): foot target in
  // body frame B -> coxa frame -> clamp to the safe reach margin -> leg IK.
  // Sets `reach_limited` true when the foot had to be pulled in to stay
  // reachable. Used by the gait pipeline so commanded strides never leave the
  // workspace; single-leg maintenance targets keep using solveBody so the
  // operator still sees the raw `reachable` flag.
  IkResult solveBodyLimited(uint8_t leg, float bx, float by, float bz,
                            bool& reach_limited) const;

  // Full chain with body pose: a world-fixed foot (neutral body frame) under a
  // body pose -> moved body frame -> coxa frame -> leg IK.
  IkResult solveBodyPose(uint8_t leg, const BodyPose& pose, float wx, float wy,
                         float wz) const;

  const LegIk& legIk() const { return ik_; }

 private:
  struct LegXform {
    float hip_x_mm = 0.0f;
    float hip_y_mm = 0.0f;
    float z_off_mm = 0.0f;  // mount_z + coxa lift
    float cos_a = 1.0f;     // cos/sin of -(mount_yaw + 90deg)
    float sin_a = 0.0f;
  };

  LegXform legs_[config::kNumLegs];
  LegIk ik_;
};

}  // namespace gait
