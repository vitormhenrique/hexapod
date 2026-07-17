#pragma once

// ===========================================================================
// Command-frame -> body-frame axis conversion (hexapod_src-4ju.25).
//
// The public motion API and the RC bridge express planar intent in an
// operator "command frame": forward(+)/backward(-), left(+)/right(-), yaw
// CCW(+). The URDF body frame B (inverse_kinematics.md sections 2-3) has +Y
// as the mechanical front and +X pointing right; the mid legs (3/6) extend
// radially along +/-X. Feeding forward intent directly into +X strides walks
// into the mid legs' tightest reach margin and visually degenerates into a
// twist, so ControllerCore rotates command intent into B at the gait pipeline
// boundary. Pure functions, host-tested.
// ===========================================================================

namespace controller {

// Rotate a planar command-frame vector (forward, left) into body frame B:
// forward maps to +Y (mechanical front), left maps to -X (+X is the robot's
// right side).
inline void commandPlanarToBody(float forward, float left, float& body_x,
                                float& body_y) {
  body_x = -left;
  body_y = forward;
}

// Map command-frame roll (about the forward axis) and pitch (about the left
// axis) onto body-frame rotations. Forward is +Y, so command roll is a body
// pitch; left is -X, so command pitch is a negated body roll. Yaw (about +Z)
// needs no conversion.
inline void commandAttitudeToBody(float roll_about_forward,
                                  float pitch_about_left, float& body_roll,
                                  float& body_pitch) {
  body_roll = -pitch_about_left;
  body_pitch = roll_about_forward;
}

}  // namespace controller
